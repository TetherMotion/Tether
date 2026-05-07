/**
 * @file PDOQueue.hpp
 * @brief Thread-safe dual queue system for non-realtime PDO motion control
 *
 * Provides a lock-free dual queue that decouples the EtherCAT realtime cyclic
 * loop from user-space motion planning:
 *
 *  - **RxPDO queue (outbound):** The user thread pushes RxPDO frames; the
 *    cyclic loop pops the next frame each cycle and writes it to the slave.
 *
 *  - **TxPDO snapshot (inbound):** The cyclic loop publishes incoming TxPDO
 *    data every cycle; the user thread can read the latest snapshot at any
 *    time without blocking the cyclic loop.
 *
 * Both paths are designed to be wait-free on the realtime side and use only
 * atomics and a small SPSC ring buffer — no mutexes, no heap allocation in
 * the hot path.
 *
 * Usage:
 * @code
 *   #include "profiles/cia402/PDOQueue.hpp"
 *   #include "drives/AS715N/AS715NPDO.hpp"
 *
 *   using namespace EtherCAT;
 *
 *   PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04> queue;
 *
 *   // User thread: push a new RxPDO command
 *   AS715N_RxPDO_1705 cmd{};
 *   cmd.target_position = 50000;
 *   cmd.modes_of_operation = 8;
 *   queue.pushRx(cmd);
 *
 *   // Cyclic loop: pop the next RxPDO (returns false if empty)
 *   AS715N_RxPDO_1705 out{};
 *   if (queue.popRx(out)) { memcpy(rx_buffer, &out, sizeof(out)); }
 *
 *   // Cyclic loop: publish latest TxPDO
 *   queue.publishTx(*reinterpret_cast<const AS715N_TxPDO_1B04*>(tx_buffer));
 *
 *   // User thread: read latest TxPDO snapshot
 *   auto snap = queue.latestTx();
 * @endcode
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace EtherCAT {

/**
 * @brief Thread-safe dual PDO queue for decoupled motion control.
 *
 * @tparam RxPDO  Packed RxPDO struct (e.g. AS715N_RxPDO_1705)
 * @tparam TxPDO  Packed TxPDO struct (e.g. AS715N_TxPDO_1B04)
 * @tparam Depth  Ring buffer depth for the RxPDO queue (must be power of 2)
 */
template <typename RxPDO, typename TxPDO, size_t Depth = 64>
class PDOQueue {
    static_assert(std::is_trivially_copyable<RxPDO>::value,
                  "RxPDO must be trivially copyable");
    static_assert(std::is_trivially_copyable<TxPDO>::value,
                  "TxPDO must be trivially copyable");
    static_assert((Depth & (Depth - 1)) == 0,
                  "Depth must be a power of two");

public:
    PDOQueue() {
        std::memset(&m_tx_snapshot, 0, sizeof(TxPDO));
        std::memset(m_ring, 0, sizeof(m_ring));
    }

    // -----------------------------------------------------------------
    //  RxPDO queue  (user thread → cyclic loop)
    // -----------------------------------------------------------------

    /**
     * @brief Push an RxPDO frame into the outbound queue.
     *
     * Called from the **user thread**.  Returns false if the ring is full.
     */
    bool pushRx(const RxPDO& frame) {
        const size_t w = m_write.load(std::memory_order_relaxed);
        const size_t r = m_read.load(std::memory_order_acquire);
        if (w - r >= Depth) return false;              // full
        m_ring[w & kMask] = frame;
        m_write.store(w + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop the next RxPDO frame from the queue.
     *
     * Called from the **cyclic realtime loop**.  Returns false if empty.
     */
    bool popRx(RxPDO& frame) {
        const size_t r = m_read.load(std::memory_order_relaxed);
        const size_t w = m_write.load(std::memory_order_acquire);
        if (r == w) return false;                      // empty
        frame = m_ring[r & kMask];
        m_read.store(r + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Number of RxPDO frames currently queued.
     */
    size_t rxPending() const {
        return m_write.load(std::memory_order_acquire)
             - m_read.load(std::memory_order_acquire);
    }

    /**
     * @brief Discard all pending RxPDO frames.
     */
    void clearRx() {
        m_read.store(m_write.load(std::memory_order_acquire),
                     std::memory_order_release);
    }

    // -----------------------------------------------------------------
    //  TxPDO snapshot  (cyclic loop → user thread)
    // -----------------------------------------------------------------

    /**
     * @brief Publish the latest TxPDO data (called from the cyclic loop).
     *
     * Uses a sequence-lock pattern: the reader retries when it detects a
     * torn write.  The writer (cyclic loop) is wait-free.
     */
    void publishTx(const TxPDO& frame) {
        m_tx_seq.fetch_add(1, std::memory_order_release);   // odd  → write in progress
        m_tx_snapshot = frame;
        m_tx_seq.fetch_add(1, std::memory_order_release);   // even → write complete
    }

    /**
     * @brief Read the latest TxPDO snapshot (called from any thread).
     *
     * Retries automatically on torn reads.
     */
    TxPDO latestTx() const {
        TxPDO tmp;
        size_t seq0, seq1;
        do {
            seq0 = m_tx_seq.load(std::memory_order_acquire);
            tmp  = m_tx_snapshot;
            seq1 = m_tx_seq.load(std::memory_order_acquire);
        } while (seq0 != seq1 || (seq0 & 1));
        return tmp;
    }

    /**
     * @brief Check whether at least one TxPDO has been published.
     */
    bool hasTxData() const {
        return m_tx_seq.load(std::memory_order_acquire) >= 2;
    }

private:
    static constexpr size_t kMask = Depth - 1;

    // RxPDO SPSC ring
    RxPDO               m_ring[Depth];
    std::atomic<size_t>  m_write{0};
    std::atomic<size_t>  m_read{0};

    // TxPDO seqlock snapshot
    TxPDO                        m_tx_snapshot;
    mutable std::atomic<size_t>  m_tx_seq{0};
};

} // namespace EtherCAT
