/**
 * @file TransactionRouter.hpp
 * @brief Race-free EtherCAT transaction routing by datagram index
 *
 * @details
 * TransactionRouter replaces ConditionalPacketRouter with a much simpler,
 * inherently race-free design.  Every EtherCAT datagram carries an 8-bit
 * index (`idx`).  We maintain one slot per possible index value (0-255).
 *
 * ## Why this is race-free
 *
 * The old ConditionalPacketRouter required the caller to either:
 *  1. Pre-register a waiter, send, then wait  (three separate steps)
 *  2. Send, then register+wait               (two steps, prone to races)
 *
 * TransactionRouter provides `sendAndWait()` which atomically marks the
 * slot as pending, sends the frame, and waits — all in the correct order.
 * Because slots are keyed by idx (which is already allocated before any
 * operation), the RX thread can always deliver a response to the right
 * slot regardless of timing.
 *
 * ## Architecture
 *
 * ```
 *  Caller thread                     RX thread
 *  ─────────────                     ─────────
 *  slot[idx].pending = true          eth->poll()
 *       │                                 │
 *       ▼                                 ▼
 *  sendRawFrame(frame)              routePacket(dgram)
 *       │                                 │
 *       ▼                            slot[dgram.idx].pending?
 *  slot[idx].cv.wait_for(...)             │ yes
 *       │                                 ▼
 *       │◄────── cv.notify_one() ◄─ copy result, set completed
 *       ▼
 *  return result
 * ```
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>

#include "tether/ethercat/EtherCATTypes.hpp"

namespace EtherCAT {

// ============================================================================
// WaitResult — kept compatible with the old router's WaitResult
// ============================================================================

struct WaitResult {
    bool     success{false};
    bool     timeout{false};
    uint16_t wkc{0};
    uint16_t data_length{0};
    Command  cmd{};
    uint16_t adp{0};
    uint16_t ado{0};
    uint8_t  idx{0};

    static WaitResult Timeout() {
        WaitResult r{};
        r.timeout = true;
        return r;
    }

    static WaitResult Success(uint16_t wkc, uint16_t data_len,
                              Command cmd, uint16_t adp, uint16_t ado,
                              uint8_t idx) {
        WaitResult r{};
        r.success     = true;
        r.wkc         = wkc;
        r.data_length = data_len;
        r.cmd         = cmd;
        r.adp         = adp;
        r.ado         = ado;
        r.idx         = idx;
        return r;
    }
};

// ============================================================================
// PacketFilter — kept for backward compatibility with EtherCATRetry
// ============================================================================

struct PacketFilter {
    Command  command{};
    bool     match_command{false};
    uint8_t  idx{0};
    uint16_t slave_index{0};
    uint16_t ado{0};
    uint16_t adp{0};
    bool     match_idx{false};
    bool     match_slave_index{false};
    bool     match_ado{false};
    bool     match_adp{false};
    uint16_t min_wkc{0};
    uint32_t logical_addr{0};
    uint16_t logical_length{0};
    bool     match_logical{false};

    static PacketFilter byIndex(uint8_t idx) {
        PacketFilter f{};
        f.idx       = idx;
        f.match_idx = true;
        return f;
    }

    static PacketFilter aprd(uint16_t slave_index, uint16_t ado, uint8_t idx) {
        PacketFilter f{};
        f.command           = Command::APRD;
        f.match_command     = true;
        f.slave_index       = slave_index;
        f.ado               = ado;
        f.idx               = idx;
        f.match_slave_index = true;
        f.match_ado         = true;
        f.match_idx         = true;
        return f;
    }

    static PacketFilter apwr(uint16_t slave_index, uint16_t ado, uint8_t idx) {
        PacketFilter f{};
        f.command           = Command::APWR;
        f.match_command     = true;
        f.slave_index       = slave_index;
        f.ado               = ado;
        f.idx               = idx;
        f.match_slave_index = true;
        f.match_ado         = true;
        f.match_idx         = true;
        return f;
    }

    static PacketFilter lrw(uint32_t logical_addr, uint16_t length, uint8_t idx) {
        PacketFilter f{};
        f.command        = Command::LRW;
        f.match_command  = true;
        f.logical_addr   = logical_addr;
        f.logical_length = length;
        f.idx            = idx;
        f.match_logical  = true;
        f.match_idx      = true;
        return f;
    }

    static PacketFilter sdo(uint16_t slave_index, uint16_t mbx_read_addr, uint8_t idx) {
        return aprd(slave_index, mbx_read_addr, idx);
    }

    static PacketFilter brd(uint16_t ado, uint8_t idx) {
        PacketFilter f{};
        f.command       = Command::BRD;
        f.match_command = true;
        f.ado           = ado;
        f.idx           = idx;
        f.match_ado     = true;
        f.match_idx     = true;
        return f;
    }
};

// ============================================================================
// TransactionRouter
// ============================================================================

/**
 * @brief Race-free EtherCAT transaction router indexed by datagram `idx`.
 *
 * Each EtherCAT datagram carries an 8-bit index.  TransactionRouter
 * maintains 256 slots (one per possible index).  The primary API is
 * `sendAndWait()` which atomically:
 *   1. Marks the slot as pending
 *   2. Sends the frame (via user-provided callback)
 *   3. Waits for the response on that slot
 *
 * The RX thread calls `routePacket()` which delivers responses by idx.
 *
 * This design is inherently race-free because the slot is always marked
 * pending **before** the frame is sent.
 */
class TransactionRouter {
public:
    /// Number of slots (one per possible idx value, 0–255)
    static constexpr size_t kNumSlots = 256;

    TransactionRouter();
    ~TransactionRouter();

    TransactionRouter(const TransactionRouter&) = delete;
    TransactionRouter& operator=(const TransactionRouter&) = delete;

    // ----- Lifecycle --------------------------------------------------------

    /**
     * @brief Initialize the router.
     * @return true on success (always succeeds on host).
     */
    bool init();

    /**
     * @brief Shut down the router and wake all waiting threads.
     */
    void shutdown();

    // ----- RX path (called from receive thread) -----------------------------

    /**
     * @brief Route an incoming datagram to the slot matching `dgram.idx`.
     *
     * If a waiter is pending on that slot, the response data is copied
     * and the waiter is notified.
     *
     * @param dgram  Received datagram (parsed by the master).
     * @return 1 if the packet matched a pending slot, 0 otherwise.
     */
    size_t routePacket(const RxDatagram& dgram);

    // ----- TX+RX: atomic send-and-wait --------------------------------------

    using SendFunc = std::function<bool()>;

    /**
     * @brief Send a frame and wait for the matching response.
     *
     * This is the primary API.  Steps:
     *  1. Mark `slots_[idx]` as pending (under lock).
     *  2. Call `send_fn()` to transmit the frame.
     *  3. Wait on the slot's condition variable up to `timeout_ms`.
     *
     * Because step 1 happens before step 2, a fast response can never
     * be missed.
     *
     * @param idx         Datagram index to wait for.
     * @param buffer      Destination buffer for response data.
     * @param buffer_size Size of the destination buffer.
     * @param send_fn     Callable that actually transmits the frame.
     * @param timeout_ms  Maximum time to wait for a response.
     * @return WaitResult with success/timeout and metadata.
     */
    WaitResult sendAndWait(uint8_t idx,
                           uint8_t* buffer, size_t buffer_size,
                           SendFunc send_fn,
                           uint32_t timeout_ms);

    // ----- Legacy / compatibility -------------------------------------------

    /**
     * @brief Wait for a packet matching a filter (legacy API).
     *
     * This maps to `sendAndWait()` but with the send step already done.
     * Prefer `sendAndWait()` for new code to avoid race conditions.
     */
    WaitResult waitForPacket(const PacketFilter& filter,
                             uint8_t* buffer, size_t buffer_size,
                             uint32_t timeout_ms);

    /**
     * @brief Pre-register a waiter (legacy API for backward compat).
     * @return Slot index (== idx), or kNumSlots on failure.
     */
    size_t preRegisterWaiter(const PacketFilter& filter,
                             uint8_t* buffer, size_t buffer_size);

    /**
     * @brief Wait for a pre-registered slot (legacy API).
     */
    WaitResult waitForPreRegistered(size_t slot, uint32_t timeout_ms);

    /**
     * @brief Cancel a pre-registered waiter.
     */
    void cancelPreRegistered(size_t slot);

    // ----- Diagnostics ------------------------------------------------------

    bool   hasWaiters() const;
    size_t waiterCount() const;

    struct Stats {
        uint64_t packets_routed{0};
        uint64_t packets_matched{0};
        uint64_t packets_dropped{0};
        uint64_t timeouts{0};
        uint64_t registrations{0};
        uint64_t registration_failures{0};
    };

    Stats getStats() const { return stats_; }
    void  resetStats()     { stats_ = {}; }

    /// Maximum concurrent waiters (kept for backward compat with ConditionalPacketRouter::kMaxWaiters)
    static constexpr size_t kMaxWaiters = kNumSlots;

private:
    struct Slot {
        std::mutex              mtx;
        std::condition_variable cv;
        bool                    pending{false};
        bool                    completed{false};
        uint8_t*                buffer{nullptr};
        size_t                  buffer_size{0};
        RxDatagram              response{};
    };

    std::array<Slot, kNumSlots> slots_;
    std::atomic<bool>           initialized_{false};
    std::atomic<bool>           shutdown_{false};
    Stats                       stats_;
};

// ============================================================================
// Backward-compat alias
// ============================================================================

using ConditionalPacketRouter = TransactionRouter;

} // namespace EtherCAT
