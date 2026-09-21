/**
 * @file CyclicDatapath.hpp
 * @brief The master's cyclic wire datapath: reserved-index response slots,
 *        the cyclic channel (socket/PACKET_MMAP), the process image, and
 *        the exchange-suspension handshake.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * Owns all state that the cyclic send/collect paths share.  Master keeps
 * a `std::unique_ptr<CyclicDatapath>` and forwards its public cyclic API;
 * this class reaches back into Master (friend) for the interface, config,
 * packet router, and PDO/LAM facades.
 */

#pragma once

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/ProcessImage.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace EtherCAT {

class CyclicDatapath {
public:
    explicit CyclicDatapath(Master& master);
    ~CyclicDatapath();   // releases held channel cookies + notify eventfd

    CyclicDatapath(const CyclicDatapath&) = delete;
    CyclicDatapath& operator=(const CyclicDatapath&) = delete;

    // ---- Response slots ------------------------------------------------
    /// Copy-mode deposit (software path — poll thread or socket-B parse).
    void deposit(uint8_t slot_idx, Command cmd, uint16_t adp, uint16_t ado,
                 const uint8_t* payload, uint16_t datalen, uint16_t wkc,
                 uint8_t gen);
    /// View-mode deposit: payload points into channel memory held by
    /// `cookie`; the previous held cookie (if any) is released.
    /// `stamp_ns` carries the frame's kernel timestamp (0 → deposit-time).
    void publishView(uint8_t slot_idx, Command cmd, uint16_t adp, uint16_t ado,
                     const uint8_t* payload, uint16_t datalen, uint16_t wkc,
                     uint32_t cookie, uint64_t stamp_ns = 0, uint8_t gen = 0);
    uint64_t slotToken(uint8_t slot) const;
    uint8_t  slotGen(uint8_t slot) const;

    // ---- Wire send path ------------------------------------------------
    bool sendDatagram(Command cmd, uint8_t slot, uint16_t adp, uint16_t ado,
                      const void* data, uint16_t datalen, bool roundtrip);
    /// Persistent TX frame buffer for the ring backend (nullptr w/o channel).
    uint8_t* acquireTxFrame();
    /// Stamp the next send generation + compose the 26-byte frame header.
    void composeHeader(uint8_t* frame, Command cmd, uint8_t slot,
                       uint16_t adp, uint16_t ado, uint16_t datalen,
                       bool roundtrip);
    bool sendFrame(uint32_t frame_len);

    // ---- Receive path ----------------------------------------------------
    /// Route a frame received on the cyclic channel: pure-cyclic frames
    /// publish slot views, mixed/async frames go to the master's parser.
    void dispatchFrame(const CyclicFrameView& view);
    bool waitView(uint8_t slot, uint64_t token, uint32_t timeout_ns,
                  CyclicSlotView& out);
    uint32_t waitMask(uint32_t slot_mask, const uint64_t* tokens,
                      uint32_t timeout_ns, CyclicSlotView* views);
    bool wait(uint8_t slot, uint64_t token, uint32_t timeout_ns,
              RxDatagram& out);

    // ---- Setup / teardown ------------------------------------------------
    void setup(CyclicWireMode wire_mode, ImageMode image_mode,
               const std::string& shm_image_name, uint32_t rx_spin_ns,
               uint32_t slot_spin_ns,
               Master::CyclicLoopConfig::SlotWaitFallback slot_fallback,
               bool strict_wkc, const Master::MemoryLockConfig& memlock);
    void teardown();

    // ---- Exchange suspension handshake -----------------------------------
    /// Task-entry gate: returns true while suspended (caller skips the
    /// cycle's wire work) and bumps the quiesce counter the suspender
    /// waits on.
    bool gateExchange() {
        if (exchange_suspended_.load(std::memory_order_acquire)) {
            exchange_quiesced_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        return false;
    }
    bool suspended() const {
        return exchange_suspended_.load(std::memory_order_acquire);
    }
    void suspend()  { exchange_suspended_.store(true, std::memory_order_release); }
    void resume()   { exchange_suspended_.store(false, std::memory_order_release); }
    uint32_t quiescedCount() const {
        return exchange_quiesced_.load(std::memory_order_relaxed);
    }

    // ---- State (internal — accessed by Master and the test seam) ---------
    Master& master_;

    /// Fixed response slots for the reserved idx range — written by whoever
    /// parses the RX frame (poll thread OR the cyclic thread itself via
    /// direct receive), read by the cyclic thread.  seq is bumped AFTER the
    /// payload fields are written (release) so a reader that observes a seq
    /// change sees a complete datagram.
    struct RxSlot {
        std::atomic<uint64_t> seq{0};
        uint8_t  cmd{0};
        uint16_t adp{0};
        uint16_t ado{0};
        uint16_t datalen{0};
        uint16_t wkc{0};
        /// View mode: points into channel memory (cookie holds the slot).
        /// Copy mode: points at data[].  Never null after first publish.
        const uint8_t* payload{nullptr};
        /// Channel cookie of the held view; -1 = copy mode / none.
        int64_t cookie{-1};
        /// Frame arrival timestamp (kernel stamp when the channel provides
        /// one, else monotonic now at deposit).
        uint64_t stamp_ns{0};
        /// Send-generation bit echoed from the datagram's lenFlags res-bit
        /// 13 — collect rejects deposits of the previous generation.
        uint8_t  gen{0};
        uint8_t  data[1486];
    };
    std::array<RxSlot, kNumCyclicSlots> slots_{};

    /// Datapath channel (Linux: socket or PACKET_MMAP ring backend).
    /// nullptr → software deposit path (unchanged behaviour).
    std::unique_ptr<ICyclicChannel> channel_;
    ProcessImage image_;
    ImageMode active_image_mode_ = ImageMode::Buffered;

    /// Persistent cyclic TX frame buffer — avoids a 1514-byte zeroed stack
    /// buffer per cycle.  Only the cyclic thread writes it.
    uint8_t tx_buf_[1514] = {};

    /// Linux eventfd signalled on cyclic-slot deposits *while a waiter is
    /// registered* (waiters_ > 0) so a blocked cyclic waiter wakes
    /// immediately even when the poll thread consumed the frame.  Writes
    /// are suppressed when nobody can be sleeping on it — an idle cyclic
    /// loop therefore pays no syscall per deposit.  -1 when unavailable.
    int notify_fd_ = -1;
    /// Number of threads currently inside a cyclic-slot wait — gates the
    /// eventfd write in the deposit/publish paths.
    std::atomic<int> waiters_{0};

    /// Busy-poll window handed to the channel's rxPoll() — the kernel-ring
    /// spin inside the channel itself (CyclicLoopConfig::rx_spin_ns).
    uint32_t rx_spin_ns_ = 0;
    /// Spin window applied in waitView before blocking: polls the slot
    /// sequence word and channel rxPending() — pure memory reads, so a
    /// ring-slot DMA write is visible with zero syscalls.  From
    /// CyclicLoopConfig::slot_spin_ns; 0 disables.
    uint32_t slot_spin_ns_ = 0;
    /// No-fd wait policy for the cyclic slot wait (Q22).
    Master::CyclicLoopConfig::SlotWaitFallback slot_wait_fallback_ =
        Master::CyclicLoopConfig::SlotWaitFallback::Yield;

    /// Split-exchange collect-task invocations — diagnostic counter that
    /// also lets tests observe collect-task ordering within a phase.
    std::atomic<uint64_t> collect_calls_{0};

    /// Per-slot send generation (lenFlags res-bit 13), toggled on every
    /// cyclic send — written/read on the cyclic thread only.
    std::array<uint8_t, kNumCyclicSlots> slot_gen_{};

    /// Exchange suspension for mid-loop mapping mutation (slave
    /// recovery): while set, the exchange/collect tasks skip their work
    /// and bump exchange_quiesced_ — the suspender waits for a bump to
    /// prove the loop thread passed a quiesce point.  NOT reentrant.
    std::atomic<bool>     exchange_suspended_{false};
    std::atomic<uint32_t> exchange_quiesced_{0};
};

} // namespace EtherCAT
