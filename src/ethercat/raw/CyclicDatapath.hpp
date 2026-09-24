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
    /// `idx` is the wire datagram index in the fastpath range [0xE0..0xFD]:
    /// PDO slices land in slots [0..15], cyclic datagrams in [16..21].
    void deposit(uint8_t idx, Command cmd, uint16_t adp, uint16_t ado,
                 const uint8_t* payload, uint16_t datalen, uint16_t wkc,
                 uint8_t gen);
    /// View-mode deposit: payload points into channel memory held by
    /// `cookie`; the previous held cookie (if any) is released.
    /// `stamp_ns` carries the frame's kernel timestamp (0 → deposit-time).
    void publishView(uint8_t idx, Command cmd, uint16_t adp, uint16_t ado,
                     const uint8_t* payload, uint16_t datalen, uint16_t wkc,
                     uint32_t cookie, uint64_t stamp_ns = 0, uint8_t gen = 0);
    uint64_t slotToken(uint8_t slot) const;
    uint8_t  slotGen(uint8_t slot) const;
    /// PDO-slice slot variants — `slice` in [0, kNumSliceSlots).
    uint64_t sliceSlotToken(uint8_t slice) const;
    uint8_t  sliceSlotGen(uint8_t slice) const;

    // ---- Wire send path ------------------------------------------------
    bool sendDatagram(Command cmd, uint8_t slot, uint16_t adp, uint16_t ado,
                      const void* data, uint16_t datalen, bool roundtrip);
    /// Send on a dedicated PDO-slice index (wire idx 0xE0 + slice_slot).
    bool sendSliceDatagram(Command cmd, uint8_t slice_slot,
                           uint16_t adp, uint16_t ado,
                           const void* data, uint16_t datalen,
                           bool roundtrip);
    /// Persistent TX frame buffer for the ring backend (nullptr w/o channel).
    uint8_t* acquireTxFrame();
    /// Stamp the next send generation + compose the frame header — one
    /// memcpy of the baked per-(idx,gen) template (see ensureTemplate()).
    void composeHeader(uint8_t* frame, Command cmd, uint8_t slot,
                       uint16_t adp, uint16_t ado, uint16_t datalen,
                       bool roundtrip);
    /// Byte offset of the datagram payload inside a composed TX frame —
    /// 26 untagged, 30 when a TX VLAN tag is baked into the templates.
    uint32_t payloadOffset() const { return tx_prefix_len_; }
    bool sendFrame(uint32_t frame_len);

    // ---- Receive path ----------------------------------------------------
    /// Route a frame received on the cyclic channel: pure-fastpath frames
    /// (every datagram idx in [0xE0..0xFD]) publish slot views, mixed/async
    /// frames go to the master's parser.
    void dispatchFrame(const CyclicFrameView& view);
    bool waitView(uint8_t slot, uint64_t token, uint32_t timeout_ns,
                  CyclicSlotView& out);
    bool waitSliceView(uint8_t slice, uint64_t token, uint32_t timeout_ns,
                       CyclicSlotView& out);
    uint32_t waitMask(uint32_t slot_mask, const uint64_t* tokens,
                      uint32_t timeout_ns, CyclicSlotView* views);
    /// Masked wait over PDO-slice slots [0, kNumSliceSlots).
    uint32_t waitSliceMask(uint32_t slice_mask, const uint64_t* tokens,
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
        uint8_t  data[kMaxDatagramDataSize];
    };
    /// Unified fastpath slot bank, indexed by `idx - kFastSlotBaseIdx`:
    ///   [0..15]  PDO-slice slots   (wire idx 0xE0..0xEF)
    ///   [16..21] cyclic image slots (wire idx 0xF8..0xFD)
    /// Slice slot t and cyclic slot s therefore never share a mailbox —
    /// a sliced full-image collect and a custom slice collect can be
    /// in flight at once.
    /// Wire idx → array index.  The two wire ranges are contiguous on
    /// the wire but not in the array: slice 0xE0..0xEF → [0..15],
    /// cyclic 0xF8..0xFD → [16..21] (the 0xF0..0xF7 gap has no slot).
    /// Callers must gate on isSlotIdx() — gap indices have no mapping.
    static constexpr uint8_t fastIndex(uint8_t idx) {
        return isSliceIdx(idx)
            ? static_cast<uint8_t>(idx - kSliceSlotBaseIdx)
            : static_cast<uint8_t>(kNumSliceSlots +
                                   (idx - kCyclicSlotBaseIdx));
    }
    static constexpr uint8_t cyclicFastIndex(uint8_t slot) {
        return static_cast<uint8_t>(kNumSliceSlots + slot);
    }
    /// Array index → wire idx (inverse of fastIndex).
    static constexpr uint8_t wireIndex(uint8_t fast_idx) {
        return fast_idx < kNumSliceSlots
            ? static_cast<uint8_t>(kSliceSlotBaseIdx + fast_idx)
            : static_cast<uint8_t>(kCyclicSlotBaseIdx +
                                   (fast_idx - kNumSliceSlots));
    }
    std::array<RxSlot, kNumFastSlots> slots_{};

    /// Datapath channel (Linux: socket or PACKET_MMAP ring backend).
    /// nullptr → software deposit path (unchanged behaviour).
    std::unique_ptr<ICyclicChannel> channel_;
    ProcessImage image_;
    ImageMode active_image_mode_ = ImageMode::Buffered;

    /// Persistent cyclic TX frame buffer — avoids a zeroed stack buffer
    /// per cycle.  Only the cyclic thread writes it.  Jumbo-sized so a
    /// raised Master::Config::max_frame_size can use the whole slice.
    uint8_t tx_buf_[kMaxJumboFrameSize] = {};

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
    /// send on that index — written/read on the cyclic thread only.
    std::array<uint8_t, kNumFastSlots> slot_gen_{};

    /**
     * @brief Baked wire-header templates, [fast_idx][send_gen].
     *
     * Everything in a cyclic frame header except the generation bit is
     * invariant while the PDO layout is stable — destination/source MAC,
     * EtherType (or the inline 802.1Q tag under VLAN encapsulation), the
     * EtherCAT frame header, and the datagram header incl. the reserved
     * idx.  The two gen variants are baked together the first time an
     * index is used with a given (cmd, adp, ado, len, roundtrip) key, so
     * the hot path is a bounds check + one contiguous ≤32-byte copy —
     * no field-by-field assembly, no per-cycle VID/MAC gathering.
     * ensureTemplate() re-bakes on a key mismatch (mapping re-registration,
     * a different command borrowing the index) — rare and still correct.
     */
    struct HdrTemplate {
        uint8_t  bytes[32];   // eth(14) + vlan(4) + ecat(2) + dg-hdr(10)
        uint16_t len{0};      // bytes used == TX payload offset
        uint8_t  cmd{0};
        uint16_t adp{0}, ado{0}, datalen{0};
        bool     roundtrip{false}, valid{false};
    };
    std::array<std::array<HdrTemplate, 2>, kNumFastSlots> hdr_tmpl_{};

    /// Fetch the baked template for (index, gen), re-baking both gen
    /// variants when the key changed.  Cyclic thread only.
    const HdrTemplate& ensureTemplate(uint8_t fast_idx, uint8_t gen,
                                      Command cmd, uint16_t adp,
                                      uint16_t ado, uint16_t datalen,
                                      bool roundtrip);
    /// VID inserted into TX headers when the channel carries tagged
    /// traffic (0 = untagged).  Decided at setup(): the tag is baked into
    /// templates only when the channel is active — the no-channel fallback
    /// still routes through sendWithEncapsulation() which tags itself.
    uint16_t tx_vlan_ = 0;
    /// == hdr_tmpl_ prefix length on the active path (26 or 30).
    uint16_t tx_prefix_len_ = kCyclicFramePayloadOff;
    /// Common send body for cyclic and slice datagrams.
    bool sendFastDatagram(Command cmd, uint8_t fast_idx,
                          uint16_t adp, uint16_t ado,
                          const void* data, uint16_t datalen,
                          bool roundtrip);
    /// Masked wait shared by waitMask()/waitSliceMask() — `slot_base`
    /// translates mask bits into slots_ indexes.
    uint32_t waitMaskImpl(uint32_t slot_mask, const uint64_t* tokens,
                          uint32_t timeout_ns, CyclicSlotView* views,
                          uint8_t slot_base, uint8_t slot_count);
    /// Single-slot wait shared by waitView()/waitSliceView().
    bool waitViewImpl(uint8_t fast_idx, uint64_t token, uint32_t timeout_ns,
                      CyclicSlotView& out);

    /// Exchange suspension for mid-loop mapping mutation (slave
    /// recovery): while set, the exchange/collect tasks skip their work
    /// and bump exchange_quiesced_ — the suspender waits for a bump to
    /// prove the loop thread passed a quiesce point.  NOT reentrant.
    std::atomic<bool>     exchange_suspended_{false};
    std::atomic<uint32_t> exchange_quiesced_{0};
};

} // namespace EtherCAT
