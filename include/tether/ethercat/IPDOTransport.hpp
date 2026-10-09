/**
 * @file IPDOTransport.hpp
 * @brief IPDOTransport — abstract transport interface for PDO I/O.
 *
 * Split out of PDOManager.hpp.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/PDOMapping.hpp"
#include "tether/ethercat/PDOTypes.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

class IPDOTransport {
public:
    virtual ~IPDOTransport() = default;

    /// Fire-and-forget index constant
    static constexpr uint8_t kFireAndForgetIdx = 0xFE;

    // ------------------------------------------------------------------
    // Cyclic fast path: reserved index range + fixed response slots.
    //
    // Datagrams sent with a fastpath idx (0x9C..0xFF minus 0xFE — the
    // rotating pool, the PDO-slice band 0xE0..0xEF and the DC index 0xFF)
    // bypass TransactionRouter entirely: the RX parser deposits them into
    // fixed preallocated slots that the cyclic thread polls on a sequence
    // counter — no mutex, no condition variable, and no RxDatagram copy on
    // the hot path.  allocIdx() never returns an index in this range.
    // ------------------------------------------------------------------
    static constexpr uint8_t kCyclicSlotBase = ::EtherCAT::kCyclicSlotBaseIdx;
    static constexpr size_t  kNumCyclicSlots = ::EtherCAT::kNumCyclicSlots;

    /// @return true if the transport implements the cyclic slot fast path.
    virtual bool supportsCyclicFastPath() const { return false; }

    /**
     * @brief Read the current sequence token of a cyclic slot.
     *
     * Call BEFORE sendCyclicDatagram().  The token distinguishes the
     * response belonging to the upcoming send from stale deposits.
     */
    virtual uint64_t cyclicSlotToken(uint8_t slot) {
        (void)slot; return 0;
    }

    /**
     * @brief Generation bit of the last datagram sent on @p slot.
     *
     * Stamped into lenFlags reserved bit 13 by sendCyclicDatagram()/
     * composeCyclicHeader() and echoed verbatim by the ring — a deposit
     * carrying the previous generation is a stale response that outlived
     * its cycle's timeout.  Default 0 disables the check (transports
     * without a stamped header echo views with gen==0 anyway).
     */
    virtual uint8_t cyclicSlotGen(uint8_t slot) {
        (void)slot; return 0;
    }

    /**
     * @brief Send a single datagram on a reserved cyclic slot index.
     * @param slot   Slot number in [0, kNumCyclicSlots)
     * @return true if the frame was handed to the wire.
     */
    virtual bool sendCyclicDatagram(Command cmd, uint8_t slot,
                                    uint16_t adp, uint16_t ado,
                                    const void* data, uint16_t datalen,
                                    bool roundtrip) {
        (void)cmd; (void)slot; (void)adp; (void)ado;
        (void)data; (void)datalen; (void)roundtrip;
        return false;
    }

    /**
     * @brief Wait until the response for @p token arrives on @p slot.
     *
     * On Linux this blocks in ppoll() on the socket + a deposit eventfd —
     * the calling thread sleeps until either the response lands or the
     * deadline passes.  Without a receive path it falls back to a bounded
     * sequence-counter spin.
     *
     * @param timeout_ns  Maximum wait in nanoseconds (sub-ms scale).
     * @return true and fills @p out on success; false on timeout/cancel.
     */
    virtual bool waitCyclicSlot(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, RxDatagram& out) {
        (void)slot; (void)token; (void)timeout_ns; (void)out;
        return false;
    }

    /**
     * @brief View-returning variant of waitCyclicSlot().
     *
     * `out.payload` points into the slot's inline buffer (software path)
     * or into channel-owned ring/bank memory (channel path — `out.channel`
     * / `out.cookie` then identify the held region).
     */
    virtual bool waitCyclicSlotView(uint8_t slot, uint64_t token,
                                    uint32_t timeout_ns,
                                    CyclicSlotView& out) {
        (void)slot; (void)token; (void)timeout_ns; (void)out;
        return false;
    }

    /**
     * @brief Wait until every slot in @p slot_mask has a deposit newer
     *        than its token — one wait for a whole multi-slice exchange.
     *
     * Transports with a real wake path override this so a sliced collect
     * pays ONE sleep instead of one per slice (Q3).  The default
     * implementation degenerates to a per-slot waitCyclicSlotView() loop
     * sharing the same deadline — correct on every transport, just
     * syscall-heavier.
     *
     * @param slot_mask  Bitmask over slots [0, kNumCyclicSlots)
     * @param tokens     Per-slot seq tokens, indexed by slot number
     * @param timeout_ns Shared deadline budget across the whole mask
     * @param views      Output array, indexed by slot number — filled
     *                   only for slots that arrived
     * @return The subset of @p slot_mask whose responses arrived before
     *         the deadline — full success iff the return == slot_mask.
     */
    virtual uint32_t waitCyclicSlotMask(uint32_t slot_mask,
                                        const uint64_t* tokens,
                                        uint32_t timeout_ns,
                                        CyclicSlotView* views) {
        // Per-slot fallback: walk the mask, sharing one CLOCK_MONOTONIC
        // deadline across slots.  The mask is 32 bits — only positions
        // 0..31 are reachable through this API (pooled exchanges use
        // waitCyclicPool for the full 0..82 range).
        uint32_t arrived = 0;
        const uint64_t deadline = monoNowNsFallback() + timeout_ns;
        for (uint8_t s = 0; s < 32 && s < kNumCyclicSlots; ++s) {
            if (!(slot_mask & (1u << s))) continue;
            const uint64_t now = monoNowNsFallback();
            const uint32_t remain = now < deadline
                ? static_cast<uint32_t>(deadline - now) : 0;
            if (!waitCyclicSlotView(s, tokens[s], remain, views[s]))
                break;   // deadline shared — later slots are worse off
            arrived |= 1u << s;
        }
        return arrived;
    }

    /**
     * @brief Acquire the channel's next TX frame buffer (Rotating image
     *        mode).  nullptr when no channel exists.
     */
    virtual uint8_t* acquireCyclicTxFrame() { return nullptr; }

    /**
     * @brief Compose [eth][ecat][dg-hdr] into an acquired frame buffer
     *        (payload region follows at offset 26).
     */
    virtual void composeCyclicHeader(uint8_t* frame, Command cmd,
                                     uint8_t slot, uint16_t adp,
                                     uint16_t ado, uint16_t datalen,
                                     bool roundtrip) {
        (void)frame; (void)cmd; (void)slot; (void)adp; (void)ado;
        (void)datalen; (void)roundtrip;
    }

    /// Commit the frame composed into the last acquireCyclicTxFrame() buffer.
    virtual bool sendCyclicFrame(uint32_t frame_len) {
        (void)frame_len; return false;
    }

    /// @return the cyclic channel, or nullptr on the software path.
    virtual ICyclicChannel* cyclicChannel() { return nullptr; }

    // ------------------------------------------------------------------
    // PDO-slice fast path: dedicated idx pool [kSliceSlotBase, +kNumSliceSlots)
    //
    // User-defined PDO slices exchange on their own reserved indices —
    // a different pipeline from the cyclic image slots: responses land
    // in per-slice deposit slots, demultiplexed by the same socket
    // filter.  A slice's datagrams and the cyclic image's datagrams can
    // be in flight at once without sharing mailboxes.
    // ------------------------------------------------------------------
    static constexpr uint8_t kSliceSlotBase = ::EtherCAT::kSliceSlotBaseIdx;
    static constexpr size_t  kNumSliceSlots = ::EtherCAT::kNumSliceSlots;

    /// Seq token of a slice slot — call BEFORE sendSliceDatagram().
    virtual uint64_t sliceSlotToken(uint8_t slice) {
        (void)slice; return 0;
    }
    /// Generation bit of the last datagram sent on @p slice (stale guard).
    virtual uint8_t sliceSlotGen(uint8_t slice) {
        (void)slice; return 0;
    }
    /// Send a datagram on a dedicated slice index (wire idx = base + slot).
    virtual bool sendSliceDatagram(Command cmd, uint8_t slice_slot,
                                   uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t datalen,
                                   bool roundtrip) {
        (void)cmd; (void)slice_slot; (void)adp; (void)ado;
        (void)data; (void)datalen; (void)roundtrip;
        return false;
    }
    /// Single-slot view wait over a slice index — same semantics as
    /// waitCyclicSlotView().
    virtual bool waitSliceSlotView(uint8_t slice, uint64_t token,
                                   uint32_t timeout_ns,
                                   CyclicSlotView& out) {
        (void)slice; (void)token; (void)timeout_ns; (void)out;
        return false;
    }
    /**
     * @brief One wake for a whole multi-run slice exchange — same
     *        semantics as waitCyclicSlotMask() over the slice index
     *        space [0, kNumSliceSlots).  The default degenerates to a
     *        per-slot wait loop sharing the deadline.
     */
    virtual uint32_t waitSliceSlotMask(uint32_t slice_mask,
                                       const uint64_t* tokens,
                                       uint32_t timeout_ns,
                                       CyclicSlotView* views) {
        uint32_t arrived = 0;
        const uint64_t deadline = monoNowNsFallback() + timeout_ns;
        for (uint8_t s = 0; s < kNumSliceSlots; ++s) {
            if (!(slice_mask & (1u << s))) continue;
            const uint64_t now = monoNowNsFallback();
            const uint32_t remain = now < deadline
                ? static_cast<uint32_t>(deadline - now) : 0;
            if (!waitSliceSlotView(s, tokens[s], remain, views[s]))
                break;
            arrived |= 1u << s;
        }
        return arrived;
    }
    /// Byte offset of the datagram payload in a composed cyclic TX frame
    /// (26 untagged, 30 with a baked TX VLAN tag).
    virtual uint32_t cyclicPayloadOffset() const { return 26; }

    /**
     * @brief Send one frame carrying several fastpath-band datagrams.
     *
     * All datagrams ride the same Ethernet frame (M-bit chaining) — the
     * pooled LRW exchange uses it to append the 64-bit counter trailer
     * and to read the configured slave's DC timepoint in the same frame
     * as the PDO data.  Every `idx` must satisfy isSlotIdx().
     * @return true if the frame was handed to the wire; false when the
     *         transport cannot compose multi-datagram cyclic frames.
     */
    virtual bool sendPoolFrame(const CyclicDgramSpec* dgs, size_t count) {
        (void)dgs; (void)count; return false;
    }

    /**
     * @brief Wait for deposits on an arbitrary list of rotating-pool
     *        positions — one wake re-scans the whole list.
     *
     * @param positions  Pool positions (0..kCyclicDcPoolPos)
     * @param tokens     Per-position seq tokens taken at send time
     * @param count      List length
     * @param timeout_ns Shared deadline budget
     * @param views      Output — filled for arrived positions only
     * @param arrived    Output — set per arrived position
     * @return Number of arrived positions.  The default degenerates to
     *         a per-position waitCyclicSlotView() loop sharing the
     *         deadline.
     */
    virtual uint8_t waitCyclicPool(const uint8_t* positions,
                                   const uint64_t* tokens,
                                   uint8_t count, uint32_t timeout_ns,
                                   CyclicSlotView* views, bool* arrived) {
        if (!positions || !tokens || !views || !arrived || !count) return 0;
        const uint64_t deadline = monoNowNsFallback() + timeout_ns;
        uint8_t n = 0;
        for (uint8_t i = 0; i < count; ++i) {
            const uint64_t now = monoNowNsFallback();
            const uint32_t remain = now < deadline
                ? static_cast<uint32_t>(deadline - now) : 0;
            arrived[i] = waitCyclicSlotView(positions[i], tokens[i],
                                            remain, views[i]);
            if (arrived[i]) ++n;
        }
        return n;
    }

    virtual bool writeRegister(uint16_t adp, uint16_t ado,
                               const void* data, uint16_t len,
                               unsigned int timeout_ms) = 0;

    virtual bool readRegister(uint16_t adp, uint16_t ado,
                              void* data, uint16_t len,
                              unsigned int timeout_ms) = 0;

    virtual bool sendSingleDatagram(Command cmd, uint8_t idx,
                                    uint16_t adp, uint16_t ado,
                                    const void* data, uint16_t datalen,
                                    bool roundtrip) = 0;

    virtual size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) = 0;

    virtual bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                    RxDatagram& out) = 0;

    /// Pre-register a response waiter slot for @p idx BEFORE sending the frame.
    /// This avoids the send-then-register race when multiple datagrams share
    /// one frame.  Returns a slot handle on success.  kPreRegInvalid means
    /// the transport does not support pre-registration; kPreRegBusy means
    /// a live waiter owns @p idx's slot — the caller must NOT steal it and
    /// should retry with a different idx (or give up).  A successful claim
    /// prunes the slot: it atomically invalidates any stale registration
    /// state that belonged to an earlier request on the same idx.
    virtual size_t preRegisterResponseWaiter(uint8_t idx,
                                             uint8_t* buffer, size_t buffer_size) {
        (void)idx; (void)buffer; (void)buffer_size;
        return static_cast<size_t>(-1);
    }

    /// Wait for a previously pre-registered response.  Fills @p out on success.
    virtual bool waitForPreRegistered(size_t slot, unsigned int timeout_ms,
                                      RxDatagram& out) {
        (void)slot; (void)timeout_ms; (void)out;
        return false;
    }

    /// Cancel a previously pre-registered waiter slot.  Called when the
    /// batched send path is abandoned so the slot doesn't linger with a
    /// dangling response buffer.
    virtual void cancelPreRegistered(size_t slot) { (void)slot; }

    /// Sentinel returned by preRegisterResponseWaiter when unsupported.
    static constexpr size_t kPreRegInvalid = static_cast<size_t>(-1);
    /// Sentinel returned by preRegisterResponseWaiter when @p idx's slot is
    /// owned by a live waiter — pick another idx, never steal it.
    static constexpr size_t kPreRegBusy = static_cast<size_t>(-2);

    virtual uint8_t  allocIdx() = 0;
    virtual uint16_t adpForSlaveIndex(uint16_t slave_index) = 0;

    // ------------------------------------------------------------------
    // Stall recovery hooks (used by LogicalAddressManager self-heal).
    // ------------------------------------------------------------------

    /**
     * @brief Best-effort drain of already-received wire frames through
     *        normal routing.
     *
     * Called by the LRW exchange before claiming a response slot, after a
     * host stall, and on response timeout: queued responses for
     * still-pending slots are DELIVERED (they are legitimate late replies
     * — draining gives in-flight requests their chance to complete) and
     * frames for dead slots fall into the unrouted counter.  Draining the
     * kernel backlog BEFORE the next registration keeps a stale echo from
     * satisfying a NEW waiter on a reused idx.
     *
     * @return frames drained (0 if unsupported — e.g. no direct-receive
     *         path under VLAN encapsulation).
     */
    virtual int drainWire(int max_frames) {
        (void)max_frames; return 0;
    }

    /**
     * @brief Drop every pending response waiter — explicit escape hatch.
     *
     * Frees all TransactionRouter slots; waiters wake into their timeout
     * path.  NOT called by the LRW exchange self-heal: polled exchanges
     * only ever prune the slot they transmit on (claim + drain), so
     * unrelated async waiters keep their chance to be fulfilled.
     */
    virtual void purgePendingResponses() {}

    /**
     * @brief Human-readable diagnosis of the TX path for the LAM's
     *        send-failure escalation worker.
     *
     * Runs on a dedicated NON-RT diagnostic thread — safe to block,
     * perform ioctls and read sysfs.  Should report link/carrier state,
     * pending socket errors and NIC drop counters where available.
     *
     * @return diagnosis text; empty string = unsupported.
     */
    virtual std::string txFailureDiagnostics() { return {}; }

    /// errno from the most recent failed TX send (0 = none/unsupported).
    /// The kernel's own verdict for the failure-path log: ENOBUFS = TX
    /// ring wedged / carrier lost, ENETDOWN = interface down, ENODEV =
    /// interface gone, EPERM = filtered, EAGAIN = nonblocking queue full.
    virtual int lastSendErrno() const { return 0; }

    /// @return true if cancellation has been requested (e.g. during shutdown).
    /// Used by callers to suppress error logging when failures are expected.
    virtual bool isCancelRequested() const { return false; }

    /// Maximum EtherCAT payload bytes that fit in one Ethernet frame,
    /// including per-datagram overhead.  Used to size partial LRW slices.
    virtual size_t maxEtherCATPayloadPerFrame() const { return 1498; }
};
} // namespace EtherCAT
