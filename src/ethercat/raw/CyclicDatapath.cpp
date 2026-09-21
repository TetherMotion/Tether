/**
 * @file CyclicDatapath.cpp
 * @brief CyclicDatapath — reserved-index cyclic response slots and the
 *        cyclic wire fast path (deposit/publish/wait/dispatch,
 *        generation bits); also holds the thin Master:: forwarders that
 *        keep the public/test API unchanged.
 */

#include "raw/CyclicDatapath.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "raw/internal.hpp"
#include "raw/PacketDebugger.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"

#include <chrono>
#include <algorithm>
#include <cstring>
#include <inttypes.h>
#ifdef __linux__
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// CyclicDatapath lifecycle
// ============================================================================

CyclicDatapath::CyclicDatapath(Master& master) : master_(master)
{
#ifdef __linux__
    notify_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
#endif
}

CyclicDatapath::~CyclicDatapath()
{
    // Release ring/bank cookies held by the response slots.
    if (channel_) {
        for (auto& s : slots_) {
            if (s.cookie >= 0)
                channel_->rxRelease(static_cast<uint32_t>(s.cookie));
        }
    }
#ifdef __linux__
    if (notify_fd_ >= 0) {
        ::close(notify_fd_);
    }
#endif
}


void CyclicDatapath::deposit(uint8_t slot_idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc, uint8_t gen)
{
    auto& s = slots_[slot_idx];
    // A copy-mode deposit replaces any held channel view — release the old
    // cookie first so ring slots aren't pinned forever.
    if (s.cookie >= 0 && channel_) {
        channel_->rxRelease(static_cast<uint32_t>(s.cookie));
    }
    s.cookie  = -1;
    s.payload = s.data;
    s.cmd     = static_cast<uint8_t>(cmd);
    s.adp     = adp;
    s.ado     = ado;
    s.datalen = datalen;
    s.wkc     = wkc;
    s.gen     = gen;
    s.stamp_ns = Tether::Platform::Clock::instance().getMicroseconds() * 1000;
    if (datalen > 0) {
        std::memcpy(s.data, payload,
                    std::min<size_t>(datalen, sizeof(s.data)));
    }
    // seq_cst publish (Q13): the waiter's token compare must see a total
    // order with this bump — acquire/release leaves an S-store-buffer
    // window where a waiter can observe the new seq before the payload
    // fields on weak memory.
    s.seq.fetch_add(1, std::memory_order_seq_cst);

#ifdef __linux__
    // Wake a cyclic thread blocked in ppoll() — needed when the poll thread
    // won the recv race and this deposit happened on the "wrong" fd.  The
    // write is gated on a registered waiter: with the BPF demux working,
    // deposits almost never coincide with a sleep, so the idle path pays
    // no syscall per frame.
    if (notify_fd_ >= 0 &&
        waiters_.load(std::memory_order_acquire) > 0) {
        const uint64_t one = 1;
        ssize_t r = ::write(notify_fd_, &one, sizeof(one));
        (void)r;  // EAGAIN (counter saturated) is fine — waiter already woken
    }
#endif
}

void CyclicDatapath::publishView(uint8_t slot_idx, Command cmd,
                                   uint16_t adp, uint16_t ado,
                                   const uint8_t* payload, uint16_t datalen,
                                   uint16_t wkc, uint32_t cookie,
                                   uint64_t stamp_ns, uint8_t gen)
{
    auto& s = slots_[slot_idx];
    if (s.cookie >= 0 && channel_) {
        channel_->rxRelease(static_cast<uint32_t>(s.cookie));
    }
    s.cookie  = static_cast<int64_t>(cookie);
    s.payload = payload;
    s.cmd     = static_cast<uint8_t>(cmd);
    s.adp     = adp;
    s.ado     = ado;
    s.datalen = datalen;
    s.wkc     = wkc;
    s.gen     = gen;
    // The channel's kernel stamp when present, else stamp at deposit.
    s.stamp_ns = stamp_ns ? stamp_ns
        : Tether::Platform::Clock::instance().getMicroseconds() * 1000;
    s.seq.fetch_add(1, std::memory_order_seq_cst);   // see depositCyclicSlot

#ifdef __linux__
    if (notify_fd_ >= 0 &&
        waiters_.load(std::memory_order_acquire) > 0) {
        const uint64_t one = 1;
        ssize_t r = ::write(notify_fd_, &one, sizeof(one));
        (void)r;
    }
#endif
}

uint64_t CyclicDatapath::slotToken(uint8_t slot) const
{
    if (slot >= IPDOTransport::kNumCyclicSlots) return 0;
    return slots_[slot].seq.load(std::memory_order_seq_cst);
}

uint8_t CyclicDatapath::slotGen(uint8_t slot) const
{
    if (slot >= IPDOTransport::kNumCyclicSlots) return 0;
    return slot_gen_[slot];
}

namespace {

/// Build the 26-byte [eth][ecat][datagram-hdr] prefix of a cyclic frame.
/// `gen` stamps the send-generation bit into lenFlags res-bit 13 — echoed
/// verbatim by the ring so collect can reject stale deposits.
void buildCyclicFrameHeader(uint8_t* buf, const uint8_t* src_mac,
                            EtherCAT::Command cmd, uint8_t slot,
                            uint16_t adp, uint16_t ado, uint16_t datalen,
                            bool roundtrip, uint8_t gen)
{
    using namespace EtherCAT::Raw;
    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    auto* hdr = reinterpret_cast<EtherCATSingleDgramFrameHeader*>(buf);
    std::memcpy(hdr->eth.dst, dst_mac, 6);
    std::memcpy(hdr->eth.src, src_mac, 6);
    hdr->eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);

    const uint16_t payload_len = static_cast<uint16_t>(
        sizeof(EtherCATDatagramHeader) + datalen + sizeof(uint16_t));
    constexpr uint16_t type = 0x1;
    hdr->ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu) | ((type & 0x0Fu) << 12)));

    hdr->dg.cmd    = cmd;
    hdr->dg.idx    = EtherCAT::IPDOTransport::kCyclicSlotBase + slot;
    hdr->dg.adp_le = host_to_le16(adp);
    hdr->dg.ado_le = host_to_le16(ado);
    const uint16_t flags =
        (roundtrip ? (1u << 14) : 0u) |
        (static_cast<uint16_t>(gen & 0x1u) << 13);
    hdr->dg.lenFlags.raw_le =
        host_to_le16(static_cast<uint16_t>((datalen & 0x07FFu) | flags));
    hdr->dg.irq_le = host_to_le16(0);
}

} // namespace

bool CyclicDatapath::sendDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    using namespace Raw;
    if (slot >= IPDOTransport::kNumCyclicSlots) return false;
    if (master_.cancel_requested_.load(std::memory_order_acquire)) return false;

    // Toggle the send generation — collect rejects echoes of the previous
    // generation (stale deposits surviving a timed-out cycle).
    const uint8_t gen =
        (slot_gen_[slot] = slot_gen_[slot] ^ 1u);

    // Channel path (socket backend): header on the stack + payload in place
    // → one sendmsg(), zero payload copies.  Ring backend would copy once
    // into a TX slot — callers in Rotating image mode use sendCyclicFrame
    // to avoid even that.
    if (channel_) {
        uint8_t hdr_buf[kCyclicFramePayloadOff];
        buildCyclicFrameHeader(hdr_buf, master_.src_mac_, cmd, slot, adp, ado,
                               datalen, roundtrip, gen);
        CyclicTxParts parts;
        parts.header      = hdr_buf;
        parts.header_len  = kCyclicFramePayloadOff;
        parts.payload     = static_cast<const uint8_t*>(data);
        parts.payload_len = datalen;
        parts.wkc         = 0;
        if (master_.debug_flags_.txPackets) {
            // Compose into tx_buf_ purely for the debug dump.
            std::memcpy(tx_buf_, hdr_buf, sizeof(hdr_buf));
            uint8_t* p = tx_buf_ + sizeof(hdr_buf);
            if (datalen > 0) std::memcpy(p, data, datalen);
            *reinterpret_cast<uint16_t*>(p + datalen) = host_to_le16(0);
            PacketDebugger::printEtherCATFrame(tx_buf_,
                kCyclicFramePayloadOff + datalen + sizeof(uint16_t),
                true, false);
        }
        return channel_->txSendParts(parts);
    }

    constexpr size_t kMinEthFrameNoFcs = 60;

    const size_t required_len =
        sizeof(EtherCATSingleDgramFrameHeader) + datalen + sizeof(uint16_t);
    if (required_len > sizeof(tx_buf_)) {
        return false;
    }
    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;

    buildCyclicFrameHeader(tx_buf_, master_.src_mac_, cmd, slot, adp, ado,
                           datalen, roundtrip, gen);

    uint8_t* payload = tx_buf_ + sizeof(EtherCATSingleDgramFrameHeader);
    if (datalen > 0) {
        if (data) std::memcpy(payload, data, datalen);
        else      std::memset(payload, 0, datalen);
    }
    *reinterpret_cast<uint16_t*>(payload + datalen) = host_to_le16(0);

    if (master_.debug_flags_.txPackets) {
        PacketDebugger::printEtherCATFrame(tx_buf_, frame_len, true, false);
    }
    // Single attempt — no retry spin on the cyclic path; a missed cycle is
    // reported via the error counter rather than retried inside the deadline.
    return master_.sendWithEncapsulation(tx_buf_, frame_len);
}

uint8_t* CyclicDatapath::acquireTxFrame()
{
    return channel_ ? channel_->txAcquire() : nullptr;
}

bool CyclicDatapath::sendFrame(uint32_t frame_len)
{
    if (!channel_) return false;
    return channel_->txCommitFrame(frame_len);
}

void CyclicDatapath::composeHeader(uint8_t* frame, Command cmd, uint8_t slot,
                                 uint16_t adp, uint16_t ado, uint16_t datalen,
                                 bool roundtrip)
{
    const uint8_t gen =
        (slot_gen_[slot] = slot_gen_[slot] ^ 1u);
    buildCyclicFrameHeader(frame, master_.src_mac_, cmd, slot, adp, ado,
                           datalen, roundtrip, gen);
}

/**
 * @brief Walk a frame received on the cyclic channel.
 *
 * Pure-cyclic frames (all datagrams idx >= kCyclicSlotBase) publish their
 * payload views into the slots without copying.  Anything else (async or
 * mixed frames — only possible when the BPF demux is absent/failed) is
 * routed through the regular parser, which deposits/routes per datagram.
 */
void CyclicDatapath::dispatchFrame(const CyclicFrameView& v)
{
    using namespace Raw;
    const uint8_t* f    = v.frame;
    const size_t   flen = v.frame_len;
    constexpr size_t kEcatHdr = sizeof(EtherCAT::EthernetHeader);       // 14
    constexpr size_t kDgHdr   = sizeof(EtherCAT::DatagramHeader);       // 10
    // Q15: with the VLAN-aware demux a 0x8100-tagged frame can arrive on
    // the cyclic socket — the EtherCAT header sits 4 bytes deeper.
    size_t base = kEcatHdr;
    if (flen >= kEcatHdr + 4 &&
        f[12] == 0x81 && f[13] == 0x00) {
        base += 4;
    }
    if (flen < base + sizeof(EtherCAT::FrameHeader)) return;

    const uint16_t ec_len = le16_to_host(
        *reinterpret_cast<const uint16_t*>(f + base)) & 0x07FFu;
    if (flen < base + sizeof(EtherCAT::FrameHeader) + ec_len) return;

    // Classify pass: any non-cyclic idx → whole frame to the parser.
    size_t off = base + sizeof(EtherCAT::FrameHeader);
    size_t rem = ec_len;
    bool   all_cyclic = true;
    while (rem >= kDgHdr + sizeof(uint16_t) && off + kDgHdr <= flen) {
        const uint8_t  idx       = f[off + 1];
        const uint16_t len_flags = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 6));
        const uint16_t dl   = len_flags & 0x07FFu;
        const bool     more = (len_flags & 0x8000u) != 0;
        if (idx <  IPDOTransport::kCyclicSlotBase ||
            idx >= IPDOTransport::kCyclicSlotBase +
                   IPDOTransport::kNumCyclicSlots) {
            all_cyclic = false;
            break;
        }
        const size_t dgt = kDgHdr + dl + sizeof(uint16_t);
        if (rem < dgt || off + dgt > flen) { all_cyclic = false; break; }
        rem -= dgt; off += dgt;
        if (!more) break;
    }

    if (!all_cyclic) {
        master_.handleRxFrame(f, flen);   // parser deposits/routes per datagram
        return;
    }

    // Publish pass — every datagram is cyclic; hand payload views to slots.
    off = base + sizeof(EtherCAT::FrameHeader);
    rem = ec_len;
    while (rem >= kDgHdr + sizeof(uint16_t) && off + kDgHdr <= flen) {
        const uint8_t  idx       = f[off + 1];
        const uint8_t  cmd       = f[off];
        const uint16_t adp       = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 2));
        const uint16_t ado       = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 4));
        const uint16_t len_flags = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 6));
        const uint16_t dl   = len_flags & 0x07FFu;
        const bool     more = (len_flags & 0x8000u) != 0;
        const size_t   data_off = off + kDgHdr;
        const size_t   dgt      = kDgHdr + dl + sizeof(uint16_t);
        if (rem < dgt || off + dgt > flen) break;
        const uint16_t wkc = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + data_off + dl));

        channel_->rxHold(v.cookie);
        publishView(idx - IPDOTransport::kCyclicSlotBase,
                              static_cast<Command>(cmd), adp, ado,
                              f + data_off, dl, wkc, v.cookie, v.stamp_ns,
                              static_cast<uint8_t>((len_flags >> 13) & 0x1u));
        rem -= dgt; off += dgt;
        if (!more) break;
    }
}

bool CyclicDatapath::waitView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    if (slot >= IPDOTransport::kNumCyclicSlots) return false;
    auto& s = slots_[slot];

    auto read_slot = [&]() -> bool {
        // Seqlock read: the publisher writes the fields then bumps seq
        // (release).  Re-check seq after the read — a second publish
        // landing mid-read is detected and retried.
        for (int tries = 0; tries < 8; ++tries) {
            const uint64_t s0 = s.seq.load(std::memory_order_acquire);
            out.cmd     = s.cmd;
            out.adp     = s.adp;
            out.ado     = s.ado;
            out.datalen = s.datalen;
            out.wkc     = s.wkc;
            out.gen     = s.gen;
            out.stamp_ns = s.stamp_ns;
            out.payload = s.payload ? s.payload : s.data;
            const int64_t cookie = s.cookie;
            out.cookie  = cookie >= 0 ? static_cast<uint32_t>(cookie) : 0;
            out.channel = cookie >= 0 ? channel_.get() : nullptr;
            if (s.seq.load(std::memory_order_acquire) == s0) return true;
        }
        return true;   // accept last read — publisher is single-writer
    };

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    // Fast path: response already deposited.  seq_cst pairs with the
    // deposit's publish bump (Q13).
    if (s.seq.load(std::memory_order_seq_cst) != token) {
        return read_slot();
    }

    // Register as a waiter BEFORE any blocking point so a deposit landing
    // between the fast-path check and the sleep still wakes us (deposit
    // paths write the eventfd only while waiters_ > 0).  The seq
    // re-check inside the loop covers deposits that raced ahead of the
    // registration.
    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{waiters_};
    waiters_.fetch_add(1, std::memory_order_acq_rel);

    // Unified wait: one ppoll over every wake source — the deposit eventfd
    // (covers frames consumed by the poll thread / socket-B deposits when
    // the BPF is absent) and the wire fd (the channel's own socket, or the
    // iface socket on the no-channel path).
#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (channel_) {
        wire_fd = channel_->fd();
    } else if (master_.iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        if (s.seq.load(std::memory_order_seq_cst) != token) {
            return read_slot();
        }
        if (master_.cancel_requested_.load(std::memory_order_acquire)) return false;
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        int64_t remain = deadline_ns - now_ns;
        if (remain <= 0) return false;

        if (channel_) {
            // Spin phase: the ring backend's rxPending() is pure memory
            // reads — a NIC DMA write becomes visible before the kernel
            // could ever wake a ppoll() sleeper.  Shares the deadline
            // budget; skipped entirely when slot_spin_ns_ == 0.  (This is
            // the *slot-wait* spin — rx_spin_ns_ is the channel's own
            // rxPoll busy-poll, a separate knob, Q11.)
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(remain, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    if (s.seq.load(std::memory_order_seq_cst) != token) {
                        return read_slot();
                    }
                    if (channel_->rxPending()) break;
                }
            }
            CyclicFrameView views[8];
            const int n = channel_->rxPoll(views, 8, 0);
            for (int i = 0; i < n; ++i) dispatchFrame(views[i]);
            if (n > 0) continue;      // re-check slots before sleeping
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;   // deadline check at loop top
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            if (ret == 0) return false;  // deadline reached

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (channel_) {
                    CyclicFrameView views[8];
                    const int n = channel_->rxPoll(views, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchFrame(views[i]);
                } else {
                    // Bounded drain: an async burst must not burn the
                    // whole RX budget — POLLIN stays asserted so the
                    // remaining frames are handled next iteration.
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!master_.iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        master_.handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        // No fd wake-up path (non-Linux / no eventfd / no wire fd): loop —
        // a bounded spin on the sequence counter plus channel drain, with
        // the seq/cancel/deadline checks at the top bounding it so a dead
        // link cannot wedge the cyclic thread.  Q22: the fallback policy
        // is user-configurable — Yield (default) drops the thread's time
        // slice each pass; Spin keeps the tightest re-check cadence at
        // full CPU burn.
        if (slot_wait_fallback_ ==
            Master::CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
}

uint32_t CyclicDatapath::waitMask(uint32_t slot_mask,
                                    const uint64_t* tokens,
                                    uint32_t timeout_ns,
                                    CyclicSlotView* views)
{
    // Q3: one wake for the whole mask — a sliced collect pays a single
    // ppoll registration instead of one sleep per slice.  Each wake
    // re-scans all outstanding bits; a deposit on ANY slot is progress.
    slot_mask &= (1u << IPDOTransport::kNumCyclicSlots) - 1u;
    if (!slot_mask || !tokens || !views) return 0;

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_ns =
        clock.getMicroseconds() * 1000 + static_cast<int64_t>(timeout_ns);

    // Outstanding bits → arrived so far; fill views for arrived slots.
    auto scan = [&]() -> uint32_t {
        uint32_t outstanding = 0;
        for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots; ++s) {
            if (!(slot_mask & (1u << s))) continue;
            if (slots_[s].seq.load(std::memory_order_seq_cst)
                != tokens[s]) continue;
            outstanding |= 1u << s;
        }
        return outstanding;
    };
    auto fill = [&](uint32_t arrived) {
        for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots; ++s) {
            if (!(arrived & (1u << s))) continue;
            auto& slot = slots_[s];
            for (int tries = 0; tries < 8; ++tries) {
                const uint64_t s0 = slot.seq.load(std::memory_order_acquire);
                views[s].cmd     = slot.cmd;
                views[s].adp     = slot.adp;
                views[s].ado     = slot.ado;
                views[s].datalen = slot.datalen;
                views[s].wkc     = slot.wkc;
                views[s].stamp_ns = slot.stamp_ns;
                views[s].payload = slot.payload ? slot.payload : slot.data;
                const int64_t cookie = slot.cookie;
                views[s].cookie  = cookie >= 0
                    ? static_cast<uint32_t>(cookie) : 0;
                views[s].channel = cookie >= 0 ? channel_.get()
                                               : nullptr;
                if (slot.seq.load(std::memory_order_acquire) == s0) break;
            }
        }
    };

    uint32_t outstanding = scan();
    if (!outstanding) { fill(slot_mask); return slot_mask; }

    struct WaiterGuard {
        std::atomic<int>& c;
        ~WaiterGuard() { c.fetch_sub(1, std::memory_order_acq_rel); }
    } waiter_guard{waiters_};
    waiters_.fetch_add(1, std::memory_order_acq_rel);

#ifdef __linux__
    struct pollfd fds[2];
    nfds_t nfds = 0;
    int efd_pos = -1, wire_pos = -1;
    if (notify_fd_ >= 0) {
        efd_pos = static_cast<int>(nfds);
        fds[nfds++] = { notify_fd_, POLLIN, 0 };
    }
    int wire_fd = -1;
    if (channel_) {
        wire_fd = channel_->fd();
    } else if (master_.iface_.receive) {
        wire_fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
    }
    if (wire_fd >= 0) {
        wire_pos = static_cast<int>(nfds);
        fds[nfds++] = { wire_fd, POLLIN, 0 };
    }
#endif

    while (true) {
        outstanding = scan();
        if (!outstanding) { fill(slot_mask); return slot_mask; }
        if (master_.cancel_requested_.load(std::memory_order_acquire)) {
            const uint32_t arrived = slot_mask & ~outstanding;
            fill(arrived);
            return arrived;
        }
        const int64_t now_ns = clock.getMicroseconds() * 1000;
        if (deadline_ns - now_ns <= 0) {
            const uint32_t arrived = slot_mask & ~outstanding;
            fill(arrived);
            return arrived;
        }

        if (channel_) {
            const uint32_t spin_ns = slot_spin_ns_;
            if (spin_ns > 0) {
                const int64_t spin_end = now_ns +
                    std::min<int64_t>(deadline_ns - now_ns, spin_ns);
                while (clock.getMicroseconds() * 1000 < spin_end) {
                    bool any = false;
                    for (uint8_t s = 0; s < IPDOTransport::kNumCyclicSlots;
                         ++s) {
                        if (!(slot_mask & (1u << s))) continue;
                        if (slots_[s].seq.load(
                                std::memory_order_seq_cst) != tokens[s]) {
                            any = true; break;
                        }
                    }
                    if (any || channel_->rxPending()) break;
                }
            }
            CyclicFrameView fviews[8];
            const int n = channel_->rxPoll(fviews, 8, 0);
            for (int i = 0; i < n; ++i) dispatchFrame(fviews[i]);
            if (n > 0) continue;
        }

#ifdef __linux__
        if (nfds > 0) {
            const int64_t remain2 = deadline_ns -
                                    clock.getMicroseconds() * 1000;
            if (remain2 <= 0) continue;
            struct timespec ts;
            ts.tv_sec  = remain2 / 1'000'000'000LL;
            ts.tv_nsec = remain2 % 1'000'000'000LL;
            const int ret = ppoll(fds, nfds, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                outstanding = scan();
                const uint32_t arrived = slot_mask & ~outstanding;
                fill(arrived);
                return arrived;
            }
            if (ret == 0) break;   // deadline reached — final scan below

            if (wire_pos >= 0 && (fds[wire_pos].revents & POLLIN)) {
                if (channel_) {
                    CyclicFrameView fviews[8];
                    const int n = channel_->rxPoll(fviews, 8, 0);
                    for (int i = 0; i < n; ++i)
                        dispatchFrame(fviews[i]);
                } else {
                    uint8_t buf[1600];
                    for (int i = 0; i < 8; ++i) {
                        size_t n = 0;
                        if (!master_.iface_.receive(buf, sizeof(buf), &n) || n == 0)
                            break;
                        master_.handleRxFrame(buf, n);
                    }
                }
            }
            if (efd_pos >= 0 && (fds[efd_pos].revents & POLLIN)) {
                uint64_t v;
                ssize_t r = ::read(notify_fd_, &v, sizeof(v));
                (void)r;
            }
            continue;
        }
#endif

        if (slot_wait_fallback_ ==
            Master::CyclicLoopConfig::SlotWaitFallback::Yield) {
            std::this_thread::yield();
        }
    }
    // ppoll deadline expired — one last scan, then report what arrived.
    outstanding = scan();
    const uint32_t arrived = slot_mask & ~outstanding;
    fill(arrived);
    return arrived;
}

bool CyclicDatapath::wait(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out)
{
    CyclicSlotView view{};
    if (!waitView(slot, token, timeout_ns, view)) return false;
    out.idx     = IPDOTransport::kCyclicSlotBase + slot;
    out.cmd     = static_cast<Command>(view.cmd);
    out.adp     = view.adp;
    out.ado     = view.ado;
    out.datalen = view.datalen;
    out.wkc     = view.wkc;
    if (view.datalen > 0 && view.payload) {
        std::memcpy(out.data, view.payload,
                    std::min<size_t>(view.datalen, sizeof(out.data)));
    }
    return true;
}


// ============================================================================
// Shared cyclic datapath bring-up/teardown (cyclic + async loops)
// ============================================================================

void CyclicDatapath::setup(CyclicWireMode wire_mode,
                                 ImageMode image_mode,
                                 const std::string& shm_image_name,
                                 uint32_t rx_spin_ns,
                                 uint32_t slot_spin_ns,
                                 Master::CyclicLoopConfig::SlotWaitFallback
                                     slot_fallback,
                                 bool strict_wkc,
                                 const Master::MemoryLockConfig& memlock)
{
    rx_spin_ns_         = rx_spin_ns;
    slot_spin_ns_       = slot_spin_ns;
    slot_wait_fallback_ = slot_fallback;
    // The channel needs a raw AF_PACKET fd — only the direct-EtherCAT path
    // exposes one (master_.iface_.receive/native_handle are stripped for VLAN and
    // absent under UDP encapsulation or polling transports).
    channel_.reset();
    active_image_mode_ = ImageMode::Buffered;
    image_.configure({});

#ifdef __linux__
    if (master_.iface_.receive && master_.iface_.native_handle &&
        !master_.isUdpEncapsulationEnabled()) {
        const int fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
        sockaddr_ll sll{};
        socklen_t sll_len = sizeof(sll);
        int ifindex = 0;
        if (::getsockname(fd, reinterpret_cast<sockaddr*>(&sll),
                          &sll_len) == 0) {
            ifindex = sll.sll_ifindex;
        }
        CyclicChannelConfig cc;
        cc.ifindex    = ifindex;
        cc.async_fd   = fd;
        cc.wire_mode  = wire_mode;
        cc.rx_spin_ns = rx_spin_ns;
        cc.frame_size = master_.config_.max_frame_size;
        channel_ = createCyclicChannel(cc);
        if (!channel_) {
            TETHER_LOGW(TAG, "cyclic channel unavailable — using software "
                             "deposit path (no ring/BPF acceleration)");
        }
    }
#endif

    ImageMode mode = image_mode;
    if (mode == ImageMode::Rotating && !channel_) {
        TETHER_LOGW(TAG, "Rotating image mode requires a cyclic channel — "
                         "falling back to Direct");
        mode = ImageMode::Direct;
    }
    if (mode != ImageMode::Buffered && master_.pdo_ &&
        master_.logical_addr_mgr_ && master_.logical_addr_mgr_->isInitialized()) {
        const char* shm = shm_image_name.empty()
                        ? nullptr : shm_image_name.c_str();
        if (master_.pdo_->configureProcessImage(image_, mode, shm)) {
            active_image_mode_ = mode;
            TETHER_LOGI(TAG, "process image active: mode={} rx={}B tx={}B{}",
                        static_cast<int>(mode),
                        image_.outputBytes(),
                        image_.inputBytes(),
                        image_.shmBacked() ? " [shm]" : "");
        } else {
            TETHER_LOGW(TAG, "process image configure failed — "
                             "buffered exchange");
        }
    }
    if (master_.pdo_) {
        master_.pdo_->setCyclicStrictWkc(strict_wkc);
    }

    // ---- Memory locking: image + cyclic buffers (opt-out sections) -----
    if (memlock.lock_image && image_.configured()) {
        // Lock whatever backing regions the configured mode uses.
        if (uint8_t* w = image_.outputWrite())
            Tether::Platform::lockMemory(w, image_.imageBytes());
        if (uint8_t* b = image_.inputWriteBank())
            Tether::Platform::lockMemory(b, image_.imageBytes());
    }
    if (memlock.lock_slots) {
        Tether::Platform::lockMemory(slots_.data(),
                                     sizeof(slots_));
        Tether::Platform::lockMemory(tx_buf_,
                                     sizeof(tx_buf_));
    }
}

void CyclicDatapath::teardown()
{
    // Drop the held input-view cookie before the channel dies.
    image_.configure({});
    // Release ring/bank cookies held by the cyclic slots, then tear down.
    if (channel_) {
        for (auto& s : slots_) {
            if (s.cookie >= 0) {
                channel_->rxRelease(static_cast<uint32_t>(s.cookie));
            }
            s.cookie  = -1;
            s.payload = nullptr;
        }
    }
    channel_.reset();
    active_image_mode_ = ImageMode::Buffered;
}

// ============================================================================

// ============================================================================
// Master forwarders — the public/test surface is unchanged; the state and
// the work live on CyclicDatapath.
// ============================================================================

uint64_t Master::cyclicSlotToken(uint8_t slot) const
{
    return datapath_->slotToken(slot);
}

uint8_t Master::cyclicSlotGen(uint8_t slot) const
{
    return datapath_->slotGen(slot);
}

bool Master::sendCyclicDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    return datapath_->sendDatagram(cmd, slot, adp, ado,
                                   data, datalen, roundtrip);
}

uint8_t* Master::acquireCyclicTxFrame() { return datapath_->acquireTxFrame(); }

void Master::composeCyclicHeader(uint8_t* frame, Command cmd, uint8_t slot,
                                 uint16_t adp, uint16_t ado, uint16_t datalen,
                                 bool roundtrip)
{
    datapath_->composeHeader(frame, cmd, slot, adp, ado, datalen, roundtrip);
}

bool Master::sendCyclicFrame(uint32_t frame_len)
{
    return datapath_->sendFrame(frame_len);
}

void Master::dispatchChannelFrame(const CyclicFrameView& v)
{
    datapath_->dispatchFrame(v);
}

bool Master::waitCyclicSlotView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    return datapath_->waitView(slot, token, timeout_ns, out);
}

uint32_t Master::waitCyclicSlotMask(uint32_t slot_mask,
                                    const uint64_t* tokens,
                                    uint32_t timeout_ns,
                                    CyclicSlotView* views)
{
    return datapath_->waitMask(slot_mask, tokens, timeout_ns, views);
}

bool Master::waitCyclicSlot(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out)
{
    return datapath_->wait(slot, token, timeout_ns, out);
}

void Master::depositCyclicSlot(uint8_t slot_idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc, uint8_t gen)
{
    datapath_->deposit(slot_idx, cmd, adp, ado, payload, datalen, wkc, gen);
}

void Master::publishCyclicSlotView(uint8_t slot_idx, Command cmd,
                                   uint16_t adp, uint16_t ado,
                                   const uint8_t* payload, uint16_t datalen,
                                   uint16_t wkc, uint32_t cookie,
                                   uint64_t stamp_ns, uint8_t gen)
{
    datapath_->publishView(slot_idx, cmd, adp, ado, payload, datalen,
                           wkc, cookie, stamp_ns, gen);
}

void Master::setupCyclicDatapath(CyclicWireMode wire_mode,
                                 ImageMode image_mode,
                                 const std::string& shm_image_name,
                                 uint32_t rx_spin_ns,
                                 uint32_t slot_spin_ns,
                                 CyclicLoopConfig::SlotWaitFallback slot_fallback,
                                 bool strict_wkc,
                                 const MemoryLockConfig& memlock)
{
    datapath_->setup(wire_mode, image_mode, shm_image_name, rx_spin_ns,
                     slot_spin_ns, slot_fallback, strict_wkc, memlock);
}

void Master::teardownCyclicDatapath()
{
    datapath_->teardown();
}

ICyclicChannel* Master::cyclicChannel() const
{
    return datapath_->channel_.get();
}

ProcessImage& Master::processImage() { return datapath_->image_; }
const ProcessImage& Master::processImage() const { return datapath_->image_; }

bool Master::cyclicExchangeSuspended() const
{
    return datapath_->suspended();
}

} // namespace EtherCAT
