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


void CyclicDatapath::deposit(uint8_t idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc, uint8_t gen)
{
    // isSlotIdx, not isFastPathIdx: the 0xF0..0xF7 filter-reserved gap
    // maps to no slot — fastIndex() would produce a garbage index.
    if (!isSlotIdx(idx)) return;
    auto& s = slots_[fastIndex(idx)];
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
    // Boot-time monotonic — same domain as monoNowNs()/kernel stamps the
    // channel emits; Clock::getMicroseconds() is process-relative and
    // would make RTT math underflow.
    s.stamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
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

void CyclicDatapath::publishView(uint8_t idx, Command cmd,
                                   uint16_t adp, uint16_t ado,
                                   const uint8_t* payload, uint16_t datalen,
                                   uint16_t wkc, uint32_t cookie,
                                   uint64_t stamp_ns, uint8_t gen)
{
    if (!isSlotIdx(idx)) return;   // 0xF0..0xF7 gap has no slot
    auto& s = slots_[fastIndex(idx)];
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
    // The channel's kernel stamp when present (already monotonic), else
    // stamp at deposit — same boot-time monotonic domain as the channel.
    s.stamp_ns = stamp_ns ? stamp_ns : static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
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
    if (slot > kCyclicDcPoolPos) return 0;
    return slots_[cyclicFastIndex(slot)].seq.load(std::memory_order_seq_cst);
}

uint8_t CyclicDatapath::slotGen(uint8_t slot) const
{
    if (slot > kCyclicDcPoolPos) return 0;
    return slot_gen_[cyclicFastIndex(slot)];
}

uint64_t CyclicDatapath::sliceSlotToken(uint8_t slice) const
{
    if (slice >= kNumSliceSlots) return 0;
    return slots_[sliceFastIndex(slice)].seq.load(std::memory_order_seq_cst);
}

uint8_t CyclicDatapath::sliceSlotGen(uint8_t slice) const
{
    if (slice >= kNumSliceSlots) return 0;
    return slot_gen_[sliceFastIndex(slice)];
}

const CyclicDatapath::HdrTemplate& CyclicDatapath::ensureTemplate(
    uint8_t fast_idx, uint8_t gen, Command cmd,
    uint16_t adp, uint16_t ado, uint16_t datalen, bool roundtrip)
{
    auto& slot_tmpl = hdr_tmpl_[fast_idx];
    auto& t = slot_tmpl[gen & 1u];
    if (t.valid &&
        t.cmd == static_cast<uint8_t>(cmd) && t.adp == adp &&
        t.ado == ado && t.datalen == datalen &&
        t.roundtrip == roundtrip) {
        return t;
    }

    // Key changed — bake BOTH gen variants so the next cycle's toggled
    // send hits a ready template too.  Everything below is invariant per
    // (index, gen): MACs, EtherType/VLAN tag, ECAT header, datagram
    // header — assembled once, memcpy'd per send.
    using namespace Raw;
    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    for (uint8_t g = 0; g < 2; ++g) {
        auto& d = slot_tmpl[g];
        uint8_t* p = d.bytes;
        std::memcpy(p, dst_mac, 6);
        std::memcpy(p + 6, master_.src_mac_, 6);
        p += 12;
        if (tx_vlan_) {
            // Inline 802.1Q tag — the demux/dispatch already parse it.
            *reinterpret_cast<uint16_t*>(p) =
                host_to_be16(kEtherType8021Q);
            p += 2;
            *reinterpret_cast<uint16_t*>(p) = host_to_be16(tx_vlan_);
            p += 2;
        }
        *reinterpret_cast<uint16_t*>(p) =
            host_to_be16(kEtherTypeEtherCAT);
        p += 2;
        const uint16_t payload_len = static_cast<uint16_t>(
            sizeof(EtherCATDatagramHeader) + datalen + sizeof(uint16_t));
        *reinterpret_cast<uint16_t*>(p) = host_to_le16(
            static_cast<uint16_t>((payload_len & 0x07FFu) | (0x1u << 12)));
        p += 2;
        p[0] = static_cast<uint8_t>(cmd);
        p[1] = wireIndex(fast_idx);
        *reinterpret_cast<uint16_t*>(p + 2) = host_to_le16(adp);
        *reinterpret_cast<uint16_t*>(p + 4) = host_to_le16(ado);
        const uint16_t flags =
            (roundtrip ? (1u << 14) : 0u) |
            (static_cast<uint16_t>(g) << 13);
        *reinterpret_cast<uint16_t*>(p + 6) = host_to_le16(
            static_cast<uint16_t>((datalen & 0x07FFu) | flags));
        *reinterpret_cast<uint16_t*>(p + 8) = host_to_le16(0);   // irq
        p += 10;
        d.len       = static_cast<uint16_t>(p - d.bytes);
        d.cmd       = static_cast<uint8_t>(cmd);
        d.adp       = adp;
        d.ado       = ado;
        d.datalen   = datalen;
        d.roundtrip = roundtrip;
        d.valid     = true;
    }
    return slot_tmpl[gen & 1u];
}

bool CyclicDatapath::sendFastDatagram(Command cmd, uint8_t fast_idx,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    using namespace Raw;
    if (fast_idx >= kNumFastSlots) return false;
    if (master_.cancel_requested_.load(std::memory_order_acquire)) return false;

    // Toggle the send generation — collect rejects echoes of the previous
    // generation (stale deposits surviving a timed-out cycle).
    const uint8_t gen =
        (slot_gen_[fast_idx] = slot_gen_[fast_idx] ^ 1u);
    const HdrTemplate& hdr = ensureTemplate(fast_idx, gen, cmd, adp, ado,
                                            datalen, roundtrip);

    // Channel path (socket backend): template + payload in place → one
    // sendmsg(), zero payload copies, zero header assembly.  Ring backend
    // would copy once into a TX slot — callers in Rotating image mode use
    // sendCyclicFrame to avoid even that.
    static const uint8_t kZeroPayload[kMaxDatagramDataSize] = {};
    if (channel_) {
        const uint8_t* payload_src = static_cast<const uint8_t*>(data);
        if (!payload_src) payload_src = kZeroPayload;   // read datagrams
        CyclicTxParts parts;
        parts.header      = hdr.bytes;
        parts.header_len  = hdr.len;
        parts.payload     = payload_src;
        parts.payload_len = datalen;
        parts.wkc         = 0;
        if (master_.debug_flags_.txPackets) {
            // Compose into tx_buf_ purely for the debug dump.
            std::memcpy(tx_buf_, hdr.bytes, hdr.len);
            uint8_t* p = tx_buf_ + hdr.len;
            if (datalen > 0) std::memcpy(p, payload_src, datalen);
            *reinterpret_cast<uint16_t*>(p + datalen) = host_to_le16(0);
            PacketDebugger::printEtherCATFrame(tx_buf_,
                hdr.len + datalen + sizeof(uint16_t), true, false);
        }
        return channel_->txSendParts(parts);
    }

    constexpr size_t kMinEthFrameNoFcs = 60;

    const size_t required_len = hdr.len + datalen + sizeof(uint16_t);
    if (required_len > sizeof(tx_buf_)) {
        return false;
    }
    const size_t frame_len =
        (required_len < kMinEthFrameNoFcs) ? kMinEthFrameNoFcs : required_len;

    std::memcpy(tx_buf_, hdr.bytes, hdr.len);

    uint8_t* payload = tx_buf_ + hdr.len;
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

bool CyclicDatapath::sendDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    if (slot > kCyclicDcPoolPos) return false;
    return sendFastDatagram(cmd, cyclicFastIndex(slot), adp, ado,
                            data, datalen, roundtrip);
}

bool CyclicDatapath::sendSliceDatagram(Command cmd, uint8_t slice_slot,
                                   uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t datalen,
                                   bool roundtrip)
{
    if (slice_slot >= kNumSliceSlots) return false;
    return sendFastDatagram(cmd, sliceFastIndex(slice_slot), adp, ado,
                            data, datalen, roundtrip);
}

bool CyclicDatapath::sendPoolFrame(const CyclicDgramSpec* dgs,
                                       size_t count)
{
    using namespace Raw;
    if (!dgs || count == 0) return false;
    if (master_.cancel_requested_.load(std::memory_order_acquire))
        return false;

    // Compose [eth(+vlan) | ecat-hdr | dgram | wkc]×count into tx_buf_ —
    // dynamic assembly rather than the per-(idx,gen) templates: rotating
    // indices would blow the template bank up (82 positions × keys), and
    // the second/third datagrams (counter trailer host byte order, DC
    // read) are short enough that field-by-field writes cost ~nothing
    // next to the payload memcpy.
    uint32_t ec_len = 0;
    for (size_t i = 0; i < count; ++i) {
        const uint32_t dl = dgs[i].datalen + dgs[i].tail_len;
        if (dl > 0x07FFu) return false;
        ec_len += sizeof(EtherCATDatagramHeader) + dl + sizeof(uint16_t);
    }
    if (ec_len > 0x07FFu) return false;

    uint8_t* dst;
    if (channel_) {
        dst = channel_->txAcquire();
        if (!dst) return false;
    } else {
        dst = tx_buf_;
    }
    uint8_t* p = dst;

    constexpr uint8_t dst_mac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
    std::memcpy(p, dst_mac, 6);
    std::memcpy(p + 6, master_.src_mac_, 6);
    p += 12;
    if (tx_vlan_) {
        *reinterpret_cast<uint16_t*>(p) = host_to_be16(kEtherType8021Q);
        p += 2;
        *reinterpret_cast<uint16_t*>(p) = host_to_be16(tx_vlan_);
        p += 2;
    }
    *reinterpret_cast<uint16_t*>(p) = host_to_be16(kEtherTypeEtherCAT);
    p += 2;

    *reinterpret_cast<uint16_t*>(p) = host_to_le16(
        static_cast<uint16_t>((ec_len & 0x07FFu) | (0x1u << 12)));
    p += 2;

    for (size_t i = 0; i < count; ++i) {
        const CyclicDgramSpec& dg = dgs[i];
        const uint32_t dl = dg.datalen + dg.tail_len;
        const bool last = (i + 1 == count);

        // Generation bit for pool-index datagrams when requested —
        // collect rejects deposits echoing the previous generation.
        uint8_t gen = 0;
        if (dg.stamp_gen && isSlotIdx(dg.idx)) {
            const uint8_t fi = fastIndex(dg.idx);
            gen = (slot_gen_[fi] = slot_gen_[fi] ^ 1u);
        }

        p[0] = static_cast<uint8_t>(dg.cmd);
        p[1] = dg.idx;
        *reinterpret_cast<uint16_t*>(p + 2) = host_to_le16(dg.adp);
        *reinterpret_cast<uint16_t*>(p + 4) = host_to_le16(dg.ado);
        const uint16_t flags =
            (last ? 0u : (1u << 15)) |
            (dg.roundtrip ? (1u << 14) : 0u) |
            (static_cast<uint16_t>(gen) << 13);
        *reinterpret_cast<uint16_t*>(p + 6) = host_to_le16(
            static_cast<uint16_t>((dl & 0x07FFu) | flags));
        *reinterpret_cast<uint16_t*>(p + 8) = host_to_le16(0);   // irq
        p += 10;
        if (dg.datalen > 0) {
            if (dg.data) std::memcpy(p, dg.data, dg.datalen);
            else         std::memset(p, 0, dg.datalen);
            p += dg.datalen;
        }
        if (dg.tail_len > 0) {
            if (dg.tail) std::memcpy(p, dg.tail, dg.tail_len);
            else         std::memset(p, 0, dg.tail_len);
            p += dg.tail_len;
        }
        *reinterpret_cast<uint16_t*>(p) = host_to_le16(0);   // wkc
        p += 2;
    }

    const uint32_t frame_len = static_cast<uint32_t>(p - dst);
    if (master_.debug_flags_.txPackets) {
        PacketDebugger::printEtherCATFrame(dst, frame_len, true, false);
    }
    if (channel_) {
        return channel_->txCommitFrame(frame_len);
    }
    constexpr size_t kMinEthFrameNoFcs = 60;
    return master_.sendWithEncapsulation(
        tx_buf_, frame_len < kMinEthFrameNoFcs ? kMinEthFrameNoFcs
                                              : frame_len);
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
    if (slot > kCyclicDcPoolPos) return;
    const uint8_t fi = cyclicFastIndex(slot);
    const uint8_t gen =
        (slot_gen_[fi] = slot_gen_[fi] ^ 1u);
    const HdrTemplate& hdr = ensureTemplate(fi, gen, cmd, adp, ado,
                                            datalen, roundtrip);
    std::memcpy(frame, hdr.bytes, hdr.len);   // one contiguous ≤32 B store
}

/**
 * @brief Walk a frame received on the cyclic channel.
 *
 * Pure-fastpath frames (every datagram idx in [0xE0..0xFD] — PDO slices
 * and cyclic slots) publish their payload views into the slots without
 * copying.  Anything else (async or mixed frames — only possible when
 * the BPF demux is absent/failed) is routed through the regular parser,
 * which deposits/routes per datagram.
 */
void CyclicDatapath::dispatchFrame(const CyclicFrameView& v)
{
    using namespace Raw;
    dispatch_frames_.fetch_add(1, std::memory_order_relaxed);
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

    // Classify pass: any non-fastpath idx → whole frame to the parser.
    size_t off = base + sizeof(EtherCAT::FrameHeader);
    size_t rem = ec_len;
    bool   all_cyclic = true;
    while (rem >= kDgHdr + sizeof(uint16_t) && off + kDgHdr <= flen) {
        const uint8_t  idx       = f[off + 1];
        const uint16_t len_flags = le16_to_host(
            *reinterpret_cast<const uint16_t*>(f + off + 6));
        const uint16_t dl   = len_flags & 0x07FFu;
        const bool     more = (len_flags & 0x8000u) != 0;
        if (!isFastPathIdx(idx)) {
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

        // Hold only slots we can publish — an idx in the 0xF0..0xF7 gap
        // (fastpath but no slot) would leak the block pin forever.
        if (isSlotIdx(idx)) {
            channel_->rxHold(v.cookie);
            publishView(idx, static_cast<Command>(cmd), adp, ado,
                                  f + data_off, dl, wkc, v.cookie, v.stamp_ns,
                                  static_cast<uint8_t>((len_flags >> 13) & 0x1u));
        } else {
            dispatch_unrouted_.fetch_add(1, std::memory_order_relaxed);
        }
        rem -= dgt; off += dgt;
        if (!more) break;
    }
}

void CyclicDatapath::drainChannel(int max_sweeps)
{
    if (!channel_) return;
    CyclicFrameView views[8];
    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        const int n = channel_->rxPoll(views, 8, 0);
        if (n <= 0) return;
        for (int i = 0; i < n; ++i) dispatchFrame(views[i]);
    }
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
    channel_.reset();
    tx_vlan_            = 0;
    tx_prefix_len_      = kCyclicFramePayloadOff;
    for (auto& gen_pair : hdr_tmpl_)
        for (auto& t : gen_pair) t.valid = false;
    active_image_mode_ = ImageMode::Buffered;
    image_.configure({});

#ifdef __linux__
    // The channel needs a raw AF_PACKET fd.  Direct EtherCAT exposes it as
    // iface_.native_handle; under VLAN encapsulation iface_ is the router
    // stub (no handle) and the app hands the wire socket in via
    // Master::Config::wire_fd instead.
    int fd = -1;
    if (master_.config_.wire_fd >= 0) {
        fd = master_.config_.wire_fd;
    } else if (master_.iface_.receive && master_.iface_.native_handle) {
        fd = static_cast<int>(
            reinterpret_cast<intptr_t>(master_.iface_.native_handle));
    }
    if (fd >= 0 && !master_.isUdpEncapsulationEnabled()) {
        sockaddr_ll sll{};
        socklen_t sll_len = sizeof(sll);
        int ifindex = 0;
        if (::getsockname(fd, reinterpret_cast<sockaddr*>(&sll),
                          &sll_len) == 0) {
            ifindex = sll.sll_ifindex;
        }

        // Under VLAN encapsulation the socket-A filter must keep the
        // VID clause AND add the fastpath-idx demux — SO_ATTACH_FILTER
        // replaces rather than stacks, so one composed program per
        // socket is generated:  cyclic = encap ∧ idx∈[0x9C,0xFF]\{0xFE},
        // async = encap ∧ its complement.  Without this the mirror
        // attach would strip the VID filter (accepting all traffic) and
        // tagged cyclic frames would double-deliver via the router.
        const auto& we = master_.config_.wire_encap;
        std::vector<CBPFInsn> accept_prog, async_prog;
        if (we.tagged()) {
            CBPFSpec spec;
            spec.untagged_ethercat = false;
            spec.tagged_ethercat   = true;
            spec.vlan_delivery     = we.delivery_hint;
            if (!we.rx_vlan_any) {
                // A range clause only when a VID set was configured —
                // rx_any must accept every TCI like the main filter.
                spec.vlan_range = CBPFVlanRange{
                    we.rx_vlan_lo,
                    we.rx_vlan_hi ? we.rx_vlan_hi : we.rx_vlan_lo};
            }
            spec.first_idx_range = CBPFIdxRange{
                kFastSlotBaseIdx, kFastSlotEndIdx,
                kFastSlotReservedIdx};   // 0xFE stays async
            accept_prog = CBPFProgramFactory::build(spec);
            spec.first_idx_exclude = true;
            async_prog = CBPFProgramFactory::build(spec);
        }

        CyclicChannelConfig cc;
        cc.ifindex    = ifindex;
        cc.async_fd   = fd;
        cc.wire_mode  = wire_mode;
        cc.rx_spin_ns = rx_spin_ns;
        cc.frame_size = master_.config_.max_frame_size;
        cc.accept_prog     = accept_prog.empty() ? nullptr
                                                 : accept_prog.data();
        cc.accept_prog_len = accept_prog.size();
        cc.async_prog      = async_prog.empty() ? nullptr
                                                : async_prog.data();
        cc.async_prog_len  = async_prog.size();
        channel_ = createCyclicChannel(cc);
        if (!channel_) {
            TETHER_LOGW(TAG, "cyclic channel unavailable — using software "
                             "deposit path (no ring/BPF acceleration)");
        } else if (we.tx_vlan) {
            // Bake the TX tag into every header template — tagged frames
            // go straight to the channel socket, bypassing the router's
            // mutex + full-frame copy on every cyclic send.
            tx_vlan_       = we.tx_vlan;
            tx_prefix_len_ = kCyclicFramePayloadOff + 4;
            TETHER_LOGI(TAG, "cyclic channel: TX VLAN {} baked into "
                             "header templates", we.tx_vlan);
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

} // namespace EtherCAT
