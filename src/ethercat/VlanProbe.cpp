/**
 * @file VlanProbe.cpp
 * @brief Startup VLAN-delivery probe — implementation.
 *
 * Wire format of the probe (same construction as ec_filter_probe):
 *
 *   [dst 6][src 6][0x8100 TCI]?[0x88A4][ecat-hdr 2][LRW][idx][addr 4]
 *   [len 2][data][wkc 2][pad to 60]
 *
 * The logical address is deliberately unmapped, so every slave forwards
 * the frame unchanged and it returns with WKC=0 — a missing reply means
 * the segment is open or the VID never reached the bus.
 */

#include "tether/ethercat/VlanProbe.hpp"

#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "hal/IEthernet.hpp"
#endif

namespace EtherCAT {

const char* toString(VlanTagDelivery d) {
    switch (d) {
    case VlanTagDelivery::NoReply:     return "no-reply";
    case VlanTagDelivery::Untagged:    return "untagged";
    case VlanTagDelivery::InlineTag:   return "inline-tag";
    case VlanTagDelivery::StrippedTag: return "stripped-tag";
    }
    return "?";
}

VlanTagDelivery classifyVlanReply(const uint8_t* frame, size_t len,
                                  bool aux_vlan_valid, uint16_t aux_tci,
                                  uint16_t aux_tpid,
                                  uint16_t& out_vid, uint16_t& out_tpid) {
    out_vid  = 0;
    out_tpid = 0;
    if (!frame || len < 14) return VlanTagDelivery::NoReply;

    const uint16_t etype =
        static_cast<uint16_t>((frame[12] << 8) | frame[13]);
    // Inline tag: a tag TPID sits at [12].  Accept the common TPIDs —
    // 0x8100 (802.1Q), 0x88A8/0x9200 (802.1ad provider) — the TPID itself
    // is reported to the caller.
    if (len >= 18 &&
        (etype == 0x8100 || etype == 0x88A8 || etype == 0x9100 ||
         etype == 0x9200)) {
        out_tpid = etype;
        out_vid  = static_cast<uint16_t>(
            ((frame[14] & 0x0F) << 8) | frame[15]);
        return VlanTagDelivery::InlineTag;
    }
    if (aux_vlan_valid) {
        out_vid  = static_cast<uint16_t>(aux_tci & 0x0FFF);
        out_tpid = aux_tpid;
        return VlanTagDelivery::StrippedTag;
    }
    return VlanTagDelivery::Untagged;
}

#if defined(__linux__)

namespace {

constexpr uint8_t kProbeDstMac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
constexpr uint8_t kCmdLRW         = 0x0C;
constexpr uint16_t kEtherTypeEcat = 0x88A4;

/// Build the tagged-or-untagged bogus-LRW probe frame.  Returns length.
size_t buildProbeFrame(uint8_t* out, const uint8_t src[6], uint16_t vid) {
    uint8_t* p = out;
    std::memcpy(p, kProbeDstMac, 6); p += 6;
    std::memcpy(p, src, 6);          p += 6;
    if (vid) {
        const uint16_t tpid = htons(0x8100);
        const uint16_t tci  = htons(vid & 0x0FFF);
        std::memcpy(p, &tpid, 2); p += 2;
        std::memcpy(p, &tci, 2);  p += 2;
    }
    const uint16_t etype = htons(kEtherTypeEcat);
    std::memcpy(p, &etype, 2); p += 2;

    constexpr uint16_t dlen = 8;
    // EtherCAT frame header: 11-bit datagram-section length, type=1.
    const uint16_t ecat_hdr =
        htons(static_cast<uint16_t>(10 + dlen + 2) | (1u << 12));
    std::memcpy(p, &ecat_hdr, 2); p += 2;

    *p++ = kCmdLRW;
    *p++ = kVlanProbeIdx;
    const uint32_t addr = kVlanProbeLogicalAddr;   // little-endian on wire
    std::memcpy(p, &addr, 4); p += 4;
    const uint16_t lenflags = htons(dlen);
    std::memcpy(p, &lenflags, 2); p += 2;
    std::memset(p, 0, dlen); p += dlen;            // LRW payload (ignored)
    const uint16_t wkc = 0;
    std::memcpy(p, &wkc, 2); p += 2;

    size_t total = static_cast<size_t>(p - out);
    if (total < 60) { std::memset(p, 0, 60 - total); total = 60; }
    return total;
}

int64_t nowUs() {
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1'000'000LL + ts.tv_nsec / 1000;
}

/// True when @p frame is our probe's reply: EtherCAT ethertype (after any
/// inline tag), LRW cmd byte and our idx at the delivery-appropriate offset.
bool isProbeReply(const uint8_t* frame, size_t len, VlanTagDelivery del) {
    if (len < 40) return false;
    size_t base;
    if (del == VlanTagDelivery::InlineTag) {
        if (len < 44 ||
            ((frame[16] << 8) | frame[17]) != kEtherTypeEcat) return false;
        base = 18;
    } else {
        if (((frame[12] << 8) | frame[13]) != kEtherTypeEcat) return false;
        base = 14;
    }
    // [base]=ecat-hdr(2) [base+2]=cmd [base+3]=idx
    return frame[base + 2] == kCmdLRW && frame[base + 3] == kVlanProbeIdx;
}

} // namespace

std::optional<VlanProbeResult> probeVlanTagDelivery(
    int fd, int ifindex, const uint8_t src_mac[6], uint16_t vid,
    uint32_t timeout_ms) {
    if (fd < 0 || ifindex <= 0) return std::nullopt;

    // Ensure auxdata so a stripped tag is visible.  Soft-fail: if the
    // kernel lacks PACKET_AUXDATA the probe still classifies inline/untagged.
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_AUXDATA, &one, sizeof(one));

    uint8_t frame[1600];
    const size_t flen = buildProbeFrame(frame, src_mac, vid);

    struct sockaddr_ll dst{};
    dst.sll_family   = AF_PACKET;
    dst.sll_protocol = htons(ETH_P_ALL);
    dst.sll_ifindex  = ifindex;

    const int64_t t0    = nowUs();
    const int64_t t_end = t0 + static_cast<int64_t>(timeout_ms) * 1000;
    if (::sendto(fd, frame, flen, 0,
                 reinterpret_cast<const sockaddr*>(&dst), sizeof(dst)) < 0)
        return std::nullopt;

    while (true) {
        const int64_t remain_ms = (t_end - nowUs() + 999) / 1000;
        if (remain_ms <= 0) break;
        struct pollfd pfd{fd, POLLIN, 0};
        if (::poll(&pfd, 1, static_cast<int>(remain_ms)) <= 0) break;
        if (pfd.revents & (POLLERR | POLLNVAL)) break;

        uint8_t buf[2048];
        alignas(cmsghdr) char cbuf[CMSG_SPACE(sizeof(tpacket_auxdata))];
        struct sockaddr_ll sll{};
        struct iovec iov{buf, sizeof(buf)};
        struct msghdr msg{};
        msg.msg_name       = &sll;
        msg.msg_namelen    = sizeof(sll);
        msg.msg_iov        = &iov;
        msg.msg_iovlen     = 1;
        msg.msg_control    = cbuf;
        msg.msg_controllen = sizeof(cbuf);
        const ssize_t n = ::recvmsg(fd, &msg, MSG_DONTWAIT);
        if (n <= 0) continue;
        if (sll.sll_pkttype == PACKET_OUTGOING) continue;   // our own copy

        bool     aux_valid = false;
        uint16_t aux_tci = 0, aux_tpid = 0;
        for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level != SOL_PACKET ||
                c->cmsg_type != PACKET_AUXDATA) continue;
            const auto* aux =
                reinterpret_cast<const tpacket_auxdata*>(CMSG_DATA(c));
            aux_valid = (aux->tp_status & TP_STATUS_VLAN_VALID) != 0;
            aux_tci   = aux->tp_vlan_tci;
#ifdef TP_STATUS_VLAN_TPID_VALID
            if (aux->tp_status & TP_STATUS_VLAN_TPID_VALID)
                aux_tpid = aux->tp_vlan_tpid;
#endif
        }

        VlanProbeResult res;
        res.delivery = classifyVlanReply(buf, static_cast<size_t>(n),
                                       aux_valid, aux_tci, aux_tpid,
                                       res.vid, res.tpid);
        if (res.delivery == VlanTagDelivery::NoReply) continue;
        if (!isProbeReply(buf, static_cast<size_t>(n), res.delivery))
            continue;                       // some other EtherCAT traffic

        res.rtt_us = nowUs() - t0;
        return res;
    }

    VlanProbeResult res;                    // ran fine, nothing came back
    return res;
}

std::optional<VlanProbeResult> probeVlanTagDelivery(
    HAL::IEthernet& eth, uint16_t vid, uint32_t timeout_ms) {
    const int fd =
        static_cast<int>(reinterpret_cast<intptr_t>(eth.nativeHandle()));
    const int ifindex =
        static_cast<int>(::if_nametoindex(eth.getInterfaceName()));
    if (fd < 0 || !ifindex) return std::nullopt;

    uint8_t mac[6] = {};
    const int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s >= 0) {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, eth.getInterfaceName(), IFNAMSIZ - 1);
        if (::ioctl(s, SIOCGIFHWADDR, &ifr) == 0)
            std::memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
        ::close(s);
    }
    return probeVlanTagDelivery(fd, ifindex, mac, vid, timeout_ms);
}

#else  // !__linux__

std::optional<VlanProbeResult> probeVlanTagDelivery(
    int, int, const uint8_t*, uint16_t, uint32_t) {
    return std::nullopt;
}

std::optional<VlanProbeResult> probeVlanTagDelivery(
    HAL::IEthernet&, uint16_t, uint32_t) {
    return std::nullopt;
}

#endif
} // namespace EtherCAT
