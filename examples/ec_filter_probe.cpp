/**
 * @file ec_filter_probe.cpp
 * @brief RX filter probe — which socket config receives which replies?
 *
 * Sends deliberately unreachable datagrams (bogus logical-address LRWs,
 * FPRD/APRD to slave address/position 999) and verifies which of several
 * candidate socket configurations the returning frames land on:
 *
 *   bare    - AF_PACKET, no filter (control)
 *   encap   - composed spec, encapsulation clause only (no idx demux)
 *   cyclic  - composed spec: encap ∧ first-idx ∈ [0xE0,0xFD]  (production)
 *   async   - composed spec: encap ∧ first-idx ∉ [0xE0,0xFD]  (production)
 *
 * All RX sockets set PACKET_IGNORE_OUTGOING like the production sockets,
 * so only returning (inbound) frames are counted.  Bogus targets make the
 * slaves forward untouched frames back with WKC=0 — this tests the socket
 * path, not slave behaviour.
 *
 * Usage (requires root / CAP_NET_RAW):
 *   sudo ./ec_filter_probe -i enp1s0f0              # untagged
 *   sudo ./ec_filter_probe -i enp1s0f0 -v 1999      # VLAN 1999
 */

#include "tether/ethercat/CBPFProgramFactory.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef __linux__
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr uint8_t  kDstMac[6]    = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
constexpr uint8_t  kCmdAPRD      = 0x01;
constexpr uint8_t  kCmdFPRD      = 0x04;
constexpr uint8_t  kCmdLRW       = 0x0C;
constexpr uint8_t  kFastIdxLo    = 0xE0;
constexpr uint8_t  kFastIdxHi    = 0xFD;

struct ProbeSocket {
    const char* name;
    int         fd = -1;
};

int openRx(int ifindex, const EtherCAT::CBPFInsn* prog = nullptr,
           size_t prog_len = 0) {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) { perror("socket"); return -1; }
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        perror("bind"); ::close(fd); return -1;
    }
    if (prog && prog_len &&
        !EtherCAT::CBPFProgramFactory::attach(fd, prog, prog_len)) {
        fprintf(stderr, "filter attach failed: %s\n", strerror(errno));
        ::close(fd); return -1;
    }
    return fd;
}

size_t buildFrame(uint8_t* out, const uint8_t src[6], uint16_t vlan,
                  uint8_t cmd, uint8_t idx, uint32_t addr,
                  const uint8_t* data, uint16_t dlen) {
    uint8_t* p = out;
    std::memcpy(p, kDstMac, 6); p += 6;
    std::memcpy(p, src, 6);     p += 6;
    if (vlan) {
        uint16_t tpid = htons(0x8100);
        uint16_t tci  = htons(vlan & 0x0FFF);
        std::memcpy(p, &tpid, 2); p += 2;
        std::memcpy(p, &tci, 2);  p += 2;
    }
    uint16_t etype = htons(0x88A4);
    std::memcpy(p, &etype, 2); p += 2;

    // EtherCAT frame header: 11-bit datagram-section length, type=1.
    uint16_t ecat_len = static_cast<uint16_t>(10 + dlen + 2);
    uint16_t ecat_hdr = htons(ecat_len | (1u << 12));
    std::memcpy(p, &ecat_hdr, 2); p += 2;

    *p++ = cmd;
    *p++ = idx;
    std::memcpy(p, &addr, 4); p += 4;          // little-endian ADP/ADR or logical
    uint16_t lenflags = htons(dlen);           // 11-bit length, flags 0
    std::memcpy(p, &lenflags, 2); p += 2;
    if (dlen) { std::memcpy(p, data, dlen); p += dlen; }
    uint16_t wkc = 0;
    std::memcpy(p, &wkc, 2); p += 2;

    size_t total = static_cast<size_t>(p - out);
    if (total < 60) {                          // pad to Ethernet minimum
        std::memset(p, 0, 60 - total);
        total = 60;
    }
    return total;
}

struct RxHit { int count = 0; uint16_t last_wkc = 0; size_t last_len = 0; };

// --- kernel packet-ring probe --------------------------------------------
//
// Reproduces the production cyclic channel's PACKET_MMAP usage in isolation
// and reports whether the kernel actually drains a TX ring / fills an RX
// ring under each configuration.  The failure we are hunting: every TX slot
// stuck in TP_STATUS_SEND_REQUEST (kernel never transmits).

int openBoundRaw(int ifindex) {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) return -1;
    struct sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        ::close(fd); return -1;
    }
    return fd;
}

/**
 * Put one frame into a PACKET_TX_RING slot, kick with sendto(), then check
 * whether tp_status returns to TP_STATUS_AVAILABLE (kernel drained it).
 * @param rx_v3  also configure a TPACKET_V3 RX ring on the same socket
 *               (the production cyclic-channel layout).
 */
const char* txRingTest(int ifindex, int packet_version, bool rx_v3,
                       const uint8_t* frame, size_t flen) {
    int fd = openBoundRaw(ifindex);
    if (fd < 0) return "socket/bind failed";

    size_t rx_bytes = 0;
    if (packet_version == TPACKET_V3) {
        if (::setsockopt(fd, SOL_PACKET, PACKET_VERSION,
                         &packet_version, sizeof(packet_version)) < 0)
            { ::close(fd); return "PACKET_VERSION=V3 failed"; }
    }
    if (rx_v3) {
        struct tpacket_req3 rreq{};
        rreq.tp_block_size = 1 << 16;
        rreq.tp_block_nr   = 8;
        rreq.tp_frame_size = 2048;
        rreq.tp_frame_nr   = (rreq.tp_block_size / rreq.tp_frame_size) *
                             rreq.tp_block_nr;
        rreq.tp_retire_blk_tov = 10;
        if (::setsockopt(fd, SOL_PACKET, PACKET_RX_RING,
                         &rreq, sizeof(rreq)) < 0)
            { ::close(fd); return "PACKET_RX_RING(V3) failed"; }
        rx_bytes = static_cast<size_t>(rreq.tp_block_size) * rreq.tp_block_nr;
    }

    const long pg = ::sysconf(_SC_PAGESIZE);
    struct tpacket_req treq{};
    treq.tp_block_size = static_cast<unsigned>(pg);
    treq.tp_block_nr   = 32;
    treq.tp_frame_size = 2048;
    treq.tp_frame_nr   = (treq.tp_block_size / treq.tp_frame_size) *
                         treq.tp_block_nr;
    if (::setsockopt(fd, SOL_PACKET, PACKET_TX_RING,
                     &treq, sizeof(treq)) < 0)
        { ::close(fd); return "PACKET_TX_RING failed"; }

    const size_t tx_bytes = static_cast<size_t>(treq.tp_block_size) *
                            treq.tp_block_nr;
    const size_t map_len  = rx_bytes + tx_bytes;
    uint8_t* map = static_cast<uint8_t*>(
        ::mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == MAP_FAILED) { ::close(fd); return "mmap failed"; }

    auto* hdr = reinterpret_cast<struct tpacket_hdr*>(map + rx_bytes);
    if (flen > static_cast<size_t>(treq.tp_frame_size - TPACKET_HDRLEN))
        { ::munmap(map, map_len); ::close(fd); return "frame > tp_frame_size"; }
    std::memcpy(reinterpret_cast<uint8_t*>(hdr) + TPACKET_HDRLEN, frame, flen);
    hdr->tp_len   = static_cast<uint32_t>(flen);
    hdr->tp_status = TP_STATUS_SEND_REQUEST;

    struct sockaddr_ll dst{};
    dst.sll_family  = AF_PACKET;
    dst.sll_ifindex = ifindex;
    if (::sendto(fd, nullptr, 0, 0, reinterpret_cast<sockaddr*>(&dst),
                 sizeof(dst)) < 0)
        { ::munmap(map, map_len); ::close(fd); return "kick sendto failed"; }

    struct pollfd pfd{fd, POLLOUT, 0};
    ::poll(&pfd, 1, 50);

    const uint32_t st = hdr->tp_status;
    ::munmap(map, map_len);
    ::close(fd);
    if (st == TP_STATUS_AVAILABLE)        return "DRAINED (tx on wire)";
    if (st & TP_STATUS_SEND_REQUEST)      return "STUCK in SEND_REQUEST";
    if (st & TP_STATUS_WRONG_FORMAT)      return "WRONG_FORMAT";
    return "UNKNOWN status";
}

/**
 * RX ring test: composed cyclic filter + TPACKET_V3 RX ring — the exact
 * production receive path.  Sends one probe datagram on a helper TX socket
 * and reports whether the reply appears in the ring.
 */
const char* rxRingTest(int ifindex, const EtherCAT::CBPFInsn* prog,
                       size_t prog_len, int tx_fd,
                       const uint8_t* frame, size_t flen, uint8_t want_idx,
                       size_t idx_off) {
    int fd = openBoundRaw(ifindex);
    if (fd < 0) return "socket/bind failed";
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    int v3 = TPACKET_V3;
    if (::setsockopt(fd, SOL_PACKET, PACKET_VERSION, &v3, sizeof(v3)) < 0)
        { ::close(fd); return "PACKET_VERSION=V3 failed"; }
    if (prog && prog_len &&
        !EtherCAT::CBPFProgramFactory::attach(fd, prog, prog_len))
        { ::close(fd); return "composed BPF attach failed"; }

    struct tpacket_req3 rreq{};
    rreq.tp_block_size = 1 << 16;
    rreq.tp_block_nr   = 8;
    rreq.tp_frame_size = 2048;
    rreq.tp_frame_nr   = (rreq.tp_block_size / rreq.tp_frame_size) *
                         rreq.tp_block_nr;
    rreq.tp_retire_blk_tov = 10;
    if (::setsockopt(fd, SOL_PACKET, PACKET_RX_RING, &rreq, sizeof(rreq)) < 0)
        { ::close(fd); return "PACKET_RX_RING(V3) failed"; }

    const size_t map_len = static_cast<size_t>(rreq.tp_block_size) *
                           rreq.tp_block_nr;
    uint8_t* map = static_cast<uint8_t*>(
        ::mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == MAP_FAILED) { ::close(fd); return "mmap failed"; }

    struct sockaddr_ll dst{};
    dst.sll_family  = AF_PACKET;
    dst.sll_ifindex = ifindex;
    if (::sendto(tx_fd, frame, flen, 0, reinterpret_cast<sockaddr*>(&dst),
                 sizeof(dst)) < 0)
        { ::munmap(map, map_len); ::close(fd); return "probe sendto failed"; }

    struct pollfd pfd{fd, POLLIN, 0};
    ::poll(&pfd, 1, 30);

    const char* result = "ring empty (filter rejected / no reply)";
    for (unsigned b = 0; b < rreq.tp_block_nr; ++b) {
        auto* bd = reinterpret_cast<struct tpacket_block_desc*>(
            map + static_cast<size_t>(b) * rreq.tp_block_size);
        if (!(bd->hdr.bh1.block_status & TP_STATUS_USER)) continue;
        auto* ph = reinterpret_cast<struct tpacket3_hdr*>(
            reinterpret_cast<uint8_t*>(bd) +
            bd->hdr.bh1.offset_to_first_pkt);
        const uint8_t* pkt = reinterpret_cast<uint8_t*>(ph) + ph->tp_mac;
        // Ring frames have the tag stripped → idx at 17; inline tag → 21.
        const size_t off =
            (ph->tp_snaplen > 14 && pkt[12] == 0x81 && pkt[13] == 0x00)
                ? 21 : 17;
        if (ph->tp_snaplen > off && pkt[off] == want_idx)
            result = "reply delivered to ring";
        else
            result = "ring has frames but wrong idx";
        bd->hdr.bh1.block_status = TP_STATUS_KERNEL;
    }
    ::munmap(map, map_len);
    ::close(fd);
    return result;
}

/**
 * VLAN auxdata probe: does the kernel strip the 802.1Q tag into
 * tpacket_auxdata (recvmsg cmsg), or leave it inline in the frame bytes?
 * This decides which leg of the composed cBPF (offset-21 inline vs
 * offset-17 stripped + SKF_AD_VLAN_*) the replies actually exercise.
 */
const char* auxdataTest(int ifindex, int tx_fd, const uint8_t* frame,
                        size_t flen, char* detail, size_t dlen) {
    int fd = openBoundRaw(ifindex);
    if (fd < 0) return "socket/bind failed";
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    if (::setsockopt(fd, SOL_PACKET, PACKET_AUXDATA, &one, sizeof(one)) < 0) {
        ::close(fd); return "PACKET_AUXDATA unsupported";
    }

    struct sockaddr_ll dst{};
    dst.sll_family  = AF_PACKET;
    dst.sll_ifindex = ifindex;
    if (::sendto(tx_fd, frame, flen, 0, reinterpret_cast<sockaddr*>(&dst),
                 sizeof(dst)) < 0) { ::close(fd); return "probe sendto failed"; }

    struct pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, 30) <= 0) { ::close(fd); return "no reply received"; }

    uint8_t buf[2048];
    uint8_t cbuf[256];
    struct iovec iov{buf, sizeof(buf)};
    struct msghdr msg{};
    msg.msg_iov = &iov; msg.msg_iovlen = 1;
    msg.msg_control = cbuf; msg.msg_controllen = sizeof(cbuf);
    ssize_t n = ::recvmsg(fd, &msg, 0);
    ::close(fd);
    if (n <= 0) return "recvmsg failed";

    const bool inline_tag = (n > 14 && buf[12] == 0x81 && buf[13] == 0x00);
    int aux_vid = -1, aux_status = -1;
    for (struct cmsghdr* c = CMSG_FIRSTHDR(&msg); c;
         c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_PACKET && c->cmsg_type == PACKET_AUXDATA) {
            const auto* aux =
                reinterpret_cast<const struct tpacket_auxdata*>(CMSG_DATA(c));
            aux_status = aux->tp_status;
            if (aux->tp_status & TP_STATUS_VLAN_VALID)
                aux_vid = aux->tp_vlan_tci & 0x0FFF;
        }
    }
    snprintf(detail, dlen,
             "%s tag in data; auxdata status=0x%x vlan_vid=%d "
             "(vlan_valid=%d) — %s",
             inline_tag ? "inline 0x8100" : "stripped (0x88A4 at [12])",
             aux_status, aux_vid,
             (aux_status >= 0 && (aux_status & TP_STATUS_VLAN_VALID)) ? 1 : 0,
             inline_tag ? "BPF must parse inline tag (idx@21)"
                        : "BPF auxdata leg applies (idx@17)");
    return "ok";
}

} // namespace

int main(int argc, char** argv) {
    std::string ifname;
    uint16_t vlan = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-i" || a == "--interface") && i + 1 < argc) ifname = argv[++i];
        else if ((a == "-v" || a == "--vlan") && i + 1 < argc)
            vlan = static_cast<uint16_t>(atoi(argv[++i]));
        else { fprintf(stderr, "usage: %s -i <iface> [-v <vlan>]\n", argv[0]); return 2; }
    }
    if (ifname.empty()) { fprintf(stderr, "usage: %s -i <iface> [-v <vlan>]\n", argv[0]); return 2; }

    const int ifindex = if_nametoindex(ifname.c_str());
    if (!ifindex) { fprintf(stderr, "unknown interface %s\n", ifname.c_str()); return 1; }

    // Source MAC of the interface.
    uint8_t src[6] = {};
    {
        int s = ::socket(AF_INET, SOCK_DGRAM, 0);
        struct ifreq ifr{}; std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (::ioctl(s, SIOCGIFHWADDR, &ifr) == 0)
            std::memcpy(src, ifr.ifr_hwaddr.sa_data, 6);
        ::close(s);
    }

    // --- RX sockets -----------------------------------------------------
    EtherCAT::CBPFSpec spec;
    spec.untagged_ethercat = (vlan == 0);
    spec.tagged_ethercat   = (vlan != 0);
    if (vlan)
        spec.vlan_range = EtherCAT::CBPFVlanRange{vlan, vlan};

    std::vector<EtherCAT::CBPFInsn> encap_prog, cyc_prog, async_prog;
    encap_prog = EtherCAT::CBPFProgramFactory::build(spec);
    spec.first_idx_range   = EtherCAT::CBPFIdxRange{kFastIdxLo, kFastIdxHi};
    spec.first_idx_exclude = false;
    cyc_prog = EtherCAT::CBPFProgramFactory::build(spec);
    spec.first_idx_exclude = true;
    async_prog = EtherCAT::CBPFProgramFactory::build(spec);

    ProbeSocket socks[] = {
        {"bare",   openRx(ifindex)},
        {"encap",  openRx(ifindex, encap_prog.data(), encap_prog.size())},
        {"cyclic", openRx(ifindex, cyc_prog.data(),   cyc_prog.size())},
        {"async",  openRx(ifindex, async_prog.data(), async_prog.size())},
    };
    for (auto& s : socks) if (s.fd < 0) return 1;

    int tx = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (tx < 0) { perror("tx socket"); return 1; }
    struct sockaddr_ll dst{};
    dst.sll_family   = AF_PACKET;
    dst.sll_protocol = htons(ETH_P_ALL);
    dst.sll_ifindex  = ifindex;

    // --- probe frames ----------------------------------------------------
    // First-idx values chosen to land on both sides of the fastpath range.
    struct Probe { const char* name; uint8_t cmd; uint8_t idx; uint32_t addr; uint16_t dlen; };
    const Probe probes[] = {
        {"LRW  bogus-logical idx=0xF8", kCmdLRW,  0xF8, 0xDEADBEEFu,        32},
        {"LRW  bogus-logical idx=0xE0", kCmdLRW,  0xE0, 0x00010000u,        16},
        {"APRD position -999 idx=0xFE", kCmdAPRD, 0xFE, (0xFC19u) | (0x0130u << 16), 2},
        {"FPRD station  999  idx=0x55", kCmdFPRD, 0x55, 999u | (0x0130u << 16),      2},
        {"LRW  bogus-logical idx=0x10", kCmdLRW,  0x10, 0xABCDEF00u,        16},
    };

    uint8_t frame[1600];
    RxHit hits[sizeof(socks) / sizeof(socks[0])][sizeof(probes) / sizeof(probes[0])] = {};

    for (size_t pi = 0; pi < sizeof(probes) / sizeof(probes[0]); ++pi) {
        const Probe& pr = probes[pi];
        uint8_t payload[64] = {};
        const size_t flen = buildFrame(frame, src, vlan, pr.cmd, pr.idx,
                                       pr.addr, payload, pr.dlen);
        if (::sendto(tx, frame, flen, 0, reinterpret_cast<sockaddr*>(&dst),
                     sizeof(dst)) < 0) {
            perror("sendto"); continue;
        }
        // ~20 ms for the frame to traverse the segment and return.
        struct pollfd pfds[4];
        for (size_t si = 0; si < 4; ++si) { pfds[si].fd = socks[si].fd; pfds[si].events = POLLIN; }
        ::poll(pfds, 4, 20);
        uint8_t buf[2048];
        for (size_t si = 0; si < 4; ++si) {
            ssize_t n;
            while ((n = ::recv(socks[si].fd, buf, sizeof(buf), 0)) > 0) {
                // Replies arrive with the VLAN tag stripped into auxdata
                // (rx-vlan-offload) → idx at 17.  Inline-tag copies (self-TX
                // echo) → idx at 21.  Auto-detect on the wire ethertype.
                const size_t ioff =
                    (n > 14 && buf[12] == 0x81 && buf[13] == 0x00) ? 21 : 17;
                if (static_cast<size_t>(n) > ioff && buf[ioff] == pr.idx) {
                    auto& h = hits[si][pi];
                    h.count++;
                    h.last_len = n;
                    if (static_cast<size_t>(n) > ioff + 10 + pr.dlen + 1)
                        h.last_wkc = buf[ioff + 10 + pr.dlen] |
                                     (buf[ioff + 10 + pr.dlen + 1] << 8);
                }
            }
        }
    }

    printf("\n=== reply matrix (vlan=%u) ===\n", vlan);
    printf("%-34s | %-16s | %-16s | %-16s | %-16s\n",
           "probe", socks[0].name, socks[1].name, socks[2].name, socks[3].name);
    for (size_t pi = 0; pi < sizeof(probes) / sizeof(probes[0]); ++pi) {
        printf("%-34s |", probes[pi].name);
        for (size_t si = 0; si < 4; ++si) {
            const auto& h = hits[si][pi];
            if (h.count)
                printf(" %d reply wkc=%-4u |", h.count, h.last_wkc);
            else
                printf(" %-16s |", "-");
        }
        printf("\n");
    }
    printf("\nExpected: every probe lands on 'bare' and 'encap' (WKC=0 — "
           "bogus targets), fastpath idx (E0-FD) only on 'cyclic', "
           "others only on 'async'.\n");

    // --- kernel packet-ring matrix --------------------------------------
    // Reproduce the production cyclic channel's PACKET_MMAP usage: does the
    // kernel drain a TX ring (slots return from SEND_REQUEST to AVAILABLE)
    // and does a composed filter + TPACKET_V3 RX ring deliver replies?
    {
        uint8_t payload[8] = {};
        const size_t flen = buildFrame(frame, src, vlan, kCmdLRW, 0xF8,
                                       0xDEADBEEFu, payload, sizeof(payload));
        const size_t idx_off = vlan ? 21 : 17;

        printf("\n=== kernel packet-ring matrix ===\n");
        char aux_detail[256] = {};
        const char* aux_res = auxdataTest(ifindex, tx, frame, flen,
                                          aux_detail, sizeof(aux_detail));
        printf("VLAN tag delivery (PACKET_AUXDATA):                 %s%s%s\n",
               aux_res, aux_detail[0] ? " — " : "", aux_detail);
        printf("TX ring, no PACKET_VERSION/RX ring (plain v2 socket): %s\n",
               txRingTest(ifindex, 0, false, frame, flen));
        printf("TX ring on TPACKET_V3 socket, no RX ring:           %s\n",
               txRingTest(ifindex, TPACKET_V3, false, frame, flen));
        printf("TX ring + TPACKET_V3 RX ring (production layout):   %s\n",
               txRingTest(ifindex, TPACKET_V3, true, frame, flen));

        printf("V3 RX ring + composed cyclic filter, idx 0xF8:      %s\n",
               rxRingTest(ifindex, cyc_prog.data(), cyc_prog.size(),
                          tx, frame, flen, 0xF8, idx_off));
        printf("V3 RX ring + composed async filter,  idx 0xF8:      %s\n",
               rxRingTest(ifindex, async_prog.data(), async_prog.size(),
                          tx, frame, flen, 0xF8, idx_off));
    }
    return 0;
}

#else
int main() {
    fprintf(stderr, "ec_filter_probe is Linux-only.\n");
    return 1;
}
#endif
