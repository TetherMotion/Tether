/**
 * @file ecat_proxy_external_source.cpp
 * @brief External PDO source client for Tether's EtherCAT proxy
 *        forwarding — injects proxy-band PDO datagrams and prints the
 *        echoed responses.
 *
 * This program plays the role of the "external master": it talks to a
 * running Tether master that has proxy forwarding enabled, NOT to the
 * slaves directly.  Tether validates every injected frame (PDO commands
 * only, datagram index inside the configured band, logical-address ACL
 * with optional value constraints) before it reaches the bus.
 *
 * ----------------------------------------------------------------------
 * Tether-side configuration (the bus side)
 * ----------------------------------------------------------------------
 *
 *   EtherCAT::MasterConfig cfg;
 *   cfg.iface = "enp3s0";                 // EtherCAT bus interface
 *   EtherCAT::ProxyForwardConfig px;
 *   px.enabled        = true;
 *   px.ifname         = "proxy0";         // interface facing this client
 *   px.idx_base       = 0x40;             // reserved datagram-index band
 *   px.idx_count      = 16;               //   [0x40 .. 0x4F]
 *   px.udp_input_port = 0x88A4;           // optional ETG.1000.3 input (0=off)
 *   // Optional — restrict PDO windows / force explicit ACL instead of
 *   // the auto-generated one built from the PDO map:
 *   //   px.pdo_selection.slaves = {0};  px.pdo_selection.include_inputs = false;
 *   //   px.filter = EtherCAT::ProxyPdoFilter{{ {0x1000,8,true,true}, ... }};
 *   cfg.proxy = px;
 *
 * ----------------------------------------------------------------------
 * Lab setup WITHOUT EtherCAT hardware (veth pair)
 * ----------------------------------------------------------------------
 *
 *   # "Bus" side: veth pair A stands in for the wire; "proxy" pair B for
 *   # the link between Tether and this client.
 *   ip link add pxe0 type veth peer name pxe1        # bus:   Tether <-> slaves
 *   ip link add pxp0 type veth peer name pxp1        # proxy: Tether <-> client
 *   ip link set pxe0 up ; ip link set pxe1 up
 *   ip link set pxp0 up ; ip link set pxp1 up
 *   # Only needed for --udp mode:
 *   ip addr add 10.99.0.1/24 dev pxp0
 *   ip addr add 10.99.0.2/24 dev pxp1
 *
 *   Tether master:  cfg.iface = "pxe0", px.ifname = "pxp0".
 *   This client:    ./ecat_proxy_external_source pxp1
 *                   ./ecat_proxy_external_source pxp1 --udp 10.99.0.1
 *
 *   A real "slave" can be anything that echoes EtherCAT frames — e.g. a
 *   second Tether instance in slave mode, SOEM's slave example, or the
 *   FakeSlave responder in tests/ethercat/test_proxy_integration.cpp.
 *
 *   Cleanup:  ip link del pxe0 ; ip link del pxp0
 *
 * ----------------------------------------------------------------------
 * Rules the injected traffic must obey
 * ----------------------------------------------------------------------
 *  - Datagram index inside the proxy band (idx_base .. idx_base+count-1).
 *  - Commands LRD / LWR / LRW only (PDO logical commands).
 *  - Logical address covered by the armed filter (auto-generated from the
 *    PDO map unless cfg.proxy.filter is set).  LWR needs write caps,
 *    LRD read caps, LRW both; every payload byte must be covered.
 *  - Write payloads must satisfy any armed value constraints.
 * Anything else is dropped by Tether — out-of-band frames additionally
 * bump the monitor counter / error log.
 *
 * Usage:
 *   ecat_proxy_external_source <ifname> [--idx N] [--adr HEX]
 *                            [--len N] [--count N] [--udp <ip[:port]>]
 *   (requires CAP_NET_RAW; run as root or with the cap)
 */

#include <arpa/inet.h>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif
#ifndef PACKET_IGNORE_OUTGOING
#define PACKET_IGNORE_OUTGOING 23
#endif

namespace {

constexpr size_t kDg = 14 + 2;            // first datagram offset
constexpr size_t kDgHdr = 10, kWkc = 2;

/// [eth 14][ecat hdr 2][dg hdr 10][data][wkc] — single-datagram frame.
std::vector<uint8_t> makeFrame(uint8_t cmd, uint8_t idx, uint32_t adr,
                               const uint8_t* data, uint16_t dlen)
{
    const uint16_t ec_len = uint16_t(kDgHdr + dlen + kWkc);
    std::vector<uint8_t> f(std::max<size_t>(60, 16 + ec_len), 0);
    f[12] = 0x88; f[13] = 0xA4;                      // EtherCAT EtherType
    f[14] = uint8_t(ec_len & 0xFF);
    f[15] = uint8_t((ec_len >> 8) | 0x10);           // type field = 1
    f[kDg]     = cmd;
    f[kDg + 1] = idx;
    std::memcpy(f.data() + kDg + 2, &adr, 4);        // ADP|ADO = logical
    f[kDg + 6] = uint8_t(dlen & 0xFF);
    f[kDg + 7] = uint8_t(dlen >> 8);                 // M=0: last datagram
    std::memcpy(f.data() + kDg + kDgHdr, data, dlen);
    return f;
}

int openRaw(const char* ifname)
{
    const unsigned idx = ::if_nametoindex(ifname);
    if (!idx) { std::fprintf(stderr, "if_nametoindex(%s) failed\n", ifname);
                return -1; }
    const int fd = ::socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC,
                            htons(ETH_P_ALL));
    if (fd < 0) { std::perror("socket(AF_PACKET) — need CAP_NET_RAW");
                  return -1; }
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = static_cast<int>(idx);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        std::perror("bind"); ::close(fd); return -1;
    }
    return fd;
}

bool sendRaw(int fd, const char* ifname, const uint8_t* f, size_t n)
{
    sockaddr_ll sll{};
    sll.sll_family  = AF_PACKET;
    sll.sll_ifindex = static_cast<int>(::if_nametoindex(ifname));
    sll.sll_halen   = 6;
    std::memcpy(sll.sll_addr, f, 6);
    return ::sendto(fd, f, n, 0, reinterpret_cast<sockaddr*>(&sll),
                    sizeof(sll)) == (ssize_t)n;
}

void printResponse(const uint8_t* r, size_t n, bool udp)
{
    const size_t dgo = udp ? 2 : kDg;      // UDP payload has no eth header
    if (n < dgo + kDgHdr + kWkc) { std::puts("  <short response>"); return; }
    const uint16_t dl = uint16_t(r[dgo + 6] | ((r[dgo + 7] & 0x07) << 8));
    const size_t wkc = dgo + kDgHdr + dl;
    std::printf("  idx=0x%02X adr=0x%02X%02X%02X%02X wkc=%" PRIu16
                " data:", r[dgo + 1], r[dgo + 5], r[dgo + 4],
                r[dgo + 3], r[dgo + 2],
                uint16_t(r[wkc] | (r[wkc + 1] << 8)));
    for (uint16_t i = 0; i < dl && i < 16; ++i)
        std::printf(" %02X", r[dgo + kDgHdr + i]);
    std::putchar('\n');
}

} // namespace

int main(int argc, char** argv)
{
    const char* ifname = argc > 1 ? argv[1] : nullptr;
    uint8_t idx = 0x40;                    // first index of the proxy band
    uint32_t adr = 0x1000;                 // must be inside the armed ACL
    uint16_t dlen = 8;
    unsigned count = 5;
    std::string udp_ip;
    uint16_t udp_port = 0x88A4;

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            return i + 1 < argc ? argv[++i] : "";
        };
        if (a == "--idx")        idx   = uint8_t(std::stoul(next(), nullptr, 0));
        else if (a == "--adr")   adr   = uint32_t(std::stoul(next(), nullptr, 0));
        else if (a == "--len")   dlen  = uint16_t(std::stoul(next()));
        else if (a == "--count") count = unsigned(std::stoul(next()));
        else if (a == "--udp") {
            udp_ip = next();
            if (auto c = udp_ip.find(':'); c != std::string::npos) {
                udp_port = uint16_t(std::stoul(udp_ip.substr(c + 1)));
                udp_ip.resize(c);
            }
        } else {
            std::fprintf(stderr, "unknown arg '%s'\n", a.c_str());
            return 2;
        }
    }
    if (!ifname) {
        std::fprintf(stderr, "usage: %s <ifname> [--idx N] [--adr HEX] "
                     "[--len N] [--count N] [--udp <ip[:port]>]\n", argv[0]);
        return 2;
    }

    // LRW over [adr, adr+len): reads inputs AND writes outputs in one
    // datagram — the pattern SOEM-style cyclic sources use.
    constexpr uint8_t kLRW = 0x0C;
    std::vector<uint8_t> data(dlen);
    for (uint16_t i = 0; i < dlen; ++i) data[i] = uint8_t(0x10 + i);
    const auto frame = makeFrame(kLRW, idx, adr, data.data(), dlen);
    // UDP payload = bare EtherCAT frame (Ethernet header stripped).
    const std::vector<uint8_t> payload(frame.begin() + 14, frame.end());

    std::printf("proxy source on %s: LRW idx=0x%02X adr=0x%08X len=%u "
                "(%s mode, %u frame%s)\n", ifname, idx, adr, dlen,
                udp_ip.empty() ? "raw EtherCAT" : "EtherCAT/UDP",
                count, count == 1 ? "" : "s");

    int fd = -1;
    sockaddr_in dst{};
    if (udp_ip.empty()) {
        fd = openRaw(ifname);
    } else {
        fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        dst.sin_family      = AF_INET;
        dst.sin_port        = htons(udp_port);
        if (::inet_pton(AF_INET, udp_ip.c_str(), &dst.sin_addr) != 1) {
            std::fprintf(stderr, "bad IPv4 '%s'\n", udp_ip.c_str());
            return 2;
        }
    }
    if (fd < 0) return 1;

    unsigned got = 0;
    for (unsigned i = 0; i < count; ++i) {
        const bool sent = udp_ip.empty()
            ? sendRaw(fd, ifname, frame.data(), frame.size())
            : ::sendto(fd, payload.data(), payload.size(), 0,
                       reinterpret_cast<sockaddr*>(&dst),
                       sizeof(dst)) == (ssize_t)payload.size();
        if (!sent) { std::perror("send"); break; }

        // Await the (possibly filtered/rejected) response.
        pollfd p{fd, POLLIN, 0};
        uint8_t buf[2048];
        bool answered = false;
        for (int w = 0; w < 10 && !answered; ++w) {           // ~1 s
            if (::poll(&p, 1, 100) <= 0) continue;
            const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) continue;
            if (udp_ip.empty() &&                       // skip IPv6 ND noise
                !(n > 17 && buf[12] == 0x88 && buf[13] == 0xA4 &&
                  buf[kDg + 1] == idx)) continue;
            std::printf("[%u] response:", i);
            printResponse(buf, static_cast<size_t>(n), !udp_ip.empty());
            answered = true; ++got;
        }
        if (!answered)
            std::printf("[%u] no response (rejected/filtered/dropped?)\n", i);
        if (i + 1 < count)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::printf("%u/%u exchanges answered\n", got, count);
    ::close(fd);
    return got || count == 0 ? 0 : 1;
}
