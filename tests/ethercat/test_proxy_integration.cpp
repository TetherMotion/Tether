/**
 * @file test_proxy_integration.cpp
 * @brief Live end-to-end tests for the external-source PDO proxy
 *        ("EtherCAT proxy") over kernel veth pairs.
 *
 * Topology built per test (requires CAP_NET_ADMIN + CAP_NET_RAW —
 * skipped cleanly without them):
 *
 *   [fake slave] pxe1 <-> pxe0 [ProxyForwarder] pxp0 <-> pxp1 [external src]
 *        AF_PACKET            AF_PACKET×3 (+UDP)          AF_PACKET / UDP
 *
 * The pxe and pxp pairs are veth interfaces created with `ip link`.
 * The fake slave echoes
 * the "external source" injects proxy-band PDO traffic either as raw
 * EtherCAT frames on pxp1 or as ETG.1000.3 UDP datagrams to pxp0's IPv4
 * address (10.99.0.1 / 10.99.0.2 on the two ends).
 */

#include <gtest/gtest.h>

#ifdef __linux__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif
#ifndef PACKET_IGNORE_OUTGOING
#define PACKET_IGNORE_OUTGOING 23
#endif

#include "tether/ethercat/ProxyForwardConfig.hpp"
#include "tether/ethercat/Types.hpp"
#include "ethercat/raw/ProxyForwarder.hpp"

using namespace EtherCAT;

namespace {

// [eth 14][ecat hdr 2][dg hdr 10][data][wkc] — single-datagram frames.
constexpr size_t kDgOff   = 14 + 2;
constexpr size_t kDgHdr   = 10;
constexpr size_t kWkcLen  = 2;

constexpr uint8_t kLRD = static_cast<uint8_t>(Command::LRD);  // 0x0A
constexpr uint8_t kLWR = static_cast<uint8_t>(Command::LWR);  // 0x0B
constexpr uint8_t kLRW = static_cast<uint8_t>(Command::LRW);  // 0x0C
constexpr uint8_t kFPRD = static_cast<uint8_t>(Command::FPRD);

// Proxy band used throughout: [0x40, 0x4F].
constexpr uint8_t kLo = 0x40, kHi = 0x4F;
constexpr uint16_t kUdpPort = 0x88A4;

std::vector<uint8_t> makeFrame(uint8_t cmd, uint8_t idx, uint32_t adr,
                               const std::vector<uint8_t>& data = {})
{
    const uint16_t dl = static_cast<uint16_t>(data.size());
    const uint16_t ec_len = static_cast<uint16_t>(kDgHdr + dl + kWkcLen);
    std::vector<uint8_t> f(std::max<size_t>(60, 14 + 2 + ec_len), 0);
    f[12] = 0x88; f[13] = 0xA4;                       // EtherType EtherCAT
    f[14] = static_cast<uint8_t>(ec_len & 0xFF);
    f[15] = static_cast<uint8_t>((ec_len >> 8) | 0x10);  // type = 1
    f[kDgOff]     = cmd;
    f[kDgOff + 1] = idx;
    f[kDgOff + 2] = static_cast<uint8_t>(adr & 0xFF);
    f[kDgOff + 3] = static_cast<uint8_t>((adr >> 8) & 0xFF);
    f[kDgOff + 4] = static_cast<uint8_t>((adr >> 16) & 0xFF);
    f[kDgOff + 5] = static_cast<uint8_t>((adr >> 24) & 0xFF);
    f[kDgOff + 6] = static_cast<uint8_t>(dl & 0xFF);
    f[kDgOff + 7] = static_cast<uint8_t>(dl >> 8);    // M=0: last datagram
    if (dl) std::memcpy(f.data() + kDgOff + kDgHdr, data.data(), dl);
    return f;
}

/// EtherCAT payload of a single-dg frame: [ecat hdr][dg hdr][data][wkc].
std::vector<uint8_t> ecatPayload(const std::vector<uint8_t>& frame) {
    const uint16_t ec_len = static_cast<uint16_t>(
        frame[14] | ((frame[15] & 0x07) << 8));
    return {frame.begin() + 14, frame.begin() + 14 + 2 + ec_len};
}

int runIp(const std::string& args) {
    const std::string cmd = "ip " + args + " 2>/dev/null";
    return ::system(cmd.c_str());
}

int openPacketSock(const char* ifname) {
    const unsigned idx = ::if_nametoindex(ifname);
    if (!idx) return -1;
    const int fd = ::socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC,
                            htons(ETH_P_ALL));
    if (fd < 0) return -1;
    int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int one = 1;
    // Suppress our own TX echo — only peer-originated traffic matters.
    ::setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = static_cast<int>(idx);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        ::close(fd); return -1;
    }
    return fd;
}

ssize_t sendOn(int fd, const char* ifname, const uint8_t* f, size_t len) {
    sockaddr_ll sll{};
    sll.sll_family  = AF_PACKET;
    sll.sll_ifindex = static_cast<int>(::if_nametoindex(ifname));
    sll.sll_halen   = 6;
    std::memcpy(sll.sll_addr, f, 6);
    return ::sendto(fd, f, len, MSG_DONTWAIT,
                    reinterpret_cast<sockaddr*>(&sll), sizeof(sll));
}

/// Poll-recv the first datagram for which `pred` holds (default: any),
/// within `timeout_ms`; {} on timeout.  Link-local IPv6 ND chatter on
/// freshly-created veth pairs makes the filter non-optional for
/// AF_PACKET sockets.
std::vector<uint8_t> recvFrame(int fd,
    bool (*pred)(const uint8_t*, size_t) = nullptr,
    int timeout_ms = 3000)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    uint8_t buf[2048];
    for (;;) {
        const auto left = std::chrono::duration_cast<
            std::chrono::milliseconds>(deadline -
                std::chrono::steady_clock::now()).count();
        if (left <= 0) return {};
        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, static_cast<int>(left)) <= 0) return {};
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) continue;
        if (!pred || pred(buf, static_cast<size_t>(n)))
            return {buf, buf + n};
    }
}

bool isEcat(const uint8_t* f, size_t n) {
    return n > 16 && f[12] == 0x88 && f[13] == 0xA4;
}

/// Echo responder standing in for the EtherCAT segment: copies any
/// EtherCAT frame back with WKC=1 on its last datagram.
struct FakeSlave {
    int fd = -1;
    std::atomic<bool> stop{false};
    std::atomic<unsigned> received{0};
    std::thread th;
    const char* ifname = nullptr;

    bool start(const char* nif) {
        ifname = nif;
        fd = openPacketSock(nif);
        if (fd < 0) return false;
        th = std::thread([this] { loop(); });
        return true;
    }
    ~FakeSlave() {
        stop.store(true);
        if (th.joinable()) th.join();
        if (fd >= 0) ::close(fd);
    }
    void loop() {
        uint8_t buf[2048];
        while (!stop.load(std::memory_order_relaxed)) {
            pollfd p{fd, POLLIN, 0};
            if (::poll(&p, 1, 50) <= 0) continue;
            const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
            if (n < 18 + 2 + kDgHdr + kWkcLen) continue;
            if (buf[12] != 0x88 || buf[13] != 0xA4) continue;
            received.fetch_add(1, std::memory_order_relaxed);
            const uint16_t dl = static_cast<uint16_t>(
                buf[kDgOff + 6] | ((buf[kDgOff + 7] & 0x07) << 8));
            const size_t wkc = kDgOff + kDgHdr + dl;      // single-dg WKC
            buf[wkc]     = 1;
            buf[wkc + 1] = 0;
            sendOn(fd, ifname, buf, static_cast<size_t>(n));
        }
    }
};

} // namespace

class ProxyIntegration : public ::testing::Test {
protected:
    // Unique per-PID names keep concurrent CI jobs apart (IFNAMSIZ=16).
    std::string e0, e1, p0, p1;
    int ecat_ifindex = 0;

    void SetUp() override {
        const auto tag = std::to_string(::getpid() & 0xFFFF);
        e0 = "pxe0_" + tag; e1 = "pxe1_" + tag;
        p0 = "pxp0_" + tag; p1 = "pxp1_" + tag;
        if (runIp("-Version") != 0)
            GTEST_SKIP() << "iproute2 'ip' binary unavailable";
        // veth creation is the privilege probe (CAP_NET_ADMIN).
        if (runIp("link add " + e0 + " type veth peer name " + e1) != 0)
            GTEST_SKIP() << "cannot create veth pair — need CAP_NET_ADMIN";
        if (runIp("link add " + p0 + " type veth peer name " + p1) != 0) {
            runIp("link del " + e0);
            GTEST_SKIP() << "cannot create second veth pair";
        }
        for (const char* n : {e0.c_str(), e1.c_str(), p0.c_str(), p1.c_str()})
            ASSERT_EQ(runIp(std::string("link set ") + n + " up"), 0);
        // Proxy pair gets IPv4 for the UDP-input tests.
        ASSERT_EQ(runIp("addr add 10.99.0.1/24 dev " + p0), 0);
        ASSERT_EQ(runIp("addr add 10.99.0.2/24 dev " + p1), 0);
        ecat_ifindex = static_cast<int>(::if_nametoindex(e0.c_str()));
        ASSERT_NE(ecat_ifindex, 0);
    }

    void TearDown() override {
        runIp("link del " + e0);       // deleting one end drops the pair
        runIp("link del " + p0);
    }

    /// Forwarder armed with a write-only window at 0x1000..0x100F.
    ProxyForwardConfig cfg(uint16_t udp_port = 0) const {
        ProxyForwardConfig c;
        c.enabled       = true;
        c.ifname        = p0;
        c.idx_base      = kLo;
        c.idx_count     = static_cast<uint8_t>(kHi - kLo + 1);
        c.udp_input_port = udp_port;
        ProxyPdoFilter f;
        f.access.push_back({0x1000, 16, /*read=*/true, /*write=*/true});
        c.filter = f;
        return c;
    }
};

// ------------------------------------------------------------------
// Raw EtherCAT path: inject on pxp1 → slave sees it on pxe1 →
// response forwarded back to pxp1.
// ------------------------------------------------------------------
TEST_F(ProxyIntegration, RawFrameEndToEnd)
{
    FakeSlave slave;
    ASSERT_TRUE(slave.start(e1.c_str()));
    const int src = openPacketSock(p1.c_str());
    ASSERT_GE(src, 0) << "AF_PACKET failed — need CAP_NET_RAW";

    ProxyForwarder fwd;
    const WireEncap encap{};
    auto c = cfg();
    ASSERT_TRUE(fwd.start(c, ecat_ifindex, encap, &*c.filter))
        << fwd.lastError();

    // In-band LWR inside the ACL window → forwarded to the slave.
    const auto good = makeFrame(kLWR, kLo, 0x1000, {1, 2, 3, 4});
    ASSERT_GT(sendOn(src, p1.c_str(), good.data(), good.size()), 0);

    // Wait for the slave to have echoed it.
    for (int i = 0; i < 100 && slave.received.load() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_EQ(slave.received.load(), 1u);

    const auto resp = recvFrame(src, isEcat);
    ASSERT_FALSE(resp.empty()) << "no forwarded response on proxy if";
    EXPECT_EQ(resp[kDgOff + 1], kLo);         // same idx
    // WKC byte set by the fake slave.
    const uint16_t dl = static_cast<uint16_t>(
        resp[kDgOff + 6] | ((resp[kDgOff + 7] & 0x07) << 8));
    EXPECT_EQ(resp[kDgOff + kDgHdr + dl], 1);

    const auto st = fwd.stats();
    EXPECT_GE(st.tx_forwarded, 1u);
    EXPECT_GE(st.rx_forwarded, 1u);
    ::close(src);
    fwd.stop();
}

// ------------------------------------------------------------------
// Out-of-band index and ACL violation never reach the bus.
// ------------------------------------------------------------------
TEST_F(ProxyIntegration, OutOfBandAndAclDropped)
{
    FakeSlave slave;
    ASSERT_TRUE(slave.start(e1.c_str()));
    const int src = openPacketSock(p1.c_str());
    ASSERT_GE(src, 0) << "AF_PACKET failed — need CAP_NET_RAW";

    ProxyForwarder fwd;
    const WireEncap encap{};
    auto c = cfg();
    ASSERT_TRUE(fwd.start(c, ecat_ifindex, encap, &*c.filter))
        << fwd.lastError();

    // Out-of-band idx → monitor socket counts it, nothing forwarded.
    const auto oob = makeFrame(kLWR, 0x20, 0x1000, {1, 2, 3, 4});
    sendOn(src, p1.c_str(), oob.data(), oob.size());

    // In-band but outside the ACL window → filtered.
    const auto acl = makeFrame(kLWR, kLo, 0x2000, {1, 2, 3, 4});
    sendOn(src, p1.c_str(), acl.data(), acl.size());

    // In-band but a non-PDO command → structurally rejected.
    const auto npec = makeFrame(kFPRD, kLo, 0x1000, {0, 0});
    sendOn(src, p1.c_str(), npec.data(), npec.size());

    for (int i = 0; i < 100 &&
         (fwd.stats().out_of_band == 0 || fwd.stats().tx_rejected == 0 ||
          fwd.stats().filtered == 0); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const auto st = fwd.stats();
    EXPECT_GE(st.out_of_band, 1u);     // monitor saw the 0x20 frame
    EXPECT_GE(st.tx_rejected, 1u);     // FPRD rejected
    EXPECT_GE(st.filtered, 1u);        // 0x2000 outside the ACL window
    EXPECT_EQ(slave.received.load(), 0u);   // nothing reached the bus
    EXPECT_EQ(st.tx_forwarded, 0u);
    ::close(src);
    fwd.stop();
}

// ------------------------------------------------------------------
// ETG.1000.3 UDP input: bare EtherCAT frame in a UDP datagram →
// decapsulated, validated, injected; response re-encapsulated back.
// ------------------------------------------------------------------
TEST_F(ProxyIntegration, UdpInputEndToEnd)
{
    FakeSlave slave;
    ASSERT_TRUE(slave.start(e1.c_str()));

    ProxyForwarder fwd;
    const WireEncap encap{};
    auto c = cfg(kUdpPort);
    ASSERT_TRUE(fwd.start(c, ecat_ifindex, encap, &*c.filter))
        << fwd.lastError();

    const int udp = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    ASSERT_GE(udp, 0);
    int flags = ::fcntl(udp, F_GETFL, 0);
    ::fcntl(udp, F_SETFL, flags | O_NONBLOCK);
    sockaddr_in mine{};
    mine.sin_family      = AF_INET;
    mine.sin_addr.s_addr = htonl(INADDR_ANY);
    mine.sin_port        = 0;
    ASSERT_EQ(::bind(udp, reinterpret_cast<sockaddr*>(&mine),
                     sizeof(mine)), 0);

    sockaddr_in dst{};
    dst.sin_family      = AF_INET;
    dst.sin_addr.s_addr = htonl(0x0A630001);        // 10.99.0.1
    dst.sin_port        = htons(kUdpPort);

    // Bare EtherCAT frame (no Ethernet header) as UDP payload.
    const auto frame   = makeFrame(kLWR, kLo, 0x1000, {0xAA, 0xBB});
    const auto payload = ecatPayload(frame);
    ASSERT_GT(::sendto(udp, payload.data(), payload.size(), 0,
                       reinterpret_cast<sockaddr*>(&dst), sizeof(dst)), 0);

    for (int i = 0; i < 100 && slave.received.load() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_EQ(slave.received.load(), 1u);

    // Response comes back as a UDP datagram = bare EtherCAT frame.
    const auto resp = recvFrame(udp);
    ASSERT_FALSE(resp.empty()) << "no UDP reply";
    ASSERT_GT(resp.size(), 2 + kDgHdr);
    // payload[0..1] = ecat hdr; first datagram cmd at +2, idx at +3.
    EXPECT_EQ(resp[2], kLWR);
    EXPECT_EQ(resp[3], kLo);

    const auto st = fwd.stats();
    EXPECT_GE(st.tx_forwarded, 1u);
    ::close(udp);
    fwd.stop();
}

// ------------------------------------------------------------------
// UDP input is opt-in: without udp_input_port nothing listens.
// ------------------------------------------------------------------
TEST_F(ProxyIntegration, UdpIgnoredWhenNotConfigured)
{
    FakeSlave slave;
    ASSERT_TRUE(slave.start(e1.c_str()));

    ProxyForwarder fwd;
    const WireEncap encap{};
    auto c = cfg(/*udp_port=*/0);
    ASSERT_TRUE(fwd.start(c, ecat_ifindex, encap, &*c.filter))
        << fwd.lastError();

    const int udp = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    ASSERT_GE(udp, 0);
    sockaddr_in dst{};
    dst.sin_family      = AF_INET;
    dst.sin_addr.s_addr = htonl(0x0A630001);
    dst.sin_port        = htons(kUdpPort);
    const auto payload = ecatPayload(makeFrame(kLWR, kLo, 0x1000, {1}));
    ::sendto(udp, payload.data(), payload.size(), 0,
             reinterpret_cast<sockaddr*>(&dst), sizeof(dst));

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_EQ(slave.received.load(), 0u);
    EXPECT_EQ(fwd.stats().tx_forwarded, 0u);
    ::close(udp);
    fwd.stop();
}

#else // !__linux__

TEST(ProxyIntegration, LinuxOnly) { GTEST_SKIP() << "Linux-only test"; }

#endif
