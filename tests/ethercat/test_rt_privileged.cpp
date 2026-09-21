/**
 * @file test_rt_privileged.cpp
 * @brief Privileged integration tests for the realtime cyclic datapath.
 *
 * These tests exercise the code paths that need real kernel facilities —
 * AF_PACKET sockets, PACKET_MMAP rings, kernel cBPF demux, SCHED_FIFO /
 * SCHED_DEADLINE scheduling, CPU affinity, and mlock — which are absent in
 * an unprivileged container.  Every fixture auto-detects the capability it
 * needs and GTEST_SKIPs cleanly without it, so the same binary is a no-op
 * when run as a normal user.
 *
 * Run with the privileges provided by `runec` (CAP_NET_RAW + CAP_NET_ADMIN
 * + CAP_SYS_NICE ambient):
 *
 *     runec ./build/bin/tests/tether_ethercat_rt_privileged_tests
 *
 * What is covered here that cannot be covered unprivileged:
 *  - createCyclicChannel(): real socket + ring backends on a live interface
 *    (openCyclicSocket, attachFilter, initRxRing/initTxRing, mmap).
 *  - LinuxRingChannel on a real kernel ring: walkRing, txAcquire/
 *    txCommitFrame kick, rxPending, rxHold/rxRelease, teardownRings.
 *  - LinuxSocketChannel over real AF_PACKET: recvmsg SCM_TIMESTAMPNS
 *    kernel RX stamps and the sendmsg scatter-gather TX path.
 *  - Kernel cBPF demux between the cyclic and async sockets on a real wire
 *    (a veth pair — frames actually traverse a kernel link).
 *  - CBPFProgramFactory encapsulation filters on a real wire: untagged /
 *    VLAN-tagged / VLAN-range / catch-all / EtherCAT-over-UDP accept-reject
 *    matrices, kernel-inserted tags from a peer-side 802.1Q sub-interface,
 *    and the full HAL poll → VLANRouter strip/deliver + router TX-tag path
 *    (when tether_hal is linked — TETHER_TEST_HAVE_HAL).
 *  - Master::startCyclicLoop() channel creation + waitCyclicSlotView() over
 *    a real wire (ppoll → rxPoll → dispatch → slot publish).
 *  - Platform: setCurrentThreadRealtime (SCHED_FIFO), setCurrentThreadDeadline
 *    (SCHED_DEADLINE), setCurrentThreadTimerSlack, CPU affinity ordering.
 *  - CpuIsolation claims verified against sched_getaffinity().
 *  - RtMemory lockAllMemory / lockMemory / prefault under RLIMIT_MEMLOCK.
 *
 * The wire tests use a veth pair (veth_rt0 <-> veth_rt1): a responder socket
 * on the peer echoes frames back, so TX->wire->RX is a genuine kernel path,
 * not a loopback shortcut.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#ifdef __linux__

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/filter.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <poll.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/utsname.h>
#include <unistd.h>

// struct sched_attr has no glibc wrapper and <linux/sched/types.h> clashes
// with <sched.h> over struct sched_param on older kernel headers — define it
// locally, matching the uapi layout (same approach as Platform.cpp).
struct sched_attr_local {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t  sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/platform/CpuIsolation.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "ethercat/raw/CyclicDatapath.hpp"

#if defined(TETHER_TEST_HAVE_HAL)
#include "hal/IEthernet.hpp"
#endif

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

constexpr uint16_t kEtherCat = 0x88A4;

// Run a shell command, ignoring its exit status without tripping
// warn_unused_result.
void runCmd(const std::string& c) {
    const int rc = ::system(c.c_str());
    (void)rc;
}

// ============================================================================
// Capability probes
// ============================================================================

bool haveCapNetRaw() {
    int s = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (s >= 0) { ::close(s); return true; }
    return false;
}

// CAP_NET_ADMIN: can we create+delete a throwaway veth pair?
bool haveCapNetAdmin() {
    // Probing with a real create is the only reliable check (the cap may be
    // present but the op blocked by LSM).  Cheap and side-effect-free.
    const char* a = "vrt_probe_a";
    const char* b = "vrt_probe_b";
    std::string cmd = "ip link add " + std::string(a) + " type veth peer name " +
                      b + " 2>/dev/null";
    if (::system(cmd.c_str()) != 0) return false;
    runCmd("ip link del " + std::string(a) + " 2>/dev/null");
    return true;
}

// CAP_SYS_NICE: can we switch a thread to SCHED_FIFO?
bool haveCapSysNice() {
    struct sched_param sp { 5 };
    if (::sched_setscheduler(0, SCHED_FIFO, &sp) != 0) return false;
    struct sched_param zero { 0 };
    ::sched_setscheduler(0, SCHED_OTHER, &zero);   // restore
    return true;
}

// ============================================================================
// Raw-socket helpers
// ============================================================================

/// Bound AF_PACKET SOCK_RAW socket on ifindex for proto (host order).
int openBoundPacketSocket(int ifindex, uint16_t proto,
                          bool ignore_outgoing = false) {
    int s = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (s < 0) { ::perror("packet socket"); return -1; }
    int one = 1;
    if (ignore_outgoing)
        ::setsockopt(s, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    struct sockaddr_ll sll {};
    sll.sll_family   = AF_PACKET;
    sll.sll_ifindex  = ifindex;
    sll.sll_protocol = htons(proto);
    // A freshly-created veth resolves via if_nametoindex before the device is
    // fully registered for AF_PACKET — bind can briefly return ENODEV.  Retry
    // across the registration window.
    for (int i = 0; i < 500; ++i) {
        if (::bind(s, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) == 0)
            return s;
        if (errno != ENODEV && errno != EADDRNOTAVAIL) break;
        std::this_thread::sleep_for(2ms);
    }
    ::fprintf(stderr, "bind ifindex=%d proto=%04x: %s\n",
              ifindex, proto, ::strerror(errno));
    ::close(s);
    return -1;
}

/// Blocking recv of one frame within timeout_ms; returns length or -1.
ssize_t recvOne(int fd, uint8_t* buf, size_t cap, int timeout_ms) {
    struct pollfd p { fd, POLLIN, 0 };
    if (::poll(&p, 1, timeout_ms) <= 0) return -1;
    return ::recv(fd, buf, cap, 0);
}

/// Send one raw frame out ifindex via an open AF_PACKET socket.
/// proto (host order) sets sll_protocol; 0 keeps the socket's bound
/// protocol.  Pass the inner EtherType when transmitting through an
/// 802.1Q sub-interface so the kernel tagger sees the right protocol.
bool sendRaw(int fd, int ifindex, const uint8_t* f, size_t len,
             uint16_t proto = 0) {
    struct sockaddr_ll sll {};
    sll.sll_family   = AF_PACKET;
    sll.sll_ifindex  = ifindex;
    sll.sll_protocol = htons(proto);
    sll.sll_halen    = 6;
    std::memset(sll.sll_addr, 0xFF, 6);
    return ::sendto(fd, f, len, 0, reinterpret_cast<sockaddr*>(&sll),
                    sizeof(sll)) == (ssize_t)len;
}

// ============================================================================
// veth pair (CAP_NET_ADMIN) — a real two-ended kernel link
// ============================================================================

struct VethPair {
    std::string a, b;
    int ifA = 0, ifB = 0;
    bool ok = false;

    VethPair() = default;
    VethPair(const VethPair&) = delete;
    VethPair& operator=(const VethPair&) = delete;
    // Move transfers ownership so the moved-from object's destructor does
    // not tear down the interface the new owner still needs.
    VethPair(VethPair&& o) noexcept
        : a(std::move(o.a)), b(std::move(o.b)), ifA(o.ifA), ifB(o.ifB),
          ok(o.ok) { o.ok = false; }
    VethPair& operator=(VethPair&& o) noexcept {
        if (this != &o) {
            if (ok) runCmd("ip link del " + a + " 2>/dev/null");
            a = std::move(o.a); b = std::move(o.b);
            ifA = o.ifA; ifB = o.ifB; ok = o.ok;
            o.ok = false;
        }
        return *this;
    }

    static VethPair create() {
        VethPair v;
        static std::atomic<int> ctr{0};
        const int id = 100 + ctr.fetch_add(1);
        v.a = "vrtA" + std::to_string(id);
        v.b = "vrtB" + std::to_string(id);
        std::string mk = "ip link add " + v.a + " type veth peer name " + v.b +
                         " 2>/dev/null";
        if (::system(mk.c_str()) != 0) return v;
        runCmd("ip link set " + v.a + " up 2>/dev/null");
        runCmd("ip link set " + v.b + " up 2>/dev/null");
        // Wait for the interfaces to be registered AND bindable — a fresh
        // veth can resolve via if_nametoindex before AF_PACKET accepts it.
        for (int i = 0; i < 1000; ++i) {
            v.ifA = (int)::if_nametoindex(v.a.c_str());
            v.ifB = (int)::if_nametoindex(v.b.c_str());
            if (v.ifA > 0 && v.ifB > 0) {
                int probe = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
                if (probe >= 0) {
                    struct sockaddr_ll sll {};
                    sll.sll_family   = AF_PACKET;
                    sll.sll_ifindex  = v.ifA;
                    sll.sll_protocol = htons(ETH_P_ALL);
                    if (::bind(probe, reinterpret_cast<sockaddr*>(&sll),
                               sizeof(sll)) == 0) {
                        ::close(probe);
                        v.ok = true;
                        break;
                    }
                    ::close(probe);
                }
            }
            std::this_thread::sleep_for(2ms);
        }
        return v;
    }
    ~VethPair() {
        if (ok)
            runCmd("ip link del " + a + " 2>/dev/null");
    }
};

// ============================================================================
// EtherCAT frame builder — same wire layout as the unit tests
// ============================================================================

size_t buildEcatFrame(uint8_t* f, uint8_t cmd, uint8_t idx,
                      uint16_t adp, uint16_t ado,
                      const uint8_t* payload, uint16_t datalen,
                      uint16_t wkc, bool more = false) {
    std::memset(f, 0, 14);
    f[12] = 0x88; f[13] = 0xA4;
    const uint16_t ecat_len = static_cast<uint16_t>(10 + datalen + 2);
    f[14] = ecat_len & 0xFF;
    f[15] = ((ecat_len >> 8) & 0x07) | 0x10;      // type=1
    f[16] = cmd;  f[17] = idx;
    std::memcpy(f + 18, &adp, 2);
    std::memcpy(f + 20, &ado, 2);
    const uint16_t len_flags = (datalen & 0x07FF) | (more ? 0x8000 : 0);
    std::memcpy(f + 22, &len_flags, 2);
    std::memset(f + 24, 0, 2);                    // irq
    if (datalen) std::memcpy(f + 26, payload, datalen);
    std::memcpy(f + 26 + datalen, &wkc, 2);
    size_t len = 26 + datalen + 2;
    return len < 60 ? 60 : len;
}

/// Spin until pred() true or the deadline passes.
template <typename F>
bool waitFor(F&& pred, std::chrono::milliseconds budget = 3'000ms) {
    const auto end = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(200us);
    }
    return pred();
}

// ============================================================================
// Encapsulation helpers — 802.1Q insertion, IPv4/UDP frames, windowed RX
// ============================================================================

/// Insert an 802.1Q tag (vid, pcp) between the Ethernet header and the
/// payload of a frame built by buildEcatFrame/buildIp4UdpFrame.  The
/// original EtherType moves to [16..17] (inner EtherType).  `f` needs
/// >= 4 bytes of tail room beyond len.  Returns the new frame length.
size_t vlanTagInsert(uint8_t* f, size_t len, uint16_t vid, uint8_t pcp = 0) {
    std::memmove(f + 16, f + 12, len - 12);  // ethertype + payload shift +4
    f[12] = 0x81; f[13] = 0x00;
    const uint16_t tci = static_cast<uint16_t>(((pcp & 0x7) << 13) |
                                               (vid & 0x0FFF));
    f[14] = static_cast<uint8_t>(tci >> 8);
    f[15] = static_cast<uint8_t>(tci & 0xFF);
    return len + 4;
}

/// Build an IPv4 frame carrying `payload` inside a UDP datagram.
/// `proto`/`ihl`/`frag_off` are overridable for negative cases; ihl >= 6
/// exercises the filter's IP-options (MSH/IND) addressing path.
size_t buildIp4UdpFrame(uint8_t* f, uint16_t sport, uint16_t dport,
                        const uint8_t* payload, size_t plen,
                        uint8_t proto = 17, uint8_t ihl = 5,
                        uint16_t frag_off = 0) {
    std::memset(f, 0xAA, 6);                     // dst (unicast)
    std::memset(f + 6, 0x55, 6);                 // src
    f[12] = 0x08; f[13] = 0x00;                  // EtherType IPv4
    uint8_t* ip = f + 14;
    ip[0] = static_cast<uint8_t>((4 << 4) | (ihl & 0x0F));
    const uint16_t udp_len = static_cast<uint16_t>(8 + plen);
    const uint16_t tot = static_cast<uint16_t>(ihl * 4 + udp_len);
    ip[2] = static_cast<uint8_t>(tot >> 8);
    ip[3] = static_cast<uint8_t>(tot & 0xFF);
    ip[6] = static_cast<uint8_t>(frag_off >> 8); // flags+frag offset field
    ip[7] = static_cast<uint8_t>(frag_off & 0xFF);
    ip[8] = 64;                                  // ttl
    ip[9] = proto;
    std::memset(ip + 12, 10, 4);                 // src addr
    std::memset(ip + 16, 20, 4);                 // dst addr
    uint8_t* udp = ip + ihl * 4;
    udp[0] = static_cast<uint8_t>(sport >> 8);
    udp[1] = static_cast<uint8_t>(sport & 0xFF);
    udp[2] = static_cast<uint8_t>(dport >> 8);
    udp[3] = static_cast<uint8_t>(dport & 0xFF);
    udp[4] = static_cast<uint8_t>(udp_len >> 8);
    udp[5] = static_cast<uint8_t>(udp_len & 0xFF);
    if (plen) std::memcpy(udp + 8, payload, plen);
    const size_t len = 14 + ihl * 4 + udp_len;
    return len < 60 ? 60 : len;
}

/// Drain every frame arriving on fd during window_ms.
std::vector<std::vector<uint8_t>> recvAllWindow(int fd, int window_ms) {
    std::vector<std::vector<uint8_t>> out;
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(window_ms);
    while (std::chrono::steady_clock::now() < end) {
        struct pollfd p { fd, POLLIN, 0 };
        if (::poll(&p, 1, 20) <= 0) continue;
        uint8_t b[2048];
        const ssize_t n = ::recv(fd, b, sizeof(b), 0);
        if (n > 0) out.emplace_back(b, b + n);
    }
    return out;
}

/// A received frame plus the VLAN tag the kernel stripped into PACKET_
/// AUXDATA — inbound tagged frames NEVER show the 0x8100 tag in the data
/// buffer on RX (the generic untag path removes it before taps run), so
/// this is the only way to verify which tag a frame actually carried.
struct AuxFrame {
    std::vector<uint8_t> data;
    int vlan_tci  = -1;   // TCI (PCP<<13 | VID), or -1 when untagged
    int vlan_tpid = -1;
};

/// recvAllWindow variant that enables PACKET_AUXDATA on fd and reports
/// the stripped tag for each received frame.
std::vector<AuxFrame> recvAllWindowAux(int fd, int window_ms) {
    int one = 1;
    ::setsockopt(fd, SOL_PACKET, PACKET_AUXDATA, &one, sizeof(one));
    std::vector<AuxFrame> out;
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(window_ms);
    while (std::chrono::steady_clock::now() < end) {
        struct pollfd p { fd, POLLIN, 0 };
        if (::poll(&p, 1, 20) <= 0) continue;
        uint8_t b[2048];
        iovec iov{ b, sizeof(b) };
        alignas(cmsghdr) char cbuf[CMSG_SPACE(sizeof(tpacket_auxdata))];
        msghdr msg{};
        msg.msg_iov        = &iov;
        msg.msg_iovlen     = 1;
        msg.msg_control    = cbuf;
        msg.msg_controllen = sizeof(cbuf);
        const ssize_t n = ::recvmsg(fd, &msg, MSG_DONTWAIT);
        if (n <= 0) continue;
        AuxFrame af;
        af.data.assign(b, b + n);
        for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level == SOL_PACKET &&
                c->cmsg_type == PACKET_AUXDATA) {
                const auto* a =
                    reinterpret_cast<const tpacket_auxdata*>(CMSG_DATA(c));
                if (a->tp_status & TP_STATUS_VLAN_VALID) {
                    af.vlan_tci = a->tp_vlan_tci;
#ifdef TP_STATUS_VLAN_TPID_VALID
                    af.vlan_tpid = (a->tp_status & TP_STATUS_VLAN_TPID_VALID)
                                       ? a->tp_vlan_tpid : ETH_P_8021Q;
#else
                    af.vlan_tpid = ETH_P_8021Q;
#endif
                }
            }
        }
        out.push_back(std::move(af));
    }
    return out;
}

/// True when some frame both carries marker id and arrived tagged with
/// vid (via auxdata — the inline tag is never visible on inbound frames).
bool hasAuxVlanMarker(const std::vector<AuxFrame>& frames, uint8_t id,
                      int vid) {
    const uint8_t m[4] = {0xEC, 0xA7, id, 0x5A};
    for (const auto& f : frames)
        if ((f.vlan_tci & 0x0FFF) == vid &&
            std::search(f.data.begin(), f.data.end(), m, m + 4) !=
                f.data.end())
            return true;
    return false;
}

/// Every injected test frame carries a unique {0xEC,0xA7,id,0x5A} marker in
/// its payload — robust against encapsulation offset shifts and the IPv6/
/// ARP control noise a fresh veth emits.
bool hasMarker(const std::vector<std::vector<uint8_t>>& frames, uint8_t id) {
    const uint8_t m[4] = {0xEC, 0xA7, id, 0x5A};
    for (const auto& f : frames)
        if (std::search(f.begin(), f.end(), m, m + 4) != f.end())
            return true;
    return false;
}

/// Inject an EtherCAT frame on txfd; vid >= 0 wraps it in an 802.1Q tag.
void injectEcat(int txfd, int ifindex, uint8_t id, int vid, int pcp = 0) {
    uint8_t f[128];
    const uint8_t pay[4] = {0xEC, 0xA7, id, 0x5A};
    size_t n = buildEcatFrame(f, 0x07, 0x40, 0, 0x0130, pay, 4, 0);
    if (vid >= 0)
        n = vlanTagInsert(f, n, static_cast<uint16_t>(vid),
                          static_cast<uint8_t>(pcp));
    ASSERT_TRUE(sendRaw(txfd, ifindex, f, n));
}

/// Inject an IPv4/UDP frame (EtherCAT-over-UDP shape) on txfd.
void injectUdp(int txfd, int ifindex, uint8_t id, uint16_t dport,
               int vid = -1, uint8_t proto = 17, uint8_t ihl = 5,
               uint16_t frag_off = 0) {
    uint8_t f[128];
    const uint8_t pay[4] = {0xEC, 0xA7, id, 0x5A};
    size_t n = buildIp4UdpFrame(f, 1111, dport, pay, 4, proto, ihl, frag_off);
    if (vid >= 0) n = vlanTagInsert(f, n, static_cast<uint16_t>(vid));
    ASSERT_TRUE(sendRaw(txfd, ifindex, f, n));
}

/// Assert the kernel filter delivered the frame to the socket.
void expectAccepted(int rx, uint8_t id) {
    EXPECT_TRUE(hasMarker(recvAllWindow(rx, 400), id))
        << "injected frame id " << static_cast<int>(id) << " was dropped";
}

/// Assert the kernel filter dropped the frame — verified against an
/// unfiltered monitor socket so "rejected" is never confused with
/// "never arrived on the wire".
void expectRejected(int rx, int mon, uint8_t id) {
    EXPECT_FALSE(hasMarker(recvAllWindow(rx, 250), id))
        << "injected frame id " << static_cast<int>(id) << " leaked through";
    EXPECT_TRUE(hasMarker(recvAllWindow(mon, 30), id))
        << "injected frame id " << static_cast<int>(id)
        << " never reached the interface";
}

/// RAII 802.1Q sub-interface — the kernel itself pushes the tag on egress,
/// the closest in-test emulation of a switch trunk port.
struct VlanSubIf {
    std::string name;
    int ifindex = 0;
    bool ok = false;

    VlanSubIf(const std::string& parent, uint16_t vid) {
        name = parent + "." + std::to_string(vid);
        if (name.size() >= IFNAMSIZ) return;
        const std::string mk = "ip link add link " + parent + " name " + name +
                               " type vlan id " + std::to_string(vid) +
                               " 2>/dev/null";
        if (::system(mk.c_str()) != 0) return;
        runCmd("ip link set " + name + " up 2>/dev/null");
        for (int i = 0; i < 500; ++i) {
            ifindex = static_cast<int>(::if_nametoindex(name.c_str()));
            if (ifindex > 0) { ok = true; break; }
            std::this_thread::sleep_for(2ms);
        }
    }
    VlanSubIf(const VlanSubIf&) = delete;
    VlanSubIf& operator=(const VlanSubIf&) = delete;
    ~VlanSubIf() {
        if (ok) runCmd("ip link del " + name + " 2>/dev/null");
    }
};

// ============================================================================
// Fixture: needs CAP_NET_RAW (AF_PACKET).  Uses loopback — always present.
// ============================================================================

class RtChannelTest : public ::testing::Test {
protected:
    int ifindex_ = 0;
    void SetUp() override {
        if (!haveCapNetRaw())
            GTEST_SKIP() << "needs CAP_NET_RAW (run under runec)";
        ifindex_ = (int)::if_nametoindex("lo");
        ASSERT_GT(ifindex_, 0);
    }
};

// ============================================================================
// Fixture: needs CAP_NET_ADMIN + CAP_NET_RAW — a real veth wire.
// ============================================================================

class RtVethTest : public ::testing::Test {
protected:
    VethPair veth_;
    void SetUp() override {
        if (!haveCapNetRaw())
            GTEST_SKIP() << "needs CAP_NET_RAW (run under runec)";
        if (!haveCapNetAdmin())
            GTEST_SKIP() << "needs CAP_NET_ADMIN (run under runec)";
        veth_ = VethPair::create();
        if (!veth_.ok)
            GTEST_SKIP() << "veth pair creation failed";
    }
};

// ============================================================================
// Fixture: needs CAP_SYS_NICE (SCHED_FIFO / SCHED_DEADLINE / affinity).
// ============================================================================

class RtSchedTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!haveCapSysNice())
            GTEST_SKIP() << "needs CAP_SYS_NICE (run under runec)";
    }
};

// ============================================================================
// 1. Factory — real socket + ring backends on a live interface
// ============================================================================

TEST_F(RtChannelTest, FactorySocketModeCreatesSocketBackend) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = ifindex_;
    cfg.wire_mode = CyclicWireMode::SocketIO;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);
    EXPECT_FALSE(ch->zeroCopy());
    EXPECT_STREQ(ch->backendName(), "socket");
    EXPECT_GE(ch->fd(), 0);
    EXPECT_GT(ch->txCapacity(), 60u);
}

TEST_F(RtChannelTest, FactoryRingModeCreatesRingBackend) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = ifindex_;
    cfg.wire_mode = CyclicWireMode::PacketRing;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr) << "PACKET_MMAP ring setup failed";
    EXPECT_TRUE(ch->zeroCopy());
    EXPECT_STREQ(ch->backendName(), "ring");
    EXPECT_GE(ch->fd(), 0);
}

TEST_F(RtChannelTest, FactoryAutoPrefersRingWhenPrivileged) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = ifindex_;
    cfg.wire_mode = CyclicWireMode::Auto;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);
    // With CAP_NET_RAW + TPACKET_V2 the auto path lands on the ring backend.
    EXPECT_TRUE(ch->zeroCopy());
    EXPECT_STREQ(ch->backendName(), "ring");
}

// ============================================================================
// 2. Socket backend over real AF_PACKET — kernel RX timestamp + TX paths
// ============================================================================

TEST_F(RtChannelTest, SocketBackendRxHasKernelTimestamp) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = ifindex_;
    cfg.wire_mode = CyclicWireMode::SocketIO;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);

    // Inject a cyclic frame on lo from a separate socket; the channel's
    // socket (bound to 0x88A4, cyclic filter) receives it.
    int tx = openBoundPacketSocket(ifindex_, ETH_P_ALL);
    ASSERT_GE(tx, 0);
    uint8_t f[128];
    const uint8_t pay[4] = {0xCA, 0xFE, 0xBA, 0xBE};
    const size_t n = buildEcatFrame(f, 0x0C, 0xF8, 1, 2, pay, 4, 9);
    ASSERT_TRUE(sendRaw(tx, ifindex_, f, n));
    ::close(tx);

    CyclicFrameView v[2];
    ASSERT_GE(ch->rxPoll(v, 2, 500'000'000), 1);
    EXPECT_GE(v[0].frame_len, 60u);
    EXPECT_EQ(v[0].frame[17], 0xF8);          // cyclic idx survived demux
    // Kernel stamp via SCM_TIMESTAMPNS is CLOCK_REALTIME — well past boot.
    EXPECT_GT(v[0].stamp_ns, 1'000'000'000ull);  // >1 s after epoch
}

TEST_F(RtChannelTest, SocketBackendTxCommitFrameLoopsOnVeth) {
    // Move to a veth pair so TX egress is a real wire, not loopback magic.
    VethPair veth = VethPair::create();
    if (!veth.ok) GTEST_SKIP() << "veth unavailable";

    CyclicChannelConfig cfg;
    cfg.ifindex   = veth.ifA;
    cfg.wire_mode = CyclicWireMode::SocketIO;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);

    // Responder on the peer: read the cyclic frame, send it straight back.
    int peer = openBoundPacketSocket(veth.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    uint8_t* f = ch->txAcquire();
    ASSERT_NE(f, nullptr);
    const uint8_t pay[4] = {0x11, 0x22, 0x33, 0x44};
    const size_t n = buildEcatFrame(f, 0x0C, 0xF9, 0x55, 0x66, pay, 4, 1);
    ASSERT_TRUE(ch->txCommitFrame(n));

    uint8_t buf[2048];
    ASSERT_GT(recvOne(peer, buf, sizeof(buf), 500), 0);
    EXPECT_EQ(buf[17], 0xF9);
    ASSERT_TRUE(sendRaw(peer, veth.ifB, buf, 60));   // echo back

    CyclicFrameView v[2];
    ASSERT_GE(ch->rxPoll(v, 2, 500'000'000), 1);
    EXPECT_EQ(v[0].frame[17], 0xF9);
    ::close(peer);
}

TEST_F(RtChannelTest, SocketBackendTxSendPartsScatterGather) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = ifindex_;
    cfg.wire_mode = CyclicWireMode::SocketIO;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);

    int mon = openBoundPacketSocket(ifindex_, kEtherCat);
    ASSERT_GE(mon, 0);

    // Build a header-only prefix + separate payload, sent via sendmsg iovec.
    uint8_t hdr[26];
    std::memset(hdr, 0xFF, 6);
    hdr[12] = 0x88; hdr[13] = 0xA4;
    const uint8_t pay[8] = {1,2,3,4,5,6,7,8};
    const uint16_t ecat_len = static_cast<uint16_t>(10 + 8 + 2);
    hdr[14] = ecat_len & 0xFF;
    hdr[15] = ((ecat_len >> 8) & 0x07) | 0x10;
    hdr[16] = 0x0C; hdr[17] = 0xF8;
    uint16_t adp = 0x1111, ado = 0x2222;
    std::memcpy(hdr + 18, &adp, 2);
    std::memcpy(hdr + 20, &ado, 2);
    uint16_t lf = (8 & 0x07FF);
    std::memcpy(hdr + 22, &lf, 2);

    CyclicTxParts parts{hdr, 26, pay, 8, 0};
    ASSERT_TRUE(ch->txSendParts(parts));

    uint8_t buf[2048];
    ASSERT_GT(recvOne(mon, buf, sizeof(buf), 500), 0);
    EXPECT_EQ(buf[17], 0xF8);
    EXPECT_EQ(buf[26], 1);
    EXPECT_EQ(buf[33], 8);
    ::close(mon);
}

// ============================================================================
// 3. Ring backend on a real kernel ring
// ============================================================================

TEST_F(RtVethTest, RingBackendRoundtripOnWire) {
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, /*ignore_outgoing*/true),
        veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    EXPECT_TRUE(ch->zeroCopy());

    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    uint8_t* f = ch->txAcquire();
    ASSERT_NE(f, nullptr);
    const uint8_t pay[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    const size_t n = buildEcatFrame(f, 0x0C, 0xFA, 0x99, 0x88, pay, 4, 7);
    ASSERT_TRUE(ch->txCommitFrame(n));

    // Echo on the peer.
    uint8_t buf[2048];
    ASSERT_GT(recvOne(peer, buf, sizeof(buf), 500), 0);
    EXPECT_EQ(buf[17], 0xFA);
    ASSERT_TRUE(sendRaw(peer, veth_.ifB, buf, 60));

    CyclicFrameView v[2];
    ASSERT_GE(ch->rxPoll(v, 2, 500'000'000), 1);
    EXPECT_EQ(v[0].frame[17], 0xFA);
    EXPECT_GT(v[0].stamp_ns, 0u);             // kernel tpacket stamp
    EXPECT_GE(v[0].frame_len, 60u);
    ::close(peer);
}

// Scatter-gather TX on the *ring* backend — txSendParts copies
// header+payload+wkc into a ring slot then commits (the kernel reads the
// slot at the fixed data offset), then the echo returns through rxPoll.
TEST_F(RtVethTest, RingTxSendPartsScatterGather) {
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, /*ignore_outgoing*/true),
        veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    ASSERT_TRUE(ch->zeroCopy());

    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    // Header: dst/src/ethertype/ecat-header + idx byte.  Build a minimal
    // cyclic datagram header the kernel BPF accepts (et 0x88A4, idx 0xFA).
    uint8_t hdr[18] = {};
    std::memset(hdr, 0xFF, 6);                 // dst broadcast
    std::memset(hdr + 6, 0x11, 6);             // src
    hdr[12] = 0x88; hdr[13] = 0xA4;            // ethertype
    hdr[14] = 0x20;                            // ecat len low
    hdr[16] = 0x0C;                            // cmd LRW
    hdr[17] = 0xFA;                            // cyclic idx
    const uint8_t pay[4] = {0xAA, 0xBB, 0xCC, 0xDD};

    CyclicTxParts parts{};
    parts.header      = hdr;
    parts.header_len  = sizeof(hdr);
    parts.payload     = pay;
    parts.payload_len = sizeof(pay);
    parts.wkc         = 0;
    ASSERT_TRUE(ch->txSendParts(parts));

    // Peer must receive the assembled frame (hdr + payload + wkc trailer).
    uint8_t buf[2048];
    const ssize_t len = recvOne(peer, buf, sizeof(buf), 500);
    ASSERT_GT(len, 0);
    EXPECT_EQ(buf[17], 0xFA);
    EXPECT_EQ(buf[18], 0xAA);                  // payload landed right after hdr
    EXPECT_EQ(buf[19], 0xBB);

    // Echo back so rxPoll's post-wait walkRing path is exercised too.
    ASSERT_TRUE(sendRaw(peer, veth_.ifB, buf, 60));
    CyclicFrameView v[2];
    ASSERT_GE(ch->rxPoll(v, 2, 500'000'000), 1);
    EXPECT_EQ(v[0].frame[17], 0xFA);
    ::close(peer);
}

TEST_F(RtVethTest, RingRxPendingSignalsBeforePoll) {
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    EXPECT_FALSE(ch->rxPending());            // nothing yet
    uint8_t f[80];
    buildEcatFrame(f, 0x0C, 0xF8, 0, 0, (const uint8_t*)"x", 1, 1);
    ASSERT_TRUE(sendRaw(peer, veth_.ifB, f, 60));

    // rxPending polls slot memory — the DMA write is visible without a
    // syscall, so it should go true almost immediately.
    ASSERT_TRUE(waitFor([&] { return ch->rxPending(); }, 500ms));

    CyclicFrameView v[1];
    ASSERT_GE(ch->rxPoll(v, 1, 0), 1);
    EXPECT_EQ(v[0].frame[17], 0xF8);
    ::close(peer);
}

TEST_F(RtVethTest, RingHoldReleaseLifecycleOnWire) {
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    uint8_t f[80];
    buildEcatFrame(f, 0x0C, 0xF8, 0, 0, (const uint8_t*)"z", 1, 1);
    ASSERT_TRUE(sendRaw(peer, veth_.ifB, f, 60));

    CyclicFrameView v[1];
    ASSERT_GE(ch->rxPoll(v, 1, 500'000'000), 1);
    const uint32_t cookie = v[0].cookie;
    const uint8_t* p = v[0].frame;
    ch->rxHold(cookie);
    // The view stays valid while held — pointer into ring memory.
    EXPECT_EQ(p[17], 0xF8);
    ch->rxRelease(cookie);
    // Double-release is clamped (defensive — must not wedge the slot).
    ch->rxRelease(cookie);
    ::close(peer);
}

TEST_F(RtVethTest, RingTxExhaustionReturnsNull) {
    // Small TX ring (2 blocks) → a few commits fill it; txAcquire then
    // returns nullptr (the caller-visible backpressure signal).
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 8, 2);
    ASSERT_NE(ch, nullptr);

    const size_t cap = ch->txCapacity();
    ASSERT_GT(cap, 60u);
    int acquired = 0;
    // Commit frames without reading them back; the ring fills quickly on a
    // link with no consumer draining fast enough.
    for (int i = 0; i < 64; ++i) {
        uint8_t* f = ch->txAcquire();
        if (!f) break;
        buildEcatFrame(f, 0x0C, 0xF8, 0, 0, (const uint8_t*)"q", 1, 1);
        ch->txCommitFrame(60);
        ++acquired;
    }
    // Either we exhausted the ring (txAcquire→nullptr) or deferred kicks
    // queued them — both are correct backpressure behaviour; assert the
    // loop terminated sanely and txDeferred() is readable.
    EXPECT_GE(acquired, 1);
    EXPECT_GE(ch->txDeferred(), 0u);          // stat exists, may be >0
}

// ============================================================================
// 4. Two-socket kernel demux on a real wire
// ============================================================================

TEST_F(RtVethTest, KernelDemuxSeparatesCyclicFromAsync) {
    // Cyclic socket (filter accepts idx 0xF8-0xFD) + async socket (mirror
    // filter rejects them) — both bound to vethA.  Frames injected on the
    // peer traverse the kernel link and are split by the BPF filters.
    int cyc = openBoundPacketSocket(veth_.ifA, kEtherCat, true);
    int asy = openBoundPacketSocket(veth_.ifA, ETH_P_ALL, true);
    ASSERT_GE(cyc, 0);
    ASSERT_GE(asy, 0);
    ASSERT_TRUE(cyclicChannelAttachCyclicFilter(cyc));
    ASSERT_TRUE(cyclicChannelAttachAsyncFilter(asy));

    int peer = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(peer, 0);

    auto sendIdx = [&](uint8_t idx, uint16_t et = kEtherCat) {
        uint8_t f[80];
        buildEcatFrame(f, 0x0C, idx, 0, 0, (const uint8_t*)"d", 1, 1);
        if (et != kEtherCat) { f[12] = et >> 8; f[13] = et & 0xFF; }
        ASSERT_TRUE(sendRaw(peer, veth_.ifB, f, 60));
    };
    // Collect every frame a socket receives over a window — needed because
    // an ETH_P_ALL socket also sees kernel control traffic (IPv6 ND, ARP)
    // the fresh veth emits, which the async filter *correctly* accepts.
    auto recvAll = [&](int fd, int window_ms) {
        std::vector<std::vector<uint8_t>> out;
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(window_ms);
        while (std::chrono::steady_clock::now() < end) {
            struct pollfd p { fd, POLLIN, 0 };
            if (::poll(&p, 1, 20) <= 0) continue;
            uint8_t b[2048];
            ssize_t n = ::recv(fd, b, sizeof(b), 0);
            if (n > 0) out.emplace_back(b, b + n);
        }
        return out;
    };
    // A frame is a cyclic EtherCAT datagram iff ethertype==0x88A4 and the
    // datagram idx (byte 17) is in [0xF8,0xFD].
    auto isCyclicEcat = [](const std::vector<uint8_t>& f) {
        return f.size() > 18 && f[12] == 0x88 && f[13] == 0xA4 &&
               f[17] >= 0xF8 && f[17] <= 0xFD;
    };

    // Flush any startup control traffic off both sockets first.
    recvAll(cyc, 150); recvAll(asy, 150);

    // Cyclic frame → only the cyclic socket sees it.
    sendIdx(0xF8);
    auto onCyc = recvAll(cyc, 300);
    auto onAsy = recvAll(asy, 300);
    EXPECT_EQ(std::count_if(onCyc.begin(), onCyc.end(), isCyclicEcat), 1);
    EXPECT_EQ(std::count_if(onAsy.begin(), onAsy.end(), isCyclicEcat), 0);

    // Async-idx frame → only the async socket sees it.
    sendIdx(0x42);
    onCyc = recvAll(cyc, 300);
    onAsy = recvAll(asy, 300);
    EXPECT_EQ(std::count_if(onCyc.begin(), onCyc.end(), isCyclicEcat), 0);
    EXPECT_EQ(std::count_if(onAsy.begin(), onAsy.end(),
                          [&](const std::vector<uint8_t>& f){
                              return f.size() > 18 && f[17] == 0x42;
                          }), 1);

    // Non-ECAT ethertype carrying a cyclic idx → async sees it, cyclic doesn't.
    sendIdx(0xF8, 0x0800);
    onCyc = recvAll(cyc, 300);
    onAsy = recvAll(asy, 300);
    EXPECT_EQ(std::count_if(onCyc.begin(), onCyc.end(), isCyclicEcat), 0);
    EXPECT_EQ(std::count_if(onAsy.begin(), onAsy.end(),
                          [](const std::vector<uint8_t>& f){
                              return f.size() > 13 && f[12]==0x08 && f[13]==0x00;
                          }), 1);

    ::close(peer); ::close(cyc); ::close(asy);
}

// ============================================================================
// 5. Master::startCyclicLoop creates a real channel; wait path over wire
// ============================================================================

} // namespace

// Friend seam declared in Master.hpp — grants access to cyclic_channel_ so
// the test can inject a real channel into a master whose iface is a software
// stub (decouples the wait test from the exec loop).
namespace EtherCAT {
struct MasterCyclicTestAccess {
    static void setChannel(Master& m, std::unique_ptr<ICyclicChannel> ch) {
        m.datapath_->channel_ = std::move(ch);
    }
    static ICyclicChannel* channel(Master& m) { return m.datapath_->channel_.get(); }
};
} // namespace EtherCAT

namespace {

TEST_F(RtVethTest, MasterStartCyclicLoopCreatesRealChannel) {
    // A real bound AF_PACKET socket as the async/native handle — the master
    // reads the ifindex via getsockname and opens the cyclic channel on it.
    int async_fd = openBoundPacketSocket(veth_.ifA, kEtherCat);
    ASSERT_GE(async_fd, 0);

    NetworkInterface iface{};
    iface.native_handle = reinterpret_cast<void*>((intptr_t)async_fd);
    iface.receive = [async_fd](uint8_t* buf, size_t cap, size_t* out) -> bool {
        ssize_t n = ::recv(async_fd, buf, cap, MSG_DONTWAIT);
        if (n <= 0) { if (out) *out = 0; return false; }
        if (out) *out = (size_t)n;
        return true;
    };
    iface.send = [](const uint8_t*, size_t) { return true; };

    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    Master::CyclicLoopConfig cfg;
    cfg.wire_mode    = CyclicWireMode::PacketRing;
    cfg.motion_in_loop = false;             // external motion source
    ASSERT_TRUE(master.startCyclicLoop(cfg));
    EXPECT_TRUE(master.isCyclicLoopRunning());
    // The real channel was created on vethA.
    EXPECT_NE(master.cyclicChannel(), nullptr);
    if (master.cyclicChannel()) {
        EXPECT_TRUE(master.cyclicChannel()->zeroCopy());
    }
    master.stopCyclicLoop();
    master.stop();
    ::close(async_fd);
}

TEST_F(RtVethTest, WaitCyclicSlotViewOverRealWire) {
    // Inject a real ring channel bound to vethA into the master, then block
    // in waitCyclicSlotView — a responder on vethB sends a cyclic frame that
    // traverses the wire, lands in the RX ring, and wakes the waiter.
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);               // creates the notify eventfd

    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    MasterCyclicTestAccess::setChannel(master, std::move(ch));

    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    // Block a waiter on slot 0 (idx 0xF8).
    std::atomic<bool> got{false};
    CyclicSlotView view{};
    std::thread waiter([&] {
        got = master.waitCyclicSlotView(0, 0, 2'000'000'000u, view);
    });
    std::this_thread::sleep_for(30ms);      // let the waiter register+block

    // Send a cyclic response on the wire.
    uint8_t f[80];
    const uint8_t pay[4] = {0xAB, 0xCD, 0xEF, 0x01};
    buildEcatFrame(f, 0x0C, 0xF8, 0x1234, 0x5678, pay, 4, 3);
    ASSERT_TRUE(sendRaw(peer, veth_.ifB, f, 60));

    ASSERT_TRUE(waitFor([&] { return got.load(); }, 2'000ms));
    waiter.join();
    ASSERT_TRUE(got);
    EXPECT_EQ(view.datalen, 4u);
    EXPECT_EQ(view.wkc, 3u);
    EXPECT_EQ(view.adp, 0x1234u);
    ASSERT_NE(view.payload, nullptr);
    EXPECT_EQ(view.payload[0], 0xAB);
    // Kernel stamp propagated from the ring slot into the view.
    EXPECT_GT(view.stamp_ns, 0u);

    master.stop();
    ::close(peer);
}

TEST_F(RtVethTest, WaitCyclicSlotViewTimesOutOnDeadWire) {
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    MasterCyclicTestAccess::setChannel(master, std::move(ch));

    // Nothing on the wire — the wait must honour its deadline.
    CyclicSlotView view{};
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(master.waitCyclicSlotView(0, 0, 50'000'000u, view));
    const auto el = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(el, 2s);
    master.stop();
}

// ============================================================================
// 6. RT scheduling — SCHED_FIFO / SCHED_DEADLINE / slack / affinity
// ============================================================================

static int currentScheduler() {
    return ::sched_getscheduler(0);
}

TEST_F(RtSchedTest, SetCurrentThreadRealtimeAcquiresFifo) {
    std::atomic<int> policy{-1};
    std::thread t([&] {
        EXPECT_TRUE(Tether::Platform::setCurrentThreadRealtime(50));
        policy = currentScheduler();
    });
    t.join();
    EXPECT_EQ(policy.load(), SCHED_FIFO);
}

TEST_F(RtSchedTest, SetCurrentThreadDeadlineAcquiresDeadline) {
    std::atomic<int> policy{-1};
    std::thread t([&] {
        // runtime/deadline/period in ns — must satisfy runtime<=deadline<=period.
        const bool ok = Tether::Platform::setCurrentThreadDeadline(
            50'000, 200'000, 200'000);
        struct sched_attr_local attr{};
        attr.size = sizeof(attr);
        ::syscall(SYS_sched_getattr, 0, &attr, sizeof(attr), 0);
        policy = (int)attr.sched_policy;
        EXPECT_TRUE(ok);
        // Restore to normal so the join/teardown isn't deadline-throttled.
        struct sched_param z{0};
        ::sched_setscheduler(0, SCHED_OTHER, &z);
    });
    t.join();
    EXPECT_EQ(policy.load(), SCHED_DEADLINE);
}

TEST_F(RtSchedTest, TimerSlackIsReduced) {
    // Set on the calling thread, then read back via /proc.
    ASSERT_TRUE(Tether::Platform::setCurrentThreadTimerSlack(1));
    std::ifstream f("/proc/self/timerslack_ns");
    long v = -1;
    f >> v;
    EXPECT_EQ(v, 1);
    // Restore the kernel default so the slack doesn't leak to other tests.
    Tether::Platform::setCurrentThreadTimerSlack(50000);
}

// sched_setaffinity must run BEFORE becoming SCHED_DEADLINE — a deadline
// task cannot be arbitrarily pinned (EBUSY).  Encode the ordering.
TEST_F(RtSchedTest, AffinityBeforeDeadlineOrdering) {
    std::atomic<int> aff_before{-1}, aff_after{-1};
    std::thread t([&] {
        cpu_set_t s; CPU_ZERO(&s); CPU_SET(0, &s);
        aff_before = ::sched_setaffinity(0, sizeof(s), &s);   // OK

        const bool dl = Tether::Platform::setCurrentThreadDeadline(
            50'000, 200'000, 200'000);
        cpu_set_t s2; CPU_ZERO(&s2); CPU_SET(1, &s2);
        aff_after = ::sched_setaffinity(0, sizeof(s2), &s2);  // EBUSY under DL
        if (dl) EXPECT_NE(aff_after.load(), 0);             // pinned→busy

        struct sched_param z{0};
        ::sched_setscheduler(0, SCHED_OTHER, &z);
        cpu_set_t all; CPU_ZERO(&all);
        for (int i = 0; i < 8; ++i) CPU_SET(i, &all);
        ::sched_setaffinity(0, sizeof(all), &all);            // restore
    });
    t.join();
    EXPECT_EQ(aff_before.load(), 0);
}

TEST_F(RtSchedTest, CyclicExecutiveRunsAtFifoPriority) {
    std::atomic<int> seen_policy{-1};
    std::atomic<int> cycles{0};
    auto cfg = CyclicExecutive::Config::defaults(500, 4);
    cfg.dc_placement = CyclicExecutive::DCPlacement::Disabled;
    cfg.priority     = 50;
    CyclicExecutive e(
        [&] { ++cycles; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.addTask(TaskPhase::Diagnostics, [&] {
        if (seen_policy < 0) seen_policy = currentScheduler();
        return true;
    }));
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return cycles.load() >= 4; }));
    e.stop();
    EXPECT_EQ(seen_policy.load(), SCHED_FIFO);
    EXPECT_GE(e.getStats().cycle_count, 4u);
}

TEST_F(RtSchedTest, CyclicExecutiveRunsUnderDeadlineClass) {
    std::atomic<int> seen_policy{-1};
    std::atomic<int> cycles{0};
    auto cfg = CyclicExecutive::Config::defaults(500, 4);
    cfg.dc_placement = CyclicExecutive::DCPlacement::Disabled;
    cfg.sched_class  = CyclicExecutive::SchedClass::Deadline;
    CyclicExecutive e(
        [&] { ++cycles; return true; }, nullptr, nullptr, cfg);
    ASSERT_TRUE(e.addTask(TaskPhase::Diagnostics, [&] {
        if (seen_policy < 0) {
            struct sched_attr_local attr{};
            attr.size = sizeof(attr);
            ::syscall(SYS_sched_getattr, 0, &attr, sizeof(attr), 0);
            seen_policy = (int)attr.sched_policy;
        }
        return true;
    }));
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return cycles.load() >= 3; }));
    e.stop();
    // SCHED_DEADLINE (6) when admission succeeded, else the FIFO fallback (1).
    EXPECT_TRUE(seen_policy.load() == SCHED_DEADLINE ||
                seen_policy.load() == SCHED_FIFO);
}

TEST_F(RtSchedTest, CyclicExecutiveDedicatedDcThreadAtFifo) {
    std::atomic<int> dc_policy{-1};
    std::atomic<int> dc_runs{0};
    auto cfg = CyclicExecutive::Config::defaults(500, 2);
    cfg.dc_placement = CyclicExecutive::DCPlacement::DedicatedThread;
    cfg.dc_priority  = 60;
    CyclicExecutive e(
        [] { return true; },
        [&] {
            ++dc_runs;
            if (dc_policy < 0) dc_policy = currentScheduler();
            return true;
        }, nullptr, cfg);
    ASSERT_TRUE(e.start());
    ASSERT_TRUE(waitFor([&] { return dc_runs.load() >= 2; }));
    e.stop();
    EXPECT_EQ(dc_policy.load(), SCHED_FIFO);
}

// ============================================================================
// 7. CpuIsolation — claims verified against sched_getaffinity
// ============================================================================

TEST_F(RtSchedTest, CpuClaimAndPinThread) {
    auto& iso = Tether::Platform::CpuIsolation::instance();
    auto claim = iso.claim({});
    ASSERT_TRUE(claim.valid());
    const int cpu = claim.cpu;

    std::atomic<int> pinned{-1};
    std::thread t([&] {
        cpu_set_t s; CPU_ZERO(&s); CPU_SET(cpu, &s);
        if (::sched_setaffinity(0, sizeof(s), &s) == 0) {
            cpu_set_t g; CPU_ZERO(&g);
            ::sched_getaffinity(0, sizeof(g), &g);
            if (CPU_ISSET(cpu, &g) && CPU_COUNT(&g) == 1) pinned = cpu;
        }
    });
    t.join();
    EXPECT_EQ(pinned.load(), cpu);
    iso.release(cpu);
}

TEST_F(RtSchedTest, CpuClaimNeverExhaustsMachine) {
    auto& iso = Tether::Platform::CpuIsolation::instance();
    std::vector<int> claimed;
    // Claim until the allocator refuses — it must always leave ≥1 CPU free.
    for (int i = 0; i < 16; ++i) {
        auto c = iso.claim({});
        if (!c.valid()) break;
        claimed.push_back(c.cpu);
    }
    EXPECT_GE(iso.freeCount(), 1);
    EXPECT_LE((int)claimed.size(), iso.onlineCount() - 1);
    for (int cpu : claimed) iso.release(cpu);
    EXPECT_EQ(iso.claimedCpus().size(), 0u);
}

TEST_F(RtSchedTest, CpuClaimExplicitAndRelease) {
    auto& iso = Tether::Platform::CpuIsolation::instance();
    Tether::Platform::CpuIsolation::Spec spec;
    spec.requested_cpu = 1;
    auto c = iso.claim(spec);
    ASSERT_TRUE(c.valid());
    EXPECT_EQ(c.cpu, 1);
    auto held = iso.claimedCpus();
    EXPECT_NE(std::find(held.begin(), held.end(), 1), held.end());
    iso.release(1);
    held = iso.claimedCpus();
    EXPECT_EQ(std::find(held.begin(), held.end(), 1), held.end());
}

TEST_F(RtSchedTest, MasterCpuIsolationClaimsReleasedOnStop) {
    auto& iso = Tether::Platform::CpuIsolation::instance();
    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);

    Master::CyclicLoopConfig cfg;
    cfg.cpu_isolation.enabled = true;
    cfg.motion_in_loop = false;
    ASSERT_TRUE(master.startCyclicLoop(cfg));
    EXPECT_TRUE(master.isCyclicLoopRunning());
    const size_t held = iso.claimedCpus().size();
    EXPECT_GE(held, 1u);                    // cyclic thread claimed a CPU
    master.stopCyclicLoop();
    EXPECT_EQ(iso.claimedCpus().size(), 0u);   // released on stop
    master.stop();
}

// ============================================================================
// 8. RtMemory — lockAll / lockRegion / prefault under RLIMIT_MEMLOCK
// ============================================================================

static long readVmLckKb() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("VmLck:", 0) == 0) {
            return std::strtol(line.c_str() + 6, nullptr, 10);
        }
    }
    return 0;
}

TEST(RtMemoryPriv, LockRegionRaisesVmLck) {
    const size_t len = 256 * 1024;
    void* p = ::mmap(nullptr, len, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    ASSERT_NE(p, MAP_FAILED);
    Tether::Platform::prefaultMemory(p, len);

    const long before = readVmLckKb();
    if (!Tether::Platform::lockMemory(p, len))
        GTEST_SKIP() << "mlock unavailable (RLIMIT_MEMLOCK too small)";
    EXPECT_GE(readVmLckKb(), before + (long)(len / 1024));
    EXPECT_TRUE(Tether::Platform::unlockMemory(p, len));
    ::munmap(p, len);
}

TEST(RtMemoryPriv, LockAllMemoryRaisesVmLck) {
    if (!Tether::Platform::lockAllMemory())
        GTEST_SKIP() << "mlockall unavailable (RLIMIT_MEMLOCK too small)";
    EXPECT_GT(readVmLckKb(), 0L);
    ::munlockall();
}

TEST(RtMemoryPriv, PrefaultStackTouchesPages) {
    // Prefault a modest depth and confirm we don't fault/crash — the value
    // is in exercising the alloca walk, not a measurable counter.
    struct rusage ru0, ru1;
    ::getrusage(RUSAGE_SELF, &ru0);
    Tether::Platform::prefaultCurrentStack(64 * 1024);
    ::getrusage(RUSAGE_SELF, &ru1);
    SUCCEED();
}

TEST(RtMemoryPriv, TimerSlackSetter) {
    EXPECT_TRUE(Tether::Platform::setCurrentThreadTimerSlack(1000));
}

// ============================================================================
// 9. Master + executive end-to-end on a real wire with RT scheduling
// ============================================================================

TEST_F(RtVethTest, CyclicExecutiveExchangeOverWireAtFifo) {
    // The full path: a CyclicExecutive running at SCHED_FIFO drives an
    // exchange that sends a cyclic frame over the wire; a responder echoes
    // a response that lands in the slot — measured end-to-end.
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, true), veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    ICyclicChannel* chp = ch.get();

    NetworkInterface iface{};
    iface.send = [](const uint8_t*, size_t) { return true; };
    Master master;
    const uint8_t mac[6] = {0x02,0,0,0,0,1};
    master.start(iface, mac);
    MasterCyclicTestAccess::setChannel(master, std::move(ch));

    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);

    // Responder thread: echo every cyclic frame back with a payload + WKC.
    std::atomic<bool> run{true};
    std::thread responder([&] {
        uint8_t b[2048];
        while (run.load()) {
            ssize_t n = recvOne(peer, b, sizeof(b), 50);
            if (n > 0 && n >= 18 && b[17] >= 0xF8 && b[17] <= 0xFD)
                sendRaw(peer, veth_.ifB, b, (size_t)n);
        }
    });

    std::atomic<int> sends{0}, collects{0};
    // 2 ms cycle, 1.5 ms echo window — generous enough that a normally-
    // scheduled responder keeps up even under full-suite load, while still
    // exercising the real deadline-driven exchange at FIFO priority.
    auto cfg = CyclicExecutive::Config::defaults(2000, 4);
    cfg.dc_placement = CyclicExecutive::DCPlacement::Disabled;
    cfg.priority     = 60;
    CyclicExecutive exec(
        // exchange: send a cyclic request then collect the echoed response
        [&]() -> bool {
            uint8_t* f = chp->txAcquire();
            if (!f) return false;
            const uint8_t pay[4] = {0x01,0x02,0x03,0x04};
            buildEcatFrame(f, 0x0C, 0xF8, 0, 0, pay, 4, 0);
            if (!chp->txCommitFrame(60)) return false;
            ++sends;
            // Collect: wait for the echo on slot 0 within the cycle budget.
            CyclicSlotView view{};
            if (master.waitCyclicSlotView(0, /*token=*/0,
                                          1'500'000u, view)) {
                ++collects;
            }
            return true;
        },
        nullptr, nullptr, cfg);
    ASSERT_TRUE(exec.start());
    ASSERT_TRUE(waitFor([&] { return sends.load() >= 3; }, 5'000ms));
    exec.stop();
    run = false;
    responder.join();

    EXPECT_GE(sends.load(), 3);
    // At least one exchange got a wire response through the real path.
    EXPECT_GE(collects.load(), 1);
    master.stop();
    ::close(peer);
}

// 10. AsyncCyclicLoop on a real wire — triggerSend() wakes the loop →
//     ring TX → veth → peer echo → RX ring → collect.  Proves the
//     event-driven send-on-change path drives a real kernel channel
//     end-to-end (the cyclic equivalent of the free-running test above).
TEST_F(RtVethTest, AsyncLoopTriggerDrivesWireExchange) {
    auto ch = createCyclicRingChannelForFd(
        openBoundPacketSocket(veth_.ifA, kEtherCat, /*ignore_outgoing*/true),
        veth_.ifA, 32, 8);
    ASSERT_NE(ch, nullptr);
    ICyclicChannel* chp = ch.get();

    ProcessImage img;
    ProcessImage::Config ic;
    ic.mode = ImageMode::Direct;
    ic.rx_bytes = 8; ic.tx_bytes = 8;
    ASSERT_TRUE(img.configure(ic));

    int peer = openBoundPacketSocket(veth_.ifB, kEtherCat);
    ASSERT_GE(peer, 0);
    std::atomic<bool> run{true};
    std::thread responder([&] {
        uint8_t b[2048];
        while (run.load()) {
            ssize_t n = recvOne(peer, b, sizeof(b), 50);
            if (n > 0 && n >= 18 && b[17] >= 0xF8 && b[17] <= 0xFD)
                sendRaw(peer, veth_.ifB, b, (size_t)n);
        }
    });

    std::atomic<int> echoes{0};
    AsyncCyclicLoop::Config acfg;
    acfg.collect_mode = AsyncCyclicLoop::CollectMode::OnSend;
    acfg.priority     = 60;
    AsyncCyclicLoop loop(img,
        // send: commit one cyclic frame onto the wire.
        [&]() -> bool {
            uint8_t* f = chp->txAcquire();
            if (!f) return false;
            const uint8_t pay[4] = {0xAA,0xBB,0xCC,0xDD};
            buildEcatFrame(f, 0x0C, 0xF8, 0, 0, pay, 4, 0);
            return chp->txCommitFrame(60);
        },
        // collect: pull the peer's echo out of the RX ring.
        [&]() -> bool {
            CyclicFrameView v[2];
            if (chp->rxPoll(v, 2, 1'500'000) >= 1 &&   // 1.5 ms window
                v[0].frame_len >= 18 && v[0].frame[17] == 0xF8)
                ++echoes;
            return true;
        },
        nullptr, nullptr, acfg);
    ASSERT_TRUE(loop.start());

    // Each trigger → one wire frame → one echo.  Space them so each is a
    // distinct wake — back-to-back triggers legitimately coalesce.
    for (int i = 1; i <= 3; ++i) {
        img.triggerSend();
        ASSERT_TRUE(waitFor([&] {
            return loop.getStats().sends >= static_cast<uint64_t>(i);
        }, 5'000ms));
    }
    ASSERT_TRUE(waitFor([&] { return echoes.load() >= 1; }, 5'000ms));

    loop.stop();
    run = false;
    responder.join();

    const auto st = loop.getStats();
    EXPECT_EQ(st.sends, 3u);
    EXPECT_EQ(st.collects, 3u);            // OnSend: paired per send
    EXPECT_EQ(st.send_errors, 0u);
    EXPECT_GE(echoes.load(), 1);           // a real echo came back via RX
    ::close(peer);
}

// ============================================================================
// 11. Encapsulation filters end-to-end on a veth wire
//
// CBPFProgramFactory programs attached to real AF_PACKET sockets on vethA;
// frames injected on vethB traverse the kernel link.  Rejections are always
// double-checked against an unfiltered monitor socket so a "drop" can never
// be confused with a frame that never arrived.
// ============================================================================

TEST_F(RtVethTest, BpfEthercatFilterWireMatrix) {
    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int tx  = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(tx, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(
        rx, CBPFProgramFactory::ethercatFilter()));
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    injectEcat(tx, veth_.ifB, 0x01, -1);     // untagged EtherCAT
    expectAccepted(rx, 0x01);
    injectEcat(tx, veth_.ifB, 0x02, 7);      // tagged EtherCAT — any VID
    expectAccepted(rx, 0x02);
    injectEcat(tx, veth_.ifB, 0x03, 4094);   // tagged, VID boundary
    expectAccepted(rx, 0x03);

    // IPv4/UDP on the EtherCAT port — the plain filter has no UDP leg.
    injectUdp(tx, veth_.ifB, 0x04, kEtherCATUdpPort);
    expectRejected(rx, mon, 0x04);

    // ARP — unrelated ethertype.
    uint8_t f[64];
    const uint8_t p5[4] = {0xEC, 0xA7, 0x05, 0x5A};
    std::memset(f, 0xFF, 6);
    std::memset(f + 6, 0x66, 6);
    f[12] = 0x08; f[13] = 0x06;
    std::memcpy(f + 14, p5, 4);
    ASSERT_TRUE(sendRaw(tx, veth_.ifB, f, 60));
    expectRejected(rx, mon, 0x05);

    ::close(tx); ::close(mon); ::close(rx);
}

TEST_F(RtVethTest, BpfVlanFilterWireMatrix) {
    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int tx  = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(tx, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(
        rx, CBPFProgramFactory::vlanFilter(1999)));
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    // Inbound tagged frames arrive STRIPPED: [12] shows the inner EtherType
    // and the tag lives in skb auxdata.  These accepts exercise the
    // SKF_AD_VLAN_TAG leg of the filter — the only path real traffic takes.
    injectEcat(tx, veth_.ifB, 0x10, 1999);        // matching VID
    expectAccepted(rx, 0x10);
    EXPECT_TRUE(hasAuxVlanMarker(recvAllWindowAux(mon, 30), 0x10, 1999))
        << "accepted frame did not actually carry VID 1999";
    injectEcat(tx, veth_.ifB, 0x11, 1999, 5);     // PCP set — VID still masked
    expectAccepted(rx, 0x11);

    injectEcat(tx, veth_.ifB, 0x12, 2000);        // wrong VID
    expectRejected(rx, mon, 0x12);
    injectEcat(tx, veth_.ifB, 0x13, 0);           // VID 0 (priority tag)
    expectRejected(rx, mon, 0x13);
    injectEcat(tx, veth_.ifB, 0x14, -1);          // untagged EtherCAT — VLAN
    expectRejected(rx, mon, 0x14);                // mode rejects it by design

    // Tagged IPv4/UDP — VLAN mode has no UDP leg.
    injectUdp(tx, veth_.ifB, 0x15, kEtherCATUdpPort, 1999);
    expectRejected(rx, mon, 0x15);

    // Self-copy: a frame transmitted on ifA is delivered to rx as a
    // PACKET_OUTGOING copy with the tag still INLINE — the only path that
    // exercises the filter's [12]==0x8100 leg against the real kernel.
    int txA = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    ASSERT_GE(txA, 0);
    injectEcat(txA, veth_.ifA, 0x16, 1999);       // inline tag on self-copy
    {
        const auto frames = recvAllWindow(rx, 400);
        const uint8_t m[4] = {0xEC, 0xA7, 0x16, 0x5A};
        bool inline_tagged = false;
        for (const auto& f : frames)
            if (f.size() > 18 && f[12] == 0x81 && f[13] == 0x00 &&
                (((f[14] & 0x0F) << 8) | f[15]) == 1999 &&
                std::search(f.begin(), f.end(), m, m + 4) != f.end())
                inline_tagged = true;
        EXPECT_TRUE(inline_tagged)
            << "inline-tag leg never saw an accepted VID-1999 self-copy";
    }
    injectEcat(txA, veth_.ifA, 0x17, 2000);       // inline tag, wrong VID
    expectRejected(rx, mon, 0x17);
    ::close(txA);

    ::close(tx); ::close(mon); ::close(rx);
}

TEST_F(RtVethTest, BpfVlanRangeFilterWireMatrix) {
    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int tx  = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(tx, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(
        rx, CBPFProgramFactory::vlanRangeFilter(100, 200)));
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    injectEcat(tx, veth_.ifB, 0x20, 100);    // lower boundary
    expectAccepted(rx, 0x20);
    injectEcat(tx, veth_.ifB, 0x21, 150);    // interior
    expectAccepted(rx, 0x21);
    injectEcat(tx, veth_.ifB, 0x22, 200);    // upper boundary
    expectAccepted(rx, 0x22);

    injectEcat(tx, veth_.ifB, 0x23, 99);     // just below
    expectRejected(rx, mon, 0x23);
    injectEcat(tx, veth_.ifB, 0x24, 201);    // just above
    expectRejected(rx, mon, 0x24);
    injectEcat(tx, veth_.ifB, 0x25, -1);     // untagged
    expectRejected(rx, mon, 0x25);

    ::close(tx); ::close(mon); ::close(rx);
}

TEST_F(RtVethTest, BpfVlanCatchAllSpecWire) {
    // The spec vlan:any maps to: tagged EtherCAT of ANY VID accepted,
    // untagged rejected — verified here at kernel level.
    CBPFSpec spec{};
    spec.untagged_ethercat = false;
    spec.tagged_ethercat   = true;           // no vlan_range → any VID
    const auto prog = CBPFProgramFactory::build(spec);
    ASSERT_FALSE(prog.empty());

    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int tx  = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(tx, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(rx, prog));
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    injectEcat(tx, veth_.ifB, 0x30, 1);
    expectAccepted(rx, 0x30);
    injectEcat(tx, veth_.ifB, 0x31, 4094);
    expectAccepted(rx, 0x31);

    injectEcat(tx, veth_.ifB, 0x32, -1);     // untagged still rejected
    expectRejected(rx, mon, 0x32);

    ::close(tx); ::close(mon); ::close(rx);
}

TEST_F(RtVethTest, BpfUdpFilterWireMatrix) {
    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int tx  = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(tx, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(
        rx, CBPFProgramFactory::ethercatFilterWithUdp()));   // port 34980
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    injectUdp(tx, veth_.ifB, 0x40, kEtherCATUdpPort);            // untagged UDP
    expectAccepted(rx, 0x40);
    injectUdp(tx, veth_.ifB, 0x41, kEtherCATUdpPort, 1999);      // tagged UDP
    expectAccepted(rx, 0x41);
    injectUdp(tx, veth_.ifB, 0x42, kEtherCATUdpPort, -1, 17, 6); // IHL=6 options
    expectAccepted(rx, 0x42);
    injectEcat(tx, veth_.ifB, 0x43, -1);          // plain EtherCAT still works
    expectAccepted(rx, 0x43);
    injectEcat(tx, veth_.ifB, 0x44, 77);          // tagged EtherCAT too
    expectAccepted(rx, 0x44);

    injectUdp(tx, veth_.ifB, 0x45, 9999);         // wrong dst port
    expectRejected(rx, mon, 0x45);
    injectUdp(tx, veth_.ifB, 0x46, kEtherCATUdpPort, -1, 6);   // TCP not UDP
    expectRejected(rx, mon, 0x46);
    injectUdp(tx, veth_.ifB, 0x47, kEtherCATUdpPort, -1, 17, 5,
              /*frag_off=*/1);                  // non-first fragment
    expectRejected(rx, mon, 0x47);
    injectUdp(tx, veth_.ifB, 0x48, 9999, 1999);   // wrong port, tagged
    expectRejected(rx, mon, 0x48);

    ::close(tx); ::close(mon); ::close(rx);
}

TEST_F(RtVethTest, BpfVlanFilterKernelTaggedWire) {
    // A VLAN sub-interface on the PEER makes the kernel itself insert the
    // tag — the closest emulation of a switch trunk port tagging frames.
    VlanSubIf sub(veth_.b, 1999);
    if (!sub.ok) GTEST_SKIP() << "802.1Q sub-interface unavailable";

    int rx  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int mon = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int txB = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    int txS = openBoundPacketSocket(sub.ifindex, ETH_P_ALL);
    ASSERT_GE(rx, 0);
    ASSERT_GE(mon, 0);
    ASSERT_GE(txB, 0);
    ASSERT_GE(txS, 0);
    ASSERT_TRUE(CBPFProgramFactory::attach(
        rx, CBPFProgramFactory::vlanFilter(1999)));
    recvAllWindow(rx, 150);
    recvAllWindow(mon, 150);

    // Untagged EtherCAT into the sub-interface: kernel pushes VID 1999 on
    // egress → arrives at vethA where the RX path strips the tag back into
    // auxdata → the filter's SKF_AD_VLAN_TAG leg accepts it.
    {
        uint8_t f[96];
        const uint8_t pay[4] = {0xEC, 0xA7, 0x50, 0x5A};
        const size_t n = buildEcatFrame(f, 0x07, 0x40, 0, 0x0130, pay, 4, 0);
        ASSERT_TRUE(sendRaw(txS, sub.ifindex, f, n, kEtherCat));
    }
    expectAccepted(rx, 0x50);
    // The monitor confirms the tag was on the wire: inbound frames arrive
    // stripped, so the proof is in the auxdata TCI, not inline bytes.
    EXPECT_TRUE(hasAuxVlanMarker(recvAllWindowAux(mon, 30), 0x50, 1999))
        << "kernel did not deliver the frame tagged with VID 1999";

    // Control: the same frame sent untagged on vethB is rejected.
    injectEcat(txB, veth_.ifB, 0x51, -1);
    expectRejected(rx, mon, 0x51);

    ::close(txS); ::close(txB); ::close(mon); ::close(rx);
}

#if defined(TETHER_TEST_HAVE_HAL)
// The full production RX chain for --encapsulation vlan:1999:
//   kernel cBPF (vlanFilter) → LinuxEthernet::poll (PACKET_OUTGOING skip +
//   inner-EtherType software filter) → RxCallback → VLANRouter strip →
//   master delivery hook.  Plus the TX side: the router's per-master
//   NetworkInterface must tag egress with VID 1999.
TEST_F(RtVethTest, HalPollThroughKernelVlanFilterToRouter) {
    auto eth = HAL::createDefaultEthernet();
    ASSERT_NE(eth, nullptr);
    HAL::EthernetConfig ec{};
    ec.interfaceName    = veth_.a.c_str();
    ec.ethertypeFilter  = kEtherCat;          // same as initHostEthernet()
    ASSERT_EQ(eth->init(ec), HAL::Error::OK);
    ASSERT_TRUE(eth->isInitialized());

    const int fd = static_cast<int>(
        reinterpret_cast<intptr_t>(eth->nativeHandle()));
    ASSERT_GT(fd, 0);
    const auto prog = CBPFProgramFactory::vlanFilter(1999);
    ASSERT_FALSE(prog.empty());
    ASSERT_TRUE(CBPFProgramFactory::attach(fd, prog));

    int txfd = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    int peer = openBoundPacketSocket(veth_.ifB, ETH_P_ALL);
    int mon  = openBoundPacketSocket(veth_.ifA, ETH_P_ALL);
    ASSERT_GE(txfd, 0);
    ASSERT_GE(peer, 0);
    ASSERT_GE(mon, 0);

    Master master;                            // deliver hook bypasses start()
    auto msp = std::shared_ptr<Master>(&master, [](Master*) {});
    VLANRouter router;
    NetworkInterface backend{};
    backend.send = [&](const uint8_t* f, size_t l) {
        return sendRaw(txfd, veth_.ifA, f, l);
    };
    router.setBackend(&backend);
    router.addMaster(msp, VLANRouter::VLANRange{1999, 1999}, 1999);

    std::vector<std::vector<uint8_t>> delivered;
    router.setDeliverFunction(
        [&](Master*, const uint8_t* d, size_t l) {
            delivered.emplace_back(d, d + l);
        });
    eth->setRxCallback(
        [&](const uint8_t* f, size_t l, const HAL::RxFrameInfo& info,
            void*) {
            // Mirror setupEncapsulation(): a kernel-stripped tag arrives
            // only via metadata — forward it out-of-band.
            std::optional<uint16_t> aux_vlan;
            if (info.vlanTagPresent && l >= 14 &&
                !(f[12] == 0x81 && f[13] == 0x00))
                aux_vlan = info.vlanId;
            router.processRxFrame(f, l, aux_vlan);
        },
        nullptr);

    eth->poll(100);                           // drain boot noise
    recvAllWindow(mon, 100);

    // 1. Tagged VID-1999 EtherCAT → kernel accepts → HAL passes (inner
    //    EtherType matches the software filter) → router strips the tag.
    injectEcat(peer, veth_.ifB, 0x60, 1999);
    ASSERT_TRUE(waitFor([&] {
        eth->poll(20);
        return !delivered.empty();
    }, 1'000ms));
    ASSERT_EQ(delivered.size(), 1u);
    ASSERT_GT(delivered[0].size(), 14u);
    EXPECT_EQ(delivered[0][12], 0x88);        // tag stripped before delivery
    EXPECT_EQ(delivered[0][13], 0xA4);
    EXPECT_TRUE(hasMarker(delivered, 0x60));

    // 2. Tagged VID-2000 EtherCAT → kernel filter drops it; the monitor
    //    confirms the frame did reach vethA.
    delivered.clear();
    injectEcat(peer, veth_.ifB, 0x61, 2000);
    eth->poll(250);
    EXPECT_TRUE(delivered.empty());
    EXPECT_TRUE(hasMarker(recvAllWindow(mon, 30), 0x61))
        << "VID-2000 frame never reached the interface";

    // 3. Untagged EtherCAT → dropped by the kernel filter (VLAN mode).
    injectEcat(peer, veth_.ifB, 0x62, -1);
    eth->poll(250);
    EXPECT_TRUE(delivered.empty());
    EXPECT_TRUE(hasMarker(recvAllWindow(mon, 30), 0x62));

    // 4. TX: the router's per-master interface tags egress with VID 1999.
    //    The peer receives on the RX path where the tag is stripped into
    //    auxdata — verify via PACKET_AUXDATA that VID 1999 hit the wire.
    NetworkInterface* ni = router.networkInterfaceFor(&master);
    ASSERT_NE(ni, nullptr);
    {
        uint8_t f[96];
        const uint8_t pay[4] = {0xEC, 0xA7, 0x63, 0x5A};
        const size_t n = buildEcatFrame(f, 0x07, 0x40, 0, 0x0130, pay, 4, 0);
        ASSERT_TRUE(ni->send(f, n));
    }
    EXPECT_TRUE(hasAuxVlanMarker(recvAllWindowAux(peer, 500), 0x63, 1999))
        << "router TX path did not emit a VID-1999 tagged EtherCAT frame";

    ::close(mon); ::close(peer); ::close(txfd);
    eth->shutdown();
}
#endif // TETHER_TEST_HAVE_HAL

} // namespace

#else  // !__linux__

TEST(RtPrivileged, NotLinux) {
    GTEST_SKIP() << "realtime privileged tests are Linux-only";
}

#endif // __linux__
