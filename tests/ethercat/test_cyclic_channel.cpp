/**
 * @file test_cyclic_channel.cpp
 * @brief Unit + integration tests for the cyclic wire channel.
 *
 * Coverage:
 *  - ICyclicChannel default txSendParts implementation (iovec staging).
 *  - cyclicChannelBpfProgram: exported instruction words, both filters.
 *  - Userspace cBPF interpreter over the *production* program bytes —
 *    exhaustive truth-table incl. boundary indexes and short frames.
 *  - Real kernel BPF execution: SO_ATTACH_FILTER on an AF_UNIX
 *    SOCK_DGRAM socketpair (unprivileged) — proves the exact program
 *    bytes are accepted by and behave correctly in the kernel's BPF
 *    interpreter.
 *  - LinuxSocketChannel over a caller-provided fd (unix dgram pair):
 *    rxPoll / rxHold / rxRelease / bank exhaustion / drop counter.
 *  - Live AF_PACKET tests on the loopback interface (skipped without
 *    CAP_NET_RAW): two-socket BPF demux + ring channel RX/TX.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Types.hpp"

#ifdef __linux__
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/filter.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

using namespace EtherCAT;

namespace {

#ifdef __linux__

// ============================================================================
// Userspace cBPF interpreter — the *exact* instruction subset our programs
// use.  Runs the production bytes, not a reimplementation of the policy.
// Kernel semantics: an out-of-bounds BPF_ABS load rejects the packet
// (filter returns 0 for the whole program).
// ============================================================================

uint32_t runBpf(const sock_filter* prog, size_t n,
                const uint8_t* pkt, size_t len)
{
    uint32_t a = 0;
    size_t pc = 0;
    while (pc < n) {
        const sock_filter& in = prog[pc];
        const uint16_t cls = in.code & 0x07;
        switch (cls) {
        case BPF_LD: {
            const uint32_t k = in.k;
            const uint16_t sz = in.code & 0x18;
            const size_t need = (sz == BPF_B) ? 1 : (sz == BPF_H) ? 2 : 4;
            if ((in.code & 0xe0) != BPF_ABS || k + need > len)
                return 0;                       // OOB → kernel drops packet
            if (sz == BPF_B)      a = pkt[k];
            else if (sz == BPF_H) a = (pkt[k] << 8) | pkt[k + 1];
            else                  a = (pkt[k] << 24) | (pkt[k+1] << 8)
                                    | (pkt[k+2] << 16) | pkt[k+3];
            ++pc;
            break;
        }
        case BPF_JMP: {
            const uint16_t op = in.code & 0xf0;
            bool taken = false;
            if (op == BPF_JEQ)      taken = (a == in.k);
            else if (op == BPF_JGE) taken = (a >= in.k);
            else if (op == BPF_JGT) taken = (a >  in.k);
            else if (op == BPF_JA)  taken = true;
            pc += taken ? (in.jt + 1) : (in.jf + 1);
            break;
        }
        case BPF_RET:
            return in.k;
        default:
            ADD_FAILURE() << "unsupported insn class " << cls;
            return 0;
        }
    }
    return 0;
}

/// Minimal EtherCAT frame: [dst 6][src 6][0x88A4][ecat hdr][cmd][idx]...
std::vector<uint8_t> makeEcatFrame(uint8_t idx, uint16_t ethertype = 0x88A4,
                                   size_t len = 60)
{
    std::vector<uint8_t> f(len, 0);
    if (len > 13) { f[12] = ethertype >> 8; f[13] = ethertype & 0xFF; }
    if (len > 14) f[14] = 0x10;
    if (len > 16) f[16] = 0x0C;             // cmd = LRW (arbitrary)
    if (len > 17) f[17] = idx;
    return f;
}

#endif // __linux__

} // anonymous namespace

#ifdef __linux__

// ============================================================================
// BPF program structure
// ============================================================================

TEST(CyclicBpf, ProgramShape) {
    CyclicBpfInsn prog[8];
    ASSERT_EQ(cyclicChannelBpfProgram(true, prog, 8), kCyclicBpfInsnCount);
    ASSERT_EQ(cyclicChannelBpfProgram(false, prog, 8), kCyclicBpfInsnCount);
    // Too-small buffer must return 0 without writing.
    EXPECT_EQ(cyclicChannelBpfProgram(true, prog, 3), 0u);
    EXPECT_EQ(cyclicChannelBpfProgram(true, nullptr, 0), 0u);
}

TEST(CyclicBpf, InsnLayoutMatchesSockFilter) {
    static_assert(sizeof(CyclicBpfInsn) == sizeof(sock_filter));
    EXPECT_EQ(offsetof(CyclicBpfInsn, code), offsetof(sock_filter, code));
    EXPECT_EQ(offsetof(CyclicBpfInsn, jt),   offsetof(sock_filter, jt));
    EXPECT_EQ(offsetof(CyclicBpfInsn, jf),   offsetof(sock_filter, jf));
    EXPECT_EQ(offsetof(CyclicBpfInsn, k),    offsetof(sock_filter, k));
}

TEST(CyclicBpf, CyclicFilterAcceptsOnlyCyclicIdx) {
    CyclicBpfInsn prog[8];
    ASSERT_EQ(cyclicChannelBpfProgram(true, prog, 8), kCyclicBpfInsnCount);
    const auto* f = reinterpret_cast<const sock_filter*>(prog);

    for (uint8_t idx : {0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD}) {
        auto frame = makeEcatFrame(idx);
        EXPECT_NE(runBpf(f, 7, frame.data(), frame.size()), 0u)
            << "idx 0x" << std::hex << (int)idx;
    }
    for (uint8_t idx : {0x00, 0x01, 0x7F, 0xF7, 0xFE, 0xFF}) {
        auto frame = makeEcatFrame(idx);
        EXPECT_EQ(runBpf(f, 7, frame.data(), frame.size()), 0u)
            << "idx 0x" << std::hex << (int)idx;
    }
}

TEST(CyclicBpf, AsyncFilterRejectsOnlyCyclicIdx) {
    CyclicBpfInsn prog[8];
    ASSERT_EQ(cyclicChannelBpfProgram(false, prog, 8), kCyclicBpfInsnCount);
    const auto* f = reinterpret_cast<const sock_filter*>(prog);

    for (uint8_t idx : {0xF8, 0xFA, 0xFD}) {
        auto frame = makeEcatFrame(idx);
        EXPECT_EQ(runBpf(f, 7, frame.data(), frame.size()), 0u);
    }
    for (uint8_t idx : {0x00, 0x42, 0xF7, 0xFE, 0xFF}) {
        auto frame = makeEcatFrame(idx);
        EXPECT_NE(runBpf(f, 7, frame.data(), frame.size()), 0u);
    }
}

TEST(CyclicBpf, NonEtherCatEtherTypes) {
    CyclicBpfInsn progA[8], progB[8];
    cyclicChannelBpfProgram(true, progA, 8);
    const auto* cyc = reinterpret_cast<const sock_filter*>(progA);
    cyclicChannelBpfProgram(false, progB, 8);
    const auto* asy = reinterpret_cast<const sock_filter*>(progB);

    for (uint16_t et : {0x0800, 0x8100, 0x0001, 0xFFFF}) {
        auto frame = makeEcatFrame(0xF8, et);
        EXPECT_EQ(runBpf(cyc, 7, frame.data(), frame.size()), 0u)
            << "et 0x" << std::hex << et;
        EXPECT_NE(runBpf(asy, 7, frame.data(), frame.size()), 0u)
            << "et 0x" << std::hex << et;
    }
}

TEST(CyclicBpf, ShortFramesRejectedOnBoth) {
    // Kernel semantics: an out-of-bounds ABS load fails the whole filter →
    // the packet is rejected for that socket.  Malformed <18-byte frames
    // therefore land on NEITHER socket.
    CyclicBpfInsn progA[8], progB[8];
    cyclicChannelBpfProgram(true, progA, 8);
    const auto* cyc = reinterpret_cast<const sock_filter*>(progA);
    cyclicChannelBpfProgram(false, progB, 8);
    const auto* asy = reinterpret_cast<const sock_filter*>(progB);

    for (size_t len : {0u, 10u, 13u, 14u, 16u, 17u}) {
        auto frame = makeEcatFrame(0xF8, 0x88A4, len);
        EXPECT_EQ(runBpf(cyc, 7, frame.data(), frame.size()), 0u)
            << "len " << len;
        // Async also drops ECAT-short frames; <14B drops before the
        // ethertype check even runs.
        EXPECT_EQ(runBpf(asy, 7, frame.data(), frame.size()), 0u)
            << "len " << len;
    }
    // A ≥14-byte frame with a non-ECAT ethertype IS accepted by the async
    // filter (the OOB idx load is never reached).
    auto vlan = makeEcatFrame(0, 0x8100, 18);
    EXPECT_EQ(runBpf(cyc, 7, vlan.data(), vlan.size()), 0u);
    EXPECT_NE(runBpf(asy, 7, vlan.data(), vlan.size()), 0u);
}

// ============================================================================
// Real kernel BPF execution — SO_ATTACH_FILTER on AF_UNIX SOCK_DGRAM.
// The kernel verifier accepts the program and the in-kernel interpreter
// runs it per datagram on payload bytes.  Proves the exact production
// instruction stream works in the real BPF engine.
// ============================================================================

TEST(CyclicBpfKernel, UnixSocketpairRunsCyclicProgramInKernel) {
    int sv[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    CyclicBpfInsn prog[8];
    ASSERT_EQ(cyclicChannelBpfProgram(true, prog, 8), kCyclicBpfInsnCount);
    sock_fprog fp{static_cast<unsigned short>(kCyclicBpfInsnCount),
                       reinterpret_cast<sock_filter*>(prog)};
    ASSERT_EQ(setsockopt(sv[1], SOL_SOCKET, SO_ATTACH_FILTER,
                       &fp, sizeof(fp)), 0)
        << "kernel refused cyclic BPF: " << strerror(errno);

    auto sendDgram = [&](uint16_t et, uint8_t idx, size_t len = 60) {
        std::vector<uint8_t> d(len, 0);
        if (len > 13) { d[12] = et >> 8; d[13] = et & 0xFF; }
        if (len > 17) d[17] = idx;
        ASSERT_EQ(send(sv[0], d.data(), d.size(), 0), (ssize_t)d.size());
    };
    auto received = [&]() -> bool {
        uint8_t buf[2048];
        pollfd pfd{sv[1], POLLIN, 0};
        if (poll(&pfd, 1, 50) <= 0) return false;
        return recv(sv[1], buf, sizeof(buf), 0) > 0;
    };

    sendDgram(0x88A4, 0xF8);  EXPECT_TRUE(received());
    sendDgram(0x88A4, 0xFD);  EXPECT_TRUE(received());
    sendDgram(0x88A4, 0xF7);  EXPECT_FALSE(received());
    sendDgram(0x88A4, 0xFE);  EXPECT_FALSE(received());
    sendDgram(0x88A4, 0x00);  EXPECT_FALSE(received());
    sendDgram(0x88A4, 0xFF);  EXPECT_FALSE(received());
    sendDgram(0x0800, 0xF8);  EXPECT_FALSE(received());   // non-ECAT
    sendDgram(0x88A4, 0xF8, 17); EXPECT_FALSE(received()); // ECAT-short → OOB

    close(sv[0]); close(sv[1]);
}

TEST(CyclicBpfKernel, UnixSocketpairRunsAsyncProgramInKernel) {
    int sv[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    CyclicBpfInsn prog[8];
    cyclicChannelBpfProgram(false, prog, 8);
    sock_fprog fp{static_cast<unsigned short>(kCyclicBpfInsnCount),
                       reinterpret_cast<sock_filter*>(prog)};
    ASSERT_EQ(setsockopt(sv[1], SOL_SOCKET, SO_ATTACH_FILTER,
                       &fp, sizeof(fp)), 0);

    auto sendDgram = [&](uint16_t et, uint8_t idx, size_t len = 60) {
        std::vector<uint8_t> d(len, 0);
        if (len > 13) { d[12] = et >> 8; d[13] = et & 0xFF; }
        if (len > 17) d[17] = idx;
        ASSERT_EQ(send(sv[0], d.data(), d.size(), 0), (ssize_t)d.size());
    };
    auto received = [&]() -> bool {
        uint8_t buf[2048];
        pollfd pfd{sv[1], POLLIN, 0};
        if (poll(&pfd, 1, 50) <= 0) return false;
        return recv(sv[1], buf, sizeof(buf), 0) > 0;
    };

    sendDgram(0x88A4, 0xF8);  EXPECT_FALSE(received());
    sendDgram(0x88A4, 0x42);  EXPECT_TRUE(received());
    sendDgram(0x88A4, 0xFE);  EXPECT_TRUE(received());
    sendDgram(0x88A4, 0xFF);  EXPECT_TRUE(received());
    sendDgram(0x0800, 0xF8);  EXPECT_TRUE(received());    // non-ECAT
    sendDgram(0x8100, 0xF8);  EXPECT_TRUE(received());    // VLAN → async
    sendDgram(0x88A4, 0xF8, 17); EXPECT_FALSE(received()); // OOB → drop

    close(sv[0]); close(sv[1]);
}

// ============================================================================
// LinuxSocketChannel over a caller-provided fd (unix dgram pair)
// ============================================================================

class SocketChannelTest : public ::testing::Test {
protected:
    int peer_ = -1;
    std::unique_ptr<ICyclicChannel> ch_;

    void SetUp() override {
        int sv[2];
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);
        peer_ = sv[0];
        ch_ = createCyclicSocketChannelForFd(sv[1], 0);
        ASSERT_NE(ch_, nullptr);
        EXPECT_STREQ(ch_->backendName(), "socket");
        EXPECT_FALSE(ch_->zeroCopy());
        EXPECT_EQ(ch_->fd(), sv[1]);
    }
    void TearDown() override {
        ch_.reset();
        close(peer_);
    }
    void sendDgram(const uint8_t* d, size_t n) {
        ASSERT_EQ(send(peer_, d, n, 0), (ssize_t)n);
    }
};

TEST_F(SocketChannelTest, RxPollDeliversDatagrams) {
    const uint8_t d[32] = {0xDE, 0xAD};
    sendDgram(d, sizeof(d));
    CyclicFrameView v[8];
    const int n = ch_->rxPoll(v, 8, 100'000);
    ASSERT_EQ(n, 1);
    EXPECT_EQ(v[0].frame[0], 0xDE);
    EXPECT_EQ(v[0].frame[1], 0xAD);
    EXPECT_EQ(v[0].frame_len, 32u);
    EXPECT_NE(v[0].stamp_ns, 0u);
    EXPECT_EQ(ch_->droppedRx(), 0u);
}

TEST_F(SocketChannelTest, RxPollTimeoutReturnsZero) {
    CyclicFrameView v[8];
    auto t0 = std::chrono::steady_clock::now();
    EXPECT_EQ(ch_->rxPoll(v, 8, 30'000'000), 0);
    auto dt = std::chrono::steady_clock::now() - t0;
    EXPECT_GE(std::chrono::duration_cast<std::chrono::microseconds>(dt).count(),
              25'000LL);
}

TEST_F(SocketChannelTest, RxPollNonblockingReturnsImmediately) {
    CyclicFrameView v[8];
    EXPECT_EQ(ch_->rxPoll(v, 8, 0), 0);
}

TEST_F(SocketChannelTest, BankExhaustionDefersAndCounts) {
    const uint8_t d[8] = {};
    for (int i = 0; i < 6; ++i) sendDgram(d, sizeof(d));
    CyclicFrameView v[8];
    // Bank depth is 4 → first 4 emitted; the poll then hits bank-full and
    // counts one deferral; remaining datagrams stay queued in the kernel.
    const int n = ch_->rxPoll(v, 8, 50'000'000);
    EXPECT_EQ(n, 4);
    EXPECT_EQ(ch_->droppedRx(), 1u);

    // Release one view → its bank slot recycles on the next poll.
    ch_->rxRelease(v[0].cookie);
    CyclicFrameView v2[8];
    EXPECT_EQ(ch_->rxPoll(v2, 8, 50'000'000), 2);  // the two still queued
    EXPECT_EQ(ch_->droppedRx(), 1u);
}

TEST_F(SocketChannelTest, HeldSlotIsNotRecycled) {
    const uint8_t d1[8] = {0x01};
    const uint8_t d2[8] = {0x02};
    sendDgram(d1, sizeof(d1));
    CyclicFrameView v[8];
    ASSERT_EQ(ch_->rxPoll(v, 8, 50'000'000), 1);
    ch_->rxHold(v[0].cookie);               // pin past the next poll
    sendDgram(d2, sizeof(d2));
    CyclicFrameView v2[8];
    EXPECT_EQ(ch_->rxPoll(v2, 8, 50'000'000), 1);
    EXPECT_EQ(v[0].frame[0], 0x01);         // held view still valid
    EXPECT_EQ(v2[0].frame[0], 0x02);
    ch_->rxRelease(v[0].cookie);
    // Releasing an already-free cookie must be a safe no-op (clamped at 0).
    ch_->rxRelease(v[0].cookie);
    sendDgram(d1, sizeof(d1));
    EXPECT_EQ(ch_->rxPoll(v2, 8, 50'000'000), 1);
}

TEST_F(SocketChannelTest, OutOfRangeCookieIsIgnored) {
    ch_->rxHold(9999);
    ch_->rxRelease(9999);
    CyclicFrameView v[8];
    EXPECT_EQ(ch_->rxPoll(v, 8, 0), 0);     // no crash, nothing pending
}

TEST_F(SocketChannelTest, TxAcquireAndCommitFailOnUnixFd) {
    // sendto(sockaddr_ll) on AF_UNIX → EINVAL: exercises the failure path.
    uint8_t* buf = ch_->txAcquire();
    ASSERT_NE(buf, nullptr);
    EXPECT_GE(ch_->txCapacity(), 60u);
    std::memset(buf, 0, 64);
    EXPECT_FALSE(ch_->txCommitFrame(60));
}

TEST_F(SocketChannelTest, TxSendPartsFailsCleanlyOnUnixFd) {
    CyclicTxParts p{};
    uint8_t hdr[26] = {}; uint8_t pay[4] = {1,2,3,4};
    p.header = hdr; p.header_len = 26;
    p.payload = pay; p.payload_len = 4;
    p.wkc = 7;
    EXPECT_FALSE(ch_->txSendParts(p));      // sendmsg fails on unix fd
}

TEST_F(SocketChannelTest, DefaultTxSendPartsStagesBuffer) {
    // Exercise the base-class staging impl on a minimal subclass.
    struct DefaultChannel : ICyclicChannel {
        uint8_t buf[1600] = {};
        int commits = 0; uint32_t last_len = 0;
        uint8_t* txAcquire() override { return buf; }
        size_t txCapacity() const override { return sizeof(buf); }
        bool txCommitFrame(uint32_t n) override {
            ++commits; last_len = n; return true;
        }
        int rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
        void rxHold(uint32_t) override {}
        void rxRelease(uint32_t) override {}
        int fd() const override { return -1; }
        bool zeroCopy() const override { return false; }
        const char* backendName() const override { return "test"; }
    } d;
    CyclicTxParts p{};
    uint8_t hdr[26] = {}; hdr[0] = 0xEE; hdr[25] = 0x99;
    uint8_t pay[4] = {0x11, 0x22, 0x33, 0x44};
    p.header = hdr; p.header_len = 26;
    p.payload = pay; p.payload_len = 4;
    p.wkc = 0xBEEF;
    ASSERT_TRUE(d.txSendParts(p));
    EXPECT_EQ(d.commits, 1);
    EXPECT_EQ(d.last_len, 26u + 4u + 2u);
    EXPECT_EQ(d.buf[0], 0xEE);
    EXPECT_EQ(d.buf[25], 0x99);
    EXPECT_EQ(d.buf[26], 0x11);
    EXPECT_EQ(d.buf[29], 0x44);
    EXPECT_EQ(d.buf[30], 0xEF);             // wkc little-endian
    EXPECT_EQ(d.buf[31], 0xBE);
}

TEST_F(SocketChannelTest, DefaultTxSendPartsRejectsOversize) {
    struct SmallChannel : ICyclicChannel {
        uint8_t buf[32];
        uint8_t* txAcquire() override { return buf; }
        size_t txCapacity() const override { return sizeof(buf); }
        bool txCommitFrame(uint32_t) override { return true; }
        int rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
        void rxHold(uint32_t) override {}
        void rxRelease(uint32_t) override {}
        int fd() const override { return -1; }
        bool zeroCopy() const override { return false; }
        const char* backendName() const override { return "s"; }
    } s;
    CyclicTxParts p{};
    uint8_t dummy[40] = {};
    p.header = dummy; p.header_len = 26;
    p.payload = dummy; p.payload_len = 16;   // 26+16+2 > 32
    EXPECT_FALSE(s.txSendParts(p));
}

TEST_F(SocketChannelTest, DefaultTxSendPartsAcquireFail) {
    struct NullAcquire : ICyclicChannel {
        uint8_t* txAcquire() override { return nullptr; }
        size_t txCapacity() const override { return 64; }
        bool txCommitFrame(uint32_t) override { return true; }
        int rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
        void rxHold(uint32_t) override {}
        void rxRelease(uint32_t) override {}
        int fd() const override { return -1; }
        bool zeroCopy() const override { return false; }
        const char* backendName() const override { return "n"; }
    } n;
    CyclicTxParts p{};
    uint8_t hdr[26] = {};
    p.header = hdr; p.header_len = 26;
    EXPECT_FALSE(n.txSendParts(p));
}

TEST_F(SocketChannelTest, NullPayloadOmitsIovec) {
    struct DefaultChannel : ICyclicChannel {
        uint8_t buf[1600] = {};
        int commits = 0; uint32_t last_len = 0;
        uint8_t* txAcquire() override { return buf; }
        size_t txCapacity() const override { return sizeof(buf); }
        bool txCommitFrame(uint32_t n) override {
            ++commits; last_len = n; return true;
        }
        int rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
        void rxHold(uint32_t) override {}
        void rxRelease(uint32_t) override {}
        int fd() const override { return -1; }
        bool zeroCopy() const override { return false; }
        const char* backendName() const override { return "t"; }
    } d;
    CyclicTxParts p{};
    uint8_t hdr[26] = {}; hdr[5] = 0x55;
    p.header = hdr; p.header_len = 26;
    p.payload = nullptr; p.payload_len = 0;
    p.wkc = 0x1234;
    ASSERT_TRUE(d.txSendParts(p));
    EXPECT_EQ(d.last_len, 28u);              // header + wkc only
    EXPECT_EQ(d.buf[26], 0x34);
}

// ============================================================================
// Master-level cyclic datapath: software deposit, channel view publish,
// mixed-frame routing.  Uses the MasterCyclicTestAccess friend seam.
// ============================================================================

namespace EtherCAT {
/// Defined in Master.hpp via friend — grants access to cyclic_channel_ and
/// dispatchChannelFrame for tests.
struct MasterCyclicTestAccess {
    static void setChannel(Master& m, std::unique_ptr<ICyclicChannel> ch) {
        m.cyclic_channel_ = std::move(ch);
    }
    static void dispatch(Master& m, const CyclicFrameView& v) {
        m.dispatchChannelFrame(v);
    }
};
} // namespace EtherCAT

namespace {

/// Scripted channel for Master tests — records rxHold/rxRelease and serves
/// an owned TX staging buffer.
class StubChannel : public ICyclicChannel {
public:
    std::atomic<int> holds{0};
    uint8_t tx_buf[1600] = {};
    int     tx_send_parts_calls = 0;
    CyclicTxParts last_parts{};
    uint8_t hdr_copy[64] = {};   // header points into a dead stack buffer
    bool    send_parts_result = true;

    uint8_t* txAcquire() override { return tx_buf; }
    size_t   txCapacity() const override { return sizeof(tx_buf); }
    bool     txCommitFrame(uint32_t) override { return true; }
    bool     txSendParts(const CyclicTxParts& p) override {
        ++tx_send_parts_calls;
        last_parts = p;
        std::memcpy(hdr_copy, p.header,
                    std::min<size_t>(p.header_len, sizeof(hdr_copy)));
        last_parts.header = hdr_copy;
        return send_parts_result;
    }
    int  rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
    void rxHold(uint32_t)   override { holds.fetch_add(1); }
    void rxRelease(uint32_t) override { holds.fetch_sub(1); }
    int  fd() const override { return -1; }
    bool zeroCopy() const override { return true; }
    const char* backendName() const override { return "stub"; }
};

/// Build a wire-format EtherCAT frame with one datagram.
/// Returns frame length (>=60).
size_t buildEcatFrame(uint8_t* f, uint8_t cmd, uint8_t idx,
                      uint16_t adp, uint16_t ado,
                      const uint8_t* payload, uint16_t datalen,
                      uint16_t wkc, bool more = false)
{
    std::memset(f, 0, 14);
    f[12] = 0x88; f[13] = 0xA4;
    const uint16_t ecat_len = static_cast<uint16_t>(10 + datalen + 2);
    f[14] = ecat_len & 0xFF;
    f[15] = ((ecat_len >> 8) & 0x07) | 0x10;      // type=1
    f[16] = cmd;
    f[17] = idx;
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

} // anonymous namespace

class MasterCyclicTest : public ::testing::Test {
protected:
    Master master_;
    NetworkInterface iface_{};
    uint8_t mac_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    void SetUp() override {
        iface_.send = [](const uint8_t*, size_t) { return true; };
        master_.start(iface_, mac_);
    }
    void TearDown() override { master_.stop(); }
};

TEST_F(MasterCyclicTest, SoftwareDepositCopyPath) {
    // Inject a cyclic response through the public frame entry — the parser
    // deposits into slot 0 by copy (no channel → channel=nullptr).
    uint8_t frame[128];
    const uint8_t pay[4] = {0x11, 0x22, 0x33, 0x44};
    const size_t n = buildEcatFrame(frame, 0x0C, 0xF8, 0x1234, 0x5678,
                                    pay, 4, 3);
    master_.handleRxFrame(frame, n);

    CyclicSlotView view{};
    ASSERT_TRUE(master_.waitCyclicSlotView(0, /*token=*/0, 0, view));
    EXPECT_EQ(view.datalen, 4u);
    EXPECT_EQ(view.wkc, 3u);
    EXPECT_EQ(view.adp, 0x1234u);
    EXPECT_EQ(view.ado, 0x5678u);
    ASSERT_NE(view.payload, nullptr);
    EXPECT_EQ(view.payload[0], 0x11);
    EXPECT_EQ(view.payload[3], 0x44);
    EXPECT_EQ(view.channel, nullptr);   // copy path — no channel view
}

TEST_F(MasterCyclicTest, AsyncIdxDoesNotDepositCyclicSlot) {
    uint8_t frame[128];
    const uint8_t pay[4] = {0xAA};
    const size_t n = buildEcatFrame(frame, 0x07, 0x42, 0, 0, pay, 4, 1);
    master_.handleRxFrame(frame, n);

    CyclicSlotView view{};
    // Slot 0 seq unchanged → wait on token=0 times out fast.
    EXPECT_FALSE(master_.waitCyclicSlotView(0, 0, 1'000'000, view));
}

TEST_F(MasterCyclicTest, ChannelViewPublishPath) {
    auto stub = std::make_unique<StubChannel>();
    StubChannel* stubp = stub.get();
    MasterCyclicTestAccess::setChannel(master_, std::move(stub));

    uint8_t frame[128];
    const uint8_t pay[6] = {9, 8, 7, 6, 5, 4};
    const size_t n = buildEcatFrame(frame, 0x0C, 0xFA, 1, 2, pay, 6, 2);
    CyclicFrameView v{};
    v.frame = frame; v.frame_len = n; v.cookie = 7;
    MasterCyclicTestAccess::dispatch(master_, v);

    // The datagram (idx 0xFA → slot 2) published as a view into `frame`.
    CyclicSlotView view{};
    ASSERT_TRUE(master_.waitCyclicSlotView(2, 0, 0, view));
    EXPECT_EQ(view.payload, frame + 26);        // zero-copy — same buffer
    EXPECT_EQ(view.datalen, 6u);
    EXPECT_EQ(view.cookie, 7u);
    EXPECT_EQ(view.channel, stubp);
    EXPECT_EQ(stubp->holds.load(), 1);          // one slot holds the cookie

    // Second publish on the same slot releases the old cookie.
    MasterCyclicTestAccess::dispatch(master_, v);
    EXPECT_EQ(stubp->holds.load(), 1);          // still exactly one hold
}

TEST_F(MasterCyclicTest, MixedFrameGoesToParser) {
    auto stub = std::make_unique<StubChannel>();
    StubChannel* stubp = stub.get();
    MasterCyclicTestAccess::setChannel(master_, std::move(stub));

    // Two datagrams in one frame: cyclic 0xF8 then async 0x42.
    uint8_t frame[256];
    const uint8_t pay[4] = {1, 2, 3, 4};
    size_t n = buildEcatFrame(frame, 0x0C, 0xF8, 0, 0, pay, 4, 1,
                              /*more=*/true);
    // Append second datagram at offset 44 (26+4+2=32 → next dg at 32... wait
    // build: hdr 26 + data 4 + wkc 2 = 32; second dg hdr starts at 32).
    const size_t off = 26 + 4 + 2;
    frame[off + 0] = 0x07;                       // cmd FPRD-ish
    frame[off + 1] = 0x42;                       // async idx
    std::memset(frame + off + 2, 0, 8);          // adp/ado/len/irq
    const uint16_t lf = 4;
    std::memcpy(frame + off + 6, &lf, 2);
    const uint16_t wkc2 = 1;
    std::memcpy(frame + off + 10 + 4, &wkc2, 2);
    // Fix the ecat-len field to cover both datagrams.
    const uint16_t ecat_len = static_cast<uint16_t>(10 + 4 + 2 + 10 + 4 + 2);
    frame[14] = ecat_len & 0xFF;
    frame[15] = ((ecat_len >> 8) & 0x07) | 0x10;
    n = off + 10 + 4 + 2;

    CyclicFrameView v{};
    v.frame = frame; v.frame_len = n; v.cookie = 3;
    MasterCyclicTestAccess::dispatch(master_, v);

    // The cyclic datagram was deposited by the PARSER (copy path), not by
    // view publish — the channel hold count stays 0.
    EXPECT_EQ(stubp->holds.load(), 0);
    CyclicSlotView view{};
    ASSERT_TRUE(master_.waitCyclicSlotView(0, 0, 0, view));
    EXPECT_EQ(view.channel, nullptr);            // copy-mode deposit
    EXPECT_EQ(view.datalen, 4u);
    EXPECT_EQ(view.payload[0], 1);
}

TEST_F(MasterCyclicTest, ShortChannelFrameIgnored) {
    auto stub = std::make_unique<StubChannel>();
    MasterCyclicTestAccess::setChannel(master_, std::move(stub));
    uint8_t frame[16] = {};
    CyclicFrameView v{};
    v.frame = frame; v.frame_len = 10; v.cookie = 0;
    MasterCyclicTestAccess::dispatch(master_, v);   // must not crash/publish
    CyclicSlotView view{};
    EXPECT_FALSE(master_.waitCyclicSlotView(0, 0, 1'000'000, view));
}

TEST_F(MasterCyclicTest, SendCyclicDatagramUsesChannelParts) {
    auto stub = std::make_unique<StubChannel>();
    StubChannel* stubp = stub.get();
    MasterCyclicTestAccess::setChannel(master_, std::move(stub));

    const uint8_t pay[8] = {0xAB};
    ASSERT_TRUE(master_.sendCyclicDatagram(Command::LRW, 0, 0, 0x0001,
                                           pay, 8, true));
    EXPECT_EQ(stubp->tx_send_parts_calls, 1);
    EXPECT_EQ(stubp->last_parts.header_len, 26u);
    EXPECT_EQ(stubp->last_parts.payload_len, 8u);
    EXPECT_EQ(stubp->last_parts.payload, pay);
    // The staged header must carry idx 0xF8 at offset 17.
    EXPECT_EQ(stubp->last_parts.header[17], 0xF8);
}

TEST_F(MasterCyclicTest, NoChannelAcquireAndFrameSendFail) {
    EXPECT_EQ(master_.acquireCyclicTxFrame(), nullptr);
    EXPECT_FALSE(master_.sendCyclicFrame(60));
}

TEST_F(MasterCyclicTest, WaitTimeoutAndCancel) {
    CyclicSlotView view{};
    auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(master_.waitCyclicSlotView(1, 0, 5'000'000, view));
    auto dt = std::chrono::steady_clock::now() - t0;
    EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 4);

    master_.requestCancel();
    EXPECT_FALSE(master_.waitCyclicSlotView(1, 0, 500'000'000, view));
    master_.clearCancel();
}

TEST_F(MasterCyclicTest, WaitCyclicSlotOutOfRange) {
    CyclicSlotView view{};
    EXPECT_FALSE(master_.waitCyclicSlotView(0xFF, 0, 0, view));
}

TEST_F(MasterCyclicTest, ComposeCyclicHeaderLayout) {
    uint8_t frame[64] = {};
    master_.composeCyclicHeader(frame, Command::LRW, 2, 0x1111, 0x2222,
                                8, true);
    EXPECT_EQ(frame[12], 0x88);
    EXPECT_EQ(frame[13], 0xA4);
    EXPECT_EQ(frame[16], 0x0C);          // LRW
    EXPECT_EQ(frame[17], 0xFA);          // 0xF8 + slot 2
    uint16_t adp, ado;
    std::memcpy(&adp, frame + 18, 2);
    std::memcpy(&ado, frame + 20, 2);
    EXPECT_EQ(adp, 0x1111);
    EXPECT_EQ(ado, 0x2222);
}

// ============================================================================
// Live AF_PACKET tests on loopback — auto-skipped without CAP_NET_RAW
// ============================================================================

static bool haveCapNetRaw() {
    int s = socket(AF_PACKET, SOCK_RAW, htons(0x88A4));
    if (s >= 0) { close(s); return true; }
    return false;
}

class LivePacketTest : public ::testing::Test {
protected:
    unsigned ifindex_ = 0;
    void SetUp() override {
        if (!haveCapNetRaw())
            GTEST_SKIP() << "CAP_NET_RAW unavailable — skipping live test";
        ifindex_ = if_nametoindex("lo");
        ASSERT_NE(ifindex_, 0u);
    }
    int makePacketSock(unsigned ifindex, bool cyclic_filter,
                       uint16_t proto = 0x88A4) {
        int s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
        if (s < 0) return -1;
        const bool ok = cyclic_filter
            ? cyclicChannelAttachCyclicFilter(s)
            : cyclicChannelAttachAsyncFilter(s);
        if (!ok) { close(s); return -1; }
        int one = 1;
        setsockopt(s, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
        sockaddr_ll ll{};
        ll.sll_family   = AF_PACKET;
        ll.sll_ifindex  = (int)ifindex;
        ll.sll_protocol = htons(proto);
        if (bind(s, (sockaddr*)&ll, sizeof(ll)) != 0) { close(s); return -1; }
        return s;
    }
};

TEST_F(LivePacketTest, KernelDemuxOnLoopback) {
    // Two sockets on lo with opposite filters: an EtherCAT frame with a
    // cyclic idx must appear ONLY on the cyclic socket.  The async socket
    // is bound to ETH_P_ALL like a real async socket so non-ECAT traffic
    // also reaches it.
    int cyc = makePacketSock(ifindex_, true, 0x88A4);
    int asy = makePacketSock(ifindex_, false, ETH_P_ALL);
    ASSERT_GE(cyc, 0); ASSERT_GE(asy, 0);

    int tx = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    ASSERT_GE(tx, 0);

    auto sendFrame = [&](uint8_t idx, uint16_t et = 0x88A4) {
        uint8_t f[60] = {};
        std::memset(f, 0xFF, 6);             // dst broadcast
        f[12] = et >> 8; f[13] = et & 0xFF;
        f[14] = 0x20;                         // ecat len low (arbitrary)
        f[16] = 0x0C; f[17] = idx;
        sockaddr_ll ll{};
        ll.sll_family  = AF_PACKET;
        ll.sll_ifindex = (int)ifindex_;
        ll.sll_halen   = 6;
        std::memset(ll.sll_addr, 0xFF, 6);
        ASSERT_EQ(sendto(tx, f, sizeof(f), 0, (sockaddr*)&ll, sizeof(ll)),
                  (ssize_t)sizeof(f));
    };
    auto got = [&](int fd) -> bool {
        uint8_t b[2048];
        pollfd p{fd, POLLIN, 0};
        if (poll(&p, 1, 300) <= 0) return false;
        return recv(fd, b, sizeof(b), 0) > 0;
    };

    sendFrame(0xF8);
    EXPECT_TRUE(got(cyc));
    EXPECT_FALSE(got(asy));

    sendFrame(0x42);
    EXPECT_FALSE(got(cyc));
    EXPECT_TRUE(got(asy));

    sendFrame(0xFD);
    EXPECT_TRUE(got(cyc));
    EXPECT_FALSE(got(asy));

    sendFrame(0xFE);
    EXPECT_FALSE(got(cyc));
    EXPECT_TRUE(got(asy));

    sendFrame(0xF8, 0x0800);                 // non-ECAT ethertype
    EXPECT_FALSE(got(cyc));
    EXPECT_TRUE(got(asy));

    close(tx); close(cyc); close(asy);
}

TEST_F(LivePacketTest, RingChannelRxAndTx) {
    int s = socket(AF_PACKET, SOCK_RAW, htons(0x88A4));
    ASSERT_GE(s, 0);
    int one = 1;
    setsockopt(s, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
    sockaddr_ll ll{};
    ll.sll_family   = AF_PACKET;
    ll.sll_ifindex  = (int)ifindex_;
    ll.sll_protocol = htons(0x88A4);
    ASSERT_EQ(bind(s, (sockaddr*)&ll, sizeof(ll)), 0);

    auto ch = createCyclicRingChannelForFd(s, (int)ifindex_, 16, 8);
    ASSERT_NE(ch, nullptr);
    EXPECT_TRUE(ch->zeroCopy());
    EXPECT_STREQ(ch->backendName(), "ring");

    // TX: compose + commit a cyclic frame in a ring slot — lo loops it
    // back and the RX ring should surface it.
    uint8_t* f = ch->txAcquire();
    ASSERT_NE(f, nullptr);
    EXPECT_GT(ch->txCapacity(), 60u);
    std::memset(f, 0, 60);
    std::memset(f, 0xFF, 6);
    f[12] = 0x88; f[13] = 0xA4; f[16] = 0x0C; f[17] = 0xF8;
    ASSERT_TRUE(ch->txCommitFrame(60));

    CyclicFrameView v[4];
    int n = ch->rxPoll(v, 4, 500'000'000);
    ASSERT_GE(n, 1);
    bool saw = false;
    for (int i = 0; i < n; ++i)
        if (v[i].frame_len >= 18 && v[i].frame[17] == 0xF8) saw = true;
    EXPECT_TRUE(saw);
    ch->rxRelease(v[0].cookie);
}

TEST_F(LivePacketTest, FactoryAutoOnLoopback) {
    CyclicChannelConfig cfg;
    cfg.ifindex   = (int)ifindex_;
    cfg.wire_mode = CyclicWireMode::Auto;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);
    uint8_t* f = ch->txAcquire();
    EXPECT_NE(f, nullptr);
}

// Runs unprivileged — ifindex is validated before any socket is created.
TEST(CyclicChannelFactory, InvalidIfindexFails) {
    CyclicChannelConfig cfg;
    cfg.ifindex = 0;
    EXPECT_EQ(createCyclicChannel(cfg), nullptr);
    cfg.ifindex = -5;
    EXPECT_EQ(createCyclicChannel(cfg), nullptr);
}

TEST_F(LivePacketTest, AsyncFilterAttachViaConfig) {
    // The factory attaches the mirror filter to async_fd when provided.
    int asy = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    ASSERT_GE(asy, 0);
    CyclicChannelConfig cfg;
    cfg.ifindex  = (int)ifindex_;
    cfg.async_fd = asy;
    auto ch = createCyclicChannel(cfg);
    ASSERT_NE(ch, nullptr);
    // The async socket now carries the mirror filter: send a cyclic frame
    // and verify socket B does NOT see it.
    uint8_t* f = ch->txAcquire();
    ASSERT_NE(f, nullptr);
    std::memset(f, 0, 60);
    std::memset(f, 0xFF, 6);
    f[12] = 0x88; f[13] = 0xA4; f[16] = 0x0C; f[17] = 0xF9;
    ASSERT_TRUE(ch->txCommitFrame(60));

    // Cyclic socket receives; async (filtered) must not.
    CyclicFrameView v[4];
    EXPECT_GE(ch->rxPoll(v, 4, 500'000'000), 1);
    uint8_t b[2048];
    pollfd p{asy, POLLIN, 0};
    if (poll(&p, 1, 300) > 0) {
        // Any frame arriving must not be the cyclic one.
        ssize_t n = recv(asy, b, sizeof(b), 0);
        if (n >= 18) EXPECT_NE(b[17], 0xF9);
    }
    close(asy);
}

#endif // __linux__
