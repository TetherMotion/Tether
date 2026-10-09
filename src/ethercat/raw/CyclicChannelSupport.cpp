/**
 * @file CyclicChannelSupport.cpp
 * @brief Shared Linux cyclic-channel helpers + exported BPF filter helpers.
 */

#include "raw/CyclicChannelSupport.hpp"

#if defined(__linux__)

#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "hal/IEthernet.hpp"
#include "logging/Logger.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <ctime>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/filter.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif
#ifndef PACKET_IGNORE_OUTGOING
#define PACKET_IGNORE_OUTGOING 23
#endif

namespace EtherCAT {

static const char* TAG = "cyc_chan";

// ============================================================================
// BPF demux
// ============================================================================
//
// Frame layout (untagged): [eth 14][ecat hdr 2][dg: cmd|idx|adp|ado|len|irq].
// The first datagram's idx sits at byte offset 17.  Cyclic frames carry only
// reserved idxes; async frames can never get them (allocIdx skips the range)
// — so first-idx demux is exact.  Frames shorter than 18 bytes make the idx
// load fault out-of-bounds — kernel cBPF semantics then reject the packet on
// BOTH sockets (malformed traffic is dropped entirely, which is desirable).
// VLAN-tagged frames show EtherType 0x8100: the program unwraps them
// (inner EtherType at [16:18], idx at 21) so tagged fastpath traffic
// reaches socket A too.

constexpr int  kIdxByteOffset     = 17;   // untagged: first dg idx
constexpr int  kVlanIdxByteOffset = 21;   // 802.1Q-tagged: +4 tag bytes
constexpr int  kVlanInnerEthOff   = 16;   // inner EtherType offset
constexpr uint16_t kEtherCatType  = 0x88A4;
constexpr uint16_t kVlanType      = 0x8100;

static_assert(sizeof(CyclicBpfInsn) == sizeof(struct sock_filter),
              "CyclicBpfInsn must be layout-compatible with sock_filter");

// Socket A / socket B demux program — see cyclicChannelBpfProgram().
// Q15: VLAN-aware — a 0x8100 tag shifts the frame by 4 bytes; the inner
// EtherType lives at [16:18] and the first idx at byte 21.  Non-ECAT and
// VLAN-non-ECAT traffic falls through to the B verdict so socket A can be
// bound to ETH_P_ALL and still see only cyclic EtherCAT.
size_t buildFilterProg(bool accept_cyclic, struct sock_filter* p) {
    // The accept range covers the fastpath band (0x9C..0xFF) EXCEPT the
    // 0xFE fire-and-forget index — its echoes feed txpdo_rx_queue_
    // consumers through the async parser, so #11 bounces them to the B
    // verdict.  Async allocIdx() never reaches 0x9C, so first-idx demux
    // stays exact.
    const uint32_t lo = kFastSlotBaseIdx;
    const uint32_t hi = kFastSlotEndIdx;
    const struct sock_filter prog[] = {
        /* 0 */ BPF_STMT(BPF_LD | BPF_H | BPF_ABS, 12),              // EtherType
        /* 1 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kEtherCatType, 0, 2),
        /* 2 */ BPF_STMT(BPF_LD | BPF_B | BPF_ABS, kIdxByteOffset),  // idx
        /* 3 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, 0, 5, 0),        // → #9
        /* 4 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kVlanType, 0, 8),
        /* 5 */ BPF_STMT(BPF_LD | BPF_H | BPF_ABS, kVlanInnerEthOff),
        /* 6 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, kEtherCatType, 0, 6),
        /* 7 */ BPF_STMT(BPF_LD | BPF_B | BPF_ABS, kVlanIdxByteOffset),
        /* 8 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, 0, 0, 0),        // → #9
        /* 9 */ BPF_JUMP(BPF_JMP | BPF_JGE | BPF_K, lo, 0, 3),       // <lo → #13
        /*10 */ BPF_JUMP(BPF_JMP | BPF_JGT | BPF_K, hi, 2, 0),       // >hi → #13
        /*11 */ BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K,
                        kFastSlotReservedIdx, 1, 0),                 // 0xFE → #13
        /*12 */ BPF_STMT(BPF_RET | BPF_K,
                        accept_cyclic ? 0xFFFFFFFFu : 0u),           // A: accept
        /*13 */ BPF_STMT(BPF_RET | BPF_K,
                        accept_cyclic ? 0u : 0xFFFFFFFFu),           // B: accept
    };
    std::memcpy(p, prog, sizeof(prog));
    return sizeof(prog) / sizeof(prog[0]);
}

bool attachFilter(int fd, struct sock_filter* prog, size_t n,
                  bool lock = false) {
    struct sock_fprog fp;
    fp.len    = static_cast<unsigned short>(n);
    fp.filter = prog;
    if (setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &fp, sizeof(fp)) < 0)
        return false;
    if (lock) {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_LOCK_FILTER, &one, sizeof(one));
    }
    return true;
}

int openCyclicSocket(int ifindex,
                     const CBPFInsn* accept_prog,
                     size_t accept_prog_len) {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) return -1;

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int one = 1;
    // Q18: PACKET_IGNORE_OUTGOING drops own-TX echoes upstream of the BPF.
    // Probe the sockopt result — a kernel that lacks it leaves the async
    // socket seeing its own cyclic frames (correctness unaffected, wakeup
    // cost) — warn so the operator knows.
    if (setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING,
                   &one, sizeof(one)) < 0) {
        TETHER_LOGW(TAG, "PACKET_IGNORE_OUTGOING unsupported ({}) — own "
                    "transmits will echo on the RX path", strerror(errno));
    }
    setsockopt(fd, SOL_PACKET, PACKET_TIMESTAMP, &one, sizeof(one));
    // Kernel RX timestamps arrive via recvmsg SCM_TIMESTAMPNS cmsgs.
    setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one));

#ifdef PACKET_QDISC_BYPASS
    // Cyclic TX bypasses the qdisc layer entirely — no dequeue
    // scheduling on the latency-critical send path.  Side effect: without
    // the qdisc absorbing frames, a wedged TX ring (no carrier) fails
    // sendto() with ENOBUFS immediately instead of dropping silently —
    // the exchange's ENOBUFS fast-fail and the TX-diagnostics worker rely
    // on this signal.
    setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS, &one, sizeof(one));
#endif

    // ETH_P_ALL + the extended demux program: VLAN-tagged EtherCAT frames
    // (0x8100 outer) reach the socket and are filtered by inner ethertype;
    // the program rejects everything non-cyclic kernel-side.
    struct sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&sll), sizeof(sll)) < 0) {
        ::close(fd);
        return -1;
    }
    // Register for nic-mon's PACKET_STATISTICS dump on rx_dropped spikes;
    // dead fds are pruned automatically when the channel closes.
    HAL::registerDiagPacketSocket(fd, "cyclic");

    if (accept_prog && accept_prog_len) {
        // Caller-composed program (encap ∧ idx∈fastpath) — replaces the
        // built-in demux so an encapsulation clause isn't dropped.
        if (!CBPFProgramFactory::attach(fd, accept_prog, accept_prog_len)) {
            TETHER_LOGW(TAG, "cyclic composed BPF attach failed: {}",
                        strerror(errno));
        }
    } else {
        struct sock_filter prog[kCyclicBpfInsnCount];
        const size_t n = buildFilterProg(true, prog);
        if (!attachFilter(fd, prog, n)) {
            // Soft failure — the channel still works; crosstalk determinism
            // is lost but correctness is not (parser keys on idx either way).
            TETHER_LOGW(TAG, "cyclic BPF attach failed: {}", strerror(errno));
        }
    }
    return fd;
}

uint64_t monoNowNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

// Kernel RX stamps (tp_sec/tp_nsec, SCM_TIMESTAMPNS) are CLOCK_REALTIME.
// Consumers compare against CLOCK_MONOTONIC deadlines/send times — convert
// once at the channel boundary so CyclicSlotView::stamp_ns is always
// monotonic-domain regardless of which stamp path produced it.
uint64_t rtStampToMonoNs(uint64_t rt_ns) {
    static std::atomic<int64_t> rt_minus_mono_ns{INT64_MIN};
    int64_t off = rt_minus_mono_ns.load(std::memory_order_relaxed);
    if (off == INT64_MIN) {
        struct timespec a, b;
        clock_gettime(CLOCK_REALTIME, &a);
        clock_gettime(CLOCK_MONOTONIC, &b);
        off = static_cast<int64_t>(
                  static_cast<uint64_t>(a.tv_sec) * 1'000'000'000ULL +
                  static_cast<uint64_t>(a.tv_nsec)) -
              static_cast<int64_t>(
                  static_cast<uint64_t>(b.tv_sec) * 1'000'000'000ULL +
                  static_cast<uint64_t>(b.tv_nsec));
        rt_minus_mono_ns.store(off, std::memory_order_relaxed);
    }
    return (off > 0 && rt_ns > static_cast<uint64_t>(off))
               ? rt_ns - static_cast<uint64_t>(off)
               : rt_ns;
}

// ppoll on a single fd for POLLIN, timeout in ns → >0 ready, 0 timeout, <0 err
int waitReadable(int fd, uint32_t timeout_ns) {
    struct pollfd pfd { fd, POLLIN, 0 };
    struct timespec ts {
        static_cast<time_t>(timeout_ns / 1'000'000'000u),
        static_cast<long>(timeout_ns % 1'000'000'000u)
    };
    int ret;
    do { ret = ppoll(&pfd, 1, &ts, nullptr); } while (ret < 0 && errno == EINTR);
    return ret;
}

// ============================================================================
// Exported filter helpers (usable by tests and the async-socket attach)
// ============================================================================

bool cyclicChannelAttachCyclicFilter(int fd) {
    struct sock_filter prog[kCyclicBpfInsnCount];
    return attachFilter(fd, prog, buildFilterProg(true, prog));
}

bool cyclicChannelAttachAsyncFilter(int fd) {
    struct sock_filter prog[kCyclicBpfInsnCount];
    return attachFilter(fd, prog, buildFilterProg(false, prog));
}

size_t cyclicChannelBpfProgram(bool accept_cyclic,
                               CyclicBpfInsn* out, size_t cap) {
    if (!out || cap < kCyclicBpfInsnCount) return 0;
    struct sock_filter prog[kCyclicBpfInsnCount];
    const size_t n = buildFilterProg(accept_cyclic, prog);
    static_assert(sizeof(prog) == kCyclicBpfInsnCount * sizeof(CyclicBpfInsn));
    std::memcpy(out, prog, sizeof(prog));
    return n;
}

} // namespace EtherCAT

#endif // __linux__

