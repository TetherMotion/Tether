#pragma once

/**
 * @file CyclicChannel.hpp
 * @brief Zero-copy-capable wire datapath for the cyclic executive.
 *
 * A cyclic channel owns the *cyclic* side of the EtherCAT wire: frame
 * composition (TX) and frame reception (RX).  It exists alongside the normal
 * NetworkInterface socket, which keeps carrying asynchronous traffic
 * (SDO/mailbox/status polling).
 *
 * Linux implementations demultiplex in the kernel: the cyclic socket carries
 * a classic-BPF filter accepting only EtherCAT frames whose first datagram
 * index lies in the reserved cyclic range (kCyclicSlotBaseIdx..+kNumCyclicSlots),
 * while the async socket gets the mirror filter.  Cross-traffic is therefore
 * impossible, not merely unlikely — and a filter failure falls back to a
 * software deposit path without breaking correctness.
 *
 * Buffer model
 * ------------
 * RX frames are delivered as CyclicFrameView — pointers into channel-owned
 * memory (a PACKET_MMAP ring block or a frame bank slot).  A view stays valid
 * until its cookie's hold count drops to zero; consumers that keep a pointer
 * past the next rxPoll() must call rxHold() and later rxRelease().  This lets
 * the cyclic slot table and the process image publish *pointers into the
 * ring* instead of copies.
 *
 * TX offers two styles:
 *   - txAcquire()/txCommitFrame(): compose a complete Ethernet frame in place
 *     (needed by the Rotating image mode — the app writes the payload region
 *     of the acquired buffer directly).
 *   - txSendParts(): header + payload + WKC from separate buffers; the socket
 *     backend uses sendmsg() scatter-gather (zero payload copy), the ring
 *     backend does one contiguous copy into the slot.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include "tether/ethercat/CBPFProgramFactory.hpp"   // CBPFInsn

namespace EtherCAT {

/// Wire offset of the first datagram's payload in a single-datagram cyclic
/// frame: eth(14) + ecat-hdr(2) + dg-hdr(10).
inline constexpr uint32_t kCyclicFramePayloadOff = 26;

// ============================================================================
// Types
// ============================================================================

/// A received Ethernet frame, zero-copy view into channel memory.
struct CyclicFrameView {
    const uint8_t* frame     = nullptr;  ///< full frame incl. Ethernet header
    uint32_t       frame_len = 0;
    uint64_t       stamp_ns  = 0;        ///< rx timestamp (hw when available)
    uint32_t       cookie    = 0;        ///< opaque — for rxHold/rxRelease
};

/// Scatter-gather TX description: EtherCAT datagram(s) without staging copy.
struct CyclicTxParts {
    const uint8_t* header;       ///< eth + ecat + first datagram header
    uint16_t       header_len;   ///< bytes at header
    const uint8_t* payload;      ///< datagram payload (may be nullptr)
    uint16_t       payload_len;
    uint16_t       wkc;          ///< appended after payload (usually 0 on TX)
};

/// How the cyclic channel reaches the wire.
enum class CyclicWireMode : uint8_t {
    Auto,        ///< Prefer PACKET_MMAP rings; fall back to socket (logged)
    SocketIO,    ///< Plain AF_PACKET socket: recv + sendmsg
    PacketRing,  ///< PACKET_MMAP rings only — creation fails if unsupported
};

// ============================================================================
// Channel interface
// ============================================================================

class ICyclicChannel {
public:
    virtual ~ICyclicChannel() = default;

    // ---- TX ------------------------------------------------------------

    /**
     * @brief Writable buffer for the next cyclic frame.
     * @return >= txCapacity() writable bytes, or nullptr when no TX slot is
     *         currently available (ring full — caller should count and skip).
     */
    virtual uint8_t* txAcquire() = 0;

    /// Maximum bytes one composed frame may occupy.
    virtual size_t   txCapacity() const = 0;

    /// Send the frame composed in the last txAcquire() buffer.
    virtual bool     txCommitFrame(uint32_t frame_len) = 0;

    /**
     * @brief Send header+payload+wkc without requiring txAcquire staging.
     *
     * Socket backend: sendmsg() with an iovec — zero payload copies.
     * Ring backend: one contiguous copy into a TX slot.
     * Default: stage into txAcquire()/txCommitFrame() (one copy).
     */
    virtual bool txSendParts(const CyclicTxParts& parts) {
        uint8_t* dst = txAcquire();
        if (!dst) return false;
        const uint32_t total = parts.header_len +
            (parts.payload ? parts.payload_len : 0) + sizeof(uint16_t);
        if (total > txCapacity()) return false;
        std::memcpy(dst, parts.header, parts.header_len);
        dst += parts.header_len;
        if (parts.payload && parts.payload_len) {
            std::memcpy(dst, parts.payload, parts.payload_len);
            dst += parts.payload_len;
        }
        uint16_t wkc_le = parts.wkc;   // little-endian on the wire
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        wkc_le = static_cast<uint16_t>((parts.wkc >> 8) | (parts.wkc << 8));
#endif
        std::memcpy(dst, &wkc_le, sizeof(wkc_le));
        return txCommitFrame(total);
    }

    // ---- RX ------------------------------------------------------------

    /**
     * @brief Collect pending frames.
     *
     * @param views      output array
     * @param max_views  capacity of views
     * @param timeout_ns max wait for the first frame (0 = non-blocking)
     * @return number of views filled, or <0 on error
     *
     * Returned buffers may be recycled by the next rxPoll() unless held via
     * rxHold().  Consumers keeping payload pointers across polls must hold.
     */
    virtual int  rxPoll(CyclicFrameView* views, int max_views,
                        uint32_t timeout_ns) = 0;

    /**
     * @brief Cheap non-syscall check: is at least one frame pending?
     *
     * Ring backend: scans slot headers (memory reads — used for spin waits
     * that see a DMA write with zero syscalls and zero scheduler latency).
     * Socket backend: returns false (frames are kernel-internal until
     * recv — cannot be memory-polled).
     */
    virtual bool rxPending() const { return false; }

    /// Pin a view's buffer beyond the next rxPoll (refcount ++).
    virtual void rxHold(uint32_t cookie)   = 0;
    /// Drop one hold (refcount --; buffer frees when it reaches 0).
    virtual void rxRelease(uint32_t cookie) = 0;

    // ---- Introspection ---------------------------------------------------

    /// fd for ppoll integration, or -1 when the backend has none.
    virtual int  fd() const = 0;
    /// True when RX views point into kernel-shared ring memory.
    virtual bool zeroCopy() const = 0;
    virtual const char* backendName() const = 0;
    /**
     * @brief Number of rxPoll() calls that could not emit a received frame
     *        because every bank/slot was held or still consumed.  The
     *        datagram stays queued in the kernel — this counts deferrals,
     *        not lost frames.  0 on backends without a bounded bank.
     */
    virtual uint64_t droppedRx() const { return 0; }
    /**
     * @brief txCommitFrame() calls whose kernel kick was deferred (TX queue
     *        full — the slot stays queued and flushes on the next kick).
     *        A rising count means cyclic frames are leaving late: a
     *        deadline-relevant event worth surfacing in stats.
     */
    virtual uint64_t txDeferred() const { return 0; }
};

// ============================================================================
// Configuration / factory
// ============================================================================

struct CyclicChannelConfig {
    int      ifindex = 0;              ///< required — interface to bind
    CyclicWireMode wire_mode = CyclicWireMode::Auto;

    uint32_t rx_ring_blocks = 128;     ///< PACKET_RX_RING blocks (4 KiB each)
    uint32_t tx_ring_blocks = 16;      ///< PACKET_TX_RING blocks

    /**
     * @brief Spin window applied inside rxPoll() before falling back to a
     *        blocking wait: the ring backend busy-polls slot memory for up
     *        to this long — the frame is visible the instant the NIC's DMA
     *        writes it, with no syscall and no scheduler wake.  0 disables.
     *        Socket backend ignores this (kernel-internal frames cannot be
     *        memory-polled).  Keep the window well under the RX budget.
     */
    uint32_t rx_spin_ns = 0;

    /**
     * @brief Prototype: use TPACKET_V3 block-mode RX instead of V2
     *        per-frame slots.  V3 retires whole blocks in one write —
     *        fewer retire transactions under burst load, at the cost of
     *        block-granular hold semantics (rxHold pins the frame's whole
     *        block until released).  TX stays V2-format (V3 is RX-only).
     *        Default off: V2 keeps finer per-slot recycling; V3 exists for
     *        profiling comparison under high-RX-rate workloads (Q17).
     */
    bool     rx_tpacket_v3 = false;
    uint32_t rx_v3_retire_us = 10'000;  ///< tp_retire_blk_tov (µs)

    /// Max frame bytes the channel must carry (Ethernet header + payload,
    /// excl. FCS).  Default 1600 covers standard frames; raise to match
    /// Master::Config::max_frame_size on jumbo links so ring slots and the
    /// socket staging buffer can hold a full frame.
    uint32_t frame_size = 1600;

    int      async_fd = -1;            ///< async socket (socket B) — the mirror
                                       ///< BPF is attached here when >= 0

    /**
     * @brief Optional caller-built demux programs, replacing the built-in
     *        VLAN-aware accept/mirror programs.
     *
     * Needed when an encapsulation filter (VID range, UDP) must compose
     * with the cyclic-idx demux: SO_ATTACH_FILTER replaces rather than
     * stacks, so the encap clause and the idx clause must live in ONE
     * generated program (see CBPFSpec::first_idx_range).  Pointers need
     * only outlive the createCyclicChannel() call.
     */
    const CBPFInsn* accept_prog = nullptr;  ///< cyclic socket's program
    size_t          accept_prog_len = 0;
    const CBPFInsn* async_prog  = nullptr;  ///< async socket's program
    size_t          async_prog_len  = 0;
};

/**
 * @brief Create the platform cyclic channel.
 *
 * Returns nullptr when no cyclic socket can be created at all (missing
 * CAP_NET_RAW, bad ifindex) — the caller then keeps the software deposit path.
 * WireMode::Auto falls back from ring to socket with a log line.
 */
std::unique_ptr<ICyclicChannel> createCyclicChannel(
    const CyclicChannelConfig& cfg);

/**
 * @brief Attach the cyclic-idx accept filter to an AF_PACKET socket (Linux).
 * @return false on unsupported platform / setsockopt failure.
 */
bool cyclicChannelAttachCyclicFilter(int fd);

/**
 * @brief Attach the mirror filter (drop cyclic-idx frames) to an AF_PACKET
 *        socket — used on the async socket (socket B).
 */
bool cyclicChannelAttachAsyncFilter(int fd);

// ============================================================================
// BPF program access (for tests and embedders that want to attach themselves)
// ============================================================================

/**
 * @brief One classic-BPF instruction — byte-identical layout to Linux
 *        `struct sock_filter` {u16 code; u8 jt; u8 jf; u32 k}.
 */
struct CyclicBpfInsn {
    uint16_t code;
    uint8_t  jt;
    uint8_t  jf;
    uint32_t k;
};
static_assert(sizeof(CyclicBpfInsn) == 8, "must match struct sock_filter");

/// Number of instructions in each demux filter program (VLAN-aware
/// variant — see cyclicChannelBpfProgram).
inline constexpr size_t kCyclicBpfInsnCount = 13;

/**
 * @brief Build the kernel demux filter program.
 * @param accept_cyclic  true → socket-A program (accept iff first-datagram
 *        idx ∈ [0xF8,0xFD]); false → socket-B mirror.
 * @return instructions written, or 0 if @p cap is too small.
 */
size_t cyclicChannelBpfProgram(bool accept_cyclic,
                               CyclicBpfInsn* out, size_t cap);

/**
 * @brief Test/embedding seam: build a socket-backend channel over a
 *        caller-provided datagram fd.
 *
 * Production callers should use createCyclicChannel().  The fd must be
 * nonblocking and, for real traffic, a bound AF_PACKET socket — tests may
 * pass AF_UNIX/AF_INET datagram sockets to exercise the RX bank and
 * hold/release machinery without CAP_NET_RAW.
 *
 * @param fd       owned by the channel on success (closed on destruction)
 * @param ifindex  used for sockaddr_ll addressing on TX
 */
std::unique_ptr<ICyclicChannel> createCyclicSocketChannelForFd(
    int fd, int ifindex);

/**
 * @brief Same seam for the PACKET_MMAP ring backend — the fd must be a
 *        bound AF_PACKET socket; returns nullptr when ring setup fails
 *        (no CAP_NET_RAW, kernel without TPACKET_V2, OOM on mmap).
 */
std::unique_ptr<ICyclicChannel> createCyclicRingChannelForFd(
    int fd, int ifindex, uint32_t rx_ring_blocks = 128,
    uint32_t tx_ring_blocks = 16, uint32_t rx_spin_ns = 0);

/**
 * @brief Test seam: ring backend over caller-provided ring buffers.
 *
 * `rx_ring`/`tx_ring` point to caller-owned memory laid out as
 * `tpacket2_hdr`-sized frames (`frame_size` bytes per frame, `frames`
 * slots).  The channel borrows the memory — it is never unmapped or
 * freed.  `fd` is used only for the TX kick (a datagram socketpair fd
 * suffices) and for `fd()`/ppoll wake-ups.  Lets tests drive the
 * walk/hold/cursor machinery without CAP_NET_RAW.  Returns nullptr on
 * invalid geometry or non-Linux builds.
 */
std::unique_ptr<ICyclicChannel> createCyclicRingChannelForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t rx_frame_size, uint32_t rx_frames,
    void* tx_ring, uint32_t tx_frame_size, uint32_t tx_frames,
    uint32_t rx_spin_ns = 0);

/**
 * @brief Test seam: TPACKET_V3 block-mode RX over caller memory.
 *
 * `rx_ring` is `blocks` blocks of `block_size` bytes, each beginning with
 * a `tpacket_block_desc` — tests synthesize block headers + tpacket3_hdr
 * frame chains to exercise the V3 walk/retire path without CAP_NET_RAW.
 * TX falls back to the socket path.  Returns nullptr off-Linux.
 */
std::unique_ptr<ICyclicChannel> createCyclicRingChannelV3ForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t block_size, uint32_t blocks);

/**
 * @brief Zero-copy view of a published cyclic-slot response payload.
 *
 * `payload` points either into the slot's inline buffer (software deposit
 * path) or into channel-owned ring/bank memory (view path — `channel` and
 * `cookie` identify the held region).  The view stays valid until the next
 * publish on that slot.
 */
struct CyclicSlotView {
    const uint8_t*  payload = nullptr;
    uint16_t        datalen = 0;
    uint16_t        wkc     = 0;
    uint8_t         cmd     = 0;
    uint16_t        adp     = 0;
    uint16_t        ado     = 0;
    /// Channel cookie + owner for held views; nullptr/0 for inline copies.
    uint32_t        cookie  = 0;
    ICyclicChannel* channel = nullptr;
    /// Frame arrival timestamp (kernel stamp when available, else the
    /// deposit's monotonic now).
    uint64_t        stamp_ns = 0;
    /// Send-generation bit echoed from the datagram's lenFlags reserved
    /// bit 13 — distinguishes a fresh response from a stale deposit that
    /// survived a timed-out cycle (stale-deposit ABA guard).
    uint8_t         gen     = 0;
};

} // namespace EtherCAT
