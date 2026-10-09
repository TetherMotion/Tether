/**
 * @file EtherCATCommandTypes.hpp
 * @brief EtherCAT types: EtherCAT Command Types
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// EtherCAT Command Types
// ============================================================================

/**
 * @brief EtherCAT datagram command types
 */
enum class Command : uint8_t {
    NOP  = 0x00,  ///< No operation
    APRD = 0x01,  ///< Auto Position Read
    APWR = 0x02,  ///< Auto Position Write
    APRW = 0x03,  ///< Auto Position Read/Write
    FPRD = 0x04,  ///< Configured Address Read
    FPWR = 0x05,  ///< Configured Address Write
    FPRW = 0x06,  ///< Configured Address Read/Write
    BRD  = 0x07,  ///< Broadcast Read
    BWR  = 0x08,  ///< Broadcast Write
    BRW  = 0x09,  ///< Broadcast Read/Write
    LRD  = 0x0A,  ///< Logical Read
    LWR  = 0x0B,  ///< Logical Write
    LRW  = 0x0C,  ///< Logical Read/Write
    ARMW = 0x0D,  ///< Auto Read Multiple Write
    FRMW = 0x0E,  ///< Configured Read Multiple Write
};

/**
 * @brief Convert command to string name
 */
inline const char* commandToString(Command cmd) {
    switch (cmd) {
        case Command::NOP:  return "NOP";
        case Command::APRD: return "APRD";
        case Command::APWR: return "APWR";
        case Command::APRW: return "APRW";
        case Command::FPRD: return "FPRD";
        case Command::FPWR: return "FPWR";
        case Command::FPRW: return "FPRW";
        case Command::BRD:  return "BRD";
        case Command::BWR:  return "BWR";
        case Command::BRW:  return "BRW";
        case Command::LRD:  return "LRD";
        case Command::LWR:  return "LWR";
        case Command::LRW:  return "LRW";
        case Command::ARMW: return "ARMW";
        case Command::FRMW: return "FRMW";
        default: return "UNK";
    }
}

/**
 * @brief Check if command is a read operation
 */
inline bool isReadCommand(Command cmd) {
    return cmd == Command::APRD || cmd == Command::FPRD || 
           cmd == Command::BRD || cmd == Command::LRD ||
           cmd == Command::APRW || cmd == Command::FPRW ||
           cmd == Command::BRW || cmd == Command::LRW;
}

/**
 * @brief Check if command is a write operation
 */
inline bool isWriteCommand(Command cmd) {
    return cmd == Command::APWR || cmd == Command::FPWR || 
           cmd == Command::BWR || cmd == Command::LWR ||
           cmd == Command::APRW || cmd == Command::FPRW ||
           cmd == Command::BRW || cmd == Command::LRW;
}

/**
 * @brief Received datagram structure
 * 
 * Passed through the RX queue when a response is received.
 */
struct RxDatagram {
    uint8_t idx;              ///< Matching index from request
    Command cmd;              ///< Command type
    uint16_t adp;             ///< Address position
    uint16_t ado;             ///< Address offset
    uint16_t datalen;         ///< Data length
    uint16_t wkc;             ///< Working Counter (incremented by each slave that processes)
    uint8_t data[kMaxDatagramDataSize]; ///< Payload (max single-datagram size)
};

/**
 * @brief Reserved datagram-index band for the cyclic fast path.
 *
 * The whole band [kFastSlotBaseIdx, kFastSlotEndIdx] = 0x9C..0xFF (100
 * indices) is reserved for cyclic execution: async allocIdx() never
 * returns an index >= kFastSlotBaseIdx.  Datagrams sent on a band index
 * are deposited into fixed per-index mailboxes by the RX parser instead
 * of going through TransactionRouter — no mutex, no condition variable
 * on the cyclic hot path.
 *
 * Band layout:
 *   0x9C..0xDF  rotating cyclic-request pool, positions 0..67
 *   0xE0..0xEF  user-defined PDO-slice slots (fixed per slice)
 *   0xF0..0xFD  rotating cyclic-request pool, positions 68..81
 *   0xFE        fire-and-forget (reserved — no mailbox)
 *   0xFF        dedicated DC-timepoint datagram index (pool position 82)
 *
 * The rotating pool gives the cyclic LRW exchange up to
 * kNumCyclicSlots (82) requests in flight: each send draws the next
 * pool position, and a position's pending request is only cleared when
 * that same position is re-armed — late responses always land in their
 * own mailbox.  Combined with the 64-bit counter trailer on the last
 * LRW datagram of a cycle, a stale echo can never silently satisfy a
 * re-armed request.
 */
inline constexpr uint8_t kFastSlotBaseIdx   = 0x9C;
inline constexpr uint8_t kFastSlotEndIdx    = 0xFF;
inline constexpr size_t  kNumFastSlots      = 100;   ///< slots 0x9C..0xFF

inline constexpr uint8_t kSliceSlotBaseIdx  = 0xE0;
inline constexpr size_t  kNumSliceSlots     = 16;    ///< slots 0xE0..0xEF

/// Wire index of the dedicated DC-timepoint datagram (pool pos 82).
inline constexpr uint8_t kDcTimeIdx         = 0xFF;
/// Fire-and-forget wire index — inside the band but owns no mailbox.
inline constexpr uint8_t kFastSlotReservedIdx = 0xFE;

/// Rotating cyclic-request pool depth (positions 0..81).
inline constexpr size_t  kNumCyclicSlots    = 82;
/// Pool position pinned for the DC-timepoint datagram (wire 0xFF).
inline constexpr uint8_t kCyclicDcPoolPos   = 82;
/// Back-compat alias: the cyclic band starts at the pool base.
inline constexpr uint8_t kCyclicSlotBaseIdx = kFastSlotBaseIdx;

/// Rotating pool position -> wire index (pos 82 = the DC index).
inline constexpr uint8_t cyclicPoolWireIdx(uint8_t pos) {
    if (pos < 68) return static_cast<uint8_t>(0x9C + pos);   // 0x9C..0xDF
    if (pos < 82) return static_cast<uint8_t>(0xF0 + pos - 68); // 0xF0..0xFD
    return kDcTimeIdx;                                       // pos 82
}

/// Wire index -> rotating pool position; kCyclicDcPoolPos for the DC
/// index and 0xFF for slice/fire-and-forget indices (not pool members).
inline constexpr uint8_t cyclicWirePoolPos(uint8_t idx) {
    if (idx >= 0x9C && idx <= 0xDF) return static_cast<uint8_t>(idx - 0x9C);
    if (idx >= 0xF0 && idx <= 0xFD)
        return static_cast<uint8_t>(68 + (idx - 0xF0));
    if (idx == kDcTimeIdx) return kCyclicDcPoolPos;
    return 0xFF;
}

/// 8-byte monotonically increasing counter appended to the last LRW
/// datagram of a cyclic exchange (lands on unmapped logical space past
/// the image end, so slaves pass it through verbatim).  Verified on the
/// response — a stale echo carries an older counter and is rejected.
inline constexpr uint16_t kLrwCounterTrailerBytes = 8;

/// True when @p idx lies in the reserved fastpath band [0x9C, 0xFF]
/// minus the 0xFE fire-and-forget index (its echoes belong to the async
/// parser path — txpdo_rx_queue_ consumers).
inline constexpr bool isFastPathIdx(uint8_t idx) {
    return idx >= kFastSlotBaseIdx && idx <= kFastSlotEndIdx &&
           idx != kFastSlotReservedIdx;
}
/// True when @p idx is a user-slice index (0xE0..0xEF).
inline constexpr bool isSliceIdx(uint8_t idx) {
    return idx >= kSliceSlotBaseIdx &&
           idx <  kSliceSlotBaseIdx + static_cast<uint8_t>(kNumSliceSlots);
}
/// True when @p idx is a cyclic-pool or DC index — everything in the
/// band except the slice range and 0xFE.
inline constexpr bool isCyclicIdx(uint8_t idx) {
    return isFastPathIdx(idx) && !isSliceIdx(idx);
}
/// True when @p idx maps to a real fastpath mailbox slot.
inline constexpr bool isSlotIdx(uint8_t idx) {
    return isFastPathIdx(idx);
}

/**
 * @brief One datagram in a multi-datagram cyclic fastpath frame.
 *
 * `idx` is the WIRE datagram index — a rotating-pool index via
 * cyclicPoolWireIdx(), a PDO-slice index (0xE0+slot), or the dedicated
 * kDcTimeIdx.  The datagram covers `datalen + tail_len` bytes: `data`
 * followed by `tail` (e.g. the 64-bit LRW counter trailer — appended
 * without requiring the source buffer to over-allocate).
 */
struct CyclicDgramSpec {
    Command      cmd{Command::NOP};
    uint8_t      idx{0};
    uint16_t     adp{0};
    uint16_t     ado{0};
    const void*  data{nullptr};
    uint16_t     datalen{0};
    const void*  tail{nullptr};
    uint16_t     tail_len{0};
    bool         roundtrip{true};
    /// Stamp the per-slot send-generation bit (lenFlags res-bit 13) —
    /// the echoed bit lets collect reject deposits of an older send.
    bool         stamp_gen{false};
};

/**
 * @brief Outcome of the most recent collect of one cyclic datagram.
 *
 * Distinguishes the failure classes a "cycle failed" bool cannot carry:
 * a datagram that never came back (Timeout — wire loss / dead slave /
 * filter killed the reply) is a different fault from one that returned
 * with a bad working counter (WkcError — slave dropped out of OP, FMMU
 * hole, partial processing) or from only a previous-generation echo
 * arriving (Stale — a reply that survived its own cycle's deadline).
 */
enum class CyclicSliceStatus : uint8_t {
    None,      ///< No collect has run yet
    Ok,        ///< Reply in time, WKC valid
    Timeout,   ///< No reply before the cycle deadline — packet didn't circulate
    WkcError,  ///< Reply arrived but WKC == 0 or != expected
    Stale,     ///< Only a previous-generation echo arrived this cycle
    SendError, ///< The datagram could not be sent this cycle
};

/// Per-slice cyclic health snapshot — written by the cyclic thread during
/// collect, read by diagnostic callers.  The runtime stores it packed in
/// a single atomic word (pack()/unpack()) so readers never see torn
/// fields.
struct CyclicSliceHealth {
    uint16_t expected_wkc = 0xFFFF;  ///< LogicalAddressManager::kWkcUnknown
    uint16_t last_wkc     = 0;       ///< WKC of the last reply that arrived
    CyclicSliceStatus last_status = CyclicSliceStatus::None;
    /// Consecutive non-Ok cycles on this slice — a slowly flapping slave
    /// shows up here even when cumulative counters look fine.
    uint32_t consecutive_failures = 0;

    /// Pack the whole snapshot into a single word so the cyclic thread
    /// can publish it with one atomic store and readers never see a torn
    /// mix of fields.  Layout: expected[0:16) last[16:32) status[32:40)
    /// failures[40:64) — failures saturate at 2^24-1.
    uint64_t pack() const {
        const uint64_t f = consecutive_failures > 0xFFFFFFu
                               ? 0xFFFFFFu : consecutive_failures;
        return uint64_t(expected_wkc) |
               (uint64_t(last_wkc) << 16) |
               (uint64_t(static_cast<uint8_t>(last_status)) << 32) |
               (f << 40);
    }
    static CyclicSliceHealth unpack(uint64_t v) {
        CyclicSliceHealth h;
        h.expected_wkc = static_cast<uint16_t>(v);
        h.last_wkc     = static_cast<uint16_t>(v >> 16);
        h.last_status  = static_cast<CyclicSliceStatus>(
                             static_cast<uint8_t>(v >> 32));
        h.consecutive_failures = static_cast<uint32_t>(v >> 40);
        return h;
    }
};

/// Short stable names for logs — same style as toString(VlanTagDelivery).
inline const char* toString(CyclicSliceStatus s) {
    switch (s) {
    case CyclicSliceStatus::Ok:        return "ok";
    case CyclicSliceStatus::Timeout:   return "timeout";
    case CyclicSliceStatus::WkcError:  return "wkc-err";
    case CyclicSliceStatus::Stale:     return "stale";
    case CyclicSliceStatus::SendError: return "send-err";
    case CyclicSliceStatus::None:
    default:                           return "none";
    }
}

/**
 * @brief How the kernel delivers 802.1Q tags to packet sockets on this
 *        NIC/driver — detected at startup by probeVlanTagDelivery().
 *
 * Used to prune dead legs from generated cBPF programs: a NIC that always
 * strips the tag into skb auxdata never produces the inline layout (and
 * vice versa), so half the generated program is unreachable on any given
 * machine.
 */
enum class VlanDeliveryHint : uint8_t {
    Auto,         ///< delivery unknown — emit legs for both (default, safe)
    StrippedOnly, ///< kernel strips tags into auxdata — no inline-tag leg
    InlineOnly,   ///< kernel keeps tags inline — no SKF_AD_VLAN_* loads
};

/**
 * @brief Wire encapsulation descriptor for the cyclic datapath.
 *
 * A distilled copy of the application's encapsulation settings (the
 * example-layer EncapsulationConfig is not visible to the master):
 * which VLAN tag — if any — cyclic TX headers must carry inline, and
 * which VID range the socket-level demux filter must accept.  The
 * datapath bakes the TX tag into its per-index header templates so a
 * tagged cyclic send is one contiguous header memcpy + sendmsg — the
 * router's per-frame rebuild never runs on the cyclic path.
 */
struct WireEncap {
    uint16_t tx_vlan     = 0;     ///< VID to insert on TX (0 = untagged)
    uint16_t rx_vlan_lo  = 0;     ///< Accepted RX VID range; 0/0 = untagged
    uint16_t rx_vlan_hi  = 0;
    bool     rx_vlan_any = false; ///< Accept any tagged EtherCAT frame

    /// Detected tag delivery for this NIC — set from probeVlanTagDelivery.
    /// Shrinks the composed cyclic/async demux programs by dropping the
    /// leg that can never execute.
    VlanDeliveryHint delivery_hint = VlanDeliveryHint::Auto;

    /// Measured wire round-trip (ns) from the startup probe — 0 when
    /// unmeasured.  startCyclicLoop() auto-sizes the collect deadline
    /// (CyclicLoopConfig::rx_budget_ns == 0) from this with headroom, and
    /// warns when it cannot fit the configured cycle period.
    uint32_t wire_rtt_ns = 0;

    /// True when tagged frames are part of the RX filter.
    bool tagged()   const { return rx_vlan_any || rx_vlan_lo != 0; }
    /// True when the link is untagged EtherCAT (no VLAN filtering).
    bool untagged() const { return !tagged(); }
    /// Wire prefix length of a composed cyclic TX frame on the channel.
    uint16_t prefixLen() const { return tx_vlan ? 30 : 26; }
};

/**
 * @brief Reserved index for datagrams piggybacked inside cyclic frames
 *        (mailbox/async traffic riding the cyclic wire slot).
 *
 * The kernel BPF demux keys on the FIRST datagram's idx: a frame whose
 * first datagram carries a cyclic idx is steered to the cyclic socket
 * wholesale — piggybacked datagrams with this idx then fall out of the
 * slot range and are forwarded to the regular parser, which routes them
 * normally.  Keep LRW first; piggyback datagrams must use this idx (or
 * any non-cyclic idx) so the demux stays exact (Q14).
 */
inline constexpr uint8_t kPiggybackIdx = 0xFE;

/**
 * @brief Specification for a single datagram within a multi-datagram frame
 *
 * Used by Master::sendMultiDatagram() to pack multiple datagrams into one
 * or more Ethernet frames.  Each spec describes one datagram's command,
 * address, data, and roundtrip behavior.
 */
struct MultiDatagramSpec {
    Command     cmd;        ///< Datagram command (APRD, APWR, FPRD, FPWR, etc.)
    uint8_t     idx;        ///< Datagram index for response matching
    uint16_t    adp;        ///< Address position (slave-dependent)
    uint16_t    ado;        ///< Address offset (register address or logical low word)
    const void* data;      ///< Payload data (nullptr for read-only datagrams)
    uint16_t    datalen;    ///< Payload length in bytes
    bool        roundtrip;  ///< If true, sets the circulating flag (C bit)
};

/**
 * @brief Result of a single read operation within a batch transaction
 */
struct BatchReadResult {
    bool success{false};       ///< Whether the response was received
    uint16_t wkc{0};           ///< Working counter from the response
    uint16_t datalen{0};       ///< Actual data length received
    const uint8_t* data{nullptr};  ///< Pointer to data (valid until BatchTransaction is destroyed)
};

} // namespace EtherCAT
