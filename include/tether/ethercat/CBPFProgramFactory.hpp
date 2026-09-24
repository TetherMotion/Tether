#pragma once

/**
 * @file CBPFProgramFactory.hpp
 * @brief Classic-BPF (SO_ATTACH_FILTER) program factory for EtherCAT sockets.
 *
 * Generates small, fixed-shape kernel socket filters that reject
 * non-EtherCAT traffic before it reaches userspace:
 *
 *   - @ref ethercatFilter         untagged EtherCAT + 802.1Q-tagged EtherCAT
 *                                 (any VID)
 *   - @ref ethercatFilterWithUdp  additionally accepts EtherCAT-over-UDP
 *                                 (IPv4/UDP dst port 34980, ETG.1000.3)
 *   - @ref vlanFilter / @ref vlanRangeFilter  accept ONLY 802.1Q frames whose
 *                                 VID lies in the given range AND whose inner
 *                                 EtherType is EtherCAT.  In VLAN mode
 *                                 untagged frames — including untagged
 *                                 EtherCAT — are rejected; see
 *                                 docs/EtherCATBPFFiltering.md.
 *
 * The generated programs are portable (plain data, no Linux headers
 * required); attach() wires them to a socket via SO_ATTACH_FILTER on Linux.
 *
 * @ref cbpfExecute is a user-space interpreter implementing kernel cBPF
 * semantics — used by unit tests to run synthesized packets through
 * generated programs without CAP_NET_RAW, and usable by embedders for
 * verification.
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "tether/ethercat/Types.hpp"   // kEtherTypeEtherCAT

namespace EtherCAT {

/**
 * @brief One classic-BPF instruction — byte-identical layout to Linux
 *        `struct sock_filter` {u16 code; u8 jt; u8 jf; u32 k}.
 */
struct CBPFInsn {
    uint16_t code;
    uint8_t  jt;
    uint8_t  jf;
    uint32_t k;
};
static_assert(sizeof(CBPFInsn) == 8, "must match struct sock_filter");

/// cBPF opcode constants (Linux/BSD uapi values — stable ABI).
namespace cbpf {
// Instruction classes
inline constexpr uint16_t LD   = 0x00;
inline constexpr uint16_t LDX  = 0x01;
inline constexpr uint16_t ST   = 0x02;
inline constexpr uint16_t STX  = 0x03;
inline constexpr uint16_t ALU  = 0x04;
inline constexpr uint16_t JMP  = 0x05;
inline constexpr uint16_t RET  = 0x06;
inline constexpr uint16_t MISC = 0x07;
// Operand sizes (ld/ldx/st/stx)
inline constexpr uint16_t W    = 0x00;  ///< 32-bit word
inline constexpr uint16_t H    = 0x08;  ///< 16-bit halfword
inline constexpr uint16_t B    = 0x10;  ///< 8-bit byte
// Addressing modes
inline constexpr uint16_t IMM  = 0x00;
inline constexpr uint16_t ABS  = 0x20;
inline constexpr uint16_t IND  = 0x40;
inline constexpr uint16_t MEM  = 0x60;
inline constexpr uint16_t LEN  = 0x80;
inline constexpr uint16_t MSH  = 0xa0;
// ALU / jump operations
inline constexpr uint16_t ADD  = 0x00;
inline constexpr uint16_t SUB  = 0x10;
inline constexpr uint16_t MUL  = 0x20;
inline constexpr uint16_t DIV  = 0x30;
inline constexpr uint16_t OR   = 0x40;
inline constexpr uint16_t AND  = 0x50;
inline constexpr uint16_t LSH  = 0x60;
inline constexpr uint16_t RSH  = 0x70;
inline constexpr uint16_t NEG  = 0x80;
inline constexpr uint16_t MOD  = 0x90;
inline constexpr uint16_t XOR  = 0xa0;
inline constexpr uint16_t JA   = 0x00;
inline constexpr uint16_t JEQ  = 0x10;
inline constexpr uint16_t JGT  = 0x20;
inline constexpr uint16_t JGE  = 0x30;
inline constexpr uint16_t JSET = 0x40;
// Operand source / misc
inline constexpr uint16_t K    = 0x00;
inline constexpr uint16_t X    = 0x08;
inline constexpr uint16_t A    = 0x10;
inline constexpr uint16_t TAX  = 0x00;  ///< MISC: X = A
inline constexpr uint16_t TXA  = 0x80;  ///< MISC: A = X
} // namespace cbpf

/// EtherCAT-over-UDP destination port (ETG.1000.3) — same value as the
/// EtherCAT EtherType, 0x88A4 = 34980.
inline constexpr uint16_t kEtherCATUdpPort = 0x88A4;
/// 802.1Q VLAN tag protocol identifier.
inline constexpr uint16_t kEtherType8021Q  = 0x8100;
/// IPv4 EtherType (EtherCAT-over-UDP encapsulation).
inline constexpr uint16_t kEtherTypeIPv4   = 0x0800;

/**
 * @brief Kernel socket-filter ancillary data offsets (linux/filter.h uapi).
 *
 * On the RX path the kernel removes 802.1Q tags *before* delivering a frame
 * to packet sockets (rx-vlan-offload / the generic untag in
 * __netif_receive_skb_core): the frame data then shows the INNER EtherType
 * at [12] and the tag lives only in skb metadata.  cBPF programs read that
 * metadata through "ancillary data" loads: LD|W|ABS with k = SKF_AD_OFF+id.
 * This is the same mechanism libpcap generates for `vlan` expressions.
 */
inline constexpr uint32_t kSkfAdOff             = 0xFFFFF000u;  // SKF_AD_OFF
inline constexpr uint32_t kSkfAdVlanTag        = kSkfAdOff + 44; // TCI
inline constexpr uint32_t kSkfAdVlanTagPresent = kSkfAdOff + 48; // 0/1

/**
 * @brief Stripped-tag metadata for the user-space interpreter.
 *
 * Models what the kernel reports through SKF_AD_VLAN_* loads (equivalently
 * PACKET_AUXDATA on recvmsg): a frame whose 802.1Q tag was removed from the
 * data buffer before delivery.  Pass nullptr (default) for "no tag".
 */
struct CBPFAuxData {
    std::optional<uint16_t> vlan_tci;   ///< stripped TCI (set = tag present)
    uint16_t vlan_tpid = 0;             ///< TPID of the stripped tag
};

/// Inclusive VLAN-ID range for tagged-frame acceptance.
struct CBPFVlanRange {
    uint16_t start = 0;
    uint16_t end   = 4095;
    bool contains(uint16_t vid) const { return vid >= start && vid <= end; }
};

/**
 * @brief Declarative description of which frames a filter accepts.
 *
 * `vlan_range`, when set, restricts *all* tagged acceptance (EtherCAT and
 * UDP) to frames whose 802.1Q VID lies in the range.  Untagged acceptance
 * is independent.
 */
/// Inclusive datagram-index range (first datagram of a frame).
struct CBPFIdxRange {
    uint8_t start = 0;
    uint8_t end   = 0;
    bool contains(uint8_t idx) const { return idx >= start && idx <= end; }
};

struct CBPFSpec {
    bool untagged_ethercat = true;   ///< EtherType 0x88A4 directly
    bool untagged_udp      = false;  ///< IPv4/UDP dst `udp_port`, untagged
    bool tagged_ethercat   = true;   ///< TPID `vlan_tpid` + inner 0x88A4
    bool tagged_udp        = false;  ///< TPID `vlan_tpid` + inner IPv4/UDP
    std::optional<CBPFVlanRange> vlan_range;  ///< restrict tagged VIDs
    uint16_t udp_port  = kEtherCATUdpPort;
    uint16_t vlan_tpid = kEtherType8021Q;

    /**
     * @brief Optional constraint on the FIRST datagram's idx, applied to
     *        the direct-EtherCAT legs only (UDP-encapsulated frames are
     *        exempt — their datagrams sit behind the IP/UDP headers).
     *
     * When set, an EtherCAT frame is accepted only if its first idx
     *   - lies inside `first_idx_range`        (first_idx_exclude=false)
     *   - lies outside `first_idx_range`       (first_idx_exclude=true)
     *
     * Used to compose the cyclic demux with an encapsulation filter:
     * the cyclic socket accepts encap ∧ idx∈fastpath while the async
     * socket accepts encap ∧ idx∉fastpath — SO_ATTACH_FILTER replaces
     * rather than stacks, so both must be generated as one program.
     */
    std::optional<CBPFIdxRange> first_idx_range;
    bool first_idx_exclude = false;
};

class CBPFProgramFactory {
public:
    /**
     * @brief Build a filter program for @p spec.
     * @return instructions, or an empty vector when the spec accepts
     *         nothing (attaching an empty program would fail anyway).
     */
    static std::vector<CBPFInsn> build(const CBPFSpec& spec);

    /// Accept untagged EtherCAT and 802.1Q-tagged EtherCAT (any VID).
    static std::vector<CBPFInsn> ethercatFilter();

    /// Like ethercatFilter() but also accepts EtherCAT-over-UDP
    /// (IPv4/UDP dst @p port), tagged or untagged.
    static std::vector<CBPFInsn> ethercatFilterWithUdp(
        uint16_t port = kEtherCATUdpPort);

    /**
     * @brief VLAN-filtered mode (--encapsulation vlan:N): accept ONLY
     *        802.1Q frames with VID == @p vid whose inner EtherType is
     *        EtherCAT — whether the tag arrives inline (self-TX copies) or
     *        stripped to skb auxdata (the normal kernel RX path, where the
     *        frame data shows the INNER EtherType at [12] and the tag is
     *        only readable via SKF_AD_VLAN_* ancillary loads).
     *
     * Untagged frames — including untagged EtherCAT — and UDP-encapsulated
     * EtherCAT are rejected.  Returns an empty vector for vid > 4095.
     */
    static std::vector<CBPFInsn> vlanFilter(uint16_t vid);

    /// Range variant of vlanFilter() (--encapsulation vlan:lo-hi).
    static std::vector<CBPFInsn> vlanRangeFilter(uint16_t lo, uint16_t hi);

    /**
     * @brief Attach a program to a socket fd via SO_ATTACH_FILTER (Linux).
     *
     * Also applies SO_LOCK_FILTER (best effort) so the filter cannot be
     * detached or weakened later.
     * @return false on unsupported platform or setsockopt failure.
     */
    static bool attach(int fd, const CBPFInsn* prog, size_t count);
    static bool attach(int fd, const std::vector<CBPFInsn>& prog) {
        return attach(fd, prog.data(), prog.size());
    }
};

/**
 * @brief Execute a cBPF program over a packet buffer (kernel semantics).
 *
 * Returns the program's return value: 0 = reject, non-zero = accept
 * (the kernel interprets it as the accepted byte count).  Out-of-bounds
 * loads, division/modulo by zero, and unknown opcodes abort with 0 —
 * matching the kernel's classic-BPF interpreter.
 *
 * @param aux  Optional stripped-tag metadata — when set, SKF_AD_VLAN_*
 *             loads report it; when nullptr they report "no tag present",
 *             matching a socket that receives a genuinely untagged frame.
 */
uint32_t cbpfExecute(const CBPFInsn* prog, size_t count,
                     const uint8_t* data, size_t len,
                     const CBPFAuxData* aux = nullptr);

inline uint32_t cbpfExecute(const std::vector<CBPFInsn>& prog,
                            const uint8_t* data, size_t len,
                            const CBPFAuxData* aux = nullptr) {
    return cbpfExecute(prog.data(), prog.size(), data, len, aux);
}

} // namespace EtherCAT
