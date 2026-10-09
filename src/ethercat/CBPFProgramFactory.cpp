/**
 * @file CBPFProgramFactory.cpp
 * @brief cBPF program generation, kernel attachment, and interpretation.
 *
 * Frame layouts the generated programs parse:
 *
 *   untagged EtherCAT : [eth 14][EtherType 0x88A4][...]
 *   tagged EtherCAT   : [eth 14][TPID 0x8100][TCI][EtherType 0x88A4][...]
 *   untagged UDP      : [eth 14][EtherType 0x0800][IPv4 ihl][UDP][...]
 *   tagged UDP        : [eth 14][TPID][TCI][EtherType 0x0800][IPv4 ihl][UDP]
 *
 * Note: on the RX path the kernel removes the 802.1Q tag before packet
 * sockets see the frame (rx-vlan-offload / generic untag).  The data then
 * shows the INNER EtherType at [12] and the tag is only in skb auxdata —
 * programs generated for VLAN mode therefore also contain an
 * SKF_AD_VLAN_TAG_PRESENT / SKF_AD_VLAN_TAG leg that validates the stripped
 * tag.  Inline-tag legs remain for self-TX copies (PACKET_OUTGOING).
 *
 * All loads that would run past the end of the packet fault in the kernel
 * interpreter and reject the packet — truncated frames need no explicit
 * length checks.
 */

#include "tether/ethercat/CBPFProgramFactory.hpp"

#include <cstring>
#include <unordered_map>

#if defined(__linux__)
#include <errno.h>
#include <linux/filter.h>
#include <sys/socket.h>
#endif

namespace EtherCAT {

namespace {

// ============================================================================
// Assembler — emits instructions with symbolic labels, resolves jump offsets
// in a final pass.  JT/JF offsets are u8 (max 255); JA uses the k field.
// ============================================================================

class Asm {
public:
    int label() { return next_label_++; }
    void mark(int l) { labels_[l] = insns_.size(); }

    void stmt(uint16_t code, uint32_t k) {
        insns_.push_back({code, 0, 0, k});
    }

    /// Conditional jump: op is JEQ/JGT/JGE/JSET (combined JMP|op|K).
    void jump(uint16_t code, uint32_t k, int jt, int jf) {
        fixups_.push_back({insns_.size(), jt, Field::JT});
        fixups_.push_back({insns_.size(), jf, Field::JF});
        insns_.push_back({code, 0, 0, k});
    }

    /// Unconditional jump (offset in k, relative to the next instruction).
    void ja(int target) {
        fixups_.push_back({insns_.size(), target, Field::K});
        insns_.push_back({cbpf::JMP | cbpf::JA | cbpf::K, 0, 0, 0});
    }

    std::vector<CBPFInsn> finish() {
        for (const Fixup& f : fixups_) {
            const auto it = labels_.find(f.label);
            if (it == labels_.end()) return {};      // unmarked label — bug
            const long off = static_cast<long>(it->second) -
                             static_cast<long>(f.at) - 1;
            if (off < 0) return {};                  // cBPF only jumps forward
            if (f.field == Field::K) {
                insns_[f.at].k = static_cast<uint32_t>(off);
            } else {
                if (off > 255) return {};            // jt/jf are 8-bit
                (f.field == Field::JT ? insns_[f.at].jt : insns_[f.at].jf) =
                    static_cast<uint8_t>(off);
            }
        }
        return insns_;
    }

private:
    enum class Field : uint8_t { JT, JF, K };
    struct Fixup { size_t at; int label; Field field; };

    std::vector<CBPFInsn> insns_;
    std::vector<Fixup>    fixups_;
    std::unordered_map<int, size_t> labels_;
    int next_label_ = 0;
};

// ============================================================================
// Wire offsets / constants
// ============================================================================

constexpr uint32_t kEthTypeOff      = 12;
constexpr uint32_t kVlanTciOff      = 14;  // VID = TCI & 0x0FFF
constexpr uint32_t kVlanInnerEthOff = 16;
constexpr uint32_t kVidMask         = 0x0FFF;
constexpr uint32_t kIpv4ProtoOff    = 9;   // relative to IP header base
constexpr uint32_t kIpv4FragOff     = 6;   // flags(3) + fragment offset(13)
constexpr uint32_t kFragOffsetMask  = 0x1FFF;
constexpr uint32_t kUdpDstPortOff   = 2;   // relative to UDP header base
constexpr uint32_t kProtoUdp        = 17;
constexpr uint32_t kAccept          = 0xFFFFFFFFu;

/**
 * Emit the IPv4/UDP check for an IP header at absolute offset `base`:
 * proto==UDP, IHL>=5, first fragment, dst port == `port`.
 */
/// Wire offsets of the first datagram's idx byte.
constexpr uint32_t kFirstIdxOff       = 17;  // untagged EtherCAT
constexpr uint32_t kFirstIdxOffTagged = 21;  // inline 802.1Q tag

/**
 * Emit a first-datagram-idx range check for a frame whose datagram idx
 * sits at absolute offset `off`.  Accepts the frame when the idx is
 * inside/outside the spec range per `exclude`.
 */
void emitIdxCheck(Asm& a, const CBPFSpec& spec, uint32_t off,
                  bool exclude, int l_accept, int l_reject) {
    const auto& r = *spec.first_idx_range;
    int c;
    a.stmt(cbpf::LD | cbpf::B | cbpf::ABS, off);
    c = a.label();
    a.jump(cbpf::JMP | cbpf::JGE | cbpf::K, r.start, c,
           exclude ? l_accept : l_reject);   // idx < lo
    a.mark(c);
    c = a.label();
    a.jump(cbpf::JMP | cbpf::JGT | cbpf::K, r.end,
           exclude ? l_accept : l_reject, c); // idx > hi
    a.mark(c);
    if (r.hole) {
        // In-range but hole → the out-of-range verdict (0xFE stays on
        // the async socket under the socket-pair demux).
        c = a.label();
        a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, *r.hole,
               exclude ? l_accept : l_reject, c);
        a.mark(c);
    }
    a.ja(exclude ? l_reject : l_accept);
}

void emitUdpCheck(Asm& a, uint32_t base, uint32_t port,
                  int l_accept, int l_reject) {
    int c;
    a.stmt(cbpf::LD | cbpf::B | cbpf::ABS, base + kIpv4ProtoOff);
    c = a.label();
    a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kProtoUdp, c, l_reject);
    a.mark(c);
    a.stmt(cbpf::LD | cbpf::B | cbpf::ABS, base);          // ver|ihl
    a.stmt(cbpf::ALU | cbpf::AND | cbpf::K, 0x0F);         // ihl
    c = a.label();
    a.jump(cbpf::JMP | cbpf::JGE | cbpf::K, 5, c, l_reject); // IHL >= 5
    a.mark(c);
    a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, base + kIpv4FragOff);
    c = a.label();
    a.jump(cbpf::JMP | cbpf::JSET | cbpf::K, kFragOffsetMask,
           l_reject, c);                                  // non-first frag → drop
    a.mark(c);
    a.stmt(cbpf::LDX | cbpf::MSH | cbpf::B, base);         // X = ihl * 4
    a.stmt(cbpf::LD | cbpf::H | cbpf::IND,
           base + kUdpDstPortOff);                         // [X + base + 2]
    a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, port, l_accept, l_reject);
}

} // namespace

// ============================================================================
// Program builders
// ============================================================================

std::vector<CBPFInsn> CBPFProgramFactory::build(const CBPFSpec& spec) {
    const bool tagged = spec.tagged_ethercat || spec.tagged_udp;
    if (!spec.untagged_ethercat && !spec.untagged_udp && !tagged)
        return {};   // spec accepts nothing

    // Delivery hint: prune the legs that cannot execute on this NIC.
    //   inline_leg  — [12] ever carries a tag TPID (no kernel stripping)
    //   auxdata_leg — SKF_AD_VLAN_* loads can ever report a stripped tag
    const bool inline_leg =
        tagged && spec.vlan_delivery != VlanDeliveryHint::StrippedOnly;
    const bool auxdata_leg =
        tagged && spec.vlan_delivery != VlanDeliveryHint::InlineOnly;

    Asm a;
    const int l_accept = a.label(), l_reject = a.label();
    const int l_tag    = a.label();   // inline 802.1Q header at [12]
    const int l_tag_inner = a.label();// VID ok — check inner EtherType
    const int l_ecat   = a.label();   // EtherType 0x88A4 at [12]
    const int l_udpw   = a.label();   // EtherType IPv4 at [12]
    const int l_udp14  = a.label();   // IPv4/UDP check, IP base = 14
    const int l_tudp   = a.label();   // IPv4/UDP check, IP base = 18
    const int l_ecat_idx  = a.label();// untagged EtherCAT + first-idx check
    const int l_tecat_idx = a.label();// tagged EtherCAT + first-idx check
    const bool idx_check = spec.first_idx_range.has_value();

    int c;

    // VID comparison for the current value in A (masked already by caller).
    // Jumps to l_ok when A (a VID) lies in the spec's range.
    auto emitVidRangeCheck = [&](int l_ok, int l_rej) {
        if (spec.vlan_range) {
            const auto& r = *spec.vlan_range;
            if (r.start == r.end) {
                int cc = a.label();
                a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, r.start, cc, l_rej);
                a.mark(cc);
            } else {
                int cc = a.label();
                a.jump(cbpf::JMP | cbpf::JGE | cbpf::K, r.start, cc, l_rej);
                a.mark(cc);
                cc = a.label();
                a.jump(cbpf::JMP | cbpf::JGT | cbpf::K, r.end, l_rej, cc);
                a.mark(cc);
            }
        }
        a.ja(l_ok);
    };

    // --- EtherType dispatch on [12] ----------------------------------------
    //
    // [12] carries the *visible* EtherType: the outer TPID when a tag is
    // still inline (self-TX copies), otherwise the inner EtherType — both
    // for genuinely untagged frames and for frames whose tag the kernel
    // stripped into skb auxdata on RX.  The two wire legs below consult
    // SKF_AD_VLAN_TAG_* to tell those apart.
    a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, kEthTypeOff);
    if (inline_leg) {
        c = a.label();
        a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, spec.vlan_tpid, l_tag, c);
        a.mark(c);
    }
    if (spec.untagged_ethercat || spec.tagged_ethercat) {
        c = a.label();
        a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kEtherTypeEtherCAT,
               l_ecat, c);
        a.mark(c);
    }
    if (spec.untagged_udp || spec.tagged_udp) {
        c = a.label();
        a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kEtherTypeIPv4, l_udpw, c);
        a.mark(c);
    }
    a.ja(l_reject);

    // --- inline 802.1Q path -------------------------------------------------
    if (inline_leg) {
        a.mark(l_tag);
        a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, kVlanTciOff);
        a.stmt(cbpf::ALU | cbpf::AND | cbpf::K, kVidMask);   // A = VID
        emitVidRangeCheck(l_tag_inner, l_reject);
        a.mark(l_tag_inner);
        a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, kVlanInnerEthOff);
        if (spec.tagged_ethercat) {
            c = a.label();
            a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kEtherTypeEtherCAT,
                   idx_check ? l_tecat_idx : l_accept, c);
            a.mark(c);
        }
        if (spec.tagged_udp) {
            a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kEtherTypeIPv4,
                   l_tudp, l_reject);
        } else {
            a.ja(l_reject);
        }
    }

    // --- wire legs: inner EtherType visible at [12] -------------------------
    //
    // emitWireLeg(l, unt_ok, tag_ok, l_handler): a frame showing the inner
    // EtherType at [12] may be untagged or carry a kernel-stripped tag.
    // Consult SKF_AD_VLAN_TAG_PRESENT / SKF_AD_VLAN_TAG (same mechanism as
    // libpcap's `vlan` primitive) to decide which spec leg applies.
    auto emitWireLeg = [&](int l_wire, bool unt_ok, bool tag_ok,
                           int l_handler) {
        a.mark(l_wire);
        if (unt_ok && tag_ok && !spec.vlan_range) {
            a.ja(l_handler);        // every tag state is acceptable
            return;
        }
        if (!auxdata_leg) {
            // Kernel never strips: a wire leg frame is genuinely untagged.
            a.ja(unt_ok ? l_handler : l_reject);
            return;
        }
        const int l_notag = a.label();
        a.stmt(cbpf::LD | cbpf::W | cbpf::ABS, kSkfAdVlanTagPresent);
        c = a.label();
        a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, 0, l_notag, c);
        a.mark(c);
        // Tag present (stripped): the tagged rules apply.
        if (!tag_ok) {
            a.ja(l_reject);
        } else {
            a.stmt(cbpf::LD | cbpf::W | cbpf::ABS, kSkfAdVlanTag);
            a.stmt(cbpf::ALU | cbpf::AND | cbpf::K, kVidMask);
            emitVidRangeCheck(l_handler, l_reject);
        }
        a.mark(l_notag);
        a.ja(unt_ok ? l_handler : l_reject);
    };

    if (spec.untagged_ethercat || spec.tagged_ethercat)
        emitWireLeg(l_ecat, spec.untagged_ethercat, spec.tagged_ethercat,
                    idx_check ? l_ecat_idx : l_accept);
    if (spec.untagged_udp || spec.tagged_udp)
        emitWireLeg(l_udpw, spec.untagged_udp, spec.tagged_udp, l_udp14);

    // --- first-datagram-idx legs (direct EtherCAT only) -------------------
    //
    // A stripped-tag frame carries the untagged layout in the data buffer,
    // so the wire leg's EtherType accept funnels into the offset-17 check;
    // an inline tag reaches the offset-21 check instead.
    if (idx_check) {
        if (spec.untagged_ethercat || spec.tagged_ethercat) {
            a.mark(l_ecat_idx);
            emitIdxCheck(a, spec, kFirstIdxOff, spec.first_idx_exclude,
                         l_accept, l_reject);
        }
        if (inline_leg && spec.tagged_ethercat) {
            a.mark(l_tecat_idx);
            emitIdxCheck(a, spec, kFirstIdxOffTagged,
                         spec.first_idx_exclude, l_accept, l_reject);
        }
    }

    // --- UDP payload checks --------------------------------------------------
    if (spec.untagged_udp || spec.tagged_udp) {
        a.mark(l_udp14);
        emitUdpCheck(a, 14, spec.udp_port, l_accept, l_reject);
    }
    if (inline_leg && spec.tagged_udp) {
        a.mark(l_tudp);
        emitUdpCheck(a, 18, spec.udp_port, l_accept, l_reject);
    }

    // --- verdicts -----------------------------------------------------------
    a.mark(l_accept);
    a.stmt(cbpf::RET | cbpf::K, kAccept);
    a.mark(l_reject);
    a.stmt(cbpf::RET | cbpf::K, 0);

    return a.finish();
}

std::vector<CBPFInsn> CBPFProgramFactory::ethercatFilter() {
    return build(CBPFSpec{});
}

std::vector<CBPFInsn> CBPFProgramFactory::ethercatFilterWithUdp(uint16_t port) {
    CBPFSpec s;
    s.untagged_udp = true;
    s.tagged_udp   = true;
    s.udp_port     = port;
    return build(s);
}

std::vector<CBPFInsn> CBPFProgramFactory::vlanFilter(uint16_t vid) {
    if (vid > 4095) return {};
    return vlanRangeFilter(vid, vid);
}

std::vector<CBPFInsn> CBPFProgramFactory::vlanRangeFilter(uint16_t lo,
                                                        uint16_t hi) {
    if (lo > hi || hi > 4095) return {};
    CBPFSpec s;
    s.untagged_ethercat = false;   // VLAN mode rejects untagged (documented)
    s.tagged_ethercat   = true;
    s.vlan_range        = CBPFVlanRange{lo, hi};
    return build(s);
}

// ============================================================================
// Kernel attachment (Linux)
// ============================================================================

bool CBPFProgramFactory::attach(int fd, const CBPFInsn* prog, size_t count,
                                bool lock) {
#if defined(__linux__)
    if (!prog || count == 0 || count > 0xFFFF) return false;
    static_assert(sizeof(CBPFInsn) == sizeof(struct sock_filter),
                  "CBPFInsn must be layout-compatible with sock_filter");
    struct sock_fprog fp;
    fp.len    = static_cast<unsigned short>(count);
    fp.filter = reinterpret_cast<struct sock_filter*>(
                    const_cast<CBPFInsn*>(prog));
    if (::setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &fp, sizeof(fp)) < 0)
        return false;
    if (lock) {
        int one = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_LOCK_FILTER, &one, sizeof(one));
    }
    return true;
#else
    (void)fd; (void)prog; (void)count; (void)lock;
    return false;
#endif
}

} // namespace EtherCAT
