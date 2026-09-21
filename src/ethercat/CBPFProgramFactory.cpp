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

    Asm a;
    const int l_accept = a.label(), l_reject = a.label();
    const int l_tag    = a.label();   // inline 802.1Q header at [12]
    const int l_tag_inner = a.label();// VID ok — check inner EtherType
    const int l_ecat   = a.label();   // EtherType 0x88A4 at [12]
    const int l_udpw   = a.label();   // EtherType IPv4 at [12]
    const int l_udp14  = a.label();   // IPv4/UDP check, IP base = 14
    const int l_tudp   = a.label();   // IPv4/UDP check, IP base = 18

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
    if (tagged) {
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
    if (tagged) {
        a.mark(l_tag);
        a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, kVlanTciOff);
        a.stmt(cbpf::ALU | cbpf::AND | cbpf::K, kVidMask);   // A = VID
        emitVidRangeCheck(l_tag_inner, l_reject);
        a.mark(l_tag_inner);
        a.stmt(cbpf::LD | cbpf::H | cbpf::ABS, kVlanInnerEthOff);
        if (spec.tagged_ethercat) {
            c = a.label();
            a.jump(cbpf::JMP | cbpf::JEQ | cbpf::K, kEtherTypeEtherCAT,
                   l_accept, c);
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
                    l_accept);
    if (spec.untagged_udp || spec.tagged_udp)
        emitWireLeg(l_udpw, spec.untagged_udp, spec.tagged_udp, l_udp14);

    // --- UDP payload checks --------------------------------------------------
    if (spec.untagged_udp || spec.tagged_udp) {
        a.mark(l_udp14);
        emitUdpCheck(a, 14, spec.udp_port, l_accept, l_reject);
    }
    if (tagged && spec.tagged_udp) {
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

bool CBPFProgramFactory::attach(int fd, const CBPFInsn* prog, size_t count) {
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
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_LOCK_FILTER, &one, sizeof(one));
    return true;
#else
    (void)fd; (void)prog; (void)count;
    return false;
#endif
}

// ============================================================================
// User-space interpreter (kernel classic-BPF semantics)
// ============================================================================

uint32_t cbpfExecute(const CBPFInsn* prog, size_t count,
                     const uint8_t* data, size_t len,
                     const CBPFAuxData* aux) {
    uint32_t A = 0, X = 0;
    uint32_t M[16] = {};
    size_t pc = 0;

    auto mem_read = [&](uint32_t off, int size, uint32_t& out) -> bool {
        if (off > len || size > len - off) return false;
        switch (size) {
        case 1: out = data[off]; return true;
        case 2: out = static_cast<uint16_t>((data[off] << 8) | data[off + 1]);
                return true;
        case 4: out = (static_cast<uint32_t>(data[off]) << 24) |
                      (static_cast<uint32_t>(data[off + 1]) << 16) |
                      (static_cast<uint32_t>(data[off + 2]) << 8) |
                      static_cast<uint32_t>(data[off + 3]);
                return true;
        default: return false;
        }
    };

    while (pc < count) {
        const CBPFInsn& in = prog[pc];
        const uint16_t cls  = in.code & 0x07;
        const uint16_t size = in.code & 0x18;
        const uint16_t mode = in.code & 0xe0;
        const uint16_t op   = in.code & 0xf0;
        const uint16_t src  = in.code & 0x08;
        uint32_t tmp;

        switch (cls) {
        case cbpf::LD:
            switch (size) {
            case cbpf::W: case cbpf::H: case cbpf::B: break;
            default: return 0;
            }
            switch (mode) {
            case cbpf::IMM: A = in.k; break;
            case cbpf::LEN: A = static_cast<uint32_t>(len); break;
            case cbpf::ABS:
                if (in.k >= kSkfAdOff) {
                    // SKF_AD_* ancillary data (kernel skb metadata).  Only
                    // defined for LD|W|ABS; other sizes abort the program.
                    if (size != cbpf::W) return 0;
                    switch (in.k - kSkfAdOff) {
                    case 44:    // SKF_AD_VLAN_TAG: TCI or 0
                        A = (aux && aux->vlan_tci) ? *aux->vlan_tci : 0;
                        break;
                    case 48:    // SKF_AD_VLAN_TAG_PRESENT: 1 or 0
                        A = (aux && aux->vlan_tci) ? 1 : 0;
                        break;
                    default: return 0;   // unsupported field — kernel aborts
                    }
                    break;   // falls to the shared ++pc below
                }
                if (!mem_read(in.k, size == cbpf::W ? 4 : size == cbpf::H ? 2 : 1, tmp))
                    return 0;
                A = tmp; break;
            case cbpf::IND:
                if (!mem_read(in.k + X, size == cbpf::W ? 4 : size == cbpf::H ? 2 : 1, tmp))
                    return 0;
                A = tmp; break;
            case cbpf::MEM:
                if (in.k >= 16) return 0;
                A = M[in.k]; break;
            default: return 0;
            }
            ++pc; break;

        case cbpf::LDX:
            switch (mode) {
            case cbpf::IMM: X = in.k; break;
            case cbpf::LEN: X = static_cast<uint32_t>(len); break;
            case cbpf::MEM:
                if (in.k >= 16) return 0;
                X = M[in.k]; break;
            case cbpf::MSH:
                if (size != cbpf::B) return 0;
                if (!mem_read(in.k, 1, tmp)) return 0;
                X = (tmp & 0x0F) * 4; break;
            default: return 0;
            }
            ++pc; break;

        case cbpf::ST:
            if (in.k >= 16) return 0;
            M[in.k] = A; ++pc; break;
        case cbpf::STX:
            if (in.k >= 16) return 0;
            M[in.k] = X; ++pc; break;

        case cbpf::ALU: {
            const uint32_t s = (src == cbpf::X) ? X : in.k;
            switch (op) {
            case cbpf::ADD: A += s; break;
            case cbpf::SUB: A -= s; break;
            case cbpf::MUL: A *= s; break;
            case cbpf::DIV: if (s == 0) return 0; A /= s; break;
            case cbpf::MOD: if (s == 0) return 0; A %= s; break;
            case cbpf::OR:  A |= s; break;
            case cbpf::AND: A &= s; break;
            case cbpf::LSH: A <<= s; break;
            case cbpf::RSH: A >>= s; break;
            case cbpf::NEG: A = ~A + 1; break;   // -A (mod 2^32)
            case cbpf::XOR: A ^= s; break;
            default: return 0;
            }
            ++pc; break;
        }

        case cbpf::JMP: {
            const uint32_t s = (src == cbpf::X) ? X : in.k;
            bool taken;
            switch (op) {
            case cbpf::JA:   pc += in.k + 1; continue;
            case cbpf::JEQ:  taken = (A == s); break;
            case cbpf::JGT:  taken = (A > s);  break;
            case cbpf::JGE:  taken = (A >= s); break;
            case cbpf::JSET: taken = (A & s) != 0; break;
            default: return 0;
            }
            pc += (taken ? in.jt : in.jf) + 1;
            break;
        }

        case cbpf::RET:
            switch (in.code & 0x18) {        // RVAL field: K=0x00, A=0x10
            case 0x00: return in.k;
            case 0x10: return A;
            default: return 0;
            }

        case cbpf::MISC:
            switch (in.code & 0xf8) {        // MISCOP field
            case cbpf::TAX: X = A; ++pc; break;
            case cbpf::TXA: A = X; ++pc; break;
            default: return 0;
            }
            break;

        default: return 0;
        }
    }
    return 0;   // fell off the end — kernel treats as drop
}

} // namespace EtherCAT
