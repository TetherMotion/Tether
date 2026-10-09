/**
 * @file CBPFInterpreter.cpp
 * @brief CBPFProgramFactory — user-space classic-BPF interpreter.
 *
 * TU split out of CBPFProgramFactory.cpp.
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
