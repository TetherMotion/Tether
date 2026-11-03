/**
 * @file test_cbpf_interpreter.cpp
 * @brief User-space classic-BPF interpreter coverage
 *        (src/ethercat/CBPFInterpreter.cpp).
 *
 * Exercises cbpfExecute directly: every instruction class, load mode,
 * ALU operation, jump type, MISC transfers, out-of-bounds reads and
 * the SKF_AD_VLAN ancillary loads used for stripped 802.1Q tags.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "tether/ethercat/CBPFProgramFactory.hpp"

using namespace EtherCAT;
using namespace EtherCAT::cbpf;

namespace {

/// Ethernet-ish test frame: dst(6) src(6) ethertype(2)=0x88A4 payload.
std::vector<uint8_t> frame() {
    std::vector<uint8_t> f(64, 0);
    for (int i = 0; i < 6; ++i) f[i] = static_cast<uint8_t>(0xAA + i);  // dst
    for (int i = 0; i < 6; ++i) f[6 + i] = static_cast<uint8_t>(0x11 + i); // src
    f[12] = 0x88; f[13] = 0xA4;   // EtherType EtherCAT
    f[14] = 0x01; f[15] = 0x02; f[16] = 0x03; f[17] = 0x04;  // payload word
    return f;
}

CBPFInsn I(uint16_t code, uint32_t k, uint8_t jt = 0, uint8_t jf = 0) {
    return CBPFInsn{code, jt, jf, k};
}

// ============================================================================
// Loads
// ============================================================================

TEST(CBPFInterpreterTest, RetKReturnsImmediate) {
    std::vector<CBPFInsn> prog{I(RET | K, 0xFFFF)};
    EXPECT_EQ(cbpfExecute(prog, frame().data(), 64), 0xFFFFu);
}

TEST(CBPFInterpreterTest, RetAReturnsAccumulator) {
    std::vector<CBPFInsn> prog{
        I(LD | IMM | W, 1234),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, frame().data(), 64), 1234u);
}

TEST(CBPFInterpreterTest, LdAbsByteHalfWord) {
    auto f = frame();
    std::vector<CBPFInsn> bProg{I(LD | ABS | B, 12), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(bProg, f.data(), f.size()), 0x88u);
    std::vector<CBPFInsn> hProg{I(LD | ABS | H, 12), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(hProg, f.data(), f.size()), 0x88A4u);
    std::vector<CBPFInsn> wProg{I(LD | ABS | W, 14), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(wProg, f.data(), f.size()), 0x01020304u);
}

TEST(CBPFInterpreterTest, LdAbsOutOfBoundsRejects) {
    auto f = frame();
    std::vector<CBPFInsn> prog{I(LD | ABS | W, 62), I(RET | A, 0)};
    // Word read at offset 62 exceeds 64-byte frame → interpreter returns 0
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0u);
}

TEST(CBPFInterpreterTest, LdLenReturnsPacketLength) {
    auto f = frame();
    std::vector<CBPFInsn> prog{I(LD | LEN | W, 0), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), f.size());
}

TEST(CBPFInterpreterTest, LdIndUsesXOffset) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LDX | IMM | W, 12),            // X = 12
        I(LD | IND | B, 1),              // A = data[X + 1] = data[13] = 0xA4
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0xA4u);
}

TEST(CBPFInterpreterTest, LdMemReadsScratch) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | IMM | W, 0xDEAD),
        I(ST, 0),                        // M[0] = A
        I(LD | IMM | W, 0),
        I(LD | MEM | W, 0),              // A = M[0]
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0xDEADu);
}

TEST(CBPFInterpreterTest, LdxMemAndMsh) {
    auto f = frame();
    // LDX MEM: X = M[3]
    std::vector<CBPFInsn> prog{
        I(LD | IMM | W, 77),
        I(ST, 3),
        I(LDX | MEM | W, 3),             // X = M[3] = 77
        I(MISC | TXA, 0),                // A = X
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 77u);

    // MSH: X = 4 * (data[k] & 0x0F) — the classic IP-header-length load
    f[20] = 0x45;                        // IP version/IHL = 5
    std::vector<CBPFInsn> msh{
        I(LDX | MSH | B, 20),            // X = 4 * (0x45 & 0xF) = 20
        I(MISC | TXA, 0),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(msh, f.data(), f.size()), 20u);
}

TEST(CBPFInterpreterTest, StxStoresXToScratch) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LDX | IMM | W, 55),
        I(STX, 1),                       // M[1] = X
        I(LDX | MEM | W, 1),             // X = M[1]
        I(MISC | TXA, 0),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 55u);
}

// ============================================================================
// ALU
// ============================================================================

TEST(CBPFInterpreterTest, AluKOperations) {
    auto f = frame();
    auto run = [&](uint16_t op, uint32_t a0, uint32_t k) {
        std::vector<CBPFInsn> prog{
            I(LD | IMM | W, a0), I(ALU | op | K, k), I(RET | A, 0)};
        return cbpfExecute(prog, f.data(), f.size());
    };
    EXPECT_EQ(run(ADD, 10, 5), 15u);
    EXPECT_EQ(run(SUB, 10, 5), 5u);
    EXPECT_EQ(run(MUL, 10, 5), 50u);
    EXPECT_EQ(run(DIV, 10, 5), 2u);
    EXPECT_EQ(run(OR,  0xF0, 0x0F), 0xFFu);
    EXPECT_EQ(run(AND, 0xF0, 0x1F), 0x10u);
    EXPECT_EQ(run(LSH, 1, 4), 16u);
    EXPECT_EQ(run(RSH, 256, 4), 16u);
    EXPECT_EQ(run(MOD, 10, 3), 1u);
    EXPECT_EQ(run(XOR, 0xFF, 0x0F), 0xF0u);
    EXPECT_EQ(run(NEG, 5, 0), static_cast<uint32_t>(-5));
}

TEST(CBPFInterpreterTest, AluXOperations) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LDX | IMM | W, 3),             // X = 3
        I(LD | IMM | W, 10),             // A = 10
        I(ALU | MUL | X, 0),             // A = A * X = 30
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 30u);
}

TEST(CBPFInterpreterTest, AluDivByZeroRejects) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | IMM | W, 10), I(ALU | DIV | K, 0), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0u);
    std::vector<CBPFInsn> mod{
        I(LD | IMM | W, 10), I(ALU | MOD | K, 0), I(RET | A, 0)};
    EXPECT_EQ(cbpfExecute(mod, f.data(), f.size()), 0u);
}

// ============================================================================
// Jumps
// ============================================================================

TEST(CBPFInterpreterTest, JaSkipsForward) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(JMP | JA, 1),                  // skip next insn
        I(LD | IMM | W, 111),
        I(LD | IMM | W, 222),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 222u);
}

TEST(CBPFInterpreterTest, JeqBranchesOnMatch) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | ABS | H, 12),             // A = 0x88A4
        I(JMP | JEQ | K, 0x88A4, 2, 0),  // match → skip else block
        I(LD | IMM | W, 1),              // else path (not taken)
        I(RET | A, 0),
        I(LD | IMM | W, 2),              // match path
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 2u);
}

TEST(CBPFInterpreterTest, JeqBranchesOnMiss) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | ABS | H, 12),             // A = 0x88A4
        I(JMP | JEQ | K, 0x0800, 2, 0),  // match → skip else block
        I(LD | IMM | W, 1),              // else path
        I(RET | A, 0),
        I(LD | IMM | W, 2),              // match path (not taken)
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 1u);
}

TEST(CBPFInterpreterTest, JgtJgeJsetSemantics) {
    auto f = frame();
    auto run = [&](uint16_t op, uint32_t a, uint32_t k) {
        std::vector<CBPFInsn> prog{
            I(LD | IMM | W, a),
            I(JMP | op | K, k, 2, 0),    // taken → skip not-taken block
            I(LD | IMM | W, 1),          // not-taken path
            I(RET | A, 0),
            I(LD | IMM | W, 2),          // taken path
            I(RET | A, 0),
        };
        return cbpfExecute(prog, f.data(), f.size());
    };
    EXPECT_EQ(run(JGT, 10, 5), 2u);   // 10 > 5 → taken
    EXPECT_EQ(run(JGT, 5, 10), 1u);   // not taken
    EXPECT_EQ(run(JGE, 10, 10), 2u);  // equal counts
    EXPECT_EQ(run(JGE, 9, 10), 1u);
    EXPECT_EQ(run(JSET, 0xF0, 0x10), 2u);  // bit set
    EXPECT_EQ(run(JSET, 0xF0, 0x01), 1u);  // bit clear
}

// ============================================================================
// MISC
// ============================================================================

TEST(CBPFInterpreterTest, TaxTransfersAccumulator) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | IMM | W, 99),
        I(MISC | TAX, 0),                // X = A
        I(LD | IND | B, 0),              // A = data[X + 0] = data[99] — OOB!
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0u);  // OOB → 0
}

// ============================================================================
// SKF_AD ancillary loads (stripped 802.1Q tags)
// ============================================================================

TEST(CBPFInterpreterTest, VlanTagPresentReportsNoTag) {
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | W | ABS, kSkfAdVlanTagPresent),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size(), nullptr), 0u);
}

TEST(CBPFInterpreterTest, VlanTagPresentReportsTag) {
    auto f = frame();
    CBPFAuxData aux{};
    aux.vlan_tci = 0x0064;  // VID 100
    aux.vlan_tpid = kEtherType8021Q;
    std::vector<CBPFInsn> prog{
        I(LD | W | ABS, kSkfAdVlanTagPresent),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size(), &aux), 1u);
}

TEST(CBPFInterpreterTest, VlanTagReturnsTci) {
    auto f = frame();
    CBPFAuxData aux{};
    aux.vlan_tci = 0xE123;  // PCP/DEI + VID
    std::vector<CBPFInsn> prog{
        I(LD | W | ABS, kSkfAdVlanTag),
        I(RET | A, 0),
    };
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size(), &aux), 0xE123u);
}

TEST(CBPFInterpreterTest, VlanTaggedEtherCatFilter) {
    // Program: accept when (tag present AND inner ethertype 0x88A4) or
    // untagged 0x88A4 — the real dual-path pattern.
    auto f = frame();
    std::vector<CBPFInsn> prog{
        I(LD | W | ABS, kSkfAdVlanTagPresent),   // 0
        I(JMP | JEQ | K, 0, 2, 0),               // 1: no tag → jump +2
        I(LD | H | ABS, 12),                     // 2: tagged → inner type
        I(JMP | JA, 1, 0, 0),                    // 3: join compare path
        I(LD | H | ABS, 12),                     // 4: untagged → type
        I(JMP | JEQ | K, 0x88A4, 0, 1),          // 5: EtherCAT?
        I(RET | K, 0xFFFF),                      // 6: accept
        I(RET | K, 0),                           // 7: drop
    };
    // Untagged EtherCAT frame → accept.
    EXPECT_NE(cbpfExecute(prog, f.data(), f.size(), nullptr), 0u);
    // Tagged EtherCAT frame (inner type still at [12] after strip) → accept.
    CBPFAuxData aux{};
    aux.vlan_tci = 7;
    aux.vlan_tpid = kEtherType8021Q;
    EXPECT_NE(cbpfExecute(prog, f.data(), f.size(), &aux), 0u);
    // Non-EtherCAT payload → drop.
    f[12] = 0x08; f[13] = 0x00;
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size(), nullptr), 0u);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(CBPFInterpreterTest, EmptyProgramReturnsZero) {
    auto f = frame();
    EXPECT_EQ(cbpfExecute(nullptr, 0, f.data(), f.size(), nullptr), 0u);
}

TEST(CBPFInterpreterTest, NoRetFallsOffEndReturningZero) {
    auto f = frame();
    std::vector<CBPFInsn> prog{I(LD | IMM | W, 42)};  // no RET
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0u);
}

TEST(CBPFInterpreterTest, RunOffEndReturnsZero) {
    auto f = frame();
    std::vector<CBPFInsn> prog{I(JMP | JA, 5)};  // jump past end
    EXPECT_EQ(cbpfExecute(prog, f.data(), f.size()), 0u);
}

} // namespace
