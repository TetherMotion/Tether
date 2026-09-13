/**
 * @file test_beckhoff_terminals.cpp
 * @brief Unit tests for the Beckhoff terminal driver layer
 *
 * Coverage:
 *  - PdoChannelLayout: pure SII PDO->image layout resolution (value
 *    channels, byte/bit offsets, subindex-aware and wildcard lookup,
 *    multi-value channels, misalignment rejection)
 *  - PositionTracker: counter wraparound / multi-turn unwrapping
 *  - CompactDriveTerminal::decodeState: CiA402 statusword decode table
 *  - Device registries: vendor/product matching incl. EP/ER/EJ variants
 *  - Multi*Terminal chains: flat channel indexing, module offsets,
 *    capacity limit, out-of-range access — with fake terminals
 *  - Field-offset resolution for the EL7031 POS interface, EL2522 PTO
 *    (incl. ENC object disambiguation), EL2502/EL2535 PWM, ELM7211 DRV
 *  - FSoE: MasterConnectionConfig derivation + end-to-end PDO exchange
 *    against the FSoESlave emulator (the same path SafetyTerminal uses)
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "tether/Beckhoff/CompactDriveTerminal.hpp"
#include "tether/Beckhoff/InputTerminal.hpp"
#include "tether/Beckhoff/MultiPositionInputTerminal.hpp"
#include "tether/Beckhoff/OutputTerminal.hpp"
#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/Beckhoff/PositionInputTerminal.hpp"
#include "tether/Beckhoff/PositionTracker.hpp"
#include "tether/Beckhoff/PulseTrainTerminal.hpp"
#include "tether/Beckhoff/PwmTerminal.hpp"
#include "tether/Beckhoff/SafetyTerminal.hpp"
#include "tether/Beckhoff/StepperTerminal.hpp"
#include "tether/ethercat/Master.hpp"

#if TETHER_BECKHOFF_TEST_FSOE
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"
#include "tether/fsoe/FSoECRC.hpp"
#endif

using namespace EtherCAT;
using namespace EtherCAT::Beckhoff;
using EtherCAT::SII::SIIPDO;
using EtherCAT::SII::SIIPDOEntry;

namespace {

// ---------------------------------------------------------------------------
// Fixture helpers — build SIIPDO lists mirroring the real ESIs
// ---------------------------------------------------------------------------

SIIPDOEntry e(uint16_t idx, uint8_t sub, uint8_t bits) {
    SIIPDOEntry x{};
    x.index = idx;
    x.subindex = sub;
    x.bit_length = bits;
    return x;
}

SIIPDO pdo(uint16_t index, uint8_t sm, std::vector<SIIPDOEntry> entries) {
    SIIPDO p{};
    p.pdo_index = index;
    p.sync_manager = sm;
    p.n_entries = static_cast<uint8_t>(entries.size());
    p.entries = std::move(entries);
    return p;
}

/// EL5001: SM3, one 32-bit counter value.
std::vector<SIIPDO> el5001Tx() {
    return {pdo(0x1A00, 3, {e(0x6000, 0x11, 32)})};
}

/// EL5101: SM3 = status word (16b) + counter (16b) + latch (16b).
std::vector<SIIPDO> el5101Tx() {
    return {pdo(0x1A03, 3, {e(0x6000, 0x01, 8), e(0x6000, 0x02, 8),
                            e(0x6000, 0x11, 16), e(0x6000, 0x12, 16)})};
}

/// EL5042: SM3 = two 64-bit BiSS positions.
std::vector<SIIPDO> el5042Tx() {
    return {pdo(0x1A00, 3, {e(0x6000, 0x11, 64)}),
            pdo(0x1A01, 3, {e(0x6010, 0x11, 64)})};
}

/// EL5152-style: SM3 = per-channel 16 bit-field status flags + 32-bit
/// counter + 32-bit latch (the real EL5151/52 "ENC Inputs" PDO shape:
/// status is always packed 1-bit entries, never a plain 16-bit word).
std::vector<SIIPDO> el5151Tx() {
    auto status = [](uint16_t obj) {
        return std::vector<SIIPDOEntry>{
            e(obj, 1, 1),  e(obj, 2, 1),  e(obj, 3, 1),  e(0, 0, 5),
            e(obj, 9, 1),  e(obj, 10, 1), e(obj, 11, 1), e(0, 0, 1),
            e(obj, 13, 1), e(0, 0, 2),    e(0x1800, 9, 1)};
    };
    return {pdo(0x1A00, 3, [&]{ auto v = status(0x6000);
                                v.push_back(e(0x6000, 0x11, 32));
                                v.push_back(e(0x6000, 0x12, 32));
                                return v; }()),
            pdo(0x1A01, 3, [&]{ auto v = status(0x6010);
                                v.push_back(e(0x6010, 0x11, 32));
                                v.push_back(e(0x6010, 0x12, 32));
                                return v; }())};
}

/// EL7031 SM2 output image (8B, real ESI default):
///   0x1600 ENC Control compact: 4 flag bits + 4 pad + 8 pad + set-counter(16)
///   0x1602 STM Control:         3 flag bits + 5 pad + 8 pad
///   0x1604 STM Velocity:        velocity(16)
std::vector<SIIPDO> el7031Rx() {
    return {
        pdo(0x1600, 2, {e(0x0000, 0, 1),  // pad
                        e(0x7000, 2, 1),  // latch ext pos
                        e(0x7000, 3, 1),  // set counter
                        e(0x7000, 4, 1),  // latch ext neg
                        e(0x0000, 0, 4),  // pad
                        e(0x0000, 0, 8),  // pad
                        e(0x7000, 0x17, 16)}), // set counter value
        pdo(0x1602, 2, {e(0x7010, 1, 1),  // enable
                        e(0x7010, 2, 1),  // reset
                        e(0x7010, 3, 1),  // reduce torque
                        e(0x0000, 0, 5),  // pad
                        e(0x0000, 0, 8)}),// pad
        pdo(0x1604, 2, {e(0x7010, 0x21, 16)}), // velocity
    };
}

/// EL7031 SM3 input image (8B, real ESI default):
///   0x1A00 ENC Status compact: 16 flag/pad bits + counter(16) + latch(16)
///   0x1A03 STM Status:         16 flag/pad bits
std::vector<SIIPDO> el7031Tx() {
    return {
        pdo(0x1A00, 3, {e(0x0000, 0, 1),   // pad
                        e(0x6000, 2, 1),   // latch ext valid
                        e(0x6000, 3, 1),   // set counter done
                        e(0x6000, 4, 1),   // underflow
                        e(0x6000, 5, 1),   // overflow
                        e(0x0000, 0, 3),   // pad
                        e(0x0000, 0, 4),   // pad
                        e(0x6000, 0x0D, 1),// ext latch status
                        e(0x1C32, 0x20, 1),// sync error
                        e(0x0000, 0, 1),   // pad
                        e(0x1800, 9, 1),   // TxPDO toggle
                        e(0x6000, 0x11, 16), // counter value
                        e(0x6000, 0x12, 16)}), // latch value
        pdo(0x1A03, 3, {e(0x6010, 1, 1), e(0x6010, 2, 1), e(0x6010, 3, 1),
                        e(0x6010, 4, 1), e(0x6010, 5, 1), e(0x6010, 6, 1),
                        e(0x6010, 7, 1), e(0x0000, 0, 1), e(0x0000, 0, 3),
                        e(0x6010, 12, 1), e(0x6010, 13, 1),
                        e(0x1C32, 0x20, 1), e(0x0000, 0, 1),
                        e(0x1803, 9, 1)}),
    };
}

/// EL2522 SM2: per channel PTO ctrl (bits+freq16) + target(32) + ENC ctrl.
std::vector<SIIPDO> el2522Rx() {
    return {
        pdo(0x1600, 2, {e(0x7000, 1, 1), e(0x7000, 2, 1), e(0x7000, 3, 1),
                        e(0x7000, 0, 5), e(0x7000, 0x11, 16)}),
        pdo(0x1603, 2, {e(0x7000, 0x12, 32)}),
        pdo(0x1605, 2, {e(0x7010, 1, 1), e(0x7010, 2, 1), e(0x7010, 3, 1),
                        e(0x7010, 0, 5), e(0x7010, 0x11, 16)}),
        pdo(0x1608, 2, {e(0x7010, 0x12, 32)}),
        pdo(0x160B, 2, {e(0x7020, 3, 1), e(0x7020, 0, 7),
                        e(0x7020, 0x11, 32)}),
        pdo(0x160D, 2, {e(0x7030, 3, 1), e(0x7030, 0, 7),
                        e(0x7030, 0x11, 32)}),
    };
}

/// EL2522 SM3: per channel PTO status + ENC status (counter 32b).
std::vector<SIIPDO> el2522Tx() {
    return {
        pdo(0x1A00, 3, {e(0x6000, 1, 1), e(0x6000, 2, 1), e(0x6000, 0, 4),
                        e(0x6000, 7, 1), e(0x6000, 0, 6),
                        e(0x6000, 0x14, 1), e(0x6000, 0, 1),
                        e(0x6000, 0x16, 1)}),
        pdo(0x1A01, 3, {e(0x6010, 1, 1), e(0x6010, 2, 1), e(0x6010, 0, 4),
                        e(0x6010, 7, 1), e(0x6010, 0, 6),
                        e(0x6010, 0x14, 1), e(0x6010, 0, 1),
                        e(0x6010, 0x16, 1)}),
        pdo(0x1A03, 3, {e(0x6020, 3, 1), e(0x6020, 4, 1), e(0x6020, 5, 1),
                        e(0x6020, 0, 8), e(0x6020, 0x14, 1),
                        e(0x6020, 0x15, 1), e(0x6020, 0x16, 1),
                        e(0x6020, 0x11, 32)}),
        pdo(0x1A05, 3, {e(0x6030, 3, 1), e(0x6030, 4, 1), e(0x6030, 5, 1),
                        e(0x6030, 0, 8), e(0x6030, 0x14, 1),
                        e(0x6030, 0x15, 1), e(0x6030, 0x16, 1),
                        e(0x6030, 0x11, 32)}),
    };
}

/// EL2502 SM2: two 16-bit PWM duty values.
std::vector<SIIPDO> el2502Rx() {
    return {pdo(0x1600, 2, {e(0x7000, 0x11, 16)}),
            pdo(0x1601, 2, {e(0x7010, 0x11, 16)})};
}

/// EL2535 SM2: per channel control bits + 16-bit duty.
std::vector<SIIPDO> el2535Rx() {
    return {
        pdo(0x1600, 2, {e(0x7000, 1, 1), e(0x7000, 0, 4), e(0x7000, 6, 1),
                        e(0x7000, 7, 1), e(0x7000, 0, 1),
                        e(0x7000, 0x11, 16)}),
        pdo(0x1601, 2, {e(0x7010, 1, 1), e(0x7010, 0, 4), e(0x7010, 6, 1),
                        e(0x7010, 7, 1), e(0x7010, 0, 1),
                        e(0x7010, 0x11, 16)}),
    };
}

/// EL2535 SM3: per channel status bits.
std::vector<SIIPDO> el2535Tx() {
    return {
        pdo(0x1A00, 3, {e(0x6000, 1, 1), e(0x6000, 0, 3), e(0x6000, 5, 1),
                        e(0x6000, 6, 1), e(0x6000, 7, 1), e(0x1800, 9, 1)}),
        pdo(0x1A02, 3, {e(0x6010, 1, 1), e(0x6010, 0, 3), e(0x6010, 5, 1),
                        e(0x6010, 6, 1), e(0x6010, 7, 1), e(0x1802, 9, 1)}),
    };
}

/// ELM7211 SM2: controlword + modes + target position.
std::vector<SIIPDO> elm7211Rx() {
    return {pdo(0x1610, 2, {e(0x7010, 1, 16), e(0x7010, 3, 8)}),
            pdo(0x1611, 2, {e(0x7010, 5, 32)})};
}

/// ELM7211 SM3: feedback position + statusword + following error.
std::vector<SIIPDO> elm7211Tx() {
    return {pdo(0x1A00, 3, {e(0x6000, 0x11, 32)}),
            pdo(0x1A10, 3, {e(0x6010, 1, 16), e(0x6010, 3, 8)}),
            pdo(0x1A11, 3, {e(0x6010, 6, 32)})};
}

/// EL7062 SM2: two-axis DRV outputs (ch2 at 0x7080).
std::vector<SIIPDO> el7062Rx() {
    return {pdo(0x1600, 2, {e(0x7010, 1, 16), e(0x7010, 5, 32)}),
            pdo(0x1680, 2, {e(0x7080, 1, 16), e(0x7080, 5, 32)})};
}
std::vector<SIIPDO> el7062Tx() {
    return {pdo(0x1A00, 3, {e(0x6000, 0x11, 32)}),
            pdo(0x1A10, 3, {e(0x6010, 1, 16), e(0x6010, 7, 32)}),
            pdo(0x1A80, 3, {e(0x6070, 0x11, 32)}),
            pdo(0x1A90, 3, {e(0x6080, 1, 16), e(0x6080, 7, 32)})};
}

DiscoveredSlave slaveWith(uint32_t vid, uint32_t pc, uint16_t idx = 0) {
    DiscoveredSlave s;
    s.index = idx;
    s.vendor_id = vid;
    s.product_code = pc;
    return s;
}

} // anonymous namespace

// ============================================================================
// PdoChannelLayout — resolveValueChannels
// ============================================================================

#if TETHER_ENABLE_SII

TEST(PdoChannelLayout, Single32BitChannel) {
    auto pdos = el5001Tx();
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    ASSERT_TRUE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
    ASSERT_EQ(chans.size(), 1u);
    EXPECT_EQ(chans[0].values.size(), 1u);
    EXPECT_EQ(chans[0].values[0].byte_off, 0u);
    EXPECT_EQ(chans[0].values[0].bit_len, 32u);
    EXPECT_EQ(bits, 32u);
}

TEST(PdoChannelLayout, StatusPrefixAndTwoValues) {
    auto pdos = el5101Tx();
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    ASSERT_TRUE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
    ASSERT_EQ(chans.size(), 1u);
    // Two >=16-bit entries: counter + latch
    ASSERT_EQ(chans[0].values.size(), 2u);
    EXPECT_EQ(chans[0].values[0].index, 0x6000);
    EXPECT_EQ(chans[0].values[0].subindex, 0x11);
    EXPECT_EQ(chans[0].values[0].byte_off, 2u);   // after 16-bit status
    EXPECT_EQ(chans[0].values[1].byte_off, 4u);
    EXPECT_EQ(chans[0].status_off, 0);
    EXPECT_EQ(bits, 48u);
}

TEST(PdoChannelLayout, SixtyFourBitValues) {
    auto pdos = el5042Tx();
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    ASSERT_TRUE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
    ASSERT_EQ(chans.size(), 2u);
    EXPECT_EQ(chans[0].values[0].bit_len, 64u);
    EXPECT_EQ(chans[0].values[0].byte_off, 0u);
    EXPECT_EQ(chans[1].values[0].byte_off, 8u);
    EXPECT_EQ(bits, 128u);
}

TEST(PdoChannelLayout, MultiChannel) {
    auto pdos = el5151Tx();
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    ASSERT_TRUE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
    ASSERT_EQ(chans.size(), 2u);
    // ch0: 2B status flags + counter@2 + latch@6; ch1 starts at byte 10.
    EXPECT_EQ(chans[0].values[0].byte_off, 2u);
    EXPECT_EQ(chans[0].values[1].byte_off, 6u);
    EXPECT_EQ(chans[0].status_off, 0);
    EXPECT_EQ(chans[1].values[0].byte_off, 12u);
    EXPECT_EQ(chans[1].status_off, 10);
    EXPECT_EQ(bits, 160u);
}

TEST(PdoChannelLayout, MissingSyncManagerYieldsNoChannels) {
    auto pdos = el5001Tx();
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    // PDOs are assigned to SM3 — asking for SM2 must find nothing.
    EXPECT_TRUE(detail::resolveValueChannels(pdos, 2, 16, chans, bits));
    EXPECT_TRUE(chans.empty());
    EXPECT_EQ(bits, 0u);
}

TEST(PdoChannelLayout, NonByteAlignedLayoutRejected) {
    // A PDO whose total bit count is not a multiple of 8 after a value
    // field — e.g. 16-bit status + 16-bit value + 4-bit tail.
    auto pdos = std::vector<SIIPDO>{
        pdo(0x1A00, 3, {e(0x6000, 1, 16), e(0x6000, 0x11, 16),
                        e(0x6000, 9, 4)})};
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    EXPECT_FALSE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
    EXPECT_TRUE(chans.empty());
}

TEST(PdoChannelLayout, ValueOnOddByteBoundaryRejected) {
    // Value field starting mid-byte cannot be read as a byte-aligned word.
    auto pdos = std::vector<SIIPDO>{
        pdo(0x1A00, 3, {e(0x6000, 1, 4), e(0x6000, 0x11, 16)})};
    std::vector<detail::ResolvedChannel> chans;
    uint32_t bits = 0;
    EXPECT_FALSE(detail::resolveValueChannels(pdos, 3, 16, chans, bits));
}

// ============================================================================
// PdoChannelLayout — findEntryBitOffset / findValueEntry / firstPdoIndex
// ============================================================================

TEST(PdoChannelLayout, BitOffsetsByIndexAndSubindex) {
    auto pdos = el7031Rx();
    uint32_t off = 0;
    // ENC control byte is the first thing in the SM2 image.
    EXPECT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7000, 2, 0, off));
    EXPECT_EQ(off, 1u);                       // second bit of byte 0
    EXPECT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7000, 0x17, 0, off));
    EXPECT_EQ(off, 16u);                      // set-counter value at byte 2
    // STM control byte at byte 4.
    EXPECT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7010, 1, 0, off));
    EXPECT_EQ(off, 32u);
    // Velocity at byte 6.
    uint8_t vlen = 0;
    EXPECT_TRUE(detail::findValueEntry(pdos, 2, 0x7010, 0x21, 0, 16,
                                       off, vlen));
    EXPECT_EQ(off, 48u);
    EXPECT_EQ(vlen, 16u);
}

TEST(PdoChannelLayout, WildcardSubindexFindsFirstEntry) {
    auto pdos = el7031Rx();
    uint32_t off = 0;
    EXPECT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7000, -1, 0, off));
    EXPECT_EQ(off, 1u);   // first 0x7000 entry is sub:02 (bit 0 is index-0 pad)
}

TEST(PdoChannelLayout, MissingEntryReturnsFalse) {
    auto pdos = el7031Rx();
    uint32_t off = 0;
    EXPECT_FALSE(detail::findEntryBitOffset(pdos, 2, 0x7099, 1, 0, off));
    EXPECT_FALSE(detail::findEntryBitOffset(pdos, 3, 0x7000, 1, 0, off));
}

TEST(PdoChannelLayout, OccurrenceSelectsLaterEntries) {
    // Two entries with the same index — the second occurrence.
    auto pdos = std::vector<SIIPDO>{
        pdo(0x1A00, 3, {e(0x6000, 1, 8), e(0x6000, 1, 8)})};
    uint32_t off = 0;
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 3, 0x6000, 1, 1, off));
    EXPECT_EQ(off, 8u);
}

TEST(PdoChannelLayout, FirstPdoIndex) {
    auto pdos = el7031Tx();
    EXPECT_EQ(detail::firstPdoIndex(pdos, 3), 0x1A00);
    EXPECT_EQ(detail::firstPdoIndex(pdos, 2), 0u);
}

// ============================================================================
// EL7031 POS-interface field offsets — the exact offsets the driver consumes
// ============================================================================

TEST(StepperLayout, OutputImageFieldOffsets) {
    auto pdos = el7031Rx();
    uint32_t off = 0;
    // STM enable/reset/reduce-torque at byte 4, bits 0-2.
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7010, 1, 0, off));
    EXPECT_EQ(off, 32u);
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7010, 2, 0, off));
    EXPECT_EQ(off, 33u);
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7010, 3, 0, off));
    EXPECT_EQ(off, 34u);
    // Velocity at byte 6 (16-bit).
    uint8_t bl = 0;
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7010, 0x21, 0, 16, off, bl));
    EXPECT_EQ(off, 48u);
    EXPECT_EQ(bl, 16u);
    // Set-counter value at byte 2 (subindex 0x17 in the real object).
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7000, 0x17, 0, 16, off, bl));
    EXPECT_EQ(off, 16u);
}

TEST(StepperLayout, InputImageFieldOffsets) {
    auto pdos = el7031Tx();
    uint32_t off = 0;
    // ENC status bits at byte 0.
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 3, 0x6000, 2, 0, off));
    EXPECT_EQ(off, 1u);   // latch ext valid = bit 1
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 3, 0x6000, 5, 0, off));
    EXPECT_EQ(off, 4u);   // overflow = bit 4
    // Counter at byte 2, latch at byte 4 (after the 16 flag/pad bits).
    uint8_t bl = 0;
    ASSERT_TRUE(detail::findValueEntry(pdos, 3, 0x6000, 0x11, 0, 16, off, bl));
    EXPECT_EQ(off, 16u);
    ASSERT_TRUE(detail::findValueEntry(pdos, 3, 0x6000, 0x12, 0, 16, off, bl));
    EXPECT_EQ(off, 32u);
    // STM status first bit at byte 6.
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 3, 0x6010, -1, 0, off));
    EXPECT_EQ(off, 48u);
}

// ============================================================================
// EL2522 PTO — ENC object disambiguation (claimed-set semantics)
// ============================================================================

TEST(PulseTrainLayout, PtoFieldsPerChannel) {
    auto pdos = el2522Rx();
    uint8_t bl = 0;
    uint32_t off = 0;
    // ch1 frequency value: PDO 0x1600 ends at bit 24 → freq at byte 1.
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7000, 0x11, 0, 8, off, bl));
    EXPECT_EQ(off, 8u);
    // ch2 frequency value lives after PDOs 0x1600 (3B) + 0x1603 (4B) = 7B.
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7010, 0x11, 0, 8, off, bl));
    EXPECT_EQ(off, 7u * 8 + 8u);
}

TEST(PulseTrainLayout, EncObjectsDisambiguatedFromPtoAndPls) {
    auto pdos = el2522Rx();
    uint8_t bl = 0;
    uint32_t off = 0;
    // The ENC-control discriminator used by PulseTrainTerminal:
    //   object has a 1-bit entry at :03, no entry at :01, and a value at :17.
    auto isEncCtrl = [&](uint16_t obj) {
        uint32_t o = 0; uint8_t b = 0;
        if (!detail::findValueEntry(pdos, 2, obj, 3, 0, 1, o, b) || b != 1)
            return false;
        if (detail::findValueEntry(pdos, 2, obj, 1, 0, 1, o, b))
            return false;
        return detail::findValueEntry(pdos, 2, obj, 0x11, 0, 8, o, b);
    };
    EXPECT_FALSE(isEncCtrl(0x7000));  // PTO ch1 — has :01 freq select
    EXPECT_FALSE(isEncCtrl(0x7010));  // PTO ch2 — has :01
    EXPECT_TRUE(isEncCtrl(0x7020));   // ENC ch1
    EXPECT_TRUE(isEncCtrl(0x7030));   // ENC ch2
    EXPECT_FALSE(isEncCtrl(0x7040));  // absent
}

// ============================================================================
// ELM7211 / EL7062 DRV — CiA402-semantic field offsets
// ============================================================================

TEST(CompactDriveLayout, Elm7211Fields) {
    auto rx = elm7211Rx();
    auto tx = elm7211Tx();
    uint8_t bl = 0;
    uint32_t off = 0;
    // Controlword: first 2 bytes of SM2.
    ASSERT_TRUE(detail::findValueEntry(rx, 2, 0x7010, 1, 0, 8, off, bl));
    EXPECT_EQ(off, 0u); EXPECT_EQ(bl, 16u);
    // Target position at byte 3 (after 0x1610 = ctrl2+modes1).
    ASSERT_TRUE(detail::findValueEntry(rx, 2, 0x7010, 5, 0, 8, off, bl));
    EXPECT_EQ(off, 24u); EXPECT_EQ(bl, 32u);
    // Statusword in SM3 at byte 4 (after 0x1A00 position).
    ASSERT_TRUE(detail::findValueEntry(tx, 3, 0x6010, 1, 0, 8, off, bl));
    EXPECT_EQ(off, 32u);
    // Following error: 0x1A11 starts at byte 7 (0x1A00 4B + 0x1A10 3B).
    ASSERT_TRUE(detail::findValueEntry(tx, 3, 0x6010, 6, 0, 8, off, bl));
    EXPECT_EQ(off, 56u);
    // FB position at byte 0.
    ASSERT_TRUE(detail::findValueEntry(tx, 3, 0x6000, 0x11, 0, 8, off, bl));
    EXPECT_EQ(off, 0u); EXPECT_EQ(bl, 32u);
}

TEST(CompactDriveLayout, El7062SecondAxisAt7080) {
    auto rx = el7062Rx();
    uint8_t bl = 0;
    uint32_t off = 0;
    // Axis-1 controlword at byte 0 of PDO 0x1680 → image byte 6.
    ASSERT_TRUE(detail::findValueEntry(rx, 2, 0x7080, 1, 0, 8, off, bl));
    EXPECT_EQ(off, 48u);
}

// ============================================================================
// EL2502/EL2535 PWM field offsets
// ============================================================================

TEST(PwmLayout, El2502DutyOffsets) {
    auto pdos = el2502Rx();
    uint8_t bl = 0;
    uint32_t off = 0;
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7000, 0x11, 0, 8, off, bl));
    EXPECT_EQ(off, 0u);
    ASSERT_TRUE(detail::findValueEntry(pdos, 2, 0x7010, 0x11, 0, 8, off, bl));
    EXPECT_EQ(off, 16u);    // channel 2 duty at byte 2
}

TEST(PwmLayout, El2535ControlBits) {
    auto pdos = el2535Rx();
    uint32_t off = 0;
    // Enable at bit 5, reset at bit 6 of each channel control byte.
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7000, 6, 0, off));
    EXPECT_EQ(off, 5u);
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7000, 7, 0, off));
    EXPECT_EQ(off, 6u);
    ASSERT_TRUE(detail::findEntryBitOffset(pdos, 2, 0x7010, 6, 0, off));
    EXPECT_EQ(off, 24u + 5u);
}

// ============================================================================
// PositionTracker — wraparound / multi-turn
// ============================================================================

TEST(PositionTracker, ForwardWrap16Bit) {
    PositionTracker t(16);
    EXPECT_EQ(t.update(0xFFF0), 0xFFF0);
    EXPECT_EQ(t.update(0xFFFF), 0xFFFF);
    EXPECT_EQ(t.update(0x0002), 65538);    // wrapped forward by 3
    EXPECT_EQ(t.turns(), 1);
}

TEST(PositionTracker, ReverseWrap16Bit) {
    PositionTracker t(16);
    t.update(5);
    EXPECT_EQ(t.update(65530), -6);        // 5 - 11 counts, wrapped backward
    EXPECT_EQ(t.turns(), -1);
}

TEST(PositionTracker, MultiTurn) {
    PositionTracker t(16);
    t.update(0);
    int64_t p = 0;
    for (int turn = 0; turn < 3; ++turn) {
        for (uint32_t v = 0; v <= 0xFF00; v += 0x100) {
            p = t.update(v);
        }
        p = t.update(0);   // wrap point
    }
    // 3 forward wraps → position crossed 0 twice going up.
    EXPECT_GE(p, 65536);
}

TEST(PositionTracker, NoFalseWrapOnLargeJump) {
    PositionTracker t(16);
    t.update(0);
    // A forward jump of exactly half the range is not a wrap.
    EXPECT_EQ(t.update(0x7FFF), 0x7FFF);
}

TEST(PositionTracker, ResetAndAdjust) {
    PositionTracker t(16);
    t.update(100);
    t.update(65530);     // wrapped
    t.adjust(-t.position());   // rebase to zero
    EXPECT_EQ(t.position(), 0);
    t.reset();
    EXPECT_EQ(t.update(5000), 5000);   // new origin
}

TEST(PositionTracker, EightBitCounter) {
    PositionTracker t(8);
    t.update(250);
    EXPECT_EQ(t.update(10), 266);      // 250→255→0→10 = +16
}

// ============================================================================
// CompactDriveTerminal::decodeState — CiA402 statusword table
// ============================================================================

TEST(DriveState, CanonicalDecode) {
    using D = DriveState;
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0000),
              D::NotReadyToSwitchOn);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0040),
              D::SwitchOnDisabled);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0021),
              D::ReadyToSwitchOn);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0023),
              D::SwitchedOn);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0027),
              D::OperationEnabled);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0007),
              D::QuickStopActive);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x000F),
              D::FaultReactionActive);
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0008),
              D::Fault);
    // High bits (warning/target reached/etc.) must not disturb decode.
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0627),
              D::OperationEnabled);
    // Bit 3 set anywhere (with no fault-reaction bits) still decodes Fault.
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x1238),
              D::Fault);
    // A lone bit-1 pattern matches no canonical state.
    EXPECT_EQ(CompactDriveTerminal::decodeState(0x0002),
              D::Unknown);
}

// ============================================================================
// Device registries — vendor/product matching
// ============================================================================

TEST(DeviceRegistry, MatchesElFamilies) {
    EXPECT_TRUE(PositionInputTerminal::matches(
        slaveWith(0x00000002, 0x13ED3052), Devices::EL5101));
    EXPECT_TRUE(StepperTerminal::matches(
        slaveWith(0x00000002, 0x1B773052), Devices::EL7031));
    EXPECT_TRUE(CompactDriveTerminal::matches(
        slaveWith(0x00000002, 0x1C213052), Devices::EL7201));
    EXPECT_TRUE(PwmTerminal::matches(
        slaveWith(0x00000002, 0x09C63052), Devices::EL2502));
    EXPECT_TRUE(PulseTrainTerminal::matches(
        slaveWith(0x00000002, 0x09DA3052), Devices::EL2522));
#if TETHER_BECKHOFF_TEST_FSOE
    EXPECT_TRUE(SafetyTerminal::matches(
        slaveWith(0x00000002, 0x077E3052), Devices::EL1918));
#endif
}

TEST(DeviceRegistry, RejectsWrongProductOrVendor) {
    auto s = slaveWith(0x00000002, 0x1B773052);
    EXPECT_FALSE(StepperTerminal::matches(s, Devices::EL7041));
    auto bad_vendor = slaveWith(0x0000000A, 0x1B773052);
    EXPECT_FALSE(StepperTerminal::matches(bad_vendor, Devices::EL7031));
    // A slave that reported no identity doesn't match either.
    DiscoveredSlave bare;
    EXPECT_FALSE(StepperTerminal::matches(bare, Devices::EL7031));
}

TEST(DeviceRegistry, EpErEjVariants) {
    EXPECT_TRUE(PositionInputTerminal::matches(
        slaveWith(0x00000002, 0x13ED2852), Devices::EJ5101));
    EXPECT_TRUE(PositionInputTerminal::matches(
        slaveWith(0x00000002, 0x13ED4052), Devices::EP5101));
    EXPECT_TRUE(StepperTerminal::matches(
        slaveWith(0x00000002, 0x1B814052), Devices::EP7041));
    EXPECT_TRUE(CompactDriveTerminal::matches(
        slaveWith(0x00000002, 0x1B962852), Devices::EJ7062));
    EXPECT_TRUE(PwmTerminal::matches(
        slaveWith(0x00000002, 0x0A042852), Devices::EJ2564));
    EXPECT_TRUE(OutputTerminal::matches(
        slaveWith(0x00000002, 0x09234052), Devices::EP2339));
    EXPECT_TRUE(InputTerminal::matches(
        slaveWith(0x00000002, 0x071B4052), Devices::EP1819));
}

TEST(DeviceRegistry, IdentityChannelCounts) {
    EXPECT_EQ(Devices::EL2502.num_bits, 2u);
    EXPECT_EQ(Devices::EL2564.num_bits, 4u);
    EXPECT_EQ(Devices::EL2522.num_bits, 2u);
    EXPECT_EQ(Devices::EL7062.num_bits, 2u);
    EXPECT_EQ(Devices::ELM7222.num_bits, 2u);
#if TETHER_BECKHOFF_TEST_FSOE
    EXPECT_EQ(Devices::EL1918.num_bits, 8u);
    EXPECT_EQ(Devices::EL2912.num_bits, 2u);
#endif
}

// ============================================================================
// MultiPositionInputTerminal — chain flattening with fake terminals
// ============================================================================

namespace {

class FakePositionTerminal : public IPositionInputTerminal {
public:
    FakePositionTerminal(uint16_t idx, size_t channels, size_t values = 1,
                         size_t bits = 32)
        : idx_(idx), ch_(channels), vals_(values), bits_(bits) {}

    uint16_t slaveIndex() const override { return idx_; }
    size_t channelCount() const override { return ch_; }
    const char* deviceName() const override { return "FakeEnc"; }
    size_t valueCount(size_t) const override { return vals_; }
    int64_t value(size_t ch, size_t i) const override {
        return static_cast<int64_t>(idx_) * 1000 +
               static_cast<int64_t>(ch) * 100 + static_cast<int64_t>(i);
    }
    uint64_t rawValue(size_t ch, size_t i) const override {
        return static_cast<uint64_t>(value(ch, i));
    }
    uint16_t status(size_t) const override { return 0xBEEF; }
    bool hasStatus(size_t) const override { return true; }
    size_t channelBits(size_t) const override { return bits_; }

    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }

private:
    uint16_t idx_;
    size_t ch_, vals_, bits_;
};

} // anonymous namespace

TEST(MultiPosition, FlatChannelIndexing) {
    Master master;
    MultiPositionInputTerminal<> chain(master);
    // Attach out of bus order — the chain sorts by slave index.
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(7, 1)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(3, 2, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(5, 1)));

    EXPECT_EQ(chain.moduleCount(), 3u);
    EXPECT_EQ(chain.channelCount(), 4u);
    // Sorted: slave 3 (2ch), slave 5 (1ch), slave 7 (1ch).
    EXPECT_EQ(chain.slaveIndex(0), 3);
    EXPECT_EQ(chain.slaveIndex(1), 5);
    EXPECT_EQ(chain.slaveIndex(2), 7);
    EXPECT_EQ(chain.channelOffset(0), 0u);
    EXPECT_EQ(chain.channelOffset(1), 2u);
    EXPECT_EQ(chain.channelOffset(2), 3u);

    // Flat indexing lands on the right module+channel.
    EXPECT_EQ(chain.value(0, 0), 3000);   // slave3 ch0 val0
    EXPECT_EQ(chain.value(1, 1), 3101);   // slave3 ch1 val1
    EXPECT_EQ(chain.position(2), 5000);   // slave5 ch0
    EXPECT_EQ(chain.latch(0), 3001);      // slave3 ch0 val1 (latch)
    EXPECT_TRUE(chain.hasLatch(0));
    EXPECT_FALSE(chain.hasLatch(2));      // slave5 has 1 value only
    EXPECT_EQ(chain.status(3), 0xBEEF);
}

TEST(MultiPosition, OutOfRangeThrows) {
    Master master;
    MultiPositionInputTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(1, 1)));
    EXPECT_THROW((void)chain.position(1), std::out_of_range);
    EXPECT_THROW((void)chain.module(5), std::out_of_range);
}

TEST(MultiPosition, CapacityLimitDropsOverflowingModules) {
    Master master;
    MultiPositionInputTerminal<3> chain(master);   // cap = 3 channels
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(1, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(2, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakePositionTerminal>(3, 1)));
    EXPECT_EQ(chain.channelCount(), 3u);
    EXPECT_EQ(chain.moduleCount(), 2u);            // 2+2 would overflow
    EXPECT_EQ(chain.slaveIndex(1), 3);             // module at slave 2 dropped
}

TEST(MultiPosition, EmptyChainDetectFails) {
    Master master;
    MultiPositionInputTerminal<> chain(master);
    auto r = chain.configure();
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Beckhoff::Error::NoDeviceFound);
}

// ============================================================================
// FSoE — connection config derivation + end-to-end PDO exchange
// ============================================================================

#if TETHER_BECKHOFF_TEST_FSOE

TEST(SafetyTerminal, ImageSizeDerivationMatchesEsi) {
    // EL1918: SM2 out 8B, SM3 in 9B → safe data = image - 5 (cmd+CRC+ID).
    // This is the arithmetic SafetyTerminal::configure() applies.
    const uint8_t in_size = 9 - 5;   // = 4
    const uint8_t out_size = 8 - 5;  // = 3
    FSoE::MasterConnectionConfig fc{};
    fc.slave_safety_addr = 0x0001;
    fc.input_size = in_size;
    fc.output_size = out_size;
    FSoE::FSoEMasterConnection conn(fc);
    EXPECT_FALSE(conn.isOperational());
    EXPECT_FALSE(conn.isFailSafe());
}

TEST(SafetyTerminal, FsoeHandshakeOverPdoBuffers) {
    // Replicate SafetyTerminal::exchange(): run the FSoE state machine
    // through raw PDO buffers against the FSoESlave emulator.  Mirrors
    // the EL1918/EL2912 geometry: 4B safe inputs + 3B safe outputs with
    // a 1-cycle EtherCAT pipeline delay (master TX in cycle N is
    // answered by the slave's TxPDO in cycle N+1).
    FSoE::FSoESlaveConfig sc{};
    sc.slaveAddress = 0x1001;
    sc.safetyAddress = 0x0001;
    sc.connectionId = 0x0001;
    sc.safetyLevel = FSoE::SIL::SIL2;
    sc.watchdogTimeoutMs = 200;
    sc.connectionTimeoutMs = 5000;
    sc.sessionTimeoutMs = 10000;
    sc.safeInputSize = 4;
    sc.safeOutputSize = 3;
    sc.autoRecoveryEnabled = false;
    FSoE::FSoESlave slave(sc);
    slave.initialize();

    FSoE::MasterConnectionConfig mc{};
    mc.slave_addr = 0x1001;
    mc.slave_safety_addr = 0x0001;
    mc.connection_id = 0x0001;
    mc.master_addr = 0x1001;
    mc.watchdog_timeout_ms = 200;
    mc.conn_timeout_ms = 5000;
    mc.safety_level = FSoE::SIL::SIL2;
    mc.input_size = 4;
    mc.output_size = 3;
    mc.slave_response_delay_cycles = 1;
    FSoE::FSoEMasterConnection conn(mc);
    ASSERT_TRUE(conn.initialize());

    // Sequence trace for diagnosing handshake stalls.
    std::string trace;
    conn.setSequenceTraceCallback(
        [&trace](const FSoE::SequenceTraceInfo& i) {
            if (i.cycle > 30) return;    // cap the log
            trace += std::format("cyc{} {}->{} rx:{} tx:{} cmd:{} {}\n",
                                 i.cycle, i.state_before, i.state_after,
                                 i.frame_accepted, i.tx_rebuilt,
                                 i.rx_cmd, i.reason ? i.reason : "");
        });

    // PDO images carry the FSoE frame (safe payload + protocol overhead).
    const size_t rx_pdo_size = FSoE::CRC::fsoeFrameSize(sc.safeInputSize);
    const size_t tx_pdo_size = FSoE::CRC::fsoeFrameSize(mc.output_size);
    std::vector<uint8_t> last_resp(rx_pdo_size, 0);

    uint64_t now = 0;
    bool reached_data = false;
    for (int cycle = 0; cycle < 400 && !reached_data; ++cycle) {
        now += 15;
        // Master: consume the slave's current TxPDO, build next RX frame.
        uint8_t out_img[64] = {};
        conn.exchangeViaPDO(out_img, tx_pdo_size,
                            last_resp.data(), last_resp.size(), now);
        // Slave: process the frame and update its TxPDO for next cycle.
        slave.update(now);
        slave.processRxFrame(out_img, tx_pdo_size);
        uint8_t resp[64] = {};
        const size_t n = slave.prepareTxFrame(resp, sizeof(resp));
        last_resp.assign(resp, resp + n);
        if (last_resp.size() < rx_pdo_size) last_resp.resize(rx_pdo_size, 0);

        reached_data = conn.isOperational();
        ASSERT_FALSE(conn.isFailSafe()) << "entered fail-safe during "
                                        << "handshake at cycle " << cycle
                                        << "\n" << trace;
    }
    EXPECT_TRUE(reached_data) << trace;
}

TEST(SafetyTerminal, SafeIoBitAccess) {
    FSoE::MasterConnectionConfig mc{};
    mc.slave_addr = 0x1001;
    mc.slave_safety_addr = 0x0001;
    mc.connection_id = 0x0001;
    mc.master_addr = 0x1001;
    mc.watchdog_timeout_ms = 200;
    mc.conn_timeout_ms = 5000;
    mc.safety_level = FSoE::SIL::SIL2;
    mc.input_size = 4;
    mc.output_size = 3;
    FSoE::FSoEMasterConnection conn(mc);
    ASSERT_TRUE(conn.initialize());
    // Safe output bits round-trip into the staged output data.
    EXPECT_TRUE(conn.setSafeOutputBit(0, true));
    EXPECT_TRUE(conn.setSafeOutputBit(7, true));
    EXPECT_FALSE(conn.setSafeOutputBit(64, true));   // out of range
    EXPECT_FALSE(conn.getSafeInputBit(0));           // no data yet
}

#endif // TETHER_BECKHOFF_TEST_FSOE

#endif // TETHER_ENABLE_SII

// ============================================================================
// Tests that don't need SII
// ============================================================================

TEST(PositionTrackerNoSii, SixtyFourBitCounter) {
    PositionTracker t(64);
    // Wrapping a 64-bit counter forward by 6 via wrapping arithmetic.
    EXPECT_EQ(t.update(~uint64_t{0} - 4), -5);
    EXPECT_EQ(t.update(0), 0);
    EXPECT_EQ(t.turns(), 0);   // meaningless for 64-bit counters
}
