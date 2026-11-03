/**
 * @file test_rsap_information_processor.cpp
 * @brief Static-API coverage for RSAPInformationProcessor: the error-code
 *        table, scalar value names, snapshot decode (all/triggered),
 *        anyTriggered, and the five-panel DisplayPanel model — driven by
 *        fabricated snapshots, no wire needed.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "tether/drives/NexcobotESC211/RSAPInformationProcessor.hpp"

using Proc = EtherCAT::Drives::ESC211::RSAPInformationProcessor;
using EtherCAT::Drives::ESC211::RSAPInformationProcessor;

namespace {

/// Mark every object read-ok, then patch specific (index, subindex) slots.
struct SnapBuilder {
    Proc::Snapshot snap{};
    SnapBuilder() {
        snap.ok.fill(true);
        snap.read_ok = Proc::kObjectCount;
    }
    SnapBuilder& set(uint16_t index, uint8_t subindex, uint32_t value) {
        for (size_t i = 0; i < Proc::kObjectCount; ++i) {
            const auto& o = Proc::kObjects[i];
            if (o.index == index && o.subindex == subindex) {
                snap.value[i] = value;
                return *this;
            }
        }
        ADD_FAILURE() << "no kObjects slot for 0x" << std::hex << index;
        return *this;
    }
    SnapBuilder& fail(uint16_t index, uint8_t subindex) {
        for (size_t i = 0; i < Proc::kObjectCount; ++i) {
            const auto& o = Proc::kObjects[i];
            if (o.index == index && o.subindex == subindex) {
                snap.ok[i] = false;
                --snap.read_ok;
                return *this;
            }
        }
        ADD_FAILURE() << "no kObjects slot for 0x" << std::hex << index;
        return *this;
    }
};

bool hasCell(const Proc::DisplayPanel& p, const std::string& text) {
    for (const auto& r : p.rows)
        for (const auto& c : r.cells)
            if (c.text == text) return true;
    return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// errorCodeName — sweep the RSAP/CONFIG/STO/SYSMGR/FSoE ranges + gaps.
// ---------------------------------------------------------------------------
TEST(RSAPInfoStatic, ErrorCodeNameTable) {
    EXPECT_STREQ(RSAPInformationProcessor::errorCodeName(0), "success");
    // RSAP range ends and interior
    for (int32_t c : {-1, -9, -17}) {
        EXPECT_NE(RSAPInformationProcessor::errorCodeName(c), nullptr) << c;
    }
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-18), nullptr);
    // Safety DI/DO diagnosis ranges (inclusive boundaries)
    EXPECT_STREQ(RSAPInformationProcessor::errorCodeName(-310),
                 "Safety DI diagnosis error");
    EXPECT_STREQ(RSAPInformationProcessor::errorCodeName(-300),
                 "Safety DI diagnosis error");
    EXPECT_STREQ(RSAPInformationProcessor::errorCodeName(-330),
                 "Safety DO diagnosis error");
    EXPECT_STREQ(RSAPInformationProcessor::errorCodeName(-320),
                 "Safety DO diagnosis error");
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-299), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-331), nullptr);
    // CONFIG + STO
    EXPECT_NE(RSAPInformationProcessor::errorCodeName(-101), nullptr);
    EXPECT_NE(RSAPInformationProcessor::errorCodeName(-112), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-113), nullptr);
    EXPECT_NE(RSAPInformationProcessor::errorCodeName(-201), nullptr);
    EXPECT_NE(RSAPInformationProcessor::errorCodeName(-202), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-203), nullptr);
    // SYSMGR
    for (int32_t c : {-10000001, -10000002, -10000009, -10000010,
                      -10000011, -10000100, -10000101, -10000102,
                      -10000201, -10000301, -10000401}) {
        EXPECT_NE(RSAPInformationProcessor::errorCodeName(c), nullptr) << c;
    }
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-10000402), nullptr);
    // FSoE groups
    for (int32_t c : {-20001001, -20001007, -20002001, -20002004,
                      -20002013, -20006002, -20006004, -20007001,
                      -20007012}) {
        EXPECT_NE(RSAPInformationProcessor::errorCodeName(c), nullptr) << c;
    }
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-20001008), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-20002003), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-20007013), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(-999), nullptr);
    EXPECT_EQ(RSAPInformationProcessor::errorCodeName(1), nullptr);
}

// ---------------------------------------------------------------------------
// scalarName — every decoded index plus an unmapped index.
// ---------------------------------------------------------------------------
TEST(RSAPInfoStatic, ScalarNameTables) {
    // RSAP state 0..4 named, 5+ -> "Unknown"
    for (uint32_t v = 0; v < 5; ++v) {
        EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4000, v).empty());
    }
    EXPECT_EQ(RSAPInformationProcessor::scalarName(0x4000, 99), "Unknown");
    // Monitoring sub-state is a bitfield name composer
    EXPECT_TRUE(RSAPInformationProcessor::scalarName(0x4001, 0).empty());
    EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4001, 1).empty());
    EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4001, 0xFF).empty());
    // Error code: named vs unnamed
    EXPECT_STREQ(RSAPInformationProcessor::scalarName(0x4002, 0).c_str(),
                 "success");
    EXPECT_FALSE(
        RSAPInformationProcessor::scalarName(0x4002, 0xFF36).empty());  // -202
    EXPECT_TRUE(
        RSAPInformationProcessor::scalarName(0x4002, 0xBEEF).empty());  // unknown
    // Operation mode 0..3 named
    for (uint32_t v = 0; v < 4; ++v) {
        EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4003, v).empty());
    }
    EXPECT_EQ(RSAPInformationProcessor::scalarName(0x4003, 7), "Unknown");
    // Stop state 0..2 named
    for (uint32_t v = 0; v < 3; ++v) {
        EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4004, v).empty());
    }
    EXPECT_EQ(RSAPInformationProcessor::scalarName(0x4004, 9), "Unknown");
    // Stop category 0..4 named
    for (uint32_t v = 0; v < 5; ++v) {
        EXPECT_FALSE(RSAPInformationProcessor::scalarName(0x4005, v).empty());
    }
    EXPECT_EQ(RSAPInformationProcessor::scalarName(0x4005, 8), "Unknown");
    // Unmapped index -> empty
    EXPECT_TRUE(RSAPInformationProcessor::scalarName(0x4099, 0).empty());
}

// ---------------------------------------------------------------------------
// anyTriggered / decodeAll / decodeTriggered on fabricated snapshots.
// ---------------------------------------------------------------------------
TEST(RSAPInfoStatic, AnyTriggered) {
    Proc::Snapshot empty{};
    EXPECT_FALSE(Proc::anyTriggered(empty));              // all failed -> false
    EXPECT_FALSE(Proc::anyTriggered(SnapBuilder{}.snap)); // all ok, all zero

    auto bitfield = SnapBuilder{}.set(0x4006, 0, 0x01).snap;
    EXPECT_TRUE(Proc::anyTriggered(bitfield));

    auto err = SnapBuilder{}.set(0x4002, 0, 0xFF36).snap;  // -202
    EXPECT_TRUE(Proc::anyTriggered(err));

    // A set bit on an object that failed to read does not count.
    auto failed = SnapBuilder{}.set(0x4006, 0, 0xFF).fail(0x4006, 0).snap;
    EXPECT_FALSE(Proc::anyTriggered(failed));

    // Scalar non-error objects never trigger.
    auto scalar = SnapBuilder{}.set(0x4000, 0, 4).snap;
    EXPECT_FALSE(Proc::anyTriggered(scalar));
}

TEST(RSAPInfoStatic, DecodeAllFormats) {
    auto snap = SnapBuilder{}
                    .set(0x4000, 0, 4)        // RSAP state ERROR -> named
                    .set(0x4002, 0, 0xFF36)   // error code -> named
                    .set(0x4006, 0, 0x05)     // fault bits 0,2
                    .set(0x4007, 0, 0x80)     // likely-unlabeled bit
                    .fail(0x4019, 0)          // skipped entirely
                    .snap;
    const auto lines = Proc::decodeAll(snap);
    EXPECT_EQ(lines.size(), Proc::kObjectCount - 1);
    bool saw_named_scalar = false, saw_bitfield = false;
    for (const auto& l : lines) {
        if (l.find("0x4000") != std::string::npos &&
            l.find('[') != std::string::npos) {
            saw_named_scalar = true;
        }
        if (l.find("0x4006") != std::string::npos) {
            saw_bitfield = true;
            EXPECT_NE(l.find('('), std::string::npos);
        }
        EXPECT_EQ(l.find("0x4019"), std::string::npos);
    }
    EXPECT_TRUE(saw_named_scalar);
    EXPECT_TRUE(saw_bitfield);
}

TEST(RSAPInfoStatic, DecodeTriggered) {
    EXPECT_TRUE(Proc::decodeTriggered(SnapBuilder{}.snap).empty());

    auto snap = SnapBuilder{}
                    .set(0x4002, 0, 0xFF36)   // scalar error code
                    .set(0x4006, 0, 0x01)     // bitfield, labeled bit
                    .set(0x4007, 0, 0x80)     // bitfield, unlabeled bit
                    .set(0x4000, 0, 4)        // scalar non-error: ignored
                    .snap;
    const auto lines = Proc::decodeTriggered(snap);
    ASSERT_EQ(lines.size(), 3u);
    bool saw_err = false, saw_labeled = false, saw_raw_bit = false;
    for (const auto& l : lines) {
        if (l.find("0x4002") != std::string::npos) saw_err = true;
        if (l.find("0x4006") != std::string::npos) {
            saw_labeled = true;
            EXPECT_EQ(l.find("bit 0"), std::string::npos);  // named, not raw
        }
        if (l.find("0x4007") != std::string::npos) {
            saw_raw_bit = true;
            EXPECT_NE(l.find("bit 7"), std::string::npos);  // unnamed fallback
        }
    }
    EXPECT_TRUE(saw_err && saw_labeled && saw_raw_bit);
}

// ---------------------------------------------------------------------------
// buildDisplayPanels — every tone branch across the five panels.
// ---------------------------------------------------------------------------
TEST(RSAPInfoStatic, DisplayPanelsAllBranches) {
    auto snap = SnapBuilder{}
                    .set(0x4000, 0, 4)        // RSAP ERROR -> Fault tone
                    .set(0x4001, 0, 0x03)     // sub-state bits
                    .set(0x4003, 0, 2)        // op mode
                    .set(0x4004, 0, 3)        // stop state >2 -> Warn
                    .set(0x4005, 0, 5)        // stop category >4 -> Warn
                    .set(0x4002, 0, 0xFF36)   // error code -> named Fault
                    .set(0x4006, 0, 0x01)     // fault bits
                    .set(0x400A, 0, 0x0001)   // input discrepancy
                    .set(0x4008, 1, 0x01)     // pos valid: D1 ok, D2 missing
                    .set(0x4008, 2, 0x03)     // vel valid: both ok
                    .set(0x4008, 3, 0x02)     // err: D2 fault
                    .set(0x4008, 4, 0x03)     // sto: both ok
                    .set(0x4008, 5, 0x01)     // sos: D2 missing
                    .set(0x4007, 0, 0x05)     // drive status bits
                    .set(0x4009, 1, 0xFF)     // user bits incl. unlabeled
                    .set(0x4009, 2, 0x00)     // none
                    .set(0x400B, 0, 0x01)     // EStop wired
                    .set(0x4015, 0, 0x01)     // output discrepancy
                    .set(0x4016, 0, 0x01)     // safety out set
                    .set(0x4017, 0, 0x01)     // control fn set
                    .set(0x4018, 0, 0x01)     // limit fn set
                    .set(0x4019, 0, 0x01)     // axis limit hit
                    .fail(0x400C, 0)          // n/a row in Safety inputs
                    .snap;

    const auto panels = Proc::buildDisplayPanels(snap, /*expected_drives=*/2);
    ASSERT_EQ(panels.size(), 5u);

    // Panel 1: state ERROR -> Fault cell, named error code appended.
    const auto& p1 = panels[0];
    EXPECT_NE(p1.title.find("RSAP"), std::string::npos);
    bool saw_fault_cell = false;
    for (const auto& r : p1.rows)
        for (const auto& c : r.cells)
            if (c.tone == Proc::DisplayTone::Fault) saw_fault_cell = true;
    EXPECT_TRUE(saw_fault_cell);
    EXPECT_TRUE(hasCell(p1, "-202"));

    // Panel 2: drive matrix — Ok, Missing and Fault tones all present.
    const auto& p2 = panels[1];
    EXPECT_TRUE(hasCell(p2, "D1") && hasCell(p2, "D2"));
    EXPECT_TRUE(hasCell(p2, "█"));
    bool saw_missing = false;
    for (const auto& r : p2.rows)
        for (const auto& c : r.cells)
            if (c.tone == Proc::DisplayTone::Missing) saw_missing = true;
    EXPECT_TRUE(saw_missing);
    EXPECT_TRUE(hasCell(p2, "none"));    // usr D2 row: zero value
    EXPECT_TRUE(hasCell(p2, "U1") && hasCell(p2, "U8"));  // usr D1: all bits

    // Panel 3: safety inputs — expected-set semantics + n/a row.
    const auto& p3 = panels[2];
    EXPECT_TRUE(hasCell(p3, "n/a"));     // 0x400C failed

    // Panels 4+5 exist with their titles.
    EXPECT_NE(panels[3].title.find("Outputs"), std::string::npos);
    EXPECT_NE(panels[4].title.find("Limits"), std::string::npos);
}

TEST(RSAPInfoStatic, DisplayPanelsDriveCountDerivationAndClamp) {
    // expected_drives=0 derives the count from the highest set drive bit.
    auto derived = Proc::buildDisplayPanels(
        SnapBuilder{}.set(0x4008, 1, 0x07).snap, 0);
    EXPECT_TRUE(hasCell(derived[1], "D3"));
    EXPECT_FALSE(hasCell(derived[1], "D4"));

    // >7 clamps to 7.
    auto clamped = Proc::buildDisplayPanels(SnapBuilder{}.snap, 9);
    EXPECT_TRUE(hasCell(clamped[1], "D7"));
    EXPECT_FALSE(hasCell(clamped[1], "D8"));

    // All-zero snapshot: "0 success" error cell, "none"/Muted fallbacks.
    auto empty = Proc::buildDisplayPanels(SnapBuilder{}.snap, 1);
    EXPECT_TRUE(hasCell(empty[0], "0 success"));
}

// ---------------------------------------------------------------------------
// kObjects table sanity — index order + byte widths.
// ---------------------------------------------------------------------------
TEST(RSAPInfoStatic, ObjectTableSanity) {
    uint16_t last = 0;
    for (const auto& o : Proc::kObjects) {
        EXPECT_GE(o.index, last);
        last = o.index;
        EXPECT_TRUE(o.bytes == 1 || o.bytes == 2);
        EXPECT_NE(o.name, nullptr);
    }
    EXPECT_EQ(Proc::kObjects.front().index, 0x4000);
    EXPECT_EQ(Proc::kObjects.back().index, 0x4023);
}
