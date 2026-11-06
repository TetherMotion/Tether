/**
 * @file test_pdo_utils.cpp
 * @brief Coverage for EtherCAT::Utils PDO helpers (src/ethercat/utils/PDO.cpp):
 *        pdoToString formatting (all field-size / bounds / sign-extension
 *        branches), pdoMappingToString, readPDOMapping over a mocked CoE
 *        transport, and the printPDOMapping logging wrapper.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/utils/PDO.hpp"

using namespace EtherCAT;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;

namespace {

// ============================================================================
// pdoToString — pure formatting
// ============================================================================

TEST(PdoToStringCov, HexdumpWithoutFields) {
    uint8_t buf[20];
    for (int i = 0; i < 20; ++i) buf[i] = static_cast<uint8_t>(i);
    std::string out = Utils::pdoToString(true, 0x1A00, buf, sizeof(buf));
    EXPECT_NE(out.find("TxPDO 0x1A00"), std::string::npos);
    EXPECT_NE(out.find("0000:"), std::string::npos);
    EXPECT_NE(out.find(" 13"), std::string::npos);   // second line, byte 0x13
}

TEST(PdoToStringCov, EmptyBufferHexdump) {
    std::string out = Utils::pdoToString(false, 0x1600, nullptr, 0);
    EXPECT_NE(out.find("RxPDO 0x1600"), std::string::npos);
    EXPECT_NE(out.find("<empty>"), std::string::npos);
}

TEST(PdoToStringCov, FieldsNoBuffer) {
    Utils::PDOFieldDescriptor fields[1] = {};
    fields[0].index = 0x6041;
    fields[0].subindex = 0;
    fields[0].offset = 0;
    fields[0].size = 2;
    fields[0].description = "status";
    std::string out = Utils::pdoToString(true, 0x1A00, nullptr, 0, fields, 1);
    EXPECT_NE(out.find("<no buffer>"), std::string::npos);
    EXPECT_NE(out.find("status"), std::string::npos);
}

TEST(PdoToStringCov, FieldOutOfRange) {
    uint8_t buf[4] = {};
    Utils::PDOFieldDescriptor fields[1] = {};
    fields[0].index = 0x6064;
    fields[0].offset = 2;
    fields[0].size = 8;                 // 2+8 > 4 → out of range
    fields[0].description = nullptr;    // "<unnamed>" fallback
    std::string out = Utils::pdoToString(true, 0x1A00, buf, sizeof(buf), fields, 1);
    EXPECT_NE(out.find("<out of range>"), std::string::npos);
    EXPECT_NE(out.find("<unnamed>"), std::string::npos);
}

TEST(PdoToStringCov, FieldSizesAndSignExtension) {
    uint8_t buf[16] = {};
    buf[0] = 0xFF;                        // s8  = -1
    buf[1] = 0xFF; buf[2] = 0x7F;         // s16 = 0x7FFF
    buf[3] = 0xFF; buf[4] = 0xFF;
    buf[5] = 0xFF; buf[6] = 0xFF;         // s32 = -1
    buf[7] = 0x01; buf[8] = 0x00;
    buf[9] = 0x00; buf[10] = 0x00;
    buf[11] = 0x00; buf[12] = 0x00;
    buf[13] = 0x00; buf[14] = 0x80;       // s64 = negative via sign bit

    Utils::PDOFieldDescriptor fields[4] = {};
    fields[0] = {0x6000, 0, 0, 1, "u8"};
    fields[1] = {0x6001, 0, 1, 2, "u16"};
    fields[2] = {0x6002, 0, 3, 4, "u32"};
    fields[3] = {0x6003, 0, 7, 8, "u64"};
    std::string out = Utils::pdoToString(true, 0x1A00, buf, sizeof(buf), fields, 4);
    EXPECT_NE(out.find("u8"), std::string::npos);
    EXPECT_NE(out.find("(-1)"), std::string::npos);    // sign-extended s8
    EXPECT_NE(out.find("(32767)"), std::string::npos); // s16
    EXPECT_NE(out.find("u64"), std::string::npos);
}

TEST(PdoToStringCov, OddFieldSizePrintsRawOnly) {
    uint8_t buf[4] = {0xAA, 0xBB, 0xCC, 0x00};
    Utils::PDOFieldDescriptor fields[1] = {};
    fields[0] = {0x6004, 0, 0, 3, "odd3"};   // size 3 → raw hex only, no (dec)
    std::string out = Utils::pdoToString(false, 0x1600, buf, sizeof(buf), fields, 1);
    EXPECT_NE(out.find("odd3"), std::string::npos);
    EXPECT_NE(out.find("0xCCBBAA"), std::string::npos);
    EXPECT_EQ(out.find("(-"), std::string::npos);      // no signed parens
}

// ============================================================================
// pdoMappingToString — pure formatting
// ============================================================================

TEST(PdoMappingToStringCov, EmptyList) {
    std::string out = Utils::pdoMappingToString(true, 0x1A00, {});
    EXPECT_NE(out.find("TxPDO 0x1A00"), std::string::npos);
    EXPECT_NE(out.find("Total mapped: 0 bytes"), std::string::npos);
}

TEST(PdoMappingToStringCov, EntriesFormat) {
    std::vector<Utils::PDOMappingEntry> e(2);
    e[0] = {0x6041, 0, 16, 0};
    e[1] = {0x6064, 0, 32, 2};
    std::string out = Utils::pdoMappingToString(false, 0x1600, e);
    EXPECT_NE(out.find("RxPDO 0x1600"), std::string::npos);
    EXPECT_NE(out.find("0x6041"), std::string::npos);
    EXPECT_NE(out.find("0x6064"), std::string::npos);
    EXPECT_NE(out.find("Total mapped: 6 bytes"), std::string::npos);  // 2+32/8
}

// ============================================================================
// readPDOMapping — needs CoEManager + mocked SDO transport
// ============================================================================

uint64_t realMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class MockSDOTransport : public SDO::ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, uint8_t*, size_t, size_t*, bool,
                 unsigned int, unsigned int), (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, const uint8_t*, size_t, bool,
                 unsigned int, unsigned int), (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

/// Scripted upload: returns per-subindex data for one object index.
auto uploadBySub(std::vector<std::pair<uint8_t, std::vector<uint8_t>>> table,
                 std::vector<uint8_t> def = {}) {
    return [table, def](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                        uint16_t, uint8_t sub, uint8_t* out, size_t, size_t* out_len,
                        bool, unsigned int, unsigned int) -> bool {
        for (const auto& [s, d] : table) {
            if (s == sub) {
                memcpy(out, d.data(), d.size());
                if (out_len) *out_len = d.size();
                return true;
            }
        }
        if (!def.empty()) {
            memcpy(out, def.data(), def.size());
            if (out_len) *out_len = def.size();
            return true;
        }
        return false;
    };
}

TEST(ReadPdoMappingCov, ReadsCountThenEntries) {
    NiceMock<MockSDOTransport> t;
    ON_CALL(t, getMicroseconds()).WillByDefault(Invoke(&realMicros));
    uint32_t e1 = (0x6041u << 16) | (0u << 8) | 16u;   // 0x6041:00 16 bits
    uint32_t e2 = (0x6064u << 16) | (0u << 8) | 32u;   // 0x6064:00 32 bits
    ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(uploadBySub({
            {0, {2}},                                                  // count
            {1, {uint8_t(e1), uint8_t(e1>>8), uint8_t(e1>>16), uint8_t(e1>>24)}},
            {2, {uint8_t(e2), uint8_t(e2>>8), uint8_t(e2>>16), uint8_t(e2>>24)}},
        })));

    CoE::CoEManager mgr(0, t);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    std::vector<Utils::PDOMappingEntry> entries;
    ASSERT_TRUE(Utils::readPDOMapping(mgr, 0x1A00, entries));
    ASSERT_EQ(entries.size(), 2u);
    EXPECT_EQ(entries[0].index, 0x6041);
    EXPECT_EQ(entries[0].bit_length, 16);
    EXPECT_EQ(entries[0].byte_offset, 0);
    EXPECT_EQ(entries[1].index, 0x6064);
    EXPECT_EQ(entries[1].bit_length, 32);
    EXPECT_EQ(entries[1].byte_offset, 2);        // ran after 16 bits
    mgr.deinit();
}

TEST(ReadPdoMappingCov, CountReadFails) {
    NiceMock<MockSDOTransport> t;
    ON_CALL(t, getMicroseconds()).WillByDefault(Invoke(&realMicros));
    ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(::testing::Return(false));

    CoE::CoEManager mgr(0, t);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    std::vector<Utils::PDOMappingEntry> entries;
    EXPECT_FALSE(Utils::readPDOMapping(mgr, 0x1A00, entries));
    EXPECT_TRUE(entries.empty());
    mgr.deinit();
}

TEST(ReadPdoMappingCov, EntryFailureKeepsPartial) {
    NiceMock<MockSDOTransport> t;
    ON_CALL(t, getMicroseconds()).WillByDefault(Invoke(&realMicros));
    uint32_t e1 = (0x6041u << 16) | 16u;
    ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(uploadBySub({
            {0, {3}},                                                  // count 3
            {1, {uint8_t(e1), uint8_t(e1>>8), uint8_t(e1>>16), uint8_t(e1>>24)}},
            // sub 2 missing → readU32 fails → break with 1 entry
        })));

    CoE::CoEManager mgr(0, t);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    std::vector<Utils::PDOMappingEntry> entries;
    ASSERT_TRUE(Utils::readPDOMapping(mgr, 0x1A00, entries));
    EXPECT_EQ(entries.size(), 1u);
    mgr.deinit();
}

TEST(ReadPdoMappingCov, ZeroCountReturnsEmptySuccess) {
    NiceMock<MockSDOTransport> t;
    ON_CALL(t, getMicroseconds()).WillByDefault(Invoke(&realMicros));
    ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(uploadBySub({{0, {0}}})));   // count = 0

    CoE::CoEManager mgr(0, t);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    std::vector<Utils::PDOMappingEntry> entries;
    ASSERT_TRUE(Utils::readPDOMapping(mgr, 0x1A00, entries));
    EXPECT_TRUE(entries.empty());
    mgr.deinit();
}

// ============================================================================
// printPDOMapping — logging wrapper (both branches)
// ============================================================================

TEST(PrintPdoMappingCov, SuccessAndFailurePaths) {
    NiceMock<MockSDOTransport> t;
    ON_CALL(t, getMicroseconds()).WillByDefault(Invoke(&realMicros));

    CoE::CoEManager mgr(0, t);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    // First: success path — count = 1 + one entry.
    {
        uint32_t e1 = (0x6040u << 16) | 16u;
        ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(uploadBySub({
                {0, {1}},
                {1, {uint8_t(e1), uint8_t(e1>>8), uint8_t(e1>>16), uint8_t(e1>>24)}},
            })));
        Utils::printPDOMapping(mgr, true, 0x1A00, "pdo_test", 200);
    }
    // Then: failure path — all uploads fail → WARN branch.
    {
        ON_CALL(t, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(::testing::Return(false));
        Utils::printPDOMapping(mgr, false, 0x1600, "pdo_test", 200);
    }
    mgr.deinit();
    SUCCEED();
}

}  // namespace
