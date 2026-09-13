/**
 * @file test_sii_demand_parser.cpp
 * @brief Comprehensive tests for the demand-driven SII parser
 *
 * Tests cover:
 *   - Parsing entirely from a pre-populated cache
 *   - Cache misses returning exact required word-pair addresses
 *   - Non-contiguous required word ranges
 *   - Re-running after adding requested words
 *   - Completion after multiple iterations
 *   - Variable category placement and sizes
 *   - End markers and unknown categories
 *   - No EEPROM-size assumption
 *   - EEPROM size learned from data and used as boundary
 *   - Feature masks and implicit string dependencies
 *   - Byte reads crossing word boundaries
 *   - Category data parsing (strings, general, FMMU, sync managers, PDOs, DC)
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "tether/sii/SIIDemandParser.hpp"
#include "tether/sii/SIIManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::SII;

// ============================================================================
// Helpers
// ============================================================================

/// Populate a cache with word-pair data from a map.
static void populateCache(SIISlaveCache& cache,
                          const std::vector<std::pair<uint16_t, uint32_t>>& pairs) {
    for (const auto& [addr, val] : pairs) {
        cache.setWordPair(addr, val);
    }
}

/// Build a 32-bit word-pair from two 16-bit words.
static inline uint32_t makeDWord(uint16_t lo, uint16_t hi) {
    return static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
}

/// Build a 32-bit word-pair from 4 bytes (little-endian).
static inline uint32_t makeDWordBytes(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return static_cast<uint32_t>(b0) |
           (static_cast<uint32_t>(b1) << 8) |
           (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 24);
}

// ============================================================================
// Config Area Tests
// ============================================================================

class SIIDemandParserTest : public ::testing::Test {
protected:
    SIISlaveCache cache;
    SIIData data;
    SIIDemandParser parser;

    void SetUp() override {
        cache.clear();
        data = SIIData{};
    }
};

TEST_F(SIIDemandParserTest, EmptyCache_RequestsConfigArea) {
    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    EXPECT_TRUE(result.needsWords());
    ASSERT_EQ(result.needed_word_pairs.size(), 4u);
    EXPECT_EQ(result.needed_word_pairs[0], 0x0000u);
    EXPECT_EQ(result.needed_word_pairs[1], 0x0002u);
    EXPECT_EQ(result.needed_word_pairs[2], 0x0004u);
    EXPECT_EQ(result.needed_word_pairs[3], 0x0006u);
}

TEST_F(SIIDemandParserTest, ConfigArea_PartialCache_RequestsMissing) {
    // Provide only 2 of 4 word-pairs
    cache.setWordPair(0x0000, makeDWord(0x0001, 0x0002));
    cache.setWordPair(0x0002, makeDWord(0x0003, 0x0004));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    EXPECT_TRUE(result.needsWords());
    ASSERT_EQ(result.needed_word_pairs.size(), 2u);
    EXPECT_EQ(result.needed_word_pairs[0], 0x0004u);
    EXPECT_EQ(result.needed_word_pairs[1], 0x0006u);
}

TEST_F(SIIDemandParserTest, ConfigArea_Complete_AdvancesToIdentity) {
    // Provide all config area word-pairs
    cache.setWordPair(0x0000, makeDWord(0x0001, 0x0002));
    cache.setWordPair(0x0002, makeDWord(0x0003, 0x0004));
    cache.setWordPair(0x0004, makeDWord(0x0005, 0x0006));
    cache.setWordPair(0x0006, makeDWord(0x0007, 0x0008));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    // Config area done, now needs identity words
    EXPECT_TRUE(result.needsWords());
    // Should request identity word-pairs
    bool has_0x0008 = false;
    for (auto addr : result.needed_word_pairs) {
        if (addr == 0x0008) has_0x0008 = true;
    }
    EXPECT_TRUE(has_0x0008);
}

// ============================================================================
// Identity Tests
// ============================================================================

TEST_F(SIIDemandParserTest, Identity_ParsesVendorProductRevisionSerial) {
    // Config area
    cache.setWordPair(0x0000, makeDWord(0x0001, 0x0002));
    cache.setWordPair(0x0002, makeDWord(0x0003, 0x0004));
    cache.setWordPair(0x0004, makeDWord(0x0005, 0x0006));
    cache.setWordPair(0x0006, makeDWord(0x0007, 0x0008));
    // Identity
    cache.setWordPair(0x0008, makeDWord(0x0002, 0x0000));   // vendor lo=0x0002, hi=0x0000
    cache.setWordPair(0x000A, makeDWord(0x3052, 0x03F6));   // product lo=0x3052, hi=0x03F6
    cache.setWordPair(0x000C, makeDWord(0x0000, 0x0012));   // revision
    cache.setWordPair(0x000E, makeDWord(0x0000, 0x0000));   // serial
    // Mailbox
    cache.setWordPair(0x0014, makeDWord(0, 0));
    cache.setWordPair(0x0016, makeDWord(0, 0));
    cache.setWordPair(0x0018, makeDWord(0, 0));
    cache.setWordPair(0x001A, makeDWord(0, 0));
    cache.setWordPair(0x001C, makeDWord(0, 0));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    // Should advance past identity and request size info (0x002E)
    EXPECT_TRUE(result.needsWords());
    EXPECT_EQ(data.identity.vendor_id, 0x00000002u);
    EXPECT_EQ(data.identity.product_code, 0x03F63052u);
    EXPECT_EQ(data.identity.revision_number, 0x00120000u);
    EXPECT_EQ(data.identity.serial_number, 0x00000000u);
}

// ============================================================================
// Size Info Tests
// ============================================================================

TEST_F(SIIDemandParserTest, SizeInfo_ZeroKbits_NoBoundaryAssumption) {
    // Full config + identity + mailbox
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    // Size info: 0 kbits
    cache.setWordPair(0x002E, makeDWord(0, 0));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    // With 0 kbits, eeprom_size_words is 0, so no boundary check.
    // Should advance to category scan and request 0x0040.
    EXPECT_TRUE(result.needsWords());
    EXPECT_EQ(data.eeprom_size_kbits, 0);
    EXPECT_EQ(data.eeprom_size_words, 0);
    bool has_0x0040 = false;
    for (auto addr : result.needed_word_pairs) {
        if (addr == 0x0040) has_0x0040 = true;
    }
    EXPECT_TRUE(has_0x0040);
}

TEST_F(SIIDemandParserTest, SizeInfo_NonZeroKbits_LearnsBoundary) {
    // Full config + identity + mailbox
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    // Size info: 16 kbits = 1024 words
    cache.setWordPair(0x002E, makeDWord(16, 0));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    EXPECT_EQ(data.eeprom_size_kbits, 16);
    EXPECT_EQ(data.eeprom_size_words, 1024);
}

// ============================================================================
// Category Scan Tests
// ============================================================================

TEST_F(SIIDemandParserTest, CategoryScan_CatEnd_CompletesImmediately) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));
    // Category: CAT_END (type=0xFFFF)
    cache.setWordPair(0x0040, makeDWord(0xFFFF, 0));

    parser.init(CAT_MASK_ALL);
    auto result = parser.parse(cache, data);

    EXPECT_TRUE(result.isComplete());
    EXPECT_TRUE(data.parse_complete);
}

TEST_F(SIIDemandParserTest, CategoryScan_BlankHeaders_Completes) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));
    // Category: blank (type=0, size=0)
    cache.setWordPair(0x0040, makeDWord(0, 0));

    parser.init(CAT_MASK_ALL);
    auto result = parser.parse(cache, data);

    EXPECT_TRUE(result.isComplete());
}

TEST_F(SIIDemandParserTest, CategoryScan_UnknownCategory_Skipped) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));
    // Unknown category (type=0x0099, size=4) then CAT_END
    cache.setWordPair(0x0040, makeDWord(0x0099, 4));
    // Skip 4 words of data (2 word-pairs)
    cache.setWordPair(0x0042, makeDWord(0, 0));
    cache.setWordPair(0x0044, makeDWord(0, 0));
    // CAT_END at 0x0046
    cache.setWordPair(0x0046, makeDWord(0xFFFF, 0));

    parser.init(CAT_MASK_ALL);
    auto result = parser.parse(cache, data);

    EXPECT_TRUE(result.isComplete());
}

TEST_F(SIIDemandParserTest, CategoryScan_RequestedCategory_NeedsData) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));
    // FMMU category (type=0x002A, size=2) — data at 0x0042-0x0043
    cache.setWordPair(0x0040, makeDWord(CAT_FMMU, 2));

    parser.init(CAT_MASK_FMMU);
    auto result = parser.parse(cache, data);

    // Should request the data word-pair at 0x0042
    EXPECT_TRUE(result.needsWords());
    ASSERT_EQ(result.needed_word_pairs.size(), 1u);
    EXPECT_EQ(result.needed_word_pairs[0], 0x0042u);
}

TEST_F(SIIDemandParserTest, CategoryScan_UnrequestedCategory_SkipsData) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));
    // FMMU category (type=0x002A, size=2) — NOT requested
    cache.setWordPair(0x0040, makeDWord(CAT_FMMU, 2));
    // CAT_END at 0x0044 (after skipping 2 words of FMMU data)
    cache.setWordPair(0x0044, makeDWord(0xFFFF, 0));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    // Should skip FMMU data and find CAT_END
    EXPECT_TRUE(result.isComplete());
}

TEST_F(SIIDemandParserTest, CategoryScan_OddSizeCategory_NextHeaderAtOddAddr) {
    // Regression test: when a category has an odd word count, the next
    // category header starts at an odd word address. The parser must
    // correctly read the header using readWord() (which handles odd
    // addresses) rather than readWordPair() (which aligns to even).
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));

    // Category at 0x0040: CAT_FMMU (type=40), size=3 (ODD) — not requested
    // Header word-pair 0x0040: [40, 3]
    cache.setWordPair(0x0040, makeDWord(40, 3));
    // Data: 3 words at 0x0042, 0x0043, 0x0044 (not parsed, not requested)
    cache.setWordPair(0x0042, makeDWord(0, 0));
    // Word 0x0044 = 0 (3rd data word), word 0x0045 = 0xFFFF (CAT_END type)
    // Next category header at ODD word address 0x0045!
    cache.setWordPair(0x0044, makeDWord(0, 0xFFFF));
    // Word 0x0046 = 0 (CAT_END size), word 0x0047 = 0
    cache.setWordPair(0x0046, makeDWord(0, 0));

    parser.init(CAT_MASK_NONE);
    auto result = parser.parse(cache, data);

    // Should skip the FMMU data and find CAT_END at the odd address
    EXPECT_TRUE(result.isComplete());
}

// ============================================================================
// Multi-Iteration Tests
// ============================================================================

TEST_F(SIIDemandParserTest, MultiIteration_CompletesAfterFetchingWords) {
    // Start with empty cache, simulate fetching words iteratively
    parser.init(CAT_MASK_NONE);

    // Iteration 1: requests config area
    auto r1 = parser.parse(cache, data);
    EXPECT_TRUE(r1.needsWords());
    EXPECT_EQ(r1.needed_word_pairs.size(), 4u);

    // Fetch config area
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));

    // Iteration 2: config done, requests identity
    auto r2 = parser.parse(cache, data);
    EXPECT_TRUE(r2.needsWords());

    // Fetch identity + mailbox
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));

    // Iteration 3: identity done, requests size info
    auto r3 = parser.parse(cache, data);
    EXPECT_TRUE(r3.needsWords());

    // Fetch size info
    cache.setWordPair(0x002E, makeDWord(0, 0));

    // Iteration 4: size info done, requests category scan
    auto r4 = parser.parse(cache, data);
    EXPECT_TRUE(r4.needsWords());

    // Fetch CAT_END
    cache.setWordPair(0x0040, makeDWord(0xFFFF, 0));

    // Iteration 5: complete
    auto r5 = parser.parse(cache, data);
    EXPECT_TRUE(r5.isComplete());
    EXPECT_TRUE(data.parse_complete);
}

// ============================================================================
// Implicit String Dependency Tests
// ============================================================================

TEST_F(SIIDemandParserTest, GeneralCategory_ImplicitlyRequestsStrings) {
    // Full config + identity + mailbox + size info
    for (uint16_t a = 0x0000; a <= 0x0006; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x0008, makeDWord(0, 0));
    cache.setWordPair(0x000A, makeDWord(0, 0));
    cache.setWordPair(0x000C, makeDWord(0, 0));
    cache.setWordPair(0x000E, makeDWord(0, 0));
    for (uint16_t a = 0x0014; a <= 0x001C; a += 2)
        cache.setWordPair(a, makeDWord(0, 0));
    cache.setWordPair(0x002E, makeDWord(0, 0));

    // Requesting CAT_MASK_GENERAL should implicitly include CAT_MASK_STRINGS
    parser.init(CAT_MASK_GENERAL);

    // The parser should request both STRINGS and GENERAL categories
    // First category header at 0x0040
    auto result = parser.parse(cache, data);
    EXPECT_TRUE(result.needsWords());
    EXPECT_EQ(result.needed_word_pairs[0], 0x0040u);
}

// ============================================================================
// Cache Reading Helper Tests
// ============================================================================

TEST_F(SIIDemandParserTest, ReadWordPair_Hit) {
    cache.setWordPair(0x0040, 0xDEADBEEF);
    uint32_t val = 0;
    EXPECT_TRUE(SIIDemandParser::readWordPair(cache, 0x0040, val));
    EXPECT_EQ(val, 0xDEADBEEFu);
}

TEST_F(SIIDemandParserTest, ReadWordPair_Miss) {
    uint32_t val = 0;
    EXPECT_FALSE(SIIDemandParser::readWordPair(cache, 0x0040, val));
}

TEST_F(SIIDemandParserTest, ReadWord_LowWord) {
    cache.setWordPair(0x0040, makeDWord(0x1234, 0x5678));
    uint16_t val = 0;
    EXPECT_TRUE(SIIDemandParser::readWord(cache, 0x0040, val));
    EXPECT_EQ(val, 0x1234u);
}

TEST_F(SIIDemandParserTest, ReadWord_HighWord) {
    cache.setWordPair(0x0040, makeDWord(0x1234, 0x5678));
    uint16_t val = 0;
    EXPECT_TRUE(SIIDemandParser::readWord(cache, 0x0041, val));
    EXPECT_EQ(val, 0x5678u);
}

TEST_F(SIIDemandParserTest, ReadBytes_CrossesWordBoundary) {
    // Two word-pairs: 0x0040 = [0xAABB, 0xCCDD], 0x0042 = [0xEEFF, 0x0011]
    cache.setWordPair(0x0040, makeDWordBytes(0xBB, 0xAA, 0xDD, 0xCC));
    cache.setWordPair(0x0042, makeDWordBytes(0xFF, 0xEE, 0x11, 0x00));

    // Read 6 bytes starting at byte address 0x81 (word 0x0040, byte 1)
    // Word 0x0040 bytes: [0xBB, 0xAA, 0xDD, 0xCC]
    // Starting at byte offset 1: 0xAA, 0xDD, 0xCC, then word 0x0042: 0xFF, 0xEE, 0x11
    uint8_t buf[6] = {};
    size_t n = SIIDemandParser::readBytes(cache, 0x81, buf, 6);
    EXPECT_EQ(n, 6u);
    EXPECT_EQ(buf[0], 0xAA);
    EXPECT_EQ(buf[1], 0xDD);
    EXPECT_EQ(buf[2], 0xCC);
    EXPECT_EQ(buf[3], 0xFF);
    EXPECT_EQ(buf[4], 0xEE);
    EXPECT_EQ(buf[5], 0x11);
}

TEST_F(SIIDemandParserTest, FirstMissingWordPair_AllCached) {
    cache.setWordPair(0x0040, 0);
    cache.setWordPair(0x0042, 0);
    EXPECT_EQ(SIIDemandParser::firstMissingWordPair(cache, 0x0040, 4), 0xFFFF);
}

TEST_F(SIIDemandParserTest, FirstMissingWordPair_FirstMissing) {
    EXPECT_EQ(SIIDemandParser::firstMissingWordPair(cache, 0x0040, 4), 0x0040u);
}

TEST_F(SIIDemandParserTest, FirstMissingWordPair_SecondMissing) {
    cache.setWordPair(0x0040, 0);
    EXPECT_EQ(SIIDemandParser::firstMissingWordPair(cache, 0x0040, 4), 0x0042u);
}

// ============================================================================
// Shared Category Parsing Function Tests
// ============================================================================

TEST(SIIDemandParserSharedTest, ParseStringsFromBuffer_SingleString) {
    SIIData data;
    // 1 string of length 5: "Hello"
    uint8_t buf[] = {1, 5, 'H', 'e', 'l', 'l', 'o'};
    EXPECT_TRUE(parseStringsFromBuffer(buf, sizeof(buf), data));
    ASSERT_EQ(data.strings.count(), 1u);
    EXPECT_STREQ(data.strings.getString(1), "Hello");  // 1-based index
}

TEST(SIIDemandParserSharedTest, ParseStringsFromBuffer_MultipleStrings) {
    SIIData data;
    // 2 strings: "AB" and "CDE"
    uint8_t buf[] = {2, 2, 'A', 'B', 3, 'C', 'D', 'E'};
    EXPECT_TRUE(parseStringsFromBuffer(buf, sizeof(buf), data));
    ASSERT_EQ(data.strings.count(), 2u);
    EXPECT_STREQ(data.strings.getString(1), "AB");  // 1-based index
    EXPECT_STREQ(data.strings.getString(2), "CDE");
}

TEST(SIIDemandParserSharedTest, ParseGeneralFromBuffer_BasicFields) {
    SIIData data;
    uint8_t buf[20] = {};
    buf[0] = 1;  // group_idx
    buf[2] = 3;  // order_idx
    buf[3] = 4;  // name_idx
    buf[5] = 0x01;  // coe_details
    buf[9] = 2;  // ds402_channels
    buf[12] = 0x30;  // current_ebus lo (-2000 = 0xF830)
    buf[13] = 0xF8;  // current_ebus hi

    EXPECT_TRUE(parseGeneralFromBuffer(buf, sizeof(buf), data));
    EXPECT_TRUE(data.has_general);
    EXPECT_EQ(data.general.group_idx, 1);
    EXPECT_EQ(data.general.order_idx, 3);
    EXPECT_EQ(data.general.name_idx, 4);
    EXPECT_EQ(data.general.coe_details, 0x01);
    EXPECT_EQ(data.general.ds402_channels, 2);
    EXPECT_EQ(data.general.current_ebus, -2000);
}

TEST(SIIDemandParserSharedTest, ParseFMMUFromBuffer) {
    SIIData data;
    uint8_t buf[] = {1, 2, 3, 4};
    EXPECT_TRUE(parseFMMUFromBuffer(buf, sizeof(buf), data));
    EXPECT_EQ(data.fmmu_count, 4u);
    EXPECT_EQ(data.fmmus[0].fmmu_type, 1);
    EXPECT_EQ(data.fmmus[1].fmmu_type, 2);
    EXPECT_EQ(data.fmmus[2].fmmu_type, 3);
    EXPECT_EQ(data.fmmus[3].fmmu_type, 4);
}

TEST(SIIDemandParserSharedTest, ParseSyncManagerFromBuffer) {
    SIIData data;
    uint8_t buf[8] = {};
    buf[0] = 0x00; buf[1] = 0x10;  // phys_start = 0x1000
    buf[2] = 0x00; buf[3] = 0x01;  // length = 0x0100
    buf[4] = 0x02;                  // control
    buf[7] = 0x03;                  // sm_type

    EXPECT_TRUE(parseSyncManagerFromBuffer(buf, sizeof(buf), data));
    EXPECT_EQ(data.sm_count, 1);
    EXPECT_EQ(data.sync_managers[0].phys_start_address, 0x1000u);
    EXPECT_EQ(data.sync_managers[0].length, 0x0100u);
    EXPECT_EQ(data.sync_managers[0].sm_type, 0x03);
}

TEST(SIIDemandParserSharedTest, ParsePDOFromBuffer_TxPDO) {
    SIIData data;
    // 1 PDO with 1 entry
    uint8_t buf[16] = {};
    buf[0] = 0xA0; buf[1] = 0x1A;  // pdo_index = 0x1AA0
    buf[2] = 1;                     // n_entries
    buf[3] = 0;                     // sync_manager
    // Entry: index=0x6000, subindex=0, name=0, type=0, bits=8
    buf[8] = 0x00; buf[9] = 0x60;  // index
    buf[10] = 0;                    // subindex
    buf[13] = 8;                    // bit_length (entry offset 5)

    EXPECT_TRUE(parsePDOFromBuffer(buf, sizeof(buf), data, true));
    ASSERT_EQ(data.tx_pdos.size(), 1u);
    EXPECT_EQ(data.tx_pdos[0].pdo_index, 0x1AA0u);
    EXPECT_EQ(data.tx_pdos[0].n_entries, 1);
    ASSERT_EQ(data.tx_pdos[0].entries.size(), 1u);
    EXPECT_EQ(data.tx_pdos[0].entries[0].index, 0x6000u);
    EXPECT_EQ(data.tx_pdos[0].entries[0].bit_length, 8);
}

TEST(SIIDemandParserSharedTest, ParseDCFromBuffer) {
    SIIData data;
    uint8_t buf[24] = {};
    buf[0] = 0x40; buf[1] = 0x0D; buf[2] = 0x03; buf[3] = 0x00;  // cycle_time_0 = 200000

    EXPECT_TRUE(parseDCFromBuffer(buf, sizeof(buf), data));
    ASSERT_EQ(data.dc_configs.size(), 1u);
    EXPECT_EQ(data.dc_configs[0].cycle_time_0, 200000u);
}
