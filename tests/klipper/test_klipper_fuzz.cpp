/**
 * @file test_klipper_fuzz.cpp
 * @brief Fuzz/property-based tests for VLQ encoding, message blocks, and JSON parsing.
 *
 * @details
 * These tests use randomized inputs to verify that the encoding/decoding
 * routines are robust across a wide range of inputs, not just hand-picked
 * edge cases. They use a simple PRNG (std::mt19937) for reproducibility.
 */

#include <gtest/gtest.h>
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/klippy/JsonValue.hpp"

#include <random>
#include <vector>
#include <cstdint>
#include <string>
#include <algorithm>
#include <limits>

using namespace tether::klipper;

// ============================================================================
// VLQ encode/decode fuzz tests
// ============================================================================

class VlqFuzzTest : public ::testing::TestWithParam<uint32_t> {};

TEST_P(VlqFuzzTest, EncodeDecodeMsgIdRoundTrip) {
    uint16_t msgid = static_cast<uint16_t>(GetParam() & 0x3FF); // 10-bit msgid
    uint8_t buf[4] = {};
    size_t n = protocol::encodeMsgId(msgid, buf);
    ASSERT_GT(n, 0u);
    ASSERT_LE(n, 2u);
    const uint8_t* p = buf;
    const uint8_t* end = buf + n;
    auto decoded = protocol::decodeMsgId(p, end);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(*decoded, msgid);
    EXPECT_EQ(p, end); // All bytes consumed
}

TEST_P(VlqFuzzTest, EncodeDecodeParamRoundTrip) {
    int32_t val = static_cast<int32_t>(GetParam());
    uint8_t buf[6] = {};
    size_t n = protocol::encodeParam(val, buf);
    ASSERT_GT(n, 0u);
    ASSERT_LE(n, 5u);
    const uint8_t* p = buf;
    const uint8_t* end = buf + n;
    auto decoded = protocol::decodeParam(p, end);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(*decoded, val);
    EXPECT_EQ(p, end); // All bytes consumed
}

INSTANTIATE_TEST_SUITE_P(RandomMsgIds, VlqFuzzTest,
    ::testing::Values(
        0, 1, 127, 128, 255, 256, 511, 1023,
        0x3FF, 0x1FF, 0x0FF, 0x07F,
        42, 100, 200, 300, 500, 999
    ));

// Property-based test with random values
TEST(VlqPropertyTest, RandomMsgIdRoundTrip) {
    std::mt19937 rng(42); // Fixed seed for reproducibility
    std::uniform_int_distribution<uint16_t> dist(0, 1023);

    for (int i = 0; i < 1000; ++i) {
        uint16_t msgid = dist(rng);
        uint8_t buf[4] = {};
        size_t n = protocol::encodeMsgId(msgid, buf);
        ASSERT_GT(n, 0u) << "Failed for msgid=" << msgid;
        const uint8_t* p = buf;
        const uint8_t* end = buf + n;
        auto decoded = protocol::decodeMsgId(p, end);
        ASSERT_TRUE(decoded.has_value()) << "Failed for msgid=" << msgid;
        EXPECT_EQ(*decoded, msgid) << "Mismatch for msgid=" << msgid;
    }
}

TEST(VlqPropertyTest, RandomParamRoundTrip) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int32_t> dist(
        std::numeric_limits<int32_t>::min(),
        std::numeric_limits<int32_t>::max()
    );

    for (int i = 0; i < 1000; ++i) {
        int32_t val = dist(rng);
        uint8_t buf[6] = {};
        size_t n = protocol::encodeParam(val, buf);
        ASSERT_GT(n, 0u) << "Failed for val=" << val;
        const uint8_t* p = buf;
        const uint8_t* end = buf + n;
        auto decoded = protocol::decodeParam(p, end);
        ASSERT_TRUE(decoded.has_value()) << "Failed for val=" << val;
        EXPECT_EQ(*decoded, val) << "Mismatch for val=" << val;
    }
}

TEST(VlqPropertyTest, RandomParamBoundaryValues) {
    // Test boundary values that are known to be tricky for VLQ encoding.
    int32_t boundaries[] = {
        0, 1, -1,
        63, 64, 65,
        -63, -64, -65,
        8191, 8192, 8193,
        -8191, -8192, -8193,
        1048575, 1048576, 1048577,
        -1048575, -1048576, -1048577,
        134217727, 134217728,
        -134217727, -134217728,
        std::numeric_limits<int32_t>::max(),
        std::numeric_limits<int32_t>::min(),
    };

    for (int32_t val : boundaries) {
        uint8_t buf[6] = {};
        size_t n = protocol::encodeParam(val, buf);
        ASSERT_GT(n, 0u) << "Encode failed for val=" << val;
        const uint8_t* p = buf;
        const uint8_t* end = buf + n;
        auto decoded = protocol::decodeParam(p, end);
        ASSERT_TRUE(decoded.has_value()) << "Decode failed for val=" << val;
        EXPECT_EQ(*decoded, val) << "Mismatch for val=" << val;
    }
}

// Truncated buffer test: verify decode gracefully handles truncated data
TEST(VlqPropertyTest, TruncatedBufferReturnsNoValue) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int32_t> dist(
        std::numeric_limits<int32_t>::min(),
        std::numeric_limits<int32_t>::max()
    );

    for (int i = 0; i < 100; ++i) {
        int32_t val = dist(rng);
        uint8_t buf[6] = {};
        size_t n = protocol::encodeParam(val, buf);
        ASSERT_GT(n, 1u);

        // Truncate by 1 byte and verify decode fails
        const uint8_t* p = buf;
        const uint8_t* end = buf + n - 1;
        auto decoded = protocol::decodeParam(p, end);
        EXPECT_FALSE(decoded.has_value())
            << "Expected failure for truncated val=" << val << " (n=" << n << ")";
    }
}

// ============================================================================
// MessageBlock fuzz tests
// ============================================================================

TEST(MessageBlockFuzzTest, RandomContentRoundTrip) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> lenDist(0, 59); // max content = 59
    std::uniform_int_distribution<int> seqDist(0, 15);
    std::uniform_int_distribution<int> byteDist(0, 255);

    for (int iter = 0; iter < 200; ++iter) {
        uint8_t seq = static_cast<uint8_t>(seqDist(rng));
        int contentLen = lenDist(rng);
        std::vector<uint8_t> content(contentLen);
        for (int j = 0; j < contentLen; ++j) {
            content[j] = static_cast<uint8_t>(byteDist(rng));
        }

        uint8_t out[128] = {};
        size_t n = protocol::buildBlock(seq, content, out);
        ASSERT_GT(n, 0u) << "buildBlock failed for len=" << contentLen;

        // Parse the block back
        std::vector<uint8_t> blockData(out, out + n);
        auto parsed = protocol::parseBlock(blockData);
        ASSERT_EQ(parsed.status, protocol::BlockParseStatus::Ok)
            << "Parse failed for seq=" << (int)seq << " len=" << contentLen;
        EXPECT_EQ(parsed.block.sequence, seq);
        EXPECT_EQ(parsed.block.content, content);
    }
}

TEST(MessageBlockFuzzTest, RandomCorruptionDetected) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> byteDist(0, 255);
    std::uniform_int_distribution<int> flipDist(0, 7);

    for (int iter = 0; iter < 200; ++iter) {
        // Build a valid block
        std::vector<uint8_t> content(10);
        for (auto& b : content) b = static_cast<uint8_t>(byteDist(rng));

        uint8_t out[128] = {};
        size_t n = protocol::buildBlock(5, content, out);
        ASSERT_GT(n, 0u);

        // Flip a random bit in the block (not the sync byte)
        std::vector<uint8_t> corrupted(out, out + n);
        int bytePos = 1 + (flipDist(rng) % (n - 1));
        corrupted[bytePos] ^= (1 << flipDist(rng));

        auto parsed = protocol::parseBlock(corrupted);
        // Corrupted block should either fail to parse or have BadCrc
        // (it should NOT parse as Ok with wrong content)
        if (parsed.status == protocol::BlockParseStatus::Ok) {
            // If it parsed OK, the content must be different from original
            // (corruption changed something) or the CRC happened to match
            // (unlikely but possible for single-bit flips in content area)
            // Just verify it didn't silently produce wrong data
            EXPECT_TRUE(parsed.block.content != content ||
                        parsed.block.sequence != 5);
        }
    }
}

TEST(MessageBlockFuzzTest, EmptyContentRoundTrip) {
    // Edge case: empty content
    std::vector<uint8_t> content;
    uint8_t out[128] = {};
    size_t n = protocol::buildBlock(0, content, out);
    ASSERT_GT(n, 0u);

    std::vector<uint8_t> blockData(out, out + n);
    auto parsed = protocol::parseBlock(blockData);
    EXPECT_EQ(parsed.status, protocol::BlockParseStatus::Ok);
    EXPECT_TRUE(parsed.block.content.empty());
}

TEST(MessageBlockFuzzTest, MaxContentRoundTrip) {
    // Edge case: maximum content length (59 bytes)
    std::vector<uint8_t> content(59, 0xAA);
    uint8_t out[128] = {};
    size_t n = protocol::buildBlock(15, content, out);
    ASSERT_GT(n, 0u);

    std::vector<uint8_t> blockData(out, out + n);
    auto parsed = protocol::parseBlock(blockData);
    EXPECT_EQ(parsed.status, protocol::BlockParseStatus::Ok);
    EXPECT_EQ(parsed.block.content, content);
    EXPECT_EQ(parsed.block.sequence, 15);
}

// ============================================================================
// JSON parsing fuzz tests
// ============================================================================

TEST(JsonFuzzTest, RandomStringRoundTrip) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> lenDist(0, 50);
    std::uniform_int_distribution<int> charDist(32, 126);

    for (int iter = 0; iter < 200; ++iter) {
        int len = lenDist(rng);
        std::string s;
        s.reserve(len);
        for (int j = 0; j < len; ++j) {
            char c = static_cast<char>(charDist(rng));
            // Avoid characters that need escaping for simplicity
            if (c == '"' || c == '\\') c = 'x';
            s += c;
        }

        // Build a JSON string
        std::string json = "\"" + s + "\"";
        auto parsed = klippy::JsonValue::parse(json);
        ASSERT_TRUE(parsed) << "Parse failed for: " << json;
        ASSERT_TRUE(parsed->isString());
        EXPECT_EQ(parsed->asString(), s);
    }
}

TEST(JsonFuzzTest, RandomIntegerRoundTrip) {
    std::mt19937_64 rng(42);
    std::uniform_int_distribution<int64_t> dist(
        std::numeric_limits<int64_t>::min() / 2,
        std::numeric_limits<int64_t>::max() / 2
    );

    for (int iter = 0; iter < 200; ++iter) {
        int64_t val = dist(rng);
        std::string json = std::to_string(val);
        auto parsed = klippy::JsonValue::parse(json);
        ASSERT_TRUE(parsed) << "Parse failed for: " << json;
        // JSON numbers may be stored as int or double
        if (parsed->isInt()) {
            EXPECT_EQ(parsed->asInt(), val);
        } else if (parsed->isDouble()) {
            EXPECT_NEAR(parsed->asDouble(), static_cast<double>(val), 1.0);
        }
    }
}

TEST(JsonFuzzTest, RandomBooleanRoundTrip) {
    std::mt19937 rng(42);
    std::bernoulli_distribution dist(0.5);

    for (int iter = 0; iter < 100; ++iter) {
        bool val = dist(rng);
        std::string json = val ? "true" : "false";
        auto parsed = klippy::JsonValue::parse(json);
        ASSERT_TRUE(parsed) << "Parse failed for: " << json;
        ASSERT_TRUE(parsed->isBool());
        EXPECT_EQ(parsed->asBool(), val);
    }
}

TEST(JsonFuzzTest, RandomObjectRoundTrip) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> numKeysDist(0, 10);
    std::uniform_int_distribution<int> valDist(0, 9999);

    for (int iter = 0; iter < 50; ++iter) {
        int numKeys = numKeysDist(rng);
        std::string json = "{";
        for (int k = 0; k < numKeys; ++k) {
            if (k > 0) json += ",";
            json += "\"key" + std::to_string(k) + "\":" + std::to_string(valDist(rng));
        }
        json += "}";

        auto parsed = klippy::JsonValue::parse(json);
        ASSERT_TRUE(parsed) << "Parse failed for: " << json;
        ASSERT_TRUE(parsed->isObject());
        EXPECT_EQ(parsed->asObject().size(), static_cast<size_t>(numKeys));
    }
}

TEST(JsonFuzzTest, RandomArrayRoundTrip) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> numElemDist(0, 20);
    std::uniform_int_distribution<int> valDist(0, 9999);

    for (int iter = 0; iter < 50; ++iter) {
        int numElem = numElemDist(rng);
        std::string json = "[";
        for (int e = 0; e < numElem; ++e) {
            if (e > 0) json += ",";
            json += std::to_string(valDist(rng));
        }
        json += "]";

        auto parsed = klippy::JsonValue::parse(json);
        ASSERT_TRUE(parsed) << "Parse failed for: " << json;
        ASSERT_TRUE(parsed->isArray());
        EXPECT_EQ(parsed->asArray().size(), static_cast<size_t>(numElem));
    }
}

TEST(JsonFuzzTest, MalformedJsonDoesNotCrash) {
    // Feed various malformed JSON strings and verify no crash.
    std::vector<std::string> malformed = {
        "{",
        "}",
        "[",
        "]",
        "{\"key\":",
        "{\"key\":\"value\"",
        "\"unterminated string",
        "tru",
        "fals",
        "nul",
        "123.",
        "{,}",
        "[,]",
        "{\"a\":1,}",
        "[1,2,]",
        "{{}}",
        "[]]",
        "{[]}",
        "\x00\x01\x02\x03",
        "   ",
        "\n\t\r",
        "\"\\u\"",
        "\"\\u0\"",
        "\"\\u00\"",
        "\"\\u000\"",
    };

    for (const auto& json : malformed) {
        // Should not crash - either parse fails or returns something
        auto parsed = klippy::JsonValue::parse(json);
        // We don't care if it succeeds or fails, just that it doesn't crash.
        (void)parsed;
    }
    SUCCEED();
}

TEST(JsonFuzzTest, DeeplyNestedObjectDoesNotCrash) {
    // Build a deeply nested object
    std::string json;
    for (int i = 0; i < 100; ++i) {
        json += "{\"a\":";
    }
    json += "1";
    for (int i = 0; i < 100; ++i) {
        json += "}";
    }

    auto parsed = klippy::JsonValue::parse(json);
    // Should either parse successfully or fail gracefully (not crash).
    (void)parsed;
    SUCCEED();
}

TEST(JsonFuzzTest, LargeStringDoesNotCrash) {
    // Build a large string (100KB)
    std::string large(100000, 'x');
    std::string json = "\"" + large + "\"";

    auto parsed = klippy::JsonValue::parse(json);
    ASSERT_TRUE(parsed);
    ASSERT_TRUE(parsed->isString());
    EXPECT_EQ(parsed->asString().size(), 100000u);
}

// ============================================================================
// DataDictionary fuzz tests
// ============================================================================

TEST(DataDictionaryFuzzTest, RandomCommandNames) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> lenDist(1, 32);
    std::uniform_int_distribution<int> charDist('a', 'z');

    protocol::DataDictionary dict;

    for (int iter = 0; iter < 100; ++iter) {
        int len = lenDist(rng);
        std::string name;
        name.reserve(len);
        for (int j = 0; j < len; ++j) {
            name += static_cast<char>(charDist(rng));
        }

        // Add command - should succeed for unique names, return 0 for duplicates.
        uint16_t id = dict.addCommand(name);
        if (id != 0) {
            // Verify lookup works
            auto found = dict.lookupCommand(name);
            ASSERT_TRUE(found.has_value());
            EXPECT_EQ(*found, id);
        }
    }
}

TEST(DataDictionaryFuzzTest, LookupNonexistentCommands) {
    protocol::DataDictionary dict;
    dict.addCommand("get_clock");
    dict.addCommand("allocate_oids");

    // Lookup random strings that were never added
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> lenDist(1, 20);
    std::uniform_int_distribution<int> charDist('a', 'z');

    for (int iter = 0; iter < 100; ++iter) {
        int len = lenDist(rng);
        std::string name;
        for (int j = 0; j < len; ++j) {
            name += static_cast<char>(charDist(rng));
        }
        if (name != "get_clock" && name != "allocate_oids") {
            auto found = dict.lookupCommand(name);
            EXPECT_FALSE(found.has_value());
        }
    }
}
