/**
 * @file test_tether_utils.cpp
 * @brief Unit tests for generic tether/utils and TUI helpers:
 *        SeqLockBuffer, Crc (CRC-16/CRC-32), TextWrap
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "tether/utils/Crc.hpp"
#include "tether/utils/SeqLockBuffer.hpp"
#include "tether/terminal_ui/TextWrap.hpp"

// ============================================================================
// SeqLockBuffer
// ============================================================================

namespace {

struct TestPayload {
    uint32_t a = 0;
    uint32_t b = 0;
};

} // namespace

TEST(SeqLockBuffer, WriteThenReadReturnsLatest) {
    EtherCAT::Utils::SeqLockBuffer<TestPayload> buf;
    TestPayload in{1, 2}, out{};
    buf.write(in);
    ASSERT_TRUE(buf.read(out));
    EXPECT_EQ(out.a, 1u);
    EXPECT_EQ(out.b, 2u);

    buf.write({3, 4});
    ASSERT_TRUE(buf.read(out));
    EXPECT_EQ(out.a, 3u);
    EXPECT_EQ(out.b, 4u);
}

TEST(SeqLockBuffer, SequenceIncrementsPerWrite) {
    EtherCAT::Utils::SeqLockBuffer<TestPayload> buf;
    const uint64_t s0 = buf.sequence();
    buf.write({});
    const uint64_t s1 = buf.sequence();
    buf.write({});
    const uint64_t s2 = buf.sequence();
    EXPECT_EQ(s1 - s0, 2u);   // odd (in-progress) -> even (published)
    EXPECT_EQ(s2 - s1, 2u);
    EXPECT_EQ(s0 % 2u, 0u);   // idle sequence is even
    EXPECT_EQ(s2 % 2u, 0u);
}

TEST(SeqLockBuffer, InitialReadReturnsDefaultOrRetries) {
    EtherCAT::Utils::SeqLockBuffer<TestPayload> buf;
    TestPayload out{9, 9};
    // No write yet: read() succeeds (seq even/unchanged) copying a
    // zero-initialised payload; either way out must be a coherent value.
    if (buf.read(out)) {
        EXPECT_EQ(out.a, 0u);
        EXPECT_EQ(out.b, 0u);
    }
}

TEST(SeqLockBuffer, ConcurrentWriterReader) {
    EtherCAT::Utils::SeqLockBuffer<TestPayload> buf;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> reads{0};
    std::atomic<uint64_t> torn{0};

    std::thread writer([&] {
        for (uint32_t i = 1; !stop.load(std::memory_order_relaxed); ++i) {
            buf.write({i, i});   // coherent pairs only
        }
    });
    std::thread reader([&] {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
            const TestPayload out = buf.read_retry(64);
            ++reads;
            if (out.a != out.b) ++torn;   // torn snapshot detected
        }
    });

    reader.join();
    stop.store(true);
    writer.join();

    EXPECT_GT(reads.load(), 0u);
    EXPECT_EQ(torn.load(), 0u);
}

// ============================================================================
// Crc
// ============================================================================

TEST(Crc, Crc32KnownVector) {
    // zlib.crc32(b"123456789") == 0xCBF43926
    const std::string s = "123456789";
    EXPECT_EQ(EtherCAT::Utils::crc32(
                  reinterpret_cast<const uint8_t*>(s.data()), s.size()),
              0xCBF43926u);
}

TEST(Crc, Crc32Empty) {
    EXPECT_EQ(EtherCAT::Utils::crc32(nullptr, 0), 0u);
    EXPECT_EQ(EtherCAT::Utils::crc32(std::vector<uint8_t>{}), 0u);
}

TEST(Crc, Crc16KnownVector) {
    // CRC-16/ARC (reflected 0x8005, init 0x0000, no final XOR) of
    // "123456789" == 0xBB3D
    const std::string s = "123456789";
    EXPECT_EQ(EtherCAT::Utils::crc16(
                  reinterpret_cast<const uint8_t*>(s.data()), s.size()),
              0xBB3Du);
}

TEST(Crc, Crc16ModbusInit) {
    // CRC-16/MODBUS (same poly, init 0xFFFF) of "123456789" == 0x4B37
    const std::string s = "123456789";
    EXPECT_EQ(EtherCAT::Utils::crc16(
                  reinterpret_cast<const uint8_t*>(s.data()), s.size(),
                  0xFFFFu),
              0x4B37u);
}

TEST(Crc, Crc16IncrementalEqualsOneShot) {
    const std::string s = "123456789";
    const auto* p = reinterpret_cast<const uint8_t*>(s.data());
    const uint16_t one_shot = EtherCAT::Utils::crc16(p, s.size());
    uint16_t inc = EtherCAT::Utils::crc16Update(0x0000, p, 4);
    inc = EtherCAT::Utils::crc16Update(inc, p + 4, s.size() - 4);
    EXPECT_EQ(inc, one_shot);
}

TEST(Crc, Crc32PaddedZeroFillsToBufSize) {
    const std::string s = "ABC";
    const auto* p = reinterpret_cast<const uint8_t*>(s.data());
    // Padding to bufSize must equal CRC of data + (bufSize-len) zeros.
    const uint32_t padded = EtherCAT::Utils::crc32Padded(p, s.size(), 8);
    const std::vector<uint8_t> manual = {'A', 'B', 'C', 0, 0, 0, 0, 0};
    EXPECT_EQ(padded, EtherCAT::Utils::crc32(manual));
}

TEST(Crc, Crc32PaddedTruncatesWhenLonger) {
    const std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    const std::vector<uint8_t> first3 = {1, 2, 3};
    EXPECT_EQ(EtherCAT::Utils::crc32Padded(data, 3),
              EtherCAT::Utils::crc32(first3));
}

// ============================================================================
// TextWrap
// ============================================================================

TEST(TextWrap, ShortLineUnchanged) {
    const auto lines = Tether::TUI::wrapText("hello", 10);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "hello");
}

TEST(TextWrap, WrapsAtSpace) {
    const auto lines = Tether::TUI::wrapText("aaa bbb ccc", 7);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "aaa bbb");
    EXPECT_EQ(lines[1], "ccc");
}

TEST(TextWrap, HardBreaksLongWord) {
    const auto lines = Tether::TUI::wrapText("abcdefghij", 4);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0], "abcd");
    EXPECT_EQ(lines[1], "efgh");
    EXPECT_EQ(lines[2], "ij");
}

TEST(TextWrap, PreservesNewlinesAndEmptyLines) {
    const auto lines = Tether::TUI::wrapText("one\n\nthree", 10);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0], "one");
    EXPECT_EQ(lines[1], "");
    EXPECT_EQ(lines[2], "three");
}

TEST(TextWrap, ZeroWidthReturnsEmpty) {
    EXPECT_TRUE(Tether::TUI::wrapText("abc", 0).empty());
    EXPECT_TRUE(Tether::TUI::wrapText("abc", -1).empty());
}
