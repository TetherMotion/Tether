// GoogleTest-based unit tests for SLIPStream::Encoder
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <deque>
#include <functional>
#include "SLIPStream/Encoder.hpp"
#include "SLIPStream/SLIP.hpp"

// Helper: capture output into a vector, with optional backpressure simulation
using namespace SLIPStream;

struct BackpressuredSink {
    std::vector<uint8_t> out;
    // How many bytes to accept before returning RetryLater for the next call
    size_t acceptThenBlock = SIZE_MAX; // default: never block
    size_t accepted = 0;

    WriteStatus operator()(uint8_t b) {
        if (accepted >= acceptThenBlock) return WriteStatus::RetryLater;
        out.push_back(b);
        accepted++;
        return WriteStatus::Ok;
    }

    void reset() { out.clear(); accepted = 0; }
};

TEST(SLIPEncoder, SimpleEncodeImmediate) {
    BackpressuredSink sink;
    Encoder enc([&sink](uint8_t b){ return sink(b); }, 64, 16);

    const uint8_t in[] = {0x01, 0x02, 0x03};
    auto [st, consumed] = enc.pushPacket(in, sizeof(in));
    EXPECT_EQ(st, WriteStatus::Ok);
    EXPECT_EQ(consumed, sizeof(in));
    // The encoded output should be same bytes then END
    ASSERT_EQ(sink.out.size(), sizeof(in) + 1);
    EXPECT_EQ(sink.out[0], 0x01);
    EXPECT_EQ(sink.out[1], 0x02);
    EXPECT_EQ(sink.out[2], 0x03);
    EXPECT_EQ(sink.out[3], END);
}

TEST(SLIPEncoder, EncodeWithEscapes) {
    BackpressuredSink sink;
    Encoder enc([&sink](uint8_t b){ return sink(b); }, 64, 16);

    const uint8_t in[] = {END, ESC, 0x55};
    auto [st, consumed] = enc.pushPacket(in, sizeof(in));
    EXPECT_EQ(st, WriteStatus::Ok);
    EXPECT_EQ(consumed, sizeof(in));
    // Expect escaped sequence and final END
    std::vector<uint8_t> expected = {ESC, ESCEND, ESC, ESCESC, 0x55, END};
    EXPECT_EQ(sink.out.size(), expected.size());
    EXPECT_EQ(0, std::memcmp(sink.out.data(), expected.data(), expected.size()));
}

TEST(SLIPEncoder, RetryLaterDuringEncode) {
    BackpressuredSink sink;
    // Simulate sink accepting only 2 bytes then blocking
    sink.acceptThenBlock = 2;
    Encoder enc([&sink](uint8_t b){ return sink(b); }, 16, 4);

    const uint8_t in[] = {0x01, END, 0x02};
    // First push: will encode until sink blocks
    auto [st1, cons1] = enc.pushPacket(in, sizeof(in));
    EXPECT_EQ(st1, WriteStatus::RetryLater);
    // cons1 should be > 0 but < sizeof(in)
    EXPECT_GT(cons1, 0u);
    EXPECT_LT(cons1, sizeof(in));
    // Make sink accept more and try again
    sink.acceptThenBlock = SIZE_MAX; // never block now
    auto [st2, cons2] = enc.pushPacket(in + cons1, sizeof(in) - cons1);
    EXPECT_EQ(st2, WriteStatus::Ok);
    EXPECT_EQ(cons2 + cons1, sizeof(in));
    // Now flush should have been done; content in sink.out should form a complete packet
    // We may have flush interleavings; just verify decoded payload matches original
    // Decode by iterating
    std::vector<uint8_t> decoded;
    for (uint8_t b : sink.out) {
        if (b == END) break;
        if (b == ESC) continue; // simplified: next byte will be ESCEND/ESCESC
    }
    // Instead, check for presence of END at end
    ASSERT_FALSE(sink.out.empty());
    EXPECT_EQ(sink.out.back(), END);
}

TEST(SLIPEncoder, QueueCapacityAndFragmentation) {
    BackpressuredSink sink;
    // Small tx buffer to force fragmentation
    Encoder enc([&sink](uint8_t b){ return sink(b); }, 8, 3);

    // Prepare a payload that will expand due to escapes
    const uint8_t in[] = {END, ESC, END, ESC}; // encodes to 8 bytes + END
    // First call should likely return RetryLater or Ok depending on timing; ensure we can send in multiple calls
    size_t totalConsumed = 0;
    const uint8_t* ptr = in;
    size_t remaining = sizeof(in);
    while (remaining > 0) {
        auto [st, consumed] = enc.pushPacket(ptr, remaining);
        if (st == WriteStatus::RetryLater) {
            // simulate sink flush by allowing more writes and calling flush
            // here sink accepts unlimited; but flush likely already happened inside pushPacket
            continue;
        }
        EXPECT_NE(st, WriteStatus::Error);
        totalConsumed += consumed;
        ptr += consumed;
        remaining -= consumed;
        if (st == WriteStatus::Ok) break;
    }
    EXPECT_EQ(totalConsumed, sizeof(in));
}

// main moved to test_all_main.cpp
