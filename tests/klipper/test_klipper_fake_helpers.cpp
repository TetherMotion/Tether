/**
 * @file test_klipper_fake_helpers.cpp
 * @brief Tests for FakeClock and FakeTransport test helpers.
 */

#include <gtest/gtest.h>
#include "test_klipper_fake_clock.hpp"

#include <thread>

using namespace tether::klipper::test;

TEST(FakeClockTest, InitialTimeIsEpoch) {
    FakeClock clock;
    EXPECT_EQ(clock.now().time_since_epoch().count(), 0);
}

TEST(FakeClockTest, AdvanceMilliseconds) {
    FakeClock clock;
    auto start = clock.now();
    clock.advanceMs(50);
    auto elapsed = clock.now() - start;
    EXPECT_EQ(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50);
}

TEST(FakeClockTest, AdvanceMicroseconds) {
    FakeClock clock;
    auto start = clock.now();
    clock.advanceUs(200);
    auto elapsed = clock.now() - start;
    EXPECT_EQ(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count(), 200);
}

TEST(FakeClockTest, MultipleAdvances) {
    FakeClock clock;
    auto start = clock.now();
    clock.advanceMs(10);
    clock.advanceMs(20);
    clock.advanceMs(30);
    auto elapsed = clock.now() - start;
    EXPECT_EQ(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 60);
}

TEST(FakeClockTest, HasElapsedSince) {
    FakeClock clock;
    auto start = clock.now();
    clock.advanceMs(100);
    EXPECT_TRUE(clock.hasElapsedSince(start, std::chrono::milliseconds(50)));
    EXPECT_TRUE(clock.hasElapsedSince(start, std::chrono::milliseconds(100)));
    EXPECT_FALSE(clock.hasElapsedSince(start, std::chrono::milliseconds(200)));
}

TEST(FakeClockTest, NoRealTimePasses) {
    FakeClock clock;
    auto before = std::chrono::steady_clock::now();
    clock.advanceMs(10000);
    auto after = std::chrono::steady_clock::now();
    // Advancing the fake clock should take negligible real time.
    auto realElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    EXPECT_LT(realElapsed.count(), 100);
}

// --- FakeTransport tests ---

TEST(FakeTransportTest, BasicRoundTrip) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();

    std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    EXPECT_EQ(ta->write(data), data.size());
    EXPECT_EQ(ta->available(), 0u);
    EXPECT_EQ(tb->available(), data.size());

    std::vector<uint8_t> buf(100);
    size_t n = tb->read(buf.data(), 100);
    EXPECT_EQ(n, data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        EXPECT_EQ(buf[i], data[i]);
    }
}

TEST(FakeTransportTest, DropWriteError) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();

    ta->injectDropWrite(3);
    std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    ta->write(data);
    // First 3 bytes dropped, only 2 should arrive.
    EXPECT_EQ(tb->available(), 2u);
    std::vector<uint8_t> buf(100);
    size_t n = tb->read(buf.data(), 100);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(buf[0], 4);
    EXPECT_EQ(buf[1], 5);
}

TEST(FakeTransportTest, FailReadError) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();

    std::vector<uint8_t> data = {1, 2, 3};
    ta->write(data);

    tb->injectFailRead(1);
    std::vector<uint8_t> buf(100);
    size_t n1 = tb->read(buf.data(), 100);
    EXPECT_EQ(n1, 0u);

    // Second read should succeed.
    size_t n2 = tb->read(buf.data(), 100);
    EXPECT_EQ(n2, 3u);
}

TEST(FakeTransportTest, CorruptError) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();

    ta->injectCorruption();
    std::vector<uint8_t> data = {0x00, 0x01, 0x02};
    ta->write(data);

    std::vector<uint8_t> buf(100);
    size_t n = tb->read(buf.data(), 100);
    EXPECT_EQ(n, 3u);
    // First byte should have bit 0 flipped: 0x00 -> 0x01.
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x01);
    EXPECT_EQ(buf[2], 0x02);
}

TEST(FakeTransportTest, ClearErrors) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();

    ta->injectDropWrite(100);
    EXPECT_TRUE(ta->hasErrors());
    ta->clearErrors();
    EXPECT_FALSE(ta->hasErrors());

    std::vector<uint8_t> data = {1, 2, 3};
    ta->write(data);
    EXPECT_EQ(tb->available(), 3u);
}

TEST(FakeTransportTest, IsOpen) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    EXPECT_FALSE(ta->isOpen());
    ta->open();
    EXPECT_TRUE(ta->isOpen());
    ta->close();
    EXPECT_FALSE(ta->isOpen());
}

TEST(FakeTransportTest, CloseAndReopen) {
    auto ta = std::make_shared<FakeTransport>();
    auto tb = std::make_shared<FakeTransport>();
    ta->wire(tb);
    ta->open();
    tb->open();
    ta->close();
    ta->open();
    EXPECT_TRUE(ta->isOpen());
}
