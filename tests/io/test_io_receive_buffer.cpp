/**
 * @file test_io_receive_buffer.cpp
 * @brief Tests for static and bounded dynamic receive buffers.
 */
#include <gtest/gtest.h>
#include "tether/io/ReceiveBuffer.hpp"

#include <array>
#include <limits>

using namespace tether::io;

TEST(StaticReceiveBufferTest, AppendsAndResets) {
    StaticReceiveBuffer<4> buffer;
    const std::array<uint8_t, 3> input{1, 2, 3};

    EXPECT_TRUE(buffer.append(input.data(), input.size()));
    EXPECT_EQ(buffer.size(), 3u);
    EXPECT_EQ(buffer.capacity(), 4u);
    EXPECT_EQ(buffer.maxCapacity(), 4u);
    EXPECT_EQ(buffer.data()[2], 3u);
    EXPECT_FALSE(buffer.append(input.data(), 2));
    EXPECT_EQ(buffer.size(), 3u);

    buffer.clear();
    EXPECT_EQ(buffer.size(), 0u);
    EXPECT_TRUE(buffer.append(input.data(), input.size()));
}

TEST(StaticReceiveBufferTest, ResizeCannotExceedCapacity) {
    StaticReceiveBuffer<8> buffer;
    EXPECT_TRUE(buffer.resize(8));
    EXPECT_FALSE(buffer.resize(9));
    EXPECT_EQ(buffer.size(), 8u);
}

TEST(DynamicReceiveBufferTest, GrowsGeometricallyWithinMaximum) {
    DynamicReceiveBuffer buffer(2, 10);
    const std::array<uint8_t, 7> input{1, 2, 3, 4, 5, 6, 7};

    EXPECT_TRUE(buffer.append(input.data(), input.size()));
    EXPECT_EQ(buffer.size(), 7u);
    EXPECT_GE(buffer.capacity(), 7u);
    EXPECT_LE(buffer.capacity(), 10u);
    EXPECT_EQ(buffer.maxCapacity(), 10u);
    EXPECT_TRUE(buffer.append(input.data(), 3));
    EXPECT_EQ(buffer.size(), 10u);
    EXPECT_FALSE(buffer.append(input.data(), 1));
}

TEST(DynamicReceiveBufferTest, InitialCapacityIsClampedToMaximum) {
    DynamicReceiveBuffer buffer(100, 8);
    EXPECT_EQ(buffer.capacity(), 8u);
    EXPECT_TRUE(buffer.resize(8));
    EXPECT_FALSE(buffer.resize(9));
}

TEST(DynamicReceiveBufferTest, ProtectsMaximumAndClearRetainsStorage) {
    DynamicReceiveBuffer buffer(1, 16);
    EXPECT_FALSE(buffer.resize(17));
    EXPECT_TRUE(buffer.resize(12));
    const size_t capacity = buffer.capacity();
    buffer.clear();
    EXPECT_EQ(buffer.size(), 0u);
    EXPECT_EQ(buffer.capacity(), capacity);
}

TEST(DynamicReceiveBufferTest, ZeroMaximumRejectsData) {
    DynamicReceiveBuffer buffer(4, 0);
    EXPECT_EQ(buffer.capacity(), 0u);
    EXPECT_FALSE(buffer.resize(1));
    EXPECT_TRUE(buffer.append(nullptr, 0));
}
