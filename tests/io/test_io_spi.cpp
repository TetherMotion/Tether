/**
 * @file test_io_spi.cpp
 * @brief Coverage for PosixSpiDriver — all paths reachable without a
 *        real /dev/spidev device (open failures, ioctl reject, state
 *        transitions, transfer guards, convenience overload).
 */

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "tether/io/SpiDriver.hpp"

using tether::io::ISpiDriver;
using tether::io::PosixSpiDriver;

#if !defined(ESP_PLATFORM)

TEST(SpiDriverCov, FreshDriverNotOpen) {
    PosixSpiDriver d;
    EXPECT_FALSE(d.isOpen());
}

TEST(SpiDriverCov, OpenNonexistentDeviceFails) {
    PosixSpiDriver d;
    EXPECT_FALSE(d.open("/dev/spidev-nonexistent-xyz", 0, 1'000'000));
    EXPECT_FALSE(d.isOpen());
    // Reopen attempts keep failing cleanly.
    EXPECT_FALSE(d.open("/dev/spidev-nonexistent-xyz", 0, 1'000'000));
}

TEST(SpiDriverCov, OpenNonSpiDeviceFailsConfig) {
    PosixSpiDriver d;
    // /dev/null opens fine but is not an SPI device — the mode ioctl
    // fails and the driver reports failure + releases the fd.
    EXPECT_FALSE(d.open("/dev/null", 0, 1'000'000));
    EXPECT_FALSE(d.isOpen());
}

TEST(SpiDriverCov, OpenNullPathFails) {
    PosixSpiDriver d;
    EXPECT_FALSE(d.open("/nonexistent-dir/spidev0.0", 0, 1'000'000));
}

TEST(SpiDriverCov, TransferUnopenedReturnsZero) {
    PosixSpiDriver d;
    const std::array<uint8_t, 4> tx{1, 2, 3, 4};
    std::array<uint8_t, 4> rx{};
    EXPECT_EQ(d.transfer(tx, rx.data()), 0u);
}

TEST(SpiDriverCov, TransferEmptyReturnsZero) {
    PosixSpiDriver d;
    const std::span<const uint8_t> empty{};
    EXPECT_EQ(d.transfer(empty, nullptr), 0u);
}

TEST(SpiDriverCov, ConvenienceOverloadUnopened) {
    PosixSpiDriver d;
    const std::array<uint8_t, 3> tx{0xAA, 0xBB, 0xCC};
    auto rx = d.transfer(std::span<const uint8_t>(tx));
    EXPECT_TRUE(rx.empty());   // 0-byte transfer → empty vector
}

TEST(SpiDriverCov, CloseIdempotent) {
    PosixSpiDriver d;
    d.close();   // not open — no-op
    EXPECT_FALSE(d.isOpen());
    d.close();
    EXPECT_FALSE(d.isOpen());
}

TEST(SpiDriverCov, FailedOpenLeavesClosedState) {
    PosixSpiDriver d;
    EXPECT_FALSE(d.open("/dev/null", 0, 500'000));
    EXPECT_FALSE(d.isOpen());
    // A later successful-looking open path would still start fresh.
    EXPECT_FALSE(d.open("/nonexistent", 0, 500'000));
    EXPECT_FALSE(d.isOpen());
    d.close();
}

#endif // !ESP_PLATFORM
