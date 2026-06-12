/**
 * @file test_hal_types.cpp
 * @brief Unit tests for HAL type definitions
 */

#include <gtest/gtest.h>
#include <magic_enum/magic_enum.hpp>
#include "tether/hal/HALTypes.hpp"

using namespace EtherCAT::HAL;

// ============================================================================
// MacAddress Tests
// ============================================================================

TEST(MacAddressTest, DefaultConstruction) {
    MacAddress mac;
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(mac.bytes[i], 0);
    }
}

TEST(MacAddressTest, SixByteConstruction) {
    MacAddress mac(0x11, 0x22, 0x33, 0x44, 0x55, 0x66);
    EXPECT_EQ(mac.bytes[0], 0x11);
    EXPECT_EQ(mac.bytes[1], 0x22);
    EXPECT_EQ(mac.bytes[2], 0x33);
    EXPECT_EQ(mac.bytes[3], 0x44);
    EXPECT_EQ(mac.bytes[4], 0x55);
    EXPECT_EQ(mac.bytes[5], 0x66);
}

TEST(MacAddressTest, PointerConstruction) {
    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    MacAddress mac(data);
    EXPECT_EQ(mac.bytes[0], 0xAA);
    EXPECT_EQ(mac.bytes[5], 0xFF);
}

TEST(MacAddressTest, Equality) {
    MacAddress mac1(0x11, 0x22, 0x33, 0x44, 0x55, 0x66);
    MacAddress mac2(0x11, 0x22, 0x33, 0x44, 0x55, 0x66);
    MacAddress mac3(0x11, 0x22, 0x33, 0x44, 0x55, 0x67);
    
    EXPECT_EQ(mac1, mac2);
    EXPECT_NE(mac1, mac3);
}

TEST(MacAddressTest, IsZero) {
    MacAddress zero;
    MacAddress nonZero(0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
    
    EXPECT_TRUE(zero.isZero());
    EXPECT_FALSE(nonZero.isZero());
}

TEST(MacAddressTest, IsBroadcast) {
    MacAddress broadcast(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    MacAddress notBroadcast(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE);
    
    EXPECT_TRUE(broadcast.isBroadcast());
    EXPECT_FALSE(notBroadcast.isBroadcast());
}

TEST(MacAddressTest, IsMulticast) {
    MacAddress multicast(0x01, 0x00, 0x5E, 0x00, 0x00, 0x01);  // IPv4 multicast
    MacAddress unicast(0x02, 0x00, 0x00, 0x00, 0x00, 0x01);    // Locally administered unicast
    
    EXPECT_TRUE(multicast.isMulticast());
    EXPECT_FALSE(unicast.isMulticast());
}

TEST(MacAddressTest, IsLocallyAdministered) {
    MacAddress local(0x02, 0x00, 0x00, 0x00, 0x00, 0x01);
    MacAddress universal(0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E);
    
    EXPECT_TRUE(local.isLocallyAdministered());
    EXPECT_FALSE(universal.isLocallyAdministered());
}

// ============================================================================
// Error Tests
// ============================================================================

TEST(ErrorTest, ErrorValues) {
    // Verify error enum values exist
    EXPECT_NE(Error::OK, Error::Timeout);
    EXPECT_NE(Error::OK, Error::InvalidArgument);
    EXPECT_NE(Error::OK, Error::NotInitialized);
}

TEST(ErrorTest, ErrorToString) {
    EXPECT_STREQ(magic_enum::enum_name(Error::OK).data(), "OK");
    EXPECT_STREQ(magic_enum::enum_name(Error::Timeout).data(), "Timeout");
    EXPECT_STREQ(magic_enum::enum_name(Error::InvalidArgument).data(), "InvalidArgument");
}

// ============================================================================
// Result Tests
// ============================================================================

TEST(ResultTest, SuccessValue) {
    Result<int> r(42);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.value, 42);
}

TEST(ResultTest, ErrorValue) {
    Result<int> r;
    r.error = Error::Timeout;
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error, Error::Timeout);
}

// ============================================================================
// Timestamp/Duration Tests
// ============================================================================

TEST(TimingTest, TimestampType) {
    Timestamp ts = 1234567890;
    EXPECT_EQ(ts, 1234567890u);
}

TEST(TimingTest, DurationTypes) {
    Milliseconds ms = 1000;
    Microseconds us = 1000000;
    
    EXPECT_EQ(ms, 1000);
    EXPECT_EQ(us, 1000000);
}

// ============================================================================
// EtherType Constants Tests
// ============================================================================

TEST(EtherTypeTest, Constants) {
    EXPECT_EQ(kEtherTypeEtherCAT, 0x88A4);
    EXPECT_EQ(kEtherType8021Q, 0x8100);
}
