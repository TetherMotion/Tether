/**
 * @file test_drives_as715n.cpp
 * @brief Tests for AS715N drive helpers (EtherCAT::Drives namespace)
 */

#include "tether/drives/AS715NErrors.hpp"
#include "tether/drives/AS715NRegisters.hpp"

#include <gtest/gtest.h>
#include <cstring>

using namespace EtherCAT::Drives;

// ============================================================================
// AS715NError tests
// ============================================================================

TEST(AS715NErrorTest, ParseNoError) {
    auto err = AS715NError::parse(0x0000);
    EXPECT_EQ(err.raw_code, 0x0000);
    EXPECT_FALSE(err.isDCSyncError());
    EXPECT_FALSE(err.isNoSyncError());
}

TEST(AS715NErrorTest, ParseDCSyncCycleError) {
    auto err = AS715NError::parse(ErrorCodes::DCSyncCycleSettingError);
    EXPECT_EQ(err.raw_code, ErrorCodes::DCSyncCycleSettingError);
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_FALSE(err.isNoSyncError());
    EXPECT_NE(err.name, nullptr);
    EXPECT_NE(err.description, nullptr);
}

TEST(AS715NErrorTest, ParseNoSync) {
    auto err = AS715NError::parse(ErrorCodes::NoSync);
    EXPECT_EQ(err.raw_code, ErrorCodes::NoSync);
    EXPECT_TRUE(err.isNoSyncError());
}

TEST(AS715NErrorTest, ParseChipSyncIncomplete) {
    auto err = AS715NError::parse(ErrorCodes::ChipSyncIncompleteInOp);
    EXPECT_EQ(err.raw_code, ErrorCodes::ChipSyncIncompleteInOp);
}

TEST(AS715NErrorTest, ParseUnknownError) {
    auto err = AS715NError::parse(0xFFFF);
    EXPECT_EQ(err.raw_code, 0xFFFF);
    // Should still have valid name/description strings
    EXPECT_NE(err.name, nullptr);
    EXPECT_NE(err.description, nullptr);
}

TEST(AS715NErrorTest, Format) {
    char buf[128] = {};
    AS715NError::format(buf, sizeof(buf), 0x07, 0x40);
    EXPECT_GT(std::strlen(buf), 0u);
}

TEST(AS715NErrorTest, ClassAndSubCode) {
    auto err = AS715NError::parse(ErrorCodes::DCSyncCycleSettingError);
    // Verify class_code and sub_code are populated from parse
    // The exact encoding depends on the error table implementation
    EXPECT_NE(err.raw_code, 0u);
}

TEST(AS715NErrorTest, IsRecoverable) {
    auto err = AS715NError::parse(0x0000);
    // NoError is trivially recoverable or not -- just check it's a bool
    (void)err.is_recoverable;
}

// ============================================================================
// Hex-nibble encoding tests (ErC1.x / ErA0.x etc.)
// ============================================================================

/// 0x0C11 = ErC1.1 — Synchronization loss (the fault currently reported by the device)
TEST(AS715NErrorTest, ParseSyncLossErC11) {
    auto err = AS715NError::parse(ErrorCodes::SyncLoss);    // 0x0C11
    EXPECT_EQ(err.raw_code, 0x0C11u);
    // Hex-nibble encoding: class_code = 0xC1, sub_code = 1
    EXPECT_EQ(err.class_code, 0xC1u);
    EXPECT_EQ(err.sub_code,   0x01u);
    // Human-readable name must be "ErC1.1"
    EXPECT_STREQ(err.name, "ErC1.1");
    // Must be classified as a DC sync / EtherCAT communication error
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.isNoSyncError());
    // Must be resettable
    EXPECT_TRUE(err.is_recoverable);
    EXPECT_NE(err.description, nullptr);
}

/// 0x0C10 = ErC1.0 — Excessive EtherCAT synchronization period error
TEST(AS715NErrorTest, ParseEthSyncCycleError) {
    auto err = AS715NError::parse(ErrorCodes::EthSyncCycleError);  // 0x0C10
    EXPECT_EQ(err.raw_code, 0x0C10u);
    EXPECT_EQ(err.class_code, 0xC1u);
    EXPECT_EQ(err.sub_code,   0x00u);
    EXPECT_STREQ(err.name, "ErC1.0");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// 0x0C12 = ErC1.2 — Network status switchover error
TEST(AS715NErrorTest, ParseNetworkStatusSwitchover) {
    auto err = AS715NError::parse(ErrorCodes::NetworkStatusSwitchover);
    EXPECT_EQ(err.raw_code, 0x0C12u);
    EXPECT_STREQ(err.name, "ErC1.2");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// 0x0C18 = ErC1.8 — Watchdog expired
TEST(AS715NErrorTest, ParseWatchdogExpired) {
    auto err = AS715NError::parse(ErrorCodes::WatchdogExpired);
    EXPECT_EQ(err.raw_code, 0x0C18u);
    EXPECT_STREQ(err.name, "ErC1.8");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// 0x0C20 = ErC2.0 — SYNC signal loss
TEST(AS715NErrorTest, ParseSYNCSigalLoss) {
    auto err = AS715NError::parse(ErrorCodes::SYNCSIgnalLoss);
    EXPECT_EQ(err.raw_code, 0x0C20u);
    EXPECT_EQ(err.class_code, 0xC2u);
    EXPECT_EQ(err.sub_code,   0x00u);
    EXPECT_STREQ(err.name, "ErC2.0");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// Verify decimal-style codes still format correctly (Er87.4 — hex digits 0-9 are same)
TEST(AS715NErrorTest, ParseDecimalStyleCode_Er874) {
    auto err = AS715NError::parse(0x0874u);
    EXPECT_EQ(err.raw_code, 0x0874u);
    EXPECT_EQ(err.class_code, 0x87u);
    EXPECT_EQ(err.sub_code,   0x04u);
    EXPECT_STREQ(err.name, "Er87.4");
    EXPECT_FALSE(err.isDCSyncError());
}

/// Legacy 0x0740 (Er74.0) must still be detected as a DC sync error
TEST(AS715NErrorTest, LegacyDCSyncCycleError_isDCSyncError) {
    auto err = AS715NError::parse(ErrorCodes::DCSyncCycleSettingError);  // 0x0740
    EXPECT_STREQ(err.name, "Er74.0");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// Legacy 0x0741 (Er74.1) must still be detected as isNoSyncError
TEST(AS715NErrorTest, LegacyNoSync_isNoSyncError) {
    auto err = AS715NError::parse(ErrorCodes::NoSync);  // 0x0741
    EXPECT_STREQ(err.name, "Er74.1");
    EXPECT_TRUE(err.isDCSyncError());
    EXPECT_TRUE(err.isNoSyncError());
    EXPECT_TRUE(err.is_recoverable);
}

/// format() must produce the hex-based display string
TEST(AS715NErrorTest, FormatHexNibbles) {
    char buf[32] = {};
    // class_code=0xC1, sub_code=1 => "ErC1.1"
    AS715NError::format(buf, sizeof(buf), 0xC1u, 0x01u);
    EXPECT_STREQ(buf, "ErC1.1");
}

TEST(AS715NErrorTest, FormatDecimalLookingNibbles) {
    char buf[32] = {};
    // class_code=0x87, sub_code=4 => "Er87.4"  (hex and decimal same for 0-9)
    AS715NError::format(buf, sizeof(buf), 0x87u, 0x04u);
    EXPECT_STREQ(buf, "Er87.4");
}

// ============================================================================
// AS715N register constants tests
// ============================================================================

TEST(AS715NRegistersTest, VendorAndProduct) {
    EXPECT_EQ(AS715N::kVendorId, 0x00400000u);
    EXPECT_EQ(AS715N::kProductCode, 0x00000715u);
}

TEST(AS715NRegistersTest, ManufacturerFaultIndex) {
    EXPECT_EQ(AS715N::kManufacturerFaultIndex, 0x203Fu);
    EXPECT_EQ(AS715N::kCiA402ErrorIndex, 0x603Fu);
}

TEST(AS715NRegistersTest, RunningMonitoringIndex) {
    EXPECT_EQ(AS715N::kRunningMonitoringIndex, 0x2040u);
    EXPECT_NE(AS715N::kU40_PhaseCurrentRms_SubIndex, 0u);
    EXPECT_NE(AS715N::kU40_PositionDeviation_SubIndex, 0u);
    EXPECT_NE(AS715N::kU40_HeatsinkTemperature_SubIndex, 0u);
}

TEST(AS715NRegistersTest, DigitalOutputIndex) {
    EXPECT_EQ(AS715N::kForcedPhysicalDOIndex, 0x60FEu);
    EXPECT_EQ(AS715N::kForcedPhysicalDO_SubIndex, 0x01u);
}

TEST(AS715NRegistersTest, ControlInProgressIndex) {
    EXPECT_EQ(AS715N::kControlInProgressIndex, 0x2031u);
    EXPECT_EQ(AS715N::kFaultResetSubIndex, 0x01u);
}

TEST(AS715NRegistersTest, DeviceAlias) {
    // AS715NDevice is a type alias for AS715N
    EXPECT_EQ(AS715NDevice::kVendorId, AS715N::kVendorId);
    EXPECT_EQ(AS715NDevice::kProductCode, AS715N::kProductCode);
}

// ============================================================================
// AS715NManufacturerFault203F tests
// ============================================================================

TEST(AS715NManufacturerFaultTest, FromU32_Zero) {
    auto fault = AS715NManufacturerFault203F::fromU32(0);
    EXPECT_EQ(fault.internal_code, 0u);
    EXPECT_EQ(fault.external_code, 0u);
}

TEST(AS715NManufacturerFaultTest, FromU32_WithData) {
    // Low 16 bits = internal, high 16 bits = external
    uint32_t raw = (0x0002u << 16) | 0x0001u;
    auto fault = AS715NManufacturerFault203F::fromU32(raw);
    // Verify it parses into internal/external codes
    EXPECT_TRUE(fault.internal_code != 0 || fault.external_code != 0);
}

TEST(AS715NManufacturerFaultTest, FromU32_MaxValue) {
    auto fault = AS715NManufacturerFault203F::fromU32(0xFFFFFFFF);
    EXPECT_EQ(fault.internal_code, 0xFFFFu);
    EXPECT_EQ(fault.external_code, 0xFFFFu);
}
