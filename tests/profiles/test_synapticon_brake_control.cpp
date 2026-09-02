/**
 * @file test_synapticon_brake_control.cpp
 * @brief Tests for Synapticon brake control helpers (0x2004)
 *
 * Verifies the 0x2004 register layout, brake status / release strategy enum
 * values, and the BrakeControl convenience API surface.  The SDO-dependent
 * methods (disengageBrake, engageBrake, etc.) require a live CoEManager and
 * are exercised on real hardware; here we validate the compile-time contract
 * and object dictionary metadata.
 */
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/Registers/DriveConfig2000.hpp"

#include <gtest/gtest.h>

using namespace EtherCAT::Drives;
using namespace EtherCAT::Drives::Synapticon;
using namespace EtherCAT::Drives::Registers::Synapticon::Obj2004;

// ============================================================================
// 0x2004 object index / subindex constants
// ============================================================================
TEST(SynapticonBrakeControlTest, ObjectIndexIs2004) {
    EXPECT_EQ(ObjectIndex, 0x2004u);
}

TEST(SynapticonBrakeControlTest, SubindexConstants) {
    EXPECT_EQ(kSubPullVoltage,            0x01u);
    EXPECT_EQ(kSubHoldVoltage,            0x02u);
    EXPECT_EQ(kSubPullTime,               0x03u);
    EXPECT_EQ(kSubReleaseStrategy,        0x04u);
    EXPECT_EQ(kSubControllerDisableDelay, 0x05u);
    EXPECT_EQ(kSubBrakeStatus,            0x07u);
    EXPECT_EQ(kSubOutputVoltage,          0x0Au);
    EXPECT_EQ(kSubSwitchingFrequency,     0x0Bu);
}

// ============================================================================
// Brake status enum (0x2004:7) matches Synapticon documentation
// ============================================================================
TEST(SynapticonBrakeControlTest, BrakeStatusOptionsValues) {
    EXPECT_EQ(static_cast<uint8_t>(BrakeStatusOptions::NotConfigured), 0u);
    EXPECT_EQ(static_cast<uint8_t>(BrakeStatusOptions::Engaged),       1u);
    EXPECT_EQ(static_cast<uint8_t>(BrakeStatusOptions::Disengaged),    2u);
}

// ============================================================================
// Release strategy enum (0x2004:4) matches Synapticon documentation
// ============================================================================
TEST(SynapticonBrakeControlTest, ReleaseStrategyOptionsValues) {
    EXPECT_EQ(static_cast<uint8_t>(ReleaseStrategyOptions::ManualOutputVoltage), 0u);
    EXPECT_EQ(static_cast<uint8_t>(ReleaseStrategyOptions::ClutchStyleBrake),    1u);
    EXPECT_EQ(static_cast<uint8_t>(ReleaseStrategyOptions::PinBrake),            2u);
}

// ============================================================================
// Register entry metadata for BrakeStatus
// ============================================================================
TEST(SynapticonBrakeControlTest, BrakeStatusRegisterMetadata) {
    EXPECT_EQ(BrakeStatus.index,    0x2004u);
    EXPECT_EQ(BrakeStatus.subindex, kSubBrakeStatus);
    EXPECT_EQ(BrakeStatus.data_type,
              EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8);
    // 0x2004:7 is readwrite per the Synapticon documentation — it must NOT
    // be marked ReadOnly (it controls the brake in automatic mode).
    EXPECT_NE(BrakeStatus.modification_mode,
              EtherCAT::ObjectDictionary::ModificationMode::ReadOnly);
    EXPECT_EQ(BrakeStatus.min_value, 0);
    EXPECT_EQ(BrakeStatus.max_value, 2);
}

TEST(SynapticonBrakeControlTest, ReleaseStrategyRegisterMetadata) {
    EXPECT_EQ(ReleaseStrategy.index,    0x2004u);
    EXPECT_EQ(ReleaseStrategy.subindex, kSubReleaseStrategy);
    EXPECT_TRUE(ReleaseStrategy.options_enum.has_value());
}

TEST(SynapticonBrakeControlTest, BrakeStatusRegisterHasOptionsEnum) {
    EXPECT_TRUE(BrakeStatus.options_enum.has_value());
}

// ============================================================================
// BrakeControl API surface / constants
// ============================================================================
TEST(SynapticonBrakeControlTest, BrakeControlTimeoutConstant) {
    EXPECT_GT(kBrakeSdoTimeoutMs, 0u);
    EXPECT_GT(kBrakeActuationDelayMs, 0u);
}

TEST(SynapticonBrakeControlTest, BrakeStatusValueAliasMatchesEnum) {
    // BrakeStatusValue is an alias for Obj2004::BrakeStatusOptions
    BrakeStatusValue v = BrakeStatusValue::Disengaged;
    EXPECT_EQ(static_cast<uint8_t>(v), 2u);
}

// ============================================================================
// 0x2004 register list contains the brake entries
// ============================================================================
TEST(SynapticonBrakeControlTest, RegisterListContainsBrakeEntries) {
    bool found_brake_status = false;
    bool found_release_strategy = false;
    for (const auto* entry : kRegisterList) {
        if (entry->subindex == kSubBrakeStatus) {
            found_brake_status = true;
            EXPECT_EQ(entry->index, 0x2004u);
        }
        if (entry->subindex == kSubReleaseStrategy) {
            found_release_strategy = true;
        }
    }
    EXPECT_TRUE(found_brake_status);
    EXPECT_TRUE(found_release_strategy);
}
