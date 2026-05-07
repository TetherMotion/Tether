#include "gtest/gtest.h"
#include <vector>
#include "profiles/cia402/60xx-Parameters.hpp"

using namespace CiA402::Parameters60xx;

TEST(CiA402_60xxParameters, ListSizeAndOrder) {
    // at least the previously enumerated registers
    EXPECT_GE(kRegisterList.size(), 48u);
    for (size_t i = 0; i < kRegisterList.size(); ++i) {
        auto *e = kRegisterList[i];
        if (e == nullptr) {
            ADD_FAILURE() << "nullptr entry at position " << i;
        }
        // continue regardless
    }

    // basic sanity: first element should be lowest index (DI4)
    EXPECT_EQ(kRegisterList.front()->index, 0x2004);

    // verify presence of a few expected indexes
    const std::vector<uint32_t> expected = {
        0x6040, // control word
        0x603F, // error code
        0x605A, // quickstop option
        0x6062, // position demand
        0x6071, // target torque
        0x607D, // software limits
        0x60B8, // touch probe function
        0x60E6, // encoder increments addl
        0x60F4, // position deviation
        0x60FC, // DI status
        0x6502, // supported drive modes
        0x607A, // target position
        0x607B, // position range limit
        0x607C, // home offset
        0x6080, // max motor speed
        0x60E3, // supported homing methods
        0x2006, // mechanical limit
        0x2004  // DI functions
    };
    for (uint32_t idx : expected) {
        bool found = false;
        for (auto *e : kRegisterList) {
            if (e->index == idx) { found = true; break; }
        }
        EXPECT_TRUE(found) << "missing index 0x" << std::hex << idx;
    }
}

TEST(CiA402_60xxParameters, OperationModeEntry) {
    const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry* entry = nullptr;
    for (auto *e : kRegisterList) {
        if (e->index == 0x6060) {
            entry = e;
            break;
        }
    }
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->data_type, EtherCAT::slave::ObjectDictionaryDataType::Integer8);
}

TEST(CiA402_60xxParameters, ValueRanges) {
    EXPECT_EQ(ReferencePolarity.min_value, 0);
    EXPECT_EQ(ReferencePolarity.max_value, 255);
    EXPECT_EQ(SpeedDeviation.min_value, INT32_MIN);
    EXPECT_EQ(SpeedDeviation.max_value, INT32_MAX);
    EXPECT_EQ(TorqueOffset.min_value, -4000);
    EXPECT_EQ(TorqueOffset.max_value, 4000);
    EXPECT_EQ(TargetVelocity.min_value, INT32_MIN);
    EXPECT_EQ(TargetVelocity.max_value, INT32_MAX);
    EXPECT_EQ(TargetTorque.min_value, -4000);
    EXPECT_EQ(TargetTorque.max_value, 4000);
    EXPECT_EQ(MaxTorque.min_value, 0);
    EXPECT_EQ(MaxTorque.max_value, 4000);
    EXPECT_EQ(MaxSpeed.min_value, 0);
    EXPECT_EQ(MaxSpeed.max_value, UINT32_MAX);
    EXPECT_EQ(PositiveTorqueLimit.min_value, 0);
    EXPECT_EQ(PositiveTorqueLimit.max_value, 4000);
    EXPECT_EQ(NegativeTorqueLimit.min_value, 0);
    EXPECT_EQ(NegativeTorqueLimit.max_value, 4000);
    EXPECT_EQ(MinSoftwarePositionLimit.min_value, INT32_MIN);
    EXPECT_EQ(MinSoftwarePositionLimit.max_value, INT32_MAX);
    EXPECT_EQ(MaxSoftwarePositionLimit.min_value, INT32_MIN);
    EXPECT_EQ(MaxSoftwarePositionLimit.max_value, INT32_MAX);
    EXPECT_EQ(MechanicalLimitPosition.min_value, 0);
    EXPECT_EQ(MechanicalLimitPosition.max_value, 2);
    // touch probes are RO no range checks, but ensure types
    EXPECT_EQ(TouchProbe1PosEdge.data_type, EtherCAT::slave::ObjectDictionaryDataType::Integer32);
    EXPECT_EQ(TouchProbe1PosEdge.modification_mode, ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly);
    EXPECT_EQ(TouchProbeFunction.data_type, EtherCAT::slave::ObjectDictionaryDataType::Unsigned16);
}
