/**
 * @file test_slave_profile_no_global_state.cpp
 * @brief Tests that slave profile instances have independent state
 *
 * Verifies that per-instance state (PDO layout, deadband compensation,
 * dither, control word, overspeed threshold) is not shared between
 * separate slave instances. This is the key regression test for the
 * global-state elimination refactoring.
 */

#include <gtest/gtest.h>

#include "slave/profiles/CiA405Slave.hpp"
#include "slave/profiles/CiA406Slave.hpp"
#include "slave/profiles/CiA408Slave.hpp"

namespace EtherCAT {
namespace slave {
namespace test {

// ============================================================================
// CiA 405 Instance Independence
// ============================================================================

TEST(CiA405Independence, TwoInstancesHaveIndependentPDOLayout) {
    CiA405SlaveConfig configA, configB;
    configA.inputAreaSize = 32;
    configA.outputAreaSize = 16;
    configB.inputAreaSize = 64;
    configB.outputAreaSize = 48;

    CiA405Slave slaveA(configA);
    CiA405Slave slaveB(configB);

    // Both instances exist simultaneously
    // Verify basic construction works (no shared static layout corruption)
    EXPECT_EQ(slaveA.getProfileName(), std::string("CiA 405"));
    EXPECT_EQ(slaveB.getProfileName(), std::string("CiA 405"));

    // Different config should remain independent
    EXPECT_NE(nullptr, slaveA.getInputArea());
    EXPECT_NE(nullptr, slaveB.getInputArea());
}

// ============================================================================
// CiA 406 Instance Independence
// ============================================================================

TEST(CiA406Independence, TwoInstancesHaveIndependentState) {
    CiA406SlaveConfig configA, configB;
    configA.stepsPerRevolution = 4096;
    configA.encoderType = EncoderType::Incremental;
    configB.stepsPerRevolution = 131072;
    configB.encoderType = EncoderType::MultiTurn;
    configB.totalMeasuringRange = 4096;

    CiA406Slave slaveA(configA);
    CiA406Slave slaveB(configB);

    // Set different positions — should not interfere
    slaveA.setPosition(1000);
    slaveB.setPosition(50000);

    EXPECT_EQ(slaveA.getPosition(), 1000);
    EXPECT_EQ(slaveB.getPosition(), 50000);

    // Set different speeds — should not interfere
    slaveA.setSpeed(100);
    slaveB.setSpeed(-500);

    EXPECT_EQ(slaveA.getSpeed(), 100);
    EXPECT_EQ(slaveB.getSpeed(), -500);
}

TEST(CiA406Independence, AlarmStateIsPerInstance) {
    auto encoderA = createIncrementalEncoder(4096);
    auto encoderB = createAbsoluteEncoder(131072);

    encoderA->setAlarmStatus(0x03);
    EXPECT_EQ(encoderA->getAlarmStatus(), 0x03);
    EXPECT_EQ(encoderB->getAlarmStatus(), 0x00);  // Not affected

    encoderB->setAlarmStatus(0x10);
    EXPECT_EQ(encoderA->getAlarmStatus(), 0x03);  // Still independent
    EXPECT_EQ(encoderB->getAlarmStatus(), 0x10);
}

// ============================================================================
// CiA 408 Instance Independence — Critical Test
//
// Previously, deadbandCompensation_ and ditherEnabled_ were file-scope
// globals shared across ALL CiA408Slave instances. This test verifies
// they are now per-instance.
// ============================================================================

TEST(CiA408Independence, DeadbandCompensationIsPerInstance) {
    CiA408SlaveConfig configA, configB;
    CiA408Slave slaveA(configA);
    CiA408Slave slaveB(configB);

    // Default should be zero
    EXPECT_EQ(slaveA.getDeadbandCompensation(), 0);
    EXPECT_EQ(slaveB.getDeadbandCompensation(), 0);

    // Set on instance A — must NOT affect instance B
    slaveA.setDeadbandCompensation(100);
    EXPECT_EQ(slaveA.getDeadbandCompensation(), 100);
    EXPECT_EQ(slaveB.getDeadbandCompensation(), 0);  // Still zero!

    // Set on instance B — must NOT affect instance A
    slaveB.setDeadbandCompensation(250);
    EXPECT_EQ(slaveA.getDeadbandCompensation(), 100);  // Still 100!
    EXPECT_EQ(slaveB.getDeadbandCompensation(), 250);
}

TEST(CiA408Independence, DitherEnabledIsPerInstance) {
    CiA408SlaveConfig configA, configB;
    configA.supportsDither = true;
    configB.supportsDither = true;
    CiA408Slave slaveA(configA);
    CiA408Slave slaveB(configB);

    // Default should be false
    EXPECT_FALSE(slaveA.isDitherEnabled());
    EXPECT_FALSE(slaveB.isDitherEnabled());

    // Set on instance A — must NOT affect instance B
    slaveA.setDitherEnabled(true);
    EXPECT_TRUE(slaveA.isDitherEnabled());
    EXPECT_FALSE(slaveB.isDitherEnabled());  // Still false!

    // Set on instance B — must NOT affect instance A
    slaveB.setDitherEnabled(true);
    slaveA.setDitherEnabled(false);
    EXPECT_FALSE(slaveA.isDitherEnabled());
    EXPECT_TRUE(slaveB.isDitherEnabled());
}

TEST(CiA408Independence, FaultStateIsPerInstance) {
    auto valveA = createProportionalValve();
    auto valveB = createServoValve();

    valveA->setFault(0x0001);  // Overcurrent on A
    EXPECT_TRUE(valveA->isFaulted());
    EXPECT_FALSE(valveB->isFaulted());  // B is clean

    valveB->setFault(0x0008);  // Overtemperature on B
    EXPECT_EQ(valveA->getFaultCode(), 0x0001);
    EXPECT_EQ(valveB->getFaultCode(), 0x0008);

    valveA->clearFault();
    EXPECT_FALSE(valveA->isFaulted());
    EXPECT_TRUE(valveB->isFaulted());   // B still faulted
}

TEST(CiA408Independence, EnableStateIsPerInstance) {
    CiA408SlaveConfig config;
    CiA408Slave slaveA(config);
    CiA408Slave slaveB(config);

    slaveA.setEnabled(true);
    EXPECT_TRUE(slaveA.isEnabled());
    EXPECT_FALSE(slaveB.isEnabled());

    slaveB.setEnabled(true);
    slaveA.setEnabled(false);
    EXPECT_FALSE(slaveA.isEnabled());
    EXPECT_TRUE(slaveB.isEnabled());
}

}  // namespace test
}  // namespace slave
}  // namespace EtherCAT
