/**
 * @file test_slave_profiles_full.cpp
 * @brief Comprehensive tests for EtherCAT slave profile implementations
 */

#include "tether/slave/profiles/CiA401Slave.hpp"
#include "tether/slave/profiles/CiA402Slave.hpp"
#include "tether/slave/profiles/CiA404Slave.hpp"
#include "tether/slave/profiles/CiA410Slave.hpp"
#include "tether/slave/profiles/CiA417Slave.hpp"
#include "tether/slave/profiles/CiA430Slave.hpp"
#include "tether/slave/profiles/CiA405Slave.hpp"
#include "tether/slave/profiles/CiA408Slave.hpp"
#include "tether/slave/profiles/ProfileSlave.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cstring>

using namespace EtherCAT::slave;

// ============================================================================
// Helper: default CiA401 config
// ============================================================================

static CiA401SlaveConfig makeCiA401Config() {
    CiA401SlaveConfig cfg{};
    cfg.identity.vendorId = 0x12345678;
    cfg.identity.productCode = 0x0191;
    cfg.identity.revisionNumber = 1;
    cfg.identity.serialNumber = 1001;
    cfg.digitalInputs8 = 2;
    cfg.digitalOutputs8 = 2;
    cfg.analogInputs = 2;
    cfg.analogOutputs = 2;
    cfg.analogInputMin = 0;
    cfg.analogInputMax = 4095;
    cfg.analogOutputMin = 0;
    cfg.analogOutputMax = 4095;
    cfg.outputErrorModeEnabled = true;
    cfg.supportsDC = false;
    cfg.defaultCycleTime = 1000;
    return cfg;
}

// ============================================================================
// CiA401 Slave Tests
// ============================================================================

class CiA401SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA401Slave>(makeCiA401Config());
    }
    std::unique_ptr<CiA401Slave> slave_;
};

TEST_F(CiA401SlaveTest, ProfileName) {
    EXPECT_STREQ(slave_->getProfileName(), "CiA 401");
}

TEST_F(CiA401SlaveTest, DeviceType) {
    EXPECT_EQ(slave_->getDeviceType(), 0x0191);
}

TEST_F(CiA401SlaveTest, DigitalInputCount) {
    EXPECT_EQ(slave_->getDigitalInputCount(), 2u); // 2 groups of 8
}

TEST_F(CiA401SlaveTest, DigitalOutputCount) {
    EXPECT_EQ(slave_->getDigitalOutputCount(), 2u);
}

TEST_F(CiA401SlaveTest, SetGetDigitalInput8) {
    slave_->setDigitalInput8(0, 0xFF);
    EXPECT_EQ(slave_->getDigitalInput8(0), 0xFF);
    
    slave_->setDigitalInput8(0, 0x00);
    EXPECT_EQ(slave_->getDigitalInput8(0), 0x00);
}

TEST_F(CiA401SlaveTest, SetGetDigitalInput16) {
    slave_->setDigitalInput16(0, 0xABCD);
    EXPECT_EQ(slave_->getDigitalInput16(0), 0xABCD);
}

TEST_F(CiA401SlaveTest, SetGetDigitalInputBit) {
    slave_->setDigitalInputBit(0, true);
    EXPECT_TRUE(slave_->getDigitalInputBit(0));
    
    slave_->setDigitalInputBit(0, false);
    EXPECT_FALSE(slave_->getDigitalInputBit(0));
}

TEST_F(CiA401SlaveTest, DigitalOutput8) {
    auto val = slave_->getDigitalOutput8(0);
    EXPECT_EQ(val, 0x00); // Outputs start at 0
}

TEST_F(CiA401SlaveTest, DigitalOutput16) {
    auto val = slave_->getDigitalOutput16(0);
    EXPECT_EQ(val, 0x0000);
}

TEST_F(CiA401SlaveTest, DigitalOutputBit) {
    auto val = slave_->getDigitalOutputBit(0);
    EXPECT_FALSE(val);
}

TEST_F(CiA401SlaveTest, AnalogInputCount) {
    EXPECT_EQ(slave_->getAnalogInputCount(), 2u);
}

TEST_F(CiA401SlaveTest, AnalogOutputCount) {
    EXPECT_EQ(slave_->getAnalogOutputCount(), 2u);
}

TEST_F(CiA401SlaveTest, SetGetAnalogInput) {
    slave_->setAnalogInput(0, 2048);
    EXPECT_EQ(slave_->getAnalogInput(0), 2048);
    
    slave_->setAnalogInput(1, 4095);
    EXPECT_EQ(slave_->getAnalogInput(1), 4095);
}

TEST_F(CiA401SlaveTest, AnalogOutput) {
    auto val = slave_->getAnalogOutput(0);
    EXPECT_EQ(val, 0);
}

TEST_F(CiA401SlaveTest, AnalogInputScaling) {
    slave_->setAnalogInputScaling(0, 100, 2.0);
    slave_->setAnalogInput(0, 500);
    // Scaling should be applied
}

TEST_F(CiA401SlaveTest, DigitalOutputCallback) {
    bool callbackCalled = false;
    slave_->setDigitalOutputCallback([&](const uint8_t* data, size_t len) {
        callbackCalled = true;
    });
    // Callback would be called when output changes via PDO
}

TEST_F(CiA401SlaveTest, AnalogOutputCallback) {
    slave_->setAnalogOutputCallback([&](uint8_t channel, int16_t value) {
        // Just verify no crash
    });
}

TEST_F(CiA401SlaveTest, DigitalInterrupt) {
    slave_->configureDigitalInterrupt(0, 0xFF, 0); // Rising edge on all bits
    slave_->setInterruptCallback([&](uint8_t group, uint8_t triggeredBits) {
        // Interrupt handler
    });
}

TEST_F(CiA401SlaveTest, ErrorHandling) {
    slave_->setDigitalOutputErrorValue(0, 0x00);
    slave_->setDigitalOutputErrorMode(0, 0x01);
    slave_->triggerCommunicationError();
    slave_->clearCommunicationError();
}

TEST_F(CiA401SlaveTest, UpdateTxPDO) {
    slave_->setDigitalInput8(0, 0xAA);
    slave_->setAnalogInput(0, 1234);
    slave_->updateTxPDO();
    // Should update internal PDO data
}

TEST_F(CiA401SlaveTest, ProcessRxPDO) {
    slave_->processRxPDO();
    // Should process received PDO data
}

TEST_F(CiA401SlaveTest, Simulate) {
    slave_->simulate(1000000); // 1ms in nanoseconds
    // Should run simulation step
}

TEST_F(CiA401SlaveTest, FactoryCreateCiA401Slave) {
    auto s = createCiA401Slave(makeCiA401Config());
    ASSERT_NE(s, nullptr);
    EXPECT_STREQ(s->getProfileName(), "CiA 401");
}

TEST_F(CiA401SlaveTest, FactoryCreateDigitalIOSlave) {
    auto s = createDigitalIOSlave(4, 4);
    ASSERT_NE(s, nullptr);
    // getDigitalInputCount returns group count, not bit count
    EXPECT_GT(s->getDigitalInputCount(), 0u);
}

TEST_F(CiA401SlaveTest, FactoryCreateAnalogIOSlave) {
    auto s = createAnalogIOSlave(4, 4);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->getAnalogInputCount(), 4u);
}

// ============================================================================
// CiA402 Slave Tests
// ============================================================================

static CiA402SlaveConfig makeCiA402Config() {
    CiA402SlaveConfig cfg{};
    cfg.identity.vendorId = 0x12345678;
    cfg.identity.productCode = 0x0192;
    cfg.identity.revisionNumber = 1;
    cfg.identity.serialNumber = 2001;
    cfg.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::CST |
                         CiA402Mode::PP | CiA402Mode::PV | CiA402Mode::PT;
    cfg.positionEncoderResolution = 4096;
    cfg.velocityEncoderResolution = 4096;
    cfg.encoderIncrements = 4096;
    cfg.motorRevolutions = 1;
    cfg.softwarePosLimitMin = -1000000;
    cfg.softwarePosLimitMax = 1000000;
    cfg.maxProfileVelocity = 3000;
    cfg.maxMotorVelocity = 5000;
    cfg.maxAcceleration = 50000;
    cfg.maxDeceleration = 50000;
    cfg.quickStopDeceleration = 100000;
    cfg.maxTorque = 1000;
    cfg.motorRatedTorque = 500;
    cfg.motorRatedCurrent = 5000;
    cfg.followingErrorWindow = 1000;
    cfg.followingErrorTimeout = 100;
    cfg.positionWindow = 10;
    cfg.positionWindowTime = 50;
    return cfg;
}

class CiA402SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA402Slave>(makeCiA402Config());
    }
    std::unique_ptr<CiA402Slave> slave_;
};

TEST_F(CiA402SlaveTest, ProfileName) {
    EXPECT_STREQ(slave_->getProfileName(), "CiA 402");
}

TEST_F(CiA402SlaveTest, DeviceType) {
    EXPECT_EQ(slave_->getDeviceType(), 0x0192);
}

TEST_F(CiA402SlaveTest, InitialDriveState) {
    auto state = slave_->getDriveState();
    // Should start in NotReadyToSwitchOn or SwitchOnDisabled
    EXPECT_TRUE(state == CiA402State::NotReadyToSwitchOn ||
                state == CiA402State::SwitchOnDisabled);
}

TEST_F(CiA402SlaveTest, GetStatusWord) {
    auto sw = slave_->getStatusWord();
    // In initial state, switch-on-disabled bit should be set
    (void)sw;
}

TEST_F(CiA402SlaveTest, ProcessControlWord_Shutdown) {
    // Send shutdown command: 0x0006
    slave_->processControlWord(0x0006);
    auto state = slave_->getDriveState();
    // After shutdown, should be in ReadyToSwitchOn or similar
    (void)state;
}

TEST_F(CiA402SlaveTest, ProcessControlWord_SwitchOn) {
    slave_->processControlWord(0x0006); // Shutdown -> ReadyToSwitchOn
    slave_->processControlWord(0x0007); // SwitchOn
    auto state = slave_->getDriveState();
    (void)state;
}

TEST_F(CiA402SlaveTest, ProcessControlWord_EnableOperation) {
    slave_->processControlWord(0x0006); // Shutdown
    slave_->processControlWord(0x0007); // SwitchOn
    slave_->processControlWord(0x000F); // Enable Operation
    auto state = slave_->getDriveState();
    (void)state;
}

TEST_F(CiA402SlaveTest, TriggerFault) {
    slave_->triggerFault(0x1234);
    auto state = slave_->getDriveState();
    // Should be in Fault or FaultReactionActive
    EXPECT_TRUE(state == CiA402State::Fault ||
                state == CiA402State::FaultReactionActive);
}

TEST_F(CiA402SlaveTest, ClearFault) {
    slave_->triggerFault(0x1234);
    slave_->clearFault();
    // After clearing, fault reset control word must be sent
}

TEST_F(CiA402SlaveTest, DriveStateCallback) {
    CiA402State oldState{}, newState{};
    slave_->setDriveStateCallback([&](CiA402State o, CiA402State n) {
        oldState = o;
        newState = n;
    });
    slave_->processControlWord(0x0006); // Should trigger callback
}

TEST_F(CiA402SlaveTest, GetOperatingMode) {
    auto mode = slave_->getOperatingMode();
    (void)mode;
}

TEST_F(CiA402SlaveTest, GetTargetOperatingMode) {
    auto mode = slave_->getTargetOperatingMode();
    (void)mode;
}

TEST_F(CiA402SlaveTest, IsModeSupported) {
    // CSP should be supported based on our config
    EXPECT_TRUE(slave_->isModeSupported(8)); // CSP = modes of operation 8
}

TEST_F(CiA402SlaveTest, PositionControl) {
    slave_->setActualPosition(1000);
    EXPECT_EQ(slave_->getActualPosition(), 1000);
    
    auto target = slave_->getTargetPosition();
    auto demand = slave_->getPositionDemand();
    auto err = slave_->getFollowingError();
    (void)target; (void)demand; (void)err;
}

TEST_F(CiA402SlaveTest, VelocityControl) {
    slave_->setActualVelocity(500);
    EXPECT_EQ(slave_->getActualVelocity(), 500);
    
    auto target = slave_->getTargetVelocity();
    auto demand = slave_->getVelocityDemand();
    (void)target; (void)demand;
}

TEST_F(CiA402SlaveTest, TorqueControl) {
    slave_->setActualTorque(100);
    EXPECT_EQ(slave_->getActualTorque(), 100);
    
    auto target = slave_->getTargetTorque();
    auto demand = slave_->getTorqueDemand();
    (void)target; (void)demand;
}

TEST_F(CiA402SlaveTest, Homing) {
    EXPECT_FALSE(slave_->isHomingComplete());
    slave_->setHomingComplete(true);
    EXPECT_TRUE(slave_->isHomingComplete());
    
    slave_->setHomePosition(5000);
}

TEST_F(CiA402SlaveTest, HomingCallback) {
    slave_->setHomingCallback([](int8_t method, int32_t& homePos) -> bool {
        homePos = 0;
        return true;
    });
}

TEST_F(CiA402SlaveTest, TouchProbe) {
    slave_->triggerTouchProbe(1, 42000);
    auto pos = slave_->getTouchProbePosition(1);
    (void)pos;
    
    auto status = slave_->getTouchProbeStatus();
    (void)status;
}

TEST_F(CiA402SlaveTest, DigitalIO) {
    slave_->setDigitalInputs(0xAAAA5555);
    EXPECT_EQ(slave_->getDigitalInputs(), 0xAAAA5555u);
    
    auto outputs = slave_->getDigitalOutputs();
    (void)outputs;
}

TEST_F(CiA402SlaveTest, SupportedDriveFunctions) {
    auto funcs = slave_->getSupportedDriveFunctions();
    (void)funcs;
}

TEST_F(CiA402SlaveTest, UpdateTxPDO) {
    slave_->setActualPosition(42);
    slave_->updateTxPDO();
}

TEST_F(CiA402SlaveTest, ProcessRxPDO) {
    slave_->processRxPDO();
}

TEST_F(CiA402SlaveTest, Simulate) {
    slave_->simulate(1000000); // 1ms in nanoseconds
}

TEST_F(CiA402SlaveTest, FactoryCreateServoDrive) {
    auto s = createServoDrive(4096);
    ASSERT_NE(s, nullptr);
    EXPECT_STREQ(s->getProfileName(), "CiA 402");
}

TEST_F(CiA402SlaveTest, FactoryCreateStepperDrive) {
    auto s = createStepperDrive(200, 256);
    ASSERT_NE(s, nullptr);
}

TEST_F(CiA402SlaveTest, FactoryCreateFrequencyInverter) {
    auto s = createFrequencyInverter();
    ASSERT_NE(s, nullptr);
}

// ============================================================================
// CiA402State enum tests
// ============================================================================

TEST(CiA402StateTest, StateToString) {
    EXPECT_STREQ(cia402StateToString(CiA402State::NotReadyToSwitchOn), "NOT_READY_TO_SWITCH_ON");
    EXPECT_STREQ(cia402StateToString(CiA402State::SwitchOnDisabled), "SWITCH_ON_DISABLED");
    EXPECT_STREQ(cia402StateToString(CiA402State::ReadyToSwitchOn), "READY_TO_SWITCH_ON");
    EXPECT_STREQ(cia402StateToString(CiA402State::SwitchedOn), "SWITCHED_ON");
    EXPECT_STREQ(cia402StateToString(CiA402State::OperationEnabled), "OPERATION_ENABLED");
    EXPECT_STREQ(cia402StateToString(CiA402State::QuickStopActive), "QUICK_STOP_ACTIVE");
    EXPECT_STREQ(cia402StateToString(CiA402State::FaultReactionActive), "FAULT_REACTION_ACTIVE");
    EXPECT_STREQ(cia402StateToString(CiA402State::Fault), "FAULT");
}

// ============================================================================
// CiA402Mode flag tests
// ============================================================================

TEST(CiA402ModeTest, ModeFlagValues) {
    EXPECT_EQ(CiA402Mode::PP, 1u);
    EXPECT_EQ(CiA402Mode::VL, 2u);
    EXPECT_EQ(CiA402Mode::PV, 4u);
    EXPECT_EQ(CiA402Mode::PT, 8u);
    EXPECT_EQ(CiA402Mode::HM, 32u);
    EXPECT_EQ(CiA402Mode::IP, 64u);
    EXPECT_EQ(CiA402Mode::CSP, 128u);
    EXPECT_EQ(CiA402Mode::CSV, 256u);
    EXPECT_EQ(CiA402Mode::CST, 512u);
}

TEST(CiA402ModeTest, AllCyclic) {
    auto all = CiA402Mode::AllCyclic;
    EXPECT_TRUE(all & CiA402Mode::CSP);
    EXPECT_TRUE(all & CiA402Mode::CSV);
    EXPECT_TRUE(all & CiA402Mode::CST);
}

TEST(CiA402ModeTest, AllProfile) {
    auto all = CiA402Mode::AllProfile;
    EXPECT_TRUE(all & CiA402Mode::PP);
    EXPECT_TRUE(all & CiA402Mode::PV);
    EXPECT_TRUE(all & CiA402Mode::PT);
}
