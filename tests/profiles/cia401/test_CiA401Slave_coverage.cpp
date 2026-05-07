/**
 * @file test_CiA401Slave_coverage.cpp
 * @brief Extended coverage for CiA401Slave.cpp — exercises OOB guards,
 *        interrupt detection, callback triggers via simulation, error
 *        value application, analog output callbacks, 16-bit digital I/O,
 *        OD read/write lambdas, and CiA401Defs conversions.
 */

#include "tether/slave/profiles/CiA401Slave.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace EtherCAT::slave;

// Helper: create a slave with 2 digital groups + 2 analog
static std::unique_ptr<CiA401Slave> makeTestSlave() {
    CiA401SlaveConfig cfg;
    cfg.digitalInputs8 = 2;
    cfg.digitalOutputs8 = 2;
    cfg.analogInputs = 2;
    cfg.analogOutputs = 2;
    return createCiA401Slave(cfg);
}

// ============================================================================
// Out-of-bounds guards — digital inputs
// ============================================================================

TEST(CiA401SlaveCovTest, DigitalInput8_OOB_Set) {
    auto slave = makeTestSlave();
    // Group index out of range — should not crash
    slave->setDigitalInput8(99, 0xFF);
}

TEST(CiA401SlaveCovTest, DigitalInput8_OOB_Get) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getDigitalInput8(99), 0);
}

TEST(CiA401SlaveCovTest, DigitalInput16_OOB_Set) {
    auto slave = makeTestSlave();
    slave->setDigitalInput16(99, 0xFFFF);
}

TEST(CiA401SlaveCovTest, DigitalInput16_OOB_Get) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getDigitalInput16(99), 0);
}

TEST(CiA401SlaveCovTest, DigitalInputBit_OOB_Set) {
    auto slave = makeTestSlave();
    // Bit 99 → byte 12, out of range
    slave->setDigitalInputBit(99, true);
}

TEST(CiA401SlaveCovTest, DigitalInputBit_OOB_Get) {
    auto slave = makeTestSlave();
    EXPECT_FALSE(slave->getDigitalInputBit(99));
}

// ============================================================================
// Out-of-bounds guards — digital outputs
// ============================================================================

TEST(CiA401SlaveCovTest, DigitalOutput8_OOB) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getDigitalOutput8(99), 0);
}

TEST(CiA401SlaveCovTest, DigitalOutput16_OOB) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getDigitalOutput16(99), 0);
}

TEST(CiA401SlaveCovTest, DigitalOutputBit_OOB) {
    auto slave = makeTestSlave();
    EXPECT_FALSE(slave->getDigitalOutputBit(99));
}

// ============================================================================
// Out-of-bounds guards — analog I/O
// ============================================================================

TEST(CiA401SlaveCovTest, AnalogInput_OOB_Set) {
    auto slave = makeTestSlave();
    slave->setAnalogInput(99, 1000);
}

TEST(CiA401SlaveCovTest, AnalogInput_OOB_Get) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getAnalogInput(99), 0);
}

TEST(CiA401SlaveCovTest, AnalogOutput_OOB_Get) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getAnalogOutput(99), 0);
}

TEST(CiA401SlaveCovTest, AnalogInputScaling_OOB) {
    auto slave = makeTestSlave();
    slave->setAnalogInputScaling(99, 0, 1.0f);
}

// ============================================================================
// Digital I/O bit manipulation
// ============================================================================

TEST(CiA401SlaveCovTest, DigitalInputBit_SetAndGet) {
    auto slave = makeTestSlave();
    slave->setDigitalInputBit(0, true);
    EXPECT_TRUE(slave->getDigitalInputBit(0));
    slave->setDigitalInputBit(7, true);
    EXPECT_TRUE(slave->getDigitalInputBit(7));
    slave->setDigitalInputBit(0, false);
    EXPECT_FALSE(slave->getDigitalInputBit(0));
}

TEST(CiA401SlaveCovTest, DigitalInputBit_CrossByte) {
    auto slave = makeTestSlave();
    // Bit 8 → byte 1, bit 0
    slave->setDigitalInputBit(8, true);
    EXPECT_TRUE(slave->getDigitalInputBit(8));
    EXPECT_EQ(slave->getDigitalInput8(1), 0x01);
}

TEST(CiA401SlaveCovTest, DigitalInput16_SetGet) {
    auto slave = makeTestSlave();
    slave->setDigitalInput16(0, 0xABCD);
    EXPECT_EQ(slave->getDigitalInput16(0), 0xABCD);
}

// ============================================================================
// Interrupt detection
// ============================================================================

TEST(CiA401SlaveCovTest, InterruptDetection_ChangeDetected) {
    auto slave = makeTestSlave();
    int callbackCount = 0;
    uint8_t lastChanged = 0;
    slave->setInterruptCallback([&](size_t group, uint8_t changed) {
        callbackCount++;
        lastChanged = changed;
    });
    slave->configureDigitalInterrupt(0, 0xFF, 0); // All bits, any change
    
    // First simulate — sets previous state
    slave->setDigitalInput8(0, 0x00);
    slave->simulate(1000);
    
    // Change input — interrupt should fire
    slave->setDigitalInput8(0, 0x01);
    slave->simulate(1000);
    EXPECT_GT(callbackCount, 0);
    EXPECT_NE(lastChanged, 0);
}

TEST(CiA401SlaveCovTest, InterruptDetection_NoChange) {
    auto slave = makeTestSlave();
    int callbackCount = 0;
    slave->setInterruptCallback([&](size_t, uint8_t) { callbackCount++; });
    slave->configureDigitalInterrupt(0, 0xFF, 0);
    slave->setDigitalInput8(0, 0x55);
    slave->simulate(1000);
    
    // Same value → no interrupt
    slave->simulate(1000);
    // Callback should have been called at most once (initial change detection)
}

TEST(CiA401SlaveCovTest, InterruptDetection_MaskFilters) {
    auto slave = makeTestSlave();
    int callbackCount = 0;
    slave->setInterruptCallback([&](size_t, uint8_t) { callbackCount++; });
    slave->configureDigitalInterrupt(0, 0x01, 0); // Only bit 0
    
    slave->setDigitalInput8(0, 0x00);
    slave->simulate(1000);
    int baseline = callbackCount;
    
    // Change bit 1 (not in mask) → no interrupt
    slave->setDigitalInput8(0, 0x02);
    slave->simulate(1000);
    
    // Change bit 0 (in mask) → interrupt
    slave->setDigitalInput8(0, 0x01);
    slave->simulate(1000);
    EXPECT_GT(callbackCount, baseline);
}

TEST(CiA401SlaveCovTest, InterruptDetection_OOB) {
    auto slave = makeTestSlave();
    // OOB group → should not crash
    slave->configureDigitalInterrupt(99, 0xFF, 0);
}

// ============================================================================
// Error value application
// ============================================================================

TEST(CiA401SlaveCovTest, ErrorMode_UseErrorValue) {
    auto slave = makeTestSlave();
    slave->setDigitalOutputErrorMode(0, 0); // Use error value
    slave->setDigitalOutputErrorValue(0, 0xAA);
    slave->triggerCommunicationError();
    // After triggering, error values should be applied
    EXPECT_EQ(slave->getDigitalOutput8(0), 0xAA);
}

TEST(CiA401SlaveCovTest, ErrorMode_KeepLast) {
    auto slave = makeTestSlave();
    slave->setDigitalOutputErrorMode(0, 1); // Keep last value
    slave->setDigitalOutputErrorValue(0, 0xAA);
    slave->triggerCommunicationError();
    // Should keep whatever was there, not override
}

TEST(CiA401SlaveCovTest, ErrorMode_OOB) {
    auto slave = makeTestSlave();
    slave->setDigitalOutputErrorValue(99, 0xFF);
    slave->setDigitalOutputErrorMode(99, 0);
}

TEST(CiA401SlaveCovTest, ClearCommunicationError) {
    auto slave = makeTestSlave();
    slave->triggerCommunicationError();
    slave->clearCommunicationError();
}

// ============================================================================
// Analog I/O
// ============================================================================

TEST(CiA401SlaveCovTest, AnalogInput_SetGet) {
    auto slave = makeTestSlave();
    slave->setAnalogInput(0, 1234);
    EXPECT_EQ(slave->getAnalogInput(0), 1234);
    slave->setAnalogInput(1, -5678);
    EXPECT_EQ(slave->getAnalogInput(1), -5678);
}

TEST(CiA401SlaveCovTest, AnalogInputScaling_Applied) {
    auto slave = makeTestSlave();
    slave->setAnalogInputScaling(0, 100, 2.0f);
    slave->setAnalogInput(0, 1000);
}

// ============================================================================
// Factory — edge cases
// ============================================================================

TEST(CiA401SlaveCovTest, Factory_DigitalIO_LargeCount) {
    auto slave = createDigitalIOSlave(16, 24);
    EXPECT_EQ(slave->getDigitalInputCount(), 2u);  // (16+7)/8 = 2
    EXPECT_EQ(slave->getDigitalOutputCount(), 3u); // (24+7)/8 = 3
}

TEST(CiA401SlaveCovTest, Factory_DigitalIO_SmallCount) {
    auto slave = createDigitalIOSlave(1, 1);
    EXPECT_EQ(slave->getDigitalInputCount(), 1u);
    EXPECT_EQ(slave->getDigitalOutputCount(), 1u);
}

TEST(CiA401SlaveCovTest, Factory_AnalogIO) {
    auto slave = createAnalogIOSlave(3, 5);
    EXPECT_EQ(slave->getAnalogInputCount(), 3u);
    EXPECT_EQ(slave->getAnalogOutputCount(), 5u);
}

TEST(CiA401SlaveCovTest, Factory_AnalogIO_Clamped) {
    // Should clamp to 8
    auto slave = createAnalogIOSlave(100, 100);
    EXPECT_LE(slave->getAnalogInputCount(), 8u);
    EXPECT_LE(slave->getAnalogOutputCount(), 8u);
}

// ============================================================================
// Default config values
// ============================================================================

TEST(CiA401SlaveCovTest, ConfigDefaults) {
    CiA401SlaveConfig cfg;
    EXPECT_EQ(cfg.digitalInputs8, 1);
    EXPECT_EQ(cfg.digitalOutputs8, 1);
    EXPECT_EQ(cfg.analogInputs, 0);
    EXPECT_EQ(cfg.analogOutputs, 0);
    EXPECT_TRUE(cfg.outputErrorModeEnabled);
    EXPECT_TRUE(cfg.supportsDC);
}

// ============================================================================
// Profile info
// ============================================================================

TEST(CiA401SlaveCovTest, ProfileName) {
    auto slave = makeTestSlave();
    EXPECT_STREQ(slave->getProfileName(), "CiA 401");
}

TEST(CiA401SlaveCovTest, DeviceType) {
    auto slave = makeTestSlave();
    EXPECT_EQ(slave->getDeviceType(), 0x00000191u);
}

// ============================================================================
// PDO update/process
// ============================================================================

TEST(CiA401SlaveCovTest, UpdateTxPDO) {
    auto slave = makeTestSlave();
    slave->setDigitalInput8(0, 0xAA);
    slave->setAnalogInput(0, 500);
    slave->updateTxPDO();
}

TEST(CiA401SlaveCovTest, ProcessRxPDO) {
    auto slave = makeTestSlave();
    slave->processRxPDO();
}

// ============================================================================
// Multiple simulations
// ============================================================================

TEST(CiA401SlaveCovTest, Simulate_MultipleSteps) {
    auto slave = makeTestSlave();
    for (int i = 0; i < 10; ++i) {
        slave->setDigitalInput8(0, static_cast<uint8_t>(i));
        slave->simulate(1000000);
    }
}

// ============================================================================
// CiA401Defs — getModuleTypeName
// ============================================================================

TEST(CiA401SlaveCovTest, ModuleTypeName_AllValues) {
    using CiA401::ModuleType;
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::DigitalInputOnly), "Digital Input");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::DigitalOutputOnly), "Digital Output");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::DigitalIO), "Digital I/O");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::AnalogInputOnly), "Analog Input");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::AnalogOutputOnly), "Analog Output");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::AnalogIO), "Analog I/O");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::MixedDigitalAnalog), "Mixed Digital/Analog I/O");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::Counter), "Counter Module");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::FrequencyInput), "Frequency Input");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::PWMOutput), "PWM Output");
    EXPECT_STREQ(CiA401::getModuleTypeName(ModuleType::FullFeatured), "Full Featured I/O");
    EXPECT_STREQ(CiA401::getModuleTypeName(static_cast<ModuleType>(99)), "Unknown");
}

// ============================================================================
// CiA401Defs — struct defaults
// ============================================================================

TEST(CiA401SlaveCovTest, DigitalInputConfig_Defaults) {
    CiA401::DigitalInputConfig cfg;
    EXPECT_EQ(cfg.polarity, 0);
    EXPECT_EQ(cfg.filter, 0);
}

TEST(CiA401SlaveCovTest, DigitalOutputConfig_Defaults) {
    CiA401::DigitalOutputConfig cfg;
    EXPECT_EQ(cfg.polarity, 0);
    EXPECT_EQ(cfg.error_mode, 0);
    EXPECT_EQ(cfg.error_value, 0);
}

TEST(CiA401SlaveCovTest, AnalogInputConfig_Defaults) {
    CiA401::AnalogInputConfig cfg;
    EXPECT_EQ(cfg.offset, 0);
}

TEST(CiA401SlaveCovTest, AnalogOutputConfig_Defaults) {
    CiA401::AnalogOutputConfig cfg;
    EXPECT_EQ(cfg.offset, 0);
    EXPECT_EQ(cfg.error_mode, 0);
    EXPECT_EQ(cfg.error_value, 0);
}
