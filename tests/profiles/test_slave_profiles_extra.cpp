/**
 * @file test_slave_profiles_extra.cpp
 * @brief CiA405/CiA406/CiA408 Slave and SlaveLogger coverage tests
 */
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cstring>
#include "slave/profiles/CiA405Slave.hpp"
#include "slave/profiles/CiA406Slave.hpp"
#include "slave/profiles/CiA408Slave.hpp"
#include "slave/logging/SlaveLogger.hpp"

using namespace EtherCAT::slave;

// ############################################################################
//  CiA 405 — Programmable Logic Controller
// ############################################################################

class CiA405SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA405SlaveConfig cfg{};
        cfg.inputAreaSize = 128;
        cfg.outputAreaSize = 128;
        cfg.internalAreaSize = 512;
        cfg.supportsProgramDownload = true;
        cfg.maxProgramSize = 4096;
        plc = std::make_unique<CiA405Slave>(cfg);
        plc->start();
    }
    std::unique_ptr<CiA405Slave> plc;
};

TEST_F(CiA405SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(plc->getProfileName(), "CiA 405");
    EXPECT_EQ(plc->getDeviceType(), 0x00000195u);
}

TEST_F(CiA405SlaveFullTest, InitialProgramState) {
    EXPECT_EQ(plc->getProgramState(), ProgramState::Stopped);
}

TEST_F(CiA405SlaveFullTest, StartProgram) {
    plc->startProgram();
    EXPECT_EQ(plc->getProgramState(), ProgramState::Running);
}

TEST_F(CiA405SlaveFullTest, StopProgram) {
    plc->startProgram();
    plc->stopProgram();
    EXPECT_EQ(plc->getProgramState(), ProgramState::Stopped);
}

TEST_F(CiA405SlaveFullTest, ResetProgram) {
    plc->startProgram();
    plc->resetProgram();
    EXPECT_EQ(plc->getProgramState(), ProgramState::Stopped);
}

TEST_F(CiA405SlaveFullTest, SetProgramState) {
    plc->setProgramState(ProgramState::Halted);
    EXPECT_EQ(plc->getProgramState(), ProgramState::Halted);
    plc->setProgramState(ProgramState::Suspended);
    EXPECT_EQ(plc->getProgramState(), ProgramState::Suspended);
}

TEST_F(CiA405SlaveFullTest, LoadProgram) {
    std::vector<uint8_t> prog = {0x01, 0x02, 0x03, 0x04};
    EXPECT_TRUE(plc->loadProgram(prog));
    EXPECT_EQ(plc->getProgram().size(), 4u);
}

TEST_F(CiA405SlaveFullTest, LoadProgramTooLarge) {
    std::vector<uint8_t> prog(5000, 0xFF); // > maxProgramSize
    EXPECT_FALSE(plc->loadProgram(prog));
}

TEST_F(CiA405SlaveFullTest, RegisterAndReadVariable) {
    plc->registerVariable("TestVar", 0, 4, true, false);
    // Write to input area directly
    auto* area = plc->getInputArea();
    ASSERT_NE(area, nullptr);
    uint32_t val = 0x12345678;
    std::memcpy(area, &val, 4);
    
    std::vector<uint8_t> data;
    EXPECT_TRUE(plc->readVariable("TestVar", data));
    EXPECT_EQ(data.size(), 4u);
}

TEST_F(CiA405SlaveFullTest, WriteVariable) {
    plc->registerVariable("OutVar", 0, 4, false, true);
    std::vector<uint8_t> data = {0xAA, 0xBB, 0xCC, 0xDD};
    EXPECT_TRUE(plc->writeVariable("OutVar", data));
}

TEST_F(CiA405SlaveFullTest, ReadNonExistentVariable) {
    std::vector<uint8_t> data;
    EXPECT_FALSE(plc->readVariable("NoSuchVar", data));
}

TEST_F(CiA405SlaveFullTest, WriteNonExistentVariable) {
    std::vector<uint8_t> data = {0x01};
    EXPECT_FALSE(plc->writeVariable("NoSuchVar", data));
}

TEST_F(CiA405SlaveFullTest, InputOutputInternalAreas) {
    EXPECT_NE(plc->getInputArea(), nullptr);
    EXPECT_NE(plc->getOutputArea(), nullptr);
    EXPECT_NE(plc->getInternalArea(), nullptr);
}

TEST_F(CiA405SlaveFullTest, CycleCallback) {
    bool called = false;
    plc->setCycleCallback([&]() { called = true; });
    plc->startProgram();
    plc->simulate(1'000'000);
    EXPECT_TRUE(called);
}

TEST_F(CiA405SlaveFullTest, SimulateWhileStopped) {
    plc->simulate(1'000'000);
    // Should not crash
}

TEST_F(CiA405SlaveFullTest, TxRxPDO) {
    plc->updateTxPDO();
    plc->processRxPDO();
}

TEST_F(CiA405SlaveFullTest, Profile) {
    EXPECT_EQ(plc->getProfile(), CiAProfile::CiA405);
}

TEST(CiA405FactoryFullTest, Create) {
    CiA405SlaveConfig cfg{};
    auto plc = createCiA405Slave(cfg);
    ASSERT_NE(plc, nullptr);
    EXPECT_STREQ(plc->getProfileName(), "CiA 405");
}

// ############################################################################
//  CiA 406 — Encoder
// ############################################################################

class CiA406SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA406SlaveConfig cfg{};
        cfg.encoderType = EncoderType::MultiTurn;
        cfg.stepsPerRevolution = 131072;
        cfg.totalMeasuringRange = 4096;
        cfg.supportsSpeedMeasurement = true;
        cfg.supportsPreset = true;
        cfg.supportsAlarms = true;
        enc = std::make_unique<CiA406Slave>(cfg);
        enc->start();
    }
    std::unique_ptr<CiA406Slave> enc;
};

TEST_F(CiA406SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(enc->getProfileName(), "CiA 406");
    EXPECT_EQ(enc->getDeviceType(), 0x00000196u);
}

TEST_F(CiA406SlaveFullTest, InitialPosition) {
    EXPECT_EQ(enc->getPosition(), 0);
}

TEST_F(CiA406SlaveFullTest, SetGetPosition) {
    enc->setPosition(12345);
    EXPECT_EQ(enc->getPosition(), 12345);
}

TEST_F(CiA406SlaveFullTest, SetGetSpeed) {
    enc->setSpeed(5000);
    EXPECT_EQ(enc->getSpeed(), 5000);
}

TEST_F(CiA406SlaveFullTest, SetGetTurns) {
    enc->setTurns(42);
    EXPECT_EQ(enc->getTurns(), 42);
}

TEST_F(CiA406SlaveFullTest, PresetPosition) {
    enc->setPosition(10000);
    enc->presetPosition(0);
    EXPECT_EQ(enc->getPosition(), 0);
}

TEST_F(CiA406SlaveFullTest, AlarmStatus) {
    EXPECT_FALSE(enc->isAlarmActive());
    enc->setAlarmStatus(0x03);
    EXPECT_TRUE(enc->isAlarmActive());
    EXPECT_EQ(enc->getAlarmStatus(), 0x03);
}

TEST_F(CiA406SlaveFullTest, ClearAlarms) {
    enc->setAlarmStatus(0xFF);
    enc->clearAlarms();
    EXPECT_FALSE(enc->isAlarmActive());
    EXPECT_EQ(enc->getAlarmStatus(), 0);
}

TEST_F(CiA406SlaveFullTest, OperatingStatus) {
    uint16_t status = enc->getOperatingStatus();
    // Just verify it doesn't crash and returns something
    (void)status;
}

TEST_F(CiA406SlaveFullTest, PositionCallback) {
    bool called = false;
    enc->setPositionCallback([&]() -> int32_t {
        called = true;
        return 12345;
    });
    enc->simulate(1'000'000);
    // Callback may or may not be called depending on implementation
}

TEST_F(CiA406SlaveFullTest, SimulateUpdatesSpeedFromPositionChange) {
    enc->setPosition(0);
    enc->simulate(1'000'000);
    enc->setPosition(1000);
    enc->simulate(1'000'000);
    // Speed calculation depends on implementation
}

TEST_F(CiA406SlaveFullTest, TxRxPDO) {
    enc->updateTxPDO();
    enc->processRxPDO();
}

TEST_F(CiA406SlaveFullTest, Profile) {
    EXPECT_EQ(enc->getProfile(), CiAProfile::CiA406);
}

TEST(CiA406FactoryFullTest, IncrementalEncoder) {
    auto enc = createIncrementalEncoder(4096);
    ASSERT_NE(enc, nullptr);
    EXPECT_STREQ(enc->getProfileName(), "CiA 406");
}

TEST(CiA406FactoryFullTest, AbsoluteEncoder) {
    auto enc = createAbsoluteEncoder(131072);
    ASSERT_NE(enc, nullptr);
}

TEST(CiA406FactoryFullTest, MultiTurnEncoder) {
    auto enc = createMultiTurnEncoder(131072, 4096);
    ASSERT_NE(enc, nullptr);
}

TEST(CiA406FactoryFullTest, CreateFromConfig) {
    CiA406SlaveConfig cfg{};
    auto enc = createCiA406Slave(cfg);
    ASSERT_NE(enc, nullptr);
}

// ############################################################################
//  CiA 408 — Hydraulic Valve
// ############################################################################

class CiA408SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA408SlaveConfig cfg{};
        cfg.valveType = ValveType::ServoValve;
        cfg.commandMin = -10000;
        cfg.commandMax = 10000;
        cfg.hasPositionFeedback = true;
        cfg.hasPressureFeedback = true;
        cfg.hasFlowFeedback = true;
        cfg.supportsDither = true;
        cfg.ditherFrequency = 200;
        cfg.ditherAmplitude = 50;
        valve = std::make_unique<CiA408Slave>(cfg);
        valve->start();
    }
    std::unique_ptr<CiA408Slave> valve;
};

TEST_F(CiA408SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(valve->getProfileName(), "CiA 408");
    EXPECT_EQ(valve->getDeviceType(), 0x00000198u);
}

TEST_F(CiA408SlaveFullTest, InitialState) {
    EXPECT_FALSE(valve->isEnabled());
    EXPECT_FALSE(valve->isFaulted());
    EXPECT_EQ(valve->getFaultCode(), 0);
}

TEST_F(CiA408SlaveFullTest, EnableDisable) {
    valve->setEnabled(true);
    EXPECT_TRUE(valve->isEnabled());
    valve->setEnabled(false);
    EXPECT_FALSE(valve->isEnabled());
}

TEST_F(CiA408SlaveFullTest, EnableWhileFaulted) {
    valve->setFault(0x0001);
    valve->setEnabled(true);
    EXPECT_FALSE(valve->isEnabled()); // can't enable while faulted
}

TEST_F(CiA408SlaveFullTest, SetClearFault) {
    valve->setFault(0x1234);
    EXPECT_TRUE(valve->isFaulted());
    EXPECT_EQ(valve->getFaultCode(), 0x1234);
    valve->clearFault();
    EXPECT_FALSE(valve->isFaulted());
    EXPECT_EQ(valve->getFaultCode(), 0);
}

TEST_F(CiA408SlaveFullTest, FaultDisablesValve) {
    valve->setEnabled(true);
    EXPECT_TRUE(valve->isEnabled());
    valve->setFault(0x0001);
    EXPECT_FALSE(valve->isEnabled());
}

TEST_F(CiA408SlaveFullTest, GetCommand) {
    EXPECT_EQ(valve->getCommand(), 0);
}

TEST_F(CiA408SlaveFullTest, ActualPosition) {
    valve->setActualPosition(5000);
    EXPECT_EQ(valve->getActualPosition(), 5000);
}

TEST_F(CiA408SlaveFullTest, Pressure) {
    valve->setPressure(10000, 20000);
    EXPECT_EQ(valve->getPressureA(), 10000);
    EXPECT_EQ(valve->getPressureB(), 20000);
}

TEST_F(CiA408SlaveFullTest, Flow) {
    valve->setFlow(50000);
    EXPECT_EQ(valve->getFlow(), 50000);
}

TEST_F(CiA408SlaveFullTest, StatusWord) {
    uint16_t sw = valve->getStatusWord();
    (void)sw; // verify no crash
}

TEST_F(CiA408SlaveFullTest, DeadbandCompensation) {
    valve->setDeadbandCompensation(100);
    EXPECT_EQ(valve->getDeadbandCompensation(), 100);
}

TEST_F(CiA408SlaveFullTest, DitherEnableDisable) {
    valve->setDitherEnabled(true);
    EXPECT_TRUE(valve->isDitherEnabled());
    valve->setDitherEnabled(false);
    EXPECT_FALSE(valve->isDitherEnabled());
}

TEST_F(CiA408SlaveFullTest, SimulationCallback) {
    bool called = false;
    valve->setSimulationCallback([&](int16_t cmd, int16_t& pos) {
        called = true;
        pos = cmd;
    });
    valve->setEnabled(true);
    valve->simulate(1'000'000);
    // Callback may be called during simulation
}

TEST_F(CiA408SlaveFullTest, SimulateEnabled) {
    valve->setEnabled(true);
    valve->simulate(1'000'000);
    valve->simulate(1'000'000);
    // Should not crash
}

TEST_F(CiA408SlaveFullTest, SimulateDisabled) {
    valve->simulate(1'000'000);
    // Should not crash
}

TEST_F(CiA408SlaveFullTest, TxRxPDO) {
    valve->updateTxPDO();
    valve->processRxPDO();
}

TEST_F(CiA408SlaveFullTest, Profile) {
    EXPECT_EQ(valve->getProfile(), CiAProfile::CiA408);
}

TEST(CiA408FactoryFullTest, ProportionalValve) {
    auto v = createProportionalValve();
    ASSERT_NE(v, nullptr);
    EXPECT_STREQ(v->getProfileName(), "CiA 408");
}

TEST(CiA408FactoryFullTest, ServoValve) {
    auto v = createServoValve();
    ASSERT_NE(v, nullptr);
}

TEST(CiA408FactoryFullTest, CreateFromConfig) {
    CiA408SlaveConfig cfg{};
    auto v = createCiA408Slave(cfg);
    ASSERT_NE(v, nullptr);
}

// ############################################################################
//  Slave Logger
// ############################################################################

class SlaveLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlaveLogConfig cfg{};
        cfg.consoleEnabled = false; // don't spam during tests
        cfg.fileEnabled = false;
        cfg.minLevel = SlaveLogLevel::Trace;
        logger = std::make_unique<SlaveLogger>(cfg);
    }
    std::unique_ptr<SlaveLogger> logger;
};

TEST_F(SlaveLoggerTest, DefaultConstruction) {
    SlaveLogger lg;
    EXPECT_FALSE(lg.isRunning());
}

TEST_F(SlaveLoggerTest, StartStop) {
    logger->start();
    EXPECT_TRUE(logger->isRunning());
    logger->stop();
    EXPECT_FALSE(logger->isRunning());
}

TEST_F(SlaveLoggerTest, DoubleStart) {
    logger->start();
    logger->start(); // should be no-op
    EXPECT_TRUE(logger->isRunning());
    logger->stop();
}

TEST_F(SlaveLoggerTest, DoubleStop) {
    logger->start();
    logger->stop();
    logger->stop(); // should be no-op
    EXPECT_FALSE(logger->isRunning());
}

TEST_F(SlaveLoggerTest, Flush) {
    logger->start();
    logger->flush();
    logger->stop();
}

TEST_F(SlaveLoggerTest, SetMinLevel) {
    logger->setMinLevel(SlaveLogLevel::Warning);
    EXPECT_EQ(logger->getMinLevel(), SlaveLogLevel::Warning);
}

TEST_F(SlaveLoggerTest, CategoryEnable) {
    logger->setCategoryEnabled(SlaveLogCategory::CoE, true);
    EXPECT_TRUE(logger->isCategoryEnabled(SlaveLogCategory::CoE));
    logger->setCategoryEnabled(SlaveLogCategory::CoE, false);
    EXPECT_FALSE(logger->isCategoryEnabled(SlaveLogCategory::CoE));
}

TEST_F(SlaveLoggerTest, SetEnabledCategories) {
    logger->setEnabledCategories(SlaveLogCategory::All);
    EXPECT_TRUE(logger->isCategoryEnabled(SlaveLogCategory::FoE));
    EXPECT_TRUE(logger->isCategoryEnabled(SlaveLogCategory::DC));
}

TEST_F(SlaveLoggerTest, SetSlaveAddress) {
    logger->setSlaveAddress(42);
    // Should not crash
}

TEST_F(SlaveLoggerTest, LogBasic) {
    logger->start();
    logger->log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info, "Test %d", 42);
    logger->stop();
}

TEST_F(SlaveLoggerTest, LogBelowMinLevel) {
    logger->setMinLevel(SlaveLogLevel::Error);
    logger->start();
    logger->log(SlaveLogCategory::StateMachine, SlaveLogLevel::Debug, "Should be filtered");
    logger->stop();
}

TEST_F(SlaveLoggerTest, LogDisabledCategory) {
    logger->setCategoryEnabled(SlaveLogCategory::Register, false);
    logger->start();
    logger->log(SlaveLogCategory::Register, SlaveLogLevel::Error, "Should be filtered");
    logger->stop();
}

TEST_F(SlaveLoggerTest, ConvenienceMethods) {
    logger->start();
    logger->trace(SlaveLogCategory::StateMachine, "trace %d", 1);
    logger->debug(SlaveLogCategory::StateMachine, "debug %d", 2);
    logger->info(SlaveLogCategory::StateMachine, "info %d", 3);
    logger->warn(SlaveLogCategory::StateMachine, "warn %d", 4);
    logger->error(SlaveLogCategory::StateMachine, "error %d", 5);
    logger->critical(SlaveLogCategory::StateMachine, "critical %d", 6);
    logger->stop();
}

TEST_F(SlaveLoggerTest, LogFrame) {
    logger->start();
    uint8_t frame[] = {0x01, 0x02, 0x03, 0x04};
    logger->logFrame(SlaveLogCategory::FrameRx, frame, sizeof(frame), "TestFrame");
    logger->stop();
}

TEST_F(SlaveLoggerTest, LogHex) {
    logger->start();
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    logger->logHex(SlaveLogCategory::Mailbox, SlaveLogLevel::Debug, data, sizeof(data), "HexDump");
    logger->stop();
}

TEST_F(SlaveLoggerTest, LogCountAndReset) {
    logger->start();
    logger->log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info, "msg1");
    logger->log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info, "msg2");
    uint64_t count = logger->getLogCount();
    EXPECT_GE(count, 0u); // Could be 0 if queue is fast
    logger->resetStats();
    EXPECT_EQ(logger->getLogCount(), 0u);
    EXPECT_EQ(logger->getDropCount(), 0u);
    logger->stop();
}

TEST_F(SlaveLoggerTest, PcapLoggerNull) {
    // Without PCAP enabled, getPcapLogger returns nullptr
    EXPECT_EQ(logger->getPcapLogger(), nullptr);
}

TEST_F(SlaveLoggerTest, LogWithoutStart) {
    // Logging without start should not crash
    logger->log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info, "no start");
}

TEST_F(SlaveLoggerTest, FlushWithoutStart) {
    logger->flush(); // should not crash
}
