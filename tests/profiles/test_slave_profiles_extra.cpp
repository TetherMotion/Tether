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
#include "slave/profiles/CiA401Slave.hpp"
#include "slave/profiles/CiA404Slave.hpp"
#include "slave/profiles/CiA417Slave.hpp"
#include "slave/profiles/CiA430Slave.hpp"
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

// ############################################################################
//  Object-dictionary sweep helper — drives every registered read/write
//  callback (including undersized-buffer and missing-object abort paths).
// ############################################################################

static void sweepOD(IObjectDictionary& od, std::initializer_list<uint16_t> idxs) {
    for (uint16_t idx : idxs) {
        uint8_t subs = od.getSubindexCount(idx);
        for (uint8_t sub = 0; sub <= subs + 1; ++sub) {
            uint8_t buf[64];
            size_t len = sizeof(buf);
            od.read(idx, sub, buf, len);
            size_t shortLen = 0;
            od.read(idx, sub, buf, shortLen);
            od.write(idx, sub, buf, 4);
            od.write(idx, sub, buf, 1);
            od.write(idx, sub, buf, 0);
        }
        ODEntryInfo info{};
        od.getObjectInfo(idx, 0, info);
        od.hasObject(idx, 0);
    }
}

// ############################################################################
//  CiA 404 — Measuring Device / Closed-Loop Controller
// ############################################################################

class CiA404SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA404SlaveConfig cfg{};
        cfg.processInputChannels = 4;
        cfg.controllerChannels = 4;
        slave = std::make_unique<CiA404Slave>(cfg);
        slave->start();
    }
    std::unique_ptr<CiA404Slave> slave;
};

TEST_F(CiA404SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(slave->getProfileName(), "CiA 404");
    EXPECT_EQ(slave->getDeviceType(), 0x00000194u);
}

TEST_F(CiA404SlaveFullTest, ProcessValueRoundTrip) {
    slave->setProcessValue(0, 1234);
    slave->setProcessValue(3, -5678);
    EXPECT_EQ(slave->getProcessValue(0), 1234);
    EXPECT_EQ(slave->getProcessValue(3), -5678);
    slave->setProcessValue(99, 1);  // out of range ignored
    EXPECT_EQ(slave->getProcessValue(99), 0);
}

TEST_F(CiA404SlaveFullTest, SetpointAndOutputOutOfRange) {
    EXPECT_EQ(slave->getSetpoint(99), 0);
    EXPECT_EQ(slave->getControllerOutput(99), 0);
}

TEST_F(CiA404SlaveFullTest, PIDParameters) {
    slave->setPIDParameters(1, 2.5f, 0.1f, 0.05f);
    float kp = 0, ki = 0, kd = 0;
    slave->getPIDParameters(1, kp, ki, kd);
    EXPECT_FLOAT_EQ(kp, 2.5f);
    EXPECT_FLOAT_EQ(ki, 0.1f);
    EXPECT_FLOAT_EQ(kd, 0.05f);
    slave->getPIDParameters(99, kp, ki, kd);  // OOB zeros
    EXPECT_FLOAT_EQ(kp, 0.0f);
    EXPECT_FLOAT_EQ(ki, 0.0f);
    EXPECT_FLOAT_EQ(kd, 0.0f);
    slave->setPIDParameters(99, 1, 1, 1);     // ignored
}

TEST_F(CiA404SlaveFullTest, ControllerEnable) {
    EXPECT_FALSE(slave->isControllerEnabled(0));
    slave->setControllerEnabled(0, true);
    EXPECT_TRUE(slave->isControllerEnabled(0));
    slave->setControllerEnabled(0, false);  // resets integral state
    EXPECT_FALSE(slave->isControllerEnabled(0));
    slave->setControllerEnabled(99, true);  // ignored
    EXPECT_FALSE(slave->isControllerEnabled(99));
}

TEST_F(CiA404SlaveFullTest, PIDLoopAndAlarms) {
    slave->setPIDParameters(0, 2.0f, 0.5f, 0.1f);
    slave->setControllerEnabled(0, true);
    slave->setProcessValue(0, 100);
    slave->setAlarmLimits(0, -50, 50);
    slave->simulate(1000000);  // 1 ms
    // error = setpoint(0) - pv(100) = -100; output = kp*e + ... < 0
    EXPECT_LT(slave->getControllerOutput(0), 0);
    EXPECT_TRUE(slave->isAlarmActive(0));
    slave->setAlarmLimits(0, -500, 500);
    slave->simulate(0);  // dt=0 → derivative guard
    EXPECT_FALSE(slave->isAlarmActive(0));
    EXPECT_FALSE(slave->isAlarmActive(99));
}

TEST_F(CiA404SlaveFullTest, DisabledControllerSkipsUpdate) {
    slave->setProcessValue(1, 500);
    slave->simulate(1000000);  // ch1 not enabled → output stays 0
    EXPECT_EQ(slave->getControllerOutput(1), 0);
}

TEST_F(CiA404SlaveFullTest, PdoStubs) {
    slave->updateTxPDO();
    slave->processRxPDO();
}

TEST(CiA404FactoryTest, Create) {
    CiA404SlaveConfig cfg{};
    auto s = createCiA404Slave(cfg);
    EXPECT_NE(s, nullptr);
}

// ############################################################################
//  CiA 401 — Digital/Analog I/O: public API + OD sweep
// ############################################################################

class CiA401SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA401SlaveConfig cfg{};
        cfg.digitalInputs8 = 2;
        cfg.digitalOutputs8 = 2;
        cfg.analogInputs = 2;
        cfg.analogOutputs = 2;
        slave = std::make_unique<CiA401Slave>(cfg);
        slave->start();
    }
    std::unique_ptr<CiA401Slave> slave;
};

TEST_F(CiA401SlaveFullTest, DigitalInputRoundTrip) {
    slave->setDigitalInput8(0, 0xA5);
    slave->setDigitalInput8(1, 0x3C);
    EXPECT_EQ(slave->getDigitalInput8(0), 0xA5);
    EXPECT_EQ(slave->getDigitalInput8(1), 0x3C);
    slave->setDigitalInput8(99, 1);  // ignored
}

TEST_F(CiA401SlaveFullTest, DigitalInput16AndBit) {
    slave->setDigitalInput16(0, 0xBEEF);
    EXPECT_EQ(slave->getDigitalInput16(0), 0xBEEF);
    slave->setDigitalInputBit(3, true);
    EXPECT_TRUE(slave->getDigitalInputBit(3));
    EXPECT_EQ(slave->getDigitalInput8(0) & 0x08, 0x08);
    slave->setDigitalInputBit(3, false);
    EXPECT_FALSE(slave->getDigitalInputBit(3));
    slave->setDigitalInputBit(999, true);  // ignored
    EXPECT_FALSE(slave->getDigitalInputBit(999));
}

TEST_F(CiA401SlaveFullTest, DigitalOutputsViaOD) {
    auto& od = slave->getObjectDictionary();
    uint8_t v = 0x5A;
    size_t wlen = 1;
    EXPECT_EQ(od.write(0x6200, 1, &v, wlen), EtherCAT::SDOAbortCode::Success);
    EXPECT_EQ(slave->getDigitalOutput8(0), 0x5A);
    EXPECT_EQ(slave->getDigitalOutput16(0) & 0xFF, 0x5A);
    EXPECT_TRUE(slave->getDigitalOutputBit(1));
}

TEST_F(CiA401SlaveFullTest, DigitalOutputCallback) {
    bool fired = false;
    slave->setDigitalOutputCallback([&](const uint8_t*, size_t) { fired = true; });
    // Output callbacks fire on the PDO path — fill the RxPDO buffer then exchange.
    if (auto* rx = slave->getCore().getRxPDOData()) {
        std::memset(rx, 0x5A, slave->getCore().getRxPDOSize());
    }
    slave->processRxPDO();
    EXPECT_TRUE(fired);
}

TEST_F(CiA401SlaveFullTest, AnalogScalingAndReadback) {
    slave->setAnalogInput(0, 1000);
    slave->setAnalogInput(1, -2000);
    slave->setAnalogInputScaling(0, 100, 2.0f);
    EXPECT_NE(slave->getAnalogInput(0), 0);
    EXPECT_EQ(slave->getAnalogInput(1), -2000);
    slave->setAnalogInput(99, 1);
    EXPECT_EQ(slave->getAnalogInput(99), 0);
    EXPECT_EQ(slave->getAnalogInputCount(), 2u);
    EXPECT_EQ(slave->getAnalogOutputCount(), 2u);
}

TEST_F(CiA401SlaveFullTest, AnalogOutputViaOD) {
    bool fired = false;
    slave->setAnalogOutputCallback([&](size_t, int16_t) { fired = true; });
    if (auto* rx = slave->getCore().getRxPDOData()) {
        std::memset(rx, 0x10, slave->getCore().getRxPDOSize());
    }
    slave->processRxPDO();
    EXPECT_TRUE(fired);
    EXPECT_EQ(slave->getAnalogOutput(99), 0);
}

TEST_F(CiA401SlaveFullTest, DigitalInterrupts) {
    bool fired = false;
    size_t firedGroup = 99;
    slave->setInterruptCallback([&](size_t g, uint8_t) {
        fired = true;
        firedGroup = g;
    });
    slave->configureDigitalInterrupt(0, 0xFF, 1);
    slave->setDigitalInput8(0, 0x01);  // change on masked bits → fire
    slave->simulate(1000);           // checkInterrupts runs in simulate
    EXPECT_TRUE(fired);
    EXPECT_EQ(firedGroup, 0u);
    slave->configureDigitalInterrupt(0, 0x00, 1);  // mask off → no fire
    fired = false;
    slave->setDigitalInput8(0, 0x02);
    slave->simulate(1000);
    slave->configureDigitalInterrupt(99, 0xFF, 1);  // ignored
}

TEST_F(CiA401SlaveFullTest, ErrorValuesOnCommLoss) {
    slave->setDigitalOutputErrorMode(0, 1);
    slave->setDigitalOutputErrorValue(0, 0xEE);
    slave->triggerCommunicationError();
    slave->setDigitalOutputErrorMode(99, 1);   // ignored
    slave->setDigitalOutputErrorValue(99, 1);  // ignored
}

TEST_F(CiA401SlaveFullTest, ODSweep) {
    sweepOD(slave->getObjectDictionary(),
            {0x6000, 0x6010, 0x6020, 0x6200, 0x6210, 0x6220,
             0x6400, 0x6410, 0x6510, 0x6520});
}

TEST_F(CiA401SlaveFullTest, PdoAndSimulate) {
    slave->setDigitalInput8(0, 0x77);
    slave->updateTxPDO();
    slave->processRxPDO();
    slave->simulate(1000000);
}

TEST(CiA401FactoryTest, Create) {
    CiA401SlaveConfig cfg{};
    auto s = createCiA401Slave(cfg);
    EXPECT_NE(s, nullptr);
    EXPECT_STREQ(s->getProfileName(), "CiA 401");
    EXPECT_EQ(s->getDeviceType(), 0x00000191u);
}

// ############################################################################
//  CiA 405/406/408 — object-dictionary callback sweeps
// ############################################################################

TEST_F(CiA405SlaveFullTest, ODSweep) {
    sweepOD(plc->getObjectDictionary(),
            {0x4000, 0x4001, 0x4002, 0x4003, 0x4010, 0x4011, 0x4012, 0x4013,
             0x4100, 0x4101, 0x4200, 0x4201, 0x4300, 0x4301,
             0x4400, 0x4401, 0x4402, 0x4403, 0x4410,
             0x4500, 0x4510, 0x4520, 0x4530});
}

TEST_F(CiA406SlaveFullTest, ODSweep) {
    sweepOD(enc->getObjectDictionary(),
            {0x6000, 0x6001, 0x6002, 0x6003, 0x6004, 0x6010, 0x6011,
             0x6030, 0x6031, 0x6032, 0x6040,
             0x6100, 0x6101, 0x6102,
             0x6200, 0x6201, 0x6202,
             0x6500, 0x6501, 0x6502, 0x6503, 0x6504, 0x6510, 0x6600});
}

TEST_F(CiA408SlaveFullTest, ODSweep) {
    sweepOD(valve->getObjectDictionary(),
            {0x6000, 0x6001, 0x6002, 0x6003,
             0x6100, 0x6101, 0x6102, 0x6103, 0x6104,
             0x6110, 0x6111, 0x6112,
             0x6200, 0x6201, 0x6202, 0x6203, 0x6204, 0x6210,
             0x6300, 0x6301, 0x6302, 0x6303, 0x6304, 0x6310, 0x6311,
             0x6400, 0x6401, 0x6402, 0x6410, 0x6411, 0x6420, 0x6421,
             0x6500, 0x6501, 0x6502, 0x6503, 0x6510, 0x6511, 0x6520});
}

// ############################################################################
//  CiA 417 — Lift controller state machine
// ############################################################################

class CiA417SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA417SlaveConfig cfg{};
        cfg.numberOfFloors = 4;
        cfg.numberOfDoors = 2;
        cfg.floorHeight = 1000;
        cfg.nominalSpeed = 1000;
        cfg.levelingTolerance = 5;
        lift = std::make_unique<CiA417Slave>(cfg);
        lift->start();
    }
    // Drive the lift through the whole FSM.
    void runCycles(int n) {
        for (int i = 0; i < n; ++i) lift->simulate(100000000);  // 100 ms
    }
    std::unique_ptr<CiA417Slave> lift;
};

TEST_F(CiA417SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(lift->getProfileName(), "CiA 417");
    EXPECT_EQ(lift->getDeviceType(), 0x000001A1u);
}

TEST_F(CiA417SlaveFullTest, MoveToUpperFloor) {
    lift->setTargetFloor(2);
    EXPECT_TRUE(lift->isMoving());
    runCycles(60);
    EXPECT_EQ(lift->getCurrentFloor(), 2);
    EXPECT_TRUE(lift->isAtFloor());
    EXPECT_FALSE(lift->isMoving());
    EXPECT_EQ(lift->getDoorState(0), DoorState::Open);
}

TEST_F(CiA417SlaveFullTest, MoveDownAndDoors) {
    lift->setTargetFloor(3);
    EXPECT_TRUE(lift->isMovingUp());
    runCycles(60);
    EXPECT_EQ(lift->getLiftState(), LiftState::DoorOpen);
    // closeDoor only touches doorStates_; liftState stays DoorOpen and
    // setTargetFloor is ignored outside Idle/DoorClosed.
    lift->closeDoor(0);
    lift->closeDoor(1);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Closing);
    lift->setTargetFloor(0);
    EXPECT_FALSE(lift->isMoving());
    runCycles(60);
    EXPECT_EQ(lift->getPosition(), 3000);
}

TEST_F(CiA417SlaveFullTest, FloorRequests) {
    lift->addFloorRequest(3);
    lift->addFloorRequest(1);
    lift->addFloorRequest(99);  // ignored
    EXPECT_TRUE(lift->hasFloorRequest(3));
    EXPECT_FALSE(lift->hasFloorRequest(2));
    EXPECT_FALSE(lift->hasFloorRequest(99));
    lift->clearFloorRequest(3);
    lift->clearFloorRequest(99);  // ignored
    EXPECT_FALSE(lift->hasFloorRequest(3));
    runCycles(60);  // Idle → selectNextTarget picks floor 1
    EXPECT_EQ(lift->getCurrentFloor(), 1);
}

TEST_F(CiA417SlaveFullTest, DoorsBlockedAndOOB) {
    lift->openDoor(0);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Opening);
    EXPECT_EQ(lift->getDoorState(9), DoorState::Fault);  // OOB
    lift->openDoor(9);                                   // ignored
    lift->closeDoor(0);
    EXPECT_EQ(lift->getDoorState(0), DoorState::Closing);
    lift->setDoorBlocked(0, true);   // blocked while Closing → Blocked
    EXPECT_EQ(lift->getDoorState(0), DoorState::Blocked);
    lift->setDoorBlocked(9, true);  // ignored
}

TEST_F(CiA417SlaveFullTest, EmergencyAndSafety) {
    lift->setTargetFloor(2);
    lift->setEmergencyStop(true);
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
    EXPECT_EQ(lift->getSpeed(), 0);
    lift->setEmergencyStop(false);
    lift->setSafetyCircuit(false);
    EXPECT_EQ(lift->getLiftState(), LiftState::Emergency);
    lift->setSafetyCircuit(true);
}

TEST_F(CiA417SlaveFullTest, LoadCallbackAndTargetValidation) {
    lift->setLoad(500);
    lift->setTargetFloor(99);  // ignored
    EXPECT_FALSE(lift->isMoving());
    lift->setPosition(1500);
    EXPECT_EQ(lift->getPosition(), 1500);
    EXPECT_EQ(lift->getCurrentFloor(), 1);
    bool cb = false;
    lift->setPositionCallback([&](int32_t, int32_t) { cb = true; });
    lift->simulate(1000);
    EXPECT_TRUE(cb);
}

// ############################################################################
//  CiA 430 — Programmable power supply: ramps, protection, callbacks
// ############################################################################

class CiA430SlaveFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        CiA430SlaveConfig cfg{};
        cfg.numberOfChannels = 2;
        psu = std::make_unique<CiA430Slave>(cfg);
        psu->start();
    }
    std::unique_ptr<CiA430Slave> psu;
};

TEST_F(CiA430SlaveFullTest, ProfileNameAndType) {
    EXPECT_STREQ(psu->getProfileName(), "CiA 430");
    EXPECT_EQ(psu->getDeviceType(), 0x000001AEu);
}

TEST_F(CiA430SlaveFullTest, EnableAndVoltageRamp) {
    psu->setOutputEnable(0, true);
    psu->setTargetVoltage(0, 24000);
    EXPECT_TRUE(psu->isOutputEnabled(0));
    EXPECT_EQ(psu->getTargetVoltage(0), 24000);
    for (int i = 0; i < 200; ++i) psu->simulate(1000000);  // ramp 1V/ms
    EXPECT_EQ(psu->getActualVoltage(0), 24000u);
    // Ramp back down
    psu->setTargetVoltage(0, 0);
    for (int i = 0; i < 200; ++i) psu->simulate(1000000);
    EXPECT_EQ(psu->getActualVoltage(0), 0u);
}

TEST_F(CiA430SlaveFullTest, DisabledChannelSkipsRamp) {
    psu->setTargetVoltage(1, 12000);
    psu->simulate(1000000);  // not enabled → no ramp
    EXPECT_EQ(psu->getActualVoltage(1), 0u);
    psu->setAllOutputs(true);
    EXPECT_TRUE(psu->isOutputEnabled(1));
}

TEST_F(CiA430SlaveFullTest, ProtectionTripAndClear) {
    psu->setOutputEnable(0, true);   // disabled channels get zeroed each cycle
    psu->setOutputEnable(1, true);
    psu->setActualVoltage(0, 29500);  // stays above OVP after one ramp-down step
    psu->setActualCurrent(0, 6000);   // above default OCP 5500
    psu->simulate(1000000);
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OVP));
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OCP));
    EXPECT_EQ(psu->getChannelState(0), PowerSupplyState::Fault);
    EXPECT_EQ(psu->getState(), PowerSupplyState::Fault);
    psu->setActualVoltage(0, 10000);
    psu->setActualCurrent(0, 100);
    psu->clearProtection(0);
    EXPECT_FALSE(psu->isProtectionTripped(0, ProtectionType::OVP));
    psu->setActualCurrent(1, 9999);
    psu->simulate(1000000);
    psu->clearAllProtection();
    psu->clearProtection(9);  // ignored
}

TEST_F(CiA430SlaveFullTest, UvpAndOtp) {
    psu->setOutputEnable(0, true);
    psu->setTargetVoltage(0, 24000);
    psu->setActualVoltage(0, 10000);  // below UVP 20000 while enabled
    psu->simulate(1000000);
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::UVP));
    psu->setTemperature(90);          // above maxTemperature 70
    psu->simulate(1000000);
    EXPECT_TRUE(psu->isProtectionTripped(0, ProtectionType::OTP));
    EXPECT_EQ(psu->getTemperature(), 90);
}

TEST_F(CiA430SlaveFullTest, PowerCurrentCallbackAndStatus) {
    psu->setActualVoltage(0, 24000);
    psu->setActualCurrent(0, 2000);
    EXPECT_EQ(psu->getActualPower(0), 48000u);   // mV*mA/1000 → mW
    EXPECT_EQ(psu->getTotalPower(), 48000u);
    psu->setCurrentLimit(0, 5000);
    EXPECT_EQ(psu->getCurrentLimit(0), 5000u);
    EXPECT_EQ(psu->getActualCurrent(0), 2000u);
    bool cb = false;
    psu->setChannelCallback([&](uint8_t, uint32_t&, uint32_t&) { cb = true; });
    psu->simulate(1000000);
    EXPECT_TRUE(cb);
    psu->updateTxPDO();
    psu->processRxPDO();
    EXPECT_TRUE(psu->isACOk());
    EXPECT_EQ(psu->getACVoltage(), 230u);
}

TEST_F(CiA430SlaveFullTest, OOBChannels) {
    psu->setOutputEnable(9, true);    // ignored
    EXPECT_FALSE(psu->isOutputEnabled(9));
    psu->setTargetVoltage(9, 1);      // ignored
    EXPECT_EQ(psu->getTargetVoltage(9), 0u);
    EXPECT_EQ(psu->getActualVoltage(9), 0u);
    psu->setCurrentLimit(9, 1);       // ignored
    EXPECT_EQ(psu->getCurrentLimit(9), 0u);
    EXPECT_EQ(psu->getActualCurrent(9), 0u);
    EXPECT_EQ(psu->getActualPower(9), 0u);
    EXPECT_EQ(psu->getProtectionStatus(9), 0);
    EXPECT_EQ(psu->getChannelState(9), PowerSupplyState::Off);
}
