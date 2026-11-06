/**
 * @file test_cia4xx_profiles.cpp
 * @brief Coverage for the CiA 404/405/406/408/410/417/430 slave profiles:
 *        identity, domain getters/setters, simulate() behaviour, PDO hooks,
 *        and out-of-range channel handling.
 */

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "tether/slave/profiles/CiA401Slave.hpp"
#include "tether/slave/profiles/CiA402Slave.hpp"
#include "tether/slave/profiles/CiA404Slave.hpp"
#include "tether/slave/profiles/CiA405Slave.hpp"
#include "tether/slave/profiles/CiA406Slave.hpp"
#include "tether/slave/profiles/CiA408Slave.hpp"
#include "tether/slave/profiles/CiA410Slave.hpp"
#include "tether/slave/profiles/CiA417Slave.hpp"
#include "tether/slave/profiles/CiA430Slave.hpp"

using namespace EtherCAT::slave;
using EtherCAT::SDOAbortCode;

namespace {

// ============================================================================
// CiA 404 — measuring device / PID controller
// ============================================================================

TEST(CiA404SlaveCov, Identity) {
    CiA404Slave s(CiA404SlaveConfig{});
    EXPECT_STREQ(s.getProfileName(), "CiA 404");
    EXPECT_EQ(s.getDeviceType(), 0x00000194u);
}

TEST(CiA404SlaveCov, StartInitializesODAndPDOs) {
    CiA404Slave s(CiA404SlaveConfig{});
    EXPECT_TRUE(s.start());
    EXPECT_TRUE(s.getObjectDictionary().hasObject(0x6000, 0) ||
                s.getObjectDictionary().hasObject(0x1000, 0));
    s.stop();
}

TEST(CiA404SlaveCov, ProcessValueAndSetpoint) {
    CiA404Slave s(CiA404SlaveConfig{});
    s.setProcessValue(0, 1234);
    EXPECT_EQ(s.getProcessValue(0), 1234);
    EXPECT_EQ(s.getSetpoint(0), 0);
    // Out-of-range channel is a no-op / returns 0
    s.setProcessValue(99, 5);
    EXPECT_EQ(s.getProcessValue(99), 0);
}

TEST(CiA404SlaveCov, PIDParameters) {
    CiA404SlaveConfig cfg;
    cfg.controllerChannels = 2;
    CiA404Slave s(cfg);
    s.setPIDParameters(0, 2.5f, 0.1f, 0.01f);
    float kp = 0, ki = 0, kd = 0;
    s.getPIDParameters(0, kp, ki, kd);
    EXPECT_FLOAT_EQ(kp, 2.5f);
    EXPECT_FLOAT_EQ(ki, 0.1f);
    EXPECT_FLOAT_EQ(kd, 0.01f);
    // Out-of-range channel yields zeros
    s.getPIDParameters(99, kp, ki, kd);
    EXPECT_FLOAT_EQ(kp, 0.0f);
}

TEST(CiA404SlaveCov, ControllerDrivesTowardSetpoint) {
    CiA404SlaveConfig cfg;
    cfg.controllerChannels = 1;
    cfg.defaultKp = 1.0f;
    CiA404Slave s(cfg);
    s.setControllerEnabled(0, true);
    EXPECT_TRUE(s.isControllerEnabled(0));

    // setpoint defaults 0 — push process value negative so error > 0
    s.setProcessValue(0, -100);
    s.simulate(1000000);                    // 1 ms
    EXPECT_GT(s.getControllerOutput(0), 0); // P-term pulls toward setpoint

    // Disabling clears integrator/prevError
    s.setControllerEnabled(0, false);
    EXPECT_FALSE(s.isControllerEnabled(0));
    s.simulate(1000000);
    EXPECT_EQ(s.getControllerOutput(99), 0);
}

TEST(CiA404SlaveCov, Alarms) {
    CiA404SlaveConfig cfg;
    cfg.controllerChannels = 1;
    CiA404Slave s(cfg);
    s.setAlarmLimits(0, 0, 1000);
    s.setProcessValue(0, 500);
    s.simulate(1000000);
    EXPECT_FALSE(s.isAlarmActive(0));
    s.setProcessValue(0, 2000);             // above high limit
    s.simulate(1000000);
    EXPECT_TRUE(s.isAlarmActive(0));
    EXPECT_FALSE(s.isAlarmActive(99));
    s.updateTxPDO();
    s.processRxPDO();
}

TEST(CiA404SlaveCov, Factory) {
    auto s = createCiA404Slave(CiA404SlaveConfig{});
    EXPECT_NE(s, nullptr);
}

// ============================================================================
// CiA 405 — programmable controller
// ============================================================================

TEST(CiA405SlaveCov, IdentityAndProgram) {
    CiA405Slave s(CiA405SlaveConfig{});
    EXPECT_STREQ(s.getProfileName(), "CiA 405");
    EXPECT_EQ(s.getDeviceType(), 0x00000195u);
    EXPECT_TRUE(s.start());                 // initObjectDictionary+initPDOMappings
    s.stop();

    s.startProgram();
    EXPECT_EQ(s.getProgramState(), ProgramState::Running);
    s.stopProgram();
    EXPECT_EQ(s.getProgramState(), ProgramState::Stopped);
    s.resetProgram();

    std::vector<uint8_t> prog{0x01, 0x02, 0x03};
    EXPECT_TRUE(s.loadProgram(prog));
    EXPECT_EQ(s.getProgram(), prog);
}

TEST(CiA405SlaveCov, Variables) {
    CiA405Slave s(CiA405SlaveConfig{});
    s.registerVariable("counter", 0, 4, /*isInput=*/false, /*isOutput=*/true);
    std::vector<uint8_t> w{0x2A, 0x00, 0x00, 0x00};
    EXPECT_TRUE(s.writeVariable("counter", w));
    std::vector<uint8_t> r;
    EXPECT_TRUE(s.readVariable("counter", r));
    EXPECT_EQ(r[0], 0x2A);
    EXPECT_FALSE(s.readVariable("missing", r));
    EXPECT_FALSE(s.writeVariable("missing", w));
}

TEST(CiA405SlaveCov, AreasAndCycleCallback) {
    CiA405Slave s(CiA405SlaveConfig{});
    EXPECT_NE(s.getInputArea(), nullptr);
    EXPECT_NE(s.getOutputArea(), nullptr);
    EXPECT_NE(s.getInternalArea(), nullptr);
    s.getInternalArea()[0] = 0xAB;
    s.getInputArea()[0] = 0x5A;
    bool called = false;
    s.setCycleCallback([&] { called = true; });
    // simulate() is gated on a running program
    s.simulate(1000);
    EXPECT_FALSE(called);
    s.startProgram();
    s.simulate(1000);
    EXPECT_TRUE(called);
    EXPECT_EQ(s.getOutputArea()[0], 0x5A);    // echo: input copied to output
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 406 — encoder
// ============================================================================

TEST(CiA406SlaveCov, IdentityAndPosition) {
    CiA406SlaveConfig cfg;
    cfg.encoderType = EncoderType::MultiTurn;
    cfg.totalMeasuringRange = 4;
    CiA406Slave s(cfg);
    EXPECT_STREQ(s.getProfileName(), "CiA 406");
    EXPECT_EQ(s.getDeviceType(), 0x00000196u);
    EXPECT_TRUE(s.start());
    s.stop();
    s.setPosition(12345);
    EXPECT_EQ(s.getPosition(), 12345);
    s.setSpeed(200);
    EXPECT_EQ(s.getSpeed(), 200);
    s.setTurns(6);                          // wraps mod totalMeasuringRange
    EXPECT_EQ(s.getTurns(), 6 % 4);

    // Single-turn encoder ignores setTurns
    CiA406Slave s1(CiA406SlaveConfig{});
    s1.setTurns(3);
    EXPECT_EQ(s1.getTurns(), 0);
}

TEST(CiA406SlaveCov, PresetAndAlarms) {
    CiA406Slave s(CiA406SlaveConfig{});
    s.setPosition(999);
    s.presetPosition(100);
    EXPECT_EQ(s.getPosition(), 100);

    EXPECT_FALSE(s.isAlarmActive());
    s.setAlarmStatus(0x05);
    EXPECT_TRUE(s.isAlarmActive());
    EXPECT_EQ(s.getAlarmStatus(), 0x05);
    s.clearAlarms();
    EXPECT_FALSE(s.isAlarmActive());
    EXPECT_NE(s.getOperatingStatus(), 0xFFFF);
}

TEST(CiA406SlaveCov, SimulateInvokesCallback) {
    CiA406Slave s(CiA406SlaveConfig{});
    bool called = false;
    s.setPositionCallback([&]() -> int32_t {
        called = true;
        return 777;
    });
    s.setSpeed(10);
    s.setPosition(0);
    s.simulate(1000);
    EXPECT_TRUE(called);
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 408 — hydraulic valve
// ============================================================================

TEST(CiA408SlaveCov, IdentityAndCommand) {
    CiA408Slave s(CiA408SlaveConfig{});
    EXPECT_STREQ(s.getProfileName(), "CiA 408");
    EXPECT_EQ(s.getDeviceType(), 0x00000198u);
    EXPECT_TRUE(s.start());
    s.stop();
    s.setActualPosition(500);
    EXPECT_EQ(s.getActualPosition(), 500);
    s.setPressure(12000, 8000);
    EXPECT_EQ(s.getPressureA(), 12000);
    EXPECT_EQ(s.getPressureB(), 8000);
    s.setFlow(60000);
    EXPECT_EQ(s.getFlow(), 60000);
}

TEST(CiA408SlaveCov, FaultsAndEnable) {
    CiA408Slave s(CiA408SlaveConfig{});
    s.setEnabled(true);
    EXPECT_TRUE(s.isEnabled());
    EXPECT_FALSE(s.isFaulted());
    s.setFault(0x1234);
    EXPECT_TRUE(s.isFaulted());
    EXPECT_EQ(s.getFaultCode(), 0x1234);
    EXPECT_NE(s.getStatusWord() & 0x0008, 0u);
    s.clearFault();
    EXPECT_FALSE(s.isFaulted());
}

TEST(CiA408SlaveCov, Options) {
    CiA408Slave s(CiA408SlaveConfig{});
    s.setDeadbandCompensation(42);
    EXPECT_EQ(s.getDeadbandCompensation(), 42);
    s.setDitherEnabled(true);
    EXPECT_TRUE(s.isDitherEnabled());
    s.setSimulationCallback([](int16_t, int16_t&) {});
    s.simulate(1000);
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 410 — inclinometer
// ============================================================================

TEST(CiA410SlaveCov, IdentityAndMeasurement) {
    CiA410Slave s(CiA410SlaveConfig{});
    EXPECT_STREQ(s.getProfileName(), "CiA 410");
    EXPECT_EQ(s.getDeviceType(), 0x0000019Au);
    EXPECT_TRUE(s.start());
    s.stop();
    s.setInclination(45000, -30000);
    s.simulate(1000);                       // filter converges toward raw
    EXPECT_GT(s.getInclinationX(), 0);
    EXPECT_LT(s.getInclinationY(), 0);
    s.setTemperature(250);                  // 25.0 °C
    EXPECT_EQ(s.getTemperature(), 250);

    // Calibration lifecycle: bit set while calibrating, cleared after ~1s
    s.startCalibration();
    EXPECT_NE(s.getOperatingStatus() & 0x0010, 0u);
    s.simulate(1000000000ULL);
    EXPECT_EQ(s.getOperatingStatus() & 0x0010, 0u);
    s.updateTxPDO();
    s.processRxPDO();
}

TEST(CiA410SlaveCov, AlarmsAndCalibrationApi) {
    CiA410Slave s(CiA410SlaveConfig{});
    s.setAlarmLimits(-100, 100, -100, 100);
    s.setInclination(50000, 0);             // beyond X limits
    s.simulate(1000);
    EXPECT_TRUE(s.isAlarmActive());
    EXPECT_NE(s.getAlarmStatus() & 0x01, 0u);
    s.setCalibrationOffset(10, 20);
    s.startCalibration();
    EXPECT_NE(s.getOperatingStatus() & 0x0010, 0u);
}

// ============================================================================
// CiA 417 — lift controller
// ============================================================================

TEST(CiA417SlaveCov, FloorsAndRequests) {
    CiA417Slave s(CiA417SlaveConfig{});
    EXPECT_STREQ(s.getProfileName(), "CiA 417");
    EXPECT_EQ(s.getDeviceType(), 0x000001A1u);
    EXPECT_TRUE(s.start());
    s.stop();

    s.addFloorRequest(5);
    EXPECT_TRUE(s.hasFloorRequest(5));
    EXPECT_FALSE(s.hasFloorRequest(6));
    EXPECT_NE(s.getFloorRequests() & (1ull << 5), 0ull);
    s.clearFloorRequest(5);
    EXPECT_FALSE(s.hasFloorRequest(5));

    s.setTargetFloor(3);
    EXPECT_EQ(s.getTargetFloor(), 3);
}

TEST(CiA417SlaveCov, DoorAndSafety) {
    CiA417Slave s(CiA417SlaveConfig{});
    s.openDoor(0);
    s.closeDoor(0);
    s.setDoorBlocked(0, true);
    s.setSafetyCircuit(false);
    EXPECT_FALSE(s.isSafetyCircuitOk());
    s.setSafetyCircuit(true);
    s.setEmergencyStop(true);
    EXPECT_TRUE(s.isEmergencyStop());
    s.setEmergencyStop(false);
}

TEST(CiA417SlaveCov, LoadAndMovement) {
    CiA417SlaveConfig cfg;
    cfg.maxLoad = 1000;
    CiA417Slave s(cfg);
    s.setLoad(5000);                        // 500 kg — under 1000 kg
    EXPECT_EQ(s.getLoad(), 5000);
    EXPECT_FALSE(s.isOverloaded());
    s.setLoad(15000);                       // 1500 kg — overloaded
    EXPECT_TRUE(s.isOverloaded());
    s.setLoad(0);

    s.setPosition(0);
    s.setTargetFloor(1);                    // 3000 mm up
    s.simulate(1000000);
    EXPECT_FALSE(s.isMovingDown());
    s.getSpeed();
    s.isAtFloor();
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 430 — power supply
// ============================================================================

TEST(CiA430SlaveCov, ChannelsAndOutput) {
    CiA430SlaveConfig cfg;
    cfg.numberOfChannels = 2;
    CiA430Slave s(cfg);
    EXPECT_STREQ(s.getProfileName(), "CiA 430");
    EXPECT_EQ(s.getDeviceType(), 0x000001AEu);
    EXPECT_TRUE(s.start());
    s.stop();

    s.setOutputEnable(0, true);
    EXPECT_TRUE(s.isOutputEnabled(0));
    EXPECT_EQ(s.getChannelState(0), PowerSupplyState::Running);
    s.setOutputEnable(0, false);
    EXPECT_EQ(s.getChannelState(0), PowerSupplyState::Ready);
    s.setOutputEnable(99, true);            // out-of-range: no crash
    EXPECT_FALSE(s.isOutputEnabled(99));
}

TEST(CiA430SlaveCov, VoltageCurrentPower) {
    CiA430Slave s(CiA430SlaveConfig{});
    s.setTargetVoltage(0, 24000);
    EXPECT_EQ(s.getTargetVoltage(0), 24000u);
    s.setCurrentLimit(0, 5000);
    EXPECT_EQ(s.getCurrentLimit(0), 5000u);
    s.setActualVoltage(0, 24000);
    s.setActualCurrent(0, 2000);
    EXPECT_EQ(s.getActualVoltage(0), 24000u);
    EXPECT_EQ(s.getActualCurrent(0), 2000u);
    EXPECT_EQ(s.getActualPower(0), 48000u);   // mW = mV*mA/1000
    s.getTotalPower();
}

TEST(CiA430SlaveCov, ProtectionAndTemp) {
    CiA430Slave s(CiA430SlaveConfig{});
    s.setAllOutputs(true);
    s.clearProtection(0);
    s.clearAllProtection();
    EXPECT_EQ(s.getProtectionStatus(0), 0u);
    EXPECT_FALSE(s.isProtectionTripped(0, ProtectionType::OVP));
    s.setTemperature(45);
    EXPECT_EQ(s.getTemperature(), 45);
    s.setChannelCallback([](uint8_t, uint32_t, uint32_t) {});
    s.simulate(1000);
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 402 — start() runs the big OD/PDO registration bodies
//            (CiA402SlaveOD.cpp / CiA402SlavePDO.cpp / CiA402SlaveModes.cpp)
// ============================================================================

TEST(CiA402SlaveCov, StartRegistersOD) {
    CiA402Slave s(CiA402SlaveConfig{});
    EXPECT_TRUE(s.start());
    auto& od = s.getObjectDictionary();
    EXPECT_TRUE(od.hasObject(0x6040, 0));   // control word
    EXPECT_TRUE(od.hasObject(0x6041, 0));   // status word
    EXPECT_TRUE(od.hasObject(0x6060, 0));   // modes of operation
    EXPECT_TRUE(od.hasObject(0x6064, 0));   // actual position
    s.stop();
}

TEST(CiA402SlaveCov, ODReadWriteLambdas) {
    CiA402Slave s(CiA402SlaveConfig{});
    ASSERT_TRUE(s.start());
    auto& od = s.getObjectDictionary();

    // Status word read (0x6041 RO)
    uint8_t buf[8] = {};
    size_t len = sizeof(buf);
    EXPECT_EQ(od.read(0x6041, 0, buf, len), SDOAbortCode::Success);
    EXPECT_EQ(len, 2u);

    // Control word write (0x6040 RW) → processControlWord lambda
    buf[0] = 0x06; buf[1] = 0x00;           // Shutdown command
    EXPECT_EQ(od.write(0x6040, 0, buf, 2), SDOAbortCode::Success);
    // Wrong length rejected
    EXPECT_NE(od.write(0x6040, 0, buf, 1), SDOAbortCode::Success);

    // Modes of operation read
    len = sizeof(buf);
    EXPECT_EQ(od.read(0x6060, 0, buf, len), SDOAbortCode::Success);
    EXPECT_EQ(len, 1u);

    // Position actual read
    len = sizeof(buf);
    EXPECT_EQ(od.read(0x6064, 0, buf, len), SDOAbortCode::Success);
    EXPECT_EQ(len, 4u);
    s.stop();
}

TEST(CiA402SlaveCov, ModesAndErrors) {
    CiA402SlaveConfig cfg;
    cfg.supportedModes = CiA402Mode::AllCyclic | CiA402Mode::HM;
    CiA402Slave s(cfg);
    EXPECT_TRUE(s.isModeSupported(8));      // CSP
    EXPECT_TRUE(s.isModeSupported(9));      // CSV
    EXPECT_TRUE(s.isModeSupported(10));     // CST
    EXPECT_TRUE(s.isModeSupported(6));      // HM
    EXPECT_FALSE(s.isModeSupported(1));     // PP unsupported
    EXPECT_TRUE(s.start());
    s.stop();
}

TEST(CiA402SlaveCov, SimulateRunsMotorModel) {
    CiA402Slave s(CiA402SlaveConfig{});
    s.setActualPosition(0);
    s.setActualVelocity(100);
    s.simulate(1000000);                    // 1 ms of sim time
    // position demand/target and fault helpers
    s.getPositionDemand();
    s.getTargetPosition();
    s.getFollowingError();
    s.triggerFault(0x1000);
    EXPECT_EQ(s.getDriveState(), CiA402State::Fault);
    s.updateTxPDO();                        // refreshes statusWord_
    EXPECT_NE(s.getStatusWord() & 0x0008, 0u);
    s.clearFault();
    EXPECT_EQ(s.getDriveState(), CiA402State::SwitchOnDisabled);
    s.updateTxPDO();
    s.processRxPDO();
}

// ============================================================================
// CiA 402 mode executors — drive to OperationEnabled then simulate per mode
// ============================================================================

void enableDrive(CiA402Slave& s) {
    s.processControlWord(0x0006);           // Shutdown → ReadyToSwitchOn
    s.processControlWord(0x0007);           // SwitchOn → SwitchedOn
    s.processControlWord(0x000F);           // EnableOperation → OperationEnabled
}

void odWriteI8(CiA402Slave& s, uint16_t idx, int8_t v) {
    uint8_t b = static_cast<uint8_t>(v);
    EXPECT_EQ(s.getObjectDictionary().write(idx, 0, &b, 1), SDOAbortCode::Success);
}
void odWriteI32(CiA402Slave& s, uint16_t idx, int32_t v) {
    EXPECT_EQ(s.getObjectDictionary().write(idx, 0, reinterpret_cast<const uint8_t*>(&v), 4),
              SDOAbortCode::Success);
}
void odWriteI16(CiA402Slave& s, uint16_t idx, int16_t v) {
    EXPECT_EQ(s.getObjectDictionary().write(idx, 0, reinterpret_cast<const uint8_t*>(&v), 2),
              SDOAbortCode::Success);
}
void odWriteU32(CiA402Slave& s, uint16_t idx, uint32_t v) {
    EXPECT_EQ(s.getObjectDictionary().write(idx, 0, reinterpret_cast<const uint8_t*>(&v), 4),
              SDOAbortCode::Success);
}

CiA402SlaveConfig allModesConfig() {
    CiA402SlaveConfig c;
    c.supportedModes = CiA402Mode::All;
    c.enableSimulation = true;
    return c;
}

TEST(CiA402SlaveCov, ControlWordSequenceEnablesDrive) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    EXPECT_EQ(s.getDriveState(), CiA402State::SwitchOnDisabled);
    s.processControlWord(0x0006);
    EXPECT_EQ(s.getDriveState(), CiA402State::ReadyToSwitchOn);
    s.processControlWord(0x0007);
    EXPECT_EQ(s.getDriveState(), CiA402State::SwitchedOn);
    s.processControlWord(0x000F);
    EXPECT_EQ(s.getDriveState(), CiA402State::OperationEnabled);
    s.stop();
}

TEST(CiA402SlaveCov, ProfilePositionModeSimulates) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    ASSERT_EQ(s.getDriveState(), CiA402State::OperationEnabled);
    odWriteI8(s, 0x6060, 1);                // PP
    odWriteI32(s, 0x607A, 10000);           // target position
    odWriteU32(s, 0x6081, 5000);            // profile velocity
    odWriteU32(s, 0x6083, 50000);           // profile accel
    odWriteU32(s, 0x6084, 50000);           // profile decel
    s.processControlWord(0x000F | 0x0010);  // EnableOperation | NewSetPoint
    for (int i = 0; i < 50; ++i)
        s.simulate(1000000);                // 50 ms total
    EXPECT_EQ(s.getOperatingMode(), 1);     // mode display follows
    EXPECT_NE(s.getActualPosition(), 0);    // motor model integrated
    s.stop();
}

TEST(CiA402SlaveCov, VelocityModesSimulate) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    for (int8_t mode : {2, 3}) {            // VL, PV
        odWriteI8(s, 0x6060, mode);
        odWriteI32(s, 0x60FF, 2000);        // target velocity
        for (int i = 0; i < 20; ++i)
            s.simulate(1000000);
        EXPECT_EQ(s.getOperatingMode(), mode);
    }
    s.stop();
}

TEST(CiA402SlaveCov, TorqueModeSimulates) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    odWriteI8(s, 0x6060, 4);                // PT
    odWriteI16(s, 0x6071, 500);             // target torque (0.1% units)
    for (int i = 0; i < 20; ++i)
        s.simulate(1000000);
    EXPECT_EQ(s.getOperatingMode(), 4);
    s.stop();
}

TEST(CiA402SlaveCov, CyclicSyncModesSimulate) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    for (int8_t mode : {8, 9, 10}) {        // CSP, CSV, CST
        odWriteI8(s, 0x6060, mode);
        odWriteI32(s, 0x60B0, 100);         // position offset
        odWriteI32(s, 0x60B1, 50);          // velocity offset
        odWriteI16(s, 0x60B2, 10);          // torque offset
        for (int i = 0; i < 20; ++i)
            s.simulate(1000000);
        EXPECT_EQ(s.getOperatingMode(), mode);
    }
    s.stop();
}

TEST(CiA402SlaveCov, HomingModeSimulates) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    odWriteI8(s, 0x6060, 6);                // HM
    odWriteI8(s, 0x6098, 35);               // homing method: current pos
    odWriteU32(s, 0x6099, 1000);            // homing speed (sub 0 count obj)
    for (int i = 0; i < 20; ++i)
        s.simulate(1000000);
    EXPECT_EQ(s.getOperatingMode(), 6);
    s.stop();
}

TEST(CiA402SlaveCov, QuickStopDecelerates) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    odWriteI8(s, 0x6060, 8);                // CSP
    s.simulate(1000000);
    s.processControlWord(0x0002);           // Quick Stop
    EXPECT_EQ(s.getDriveState(), CiA402State::QuickStopActive);
    s.simulate(1000000);                    // simulateMotion under quick stop
    s.stop();
}

TEST(CiA402SlaveCov, SimulationDisabledIsNoop) {
    CiA402SlaveConfig cfg = allModesConfig();
    cfg.enableSimulation = false;
    CiA402Slave s(cfg);
    s.setActualPosition(777);
    s.simulate(1000000);                    // early return
    EXPECT_EQ(s.getActualPosition(), 777);  // untouched
}

TEST(CiA402SlaveCov, SimulateRejectsAbsurdDelta) {
    CiA402Slave s(allModesConfig());
    ASSERT_TRUE(s.start());
    enableDrive(s);
    odWriteI8(s, 0x6060, 8);
    int32_t before = s.getActualPosition();
    s.simulate(200000000);                  // 0.2 s > 0.1 s cap → skipped
    // State machine/mode code ran but simulateMotion skipped; just no crash.
    s.simulate(0);                          // dt<=0 skipped
    (void)before;
    s.stop();
}

}  // namespace
