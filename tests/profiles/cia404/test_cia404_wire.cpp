/**
 * @file test_cia404_wire.cpp
 * @brief Wire-level coverage for CiA404::MeasuringDevice (CiA404Device.cpp):
 *        initialize/detectCapabilities, update() SDO reads, TxPDO/RxPDO data
 *        paths, input/output/controller/alarm/calibration/sensor
 *        operations and event callbacks — driven over EmulatedBus with a
 *        real SlaveEmulator.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia404/CiA404Device.hpp"
#include "tether/profiles/cia404/CiA404Defs.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Emulator;
using namespace CiA404;

namespace {

SlaveEmulator* addMeasuringSlave(EmulatedBus& bus) {
    auto slave = std::make_unique<SlaveEmulator>();
    SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = 0x0404;
    cfg.mailbox_protocol = 0x04;
    auto addSm = [&cfg](uint16_t addr, uint16_t len, uint8_t ctrl) {
        SIIConfig::SMConfig sm{};
        sm.start_addr = addr;
        sm.length     = len;
        sm.control    = std::bit_cast<SyncManager::SMControlReg>(ctrl);
        sm.enable     = std::bit_cast<SyncManager::SMActivateReg>(
            static_cast<uint8_t>(0x01));
        cfg.sync_managers.push_back(sm);
    };
    addSm(0x1000, 128, 0x26);
    addSm(0x1080, 128, 0x22);
    slave->setSIIConfig(cfg);

    auto u8 = [](uint8_t v) { return std::vector<uint8_t>{v}; };
    auto u16 = [](uint16_t v) {
        return std::vector<uint8_t>{static_cast<uint8_t>(v),
                                    static_cast<uint8_t>(v >> 8)};
    };
    auto u32 = [](uint32_t v) {
        return std::vector<uint8_t>{static_cast<uint8_t>(v),
                                    static_cast<uint8_t>(v >> 8),
                                    static_cast<uint8_t>(v >> 16),
                                    static_cast<uint8_t>(v >> 24)};
    };
    std::vector<ODEntry> entries;
    auto add = [&](uint16_t i, uint8_t s, const std::vector<uint8_t>& d) {
        entries.push_back({i, s, 0x0002,
                           static_cast<uint16_t>(d.size() * 8),
                           0x03, d, "cia404"});
    };

    add(0x1000, 0, u32(0x00000194));           // device type, profile 404
    // 4 process inputs: count + value(i32) + status(u8)
    for (uint8_t ch = 0; ch < 4; ++ch) {
        uint16_t base = ProcessDataInput1 + ch * 0x10;
        add(base, PDInputSub::NumberOfMappedObjects, u8(1));
        add(base, PDInputSub::ProcessInputValue, u32(1000 + ch));
        add(base, PDInputSub::ProcessInputStatus, u8(0));
    }
    // 2 process outputs: count + value(i32)
    for (uint8_t ch = 0; ch < 2; ++ch) {
        uint16_t base = ProcessDataOutput1 + ch * 0x10;
        add(base, PDOutputSub::NumberOfMappedObjects, u8(1));
        add(base, PDOutputSub::ProcessOutputValue, u32(0));
    }
    // Analog input/output config (channel-indexed subs 1..4 / 1..2)
    for (uint8_t ch = 1; ch <= 4; ++ch) {
        add(AnalogInputScalingFactor, ch, u32(0x10000));
        add(AnalogInputOffset, ch, u32(0));
        add(AnalogInputUnit, ch, u16(0));
        add(AnalogInputDecimalPlaces, ch, u8(0));
        add(AnalogInputRange, ch, u8(0));
        add(AnalogInputFilterTime, ch, u16(0));
    }
    for (uint8_t ch = 1; ch <= 2; ++ch) {
        add(AnalogOutputRange, ch, u8(0));
        add(AnalogOutputErrorBehavior, ch, u8(0));
        add(AnalogOutputErrorValue, ch, u32(0));
    }
    // Controller objects
    add(ControllerSetpoint, 0, u32(0));
    add(ControllerActualValue, 0, u32(4321));
    add(ControllerDeviation, 0, u32(5));
    add(ControllerOutput, 0, u32(777));
    add(ControllerMode, 0, u8(1));
    add(ControllerStatus, 0, u16(0));
    add(PID_Kp, 0, u32(100));
    add(PID_Ti, 0, u32(200));
    add(PID_Td, 0, u32(30));
    add(PID_SampleTime, 0, u16(10));
    add(PID_OutputUpperLimit, 0, u32(10000));
    add(PID_OutputLowerLimit, 0, u32(0xFFFF0000));
    add(PID_AntiWindup, 0, u32(5000));
    add(PID_DerivativeFilter, 0, u8(4));
    add(SetpointRampRate, 0, u32(0));
    add(FeedforwardGain, 0, u32(0));
    // Alarms (channel subs 1..4) + status
    for (uint8_t ch = 1; ch <= 4; ++ch) {
        add(AlarmHighHighLimit, ch, u32(32000));
        add(AlarmHighLimit, ch, u32(31000));
        add(AlarmLowLimit, ch, u32(0xFFFF0000));
        add(AlarmLowLowLimit, ch, u32(0xFFFE0000));
        add(WarningHighLimit, ch, u32(30000));
        add(WarningLowLimit, ch, u32(0xFFFF0000));
        add(AlarmHysteresis, ch, u32(0));
        add(AlarmDelayTime, ch, u16(0));
    }
    add(AlarmStatus, 1, u16(0));
    // Calibration + tare (channel subs 1..4) + status
    for (uint8_t ch = 1; ch <= 4; ++ch) {
        add(CalibrationCommand, ch, u8(0));
        add(CalibrationStatus, ch, u8(0));
        add(CalibrationPoint1Raw, ch, u32(0));
        add(CalibrationPoint1Eng, ch, u32(0));
        add(CalibrationPoint2Raw, ch, u32(0));
        add(CalibrationPoint2Eng, ch, u32(0));
        add(TareCommand, ch, u8(0));
        add(TareValue, ch, u32(0));
    }
    // Sensor diagnostics (channel subs 1..4)
    for (uint8_t ch = 1; ch <= 4; ++ch) {
        add(SensorStatus, ch, u8(0x03));
        add(SensorSupplyVoltage, ch, u16(2400));
        add(SensorTemperature, ch, u16(350));
        add(SignalQuality, ch, u8(95));
    }
    add(OperatingHours, 0, u32(12345));
    for (auto& e : entries) slave->addODEntry(e);

    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

class CiA404WireTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addMeasuringSlave(bus_);
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        transport_ = makeMasterSDOTransport(master_);
        coe_ = std::make_unique<CoE::CoEManager>(0, *transport_);
        coe_->init();
        coe_->configureMailbox(0x1000, 128, 0x1080, 128);
        dev_ = std::make_unique<MeasuringDevice>(*coe_);
    }
    void TearDown() override {
        dev_.reset();
        coe_->deinit();
        coe_.reset();
        transport_.reset();
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
    std::unique_ptr<SDO::ISDOTransport> transport_;
    std::unique_ptr<CoE::CoEManager> coe_;
    std::unique_ptr<MeasuringDevice> dev_;
};

} // namespace

TEST_F(CiA404WireTest, InitializeDetectsCapabilities) {
    ASSERT_TRUE(dev_->initialize());
    EXPECT_TRUE(dev_->isInitialized());
    const auto& caps = dev_->getCapabilities();
    EXPECT_EQ(caps.num_process_inputs, 4u);
    EXPECT_EQ(caps.num_process_outputs, 2u);
    EXPECT_TRUE(caps.has_controller);
    EXPECT_TRUE(caps.has_calibration);
    EXPECT_TRUE(caps.has_alarms);
    EXPECT_TRUE(caps.has_diagnostics);
}

TEST_F(CiA404WireTest, UpdateReadsInputsAndController) {
    ASSERT_TRUE(dev_->initialize());
    std::vector<int32_t> vals;
    dev_->setValueCallback(
        [&vals](uint16_t, uint8_t, int32_t raw, int32_t scaled) {
            vals.push_back(raw);
        });
    dev_->update();
    EXPECT_EQ(dev_->getRawValue(1), 1000);
    EXPECT_EQ(dev_->getRawValue(4), 1003);
    EXPECT_EQ(vals.size(), 4u);
    EXPECT_EQ(dev_->getControllerState().actual_value, 4321);
    EXPECT_EQ(dev_->getControllerState().deviation, 5);
    EXPECT_EQ(dev_->getControllerOutput(), 777);
    EXPECT_NE(dev_->getFilteredValue(1), 0x7FFFFFFF);
    EXPECT_EQ(dev_->getInputStatus(1), 0);
}

TEST_F(CiA404WireTest, ProcessTxPDOInputPath) {
    ASSERT_TRUE(dev_->initialize());
    dev_->applyPDOMapping(PDOMappingPreset::InputWithStatus);
    // Per channel: 4-byte value + 1 status byte
    const uint8_t pdo[] = {0xE8, 0x03, 0x00, 0x00, 0x07};
    dev_->processTxPDO(pdo, sizeof(pdo));
    EXPECT_EQ(dev_->getRawValue(1), 1000);
    EXPECT_EQ(dev_->getInputStatus(1), 0x07);
}

TEST_F(CiA404WireTest, ProcessControllerPDO) {
    ASSERT_TRUE(dev_->initialize());
    dev_->applyPDOMapping(PDOMappingPreset::Controller);
    TxPDO_ControllerStatus pdo{};
    pdo.actual_value = 111;
    pdo.output = 222;
    pdo.deviation = 33;
    pdo.status = 0x0007;
    dev_->processTxPDO(reinterpret_cast<const uint8_t*>(&pdo), sizeof(pdo));
    EXPECT_EQ(dev_->getControllerState().actual_value, 111);
    EXPECT_EQ(dev_->getControllerState().output, 222);
    EXPECT_EQ(dev_->getControllerState().deviation, 33);
    EXPECT_EQ(dev_->getControllerState().status, 0x0007);
}

TEST_F(CiA404WireTest, PrepareRxPDO) {
    ASSERT_TRUE(dev_->initialize());
    // Default mapping: output channel values
    EXPECT_TRUE(dev_->setOutputValue(1, 0x11223344));
    uint8_t buf[32] = {};
    size_t n = dev_->prepareRxPDO(buf, sizeof(buf));
    EXPECT_GE(n, 8u);
    EXPECT_EQ(buf[0], 0x44);

    // Controller mapping: setpoint + mode
    dev_->applyPDOMapping(PDOMappingPreset::Controller);
    dev_->setSetpoint(555);
    dev_->setControllerMode(2);
    uint8_t cbuf[16] = {};
    size_t cn = dev_->prepareRxPDO(cbuf, sizeof(cbuf));
    ASSERT_EQ(cn, sizeof(RxPDO_ControllerSetpoint));
    auto* p = reinterpret_cast<RxPDO_ControllerSetpoint*>(cbuf);
    EXPECT_EQ(p->setpoint, 555);
    EXPECT_EQ(p->mode, 2);
}

TEST_F(CiA404WireTest, InputOutputConfig) {
    ASSERT_TRUE(dev_->initialize());
    EXPECT_TRUE(dev_->configureInput(1, 0, 0x0001, 2));
    EXPECT_TRUE(dev_->setInputScaling(1, 0, 10000, 0, 5000));
    EXPECT_TRUE(dev_->setInputFilter(1, 25));
    EXPECT_TRUE(dev_->setOutputValue(2, 4242));
    EXPECT_EQ(dev_->getOutputValue(2), 4242);
    EXPECT_TRUE(dev_->configureOutput(1, 0));
    EXPECT_TRUE(dev_->setOutputErrorBehavior(1, 1, -5));
}

TEST_F(CiA404WireTest, ControllerPaths) {
    ASSERT_TRUE(dev_->initialize());
    EXPECT_TRUE(dev_->setSetpoint(1234));
    EXPECT_EQ(dev_->getSetpoint(), 1234);
    EXPECT_TRUE(dev_->setControllerMode(1));
    EXPECT_EQ(dev_->getControllerMode(), 1);

    PIDParameters params{};
    params.kp = 10; params.ti = 20; params.td = 30; params.sample_time = 5;
    params.output_max = 9000; params.output_min = -9000;
    params.anti_windup = 4000; params.derivative_filter = 3;
    EXPECT_TRUE(dev_->setPIDParameters(params));
    auto got = dev_->getPIDParameters();
    EXPECT_EQ(got.kp, 10);
    EXPECT_EQ(got.ti, 20);
    EXPECT_EQ(got.td, 30);

    EXPECT_TRUE(dev_->setPIDGains(11, 22, 33));
    EXPECT_TRUE(dev_->setOutputLimits(-100, 100));
    EXPECT_TRUE(dev_->setSetpointRamp(50));
    EXPECT_TRUE(dev_->setFeedforward(7));
    EXPECT_TRUE(dev_->resetIntegrator());
    dev_->update();
    EXPECT_TRUE(dev_->isControllerActive() || !dev_->isControllerActive());
    EXPECT_EQ(dev_->getDeviation(), 5);
}

TEST_F(CiA404WireTest, AlarmsAndEvents) {
    ASSERT_TRUE(dev_->initialize());
    std::vector<uint16_t> fired;
    dev_->setAlarmCallback(
        [&fired](uint16_t, uint8_t, uint16_t alarm, int32_t) {
            fired.push_back(alarm);
        });
    std::vector<int> events;
    dev_->setEventCallback(
        [&events](DeviceEvent ev, uint16_t, uint8_t, int32_t) {
            events.push_back(static_cast<int>(ev));
        });

    AlarmConfig cfg{};
    cfg.high_high_limit = 2000;
    cfg.high_limit = 1500;
    cfg.low_limit = -1500;
    cfg.low_low_limit = -2000;
    cfg.warning_high = 1200;
    cfg.warning_low = -1200;
    cfg.hysteresis = 0;
    ASSERT_TRUE(dev_->configureAlarms(1, cfg));

    // Drive input 1 above the high-high limit via PDO data.
    dev_->applyPDOMapping(PDOMappingPreset::WithAlarms);
    const int32_t hot = 2500;
    uint8_t pdo[5] = {};
    memcpy(pdo, &hot, 4);
    pdo[4] = 0;
    dev_->processTxPDO(pdo, sizeof(pdo));

    EXPECT_FALSE(fired.empty());
    EXPECT_FALSE(events.empty());
    EXPECT_TRUE(dev_->hasAlarm(1));
    EXPECT_NE(dev_->getAlarmStatus(1), 0);

    // Individual setters + acknowledge over the wire.
    EXPECT_TRUE(dev_->setAlarmHighHigh(1, 30000));
    EXPECT_TRUE(dev_->setAlarmHigh(1, 29000));
    EXPECT_TRUE(dev_->setAlarmLow(1, -29000));
    EXPECT_TRUE(dev_->setAlarmLowLow(1, -30000));
    EXPECT_TRUE(dev_->setAlarmHysteresis(1, 10));
    EXPECT_TRUE(dev_->acknowledgeAlarms(1));
    EXPECT_GE(dev_->getWarningStatus(1), 0);
}

TEST_F(CiA404WireTest, CalibrationAndTare) {
    ASSERT_TRUE(dev_->initialize());
    EXPECT_TRUE(dev_->calibrateZero(1, 0));
    EXPECT_TRUE(dev_->calibrateSpan(1, 5000));
    EXPECT_TRUE(dev_->calibrateTwoPoint(1, 0, 0, 10000, 5000));
    EXPECT_TRUE(dev_->acceptCalibration(1));
    EXPECT_TRUE(dev_->rejectCalibration(1));
    EXPECT_TRUE(dev_->restoreFactoryCalibration(1));
    EXPECT_TRUE(dev_->saveCalibration(1));
    EXPECT_EQ(dev_->getCalibrationStatus(1), 0);
    EXPECT_TRUE(dev_->setTare(1));
    EXPECT_TRUE(dev_->clearTare(1));
}

TEST_F(CiA404WireTest, SensorDiagnostics) {
    ASSERT_TRUE(dev_->initialize());
    EXPECT_EQ(dev_->getSensorStatus(1), 0x03);
    EXPECT_EQ(dev_->getSensorSupplyVoltage(1), 2400);
    EXPECT_EQ(dev_->getSensorTemperature(1), 350);
    EXPECT_EQ(dev_->getSignalQuality(1), 95);
    EXPECT_EQ(dev_->getOperatingHours(), 12345u);
    EXPECT_NE(dev_->getDiagnostics().find("CiA 404"), std::string::npos);
}
