/**
 * @file test_cia401_wire.cpp
 * @brief Wire-level coverage for CiA401::IOModule (CiA401IO.cpp): initialize/
 *        detectCapabilities/applyDefaultConfiguration, update() SDO reads,
 *        processTxPDO/prepareRxPDO data paths, digital/analog/counter/
 *        frequency/PWM operations, config writers and change/threshold
 *        callbacks — driven over EmulatedBus with a real SlaveEmulator.
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
#include "tether/profiles/cia401/CiA401IO.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Emulator;
using namespace CiA401;

namespace {

/// Build a CiA401-flavored emulated slave: mailbox pair + I/O object dictionary.
SlaveEmulator* addIoSlave(EmulatedBus& bus) {
    auto slave = std::make_unique<SlaveEmulator>();
    SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = 0x0401;
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
    struct E { uint16_t i; uint8_t s; std::vector<uint8_t> d; };
    const std::vector<E> entries = {
        {0x1000, 0, u32(0x00000191)},          // device type, profile 401
        {0x6000, 0, u8(2)}, {0x6000, 1, u8(0xA5)}, {0x6000, 2, u8(0x0F)},
        {0x6020, 0, u8(1)}, {0x6020, 1, u16(0x1234)},
        {0x6040, 0, u8(1)}, {0x6040, 1, u32(0xDEADBEEF)},
        {0x6200, 0, u8(1)}, {0x6200, 1, u8(0)}, {0x6200, 2, u8(0)},
        {0x6200, 3, u8(0)}, {0x6200, 4, u8(0)},
        {0x6202, 1, u8(0)}, {0x6206, 1, u8(0)}, {0x6207, 1, u8(0)},
        {0x6220, 0, u8(1)}, {0x6220, 1, u16(0)},
        {0x6240, 0, u8(1)}, {0x6240, 1, u32(0)},
        {0x6401, 0, u8(1)}, {0x6401, 1, u16(1234)},
        {0x6402, 0, u8(1)}, {0x6402, 1, u32(12345)},
        {0x6411, 0, u8(1)}, {0x6411, 1, u16(0)},
        {0x6412, 0, u8(1)}, {0x6412, 1, u32(0)},
        {0x6441, 1, u8(0)}, {0x6442, 1, u16(0)},
        {0x6451, 1, u32(0)}, {0x6452, 1, u32(0)},
        {0x6500, 0, u8(1)}, {0x6500, 1, u32(0x00BEEF00)},
        {0x6501, 1, u32(0)}, {0x6502, 1, u8(0)}, {0x6503, 1, u8(0x05)},
        {0x6510, 0, u8(1)}, {0x6510, 1, u32(5000)},
        {0x6511, 1, u32(200)}, {0x6512, 1, u16(750)},
        {0x6520, 0, u8(1)}, {0x6520, 1, u16(0)}, {0x6521, 1, u32(0)},
        {0x6002, 1, u8(0)}, {0x6003, 1, u8(0)}, {0x6005, 0, u8(0)},
        {0x6006, 1, u8(0)}, {0x6007, 1, u8(0)}, {0x6008, 1, u8(0)},
        {0x6421, 1, u8(0)}, {0x6423, 0, u8(0)},
        {0x6424, 1, u16(0)}, {0x6425, 1, u16(0)}, {0x6426, 1, u16(0)},
        {0x6431, 1, u32(0)}, {0x6432, 1, u32(0)}, {0x6440, 1, u32(0)},
    };
    for (const auto& e : entries) {
        slave->addODEntry({e.i, e.s, 0x0002,
                           static_cast<uint16_t>(e.d.size() * 8),
                           0x03, e.d, "cia401"});
    }

    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

class CiA401WireTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addIoSlave(bus_);
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        transport_ = makeMasterSDOTransport(master_);
        coe_ = std::make_unique<CoE::CoEManager>(0, *transport_);
        coe_->init();
        coe_->configureMailbox(0x1000, 128, 0x1080, 128);
        io_ = std::make_unique<IOModule>(*coe_);
    }
    void TearDown() override {
        io_.reset();
        coe_->deinit();
        coe_.reset();
        transport_.reset();
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    /// Overwrite an OD value on the wire (simulates a live device update).
    void writeOD(uint16_t index, uint8_t sub, const void* data, size_t len) {
        auto r = coe_->writeSync(index, sub, data, len, {});
        ASSERT_TRUE(r.has_value());
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
    std::unique_ptr<SDO::ISDOTransport> transport_;
    std::unique_ptr<CoE::CoEManager> coe_;
    std::unique_ptr<IOModule> io_;
};

} // namespace

TEST_F(CiA401WireTest, InitializeDetectsCapabilities) {
    ASSERT_TRUE(io_->initialize());
    EXPECT_TRUE(io_->isInitialized());
    const auto& caps = io_->getCapabilities();
    EXPECT_EQ(caps.module_type, ModuleType::MixedDigitalAnalog);
    EXPECT_EQ(caps.digital_input_8bit_blocks, 2u);
    EXPECT_EQ(caps.digital_input_16bit_blocks, 1u);
    EXPECT_EQ(caps.digital_input_32bit_blocks, 1u);
    EXPECT_EQ(caps.digital_output_8bit_blocks, 1u);
    EXPECT_EQ(caps.analog_input_16bit_channels, 1u);
    EXPECT_EQ(caps.analog_input_32bit_channels, 1u);
    EXPECT_EQ(caps.analog_output_16bit_channels, 1u);
    EXPECT_TRUE(caps.has_counters);
    EXPECT_TRUE(caps.has_frequency_input);
    EXPECT_TRUE(caps.has_pwm_output);
    EXPECT_TRUE(caps.supports_interrupt);
    EXPECT_TRUE(caps.supports_filtering);
    EXPECT_TRUE(caps.supports_polarity);
}

TEST_F(CiA401WireTest, UpdateReadsIoOverWire) {
    ASSERT_TRUE(io_->initialize());
    io_->update();
    EXPECT_TRUE(io_->readDigitalInput(0));          // 0xA5 bit0
    EXPECT_FALSE(io_->readDigitalInput(1));
    EXPECT_EQ(io_->readDigitalInput8(0), 0xA5);
    EXPECT_EQ(io_->readDigitalInput8(1), 0x0F);
    EXPECT_EQ(io_->readAnalogInput16(1), 1234);
    EXPECT_EQ(io_->readCounter(1), 0x00BEEF00u);
}

TEST_F(CiA401WireTest, DigitalWideReads) {
    ASSERT_TRUE(io_->initialize());
    // Feed 4 digital state bytes via TxPDO (Combined layout: DI bytes then AI).
    const uint8_t pdo[] = {0x78, 0x56, 0x34, 0x12};
    io_->processTxPDO(pdo, sizeof(pdo));
    EXPECT_EQ(io_->readDigitalInput16(0), 0x5678);
    EXPECT_EQ(io_->readDigitalInput32(0), 0x12345678u);
    EXPECT_TRUE(io_->readDigitalInput(3));          // bit3 of 0x78
    EXPECT_FALSE(io_->readDigitalInput(0));
}

TEST_F(CiA401WireTest, ProcessTxPDOCombinedAnalog) {
    ASSERT_TRUE(io_->initialize());
    // DI state is 4 bytes wide (2 + 2 + 4 blocks collapsed to 4 bytes),
    // then int16 analog channels follow.
    const uint8_t pdo[] = {0x01, 0x00, 0x00, 0x00, 0xE8, 0x03};
    io_->processTxPDO(pdo, sizeof(pdo));
    EXPECT_EQ(io_->readDigitalInput8(0), 0x01);
    EXPECT_EQ(io_->readAnalogInput16(1), 1000);
}

TEST_F(CiA401WireTest, DigitalCallbackOnChange) {
    ASSERT_TRUE(io_->initialize());
    std::vector<int> hits;
    io_->setDigitalInputCallback(
        [&hits](uint16_t, uint8_t channel, bool state) {
            hits.push_back(channel * 10 + (state ? 1 : 0));
        });
    std::vector<int> events;
    io_->setEventCallback(
        [&events](IOEvent ev, uint16_t, uint8_t a, int32_t b) {
            events.push_back(static_cast<int>(ev));
        });
    const uint8_t pdo[] = {0x00, 0x00, 0x00, 0x00};
    io_->processTxPDO(pdo, sizeof(pdo));
    // First cycle establishes prev state via update()
    io_->update();
    const uint8_t pdo2[] = {0xFF, 0x00, 0x00, 0x00};
    io_->processTxPDO(pdo2, sizeof(pdo2));
    EXPECT_FALSE(hits.empty());
    EXPECT_FALSE(events.empty());
}

TEST_F(CiA401WireTest, WriteOutputsOverWire) {
    ASSERT_TRUE(io_->initialize());
    EXPECT_TRUE(io_->writeDigitalOutput8(0, 0xFF));
    EXPECT_TRUE(io_->writeDigitalOutput(9, true));   // block 1 bit 1
    EXPECT_TRUE(io_->toggleDigitalOutput(8));        // toggles block 1 bit 0
    EXPECT_TRUE(io_->writeDigitalOutput16(0, 0xABCD));
    EXPECT_TRUE(io_->writeDigitalOutput32(0, 0xDEADBEEF));

    uint8_t buf[16] = {};
    size_t n = io_->prepareRxPDO(buf, sizeof(buf));
    EXPECT_GE(n, 4u);
    EXPECT_EQ(buf[0], 0xEF);   // block 0 after DO32 write
}

TEST_F(CiA401WireTest, DigitalConfigWriters) {
    ASSERT_TRUE(io_->initialize());
    EXPECT_TRUE(io_->setDigitalInputPolarity(0, 0x0F));
    EXPECT_TRUE(io_->setDigitalInputFilter(0, 0x03));
    EXPECT_TRUE(io_->setDigitalInputInterrupt(0, 0xFF, EdgeType::Any));
    EXPECT_TRUE(io_->setDigitalInputInterrupt(0, 0x0F, EdgeType::Rising));
    EXPECT_TRUE(io_->setDigitalInputInterrupt(0, 0xF0, EdgeType::Falling));
    EXPECT_TRUE(io_->setDigitalInterruptEnable(true));
    EXPECT_TRUE(io_->setDigitalOutputPolarity(0, 0xAA));
    EXPECT_TRUE(io_->setDigitalOutputErrorMode(0, ErrorMode::UseErrorValue, 0x55));
    EXPECT_TRUE(io_->setDigitalOutputErrorMode(0, ErrorMode::MaintainLastValue, 0));
}

TEST_F(CiA401WireTest, AnalogPaths) {
    ASSERT_TRUE(io_->initialize());
    io_->update();
    EXPECT_EQ(io_->readAnalogInput32(1), 12345);
    EXPECT_TRUE(io_->setAnalogInputScaling(1, 10, 0x10000));
    EXPECT_TRUE(io_->setAnalogInputRange(1, 0, 10000, 0, 5000));
    EXPECT_TRUE(io_->setAnalogInputSIUnit(1, 0x01, 0xFD));
    EXPECT_TRUE(io_->setAnalogInputThreshold(1, 2000, 100, 50,
                                           InterruptTrigger::UpperLimit));
    EXPECT_TRUE(io_->setAnalogInputInterruptEnable(true));
    EXPECT_NE(io_->readAnalogInputScaled(1), 0);
}

TEST_F(CiA401WireTest, AnalogThresholdCallback) {
    ASSERT_TRUE(io_->initialize());
    std::vector<int32_t> hits;
    io_->setAnalogThresholdCallback(
        [&hits](uint16_t, uint8_t ch, int16_t val, uint8_t kind) {
            hits.push_back(static_cast<int32_t>(val));
        });
    ASSERT_TRUE(io_->setAnalogInputThreshold(1, 1500, 100, 10,
                                             InterruptTrigger::UpperLimit |
                                             InterruptTrigger::LowerLimit));
    io_->update();                                  // baseline 1234
    int16_t high = 2000;
    writeOD(0x6401, 1, &high, sizeof(high));
    io_->update();                                  // crosses upper limit
    int16_t low = 50;
    writeOD(0x6401, 1, &low, sizeof(low));
    io_->update();                                  // crosses lower limit
    EXPECT_EQ(hits.size(), 2u);
    EXPECT_EQ(hits[0], 2000);
    EXPECT_EQ(hits[1], 50);
}

TEST_F(CiA401WireTest, AnalogOutputs) {
    ASSERT_TRUE(io_->initialize());
    EXPECT_TRUE(io_->writeAnalogOutput16(1, 4096));
    EXPECT_TRUE(io_->writeAnalogOutput32(1, 123456));
    EXPECT_TRUE(io_->setAnalogOutputScaling(1, 0, 0x10000));
    EXPECT_TRUE(io_->setAnalogOutputErrorMode(1, ErrorMode::UseErrorValue, 100));
}

TEST_F(CiA401WireTest, CounterFrequencyPwm) {
    ASSERT_TRUE(io_->initialize());
    io_->update();
    EXPECT_EQ(io_->readCounter(1), 0x00BEEF00u);
    EXPECT_EQ(io_->getCounterStatus(1), 0x05);
    EXPECT_TRUE(io_->setCounterPreset(1, 1000));
    EXPECT_TRUE(io_->loadCounterPreset(1));
    EXPECT_TRUE(io_->configureCounter(1, true, true));
    EXPECT_EQ(io_->readFrequencyInput(1), 5000u);
    EXPECT_EQ(io_->readPeriodInput(1), 200u);
    EXPECT_EQ(io_->readDutyCycleInput(1), 750);
    EXPECT_TRUE(io_->setPWMDutyCycle(1, 500));
    EXPECT_TRUE(io_->setPWMFrequency(1, 1000));
    EXPECT_TRUE(io_->configurePWM(1, 1000, 250));
}

TEST_F(CiA401WireTest, PdoMappingPresets) {
    ASSERT_TRUE(io_->initialize());
    EXPECT_TRUE(io_->applyPDOMapping(PDOMappingPreset::DigitalOnly));
    const uint8_t pdo[] = {0x55, 0xAA, 0x00, 0xFF};
    io_->processTxPDO(pdo, sizeof(pdo));
    EXPECT_EQ(io_->readDigitalInput8(0), 0x55);

    EXPECT_TRUE(io_->applyPDOMapping(PDOMappingPreset::AnalogOnly));
    const uint8_t apdo[] = {0x10, 0x00};
    io_->processTxPDO(apdo, sizeof(apdo));
    EXPECT_EQ(io_->readAnalogInput16(1), 16);

    EXPECT_TRUE(io_->applyPDOMapping(PDOMappingPreset::Full));
    EXPECT_EQ(io_->getCurrentMapping(), PDOMappingPreset::Full);
}
