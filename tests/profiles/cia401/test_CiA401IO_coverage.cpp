/**
 * @file test_CiA401IO_coverage.cpp
 * @brief Extended CiA401 IOModule coverage tests — PDO processing, callbacks,
 *        internal state exercise, digital/analog parsing paths
 */
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cstring>
#include "tether/profiles/cia401/CiA401IO.hpp"
#include "tether/profiles/cia401/CiA401Defs.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"

using namespace CiA401;

namespace {
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*, bool, unsigned int,
                   unsigned int) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t, bool, unsigned int,
                     unsigned int) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};
} // namespace

class CiA401CovTest : public ::testing::Test {
protected:
    void SetUp() override {
        transport = std::make_unique<NullSDOTransport>();
        coe = std::make_unique<EtherCAT::CoE::CoEManager>(1, *transport);
        coe->init();
        io = std::make_unique<IOModule>(*coe);
    }
    void TearDown() override {
        io.reset();
        coe->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport;
    std::unique_ptr<EtherCAT::CoE::CoEManager> coe;
    std::unique_ptr<IOModule> io;
};

// --- Digital Input ---

TEST_F(CiA401CovTest, ReadDigitalInput) {
    bool val = io->readDigitalInput(0);
    EXPECT_FALSE(val);
}

TEST_F(CiA401CovTest, ReadDigitalInput8) {
    uint8_t val = io->readDigitalInput8(0);
    EXPECT_EQ(val, 0u);
}

TEST_F(CiA401CovTest, ReadDigitalInput16) {
    uint16_t val = io->readDigitalInput16(0);
    EXPECT_EQ(val, 0u);
}

TEST_F(CiA401CovTest, ReadDigitalInput32) {
    uint32_t val = io->readDigitalInput32(0);
    EXPECT_EQ(val, 0u);
}

// --- Digital Output ---

TEST_F(CiA401CovTest, WriteDigitalOutput) {
    io->writeDigitalOutput(0, true);
    io->writeDigitalOutput(0, false);
}

TEST_F(CiA401CovTest, WriteDigitalOutput8) {
    io->writeDigitalOutput8(0, 0xFF);
}

TEST_F(CiA401CovTest, WriteDigitalOutput16) {
    io->writeDigitalOutput16(0, 0xCAFE);
}

TEST_F(CiA401CovTest, WriteDigitalOutput32) {
    io->writeDigitalOutput32(0, 0xDEADBEEF);
}

TEST_F(CiA401CovTest, SetDigitalOutputBit) {
    io->setDigitalOutputBit(0, true);
    io->setDigitalOutputBit(0, false);
}

TEST_F(CiA401CovTest, ToggleDigitalOutput) {
    io->toggleDigitalOutput(0);
}

// --- Digital configuration ---

TEST_F(CiA401CovTest, SetDigitalInputPolarity) {
    io->setDigitalInputPolarity(0, 0xFF);
}

TEST_F(CiA401CovTest, SetDigitalOutputPolarity) {
    io->setDigitalOutputPolarity(0, 0xFF);
}

TEST_F(CiA401CovTest, SetDigitalInputFilter) {
    io->setDigitalInputFilter(0, 100);
}

TEST_F(CiA401CovTest, SetDigitalInputInterrupt) {
    io->setDigitalInputInterrupt(0, 0xFF, EdgeType::Both);
}

TEST_F(CiA401CovTest, SetDigitalInterruptEnable) {
    io->setDigitalInterruptEnable(true);
    io->setDigitalInterruptEnable(false);
}

TEST_F(CiA401CovTest, SetDigitalOutputErrorMode) {
    io->setDigitalOutputErrorMode(0, 0, 0);
}

// --- Analog Input ---

TEST_F(CiA401CovTest, ReadAnalogInput16) {
    int16_t val = io->readAnalogInput16(0);
    EXPECT_EQ(val, 0);
}

TEST_F(CiA401CovTest, ReadAnalogInput32) {
    int32_t val = io->readAnalogInput32(0);
    EXPECT_EQ(val, 0);
}

TEST_F(CiA401CovTest, ReadAnalogInputScaled) {
    int32_t val = io->readAnalogInputScaled(0);
    (void)val;
}

// --- Analog Output ---

TEST_F(CiA401CovTest, WriteAnalogOutput16) {
    io->writeAnalogOutput16(0, 1000);
}

TEST_F(CiA401CovTest, WriteAnalogOutput32) {
    io->writeAnalogOutput32(0, 100000);
}

TEST_F(CiA401CovTest, WriteAnalogOutputScaled) {
    io->writeAnalogOutputScaled(0, 5000);
}

// --- Analog configuration ---

TEST_F(CiA401CovTest, SetAnalogInputScaling) {
    io->setAnalogInputScaling(0, 0, 10000);
}

TEST_F(CiA401CovTest, SetAnalogInputRange) {
    io->setAnalogInputRange(0, -10000, 10000, -10000, 10000);
}

TEST_F(CiA401CovTest, SetAnalogInputSIUnit) {
    io->setAnalogInputSIUnit(0, 0x44, 0x00);
}

TEST_F(CiA401CovTest, SetAnalogInputThreshold) {
    io->setAnalogInputThreshold(0, 8000, 2000, 100, 0);
}

TEST_F(CiA401CovTest, SetAnalogInputInterruptEnable) {
    io->setAnalogInputInterruptEnable(true);
    io->setAnalogInputInterruptEnable(false);
}

TEST_F(CiA401CovTest, SetAnalogOutputScaling) {
    io->setAnalogOutputScaling(0, 0, 10000);
}

TEST_F(CiA401CovTest, SetAnalogOutputErrorMode) {
    io->setAnalogOutputErrorMode(0, 0, 0);
}

// --- Counter ---

TEST_F(CiA401CovTest, ReadCounter) {
    auto val = io->readCounter(0);
    (void)val;
}

TEST_F(CiA401CovTest, SetCounterPreset) {
    io->setCounterPreset(0, 1000);
}

TEST_F(CiA401CovTest, LoadCounterPreset) {
    io->loadCounterPreset(0);
}

TEST_F(CiA401CovTest, ConfigureCounter) {
    io->configureCounter(0, true, true);
}

TEST_F(CiA401CovTest, GetCounterStatus) {
    auto stat = io->getCounterStatus(0);
    (void)stat;
}

// --- Frequency/PWM ---

TEST_F(CiA401CovTest, ReadFrequencyInput) {
    auto val = io->readFrequencyInput(0);
    (void)val;
}

TEST_F(CiA401CovTest, ReadPeriodInput) {
    auto val = io->readPeriodInput(0);
    (void)val;
}

TEST_F(CiA401CovTest, ReadDutyCycleInput) {
    auto val = io->readDutyCycleInput(0);
    (void)val;
}

TEST_F(CiA401CovTest, SetPWMDutyCycle) {
    io->setPWMDutyCycle(0, 500);
}

TEST_F(CiA401CovTest, SetPWMFrequency) {
    io->setPWMFrequency(0, 1000);
}

TEST_F(CiA401CovTest, ConfigurePWM) {
    io->configurePWM(0, 1000, 500);
}

// --- Callbacks ---

TEST_F(CiA401CovTest, SetDigitalInputCallback) {
    io->setDigitalInputCallback([](uint16_t, uint8_t, bool) {});
}

TEST_F(CiA401CovTest, SetAnalogThresholdCallback) {
    io->setAnalogThresholdCallback([](uint16_t, uint8_t, int32_t, uint8_t) {});
}

TEST_F(CiA401CovTest, SetEventCallback) {
    io->setEventCallback([](IOEvent, uint16_t, uint8_t, uint32_t) {});
}

// --- PDO Mapping ---

TEST_F(CiA401CovTest, ApplyPDOMappingMinimal) {
    io->applyPDOMapping(PDOMappingPreset::Minimal);
}

TEST_F(CiA401CovTest, ApplyPDOMappingFull) {
    io->applyPDOMapping(PDOMappingPreset::Full);
}

TEST_F(CiA401CovTest, GetCurrentMapping) {
    auto m = io->getCurrentMapping();
    (void)m;
}

// --- PDO processing ---

TEST_F(CiA401CovTest, ProcessTxPDOEmpty) {
    uint8_t data[32] = {};
    io->processTxPDO(data, 0);
}

TEST_F(CiA401CovTest, ProcessTxPDOWithData) {
    uint8_t data[32] = {};
    data[0] = 0xFF;
    io->processTxPDO(data, 32);
}

TEST_F(CiA401CovTest, PrepareRxPDO) {
    uint8_t data[32] = {};
    size_t len = io->prepareRxPDO(data, 32);
    EXPECT_LE(len, 32u);
}

// --- Update with callbacks installed ---

TEST_F(CiA401CovTest, UpdateWithCallbacks) {
    bool diCalled = false;
    io->setDigitalInputCallback([&](uint16_t, uint8_t, bool) { diCalled = true; });

    uint8_t data1[8] = {};
    io->processTxPDO(data1, 8);
    io->update();

    uint8_t data2[8] = {};
    data2[0] = 0x01;
    io->processTxPDO(data2, 8);
    io->update();
}

// --- Diagnostics ---

TEST_F(CiA401CovTest, GetDiagnostics) {
    auto diag = io->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA401CovTest, HasFault) {
    EXPECT_FALSE(io->hasFault());
}

TEST_F(CiA401CovTest, ResetFault) {
    io->resetFault();
}

TEST_F(CiA401CovTest, GetCapabilities) {
    auto& caps = io->getCapabilities();
    (void)caps;
}

TEST_F(CiA401CovTest, IsInitialized) {
    EXPECT_FALSE(io->isInitialized());
}

// --- Free functions ---

TEST_F(CiA401CovTest, GetPDOMappingName) {
    auto* name = getPDOMappingName(PDOMappingPreset::Minimal);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA401CovTest, GetIOEventName) {
    auto* name = getIOEventName(IOEvent::DigitalInputChanged);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA401CovTest, GetModuleTypeName) {
    auto* name = getModuleTypeName(ModuleType::FullFeatured);
    EXPECT_NE(name, nullptr);
}

TEST_F(CiA401CovTest, ApplyScaling) {
    int32_t out = applyScaling(1000, 0, 100);
    (void)out;
}

TEST_F(CiA401CovTest, ReverseScaling) {
    int32_t out = reverseScaling(100000, 0, 100);
    (void)out;
}

// --- Initialize ---

TEST_F(CiA401CovTest, InitializeReturns) {
    bool result = io->initialize();
    (void)result;
}
