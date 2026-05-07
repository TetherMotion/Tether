/**
 * @file test_ETG5000_coverage.cpp
 * @brief Extended ETG5000 ModularDevice coverage tests
 */
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "tether/etg5000/ETG5000ModularDevice.hpp"
#include "tether/etg5000/ETG5000Defs.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"

using namespace ETG5000;

namespace {
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};
} // namespace

class ETG5000CovTest : public ::testing::Test {
protected:
    void SetUp() override {
        transport = std::make_unique<NullSDOTransport>();
        sdo = std::make_unique<EtherCAT::SDO::SDOManager>(*transport);
        sdo->init();
        dev = std::make_unique<ModularDevice>(*sdo, 1);
    }
    void TearDown() override {
        dev.reset();
        sdo->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport;
    std::unique_ptr<EtherCAT::SDO::SDOManager> sdo;
    std::unique_ptr<ModularDevice> dev;
};

// --- Basic state ---

TEST_F(ETG5000CovTest, NotInitializedByDefault) {
    EXPECT_FALSE(dev->isInitialized());
}

TEST_F(ETG5000CovTest, InitializeReturnsFalse) {
    EXPECT_FALSE(dev->initialize());
}

TEST_F(ETG5000CovTest, GetConfigState) {
    auto& cs = dev->getConfigState();
    (void)cs;
}

TEST_F(ETG5000CovTest, GetState) {
    auto& st = dev->getState();
    (void)st;
}

TEST_F(ETG5000CovTest, GetStatusword) {
    uint16_t sw = dev->getStatusword();
    (void)sw;
}

TEST_F(ETG5000CovTest, GetModuleCountZero) {
    EXPECT_EQ(dev->getModuleCount(), 0u);
}

// --- Module queries on empty device ---

TEST_F(ETG5000CovTest, IsModulePresentFalse) {
    EXPECT_FALSE(dev->isModulePresent(0));
    EXPECT_FALSE(dev->isModulePresent(255));
}

TEST_F(ETG5000CovTest, IsModuleOperationalFalse) {
    EXPECT_FALSE(dev->isModuleOperational(0));
}

TEST_F(ETG5000CovTest, GetModuleStateNull) {
    auto* ms = dev->getModuleState(0);
    EXPECT_EQ(ms, nullptr);
}

TEST_F(ETG5000CovTest, GetAllModulesEmpty) {
    auto& mods = dev->getAllModules();
    EXPECT_TRUE(mods.empty());
}

TEST_F(ETG5000CovTest, GetModuleInfoFalse) {
    SlotInfo info{};
    EXPECT_FALSE(dev->getModuleInfo(0, info));
}

// --- Module read/write on empty device ---

TEST_F(ETG5000CovTest, ReadModuleInputFalse) {
    uint8_t data[4] = {};
    EXPECT_FALSE(dev->readModuleInput(0, data, 4));
}

TEST_F(ETG5000CovTest, WriteModuleOutputFalse) {
    uint8_t data[4] = {1, 2, 3, 4};
    EXPECT_FALSE(dev->writeModuleOutput(0, data, 4));
}

// --- Module offsets/sizes for absent modules ---

TEST_F(ETG5000CovTest, ModuleInputOffsetZero) {
    EXPECT_EQ(dev->getModuleInputOffset(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleOutputOffsetZero) {
    EXPECT_EQ(dev->getModuleOutputOffset(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleInputSizeZero) {
    EXPECT_EQ(dev->getModuleInputSize(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleOutputSizeZero) {
    EXPECT_EQ(dev->getModuleOutputSize(0), 0u);
}

// --- Module control on empty device ---

TEST_F(ETG5000CovTest, EnableModuleFalse) {
    EXPECT_FALSE(dev->enableModule(0, true));
}

TEST_F(ETG5000CovTest, EnableAllModulesFalse) {
    EXPECT_FALSE(dev->enableAllModules(true));
}

TEST_F(ETG5000CovTest, ResetModuleErrorsFalse) {
    EXPECT_FALSE(dev->resetModuleErrors(0));
}

TEST_F(ETG5000CovTest, ResetAllErrorsFalse) {
    EXPECT_FALSE(dev->resetAllErrors());
}

TEST_F(ETG5000CovTest, ResetModuleToDefaultsFalse) {
    EXPECT_FALSE(dev->resetModuleToDefaults(0));
}

// --- Configuration ---

TEST_F(ETG5000CovTest, SetExpectedConfiguration) {
    std::vector<ModuleDescriptor> descs;
    ModuleDescriptor md{};
    md.slot = 0;
    md.expected_type = 1;
    descs.push_back(md);
    bool result = dev->setExpectedConfiguration(descs);
    (void)result;
}

TEST_F(ETG5000CovTest, EnableAutoConfiguration) {
    bool result = dev->enableAutoConfiguration(true);
    (void)result;
}

TEST_F(ETG5000CovTest, ValidateConfiguration) {
    bool result = dev->validateConfiguration();
    (void)result;
}

TEST_F(ETG5000CovTest, AcceptConfiguration) {
    bool result = dev->acceptConfiguration();
    (void)result;
}

TEST_F(ETG5000CovTest, SaveConfiguration) {
    bool result = dev->saveConfiguration();
    (void)result;
}

TEST_F(ETG5000CovTest, ScanModules) {
    EXPECT_FALSE(dev->scanModules());
}

// --- Diagnostics ---

TEST_F(ETG5000CovTest, ModuleDiagStatus) {
    EXPECT_EQ(dev->getModuleDiagStatus(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleLastError) {
    EXPECT_EQ(dev->getModuleLastError(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleTemperature) {
    float t = dev->getModuleTemperature(0);
    (void)t;
}

TEST_F(ETG5000CovTest, ModuleSupplyVoltage) {
    float v = dev->getModuleSupplyVoltage(0);
    (void)v;
}

TEST_F(ETG5000CovTest, ModuleErrorCount) {
    EXPECT_EQ(dev->getModuleErrorCount(0), 0u);
}

TEST_F(ETG5000CovTest, ModuleOperatingHours) {
    EXPECT_EQ(dev->getModuleOperatingHours(0), 0u);
}

TEST_F(ETG5000CovTest, Diagnostics) {
    auto diag = dev->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

// --- PDO processing ---

TEST_F(ETG5000CovTest, ProcessTxPDO) {
    uint8_t data[16] = {};
    dev->processTxPDO(data, 16);
}

TEST_F(ETG5000CovTest, PrepareRxPDO) {
    uint8_t data[16] = {};
    size_t len = dev->prepareRxPDO(data, 16);
    EXPECT_LE(len, 16u);
}

TEST_F(ETG5000CovTest, Update) {
    dev->update();
}

// --- Callbacks ---

TEST_F(ETG5000CovTest, SetModuleEventCallback) {
    dev->setModuleEventCallback([](uint8_t, uint8_t) {});
}

TEST_F(ETG5000CovTest, SetConfigChangeCallback) {
    dev->setConfigChangeCallback([](const DeviceConfigState&) {});
}

TEST_F(ETG5000CovTest, SetDiagnosticCallback) {
    dev->setDiagnosticCallback([](uint8_t, uint16_t) {});
}

TEST_F(ETG5000CovTest, SetErrorCallback) {
    dev->setErrorCallback([](uint8_t, uint16_t) {});
}
