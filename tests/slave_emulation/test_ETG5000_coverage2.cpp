/**
 * @file test_ETG5000_coverage2.cpp
 * @brief Extended ETG5000 coverage with a mock SDO transport that simulates real modules.
 *
 * Targets all "happy path" branches: initialize() success, validateConfiguration()
 * with present/missing/extra/mismatched modules, processTxPDO guards & diag bitmap,
 * prepareRxPDO small buffer, updateModuleStates() transitions, callback invocations,
 * getDiagnostics() with modules, getModuleInfo() valid slot, enableAllModules disable,
 * checkHotSwapEvents, checkConfigurationState, acceptConfiguration success,
 * saveConfiguration success.
 */

#include <gtest/gtest.h>
#include <memory>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>
#include "tether/etg5000/ETG5000ModularDevice.hpp"
#include "tether/etg5000/ETG5000Defs.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"

using namespace ETG5000;

namespace {

/**
 * @brief Mock SDO transport that returns configurable data per (index, subindex).
 *
 * sdoUpload fills the caller's buffer with pre-loaded data.
 * sdoDownload records the writes.
 */
class MockSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    // Pre-load a value to return on a given (index, subindex)
    void setReadData(uint16_t index, uint8_t subindex, const void* data, size_t len) {
        auto key = std::make_pair(index, subindex);
        std::vector<uint8_t> v(static_cast<const uint8_t*>(data),
                               static_cast<const uint8_t*>(data) + len);
        read_data_[key] = std::move(v);
    }

    template<typename T>
    void setReadValue(uint16_t index, uint8_t subindex, T value) {
        setReadData(index, subindex, &value, sizeof(value));
    }

    bool upload_returns = true;
    bool download_returns = true;

    bool sdoUpload(uint16_t slave, uint8_t* mbx_cnt,
                   uint16_t wr_addr, uint16_t wr_len,
                   uint16_t rd_addr, uint16_t rd_len,
                   uint16_t index, uint8_t subindex,
                   uint8_t* buf, size_t buf_sz, size_t* out_len) override {
        if (!upload_returns) return false;
        auto key = std::make_pair(index, subindex);
        auto it = read_data_.find(key);
        if (it == read_data_.end()) {
            // Return zeros for unknown indices
            if (out_len) *out_len = 0;
            return true;
        }
        size_t cp = std::min(it->second.size(), buf_sz);
        std::memcpy(buf, it->second.data(), cp);
        if (out_len) *out_len = cp;
        return true;
    }

    bool sdoDownload(uint16_t slave, uint8_t* mbx_cnt,
                     uint16_t wr_addr, uint16_t wr_len,
                     uint16_t rd_addr, uint16_t rd_len,
                     uint16_t index, uint8_t subindex,
                     const uint8_t* data, size_t len) override {
        if (!download_returns) return false;
        last_write_index = index;
        last_write_subindex = subindex;
        return true;
    }

    uint64_t getMicroseconds() override { return time_us++; }

    uint16_t last_write_index = 0;
    uint8_t  last_write_subindex = 0;

private:
    uint64_t time_us = 0;
    std::map<std::pair<uint16_t, uint8_t>, std::vector<uint8_t>> read_data_;
};

/// Set up mock transport with N modules
void setupModules(MockSDOTransport& t, uint8_t count,
                  uint16_t module_type = ModuleType::DigitalInput,
                  uint8_t status = ModuleStatus::Operational) {
    t.setReadValue<uint8_t>(ModuleConfig::DetectedModuleCount, 0, count);
    for (uint8_t i = 0; i < count; i++) {
        t.setReadValue<uint16_t>(ModuleConfig::ModuleTypeList, i + 1, module_type);
        t.setReadValue<uint8_t>(ModuleConfig::ModuleStatusList, i + 1, status);
        t.setReadValue<uint16_t>(ModuleDiag::DiagnosticStatus, i + 1, DiagStatus::ModuleOK);
        t.setReadValue<uint16_t>(ModuleDiag::LastError, i + 1, uint16_t(0));
        t.setReadValue<uint16_t>(ModuleDiag::Temperature, i + 1, uint16_t(2500)); // 25.0 C
        t.setReadValue<uint16_t>(ModuleDiag::SupplyVoltage, i + 1, uint16_t(2400)); // 24.0 V
        t.setReadValue<uint32_t>(ModuleDiag::ErrorCount, i + 1, uint32_t(0));
        t.setReadValue<uint16_t>(ModulePDO::InputOffset, i, uint16_t(i * 4));
        t.setReadValue<uint16_t>(ModulePDO::OutputOffset, i, uint16_t(i * 2));
        t.setReadValue<uint16_t>(ModulePDO::PDOSize, i, uint16_t(4));      // input size
        t.setReadValue<uint16_t>(ModulePDO::PDOSize, i + 64, uint16_t(2)); // output size
    }
}

} // namespace

class ETG5000Cov2Test : public ::testing::Test {
protected:
    void SetUp() override {
        transport = std::make_unique<MockSDOTransport>();
    }

    std::unique_ptr<ModularDevice> createDevice() {
        sdo = std::make_unique<EtherCAT::SDO::SDOManager>(*transport);
        sdo->init();
        sdo->configureSlaveMailbox(1, 0x1000, 128, 0x1200, 128);
        return std::make_unique<ModularDevice>(*sdo, 1);
    }

    void TearDown() override {
        dev.reset();
        if (sdo) sdo->deinit();
    }

    std::unique_ptr<MockSDOTransport> transport;
    std::unique_ptr<EtherCAT::SDO::SDOManager> sdo;
    std::unique_ptr<ModularDevice> dev;
};

// ============================================================================
// initialize() success path
// ============================================================================

TEST_F(ETG5000Cov2Test, Initialize_Success) {
    setupModules(*transport, 2);
    dev = createDevice();
    EXPECT_TRUE(dev->initialize());
    EXPECT_TRUE(dev->isInitialized());
    EXPECT_EQ(dev->getModuleCount(), 2);
}

// ============================================================================
// getModuleState / isModulePresent / isModuleOperational with modules
// ============================================================================

TEST_F(ETG5000Cov2Test, ModuleState_ValidSlot) {
    setupModules(*transport, 2, ModuleType::DigitalInput, ModuleStatus::Operational);
    dev = createDevice();
    dev->initialize();

    const ModuleState* ms = dev->getModuleState(0);
    ASSERT_NE(ms, nullptr);
    EXPECT_TRUE(ms->isPresent());
    EXPECT_TRUE(ms->isOperational());
    EXPECT_FALSE(ms->hasError());
}

TEST_F(ETG5000Cov2Test, IsModulePresent_True) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->isModulePresent(0));
    EXPECT_FALSE(dev->isModulePresent(99));
}

TEST_F(ETG5000Cov2Test, IsModuleOperational_True) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Operational);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->isModuleOperational(0));
}

// ============================================================================
// getModuleInfo with valid slot
// ============================================================================

TEST_F(ETG5000Cov2Test, GetModuleInfo_Valid) {
    setupModules(*transport, 1, ModuleType::AnalogInput);
    transport->setReadValue<uint32_t>(ModuleIdent::VendorID, 1, uint32_t(0x1234));
    transport->setReadValue<uint32_t>(ModuleIdent::ProductCode, 1, uint32_t(0x5678));
    transport->setReadValue<uint32_t>(ModuleIdent::RevisionNumber, 1, uint32_t(3));
    transport->setReadValue<uint32_t>(ModuleIdent::SerialNumber, 1, uint32_t(42));
    transport->setReadValue<uint32_t>(ModuleIdent::Capabilities, 1, uint32_t(ModuleCaps::PDOInput));
    transport->setReadValue<uint32_t>(ModuleDiag::OperatingHours, 1, uint32_t(1000));

    dev = createDevice();
    dev->initialize();

    SlotInfo info{};
    EXPECT_TRUE(dev->getModuleInfo(0, info));
    EXPECT_EQ(info.slot_number, 0);
    EXPECT_EQ(info.input_size, 4);
    EXPECT_EQ(info.output_size, 2);
    EXPECT_EQ(info.input_offset, 0);
}

// ============================================================================
// Module offsets/sizes with data
// ============================================================================

TEST_F(ETG5000Cov2Test, ModuleOffsetsSizes_WithModules) {
    setupModules(*transport, 2);
    dev = createDevice();
    dev->initialize();

    EXPECT_EQ(dev->getModuleInputOffset(0), 0);
    EXPECT_EQ(dev->getModuleOutputOffset(0), 0);
    EXPECT_EQ(dev->getModuleInputSize(0), 4);
    EXPECT_EQ(dev->getModuleOutputSize(0), 2);

    EXPECT_EQ(dev->getModuleInputOffset(1), 4);
    EXPECT_EQ(dev->getModuleOutputOffset(1), 2);
}

// ============================================================================
// validateConfiguration — various scenarios
// ============================================================================

TEST_F(ETG5000Cov2Test, Validate_AllPresent_TypeMatch) {
    setupModules(*transport, 2, ModuleType::DigitalInput);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::DigitalInput, 0, 0, false, false});
    expected.push_back({1, ModuleType::DigitalInput, 0, 0, false, false});

    EXPECT_TRUE(dev->setExpectedConfiguration(expected));
    auto& cs = dev->getConfigState();
    EXPECT_EQ(cs.config_state, ConfigState::ConfigurationValid);
    EXPECT_TRUE(cs.missing_slots.empty());
    EXPECT_TRUE(cs.mismatched_slots.empty());
}

TEST_F(ETG5000Cov2Test, Validate_TypeMismatch) {
    setupModules(*transport, 1, ModuleType::DigitalInput);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::AnalogInput, 0, 0, false, false});

    EXPECT_FALSE(dev->setExpectedConfiguration(expected));
    auto& cs = dev->getConfigState();
    EXPECT_EQ(cs.config_state, ConfigState::ConfigMismatch);
    EXPECT_EQ(cs.mismatched_slots.size(), 1u);
}

TEST_F(ETG5000Cov2Test, Validate_TypeMismatch_AllowCompatible) {
    setupModules(*transport, 1, ModuleType::DigitalInput);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::AnalogInput, 0, 0, false, true}); // allow_compatible

    EXPECT_TRUE(dev->setExpectedConfiguration(expected));
    auto& cs = dev->getConfigState();
    EXPECT_TRUE(cs.mismatched_slots.empty());
}

TEST_F(ETG5000Cov2Test, Validate_MissingSlot) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::Unknown, 0, 0, false, false});
    expected.push_back({5, ModuleType::Unknown, 0, 0, false, false}); // not present

    EXPECT_FALSE(dev->setExpectedConfiguration(expected));
    auto& cs = dev->getConfigState();
    EXPECT_EQ(cs.config_state, ConfigState::ModuleMissing);
    EXPECT_FALSE(cs.missing_slots.empty());
}

TEST_F(ETG5000Cov2Test, Validate_OptionalMissing) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::Unknown, 0, 0, false, false});
    expected.push_back({5, ModuleType::Unknown, 0, 0, true, false}); // optional

    EXPECT_TRUE(dev->setExpectedConfiguration(expected));
}

TEST_F(ETG5000Cov2Test, Validate_ExtraModules) {
    setupModules(*transport, 2);
    dev = createDevice();
    dev->initialize();

    // Only expect 1 module — slot 1 is "extra"
    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::Unknown, 0, 0, false, false});

    EXPECT_TRUE(dev->setExpectedConfiguration(expected));
    auto& cs = dev->getConfigState();
    EXPECT_EQ(cs.config_state, ConfigState::ExtraModules);
    EXPECT_EQ(cs.extra_slots.size(), 1u);
}

TEST_F(ETG5000Cov2Test, Validate_ModuleWithError) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Error);
    dev = createDevice();
    dev->initialize();

    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::DigitalInput, 0, 0, false, false});

    dev->setExpectedConfiguration(expected);
    auto& cs = dev->getConfigState();
    EXPECT_FALSE(cs.all_modules_ok);
}

// ============================================================================
// acceptConfiguration — success path
// ============================================================================

TEST_F(ETG5000Cov2Test, AcceptConfiguration_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    bool cfg_callback_called = false;
    dev->setConfigChangeCallback([&](const DeviceConfigState& c) {
        cfg_callback_called = true;
    });

    EXPECT_TRUE(dev->acceptConfiguration());
    EXPECT_TRUE(cfg_callback_called);
}

// ============================================================================
// saveConfiguration — success path
// ============================================================================

TEST_F(ETG5000Cov2Test, SaveConfiguration_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->saveConfiguration());
}

// ============================================================================
// enableAllModules — disable path
// ============================================================================

TEST_F(ETG5000Cov2Test, EnableAllModules_Disable) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->enableAllModules(false));
}

// ============================================================================
// processTxPDO — guard branches
// ============================================================================

TEST_F(ETG5000Cov2Test, ProcessTxPDO_Nullptr) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    dev->processTxPDO(nullptr, 100); // nullptr guard
}

TEST_F(ETG5000Cov2Test, ProcessTxPDO_TooSmall) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    uint8_t small[2] = {0};
    dev->processTxPDO(small, sizeof(small)); // too small guard
}

TEST_F(ETG5000Cov2Test, ProcessTxPDO_NotInitialized) {
    dev = createDevice();
    ModularInputPDO pdo{};
    dev->processTxPDO(reinterpret_cast<const uint8_t*>(&pdo), sizeof(pdo));
}

TEST_F(ETG5000Cov2Test, ProcessTxPDO_WithModules) {
    setupModules(*transport, 2);
    dev = createDevice();
    dev->initialize();

    ModularInputPDO pdo{};
    pdo.statusword = 0;
    pdo.module_count = 2;
    pdo.config_state = ConfigState::ConfigurationValid;
    pdo.diag_status_bitmap[0] = 0x0001; // diag for slot 0

    dev->processTxPDO(reinterpret_cast<const uint8_t*>(&pdo), sizeof(pdo));

    auto& st = dev->getState();
    EXPECT_EQ(st.statusword, 0);
}

// ============================================================================
// prepareRxPDO — small buffer
// ============================================================================

TEST_F(ETG5000Cov2Test, PrepareRxPDO_TooSmall) {
    dev = createDevice();
    uint8_t buf[1] = {0};
    EXPECT_EQ(dev->prepareRxPDO(buf, sizeof(buf)), 0u);
}

// ============================================================================
// updateModuleStates — status transitions
// ============================================================================

TEST_F(ETG5000Cov2Test, Update_ModuleInserted) {
    // Start with NotPresent, then change status to Operational
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::NotPresent);
    dev = createDevice();
    dev->initialize();

    bool inserted = false;
    dev->setModuleEventCallback([&](uint8_t slot, uint8_t event) {
        if (event == ModuleEvent::Inserted) inserted = true;
    });

    // Change status to Operational for next read
    transport->setReadValue<uint8_t>(ModuleConfig::ModuleStatusList, 1, ModuleStatus::Operational);
    dev->update();

    EXPECT_TRUE(inserted);
}

TEST_F(ETG5000Cov2Test, Update_ModuleRemoved) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Operational);
    dev = createDevice();
    dev->initialize();

    bool removed = false;
    dev->setModuleEventCallback([&](uint8_t slot, uint8_t event) {
        if (event == ModuleEvent::Removed) removed = true;
    });

    transport->setReadValue<uint8_t>(ModuleConfig::ModuleStatusList, 1, ModuleStatus::NotPresent);
    dev->update();

    EXPECT_TRUE(removed);
}

TEST_F(ETG5000Cov2Test, Update_ModuleError) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Present);
    dev = createDevice();
    dev->initialize();

    bool error_event = false;
    bool error_cb = false;
    dev->setModuleEventCallback([&](uint8_t slot, uint8_t event) {
        if (event == ModuleEvent::Error) error_event = true;
    });
    dev->setErrorCallback([&](uint8_t slot, uint16_t code) {
        error_cb = true;
    });

    transport->setReadValue<uint8_t>(ModuleConfig::ModuleStatusList, 1, ModuleStatus::Error);
    dev->update();

    EXPECT_TRUE(error_event);
    EXPECT_TRUE(error_cb);
}

TEST_F(ETG5000Cov2Test, Update_ModuleOperational) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Present);
    dev = createDevice();
    dev->initialize();

    bool op_event = false;
    dev->setModuleEventCallback([&](uint8_t slot, uint8_t event) {
        if (event == ModuleEvent::Operational) op_event = true;
    });

    transport->setReadValue<uint8_t>(ModuleConfig::ModuleStatusList, 1, ModuleStatus::Operational);
    dev->update();

    EXPECT_TRUE(op_event);
}

TEST_F(ETG5000Cov2Test, Update_DiagChange) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    bool diag_event = false;
    bool diag_cb = false;
    dev->setModuleEventCallback([&](uint8_t slot, uint8_t event) {
        if (event == ModuleEvent::DiagUpdated) diag_event = true;
    });
    dev->setDiagnosticCallback([&](uint8_t slot, uint16_t diag) {
        diag_cb = true;
    });

    // Change diag status
    transport->setReadValue<uint16_t>(ModuleDiag::DiagnosticStatus, 1, DiagStatus::Warning | DiagStatus::OverTemperature);
    dev->update();

    EXPECT_TRUE(diag_event);
    EXPECT_TRUE(diag_cb);
}

// ============================================================================
// checkConfigurationState — state change + callback
// ============================================================================

TEST_F(ETG5000Cov2Test, Update_ConfigStateChange) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    bool cfg_cb = false;
    dev->setConfigChangeCallback([&](const DeviceConfigState& c) {
        cfg_cb = true;
    });

    // Change config state to trigger checkConfigurationState
    transport->setReadValue<uint8_t>(ModuleConfig::ConfigurationState, 0, ConfigState::ConfigMismatch);
    dev->update();

    EXPECT_TRUE(cfg_cb);
}

// ============================================================================
// checkHotSwapEvents — HotSwap bit triggers re-scan
// ============================================================================

TEST_F(ETG5000Cov2Test, HotSwapEvent) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    bool cfg_cb = false;
    dev->setConfigChangeCallback([&](const DeviceConfigState& c) {
        cfg_cb = true;
    });

    // Send TxPDO with HotSwap bit set
    ModularInputPDO pdo{};
    pdo.statusword = StatuswordBits::HotSwapEvent;
    pdo.module_count = 1;
    pdo.config_state = ConfigState::ConfigurationValid;

    dev->processTxPDO(reinterpret_cast<const uint8_t*>(&pdo), sizeof(pdo));

    EXPECT_TRUE(cfg_cb);
}

// ============================================================================
// getDiagnostics — with modules
// ============================================================================

TEST_F(ETG5000Cov2Test, Diagnostics_WithModules) {
    setupModules(*transport, 2, ModuleType::DigitalInput, ModuleStatus::Operational);
    dev = createDevice();
    dev->initialize();

    // Add expected config with missing & mismatched
    std::vector<ModuleDescriptor> expected;
    expected.push_back({0, ModuleType::AnalogInput, 0, 0, false, false}); // mismatch
    expected.push_back({5, ModuleType::Unknown, 0, 0, false, false}); // missing

    dev->setExpectedConfiguration(expected);
    auto diag = dev->getDiagnostics();
    EXPECT_FALSE(diag.empty());
    EXPECT_NE(diag.find("Slot"), std::string::npos);
}

TEST_F(ETG5000Cov2Test, Diagnostics_ModuleWithError) {
    setupModules(*transport, 1, ModuleType::DigitalInput, ModuleStatus::Error);
    dev = createDevice();
    dev->initialize();

    auto diag = dev->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

// ============================================================================
// readModuleInput / writeModuleOutput — nullptr guard
// ============================================================================

TEST_F(ETG5000Cov2Test, ReadModuleInput_Nullptr) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_FALSE(dev->readModuleInput(0, nullptr, 4));
}

TEST_F(ETG5000Cov2Test, WriteModuleOutput_Nullptr) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_FALSE(dev->writeModuleOutput(0, nullptr, 4));
}

TEST_F(ETG5000Cov2Test, ReadModuleInput_ValidSlot) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    uint8_t buf[4] = {0};
    // Success depends on SDO — mock returns true
    auto result = dev->readModuleInput(0, buf, sizeof(buf));
    EXPECT_TRUE(result);
}

TEST_F(ETG5000Cov2Test, WriteModuleOutput_ValidSlot) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    uint8_t buf[2] = {1, 2};
    auto result = dev->writeModuleOutput(0, buf, sizeof(buf));
    EXPECT_TRUE(result);
}

// ============================================================================
// enableModule / resetModuleErrors / resetModuleToDefaults with working SDO
// ============================================================================

TEST_F(ETG5000Cov2Test, EnableModule_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->enableModule(0, true));
    EXPECT_TRUE(dev->enableModule(0, false));
}

TEST_F(ETG5000Cov2Test, ResetModuleErrors_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->resetModuleErrors(0));
}

TEST_F(ETG5000Cov2Test, ResetAllErrors_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->resetAllErrors());
}

TEST_F(ETG5000Cov2Test, ResetModuleToDefaults_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->resetModuleToDefaults(0));
}

// ============================================================================
// Diag accessors with populated modules
// ============================================================================

TEST_F(ETG5000Cov2Test, DiagAccessors_WithModules) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();

    EXPECT_NE(dev->getModuleDiagStatus(0), 0xFFFF);
    EXPECT_EQ(dev->getModuleLastError(0), 0);
    EXPECT_TRUE(std::isfinite(dev->getModuleTemperature(0)));
    EXPECT_TRUE(std::isfinite(dev->getModuleSupplyVoltage(0)));
    EXPECT_EQ(dev->getModuleErrorCount(0), 0u);
    EXPECT_EQ(dev->getModuleOperatingHours(0), 0u);
}

// ============================================================================
// Constructor with use_configured_addr
// ============================================================================

TEST_F(ETG5000Cov2Test, Constructor_UseConfiguredAddr) {
    setupModules(*transport, 1);
    sdo = std::make_unique<EtherCAT::SDO::SDOManager>(*transport);
    sdo->init();
    sdo->configureSlaveMailbox(1, 0x1000, 128, 0x1200, 128);
    auto d = std::make_unique<ModularDevice>(*sdo, 1, true);
    EXPECT_FALSE(d->isInitialized());
    d.reset();
    sdo->deinit();
}

// ============================================================================
// enableAutoConfiguration success
// ============================================================================

TEST_F(ETG5000Cov2Test, EnableAutoConfiguration_Success) {
    setupModules(*transport, 1);
    dev = createDevice();
    dev->initialize();
    EXPECT_TRUE(dev->enableAutoConfiguration(true));
    EXPECT_TRUE(dev->enableAutoConfiguration(false));
}
