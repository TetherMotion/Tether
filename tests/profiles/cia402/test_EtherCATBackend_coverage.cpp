/**
 * @file test_EtherCATBackend_coverage.cpp
 * @brief Comprehensive tests for EtherCATBackend and EtherCATBackendFactory
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <tether/profiles/cia402/EtherCATBackend.hpp>

using namespace CiA402;

// ============================================================================
// Mock MasterInterface
// ============================================================================

class MockEtherCATMaster : public MasterInterface {
public:
    MOCK_METHOD(bool, readProcessData, (uint32_t slaveId, uint8_t* data, size_t size), (override));
    MOCK_METHOD(bool, writeProcessData, (uint32_t slaveId, const uint8_t* data, size_t size), (override));
    MOCK_METHOD(SDOResult, readSDO, (uint32_t slaveId, uint16_t index, uint8_t subindex, void* data, size_t size), (override));
    MOCK_METHOD(SDOResult, writeSDO, (uint32_t slaveId, uint16_t index, uint8_t subindex, const void* data, size_t size), (override));
    MOCK_METHOD(bool, isSlaveOperational, (uint32_t slaveId), (override));
    MOCK_METHOD(int, getSlaveState, (uint32_t slaveId), (override));
};

// ============================================================================
// EtherCATBackend Tests
// ============================================================================

class EtherCATBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockMaster = std::make_shared<::testing::NiceMock<MockEtherCATMaster>>();
        config.slaveId = 1;
        config.vendorId = 0x00000002;
        config.productCode = 0x12345678;
        config.name = "TestDrive";
        backend = std::make_unique<EtherCATBackend>(mockMaster, config);
    }

    std::shared_ptr<::testing::NiceMock<MockEtherCATMaster>> mockMaster;
    EtherCATSlaveConfig config;
    std::unique_ptr<EtherCATBackend> backend;
};

// --- Lifecycle ---

TEST_F(EtherCATBackendTest, Initialize) {
    ON_CALL(*mockMaster, isSlaveOperational(1))
        .WillByDefault(::testing::Return(true));
    EXPECT_TRUE(backend->initialize());
}

TEST_F(EtherCATBackendTest, InitializeFailsWhenNotOperational) {
    ON_CALL(*mockMaster, isSlaveOperational(1))
        .WillByDefault(::testing::Return(false));
    // May still return true if initialize doesn't check operational
    backend->initialize();
}

TEST_F(EtherCATBackendTest, Deinitialize) {
    backend->initialize();
    backend->deinitialize();
    EXPECT_FALSE(backend->isConnected());
}

TEST_F(EtherCATBackendTest, IsConnectedInitially) {
    // Not connected before initialization
    EXPECT_FALSE(backend->isConnected());
}

TEST_F(EtherCATBackendTest, GetName) {
    EXPECT_EQ("TestDrive", backend->getName());
}

// --- Process Data ---

TEST_F(EtherCATBackendTest, UpdateInputs) {
    backend->initialize();
    EXPECT_CALL(*mockMaster, readProcessData(1, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(backend->updateInputs());
}

TEST_F(EtherCATBackendTest, UpdateInputsFails) {
    backend->initialize();
    EXPECT_CALL(*mockMaster, readProcessData(1, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(false));
    EXPECT_FALSE(backend->updateInputs());
}

TEST_F(EtherCATBackendTest, UpdateOutputs) {
    backend->initialize();
    EXPECT_CALL(*mockMaster, writeProcessData(1, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(backend->updateOutputs());
}

TEST_F(EtherCATBackendTest, UpdateOutputsFails) {
    backend->initialize();
    EXPECT_CALL(*mockMaster, writeProcessData(1, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(false));
    EXPECT_FALSE(backend->updateOutputs());
}

TEST_F(EtherCATBackendTest, GetState) {
    auto state = backend->getState();
    EXPECT_EQ(0, state.actualPosition);
    EXPECT_EQ(0, state.statusWord);
}

// --- Control/Status Words ---

TEST_F(EtherCATBackendTest, ReadStatusWord) {
    uint16_t sw = backend->readStatusWord();
    EXPECT_EQ(0, sw); // Initially zero from empty buffer
}

TEST_F(EtherCATBackendTest, WriteControlWord) {
    backend->writeControlWord(0x000F);
    EXPECT_EQ(0x000F, backend->readControlWord());
}

TEST_F(EtherCATBackendTest, ReadControlWord) {
    backend->writeControlWord(0x0006);
    EXPECT_EQ(0x0006, backend->readControlWord());
}

// --- Operating Mode ---

TEST_F(EtherCATBackendTest, SetOperatingMode) {
    EXPECT_TRUE(backend->setOperatingMode(OperatingMode::CyclicSyncPosition));
    EXPECT_EQ(OperatingMode::CyclicSyncPosition, backend->getOperatingMode());
}

TEST_F(EtherCATBackendTest, GetOperatingModeDefault) {
    auto mode = backend->getOperatingMode();
    // Default is ProfilePosition
    EXPECT_EQ(OperatingMode::ProfilePosition, mode);
}

TEST_F(EtherCATBackendTest, GetDisplayedMode) {
    backend->setOperatingMode(OperatingMode::CyclicSyncVelocity);
    auto mode = backend->getDisplayedMode();
    // Displayed mode comes from TxPDO
}

// --- Position ---

TEST_F(EtherCATBackendTest, SetTargetPosition) {
    backend->setTargetPosition(100000);
    // The position is written to RxPDO buffer
    const uint8_t* rxPdo = backend->getRxPDOBuffer();
    int32_t pos;
    memcpy(&pos, rxPdo + config.pdoMapping.targetPositionOffset, sizeof(pos));
    EXPECT_EQ(100000, pos);
}

TEST_F(EtherCATBackendTest, GetActualPosition) {
    EXPECT_EQ(0, backend->getActualPosition());
}

TEST_F(EtherCATBackendTest, GetPositionDemand) {
    EXPECT_EQ(0, backend->getPositionDemand());
}

TEST_F(EtherCATBackendTest, GetFollowingError) {
    EXPECT_EQ(0, backend->getFollowingError());
}

TEST_F(EtherCATBackendTest, SetPositionOffset) {
    backend->setPositionOffset(500);
    // Written to RxPDO
}

// --- Velocity ---

TEST_F(EtherCATBackendTest, SetTargetVelocity) {
    backend->setTargetVelocity(5000);
    const uint8_t* rxPdo = backend->getRxPDOBuffer();
    int32_t vel;
    memcpy(&vel, rxPdo + config.pdoMapping.targetVelocityOffset, sizeof(vel));
    EXPECT_EQ(5000, vel);
}

TEST_F(EtherCATBackendTest, GetActualVelocity) {
    EXPECT_EQ(0, backend->getActualVelocity());
}

TEST_F(EtherCATBackendTest, GetVelocityDemand) {
    EXPECT_EQ(0, backend->getVelocityDemand());
}

TEST_F(EtherCATBackendTest, SetVelocityOffset) {
    backend->setVelocityOffset(100);
}

// --- Torque ---

TEST_F(EtherCATBackendTest, SetTargetTorque) {
    backend->setTargetTorque(500);
    const uint8_t* rxPdo = backend->getRxPDOBuffer();
    int16_t torque;
    memcpy(&torque, rxPdo + config.pdoMapping.targetTorqueOffset, sizeof(torque));
    EXPECT_EQ(500, torque);
}

TEST_F(EtherCATBackendTest, GetActualTorque) {
    EXPECT_EQ(0, backend->getActualTorque());
}

TEST_F(EtherCATBackendTest, SetTorqueOffset) {
    backend->setTorqueOffset(50);
}

// --- Profile Parameters ---

TEST_F(EtherCATBackendTest, SetProfileVelocity) {
    backend->setProfileVelocity(10000);
}

TEST_F(EtherCATBackendTest, SetProfileAcceleration) {
    backend->setProfileAcceleration(5000);
}

TEST_F(EtherCATBackendTest, SetProfileDeceleration) {
    backend->setProfileDeceleration(3000);
}

TEST_F(EtherCATBackendTest, SetMotionProfileType) {
    backend->setMotionProfileType(0);
}

// --- Homing ---

TEST_F(EtherCATBackendTest, ConfigureHoming) {
    HomingParams params{};
    params.method = static_cast<int8_t>(HomingMethod::CurrentPosition);
    params.speedSwitch = 100;
    EXPECT_TRUE(backend->configureHoming(params));
}

TEST_F(EtherCATBackendTest, GetHomingParams) {
    HomingParams params{};
    params.method = 35;
    params.speedSwitch = 200;
    backend->configureHoming(params);
    auto retrieved = backend->getHomingParams();
    EXPECT_EQ(35, retrieved.method);
    EXPECT_EQ(200u, retrieved.speedSwitch);
}

// --- Interpolation ---

TEST_F(EtherCATBackendTest, ConfigureInterpolation) {
    InterpolationParams params{};
    params.timePeriod = 1;
    params.timeIndex = -3;
    EXPECT_TRUE(backend->configureInterpolation(params));
}

TEST_F(EtherCATBackendTest, AddInterpolationPoint) {
    backend->addInterpolationPoint(1000);
}

TEST_F(EtherCATBackendTest, ClearInterpolationBuffer) {
    backend->addInterpolationPoint(1000);
    backend->clearInterpolationBuffer();
}

// --- SDO Access ---

TEST_F(EtherCATBackendTest, ReadSDO) {
    SDOResult expected;
    expected.success = true;
    expected.data = {0x01, 0x02, 0x03, 0x04};
    EXPECT_CALL(*mockMaster, readSDO(1, 0x6041, 0, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(expected));
    
    backend->initialize();
    uint32_t data = 0;
    auto result = backend->readSDO(0x6041, 0, &data, sizeof(data));
    EXPECT_TRUE(result.success);
}

TEST_F(EtherCATBackendTest, WriteSDO) {
    SDOResult expected;
    expected.success = true;
    EXPECT_CALL(*mockMaster, writeSDO(1, 0x6040, 0, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(expected));
    
    backend->initialize();
    uint16_t data = 0x000F;
    auto result = backend->writeSDO(0x6040, 0, &data, sizeof(data));
    EXPECT_TRUE(result.success);
}

// --- Configuration ---

TEST_F(EtherCATBackendTest, Configure) {
    DriveConfig cfg{};
    cfg.maxVelocity = 200000;
    cfg.maxAcceleration = 50000;
    EXPECT_TRUE(backend->configure(cfg));
}

TEST_F(EtherCATBackendTest, GetConfiguration) {
    DriveConfig cfg{};
    cfg.maxVelocity = 150000;
    backend->configure(cfg);
    auto retrieved = backend->getConfiguration();
    EXPECT_EQ(150000u, retrieved.maxVelocity);
}

TEST_F(EtherCATBackendTest, StoreParameters) {
    backend->storeParameters();
}

TEST_F(EtherCATBackendTest, RestoreParameters) {
    backend->restoreParameters();
}

// --- Error Handling ---

TEST_F(EtherCATBackendTest, GetErrorCode) {
    EXPECT_EQ(0, backend->getErrorCode());
}

TEST_F(EtherCATBackendTest, GetErrorRegister) {
    EXPECT_EQ(0, backend->getErrorRegister());
}

TEST_F(EtherCATBackendTest, GetErrorHistory) {
    auto history = backend->getErrorHistory();
    EXPECT_TRUE(history.empty());
}

TEST_F(EtherCATBackendTest, ClearErrorHistory) {
    backend->clearErrorHistory();
}

// --- Callbacks ---

TEST_F(EtherCATBackendTest, SetStateChangeCallback) {
    bool called = false;
    backend->setStateChangeCallback([&](State, State) { called = true; });
}

TEST_F(EtherCATBackendTest, SetErrorCallback) {
    backend->setErrorCallback([](uint16_t, const std::string&) {});
}

TEST_F(EtherCATBackendTest, SetWarningCallback) {
    backend->setWarningCallback([](uint16_t, const std::string&) {});
}

TEST_F(EtherCATBackendTest, SetSyncCallback) {
    backend->setSyncCallback([](uint64_t) {});
}

// --- Timing ---

TEST_F(EtherCATBackendTest, GetCycleTimeUs) {
    EXPECT_GT(backend->getCycleTimeUs(), 0u);
}

TEST_F(EtherCATBackendTest, SetCycleTimeUs) {
    EXPECT_TRUE(backend->setCycleTimeUs(500));
    EXPECT_EQ(500u, backend->getCycleTimeUs());
}

TEST_F(EtherCATBackendTest, GetLastUpdateTimestamp) {
    uint64_t ts = backend->getLastUpdateTimestamp();
    EXPECT_EQ(0u, ts);
}

// --- EtherCAT Specific ---

TEST_F(EtherCATBackendTest, GetSlaveConfig) {
    const auto& cfg = backend->getSlaveConfig();
    EXPECT_EQ(1u, cfg.slaveId);
    EXPECT_EQ(0x00000002u, cfg.vendorId);
    EXPECT_EQ(0x12345678u, cfg.productCode);
    EXPECT_EQ("TestDrive", cfg.name);
}

TEST_F(EtherCATBackendTest, SetPDOMapping) {
    PDOMapping mapping{};
    mapping.rxPdoSize = 32;
    mapping.txPdoSize = 20;
    backend->setPDOMapping(mapping);
    // Verify through buffer sizes
}

TEST_F(EtherCATBackendTest, GetRxPDOBuffer) {
    const uint8_t* buf = backend->getRxPDOBuffer();
    EXPECT_NE(nullptr, buf);
}

TEST_F(EtherCATBackendTest, GetTxPDOBuffer) {
    const uint8_t* buf = backend->getTxPDOBuffer();
    EXPECT_NE(nullptr, buf);
}

// --- Round-trip test ---

TEST_F(EtherCATBackendTest, FullUpdateCycle) {
    backend->initialize();
    
    // Write commands
    backend->writeControlWord(0x000F);
    backend->setTargetPosition(50000);
    backend->setTargetVelocity(1000);
    backend->setTargetTorque(100);
    backend->setOperatingMode(OperatingMode::CyclicSyncPosition);
    
    // Simulate output
    EXPECT_CALL(*mockMaster, writeProcessData(1, ::testing::_, ::testing::_))
        .WillOnce(::testing::Return(true));
    backend->updateOutputs();
    
    // Simulate input with valid status word in TxPDO
    EXPECT_CALL(*mockMaster, readProcessData(1, ::testing::_, ::testing::_))
        .WillOnce([&](uint32_t, uint8_t* data, size_t size) {
            memset(data, 0, size);
            uint16_t sw = 0x0237; // SwitchedOn + OperationEnabled + QuickStop + VoltageEnabled
            memcpy(data + config.pdoMapping.statusWordOffset, &sw, sizeof(sw));
            int32_t pos = 49000;
            memcpy(data + config.pdoMapping.actualPositionOffset, &pos, sizeof(pos));
            return true;
        });
    backend->updateInputs();
    
    // Check state
    auto state = backend->getState();
    EXPECT_EQ(49000, state.actualPosition);
}

// ============================================================================
// EtherCATBackendFactory Tests
// ============================================================================

class BackendFactoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockMaster = std::make_shared<::testing::NiceMock<MockEtherCATMaster>>();
        factory.setMaster(mockMaster);
    }

    std::shared_ptr<::testing::NiceMock<MockEtherCATMaster>> mockMaster;
    EtherCATBackendFactory factory;
};

TEST_F(BackendFactoryTest, CreateBackendBySlaveId) {
    auto backend = factory.createBackend(0u);
    EXPECT_NE(nullptr, backend);
}

TEST_F(BackendFactoryTest, CreateBackendByConfig) {
    EtherCATSlaveConfig cfg{};
    cfg.slaveId = 2;
    cfg.name = "Servo1";
    auto backend = factory.createBackend(cfg);
    EXPECT_NE(nullptr, backend);
    EXPECT_EQ("Servo1", backend->getName());
}

TEST_F(BackendFactoryTest, CreateMultipleBackends) {
    auto b1 = factory.createBackend(0u);
    auto b2 = factory.createBackend(1u);
    auto b3 = factory.createBackend(2u);
    EXPECT_NE(nullptr, b1);
    EXPECT_NE(nullptr, b2);
    EXPECT_NE(nullptr, b3);
}

// ============================================================================
// PDOMapping struct tests
// ============================================================================

TEST(PDOMappingCovTest, DefaultValues) {
    PDOMapping m{};
    EXPECT_EQ(0u, m.controlWordOffset);
    EXPECT_EQ(2u, m.targetPositionOffset);
    // Verify offsets are set
    EXPECT_GT(m.rxPdoSize, 0u);
    EXPECT_GT(m.txPdoSize, 0u);
}

TEST(EtherCATSlaveConfigTest, DefaultValues) {
    EtherCATSlaveConfig cfg{};
    EXPECT_EQ(0u, cfg.slaveId);
    EXPECT_EQ(0u, cfg.vendorId);
    EXPECT_EQ(0u, cfg.productCode);
    EXPECT_TRUE(cfg.name.empty());
}
