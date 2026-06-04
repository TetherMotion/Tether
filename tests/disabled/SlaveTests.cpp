/**
 * @file SlaveTests.cpp
 * @brief Comprehensive Unit Tests for EtherCAT Slave Implementation
 *
 * @details
 * Tests all aspects of the slave implementation including:
 * - Slave core (ESC registers, FMMU, SyncManager, DC)
 * - State machine (AL state machine, CiA 402 state machine)
 * - Mailbox protocols (CoE, FoE, EoE)
 * - Profile implementations (CiA 401, 402, etc.)
 * - HAL implementations (Direct, FIFO, Network)
 * - Master-slave integration
 */

#include <gtest/gtest.h>

#include "slave/core/SlaveCore.hpp"
#include "slave/core/SlaveTypes.hpp"
#include "slave/logging/SlaveLogger.hpp"
#include "slave/hal/ISlaveHAL.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"
#include "slave/profiles/ProfileSlave.hpp"
#include "slave/profiles/CiA401Slave.hpp"
#include "slave/profiles/CiA402Slave.hpp"
#include "hal/LoopbackHAL.hpp"
#include "hal/FIFOHAL.hpp"

#include <memory>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>

namespace EtherCAT {
namespace slave {
namespace test {

// ============================================================================
// Test Fixtures
// ============================================================================

class SlaveCoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlaveConfig config;
        config.identity.vendorId = 0x12345678;
        config.identity.productCode = 0x00000001;
        config.identity.revisionNumber = 0x00010000;
        config.identity.serialNumber = 0x00000001;
        config.identity.deviceName = "Test Slave";
        config.maxFMMUs = 8;
        config.maxSyncManagers = 8;
        config.supportsDC = true;
        
        slave_ = createSlaveCore(config);
        slave_->start();
    }
    
    void TearDown() override {
        slave_->stop();
        slave_.reset();
    }
    
    std::unique_ptr<SlaveCore> slave_;
    
    // Helper to build EtherCAT frame
    std::vector<uint8_t> buildFrame(uint8_t cmd, uint16_t adp, uint16_t ado,
                                     const uint8_t* data, size_t dataLen) {
        std::vector<uint8_t> frame(14 + 2 + 10 + dataLen + 2);
        
        // Ethernet header
        std::memset(frame.data(), 0xFF, 6);  // Dest MAC (broadcast)
        std::memset(frame.data() + 6, 0x00, 6);  // Src MAC
        frame[12] = 0x88;  // EtherType
        frame[13] = 0xA4;
        
        // EtherCAT header
        uint16_t len = 10 + dataLen + 2;
        frame[14] = len & 0xFF;
        frame[15] = ((len >> 8) & 0x07) | 0x10;  // Type = 1
        
        // Datagram header
        frame[16] = cmd;
        frame[17] = 0;  // Index
        frame[18] = adp & 0xFF;
        frame[19] = (adp >> 8) & 0xFF;
        frame[20] = ado & 0xFF;
        frame[21] = (ado >> 8) & 0xFF;
        frame[22] = dataLen & 0xFF;
        frame[23] = (dataLen >> 8) & 0x07;  // No more datagrams
        frame[24] = 0;
        frame[25] = 0;
        
        // Data
        if (data && dataLen > 0) {
            std::memcpy(frame.data() + 26, data, dataLen);
        }
        
        // WKC
        frame[26 + dataLen] = 0;
        frame[27 + dataLen] = 0;
        
        return frame;
    }
};

class CiA402Test : public ::testing::Test {
protected:
    void SetUp() override {
        CiA402SlaveConfig config;
        config.identity.vendorId = 0x12345678;
        config.identity.productCode = 0x00000402;
        config.identity.deviceName = "Test Drive";
        config.supportedModes = 
            static_cast<uint32_t>(OperatingMode::ProfilePosition) |
            static_cast<uint32_t>(OperatingMode::ProfileVelocity) |
            static_cast<uint32_t>(OperatingMode::Homing) |
            static_cast<uint32_t>(OperatingMode::CyclicSyncPosition);
        config.maxVelocity = 10000;
        config.maxAcceleration = 50000;
        
        drive_ = createCiA402Slave(config);
    }
    
    void TearDown() override {
        drive_.reset();
    }
    
    std::unique_ptr<CiA402Slave> drive_;
};

class LoopbackTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create loopback HAL
        loopback_ = hal::createDirectLoopbackHAL();
        
        // Create and attach slave
        SlaveConfig config;
        config.identity.vendorId = 0x12345678;
        config.identity.productCode = 0x00000001;
        
        auto slave = createSlaveCore(config);
        slave->start();
        loopback_->attachSlave(std::move(slave));
        
        loopback_->init();
    }
    
    void TearDown() override {
        loopback_->deinit();
        loopback_.reset();
    }
    
    std::unique_ptr<hal::LoopbackHAL> loopback_;
};

// ============================================================================
// SlaveCore Tests
// ============================================================================

TEST_F(SlaveCoreTest, Initialization) {
    EXPECT_NE(slave_, nullptr);
    EXPECT_EQ(slave_->getCurrentState(), SlaveState::Init);
}

TEST_F(SlaveCoreTest, ConfiguredAddressWrite) {
    // Write configured address using APWR command
    uint8_t data[2] = {0x01, 0x00};  // Address 0x0001
    auto frame = buildFrame(0x02, 0x0000, 0x0010, data, 2);  // APWR to station address
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    EXPECT_TRUE(slave_->processFrame(frame.data(), frame.size(), 
                                      response.data(), respLen));
    EXPECT_EQ(slave_->getConfiguredAddress(), 0x0001);
}

TEST_F(SlaveCoreTest, BroadcastRead) {
    // BRD (0x07) to read type register
    auto frame = buildFrame(0x07, 0x0000, 0x0000, nullptr, 2);
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    EXPECT_TRUE(slave_->processFrame(frame.data(), frame.size(),
                                      response.data(), respLen));
    
    // Check WKC incremented
    uint16_t wkc = response[26] | (response[27] << 8);
    EXPECT_EQ(wkc, 1);
}

TEST_F(SlaveCoreTest, ALStateTransitions) {
    // Set configured address first
    slave_->setConfiguredAddress(0x0001);
    
    // Initial state should be INIT
    EXPECT_EQ(slave_->getCurrentState(), SlaveState::Init);
    
    // Request PREOP
    uint8_t data[2] = {0x02, 0x00};  // Request PreOp
    auto frame = buildFrame(0x05, 0x0001, 0x0120, data, 2);  // FPWR to AL Control
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    EXPECT_EQ(slave_->getCurrentState(), SlaveState::PreOp);
    
    // Request SafeOp
    data[0] = 0x04;
    frame = buildFrame(0x05, 0x0001, 0x0120, data, 2);
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    EXPECT_EQ(slave_->getCurrentState(), SlaveState::SafeOp);
    
    // Request Op
    data[0] = 0x08;
    frame = buildFrame(0x05, 0x0001, 0x0120, data, 2);
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    EXPECT_EQ(slave_->getCurrentState(), SlaveState::Op);
}

TEST_F(SlaveCoreTest, FMMUConfiguration) {
    // Configure FMMU0
    uint8_t fmmuData[16] = {
        0x00, 0x00, 0x10, 0x00,  // Logical start address: 0x00100000
        0x08, 0x00,               // Length: 8 bytes
        0x00,                     // Logical start bit
        0x07,                     // Logical end bit
        0x00, 0x10,               // Physical start address: 0x1000
        0x00,                     // Physical start bit
        0x03,                     // Read + Write enable
        0x01,                     // Enable
        0x00, 0x00, 0x00          // Reserved
    };
    
    auto frame = buildFrame(0x08, 0x0000, 0x0600, fmmuData, 16);  // BWR to FMMU0
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    EXPECT_TRUE(slave_->processFrame(frame.data(), frame.size(),
                                      response.data(), respLen));
}

TEST_F(SlaveCoreTest, SyncManagerConfiguration) {
    // Configure SM0 (Mailbox In)
    uint8_t smData[8] = {
        0x00, 0x10,  // Physical start address: 0x1000
        0x80, 0x00,  // Length: 128 bytes
        0x26,        // Control: Mailbox, read
        0x00,        // Status
        0x01,        // Enable
        0x00         // EC access
    };
    
    auto frame = buildFrame(0x08, 0x0000, 0x0800, smData, 8);  // BWR to SM0
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    EXPECT_TRUE(slave_->processFrame(frame.data(), frame.size(),
                                      response.data(), respLen));
}

TEST_F(SlaveCoreTest, DCTimestamp) {
    // Enable DC
    slave_->setConfiguredAddress(0x0001);
    
    // Read DC system time
    auto frame = buildFrame(0x04, 0x0001, 0x0910, nullptr, 8);  // FPRD DC time
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    EXPECT_TRUE(slave_->processFrame(frame.data(), frame.size(),
                                      response.data(), respLen));
}

TEST_F(SlaveCoreTest, SIIRead) {
    slave_->setConfiguredAddress(0x0001);
    slave_->buildDefaultSII();
    
    // Set SII address to 8 (Vendor ID)
    uint8_t addrData[2] = {0x08, 0x00};
    auto frame = buildFrame(0x05, 0x0001, 0x0504, addrData, 2);
    
    std::vector<uint8_t> response(frame.size());
    size_t respLen = 0;
    
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    
    // Trigger read
    uint8_t ctrlData[2] = {0x01, 0x00};  // Read command
    frame = buildFrame(0x05, 0x0001, 0x0502, ctrlData, 2);
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    
    // Read result
    frame = buildFrame(0x04, 0x0001, 0x0508, nullptr, 8);
    slave_->processFrame(frame.data(), frame.size(), response.data(), respLen);
    
    // Verify vendor ID in response
    uint32_t vendorId = response[26] | (response[27] << 8) |
                        (response[28] << 16) | (response[29] << 24);
    EXPECT_EQ(vendorId, 0x12345678);
}

TEST_F(SlaveCoreTest, Watchdog) {
    slave_->setConfiguredAddress(0x0001);
    
    // Enable watchdog
    slave_->enableWatchdog(true, 100);  // 100ms timeout
    
    // Simulate time passing without reset
    for (int i = 0; i < 150; i++) {
        slave_->simulate(1000000);  // 1ms per cycle
    }
    
    // Watchdog should have expired
    EXPECT_TRUE(slave_->isWatchdogExpired());
}

// ============================================================================
// CiA 402 State Machine Tests
// ============================================================================

TEST_F(CiA402Test, InitialState) {
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchOnDisabled);
}

TEST_F(CiA402Test, StateTransitionToReadyToSwitchOn) {
    // Send Shutdown command (switch on = 0, enable voltage = 1, quick stop = 1)
    drive_->setControlword(0x0006);
    EXPECT_EQ(drive_->getDriveState(), DriveState::ReadyToSwitchOn);
}

TEST_F(CiA402Test, StateTransitionToSwitchedOn) {
    // Shutdown
    drive_->setControlword(0x0006);
    
    // Switch on
    drive_->setControlword(0x0007);
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchedOn);
}

TEST_F(CiA402Test, StateTransitionToOperationEnabled) {
    // Full sequence to operation enabled
    drive_->setControlword(0x0006);  // Shutdown
    drive_->setControlword(0x0007);  // Switch on
    drive_->setControlword(0x000F);  // Enable operation
    
    EXPECT_EQ(drive_->getDriveState(), DriveState::OperationEnabled);
}

TEST_F(CiA402Test, QuickStop) {
    // Get to operation enabled
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    // Quick stop (clear quick stop bit)
    drive_->setControlword(0x000B);
    EXPECT_EQ(drive_->getDriveState(), DriveState::QuickStopActive);
}

TEST_F(CiA402Test, FaultAndReset) {
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    // Trigger fault
    drive_->triggerFault(0x3210);
    EXPECT_EQ(drive_->getDriveState(), DriveState::Fault);
    
    // Fault reset
    drive_->setControlword(0x0080);
    EXPECT_EQ(drive_->getDriveState(), DriveState::SwitchOnDisabled);
}

TEST_F(CiA402Test, OperatingModeChange) {
    // Initially in switch on disabled
    EXPECT_TRUE(drive_->setOperatingMode(OperatingMode::ProfileVelocity));
    
    // Try changing mode while in operation enabled (should fail)
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    EXPECT_FALSE(drive_->setOperatingMode(OperatingMode::Homing));
}

TEST_F(CiA402Test, ProfilePositionMotion) {
    // Get to operation enabled
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    // Set target position
    drive_->setTargetPosition(10000);
    
    // Simulate motion
    for (int i = 0; i < 1000; i++) {
        drive_->simulate(1000000);  // 1ms cycles
    }
    
    // Should have reached target (or be close)
    EXPECT_NEAR(drive_->getActualPosition(), 10000, 100);
    EXPECT_TRUE(drive_->isTargetReached());
}

TEST_F(CiA402Test, VelocityMode) {
    drive_->setOperatingMode(OperatingMode::ProfileVelocity);
    
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    drive_->setTargetVelocity(1000);
    
    // Simulate
    int32_t initialPos = drive_->getActualPosition();
    for (int i = 0; i < 100; i++) {
        drive_->simulate(10000000);  // 10ms
    }
    
    // Position should have increased
    EXPECT_GT(drive_->getActualPosition(), initialPos);
    EXPECT_NEAR(drive_->getActualVelocity(), 1000, 100);
}

TEST_F(CiA402Test, HomingMode) {
    drive_->setOperatingMode(OperatingMode::Homing);
    
    drive_->setControlword(0x0006);
    drive_->setControlword(0x0007);
    drive_->setControlword(0x000F);
    
    // Start homing
    drive_->startHoming();
    
    // Simulate homing
    for (int i = 0; i < 200; i++) {
        drive_->simulate(10000000);
    }
    
    EXPECT_TRUE(drive_->isHomingComplete());
    EXPECT_EQ(drive_->getActualPosition(), 0);
}

TEST_F(CiA402Test, PositionLimits) {
    CiA402SlaveConfig config;
    config.positionLimitMin = -1000;
    config.positionLimitMax = 1000;
    config.maxVelocity = 10000;
    
    auto drive = createCiA402Slave(config);
    
    drive->setControlword(0x0006);
    drive->setControlword(0x0007);
    drive->setControlword(0x000F);
    
    // Try to move beyond limit
    drive->setTargetPosition(5000);
    
    for (int i = 0; i < 500; i++) {
        drive->simulate(1000000);
    }
    
    // Should be clamped to limit
    EXPECT_LE(drive->getActualPosition(), 1000);
}

// ============================================================================
// Loopback HAL Tests
// ============================================================================

TEST_F(LoopbackTest, Initialization) {
    EXPECT_TRUE(loopback_->isLinkUp());
    EXPECT_EQ(loopback_->getSlaveCount(), 1);
}

TEST_F(LoopbackTest, SendReceive) {
    // Build a simple BRD frame
    std::vector<uint8_t> frame(60);
    std::memset(frame.data(), 0xFF, 6);  // Dest
    std::memset(frame.data() + 6, 0x00, 6);  // Src
    frame[12] = 0x88; frame[13] = 0xA4;  // EtherType
    
    uint16_t len = 12;
    frame[14] = len & 0xFF;
    frame[15] = ((len >> 8) & 0x07) | 0x10;
    
    frame[16] = 0x07;  // BRD
    frame[17] = 0;
    frame[18] = frame[19] = 0;  // Address
    frame[20] = 0; frame[21] = 0;  // Register 0x0000
    frame[22] = 2; frame[23] = 0;  // Length 2
    frame[24] = frame[25] = 0;
    frame[26] = frame[27] = 0;  // Data
    frame[28] = frame[29] = 0;  // WKC
    
    loopback_->sendFrame(frame.data(), 30);
    
    std::vector<uint8_t> response(60);
    size_t respLen = 0;
    
    EXPECT_TRUE(loopback_->receiveFrame(response.data(), response.size(), respLen, 100));
    
    // Check WKC
    uint16_t wkc = response[28] | (response[29] << 8);
    EXPECT_EQ(wkc, 1);
}

TEST_F(LoopbackTest, Statistics) {
    // Send some frames
    std::vector<uint8_t> frame(30);
    std::memset(frame.data(), 0xFF, 6);
    std::memset(frame.data() + 6, 0x00, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    frame[14] = 12; frame[15] = 0x10;
    frame[16] = 0x07;  // BRD
    
    for (int i = 0; i < 10; i++) {
        loopback_->sendFrame(frame.data(), 30);
        
        std::vector<uint8_t> response(60);
        size_t respLen;
        loopback_->receiveFrame(response.data(), response.size(), respLen, 100);
    }
    
    const auto& stats = loopback_->getStats();
    EXPECT_EQ(stats.framesSent, 10);
    EXPECT_EQ(stats.framesReceived, 10);
}

TEST_F(LoopbackTest, MultipleSlaves) {
    // Add more slaves
    for (int i = 0; i < 3; i++) {
        SlaveConfig config;
        config.identity.vendorId = 0x12345678;
        config.identity.productCode = 0x00000002 + i;
        
        auto slave = createSlaveCore(config);
        slave->start();
        loopback_->attachSlave(std::move(slave));
    }
    
    EXPECT_EQ(loopback_->getSlaveCount(), 4);
    
    // BRD should hit all slaves
    std::vector<uint8_t> frame(30);
    std::memset(frame.data(), 0xFF, 6);
    std::memset(frame.data() + 6, 0x00, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    frame[14] = 12; frame[15] = 0x10;
    frame[16] = 0x07;  // BRD
    
    loopback_->sendFrame(frame.data(), 30);
    
    std::vector<uint8_t> response(60);
    size_t respLen;
    loopback_->receiveFrame(response.data(), response.size(), respLen, 100);
    
    // WKC should be 4 (all slaves processed)
    uint16_t wkc = response[28] | (response[29] << 8);
    EXPECT_EQ(wkc, 4);
}

// ============================================================================
// Mailbox/CoE Tests
// ============================================================================

class CoETest : public ::testing::Test {
protected:
    void SetUp() override {
        objectDictionary_ = createObjectDictionary();
        coeHandler_ = createCoEHandler(objectDictionary_);
        
        // Add some test objects
        objectDictionary_->createEntry(0x1000, 0, "Device Type", ObjectAccessType::ReadOnly, 4);
        objectDictionary_->createEntry(0x1001, 0, "Error Register", ObjectAccessType::ReadOnly, 1);
        objectDictionary_->createEntry(0x2000, 0, "Test Variable", ObjectAccessType::ReadWrite, 4);
    }
    
    std::shared_ptr<IObjectDictionary> objectDictionary_;
    std::unique_ptr<IMailboxHandler> coeHandler_;
    
    // Build SDO upload request
    std::vector<uint8_t> buildSDOUpload(uint16_t index, uint8_t subindex) {
        std::vector<uint8_t> request(16);
        
        // Mailbox header (6 bytes)
        request[0] = 10;  // Length
        request[1] = 0;
        request[2] = 0;   // Address
        request[3] = 0;
        request[4] = 0;   // Channel/priority
        request[5] = 0x03;  // Type = CoE
        
        // CoE header (2 bytes)
        request[6] = 0x00;
        request[7] = 0x20;  // SDO Request
        
        // SDO data
        request[8] = 0x40;  // Upload initiate
        request[9] = index & 0xFF;
        request[10] = (index >> 8) & 0xFF;
        request[11] = subindex;
        
        return request;
    }
};

TEST_F(CoETest, SDOUploadExpedited) {
    auto request = buildSDOUpload(0x1001, 0);  // Error register (1 byte)
    
    uint8_t response[64];
    size_t respLen = 0;
    
    EXPECT_TRUE(coeHandler_->processRequest(request.data(), request.size(),
                                            response, respLen));
    EXPECT_GT(respLen, 0);
}

TEST_F(CoETest, SDODownloadExpedited) {
    std::vector<uint8_t> request(16);
    
    // Mailbox header
    request[0] = 10; request[1] = 0;
    request[5] = 0x03;
    
    // CoE header
    request[6] = 0x00; request[7] = 0x20;
    
    // SDO download (expedited, 4 bytes)
    request[8] = 0x23;  // Download initiate, e=1, s=1, n=0
    request[9] = 0x00;  // Index 0x2000
    request[10] = 0x20;
    request[11] = 0;    // Subindex
    request[12] = 0x12; // Data
    request[13] = 0x34;
    request[14] = 0x56;
    request[15] = 0x78;
    
    uint8_t response[64];
    size_t respLen = 0;
    
    EXPECT_TRUE(coeHandler_->processRequest(request.data(), request.size(),
                                            response, respLen));
    
    // Verify data was written
    auto entry = objectDictionary_->getEntry(0x2000, 0);
    ASSERT_NE(entry, nullptr);
    
    uint8_t readData[4];
    size_t readLen = 4;
    entry->readData(readData, readLen);
    
    EXPECT_EQ(readData[0], 0x12);
    EXPECT_EQ(readData[1], 0x34);
    EXPECT_EQ(readData[2], 0x56);
    EXPECT_EQ(readData[3], 0x78);
}

TEST_F(CoETest, SDOAbortOnNotFound) {
    auto request = buildSDOUpload(0x9999, 0);  // Non-existent object
    
    uint8_t response[64];
    size_t respLen = 0;
    
    EXPECT_TRUE(coeHandler_->processRequest(request.data(), request.size(),
                                            response, respLen));
    
    // Should return abort with "object not found" code
    uint8_t respCommand = response[8];
    EXPECT_EQ((respCommand >> 5) & 0x07, 4);  // Abort
}

// ============================================================================
// CiA 401 I/O Tests
// ============================================================================

TEST(CiA401Test, DigitalIO) {
    CiA401SlaveConfig config;
    config.digitalInputs = 16;
    config.digitalOutputs = 16;
    
    auto slave = createCiA401Slave(config);
    
    // Set digital inputs
    slave->setDigitalInput(0, true);
    slave->setDigitalInput(7, true);
    slave->setDigitalInput(15, true);
    
    EXPECT_TRUE(slave->getDigitalInput(0));
    EXPECT_TRUE(slave->getDigitalInput(7));
    EXPECT_TRUE(slave->getDigitalInput(15));
    EXPECT_FALSE(slave->getDigitalInput(1));
    
    // Digital outputs
    slave->setDigitalOutput(5, true);
    EXPECT_TRUE(slave->getDigitalOutput(5));
}

TEST(CiA401Test, AnalogIO) {
    CiA401SlaveConfig config;
    config.analogInputs = 4;
    config.analogOutputs = 2;
    
    auto slave = createCiA401Slave(config);
    
    // Set analog input
    slave->setAnalogInput(0, 16384);  // 50% of 15-bit range
    EXPECT_EQ(slave->getAnalogInput(0), 16384);
    
    // Analog output
    slave->setAnalogOutput(0, 32767);
    EXPECT_EQ(slave->getAnalogOutput(0), 32767);
}

// ============================================================================
// FIFO HAL Tests
// ============================================================================

TEST(FIFOTest, BridgeCreation) {
    auto bridge = hal::createFIFOBridge("/tmp/ethercat_test");
    
    EXPECT_TRUE(bridge->init());
    
    auto masterHAL = bridge->createMasterHAL();
    auto slaveHAL = bridge->createSlaveHAL();
    
    EXPECT_NE(masterHAL, nullptr);
    EXPECT_NE(slaveHAL, nullptr);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(IntegrationTest, MasterSlaveFullCycle) {
    // Create loopback with CiA 402 slave
    auto loopback = hal::createDirectLoopbackHAL();
    
    CiA402SlaveConfig driveConfig;
    driveConfig.identity.vendorId = 0x12345678;
    driveConfig.identity.productCode = 0x00000402;
    
    auto drive = createCiA402Slave(driveConfig);
    
    // Set up slave core
    SlaveConfig coreConfig;
    coreConfig.identity = driveConfig.identity;
    
    auto core = createSlaveCore(coreConfig);
    core->start();
    
    loopback->attachSlave(std::move(core));
    loopback->init();
    
    // Simulate master scanning for slaves
    std::vector<uint8_t> frame(60);
    std::memset(frame.data(), 0xFF, 6);
    std::memset(frame.data() + 6, 0x00, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    
    // BRD to read type
    frame[14] = 12; frame[15] = 0x10;
    frame[16] = 0x07;  // BRD
    frame[22] = 1; frame[23] = 0;  // 1 byte
    
    loopback->sendFrame(frame.data(), 30);
    
    std::vector<uint8_t> response(60);
    size_t respLen;
    bool received = loopback->receiveFrame(response.data(), response.size(), respLen, 100);
    
    EXPECT_TRUE(received);
    
    // Set station address
    frame[16] = 0x02;  // APWR
    frame[20] = 0x10; frame[21] = 0x00;  // Register 0x0010
    frame[22] = 2; frame[23] = 0;  // 2 bytes
    frame[26] = 0x01; frame[27] = 0x00;  // Address 0x0001
    
    loopback->sendFrame(frame.data(), 32);
    loopback->receiveFrame(response.data(), response.size(), respLen, 100);
    
    // Verify address was set
    auto* slave = loopback->getSlave(0);
    ASSERT_NE(slave, nullptr);
    EXPECT_EQ(slave->getConfiguredAddress(), 0x0001);
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(PerformanceTest, FrameProcessingLatency) {
    SlaveConfig config;
    auto slave = createSlaveCore(config);
    slave->start();
    
    // Build test frame
    std::vector<uint8_t> frame(60);
    std::memset(frame.data(), 0xFF, 6);
    std::memset(frame.data() + 6, 0x00, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    frame[14] = 12; frame[15] = 0x10;
    frame[16] = 0x07;  // BRD
    
    std::vector<uint8_t> response(60);
    size_t respLen;
    
    // Measure processing time
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 10000; i++) {
        slave->processFrame(frame.data(), 30, response.data(), respLen);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    double avgLatency = duration.count() / 10000.0;
    
    std::cout << "Average frame processing latency: " << avgLatency << " us" << std::endl;
    
    // Should be well under 100us for soft realtime
    EXPECT_LT(avgLatency, 100.0);
}

TEST(PerformanceTest, LoopbackThroughput) {
    auto loopback = hal::createDirectLoopbackHAL();
    
    SlaveConfig config;
    auto slave = createSlaveCore(config);
    slave->start();
    loopback->attachSlave(std::move(slave));
    loopback->init();
    
    std::vector<uint8_t> frame(60);
    std::memset(frame.data(), 0xFF, 6);
    std::memset(frame.data() + 6, 0x00, 6);
    frame[12] = 0x88; frame[13] = 0xA4;
    frame[14] = 12; frame[15] = 0x10;
    frame[16] = 0x07;
    
    std::vector<uint8_t> response(60);
    size_t respLen;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    int count = 0;
    for (int i = 0; i < 10000; i++) {
        loopback->sendFrame(frame.data(), 30);
        if (loopback->receiveFrame(response.data(), response.size(), respLen, 10)) {
            count++;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    double throughput = count * 1000.0 / duration.count();
    
    std::cout << "Loopback throughput: " << throughput << " frames/sec" << std::endl;
    std::cout << "Success rate: " << (count * 100.0 / 10000) << "%" << std::endl;
    
    EXPECT_GT(throughput, 1000.0);  // At least 1000 frames/sec
}

}  // namespace test
}  // namespace slave
}  // namespace EtherCAT

// Main
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
