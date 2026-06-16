/**
 * @file SlaveIntegrationTests.cpp
 * @brief Comprehensive integration tests for EtherCAT slave implementation
 * 
 * Tests master-slave communication scenarios including:
 * - State machine transitions
 * - SDO read/write
 * - PDO mapping and transfer
 * - Distributed clock synchronization
 * - Profile-specific functionality
 * - FIFO communication
 */

#include "slave/SlaveCore.hpp"
#include "slave/ProfileSlave.hpp"
#include "slave/profiles/CiA402Slave.hpp"
#include "slave/profiles/CiA401Slave.hpp"
#include "slave/hal/ISlaveHAL.hpp"
#include "slave/mailbox/CoEHandler.hpp"
#include "pcap/PcapLogger.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <cstring>

namespace EtherCAT {
namespace Slave {
namespace Test {

// ============================================================================
// Test Fixtures
// ============================================================================

class SlaveCoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlaveConfig config;
        config.vendorId = 0x00000001;
        config.productCode = 0x00001000;
        config.revisionNumber = 0x00000001;
        config.serialNumber = 0x12345678;
        config.memorySize = 0x2000;
        config.maxFmmus = 4;
        config.maxSyncManagers = 4;
        config.mailboxOut = {0x1000, 256, 0};
        config.mailboxIn = {0x1100, 256, 0};
        config.supportedMailbox = {true, true, true, true, true, true};
        
        slave_ = std::make_unique<SlaveCore>(config);
    }
    
    void TearDown() override {
        slave_.reset();
    }
    
    std::unique_ptr<SlaveCore> slave_;
};

class MasterSlaveIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create slave
        SlaveConfig slaveConfig;
        slaveConfig.vendorId = 0x00000002;
        slaveConfig.productCode = 0x00000402;  // CiA 402
        slaveConfig.revisionNumber = 0x00000001;
        slaveConfig.serialNumber = 0x87654321;
        slaveConfig.memorySize = 0x4000;
        slaveConfig.maxFmmus = 4;
        slaveConfig.maxSyncManagers = 4;
        slaveConfig.mailboxOut = {0x1000, 512, 0};
        slaveConfig.mailboxIn = {0x1200, 512, 0};
        slaveConfig.supportedMailbox = {true, true, true, true, true, true};
        
        slave_ = std::make_unique<SlaveCore>(slaveConfig);
        
        // Create logger
        logger_ = createMemoryPcapLogger(1024 * 1024);
        
        // Configure SyncManagers for mailbox
        SyncManagerConfig sm0;
        sm0.startAddress = 0x1000;
        sm0.length = 512;
        sm0.direction = SMDirection::Output;
        sm0.mode = SMMode::Mailbox;
        sm0.enabled = true;
        slave_->configureSyncManager(0, sm0);
        
        SyncManagerConfig sm1;
        sm1.startAddress = 0x1200;
        sm1.length = 512;
        sm1.direction = SMDirection::Input;
        sm1.mode = SMMode::Mailbox;
        sm1.enabled = true;
        slave_->configureSyncManager(1, sm1);
    }
    
    void TearDown() override {
        slave_.reset();
        logger_.reset();
    }
    
    // Build an EtherCAT frame with a single datagram
    std::vector<uint8_t> buildFrame(uint8_t cmd, uint32_t address, 
                                    const uint8_t* data, uint16_t length) {
        // Ethernet header (14 bytes)
        std::vector<uint8_t> frame(14 + 2 + 10 + length + 2);  // ETH + ECAT header + DG header + data + WKC
        
        // Destination MAC (broadcast)
        std::memset(frame.data(), 0xFF, 6);
        
        // Source MAC
        frame[6] = 0x00; frame[7] = 0x01; frame[8] = 0x02;
        frame[9] = 0x03; frame[10] = 0x04; frame[11] = 0x05;
        
        // EtherType (0x88A4 = EtherCAT)
        frame[12] = 0xA4;
        frame[13] = 0x88;
        
        // EtherCAT header (length | type)
        uint16_t ecatHeader = (10 + length + 2) | (0x01 << 12);  // Type 1 = EtherCAT PDU
        frame[14] = ecatHeader & 0xFF;
        frame[15] = (ecatHeader >> 8) & 0xFF;
        
        // Datagram header
        frame[16] = cmd;  // Command
        frame[17] = 0;    // Index
        frame[18] = address & 0xFF;
        frame[19] = (address >> 8) & 0xFF;
        frame[20] = (address >> 16) & 0xFF;
        frame[21] = (address >> 24) & 0xFF;
        
        uint16_t lenFlags = length;  // No more datagrams
        frame[22] = lenFlags & 0xFF;
        frame[23] = (lenFlags >> 8) & 0xFF;
        
        frame[24] = 0;  // IRQ
        frame[25] = 0;
        
        // Data
        if (data && length > 0) {
            std::memcpy(frame.data() + 26, data, length);
        }
        
        // Working counter (0 initially)
        frame[26 + length] = 0;
        frame[27 + length] = 0;
        
        return frame;
    }
    
    std::unique_ptr<SlaveCore> slave_;
    std::shared_ptr<IPcapLogger> logger_;
};

// ============================================================================
// State Machine Tests
// ============================================================================

TEST_F(SlaveCoreTest, InitialState) {
    EXPECT_EQ(slave_->getState(), SlaveState::Init);
    EXPECT_EQ(slave_->getALStatusCode(), ALStatusCode::NoError);
}

TEST_F(SlaveCoreTest, StateTransitionInitToPreOp) {
    // Configure mailbox SyncManagers
    SyncManagerConfig sm0;
    sm0.startAddress = 0x1000;
    sm0.length = 256;
    sm0.direction = SMDirection::Output;
    sm0.mode = SMMode::Mailbox;
    sm0.enabled = true;
    EXPECT_TRUE(slave_->configureSyncManager(0, sm0));
    
    SyncManagerConfig sm1;
    sm1.startAddress = 0x1100;
    sm1.length = 256;
    sm1.direction = SMDirection::Input;
    sm1.mode = SMMode::Mailbox;
    sm1.enabled = true;
    EXPECT_TRUE(slave_->configureSyncManager(1, sm1));
    
    // Request PreOp
    EXPECT_TRUE(slave_->requestStateChange(SlaveState::PreOp));
    EXPECT_EQ(slave_->getState(), SlaveState::PreOp);
}

TEST_F(SlaveCoreTest, StateTransitionPreOpToSafeOp) {
    // First get to PreOp
    SyncManagerConfig sm0;
    sm0.startAddress = 0x1000;
    sm0.length = 256;
    sm0.direction = SMDirection::Output;
    sm0.mode = SMMode::Mailbox;
    sm0.enabled = true;
    slave_->configureSyncManager(0, sm0);
    
    SyncManagerConfig sm1;
    sm1.startAddress = 0x1100;
    sm1.length = 256;
    sm1.direction = SMDirection::Input;
    sm1.mode = SMMode::Mailbox;
    sm1.enabled = true;
    slave_->configureSyncManager(1, sm1);
    
    slave_->requestStateChange(SlaveState::PreOp);
    
    // Configure PDO SyncManagers
    SyncManagerConfig sm2;
    sm2.startAddress = 0x1200;
    sm2.length = 32;
    sm2.direction = SMDirection::Output;
    sm2.mode = SMMode::Buffered;
    sm2.enabled = true;
    slave_->configureSyncManager(2, sm2);
    
    // Request SafeOp
    EXPECT_TRUE(slave_->requestStateChange(SlaveState::SafeOp));
    EXPECT_EQ(slave_->getState(), SlaveState::SafeOp);
}

TEST_F(SlaveCoreTest, InvalidStateTransition) {
    // Cannot go directly from Init to Op
    EXPECT_FALSE(slave_->requestStateChange(SlaveState::Op));
    EXPECT_NE(slave_->getALStatusCode(), ALStatusCode::NoError);
}

TEST_F(SlaveCoreTest, StateTransitionBackToInit) {
    // Get to PreOp first
    SyncManagerConfig sm0;
    sm0.startAddress = 0x1000;
    sm0.length = 256;
    sm0.direction = SMDirection::Output;
    sm0.mode = SMMode::Mailbox;
    sm0.enabled = true;
    slave_->configureSyncManager(0, sm0);
    
    SyncManagerConfig sm1;
    sm1.startAddress = 0x1100;
    sm1.length = 256;
    sm1.direction = SMDirection::Input;
    sm1.mode = SMMode::Mailbox;
    sm1.enabled = true;
    slave_->configureSyncManager(1, sm1);
    
    slave_->requestStateChange(SlaveState::PreOp);
    EXPECT_EQ(slave_->getState(), SlaveState::PreOp);
    
    // Go back to Init
    EXPECT_TRUE(slave_->requestStateChange(SlaveState::Init));
    EXPECT_EQ(slave_->getState(), SlaveState::Init);
}

// ============================================================================
// Memory Access Tests
// ============================================================================

TEST_F(SlaveCoreTest, MemoryReadWrite) {
    uint8_t writeData[] = {0x11, 0x22, 0x33, 0x44};
    EXPECT_TRUE(slave_->writeMemory(0x1000, writeData, sizeof(writeData)));
    
    uint8_t readData[4];
    EXPECT_TRUE(slave_->readMemory(0x1000, readData, sizeof(readData)));
    
    EXPECT_EQ(std::memcmp(writeData, readData, sizeof(writeData)), 0);
}

TEST_F(SlaveCoreTest, MemoryAccessOutOfBounds) {
    uint8_t data[4];
    EXPECT_FALSE(slave_->readMemory(0x10000, data, sizeof(data)));
    EXPECT_FALSE(slave_->writeMemory(0x10000, data, sizeof(data)));
}

// ============================================================================
// FMMU Tests
// ============================================================================

TEST_F(SlaveCoreTest, FMMUConfiguration) {
    FMMUConfig fmmu;
    fmmu.logicalStartAddress = 0x00010000;
    fmmu.length = 32;
    fmmu.logicalStartBit = 0;
    fmmu.logicalEndBit = 7;
    fmmu.physicalStartAddress = 0x1200;
    fmmu.physicalStartBit = 0;
    fmmu.readEnabled = true;
    fmmu.writeEnabled = true;
    fmmu.enabled = true;
    
    EXPECT_TRUE(slave_->configureFMMU(0, fmmu));
    
    const FMMUConfig* retrieved = slave_->getFMMU(0);
    EXPECT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->logicalStartAddress, 0x00010000);
    EXPECT_EQ(retrieved->length, 32);
    EXPECT_TRUE(retrieved->enabled);
}

TEST_F(SlaveCoreTest, FMMULogicalAddressMapping) {
    // Configure FMMU
    FMMUConfig fmmu;
    fmmu.logicalStartAddress = 0x00010000;
    fmmu.length = 4;
    fmmu.physicalStartAddress = 0x1200;
    fmmu.readEnabled = true;
    fmmu.writeEnabled = true;
    fmmu.enabled = true;
    slave_->configureFMMU(0, fmmu);
    
    // Write via logical address
    uint8_t writeData[] = {0xAA, 0xBB, 0xCC, 0xDD};
    EXPECT_TRUE(slave_->processFMMU(0, 0x00010000, writeData, 4, true));
    
    // Verify via physical address
    uint8_t readData[4];
    EXPECT_TRUE(slave_->readMemory(0x1200, readData, 4));
    EXPECT_EQ(std::memcmp(writeData, readData, 4), 0);
}

// ============================================================================
// SyncManager Tests
// ============================================================================

TEST_F(SlaveCoreTest, SyncManagerConfiguration) {
    SyncManagerConfig sm;
    sm.startAddress = 0x1000;
    sm.length = 256;
    sm.direction = SMDirection::Output;
    sm.mode = SMMode::Mailbox;
    sm.enabled = true;
    
    EXPECT_TRUE(slave_->configureSyncManager(0, sm));
    
    const SyncManagerConfig* retrieved = slave_->getSyncManager(0);
    EXPECT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->startAddress, 0x1000);
    EXPECT_EQ(retrieved->length, 256);
    EXPECT_EQ(retrieved->direction, SMDirection::Output);
}

// ============================================================================
// PDO Tests
// ============================================================================

TEST_F(SlaveCoreTest, PDOMapping) {
    // Add TxPDO entries
    std::vector<PDOEntry> txEntries = {
        {0x6000, 0x01, 8},   // Status word
        {0x6000, 0x02, 32},  // Position
    };
    EXPECT_TRUE(slave_->addTxPDO(0x1A00, txEntries));
    
    // Add RxPDO entries
    std::vector<PDOEntry> rxEntries = {
        {0x7000, 0x01, 8},   // Control word
        {0x7000, 0x02, 32},  // Target position
    };
    EXPECT_TRUE(slave_->addRxPDO(0x1600, rxEntries));
    
    EXPECT_EQ(slave_->getTxPDOSize(), 5);  // 8 + 32 bits = 40 bits = 5 bytes
    EXPECT_EQ(slave_->getRxPDOSize(), 5);
}

// ============================================================================
// Distributed Clock Tests
// ============================================================================

TEST_F(SlaveCoreTest, DCConfiguration) {
    DCConfig dc;
    dc.sync0CycleTime = 1000000;  // 1ms
    dc.sync0ShiftTime = 100000;
    dc.sync1CycleTime = 0;
    dc.systemTimeOffset = 0;
    
    EXPECT_TRUE(slave_->configureDC(dc));
    EXPECT_TRUE(slave_->isDCEnabled());
    
    const DCConfig& retrieved = slave_->getDCConfig();
    EXPECT_EQ(retrieved.sync0CycleTime, 1000000);
}

TEST_F(SlaveCoreTest, DCTimeUpdate) {
    DCConfig dc;
    dc.sync0CycleTime = 1000000;
    slave_->configureDC(dc);
    
    uint64_t masterTime = 1000000000;  // 1 second
    slave_->updateDCTime(masterTime);
    
    EXPECT_EQ(slave_->getLocalTime(), masterTime);
}

// ============================================================================
// Frame Processing Tests
// ============================================================================

TEST_F(MasterSlaveIntegrationTest, ProcessAPRDFrame) {
    // Build APRD frame to read AL Status (0x0130)
    uint8_t data[4] = {0};
    auto frame = buildFrame(0x01, 0x0130, data, 4);  // APRD, address 0x0130
    
    uint8_t response[1518];
    size_t responseLen = sizeof(response);
    
    // Process at position 0 (auto-increment)
    EXPECT_TRUE(slave_->processFrame(frame.data() + 14, frame.size() - 14, 
                                     response, responseLen));
    
    // Check working counter incremented
    EXPECT_GT(responseLen, 0);
}

TEST_F(MasterSlaveIntegrationTest, ProcessAPWRFrame) {
    // Build APWR frame to write to AL Control (0x0120) - request PreOp
    uint8_t data[2] = {0x02, 0x00};  // PreOp state
    auto frame = buildFrame(0x02, 0x0120, data, 2);  // APWR, address 0x0120
    
    uint8_t response[1518];
    size_t responseLen = sizeof(response);
    
    EXPECT_TRUE(slave_->processFrame(frame.data() + 14, frame.size() - 14,
                                     response, responseLen));
    
    // Verify state change
    EXPECT_EQ(slave_->getState(), SlaveState::PreOp);
}

TEST_F(MasterSlaveIntegrationTest, ProcessLRWFrame) {
    // Configure FMMU for LRW
    FMMUConfig fmmu;
    fmmu.logicalStartAddress = 0x00010000;
    fmmu.length = 8;
    fmmu.physicalStartAddress = 0x1400;
    fmmu.readEnabled = true;
    fmmu.writeEnabled = true;
    fmmu.enabled = true;
    slave_->configureFMMU(0, fmmu);
    
    // Write some data to physical memory
    uint8_t physData[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    slave_->writeMemory(0x1400, physData, 8);
    
    // Build LRW frame
    uint8_t lrwData[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    auto frame = buildFrame(0x0C, 0x00010000, lrwData, 8);  // LRW
    
    uint8_t response[1518];
    size_t responseLen = sizeof(response);
    
    EXPECT_TRUE(slave_->processFrame(frame.data() + 14, frame.size() - 14,
                                     response, responseLen));
}

// ============================================================================
// EEPROM Tests
// ============================================================================

TEST_F(SlaveCoreTest, EEPROMRead) {
    uint16_t value;
    
    // Read vendor ID (words 0-1)
    EXPECT_TRUE(slave_->readEEPROM(0, value));
    EXPECT_EQ(value, 0x0001);  // Low word of vendor ID
}

TEST_F(SlaveCoreTest, EEPROMWrite) {
    uint16_t writeValue = 0x1234;
    EXPECT_TRUE(slave_->writeEEPROM(0x100, writeValue));
    
    uint16_t readValue;
    EXPECT_TRUE(slave_->readEEPROM(0x100, readValue));
    EXPECT_EQ(readValue, writeValue);
}

// ============================================================================
// CoE Handler Tests
// ============================================================================

class CoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler_ = std::make_unique<CoEHandler>();
        
        // Add some test objects
        ODEntry entry1;
        entry1.index = 0x1000;
        entry1.subIndex = 0;
        entry1.name = "Device Type";
        entry1.dataType = 0x0007;  // UNSIGNED32
        entry1.dataSize = 4;
        entry1.maxSize = 4;
        entry1.readable = true;
        entry1.writable = false;
        entry1.data = {0x92, 0x01, 0x02, 0x00};  // CiA 402
        handler_->addObject(entry1);
        
        ODEntry entry2;
        entry2.index = 0x6060;
        entry2.subIndex = 0;
        entry2.name = "Modes of Operation";
        entry2.dataType = 0x0002;  // INTEGER8
        entry2.dataSize = 1;
        entry2.maxSize = 1;
        entry2.readable = true;
        entry2.writable = true;
        entry2.data = {0x00};
        handler_->addObject(entry2);
    }
    
    std::unique_ptr<CoEHandler> handler_;
};

TEST_F(CoEHandlerTest, SDOUploadExpedited) {
    // Build SDO upload request for 0x1000:0
    uint8_t request[11] = {
        0x00, 0x00,  // Transaction ID
        0x02,        // CoE type: SDO Request
        0x40,        // Command: Upload Init Request (CCS=2)
        0x00, 0x10,  // Index: 0x1000
        0x00,        // SubIndex: 0
        0x00, 0x00, 0x00, 0x00  // Reserved
    };
    
    uint8_t response[256];
    size_t responseLen = sizeof(response);
    
    EXPECT_TRUE(handler_->processRequest(request, sizeof(request), response, responseLen));
    EXPECT_GE(responseLen, 11);
    
    // Verify response is upload init response
    EXPECT_EQ(response[2], 0x03);  // CoE SDO Response
}

TEST_F(CoEHandlerTest, SDODownloadExpedited) {
    // Build SDO download request for 0x6060:0 (Mode of Operation)
    uint8_t request[11] = {
        0x00, 0x00,  // Transaction ID
        0x02,        // CoE type: SDO Request
        0x2F,        // Command: Download Init (expedited, size indicated)
        0x60, 0x60,  // Index: 0x6060
        0x00,        // SubIndex: 0
        0x01,        // Data: Mode 1 (PP)
        0x00, 0x00, 0x00
    };
    
    uint8_t response[256];
    size_t responseLen = sizeof(response);
    
    EXPECT_TRUE(handler_->processRequest(request, sizeof(request), response, responseLen));
    
    // Verify object was written
    ODEntry* entry = handler_->getObject(0x6060, 0);
    EXPECT_NE(entry, nullptr);
    EXPECT_EQ(entry->data[0], 0x01);
}

// ============================================================================
// Profile Slave Tests
// ============================================================================

class CiA402SlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = std::make_unique<CiA402Slave>(1);
    }
    
    std::unique_ptr<CiA402Slave> slave_;
};

TEST_F(CiA402SlaveTest, InitialState) {
    // CiA 402 should start in "Not ready to switch on"
    EXPECT_EQ(slave_->getStatusWord() & 0x6F, 0x00);
}

TEST_F(CiA402SlaveTest, StateTransitions) {
    // Transition to Ready to Switch On
    slave_->setControlWord(0x0006);  // Shutdown
    slave_->update();
    EXPECT_EQ(slave_->getStatusWord() & 0x6F, 0x21);  // Ready to switch on
    
    // Transition to Switched On
    slave_->setControlWord(0x0007);  // Switch on
    slave_->update();
    EXPECT_EQ(slave_->getStatusWord() & 0x6F, 0x23);  // Switched on
    
    // Transition to Operation Enabled
    slave_->setControlWord(0x000F);  // Enable operation
    slave_->update();
    EXPECT_EQ(slave_->getStatusWord() & 0x6F, 0x27);  // Operation enabled
}

TEST_F(CiA402SlaveTest, ProfilePositionMode) {
    // Get to Operation Enabled
    slave_->setControlWord(0x0006);
    slave_->update();
    slave_->setControlWord(0x0007);
    slave_->update();
    slave_->setControlWord(0x000F);
    slave_->update();
    
    // Set PP mode
    slave_->setModesOfOperation(1);
    
    // Set target position
    slave_->setTargetPosition(10000);
    
    // Start motion (new setpoint)
    slave_->setControlWord(0x001F);  // Enable + new setpoint
    slave_->update();
    
    // Verify target ack
    EXPECT_TRUE((slave_->getStatusWord() & 0x1000) != 0);
}

// ============================================================================
// FIFO Communication Tests
// ============================================================================

#ifdef __linux__

class FIFOCommunicationTest : public ::testing::Test {
protected:
    void SetUp() override {
        basePath_ = "/tmp/ethercat_test_fifo";
    }
    
    void TearDown() override {
        // Clean up FIFOs
        unlink((basePath_ + "_tx").c_str());
        unlink((basePath_ + "_rx").c_str());
    }
    
    std::string basePath_;
};

// Note: FIFO tests require two threads or processes - simplified version here
TEST_F(FIFOCommunicationTest, FIFOCreation) {
    auto fifoHAL = createFIFOLoopbackHAL();
    EXPECT_NE(fifoHAL, nullptr);
    
    EXPECT_TRUE(fifoHAL->open(basePath_));
    EXPECT_TRUE(fifoHAL->isOpen());
    
    fifoHAL->close();
    EXPECT_FALSE(fifoHAL->isOpen());
}

#endif  // __linux__

// ============================================================================
// PcapNG Logger Tests
// ============================================================================

class PcapLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = createPcapLogger();
    }
    
    std::unique_ptr<IPcapLogger> logger_;
};

TEST_F(PcapLoggerTest, OpenClose) {
    EXPECT_TRUE(logger_->open("/tmp/test_ethercat.pcapng"));
    EXPECT_TRUE(logger_->isOpen());
    
    logger_->close();
    EXPECT_FALSE(logger_->isOpen());
    
    // Clean up
    unlink("/tmp/test_ethercat.pcapng");
}

TEST_F(PcapLoggerTest, LogPacket) {
    EXPECT_TRUE(logger_->open("/tmp/test_ethercat_log.pcapng"));
    
    // Add interface
    InterfaceInfo info;
    info.name = "eth0";
    info.description = "Test interface";
    info.linkType = 1;
    uint8_t ifaceId = logger_->addInterface(info);
    
    // Log some packets
    uint8_t packet[64] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast
    
    for (int i = 0; i < 10; i++) {
        PacketMetadata meta;
        meta.direction = (i % 2 == 0) ? PacketDirection::Outbound : PacketDirection::Inbound;
        meta.interfaceId = ifaceId;
        EXPECT_TRUE(logger_->logPacket(packet, sizeof(packet), meta));
    }
    
    EXPECT_EQ(logger_->getPacketCount(), 10);
    
    logger_->close();
    unlink("/tmp/test_ethercat_log.pcapng");
}

TEST_F(PcapLoggerTest, NullLogger) {
    auto nullLogger = createNullPcapLogger();
    
    EXPECT_TRUE(nullLogger->open("anything"));
    EXPECT_TRUE(nullLogger->isOpen());
    
    uint8_t packet[64];
    EXPECT_TRUE(nullLogger->logPacket(packet, sizeof(packet), 
                                      PacketDirection::Outbound, 0));
    
    EXPECT_EQ(nullLogger->getPacketCount(), 0);  // Null logger discards
}

// ============================================================================
// Loopback HAL Tests
// ============================================================================

class LoopbackHALTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create slave for frame processing
        SlaveConfig config;
        config.vendorId = 0x00000001;
        config.productCode = 0x00001000;
        config.memorySize = 0x2000;
        config.maxFmmus = 4;
        config.maxSyncManagers = 4;
        config.mailboxOut = {0x1000, 256, 0};
        config.mailboxIn = {0x1100, 256, 0};
        config.supportedMailbox = {true, true, true, true, true, true};
        
        slave_ = std::make_unique<SlaveCore>(config);
        
        // Configure mailbox SyncManagers
        SyncManagerConfig sm0;
        sm0.startAddress = 0x1000;
        sm0.length = 256;
        sm0.direction = SMDirection::Output;
        sm0.mode = SMMode::Mailbox;
        sm0.enabled = true;
        slave_->configureSyncManager(0, sm0);
        
        SyncManagerConfig sm1;
        sm1.startAddress = 0x1100;
        sm1.length = 256;
        sm1.direction = SMDirection::Input;
        sm1.mode = SMMode::Mailbox;
        sm1.enabled = true;
        slave_->configureSyncManager(1, sm1);
    }
    
    std::unique_ptr<SlaveCore> slave_;
};

TEST_F(LoopbackHALTest, DirectLoopback) {
    auto hal = createDirectLoopbackHAL();
    EXPECT_NE(hal, nullptr);
    
    EXPECT_TRUE(hal->open("loopback"));
    EXPECT_TRUE(hal->isOpen());
    
    hal->close();
}

TEST_F(LoopbackHALTest, ThreadedLoopback) {
    auto hal = createThreadedLoopbackHAL();
    EXPECT_NE(hal, nullptr);
    
    EXPECT_TRUE(hal->open("loopback"));
    EXPECT_TRUE(hal->isOpen());
    
    hal->close();
}

}  // namespace Test
}  // namespace Slave
}  // namespace EtherCAT

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
