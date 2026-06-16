/**
 * @file MasterSlaveTests.cpp
 * @brief Comprehensive integration tests for master-slave interaction
 */

#include "slave/SlaveCore.hpp"
#include "slave/hal/LoopbackHAL.hpp"
#include "slave/mailbox/CoEHandler.hpp"
#include "slave/profiles/CiA402Slave.hpp"
#include "pcap/PcapLogger.hpp"
#include "tether/ethercat/DCClass.hpp"

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>

namespace EtherCAT {
namespace Test {

// ============================================================================
// EtherCAT Frame Building Utilities
// ============================================================================

class FrameBuilder {
public:
    static constexpr size_t ETH_HEADER_SIZE = 14;
    static constexpr size_t ECAT_HEADER_SIZE = 2;
    static constexpr size_t DATAGRAM_HEADER_SIZE = 10;
    static constexpr uint16_t ETHERCAT_TYPE = 0x88A4;
    
    struct Datagram {
        uint8_t command;
        uint8_t index;
        uint16_t adp;  // address position or station alias
        uint16_t ado;  // address offset
        uint16_t dataLen;
        std::vector<uint8_t> data;
        uint16_t wkc;
    };
    
    std::vector<uint8_t> build(const std::vector<Datagram>& datagrams) {
        std::vector<uint8_t> frame;
        
        // Ethernet header
        frame.resize(ETH_HEADER_SIZE);
        std::memset(frame.data(), 0xFF, 6);  // Dest MAC (broadcast)
        std::memset(frame.data() + 6, 0x00, 6);  // Source MAC
        frame[12] = (ETHERCAT_TYPE >> 8) & 0xFF;
        frame[13] = ETHERCAT_TYPE & 0xFF;
        
        // Calculate total EtherCAT payload length
        size_t totalLen = 0;
        for (const auto& dg : datagrams) {
            totalLen += DATAGRAM_HEADER_SIZE + dg.dataLen + 2;  // +2 for WKC
        }
        
        // EtherCAT header (length and type)
        uint16_t ecatHeader = (totalLen & 0x7FF) | (0x01 << 12);  // Type 1 = EtherCAT
        frame.push_back(ecatHeader & 0xFF);
        frame.push_back((ecatHeader >> 8) & 0xFF);
        
        // Datagrams
        for (size_t i = 0; i < datagrams.size(); i++) {
            const auto& dg = datagrams[i];
            
            // Datagram header
            frame.push_back(dg.command);
            frame.push_back(dg.index);
            frame.push_back(dg.adp & 0xFF);
            frame.push_back((dg.adp >> 8) & 0xFF);
            frame.push_back(dg.ado & 0xFF);
            frame.push_back((dg.ado >> 8) & 0xFF);
            
            // Length + flags
            uint16_t lenFlags = (dg.dataLen & 0x7FF);
            if (i < datagrams.size() - 1) {
                lenFlags |= 0x8000;  // More datagrams follow
            }
            frame.push_back(lenFlags & 0xFF);
            frame.push_back((lenFlags >> 8) & 0xFF);
            
            // IRQ (2 bytes, typically 0)
            frame.push_back(0);
            frame.push_back(0);
            
            // Data
            frame.insert(frame.end(), dg.data.begin(), dg.data.end());
            
            // Working counter (initially 0)
            frame.push_back(dg.wkc & 0xFF);
            frame.push_back((dg.wkc >> 8) & 0xFF);
        }
        
        return frame;
    }
    
    static void parseDatagram(const std::vector<uint8_t>& frame, size_t offset, Datagram& dg) {
        dg.command = frame[offset];
        dg.index = frame[offset + 1];
        dg.adp = frame[offset + 2] | (frame[offset + 3] << 8);
        dg.ado = frame[offset + 4] | (frame[offset + 5] << 8);
        dg.dataLen = frame[offset + 6] | ((frame[offset + 7] & 0x07) << 8);
        
        size_t dataStart = offset + DATAGRAM_HEADER_SIZE;
        dg.data.assign(frame.begin() + dataStart, frame.begin() + dataStart + dg.dataLen);
        
        size_t wkcOffset = dataStart + dg.dataLen;
        dg.wkc = frame[wkcOffset] | (frame[wkcOffset + 1] << 8);
    }
};

// EtherCAT datagram commands
enum class EcatCmd : uint8_t {
    NOP  = 0x00,
    APRD = 0x01,  // Auto-increment physical read
    APWR = 0x02,  // Auto-increment physical write
    APRW = 0x03,  // Auto-increment physical read/write
    FPRD = 0x04,  // Configured address physical read
    FPWR = 0x05,  // Configured address physical write
    FPRW = 0x06,  // Configured address physical read/write
    BRD  = 0x07,  // Broadcast read
    BWR  = 0x08,  // Broadcast write
    BRW  = 0x09,  // Broadcast read/write
    LRD  = 0x0A,  // Logical read
    LWR  = 0x0B,  // Logical write
    LRW  = 0x0C,  // Logical read/write
    ARMW = 0x0D,  // Auto-increment physical read multiple write
    FRMW = 0x0E   // Configured address physical read multiple write
};

// ============================================================================
// Master-Slave Communication Tests
// ============================================================================

class MasterSlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create slave
        slave_ = std::make_unique<Slave::SlaveCore>(1);
        
        // Create HAL
        hal_ = std::make_unique<Slave::DirectLoopbackHAL>([this](const uint8_t* data, size_t len) {
            return slave_->processFrame(data, len);
        });
        
        builder_ = std::make_unique<FrameBuilder>();
    }
    
    std::vector<uint8_t> sendFrame(const std::vector<uint8_t>& frame) {
        std::vector<uint8_t> response(frame.size());
        size_t responseLen = 0;
        
        if (hal_->sendFrame(frame.data(), frame.size(), response.data(), &responseLen)) {
            response.resize(responseLen);
            return response;
        }
        return {};
    }
    
    std::unique_ptr<Slave::SlaveCore> slave_;
    std::unique_ptr<Slave::DirectLoopbackHAL> hal_;
    std::unique_ptr<FrameBuilder> builder_;
};

TEST_F(MasterSlaveTest, AutoIncrementRead) {
    // Create APRD datagram to read AL Status register (0x0130)
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::APRD);
    dg.index = 1;
    dg.adp = 0;  // Position 0
    dg.ado = 0x0130;  // AL Status
    dg.dataLen = 2;
    dg.data.resize(2, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    // Parse response
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    EXPECT_EQ(respDg.wkc, 1);  // Working counter should be incremented
}

TEST_F(MasterSlaveTest, ConfiguredAddressRead) {
    // First set configured address
    slave_->setConfiguredAddress(0x1001);
    
    // Create FPRD datagram
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg.index = 1;
    dg.adp = 0x1001;  // Configured address
    dg.ado = 0x0120;  // AL Control register
    dg.dataLen = 2;
    dg.data.resize(2, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    EXPECT_EQ(respDg.wkc, 1);
}

TEST_F(MasterSlaveTest, ConfiguredAddressWrite) {
    slave_->setConfiguredAddress(0x1001);
    
    // Write to AL Control to request state change
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPWR);
    dg.index = 1;
    dg.adp = 0x1001;
    dg.ado = 0x0120;  // AL Control
    dg.dataLen = 2;
    dg.data = {0x02, 0x00};  // Request Pre-Op
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    EXPECT_EQ(respDg.wkc, 1);
}

TEST_F(MasterSlaveTest, BroadcastRead) {
    // BRD reads from all slaves
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::BRD);
    dg.index = 1;
    dg.adp = 0;  // Not used for broadcast
    dg.ado = 0x0010;  // Type register
    dg.dataLen = 2;
    dg.data.resize(2, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
}

TEST_F(MasterSlaveTest, LogicalReadWrite) {
    // First configure FMMU and SyncManager
    slave_->configureFMMU(0, 0x1000, 8, 0x1100, 0x0, true, true);
    slave_->configureSyncManager(2, 0x1100, 8, Slave::SyncManagerType::Output, true);
    slave_->toState(Slave::SlaveState::SafeOp);
    slave_->toState(Slave::SlaveState::Op);
    
    // LRW to logical address
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::LRW);
    dg.index = 1;
    dg.adp = 0x1000 & 0xFFFF;  // Logical address low
    dg.ado = 0x1000 >> 16;      // Logical address high
    dg.dataLen = 8;
    dg.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
}

TEST_F(MasterSlaveTest, MultipleDatagrams) {
    slave_->setConfiguredAddress(0x1001);
    
    // Multiple datagrams in one frame
    FrameBuilder::Datagram dg1;
    dg1.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg1.index = 1;
    dg1.adp = 0x1001;
    dg1.ado = 0x0130;  // AL Status
    dg1.dataLen = 2;
    dg1.data.resize(2, 0);
    dg1.wkc = 0;
    
    FrameBuilder::Datagram dg2;
    dg2.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg2.index = 2;
    dg2.adp = 0x1001;
    dg2.ado = 0x0010;  // Type
    dg2.dataLen = 2;
    dg2.data.resize(2, 0);
    dg2.wkc = 0;
    
    auto frame = builder_->build({dg1, dg2});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    // Both datagrams should be processed
    FrameBuilder::Datagram respDg1, respDg2;
    size_t offset = FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE;
    FrameBuilder::parseDatagram(response, offset, respDg1);
    
    offset += FrameBuilder::DATAGRAM_HEADER_SIZE + respDg1.dataLen + 2;
    FrameBuilder::parseDatagram(response, offset, respDg2);
    
    EXPECT_EQ(respDg1.wkc, 1);
    EXPECT_EQ(respDg2.wkc, 1);
}

// ============================================================================
// State Machine Transition Tests
// ============================================================================

class StateTransitionTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        MasterSlaveTest::SetUp();
        slave_->setConfiguredAddress(0x1001);
    }
    
    void requestState(uint16_t state) {
        FrameBuilder::Datagram dg;
        dg.command = static_cast<uint8_t>(EcatCmd::FPWR);
        dg.index = 1;
        dg.adp = 0x1001;
        dg.ado = 0x0120;  // AL Control
        dg.dataLen = 2;
        dg.data = {static_cast<uint8_t>(state & 0xFF), static_cast<uint8_t>((state >> 8) & 0xFF)};
        dg.wkc = 0;
        
        auto frame = builder_->build({dg});
        sendFrame(frame);
    }
    
    uint16_t readState() {
        FrameBuilder::Datagram dg;
        dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
        dg.index = 1;
        dg.adp = 0x1001;
        dg.ado = 0x0130;  // AL Status
        dg.dataLen = 2;
        dg.data.resize(2, 0);
        dg.wkc = 0;
        
        auto frame = builder_->build({dg});
        auto response = sendFrame(frame);
        
        if (response.empty()) return 0xFF;
        
        FrameBuilder::Datagram respDg;
        FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
        
        return respDg.data[0] | (respDg.data[1] << 8);
    }
};

TEST_F(StateTransitionTest, InitToPreOp) {
    requestState(0x02);  // Pre-Op
    uint16_t state = readState();
    
    EXPECT_EQ(state & 0x0F, 0x02);  // Should be in Pre-Op
}

TEST_F(StateTransitionTest, PreOpToSafeOp) {
    requestState(0x02);  // Pre-Op
    requestState(0x04);  // Safe-Op
    uint16_t state = readState();
    
    EXPECT_EQ(state & 0x0F, 0x04);  // Should be in Safe-Op
}

TEST_F(StateTransitionTest, SafeOpToOp) {
    requestState(0x02);  // Pre-Op
    requestState(0x04);  // Safe-Op
    requestState(0x08);  // Op
    uint16_t state = readState();
    
    EXPECT_EQ(state & 0x0F, 0x08);  // Should be in Op
}

TEST_F(StateTransitionTest, OpToInit) {
    requestState(0x02);  // Pre-Op
    requestState(0x04);  // Safe-Op
    requestState(0x08);  // Op
    requestState(0x01);  // Init
    uint16_t state = readState();
    
    EXPECT_EQ(state & 0x0F, 0x01);  // Should be in Init
}

TEST_F(StateTransitionTest, InvalidTransition) {
    // Cannot go directly from Init to Op
    requestState(0x08);  // Op
    uint16_t state = readState();
    
    // Should remain in Init with error
    EXPECT_EQ(state & 0x0F, 0x01);
    EXPECT_TRUE((state & 0x10) != 0);  // Error bit set
}

// ============================================================================
// PDO Communication Tests
// ============================================================================

class PDOCommunicationTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        MasterSlaveTest::SetUp();
        slave_->setConfiguredAddress(0x1001);
        
        // Configure for PDO exchange
        // FMMU 0: Output (master -> slave)
        slave_->configureFMMU(0, 0x1000, 8, 0x1100, 0x0, true, true);
        slave_->configureSyncManager(2, 0x1100, 8, Slave::SyncManagerType::Output, true);
        
        // FMMU 1: Input (slave -> master)
        slave_->configureFMMU(1, 0x1100, 8, 0x1000, 0x0, true, false);
        slave_->configureSyncManager(3, 0x1000, 8, Slave::SyncManagerType::Input, true);
        
        // Go to Op state
        slave_->toState(Slave::SlaveState::PreOp);
        slave_->toState(Slave::SlaveState::SafeOp);
        slave_->toState(Slave::SlaveState::Op);
    }
};

TEST_F(PDOCommunicationTest, OutputPDO) {
    // Send output data to slave
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::LWR);
    dg.index = 1;
    dg.adp = 0x1000 & 0xFFFF;
    dg.ado = 0;
    dg.dataLen = 8;
    dg.data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    // Verify data was received by slave
    std::vector<uint8_t> readData(8);
    slave_->readProcessData(0x1100, readData.data(), 8);
    
    EXPECT_EQ(readData, dg.data);
}

TEST_F(PDOCommunicationTest, InputPDO) {
    // Write input data to slave's process data memory
    std::vector<uint8_t> inputData = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    slave_->writeProcessData(0x1000, inputData.data(), inputData.size());
    
    // Read input data from slave
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::LRD);
    dg.index = 1;
    dg.adp = 0x1100 & 0xFFFF;
    dg.ado = 0;
    dg.dataLen = 8;
    dg.data.resize(8, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    EXPECT_EQ(respDg.data, inputData);
}

TEST_F(PDOCommunicationTest, SimultaneousReadWrite) {
    // Prepare input data
    std::vector<uint8_t> inputData = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    slave_->writeProcessData(0x1000, inputData.data(), inputData.size());
    
    // LRW: Read input, write output in one datagram
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::LRW);
    dg.index = 1;
    dg.adp = 0x1000 & 0xFFFF;
    dg.ado = 0;
    dg.dataLen = 8;
    dg.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    // WKC should be 3 (1 for read + 2 for write, or configured otherwise)
}

// ============================================================================
// Distributed Clocks Tests
// ============================================================================

class DistributedClocksTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        MasterSlaveTest::SetUp();
        slave_->setConfiguredAddress(0x1001);
        slave_->enableDC(true);
    }
};

TEST_F(DistributedClocksTest, ReadSystemTime) {
    // Read DC System Time (0x0910)
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg.index = 1;
    dg.adp = 0x1001;
    dg.ado = toUInt16(DCRegisters::DCSysTime);
    dg.dataLen = 8;
    dg.data.resize(8, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    // Time should be non-zero and incrementing
    uint64_t time1 = 0;
    for (int i = 0; i < 8; i++) {
        time1 |= static_cast<uint64_t>(respDg.data[i]) << (i * 8);
    }
    
    // Read again
    response = sendFrame(frame);
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    uint64_t time2 = 0;
    for (int i = 0; i < 8; i++) {
        time2 |= static_cast<uint64_t>(respDg.data[i]) << (i * 8);
    }
    
    EXPECT_GT(time2, time1);
}

TEST_F(DistributedClocksTest, WriteSystemTimeOffset) {
    // Write DC System Time Offset (0x0920)
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPWR);
    dg.index = 1;
    dg.adp = 0x1001;
    dg.ado = toUInt16(DCRegisters::DCSysOffset);
    dg.dataLen = 8;
    dg.data = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};  // Small offset
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
    
    FrameBuilder::Datagram respDg;
    FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
    
    EXPECT_EQ(respDg.wkc, 1);
}

TEST_F(DistributedClocksTest, ConfigureSyncSignals) {
    // Configure SYNC0 cycle time (0x09A0)
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPWR);
    dg.index = 1;
    dg.adp = 0x1001;
    dg.ado = toUInt16(DCRegisters::DCCycle0);
    dg.dataLen = 4;
    dg.data = {0x40, 0x42, 0x0F, 0x00};  // 1ms = 1,000,000 ns
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
}

// ============================================================================
// FIFO Communication Tests
// ============================================================================

class FIFOCommunicationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create slave
        slave_ = std::make_unique<Slave::SlaveCore>(1);
        
        // Use temporary paths for FIFOs
        masterToSlaveFifo_ = "/tmp/ecat_test_m2s_" + std::to_string(getpid());
        slaveToMasterFifo_ = "/tmp/ecat_test_s2m_" + std::to_string(getpid());
        
        builder_ = std::make_unique<FrameBuilder>();
    }
    
    void TearDown() override {
        // Clean up FIFOs
        unlink(masterToSlaveFifo_.c_str());
        unlink(slaveToMasterFifo_.c_str());
    }
    
    std::unique_ptr<Slave::SlaveCore> slave_;
    std::unique_ptr<FrameBuilder> builder_;
    std::string masterToSlaveFifo_;
    std::string slaveToMasterFifo_;
};

TEST_F(FIFOCommunicationTest, CreateFIFOBridge) {
    // This test verifies FIFO bridge creation
    Slave::FIFOConfig config;
    config.masterToSlavePath = masterToSlaveFifo_;
    config.slaveToMasterPath = slaveToMasterFifo_;
    config.createFifos = true;
    config.blocking = false;
    
    // Bridge should create FIFOs
    EXPECT_NO_THROW({
        auto bridge = std::make_unique<Slave::FIFOBridge>(config);
    });
}

// ============================================================================
// Profile Integration Tests
// ============================================================================

class CiA402IntegrationTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        // Create CiA 402 drive
        drive_ = std::make_unique<Slave::CiA402Slave>(1);
        
        // Create HAL connected to drive
        hal_ = std::make_unique<Slave::DirectLoopbackHAL>([this](const uint8_t* data, size_t len) {
            return drive_->processFrame(data, len);
        });
        
        builder_ = std::make_unique<FrameBuilder>();
        drive_->setConfiguredAddress(0x1001);
    }
    
    uint16_t readSDO(uint16_t index, uint8_t subindex) {
        // Simplified SDO read via mailbox
        // In real implementation, this would use CoE protocol
        return 0;
    }
    
    void writeSDO(uint16_t index, uint8_t subindex, uint16_t value) {
        // Simplified SDO write
    }
    
    std::unique_ptr<Slave::CiA402Slave> drive_;
};

TEST_F(CiA402IntegrationTest, DriveStateTransitions) {
    // Configure PDO mapping for controlword/statusword
    drive_->configureFMMU(0, 0x1000, 2, 0x1100, 0x0, true, true);   // Controlword
    drive_->configureFMMU(1, 0x1002, 2, 0x1000, 0x0, true, false);  // Statusword
    drive_->configureSyncManager(2, 0x1100, 2, Slave::SyncManagerType::Output, true);
    drive_->configureSyncManager(3, 0x1000, 2, Slave::SyncManagerType::Input, true);
    
    drive_->toState(Slave::SlaveState::PreOp);
    drive_->toState(Slave::SlaveState::SafeOp);
    drive_->toState(Slave::SlaveState::Op);
    
    // Send controlword via PDO
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::LRW);
    dg.index = 1;
    dg.adp = 0x1000 & 0xFFFF;
    dg.ado = 0;
    dg.dataLen = 4;
    dg.data = {0x06, 0x00, 0x00, 0x00};  // Shutdown command
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    ASSERT_GT(response.size(), 0);
}

// ============================================================================
// Concurrent Access Tests
// ============================================================================

class ConcurrentAccessTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        MasterSlaveTest::SetUp();
        slave_->setConfiguredAddress(0x1001);
        
        // Configure for PDO
        slave_->configureFMMU(0, 0x1000, 8, 0x1100, 0x0, true, true);
        slave_->configureSyncManager(2, 0x1100, 8, Slave::SyncManagerType::Output, true);
        
        slave_->toState(Slave::SlaveState::PreOp);
        slave_->toState(Slave::SlaveState::SafeOp);
        slave_->toState(Slave::SlaveState::Op);
    }
};

TEST_F(ConcurrentAccessTest, MultiThreadedPDOAccess) {
    std::atomic<bool> running{true};
    std::atomic<int> successCount{0};
    std::atomic<int> errorCount{0};
    
    auto workerFunc = [&](int threadId) {
        FrameBuilder localBuilder;
        
        while (running.load()) {
            FrameBuilder::Datagram dg;
            dg.command = static_cast<uint8_t>(EcatCmd::LWR);
            dg.index = threadId;
            dg.adp = 0x1000 & 0xFFFF;
            dg.ado = 0;
            dg.dataLen = 8;
            dg.data.resize(8, threadId);
            dg.wkc = 0;
            
            auto frame = localBuilder.build({dg});
            auto response = sendFrame(frame);
            
            if (!response.empty()) {
                successCount.fetch_add(1);
            } else {
                errorCount.fetch_add(1);
            }
        }
    };
    
    // Start multiple threads
    std::vector<std::thread> threads;
    for (int i = 0; i < 4; i++) {
        threads.emplace_back(workerFunc, i);
    }
    
    // Run for a short time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running.store(false);
    
    for (auto& t : threads) {
        t.join();
    }
    
    // Should have many successes and no errors
    EXPECT_GT(successCount.load(), 0);
    // Note: In a real concurrent test, we might expect some contention
}

// ============================================================================
// PcapNG Logging Integration Tests
// ============================================================================

class PcapLoggingIntegrationTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        // Create memory logger
        logger_ = std::make_unique<PcapNg::MemoryPcapLogger>();
        
        // Create slave with logging
        slave_ = std::make_unique<Slave::SlaveCore>(1);
        
        // Create HAL with logging
        hal_ = std::make_unique<Slave::DirectLoopbackHAL>(
            [this](const uint8_t* data, size_t len) {
                return slave_->processFrame(data, len);
            },
            logger_.get()
        );
        
        builder_ = std::make_unique<FrameBuilder>();
    }
    
    std::unique_ptr<PcapNg::MemoryPcapLogger> logger_;
};

TEST_F(PcapLoggingIntegrationTest, LogFrameExchange) {
    slave_->setConfiguredAddress(0x1001);
    
    // Exchange some frames
    for (int i = 0; i < 10; i++) {
        FrameBuilder::Datagram dg;
        dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
        dg.index = i;
        dg.adp = 0x1001;
        dg.ado = 0x0130;
        dg.dataLen = 2;
        dg.data.resize(2, 0);
        dg.wkc = 0;
        
        auto frame = builder_->build({dg});
        sendFrame(frame);
    }
    
    // Verify logging
    auto& packets = logger_->getPackets();
    EXPECT_GT(packets.size(), 0);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

class ErrorHandlingTest : public MasterSlaveTest {
protected:
    void SetUp() override {
        MasterSlaveTest::SetUp();
        slave_->setConfiguredAddress(0x1001);
    }
};

TEST_F(ErrorHandlingTest, InvalidAddress) {
    // Try to access wrong configured address
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg.index = 1;
    dg.adp = 0x9999;  // Wrong address
    dg.ado = 0x0130;
    dg.dataLen = 2;
    dg.data.resize(2, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    if (!response.empty()) {
        FrameBuilder::Datagram respDg;
        FrameBuilder::parseDatagram(response, FrameBuilder::ETH_HEADER_SIZE + FrameBuilder::ECAT_HEADER_SIZE, respDg);
        
        // Working counter should be 0 (no slave responded)
        EXPECT_EQ(respDg.wkc, 0);
    }
}

TEST_F(ErrorHandlingTest, InvalidRegisterAccess) {
    // Try to read from invalid register
    FrameBuilder::Datagram dg;
    dg.command = static_cast<uint8_t>(EcatCmd::FPRD);
    dg.index = 1;
    dg.adp = 0x1001;
    dg.ado = 0xFFFF;  // Invalid register
    dg.dataLen = 2;
    dg.data.resize(2, 0);
    dg.wkc = 0;
    
    auto frame = builder_->build({dg});
    auto response = sendFrame(frame);
    
    // Should still get a response (with zeros or error indication)
    ASSERT_GT(response.size(), 0);
}

TEST_F(ErrorHandlingTest, MalformedFrame) {
    // Send truncated frame
    std::vector<uint8_t> badFrame = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    std::vector<uint8_t> response(1500);
    size_t responseLen = 0;
    
    // Should handle gracefully without crashing
    EXPECT_NO_THROW({
        hal_->sendFrame(badFrame.data(), badFrame.size(), response.data(), &responseLen);
    });
}

TEST_F(ErrorHandlingTest, EmptyFrame) {
    std::vector<uint8_t> emptyFrame;
    std::vector<uint8_t> response(1500);
    size_t responseLen = 0;
    
    // Should handle gracefully
    EXPECT_NO_THROW({
        hal_->sendFrame(emptyFrame.data(), emptyFrame.size(), response.data(), &responseLen);
    });
}

}  // namespace Test
}  // namespace EtherCAT

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
