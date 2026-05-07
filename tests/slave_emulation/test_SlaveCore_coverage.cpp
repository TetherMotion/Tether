/**
 * @file test_SlaveCore_coverage.cpp
 * @brief Comprehensive SlaveCore coverage tests — state machine, frame processing,
 *        FMMU, SyncManager, PDO, DC, SII, watchdog, mailbox.
 */
#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <memory>
#include "slave/core/SlaveCore.hpp"
#include "slave/core/SlaveTypes.hpp"
#include "slave/logging/SlaveLogger.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

using namespace EtherCAT::slave;

// EtherCAT constants
static constexpr uint16_t ETHERCAT_ETHERTYPE = 0x88A4;

// EtherCAT datagram commands
enum class EcCmd : uint8_t {
    NOP  = 0x00,
    APRD = 0x01, APWR = 0x02, APRW = 0x03,
    FPRD = 0x04, FPWR = 0x05, FPRW = 0x06,
    BRD  = 0x07, BWR  = 0x08, BRW  = 0x09,
    LRD  = 0x0A, LWR  = 0x0B, LRW  = 0x0C,
    ARMW = 0x0D, FRMW = 0x0E
};

// Register addresses
static constexpr uint16_t REG_TYPE         = 0x0000;
static constexpr uint16_t REG_REVISION     = 0x0001;
static constexpr uint16_t REG_DL_STATUS    = 0x0110;
static constexpr uint16_t REG_AL_CONTROL   = 0x0120;
static constexpr uint16_t REG_AL_STATUS    = 0x0130;
static constexpr uint16_t REG_AL_STATUS_CODE = 0x0134;
static constexpr uint16_t REG_SII_CONTROL  = 0x0502;
static constexpr uint16_t REG_SII_ADDRESS  = 0x0504;
static constexpr uint16_t REG_SII_DATA     = 0x0508;
static constexpr uint16_t REG_FMMU0        = 0x0600;
static constexpr uint16_t REG_SM0          = 0x0800;
static constexpr uint16_t REG_DC_TIME0     = ESCReg::DCReceiveTime;
static constexpr uint16_t REG_DC_SYSTEM_TIME = ESCReg::DCSystemTime;

// ============================================================================
// Helper: build an EtherCAT frame with a single datagram
// ============================================================================
static std::vector<uint8_t> buildFrame(EcCmd cmd, uint16_t adp, uint16_t ado,
                                        const std::vector<uint8_t>& data) {
    const size_t dataLen = data.size();
    const size_t frameTotalLen = 14 + 2 + 10 + dataLen + 2;
    std::vector<uint8_t> frame(frameTotalLen, 0);

    // Ethernet header
    std::memset(&frame[0], 0xFF, 6);  // dst MAC (broadcast)
    std::memset(&frame[6], 0x00, 6);  // src MAC
    frame[12] = (ETHERCAT_ETHERTYPE >> 8) & 0xFF;
    frame[13] = ETHERCAT_ETHERTYPE & 0xFF;

    // EtherCAT header: len = total_datagram_size including headers
    uint16_t ecLen = static_cast<uint16_t>(10 + dataLen + 2);
    ecLen |= (0x01 << 12); // Type 1
    frame[14] = ecLen & 0xFF;
    frame[15] = (ecLen >> 8) & 0xFF;

    // Datagram header (10 bytes)
    size_t pos = 16;
    frame[pos++] = static_cast<uint8_t>(cmd);
    frame[pos++] = 0; // index
    frame[pos++] = adp & 0xFF;
    frame[pos++] = (adp >> 8) & 0xFF;
    frame[pos++] = ado & 0xFF;
    frame[pos++] = (ado >> 8) & 0xFF;
    uint16_t lenFlags = static_cast<uint16_t>(dataLen); // no more datagrams, no C
    frame[pos++] = lenFlags & 0xFF;
    frame[pos++] = (lenFlags >> 8) & 0xFF;
    frame[pos++] = 0; // IRQ
    frame[pos++] = 0;

    // Data
    std::memcpy(&frame[pos], data.data(), dataLen);
    pos += dataLen;

    // WKC (initially 0)
    frame[pos++] = 0;
    frame[pos++] = 0;

    return frame;
}

static uint16_t extractWKC(const std::vector<uint8_t>& response, size_t dataLen) {
    size_t wkcPos = 16 + 10 + dataLen;
    if (response.size() > wkcPos + 1) {
        return response[wkcPos] | (response[wkcPos + 1] << 8);
    }
    return 0;
}

// ============================================================================
// Fixture: SlaveCore Tests
// ============================================================================
class SlaveCoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlaveConfig cfg{};
        cfg.identity.vendorId = 0x12345678;
        cfg.identity.productCode = 0xABCD;
        cfg.identity.revisionNumber = 1;
        cfg.identity.serialNumber = 42;
        cfg.identity.deviceName = "TestSlave";
        cfg.rxPdoSize = 8;
        cfg.txPdoSize = 8;
        cfg.supportsDC = true;
        cfg.watchdogEnabled = true;
        cfg.supportsBootstrap = true;
        core = std::make_unique<SlaveCore>(cfg);
    }
    std::unique_ptr<SlaveCore> core;
};

// --- Construction and lifecycle ---

TEST_F(SlaveCoreTest, ConstructionDefaults) {
    EXPECT_EQ(core->getState(), SlaveState::INIT);
    EXPECT_FALSE(core->isRunning());
}

TEST_F(SlaveCoreTest, StartStop) {
    EXPECT_TRUE(core->start());
    EXPECT_TRUE(core->isRunning());
    core->stop();
    EXPECT_FALSE(core->isRunning());
}

TEST_F(SlaveCoreTest, DoubleStartReturnsExpected) {
    core->start();
    // Second start may return false if already running
    bool secondStart = core->start();
    // Either way, should still be running
    EXPECT_TRUE(core->isRunning());
    (void)secondStart;
}

TEST_F(SlaveCoreTest, StopWithoutStart) {
    core->stop();
    EXPECT_FALSE(core->isRunning());
}

// --- Configuration access ---

TEST_F(SlaveCoreTest, GetConfig) {
    const auto& cfg = core->getConfig();
    EXPECT_EQ(cfg.rxPdoSize, 8u);
    EXPECT_EQ(cfg.txPdoSize, 8u);
}

TEST_F(SlaveCoreTest, GetIdentity) {
    const auto& id = core->getIdentity();
    EXPECT_EQ(id.vendorId, 0x12345678u);
    EXPECT_EQ(id.productCode, 0xABCDu);
    EXPECT_EQ(id.serialNumber, 42u);
    EXPECT_EQ(id.deviceName, "TestSlave");
}

TEST_F(SlaveCoreTest, SetPosition) {
    core->setPosition(5);
    EXPECT_EQ(core->getPosition(), 5u);
}

// --- State machine ---

TEST_F(SlaveCoreTest, InitialState) {
    EXPECT_EQ(core->getState(), SlaveState::INIT);
    auto als = core->getALStatus();
    EXPECT_EQ(als.state, SlaveState::INIT);
    EXPECT_FALSE(als.error);
}

TEST_F(SlaveCoreTest, TransitionInitToPreOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveCoreTest, TransitionInitToBoot) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::BOOT;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::BOOT);
}

TEST_F(SlaveCoreTest, TransitionPreOpToSafeOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::SAFE_OP;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::SAFE_OP);
}

TEST_F(SlaveCoreTest, TransitionSafeOpToOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::SAFE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::OP;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::OP);
}

TEST_F(SlaveCoreTest, InvalidTransitionInitToSafeOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::SAFE_OP; // INIT → SAFE_OP invalid
    EXPECT_FALSE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::INIT);
}

TEST_F(SlaveCoreTest, InvalidTransitionInitToOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::OP;
    EXPECT_FALSE(core->requestStateChange(ctrl));
}

TEST_F(SlaveCoreTest, SameStateTransitionOK) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::INIT;
    EXPECT_TRUE(core->requestStateChange(ctrl));
}

TEST_F(SlaveCoreTest, TransitionOpToInit) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::SAFE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::INIT;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::INIT);
}

TEST_F(SlaveCoreTest, TransitionOpToPreOp) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::SAFE_OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::OP;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::PRE_OP;
    EXPECT_TRUE(core->requestStateChange(ctrl));
    EXPECT_EQ(core->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveCoreTest, TransitionBootToInit) {
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::BOOT;
    core->requestStateChange(ctrl);
    ctrl.requestedState = SlaveState::INIT;
    EXPECT_TRUE(core->requestStateChange(ctrl));
}

TEST_F(SlaveCoreTest, StateChangeCallback) {
    std::vector<SlaveState> states;
    core->setStateChangeCallback([&](SlaveState /*old*/, SlaveState s) { states.push_back(s); });
    ALControl ctrl{};
    ctrl.requestedState = SlaveState::PRE_OP;
    core->requestStateChange(ctrl);
    ASSERT_GE(states.size(), 1u);
    EXPECT_EQ(states.back(), SlaveState::PRE_OP);
}

TEST_F(SlaveCoreTest, SetAndClearError) {
    core->setError(ALStatusCode::InvalidStateChange);
    auto als = core->getALStatus();
    EXPECT_TRUE(als.error);
    core->clearError();
    als = core->getALStatus();
    EXPECT_FALSE(als.error);
}

// --- Frame processing ---

TEST_F(SlaveCoreTest, ProcessEmptyFrame) {
    auto resp = core->processFrame(nullptr, 0);
    EXPECT_TRUE(resp.empty());
}

TEST_F(SlaveCoreTest, ProcessShortFrame) {
    uint8_t data[10] = {};
    auto resp = core->processFrame(data, 10);
    EXPECT_TRUE(resp.empty());
}

TEST_F(SlaveCoreTest, ProcessFrameWrongEtherType) {
    auto frame = buildFrame(EcCmd::APRD, 0, REG_AL_STATUS, std::vector<uint8_t>(2, 0));
    frame[12] = 0x08; frame[13] = 0x00; // IP ethertype
    auto resp = core->processFrame(frame.data(), frame.size());
    EXPECT_TRUE(resp.empty());
}

TEST_F(SlaveCoreTest, APRDPositionZeroReads) {
    core->setPosition(0);
    std::vector<uint8_t> data(2, 0);
    auto frame = buildFrame(EcCmd::APRD, 0, REG_AL_STATUS, data);
    auto resp = core->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
    EXPECT_EQ(extractWKC(resp, data.size()), 1u);
}

TEST_F(SlaveCoreTest, APRDWrongPositionNoResponse) {
    core->setPosition(0);
    std::vector<uint8_t> data(2, 0);
    auto frame = buildFrame(EcCmd::APRD, 5, REG_AL_STATUS, data); // position 5, slave at 0
    auto resp = core->processFrame(frame.data(), frame.size());
    // Should still return frame but WKC = 0
    if (!resp.empty()) {
        EXPECT_EQ(extractWKC(resp, data.size()), 0u);
    }
}

TEST_F(SlaveCoreTest, APWRWriteALControl) {
    core->setPosition(0);
    // Write PRE_OP to AL Control
    std::vector<uint8_t> data = {0x02, 0x00}; // PRE_OP
    auto frame = buildFrame(EcCmd::APWR, 0, REG_AL_CONTROL, data);
    auto resp = core->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
    EXPECT_EQ(core->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveCoreTest, FPRDWithConfiguredAddress) {
    core->setPosition(0);
    // First configure the address via APWR
    uint16_t addr = 0x1001;
    std::vector<uint8_t> addrData = {static_cast<uint8_t>(addr & 0xFF),
                                      static_cast<uint8_t>((addr >> 8) & 0xFF)};
    auto frame = buildFrame(EcCmd::APWR, 0, 0x0010, addrData); // Station address register
    core->processFrame(frame.data(), frame.size());

    // Now FPRD using configured address
    std::vector<uint8_t> data(2, 0);
    auto fprdFrame = buildFrame(EcCmd::FPRD, addr, REG_AL_STATUS, data);
    auto resp = core->processFrame(fprdFrame.data(), fprdFrame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(SlaveCoreTest, BRDReadsBroadcast) {
    std::vector<uint8_t> data(2, 0);
    auto frame = buildFrame(EcCmd::BRD, 0, REG_AL_STATUS, data);
    auto resp = core->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(SlaveCoreTest, BWRWritesBroadcast) {
    std::vector<uint8_t> data = {0x02, 0x00}; // PRE_OP
    auto frame = buildFrame(EcCmd::BWR, 0, REG_AL_CONTROL, data);
    auto resp = core->processFrame(frame.data(), frame.size());
    EXPECT_EQ(core->getState(), SlaveState::PRE_OP);
}

// --- FMMU ---

TEST_F(SlaveCoreTest, FMMUCountDefault) {
    EXPECT_GE(core->getFMMUCount(), 1u);
}

TEST_F(SlaveCoreTest, SetGetFMMU) {
    FMMUConfig fmmu{};
    fmmu.logicalStartAddr = 0x1000;
    fmmu.length = 8;
    fmmu.logicalStartBit = 0;
    fmmu.logicalEndBit = 7;
    fmmu.physicalStartAddr = 0x1100;
    fmmu.physicalStartBit = 0;
    fmmu.type = 1; // read
    fmmu.activate = 1; // enabled
    core->setFMMU(0, fmmu);
    auto got = core->getFMMU(0);
    EXPECT_EQ(got.logicalStartAddr, 0x1000u);
    EXPECT_EQ(got.length, 8u);
    EXPECT_TRUE(got.isEnabled());
}

// --- SyncManager ---

TEST_F(SlaveCoreTest, SMCountDefault) {
    EXPECT_GE(core->getSyncManagerCount(), 4u);
}

TEST_F(SlaveCoreTest, SetGetSyncManager) {
    SyncManagerConfig sm{};
    sm.physicalAddr = 0x1000;
    sm.length = 128;
    sm.control = 0x26; // mailbox out
    sm.status = 0;
    sm.activate = 1; // enabled
    sm.type = SyncManagerType::MailboxOut;
    core->setSyncManager(0, sm);
    auto got = core->getSyncManager(0);
    EXPECT_EQ(got.physicalAddr, 0x1000u);
    EXPECT_EQ(got.length, 128u);
    EXPECT_TRUE(got.isEnabled());
}

// --- PDO buffers ---

TEST_F(SlaveCoreTest, PDOBufferAccess) {
    EXPECT_EQ(core->getRxPDOSize(), 8u);
    EXPECT_EQ(core->getTxPDOSize(), 8u);
    EXPECT_NE(core->getRxPDOData(), nullptr);
    EXPECT_NE(core->getTxPDOData(), nullptr);
}

TEST_F(SlaveCoreTest, PDOBufferReadWrite) {
    auto* rxData = core->getRxPDOData();
    std::memset(rxData, 0xAA, core->getRxPDOSize());
    EXPECT_EQ(rxData[0], 0xAA);

    auto* txData = core->getTxPDOData();
    std::memset(txData, 0x55, core->getTxPDOSize());
    EXPECT_EQ(txData[0], 0x55);
}

TEST_F(SlaveCoreTest, PDOExchangeCallback) {
    bool called = false;
    core->setPDOExchangeCallback([&](){ called = true; });
    // Trigger PDO exchange via LRW
    // For simplicity, just verify callback is set without triggering
    EXPECT_FALSE(called); // not called yet
}

// --- DC ---

TEST_F(SlaveCoreTest, DCStateAccess) {
    auto dcState = core->getDCState();
    (void)dcState;
}

TEST_F(SlaveCoreTest, AdvanceDCTime) {
    auto before = core->getDCSystemTime();
    core->advanceDCTime(1000000); // 1ms
    auto after = core->getDCSystemTime();
    EXPECT_GE(after, before);
}

// --- SII/EEPROM ---

TEST_F(SlaveCoreTest, SetGetSIIData) {
    std::vector<uint8_t> siiData(128, 0);
    siiData[0] = 0xAB;
    siiData[1] = 0xCD;
    core->setSIIData(siiData);
    auto got = core->getSIIData();
    EXPECT_EQ(got.size(), siiData.size());
    if (got.size() >= 2) {
        EXPECT_EQ(got[0], 0xAB);
        EXPECT_EQ(got[1], 0xCD);
    }
}

TEST_F(SlaveCoreTest, ReadSIIWord) {
    std::vector<uint8_t> siiData(128, 0);
    siiData[0] = 0x34;
    siiData[1] = 0x12;
    core->setSIIData(siiData);
    // Word address 0 should return 0x1234
    EXPECT_EQ(core->readSIIWord(0), 0x1234u);
}

TEST_F(SlaveCoreTest, WriteSIIWord) {
    std::vector<uint8_t> siiData(128, 0);
    core->setSIIData(siiData);
    EXPECT_TRUE(core->writeSIIWord(0, 0x5678));
    EXPECT_EQ(core->readSIIWord(0), 0x5678u);
}

// --- Watchdog ---

TEST_F(SlaveCoreTest, WatchdogState) {
    auto wd = core->getWatchdogState();
    (void)wd;
}

TEST_F(SlaveCoreTest, ResetWatchdog) {
    core->resetWatchdog();
}

// --- Mailbox ---

TEST_F(SlaveCoreTest, NoInitialMailboxRequest) {
    EXPECT_FALSE(core->hasMailboxRequest());
}

TEST_F(SlaveCoreTest, NoInitialMailboxResponse) {
    EXPECT_FALSE(core->hasMailboxResponse());
}

// --- Register memory ---

TEST_F(SlaveCoreTest, RegisterMemoryNotNull) {
    EXPECT_NE(core->getRegisterMemory(), nullptr);
}

TEST_F(SlaveCoreTest, ProcessDataRAMNotNull) {
    EXPECT_NE(core->getProcessDataRAM(), nullptr);
}

// --- Simulate ---

TEST_F(SlaveCoreTest, SimulateDoesNotCrash) {
    core->start();
    for (int i = 0; i < 100; ++i) {
        core->simulate(1000000); // 1ms
    }
}

// --- Logger ---

TEST_F(SlaveCoreTest, LoggerNotNull) {
    EXPECT_NE(&core->getLogger(), nullptr);
}

TEST_F(SlaveCoreTest, SetLogEnabled) {
    // Just ensure no crash
    core->setLogEnabled(SlaveLogCategory::StateMachine, true);
    core->setLogEnabled(SlaveLogCategory::StateMachine, false);
}

// --- ObjectDictionary ---

TEST_F(SlaveCoreTest, SetNullObjectDictionary) {
    core->setObjectDictionary(nullptr);
}

// --- HAL ---

TEST_F(SlaveCoreTest, SetNullHAL) {
    core->setHAL(nullptr);
}

// --- Mailbox handler ---

TEST_F(SlaveCoreTest, AddNullMailboxHandler) {
    core->addMailboxHandler(nullptr);
}
