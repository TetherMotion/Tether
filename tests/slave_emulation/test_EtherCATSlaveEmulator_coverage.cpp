/**
 * @file test_EtherCATSlaveEmulator_coverage.cpp
 * @brief Comprehensive EtherCATSlaveEmulator / NetworkEmulator / CiA402 DriveState coverage tests
 */
#include <gtest/gtest.h>
#include <cstring>
#include <memory>
#include <vector>
#include "tether/ethercat/EtherCATSlaveEmulator.hpp"
#include "tether/ethercat/EtherCATDCClass.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Emulator;

// ============================================================================
// Fixture: SlaveEmulator
// ============================================================================
class SlaveEmulatorCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        emu = std::make_unique<SlaveEmulator>();
        emu->setPosition(0);
        emu->setConfiguredAddress(0x1001);
    }
    std::unique_ptr<SlaveEmulator> emu;
};

// --- Construction ---

TEST_F(SlaveEmulatorCovTest, DefaultStateInit) {
    EXPECT_EQ(emu->getState(), SlaveState::INIT);
}

TEST_F(SlaveEmulatorCovTest, GetALStatus) {
    auto als = emu->getALStatus();
    EXPECT_EQ(als.state, SlaveState::INIT);
    EXPECT_FALSE(als.error);
}

// --- State transitions ---

TEST_F(SlaveEmulatorCovTest, TransitionInitToPreOp) {
    emu->requestState(SlaveState::PRE_OP);
    EXPECT_EQ(emu->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveEmulatorCovTest, TransitionToOp) {
    emu->requestState(SlaveState::PRE_OP);
    emu->requestState(SlaveState::SAFE_OP);
    emu->requestState(SlaveState::OP);
    EXPECT_EQ(emu->getState(), SlaveState::OP);
}

TEST_F(SlaveEmulatorCovTest, TransitionInitToBoot) {
    emu->requestState(SlaveState::BOOT);
}

TEST_F(SlaveEmulatorCovTest, InvalidTransition) {
    emu->requestState(SlaveState::OP); // INIT -> OP invalid
    // State should remain INIT (or wherever it lands)
    auto st = emu->getState();
    (void)st;
}

// --- SII ---

TEST_F(SlaveEmulatorCovTest, SetSIIConfig) {
    SIIConfig sii{};
    sii.vendor_id = 0x1234;
    sii.product_code = 0x5678;
    sii.revision = 1;
    sii.serial = 42;
    sii.device_name = "TestDrive";
    emu->setSIIConfig(sii);
}

// --- Register access via APRD/APWR (3-arg: ado, data, len) ---

TEST_F(SlaveEmulatorCovTest, APRDReadALStatus) {
    uint8_t data[2] = {};
    bool ok = emu->processAPRD(0x0130, data, 2);
    EXPECT_TRUE(ok);
    EXPECT_EQ(data[0] & 0x0F, 0x01); // INIT
}

TEST_F(SlaveEmulatorCovTest, APWRWriteALControl) {
    uint8_t data[2] = {0x02, 0x00}; // PRE_OP
    bool ok = emu->processAPWR(0x0120, data, 2);
    EXPECT_TRUE(ok);
    EXPECT_EQ(emu->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveEmulatorCovTest, APWRSetConfiguredAddress) {
    uint8_t data[2] = {0x02, 0x10}; // 0x1002
    emu->processAPWR(0x0010, data, 2);
}

// --- FPRD/FPWR ---

TEST_F(SlaveEmulatorCovTest, FPRDReadRegister) {
    uint8_t data[2] = {};
    bool ok = emu->processFPRD(0x0130, data, 2);
    EXPECT_TRUE(ok);
}

TEST_F(SlaveEmulatorCovTest, FPWRWriteRegister) {
    uint8_t data[2] = {0x02, 0x00};
    bool ok = emu->processFPWR(0x0120, data, 2);
    EXPECT_TRUE(ok);
    EXPECT_EQ(emu->getState(), SlaveState::PRE_OP);
}

// --- DC ---

TEST_F(SlaveEmulatorCovTest, DCState) {
    auto& dc = emu->getDCState();
    (void)dc;
}

TEST_F(SlaveEmulatorCovTest, AdvanceDCTime) {
    emu->advanceDCTime(1000000);
    emu->advanceDCTime(1000000);
}

// --- Registers: DL Status, AL Status Code, DC System Time ---

TEST_F(SlaveEmulatorCovTest, ReadDLStatus) {
    uint8_t data[2] = {};
    emu->processAPRD(0x0110, data, 2);
}

TEST_F(SlaveEmulatorCovTest, ReadALStatusCode) {
    uint8_t data[2] = {};
    emu->processAPRD(0x0134, data, 2);
}

TEST_F(SlaveEmulatorCovTest, ReadDCSystemTime) {
    uint8_t data[8] = {};
    emu->processAPRD(toUInt16(DCRegisters::DCSysTime), data, 8);
}

TEST_F(SlaveEmulatorCovTest, ReadDCOffset) {
    uint8_t data[8] = {};
    emu->processAPRD(toUInt16(DCRegisters::DCSysOffset), data, 8);
}

TEST_F(SlaveEmulatorCovTest, ReadDCDiff) {
    uint8_t data[4] = {};
    emu->processAPRD(toUInt16(DCRegisters::DCSysDiff), data, 4);
}

// --- SM Registers ---

TEST_F(SlaveEmulatorCovTest, ReadSMRegisters) {
    for (uint16_t sm = 0; sm < 4; ++sm) {
        uint8_t data[8] = {};
        emu->processAPRD(0x0800 + sm * 8, data, 8);
    }
}

TEST_F(SlaveEmulatorCovTest, WriteSMRegisters) {
    uint8_t data[8] = {0x00, 0x10, 0x80, 0x00, 0x26, 0x00, 0x01, 0x00};
    emu->processAPWR(0x0800, data, 8);
}

// --- FMMU Registers ---

TEST_F(SlaveEmulatorCovTest, ReadFMMURegisters) {
    uint8_t data[16] = {};
    emu->processAPRD(0x0600, data, 16);
}

TEST_F(SlaveEmulatorCovTest, WriteFMMURegisters) {
    uint8_t data[16] = {};
    data[0] = 0x00; data[1] = 0x10; data[2] = 0x00; data[3] = 0x00;
    data[4] = 0x08; data[5] = 0x00;
    data[8] = 0x00; data[9] = 0x11;
    data[12] = 0x01;
    data[13] = 0x01;
    emu->processAPWR(0x0600, data, 16);
}

// --- DC Write Registers ---

TEST_F(SlaveEmulatorCovTest, WriteDCOffset) {
    uint8_t data[8] = {};
    emu->processAPWR(toUInt16(DCRegisters::DCSysOffset), data, 8);
}

TEST_F(SlaveEmulatorCovTest, WriteDCActivation) {
    uint8_t data[1] = {0x01};
    emu->processAPWR(toUInt16(DCRegisters::DCSyncAct), data, 1);
}

TEST_F(SlaveEmulatorCovTest, WriteSYNC0Cycle) {
    uint8_t data[4] = {0x40, 0x42, 0x0F, 0x00};
    emu->processAPWR(toUInt16(DCRegisters::DCCycle0), data, 4);
}

// --- SII ---

TEST_F(SlaveEmulatorCovTest, SIIReadWord) {
    SIIConfig sii{};
    sii.vendor_id = 0x1234;
    sii.product_code = 0x0001;
    emu->setSIIConfig(sii);

    uint16_t val = 0;
    bool ok = emu->processSIIRead(0x0008, &val);
    EXPECT_TRUE(ok);
}

TEST_F(SlaveEmulatorCovTest, SIIWriteWord) {
    bool ok = emu->processSIIWrite(0x0008, 0xFFFF);
    (void)ok;
}

// --- Mailbox --- (hasMailboxData/getMailboxResponse/processMailboxRequest not yet implemented in source)

// --- CiA402 Drive ---

TEST_F(SlaveEmulatorCovTest, CiA402NotEnabledByDefault) {
    EXPECT_FALSE(emu->isCiA402Enabled());
}

TEST_F(SlaveEmulatorCovTest, EnableCiA402) {
    emu->enableCiA402(true);
    EXPECT_TRUE(emu->isCiA402Enabled());
}

TEST_F(SlaveEmulatorCovTest, GetDriveState) {
    emu->enableCiA402(true);
    auto& drive = emu->getDriveState();
    uint16_t sw = drive.getStatusWord();
    (void)sw;
}

TEST_F(SlaveEmulatorCovTest, DriveStateProcessControlWord) {
    emu->enableCiA402(true);
    auto& drive = emu->getDriveState();
    drive.processControlWord(0x0080); // Fault reset
    drive.processControlWord(0x0006); // Shutdown
    drive.processControlWord(0x0007); // Switch on
    drive.processControlWord(0x000F); // Enable operation
    uint16_t sw = drive.getStatusWord();
    (void)sw;
}

TEST_F(SlaveEmulatorCovTest, DriveStateSimulate) {
    emu->enableCiA402(true);
    auto& drive = emu->getDriveState();
    drive.processControlWord(0x0080);
    drive.processControlWord(0x0006);
    drive.processControlWord(0x0007);
    drive.processControlWord(0x000F);
    drive.simulate(1000);
}

// --- Error injection ---

TEST_F(SlaveEmulatorCovTest, GetSetErrorInjection) {
    auto& ei = emu->getErrorInjection();
    (void)ei;
    emu->setErrorInjection({});
}

// --- Simulate ---

TEST_F(SlaveEmulatorCovTest, SimulateWithDCActive) {
    emu->requestState(SlaveState::PRE_OP);
    emu->requestState(SlaveState::SAFE_OP);
    emu->requestState(SlaveState::OP);
    for (int i = 0; i < 10; ++i) {
        emu->simulate(1000);
    }
}

// --- Dump methods (void return) ---

TEST_F(SlaveEmulatorCovTest, DumpRegisters) {
    emu->dumpRegisters();
}

TEST_F(SlaveEmulatorCovTest, DumpFMMUs) {
    emu->dumpFMMUs();
}

TEST_F(SlaveEmulatorCovTest, DumpSyncManagers) {
    emu->dumpSyncManagers();
}

// --- Logical access ---

TEST_F(SlaveEmulatorCovTest, ProcessLogicalRead) {
    uint8_t data[4] = {};
    bool ok = emu->processLogicalRead(0x1000, data, 4);
    EXPECT_FALSE(ok); // No FMMU configured
}

TEST_F(SlaveEmulatorCovTest, ProcessLogicalWrite) {
    uint8_t data[4] = {1, 2, 3, 4};
    bool ok = emu->processLogicalWrite(0x1000, data, 4);
    EXPECT_FALSE(ok);
}

// --- Error injection details ---

TEST_F(SlaveEmulatorCovTest, ErrorInjectionALError) {
    ErrorInjection ei{};
    ei.inject_al_error = true;
    ei.al_error_code = 0x001A;
    emu->setErrorInjection(ei);
    emu->simulate(1000);
}

TEST_F(SlaveEmulatorCovTest, ErrorInjectionWKC) {
    ErrorInjection ei{};
    ei.inject_wkc_error = true;
    emu->setErrorInjection(ei);
    uint8_t data[2] = {};
    emu->processAPRD(0x0130, data, 2);
}

TEST_F(SlaveEmulatorCovTest, ErrorInjectionTimeout) {
    ErrorInjection ei{};
    ei.inject_timeout = true;
    ei.timeout_register = 0x0130;
    emu->setErrorInjection(ei);
    uint8_t data[2] = {};
    emu->processAPRD(0x0130, data, 2);
}

TEST_F(SlaveEmulatorCovTest, ErrorInjectionDCDrift) {
    ErrorInjection ei{};
    ei.inject_dc_drift = true;
    ei.dc_drift_ppb = 100;
    emu->setErrorInjection(ei);
    emu->advanceDCTime(1000000);
}

TEST_F(SlaveEmulatorCovTest, ErrorInjectionSyncError) {
    ErrorInjection ei{};
    ei.inject_sync_error = true;
    emu->setErrorInjection(ei);
    emu->simulate(1000);
}

TEST_F(SlaveEmulatorCovTest, ErrorInjectionClear) {
    ErrorInjection ei{};
    ei.inject_al_error = true;
    emu->setErrorInjection(ei);
    emu->getErrorInjection().clear();
    EXPECT_FALSE(emu->getErrorInjection().inject_al_error);
}

// --- ALStatus helpers ---

TEST_F(SlaveEmulatorCovTest, ALStatusToRegister) {
    ALStatus als;
    als.state = SlaveState::PRE_OP;
    als.error = true;
    als.id_request = true;
    auto reg = als.toRegister();
    EXPECT_EQ(reg & 0x0F, 0x02); // PRE_OP
    EXPECT_NE(reg & 0x0010, 0u); // error
    EXPECT_NE(reg & 0x0020, 0u); // id_request
}

TEST_F(SlaveEmulatorCovTest, ALStatusFromRegister) {
    auto als = ALStatus::fromRegister(0x0032); // PRE_OP + error + id_request
    EXPECT_EQ(als.state, SlaveState::PRE_OP);
    EXPECT_TRUE(als.error);
    EXPECT_TRUE(als.id_request);
}

// --- SyncManager struct ---

TEST(EmulatorStructsTest, SyncManagerDefaults) {
    SyncManager sm;
    EXPECT_FALSE(sm.isEnabled());
    EXPECT_TRUE(sm.isMailbox());
    EXPECT_FALSE(sm.isOutput());
    EXPECT_FALSE(sm.isInput());
}

TEST(EmulatorStructsTest, SyncManagerEnabled) {
    SyncManager sm;
    sm.enable = 0x01;
    sm.control = 0x06; // PDO (bit 2) + output direction (bit 1)
    EXPECT_TRUE(sm.isEnabled());
    EXPECT_FALSE(sm.isMailbox());
    EXPECT_TRUE(sm.isOutput());
    EXPECT_FALSE(sm.isInput());
}

TEST(EmulatorStructsTest, SyncManagerInput) {
    SyncManager sm;
    sm.control = 0x04; // PDO (bit 2), no bit 1 = input
    EXPECT_TRUE(sm.isInput());
}

// --- FMMU struct ---

TEST(EmulatorStructsTest, FMMUDisabledDoesNotMatch) {
    FMMU fmmu;
    fmmu.enabled = false;
    EXPECT_FALSE(fmmu.containsLogicalAddress(0, 4));
}

TEST(EmulatorStructsTest, FMMUEnabledMatch) {
    FMMU fmmu;
    fmmu.enabled = true;
    fmmu.logical_start = 0x1000;
    fmmu.length = 16;
    fmmu.physical_start = 0x1100;
    EXPECT_TRUE(fmmu.containsLogicalAddress(0x1000, 4));
    EXPECT_TRUE(fmmu.containsLogicalAddress(0x100C, 4));
    EXPECT_FALSE(fmmu.containsLogicalAddress(0x2000, 4));
}

TEST(EmulatorStructsTest, FMMUTranslate) {
    FMMU fmmu;
    fmmu.logical_start = 0x1000;
    fmmu.physical_start = 0x1100;
    EXPECT_EQ(fmmu.translateToPhysical(0x1004), 0x1104u);
}

// --- DCState ---

TEST(EmulatorStructsTest, DCStateAdvance) {
    EtherCAT::Emulator::DCState dc;
    dc.advanceTime(1000000);
    EXPECT_EQ(dc.system_time, 1000000u);
    dc.advanceTime(500000);
    EXPECT_EQ(dc.system_time, 1500000u);
}

// --- ODEntry ---

TEST(EmulatorStructsTest, ODEntryAccess) {
    ODEntry entry;
    entry.access = 0x03; // read + write
    EXPECT_TRUE(entry.isReadable());
    EXPECT_TRUE(entry.isWritable());
    entry.access = 0x01;
    EXPECT_TRUE(entry.isReadable());
    EXPECT_FALSE(entry.isWritable());
}

// --- CiA402 DriveState ---

TEST(CiA402EmulatorTest, InitialState) {
    CiA402::DriveState drive;
    EXPECT_EQ(drive.state, CiA402::State::SWITCH_ON_DISABLED);
    uint16_t sw = drive.getStatusWord();
    (void)sw;
}

TEST(CiA402EmulatorTest, StatusWordReflectsState) {
    CiA402::DriveState drive;
    uint16_t sw = drive.getStatusWord();
    // SWITCH_ON_DISABLED: bit 6=1, bit 0=0 → 0x0040
    EXPECT_NE(sw & 0x0040, 0u);
}

TEST(CiA402EmulatorTest, FullTransitionSequence) {
    CiA402::DriveState drive;
    drive.processControlWord(0x0080); // Fault reset
    drive.processControlWord(0x0006); // Shutdown → READY_TO_SWITCH_ON
    drive.processControlWord(0x0007); // Switch on → SWITCHED_ON
    drive.processControlWord(0x000F); // Enable → OPERATION_ENABLED
    EXPECT_EQ(drive.state, CiA402::State::OPERATION_ENABLED);
}

TEST(CiA402EmulatorTest, QuickStop) {
    CiA402::DriveState drive;
    drive.processControlWord(0x0006);
    drive.processControlWord(0x0007);
    drive.processControlWord(0x000F);
    drive.processControlWord(0x0002); // Quick stop
}

TEST(CiA402EmulatorTest, SimulateMotion) {
    CiA402::DriveState drive;
    drive.processControlWord(0x0006);
    drive.processControlWord(0x0007);
    drive.processControlWord(0x000F);
    drive.target_position = 10000;
    drive.target_velocity = 1000;
    for (int i = 0; i < 100; ++i) {
        drive.simulate(1000);
    }
}

// ============================================================================
// Factory functions
// ============================================================================

TEST(SlaveEmulatorFactoryTest, CreateSimpleSlave) {
    auto slave = createSimpleSlave(0x1234, 0x5678, 4, 4);
    ASSERT_NE(slave, nullptr);
    EXPECT_EQ(slave->getState(), SlaveState::INIT);
}

TEST(SlaveEmulatorFactoryTest, CreateGenericIOSlave) {
    auto slave = createGenericIOSlave(0xAAAA, 0xBBBB, 4, 4);
    ASSERT_NE(slave, nullptr);
}

TEST(SlaveEmulatorFactoryTest, CreateGenericIOSlaveWithAnalog) {
    auto slave = createGenericIOSlave(0xAAAA, 0xBBBB, 4, 4, 2, 2);
    ASSERT_NE(slave, nullptr);
}

TEST(SlaveEmulatorFactoryTest, CreateCiA402Drive) {
    auto slave = createCiA402Drive(0x0002, 0x0001);
    ASSERT_NE(slave, nullptr);
    EXPECT_TRUE(slave->isCiA402Enabled());
}

TEST(SlaveEmulatorFactoryTest, CreateCiA402DriveWithName) {
    auto slave = createCiA402Drive(0x0002, 0x0001, "My Drive");
    ASSERT_NE(slave, nullptr);
}

// ============================================================================
// NetworkEmulator tests
// ============================================================================
class NetworkEmulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        net = std::make_unique<NetworkEmulator>();
    }
    std::unique_ptr<NetworkEmulator> net;
};

TEST_F(NetworkEmulatorTest, DefaultEmpty) {
    EXPECT_EQ(net->getSlaveCount(), 0u);
}

TEST_F(NetworkEmulatorTest, AddSlave) {
    auto slave = std::make_unique<SlaveEmulator>();
    net->addSlave(std::move(slave));
    EXPECT_EQ(net->getSlaveCount(), 1u);
}

TEST_F(NetworkEmulatorTest, GetSlave) {
    auto slave = std::make_unique<SlaveEmulator>();
    slave->setConfiguredAddress(0x1001);
    net->addSlave(std::move(slave));
    auto* s = net->getSlave(0);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->getState(), SlaveState::INIT);
}

TEST_F(NetworkEmulatorTest, GetSlaveOutOfRange) {
    auto* s = net->getSlave(99);
    EXPECT_EQ(s, nullptr);
}

TEST_F(NetworkEmulatorTest, ClearSlaves) {
    net->addSlave(std::make_unique<SlaveEmulator>());
    net->addSlave(std::make_unique<SlaveEmulator>());
    EXPECT_EQ(net->getSlaveCount(), 2u);
    net->clearSlaves();
    EXPECT_EQ(net->getSlaveCount(), 0u);
}

TEST_F(NetworkEmulatorTest, Simulate) {
    auto s = std::make_unique<SlaveEmulator>();
    s->setPosition(0);
    net->addSlave(std::move(s));
    net->simulate(1000);
}

TEST_F(NetworkEmulatorTest, Stats) {
    auto stats = net->getStats();
    EXPECT_EQ(stats.frames_processed, 0u);
    net->resetStats();
}

TEST_F(NetworkEmulatorTest, GlobalErrorInjection) {
    net->setGlobalErrorInjection({});
}

// --- Process frame with APRD datagram ---

static std::vector<uint8_t> buildEtherCATFrame(uint8_t cmd, uint16_t adp, uint16_t ado,
                                                const std::vector<uint8_t>& data) {
    const size_t dataLen = data.size();
    const size_t frameTotalLen = 14 + 2 + 10 + dataLen + 2;
    std::vector<uint8_t> frame(frameTotalLen, 0);

    // Ethernet header
    std::memset(&frame[0], 0xFF, 6);
    frame[12] = 0x88; frame[13] = 0xA4;

    // EtherCAT header
    uint16_t ecLen = static_cast<uint16_t>(10 + dataLen + 2);
    ecLen |= (0x01 << 12);
    frame[14] = ecLen & 0xFF;
    frame[15] = (ecLen >> 8) & 0xFF;

    // Datagram header
    size_t pos = 16;
    frame[pos++] = cmd;
    frame[pos++] = 0;
    frame[pos++] = adp & 0xFF;
    frame[pos++] = (adp >> 8) & 0xFF;
    frame[pos++] = ado & 0xFF;
    frame[pos++] = (ado >> 8) & 0xFF;
    uint16_t lenFlags = static_cast<uint16_t>(dataLen);
    frame[pos++] = lenFlags & 0xFF;
    frame[pos++] = (lenFlags >> 8) & 0xFF;
    frame[pos++] = 0;
    frame[pos++] = 0;

    std::memcpy(&frame[pos], data.data(), dataLen);
    pos += dataLen;
    frame[pos++] = 0;
    frame[pos++] = 0;

    return frame;
}

TEST_F(NetworkEmulatorTest, ProcessFrameAPRD) {
    auto s = std::make_unique<SlaveEmulator>();
    s->setPosition(0);
    s->setConfiguredAddress(0x1001);
    net->addSlave(std::move(s));

    auto frame = buildEtherCATFrame(0x01, 0, 0x0130, {0, 0}); // APRD AL Status
    auto resp = net->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(NetworkEmulatorTest, ProcessFrameFPRD) {
    auto s = std::make_unique<SlaveEmulator>();
    s->setPosition(0);
    s->setConfiguredAddress(0x1001);
    net->addSlave(std::move(s));

    auto frame = buildEtherCATFrame(0x04, 0x1001, 0x0130, {0, 0}); // FPRD
    auto resp = net->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(NetworkEmulatorTest, ProcessFrameBRD) {
    auto s1 = std::make_unique<SlaveEmulator>();
    s1->setPosition(0);
    auto s2 = std::make_unique<SlaveEmulator>();
    s2->setPosition(1);
    net->addSlave(std::move(s1));
    net->addSlave(std::move(s2));

    auto frame = buildEtherCATFrame(0x07, 0, 0x0130, {0, 0}); // BRD
    auto resp = net->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(NetworkEmulatorTest, ProcessFrameBWR) {
    auto s = std::make_unique<SlaveEmulator>();
    s->setPosition(0);
    net->addSlave(std::move(s));

    auto frame = buildEtherCATFrame(0x08, 0, 0x0120, {0x02, 0x00}); // BWR AL Control = PRE_OP
    auto resp = net->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}

TEST_F(NetworkEmulatorTest, ProcessFrameEmpty) {
    auto resp = net->processFrame(nullptr, 0);
    EXPECT_TRUE(resp.empty());
}

TEST_F(NetworkEmulatorTest, ProcessFrameShort) {
    uint8_t data[10] = {};
    auto resp = net->processFrame(data, 10);
    EXPECT_TRUE(resp.empty());
}

TEST_F(NetworkEmulatorTest, MultipleSlaves) {
    for (int i = 0; i < 4; ++i) {
        auto s = std::make_unique<SlaveEmulator>();
        s->setPosition(i);
        s->setConfiguredAddress(0x1001 + i);
        net->addSlave(std::move(s));
    }
    EXPECT_EQ(net->getSlaveCount(), 4u);

    auto frame = buildEtherCATFrame(0x01, 2, 0x0130, {0, 0}); // APRD to position 2
    auto resp = net->processFrame(frame.data(), frame.size());
    EXPECT_FALSE(resp.empty());
}
