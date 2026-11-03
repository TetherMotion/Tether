/**
 * @file test_rp20_device.cpp
 * @brief RP20Device module driver over a real Master ↔ SlaveEmulator
 *        wire (EmulatedBus).
 *
 * RP20Device is `EtherCAT::Master&`-driven: module scanning reads
 * per-slot diagnosis objects (0xA000 + slot*0x10 : 0x01), init and
 * configure write per-slot config objects (0x8000 + slot*0x10),
 * assignPDOs writes the CiA301 SM-assign records (0x1C12/0x1C13), and
 * registerPDOs binds PDO mapping buffers on the master side.  All of it
 * is exercised end-to-end with OD entries injected into the emulator.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/RP20/RP20Device.hpp"
#include "tether/drives/RP20/RP20Module.hpp"
#include "tether/drives/RP20/Registers/Common.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace RP20 = EtherCAT::Drives::Registers::RP20;
using EtherCAT::Drives::RP20Module::RP20Device;
using EtherCAT::Drives::RP20Module::ModuleType;
using EtherCAT::Drives::RP20Module::ModuleConfig;

namespace {

/// Slot-address helpers mirroring the device's own math.
constexpr uint16_t kDiagBase  = 0xA000;
constexpr uint16_t kCfgBase   = 0x8000;
constexpr uint16_t kSlotStep  = 0x10;
constexpr uint16_t kRxPDOBase = 0x1600;
constexpr uint16_t kTxPDOBase = 0x1A00;

uint16_t diagIdx(uint8_t slot) { return kDiagBase + slot * kSlotStep; }
uint16_t cfgIdx(uint8_t slot)  { return kCfgBase + slot * kSlotStep; }

class RP20DeviceTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000ACD, 0x5250, "rp20");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    static std::vector<uint8_t> le(uint32_t v, size_t n) {
        std::vector<uint8_t> d(n);
        for (size_t i = 0; i < n; ++i) d[i] = static_cast<uint8_t>(v >> (8 * i));
        return d;
    }

    void addU(uint16_t idx, uint8_t sub, uint32_t value, uint8_t bytes,
            uint8_t access = 0x03) {
        const uint16_t dt = (bytes == 1) ? 0x0005 : (bytes == 2) ? 0x0006 : 0x0007;
        slave_->addODEntry({idx, sub, dt, static_cast<uint16_t>(bytes * 8),
                            access, le(value, bytes), "x"});
    }

    /// Diagnosis object: 4-byte record, module id in the low byte of sub 1.
    void addModuleDiag(uint8_t slot, uint32_t ident) {
        slave_->addODEntry({diagIdx(slot), 0x00, 0x0005, 8, 0x01, {4}, "cnt"});
        slave_->addODEntry({diagIdx(slot), 0x01, 0x0007, 32, 0x01,
                            le(ident, 4), "diag"});
    }

    /// Full 16-subindex config record for a slot (all u8, rw).
    void addConfigRecord(uint8_t slot) {
        slave_->addODEntry({cfgIdx(slot), 0x00, 0x0005, 8, 0x01, {16}, "cnt"});
        for (uint8_t s = 1; s <= 16; ++s)
            addU(cfgIdx(slot), s, 0, 1);
    }

    /// CiA301 SM-assign records (0x1C12/0x1C13) with a few u16 subentries.
    void addSmAssignRecords() {
        for (uint16_t rec : {uint16_t(0x1C12), uint16_t(0x1C13)}) {
            slave_->addODEntry({rec, 0x00, 0x0005, 8, 0x03, {0}, "cnt"});
            for (uint8_t s = 1; s <= 8; ++s)
                addU(rec, s, 0, 2);
        }
    }

    Master master_;
    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
};

// ---------------------------------------------------------------------------
// scanModules
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, ScanModulesFindsInjectedModules) {
    addModuleDiag(0, 0x04);  // DI_16
    addModuleDiag(5, 0x08);  // DO_16_PNP

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_EQ(dev.modules().size(), 2u);
    EXPECT_EQ(dev.modules()[0].slot, 0);
    EXPECT_EQ(dev.modules()[0].descriptor->type, ModuleType::DI_16);
    EXPECT_EQ(dev.modules()[1].slot, 5);
    EXPECT_EQ(dev.modules()[1].descriptor->type, ModuleType::DO_16_PNP);
}

TEST_F(RP20DeviceTest, ScanModulesSkipsUnknownAndZeroIds) {
    addModuleDiag(0, 0x7F);  // not in the catalog
    addModuleDiag(2, 0x00);  // empty slot id
    addModuleDiag(4, 0x0C);  // Multi_DIO_8

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_EQ(dev.modules().size(), 1u);
    EXPECT_EQ(dev.modules()[0].slot, 4);
    EXPECT_EQ(dev.modules()[0].descriptor->type, ModuleType::Multi_DIO_8);
}

TEST_F(RP20DeviceTest, ScanModulesEmptyWhenNoDiagObjects) {
    RP20Device dev(master_);
    EXPECT_FALSE(dev.scanModules());  // returns !modules_.empty()
    EXPECT_TRUE(dev.modules().empty());
}

// ---------------------------------------------------------------------------
// sendInitCommands
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, SendInitCommandsWritesSlotConfig) {
    addModuleDiag(0, 0x04);  // DI_16 → 4 u8 init cmds at cfg:01..04
    addConfigRecord(0);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    EXPECT_TRUE(dev.sendInitCommands(0));
    for (uint8_t s = 1; s <= 4; ++s) {
        const auto* e = slave_->odEntry(cfgIdx(0), s);
        ASSERT_NE(e, nullptr);
        EXPECT_EQ(e->data[0], 0) << "sub " << int(s);
    }
}

TEST_F(RP20DeviceTest, SendInitCommandsFailsWithoutConfigObjects) {
    addModuleDiag(0, 0x04);  // DI_16, but no 0x8000 record

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    EXPECT_FALSE(dev.sendInitCommands(0));
}

TEST_F(RP20DeviceTest, SendInitCommandsAllTargetsPerSlotIndices) {
    addModuleDiag(0, 0x04);  // DI_16 slot 0 → cfg 0x8000
    addModuleDiag(1, 0x0C);  // Multi_DIO_8 slot 1 → cfg 0x8010
    addConfigRecord(0);
    addConfigRecord(1);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    EXPECT_TRUE(dev.sendInitCommandsAll());

    // Slot-1 write went to 0x8010, not 0x8000.
    const auto* e = slave_->odEntry(cfgIdx(1), 0x01);
    ASSERT_NE(e, nullptr);
    EXPECT_TRUE(dev.sendInitCommands(9) == false);  // out of range
}

// ---------------------------------------------------------------------------
// configureModule
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, ConfigureModuleTCWritesSignalFormFilterCjc) {
    addModuleDiag(0, 0x12);  // TC_4
    addConfigRecord(0);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());

    ModuleConfig cfg;
    cfg.tc_signal_form = RP20::TCSignalForm::Type_S;
    cfg.tc_filter = RP20::FilteringMode::Average;
    cfg.tc_cjc = RP20::ColdJunctionCompensation::External;
    ASSERT_TRUE(dev.configureModule(0, cfg));

    // 4 channels each: type at subs 1-4, filter 5-8, cjc 9-12.
    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 1)->data[0],
              static_cast<uint8_t>(RP20::TCSignalForm::Type_S));
    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 4)->data[0],
              static_cast<uint8_t>(RP20::TCSignalForm::Type_S));
    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 5)->data[0],
              static_cast<uint8_t>(RP20::FilteringMode::Average));
    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 9)->data[0],
              static_cast<uint8_t>(RP20::ColdJunctionCompensation::External));
}

TEST_F(RP20DeviceTest, ConfigureModuleAIWritesTypeAndFilter) {
    addModuleDiag(0, 0x10);  // AI_4
    addConfigRecord(0);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());

    ModuleConfig cfg;
    cfg.ai_signal_form = RP20::AISignalForm::Current_4_20mA;
    cfg.ai_filter = RP20::FilteringMode::Average;
    ASSERT_TRUE(dev.configureModule(0, cfg));

    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 1)->data[0],
              static_cast<uint8_t>(RP20::AISignalForm::Current_4_20mA));
    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 5)->data[0],
              static_cast<uint8_t>(RP20::FilteringMode::Average));
}

TEST_F(RP20DeviceTest, ConfigureModuleRDWritesTypeAndFilter) {
    addModuleDiag(0, 0x11);  // RD_4
    addConfigRecord(0);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());

    ModuleConfig cfg;
    cfg.rd_signal_form = RP20::RTDSignalForm::PT100;
    ASSERT_TRUE(dev.configureModule(0, cfg));

    EXPECT_EQ(slave_->odEntry(cfgIdx(0), 1)->data[0],
              static_cast<uint8_t>(RP20::RTDSignalForm::PT100));
}

TEST_F(RP20DeviceTest, ConfigureModuleInvalidIndexFails) {
    RP20Device dev(master_);
    EXPECT_FALSE(dev.configureModule(0, ModuleConfig{}));
    EXPECT_TRUE(dev.configureAllModules(ModuleConfig{}));  // empty loop → trivially ok
}

// ---------------------------------------------------------------------------
// registerPDOs + buffer access
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, RegisterPDOsBindsTxRxBuffers) {
    addModuleDiag(0, 0x0C);  // Multi_DIO_8: both tx and rx PDOs

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    const auto& mod = dev.modules()[0];
    EXPECT_GE(mod.tx_pdo_entry, 0);
    EXPECT_GE(mod.rx_pdo_entry, 0);
    EXPECT_FALSE(mod.tx_buffer.empty());
    EXPECT_FALSE(mod.rx_buffer.empty());
}

TEST_F(RP20DeviceTest, DigitalInputOutputRoundTripViaBuffers) {
    addModuleDiag(0, 0x0C);  // Multi_DIO_8 — 8 DI + 8 DO

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    // Inputs: poke the master-side mapping buffer directly.
    auto& mapping = master_.pdo().mapping();
    uint8_t* tx = mapping.entryDataMut(dev.modules()[0].tx_pdo_entry);
    ASSERT_NE(tx, nullptr);
    tx[0] = 0xA5;
    EXPECT_EQ(dev.readDigitalInputs(0, 0), 0xA5);

    // Outputs: write through the API, observe the bound rx_buffer.
    dev.writeDigitalOutputs(0, 0, 0x3C);
    EXPECT_EQ(dev.modules()[0].rx_buffer[0], 0x3C);
}

TEST_F(RP20DeviceTest, WriteOutputBitSetsAndClears) {
    addModuleDiag(0, 0x08);  // DO_16_PNP

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    dev.writeOutputBit(0, 0, 5, true);
    EXPECT_EQ(dev.modules()[0].rx_buffer[0] & 0x20, 0x20);
    dev.writeOutputBit(0, 0, 5, false);
    EXPECT_EQ(dev.modules()[0].rx_buffer[0] & 0x20, 0x00);
}

TEST_F(RP20DeviceTest, AnalogInOutRoundTrip) {
    addModuleDiag(0, 0x10);  // AI_4
    addModuleDiag(1, 0x20);  // AO_4

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    auto& mapping = master_.pdo().mapping();
    uint8_t* tx = mapping.entryDataMut(dev.modules()[0].tx_pdo_entry);
    ASSERT_NE(tx, nullptr);
    tx[0] = 0x34; tx[1] = 0x12;  // little-endian int16
    EXPECT_EQ(dev.readAnalogInput(0, 0), 0x1234);

    dev.writeAnalogOutput(1, 0, -1234);
    int16_t v;
    std::memcpy(&v, dev.modules()[1].rx_buffer.data(), 2);
    EXPECT_EQ(v, -1234);
}

TEST_F(RP20DeviceTest, ReadTemperatureScalesTenths) {
    addModuleDiag(0, 0x12);  // TC_4

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    auto& mapping = master_.pdo().mapping();
    uint8_t* tx = mapping.entryDataMut(dev.modules()[0].tx_pdo_entry);
    ASSERT_NE(tx, nullptr);
    tx[0] = 0xF4; tx[1] = 0x01;  // 500 → 50.0 °C
    EXPECT_FLOAT_EQ(dev.readTemperature(0, 0), 50.0f);
}

TEST_F(RP20DeviceTest, AccessorsRejectWrongModuleTypes) {
    addModuleDiag(0, 0x04);  // DI_16 — inputs only

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());

    // writes are no-ops on input-only modules; reads of wrong type return 0.
    dev.writeDigitalOutputs(0, 0, 0xFF);   // no rxpdo → early return
    dev.writeAnalogOutput(0, 0, 100);      // wrong type
    EXPECT_EQ(dev.readAnalogInput(0, 0), 0);
    EXPECT_FLOAT_EQ(dev.readTemperature(0, 0), 0.0f);
    EXPECT_EQ(dev.readDigitalInputs(0, 99), 0);  // out-of-range field
}

// ---------------------------------------------------------------------------
// assignPDOs / updateSyncManagerLengths
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, AssignPDOsWritesSmAssignRecords) {
    addModuleDiag(0, 0x0C);  // has both rx and tx
    addModuleDiag(2, 0x04);  // tx only
    addSmAssignRecords();

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.assignPDOs(0));

    // RxPDO list: slot 0 → 0x1600; TxPDO list: slots 0 and 2 → 0x1A00, 0x1A02.
    EXPECT_EQ(slave_->odEntry(0x1C12, 0)->data[0], 1);
    EXPECT_EQ(slave_->odEntry(0x1C13, 0)->data[0], 2);
    auto* rx1 = slave_->odEntry(0x1C12, 1);
    auto* tx1 = slave_->odEntry(0x1C13, 1);
    auto* tx2 = slave_->odEntry(0x1C13, 2);
    ASSERT_NE(rx1, nullptr);
    ASSERT_NE(tx1, nullptr);
    ASSERT_NE(tx2, nullptr);
    EXPECT_EQ(rx1->data[0] | (rx1->data[1] << 8), kRxPDOBase + 0);
    EXPECT_EQ(tx1->data[0] | (tx1->data[1] << 8), kTxPDOBase + 0);
    EXPECT_EQ(tx2->data[0] | (tx2->data[1] << 8), kTxPDOBase + 2);
}

TEST_F(RP20DeviceTest, UpdateSyncManagerLengthsMatchesPdoSizes) {
    addModuleDiag(0, 0x0C);
    addModuleDiag(1, 0x04);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    ASSERT_TRUE(dev.registerPDOs());
    dev.updateSyncManagerLengths(0);

    auto* cfgs = master_.pdo().slaveConfigs();
    ASSERT_NE(cfgs, nullptr);
    uint16_t total_rx = 0, total_tx = 0;
    for (const auto& mod : dev.modules()) {
        if (mod.descriptor->has_rxpdo) total_rx += mod.descriptor->rxpdo->size;
        if (mod.descriptor->has_txpdo) total_tx += mod.descriptor->txpdo->size;
    }
    EXPECT_EQ(cfgs[0].sm[2].length, total_rx);
    EXPECT_EQ(cfgs[0].rxpdo_size, total_rx);
    EXPECT_EQ(cfgs[0].sm[3].length, total_tx);
    EXPECT_EQ(cfgs[0].txpdo_size, total_tx);
}

TEST_F(RP20DeviceTest, UpdateSyncManagerLengthsAllRunsWithoutModules) {
    RP20Device dev(master_);
    dev.updateSyncManagerLengthsAll();  // no modules → still safe
    SUCCEED();
}

// ---------------------------------------------------------------------------
// bringToSafeOp / bringToOp
// ---------------------------------------------------------------------------

TEST_F(RP20DeviceTest, BringToSafeOpFailsWithNoModules) {
    RP20Device dev(master_);
    EXPECT_FALSE(dev.bringToSafeOp());
}

TEST_F(RP20DeviceTest, BringToSafeOpCompletesFullSequence) {
    addModuleDiag(0, 0x0C);
    addConfigRecord(0);
    addSmAssignRecords();

    // bringToSafeOp assumes the slave is already in PRE_OP — the emulator
    // enforces the real INIT→PRE_OP→SAFE_OP transition table.  The SDO
    // mailbox was configured in SetUp; mirror that on the Slave side.
    master_.slave(0).assumeMailboxAlreadyConfigured();
    ASSERT_EQ(master_.slave(0).transitionTo(SlaveState::PRE_OP), SlaveError::Ok);
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);

    RP20Device dev(master_);
    ASSERT_TRUE(dev.scanModules());
    EXPECT_TRUE(dev.bringToSafeOp());
    EXPECT_EQ(slave_->getState(), SlaveState::SAFE_OP);

    // OP requires an active PDO exchange — the driver's bringToOp
    // legitimately rejects when no cyclic data has moved.
    EXPECT_FALSE(dev.bringToOp());
}

// ---------------------------------------------------------------------------
// Static parsers / helpers (pure)
// ---------------------------------------------------------------------------

TEST(RP20ParsersTest, TCSignalFormParsesNamesAndAliases) {
    EXPECT_EQ(RP20Device::parseTCSignalForm("Type_K"),
              RP20::TCSignalForm::Type_K);
    EXPECT_EQ(RP20Device::parseTCSignalForm("K"), RP20::TCSignalForm::Type_K);
    EXPECT_EQ(RP20Device::parseTCSignalForm("S"), RP20::TCSignalForm::Type_S);
    EXPECT_EQ(RP20Device::parseTCSignalForm("100mV"),
              RP20::TCSignalForm::Voltage_100mV);
    // numeric fallback within range
    EXPECT_EQ(RP20Device::parseTCSignalForm("2"), RP20::TCSignalForm::Type_E);
    EXPECT_FALSE(RP20Device::parseTCSignalForm("bogus").has_value());
    EXPECT_FALSE(RP20Device::parseTCSignalForm("-1").has_value());
    EXPECT_FALSE(RP20Device::parseTCSignalForm("99").has_value());
    EXPECT_FALSE(RP20Device::parseTCSignalForm("").has_value());
}

TEST(RP20ParsersTest, AISignalFormParsesRanges) {
    EXPECT_EQ(RP20Device::parseAISignalForm("4-20mA"),
              RP20::AISignalForm::Current_4_20mA);
    EXPECT_EQ(RP20Device::parseAISignalForm("10V"),
              RP20::AISignalForm::Voltage_10V);
    EXPECT_EQ(RP20Device::parseAISignalForm("Voltage_1_5V"),
              RP20::AISignalForm::Voltage_1_5V);
    EXPECT_FALSE(RP20Device::parseAISignalForm("??").has_value());
}

TEST(RP20ParsersTest, RTDAndFilterAndCJCParse) {
    EXPECT_EQ(RP20Device::parseRTDSignalForm("PT100"),
              RP20::RTDSignalForm::PT100);
    EXPECT_EQ(RP20Device::parseRTDSignalForm("Cu100"),
              RP20::RTDSignalForm::Cu100);
    EXPECT_EQ(RP20Device::parseFilteringMode("average"),
              RP20::FilteringMode::Average);
    EXPECT_EQ(RP20Device::parseFilteringMode("None"), RP20::FilteringMode::None);
    EXPECT_EQ(RP20Device::parseColdJunctionCompensation("Internal"),
              RP20::ColdJunctionCompensation::Internal);
    EXPECT_EQ(RP20Device::parseColdJunctionCompensation("external"),
              RP20::ColdJunctionCompensation::External);
    EXPECT_FALSE(RP20Device::parseRTDSignalForm("nope").has_value());
    EXPECT_FALSE(RP20Device::parseFilteringMode("nope").has_value());
    EXPECT_FALSE(RP20Device::parseColdJunctionCompensation("nope").has_value());
}

TEST(RP20ModuleConfigTest, FormatLabelRendersSetAndUnsetFields) {
    ModuleConfig cfg;
    cfg.tc_signal_form = RP20::TCSignalForm::Type_K;
    EXPECT_NE(cfg.formatLabel(ModuleType::TC_4).find("Type_K"), std::string::npos);

    ModuleConfig empty;
    EXPECT_NE(empty.formatLabel(ModuleType::AI_4).find("default"),
              std::string::npos);
    EXPECT_EQ(empty.formatLabel(ModuleType::DI_16), "[no config]");
}

}  // namespace
