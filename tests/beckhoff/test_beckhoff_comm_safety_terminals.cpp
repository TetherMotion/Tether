/**
 * @file test_beckhoff_comm_safety_terminals.cpp
 * @brief Emulator-backed coverage for CommTerminal (EL6xxx byte-FIFO
 *        communication terminals) and SafetyTerminal (EL19xx/EL29xx
 *        TwinSAFE terminals) — SII FIFO channel resolution, ctrl/status
 *        words, FIFO data access, FSoE connection bring-up and the
 *        SII-less fallback shapes.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <span>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/CommTerminal.hpp"
#include "tether/Beckhoff/SafetyTerminal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace Dev = EtherCAT::Beckhoff::Devices;
using EtherCAT::Beckhoff::Error;

namespace {

SII::SIISyncManager mkSm(uint16_t addr, uint16_t len, uint8_t type) {
    SII::SIISyncManager sm{};
    sm.phys_start_address = addr;
    sm.length = len;
    sm.sm_type = type;
    sm.enable = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(
        static_cast<uint8_t>(0x01));
    return sm;
}

SII::SIIPDOEntry mkEntry(uint16_t index, uint8_t sub, uint8_t bits) {
    SII::SIIPDOEntry e{};
    e.index = index;
    e.subindex = sub;
    e.bit_length = bits;
    return e;
}

SII::SIIPDO mkPdo(uint16_t index, uint8_t sm,
                  std::vector<SII::SIIPDOEntry> entries) {
    SII::SIIPDO p{};
    p.pdo_index = index;
    p.sync_manager = sm;
    p.entries.assign(entries.begin(), entries.end());
    p.n_entries = static_cast<uint8_t>(p.entries.size());
    return p;
}

SlaveEmulator* addTerminalSlave(EmulatedBus& bus, uint32_t product,
                                uint16_t out_addr, uint16_t out_len,
                                uint16_t in_addr, uint16_t in_len) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = static_cast<uint16_t>(product & 0xFFFF);
    cfg.mailbox_protocol = 0x04;  // CoE
    auto addSm = [&cfg](uint16_t addr, uint16_t len, uint8_t ctrl) {
        EtherCAT::Emulator::SIIConfig::SMConfig sm{};
        sm.start_addr = addr;
        sm.length     = len;
        sm.control    = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrl);
        sm.enable     = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(
            static_cast<uint8_t>(0x01));
        cfg.sync_managers.push_back(sm);
    };
    addSm(0x1000, 128, 0x26);
    addSm(0x1080, 128, 0x22);
    addSm(out_addr, out_len, 0x64);
    addSm(in_addr, in_len, 0x40);
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

/// EL6001-shaped discovery: one FIFO channel pair, 16-bit ctrl/status
/// followed by 4 data bytes.
DiscoveredSlave makeComm(uint32_t product, const char* name,
                         uint16_t out_addr, uint16_t out_len,
                         uint16_t in_addr, uint16_t in_len,
                         uint8_t n_channels = 1, uint8_t data_bytes = 4) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = product;
    s.device_name = name;
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(out_addr, out_len, SII::SM_TYPE_PROCESS_OUT),
        mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN),
    };
    std::vector<SII::SIIPDO> rx;
    std::vector<SII::SIIPDO> tx;
    for (uint8_t ch = 0; ch < n_channels; ++ch) {
        std::vector<SII::SIIPDOEntry> oe{mkEntry(0x7000, 1, 16)};
        std::vector<SII::SIIPDOEntry> ie{mkEntry(0x6000, 1, 16)};
        for (uint8_t b = 0; b < data_bytes; ++b) {
            oe.push_back(mkEntry(0x7000, static_cast<uint8_t>(2 + b), 8));
            ie.push_back(mkEntry(0x6000, static_cast<uint8_t>(2 + b), 8));
        }
        rx.push_back(mkPdo(static_cast<uint16_t>(0x1600 + ch), 2,
                           std::move(oe)));
        tx.push_back(mkPdo(static_cast<uint16_t>(0x1A00 + ch), 3,
                           std::move(ie)));
    }
    s.rx_pdos = rx;
    s.tx_pdos = tx;
    return s;
}

/// EL1904-shaped discovery: FSoE frames ride the whole SM2/SM3 images.
DiscoveredSlave makeSafety(uint32_t product, const char* name,
                           uint16_t out_addr, uint16_t out_len,
                           uint16_t in_addr, uint16_t in_len) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = product;
    s.device_name = name;
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(out_addr, out_len, SII::SM_TYPE_PROCESS_OUT),
        mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN),
    };
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 8), mkEntry(0x7000, 2, 8),
        mkEntry(0x7000, 3, 8), mkEntry(0x7000, 4, 8),
        mkEntry(0x7000, 5, 8), mkEntry(0x7000, 6, 8)})};
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 8), mkEntry(0x6000, 2, 8),
        mkEntry(0x6000, 3, 8), mkEntry(0x6000, 4, 8),
        mkEntry(0x6000, 5, 8), mkEntry(0x6000, 6, 8)})};
    return s;
}

class BeckhoffCommSafetyTerminalsTest : public ::testing::Test {
protected:
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    void bringUp() {
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

// ============================================================================
// CommTerminal — EL6001 (single RS232 FIFO channel)
// ============================================================================

TEST_F(BeckhoffCommSafetyTerminalsTest, CommConfigureResolvesFifoChannel) {
    auto* slave = addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.channels(), 1u);
    EXPECT_EQ(term.txCapacity(0), 4u);
    EXPECT_EQ(term.rxCapacity(0), 4u);
    EXPECT_EQ(term.txCapacity(7), 0u);
    EXPECT_EQ(term.rxCapacity(7), 0u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);

    // Idempotent.
    EXPECT_TRUE(term.configure().has_value());
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommCtrlWordAndBits) {
    addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);
    ASSERT_TRUE(term.configure().has_value());

    term.setCtrlWord(0, 0x1234);
    EXPECT_EQ(term.ctrlWord(0), 0x1234u);

    term.setCtrlBit(0, 3, true);
    EXPECT_TRUE(term.ctrlBit(0, 3));
    EXPECT_EQ(term.ctrlWord(0), 0x1234u | 0x0008u);
    term.setCtrlBit(0, 3, false);
    EXPECT_FALSE(term.ctrlBit(0, 3));

    // Out-of-range channel / bit are no-ops.
    term.setCtrlWord(5, 0xFFFF);
    EXPECT_EQ(term.ctrlWord(5), 0u);
    term.setCtrlBit(0, 40, true);
    EXPECT_FALSE(term.ctrlBit(0, 40));
    EXPECT_EQ(term.ctrlWord(0), 0x1234u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommWriteAndReadFifoData) {
    auto* slave = addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);
    ASSERT_TRUE(term.configure().has_value());

    const uint8_t out[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    EXPECT_EQ(term.writeData(0, out), 4u);
    // Oversized write is capped at the channel capacity.
    const uint8_t big[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT_EQ(term.writeData(0, big), 4u);
    EXPECT_EQ(term.writeData(9, big), 0u);

    term.setCtrlWord(0, 0x0001);
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(2, 0), 0x01);
    EXPECT_EQ(slave->smBufferByteForTest(2, 1), 0x00);
    EXPECT_EQ(slave->smBufferByteForTest(2, 2), 0x01);
    EXPECT_EQ(slave->smBufferByteForTest(2, 5), 0x04);

    // Receive FIFO: status word + 4 data bytes from the emulated SM3.
    const uint8_t in[6] = {0x42, 0x00, 0xCA, 0xFE, 0xBA, 0xBE};
    slave->setSmBufferForTest(3, 0, in, sizeof(in));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.statusWord(0), 0x0042u);
    EXPECT_TRUE(term.statusBit(0, 1));
    EXPECT_FALSE(term.statusBit(0, 2));
    EXPECT_FALSE(term.statusBit(0, 40));
    EXPECT_EQ(term.statusWord(4), 0u);

    auto view = term.dataIn(0);
    ASSERT_EQ(view.size(), 4u);
    EXPECT_EQ(view[0], 0xCA);
    EXPECT_EQ(view[3], 0xBE);
    EXPECT_TRUE(term.dataIn(4).empty());

    uint8_t dst[8] = {};
    EXPECT_EQ(term.readData(0, dst), 4u);
    EXPECT_EQ(dst[0], 0xCA);
    EXPECT_EQ(dst[3], 0xBE);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommTwoChannelsResolved) {
    addTerminalSlave(bus_, 0x17723052, 0x1100, 12, 0x1180, 12);
    bringUp();
    auto ds = makeComm(0x17723052, "EL6002", 0x1100, 12, 0x1180, 12,
                       /*n_channels=*/2);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6002);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channels(), 2u);

    // Channel 1 starts after the 6-byte channel 0 image.
    term.setCtrlWord(1, 0x00FF);
    EXPECT_EQ(term.ctrlWord(1), 0x00FFu);
    EXPECT_EQ(term.ctrlWord(0), 0u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommFallbackWithoutSii) {
    addTerminalSlave(bus_, 0x17713052, 0x1100, 24, 0x1180, 24);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x17713052;
    ds.device_name = "EL6001";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channels(), 1u);
    EXPECT_EQ(term.txCapacity(0), 22u);
    EXPECT_EQ(term.rxCapacity(0), 22u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommRejectsWrongIdentity) {
    addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6021);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommSlaveIndexOutOfRange) {
    bringUp();
    Beckhoff::CommTerminal far(master_, PDO::kMaxPDOSlaves, Dev::EL6001);
    auto r = far.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommFindFirstWithScan) {
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::CommTerminal::findFirst(master_, Dev::EL6021, scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::CommTerminal::findFirst(master_, Dev::EL6001, scan);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->slaveIndex(), 0u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommFindFirstRunsOwnDiscovery) {
    addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto miss = Beckhoff::CommTerminal::findFirst(master_, Dev::EL6001);
    EXPECT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommLogicalExchangeAndStart) {
    addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);

    ASSERT_TRUE(term.prepareForLogicalExchange().has_value());
    auto& pdo = master_.pdoForSlave(0);
    auto& lam = master_.logicalAddressManagerForSlave(0);
    ASSERT_TRUE(lam.buildAddressMap(pdo.slaveConfigs(), 1));
    ASSERT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_GT(term.logicalAddress(), 0u);
    EXPECT_TRUE(term.requestOp(2000).has_value());
    EXPECT_TRUE(term.operational());
    term.stop();
}

TEST_F(BeckhoffCommSafetyTerminalsTest, CommStartWithoutManagedLoop) {
    auto* slave = addTerminalSlave(bus_, 0x17713052, 0x1100, 6, 0x1180, 6);
    bringUp();
    auto ds = makeComm(0x17713052, "EL6001", 0x1100, 6, 0x1180, 6);
    Beckhoff::CommTerminal term(master_, ds, Dev::EL6001);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
}

// ============================================================================
// SafetyTerminal — EL1904 (FSoE black channel over SM2/SM3)
// ============================================================================

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyConfigureCreatesFsoeConnection) {
    auto* slave = addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    ASSERT_TRUE(term.configure(cfg).has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);

    // A freshly initialised FSoE connection is not operational and not
    // in fail-safe state yet; state/error code are readable.
    EXPECT_FALSE(term.isOperational());
    EXPECT_FALSE(term.isFailSafe());
    EXPECT_EQ(term.fsoeErrorCode(), 0u);
    (void)term.fsoeState();
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyExchangeDrivesPdoBuffers) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    cfg.safe_input_bytes = 1;
    cfg.safe_output_bytes = 1;
    ASSERT_TRUE(term.configure(cfg).has_value());

    // First exchanges start the connection handshake; the FSoE frame is
    // written into the SM2 image (cmd byte + data + CRC + conn id).
    uint64_t now = 0;
    for (int i = 0; i < 4; ++i) {
        now += 10;
        (void)term.exchange(now);
    }
    auto out = term.rawOutput();
    ASSERT_EQ(out.size(), 6u);
    EXPECT_NE(out[0], 0u);  // FSoE command byte is populated

    term.resetConnection();
    EXPECT_FALSE(term.isOperational());
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetySafeIoAccessors) {
    addTerminalSlave(bus_, 0x0B583052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x0B583052, "EL2904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL2904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x0100;
    cfg.safe_input_bytes = 2;
    cfg.safe_output_bytes = 2;
    ASSERT_TRUE(term.configure(cfg).has_value());

    EXPECT_TRUE(term.setSafeOutputBit(0, true));
    EXPECT_TRUE(term.setSafeOutputByte(1, 0xA5));
    // Reading back before the connection is operational yields the
    // connection's current safe state (all zero).
    EXPECT_FALSE(term.safeInputBit(0));
    EXPECT_EQ(term.safeInputByte(0), 0u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyAccessorsWithoutConnection) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    // No configure() yet: every accessor degrades safely.
    EXPECT_FALSE(term.exchange(0));
    EXPECT_FALSE(term.safeInputBit(0));
    EXPECT_FALSE(term.setSafeOutputBit(0, true));
    EXPECT_EQ(term.safeInputByte(0), 0u);
    EXPECT_FALSE(term.setSafeOutputByte(0, 1));
    EXPECT_FALSE(term.isOperational());
    EXPECT_FALSE(term.isFailSafe());
    EXPECT_EQ(term.fsoeState(), 0u);
    EXPECT_EQ(term.fsoeErrorCode(), 0u);
    term.resetConnection();
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyFallbackWithoutSii) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x07703052;
    ds.device_name = "EL1904";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);
    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x0042;
    ASSERT_TRUE(term.configure(cfg).has_value());
    EXPECT_EQ(term.rawOutput().size(), 6u);
    EXPECT_EQ(term.rawInput().size(), 6u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyRejectsShortImages) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 4, 0x1D00, 4);
    bringUp();
    // A 4-byte FSoE image is too short (minimum is cmd + CRC + conn id).
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 4, 0x1D00, 4);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);
    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    auto r = term.configure(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyRejectsWrongIdentity) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL2904);
    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    auto r = term.configure(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetySlaveIndexOutOfRange) {
    bringUp();
    Beckhoff::SafetyTerminal far(master_, PDO::kMaxPDOSlaves, Dev::EL1904);
    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    auto r = far.configure(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyFindFirstWithScan) {
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::SafetyTerminal::findFirst(master_, Dev::EL2904, scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::SafetyTerminal::findFirst(master_, Dev::EL1904, scan);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->slaveIndex(), 0u);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyFindFirstRunsOwnDiscovery) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto miss = Beckhoff::SafetyTerminal::findFirst(master_, Dev::EL1904);
    EXPECT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyLogicalExchangeAndStart) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    ASSERT_TRUE(term.configure(cfg).has_value());

    // configure() already reached SAFE-OP; the logical contract is a no-op
    // afterwards and requestOp() takes it to OP.
    EXPECT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.requestOp(2000).has_value());
    EXPECT_TRUE(term.operational());
    term.stop();
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyStartWithoutManagedLoop) {
    auto* slave = addTerminalSlave(bus_, 0x07703052, 0x1200, 6, 0x1D00, 6);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 6, 0x1D00, 6);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(cfg, opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
}

TEST_F(BeckhoffCommSafetyTerminalsTest, SafetyExplicitImageSizes) {
    addTerminalSlave(bus_, 0x07703052, 0x1200, 12, 0x1D00, 12);
    bringUp();
    auto ds = makeSafety(0x07703052, "EL1904", 0x1200, 12, 0x1D00, 12);
    Beckhoff::SafetyTerminal term(master_, ds, Dev::EL1904);

    Beckhoff::SafetyTerminal::Config cfg;
    cfg.safety_addr = 0x1234;
    cfg.safe_input_bytes = 6;
    cfg.safe_output_bytes = 4;
    ASSERT_TRUE(term.configure(cfg).has_value());
    EXPECT_EQ(term.rawInput().size(), 12u);
}
