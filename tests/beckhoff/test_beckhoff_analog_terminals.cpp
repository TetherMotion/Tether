/**
 * @file test_beckhoff_analog_terminals.cpp
 * @brief Emulator-backed coverage for AnalogOutputTerminal (EL4xxx) and
 *        AnalogInputTerminal (EL30xx) — SII-driven channel-layout
 *        resolution, the SII-less fallback shapes, value/status accessors,
 *        identity rejection, factory scans, and the logical-exchange /
 *        OP bring-up paths.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <span>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/AnalogInputTerminal.hpp"
#include "tether/Beckhoff/AnalogOutputTerminal.hpp"
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

/// Emulated slave with mailbox pair plus optional process-data SMs.
SlaveEmulator* addAnalogSlave(EmulatedBus& bus, uint32_t product,
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
    addSm(0x1000, 128, 0x26);  // SM0 mailbox out
    addSm(0x1080, 128, 0x22);  // SM1 mailbox in
    addSm(out_addr, out_len, 0x64);
    addSm(in_addr, in_len, 0x40);
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

/// Analog-output discovery: SM2 outputs, one RxPDO per channel with a
/// single value entry.
DiscoveredSlave makeAnalogOut(uint32_t product, const char* name,
                              uint16_t out_addr, uint16_t out_len,
                              uint8_t n_channels, uint8_t value_bits,
                              uint16_t in_addr = 0, uint16_t in_len = 0) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = product;
    s.device_name = name;
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(out_addr, out_len, SII::SM_TYPE_PROCESS_OUT),
    };
    if (in_len > 0) {
        s.sync_managers->push_back(
            mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN));
    }
    std::vector<SII::SIIPDO> pdos;
    for (uint8_t ch = 0; ch < n_channels; ++ch) {
        pdos.push_back(mkPdo(static_cast<uint16_t>(0x1600 + ch), 2,
                             {mkEntry(static_cast<uint16_t>(0x3000 + ch), 1,
                                      value_bits)}));
    }
    s.rx_pdos = pdos;
    return s;
}

/// Analog-input discovery: SM3 inputs, one TxPDO per channel with a
/// 16-bit status prefix and a value entry.
DiscoveredSlave makeAnalogIn(uint32_t product, const char* name,
                             uint16_t in_addr, uint16_t in_len,
                             uint8_t n_channels, uint8_t value_bits,
                             bool with_status = true,
                             uint16_t out_addr = 0, uint16_t out_len = 0) {
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
    std::vector<SII::SIIPDO> pdos;
    for (uint8_t ch = 0; ch < n_channels; ++ch) {
        std::vector<SII::SIIPDOEntry> entries;
        if (with_status) {
            entries.push_back(
                mkEntry(static_cast<uint16_t>(0x6000 + ch), 1, 8));
            entries.push_back(
                mkEntry(static_cast<uint16_t>(0x6000 + ch), 2, 8));
        }
        entries.push_back(mkEntry(static_cast<uint16_t>(0x6010 + ch), 1,
                                  value_bits));
        pdos.push_back(mkPdo(static_cast<uint16_t>(0x1A00 + ch), 3,
                             std::move(entries)));
    }
    s.tx_pdos = pdos;
    return s;
}

class BeckhoffAnalogTerminalsTest : public ::testing::Test {
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
// AnalogOutputTerminal — EL4134 (4 x 16-bit voltage outputs)
// ============================================================================

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutConfigureResolvesSiiChannels) {
    auto* slave = addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.channelCount(), 4u);
    EXPECT_EQ(term.channelBits(0), 16u);
    EXPECT_EQ(term.channelBits(3), 16u);
    EXPECT_EQ(term.channelBits(4), 0u);  // out of range
    EXPECT_STREQ(term.deviceName(), "EL4134");
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);

    // configure() is idempotent.
    EXPECT_TRUE(term.configure().has_value());
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutSetValueRoundTrip) {
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    ASSERT_TRUE(term.configure().has_value());

    term.setValue(0, 16384);
    term.setValue(1, -16384);
    term.setValue(2, 0x7FFF);
    term.setValue(99, 1);  // out of range: ignored
    EXPECT_EQ(term.value(0), 16384);
    EXPECT_EQ(term.value(1), -16384);
    EXPECT_EQ(term.value(2), 0x7FFF);
    EXPECT_EQ(term.value(99), 0);

    // Value is truncated to 16 bits on the wire.
    term.setValue(3, 0x12345);
    EXPECT_EQ(term.value(3), 0x2345);

    auto img = term.rawOutput();
    ASSERT_GE(img.size(), 8u);
    int16_t v0 = 0;
    std::memcpy(&v0, img.data(), 2);
    EXPECT_EQ(v0, 16384);

    term.allZero();
    EXPECT_EQ(term.value(0), 0);
    EXPECT_EQ(term.value(1), 0);
    for (uint8_t b : term.rawOutput()) EXPECT_EQ(b, 0);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutOutputsReachSmBuffer) {
    auto* slave = addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    ASSERT_TRUE(term.configure().has_value());

    term.setValue(0, 0x1234);
    term.setValue(1, -2);
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(slave->smBufferByteForTest(2, 0), 0x34);
    EXPECT_EQ(slave->smBufferByteForTest(2, 1), 0x12);
    EXPECT_EQ(slave->smBufferByteForTest(2, 2), 0xFE);
    EXPECT_EQ(slave->smBufferByteForTest(2, 3), 0xFF);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOut32BitWidth) {
    addAnalogSlave(bus_, 0x0FEA3052, 0x1100, 16, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x0FEA3052, "EL4074", 0x1100, 16, 4, 32);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4074);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelBits(0), 32u);

    term.setValue(2, -100000);
    EXPECT_EQ(term.value(2), -100000);
    auto img = term.rawOutput();
    int32_t raw = 0;
    std::memcpy(&raw, img.data() + 8, 4);
    EXPECT_EQ(raw, -100000);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutFallbackWithoutSii) {
    // No process SM in the discovery data: the dominant Beckhoff shape is
    // assumed — num_bits channels of 2B on SM2 @ 0x1100.
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x10263052;
    ds.device_name = "EL4134";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 4u);  // EL4134 identity num_bits
    EXPECT_EQ(term.channelBits(0), 16u);
    term.setValue(1, 7);
    EXPECT_EQ(term.value(1), 7);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutNonByteAlignedLayoutRejected) {
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x10263052;
    ds.device_name = "EL4134";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0x1100, 8, SII::SM_TYPE_PROCESS_OUT),
    };
    // 1 status bit + a 16-bit value: total 17 bits is not byte-aligned.
    ds.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 1),
        mkEntry(0x7000, 2, 16)})};

    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutLayoutLargerThanSmRejected) {
    addAnalogSlave(bus_, 0x10263052, 0x1100, 2, 0, 0);
    bringUp();
    // 4 channels x 2B = 8B do not fit in a 2B SM2 region.
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 2, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutWrongIdentityRejected) {
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x0FA13052, "EL4001", 0x1100, 8, 1, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutSlaveIndexOutOfRange) {
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);
    // A second driver bound past the PDO manager's slave range fails early.
    Beckhoff::AnalogOutputTerminal far(master_, PDO::kMaxPDOSlaves,
                                       Dev::EL4134);
    auto r = far.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutFindFirstWithScan) {
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::AnalogOutputTerminal::findFirst(master_, Dev::EL4001,
                                                          scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::AnalogOutputTerminal::findFirst(master_, Dev::EL4134,
                                                         scan);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->slaveIndex(), 0u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutFindFirstRunsOwnDiscovery) {
    // The scan-less overload runs a shallow vendor/product discovery; the
    // emulated SII carries only the low 16 bits of the product code, so no
    // EL4xxx identity matches and the factory reports NoDeviceFound.
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();

    auto miss = Beckhoff::AnalogOutputTerminal::findFirst(master_, Dev::EL4134);
    EXPECT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutLogicalExchangeAndOp) {
    addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);

    ASSERT_TRUE(term.prepareForLogicalExchange().has_value());
    // The FMMU needs the shared logical map — build it as the Multi*
    // chain drivers do, then program the FMMU and enter SAFE-OP.
    auto& pdo = master_.pdoForSlave(0);
    auto& lam = master_.logicalAddressManagerForSlave(0);
    ASSERT_TRUE(lam.buildAddressMap(pdo.slaveConfigs(), 1));

    ASSERT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_GT(term.logicalAddress(), 0u);

    // Already configured -> no-op.
    EXPECT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.requestOp(2000).has_value());
    EXPECT_TRUE(term.operational());
    term.stop();
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutStartWithoutManagedLoop) {
    auto* slave = addAnalogSlave(bus_, 0x10263052, 0x1100, 8, 0, 0);
    bringUp();
    auto ds = makeAnalogOut(0x10263052, "EL4134", 0x1100, 8, 4, 16);
    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4134);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
    EXPECT_EQ(term.slaveIndex(), 0u);
    EXPECT_EQ(term.alState(), SlaveState::OP);
    term.setValue(0, 1);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogOutWithSm3InputsMapsBothRegions) {
    auto* slave = addAnalogSlave(bus_, 0x11163052, 0x1100, 4, 0x1180, 4);
    bringUp();
    auto ds = makeAnalogOut(0x11163052, "EL4374", 0x1100, 4, 2, 16,
                            0x1180, 4);
    ds.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 16), mkEntry(0x6010, 1, 16)})};

    Beckhoff::AnalogOutputTerminal term(master_, ds, Dev::EL4374);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 2u);
    EXPECT_EQ(term.rawInput().size(), 4u);

    const uint8_t img[4] = {0x34, 0x12, 0x78, 0x56};
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    auto in = term.rawInput();
    ASSERT_EQ(in.size(), 4u);
    EXPECT_EQ(in[0], 0x34);
    EXPECT_EQ(in[3], 0x56);
}

// ============================================================================
// AnalogInputTerminal — EL3062/EL3xxx style inputs
// ============================================================================

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInConfigureResolvesSiiChannels) {
    auto* slave = addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 12, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.channelCount(), 2u);
    EXPECT_EQ(term.channelBits(0), 16u);
    EXPECT_EQ(term.channelBits(5), 0u);
    EXPECT_TRUE(term.hasStatus(0));
    EXPECT_TRUE(term.hasStatus(1));
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInValuesAndStatusReadFromSmBuffer) {
    auto* slave = addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 12, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    ASSERT_TRUE(term.configure().has_value());

    // Channel 0: status 0x0001, value 0x1234. Channel 1: status 0x0080,
    // value -2.  Trailing 4 bytes are an unclaimed third slot.
    const uint8_t img[12] = {
        0x01, 0x00, 0x34, 0x12,
        0x80, 0x00, 0xFE, 0xFF,
        0xAA, 0xBB, 0xCC, 0xDD,
    };
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.value(0), 0x1234);
    EXPECT_EQ(term.status(0), 0x0001u);
    EXPECT_EQ(term.value(1), -2);
    EXPECT_EQ(term.status(1), 0x0080u);

    // Out-of-range accessors return 0 / false.
    EXPECT_EQ(term.value(9), 0);
    EXPECT_EQ(term.status(9), 0u);
    EXPECT_FALSE(term.hasStatus(9));
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogIn32BitValue) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3751", 0x1180, 12, 2, 32);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelBits(0), 32u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInWithoutStatusPrefix) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 8);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 8, 2, 16,
                           /*with_status=*/false);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 2u);
    EXPECT_FALSE(term.hasStatus(0));
    EXPECT_EQ(term.status(0), 0u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInFallbackWithoutSii) {
    // No process SM: assume num_bits channels of [16-bit status][16-bit
    // value] on SM3 @ 0x1180.
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 8);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x0BF63052;
    ds.device_name = "EL3062";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 2u);
    EXPECT_TRUE(term.hasStatus(0));
    EXPECT_EQ(term.channelBits(0), 16u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInNonByteAlignedLayoutRejected) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 8);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x0BF63052;
    ds.device_name = "EL3062";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0x1180, 8, SII::SM_TYPE_PROCESS_IN),
    };
    ds.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 3),
        mkEntry(0x6000, 2, 16)})};

    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInLayoutLargerThanSmRejected) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 4);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 4, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInWrongIdentityRejected) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 8);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 8, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3068);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInSlaveIndexOutOfRange) {
    bringUp();
    Beckhoff::AnalogInputTerminal far(master_, PDO::kMaxPDOSlaves,
                                      Dev::EL3062);
    auto r = far.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInFindFirstWithScan) {
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 12, 2, 16);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::AnalogInputTerminal::findFirst(master_, Dev::EL3068,
                                                         scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::AnalogInputTerminal::findFirst(master_, Dev::EL3062,
                                                        scan);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->slaveIndex(), 0u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInFindFirstRunsOwnDiscovery) {
    // Same 16-bit-product limitation as the output factory: the scan-less
    // overload still runs a live discovery and then reports NoDeviceFound.
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();

    auto miss = Beckhoff::AnalogInputTerminal::findFirst(master_, Dev::EL3062);
    EXPECT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInLogicalExchangeAndOp) {
    addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 12, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);

    ASSERT_TRUE(term.prepareForLogicalExchange().has_value());
    auto& pdo = master_.pdoForSlave(0);
    auto& lam = master_.logicalAddressManagerForSlave(0);
    ASSERT_TRUE(lam.buildAddressMap(pdo.slaveConfigs(), 1));

    ASSERT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_GT(term.logicalAddress(), 0u);
    EXPECT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.requestOp(2000).has_value());
    EXPECT_TRUE(term.operational());
    term.stop();
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInStartWithoutManagedLoop) {
    auto* slave = addAnalogSlave(bus_, 0x0BF63052, 0, 0, 0x1180, 12);
    bringUp();
    auto ds = makeAnalogIn(0x0BF63052, "EL3062", 0x1180, 12, 2, 16);
    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL3062);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
    EXPECT_EQ(term.lastAlStatusCode(), 0u);
}

TEST_F(BeckhoffAnalogTerminalsTest, AnalogInWithSm2OutputsMapsBothRegions) {
    auto* slave = addAnalogSlave(bus_, 0x11163052, 0x1100, 4, 0x1180, 4);
    bringUp();
    auto ds = makeAnalogIn(0x11163052, "EL4374", 0x1180, 4, 1, 16,
                           /*with_status=*/false, 0x1100, 4);
    ds.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 16)})};

    Beckhoff::AnalogInputTerminal term(master_, ds, Dev::EL4374);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 1u);
    EXPECT_EQ(term.rawOutput().size(), 4u);
    (void)slave;
}
