/**
 * @file test_beckhoff_extra_terminals.cpp
 * @brief Emulator-backed coverage for OversamplingTerminal (EL3773) and
 *        PositionInputTerminal (EL5101) — bring-up, layout resolution,
 *        sample/value/status reads, SII-less fallback shapes, and
 *        identity rejection.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/OversamplingTerminal.hpp"
#include "tether/Beckhoff/PositionInputTerminal.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace Dev = EtherCAT::Beckhoff::Devices;

namespace {

/// CoE slave with mailbox pair + one process-input SM on channel 3
/// (standard Beckhoff measurement-module layout).
SlaveEmulator* addMeasurementSlave(EmulatedBus& bus, uint32_t vendor,
                                   uint32_t product, uint16_t in_addr,
                                   uint16_t in_len) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = vendor;
    cfg.product_code = product;
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
    addSm(0,       0,   0x00); // SM2 unused
    addSm(in_addr, in_len, 0x40);  // SM3 process inputs
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

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

/// EL3773-shaped discovery: 3 channels, each a 16-bit status prefix
/// (two 8-bit entries) followed by 4 x 16-bit samples.
DiscoveredSlave makeOversamplingDiscovered(uint16_t in_addr, uint16_t in_len,
                                           uint8_t n_channels,
                                           uint8_t n_samples) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = 0x0EBD3052;  // EL3773
    s.device_name = "EL3773";
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0,       0,   SII::SM_TYPE_UNUSED),
        mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN),
    };
    std::vector<SII::SIIPDO> pdos;
    for (uint8_t ch = 0; ch < n_channels; ++ch) {
        SII::SIIPDO pdo{};
        pdo.pdo_index = static_cast<uint16_t>(0x1A00 + ch);
        pdo.sync_manager = 3;
        pdo.entries.push_back(mkEntry(static_cast<uint16_t>(0x6000 + ch), 1, 8));
        pdo.entries.push_back(mkEntry(static_cast<uint16_t>(0x6000 + ch), 2, 8));
        for (uint8_t i = 0; i < n_samples; ++i)
            pdo.entries.push_back(
                mkEntry(static_cast<uint16_t>(0x6010 + ch), i + 1, 16));
        pdo.n_entries = static_cast<uint8_t>(pdo.entries.size());
        pdos.push_back(pdo);
    }
    s.tx_pdos = pdos;
    return s;
}

/// EL5101-shaped discovery: one channel, 32-bit status prefix
/// (four 8-bit entries) + one 32-bit counter.
DiscoveredSlave makePositionDiscovered(uint16_t in_addr, uint16_t in_len) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = 0x13ED3052;  // EL5101
    s.device_name = "EL5101";
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0,       0,   SII::SM_TYPE_UNUSED),
        mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN),
    };
    SII::SIIPDO pdo{};
    pdo.pdo_index = 0x1A00;
    pdo.sync_manager = 3;
    for (uint8_t i = 1; i <= 4; ++i)
        pdo.entries.push_back(mkEntry(0x6000, i, 8));
    pdo.entries.push_back(mkEntry(0x6000, 0x11, 32));
    pdo.n_entries = static_cast<uint8_t>(pdo.entries.size());
    s.tx_pdos = std::vector<SII::SIIPDO>{pdo};
    return s;
}

class BeckhoffExtraTerminalsTest : public ::testing::Test {
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
// OversamplingTerminal — EL3773
// ============================================================================

TEST_F(BeckhoffExtraTerminalsTest, OversamplingConfigureReachesSafeOp) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052,
                                      0x1180, 30);
    bringUp();
    auto ds = makeOversamplingDiscovered(0x1180, 30, 3, 4);
    Beckhoff::OversamplingTerminal term(master_, ds, Dev::EL3773);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.channels(), 3u);
    EXPECT_EQ(term.samplesPerCycle(0), 4u);
    EXPECT_EQ(term.sampleBits(0), 16u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
}

TEST_F(BeckhoffExtraTerminalsTest, OversamplingSamplesReadFromSmBuffer) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052,
                                      0x1180, 30);
    bringUp();
    auto ds = makeOversamplingDiscovered(0x1180, 30, 3, 4);
    Beckhoff::OversamplingTerminal term(master_, ds, Dev::EL3773);
    ASSERT_TRUE(term.configure().has_value());

    // Channel 0: status 0x0000 then samples 0x1234, -1, 0x7FFF, -2.
    const uint8_t img[10] = {
        0x00, 0x00,                          // status prefix (16 bit)
        0x34, 0x12,                          //  0x1234
        0xFF, 0xFF,                          //  -1
        0xFF, 0x7F,                          //  0x7FFF
        0xFE, 0xFF,                          //  -2
    };
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.sample(0, 0), 0x1234);
    EXPECT_EQ(term.sample(0, 1), -1);
    EXPECT_EQ(term.sample(0, 2), 0x7FFF);
    EXPECT_EQ(term.sample(0, 3), -2);

    std::array<int32_t, 4> dst{};
    EXPECT_EQ(term.samples(0, dst), 4u);
    EXPECT_EQ(dst[0], 0x1234);
    EXPECT_EQ(dst[3], -2);

    EXPECT_TRUE(term.hasStatus(0));
    EXPECT_EQ(term.statusWord(0), 0x0000u);
    // Untouched channel 1 reads zero.
    EXPECT_EQ(term.sample(1, 0), 0);
}

TEST_F(BeckhoffExtraTerminalsTest, OversamplingStatusWordIncludesPrefix) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052,
                                      0x1180, 30);
    bringUp();
    auto ds = makeOversamplingDiscovered(0x1180, 30, 3, 4);
    Beckhoff::OversamplingTerminal term(master_, ds, Dev::EL3773);
    ASSERT_TRUE(term.configure().has_value());

    const uint8_t img[4] = {0x21, 0x43, 0x00, 0x00};  // status = 0x4321
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(term.statusWord(0), 0x4321u);
}

TEST_F(BeckhoffExtraTerminalsTest, OversamplingFallbackWithoutSii) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052,
                                      0x1180, 12);
    bringUp();
    // No process-input SM in the discovery data: fallback = num_bits (3)
    // channels of [16-bit status][16-bit sample] = 12 bytes.
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x0EBD3052;
    ds.device_name = "EL3773";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0,       0,   SII::SM_TYPE_UNUSED),
    };

    Beckhoff::OversamplingTerminal term(master_, ds, Dev::EL3773);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channels(), 3u);  // EL3773 identity num_bits
    EXPECT_EQ(term.samplesPerCycle(0), 1u);
    EXPECT_EQ(term.sampleBits(0), 16u);

    // Channel 0 layout: [status 16 bit][sample 16 bit].
    const uint8_t img[4] = {0x00, 0x00, 0x34, 0x12};
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(term.sample(0, 0), 0x1234);
    EXPECT_TRUE(term.hasStatus(0));
}

TEST_F(BeckhoffExtraTerminalsTest, OversamplingWrongIdentityRejected) {
    addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052, 0x1180, 30);
    bringUp();
    auto ds = makeOversamplingDiscovered(0x1180, 30, 3, 4);
    ds.product_code = 0x0EC73052;  // claim EL3783 on an EL3773
    Beckhoff::OversamplingTerminal term(master_, ds, Dev::EL3773);
    EXPECT_FALSE(term.configure().has_value());
}

TEST_F(BeckhoffExtraTerminalsTest, OversamplingMatchesAndFindFirst) {
    addMeasurementSlave(bus_, 0x00000002, 0x0EBD3052, 0x1180, 30);
    bringUp();
    auto ds = makeOversamplingDiscovered(0x1180, 30, 3, 4);
    EXPECT_TRUE(Beckhoff::OversamplingTerminal::matches(ds, Dev::EL3773));
    EXPECT_FALSE(Beckhoff::OversamplingTerminal::matches(ds, Dev::EL3783));

    std::vector<DiscoveredSlave> scan{ds};
    auto found = Beckhoff::OversamplingTerminal::findFirst(
        master_, Dev::EL3773, scan);
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->slaveIndex(), 0u);

    auto missing = Beckhoff::OversamplingTerminal::findFirst(
        master_, Dev::EL3783, scan);
    EXPECT_FALSE(missing.has_value());
}

// ============================================================================
// PositionInputTerminal — EL5101
// ============================================================================

TEST_F(BeckhoffExtraTerminalsTest, PositionInputConfigureReachesSafeOp) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x13ED3052,
                                      0x1180, 8);
    bringUp();
    auto ds = makePositionDiscovered(0x1180, 8);
    Beckhoff::PositionInputTerminal term(master_, ds, Dev::EL5101);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.channelCount(), 1u);
    EXPECT_EQ(term.valueCount(0), 1u);
    EXPECT_EQ(term.channelBits(0), 32u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
}

TEST_F(BeckhoffExtraTerminalsTest, PositionInputValueAndStatusReads) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x13ED3052,
                                      0x1180, 8);
    bringUp();
    auto ds = makePositionDiscovered(0x1180, 8);
    Beckhoff::PositionInputTerminal term(master_, ds, Dev::EL5101);
    ASSERT_TRUE(term.configure().has_value());

    // status = 0x0004_0003 (LE), counter = -2.
    const uint8_t img[8] = {0x03, 0x00, 0x04, 0x00,
                            0xFE, 0xFF, 0xFF, 0xFF};
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.value(0, 0), -2);
    EXPECT_EQ(term.rawValue(0, 0), 0xFFFFFFFEu);
    EXPECT_TRUE(term.hasStatus(0));
    EXPECT_EQ(term.status(0), 0x0003u);

    // Interface surface.
    EXPECT_EQ(term.slaveIndex(), 0u);
    EXPECT_EQ(term.channelCount(), 1u);
    EXPECT_STREQ(term.deviceName(), "EL5101");
    auto [idx, sub] = term.valueObject(0, 0);
    EXPECT_EQ(idx, 0x6000u);
    EXPECT_EQ(sub, 0x11u);
}

TEST_F(BeckhoffExtraTerminalsTest, PositionInputPositiveCounter) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x13ED3052,
                                      0x1180, 8);
    bringUp();
    auto ds = makePositionDiscovered(0x1180, 8);
    Beckhoff::PositionInputTerminal term(master_, ds, Dev::EL5101);
    ASSERT_TRUE(term.configure().has_value());

    const uint8_t img[8] = {0x00, 0x00, 0x00, 0x00,
                            0x78, 0x56, 0x34, 0x12};  // 0x12345678
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(term.value(0, 0), 0x12345678);
}

TEST_F(BeckhoffExtraTerminalsTest, PositionInputOutOfRangeGuards) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x13ED3052,
                                      0x1180, 8);
    bringUp();
    auto ds = makePositionDiscovered(0x1180, 8);
    Beckhoff::PositionInputTerminal term(master_, ds, Dev::EL5101);
    ASSERT_TRUE(term.configure().has_value());

    EXPECT_EQ(term.value(9, 0), 0);
    EXPECT_EQ(term.value(0, 9), 0);
    EXPECT_EQ(term.rawValue(9, 0), 0u);
    EXPECT_EQ(term.status(9), 0u);
    EXPECT_FALSE(term.hasStatus(9));
    EXPECT_EQ(term.channelBits(9), 0u);
    EXPECT_EQ(term.valueCount(9), 0u);
}

TEST_F(BeckhoffExtraTerminalsTest, PositionInputFallbackWithoutSii) {
    auto* slave = addMeasurementSlave(bus_, 0x00000002, 0x13ED3052,
                                      0x1180, 8);
    bringUp();
    // No process-input SM in the discovery data → fallback: num_bits (1)
    // channel of [32-bit status][32-bit counter].
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x13ED3052;
    ds.device_name = "EL5101";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0,       0,   SII::SM_TYPE_UNUSED),
    };

    Beckhoff::PositionInputTerminal term(master_, ds, Dev::EL5101);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.channelCount(), 1u);
    EXPECT_EQ(term.channelBits(0), 32u);

    const uint8_t img[8] = {0x00, 0x00, 0x00, 0x00,
                            0x2A, 0x00, 0x00, 0x00};
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(term.value(0, 0), 42);
}

TEST_F(BeckhoffExtraTerminalsTest, PositionInputFindFirst) {
    addMeasurementSlave(bus_, 0x00000002, 0x13ED3052, 0x1180, 8);
    bringUp();
    auto ds = makePositionDiscovered(0x1180, 8);
    std::vector<DiscoveredSlave> scan{ds};

    auto found = Beckhoff::PositionInputTerminal::findFirst(
        master_, Dev::EL5101, scan);
    ASSERT_TRUE(found.has_value());

    auto missing = Beckhoff::PositionInputTerminal::findFirst(
        master_, Dev::EL5151, scan);
    EXPECT_FALSE(missing.has_value());
}
