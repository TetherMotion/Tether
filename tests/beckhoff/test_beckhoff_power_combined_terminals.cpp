/**
 * @file test_beckhoff_power_combined_terminals.cpp
 * @brief Emulator-backed coverage for CombinedIoTerminal (EL2xxx packed
 *        bit I/O) and PowerMeterTerminal (EL34xx power measurement) —
 *        SII layout resolution, packed-bit access, raw field access,
 *        phase/selector measurement reads and the SII-less fallbacks.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/CombinedIoTerminal.hpp"
#include "tether/Beckhoff/PowerMeterTerminal.hpp"
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

DiscoveredSlave makeBase(uint32_t product, const char* name,
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
    return s;
}

/// EL2034-shaped packed-bit discovery: 4 input bits then 4 output bits.
DiscoveredSlave makeCombined(uint32_t product, const char* name,
                             uint16_t out_addr, uint16_t out_len,
                             uint16_t in_addr, uint16_t in_len,
                             uint8_t n_in = 4, uint8_t n_out = 4) {
    auto s = makeBase(product, name, out_addr, out_len, in_addr, in_len);
    std::vector<SII::SIIPDOEntry> ie;
    for (uint8_t i = 0; i < n_in; ++i)
        ie.push_back(mkEntry(0x6000, static_cast<uint8_t>(i + 1), 1));
    // One 16-bit raw value field follows the bits (for fieldIn tests).
    ie.push_back(mkEntry(0x6010, 1, 16));
    std::vector<SII::SIIPDOEntry> oe;
    for (uint8_t i = 0; i < n_out; ++i)
        oe.push_back(mkEntry(0x7000, static_cast<uint8_t>(i + 1), 1));
    oe.push_back(mkEntry(0x7010, 1, 16));
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, std::move(ie))};
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, std::move(oe))};
    return s;
}

/// EL3403-shaped (family A) discovery: per phase a status word plus
/// current/voltage/active-power REALs inside the 0x60x0 object.
DiscoveredSlave makePowerMeterFamilyA(uint8_t phases) {
    auto s = makeBase(0x0D4B3052, "EL3403", 0x1100, 8, 0x1180, 64);
    std::vector<SII::SIIPDOEntry> ie;
    for (uint8_t p = 0; p < phases; ++p) {
        const uint16_t base = static_cast<uint16_t>(0x6000 + 0x10 * p);
        ie.push_back(mkEntry(base, 1, 16));    // status word
        ie.push_back(mkEntry(base, 0x11, 32)); // current
        ie.push_back(mkEntry(base, 0x12, 32)); // voltage
        ie.push_back(mkEntry(base, 0x13, 32)); // active power
        ie.push_back(mkEntry(base, 0x0E, 1));  // sync error bit (last)
    }
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, std::move(ie))};
    return s;
}

/// EL3453-shaped (family B) discovery: split V/I object + power object.
DiscoveredSlave makePowerMeterFamilyB() {
    auto s = makeBase(0x0D7D3052, "EL3453", 0x1100, 8, 0x1180, 64);
    std::vector<SII::SIIPDOEntry> ie{
        mkEntry(0x6000, 1, 16),  // status
        mkEntry(0x6001, 0x11, 32),  // voltage
        mkEntry(0x6001, 0x12, 32),  // current
        mkEntry(0x6002, 0x11, 32),  // active power
        mkEntry(0x6002, 0x12, 32),  // apparent power
        mkEntry(0x6002, 0x13, 32),  // reactive power
        mkEntry(0x6002, 0x14, 16),  // power factor (integer)
    };
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, std::move(ie))};
    return s;
}

/// EL3413-shaped (family C) discovery: index selector out, result in.
DiscoveredSlave makePowerMeterFamilyC() {
    auto s = makeBase(0x0D553052, "EL3413", 0x1100, 8, 0x1180, 64);
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 0x01, 8)})};
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 16),
        mkEntry(0x6030, 0x11, 8),   // index echo
        mkEntry(0x6030, 0x12, 8),   // channel echo
        mkEntry(0x6030, 0x13, 32)})};  // value
    return s;
}

class BeckhoffPowerCombinedTerminalsTest : public ::testing::Test {
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
// CombinedIoTerminal — EL2034 (4 in / 4 out packed bits)
// ============================================================================

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedConfigureResolvesBits) {
    auto* slave = addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
    EXPECT_EQ(term.inputChannelCount(), 4u);
    EXPECT_EQ(term.outputChannelCount(), 4u);
    EXPECT_TRUE(term.configure().has_value());  // idempotent
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedInputsAndOutputs) {
    auto* slave = addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);
    ASSERT_TRUE(term.configure().has_value());

    term.setOutput(0, true);
    term.setOutput(2, true);
    EXPECT_TRUE(term.output(0));
    EXPECT_FALSE(term.output(1));
    EXPECT_TRUE(term.output(2));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(2, 0), 0x05);

    // Out-of-range accessors are inert.
    term.setOutput(9, true);
    EXPECT_FALSE(term.output(9));

    term.allOutputsOff();
    EXPECT_FALSE(term.output(0));
    EXPECT_FALSE(term.output(2));
    for (uint8_t b : term.rawOutput()) EXPECT_EQ(b, 0);

    // Inputs come from the emulated SM3 image.
    const uint8_t in[3] = {0x0A, 0x00, 0x00};
    slave->setSmBufferForTest(3, 0, in, sizeof(in));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_FALSE(term.input(0));
    EXPECT_TRUE(term.input(1));
    EXPECT_FALSE(term.input(2));
    EXPECT_TRUE(term.input(3));
    EXPECT_FALSE(term.input(9));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedRawFieldAccess) {
    addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);
    ASSERT_TRUE(term.configure().has_value());

    uint32_t bit_off = 0;
    // The 16-bit raw value follows the 4 input bits: bit offset 4.
    ASSERT_TRUE(term.findInputField(0x6010, 1, 0, bit_off));
    EXPECT_EQ(bit_off, 4u);
    EXPECT_FALSE(term.findInputField(0x9999, 1, 0, bit_off));

    ASSERT_TRUE(term.findOutputField(0x7010, 1, 0, bit_off));
    EXPECT_EQ(bit_off, 4u);
    EXPECT_FALSE(term.findOutputField(0x9999, 1, 0, bit_off));

    // The 16-bit value sits at bit 4 -> byte-aligned? No: it is not.
    uint64_t value = 0;
    EXPECT_FALSE(term.fieldIn(0x6010, 1, 0, value));
    EXPECT_FALSE(term.fieldOut(0x7010, 1, 0, 0x1234));
    EXPECT_FALSE(term.fieldIn(0x9999, 1, 0, value));
    EXPECT_FALSE(term.fieldOut(0x9999, 1, 0, 1));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedByteAlignedFieldAccess) {
    addTerminalSlave(bus_, 0x07F23052, 0x1100, 4, 0x1180, 4);
    bringUp();
    // 8 input bits + a byte-aligned 16-bit value; same on the output side.
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 4, 0x1180, 4, 8, 8);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);
    ASSERT_TRUE(term.configure().has_value());

    uint64_t value = 0;
    ASSERT_TRUE(term.fieldOut(0x7010, 1, 0, 0xBEEF));
    ASSERT_TRUE(term.fieldIn(0x6010, 1, 0, value));
    EXPECT_EQ(value, 0u);  // input image not yet populated

    auto out = term.rawOutput();
    ASSERT_GE(out.size(), 4u);
    EXPECT_EQ(out[1], 0xEF);
    EXPECT_EQ(out[2], 0xBE);

    // Non-mapped value entry -> false.
    EXPECT_FALSE(term.fieldOut(0x1234, 1, 0, 1));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedFallbackWithoutSii) {
    addTerminalSlave(bus_, 0x07F23052, 0x1100, 1, 0x1180, 1);
    bringUp();
    // No process SM in the discovery data -> num_bits packs apply
    // (0x0404 = 4 outputs, 4 inputs), both from bit 0.
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x07F23052;
    ds.device_name = "EL2034";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0x1100, 1, SII::SM_TYPE_PROCESS_OUT),
        mkSm(0x1180, 1, SII::SM_TYPE_PROCESS_IN),
    };

    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.inputChannelCount(), 4u);
    EXPECT_EQ(term.outputChannelCount(), 4u);
    term.setOutput(1, true);
    EXPECT_TRUE(term.output(1));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedRejections) {
    bringUp();
    // Wrong identity.
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal wrong(master_, ds, Dev::EL2032);
    auto r = wrong.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);

    // Slave index out of range.
    Beckhoff::CombinedIoTerminal far(master_, PDO::kMaxPDOSlaves,
                                     Dev::EL2034);
    auto r2 = far.configure();
    ASSERT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), Error::SlaveIndexOutOfRange);

    // Neither SM enabled -> SmConfigFailed.
    DiscoveredSlave empty;
    empty.index = 0;
    empty.vendor_id = 0x00000002;
    empty.product_code = 0x07F23052;
    empty.device_name = "EL2034";
    empty.sync_managers = std::vector<SII::SIISyncManager>{};
    Beckhoff::CombinedIoTerminal none(master_, empty, Dev::EL2034);
    auto r3 = none.configure();
    ASSERT_FALSE(r3.has_value());
    EXPECT_EQ(r3.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedFactories) {
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::CombinedIoTerminal::findFirst(master_, Dev::EL2032,
                                                        scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::CombinedIoTerminal::findFirst(master_, Dev::EL2034,
                                                       scan);
    ASSERT_TRUE(hit.has_value());

    addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto live = Beckhoff::CombinedIoTerminal::findFirst(master_, Dev::EL2034);
    EXPECT_FALSE(live.has_value());
    EXPECT_EQ(live.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedLogicalAndStart) {
    auto* slave = addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);

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
    (void)slave;
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, CombinedStartWithoutManagedLoop) {
    auto* slave = addTerminalSlave(bus_, 0x07F23052, 0x1100, 3, 0x1180, 3);
    bringUp();
    auto ds = makeCombined(0x07F23052, "EL2034", 0x1100, 3, 0x1180, 3);
    Beckhoff::CombinedIoTerminal term(master_, ds, Dev::EL2034);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
}

// ============================================================================
// PowerMeterTerminal — EL3403 (family A), EL3453 (family B), EL3413 (C)
// ============================================================================

namespace {

/// Write a 32-bit REAL at `byte_off` of `buf`.
void putReal(uint8_t* buf, size_t byte_off, float v) {
    std::memcpy(buf + byte_off, &v, 4);
}

}  // namespace

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterFamilyAReadsValues) {
    auto* slave = addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyA(3);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3403);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.phaseCount(), 3u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);

    // Phase 0 image: status 0x0001, then the current / voltage /
    // active-power REALs at bytes 2/6/10 and the sync-error bit after
    // them (bit 112 of the phase block).
    uint8_t img[64] = {};
    img[0] = 0x01;
    img[1] = 0x00;
    putReal(img, 2, 1.5f);    // current
    putReal(img, 6, 230.0f);  // voltage
    putReal(img, 10, 345.0f); // active power
    img[14] = 0x01;           // sync-error bit (bit 112 = byte 14 bit 0)
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.statusWord(0), 0x0001u);
    EXPECT_TRUE(term.syncError(0));
    ASSERT_TRUE(term.current(0).has_value());
    EXPECT_FLOAT_EQ(*term.current(0), 1.5f);
    ASSERT_TRUE(term.voltage(0).has_value());
    EXPECT_FLOAT_EQ(*term.voltage(0), 230.0f);
    ASSERT_TRUE(term.activePower(0).has_value());
    EXPECT_FLOAT_EQ(*term.activePower(0), 345.0f);

    // Family A has no apparent/reactive power mapping.
    EXPECT_FALSE(term.apparentPower(0).has_value());
    EXPECT_FALSE(term.reactivePower(0).has_value());
    EXPECT_FALSE(term.powerFactor(0).has_value());

    // Out-of-range phase accessors.
    EXPECT_EQ(term.statusWord(9), 0u);
    EXPECT_FALSE(term.syncError(9));
    EXPECT_FALSE(term.voltage(9).has_value());
    EXPECT_FALSE(term.current(9).has_value());
    EXPECT_FALSE(term.activePower(9).has_value());
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterFamilyBReadsValues) {
    addTerminalSlave(bus_, 0x0D7D3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyB();
    auto* slave = bus_.net.getSlave(0);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3453);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.phaseCount(), 1u);

    uint8_t img[64] = {};
    img[0] = 0x02;
    img[1] = 0x00;
    putReal(img, 2, 400.0f);   // voltage (0x6001:0x11)
    putReal(img, 6, 2.5f);     // current (0x6001:0x12)
    putReal(img, 10, 1000.0f); // active power
    putReal(img, 14, 1100.0f); // apparent power
    putReal(img, 18, 50.0f);   // reactive power
    const int16_t pf = 950;    // 16-bit integer power factor
    std::memcpy(img + 22, &pf, 2);
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_FLOAT_EQ(*term.voltage(0), 400.0f);
    EXPECT_FLOAT_EQ(*term.current(0), 2.5f);
    EXPECT_FLOAT_EQ(*term.activePower(0), 1000.0f);
    EXPECT_FLOAT_EQ(*term.apparentPower(0), 1100.0f);
    EXPECT_FLOAT_EQ(*term.reactivePower(0), 50.0f);
    ASSERT_TRUE(term.powerFactor(0).has_value());
    EXPECT_FLOAT_EQ(*term.powerFactor(0), 950.0f);  // integer -> float
    EXPECT_FALSE(term.syncError(0));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterFamilyCIndexSelectors) {
    addTerminalSlave(bus_, 0x0D553052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyC();
    auto* slave = bus_.net.getSlave(0);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3413);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.indexSelectors(), 1u);

    term.setIndexSelector(0, 0x2A);
    EXPECT_EQ(term.rawOutput()[0], 0x2A);
    term.setIndexSelector(9, 0x11);  // out of range: no-op
    EXPECT_EQ(term.rawOutput()[0], 0x2A);

    uint8_t img[64] = {};
    img[0] = 0x00;
    img[1] = 0x00;
    img[2] = 0x2A;  // index echo (0x6030:0x11)
    img[3] = 0x01;  // channel echo (0x6030:0x12)
    putReal(img, 4, 12.5f);  // value (0x6030:0x13)
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    ASSERT_TRUE(term.indexedValue(0).has_value());
    EXPECT_FLOAT_EQ(*term.indexedValue(0), 12.5f);
    ASSERT_TRUE(term.indexedIndexEcho(0).has_value());
    EXPECT_EQ(*term.indexedIndexEcho(0), 0x2Au);
    ASSERT_TRUE(term.indexedChannelEcho(0).has_value());
    EXPECT_EQ(*term.indexedChannelEcho(0), 0x01u);

    EXPECT_FALSE(term.indexedValue(5).has_value());
    EXPECT_FALSE(term.indexedIndexEcho(5).has_value());
    EXPECT_FALSE(term.indexedChannelEcho(5).has_value());
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterGenericAccess) {
    addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyA(1);
    auto* slave = bus_.net.getSlave(0);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3403);
    ASSERT_TRUE(term.configure().has_value());

    uint8_t img[64] = {};
    putReal(img, 2, 42.0f);
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    ASSERT_TRUE(term.valueReal(0x6000, 0x11).has_value());
    EXPECT_FLOAT_EQ(*term.valueReal(0x6000, 0x11), 42.0f);
    EXPECT_FALSE(term.valueReal(0xDEAD, 1).has_value());

    uint64_t raw = 0;
    EXPECT_TRUE(term.valueRaw(0x6000, 0x12, 0, raw));
    EXPECT_FALSE(term.valueRaw(0xDEAD, 1, 0, raw));

    // No RX entry is mapped in family A, so writes fail.
    EXPECT_FALSE(term.fieldOut(0x7000, 1, 0, 1));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterFallbackWithoutSii) {
    addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 4, 0x1180, 64);
    bringUp();
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x0D4B3052;
    ds.device_name = "EL3403";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3403);
    ASSERT_TRUE(term.configure().has_value());
    // Without a TxPDO list the phase layout stays unresolved.
    EXPECT_EQ(term.phaseCount(), 0u);
    EXPECT_EQ(term.indexSelectors(), 0u);
    EXPECT_FALSE(term.voltage(0).has_value());
    EXPECT_FALSE(term.valueReal(0x6000, 0x11).has_value());
    EXPECT_FALSE(term.fieldOut(0x7000, 1, 0, 1));
    uint64_t raw = 1;
    EXPECT_FALSE(term.valueRaw(0x6000, 0x11, 0, raw));
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterRejections) {
    bringUp();
    auto ds = makePowerMeterFamilyA(3);
    Beckhoff::PowerMeterTerminal wrong(master_, ds, Dev::EL3453);
    auto r = wrong.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);

    Beckhoff::PowerMeterTerminal far(master_, PDO::kMaxPDOSlaves, Dev::EL3403);
    auto r2 = far.configure();
    ASSERT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterFactories) {
    auto ds = makePowerMeterFamilyA(3);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::PowerMeterTerminal::findFirst(master_, Dev::EL3453,
                                                       scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::PowerMeterTerminal::findFirst(master_, Dev::EL3403,
                                                      scan);
    ASSERT_TRUE(hit.has_value());

    addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto live = Beckhoff::PowerMeterTerminal::findFirst(master_, Dev::EL3403);
    EXPECT_FALSE(live.has_value());
    EXPECT_EQ(live.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterLogicalAndStart) {
    auto* slave = addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyA(3);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3403);

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
    (void)slave;
}

TEST_F(BeckhoffPowerCombinedTerminalsTest, PowerMeterStartWithoutManagedLoop) {
    auto* slave = addTerminalSlave(bus_, 0x0D4B3052, 0x1100, 8, 0x1180, 64);
    bringUp();
    auto ds = makePowerMeterFamilyA(3);
    Beckhoff::PowerMeterTerminal term(master_, ds, Dev::EL3403);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
}
