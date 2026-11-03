/**
 * @file test_beckhoff_emulator.cpp
 * @brief Beckhoff terminal drivers over a real Master ↔ SlaveEmulator wire
 *        (EmulatedBus).
 *
 * The existing test_beckhoff_terminals.cpp covers layouts and registries
 * but never instantiates the terminal classes — the driver bring-up code
 * (SM programming, PDO registration, state transitions, cyclic exchange)
 * sat at ~0% coverage.  These tests hand the drivers a DiscoveredSlave
 * (skipping SII reads) and run configure()/PDO exchange end-to-end
 * against the emulator.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/EL1014.hpp"
#include "tether/Beckhoff/EL2004.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using EtherCAT::Beckhoff::EL1014;
using EtherCAT::Beckhoff::EL2004;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

/// Mailbox-less EL-style slave: a single PDO sync manager on channel 0.
SlaveEmulator* addElSlave(EmulatedBus& bus,
                          uint32_t vendor, uint32_t product,
                          uint16_t pdo_addr, uint16_t pdo_len,
                          bool is_output) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = vendor;
    cfg.product_code = product;
    cfg.mailbox_protocol = 0;  // EL2004/EL1014 have no mailbox
    EtherCAT::Emulator::SIIConfig::SMConfig sm{};
    sm.start_addr = pdo_addr;
    sm.length     = pdo_len;
    // Same control bytes the CiA402 factory uses for PDO SMs.
    sm.control    = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
        static_cast<uint8_t>(is_output ? 0x44 : 0x40));
    sm.enable     = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(
        static_cast<uint8_t>(0x01));
    cfg.sync_managers.push_back(sm);
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

DiscoveredSlave makeDiscovered(uint32_t vendor, uint32_t product,
                               const char* name, bool is_output,
                               uint16_t sm_addr, uint16_t sm_len,
                               uint8_t num_bits) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = vendor;
    s.product_code = product;
    s.device_name = name;

    SII::SIISyncManager sm{};
    sm.phys_start_address = sm_addr;
    sm.length = sm_len;
    sm.sm_type = is_output ? SII::SM_TYPE_PROCESS_OUT
                         : SII::SM_TYPE_PROCESS_IN;
    s.sync_managers = std::vector<SII::SIISyncManager>{sm};

    // PDO category: N single-bit channels → lets the driver derive the
    // SM length and bit count exactly like a real SII read.
    SII::SIIPDO pdo{};
    pdo.pdo_index = is_output ? 0x1600 : 0x1A00;
    pdo.n_entries = num_bits;
    pdo.sync_manager = 0;
    for (uint8_t i = 0; i < num_bits; ++i) {
        SII::SIIPDOEntry e{};
        e.index = is_output ? 0x7010 : 0x6010;
        e.subindex = static_cast<uint8_t>(i + 1);
        e.bit_length = 1;
        pdo.entries.push_back(e);
    }
    if (is_output) s.rx_pdos = std::vector<SII::SIIPDO>{pdo};
    else           s.tx_pdos = std::vector<SII::SIIPDO>{pdo};
    return s;
}

class BeckhoffEmulatorTest : public ::testing::Test {
protected:
    void TearDown() override {
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

} // namespace

// ============================================================================
// EL2004 — 4-channel digital output
// ============================================================================

TEST_F(BeckhoffEmulatorTest, EL2004ConfigureReachesSafeOp) {
    auto* slave = addElSlave(bus_, 0x00000002, 0x07D43052, 0x1100, 1, true);
    bringUp();

    auto ds = makeDiscovered(0x00000002, 0x07D43052, "EL2004",
                             true, 0x1100, 0, 4);
    EL2004 term(master_, ds);
    auto r = term.configure();
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.bitCount(), 4u);
    EXPECT_EQ(term.alState(), SlaveState::SAFE_OP);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
}

TEST_F(BeckhoffEmulatorTest, EL2004OutputBitsReachSmBuffer) {
    auto* slave = addElSlave(bus_, 0x00000002, 0x07D43052, 0x1100, 1, true);
    bringUp();
    auto ds = makeDiscovered(0x00000002, 0x07D43052, "EL2004",
                             true, 0x1100, 0, 4);
    EL2004 term(master_, ds);
    ASSERT_TRUE(term.configure().has_value());

    term.setBit(0, true);
    term.setBit(2, true);
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    // The wire write lands in the emulator's SM0 buffer.
    EXPECT_EQ(slave->smBufferByteForTest(0, 0), 0x05);

    term.allOn();
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(0, 0), 0x0F);

    term.allOff();
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(0, 0), 0x00);
}

TEST_F(BeckhoffEmulatorTest, EL2004WrongIdentityRejected) {
    addElSlave(bus_, 0x00000002, 0x07D43052, 0x1100, 1, true);
    bringUp();
    // Claim the slave is an EL1014 instead → identity mismatch.
    auto ds = makeDiscovered(0x00000002, 0x03F63052, "EL1014",
                             true, 0x1100, 0, 4);
    EL2004 term(master_, ds);
    EXPECT_FALSE(term.configure().has_value());
}

// ============================================================================
// EL1014 — 4-channel digital input
// ============================================================================

TEST_F(BeckhoffEmulatorTest, EL1014ConfigureReachesSafeOp) {
    auto* slave = addElSlave(bus_, 0x00000002, 0x03F63052, 0x1180, 1, false);
    bringUp();
    auto ds = makeDiscovered(0x00000002, 0x03F63052, "EL1014",
                             false, 0x1180, 0, 4);
    EL1014 term(master_, ds);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.bitCount(), 4u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
}

TEST_F(BeckhoffEmulatorTest, EL1014InputBitsReadFromSmBuffer) {
    auto* slave = addElSlave(bus_, 0x00000002, 0x03F63052, 0x1180, 1, false);
    bringUp();
    auto ds = makeDiscovered(0x00000002, 0x03F63052, "EL1014",
                             false, 0x1180, 0, 4);
    EL1014 term(master_, ds);
    ASSERT_TRUE(term.configure().has_value());

    // Drive inputs high on the emulated terminal, then exchange.
    slave->setSmBufferByteForTest(0, 0x0A);  // bits 1 and 3
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_FALSE(term.bit(0));
    EXPECT_TRUE(term.bit(1));
    EXPECT_FALSE(term.bit(2));
    EXPECT_TRUE(term.bit(3));
    EXPECT_EQ(term.bits() & 0x0F, 0x0Au);
}
