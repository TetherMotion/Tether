/**
 * @file test_slave_groups.cpp
 * @brief SlaveGroup wire transactions, Slave custom PDO mapping SDO path,
 *        and the packet-debug dump code — all over a real
 *        Master ↔ SlaveEmulator wire (EmulatedBus).
 *
 * These files were previously ~0% coverage: every SlaveGroup call is a
 * self-contained wire transaction (BWR broadcast / per-slave APWR
 * multi-datagram frame / APRD status reads), and the custom-PDO path is
 * pure CoE SDO traffic — both answerable by the emulator.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveGroup.hpp"
#include "tether/profiles/cia402/60xx-Parameters.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
using CiA402::Parameters60xx::ControlWord;
using CiA402::Parameters60xx::StatusWord;

namespace {

class SlaveGroupEmulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "g0");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        // BRD-only scan populates discovered_count (allAddressed()/broadcast
        // SDO member resolution depend on it) and allocates the Slave
        // objects — empty options skip the SII reads the emulator lacks.
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();  // drain thread calls back into master_
        master_.packetRouter().shutdown();
    }

    /// Register a writable PDO-mapping object (0xIIII:0 u8 count,
    /// 0xIIII:1..4 u32 entry words) in the emulator OD.
    void addMappingObject(uint16_t index) {
        slave_->addODEntry({index, 0, 0x0005, 8, 0x03, {0}, "map count"});
        for (uint8_t s = 1; s <= 4; ++s) {
            slave_->addODEntry(
                {index, s, 0x0007, 32, 0x03, {0, 0, 0, 0}, "map entry"});
        }
    }

    /// Register an SM PDO-assignment object (0x1C1n:0 u8, :1..8 u16).
    void addAssignObject(uint16_t index) {
        slave_->addODEntry({index, 0, 0x0005, 8, 0x03, {0}, "assign count"});
        for (uint8_t s = 1; s <= 8; ++s) {
            slave_->addODEntry(
                {index, s, 0x0006, 16, 0x03, {0, 0}, "assign entry"});
        }
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// ============================================================================
// SlaveGroup — state requests (SlaveGroup.cpp / SlaveGroup_al.cpp)
// ============================================================================

TEST_F(SlaveGroupEmulatorTest, IndexGroupRequestStateApwr) {
    SlaveGroup g(master_, {0});
    ASSERT_EQ(g.size(), 1u);
    EXPECT_FALSE(g.isBroadcast());

    EXPECT_EQ(g.requestState(SlaveState::PRE_OP), 1);
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);

    // Error-acknowledge variant sets bit 4 of AL_CONTROL.
    EXPECT_EQ(g.requestState(SlaveState::INIT, /*ack_error=*/true), 1);
    EXPECT_EQ(slave_->getState(), SlaveState::INIT);
}

TEST_F(SlaveGroupEmulatorTest, BroadcastGroupRequestStateBwr) {
    SlaveGroup g = SlaveGroup::all(master_);
    EXPECT_TRUE(g.isBroadcast());

    // One BWR datagram — WKC counts the slaves that wrote.
    EXPECT_GE(g.requestState(SlaveState::PRE_OP), 1);
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveGroupEmulatorTest, AllAddressedGroupCoversDiscoveredSlaves) {
    SlaveGroup g = SlaveGroup::allAddressed(master_);
    EXPECT_FALSE(g.isBroadcast());
    ASSERT_EQ(g.size(), 1u);
    EXPECT_EQ(g.indices()[0], 0);

    EXPECT_EQ(g.requestState(SlaveState::PRE_OP), 1);
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
}

TEST_F(SlaveGroupEmulatorTest, ReadStatesReturnsAlStatusAndCode) {
    slave_->requestState(SlaveState::PRE_OP);

    SlaveGroup g(master_, {0});
    auto states = g.readStates();
    ASSERT_EQ(states.size(), 1u);
    EXPECT_TRUE(states[0].responded);
    EXPECT_GT(states[0].wkc, 0);
    EXPECT_EQ(states[0].state(), SlaveState::PRE_OP);
    EXPECT_FALSE(states[0].errorFlag());
    EXPECT_EQ(states[0].al_status_code, 0);
}

TEST_F(SlaveGroupEmulatorTest, WaitForStatePollsUntilReached) {
    SlaveGroup g(master_, {0});
    EXPECT_TRUE(g.waitForState(SlaveState::INIT, 500));

    g.requestState(SlaveState::PRE_OP);
    EXPECT_TRUE(g.waitForState(SlaveState::PRE_OP, 500));

    // Unreachable target → false after the timeout.
    EXPECT_FALSE(g.waitForState(SlaveState::OP, 150, /*poll_interval_ms=*/5));
}

TEST_F(SlaveGroupEmulatorTest, ControlBytePacksStateNibbleAndAck) {
    EXPECT_EQ(SlaveGroup::controlByte(SlaveState::INIT, false), 0x01);
    EXPECT_EQ(SlaveGroup::controlByte(SlaveState::PRE_OP, false), 0x02);
    EXPECT_EQ(SlaveGroup::controlByte(SlaveState::OP, false), 0x08);
    EXPECT_EQ(SlaveGroup::controlByte(SlaveState::INIT, true), 0x11);
    EXPECT_EQ(SlaveGroup::controlByte(SlaveState::OP, true), 0x18);
}

TEST_F(SlaveGroupEmulatorTest, EmptyIndexGroupIsHarmless) {
    const std::vector<uint16_t> none;
    SlaveGroup g(master_, none);
    EXPECT_TRUE(g.empty());
    EXPECT_TRUE(g.readStates().empty());
}

// ============================================================================
// SlaveGroup — group CoE SDO (SlaveGroup_coe.cpp)
// ============================================================================

TEST_F(SlaveGroupEmulatorTest, GroupSdoWriteThenReadAllMembers) {
    SlaveGroup g(master_, {0});

    auto writes = g.writeEntry(ControlWord, 0x0006);
    ASSERT_EQ(writes.size(), 1u);
    EXPECT_EQ(writes[0].slave_index, 0);
    ASSERT_TRUE(writes[0].result.has_value());

    auto reads = g.readEntry(ControlWord);
    ASSERT_EQ(reads.size(), 1u);
    ASSERT_TRUE(reads[0].result.has_value());
    EXPECT_EQ(*reads[0].result, 0x0006u);
}

TEST_F(SlaveGroupEmulatorTest, GroupSdoAsyncVariants) {
    SlaveGroup g(master_, {0});

    auto wfuts = g.writeEntryAsync(ControlWord, 0x0007);
    ASSERT_EQ(wfuts.size(), 1u);
    ASSERT_TRUE(wfuts[0].get().has_value());

    auto rfuts = g.readEntryAsync(ControlWord);
    ASSERT_EQ(rfuts.size(), 1u);
    auto res = rfuts[0].get();
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(*res, 0x0007u);
}

TEST_F(SlaveGroupEmulatorTest, BroadcastGroupSdoResolvesDiscoveredMembers) {
    SlaveGroup g = SlaveGroup::all(master_);
    auto reads = g.readEntry(ControlWord);
    ASSERT_EQ(reads.size(), 1u);  // broadcast resolves to the one member
    EXPECT_TRUE(reads[0].result.has_value());
}

// ============================================================================
// Slave custom PDO mapping (Slave_pdo_custom.cpp)
// ============================================================================

TEST_F(SlaveGroupEmulatorTest, ConfigureCustomRxPDOWritesMappingEntries) {
    addMappingObject(0x1601);

    auto err = master_.slave(0).configureCustomRxPDO(
        0x1601, {CustomPDOMappingEntry{&ControlWord}});
    EXPECT_EQ(err, SlaveError::Ok);

    // Mapping object on the wire: count=1, entry word 0x60400010 (LE).
    const auto* count = slave_->odEntry(0x1601, 0);
    ASSERT_NE(count, nullptr);
    ASSERT_EQ(count->data.size(), 1u);
    EXPECT_EQ(count->data[0], 1);

    const auto* word = slave_->odEntry(0x1601, 1);
    ASSERT_NE(word, nullptr);
    ASSERT_EQ(word->data.size(), 4u);
    const uint32_t val = static_cast<uint32_t>(word->data[0]) |
                         (static_cast<uint32_t>(word->data[1]) << 8) |
                         (static_cast<uint32_t>(word->data[2]) << 16) |
                         (static_cast<uint32_t>(word->data[3]) << 24);
    EXPECT_EQ(val, 0x60400010u);
}

TEST_F(SlaveGroupEmulatorTest, ConfigureCustomTxPDOWritesMappingEntries) {
    addMappingObject(0x1A01);

    auto err = master_.slave(0).configureCustomTxPDO(
        0x1A01, {CustomPDOMappingEntry{&StatusWord},
                 CustomPDOMappingEntry{&StatusWord, /*sz=*/4}});
    EXPECT_EQ(err, SlaveError::Ok);

    const auto* count = slave_->odEntry(0x1A01, 0);
    ASSERT_NE(count, nullptr);
    ASSERT_EQ(count->data.size(), 1u);
    EXPECT_EQ(count->data[0], 2);
}

TEST_F(SlaveGroupEmulatorTest, ConfigureCustomPDOAbortsWhenObjectMissing) {
    // 0x1699 is not in the OD — first sdoWriteU8 aborts → error, not a hang.
    auto err = master_.slave(0).configureCustomRxPDO(
        0x1699, {CustomPDOMappingEntry{&ControlWord}});
    EXPECT_NE(err, SlaveError::Ok);
}

TEST_F(SlaveGroupEmulatorTest, RegisterExistingTxPDOReadsMappingFromSlave) {
    // Firmware-fixed mapping: one 16-bit statusword entry.
    slave_->addODEntry({0x1A02, 0, 0x0005, 8, 0x03, {1}, "map count"});
    slave_->addODEntry(
        {0x1A02, 1, 0x0007, 32, 0x03, {0x10, 0x00, 0x41, 0x60}, "entry"});

    auto err = master_.slave(0).registerExistingTxPDO(0x1A02);
    EXPECT_EQ(err, SlaveError::Ok);
}

TEST_F(SlaveGroupEmulatorTest, RegisterExistingTxPDOFailsWhenObjectMissing) {
    auto err = master_.slave(0).registerExistingTxPDO(0x1A09);
    EXPECT_NE(err, SlaveError::Ok);
}

TEST_F(SlaveGroupEmulatorTest, ApplyCustomPDOsWritesSmAssignments) {
    addMappingObject(0x1601);
    addMappingObject(0x1A01);
    addAssignObject(0x1C12);
    addAssignObject(0x1C13);

    ASSERT_EQ(master_.slave(0).configureCustomRxPDO(
                  0x1601, {CustomPDOMappingEntry{&ControlWord}}),
              SlaveError::Ok);
    ASSERT_EQ(master_.slave(0).configureCustomTxPDO(
                  0x1A01, {CustomPDOMappingEntry{&StatusWord}}),
              SlaveError::Ok);

    auto err = master_.slave(0).applyCustomPDOs();
    EXPECT_EQ(err, SlaveError::Ok);

    const auto* rx = slave_->odEntry(0x1C12, 0);
    ASSERT_NE(rx, nullptr);
    ASSERT_EQ(rx->data.size(), 1u);
    EXPECT_EQ(rx->data[0], 1);  // one RxPDO assigned to SM2

    const auto* tx = slave_->odEntry(0x1C13, 0);
    ASSERT_NE(tx, nullptr);
    ASSERT_EQ(tx->data.size(), 1u);
    EXPECT_EQ(tx->data[0], 1);
}

TEST_F(SlaveGroupEmulatorTest, ClearCustomPDOsResetsLocalRegistration) {
    addMappingObject(0x1601);
    ASSERT_EQ(master_.slave(0).configureCustomRxPDO(
                  0x1601, {CustomPDOMappingEntry{&ControlWord}}),
              SlaveError::Ok);

    master_.slave(0).clearCustomPDOs();
    // With nothing registered, apply is a no-op success.
    EXPECT_EQ(master_.slave(0).applyCustomPDOs(), SlaveError::Ok);
}

// ============================================================================
// PacketDebugger — debug-flag-gated frame dumps (raw/PacketDebugger.cpp)
// ============================================================================

TEST_F(SlaveGroupEmulatorTest, PacketDebugFlagsDuringWireOps) {
    master_.debugFlags().txPackets = true;
    master_.debugFlags().rxPackets = true;

    SlaveGroup g(master_, {0});
    EXPECT_EQ(g.requestState(SlaveState::PRE_OP), 1);
    auto states = g.readStates();
    ASSERT_EQ(states.size(), 1u);
    EXPECT_TRUE(states[0].responded);

    master_.debugFlags().txPackets = false;
    master_.debugFlags().rxPackets = false;
}
