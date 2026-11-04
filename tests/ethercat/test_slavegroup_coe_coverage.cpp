/**
 * @file test_slavegroup_coe_coverage.cpp
 * @brief Emulator-backed coverage for SlaveGroup CoE APIs — async/sync
 *        entry reads and writes, verifyEntry/dumpEntries, broadcast
 *        member resolution and waitForMailboxReady.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/ethercat/SlaveGroup.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

constexpr uint32_t kVendor = 0x00000002;
constexpr uint32_t kProduct = 0x0FA13052;
constexpr uint16_t kTestIndex = 0x4000;

class SlaveGroupCoeCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        for (uint16_t i = 0; i < 2; ++i) {
            auto slave = std::make_unique<SlaveEmulator>();
            Emulator::SIIConfig cfg;
            cfg.vendor_id = kVendor;
            cfg.product_code = kProduct;
            cfg.mailbox_protocol = 0x04;
            // SM0/SM1 mailbox sync managers — without them the ESC RAM
            // mailbox buffers never activate and SDO traffic gets WKC=0.
            cfg.sync_managers.push_back(
                {0x1000, 128,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(uint8_t(0x26)),
                 std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(uint8_t(0x01))});
            cfg.sync_managers.push_back(
                {0x1080, 128,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(uint8_t(0x22)),
                 std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(uint8_t(0x01))});
            slave->setSIIConfig(cfg);
            slaves_[i] = slave.get();
            bus_.net.addSlave(std::move(slave));
        }
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(2);
        for (uint16_t i = 0; i < 2; ++i) {
            master_.sdoManager(i).configureMailbox(0x1000, 128, 0x1080, 128);
            master_.slave(i).assumeMailboxAlreadyConfigured();
        }
        // One U32 test entry per slave with distinct values.
        slaves_[0]->addODEntry({kTestIndex, 0, 0x0007, 32, 0x03,
                                {0x11, 0x22, 0x33, 0x44}, "t"});
        slaves_[1]->addODEntry({kTestIndex, 0, 0x0007, 32, 0x03,
                                {0xAA, 0xBB, 0xCC, 0xDD}, "t"});
    }

    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    ObjectDictionary::ObjectDictionaryEntry entry(uint8_t dt = 0x0007) {
        ObjectDictionary::ObjectDictionaryEntry e{};
        e.index = kTestIndex;
        e.subindex = 0;
        e.data_type = static_cast<ObjectDictionary::ObjectDictionaryDataType>(dt);
        return e;
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slaves_[2] = {};
};

// ============================================================================
// memberIndices / group constructors
// ============================================================================

TEST_F(SlaveGroupCoeCoverageTest, ExplicitAndBroadcastGroups) {
    SlaveGroup explicit_group(master_, {1});
    EXPECT_EQ(explicit_group.indices(), std::vector<uint16_t>{1});
    EXPECT_FALSE(explicit_group.isBroadcast());

    auto all = SlaveGroup::all(master_);
    EXPECT_TRUE(all.isBroadcast());
}

// ============================================================================
// readEntry / writeEntry
// ============================================================================

TEST_F(SlaveGroupCoeCoverageTest, ReadEntryCollectsPerSlaveValues) {
    SlaveGroup group(master_, {0, 1});
    auto results = group.readEntry(entry());
    ASSERT_EQ(results.size(), 2u);
    ASSERT_TRUE(results[0].result.has_value());
    ASSERT_TRUE(results[1].result.has_value());
    EXPECT_EQ(*results[0].result, 0x44332211u);
    EXPECT_EQ(*results[1].result, 0xDDCCBBAAu);
}

TEST_F(SlaveGroupCoeCoverageTest, ReadEntryErrorForMissingObject) {
    SlaveGroup group(master_, {0});
    auto bad = entry();
    bad.index = 0x9999;
    auto results = group.readEntry(bad);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].result.has_value());
}

TEST_F(SlaveGroupCoeCoverageTest, WriteEntryBroadcastValue) {
    SlaveGroup group(master_, {0, 1});
    auto results = group.writeEntry(entry(), 0x01020304);
    ASSERT_EQ(results.size(), 2u);
    EXPECT_TRUE(results[0].result.has_value());
    EXPECT_TRUE(results[1].result.has_value());
    EXPECT_EQ(slaves_[0]->odEntry(kTestIndex, 0)->data[0], 0x04);
    EXPECT_EQ(slaves_[1]->odEntry(kTestIndex, 0)->data[0], 0x04);
}

TEST_F(SlaveGroupCoeCoverageTest, WriteEntryPerSlaveValues) {
    SlaveGroup group(master_, {0, 1});
    const uint64_t values[] = {0x11111111, 0x22222222};
    auto results = group.writeEntry(entry(), std::span<const uint64_t>(values, 2));
    ASSERT_EQ(results.size(), 2u);
    EXPECT_TRUE(results[0].result.has_value());
    EXPECT_TRUE(results[1].result.has_value());
    EXPECT_EQ(slaves_[0]->odEntry(kTestIndex, 0)->data[0], 0x11);
    EXPECT_EQ(slaves_[1]->odEntry(kTestIndex, 0)->data[0], 0x22);
}

TEST_F(SlaveGroupCoeCoverageTest, WriteEntrySpanSizeMismatchRefused) {
    SlaveGroup group(master_, {0, 1});
    const uint64_t values[] = {0x11111111};   // 1 value for 2 members
    auto results = group.writeEntry(entry(), std::span<const uint64_t>(values, 1));
    ASSERT_EQ(results.size(), 2u);
    EXPECT_FALSE(results[0].result.has_value());
    EXPECT_FALSE(results[1].result.has_value());
    // No write reached either slave.
    EXPECT_EQ(slaves_[0]->odEntry(kTestIndex, 0)->data[0], 0x11);
    EXPECT_EQ(slaves_[1]->odEntry(kTestIndex, 0)->data[0], 0xAA);
}

// ============================================================================
// Async variants
// ============================================================================

TEST_F(SlaveGroupCoeCoverageTest, AsyncReadAndWriteFutures) {
    SlaveGroup group(master_, {0});
    auto rf = group.readEntryAsync(entry());
    ASSERT_EQ(rf.size(), 1u);
    ASSERT_TRUE(rf[0].get().has_value());

    auto wf = group.writeEntryAsync(entry(), 0x9999);
    ASSERT_EQ(wf.size(), 1u);
    EXPECT_TRUE(wf[0].get().has_value());

    const uint64_t values[] = {0x7777};
    auto svf = group.writeEntryAsync(entry(),
                                     std::span<const uint64_t>(values, 1));
    ASSERT_EQ(svf.size(), 1u);
    EXPECT_TRUE(svf[0].get().has_value());
    EXPECT_EQ(slaves_[0]->odEntry(kTestIndex, 0)->data[0], 0x77);
}

// ============================================================================
// verifyEntry / dumpEntries
// ============================================================================

TEST_F(SlaveGroupCoeCoverageTest, VerifyEntryMatchesAndMismatches) {
    SlaveGroup group(master_, {0, 1});
    // Both slaves hold different values — a single expected value cannot
    // match both, so verifyEntry reports false.
    EXPECT_FALSE(group.verifyEntry(entry(), 0x44332211, {}, "t"));
    EXPECT_TRUE(group.verifyEntry(entry(), 0, {}, "t") == false);

    SlaveGroup only0(master_, {0});
    EXPECT_TRUE(only0.verifyEntry(entry(), 0x44332211, {}, "t"));
    EXPECT_FALSE(only0.verifyEntry(entry(), 0xDEADBEEF, {}, "t"));
}

TEST_F(SlaveGroupCoeCoverageTest, VerifyEntryFailsOnReadError) {
    SlaveGroup group(master_, {0});
    auto bad = entry();
    bad.index = 0x9999;
    EXPECT_FALSE(group.verifyEntry(bad, 0, {}, "t"));
}

TEST_F(SlaveGroupCoeCoverageTest, DumpEntriesLogsAllSlaves) {
    SlaveGroup group(master_, {0, 1});
    const auto e = entry();
    const ObjectDictionary::ObjectDictionaryEntry* entries[] = {&e};
    group.dumpEntries(std::span<const ObjectDictionary::ObjectDictionaryEntry* const>(entries, 1),
                      "t");
}

// ============================================================================
// waitForMailboxReady
// ============================================================================

TEST_F(SlaveGroupCoeCoverageTest, WaitForMailboxReadyTrue) {
    // Emulator slaves report SM0/SM1 active after mailbox config.
    SlaveGroup group(master_, {0, 1});
    EXPECT_TRUE(group.waitForMailboxReady(200, 5));
}

TEST_F(SlaveGroupCoeCoverageTest, WaitForMailboxReadyTimeout) {
    SlaveGroup group(master_, {0});
    Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    slaves_[0]->setErrorInjection(inj);
    EXPECT_FALSE(group.waitForMailboxReady(50, 5));
}

}  // namespace
