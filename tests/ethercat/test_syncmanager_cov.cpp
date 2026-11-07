/**
 * @file test_syncmanager_cov.cpp
 * @brief SyncManagerAccessor coverage over the emulated wire — PDO
 *        assignment reads/writes (incl. missing-object aborts), CommType
 *        validation, and dumpMailboxStatus failure branches.
 */

#include <gtest/gtest.h>

#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/ethercat/SyncManager.hpp"

using namespace EtherCAT;

namespace {

void addOD(Emulator::SlaveEmulator* s, uint16_t index, uint8_t sub,
           uint16_t data_type, uint16_t bits, uint8_t access,
           std::vector<uint8_t> data = {}) {
    Emulator::ODEntry e{};
    e.index = index;
    e.subindex = sub;
    e.data_type = data_type;
    e.bit_length = bits;
    e.access = access;   // 0x01 read | 0x02 write
    e.data = std::move(data);
    if (e.data.empty()) e.data.assign((bits + 7) / 8, 0);
    e.name = "od";
    s->addODEntry(e);
}

class SyncManagerCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto s = Emulator::createCiA402Drive(0x00000002, 0xCAFE, "smcov");
        slave_ = s.get();
        // SM2/SM3 PDO-assignment arrays (0x1C12/0x1C13).
        for (uint16_t idx : {0x1C12, 0x1C13}) {
            addOD(slave_, idx, 0, 0x0005, 8, 0x03);          // count U8
            for (uint8_t sub = 1; sub <= 8; ++sub)
                addOD(slave_, idx, sub, 0x0006, 16, 0x03);   // assign U16
        }
        // SM CommType array (0x1C00:sub = smIndex+1).
        for (uint8_t sub = 1; sub <= 4; ++sub)
            addOD(slave_, 0x1C00, sub, 0x0005, 8, 0x01);     // U8 RO

        bus_.net.addSlave(std::move(s));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
        master_.slave(0).assumeMailboxAlreadyConfigured();
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// ---------------------------------------------------------------------------
// PDO assignment reads.
// ---------------------------------------------------------------------------

TEST_F(SyncManagerCovTest, ReadPdoAssignCountAndEntries) {
    auto& slave = master_.slave(0);
    // Seed the assignment array over the wire: count=2, subs 0x1600/0x1601.
    ASSERT_EQ(slave.sm(2).writePDOAssignments({0x1600, 0x1601}),
              SlaveError::Ok);

    uint8_t count = 0;
    EXPECT_EQ(slave.sm(2).readPDOAssignCount(count), SlaveError::Ok);
    EXPECT_EQ(count, 2u);

    uint16_t pdo_idx = 0;
    EXPECT_EQ(slave.sm(2).readPDOAssignment(1, pdo_idx), SlaveError::Ok);
    EXPECT_EQ(pdo_idx, 0x1600u);
    EXPECT_EQ(slave.sm(2).readPDOAssignment(2, pdo_idx), SlaveError::Ok);
    EXPECT_EQ(pdo_idx, 0x1601u);

    std::vector<uint16_t> all;
    EXPECT_EQ(slave.sm(2).readAllPDOAssignments(all), SlaveError::Ok);
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[0], 0x1600u);
    EXPECT_EQ(all[1], 0x1601u);
}

// SM1's assign object (0x1C11) isn't in the emulator OD → SDO abort →
// error return on every read entry point.
TEST_F(SyncManagerCovTest, ReadAssignMissingObjectFails) {
    auto& slave = master_.slave(0);
    uint8_t count = 0;
    EXPECT_NE(slave.sm(1).readPDOAssignCount(count), SlaveError::Ok);

    uint16_t pdo_idx = 0;
    EXPECT_NE(slave.sm(1).readPDOAssignment(1, pdo_idx), SlaveError::Ok);

    std::vector<uint16_t> all;
    EXPECT_NE(slave.sm(1).readAllPDOAssignments(all), SlaveError::Ok);
}

// readAllPDOAssignments: count reports more entries than the OD has —
// the mid-loop read failure propagates as an error.
TEST_F(SyncManagerCovTest, ReadAllAssignmentsMidLoopFailure) {
    auto& slave = master_.slave(0);
    // Poke the count subindex to 9 — beyond the 8 registered subs — so
    // the loop's sub9 read aborts over SDO.
    {
        const Emulator::ODEntry* e = slave_->odEntry(0x1C12, 0);
        ASSERT_NE(e, nullptr);
        Emulator::ODEntry bigger = *e;
        bigger.data = {9};
        slave_->addODEntry(bigger);
    }

    std::vector<uint16_t> all;
    EXPECT_NE(slave.sm(2).readAllPDOAssignments(all), SlaveError::Ok);
}

// ---------------------------------------------------------------------------
// PDO assignment writes: CiA 301 ordering + clear + missing-object abort.
// ---------------------------------------------------------------------------

TEST_F(SyncManagerCovTest, WritePdoAssignmentsVectorAndClear) {
    auto& slave = master_.slave(0);
    const std::vector<uint16_t> mapping{0x1600, 0x1601, 0x1602};
    EXPECT_EQ(slave.sm(3).writePDOAssignments(mapping), SlaveError::Ok);

    // OD check: count=3, entries landed.
    const Emulator::ODEntry* cnt = slave_->odEntry(0x1C13, 0);
    ASSERT_NE(cnt, nullptr);
    ASSERT_EQ(cnt->data.size(), 1u);
    EXPECT_EQ(cnt->data[0], 3u);

    EXPECT_EQ(slave.sm(3).clearPDOAssignments(), SlaveError::Ok);
    EXPECT_EQ(slave_->odEntry(0x1C13, 0)->data[0], 0u);
}

// Writing into a missing assign object aborts on the first sdoWriteU8.
TEST_F(SyncManagerCovTest, WriteAssignMissingObjectFails) {
    auto& slave = master_.slave(0);
    EXPECT_NE(slave.sm(1).writePDOAssignments({0x1600}), SlaveError::Ok);
    EXPECT_NE(slave.sm(1).clearPDOAssignments(), SlaveError::Ok);
}

// ---------------------------------------------------------------------------
// CommType validation.
// ---------------------------------------------------------------------------

TEST_F(SyncManagerCovTest, ValidateCommTypeMatchAndMismatch) {
    auto& slave = master_.slave(0);
    // Set SM2's comm-type byte (0x1C00:3 = sm2) to ProcessOutput (0x03).
    {
        const Emulator::ODEntry* e = slave_->odEntry(0x1C00, 3);
        ASSERT_NE(e, nullptr);
        Emulator::ODEntry typed = *e;
        typed.data = {0x03};
        slave_->addODEntry(typed);
    }

    auto ok = slave.sm(2).validateCommType(
        SyncManager::CommType::ProcessOutput);
    EXPECT_TRUE(ok.valid);

    auto bad = slave.sm(2).validateCommType(
        SyncManager::CommType::ProcessInput);
    EXPECT_FALSE(bad.valid);
    EXPECT_NE(bad.message.find("mismatch"), std::string::npos);
}

// Missing 0x1C00 sub → read fails → invalid result with SDO message.
TEST_F(SyncManagerCovTest, ValidateCommTypeReadFails) {
    auto& slave = master_.slave(0);
    auto res = slave.sm(7).validateCommType(
        SyncManager::CommType::ProcessOutput);   // 0x1C00:8 not registered
    EXPECT_FALSE(res.valid);
    EXPECT_NE(res.message.find("failed"), std::string::npos);
}

// ---------------------------------------------------------------------------
// dumpMailboxStatus: missing SM registers → both read-failure branches.
// ---------------------------------------------------------------------------

TEST_F(SyncManagerCovTest, DumpMailboxStatusMissingRegs) {
    auto& slave = master_.slave(42);  // NonExistingSlave → reads fail
    slave.sm(0).dumpMailboxStatus("test-sm");
    slave.sm(1).dumpMailboxStatus("test-sm");
}
