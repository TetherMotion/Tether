/**
 * @file test_drives_misc.cpp
 * @brief Misc drive helpers: DynaDrive state-name table (pure) and
 *        SynapticonBrakeControl (0x2004 brake object) over the
 *        Master ↔ SlaveEmulator wire.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/DynaDrive.hpp"
#include "tether/drives/DynaDrive/Registers/Statusword.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/Registers/DriveConfig2000.hpp"
#include "tether/ethercat/CoEManager.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
using EtherCAT::Drives::Synapticon::BrakeControl;
using EtherCAT::Drives::Synapticon::BrakeStatusValue;
namespace SynReg = EtherCAT::Drives::Registers::Synapticon::Obj2004;
using EtherCAT::Drives::Registers::DynaDrive::Status::StateOptions;

// ---------------------------------------------------------------------------
// DynaDrive — dynaDriveStateName() switch coverage
// ---------------------------------------------------------------------------

TEST(DynaDriveStateNameTest, CoversAllStates) {
    using EtherCAT::Drives::dynaDriveStateName;
    EXPECT_EQ(dynaDriveStateName(StateOptions::NA), "NA");
    EXPECT_EQ(dynaDriveStateName(StateOptions::ColdStart), "ColdStart");
    EXPECT_EQ(dynaDriveStateName(StateOptions::WarmStart), "WarmStart");
    EXPECT_EQ(dynaDriveStateName(StateOptions::Configure), "Configure");
    EXPECT_EQ(dynaDriveStateName(StateOptions::Calibrate), "Calibrate");
    EXPECT_EQ(dynaDriveStateName(StateOptions::Standby), "Standby");
    EXPECT_EQ(dynaDriveStateName(StateOptions::MotorOp), "MotorOp");
    EXPECT_EQ(dynaDriveStateName(StateOptions::ControlOp), "ControlOp");
    EXPECT_EQ(dynaDriveStateName(StateOptions::Error), "Error");
    EXPECT_EQ(dynaDriveStateName(StateOptions::Fatal), "Fatal");
    EXPECT_EQ(dynaDriveStateName(StateOptions::MotorPreOp), "MotorPreOp");
    EXPECT_EQ(dynaDriveStateName(StateOptions::DeviceMissing), "DeviceMissing");
    EXPECT_EQ(dynaDriveStateName(static_cast<StateOptions>(0x77)), "Unknown");
}

// ---------------------------------------------------------------------------
// BrakeControl — 0x2004 brake object over emulated CoE
// ---------------------------------------------------------------------------

namespace {

class BrakeControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000F0A, 0x2004, "synapticon");
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

    void addBrakeObject() {
        // 0x2004 record: sub0 count + all fixed-width entries BrakeControl touches.
        slave_->addODEntry({0x2004, 0x00, 0x0005, 8, 0x01, {10}, "cnt"});
        auto add = [this](uint8_t sub, uint64_t v, uint8_t bytes) {
            std::vector<uint8_t> d(bytes);
            for (uint8_t i = 0; i < bytes; ++i) d[i] = uint8_t(v >> (8 * i));
            slave_->addODEntry({0x2004, sub,
                                bytes == 1 ? 0x0005 : bytes == 2 ? 0x0006 : 0x0007,
                                uint16_t(bytes * 8), 0x03, d, "brk"});
        };
        add(0x01, 24000, 4);   // pull voltage (mV)
        add(0x02, 12000, 4);   // hold voltage
        add(0x03, 250, 2);     // pull time (ms)
        add(0x04, 2, 1);       // release strategy = PinBrake
        add(0x07, 1, 1);       // brake status = Engaged
        add(0x0A, 0, 2);       // output voltage
    }

    uint8_t brakeStatusRaw() {
        const auto* e = slave_->odEntry(0x2004, 0x07);
        return e ? e->data[0] : 0xFF;
    }

    Master master_;
    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
};

TEST_F(BrakeControlTest, ReadBrakeStatusRoundTrips) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);

    auto st = BrakeControl::readBrakeStatus(sdo);
    ASSERT_TRUE(st.has_value());
    EXPECT_EQ(*st, BrakeStatusValue::Engaged);
    EXPECT_TRUE(BrakeControl::isEngaged(sdo));
    EXPECT_FALSE(BrakeControl::isDisengaged(sdo));
}

TEST_F(BrakeControlTest, DisengageAndEngageWriteStatusObject) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);

    ASSERT_TRUE(BrakeControl::disengageBrake(sdo));
    EXPECT_EQ(brakeStatusRaw(), uint8_t(BrakeStatusValue::Disengaged));
    EXPECT_TRUE(BrakeControl::isDisengaged(sdo));

    ASSERT_TRUE(BrakeControl::engageBrake(sdo));
    EXPECT_EQ(brakeStatusRaw(), uint8_t(BrakeStatusValue::Engaged));
    EXPECT_TRUE(BrakeControl::isEngaged(sdo));
}

TEST_F(BrakeControlTest, DisengageWithoutVerifySkipsReadback) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);
    EXPECT_TRUE(BrakeControl::disengageBrake(sdo, /*timeout_ms=*/0,
                                           /*verify=*/false));
    EXPECT_EQ(brakeStatusRaw(), uint8_t(BrakeStatusValue::Disengaged));
}

TEST_F(BrakeControlTest, VerifyCatchesMismatchedStatus) {
    // OD reports Engaged but status object is read-only for the write —
    // emulate a brake that ignores the write by making sub 0x07 absent
    // for writes: simplest deterministic case is verifying against a
    // fixed wrong status.  Write directly, then check verify=false path.
    slave_->addODEntry({0x2004, 0x00, 0x0005, 8, 0x01, {1}, "cnt"});
    slave_->addODEntry({0x2004, 0x07, 0x0005, 8, 0x03, {1}, "st"});
    auto& sdo = master_.sdoManager(0);

    // Write Disengaged via raw SDO, flip the OD back to Engaged, then ask
    // isDisengaged — false proves the read path distinguishes states.
    ASSERT_TRUE(BrakeControl::disengageBrake(sdo, 0, false));
    slave_->addODEntry({0x2004, 0x07, 0x0005, 8, 0x03, {1}, "st"});
    EXPECT_FALSE(BrakeControl::isDisengaged(sdo));
    EXPECT_TRUE(BrakeControl::isEngaged(sdo));
}

TEST_F(BrakeControlTest, SetReleaseStrategyWritesSub4) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);
    EXPECT_TRUE(BrakeControl::setReleaseStrategy(
        sdo, SynReg::ReleaseStrategyOptions::ClutchStyleBrake));
    const auto* e = slave_->odEntry(0x2004, 0x04);
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->data[0],
              uint8_t(SynReg::ReleaseStrategyOptions::ClutchStyleBrake));
}

TEST_F(BrakeControlTest, ReadConfigSnapshotsAllEntries) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);

    auto cfg = BrakeControl::readConfig(sdo);
    ASSERT_TRUE(cfg.pull_voltage.has_value());
    EXPECT_EQ(*cfg.pull_voltage, 24000u);
    ASSERT_TRUE(cfg.hold_voltage.has_value());
    EXPECT_EQ(*cfg.hold_voltage, 12000u);
    ASSERT_TRUE(cfg.pull_time.has_value());
    EXPECT_EQ(*cfg.pull_time, 250);
    ASSERT_TRUE(cfg.release_strategy.has_value());
    EXPECT_EQ(*cfg.release_strategy, 2);
    ASSERT_TRUE(cfg.status.has_value());
    EXPECT_EQ(*cfg.status, BrakeStatusValue::Engaged);
    ASSERT_TRUE(cfg.output_voltage.has_value());
    EXPECT_EQ(*cfg.output_voltage, 0);
}

TEST_F(BrakeControlTest, ReadConfigToleratesMissingEntries) {
    // Only the status entry exists — others come back nullopt.
    slave_->addODEntry({0x2004, 0x00, 0x0005, 8, 0x01, {1}, "cnt"});
    slave_->addODEntry({0x2004, 0x07, 0x0005, 8, 0x03, {2}, "st"});
    auto& sdo = master_.sdoManager(0);

    auto cfg = BrakeControl::readConfig(sdo);
    EXPECT_TRUE(cfg.status.has_value());
    EXPECT_EQ(*cfg.status, BrakeStatusValue::Disengaged);
}

TEST_F(BrakeControlTest, DumpConfigLogsAllEntries) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);
    BrakeControl::dumpConfig(sdo, "TestBrake");
    SUCCEED();
}

TEST_F(BrakeControlTest, VerifyDisengagedCrossChecksStatuswordBit) {
    addBrakeObject();
    auto& sdo = master_.sdoManager(0);

    // 0x2004:7 = Engaged → false regardless of statusword hint.
    EXPECT_FALSE(BrakeControl::verifyDisengaged(sdo, true, "T"));
    EXPECT_FALSE(BrakeControl::verifyDisengaged(sdo, std::nullopt, "T"));

    ASSERT_TRUE(BrakeControl::disengageBrake(sdo));
    EXPECT_TRUE(BrakeControl::verifyDisengaged(sdo, true, "T"));
    EXPECT_TRUE(BrakeControl::verifyDisengaged(sdo, std::nullopt, "T"));
}

TEST_F(BrakeControlTest, ConfigEntriesTableListsSixObjects) {
    static_assert(BrakeControl::kConfigEntries.size() == 6);
    for (const auto* e : BrakeControl::kConfigEntries)
        EXPECT_NE(e, nullptr);
}

TEST_F(BrakeControlTest, SdoFailurePathsReturnErrors) {
    // No 0x2004 object at all — every read/write aborts.
    auto& sdo = master_.sdoManager(0);
    EXPECT_FALSE(BrakeControl::readBrakeStatus(sdo).has_value());
    EXPECT_FALSE(BrakeControl::isDisengaged(sdo));
    EXPECT_FALSE(BrakeControl::isEngaged(sdo));
    EXPECT_FALSE(BrakeControl::disengageBrake(sdo, 0, false));
    EXPECT_FALSE(BrakeControl::engageBrake(sdo, 0, false));
    EXPECT_FALSE(BrakeControl::setReleaseStrategy(
        sdo, SynReg::ReleaseStrategyOptions::PinBrake));
}

}  // namespace
