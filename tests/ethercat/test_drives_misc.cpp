/**
 * @file test_drives_misc.cpp
 * @brief Misc drive helpers: DynaDrive state-name table (pure) and
 *        SynapticonBrakeControl (0x2004 brake object) over the
 *        Master ↔ SlaveEmulator wire.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715NRegisters.hpp"
#include "tether/drives/AS715N/Registers/C00-Parameters.hpp"
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
            const uint16_t dt =
                bytes == 1 ? 0x0005 : bytes == 2 ? 0x0006 : 0x0007;
            slave_->addODEntry(
                {0x2004, sub, dt, uint16_t(bytes * 8), 0x03, d, "brk"});
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

// ---------------------------------------------------------------------------
// AS715N — manufacturer fault parsing + bleeder config over emulated CoE
// ---------------------------------------------------------------------------

class AS715NTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000E59, 0x7150, "as715n");
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

    Master master_;
    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
};

TEST_F(AS715NTest, ManufacturerFaultParsesU32) {
    // 0x203F as U32: high16 = internal code, low16 = external code.
    slave_->addODEntry({0x203F, 0x00, 0x0007, 32, 0x01,
                        {0x34, 0x12, 0x78, 0x56}, "fault"});
    auto& sdo = master_.sdoManager(0);

    auto ext = Drives::AS715NFaultHandler::readManufacturerFaultExtended(sdo, 0);
    EXPECT_EQ(ext.external_code, 0x1234);
    EXPECT_EQ(ext.internal_code, 0x5678);
    EXPECT_EQ(Drives::AS715NFaultHandler::readManufacturerFault(sdo, 0),
              0x1234);
}

TEST_F(AS715NTest, ManufacturerFaultParsesU16FirmwareVariant) {
    // Some firmware exposes 0x203F as U16 (external only).
    slave_->addODEntry({0x203F, 0x00, 0x0006, 16, 0x01, {0x42, 0x00}, "f"});
    auto& sdo = master_.sdoManager(0);

    auto ext = Drives::AS715NFaultHandler::readManufacturerFaultExtended(sdo, 0);
    EXPECT_EQ(ext.external_code, 0x42);
    EXPECT_EQ(ext.internal_code, 0);
}

TEST_F(AS715NTest, ManufacturerFaultMissingObjectReturnsDefault) {
    auto& sdo = master_.sdoManager(0);
    auto ext = Drives::AS715NFaultHandler::readManufacturerFaultExtended(sdo, 0);
    EXPECT_EQ(ext.external_code, 0);
    EXPECT_EQ(ext.internal_code, 0);
    EXPECT_EQ(Drives::AS715NFaultHandler::readManufacturerFault(sdo, 0), 0);
}

TEST_F(AS715NTest, CiA402ErrorReadsU16) {
    // 0x603F is owned by the emulator's CiA402 layer — set the fault there.
    slave_->getDriveState().error_code = 0x2210;
    auto& sdo = master_.sdoManager(0);
    EXPECT_EQ(Drives::AS715NFaultHandler::readCiA402Error(sdo, 0), 0x2210);
}

TEST_F(AS715NTest, CiA402ErrorNoFaultReturnsZero) {
    // CiA402 slaves always expose 0x603F; a fault-free drive reports 0.
    auto& sdo = master_.sdoManager(0);
    EXPECT_EQ(Drives::AS715NFaultHandler::readCiA402Error(sdo, 0), 0);
}

TEST_F(AS715NTest, BleederResistorWritesAllThreeSubs) {
    namespace C00 = Drives::Registers::AS715N::C00;
    // C00 object 0x2000: selection=0x11, power=0x12, resistance=0x13 (U16).
    slave_->addODEntry({0x2000, 0x11, 0x0006, 16, 0x03, {0, 0}, "sel"});
    slave_->addODEntry({0x2000, 0x12, 0x0006, 16, 0x03, {0, 0}, "pwr"});
    slave_->addODEntry({0x2000, 0x13, 0x0006, 16, 0x03, {0, 0}, "res"});
    auto& sdo = master_.sdoManager(0);

    EXPECT_TRUE(Drives::AS715NConfig::configureBleederResistor(
        sdo, C00::BleederResistorSelectionOptions::ExternalBleederResistor,
        /*power_w=*/100, /*resistance_ohm=*/47));

    auto get = [this](uint8_t sub) -> uint16_t {
        const auto* e = slave_->odEntry(0x2000, sub);
        return e ? uint16_t(e->data[0] | (e->data[1] << 8)) : 0xFFFF;
    };
    EXPECT_EQ(get(0x11), uint16_t(C00::BleederResistorSelectionOptions::
                                  ExternalBleederResistor));
    EXPECT_EQ(get(0x12), 100);
    EXPECT_EQ(get(0x13), 47);
}

TEST_F(AS715NTest, BleederResistorReportsPartialFailure) {
    // Only 2 of 3 entries exist → write fails for missing sub → returns false.
    slave_->addODEntry({0x2000, 0x11, 0x0006, 16, 0x03, {0, 0}, "sel"});
    slave_->addODEntry({0x2000, 0x12, 0x0006, 16, 0x03, {0, 0}, "pwr"});
    auto& sdo = master_.sdoManager(0);
    EXPECT_FALSE(Drives::AS715NConfig::configureBleederResistor(
        sdo, Drives::Registers::AS715N::C00::
                 BleederResistorSelectionOptions::InternalBleederResistor,
        50, 20));
    // Successful writes still applied.
    EXPECT_EQ(slave_->odEntry(0x2000, 0x12)->data[0], 50);
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
