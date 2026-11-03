/**
 * @file test_dynadrive_controller.cpp
 * @brief Coverage for DynaDriveController — state decode/name tables and
 *        the SDO-driven FSM transitions over an emulated bus.
 *
 * The emulated slave is a plain CoE device (not a CiA402 emulator) so the
 * injected 32-bit 0x6041 statusword object is served verbatim — the CiA402
 * emulator would overwrite it from its own state machine.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/profiles/cia402/DynaDriveController.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
using DynaDriveState = EtherCAT::Drives::Registers::DynaDrive::Status::StateOptions;
using DynaDriveCW = EtherCAT::Drives::Registers::DynaDrive::Controlword::Options;

namespace {

std::vector<uint8_t> le32(uint32_t v) {
    return {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
            static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
}

std::vector<uint8_t> le16(uint16_t v) {
    return {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
}

/// Plain CoE slave with a mailbox pair (no CiA402 state machine).
SlaveEmulator* addCoESlave(EmulatedBus& bus) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = 0x0DDD3052;
    cfg.mailbox_protocol = 0x04;
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
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

class DynaDriveControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addCoESlave(bus_);
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

    void setStatusword(uint32_t v) {
        slave_->addODEntry({0x6041, 0, 0x0007, 32, 0x03, le32(v),
                            "DynaDrive statusword"});
    }
    void addControlword() {
        slave_->addODEntry({0x6040, 0, 0x0006, 16, 0x03, le16(0),
                            "DynaDrive controlword"});
    }
    uint16_t controlword() const {
        const auto* e = slave_->odEntry(0x6040, 0);
        if (!e || e->data.size() < 2) return 0xFFFF;
        return static_cast<uint16_t>(e->data[0] | (e->data[1] << 8));
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

}  // namespace

// ============================================================================
// Pure decode/name tables
// ============================================================================

TEST(DynaDriveControllerDecodeTest, DecodeStateMapsAllIds) {
    struct Case { uint32_t sw; DynaDriveState want; };
    const Case cases[] = {
        {0, DynaDriveState::NA},
        {1, DynaDriveState::ColdStart},
        {2, DynaDriveState::WarmStart},
        {3, DynaDriveState::Configure},
        {4, DynaDriveState::Calibrate},
        {5, DynaDriveState::Standby},
        {6, DynaDriveState::MotorOp},
        {7, DynaDriveState::ControlOp},
        {8, DynaDriveState::Error},
        {9, DynaDriveState::Fatal},
        {10, DynaDriveState::MotorPreOp},
        {11, DynaDriveState::DeviceMissing},
        {12, DynaDriveState::Unknown},
        {0xFFFF, DynaDriveState::Unknown},
    };
    for (const auto& c : cases) {
        EXPECT_EQ(DynaDriveController::decodeState(c.sw), c.want) << c.sw;
    }
}

TEST(DynaDriveControllerDecodeTest, DecodeIgnoresUpperBits) {
    EXPECT_EQ(DynaDriveController::decodeState(0xFFFFFF07),
              DynaDriveState::ControlOp);
}

TEST(DynaDriveControllerDecodeTest, GetStateNameCoversEveryState) {
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::NA), "NA");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::ColdStart),
                 "ColdStart");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::WarmStart),
                 "WarmStart");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Configure),
                 "Configure");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Calibrate),
                 "Calibrate");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Standby),
                 "Standby");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::MotorOp),
                 "MotorOp");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::ControlOp),
                 "ControlOp");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Error),
                 "Error");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Fatal),
                 "Fatal");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::MotorPreOp),
                 "MotorPreOp");
    EXPECT_STREQ(
        DynaDriveController::getStateName(DynaDriveState::DeviceMissing),
        "DeviceMissing");
    EXPECT_STREQ(DynaDriveController::getStateName(DynaDriveState::Unknown),
                 "Unknown");
}

// ============================================================================
// SDO reads/writes
// ============================================================================

TEST_F(DynaDriveControllerTest, ReadStatuswordReads32BitObject) {
    setStatusword(7);
    DynaDriveController ctrl(master_, 0, 500);
    uint32_t sw = 0;
    ASSERT_TRUE(ctrl.readStatusword(sw));
    EXPECT_EQ(sw, 7u);
}

TEST_F(DynaDriveControllerTest, ReadStatuswordFailsWithoutObject) {
    DynaDriveController ctrl(master_, 0, 50);
    uint32_t sw = 0;
    EXPECT_FALSE(ctrl.readStatusword(sw));
}

TEST_F(DynaDriveControllerTest, SendControlwordWritesObject) {
    addControlword();
    DynaDriveController ctrl(master_, 0, 500);
    ASSERT_TRUE(ctrl.sendControlword(DynaDriveCW::MotorOpToControlOp));
    EXPECT_EQ(controlword(),
              static_cast<uint16_t>(DynaDriveCW::MotorOpToControlOp));
}

TEST_F(DynaDriveControllerTest, SendControlwordFailsWithoutObject) {
    DynaDriveController ctrl(master_, 0, 50);
    EXPECT_FALSE(ctrl.sendControlword(DynaDriveCW::MotorOpToControlOp));
}

TEST_F(DynaDriveControllerTest, IsControlOpFollowsStatusword) {
    DynaDriveController ctrl(master_, 0, 500);
    setStatusword(7);
    EXPECT_TRUE(ctrl.isControlOp());
    setStatusword(5);
    EXPECT_FALSE(ctrl.isControlOp());
}

// ============================================================================
// FSM transitions
// ============================================================================

TEST_F(DynaDriveControllerTest, EnableWhenAlreadyControlOp) {
    addControlword();
    setStatusword(7);
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_TRUE(ctrl.enable(500));
    EXPECT_EQ(controlword(), 0);  // no controlword written
}

TEST_F(DynaDriveControllerTest, EnableFailsInUnexpectedState) {
    addControlword();
    setStatusword(0);  // NA
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_FALSE(ctrl.enable(500));
}

TEST_F(DynaDriveControllerTest, EnableFailsOnFatalState) {
    addControlword();
    setStatusword(9);  // Fatal
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_FALSE(ctrl.enable(500));
}

TEST_F(DynaDriveControllerTest, EnableFailsWhenStatuswordUnreadable) {
    DynaDriveController ctrl(master_, 0, 50);
    EXPECT_FALSE(ctrl.enable(500));
}

TEST_F(DynaDriveControllerTest, EnableFromErrorTimesOutWithoutStateChange) {
    addControlword();
    setStatusword(8);  // Error — driver sends ClearErrorsToStandby
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_FALSE(ctrl.enable(550));  // never leaves Error
    EXPECT_EQ(controlword(),
              static_cast<uint16_t>(DynaDriveCW::ClearErrorsToStandby));
}

TEST_F(DynaDriveControllerTest, EnableFromStandbyTimesOutWithoutStateChange) {
    addControlword();
    setStatusword(5);  // Standby
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_FALSE(ctrl.enable(550));
    EXPECT_EQ(controlword(),
              static_cast<uint16_t>(DynaDriveCW::StandbyToMotorPreOp));
}

TEST_F(DynaDriveControllerTest, EnableFromMotorOpTimesOutWithoutStateChange) {
    addControlword();
    setStatusword(6);  // MotorOp
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_FALSE(ctrl.enable(550));
    EXPECT_EQ(controlword(),
              static_cast<uint16_t>(DynaDriveCW::MotorOpToControlOp));
}

TEST_F(DynaDriveControllerTest, DisableFromControlOpWritesControlword) {
    addControlword();
    setStatusword(7);
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_TRUE(ctrl.disable());
    EXPECT_EQ(controlword(),
              static_cast<uint16_t>(DynaDriveCW::ControlOpToStandby));
}

TEST_F(DynaDriveControllerTest, DisableWhenNotControlOpWritesNothing) {
    addControlword();
    setStatusword(5);
    DynaDriveController ctrl(master_, 0, 500);
    EXPECT_TRUE(ctrl.disable());
    EXPECT_EQ(controlword(), 0);
}

TEST_F(DynaDriveControllerTest, DisableWithUnreadableStatuswordIsNoOp) {
    DynaDriveController ctrl(master_, 0, 50);
    EXPECT_TRUE(ctrl.disable());
}
