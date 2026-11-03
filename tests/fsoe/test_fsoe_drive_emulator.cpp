/**
 * @file test_fsoe_drive_emulator.cpp
 * @brief Coverage for Synapticon FSoE drive emulator — init/reset frame
 *        seeding, LW1/LW2 response sizing, master-frame ingestion, and
 *        the CiA402 motion-side state machine.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "tether/fsoe/EmulatedChannel.hpp"
#include "tether/fsoe/FSoECRC.hpp"
#include "tether/profiles/cia402/MotionPdoTypes.hpp"
#include "tether/fsoe/Synapticon/FSoEDriveEmulator.hpp"

using namespace EtherCAT::Drives::Synapticon::SafeMotion;
namespace FSoE = ::FSoE;

namespace {

FSoEDriveEmulatorConfig baseConfig() {
    FSoEDriveEmulatorConfig cfg;
    cfg.index = 0;
    cfg.conn_id = 0x1234;
    cfg.safety_addr = 0x0400;
    cfg.slave_addr = 0x1000;
    cfg.resp_frame_len = 31;  // LW1
    cfg.watchdog_ms = 100;
    return cfg;
}

}  // namespace

TEST(FSoEDriveEmulatorTest, InitializeSeedsLw1ResetResponse) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());
    EXPECT_EQ(emu.state(), FSoE::ConnectionState::Reset);
    EXPECT_EQ(emu.responseLen(), 31u);
    EXPECT_TRUE(FSoE::isValidCommandByte(emu.response()[0]));
    EXPECT_FALSE(emu.emulator().controlMotionAllowed());
}

TEST(FSoEDriveEmulatorTest, InitializeSeedsLw2Response) {
    auto cfg = baseConfig();
    cfg.resp_frame_len = 35;  // LW2
    FSoEDriveEmulator emu(cfg);
    ASSERT_TRUE(emu.initialize());
    EXPECT_EQ(emu.responseLen(), 35u);
}

TEST(FSoEDriveEmulatorTest, ResetRestoresInitialResponse) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    // Feed a master frame so the emulator has state, then reset.
    const uint8_t frame[4] = {FSoE::Command::Session, 0x34, 0x12, 0x00};
    emu.process(frame, sizeof(frame), 1);
    emu.reset();

    EXPECT_EQ(emu.state(), FSoE::ConnectionState::Reset);
    EXPECT_EQ(emu.responseLen(), 31u);
    EXPECT_FALSE(emu.newFrame());
    EXPECT_EQ(emu.lastMasterCmd(), 0u);
}

TEST(FSoEDriveEmulatorTest, ProcessNullFrameOnlyAdvancesWatchdog) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());
    emu.process(nullptr, 0, 10);
    EXPECT_FALSE(emu.newFrame());
    EXPECT_EQ(emu.state(), FSoE::ConnectionState::Reset);
}

TEST(FSoEDriveEmulatorTest, ProcessGarbageFrameIsIgnoredByStateMachine) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    const uint8_t garbage[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    emu.process(garbage, sizeof(garbage), 10);
    EXPECT_TRUE(emu.newFrame());
    EXPECT_EQ(emu.lastMasterCmd(), 0x11);
    EXPECT_EQ(emu.state(), FSoE::ConnectionState::Reset);
}

TEST(FSoEDriveEmulatorTest, ProcessRepeatFrameDoesNotSetNewFrame) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    const uint8_t frame[4] = {FSoE::Command::Session, 0x34, 0x12, 0x00};
    emu.process(frame, sizeof(frame), 10);
    EXPECT_TRUE(emu.newFrame());
    emu.process(frame, sizeof(frame), 20);
    EXPECT_FALSE(emu.newFrame());
}

TEST(FSoEDriveEmulatorTest, ProcessMotionWithoutSafetyDataDropsPowerStage) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    CiA402::MotionCommand cmd{};
    cmd.controlword = 0x000F;  // enable operation
    cmd.modes_of_operation = 8;
    cmd.target_position = 1000;
    emu.processMotion(cmd, 0.001);

    // Safety layer not in Data state → power stage stays disabled.
    EXPECT_EQ(emu.motionFeedback().statusword, 0x0250);
    EXPECT_EQ(emu.motionFeedback().position_actual, 0);
    EXPECT_EQ(emu.motionFeedback().modes_of_operation_display, 8);
}

TEST(FSoEDriveEmulatorTest, ProcessMotionClampsNonPositiveDt) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    CiA402::MotionCommand cmd{};
    cmd.controlword = 0x0006;
    emu.processMotion(cmd, 0.0);
    EXPECT_EQ(emu.motionFeedback().statusword, 0x0250);
    emu.processMotion(cmd, -1.0);
    EXPECT_EQ(emu.motionFeedback().position_actual, 0);
}

TEST(FSoEDriveEmulatorTest, MotionStateMachineWalksControlwords) {
    FSoEDriveEmulator emu(baseConfig());
    ASSERT_TRUE(emu.initialize());

    CiA402::MotionCommand cmd{};
    cmd.controlword = 0x0006;  // Shutdown
    emu.processMotion(cmd, 0.001);
    EXPECT_EQ(emu.motionFeedback().statusword, 0x0250);  // gated by safety

    cmd.controlword = 0x0007;  // Switch on
    emu.processMotion(cmd, 0.001);
    EXPECT_EQ(emu.motionFeedback().statusword, 0x0250);

    cmd.controlword = 0x0000;  // Disable voltage
    emu.processMotion(cmd, 0.001);
    EXPECT_EQ(emu.motionFeedback().statusword, 0x0250);
}
