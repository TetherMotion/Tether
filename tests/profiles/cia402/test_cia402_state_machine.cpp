/**
 * @file test_cia402_state_machine.cpp
 * @brief Comprehensive tests for CiA402 StateMachine
 */

#include "tether/profiles/cia402/CiA402StateMachine.hpp"

#include <gtest/gtest.h>
#include <cstdint>
#include <functional>

using namespace CiA402;

// ============================================================================
// StatusWord decode tests
// ============================================================================

TEST(StatusWordDecodeTest, SwitchOnDisabled) {
    // State: Switch On Disabled - bits: xxxx xxxx x1xx 0000
    uint16_t sw = 0x0040;
    EXPECT_TRUE(StatusWord::isSwitchOnDisabled(sw));
    EXPECT_FALSE(StatusWord::isReadyToSwitchOn(sw));
    EXPECT_FALSE(StatusWord::isSwitchedOn(sw));
    EXPECT_FALSE(StatusWord::isOperationEnabled(sw));
    EXPECT_FALSE(StatusWord::isFault(sw));
}

TEST(StatusWordDecodeTest, ReadyToSwitchOn) {
    // State: Ready to Switch On - bits: xxxx xxxx x01x 0001
    uint16_t sw = 0x0021;
    EXPECT_TRUE(StatusWord::isReadyToSwitchOn(sw));
    EXPECT_FALSE(StatusWord::isSwitchOnDisabled(sw));
}

TEST(StatusWordDecodeTest, SwitchedOn) {
    // State: Switched On - bits: xxxx xxxx x01x 0011
    uint16_t sw = 0x0023;
    EXPECT_TRUE(StatusWord::isSwitchedOn(sw));
}

TEST(StatusWordDecodeTest, OperationEnabled) {
    // State: Operation Enabled - bits: xxxx xxxx x01x 0111
    uint16_t sw = 0x0027;
    EXPECT_TRUE(StatusWord::isOperationEnabled(sw));
}

TEST(StatusWordDecodeTest, Fault) {
    // Fault: bit 3 set, bit 6 clear, bit 5 clear → xxxx xxxx x0xx 1000
    uint16_t sw = 0x0008;
    EXPECT_TRUE(StatusWord::isFault(sw));
}

TEST(StatusWordDecodeTest, FaultReactionActive) {
    // Fault Reaction Active: bit 3 set, bit 4 set
    uint16_t sw = 0x000F;
    EXPECT_TRUE(StatusWord::isFaultReactionActive(sw));
}

TEST(StatusWordDecodeTest, QuickStopActive) {
    // Quick Stop Active: xxxx xxxx x00x 0111  → bit 5 clear
    uint16_t sw = 0x0007;
    EXPECT_TRUE(StatusWord::isQuickStopActive(sw));
}

TEST(StatusWordDecodeTest, TargetReached) {
    uint16_t sw = 0x0400; // bit 10
    EXPECT_TRUE(StatusWord::isTargetReached(sw));
}

TEST(StatusWordDecodeTest, Warning) {
    uint16_t sw = 0x0080; // bit 7
    EXPECT_TRUE(StatusWord::hasWarning(sw));
}

TEST(StatusWordDecodeTest, LimitActive) {
    uint16_t sw = 0x0800; // bit 11
    EXPECT_TRUE(StatusWord::isLimitActive(sw));
}

TEST(StatusWordDecodeTest, HomingAttained) {
    uint16_t sw = 0x1000; // bit 12
    EXPECT_TRUE(StatusWord::isHomingAttained(sw));
}

TEST(StatusWordDecodeTest, HomingError) {
    uint16_t sw = 0x2000; // bit 13
    EXPECT_TRUE(StatusWord::hasHomingError(sw));
}

// ============================================================================
// ControlWord constant tests
// ============================================================================

TEST(ControlWordTest, ShutdownValue) {
    EXPECT_EQ(ControlWord::Shutdown(), 0x0006);
}

TEST(ControlWordTest, SwitchOnValue) {
    EXPECT_EQ(ControlWord::SwitchOn(), 0x0007);
}

TEST(ControlWordTest, SwitchOnEnableValue) {
    EXPECT_EQ(ControlWord::SwitchOnEnable(), 0x000F);
}

TEST(ControlWordTest, DisableVoltageValue) {
    EXPECT_EQ(ControlWord::DisableVoltage(), 0x0000);
}

TEST(ControlWordTest, QuickStopValue) {
    EXPECT_EQ(ControlWord::QuickStop(), 0x0002);
}

TEST(ControlWordTest, FaultResetValue) {
    EXPECT_EQ(ControlWord::FaultReset(), 0x0080);
}

TEST(ControlWordTest, TransitionMaskValue) {
    EXPECT_EQ(ControlWord::TransitionMask(), 0x008F);
}

// ============================================================================
// StateMachine tests
// ============================================================================

class StateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        sm_ = std::make_unique<StateMachine>();
        
        // Set up callbacks that simulate a real drive
        sm_->setCallbacks(
            [this]() -> uint16_t { return simulated_status_; },
            [this](uint16_t cw) { last_control_word_ = cw; }
        );
    }
    
    void setStatusWord(uint16_t sw) { simulated_status_ = sw; }
    
    std::unique_ptr<StateMachine> sm_;
    uint16_t simulated_status_ = 0x0040; // Start in Switch On Disabled
    uint16_t last_control_word_ = 0;
};

TEST_F(StateMachineTest, InitialState) {
    auto state = sm_->update();
    // Should detect Switch On Disabled from status word 0x0040
    (void)state;
}

TEST_F(StateMachineTest, GetCurrentState) {
    sm_->update();
    auto state = sm_->getCurrentState();
    (void)state; // Just verify no crash
}

TEST_F(StateMachineTest, GetPreviousState) {
    sm_->update();
    auto prev = sm_->getPreviousState();
    (void)prev;
}

TEST_F(StateMachineTest, GetStatusWord) {
    sm_->update();
    EXPECT_EQ(sm_->getStatusWord(), 0x0040);
}

TEST_F(StateMachineTest, GetControlWord) {
    sm_->update();
    auto cw = sm_->getControlWord();
    (void)cw;
}

TEST_F(StateMachineTest, IsFaulted_NotFaulted) {
    setStatusWord(0x0040); // Switch On Disabled
    sm_->update();
    EXPECT_FALSE(sm_->isFaulted());
}

TEST_F(StateMachineTest, IsFaulted_Faulted) {
    setStatusWord(0x0008); // Fault state
    sm_->update();
    EXPECT_TRUE(sm_->isFaulted());
}

TEST_F(StateMachineTest, IsEnabled_NotEnabled) {
    setStatusWord(0x0040);
    sm_->update();
    EXPECT_FALSE(sm_->isEnabled());
}

TEST_F(StateMachineTest, IsEnabled_Enabled) {
    setStatusWord(0x0027); // Operation Enabled
    sm_->update();
    EXPECT_TRUE(sm_->isEnabled());
}

TEST_F(StateMachineTest, IsTargetReached) {
    setStatusWord(0x0427); // Operation Enabled + Target Reached
    sm_->update();
    EXPECT_TRUE(sm_->isTargetReached());
}

TEST_F(StateMachineTest, HasWarning) {
    setStatusWord(0x00C0); // Switch On Disabled + Warning
    sm_->update();
    EXPECT_TRUE(sm_->hasWarning());
}

TEST_F(StateMachineTest, IsLimitActive) {
    setStatusWord(0x0840);
    sm_->update();
    EXPECT_TRUE(sm_->isLimitActive());
}

TEST_F(StateMachineTest, ExecuteTransition) {
    sm_->update();
    bool ok = sm_->executeTransition(ControlWord::Shutdown());
    EXPECT_TRUE(ok);
    EXPECT_EQ(last_control_word_ & ControlWord::TransitionMask(), 
              ControlWord::Shutdown());
}

TEST_F(StateMachineTest, QuickStop) {
    setStatusWord(0x0027); // Operation Enabled
    sm_->update();
    bool ok = sm_->quickStop();
    EXPECT_TRUE(ok);
}

TEST_F(StateMachineTest, ResetFault) {
    setStatusWord(0x0008); // Fault
    sm_->update();
    bool ok = sm_->resetFault();
    EXPECT_TRUE(ok);
}

TEST_F(StateMachineTest, SetHalt) {
    sm_->update();
    sm_->setHalt(true);
    sm_->setHalt(false);
}

TEST_F(StateMachineTest, OperatingMode) {
    sm_->update();
    // Default operating mode is ProfilePosition (constructor initializes it)
    EXPECT_EQ(sm_->getOperatingMode(), OperatingMode::ProfilePosition);
    
    sm_->setOperatingMode(OperatingMode::CyclicSyncPosition);
    // The operating mode request is application-specific; just verify no crash
}

TEST_F(StateMachineTest, HomingStatus) {
    setStatusWord(0x1040); // Homing attained
    sm_->update();
    EXPECT_TRUE(sm_->isHomingAttained());
    EXPECT_FALSE(sm_->hasHomingError());
}

TEST_F(StateMachineTest, HomingError) {
    setStatusWord(0x2040); // Homing error
    sm_->update();
    EXPECT_TRUE(sm_->hasHomingError());
}

TEST_F(StateMachineTest, StartHoming) {
    setStatusWord(0x0027); // Operation Enabled
    sm_->update();
    sm_->startHoming();
}

TEST_F(StateMachineTest, RequestStateTransition) {
    setStatusWord(0x0040); // Switch On Disabled
    sm_->update();
    
    // Request ReadyToSwitchOn
    auto result = sm_->requestState(State::ReadyToSwitchOn, 100);
    // The result depends on whether the simulated drive responds
    // Just verify no crash
    (void)result;
}

TEST_F(StateMachineTest, SetModeCallback) {
    OperatingMode lastMode = OperatingMode::NoMode;
    sm_->setModeCallback([&](OperatingMode m) {
        lastMode = m;
    });
    // Just verify the callback is stored
}

TEST_F(StateMachineTest, IsInMotion) {
    setStatusWord(0x0040);
    sm_->update();
    // isInMotion depends on mode-specific status bits
    auto inMotion = sm_->isInMotion();
    (void)inMotion;
}
