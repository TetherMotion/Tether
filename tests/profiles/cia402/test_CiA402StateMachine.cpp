/**
 * @file test_CiA402StateMachine.cpp
 * @brief Comprehensive tests for CiA402::StateMachine, ControlWord/StatusWord helpers
 */
#include <gtest/gtest.h>
#include <magic_enum/magic_enum.hpp>
#include "tether/profiles/cia402/CiA402StateMachine.hpp"
#include "tether/profiles/cia402/CiA402Config.hpp"

using namespace CiA402;

// ============================================================================
// ControlWord constexpr functions
// ============================================================================

TEST(ControlWordTest, Shutdown) {
    EXPECT_EQ(ControlWord::Shutdown(), 0x0006u);
}

TEST(ControlWordTest, SwitchOn) {
    EXPECT_EQ(ControlWord::SwitchOn(), 0x0007u);
}

TEST(ControlWordTest, SwitchOnEnable) {
    EXPECT_EQ(ControlWord::SwitchOnEnable(), 0x000Fu);
}

TEST(ControlWordTest, DisableVoltage) {
    EXPECT_EQ(ControlWord::DisableVoltage(), 0x0000u);
}

TEST(ControlWordTest, QuickStop) {
    EXPECT_EQ(ControlWord::QuickStop(), 0x0002u);
}

TEST(ControlWordTest, DisableOperation) {
    EXPECT_EQ(ControlWord::DisableOperation(), 0x0007u);
}

TEST(ControlWordTest, EnableOperation) {
    EXPECT_EQ(ControlWord::EnableOperation(), 0x000Fu);
}

TEST(ControlWordTest, FaultReset) {
    EXPECT_EQ(ControlWord::FaultReset(), 0x0080u);
}

TEST(ControlWordTest, TransitionMask) {
    EXPECT_EQ(ControlWord::TransitionMask(), 0x008Fu);
}

// ============================================================================
// StatusWord inline functions
// ============================================================================

TEST(StatusWordTest, ReadyToSwitchOn) {
    EXPECT_TRUE(StatusWord::isReadyToSwitchOn(0x0021));
    EXPECT_FALSE(StatusWord::isReadyToSwitchOn(0x0000));
}

TEST(StatusWordTest, SwitchedOn) {
    EXPECT_TRUE(StatusWord::isSwitchedOn(0x0023));
    EXPECT_FALSE(StatusWord::isSwitchedOn(0x0000));
}

TEST(StatusWordTest, OperationEnabled) {
    EXPECT_TRUE(StatusWord::isOperationEnabled(0x0027));
    EXPECT_FALSE(StatusWord::isOperationEnabled(0x0000));
}

TEST(StatusWordTest, Fault) {
    EXPECT_TRUE(StatusWord::isFault(0x0008));
    EXPECT_FALSE(StatusWord::isFault(0x0000));
}

TEST(StatusWordTest, FaultReactionActive) {
    EXPECT_TRUE(StatusWord::isFaultReactionActive(0x000F));
    EXPECT_FALSE(StatusWord::isFaultReactionActive(0x0000));
}

TEST(StatusWordTest, SwitchOnDisabled) {
    EXPECT_TRUE(StatusWord::isSwitchOnDisabled(0x0040));
    EXPECT_FALSE(StatusWord::isSwitchOnDisabled(0x0000));
}

TEST(StatusWordTest, QuickStopActive) {
    EXPECT_TRUE(StatusWord::isQuickStopActive(0x0007));
    EXPECT_FALSE(StatusWord::isQuickStopActive(0x0000));
}

TEST(StatusWordTest, TargetReached) {
    EXPECT_TRUE(StatusWord::isTargetReached(0x0400));
    EXPECT_FALSE(StatusWord::isTargetReached(0x0000));
}

TEST(StatusWordTest, Warning) {
    EXPECT_TRUE(StatusWord::hasWarning(0x0080));
    EXPECT_FALSE(StatusWord::hasWarning(0x0000));
}

TEST(StatusWordTest, LimitActive) {
    EXPECT_TRUE(StatusWord::isLimitActive(0x0800));
    EXPECT_FALSE(StatusWord::isLimitActive(0x0000));
}

TEST(StatusWordTest, HomingAttained) {
    EXPECT_TRUE(StatusWord::isHomingAttained(0x1000));
    EXPECT_FALSE(StatusWord::isHomingAttained(0x0000));
}

TEST(StatusWordTest, HomingError) {
    EXPECT_TRUE(StatusWord::hasHomingError(0x2000));
    EXPECT_FALSE(StatusWord::hasHomingError(0x0000));
}

// ============================================================================
// TransitionResult enum
// ============================================================================

TEST(TransitionResultTest, AllDistinct) {
    EXPECT_NE(static_cast<int>(TransitionResult::Success),
              static_cast<int>(TransitionResult::Pending));
    EXPECT_NE(static_cast<int>(TransitionResult::Pending),
              static_cast<int>(TransitionResult::InvalidTransition));
    EXPECT_NE(static_cast<int>(TransitionResult::Timeout),
              static_cast<int>(TransitionResult::FaultOccurred));
}

// ============================================================================
// StateMachine fixture
// ============================================================================

class CiA402SMTest : public ::testing::Test {
protected:
    void SetUp() override {
        sm_ = std::make_unique<StateMachine>();
        sm_->setCallbacks(
            [this]() -> uint16_t { return statusWord_; },
            [this](uint16_t cw) { controlWord_ = cw; }
        );
    }

    void setStatus(uint16_t sw) { statusWord_ = sw; }

    std::unique_ptr<StateMachine> sm_;
    uint16_t statusWord_ = 0x0040; // SwitchOnDisabled by default
    uint16_t controlWord_ = 0;
};

TEST_F(CiA402SMTest, InitialState) {
    auto state = sm_->update();
    (void)state;
    (void)sm_->getCurrentState();
    (void)sm_->getPreviousState();
}

TEST_F(CiA402SMTest, GetStatusWord) {
    sm_->update();
    EXPECT_EQ(sm_->getStatusWord(), statusWord_);
}

TEST_F(CiA402SMTest, GetControlWord) {
    (void)sm_->getControlWord();
}

TEST_F(CiA402SMTest, IsSwitchOnDisabled) {
    setStatus(0x0040); // SwitchOnDisabled
    sm_->update();
    EXPECT_FALSE(sm_->isEnabled());
    EXPECT_FALSE(sm_->isFaulted());
}

TEST_F(CiA402SMTest, UpdateReadsStatusword) {
    setStatus(0x0040);
    sm_->update();
    EXPECT_EQ(sm_->getStatusWord(), 0x0040u);
}

TEST_F(CiA402SMTest, RequestStateTransition) {
    setStatus(0x0040); // SwitchOnDisabled
    sm_->update();
    auto result = sm_->requestState(State::ReadyToSwitchOn, 10);
    (void)result;
}

TEST_F(CiA402SMTest, ExecuteTransition) {
    setStatus(0x0040);
    sm_->update();
    bool ok = sm_->executeTransition(0x0006); // Shutdown
    (void)ok;
}

TEST_F(CiA402SMTest, QuickStop) {
    setStatus(0x0027); // OperationEnabled
    sm_->update();
    bool ok = sm_->quickStop();
    (void)ok;
}

TEST_F(CiA402SMTest, ResetFault) {
    setStatus(0x0008); // Fault
    sm_->update();
    bool ok = sm_->resetFault();
    (void)ok;
}

TEST_F(CiA402SMTest, SetHalt) {
    sm_->setHalt(true);
    sm_->setHalt(false);
}

TEST_F(CiA402SMTest, OperatingMode) {
    bool ok = sm_->setOperatingMode(OperatingMode::CyclicSyncPosition);
    (void)ok;
    (void)sm_->getOperatingMode();
    (void)sm_->getDisplayedMode();
}

TEST_F(CiA402SMTest, ModeCallback) {
    OperatingMode lastMode = OperatingMode::NoMode;
    sm_->setModeCallback([&](OperatingMode m) { lastMode = m; });
    sm_->setOperatingMode(OperatingMode::ProfileVelocity);
}

TEST_F(CiA402SMTest, IsTargetReached) {
    setStatus(0x0400);
    sm_->update();
    EXPECT_TRUE(sm_->isTargetReached());
}

TEST_F(CiA402SMTest, IsNotTargetReached) {
    setStatus(0x0000);
    sm_->update();
    EXPECT_FALSE(sm_->isTargetReached());
}

TEST_F(CiA402SMTest, HasWarning) {
    setStatus(0x0080);
    sm_->update();
    EXPECT_TRUE(sm_->hasWarning());
}

TEST_F(CiA402SMTest, IsLimitActive) {
    setStatus(0x0800);
    sm_->update();
    EXPECT_TRUE(sm_->isLimitActive());
}

TEST_F(CiA402SMTest, IsInMotion) {
    (void)sm_->isInMotion();
}

TEST_F(CiA402SMTest, FaultedState) {
    setStatus(0x0008); // Fault
    sm_->update();
    EXPECT_TRUE(sm_->isFaulted());
    EXPECT_FALSE(sm_->isEnabled());
}

TEST_F(CiA402SMTest, EnabledState) {
    setStatus(0x0027); // OperationEnabled
    sm_->update();
    EXPECT_TRUE(sm_->isEnabled());
    EXPECT_FALSE(sm_->isFaulted());
}

// ============================================================================
// Homing-related
// ============================================================================

TEST_F(CiA402SMTest, HomingAttained) {
    setStatus(0x1000);
    sm_->update();
    EXPECT_TRUE(sm_->isHomingAttained());
}

TEST_F(CiA402SMTest, HomingError) {
    setStatus(0x2000);
    sm_->update();
    EXPECT_TRUE(sm_->hasHomingError());
}

TEST_F(CiA402SMTest, StartHoming) {
    bool ok = sm_->startHoming();
    (void)ok;
}

// ============================================================================
// State transition sequences
// ============================================================================

TEST_F(CiA402SMTest, FullEnableSequence) {
    // Start: SwitchOnDisabled
    setStatus(0x0040);
    sm_->update();

    // Request ReadyToSwitchOn
    sm_->requestState(State::ReadyToSwitchOn, 1);
    setStatus(0x0021); // ReadyToSwitchOn
    sm_->update();

    // Request SwitchedOn
    sm_->requestState(State::SwitchedOn, 1);
    setStatus(0x0023); // SwitchedOn
    sm_->update();

    // Request OperationEnabled
    sm_->requestState(State::OperationEnabled, 1);
    setStatus(0x0027); // OperationEnabled
    sm_->update();

    EXPECT_TRUE(sm_->isEnabled());
}

TEST_F(CiA402SMTest, FaultAndReset) {
    setStatus(0x0008); // Fault
    sm_->update();
    EXPECT_TRUE(sm_->isFaulted());

    sm_->resetFault();
    setStatus(0x0040); // SwitchOnDisabled after reset
    sm_->update();
    EXPECT_FALSE(sm_->isFaulted());
}

// ============================================================================
// CiA402Config helpers
// ============================================================================

TEST(CiA402ConfigTest, StateToString) {
    auto s = magic_enum::enum_name(State::OperationEnabled);
    EXPECT_FALSE(s.empty());
    auto s2 = magic_enum::enum_name(State::Fault);
    EXPECT_FALSE(s2.empty());
}

TEST(CiA402ConfigTest, ModeToString) {
    auto m = magic_enum::enum_name(OperatingMode::CyclicSyncPosition);
    EXPECT_FALSE(m.empty());
    auto m2 = magic_enum::enum_name(OperatingMode::Homing);
    EXPECT_FALSE(m2.empty());
}

TEST(CiA402ConfigTest, ControlWordBits) {
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::SwitchOn), 0x0001u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::EnableVoltage), 0x0002u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::QuickStop), 0x0004u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::EnableOperation), 0x0008u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::FaultReset), 0x0080u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWordBit::Halt), 0x0100u);
}

TEST(CiA402ConfigTest, StatusWordBits) {
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::ReadyToSwitchOn), 0x0001u);
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::SwitchedOn), 0x0002u);
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::OperationEnabled), 0x0004u);
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::Fault), 0x0008u);
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::Warning), 0x0080u);
    EXPECT_EQ(static_cast<uint16_t>(StatusWordBit::TargetReached), 0x0400u);
}

TEST(CiA402ConfigTest, OperatingModes) {
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::NoMode), 0);
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::ProfilePosition), 1);
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::Homing), 6);
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::CyclicSyncPosition), 8);
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::CyclicSyncVelocity), 9);
    EXPECT_EQ(static_cast<int8_t>(OperatingMode::CyclicSyncTorque), 10);
}

TEST(CiA402ConfigTest, ErrorCodes) {
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::None), 0x0000u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::GenericError), 0x1000u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::OverCurrent), 0x2310u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::OverVoltage), 0x3210u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::UnderVoltage), 0x3220u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::EncoderError), 0x5110u);
    EXPECT_EQ(static_cast<uint32_t>(ErrorCode::FollowingError), 0x8611u);
}
