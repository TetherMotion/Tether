/**
 * @file test_CiA402StateMachine_coverage.cpp
 * @brief Deep coverage tests for CiA402 StateMachine - exercises all branches
 *        in requestState, calculateControlWord, getIntermediateState, etc.
 */

#include "tether/profiles/cia402/CiA402StateMachine.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <functional>

using namespace CiA402;

// ============================================================================
// Helper: Drive simulator that responds to control words
// ============================================================================

class DriveSimulator {
public:
    DriveSimulator() : statusWord_(0x0040) {} // SwitchOnDisabled

    uint16_t readStatus() {
        // Only respond to control words after one has actually been written
        if (controlWritten_) {
            applyControlResponse();
        }
        return statusWord_;
    }

    void writeControl(uint16_t cw) {
        controlWord_ = cw;
        controlWriteCount_++;
        controlWritten_ = true;
    }

    void setStatusWord(uint16_t sw) { statusWord_ = sw; }
    uint16_t getLastControl() const { return controlWord_; }
    int getControlWriteCount() const { return controlWriteCount_; }

    // Inject fault on next update
    void injectFault() { injectFault_ = true; }
    // Freeze status (don't respond to control)
    void freeze() { frozen_ = true; }
    void unfreeze() { frozen_ = false; }

private:
    void applyControlResponse() {
        if (frozen_) return;
        if (injectFault_) {
            statusWord_ = 0x0008; // Fault
            injectFault_ = false;
            return;
        }
        uint16_t masked = controlWord_ & 0x008F;
        
        // Simulate CiA402 state machine response
        if (masked == ControlWord::Shutdown()) {
            statusWord_ = 0x0021; // ReadyToSwitchOn
        } else if (masked == ControlWord::SwitchOn()) {
            statusWord_ = 0x0023; // SwitchedOn
        } else if (masked == ControlWord::EnableOperation() || 
                   masked == ControlWord::SwitchOnEnable()) {
            statusWord_ = 0x0027; // OperationEnabled
        } else if (masked == ControlWord::DisableVoltage()) {
            statusWord_ = 0x0040; // SwitchOnDisabled
        } else if (masked == ControlWord::QuickStop()) {
            statusWord_ = 0x0007; // QuickStopActive
        } else if (masked == ControlWord::FaultReset()) {
            statusWord_ = 0x0040; // SwitchOnDisabled after fault reset
        } else if (masked == ControlWord::DisableOperation()) {
            statusWord_ = 0x0023; // SwitchedOn
        }
    }

    uint16_t statusWord_;
    uint16_t controlWord_ = 0;
    int controlWriteCount_ = 0;
    bool injectFault_ = false;
    bool frozen_ = false;
    bool controlWritten_ = false;
};

// ============================================================================
// Fixture: StateMachine with DriveSimulator
// ============================================================================

class SMCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        sm_.setCallbacks(
            [this]() -> uint16_t { return sim_.readStatus(); },
            [this](uint16_t cw) { sim_.writeControl(cw); }
        );
    }

    StateMachine sm_;
    DriveSimulator sim_;
};

// ============================================================================
// requestState: Multi-step transitions
// ============================================================================

TEST_F(SMCovTest, RequestState_AlreadyAtTarget) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    sm_.update();
    auto r = sm_.requestState(State::SwitchOnDisabled);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_NoCallbacks) {
    StateMachine bare;
    auto r = bare.requestState(State::OperationEnabled);
    EXPECT_EQ(r, TransitionResult::InvalidTransition);
}

TEST_F(SMCovTest, RequestState_SOD_to_RTSO) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    auto r = sm_.requestState(State::ReadyToSwitchOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
    EXPECT_EQ(sm_.getCurrentState(), State::ReadyToSwitchOn);
}

TEST_F(SMCovTest, RequestState_SOD_to_SwitchedOn) {
    sim_.setStatusWord(0x0040);
    auto r = sm_.requestState(State::SwitchedOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
    EXPECT_EQ(sm_.getCurrentState(), State::SwitchedOn);
}

TEST_F(SMCovTest, RequestState_SOD_to_OperationEnabled) {
    sim_.setStatusWord(0x0040);
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
    EXPECT_EQ(sm_.getCurrentState(), State::OperationEnabled);
}

TEST_F(SMCovTest, RequestState_OpEnabled_to_SOD) {
    sim_.setStatusWord(0x0027); // OperationEnabled
    sm_.update();
    auto r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_OpEnabled_to_RTSO) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    auto r = sm_.requestState(State::ReadyToSwitchOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_OpEnabled_to_SwitchedOn) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    auto r = sm_.requestState(State::SwitchedOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_OpEnabled_to_QuickStop) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    auto r = sm_.requestState(State::QuickStopActive, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_RTSO_to_SOD) {
    sim_.setStatusWord(0x0021); // ReadyToSwitchOn
    sm_.update();
    auto r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_SwitchedOn_to_SOD) {
    sim_.setStatusWord(0x0023); // SwitchedOn
    sm_.update();
    auto r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_SwitchedOn_to_RTSO) {
    sim_.setStatusWord(0x0023);
    sm_.update();
    auto r = sm_.requestState(State::ReadyToSwitchOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_SwitchedOn_to_OpEnabled) {
    sim_.setStatusWord(0x0023);
    sm_.update();
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_QuickStop_to_SOD) {
    sim_.setStatusWord(0x0007); // QuickStopActive
    sm_.update();
    auto r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

TEST_F(SMCovTest, RequestState_Fault_to_OpEnabled) {
    sim_.setStatusWord(0x0008); // Fault
    sm_.update();
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::FaultOccurred);
}

TEST_F(SMCovTest, RequestState_Fault_to_SOD) {
    sim_.setStatusWord(0x0008); // Fault
    sm_.update();
    // From Fault, requestState loop detects fault and returns FaultOccurred.
    // Must use resetFault() first, which is tested separately.
    auto r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::FaultOccurred);
}

TEST_F(SMCovTest, RequestState_Timeout) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    sim_.freeze(); // Don't respond to control
    auto r = sm_.requestState(State::OperationEnabled, 10);
    EXPECT_EQ(r, TransitionResult::Timeout);
}

TEST_F(SMCovTest, RequestState_FaultDuringTransition) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    sim_.injectFault();
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::FaultOccurred);
}

TEST_F(SMCovTest, RequestState_NoWait) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    sm_.update();
    // timeoutMs=0 means single attempt mode
    auto r = sm_.requestState(State::OperationEnabled, 0);
    // Single attempt, state probably changed to first intermediate
    // Result should be Pending since we won't have reached target
    EXPECT_NE(r, TransitionResult::InvalidTransition);
}

// ============================================================================
// requestState: FaultReactionActive
// ============================================================================

TEST_F(SMCovTest, RequestState_FaultReactionActive) {
    sim_.setStatusWord(0x000F); // FaultReactionActive
    sm_.update();
    auto r = sm_.requestState(State::OperationEnabled, 100);
    EXPECT_EQ(r, TransitionResult::FaultOccurred);
}

// ============================================================================
// requestState from RTSO to OperationEnabled
// ============================================================================

TEST_F(SMCovTest, RequestState_RTSO_to_OpEnabled) {
    sim_.setStatusWord(0x0021); // ReadyToSwitchOn
    sm_.update();
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

// ============================================================================
// executeTransition
// ============================================================================

TEST_F(SMCovTest, ExecuteTransition_NoCallback) {
    StateMachine bare;
    EXPECT_FALSE(bare.executeTransition(ControlWord::Shutdown()));
}

TEST_F(SMCovTest, ExecuteTransition_Shutdown) {
    sm_.update();
    EXPECT_TRUE(sm_.executeTransition(ControlWord::Shutdown()));
    EXPECT_EQ(sim_.getLastControl() & ControlWord::TransitionMask(),
              ControlWord::Shutdown());
}

TEST_F(SMCovTest, ExecuteTransition_SwitchOn) {
    EXPECT_TRUE(sm_.executeTransition(ControlWord::SwitchOn()));
}

TEST_F(SMCovTest, ExecuteTransition_EnableOperation) {
    EXPECT_TRUE(sm_.executeTransition(ControlWord::EnableOperation()));
}

// ============================================================================
// quickStop
// ============================================================================

TEST_F(SMCovTest, QuickStop_NoCallback) {
    StateMachine bare;
    EXPECT_FALSE(bare.quickStop());
}

TEST_F(SMCovTest, QuickStop_FromOpEnabled) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    EXPECT_TRUE(sm_.quickStop());
}

// ============================================================================
// resetFault
// ============================================================================

TEST_F(SMCovTest, ResetFault_NoCallback) {
    StateMachine bare;
    EXPECT_FALSE(bare.resetFault());
}

TEST_F(SMCovTest, ResetFault_NotInFault) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    sm_.update();
    EXPECT_FALSE(sm_.resetFault());
}

TEST_F(SMCovTest, ResetFault_InFault) {
    sim_.setStatusWord(0x0008); // Fault
    sm_.update();
    EXPECT_TRUE(sm_.resetFault());
}

// ============================================================================
// setHalt
// ============================================================================

TEST_F(SMCovTest, SetHalt_True) {
    sm_.update();
    sm_.setHalt(true);
    uint16_t cw = sim_.getLastControl();
    EXPECT_NE(cw & static_cast<uint16_t>(ControlWordBit::Halt), 0);
}

TEST_F(SMCovTest, SetHalt_False) {
    sm_.update();
    sm_.setHalt(true);
    sm_.setHalt(false);
    uint16_t cw = sim_.getLastControl();
    EXPECT_EQ(cw & static_cast<uint16_t>(ControlWordBit::Halt), 0);
}

TEST_F(SMCovTest, SetHalt_NoCallback) {
    StateMachine bare;
    bare.setHalt(true);
    bare.setHalt(false);
}

TEST_F(SMCovTest, SetHalt_DuringTransition) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    sm_.setHalt(true);
    // Now request transition with halt active
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

// ============================================================================
// setOperatingMode
// ============================================================================

TEST_F(SMCovTest, SetOperatingMode_CSP) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::CyclicSyncPosition));
    EXPECT_EQ(sm_.getOperatingMode(), OperatingMode::CyclicSyncPosition);
}

TEST_F(SMCovTest, SetOperatingMode_CSV) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::CyclicSyncVelocity));
}

TEST_F(SMCovTest, SetOperatingMode_CST) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::CyclicSyncTorque));
}

TEST_F(SMCovTest, SetOperatingMode_Homing) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::Homing));
}

TEST_F(SMCovTest, SetOperatingMode_PP) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::ProfilePosition));
}

TEST_F(SMCovTest, SetOperatingMode_PV) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::ProfileVelocity));
}

TEST_F(SMCovTest, SetOperatingMode_PT) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::ProfileTorque));
}

TEST_F(SMCovTest, SetOperatingMode_IP) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::InterpolatedPosition));
}

TEST_F(SMCovTest, SetOperatingMode_Velocity) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::Velocity));
}

TEST_F(SMCovTest, SetOperatingMode_NoMode) {
    EXPECT_TRUE(sm_.setOperatingMode(OperatingMode::NoMode));
}

TEST_F(SMCovTest, SetOperatingMode_WithModeCallback) {
    OperatingMode received = OperatingMode::NoMode;
    sm_.setModeCallback([&](OperatingMode m) { received = m; });
    sm_.setOperatingMode(OperatingMode::CyclicSyncPosition);
    EXPECT_EQ(received, OperatingMode::CyclicSyncPosition);
}

// ============================================================================
// Status queries
// ============================================================================

TEST_F(SMCovTest, IsFaulted_Fault) {
    sim_.setStatusWord(0x0008);
    sm_.update();
    EXPECT_TRUE(sm_.isFaulted());
}

TEST_F(SMCovTest, IsFaulted_FRA) {
    sim_.setStatusWord(0x000F);
    sm_.update();
    EXPECT_TRUE(sm_.isFaulted());
}

TEST_F(SMCovTest, IsFaulted_NotFault) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    EXPECT_FALSE(sm_.isFaulted());
}

TEST_F(SMCovTest, IsEnabled_True) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    EXPECT_TRUE(sm_.isEnabled());
}

TEST_F(SMCovTest, IsEnabled_False) {
    sim_.setStatusWord(0x0023);
    sm_.update();
    EXPECT_FALSE(sm_.isEnabled());
}

TEST_F(SMCovTest, IsTargetReached_True) {
    sim_.setStatusWord(0x0427); // OpEnabled + TargetReached (bit 10)
    sm_.update();
    EXPECT_TRUE(sm_.isTargetReached());
}

TEST_F(SMCovTest, IsTargetReached_False) {
    sim_.setStatusWord(0x0027); // OpEnabled, no TargetReached
    sm_.update();
    EXPECT_FALSE(sm_.isTargetReached());
}

TEST_F(SMCovTest, IsInMotion_ActiveNoTarget) {
    sim_.setStatusWord(0x0027); // OpEnabled, no TargetReached
    sm_.update();
    EXPECT_TRUE(sm_.isInMotion());
}

TEST_F(SMCovTest, IsInMotion_TargetReached) {
    sim_.setStatusWord(0x0427); // OpEnabled + TargetReached
    sm_.update();
    EXPECT_FALSE(sm_.isInMotion());
}

TEST_F(SMCovTest, IsInMotion_NotEnabled) {
    sim_.setStatusWord(0x0023); // SwitchedOn
    sm_.update();
    EXPECT_FALSE(sm_.isInMotion());
}

TEST_F(SMCovTest, HasWarning_True) {
    sim_.setStatusWord(0x00C0); // SOD + Warning (bit 7)
    sm_.update();
    EXPECT_TRUE(sm_.hasWarning());
}

TEST_F(SMCovTest, HasWarning_False) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    EXPECT_FALSE(sm_.hasWarning());
}

TEST_F(SMCovTest, IsLimitActive_True) {
    sim_.setStatusWord(0x0840); // SOD + Limit (bit 11)
    sm_.update();
    EXPECT_TRUE(sm_.isLimitActive());
}

TEST_F(SMCovTest, IsLimitActive_False) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    EXPECT_FALSE(sm_.isLimitActive());
}

// ============================================================================
// Homing
// ============================================================================

TEST_F(SMCovTest, HomingAttained_True) {
    sim_.setStatusWord(0x1027); // OpEnabled + HomingAttained (bit 12)
    sm_.update();
    EXPECT_TRUE(sm_.isHomingAttained());
}

TEST_F(SMCovTest, HomingAttained_False) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    EXPECT_FALSE(sm_.isHomingAttained());
}

TEST_F(SMCovTest, HomingError_True) {
    sim_.setStatusWord(0x2027); // OpEnabled + HomingError (bit 13)
    sm_.update();
    EXPECT_TRUE(sm_.hasHomingError());
}

TEST_F(SMCovTest, HomingError_False) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    EXPECT_FALSE(sm_.hasHomingError());
}

TEST_F(SMCovTest, StartHoming_InHomingMode) {
    sm_.setOperatingMode(OperatingMode::Homing);
    EXPECT_TRUE(sm_.startHoming());
}

TEST_F(SMCovTest, StartHoming_NotInHomingMode) {
    sm_.setOperatingMode(OperatingMode::ProfilePosition);
    EXPECT_FALSE(sm_.startHoming());
}

TEST_F(SMCovTest, StartHoming_NoCallback) {
    StateMachine bare;
    bare.setOperatingMode(OperatingMode::Homing);
    EXPECT_FALSE(bare.startHoming());
}

// ============================================================================
// update: no callback
// ============================================================================

TEST_F(SMCovTest, Update_NoReadCallback) {
    StateMachine bare;
    auto state = bare.update();
    EXPECT_EQ(state, State::NotReadyToSwitchOn);
}

// ============================================================================
// update: state change tracking
// ============================================================================

TEST_F(SMCovTest, Update_StateChanged) {
    sim_.setStatusWord(0x0040);
    sm_.update(); // SwitchOnDisabled
    EXPECT_EQ(sm_.getCurrentState(), State::SwitchOnDisabled);
    
    sim_.setStatusWord(0x0021);
    sm_.update(); // ReadyToSwitchOn
    EXPECT_EQ(sm_.getCurrentState(), State::ReadyToSwitchOn);
    EXPECT_EQ(sm_.getPreviousState(), State::SwitchOnDisabled);
}

TEST_F(SMCovTest, Update_NoChange) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    auto prev = sm_.getPreviousState();
    sm_.update(); // Same status, no state change
    EXPECT_EQ(sm_.getPreviousState(), prev);
}

// ============================================================================
// decodeState: All state encoding checks
// ============================================================================

TEST_F(SMCovTest, DecodeState_NotReadyToSwitchOn) {
    sim_.setStatusWord(0x0000);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::NotReadyToSwitchOn);
}

TEST_F(SMCovTest, DecodeState_SwitchOnDisabled) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::SwitchOnDisabled);
}

TEST_F(SMCovTest, DecodeState_ReadyToSwitchOn) {
    sim_.setStatusWord(0x0021);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::ReadyToSwitchOn);
}

TEST_F(SMCovTest, DecodeState_SwitchedOn) {
    sim_.setStatusWord(0x0023);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::SwitchedOn);
}

TEST_F(SMCovTest, DecodeState_OperationEnabled) {
    sim_.setStatusWord(0x0027);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::OperationEnabled);
}

TEST_F(SMCovTest, DecodeState_QuickStopActive) {
    sim_.setStatusWord(0x0007);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::QuickStopActive);
}

TEST_F(SMCovTest, DecodeState_Fault) {
    sim_.setStatusWord(0x0008);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::Fault);
}

TEST_F(SMCovTest, DecodeState_FaultReactionActive) {
    sim_.setStatusWord(0x000F);
    sm_.update();
    EXPECT_EQ(sm_.getCurrentState(), State::FaultReactionActive);
}

// ============================================================================
// ControlWord namespace free functions
// ============================================================================

TEST(CWBuilderTest, AllValues) {
    EXPECT_EQ(ControlWord::Shutdown(), 0x0006);
    EXPECT_EQ(ControlWord::SwitchOn(), 0x0007);
    EXPECT_EQ(ControlWord::SwitchOnEnable(), 0x000F);
    EXPECT_EQ(ControlWord::DisableVoltage(), 0x0000);
    EXPECT_EQ(ControlWord::QuickStop(), 0x0002);
    EXPECT_EQ(ControlWord::DisableOperation(), 0x0007);
    EXPECT_EQ(ControlWord::EnableOperation(), 0x000F);
    EXPECT_EQ(ControlWord::FaultReset(), 0x0080);
    EXPECT_EQ(ControlWord::TransitionMask(), 0x008F);
}

// ============================================================================
// StatusWord namespace free functions
// ============================================================================

TEST(SWDecoderCovTest, AllPatterns) {
    // SwitchOnDisabled: mask 0x004F, val 0x0040
    EXPECT_TRUE(StatusWord::isSwitchOnDisabled(0x0040));
    EXPECT_FALSE(StatusWord::isSwitchOnDisabled(0x0021));
    
    // ReadyToSwitchOn: mask 0x006F, val 0x0021
    EXPECT_TRUE(StatusWord::isReadyToSwitchOn(0x0021));
    EXPECT_FALSE(StatusWord::isReadyToSwitchOn(0x0040));
    
    // SwitchedOn: mask 0x006F, val 0x0023
    EXPECT_TRUE(StatusWord::isSwitchedOn(0x0023));
    EXPECT_FALSE(StatusWord::isSwitchedOn(0x0027));
    
    // OperationEnabled: mask 0x006F, val 0x0027
    EXPECT_TRUE(StatusWord::isOperationEnabled(0x0027));
    EXPECT_FALSE(StatusWord::isOperationEnabled(0x0023));
    
    // Fault: mask 0x004F, val 0x0008
    EXPECT_TRUE(StatusWord::isFault(0x0008));
    EXPECT_FALSE(StatusWord::isFault(0x0040));
    
    // FaultReactionActive: mask 0x004F, val 0x000F
    EXPECT_TRUE(StatusWord::isFaultReactionActive(0x000F));
    EXPECT_FALSE(StatusWord::isFaultReactionActive(0x0008));
    
    // QuickStopActive: mask 0x006F, val 0x0007
    EXPECT_TRUE(StatusWord::isQuickStopActive(0x0007));
    EXPECT_FALSE(StatusWord::isQuickStopActive(0x0027));
    
    // Bit checks
    EXPECT_TRUE(StatusWord::isTargetReached(0x0400));
    EXPECT_FALSE(StatusWord::isTargetReached(0x0000));
    
    EXPECT_TRUE(StatusWord::hasWarning(0x0080));
    EXPECT_FALSE(StatusWord::hasWarning(0x0000));
    
    EXPECT_TRUE(StatusWord::isLimitActive(0x0800));
    EXPECT_FALSE(StatusWord::isLimitActive(0x0000));
    
    EXPECT_TRUE(StatusWord::isHomingAttained(0x1000));
    EXPECT_FALSE(StatusWord::isHomingAttained(0x0000));
    
    EXPECT_TRUE(StatusWord::hasHomingError(0x2000));
    EXPECT_FALSE(StatusWord::hasHomingError(0x0000));
}

// ============================================================================
// TransitionResult enum
// ============================================================================

TEST(TransitionResultCovTest, AllValues) {
    EXPECT_NE(TransitionResult::Success, TransitionResult::Pending);
    EXPECT_NE(TransitionResult::Pending, TransitionResult::InvalidTransition);
    EXPECT_NE(TransitionResult::InvalidTransition, TransitionResult::Timeout);
    EXPECT_NE(TransitionResult::Timeout, TransitionResult::FaultOccurred);
}

// ============================================================================
// Full round-trip: SOD -> OpEnabled -> QuickStop -> SOD
// ============================================================================

TEST_F(SMCovTest, FullRoundTrip) {
    sim_.setStatusWord(0x0040); // SwitchOnDisabled
    sm_.update();
    
    // Enable
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
    EXPECT_EQ(sm_.getCurrentState(), State::OperationEnabled);
    
    // Quick stop
    EXPECT_TRUE(sm_.quickStop());
    sm_.update(); // Should move to QS state
    
    // Back to SOD
    r = sm_.requestState(State::SwitchOnDisabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

// ============================================================================
// Full round-trip: Fault -> Reset -> Enable
// ============================================================================

TEST_F(SMCovTest, FaultResetAndEnable) {
    sim_.setStatusWord(0x0008); // Fault
    sm_.update();
    EXPECT_TRUE(sm_.isFaulted());
    
    // Reset fault
    EXPECT_TRUE(sm_.resetFault());
    sm_.update(); // Should now be SOD (simulator responds to FaultReset)
    
    // Enable
    auto r = sm_.requestState(State::OperationEnabled, 500);
    EXPECT_EQ(r, TransitionResult::Success);
}

// ============================================================================
// DisplayedMode (initially same as operating mode)
// ============================================================================

TEST_F(SMCovTest, DisplayedMode_Default) {
    EXPECT_EQ(sm_.getDisplayedMode(), OperatingMode::ProfilePosition);
}

// ============================================================================
// getIntermediateState: edge cases
// ============================================================================

TEST_F(SMCovTest, IntermediateState_NotReady) {
    sim_.setStatusWord(0x0000); // NotReadyToSwitchOn
    sm_.update();
    // requestState from NotReady uses default path
    auto r = sm_.requestState(State::SwitchOnDisabled, 0);
    // NotReadyToSwitchOn doesn't have a transition - goes to default
    (void)r;
}

// ============================================================================
// calculateControlWord: Fault state
// ============================================================================

TEST_F(SMCovTest, CalculateControlWord_FromFault) {
    sim_.setStatusWord(0x0008); // Fault
    sm_.update();
    // Execute a transition - from fault, any target gets FaultReset
    sm_.executeTransition(ControlWord::FaultReset());
    EXPECT_EQ(sim_.getLastControl() & ControlWord::TransitionMask(), 
              ControlWord::FaultReset());
}

// ============================================================================
// Edge: transition pending tracking
// ============================================================================

TEST_F(SMCovTest, TransitionPending_CompletesOnTarget) {
    sim_.setStatusWord(0x0040);
    sm_.update();
    auto r = sm_.requestState(State::ReadyToSwitchOn, 500);
    EXPECT_EQ(r, TransitionResult::Success);
    // After completion, transition should no longer be pending
}
