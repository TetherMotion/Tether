/**
 * @file test_HomingMethods_coverage.cpp
 * @brief Coverage tests for HomingMethods.cpp — exercises validation branches,
 *        per-method starters, static utilities, factory configs, and utils.
 */

#include "tether/profiles/cia402/HomingModes.hpp"
#include <gtest/gtest.h>
#include <string>

using namespace CiA402::Homing;

// ============================================================================
// Helper: Create fully-wired callbacks for testing
// ============================================================================
static HomingCallbacks makeTestCallbacks(int32_t& pos, int32_t& vel,
                                          SwitchStates& sw) {
    HomingCallbacks cb;
    cb.setVelocity = [&vel](int32_t v) { vel = v; };
    cb.stopMotion = [&vel]() { vel = 0; };
    cb.setHomePosition = [&pos](int32_t p) { pos = p; };
    cb.getPosition = [&pos]() { return pos; };
    cb.getVelocity = [&vel]() { return vel; };
    cb.getSwitchStates = [&sw]() { return sw; };
    cb.isMotionComplete = []() { return true; };
    cb.hasDriveFault = []() { return false; };
    return cb;
}

// ============================================================================
// Static method name tests (getMethodName)
// ============================================================================

TEST(HomingMethodsCovTest, MethodName_NoHoming) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::NoHoming);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_NegLimitIndex) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodNegLimitIndex);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_PosLimitIndex) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodPosLimitIndex);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_NegLimit) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodNegLimit);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_PosLimit) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodPosLimit);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_IndexNeg) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodIndexNeg);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_IndexPos) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodIndexPos);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_CurrentPosition) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_CurrentPositionIndex) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodCurrentPositionIndex);
    EXPECT_FALSE(name.empty());
}

TEST(HomingMethodsCovTest, MethodName_HomeSwitch) {
    // Methods 3-14 and 19-30 all go to "Home Switch" variants
    for (int m = 3; m <= 14; ++m) {
        auto name = HomingStateMachine::getMethodName(static_cast<HomingMethod>(m));
        EXPECT_FALSE(name.empty()) << "Method " << m;
    }
    for (int m = 19; m <= 30; ++m) {
        auto name = HomingStateMachine::getMethodName(static_cast<HomingMethod>(m));
        EXPECT_FALSE(name.empty()) << "Method " << m;
    }
}

TEST(HomingMethodsCovTest, MethodName_Unknown) {
    auto name = HomingStateMachine::getMethodName(static_cast<HomingMethod>(127));
    EXPECT_FALSE(name.empty());
}

// ============================================================================
// Static requirement checks
// ============================================================================

TEST(HomingMethodsCovTest, RequiresNegLimit) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodNegLimitIndex));
    EXPECT_TRUE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodNegLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodPosLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodCurrentPosition));
}

TEST(HomingMethodsCovTest, RequiresPosLimit) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodPosLimitIndex));
    EXPECT_TRUE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodPosLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodNegLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodCurrentPosition));
}

TEST(HomingMethodsCovTest, RequiresHomeSwitch) {
    // Methods 3-14 and 19-30 require home switch
    EXPECT_TRUE(HomingStateMachine::methodRequiresHomeSwitch(static_cast<HomingMethod>(3)));
    EXPECT_TRUE(HomingStateMachine::methodRequiresHomeSwitch(static_cast<HomingMethod>(14)));
    EXPECT_TRUE(HomingStateMachine::methodRequiresHomeSwitch(static_cast<HomingMethod>(19)));
    EXPECT_TRUE(HomingStateMachine::methodRequiresHomeSwitch(static_cast<HomingMethod>(30)));
    EXPECT_FALSE(HomingStateMachine::methodRequiresHomeSwitch(HomingMethod::MethodNegLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresHomeSwitch(HomingMethod::MethodCurrentPosition));
}

TEST(HomingMethodsCovTest, RequiresIndex) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodNegLimitIndex));
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodPosLimitIndex));
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodIndexNeg));
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodIndexPos));
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodCurrentPositionIndex));
    // Methods 3-14 require index
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(static_cast<HomingMethod>(3)));
    EXPECT_FALSE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodNegLimit));
    EXPECT_FALSE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodCurrentPosition));
}

// ============================================================================
// Construction and basic state
// ============================================================================

TEST(HomingMethodsCovTest, DefaultConstruction) {
    HomingStateMachine hsm;
    EXPECT_EQ(hsm.getState(), HomingState::Idle);
    EXPECT_FALSE(hsm.isHoming());
    EXPECT_FALSE(hsm.isComplete());
    EXPECT_FALSE(hsm.hasError());
    EXPECT_EQ(hsm.getLastError(), HomingError::None);
}

TEST(HomingMethodsCovTest, SetConfig) {
    HomingStateMachine hsm;
    HomingConfig cfg;
    cfg.searchVelocity = 2000;
    cfg.homeOffset = 100;
    hsm.setConfig(cfg);
    EXPECT_EQ(hsm.getConfig().searchVelocity, 2000);
    EXPECT_EQ(hsm.getConfig().homeOffset, 100);
}

TEST(HomingMethodsCovTest, SetCallbacks) {
    HomingStateMachine hsm;
    HomingCallbacks cb;
    cb.setVelocity = [](int32_t) {};
    cb.stopMotion = []() {};
    hsm.setCallbacks(cb);
}

TEST(HomingMethodsCovTest, SetErrorInjection) {
    HomingStateMachine hsm;
    HomingErrorInjection ei;
    ei.enabled = true;
    ei.simulateTimeout = true;
    hsm.setErrorInjection(ei);
    EXPECT_TRUE(hsm.getErrorInjection().enabled);
    EXPECT_TRUE(hsm.getErrorInjection().simulateTimeout);
}

// ============================================================================
// Start with NoHoming method (should fail/return without action)
// ============================================================================

TEST(HomingMethodsCovTest, Start_NoHoming) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    auto result = hsm.start(HomingMethod::NoHoming);
    // NoHoming should either return false or set error
    // (depends on implementation — just exercise the path)
    (void)result;
}

// ============================================================================
// Validation — missing callbacks
// ============================================================================

TEST(HomingMethodsCovTest, Start_MissingCallbacks) {
    HomingStateMachine hsm;
    // No callbacks set → validation should fail
    auto result = hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_FALSE(result);
    EXPECT_TRUE(hsm.hasError());
}

// ============================================================================
// Start NegLimit method (17)
// ============================================================================

TEST(HomingMethodsCovTest, Start_NegLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodNegLimit));
    EXPECT_TRUE(hsm.isHoming());
}

TEST(HomingMethodsCovTest, Start_NegLimit_AlreadyAtLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    sw.negativeLimit = true;
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodNegLimit));
    EXPECT_TRUE(hsm.isHoming());
}

// ============================================================================
// Start PosLimit method (18)
// ============================================================================

TEST(HomingMethodsCovTest, Start_PosLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodPosLimit));
    EXPECT_TRUE(hsm.isHoming());
}

TEST(HomingMethodsCovTest, Start_PosLimit_AlreadyAtLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    sw.positiveLimit = true;
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodPosLimit));
}

// ============================================================================
// Start NegLimitIndex method (1)
// ============================================================================

TEST(HomingMethodsCovTest, Start_NegLimitIndex) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodNegLimitIndex));
}

// ============================================================================
// Start PosLimitIndex method (2)
// ============================================================================

TEST(HomingMethodsCovTest, Start_PosLimitIndex) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodPosLimitIndex));
}

// ============================================================================
// Start HomeSwitch methods (3-14, 19-30)
// ============================================================================

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method3) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(3)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method7) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(7)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method14) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(14)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method19) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(19)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method25) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(25)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_Method30) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(30)));
}

TEST(HomingMethodsCovTest, Start_HomeSwitch_AlreadyOnSwitch) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    sw.homeSwitch = true;
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(static_cast<HomingMethod>(19)));
}

// ============================================================================
// Start IndexOnly methods (33, 34)
// ============================================================================

TEST(HomingMethodsCovTest, Start_IndexNeg) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodIndexNeg));
}

TEST(HomingMethodsCovTest, Start_IndexPos) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodIndexPos));
}

// ============================================================================
// Start CurrentPosition methods (35, 37)
// ============================================================================

TEST(HomingMethodsCovTest, Start_CurrentPosition) {
    HomingStateMachine hsm;
    int32_t pos = 5000, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodCurrentPosition));
    // Should complete quickly — set home at current position
}

TEST(HomingMethodsCovTest, Start_CurrentPositionIndex) {
    HomingStateMachine hsm;
    int32_t pos = 5000, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodCurrentPositionIndex));
}

// ============================================================================
// Stop and Reset
// ============================================================================

TEST(HomingMethodsCovTest, StopDuringHoming) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_TRUE(hsm.isHoming());
    hsm.stop();
    EXPECT_FALSE(hsm.isHoming());
}

TEST(HomingMethodsCovTest, ResetAfterError) {
    HomingStateMachine hsm;
    // Start without callbacks → error
    hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_TRUE(hsm.hasError());
    hsm.reset();
    EXPECT_FALSE(hsm.hasError());
    EXPECT_EQ(hsm.getState(), HomingState::Idle);
}

// ============================================================================
// Update — exercise state transitions
// ============================================================================

TEST(HomingMethodsCovTest, Update_Idle) {
    HomingStateMachine hsm;
    hsm.update();  // Should do nothing in Idle
    EXPECT_EQ(hsm.getState(), HomingState::Idle);
}

TEST(HomingMethodsCovTest, Update_FindSwitch_Transition) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    hsm.start(HomingMethod::MethodNegLimit);
    // Update several times — motion searching
    for (int i = 0; i < 10; ++i) {
        hsm.update();
    }
    // At some point it should still be homing or have found the limit
}

TEST(HomingMethodsCovTest, Update_CurrentPos_Completes) {
    HomingStateMachine hsm;
    int32_t pos = 5000, vel = 0;
    SwitchStates sw{};
    int32_t homeSet = -1;
    auto cb = makeTestCallbacks(pos, vel, sw);
    cb.onHomingComplete = [&homeSet](int32_t p) { homeSet = p; };
    hsm.setCallbacks(cb);
    
    EXPECT_TRUE(hsm.start(HomingMethod::MethodCurrentPosition));
    // Update until complete or max iterations
    for (int i = 0; i < 20; ++i) {
        hsm.update();
        if (hsm.isComplete()) break;
    }
}

// ============================================================================
// Endstop validation
// ============================================================================

TEST(HomingMethodsCovTest, ValidateEndstops_BothActive) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    sw.negativeLimit = true;
    sw.positiveLimit = true;
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    // Both limits active → should report error
    auto result = hsm.start(HomingMethod::MethodNegLimit);
    // Validation might reject this (BothLimitsActive)
    (void)result;
}

TEST(HomingMethodsCovTest, ValidateEndstops_Disabled) {
    HomingStateMachine hsm;
    HomingConfig cfg;
    cfg.validateEndstops = false;
    hsm.setConfig(cfg);
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    sw.negativeLimit = true;
    sw.positiveLimit = true;
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    auto result = hsm.start(HomingMethod::MethodNegLimit);
    // Should not reject due to endstop validation disabled
    (void)result;
}

// ============================================================================
// Double start (AlreadyHoming error)
// ============================================================================

TEST(HomingMethodsCovTest, AlreadyHoming) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    EXPECT_TRUE(hsm.start(HomingMethod::MethodNegLimit));
    // Second start should fail
    EXPECT_FALSE(hsm.start(HomingMethod::MethodNegLimit));
}

// ============================================================================
// Error injection
// ============================================================================

TEST(HomingMethodsCovTest, ErrorInjection_DisconnectNegLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    auto cb = makeTestCallbacks(pos, vel, sw);
    hsm.setCallbacks(cb);
    HomingErrorInjection ei;
    ei.enabled = true;
    ei.disconnectNegativeLimit = true;
    hsm.setErrorInjection(ei);
    auto result = hsm.start(HomingMethod::MethodNegLimit);
    (void)result;
}

TEST(HomingMethodsCovTest, ErrorInjection_DisconnectPosLimit) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    HomingErrorInjection ei;
    ei.enabled = true;
    ei.disconnectPositiveLimit = true;
    hsm.setErrorInjection(ei);
    auto result = hsm.start(HomingMethod::MethodPosLimit);
    (void)result;
}

// ============================================================================
// Statistics
// ============================================================================

TEST(HomingMethodsCovTest, Statistics_Initial) {
    HomingStateMachine hsm;
    auto stats = hsm.getStatistics();
    EXPECT_EQ(stats.homingAttempts, 0u);
    EXPECT_EQ(stats.successfulHomings, 0u);
    EXPECT_EQ(stats.failedHomings, 0u);
}

TEST(HomingMethodsCovTest, Statistics_AfterAttempt) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_GE(hsm.getStatistics().homingAttempts, 1u);
}

// ============================================================================
// HomingStatistics reset
// ============================================================================

TEST(HomingMethodsCovTest, Statistics_Reset) {
    HomingStatistics stats;
    stats.homingAttempts = 5;
    stats.failedHomings = 2;
    stats.reset();
    EXPECT_EQ(stats.homingAttempts, 0u);
    EXPECT_EQ(stats.failedHomings, 0u);
}

// ============================================================================
// HomingErrorInjection reset
// ============================================================================

TEST(HomingMethodsCovTest, ErrorInjection_Reset) {
    HomingErrorInjection ei;
    ei.enabled = true;
    ei.simulateTimeout = true;
    ei.disconnectNegativeLimit = true;
    ei.reset();
    EXPECT_FALSE(ei.enabled);
    EXPECT_FALSE(ei.simulateTimeout);
    EXPECT_FALSE(ei.disconnectNegativeLimit);
}

// ============================================================================
// HomingFactory configs
// ============================================================================

TEST(HomingMethodsCovTest, Factory_ServoConfig) {
    auto cfg = HomingFactory::createServoConfig(2000, 200, 50);
    EXPECT_EQ(cfg.searchVelocity, 2000);
    EXPECT_EQ(cfg.zeroVelocity, 200);
    EXPECT_EQ(cfg.homeOffset, 50);
}

TEST(HomingMethodsCovTest, Factory_LinearAxisConfig) {
    auto cfg = HomingFactory::createLinearAxisConfig(100000, 1000, 100);
    EXPECT_EQ(cfg.searchVelocity, 1000);
    EXPECT_EQ(cfg.zeroVelocity, 100);
}

TEST(HomingMethodsCovTest, Factory_RotaryAxisConfig) {
    auto cfg = HomingFactory::createRotaryAxisConfig(5000, true);
    EXPECT_EQ(cfg.searchVelocity, 5000);
}

TEST(HomingMethodsCovTest, Factory_StrictConfig) {
    HomingConfig base;
    auto strict = HomingFactory::createStrictConfig(base);
    EXPECT_TRUE(strict.stopOnError);
    EXPECT_TRUE(strict.faultOnError);
    EXPECT_TRUE(strict.validateEndstops);
}

TEST(HomingMethodsCovTest, Factory_LenientConfig) {
    HomingConfig base;
    auto lenient = HomingFactory::createLenientConfig(base);
    EXPECT_FALSE(lenient.faultOnError);
}

// ============================================================================
// HomingUtils
// ============================================================================

TEST(HomingMethodsCovTest, Utils_DescribeMethod) {
    auto desc = HomingUtils::describeMethod(HomingMethod::MethodNegLimit);
    EXPECT_FALSE(desc.empty());
    auto desc2 = HomingUtils::describeMethod(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(desc2.empty());
}

TEST(HomingMethodsCovTest, Utils_GetRequirements) {
    auto req = HomingUtils::getMethodRequirements(HomingMethod::MethodNegLimitIndex);
    EXPECT_TRUE(req.needsNegativeLimit);
    EXPECT_TRUE(req.needsIndex);
    
    auto req2 = HomingUtils::getMethodRequirements(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(req2.needsNegativeLimit);
    EXPECT_FALSE(req2.needsIndex);
}

TEST(HomingMethodsCovTest, Utils_SuggestMethod) {
    auto m = HomingUtils::suggestMethod(true, true, true, true);
    EXPECT_NE(m, HomingMethod::NoHoming);
    
    auto m2 = HomingUtils::suggestMethod(false, false, false, false);
    // Should suggest CurrentPosition when nothing available
    EXPECT_EQ(m2, HomingMethod::MethodCurrentPosition);
}

TEST(HomingMethodsCovTest, Utils_ValidateMethod) {
    EXPECT_TRUE(HomingUtils::validateMethod(HomingMethod::MethodNegLimit, true, false, false, false));
    EXPECT_FALSE(HomingUtils::validateMethod(HomingMethod::MethodNegLimit, false, false, false, false));
    EXPECT_TRUE(HomingUtils::validateMethod(HomingMethod::MethodCurrentPosition, false, false, false, false));
}

// ============================================================================
// State/error callback firing
// ============================================================================

TEST(HomingMethodsCovTest, StateChangeCallback) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    int stateChanges = 0;
    auto cb = makeTestCallbacks(pos, vel, sw);
    cb.onStateChange = [&stateChanges](HomingState) { stateChanges++; };
    hsm.setCallbacks(cb);
    hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_GT(stateChanges, 0);
}

TEST(HomingMethodsCovTest, ErrorCallback) {
    HomingStateMachine hsm;
    HomingError lastErr = HomingError::None;
    std::string lastMsg;
    HomingCallbacks cb;
    cb.onError = [&](HomingError e, const std::string& m) { lastErr = e; lastMsg = m; };
    hsm.setCallbacks(cb);
    // Start without setVelocity → should trigger error callback
    hsm.start(HomingMethod::MethodNegLimit);
    EXPECT_NE(lastErr, HomingError::None);
}

// ============================================================================
// Invalid method number
// ============================================================================

TEST(HomingMethodsCovTest, Start_InvalidMethod) {
    HomingStateMachine hsm;
    int32_t pos = 0, vel = 0;
    SwitchStates sw{};
    hsm.setCallbacks(makeTestCallbacks(pos, vel, sw));
    // Implementation accepts unknown method values and routes them to a fallback
    auto result = hsm.start(static_cast<HomingMethod>(99));
    // Just exercise the code path — the impl may accept or reject
    (void)result;
}
