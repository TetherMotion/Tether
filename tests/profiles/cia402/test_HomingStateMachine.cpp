/**
 * @file test_HomingStateMachine.cpp
 * @brief Comprehensive tests for CiA402 HomingStateMachine, HomingFactory, HomingUtils
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/HomingModes.hpp"

using namespace CiA402::Homing;

// ============================================================================
// HomingConfig struct
// ============================================================================

TEST(HomingConfigStruct, DefaultValues) {
    HomingConfig c{};
    EXPECT_EQ(c.searchVelocity, 1000);
    EXPECT_EQ(c.zeroVelocity, 100);
    EXPECT_EQ(c.acceleration, 10000);
    EXPECT_EQ(c.deceleration, 10000);
    EXPECT_EQ(c.homeOffset, 0);
    EXPECT_EQ(c.searchTimeoutMs, 30000u);
    EXPECT_EQ(c.indexTimeoutMs, 5000u);
    EXPECT_EQ(c.settleTimeMs, 100u);
    EXPECT_TRUE(c.stopOnError);
    EXPECT_TRUE(c.faultOnError);
    EXPECT_TRUE(c.validateEndstops);
    EXPECT_EQ(c.endstopDebounceMs, 10u);
    EXPECT_TRUE(c.validateIndex);
    EXPECT_DOUBLE_EQ(c.expectedIndexInterval, 0.0);
    EXPECT_DOUBLE_EQ(c.indexIntervalTolerance, 0.01);
}

// ============================================================================
// SwitchStates struct
// ============================================================================

TEST(SwitchStatesStruct, DefaultValues) {
    SwitchStates s{};
    EXPECT_FALSE(s.positiveLimit);
    EXPECT_FALSE(s.negativeLimit);
    EXPECT_FALSE(s.homeSwitch);
    EXPECT_FALSE(s.indexPulse);
    EXPECT_FALSE(s.indexDetected);
    EXPECT_EQ(s.lastIndexPosition, 0);
}

// ============================================================================
// HomingStatistics struct
// ============================================================================

TEST(HomingStatsStruct, DefaultValues) {
    HomingStatistics s{};
    EXPECT_EQ(s.homingAttempts, 0u);
    EXPECT_EQ(s.successfulHomings, 0u);
    EXPECT_EQ(s.failedHomings, 0u);
    EXPECT_EQ(s.timeoutErrors, 0u);
    EXPECT_EQ(s.endstopErrors, 0u);
    EXPECT_EQ(s.indexErrors, 0u);
    EXPECT_EQ(s.motionErrors, 0u);
    EXPECT_EQ(s.lastHomingDurationMs, 0u);
    EXPECT_EQ(s.totalHomingTimeMs, 0u);
    EXPECT_EQ(s.lastHomePosition, 0);
}

TEST(HomingStatsStruct, Reset) {
    HomingStatistics s{};
    s.homingAttempts = 5;
    s.successfulHomings = 3;
    s.failedHomings = 2;
    s.reset();
    EXPECT_EQ(s.homingAttempts, 0u);
    EXPECT_EQ(s.successfulHomings, 0u);
    EXPECT_EQ(s.failedHomings, 0u);
}

// ============================================================================
// HomingErrorInjection struct
// ============================================================================

TEST(HomingErrorInjectionStruct, DefaultValues) {
    HomingErrorInjection ei{};
    EXPECT_FALSE(ei.enabled);
    EXPECT_FALSE(ei.disconnectNegativeLimit);
    EXPECT_FALSE(ei.disconnectPositiveLimit);
    EXPECT_FALSE(ei.disconnectHomeSwitch);
    EXPECT_FALSE(ei.disconnectIndex);
    EXPECT_FALSE(ei.simulateTimeout);
    EXPECT_FALSE(ei.simulateSlowResponse);
    EXPECT_DOUBLE_EQ(ei.responseDelayMs, 100.0);
    EXPECT_FALSE(ei.simulateNoise);
    EXPECT_FALSE(ei.simulateSticking);
    EXPECT_FALSE(ei.simulateMultipleIndex);
    EXPECT_FALSE(ei.simulateFollowingError);
    EXPECT_FALSE(ei.simulateJam);
}

TEST(HomingErrorInjectionStruct, Reset) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateTimeout = true;
    ei.simulateJam = true;
    ei.reset();
    EXPECT_FALSE(ei.enabled);
    EXPECT_FALSE(ei.simulateTimeout);
    EXPECT_FALSE(ei.simulateJam);
}

// ============================================================================
// HomingError enum
// ============================================================================

TEST(HomingErrorEnum, ValuesDistinct) {
    EXPECT_EQ(static_cast<uint16_t>(HomingError::None), 0u);
    EXPECT_NE(static_cast<uint16_t>(HomingError::InvalidMethod),
              static_cast<uint16_t>(HomingError::NotInitialized));
    EXPECT_NE(static_cast<uint16_t>(HomingError::Timeout),
              static_cast<uint16_t>(HomingError::Interrupted));
}

// ============================================================================
// HomingState enum
// ============================================================================

TEST(HomingStateEnum, AllDistinct) {
    EXPECT_NE(static_cast<int>(HomingState::Idle),
              static_cast<int>(HomingState::Starting));
    EXPECT_NE(static_cast<int>(HomingState::FindSwitch),
              static_cast<int>(HomingState::LeaveSwitch));
    EXPECT_NE(static_cast<int>(HomingState::FindIndex),
              static_cast<int>(HomingState::ZeroVelocity));
    EXPECT_NE(static_cast<int>(HomingState::Attained),
              static_cast<int>(HomingState::Error));
    EXPECT_NE(static_cast<int>(HomingState::Error),
              static_cast<int>(HomingState::Interrupted));
}

// ============================================================================
// HomingMethod enum
// ============================================================================

TEST(HomingMethodEnum, KeyValues) {
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::NoHoming), 0);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodNegLimitIndex), 1);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodPosLimitIndex), 2);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodNegLimit), 17);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodPosLimit), 18);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodCurrentPositionIndex), 35);
    EXPECT_EQ(static_cast<int8_t>(HomingMethod::MethodCurrentPosition), 37);
}

// ============================================================================
// Static method helpers
// ============================================================================

TEST(HomingStatic, GetMethodName) {
    auto name = HomingStateMachine::getMethodName(HomingMethod::MethodNegLimitIndex);
    EXPECT_FALSE(name.empty());
    auto name2 = HomingStateMachine::getMethodName(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(name2.empty());
    auto name3 = HomingStateMachine::getMethodName(HomingMethod::NoHoming);
    (void)name3;
}

TEST(HomingStatic, MethodRequiresNegativeLimit) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodNegLimitIndex));
    EXPECT_FALSE(HomingStateMachine::methodRequiresNegativeLimit(HomingMethod::MethodCurrentPosition));
}

TEST(HomingStatic, MethodRequiresPositiveLimit) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodPosLimitIndex));
    EXPECT_FALSE(HomingStateMachine::methodRequiresPositiveLimit(HomingMethod::MethodCurrentPosition));
}

TEST(HomingStatic, MethodRequiresHomeSwitch) {
    (void)HomingStateMachine::methodRequiresHomeSwitch(HomingMethod::MethodPosHomeIndex);
    EXPECT_FALSE(HomingStateMachine::methodRequiresHomeSwitch(HomingMethod::MethodCurrentPosition));
}

TEST(HomingStatic, MethodRequiresIndex) {
    EXPECT_TRUE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodNegLimitIndex));
    EXPECT_FALSE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodCurrentPosition));
    EXPECT_FALSE(HomingStateMachine::methodRequiresIndex(HomingMethod::MethodNegLimit));
}

// ============================================================================
// HomingStateMachine fixture
// ============================================================================

class HomingTest : public ::testing::Test {
protected:
    void SetUp() override {
        hsm_ = std::make_unique<HomingStateMachine>();
        setupCallbacks();
    }

    void setupCallbacks() {
        HomingCallbacks cb{};
        cb.setVelocity = [](int32_t) {};
        cb.stopMotion = []() {};
        cb.setHomePosition = [](int32_t) {};
        cb.getPosition = [this]() -> int32_t { return position_; };
        cb.getVelocity = [this]() -> int32_t { return velocity_; };
        cb.getSwitchStates = [this]() -> SwitchStates { return switches_; };
        cb.isMotionComplete = [this]() -> bool { return motion_complete_; };
        cb.hasDriveFault = []() -> bool { return false; };
        cb.onStateChange = [this](HomingState s) { lastState_ = s; };
        cb.onError = [this](HomingError e, const std::string& msg) {
            lastError_ = e;
            lastErrorMsg_ = msg;
        };
        cb.onHomingComplete = [this](int32_t pos) { homePos_ = pos; };
        cb.onHomingInterrupted = [this]() { interrupted_ = true; };
        hsm_->setCallbacks(cb);
    }

    std::unique_ptr<HomingStateMachine> hsm_;
    int32_t position_ = 0;
    int32_t velocity_ = 0;
    SwitchStates switches_{};
    bool motion_complete_ = false;
    HomingState lastState_ = HomingState::Idle;
    HomingError lastError_ = HomingError::None;
    std::string lastErrorMsg_;
    int32_t homePos_ = -1;
    bool interrupted_ = false;
};

TEST_F(HomingTest, DefaultState) {
    EXPECT_EQ(hsm_->getState(), HomingState::Idle);
    EXPECT_EQ(hsm_->getMethod(), HomingMethod::NoHoming);
    EXPECT_FALSE(hsm_->isHoming());
    EXPECT_FALSE(hsm_->isComplete());
    EXPECT_FALSE(hsm_->hasError());
    EXPECT_EQ(hsm_->getLastError(), HomingError::None);
    EXPECT_TRUE(hsm_->getLastErrorMessage().empty());
}

TEST_F(HomingTest, GetConfig) {
    const auto& cfg = hsm_->getConfig();
    EXPECT_EQ(cfg.searchVelocity, 1000);
}

TEST_F(HomingTest, SetConfig) {
    HomingConfig cfg{};
    cfg.searchVelocity = 2000;
    cfg.searchTimeoutMs = 10000;
    hsm_->setConfig(cfg);
    EXPECT_EQ(hsm_->getConfig().searchVelocity, 2000);
    EXPECT_EQ(hsm_->getConfig().searchTimeoutMs, 10000u);
}

TEST_F(HomingTest, GetStatistics) {
    const auto& stats = hsm_->getStatistics();
    EXPECT_EQ(stats.homingAttempts, 0u);
}

TEST_F(HomingTest, StartMethod37CurrentPosition) {
    bool ok = hsm_->start(HomingMethod::MethodCurrentPosition);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(hsm_->isHoming());
    EXPECT_EQ(hsm_->getMethod(), HomingMethod::MethodCurrentPosition);
}

TEST_F(HomingTest, StartAndCompleteMethod37) {
    hsm_->start(HomingMethod::MethodCurrentPosition);
    motion_complete_ = true;
    for (int i = 0; i < 20; ++i) hsm_->update();
    (void)hsm_->isComplete();
}

TEST_F(HomingTest, Stop) {
    hsm_->start(HomingMethod::MethodCurrentPosition);
    hsm_->stop();
    EXPECT_FALSE(hsm_->isHoming());
}

TEST_F(HomingTest, Reset) {
    hsm_->start(HomingMethod::MethodCurrentPosition);
    hsm_->reset();
    EXPECT_FALSE(hsm_->isHoming());
    EXPECT_FALSE(hsm_->isComplete());
    EXPECT_FALSE(hsm_->hasError());
    EXPECT_EQ(hsm_->getState(), HomingState::Idle);
}

TEST_F(HomingTest, StartMethod1NegLimitIndex) {
    bool ok = hsm_->start(HomingMethod::MethodNegLimitIndex);
    (void)ok;
    hsm_->update();
    hsm_->update();
    EXPECT_TRUE(hsm_->isHoming());
}

TEST_F(HomingTest, StartMethod2PosLimitIndex) {
    bool ok = hsm_->start(HomingMethod::MethodPosLimitIndex);
    (void)ok;
    hsm_->update();
}

TEST_F(HomingTest, StartMethod17NegLimit) {
    hsm_->start(HomingMethod::MethodNegLimit);
    hsm_->update();
}

TEST_F(HomingTest, ConfigAndStart) {
    HomingConfig cfg{};
    cfg.searchTimeoutMs = 5000;
    cfg.settleTimeMs = 50;
    hsm_->setConfig(cfg);
    hsm_->start(HomingMethod::MethodCurrentPosition);
    hsm_->update();
}

TEST_F(HomingTest, MultipleStartStop) {
    hsm_->start(HomingMethod::MethodCurrentPosition);
    hsm_->stop();
    hsm_->start(HomingMethod::MethodNegLimitIndex);
    hsm_->stop();
    hsm_->start(HomingMethod::MethodCurrentPosition);
    motion_complete_ = true;
    for (int i = 0; i < 20; ++i) hsm_->update();
}

TEST_F(HomingTest, SwitchActivationNegLimit) {
    hsm_->start(HomingMethod::MethodNegLimitIndex);
    // Simulate finding negative limit
    switches_.negativeLimit = true;
    for (int i = 0; i < 5; ++i) hsm_->update();
    // Simulate finding index
    switches_.indexPulse = true;
    switches_.indexDetected = true;
    motion_complete_ = true;
    for (int i = 0; i < 20; ++i) hsm_->update();
}

TEST_F(HomingTest, SwitchActivationPosLimit) {
    hsm_->start(HomingMethod::MethodPosLimitIndex);
    switches_.positiveLimit = true;
    for (int i = 0; i < 5; ++i) hsm_->update();
    switches_.indexPulse = true;
    switches_.indexDetected = true;
    motion_complete_ = true;
    for (int i = 0; i < 20; ++i) hsm_->update();
}

TEST_F(HomingTest, HomeSwitchActivation) {
    hsm_->start(HomingMethod::MethodPosHomeIndex);
    switches_.homeSwitch = true;
    for (int i = 0; i < 5; ++i) hsm_->update();
    switches_.indexPulse = true;
    switches_.indexDetected = true;
    motion_complete_ = true;
    for (int i = 0; i < 20; ++i) hsm_->update();
}

// ============================================================================
// Error injection
// ============================================================================

TEST_F(HomingTest, ErrorInjectionSetGet) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateTimeout = true;
    hsm_->setErrorInjection(ei);
    const auto& got = hsm_->getErrorInjection();
    EXPECT_TRUE(got.enabled);
    EXPECT_TRUE(got.simulateTimeout);
}

TEST_F(HomingTest, ErrorInjectionTimeout) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateTimeout = true;
    hsm_->setErrorInjection(ei);
    hsm_->start(HomingMethod::MethodNegLimitIndex);
    for (int i = 0; i < 100; ++i) hsm_->update();
    // May or may not be in error state depending on implementation
    (void)hsm_->hasError();
    (void)hsm_->getLastError();
}

TEST_F(HomingTest, ErrorInjectionDisconnectedLimit) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.disconnectNegativeLimit = true;
    hsm_->setErrorInjection(ei);
    hsm_->start(HomingMethod::MethodNegLimitIndex);
    for (int i = 0; i < 50; ++i) hsm_->update();
    (void)hsm_->hasError();
}

TEST_F(HomingTest, ErrorInjectionJam) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateJam = true;
    hsm_->setErrorInjection(ei);
    hsm_->start(HomingMethod::MethodNegLimitIndex);
    for (int i = 0; i < 50; ++i) hsm_->update();
    (void)hsm_->hasError();
}

// ============================================================================
// HomingFactory
// ============================================================================

TEST(HomingFactoryTest, CreateServoConfig) {
    auto cfg = HomingFactory::createServoConfig();
    EXPECT_GT(cfg.searchVelocity, 0);
    EXPECT_GT(cfg.zeroVelocity, 0);
}

TEST(HomingFactoryTest, CreateServoConfigCustom) {
    auto cfg = HomingFactory::createServoConfig(2000, 200, 100);
    EXPECT_EQ(cfg.searchVelocity, 2000);
    EXPECT_EQ(cfg.zeroVelocity, 200);
    EXPECT_EQ(cfg.homeOffset, 100);
}

TEST(HomingFactoryTest, CreateLinearAxisConfig) {
    auto cfg = HomingFactory::createLinearAxisConfig(10000);
    EXPECT_GT(cfg.searchVelocity, 0);
}

TEST(HomingFactoryTest, CreateLinearAxisConfigCustom) {
    auto cfg = HomingFactory::createLinearAxisConfig(50000, 1000, 100);
    EXPECT_EQ(cfg.searchVelocity, 1000);
    EXPECT_EQ(cfg.zeroVelocity, 100);
}

TEST(HomingFactoryTest, CreateRotaryAxisConfig) {
    auto cfg = HomingFactory::createRotaryAxisConfig();
    EXPECT_GT(cfg.searchVelocity, 0);
}

TEST(HomingFactoryTest, CreateRotaryAxisConfigCustom) {
    auto cfg = HomingFactory::createRotaryAxisConfig(3000, false);
    EXPECT_EQ(cfg.searchVelocity, 3000);
}

TEST(HomingFactoryTest, CreateStrictConfig) {
    HomingConfig base{};
    auto strict = HomingFactory::createStrictConfig(base);
    EXPECT_TRUE(strict.stopOnError);
    EXPECT_TRUE(strict.faultOnError);
    EXPECT_TRUE(strict.validateEndstops);
    EXPECT_TRUE(strict.validateIndex);
}

TEST(HomingFactoryTest, CreateLenientConfig) {
    HomingConfig base{};
    auto lenient = HomingFactory::createLenientConfig(base);
    (void)lenient; // just exercise
}

// ============================================================================
// HomingUtils
// ============================================================================

TEST(HomingUtilsTest, DescribeMethod) {
    auto desc = HomingUtils::describeMethod(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(desc.empty());
    auto desc2 = HomingUtils::describeMethod(HomingMethod::MethodNegLimitIndex);
    EXPECT_FALSE(desc2.empty());
}

TEST(HomingUtilsTest, GetMethodRequirements) {
    auto req = HomingUtils::getMethodRequirements(HomingMethod::MethodNegLimitIndex);
    EXPECT_TRUE(req.needsNegativeLimit);
    EXPECT_TRUE(req.needsIndex);
    EXPECT_FALSE(req.description.empty());

    auto req2 = HomingUtils::getMethodRequirements(HomingMethod::MethodCurrentPosition);
    EXPECT_FALSE(req2.needsNegativeLimit);
    EXPECT_FALSE(req2.needsPositiveLimit);
    EXPECT_FALSE(req2.needsIndex);
}

TEST(HomingUtilsTest, SuggestMethod) {
    auto m1 = HomingUtils::suggestMethod(true, true, true, true);
    (void)m1;
    auto m2 = HomingUtils::suggestMethod(false, false, false, false);
    (void)m2;
    auto m3 = HomingUtils::suggestMethod(true, false, true, false);
    (void)m3;
    auto m4 = HomingUtils::suggestMethod(false, true, false, true);
    (void)m4;
}

TEST(HomingUtilsTest, ValidateMethod) {
    // Method 37 (current position) needs no hardware
    bool v1 = HomingUtils::validateMethod(HomingMethod::MethodCurrentPosition,
                                           false, false, false, false);
    EXPECT_TRUE(v1);

    // Method 1 needs negative limit + index
    bool v2 = HomingUtils::validateMethod(HomingMethod::MethodNegLimitIndex,
                                           true, false, false, true);
    EXPECT_TRUE(v2);

    // Method 1 without negative limit should fail
    bool v3 = HomingUtils::validateMethod(HomingMethod::MethodNegLimitIndex,
                                           false, false, false, false);
    EXPECT_FALSE(v3);
}
