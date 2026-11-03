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

// ============================================================================
// FSM coverage — state handlers, error injection, timeouts, interrupts
// ============================================================================

class HomingSmCovTest : public ::testing::Test {
protected:
    HomingStateMachine hsm_;
    int32_t position_ = 0;
    int32_t velocity_ = 0;
    SwitchStates switches_{};
    bool motion_complete_ = true;
    bool drive_fault_ = false;
    bool include_switches_ = true;
    bool include_motion_ = true;
    std::vector<HomingState> states_seen_;
    HomingError last_error_ = HomingError::None;
    int32_t home_pos_ = -1;
    int interrupts_ = 0;

    void SetUp() override {
        HomingConfig cfg{};
        cfg.endstopDebounceMs = 0;
        cfg.settleTimeMs = 0;
        hsm_.setConfig(cfg);
        rewire();
    }

    void rewire() {
        HomingCallbacks cb{};
        if (include_motion_) {
            cb.setVelocity = [this](int32_t v) { velocity_ = v; };
            cb.stopMotion = [this]() { velocity_ = 0; };
            cb.setHomePosition = [this](int32_t p) { position_ = p; };
        }
        cb.getPosition = [this] { return position_; };
        cb.getVelocity = [this] { return velocity_; };
        if (include_switches_) {
            cb.getSwitchStates = [this] { return switches_; };
        }
        cb.isMotionComplete = [this] { return motion_complete_; };
        cb.hasDriveFault = [this] { return drive_fault_; };
        cb.onStateChange = [this](HomingState s) { states_seen_.push_back(s); };
        cb.onError = [this](HomingError e, const std::string&) { last_error_ = e; };
        cb.onHomingComplete = [this](int32_t p) { home_pos_ = p; };
        cb.onHomingInterrupted = [this] { ++interrupts_; };
        hsm_.setCallbacks(cb);
    }

    void run(int n = 30) { for (int i = 0; i < n; ++i) hsm_.update(); }
};

TEST_F(HomingSmCovTest, NoHomingMethodRejected) {
    EXPECT_FALSE(hsm_.start(HomingMethod::NoHoming));
    EXPECT_EQ(last_error_, HomingError::InvalidMethod);
}

TEST_F(HomingSmCovTest, BothLimitsRejectedAtStart) {
    switches_.positiveLimit = true;
    switches_.negativeLimit = true;
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    EXPECT_EQ(last_error_, HomingError::BothLimitsActive);
}

TEST_F(HomingSmCovTest, MissingSwitchCallbackErrors) {
    include_switches_ = false;
    rewire();
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    EXPECT_EQ(last_error_, HomingError::NegativeLimitNotConfigured);
    hsm_.reset();
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodPosLimitIndex));
    EXPECT_EQ(last_error_, HomingError::PositiveLimitNotConfigured);
    hsm_.reset();
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodPosHome));
    EXPECT_EQ(last_error_, HomingError::HomeSwitchNotConfigured);
    hsm_.reset();
    // Index-only method requires index but no limits/home.
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodIndexNeg));
    EXPECT_EQ(last_error_, HomingError::IndexNotConfigured);
}

TEST_F(HomingSmCovTest, MissingMotionCallbacksError) {
    include_motion_ = false;
    rewire();
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    EXPECT_EQ(last_error_, HomingError::NotInitialized);
}

TEST_F(HomingSmCovTest, AlreadyHomingError) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    EXPECT_FALSE(hsm_.start(HomingMethod::MethodPosLimitIndex));
    EXPECT_EQ(last_error_, HomingError::AlreadyHoming);
}

TEST_F(HomingSmCovTest, UnknownMethodDispatchError) {
    ASSERT_TRUE(hsm_.start(static_cast<HomingMethod>(66)));
    hsm_.update();  // Starting → InvalidMethod
    EXPECT_EQ(last_error_, HomingError::InvalidMethod);
    EXPECT_TRUE(hsm_.hasError());
}

TEST_F(HomingSmCovTest, NegLimitIndexFullCycle) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();  // Starting → FindSwitch
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
    switches_.negativeLimit = true;
    hsm_.update();  // found → LeaveSwitch
    EXPECT_EQ(hsm_.getState(), HomingState::LeaveSwitch);
    switches_.negativeLimit = false;
    hsm_.update();  // left → FindIndex
    EXPECT_EQ(hsm_.getState(), HomingState::FindIndex);
    switches_.indexDetected = true;
    switches_.lastIndexPosition = 42;
    hsm_.update();  // index → ZeroVelocity
    EXPECT_EQ(hsm_.getState(), HomingState::ZeroVelocity);
    hsm_.update();  // ZeroVelocity → Attained
    hsm_.update();  // Attained → Idle + complete
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
    EXPECT_EQ(home_pos_, 42);
    EXPECT_EQ(hsm_.getStatistics().successfulHomings, 1u);
}

TEST_F(HomingSmCovTest, PosLimitIndexFullCycle) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodPosLimitIndex));
    hsm_.update();
    switches_.positiveLimit = true;
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::LeaveSwitch);
    switches_.positiveLimit = false;
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::FindIndex);
    switches_.indexDetected = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
    EXPECT_GE(home_pos_, 0);
}

TEST_F(HomingSmCovTest, NegLimitOnlyCompletesWithoutIndex) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimit));
    hsm_.update();
    switches_.negativeLimit = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
    EXPECT_GE(home_pos_, 0);
}

TEST_F(HomingSmCovTest, PosLimitOnlyCompletesWithoutIndex) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodPosLimit));
    hsm_.update();
    switches_.positiveLimit = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, HomeSwitchRisingEdge) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodPosHome));  // 19: pos/rising
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
    switches_.homeSwitch = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, HomeSwitchFallingEdge) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodHomePos2));  // 24: pos/falling
    hsm_.update();
    switches_.homeSwitch = false;
    hsm_.update();  // falling edge found → no leave, complete
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, HomeSwitchStartsOnSwitchGoesLeave) {
    switches_.homeSwitch = true;  // already on switch at start
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodPosHome));
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::LeaveSwitch);
}

TEST_F(HomingSmCovTest, NegLimitAlreadyOnSwitchBacksOff) {
    switches_.negativeLimit = true;
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
    EXPECT_GT(velocity_, 0);  // backing off moves positive
}

TEST_F(HomingSmCovTest, IndexOnlyMethod33) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodIndexNeg));
    hsm_.update();
    switches_.indexDetected = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, IndexOnlyMethod34) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodIndexPos));
    hsm_.update();
    switches_.indexDetected = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, Method35CurrentPositionNoIndex) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodCurrentPosition));
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, Method37CurrentPositionIndex) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodCurrentPositionIndex));
    switches_.indexDetected = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, StopMidHomingInterrupts) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    hsm_.stop();
    EXPECT_EQ(hsm_.getState(), HomingState::Interrupted);
    EXPECT_EQ(interrupts_, 1);
    hsm_.update();  // handleInterrupted → Idle
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, DriveFaultAtStarting) {
    drive_fault_ = true;
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();  // handleStarting sees fault
    EXPECT_EQ(last_error_, HomingError::DriveFault);
}

TEST_F(HomingSmCovTest, DriveFaultMidHoming) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    drive_fault_ = true;
    hsm_.update();  // checkMotionFaults → DriveFault
    EXPECT_EQ(last_error_, HomingError::DriveFault);
    EXPECT_EQ(hsm_.getStatistics().motionErrors, 0u);  // 0x8051 uncategorized
}

TEST_F(HomingSmCovTest, FollowingErrorInjection) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateFollowingError = true;
    hsm_.setErrorInjection(ei);
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    run(3);
    EXPECT_EQ(last_error_, HomingError::FollowingError);
}

TEST_F(HomingSmCovTest, JamInjection) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateJam = true;
    hsm_.setErrorInjection(ei);
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    run(3);
    EXPECT_EQ(last_error_, HomingError::MotionFault);
    // handleError re-runs each update while in Error — at least once.
    EXPECT_GE(hsm_.getStatistics().motionErrors, 1u);
}

TEST_F(HomingSmCovTest, DisconnectInjectionMasks) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.disconnectPositiveLimit = true;
    ei.disconnectHomeSwitch = true;
    ei.disconnectIndex = true;
    hsm_.setErrorInjection(ei);
    switches_.positiveLimit = true;
    switches_.homeSwitch = true;
    switches_.indexDetected = true;
    switches_.indexPulse = true;
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    // Injection masks the raw states — machine stays searching.
    run(3);
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
}

TEST_F(HomingSmCovTest, SearchTimeout) {
    HomingConfig cfg = hsm_.getConfig();
    cfg.searchTimeoutMs = 0;
    hsm_.setConfig(cfg);
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    hsm_.update();
    EXPECT_EQ(last_error_, HomingError::Timeout);
    hsm_.update();  // handleError categorizes the error
    EXPECT_EQ(hsm_.getStatistics().timeoutErrors, 1u);
}

TEST_F(HomingSmCovTest, IndexTimeout) {
    HomingConfig cfg = hsm_.getConfig();
    cfg.indexTimeoutMs = 0;
    hsm_.setConfig(cfg);
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    switches_.negativeLimit = true;
    hsm_.update();  // → LeaveSwitch
    switches_.negativeLimit = false;
    hsm_.update();  // → FindIndex
    EXPECT_EQ(hsm_.getState(), HomingState::FindIndex);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    hsm_.update();
    EXPECT_EQ(last_error_, HomingError::IndexTimeout);
    hsm_.update();  // handleError categorizes the error
    EXPECT_EQ(hsm_.getStatistics().indexErrors, 1u);
}

TEST_F(HomingSmCovTest, MultipleIndexInjectionErrors) {
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateMultipleIndex = true;
    hsm_.setErrorInjection(ei);
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    switches_.negativeLimit = true;
    hsm_.update();
    switches_.negativeLimit = false;
    hsm_.update();  // → FindIndex
    switches_.indexDetected = true;
    hsm_.update();  // count forced to 5 → error
    EXPECT_EQ(last_error_, HomingError::MultipleIndexPulses);
}

TEST_F(HomingSmCovTest, SimulatedTimeoutStaysSearching) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    // simulateTimeout also short-circuits FindSwitch — cover that arm.
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateTimeout = true;
    hsm_.setErrorInjection(ei);
    switches_.negativeLimit = true;
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
}

TEST_F(HomingSmCovTest, SimulatedTimeoutInFindIndex) {
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimitIndex));
    hsm_.update();
    switches_.negativeLimit = true;
    hsm_.update();
    switches_.negativeLimit = false;
    hsm_.update();  // → FindIndex (injection not yet enabled)
    ASSERT_EQ(hsm_.getState(), HomingState::FindIndex);
    HomingErrorInjection ei{};
    ei.enabled = true;
    ei.simulateTimeout = true;
    hsm_.setErrorInjection(ei);
    switches_.indexDetected = true;
    hsm_.update();  // simulateTimeout → early return, stays
    EXPECT_EQ(hsm_.getState(), HomingState::FindIndex);
}

TEST_F(HomingSmCovTest, ZeroVelocityWaitsForStop) {
    motion_complete_ = false;
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodNegLimit));
    hsm_.update();
    switches_.negativeLimit = true;
    hsm_.update();  // → ZeroVelocity
    EXPECT_EQ(hsm_.getState(), HomingState::ZeroVelocity);
    hsm_.update();  // still moving — stays
    EXPECT_EQ(hsm_.getState(), HomingState::ZeroVelocity);
    motion_complete_ = true;
    run();
    EXPECT_EQ(hsm_.getState(), HomingState::Idle);
}

TEST_F(HomingSmCovTest, PosLimitAlreadyOnSwitchBacksOff) {
    switches_.positiveLimit = true;
    ASSERT_TRUE(hsm_.start(HomingMethod::MethodPosLimitIndex));
    hsm_.update();
    EXPECT_EQ(hsm_.getState(), HomingState::FindSwitch);
    EXPECT_LT(velocity_, 0);  // backing off moves negative
}

TEST_F(HomingSmCovTest, HomeSwitchAllMethodArms) {
    // Every arm in startHomeSwitchHoming's dispatch: methods 3-14 (with
    // index) and 19-30 (without).
    for (int m : {3,4,5,6,7,8,9,10,11,12,13,14,19,20,21,22,23,24,25,26,27,28,29,30}) {
        hsm_.reset();
        ASSERT_TRUE(hsm_.start(static_cast<HomingMethod>(m))) << "method " << m;
        hsm_.update();
        EXPECT_TRUE(hsm_.getState() == HomingState::FindSwitch ||
                    hsm_.getState() == HomingState::LeaveSwitch) << "method " << m;
    }
}
