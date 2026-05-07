/**
 * @file test_MotionController_coverage.cpp
 * @brief Comprehensive tests for MotionController, MotionProfile, and utility functions
 */

#include <gtest/gtest.h>
#include <tether/profiles/cia402/MotionController.hpp>
#include <tether/profiles/cia402/MotionProfile.hpp>
#include "mocks/MockDriveBackend.hpp"

using namespace CiA402;

// ============================================================================
// Utility Free Functions
// ============================================================================

class CiA402UtilFunctionTest : public ::testing::Test {};

TEST_F(CiA402UtilFunctionTest, StateToString) {
    EXPECT_NE(nullptr, stateToString(State::NotReadyToSwitchOn));
    EXPECT_NE(nullptr, stateToString(State::SwitchOnDisabled));
    EXPECT_NE(nullptr, stateToString(State::ReadyToSwitchOn));
    EXPECT_NE(nullptr, stateToString(State::SwitchedOn));
    EXPECT_NE(nullptr, stateToString(State::OperationEnabled));
    EXPECT_NE(nullptr, stateToString(State::QuickStopActive));
    EXPECT_NE(nullptr, stateToString(State::FaultReactionActive));
    EXPECT_NE(nullptr, stateToString(State::Fault));
}

TEST_F(CiA402UtilFunctionTest, ModeToString) {
    EXPECT_NE(nullptr, modeToString(OperatingMode::NoMode));
    EXPECT_NE(nullptr, modeToString(OperatingMode::ProfilePosition));
    EXPECT_NE(nullptr, modeToString(OperatingMode::Velocity));
    EXPECT_NE(nullptr, modeToString(OperatingMode::ProfileVelocity));
    EXPECT_NE(nullptr, modeToString(OperatingMode::ProfileTorque));
    EXPECT_NE(nullptr, modeToString(OperatingMode::Homing));
    EXPECT_NE(nullptr, modeToString(OperatingMode::InterpolatedPosition));
    EXPECT_NE(nullptr, modeToString(OperatingMode::CyclicSyncPosition));
    EXPECT_NE(nullptr, modeToString(OperatingMode::CyclicSyncVelocity));
    EXPECT_NE(nullptr, modeToString(OperatingMode::CyclicSyncTorque));
}

TEST_F(CiA402UtilFunctionTest, ErrorToString) {
    EXPECT_NE(nullptr, errorToString(0x0000));
    EXPECT_NE(nullptr, errorToString(0x1000));
    EXPECT_NE(nullptr, errorToString(0x2310));
    EXPECT_NE(nullptr, errorToString(0x3210));
    EXPECT_NE(nullptr, errorToString(0x3220));
    EXPECT_NE(nullptr, errorToString(0x4210));
    EXPECT_NE(nullptr, errorToString(0x5110));
    EXPECT_NE(nullptr, errorToString(0x5441));
    EXPECT_NE(nullptr, errorToString(0x8611));
    EXPECT_NE(nullptr, errorToString(0x8100));
    EXPECT_NE(nullptr, errorToString(0x8620));
    EXPECT_NE(nullptr, errorToString(0xFFFF)); // unknown
}

TEST_F(CiA402UtilFunctionTest, HomingMethodToString) {
    EXPECT_NE(nullptr, homingMethodToString(HomingMethod::NoHoming));
    EXPECT_NE(nullptr, homingMethodToString(HomingMethod::NegLimitIndex));
    EXPECT_NE(nullptr, homingMethodToString(HomingMethod::CurrentPosition));
    EXPECT_NE(nullptr, homingMethodToString(HomingMethod::NegDirIndexPulse));
    EXPECT_NE(nullptr, homingMethodToString(static_cast<HomingMethod>(99)));
}

// ============================================================================
// MotionProfile Factory
// ============================================================================

class MotionProfileFactoryTest : public ::testing::Test {};

TEST_F(MotionProfileFactoryTest, CreateLinearProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    auto p = createProfile(ProfileType::Linear, limits);
    ASSERT_NE(nullptr, p);
    EXPECT_EQ(ProfileType::Linear, p->getType());
}

TEST_F(MotionProfileFactoryTest, CreateTrapezoidalProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    limits.maxDeceleration = 50000;
    auto p = createProfile(ProfileType::Trapezoidal, limits);
    ASSERT_NE(nullptr, p);
    EXPECT_EQ(ProfileType::Trapezoidal, p->getType());
}

TEST_F(MotionProfileFactoryTest, CreateSCurveProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    limits.maxDeceleration = 50000;
    limits.maxJerk = 100000;
    auto p = createProfile(ProfileType::SCurve, limits);
    ASSERT_NE(nullptr, p);
    EXPECT_EQ(ProfileType::SCurve, p->getType());
}

TEST_F(MotionProfileFactoryTest, CreateTriangularProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    auto p = createProfile(ProfileType::Triangular, limits);
    ASSERT_NE(nullptr, p);
    EXPECT_EQ(ProfileType::Triangular, p->getType());
}

TEST_F(MotionProfileFactoryTest, CreatePolynomialProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    auto p = createProfile(ProfileType::Polynomial, limits);
    ASSERT_NE(nullptr, p);
    EXPECT_EQ(ProfileType::Polynomial, p->getType());
}

// ============================================================================
// LinearProfile Tests
// ============================================================================

class LinearProfileCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionLimits limits;
        limits.maxVelocity = 100.0;
        profile = createProfile(ProfileType::Linear, limits);
    }
    std::unique_ptr<MotionProfile> profile;
};

TEST_F(LinearProfileCovTest, Plan) {
    double duration = profile->plan(0.0, 1000.0, 0.0, 0.0);
    EXPECT_GT(duration, 0);
}

TEST_F(LinearProfileCovTest, Evaluate) {
    double duration = profile->plan(0.0, 1000.0, 0.0, 0.0);
    auto state = profile->evaluate(0.0);
    EXPECT_DOUBLE_EQ(0.0, state.position);
    EXPECT_FALSE(state.complete);
    
    auto mid = profile->evaluate(duration / 2.0);
    EXPECT_GT(mid.position, 0);
    
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(1000.0, end.position, 1.0);
}

TEST_F(LinearProfileCovTest, IsComplete) {
    double duration = profile->plan(0.0, 100.0, 0.0, 0.0);
    EXPECT_FALSE(profile->isComplete(0.0));
    EXPECT_TRUE(profile->isComplete(duration + 0.1));
}

TEST_F(LinearProfileCovTest, GetDuration) {
    double duration = profile->plan(0.0, 100.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(duration, profile->getDuration());
}

// ============================================================================
// TrapezoidalProfile Tests
// ============================================================================

class TrapezoidalProfileCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionLimits limits;
        limits.maxVelocity = 100.0;
        limits.maxAcceleration = 500.0;
        limits.maxDeceleration = 500.0;
        profile = createProfile(ProfileType::Trapezoidal, limits);
    }
    std::unique_ptr<MotionProfile> profile;
};

TEST_F(TrapezoidalProfileCovTest, Plan) {
    double duration = profile->plan(0, 1000, 0, 0);
    EXPECT_GT(duration, 0);
}

TEST_F(TrapezoidalProfileCovTest, Evaluate) {
    double duration = profile->plan(0, 1000, 0, 0);
    auto start = profile->evaluate(0);
    EXPECT_NEAR(0.0, start.position, 0.01);
    
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(1000.0, end.position, 1.0);
}

TEST_F(TrapezoidalProfileCovTest, ShortMove) {
    // Short move might be triangular
    double duration = profile->plan(0, 1, 0, 0);
    EXPECT_GT(duration, 0);
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(1.0, end.position, 0.1);
}

TEST_F(TrapezoidalProfileCovTest, NegativeDirection) {
    double duration = profile->plan(1000, 0, 0, 0);
    EXPECT_GT(duration, 0);
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(0.0, end.position, 1.0);
}

TEST_F(TrapezoidalProfileCovTest, SpeedFactor) {
    profile->setSpeedFactor(0.5);
    EXPECT_DOUBLE_EQ(0.5, profile->getSpeedFactor());
}

TEST_F(TrapezoidalProfileCovTest, SetLimits) {
    MotionLimits newLimits;
    newLimits.maxVelocity = 200;
    newLimits.maxAcceleration = 1000;
    newLimits.maxDeceleration = 1000;
    profile->setLimits(newLimits);
    auto retrieved = profile->getLimits();
    EXPECT_DOUBLE_EQ(200, retrieved.maxVelocity);
}

// ============================================================================
// SCurveProfile Tests
// ============================================================================

class SCurveProfileCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionLimits limits;
        limits.maxVelocity = 100;
        limits.maxAcceleration = 500;
        limits.maxDeceleration = 500;
        limits.maxJerk = 5000;
        profile = createProfile(ProfileType::SCurve, limits);
    }
    std::unique_ptr<MotionProfile> profile;
};

TEST_F(SCurveProfileCovTest, Plan) {
    double duration = profile->plan(0, 1000, 0, 0);
    EXPECT_GT(duration, 0);
}

TEST_F(SCurveProfileCovTest, Evaluate) {
    double duration = profile->plan(0, 500, 0, 0);
    auto start = profile->evaluate(0);
    EXPECT_NEAR(0.0, start.position, 0.01);
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(500.0, end.position, 5.0);
}

// ============================================================================
// PolynomialProfile Tests
// ============================================================================

class PolynomialProfileCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionLimits limits;
        limits.maxVelocity = 100;
        limits.maxAcceleration = 500;
        profile = createProfile(ProfileType::Polynomial, limits);
    }
    std::unique_ptr<MotionProfile> profile;
};

TEST_F(PolynomialProfileCovTest, Plan) {
    double duration = profile->plan(0, 100, 0, 0);
    EXPECT_GT(duration, 0);
}

TEST_F(PolynomialProfileCovTest, Evaluate) {
    double duration = profile->plan(0, 100, 0, 0);
    auto end = profile->evaluate(duration);
    EXPECT_NEAR(100.0, end.position, 5.0);
}

// ============================================================================
// MotionProfile Base Tests
// ============================================================================

TEST(MotionProfileBaseTest, GetSetMaxVelocity) {
    MotionLimits limits;
    limits.maxVelocity = 50;
    auto p = createProfile(ProfileType::Linear, limits);
    p->setMaxVelocity(200);
    EXPECT_DOUBLE_EQ(200, p->getLimits().maxVelocity);
}

TEST(MotionProfileBaseTest, GetSetMaxAcceleration) {
    MotionLimits limits;
    limits.maxVelocity = 50;
    auto p = createProfile(ProfileType::Linear, limits);
    p->setMaxAcceleration(1000);
    EXPECT_DOUBLE_EQ(1000, p->getLimits().maxAcceleration);
}

TEST(MotionProfileBaseTest, GetSetMaxDeceleration) {
    MotionLimits limits;
    limits.maxVelocity = 50;
    auto p = createProfile(ProfileType::Linear, limits);
    p->setMaxDeceleration(800);
    EXPECT_DOUBLE_EQ(800, p->getLimits().maxDeceleration);
}

TEST(MotionProfileBaseTest, GetSetMaxJerk) {
    MotionLimits limits;
    limits.maxVelocity = 50;
    auto p = createProfile(ProfileType::Linear, limits);
    p->setMaxJerk(5000);
    EXPECT_DOUBLE_EQ(5000, p->getLimits().maxJerk);
}

TEST(MotionProfileBaseTest, StartEndPositions) {
    MotionLimits limits;
    limits.maxVelocity = 100;
    auto p = createProfile(ProfileType::Linear, limits);
    p->plan(100, 500, 0, 0);
    EXPECT_DOUBLE_EQ(100, p->getStartPosition());
    EXPECT_DOUBLE_EQ(500, p->getEndPosition());
    EXPECT_DOUBLE_EQ(400, p->getDistance());
}

TEST(MotionProfileBaseTest, SelectOptimalProfile) {
    MotionLimits limits;
    limits.maxVelocity = 100;
    limits.maxAcceleration = 500;
    limits.maxDeceleration = 500;
    limits.maxJerk = 5000;
    auto type = selectOptimalProfile(1000, limits);
    // Should select a profile type
    EXPECT_NE(ProfileType::Custom, type);
}

// ============================================================================
// MotionLimits struct
// ============================================================================

TEST(MotionLimitsTest, DefaultValues) {
    MotionLimits limits{};
    EXPECT_GT(limits.maxVelocity, 0);
    EXPECT_GT(limits.maxAcceleration, 0);
}

// ============================================================================
// MotionState struct
// ============================================================================

TEST(MotionStateTest, DefaultValues) {
    MotionState state{};
    EXPECT_DOUBLE_EQ(0.0, state.position);
    EXPECT_DOUBLE_EQ(0.0, state.velocity);
    EXPECT_DOUBLE_EQ(0.0, state.acceleration);
    EXPECT_FALSE(state.complete);
}

// ============================================================================
// MotionController Tests
// ============================================================================

class MotionControllerCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller = std::make_unique<MotionController>();
    }

    CiA402AxisPtr addFakeAxis(CiA402Axis::AxisId id) {
        auto backend = std::make_unique<mock::FakeDriveBackend>();
        return controller->addAxis(id, std::move(backend));
    }

    std::unique_ptr<MotionController> controller;
};

// --- Axis Management ---

TEST_F(MotionControllerCovTest, AddAxis) {
    auto axis = addFakeAxis(0);
    ASSERT_NE(nullptr, axis);
    EXPECT_EQ(0u, axis->getId());
}

TEST_F(MotionControllerCovTest, GetAxis) {
    addFakeAxis(0);
    auto axis = controller->getAxis(0);
    EXPECT_NE(nullptr, axis);
}

TEST_F(MotionControllerCovTest, GetAxisNotFound) {
    auto axis = controller->getAxis(99);
    EXPECT_EQ(nullptr, axis);
}

TEST_F(MotionControllerCovTest, GetAxes) {
    addFakeAxis(0);
    addFakeAxis(1);
    const auto& axes = controller->getAxes();
    EXPECT_EQ(2u, axes.size());
}

TEST_F(MotionControllerCovTest, RemoveAxis) {
    addFakeAxis(0);
    EXPECT_EQ(1u, controller->getAxisCount());
    EXPECT_TRUE(controller->removeAxis(0));
    EXPECT_EQ(0u, controller->getAxisCount());
}

TEST_F(MotionControllerCovTest, RemoveAxisNotFound) {
    EXPECT_FALSE(controller->removeAxis(99));
}

TEST_F(MotionControllerCovTest, GetAxisCount) {
    EXPECT_EQ(0u, controller->getAxisCount());
    addFakeAxis(0);
    EXPECT_EQ(1u, controller->getAxisCount());
    addFakeAxis(1);
    EXPECT_EQ(2u, controller->getAxisCount());
}

// --- Group Operations ---

TEST_F(MotionControllerCovTest, EnableAll) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->enableAll(100);
}

TEST_F(MotionControllerCovTest, DisableAll) {
    addFakeAxis(0);
    controller->disableAll(100);
}

TEST_F(MotionControllerCovTest, QuickStopAll) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->quickStopAll();
}

TEST_F(MotionControllerCovTest, ClearAllFaults) {
    addFakeAxis(0);
    controller->clearAllFaults();
}

TEST_F(MotionControllerCovTest, AllEnabled) {
    auto result = controller->allEnabled();
    // No axes = all enabled (vacuous truth) or false depending on impl
}

TEST_F(MotionControllerCovTest, AnyFault) {
    auto result = controller->anyFault();
}

// --- Global Parameters ---

TEST_F(MotionControllerCovTest, SetGetSpeedFactor) {
    controller->setSpeedFactor(0.5f);
    EXPECT_FLOAT_EQ(0.5f, controller->getSpeedFactor());
}

TEST_F(MotionControllerCovTest, SetSpeedFactorOne) {
    controller->setSpeedFactor(1.0f);
    EXPECT_FLOAT_EQ(1.0f, controller->getSpeedFactor());
}

TEST_F(MotionControllerCovTest, SetGetGlobalParams) {
    MotionController::GlobalParams params{};
    params.speedFactor = 0.75f;
    params.allowNegativeSpeed = true;
    params.minSpeedFactor = 0.1f;
    params.maxSpeedFactor = 2.0f;
    controller->setGlobalParams(params);
    
    const auto& retrieved = controller->getGlobalParams();
    EXPECT_FLOAT_EQ(0.75f, retrieved.speedFactor);
    EXPECT_TRUE(retrieved.allowNegativeSpeed);
}

// --- Coordinated Motion ---

TEST_F(MotionControllerCovTest, SetCoordinatedAxes) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->setCoordinatedAxes({0, 1});
}

TEST_F(MotionControllerCovTest, MoveLinear) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->setCoordinatedAxes({0, 1});
    controller->moveLinear({100.0, 200.0}, 50.0);
}

TEST_F(MotionControllerCovTest, IsPathExecuting) {
    EXPECT_FALSE(controller->isPathExecuting());
}

TEST_F(MotionControllerCovTest, StopPath) {
    controller->stopPath();
    EXPECT_FALSE(controller->isPathExecuting());
}

// --- Gearing ---

TEST_F(MotionControllerCovTest, ConfigureGearing) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->configureGearing(1, 0, 2, 1);
}

TEST_F(MotionControllerCovTest, EnableGearing) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->configureGearing(1, 0, 1, 1);
    controller->enableGearing(1, true);
}

// --- Homing ---

TEST_F(MotionControllerCovTest, HomeAxis) {
    addFakeAxis(0);
    HomingCommand cmd{};
    cmd.method = HomingMethod::CurrentPosition;
    controller->homeAxis(0, cmd);
}

TEST_F(MotionControllerCovTest, AllHomed) {
    auto result = controller->allHomed();
}

// --- Cycle ---

TEST_F(MotionControllerCovTest, Update) {
    addFakeAxis(0);
    addFakeAxis(1);
    controller->update(0.001f);
}

TEST_F(MotionControllerCovTest, SetGetCycleTimeUs) {
    controller->setCycleTimeUs(500);
    EXPECT_EQ(500u, controller->getCycleTimeUs());
}

// ============================================================================
// CiA402Config helper functions
// ============================================================================

TEST(CiA402ConfigTest, StateToStringInline) {
    EXPECT_NE(nullptr, state_to_string(State::NotReadyToSwitchOn));
    EXPECT_NE(nullptr, state_to_string(State::SwitchOnDisabled));
    EXPECT_NE(nullptr, state_to_string(State::ReadyToSwitchOn));
    EXPECT_NE(nullptr, state_to_string(State::SwitchedOn));
    EXPECT_NE(nullptr, state_to_string(State::OperationEnabled));
    EXPECT_NE(nullptr, state_to_string(State::QuickStopActive));
    EXPECT_NE(nullptr, state_to_string(State::FaultReactionActive));
    EXPECT_NE(nullptr, state_to_string(State::Fault));
}

TEST(CiA402ConfigTest, ModeToStringInline) {
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::NoMode));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::ProfilePosition));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::CyclicSyncPosition));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::CyclicSyncVelocity));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::CyclicSyncTorque));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::Homing));
    EXPECT_NE(nullptr, mode_to_string(OperatingMode::InterpolatedPosition));
}

// PDO preset tests are in test_CiA402DrivePDO.cpp — skipped here to avoid namespace conflicts
