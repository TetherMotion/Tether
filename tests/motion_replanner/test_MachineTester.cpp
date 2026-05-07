/**
 * @file test_MachineTester.cpp
 * @brief Tests for MachineTester and its configuration structs
 *
 * Covers static factory methods, config defaults, trajectory generation,
 * friction model calculations, and test execution with mock callbacks.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/MachineTester.hpp>
#include <cmath>

using namespace MotionReplanner;

// ============================================================================
// SingleAxisTestConfig
// ============================================================================
TEST(SingleAxisTestConfigTest, Defaults) {
    SingleAxisTestConfig cfg;
    EXPECT_EQ(cfg.axis, 0);
    EXPECT_GT(cfg.amplitude, 0.0);
    EXPECT_GT(cfg.duration, 0.0);
}

// ============================================================================
// MultiAxisTestConfig
// ============================================================================
TEST(MultiAxisTestConfigTest, Defaults) {
    MultiAxisTestConfig cfg;
    EXPECT_GT(cfg.radiusU, 0.0);
    EXPECT_GT(cfg.feedRate, 0.0);
    EXPECT_GE(cfg.revolutions, 1);
}

// ============================================================================
// FrictionTestConfig
// ============================================================================
TEST(FrictionTestConfigTest, Defaults) {
    FrictionTestConfig cfg;
    EXPECT_EQ(cfg.axis, 0);
    EXPECT_GT(cfg.distance, 0.0);
}

// ============================================================================
// DelayTestConfig
// ============================================================================
TEST(DelayTestConfigTest, Defaults) {
    DelayTestConfig cfg;
    EXPECT_EQ(cfg.axis, 0);
    EXPECT_GT(cfg.stepAmplitude, 0.0);
}

// ============================================================================
// PIDTestConfig
// ============================================================================
TEST(PIDTestConfigTest, Defaults) {
    PIDTestConfig cfg;
    EXPECT_EQ(cfg.axis, 0);
    EXPECT_GT(cfg.stepAmplitude, 0.0);
}

// ============================================================================
// FrictionModel
// ============================================================================
TEST(FrictionModelTest, FrictionForcePositiveVelocity) {
    FrictionModel m;
    m.staticFriction = 10.0;
    m.coulombFriction = 5.0;
    m.viscousFriction = 0.01;
    m.stribeckVelocity = 10.0;
    m.stribeckExponent = 2.0;

    double f = m.frictionForce(100.0);
    // At high velocity, mostly coulomb + viscous
    EXPECT_GT(f, 5.0);
}

TEST(FrictionModelTest, FrictionForceZero) {
    FrictionModel m;
    m.staticFriction = 10.0;
    m.coulombFriction = 5.0;
    m.viscousFriction = 0.01;

    double f = m.frictionForce(0.0);
    (void)f; // just verify no crash/NaN
}

TEST(FrictionModelTest, FrictionForceNegativeVelocity) {
    FrictionModel m;
    m.staticFriction = 10.0;
    m.coulombFriction = 5.0;
    m.viscousFriction = 0.01;
    m.stribeckVelocity = 10.0;

    double f = m.frictionForce(-100.0);
    EXPECT_LT(f, 0.0);
}

// ============================================================================
// TestResult
// ============================================================================
TEST(TestResultTest, Defaults) {
    TestResult r;
    EXPECT_TRUE(r.testName.empty());
    EXPECT_TRUE(r.passed);  // default is true
}

// ============================================================================
// TestSequence
// ============================================================================
TEST(TestSequenceTest, Defaults) {
    TestSequence seq;
    EXPECT_TRUE(seq.name.empty());
    EXPECT_TRUE(seq.singleAxisTests.empty());
    EXPECT_TRUE(seq.multiAxisTests.empty());
}

// ============================================================================
// MachineTester
// ============================================================================
class MachineTesterTest : public ::testing::Test {
protected:
    MachineTester tester_;
};

TEST_F(MachineTesterTest, DefaultState) {
    EXPECT_TRUE(tester_.getResults().empty());
}

TEST_F(MachineTesterTest, ClearResults) {
    tester_.clearResults();
    EXPECT_TRUE(tester_.getResults().empty());
}

TEST_F(MachineTesterTest, GetHeatmapBuilder) {
    const auto& hb = tester_.getHeatmapBuilder();
    (void)hb; // verify accessible
}

TEST_F(MachineTesterTest, MutableHeatmapBuilder) {
    auto& hb = tester_.heatmapBuilder();
    (void)hb;
}

// ============================================================================
// Static factory methods
// ============================================================================
TEST(MachineTesterStaticTest, CreateQuickCalibration) {
    auto seq = MachineTester::createQuickCalibration();
    EXPECT_FALSE(seq.name.empty());
    // Should contain some test configs
    size_t totalTests = seq.singleAxisTests.size() + seq.multiAxisTests.size() +
                        (seq.frictionTest.has_value() ? 1u : 0u) +
                        (seq.delayTest.has_value() ? 1u : 0u) +
                        (seq.pidTest.has_value() ? 1u : 0u);
    EXPECT_GT(totalTests, 0u);
}

TEST(MachineTesterStaticTest, CreateFullCalibration) {
    auto seq = MachineTester::createFullCalibration();
    EXPECT_FALSE(seq.name.empty());
    size_t totalTests = seq.singleAxisTests.size() + seq.multiAxisTests.size() +
                        (seq.frictionTest.has_value() ? 1u : 0u) +
                        (seq.delayTest.has_value() ? 1u : 0u) +
                        (seq.pidTest.has_value() ? 1u : 0u);
    EXPECT_GT(totalTests, 0u);
}

TEST(MachineTesterStaticTest, CreateAxisCharacterization) {
    auto seq = MachineTester::createAxisCharacterization(0);
    EXPECT_FALSE(seq.name.empty());
}

TEST(MachineTesterStaticTest, CreateWorkspaceMapping) {
    HeatmapConfig cfg;
    auto seq = MachineTester::createWorkspaceMapping(cfg);
    EXPECT_FALSE(seq.name.empty());
}

// ============================================================================
// RunSingleAxisTest with mock callbacks
// ============================================================================
TEST_F(MachineTesterTest, RunSingleAxisRampWithMocks) {
    // Set up a mock command callback that just returns true
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });

    // Feedback callback returns the commanded trajectory as-is (perfect tracking)
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    // Status callback
    std::vector<std::string> statusMessages;
    tester_.setStatusCallback([&](const std::string& msg) {
        statusMessages.push_back(msg);
    });

    SingleAxisTestConfig cfg;
    cfg.type = SingleAxisTestType::Ramp;
    cfg.axis = 0;
    cfg.amplitude = 50.0;
    cfg.velocity = 1000.0;
    cfg.duration = 1.0;

    auto result = tester_.runSingleAxisTest(cfg);
    // Should produce a valid result
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(MachineTesterTest, RunSingleAxisSinusoidWithMocks) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    SingleAxisTestConfig cfg;
    cfg.type = SingleAxisTestType::Sinusoid;
    cfg.axis = 0;
    cfg.amplitude = 30.0;
    cfg.frequency = 2.0;
    cfg.duration = 2.0;

    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(MachineTesterTest, RunMultiAxisCircleWithMocks) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Circle;
    cfg.radiusU = 25.0;
    cfg.feedRate = 500.0;
    cfg.revolutions = 1;

    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(MachineTesterTest, RunFrictionTestWithMocks) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    FrictionTestConfig cfg;
    cfg.axis = 0;
    cfg.velocities = {100.0, 500.0, 1000.0};
    cfg.distance = 50.0;

    auto result = tester_.runFrictionTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(MachineTesterTest, RunDelayTestWithMocks) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    DelayTestConfig cfg;
    cfg.axis = 0;
    cfg.stepAmplitude = 10.0;

    auto result = tester_.runDelayTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(MachineTesterTest, RunTestSequence) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    auto seq = MachineTester::createQuickCalibration();
    auto results = tester_.runTestSequence(seq);
    EXPECT_GE(results.size(), 1u);
}

// ============================================================================
// RunPIDTest with mock callbacks
// ============================================================================
TEST_F(MachineTesterTest, RunPIDTestWithMocks) {
    std::vector<PositionSample> capturedTrajectory;
    tester_.setCommandCallback([&](const std::vector<PositionSample>& traj) {
        capturedTrajectory = traj;
        return true;
    });
    tester_.setFeedbackCallback([&]() {
        return capturedTrajectory;
    });

    PIDTestConfig cfg;
    cfg.axis = 0;
    cfg.stepAmplitude = 10.0;

    auto result = tester_.runPIDTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

// ============================================================================
// Command callback returns false (simulating failure)
// ============================================================================
TEST_F(MachineTesterTest, CommandCallbackFailure) {
    tester_.setCommandCallback([](const std::vector<PositionSample>&) {
        return false; // failure
    });
    tester_.setFeedbackCallback([]() {
        return std::vector<PositionSample>{};
    });

    SingleAxisTestConfig cfg;
    cfg.type = SingleAxisTestType::Step;
    cfg.axis = 0;

    auto result = tester_.runSingleAxisTest(cfg);
    // Should still return a result, likely with passed=false
    (void)result;
}
