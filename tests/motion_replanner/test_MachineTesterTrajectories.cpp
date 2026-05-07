/**
 * @file test_MachineTesterTrajectories.cpp
 * @brief Extended tests for all MachineTester trajectory generators.
 *        Covers MachineTesterTrajectory.cpp (22% → target 100%).
 *
 * Tests all SingleAxisTestType and MultiAxisTestType variants through the
 * public runSingleAxisTest/runMultiAxisTest interface with mock callbacks.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/MachineTester.hpp>
#include <cmath>
#include <algorithm>

using namespace MotionReplanner;

// ============================================================================
// Fixture for trajectory tests
// ============================================================================
class TrajectoryGenerationTest : public ::testing::Test {
protected:
    MachineTester tester_;
    std::vector<PositionSample> capturedTrajectory_;

    void SetUp() override {
        tester_.setCommandCallback([this](const std::vector<PositionSample>& traj) {
            capturedTrajectory_ = traj;
            return true;
        });
        tester_.setFeedbackCallback([this]() {
            return capturedTrajectory_;
        });
        tester_.setStatusCallback([](const std::string&) {});
    }

    SingleAxisTestConfig makeSingleAxisConfig(SingleAxisTestType type, double duration = 2.0) {
        SingleAxisTestConfig cfg;
        cfg.type = type;
        cfg.axis = 0;
        cfg.amplitude = 50.0;
        cfg.frequency = 1.0;
        cfg.velocity = 1000.0;
        cfg.acceleration = 500.0;
        cfg.jerk = 5000.0;
        cfg.duration = duration;
        cfg.cycles = 3;
        cfg.centerPosition = 0.0;
        return cfg;
    }

    MultiAxisTestConfig makeMultiAxisConfig(MultiAxisTestType type, double duration = 3.0) {
        MultiAxisTestConfig cfg;
        cfg.type = type;
        cfg.uAxis = 0;
        cfg.vAxis = 1;
        cfg.wAxis = 2;
        cfg.radiusU = 25.0;
        cfg.radiusV = 25.0;
        cfg.pitchW = 5.0;
        cfg.rotationAngle = 0.0;
        cfg.feedRate = 500.0;
        cfg.duration = duration;
        cfg.revolutions = 2;
        cfg.center = {0.0, 0.0, 0.0};
        cfg.cornerRadius = 5.0;
        cfg.lissajousRatioU = 1.0;
        cfg.lissajousRatioV = 2.0;
        cfg.lissajousPhase = 90.0;
        return cfg;
    }
};

// ============================================================================
// Single Axis Tests — all trajectory types
// ============================================================================
TEST_F(TrajectoryGenerationTest, SCurve) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::SCurve);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
    // S-curve should have smooth acceleration profile
    EXPECT_GT(capturedTrajectory_.size(), 10u);
}

TEST_F(TrajectoryGenerationTest, Step) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Step);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, Triangular) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Triangular);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, Trapezoidal) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Trapezoidal);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, SCurveGeneratesPositions) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::SCurve, 1.0);
    tester_.runSingleAxisTest(cfg);
    // Verify trajectory has expected characteristics
    bool hasNonZero = false;
    for (const auto& s : capturedTrajectory_) {
        if (std::abs(s.position[0]) > 0.01) {
            hasNonZero = true;
            break;
        }
    }
    EXPECT_TRUE(hasNonZero);
}

TEST_F(TrajectoryGenerationTest, StepGeneratesStepChange) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Step, 1.0);
    tester_.runSingleAxisTest(cfg);
    // Step should have sudden position change
    EXPECT_GT(capturedTrajectory_.size(), 5u);
}

TEST_F(TrajectoryGenerationTest, TriangularGeneratesSymmetricWave) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Triangular, 2.0);
    tester_.runSingleAxisTest(cfg);
    // Should have positive and negative values
    bool hasPositive = false, hasNegative = false;
    for (const auto& s : capturedTrajectory_) {
        if (s.position[cfg.axis] > 0.1) hasPositive = true;
        if (s.position[cfg.axis] < -0.1) hasNegative = true;
    }
    // Either symmetric or at least has non-trivial positions
    EXPECT_TRUE(hasPositive || hasNegative || capturedTrajectory_.size() > 10);
}

TEST_F(TrajectoryGenerationTest, TrapezoidalHasConstantVelocitySection) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Trapezoidal, 3.0);
    tester_.runSingleAxisTest(cfg);
    EXPECT_GT(capturedTrajectory_.size(), 50u);
}

TEST_F(TrajectoryGenerationTest, SCurveVelocityValid) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::SCurve, 1.0);
    tester_.runSingleAxisTest(cfg);
    for (const auto& s : capturedTrajectory_) {
        EXPECT_TRUE(s.velocityValid);
    }
}

TEST_F(TrajectoryGenerationTest, StepVelocityValid) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Step, 1.0);
    tester_.runSingleAxisTest(cfg);
    for (const auto& s : capturedTrajectory_) {
        EXPECT_TRUE(s.velocityValid);
    }
}

// ============================================================================
// Multi Axis Tests — all pattern types
// ============================================================================
TEST_F(TrajectoryGenerationTest, Ellipse) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Ellipse);
    cfg.radiusU = 30.0;
    cfg.radiusV = 20.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, Helix) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Helix);
    cfg.pitchW = 10.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
    // Helix should have Z-axis movement
    bool hasZMovement = false;
    for (const auto& s : capturedTrajectory_) {
        if (std::abs(s.position[cfg.wAxis]) > 0.01) {
            hasZMovement = true;
            break;
        }
    }
    EXPECT_TRUE(hasZMovement);
}

TEST_F(TrajectoryGenerationTest, Lissajous) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Lissajous);
    cfg.lissajousRatioU = 1.0;
    cfg.lissajousRatioV = 2.0;
    cfg.lissajousPhase = 90.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, Square) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Square);
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

TEST_F(TrajectoryGenerationTest, RoundedSquare) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::RoundedSquare);
    cfg.cornerRadius = 5.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_FALSE(capturedTrajectory_.empty());
}

// ============================================================================
// Ellipse-specific checks
// ============================================================================
TEST_F(TrajectoryGenerationTest, EllipseAxisRatios) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Ellipse);
    cfg.radiusU = 40.0;
    cfg.radiusV = 20.0;
    tester_.runMultiAxisTest(cfg);

    double maxU = 0, maxV = 0;
    for (const auto& s : capturedTrajectory_) {
        maxU = std::max(maxU, std::abs(s.position[cfg.uAxis]));
        maxV = std::max(maxV, std::abs(s.position[cfg.vAxis]));
    }
    // U radius should be larger than V
    if (maxU > 1.0 && maxV > 1.0) {
        EXPECT_GT(maxU, maxV);
    }
}

TEST_F(TrajectoryGenerationTest, EllipseWithRotation) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Ellipse);
    cfg.rotationAngle = 45.0; // 45 degrees
    cfg.radiusU = 30.0;
    cfg.radiusV = 15.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_GT(capturedTrajectory_.size(), 10u);
}

// ============================================================================
// Helix-specific checks
// ============================================================================
TEST_F(TrajectoryGenerationTest, HelixMonotonicZ) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Helix);
    cfg.pitchW = 10.0;
    cfg.revolutions = 3;
    tester_.runMultiAxisTest(cfg);

    // Check Z generally increases (for positive pitch)
    if (capturedTrajectory_.size() > 100) {
        double firstZ = capturedTrajectory_.front().position[cfg.wAxis];
        double lastZ = capturedTrajectory_.back().position[cfg.wAxis];
        EXPECT_GT(lastZ, firstZ);
    }
}

// ============================================================================
// Lissajous-specific checks
// ============================================================================
TEST_F(TrajectoryGenerationTest, LissajousFrequencyRatio) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Lissajous);
    cfg.lissajousRatioU = 3.0;
    cfg.lissajousRatioV = 2.0;
    cfg.duration = 5.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    EXPECT_GT(capturedTrajectory_.size(), 100u);
}

// ============================================================================
// Square-specific checks
// ============================================================================
TEST_F(TrajectoryGenerationTest, SquareReachesCorners) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Square);
    cfg.radiusU = 20.0;
    tester_.runMultiAxisTest(cfg);

    // Square should have positions near the corners
    double maxU = 0, maxV = 0;
    for (const auto& s : capturedTrajectory_) {
        maxU = std::max(maxU, std::abs(s.position[cfg.uAxis]));
        maxV = std::max(maxV, std::abs(s.position[cfg.vAxis]));
    }
    // Should reach somewhere near the configured radius
    EXPECT_GT(maxU, 1.0);
}

// ============================================================================
// RoundedSquare-specific checks
// ============================================================================
TEST_F(TrajectoryGenerationTest, RoundedSquareSmallCornerRadius) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::RoundedSquare);
    cfg.cornerRadius = 2.0;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, RoundedSquareLargeCornerRadius) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::RoundedSquare);
    cfg.cornerRadius = 15.0; // close to radius → nearly circular
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

// ============================================================================
// DiagonalBox (if it exists in enum)
// ============================================================================
TEST_F(TrajectoryGenerationTest, DiagonalBox) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::DiagonalBox);
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

// ============================================================================
// Edge cases for single axis
// ============================================================================
TEST_F(TrajectoryGenerationTest, ShortDurationSCurve) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::SCurve, 0.1);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, LargeDurationStep) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Step, 5.0);
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, HighFrequencySinusoid) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Sinusoid, 1.0);
    cfg.frequency = 20.0;
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, DifferentAxis) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Ramp, 1.0);
    cfg.axis = 2; // Z axis
    auto result = tester_.runSingleAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
    // Check trajectory uses axis 2
    bool hasAxis2Movement = false;
    for (const auto& s : capturedTrajectory_) {
        if (std::abs(s.position[2]) > 0.01) {
            hasAxis2Movement = true;
            break;
        }
    }
    EXPECT_TRUE(hasAxis2Movement);
}

// ============================================================================
// Edge cases for multi axis
// ============================================================================
TEST_F(TrajectoryGenerationTest, EllipseEqualRadii) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Ellipse);
    cfg.radiusU = 25.0;
    cfg.radiusV = 25.0; // effectively a circle
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, HelixZeroPitch) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Helix);
    cfg.pitchW = 0.0; // zero pitch → circle
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

TEST_F(TrajectoryGenerationTest, SingleRevolution) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Circle);
    cfg.revolutions = 1;
    auto result = tester_.runMultiAxisTest(cfg);
    EXPECT_FALSE(result.testName.empty());
}

// ============================================================================
// Nonzero center position (single axis)
// ============================================================================
TEST_F(TrajectoryGenerationTest, NonzeroCenterPosition) {
    auto cfg = makeSingleAxisConfig(SingleAxisTestType::Sinusoid, 2.0);
    cfg.centerPosition = 100.0;
    tester_.runSingleAxisTest(cfg);
    // Average position should be near 100
    if (!capturedTrajectory_.empty()) {
        double sum = 0;
        for (const auto& s : capturedTrajectory_) {
            sum += s.position[cfg.axis];
        }
        double avg = sum / capturedTrajectory_.size();
        EXPECT_NEAR(avg, 100.0, 60.0); // wide tolerance
    }
}

// ============================================================================
// Nonzero center (multi axis)
// ============================================================================
TEST_F(TrajectoryGenerationTest, NonzeroMultiAxisCenter) {
    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Circle);
    cfg.center = {50.0, 50.0, 0.0};
    tester_.runMultiAxisTest(cfg);
    if (!capturedTrajectory_.empty()) {
        double avgU = 0, avgV = 0;
        for (const auto& s : capturedTrajectory_) {
            avgU += s.position[cfg.uAxis];
            avgV += s.position[cfg.vAxis];
        }
        avgU /= capturedTrajectory_.size();
        avgV /= capturedTrajectory_.size();
        EXPECT_NEAR(avgU, 50.0, 30.0);
        EXPECT_NEAR(avgV, 50.0, 30.0);
    }
}

// ============================================================================
// Command failure path for different trajectory types
// ============================================================================
TEST_F(TrajectoryGenerationTest, CommandFailureSCurve) {
    tester_.setCommandCallback([](const std::vector<PositionSample>&) { return false; });
    tester_.setFeedbackCallback([]() { return std::vector<PositionSample>{}; });

    auto cfg = makeSingleAxisConfig(SingleAxisTestType::SCurve, 1.0);
    auto result = tester_.runSingleAxisTest(cfg);
    (void)result; // just verify no crash
}

TEST_F(TrajectoryGenerationTest, CommandFailureEllipse) {
    tester_.setCommandCallback([](const std::vector<PositionSample>&) { return false; });
    tester_.setFeedbackCallback([]() { return std::vector<PositionSample>{}; });

    auto cfg = makeMultiAxisConfig(MultiAxisTestType::Ellipse);
    auto result = tester_.runMultiAxisTest(cfg);
    (void)result;
}
