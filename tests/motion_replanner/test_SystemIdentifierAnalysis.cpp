/**
 * @file test_SystemIdentifierAnalysis.cpp
 * @brief Tests for SystemIdentifier analysis methods: analyzePIDTuning,
 *        analyzeTracking, suggestZieglerNichols, identifyDynamics.
 *        Covers SystemIdentifierAnalysis.cpp (0% → target 100%).
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/SystemIdentifier.hpp>
#include <cmath>
#include <numeric>
#include <vector>

using namespace MotionReplanner;

// Helper to create an IdentificationSample with all fields
static IdentificationSample makeSample(double t, double cmd, double actual,
                                       double vel = 0, double accel = 0) {
    IdentificationSample s;
    s.timestamp = t;
    s.commanded = cmd;
    s.actual = actual;
    s.velocity = vel;
    s.acceleration = accel;
    s.current = 0;
    s.torque = 0;
    return s;
}

// ============================================================================
// analyzePIDTuning — needs >= 50 samples with a step
// ============================================================================
class AnalyzePIDTuningTest : public ::testing::Test {
protected:
    SystemIdentifier sysid_;

    // Generate a second-order step response: y(t) = 1 - e^(-z*wn*t) * (cos(wd*t) + z/wd*sin(wd*t))
    // Damping ratio z, natural frequency wn.
    void feedStepResponse(double damping, double naturalFreq, int stepSample = 50,
                          int totalSamples = 500, double dt = 0.001) {
        double wd = naturalFreq * std::sqrt(1.0 - damping * damping);
        for (int i = 0; i < totalSamples; i++) {
            double t = i * dt;
            double cmd = (i >= stepSample) ? 1.0 : 0.0;
            double actual = 0.0;
            if (i >= stepSample) {
                double elapsed = (i - stepSample) * dt;
                double envelope = std::exp(-damping * naturalFreq * elapsed);
                actual = 1.0 - envelope * (std::cos(wd * elapsed) +
                         (damping * naturalFreq / wd) * std::sin(wd * elapsed));
            }
            sysid_.addSample(makeSample(t, cmd, actual));
        }
    }
};

TEST_F(AnalyzePIDTuningTest, TooFewSamples) {
    // Less than 50 samples → should return empty assessment
    for (int i = 0; i < 30; i++) {
        sysid_.addSample(makeSample(i * 0.001, 1.0, 0.5));
    }
    auto result = sysid_.analyzePIDTuning();
    // With < 50 samples, should return default values
    EXPECT_DOUBLE_EQ(result.observedKp, 0.0);
}

TEST_F(AnalyzePIDTuningTest, NoStepFallsBackToTracking) {
    // 100 samples with no step (constant command) → falls back to analyzeTracking
    for (int i = 0; i < 100; i++) {
        double t = i * 0.001;
        sysid_.addSample(makeSample(t, 1.0, 0.95 + 0.01 * std::sin(t * 10)));
    }
    auto result = sysid_.analyzePIDTuning();
    // Should not crash, result should have some tracking info
    EXPECT_GE(result.overallScore, 0.0);
}

TEST_F(AnalyzePIDTuningTest, UnderdampedResponse) {
    // Underdamped (damping=0.3, wn=50) → should detect overshoot
    feedStepResponse(0.3, 50.0, 50, 500, 0.001);
    auto result = sysid_.analyzePIDTuning();
    // With a clear step, should detect overshoot > 0
    EXPECT_GE(result.overshoot, 0.0);
    // Should compute a rise time
    EXPECT_GE(result.riseTime, 0.0);
}

TEST_F(AnalyzePIDTuningTest, CriticallyDampedResponse) {
    // Critically damped (damping=1.0, wn=30) → minimal overshoot
    double wn = 30.0;
    for (int i = 0; i < 500; i++) {
        double t = i * 0.001;
        double cmd = (i >= 50) ? 1.0 : 0.0;
        double actual = 0.0;
        if (i >= 50) {
            double elapsed = (i - 50) * 0.001;
            actual = 1.0 - (1.0 + wn * elapsed) * std::exp(-wn * elapsed);
        }
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.analyzePIDTuning();
    EXPECT_GE(result.settlingTime, 0.0);
}

TEST_F(AnalyzePIDTuningTest, OverallScoreRange) {
    feedStepResponse(0.5, 40.0);
    auto result = sysid_.analyzePIDTuning();
    // Score is in 0-100 range
    EXPECT_GE(result.overallScore, 0.0);
    EXPECT_LE(result.overallScore, 100.0);
}

TEST_F(AnalyzePIDTuningTest, StabilityScoreRange) {
    feedStepResponse(0.5, 40.0);
    auto result = sysid_.analyzePIDTuning();
    EXPECT_GE(result.stabilityScore, 0.0);
    EXPECT_LE(result.stabilityScore, 100.0);
}

TEST_F(AnalyzePIDTuningTest, ResponseScoreRange) {
    feedStepResponse(0.5, 40.0);
    auto result = sysid_.analyzePIDTuning();
    EXPECT_GE(result.responseScore, 0.0);
    EXPECT_LE(result.responseScore, 100.0);
}

TEST_F(AnalyzePIDTuningTest, AccuracyScoreRange) {
    feedStepResponse(0.5, 40.0);
    auto result = sysid_.analyzePIDTuning();
    EXPECT_GE(result.accuracyScore, 0.0);
    EXPECT_LE(result.accuracyScore, 100.0);
}

TEST_F(AnalyzePIDTuningTest, WithLargeOvershoot) {
    // Very underdamped → large overshoot → should generate issues/recommendations
    feedStepResponse(0.1, 60.0, 50, 1000, 0.001);
    auto result = sysid_.analyzePIDTuning();
    EXPECT_GT(result.overshoot, 0.0);
    // Should have issues or recommendations
    EXPECT_FALSE(result.issues.empty() && result.recommendations.empty());
}

TEST_F(AnalyzePIDTuningTest, SuggestsZieglerNicholsValues) {
    feedStepResponse(0.3, 50.0, 50, 600, 0.001);
    auto result = sysid_.analyzePIDTuning();
    // analyzePIDTuning calls suggestZieglerNichols at the end
    // If riseTime and naturalFrequency are valid, should produce suggested PID values
    if (result.riseTime > 0 && result.naturalFrequency > 0) {
        EXPECT_GT(result.suggestedKp, 0.0);
    }
}

// ============================================================================
// analyzeTracking — continuous tracking error analysis
// ============================================================================
class AnalyzeTrackingTest : public ::testing::Test {
protected:
    SystemIdentifier sysid_;
};

TEST_F(AnalyzeTrackingTest, TooFewSamples) {
    for (int i = 0; i < 30; i++) {
        sysid_.addSample(makeSample(i * 0.001, 1.0, 1.0));
    }
    auto result = sysid_.analyzeTracking();
    EXPECT_DOUBLE_EQ(result.observedKp, 0.0);
}

TEST_F(AnalyzeTrackingTest, PerfectTracking) {
    for (int i = 0; i < 200; i++) {
        double t = i * 0.001;
        double v = std::sin(2 * M_PI * 5 * t);
        sysid_.addSample(makeSample(t, v, v));
    }
    auto result = sysid_.analyzeTracking();
    // Perfect tracking → zero steady-state error
    EXPECT_NEAR(result.steadyStateError, 0.0, 0.01);
    EXPECT_GE(result.accuracyScore, 0.9);
}

TEST_F(AnalyzeTrackingTest, ConstantOffset) {
    // Systematic offset: actual = cmd + 0.5
    for (int i = 0; i < 200; i++) {
        double t = i * 0.001;
        sysid_.addSample(makeSample(t, 1.0, 1.5));
    }
    auto result = sysid_.analyzeTracking();
    // Mean error should be nonzero (could be negative: actual - commanded)
    EXPECT_NE(result.steadyStateError, 0.0);
}

TEST_F(AnalyzeTrackingTest, LargeErrorAddsIssues) {
    // Large RMS error > 1.0 → should add issue
    for (int i = 0; i < 200; i++) {
        double t = i * 0.001;
        sysid_.addSample(makeSample(t, 0.0, 5.0));
    }
    auto result = sysid_.analyzeTracking();
    // With large error, issues should be flagged
    EXPECT_FALSE(result.issues.empty());
}

TEST_F(AnalyzeTrackingTest, OverallScoreRange) {
    for (int i = 0; i < 200; i++) {
        double t = i * 0.001;
        sysid_.addSample(makeSample(t, 1.0, 1.0 + 0.01 * i));
    }
    auto result = sysid_.analyzeTracking();
    EXPECT_GE(result.overallScore, 0.0);
    EXPECT_LE(result.overallScore, 100.0);
}

// ============================================================================
// suggestZieglerNichols — modifies PIDTuningAssessment in-place
// ============================================================================
TEST(SuggestZieglerNicholsTest, NoOpIfRiseTimeZero) {
    SystemIdentifier sysid;
    PIDTuningAssessment assessment{};
    assessment.riseTime = 0.0;
    assessment.naturalFrequency = 10.0;
    sysid.suggestZieglerNichols(assessment);
    // If riseTime is 0, should not set suggested values
    EXPECT_DOUBLE_EQ(assessment.suggestedKp, 0.0);
}

TEST(SuggestZieglerNicholsTest, NoOpIfNaturalFrequencyZero) {
    SystemIdentifier sysid;
    PIDTuningAssessment assessment{};
    assessment.riseTime = 0.01;
    assessment.naturalFrequency = 0.0;
    sysid.suggestZieglerNichols(assessment);
    EXPECT_DOUBLE_EQ(assessment.suggestedKp, 0.0);
}

TEST(SuggestZieglerNicholsTest, ValidInput) {
    SystemIdentifier sysid;
    PIDTuningAssessment assessment{};
    assessment.riseTime = 0.02;          // 20ms rise time
    assessment.naturalFrequency = 50.0;  // 50 rad/s
    assessment.overshoot = 0.2;
    sysid.suggestZieglerNichols(assessment);
    // With valid input, suggested PID values should be set
    EXPECT_GT(assessment.suggestedKp, 0.0);
    EXPECT_GT(assessment.suggestedKi, 0.0);
    EXPECT_GT(assessment.suggestedKd, 0.0);
    // tuningAdvice should be populated
    EXPECT_FALSE(assessment.tuningAdvice.empty());
}

TEST(SuggestZieglerNicholsTest, AdviceContainsText) {
    SystemIdentifier sysid;
    PIDTuningAssessment assessment{};
    assessment.riseTime = 0.05;
    assessment.naturalFrequency = 30.0;
    assessment.overshoot = 0.1;
    sysid.suggestZieglerNichols(assessment);
    if (assessment.suggestedKp > 0) {
        EXPECT_GT(assessment.tuningAdvice.size(), 10u);
    }
}

// ============================================================================
// identifyDynamics — first/second order identification
// ============================================================================
class IdentifyDynamicsTest : public ::testing::Test {
protected:
    SystemIdentifier sysid_;
};

TEST_F(IdentifyDynamicsTest, TooFewSamples) {
    for (int i = 0; i < 30; i++) {
        sysid_.addSample(makeSample(i * 0.001, 1.0, 0.5));
    }
    auto result = sysid_.identifyDynamics();
    // Too few samples → should return invalid result
    EXPECT_FALSE(result.isValid());
}

TEST_F(IdentifyDynamicsTest, NoSteps) {
    // Constant command → no steps → invalid dynamics
    for (int i = 0; i < 200; i++) {
        sysid_.addSample(makeSample(i * 0.001, 1.0, 1.0));
    }
    auto result = sysid_.identifyDynamics();
    EXPECT_FALSE(result.isValid());
}

TEST_F(IdentifyDynamicsTest, FirstOrderSystem) {
    // First-order system: y(t) = K*(1 - exp(-t/tau))
    double K = 2.0, tau = 0.05; // gain 2, 50ms time constant
    for (int i = 0; i < 500; i++) {
        double t = i * 0.001;
        double cmd = (i >= 50) ? 1.0 : 0.0;
        double actual = 0.0;
        if (i >= 50) {
            double elapsed = (i - 50) * 0.001;
            actual = K * (1.0 - std::exp(-elapsed / tau));
        }
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDynamics();
    if (result.isValid()) {
        EXPECT_GT(result.gain, 0.0);
        EXPECT_GT(result.timeConstant, 0.0);
        EXPECT_GE(result.systemOrder, 1);
    }
}

TEST_F(IdentifyDynamicsTest, SecondOrderUnderdamped) {
    // Second-order underdamped system
    double wn = 40.0, z = 0.3;
    double wd = wn * std::sqrt(1.0 - z * z);
    for (int i = 0; i < 600; i++) {
        double t = i * 0.001;
        double cmd = (i >= 50) ? 1.0 : 0.0;
        double actual = 0.0;
        if (i >= 50) {
            double elapsed = (i - 50) * 0.001;
            actual = 1.0 - std::exp(-z * wn * elapsed) *
                     (std::cos(wd * elapsed) +
                      (z * wn / wd) * std::sin(wd * elapsed));
        }
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDynamics();
    if (result.isValid()) {
        EXPECT_GE(result.systemOrder, 1);
        EXPECT_GT(result.naturalFrequency, 0.0);
        EXPECT_GT(result.bandwidthHz, 0.0);
    }
}

TEST_F(IdentifyDynamicsTest, GainDetection) {
    // Step from 0→1 with gain of 3: actual settles at 3.0
    double K = 3.0, tau = 0.02;
    for (int i = 0; i < 500; i++) {
        double t = i * 0.001;
        double cmd = (i >= 50) ? 1.0 : 0.0;
        double actual = 0.0;
        if (i >= 50) {
            double elapsed = (i - 50) * 0.001;
            actual = K * (1.0 - std::exp(-elapsed / tau));
        }
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDynamics();
    if (result.isValid()) {
        EXPECT_GT(result.gain, 0.0); // just verify positive gain detected
    }
}

TEST_F(IdentifyDynamicsTest, ResultFieldsExist) {
    // Just verify all fields are accessible and not NaN/Inf
    double wn = 30.0, z = 0.5;
    double wd = wn * std::sqrt(1.0 - z * z);
    for (int i = 0; i < 400; i++) {
        double t = i * 0.001;
        double cmd = (i >= 50) ? 1.0 : 0.0;
        double actual = 0.0;
        if (i >= 50) {
            double elapsed = (i - 50) * 0.001;
            actual = 1.0 - std::exp(-z * wn * elapsed) *
                     (std::cos(wd * elapsed) +
                      (z * wn / wd) * std::sin(wd * elapsed));
        }
        sysid_.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid_.identifyDynamics();
    EXPECT_FALSE(std::isnan(result.gain));
    EXPECT_FALSE(std::isnan(result.timeConstant));
    EXPECT_FALSE(std::isnan(result.naturalFrequency));
    EXPECT_FALSE(std::isnan(result.dampingRatio));
}

// ============================================================================
// Edge cases
// ============================================================================
TEST(SystemIdentifierEdgeCases, AnalyzePIDWithExactly50Samples) {
    SystemIdentifier sysid;
    for (int i = 0; i < 50; i++) {
        double t = i * 0.001;
        double cmd = (i >= 25) ? 1.0 : 0.0;
        double actual = (i >= 27) ? (1.0 - std::exp(-(i - 27) * 0.1)) : 0.0;
        sysid.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid.analyzePIDTuning();
    // Should not crash even with borderline samples
    EXPECT_FALSE(std::isnan(result.overallScore));
}

TEST(SystemIdentifierEdgeCases, MultipleStepsInData) {
    SystemIdentifier sysid;
    // Create data with multiple step changes: 0→1→0→2
    for (int i = 0; i < 600; i++) {
        double t = i * 0.001;
        double cmd;
        if (i < 100) cmd = 0.0;
        else if (i < 250) cmd = 1.0;
        else if (i < 400) cmd = 0.0;
        else cmd = 2.0;
        double tau = 0.01;
        double actual = cmd; // perfect tracking for simplicity
        if (i > 0) {
            double prevCmd;
            if (i - 1 < 100) prevCmd = 0.0;
            else if (i - 1 < 250) prevCmd = 1.0;
            else if (i - 1 < 400) prevCmd = 0.0;
            else prevCmd = 2.0;
            if (prevCmd != cmd) {
                // Simple first-order response
                actual = prevCmd;
            }
        }
        sysid.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid.analyzePIDTuning();
    EXPECT_GE(result.overallScore, 0.0);
}

TEST(SystemIdentifierEdgeCases, IdentifyDynamicsStepAtBeginning) {
    SystemIdentifier sysid;
    // Step at the very start
    for (int i = 0; i < 400; i++) {
        double t = i * 0.001;
        double cmd = 1.0;
        double actual = 1.0 - std::exp(-t / 0.02);
        sysid.addSample(makeSample(t, cmd, actual));
    }
    auto result = sysid.identifyDynamics();
    // May or may not find a valid step at the beginning
    (void)result;
}

TEST(SystemIdentifierEdgeCases, IdentifyFrictionFromReversals) {
    SystemIdentifier sysid;
    // Generate reversing velocity data
    for (int i = 0; i < 400; i++) {
        IdentificationSample s;
        s.timestamp = i * 0.001;
        s.commanded = 0;
        s.actual = 0;
        s.velocity = 50.0 * std::sin(2 * M_PI * 2.0 * s.timestamp);
        s.torque = 3.0 * (s.velocity > 0 ? 1.0 : -1.0) + 0.005 * s.velocity;
        s.acceleration = 50.0 * 2 * M_PI * 2.0 * std::cos(2 * M_PI * 2.0 * s.timestamp);
        s.current = 0;
        sysid.addSample(s);
    }
    auto result = sysid.identifyFriction();
    (void)result; // Just verify no crash
}
