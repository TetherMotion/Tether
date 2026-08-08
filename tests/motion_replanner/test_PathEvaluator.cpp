/**
 * @file test_PathEvaluator.cpp
 * @brief Tests for the PathEvaluator quantitative and qualitative evaluators.
 */

#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion::replanner;
using namespace tether::motion;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.acceleration = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = 0.0;
    return s;
}

/// Build an L-shaped path: (0,0)→(100,0)→(100,100), so both X and Y vary.
std::vector<TrajectorySample> makeLPath() {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 1; i <= 10; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 100.0, i * 10.0,
                                     1, 1, 100.0 + i * 10.0));
    }
    return samples;
}

/// Build a straight line path: (0,0)→(100,10) — both X and Y vary so dim=2.
std::vector<TrajectorySample> makeLinePath() {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 20; ++i) {
        double x = i * 5.0;
        double y = i * 0.5; // Small Y variation to ensure 2D
        double s = std::sqrt(x * x + y * y);
        samples.push_back(makeSample(i * 0.01, x, y, 0, 1, s));
    }
    return samples;
}

/// Build an actual trajectory that perfectly matches the desired path.
std::vector<TrajectorySample> makePerfectActual(
    const std::vector<TrajectorySample>& desired) {
    return desired;
}

/// Build an actual trajectory with a constant lateral offset (contour error).
/// For the diagonal line (0,0)→(100,10), the normal direction is approximately
/// (-0.0995, 0.995), so we offset in that direction.
std::vector<TrajectorySample> makeOffsetActual(
    const std::vector<TrajectorySample>& desired, double offset) {
    std::vector<TrajectorySample> actual = desired;
    // Tangent direction: (100, 10)/sqrt(10100) ≈ (0.995, 0.0995)
    // Normal direction: (-0.0995, 0.995)
    double nx = -0.0995, ny = 0.995;
    for (auto& s : actual) {
        s.position[0] += offset * nx;
        s.position[1] += offset * ny;
    }
    return actual;
}

/// Build an actual trajectory with a constant lag (timing error).
std::vector<TrajectorySample> makeLaggingActual(
    const std::vector<TrajectorySample>& desired, double lagDistance) {
    std::vector<TrajectorySample> actual = desired;
    // Tangent direction: (100, 10)/sqrt(10100) ≈ (0.995, 0.0995)
    double tx = 0.995, ty = 0.0995;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        // Shift the actual position back by lagDistance along the path
        actual[i].position[0] -= lagDistance * tx;
        actual[i].position[1] -= lagDistance * ty;
    }
    return actual;
}

} // anonymous namespace

//=============================================================================
// Quantitative evaluation tests
//=============================================================================

TEST(PathEvaluator, PerfectTracking_ZeroError) {
    auto desired = makeLinePath();
    auto actual = makePerfectActual(desired);

    EvaluatorConfig config;
    config.useCertifiedContourError = false; // Use tangent for simplicity
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    EXPECT_EQ(result.sampleCount, desired.size());
    EXPECT_NEAR(result.norms.linf_contour, 0.0, 1e-6);
    EXPECT_NEAR(result.norms.l2_contour, 0.0, 1e-6);
    EXPECT_NEAR(result.shape.hausdorff, 0.0, 1e-6);
    EXPECT_NEAR(result.shape.frechet, 0.0, 1e-6);
    EXPECT_NEAR(result.shape.pathLengthRatio, 1.0, 1e-6);
}

TEST(PathEvaluator, ConstantOffset_ContourError) {
    auto desired = makeLinePath();
    double offset = 0.5; // 0.5mm lateral offset
    auto actual = makeOffsetActual(desired, offset);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // All contour errors should be approximately the offset
    EXPECT_NEAR(result.norms.linf_contour, offset, 0.1);
    EXPECT_NEAR(result.contourStats.meanError, offset, 0.1);
    EXPECT_GT(result.norms.l2_contour, 0.0);
}

TEST(PathEvaluator, LaggingActual_FollowingError) {
    auto desired = makeLinePath();
    // Add velocity to desired samples (along the path direction)
    for (auto& s : const_cast<std::vector<TrajectorySample>&>(desired)) {
        s.velocity[0] = 99.5;  // ≈ 100 mm/s * cos(atan(0.1))
        s.velocity[1] = 9.95;  // ≈ 100 mm/s * sin(atan(0.1))
    }
    double lag = 1.0; // 1mm lag
    auto actual = makeLaggingActual(desired, lag);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Following error should be approximately the lag distance
    EXPECT_GT(result.following.maxFollowingError, 0.5);
    EXPECT_GT(result.following.meanFollowingError, 0.5);
}

TEST(PathEvaluator, PathLengthRatio) {
    auto desired = makeLinePath();

    // Actual path that oscillates significantly (longer)
    std::vector<TrajectorySample> actual = desired;
    for (std::size_t i = 1; i < actual.size(); ++i) {
        // Add a large oscillation perpendicular to the path
        actual[i].position[0] += 5.0 * std::sin(static_cast<double>(i) * 0.8);
        actual[i].position[1] += 5.0 * std::cos(static_cast<double>(i) * 0.8);
    }

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Path length ratio should be > 1 (actual is longer due to oscillation)
    EXPECT_GT(result.shape.pathLengthRatio, 1.01);
}

TEST(PathEvaluator, IntegralMetrics) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // IAE should be positive (integral of |error| over path length)
    EXPECT_GT(result.integrals.iae_s, 0.0);
    // ISE should be positive (integral of error² over path length)
    EXPECT_GT(result.integrals.ise_s, 0.0);
    // IAE should be approximately offset * pathLength
    double expectedIAE = 0.5 * result.pathLength;
    EXPECT_NEAR(result.integrals.iae_s, expectedIAE, expectedIAE * 0.2);
}

TEST(PathEvaluator, NormMetrics) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // L∞ should be the max error
    EXPECT_NEAR(result.norms.linf_contour, 0.5, 0.1);
    // L2 should be less than L∞ * sqrt(pathLength)
    EXPECT_LT(result.norms.l2_contour, result.norms.linf_contour * std::sqrt(result.pathLength));
    // L1 should be less than L∞ * pathLength
    EXPECT_LT(result.norms.l1_contour, result.norms.linf_contour * result.pathLength);
}

TEST(PathEvaluator, SurfaceFinishMetrics) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Ra should be approximately 0.5mm = 500µm
    EXPECT_NEAR(result.surface.ra, 500.0, 100.0);
    // Rq should be >= Ra (RMS >= mean)
    EXPECT_GE(result.surface.rq, result.surface.ra * 0.9);
}

TEST(PathEvaluator, ShapeDistances) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Hausdorff should be approximately the offset
    EXPECT_NEAR(result.shape.hausdorff, 0.5, 0.1);
    // Frechet should be approximately the offset
    EXPECT_NEAR(result.shape.frechet, 0.5, 0.1);
    // DTW should be positive
    EXPECT_GT(result.shape.dtw, 0.0);
}

TEST(PathEvaluator, KinematicMetrics) {
    auto desired = makeLinePath();

    // Add velocity to desired
    for (auto& s : const_cast<std::vector<TrajectorySample>&>(desired)) {
        s.velocity[0] = 100.0; // 100 mm/s
    }

    // Actual with different velocity
    std::vector<TrajectorySample> actual = desired;
    for (auto& s : actual) {
        s.velocity[0] = 90.0; // 90 mm/s (10 mm/s error)
    }

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Velocity tracking error should be approximately 10 mm/s
    EXPECT_NEAR(result.kinematic.velocityTrackingRms, 10.0, 2.0);
    EXPECT_NEAR(result.kinematic.velocityTrackingMax, 10.0, 2.0);
}

TEST(PathEvaluator, EmptyInput_ReturnsEmpty) {
    std::vector<TrajectorySample> empty;

    PathEvaluator evaluator;
    auto result = evaluator.evaluateQuantitative(empty, empty);

    EXPECT_EQ(result.sampleCount, 0u);
}

TEST(PathEvaluator, LPath_CornerDetection) {
    auto desired = makeLPath();
    auto actual = makePerfectActual(desired);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto result = evaluator.evaluateQuantitative(desired, actual);

    // Perfect tracking should give small error relative to path size
    // (NURBS may smooth the 90° corner, causing some approximation error)
    EXPECT_LT(result.norms.linf_contour, 10.0);
    EXPECT_LT(result.shape.hausdorff, 10.0);
}

//=============================================================================
// Qualitative evaluation tests
//=============================================================================

TEST(PathEvaluator, Qualitative_PerfectTracking_GradeA) {
    auto desired = makeLinePath();
    auto actual = makePerfectActual(desired);

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto quant = evaluator.evaluateQuantitative(desired, actual);
    auto qual = evaluator.evaluateQualitative(quant, nullptr);

    EXPECT_EQ(qual.pathFidelity.grade, Grade::A);
    EXPECT_EQ(qual.overall.grade, Grade::A);
    EXPECT_NEAR(qual.overall.score, 1.0, 0.1);
}

TEST(PathEvaluator, Qualitative_PoorTracking_LowerGrade) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 1.0); // 1mm offset

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    // Set tight thresholds so 1mm error gets a poor grade
    config.contourErrorThresholdA = 0.001;
    config.contourErrorThresholdB = 0.01;
    config.contourErrorThresholdC = 0.1;
    config.contourErrorThresholdD = 0.5;
    PathEvaluator evaluator(config);

    auto quant = evaluator.evaluateQuantitative(desired, actual);
    auto qual = evaluator.evaluateQualitative(quant, nullptr);

    // 1mm error should get grade F with these thresholds
    EXPECT_EQ(qual.pathFidelity.grade, Grade::F);
    EXPECT_NE(qual.overall.grade, Grade::A);
}

TEST(PathEvaluator, Qualitative_DiagnosticMessages) {
    auto desired = makeLinePath();

    // Actual path significantly longer (oscillation)
    std::vector<TrajectorySample> actual = desired;
    for (std::size_t i = 1; i < actual.size(); ++i) {
        actual[i].position[1] = 2.0 * std::sin(static_cast<double>(i) * 1.0);
    }

    EvaluatorConfig config;
    config.useCertifiedContourError = false;
    PathEvaluator evaluator(config);

    auto quant = evaluator.evaluateQuantitative(desired, actual);
    auto qual = evaluator.evaluateQualitative(quant, nullptr);

    // Should have diagnostic messages about path length ratio
    EXPECT_FALSE(qual.diagnosticMessages.empty());
}

TEST(PathEvaluator, GradeToString) {
    EXPECT_EQ(gradeToString(Grade::A), "A");
    EXPECT_EQ(gradeToString(Grade::B), "B");
    EXPECT_EQ(gradeToString(Grade::C), "C");
    EXPECT_EQ(gradeToString(Grade::D), "D");
    EXPECT_EQ(gradeToString(Grade::F), "F");
}

TEST(PathEvaluator, GradeToScore) {
    EvaluatorConfig config;
    PathEvaluator evaluator(config);

    EXPECT_NEAR(evaluator.gradeToScore(Grade::A), 1.0, 1e-6);
    EXPECT_NEAR(evaluator.gradeToScore(Grade::B), 0.8, 1e-6);
    EXPECT_NEAR(evaluator.gradeToScore(Grade::C), 0.6, 1e-6);
    EXPECT_NEAR(evaluator.gradeToScore(Grade::D), 0.4, 1e-6);
    EXPECT_NEAR(evaluator.gradeToScore(Grade::F), 0.2, 1e-6);
}
