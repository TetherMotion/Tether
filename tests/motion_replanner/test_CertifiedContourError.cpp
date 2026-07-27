/**
 * @file test_CertifiedContourError.cpp
 * @brief Tests for certified contour error via pointCurveDistance
 */

#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

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
    s.segmentIndex = seg;
    s.motionType = motionType;
    return s;
}

} // anonymous namespace

//=============================================================================
// Line path tests — use L-shaped paths so both X and Y are active (dim=2)
//=============================================================================

namespace {
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
} // anonymous namespace

TEST(CertifiedContourError, PointOnLine) {
    PiecewiseNurbsPath path = convertTrajectory(makeLPath());

    // Point exactly on the first segment at (50, 0).
    RVec actual = RVec::zero(2);
    actual[0] = 50.0;
    actual[1] = 0.0;

    CertifiedContourError err = computeCertifiedContourError(path, actual, 50.0);

    EXPECT_NEAR(err.contourError, 0.0, 1e-6);
    EXPECT_NEAR(err.combinedPositionError, 0.0, 1e-6);
    EXPECT_NEAR(err.lagError, 0.0, 1e-6);
    EXPECT_TRUE(err.isCertified);
}

TEST(CertifiedContourError, PointOffsetFromLine) {
    PiecewiseNurbsPath path = convertTrajectory(makeLPath());

    // Point at (50, 5) — 5mm perpendicular offset from the first segment.
    RVec actual = RVec::zero(2);
    actual[0] = 50.0;
    actual[1] = 5.0;

    CertifiedContourError err = computeCertifiedContourError(path, actual, 50.0);

    // Contour error = 5.0 (perpendicular distance to the line).
    EXPECT_NEAR(err.contourError, 5.0, 1e-4);
    // Closest point should be at (50, 0).
    EXPECT_NEAR(err.closestPoint[0], 50.0, 1e-4);
    EXPECT_NEAR(err.closestPoint[1], 0.0, 1e-4);
    // Lag error ≈ 0 (closest point is at the same arc length).
    EXPECT_NEAR(err.lagError, 0.0, 1e-4);
    // Combined position error = 5.0 (distance to desired (50,0)).
    EXPECT_NEAR(err.combinedPositionError, 5.0, 1e-4);
}

TEST(CertifiedContourError, PointWithLagAndContour) {
    PiecewiseNurbsPath path = convertTrajectory(makeLPath());

    // Point at (40, 3) — desired at arc length 50.
    // Closest point on first segment: (40, 0), arc length 40.
    // Lag = 40 - 50 = -10 (behind).
    // Contour = 3 (perpendicular).
    RVec actual = RVec::zero(2);
    actual[0] = 40.0;
    actual[1] = 3.0;

    CertifiedContourError err = computeCertifiedContourError(path, actual, 50.0);

    EXPECT_NEAR(err.contourError, 3.0, 1e-4);
    EXPECT_NEAR(err.lagError, -10.0, 1e-3);
    // Combined = distance from (40,3) to (50,0) = sqrt(100+9) = sqrt(109)
    EXPECT_NEAR(err.combinedPositionError, std::sqrt(109.0), 1e-4);
}

//=============================================================================
// Arc path tests — the key test showing tangent-projection is wrong
//=============================================================================

TEST(CertifiedContourError, PointOffsetFromArc) {
    // CCW quarter arc, radius 50, center (0,0), from 0° to 90°.
    // Points: (50,0) → (0,50).
    std::vector<TrajectorySample> samples;
    int n = 20;
    for (int i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        double angle = alpha * M_PI / 2;
        double x = 50.0 * std::cos(angle);
        double y = 50.0 * std::sin(angle);
        double pathPos = 50.0 * alpha * M_PI / 2;
        samples.push_back(makeSample(i * 0.01, x, y, 0, 3, pathPos));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);

    // Point at radius 55, angle 45° — 5mm radial offset from the arc.
    double angle45 = M_PI / 4;
    RVec actual = RVec::zero(2);
    actual[0] = 55.0 * std::cos(angle45);
    actual[1] = 55.0 * std::sin(angle45);

    // Desired arc length: 45° = pi/4, arc length = 50 * pi/4 ≈ 39.27
    double desiredS = 50.0 * M_PI / 4;

    CertifiedContourError err = computeCertifiedContourError(path, actual, desiredS);

    // Contour error should be 5.0 (radial offset).
    // The OLD tangent-projection method would give a different (wrong) answer
    // because the tangent at 45° is not perpendicular to the radial direction
    // in the same way as on a line.
    EXPECT_NEAR(err.contourError, 5.0, 0.5);
    // Lag error should be near 0 (the closest point is at the same angle).
    EXPECT_NEAR(err.lagError, 0.0, 1.0);
}

//=============================================================================
// Multi-piece path tests
//=============================================================================

TEST(CertifiedContourError, CornerPath) {
    // Two line segments forming an L: (0,0)→(50,0)→(50,50)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 1; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 2u);

    // Point at (45, 5) — near the corner at (50, 0).
    // Closest point on piece 0 (line (0,0)→(50,0)): (45, 0), distance = 5.
    // Closest point on piece 1 (line (50,0)→(50,50)): (50, 5), distance = 5.
    // Both give the same distance; the certified method should find 5.0.
    RVec actual = RVec::zero(2);
    actual[0] = 45.0;
    actual[1] = 5.0;

    CertifiedContourError err = computeCertifiedContourError(path, actual, 45.0);

    EXPECT_NEAR(err.contourError, 5.0, 1e-3);
}

//=============================================================================
// Local search tests
//=============================================================================

TEST(CertifiedContourError, LocalSearchMatchesGlobal) {
    // Three-segment path: (0,0)→(100,0)→(100,100)→(200,100)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 1; i <= 10; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 100.0, i * 10.0,
                                     1, 1, 100.0 + i * 10.0));
    }
    for (int i = 1; i <= 10; ++i) {
        samples.push_back(makeSample((20 + i) * 0.01, 100.0 + i * 10.0, 100.0,
                                     2, 1, 200.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 3u);

    // Point near the middle of piece 1, offset by 3mm.
    RVec actual = RVec::zero(2);
    actual[0] = 103.0;
    actual[1] = 50.0;

    double desiredS = 150.0; // Middle of piece 1

    CertifiedContourError globalErr = computeCertifiedContourError(path, actual, desiredS);
    CertifiedContourError localErr = computeCertifiedContourErrorLocal(path, actual, desiredS, 1);

    // Both should find the same closest point on piece 1.
    EXPECT_NEAR(globalErr.contourError, localErr.contourError, 1e-6);
    EXPECT_NEAR(globalErr.contourError, 3.0, 1e-3);
}

//=============================================================================
// Dimension mismatch
//=============================================================================

TEST(CertifiedContourError, DimensionMismatchThrows) {
    PiecewiseNurbsPath path = convertTrajectory(makeLPath()); // dim=2

    RVec actual = RVec::zero(3); // Wrong dimension
    actual[0] = 50.0;
    actual[1] = 0.0;
    actual[2] = 0.0;

    EXPECT_THROW(computeCertifiedContourError(path, actual, 50.0),
                 std::invalid_argument);
}
