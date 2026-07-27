/**
 * @file test_CertifiedCornerDetection.cpp
 * @brief Tests for certified corner detection via CornerAnalyzer
 */

#include "tether/motion_replanner/CertifiedCornerDetection.hpp"
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
// Basic tests
//=============================================================================

TEST(CertifiedCornerDetection, SinglePieceThrows) {
    std::vector<TrajectorySample> samples;
    samples.push_back(makeSample(0, 0, 0, 0, 1));
    samples.push_back(makeSample(1, 100, 50, 0, 1, 100.0));
    PiecewiseNurbsPath path = convertTrajectory(samples);

    EXPECT_THROW(detectCorners(path), std::invalid_argument);
}

TEST(CertifiedCornerDetection, StraightJunction) {
    // Two collinear segments: (0,0)→(50,0)→(100,0)
    // Both X and Y must vary for dim=2, so use a slight Y offset that
    // keeps the junction straight. Actually, for a straight junction we
    // need the tangents to align. Use (0,0)→(50,0)→(100,0) but with a
    // tiny Y jiggle in the second segment to keep dim=2.
    // Better: use (0,0)→(50,10)→(100,20) — collinear, dim=2.
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        double x = i * 10.0;
        double y = i * 2.0;
        samples.push_back(makeSample(i * 0.01, x, y, 0, 1, i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        double x = 50.0 + i * 10.0;
        double y = 10.0 + i * 2.0;
        samples.push_back(makeSample((5 + i) * 0.01, x, y, 1, 1, 50.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 2u);

    CertifiedCornerDetection detection = detectCorners(path);
    EXPECT_EQ(detection.junctions.size(), 1u);
    EXPECT_EQ(detection.straightCount, 1);
    EXPECT_EQ(detection.cornerCount, 0);
    EXPECT_EQ(detection.cuspCount, 0);
    EXPECT_EQ(detection.junctions[0].kindString(), "Straight");
}

TEST(CertifiedCornerDetection, RightAngleCorner) {
    // L-shaped path: (0,0)→(50,0)→(50,50) — 90° corner at (50,0)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 2u);

    CertifiedCornerDetection detection = detectCorners(path);
    EXPECT_EQ(detection.junctions.size(), 1u);
    EXPECT_EQ(detection.cornerCount, 1);
    EXPECT_EQ(detection.junctions[0].kindString(), "Corner");

    // The angle should be ~90° = pi/2
    EXPECT_NEAR(detection.junctions[0].analysis.angleRad, M_PI / 2, 1e-3);

    // Tangent in: (1, 0), tangent out: (0, 1)
    EXPECT_NEAR(detection.junctions[0].analysis.tangentIn[0], 1.0, 1e-3);
    EXPECT_NEAR(detection.junctions[0].analysis.tangentIn[1], 0.0, 1e-3);
    EXPECT_NEAR(detection.junctions[0].analysis.tangentOut[0], 0.0, 1e-3);
    EXPECT_NEAR(detection.junctions[0].analysis.tangentOut[1], 1.0, 1e-3);

    // Vertex at (50, 0)
    EXPECT_NEAR(detection.junctions[0].analysis.vertex[0], 50.0, 1e-3);
    EXPECT_NEAR(detection.junctions[0].analysis.vertex[1], 0.0, 1e-3);
}

TEST(CertifiedCornerDetection, CuspDetection) {
    // Path reverses: (0,0)→(50,0)→(0,0) — 180° cusp at (50,0)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    // Second segment: (50,0)→(0,0) but with a tiny Y offset to keep dim=2
    // and avoid the start==end degenerate case. Actually the converter
    // builds a line from first to last sample, so (50,0)→(0,0) is fine
    // as long as Y varies somewhere. Use (50,0)→(0,0.001).
    for (int i = 0; i <= 5; ++i) {
        double frac = static_cast<double>(i) / 5.0;
        samples.push_back(makeSample((5 + i) * 0.01,
                                     50.0 - frac * 50.0,
                                     frac * 0.001,
                                     1, 1, 50.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 2u);

    CertifiedCornerDetection detection = detectCorners(path);
    EXPECT_EQ(detection.junctions.size(), 1u);
    // The angle is very close to 180° (π), so it should be classified as Cusp.
    EXPECT_EQ(detection.cuspCount, 1);
    EXPECT_EQ(detection.junctions[0].kindString(), "Cusp");
}

TEST(CertifiedCornerDetection, MultipleJunctions) {
    // Z-shaped path: (0,0)→(50,0)→(50,50)→(100,50)
    // Two 90° corners at (50,0) and (50,50)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 50.0 + i * 10.0, 50.0,
                                     2, 1, 100.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 3u);

    CertifiedCornerDetection detection = detectCorners(path);
    EXPECT_EQ(detection.junctions.size(), 2u);
    EXPECT_EQ(detection.cornerCount, 2);
    EXPECT_EQ(detection.straightCount, 0);
    EXPECT_EQ(detection.cuspCount, 0);

    // Both angles should be ~90°
    EXPECT_NEAR(detection.junctions[0].analysis.angleRad, M_PI / 2, 1e-3);
    EXPECT_NEAR(detection.junctions[1].analysis.angleRad, M_PI / 2, 1e-3);
}

TEST(CertifiedCornerDetection, JunctionAtLookup) {
    // 3-piece path
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 50.0 + i * 10.0, 50.0,
                                     2, 1, 100.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    CertifiedCornerDetection detection = detectCorners(path);

    // Look up junction at piece 0 (between piece 0 and 1)
    auto j0 = detection.junctionAt(0);
    ASSERT_TRUE(j0.has_value());
    EXPECT_EQ(j0->pieceInIndex, 0u);
    EXPECT_EQ(j0->pieceOutIndex, 1u);

    // Look up junction at piece 1 (between piece 1 and 2)
    auto j1 = detection.junctionAt(1);
    ASSERT_TRUE(j1.has_value());
    EXPECT_EQ(j1->pieceInIndex, 1u);
    EXPECT_EQ(j1->pieceOutIndex, 2u);

    // No junction at piece 2 (last piece)
    EXPECT_FALSE(detection.junctionAt(2).has_value());
}

TEST(CertifiedCornerDetection, PlaneBasisValid) {
    // 90° corner — the plane basis should be orthonormal.
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 10.0, 0, 0, 1, i * 10.0));
    }
    for (int i = 0; i <= 5; ++i) {
        samples.push_back(makeSample((5 + i) * 0.01, 50.0, i * 10.0,
                                     1, 1, 50.0 + i * 10.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    CertifiedCornerDetection detection = detectCorners(path);

    const auto& j = detection.junctions[0];
    // e1 should be the unit tangent in: (1, 0)
    EXPECT_NEAR(j.analysis.planeE1[0], 1.0, 1e-3);
    EXPECT_NEAR(j.analysis.planeE1[1], 0.0, 1e-3);
    // e2 should be perpendicular to e1, in the direction of t_out's
    // component perpendicular to e1. t_out = (0, 1), so e2 = (0, 1).
    EXPECT_NEAR(j.analysis.planeE2[0], 0.0, 1e-3);
    EXPECT_NEAR(j.analysis.planeE2[1], 1.0, 1e-3);
    // Orthonormality: e1·e2 = 0, |e1| = |e2| = 1
    EXPECT_NEAR(j.analysis.planeE1.dot(j.analysis.planeE2), 0.0, 1e-6);
    EXPECT_NEAR(j.analysis.planeE1.norm(), 1.0, 1e-6);
    EXPECT_NEAR(j.analysis.planeE2.norm(), 1.0, 1e-6);
}
