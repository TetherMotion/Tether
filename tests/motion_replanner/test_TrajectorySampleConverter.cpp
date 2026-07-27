/**
 * @file test_TrajectorySampleConverter.cpp
 * @brief Tests for TrajectorySample → PiecewiseNurbsPath conversion
 */

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

/// Helper: create a TrajectorySample at (x, y, z) with given segment/type.
TrajectorySample makeSample(double t, double x, double y, double z,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, z, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    return s;
}

/// Helper: create a linear segment from (x0,y0) to (x1,y1) with N samples.
std::vector<TrajectorySample> makeLinearSegment(
    int32_t seg, double x0, double y0, double x1, double y1,
    std::size_t n, double t0 = 0.0) {
    std::vector<TrajectorySample> samples;
    for (std::size_t i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        double x = x0 + alpha * (x1 - x0);
        double y = y0 + alpha * (y1 - y0);
        double pathPos = std::sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0));
        samples.push_back(makeSample(t0 + i * 0.001, x, y, 0.0, seg, 1, pathPos));
    }
    return samples;
}

/// Helper: create an arc segment (CCW) centered at (cx,cy) with radius r,
/// from startAngle to startAngle+sweep, with N samples.
std::vector<TrajectorySample> makeArcSegment(
    int32_t seg, double cx, double cy, double r,
    double startAngle, double sweep, std::size_t n,
    double t0 = 0.0, uint8_t motionType = 3) {
    std::vector<TrajectorySample> samples;
    for (std::size_t i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        double angle = startAngle + alpha * sweep;
        double x = cx + r * std::cos(angle);
        double y = cy + r * std::sin(angle);
        double pathPos = std::abs(r * alpha * sweep);
        samples.push_back(makeSample(t0 + i * 0.001, x, y, 0.0, seg, motionType, pathPos));
    }
    return samples;
}

} // anonymous namespace

//=============================================================================
// Basic conversion tests
//=============================================================================

TEST(TrajectorySampleConverter, EmptyThrows) {
    std::vector<TrajectorySample> empty;
    EXPECT_THROW(convertTrajectory(empty), std::invalid_argument);
}

TEST(TrajectorySampleConverter, AllSamePositionThrows) {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i < 5; ++i) {
        samples.push_back(makeSample(i * 0.001, 10, 20, 0, 0, 1, 0));
    }
    EXPECT_THROW(convertTrajectory(samples), std::invalid_argument);
}

TEST(TrajectorySampleConverter, SingleLineSegment) {
    // Diagonal line so both X and Y are active.
    auto samples = makeLinearSegment(0, 0, 0, 60, 80, 10);
    PiecewiseNurbsPath path = convertTrajectory(samples);

    EXPECT_EQ(path.numPieces(), 1u);
    EXPECT_EQ(path.dim(), 2u); // Both X and Y vary
    EXPECT_NEAR(path.totalLength(), 100.0, 1e-6); // sqrt(60^2+80^2) = 100

    // Start and end positions match
    RVec start = path.evaluatePosition(0.0);
    RVec end = path.evaluatePosition(path.totalLength());
    EXPECT_NEAR(start[0], 0.0, 1e-6);
    EXPECT_NEAR(start[1], 0.0, 1e-6);
    EXPECT_NEAR(end[0], 60.0, 1e-6);
    EXPECT_NEAR(end[1], 80.0, 1e-6);
}

TEST(TrajectorySampleConverter, TwoLineSegments) {
    std::vector<TrajectorySample> samples;
    auto seg0 = makeLinearSegment(0, 0, 0, 50, 0, 5);
    auto seg1 = makeLinearSegment(1, 50, 0, 50, 50, 5, 0.005);
    samples.insert(samples.end(), seg0.begin(), seg0.end());
    samples.insert(samples.end(), seg1.begin(), seg1.end());

    SegmentToPieceMap map;
    PiecewiseNurbsPath path = convertTrajectory(samples, map);

    EXPECT_EQ(path.numPieces(), 2u);
    EXPECT_EQ(path.dim(), 2u);
    EXPECT_NEAR(path.totalLength(), 100.0, 1e-6);

    // Segment-to-piece mapping
    auto p0 = map.pieceForSegment(0);
    auto p1 = map.pieceForSegment(1);
    ASSERT_TRUE(p0.has_value());
    ASSERT_TRUE(p1.has_value());
    EXPECT_EQ(*p0, 0u);
    EXPECT_EQ(*p1, 1u);

    // Nonexistent segment
    EXPECT_FALSE(map.pieceForSegment(99).has_value());
}

TEST(TrajectorySampleConverter, ThreeAxisLine) {
    std::vector<TrajectorySample> samples;
    samples.push_back(makeSample(0, 0, 0, 0, 0, 1, 0));
    samples.push_back(makeSample(1, 30, 40, 50, 0, 1, 70.0));
    // pathPos = sqrt(30^2+40^2+50^2) = sqrt(900+1600+2500) = sqrt(5000) ≈ 70.71

    PiecewiseNurbsPath path = convertTrajectory(samples);
    EXPECT_EQ(path.dim(), 3u);
    EXPECT_NEAR(path.totalLength(), std::sqrt(5000.0), 1e-4);
}

//=============================================================================
// Arc conversion tests
//=============================================================================

TEST(TrajectorySampleConverter, ArcSegmentCCW) {
    // Quarter circle, radius 50, centered at origin, CCW from 0 to 90°.
    auto samples = makeArcSegment(0, 0, 0, 50.0, 0.0, M_PI / 2, 20,
                                  0.0, 3 /*arcCCW*/);

    SegmentToPieceMap map;
    PiecewiseNurbsPath path = convertTrajectory(samples, map);

    EXPECT_EQ(path.numPieces(), 1u);
    EXPECT_EQ(path.dim(), 2u);

    // Arc length = r * |sweep| = 50 * pi/2 ≈ 78.54
    EXPECT_NEAR(path.totalLength(), 50.0 * M_PI / 2, 0.1);

    // Start point: (50, 0)
    RVec start = path.evaluatePosition(0.0);
    EXPECT_NEAR(start[0], 50.0, 0.1);
    EXPECT_NEAR(start[1], 0.0, 0.1);

    // End point: (0, 50)
    RVec end = path.evaluatePosition(path.totalLength());
    EXPECT_NEAR(end[0], 0.0, 0.1);
    EXPECT_NEAR(end[1], 50.0, 0.1);
}

TEST(TrajectorySampleConverter, ArcSegmentCW) {
    // Quarter circle, radius 50, centered at origin, CW from 90° to 0°.
    auto samples = makeArcSegment(0, 0, 0, 50.0, M_PI / 2, -M_PI / 2, 20,
                                  0.0, 2 /*arcCW*/);

    PiecewiseNurbsPath path = convertTrajectory(samples);
    EXPECT_EQ(path.numPieces(), 1u);
    EXPECT_NEAR(path.totalLength(), 50.0 * M_PI / 2, 0.1);

    // Start: (0, 50), End: (50, 0)
    RVec start = path.evaluatePosition(0.0);
    EXPECT_NEAR(start[0], 0.0, 0.1);
    EXPECT_NEAR(start[1], 50.0, 0.1);
    RVec end = path.evaluatePosition(path.totalLength());
    EXPECT_NEAR(end[0], 50.0, 0.1);
    EXPECT_NEAR(end[1], 0.0, 0.1);
}

//=============================================================================
// Mixed path tests
//=============================================================================

TEST(TrajectorySampleConverter, LineArcLinePath) {
    std::vector<TrajectorySample> samples;

    // Line: (0,0) -> (50,0)
    auto seg0 = makeLinearSegment(0, 0, 0, 50, 0, 5);
    samples.insert(samples.end(), seg0.begin(), seg0.end());

    // Arc: CCW quarter circle from (50,0), center (50,50), radius 50
    // Start angle = -90° (pointing down from center), sweep = +90°
    auto seg1 = makeArcSegment(1, 50, 50, 50.0, -M_PI / 2, M_PI / 2, 15, 0.005);
    samples.insert(samples.end(), seg1.begin(), seg1.end());

    // Line: (50, 0) -> (100, 0) — wait, the arc ends at (50,0) going up to (100,50)?
    // Center (50,50), start angle -90° → point (50, 0). End angle 0° → point (100, 50).
    // So arc goes from (50,0) to (100,50).
    // Line: (100, 50) -> (100, 100)
    auto seg2 = makeLinearSegment(2, 100, 50, 100, 100, 5, 0.020);
    samples.insert(samples.end(), seg2.begin(), seg2.end());

    SegmentToPieceMap map;
    PiecewiseNurbsPath path = convertTrajectory(samples, map);

    EXPECT_EQ(path.numPieces(), 3u);
    EXPECT_EQ(path.dim(), 2u);

    // Check junction connectivity
    RVec p0_end = path.piece(0).endPoint();
    RVec p1_start = path.piece(1).startPoint();
    EXPECT_NEAR(p0_end.distanceTo(p1_start), 0.0, 0.5);

    RVec p1_end = path.piece(1).endPoint();
    RVec p2_start = path.piece(2).startPoint();
    EXPECT_NEAR(p1_end.distanceTo(p2_start), 0.0, 0.5);

    // Segment mapping
    EXPECT_EQ(map.entries.size(), 3u);
    EXPECT_EQ(*map.pieceForSegment(0), 0u);
    EXPECT_EQ(*map.pieceForSegment(1), 1u);
    EXPECT_EQ(*map.pieceForSegment(2), 2u);
}

//=============================================================================
// Active-axis extraction tests
//=============================================================================

TEST(TrajectorySampleConverter, OnlyXAxisActive) {
    std::vector<TrajectorySample> samples;
    samples.push_back(makeSample(0, 0, 5, 3, 0, 1, 0));
    samples.push_back(makeSample(1, 100, 5, 3, 0, 1, 100.0));

    PiecewiseNurbsPath path = convertTrajectory(samples);
    EXPECT_EQ(path.dim(), 1u); // Only X varies
    EXPECT_NEAR(path.totalLength(), 100.0, 1e-6);
}

//=============================================================================
// SegmentToPieceMap tests
//=============================================================================

TEST(SegmentToPieceMap, EmptyMap) {
    SegmentToPieceMap map;
    EXPECT_FALSE(map.pieceForSegment(0).has_value());
    EXPECT_FALSE(map.pieceForSegment(-1).has_value());
}

TEST(SegmentToPieceMap, Lookup) {
    SegmentToPieceMap map;
    map.entries = {{0, 0}, {2, 1}, {5, 2}};
    EXPECT_EQ(*map.pieceForSegment(0), 0u);
    EXPECT_EQ(*map.pieceForSegment(2), 1u);
    EXPECT_EQ(*map.pieceForSegment(5), 2u);
    EXPECT_FALSE(map.pieceForSegment(1).has_value());
    EXPECT_FALSE(map.pieceForSegment(3).has_value());
}
