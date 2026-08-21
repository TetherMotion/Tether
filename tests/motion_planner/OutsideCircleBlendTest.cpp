/**
 * @file OutsideCircleBlendTest.cpp
 * @brief Tests for the OutsideCircleBlender (negative G64 outside blend).
 */

#include "tether/motion_planner/blend/OutsideCircleBlender.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

using namespace tether::motion;

// ============================================================================
// Helper: create a simple L-shaped path (two line segments forming a corner)
// ============================================================================
static PiecewiseNurbsPath makeLPath(double len1, double len2,
                                     double cornerAngle) {
    // First segment: from (0,0,0) to (len1,0,0)
    RVec A{0.0, 0.0, 0.0};
    RVec B{len1, 0.0, 0.0};

    // Second segment: from B at angle cornerAngle
    RVec C{len1 + len2 * std::cos(cornerAngle),
           len2 * std::sin(cornerAngle), 0.0};

    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(A, B));
    pieces.push_back(NurbsCurve::fromLine(B, C));
    return PiecewiseNurbsPath(std::move(pieces));
}

// ============================================================================
// Helper: create a square path with 4 corners
// ============================================================================
static PiecewiseNurbsPath makeSquarePath(double side) {
    RVec A{0.0, 0.0, 0.0};
    RVec B{side, 0.0, 0.0};
    RVec C{side, side, 0.0};
    RVec D{0.0, side, 0.0};
    RVec E{0.0, 0.0, 0.0};  // close the square

    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(A, B));
    pieces.push_back(NurbsCurve::fromLine(B, C));
    pieces.push_back(NurbsCurve::fromLine(C, D));
    pieces.push_back(NurbsCurve::fromLine(D, E));
    return PiecewiseNurbsPath(std::move(pieces));
}

// ============================================================================
// Tests
// ============================================================================

TEST(OutsideCircleBlendTest, SquareCorner90Degrees) {
    // Square path with 4 pieces (3 corners, since it's an open path)
    auto path = makeSquarePath(100.0);
    EXPECT_EQ(path.numPieces(), 4u);

    OutsideCircleBlendConfig config;
    config.radius = 5.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_GT(result.blendedCount, 0);
    EXPECT_EQ(result.skippedCount, 0);

    // 4 pieces → 3 junctions → 3 arcs + 4 trimmed lines = 7 pieces
    EXPECT_EQ(result.path->numPieces(), 7u);

    // Check that the blend arcs are degree-2 (rational quadratic arcs)
    int arcCount = 0;
    int lineCount = 0;
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        const auto& p = result.path->piece(i);
        if (p.degree() == 2) {
            arcCount++;
            // Arc radius should be ≈ 5.0
            RVec center, axis1, axis2;
            double radius;
            ASSERT_TRUE(OutsideCircleBlender::extractCircleFromArc(
                p, center, radius, axis1, axis2));
            EXPECT_NEAR(radius, 5.0, 0.1);
        } else if (p.degree() == 1) {
            lineCount++;
        }
    }
    EXPECT_EQ(arcCount, 3);
    EXPECT_EQ(lineCount, 4);

    // Total length: 3 outside arcs (270° at r=5) + trimmed lines
    // Lines: 95 + 90 + 90 + 95 = 370
    // Arcs: 3 × (5 × 3π/2) = 3 × 23.56 = 70.69
    // Total = 370 + 70.69 = 440.69
    double expectedArcLen = 5.0 * 3.0 * M_PI / 2.0;  // 270° at r=5
    double expectedTotal = 95.0 + 90.0 + 90.0 + 95.0 + 3 * expectedArcLen;
    EXPECT_NEAR(result.path->totalLength(), expectedTotal, 1.0);
}

TEST(OutsideCircleBlendTest, LPath45Degrees) {
    // L-shaped path with a 45° corner, blend radius = 3
    auto path = makeLPath(50.0, 50.0, M_PI / 4.0);  // 45° turn
    EXPECT_EQ(path.numPieces(), 2u);

    OutsideCircleBlendConfig config;
    config.radius = 3.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.blendedCount, 1);
    EXPECT_EQ(result.skippedCount, 0);

    // Should have 3 pieces: trimmed line, arc, trimmed line
    EXPECT_EQ(result.path->numPieces(), 3u);

    // The arc should be degree-2
    int arcCount = 0;
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        if (result.path->piece(i).degree() == 2) arcCount++;
    }
    EXPECT_EQ(arcCount, 1);

    // Check G0 continuity (endpoints match)
    EXPECT_TRUE(result.path->isG0Connected(1e-6));
}

TEST(OutsideCircleBlendTest, NoBlendWhenRadiusTooLarge) {
    // Path with very short segments — radius too large to fit
    auto path = makeLPath(2.0, 2.0, M_PI / 2.0);  // 90° corner, 2mm segments
    EXPECT_EQ(path.numPieces(), 2u);

    OutsideCircleBlendConfig config;
    config.radius = 10.0;  // Much larger than segment length
    config.maxTrimFraction = 0.5;  // Can only trim 1mm from each side

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    // Should skip the corner (can't trim enough)
    EXPECT_EQ(result.blendedCount, 0);
    EXPECT_EQ(result.skippedCount, 1);
}

TEST(OutsideCircleBlendTest, CollinearNoBlend) {
    // Collinear segments — no corner to blend
    RVec A{0.0, 0.0, 0.0};
    RVec B{50.0, 0.0, 0.0};
    RVec C{100.0, 0.0, 0.0};

    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(A, B));
    pieces.push_back(NurbsCurve::fromLine(B, C));
    PiecewiseNurbsPath path(std::move(pieces));

    OutsideCircleBlendConfig config;
    config.radius = 5.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.blendedCount, 0);
    // Collinear is not a corner — should be skipped
    EXPECT_EQ(result.skippedCount, 1);
}

TEST(OutsideCircleBlendTest, G0ContinuityAfterBlend) {
    // Verify that the blended path is G0 continuous (endpoints match)
    auto path = makeSquarePath(50.0);

    OutsideCircleBlendConfig config;
    config.radius = 2.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_TRUE(result.path->isG0Connected(1e-6));
}

TEST(OutsideCircleBlendTest, ArcEndpointMatchesTrimmedLine) {
    // For a 90° corner: verify that the arc starts where the trimmed line ends
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);  // 90° corner

    OutsideCircleBlendConfig config;
    config.radius = 10.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.path->numPieces(), 3u);

    // Piece 0: trimmed line (0,0) → (90,0)
    // Piece 1: arc from (90,0) around (100,0) to (100,10)
    // Piece 2: trimmed line (100,10) → (100,100)

    const auto& p0 = result.path->piece(0);
    const auto& p1 = result.path->piece(1);
    const auto& p2 = result.path->piece(2);

    // Check endpoints
    RVec p0end = p0.endPoint();
    RVec p1start = p1.startPoint();
    RVec p1end = p1.endPoint();
    RVec p2start = p2.startPoint();

    // G0 continuity
    EXPECT_NEAR(p0end.distanceTo(p1start), 0.0, 1e-6);
    EXPECT_NEAR(p1end.distanceTo(p2start), 0.0, 1e-6);

    // The arc endpoints should be at distance r=10 from the corner (100,0)
    RVec corner{100.0, 0.0, 0.0};
    EXPECT_NEAR(p1start.distanceTo(corner), 10.0, 0.1);
    EXPECT_NEAR(p1end.distanceTo(corner), 10.0, 0.1);

    // The trimmed line should end at (90, 0)
    EXPECT_NEAR(p0end[0], 90.0, 0.1);
    EXPECT_NEAR(p0end[1], 0.0, 0.1);

    // The second trimmed line should start at (100, 10)
    EXPECT_NEAR(p2start[0], 100.0, 0.1);
    EXPECT_NEAR(p2start[1], 10.0, 0.1);
}

TEST(OutsideCircleBlendTest, ArcIsOutsideMajorArc) {
    // Verify the arc is the MAJOR arc (outside), not the minor arc (inside)
    // For a 90° corner, the major arc sweeps 270°
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);

    OutsideCircleBlendConfig config;
    config.radius = 10.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.blendedCount, 1);

    // Find the arc piece
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        const auto& p = result.path->piece(i);
        if (p.degree() == 2) {
            // Arc length should be the major arc: r × (2π - π/2) = r × 3π/2
            double expectedMajor = 10.0 * 3.0 * M_PI / 2.0;
            double expectedMinor = 10.0 * M_PI / 2.0;
            double arcLen = p.length();

            // Should be close to the major arc, not the minor arc
            EXPECT_NEAR(arcLen, expectedMajor, 0.5)
                << "Arc length " << arcLen
                << " should be major arc " << expectedMajor
                << " not minor arc " << expectedMinor;
        }
    }
}

TEST(OutsideCircleBlendTest, MultipleCornersChain) {
    // Zigzag path with multiple corners
    std::vector<NurbsCurve> pieces;
    RVec prev{0.0, 0.0, 0.0};
    for (int i = 0; i < 6; ++i) {
        double x = (i + 1) * 30.0;
        double y = (i % 2 == 0) ? 20.0 : 0.0;
        RVec curr{x, y, 0.0};
        pieces.push_back(NurbsCurve::fromLine(prev, curr));
        prev = curr;
    }
    PiecewiseNurbsPath path(std::move(pieces));
    EXPECT_EQ(path.numPieces(), 6u);

    OutsideCircleBlendConfig config;
    config.radius = 3.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_GT(result.blendedCount, 0);
    EXPECT_TRUE(result.path->isG0Connected(1e-6));
}

TEST(OutsideCircleBlendTest, ZeroRadiusNoBlend) {
    auto path = makeSquarePath(50.0);

    OutsideCircleBlendConfig config;
    config.radius = 0.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.blendedCount, 0);
    // Path should be unchanged
    EXPECT_EQ(result.path->numPieces(), 4u);
}

TEST(OutsideCircleBlendTest, SinglePieceNoBlend) {
    // Single piece — no corners to blend
    RVec A{0.0, 0.0, 0.0};
    RVec B{100.0, 0.0, 0.0};
    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(A, B));
    PiecewiseNurbsPath path(std::move(pieces));

    OutsideCircleBlendConfig config;
    config.radius = 5.0;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.blendedCount, 0);
}

TEST(OutsideCircleBlendTest, ExtractCircleFromArc) {
    // Create a known arc and verify circle extraction
    RVec center{50.0, 50.0, 0.0};
    double radius = 20.0;
    RVec axis1{1.0, 0.0, 0.0};
    RVec axis2{0.0, 1.0, 0.0};
    double startAngle = 0.0;
    double sweepAngle = M_PI / 2.0;  // 90° arc

    auto arc = NurbsCurve::fromArc(center, radius, axis1, axis2,
                                    startAngle, sweepAngle);

    RVec extractedCenter, extractedAxis1, extractedAxis2;
    double extractedRadius;
    ASSERT_TRUE(OutsideCircleBlender::extractCircleFromArc(
        arc, extractedCenter, extractedRadius,
        extractedAxis1, extractedAxis2));

    EXPECT_NEAR(extractedRadius, radius, 0.01);
    EXPECT_NEAR(extractedCenter.distanceTo(center), 0.0, 0.01);
}

TEST(OutsideCircleBlendTest, LineCircleIntersectionAnalytical) {
    // Line from (0,0,0) to (100,0,0), circle center at (100,0,0) radius 10
    // Intersection should be at (90, 0, 0) → s = 90
    RVec A{0.0, 0.0, 0.0};
    RVec B{100.0, 0.0, 0.0};
    auto line = NurbsCurve::fromLine(A, B);

    RVec center{100.0, 0.0, 0.0};
    auto sOpt = OutsideCircleBlender::lineCircleIntersection(
        line, center, 10.0, true);

    ASSERT_TRUE(sOpt.has_value());
    EXPECT_NEAR(*sOpt, 90.0, 1e-6);
}

TEST(OutsideCircleBlendTest, ArcCircleIntersection) {
    // Arc from (70, 50, 0) to (50, 70, 0) — part of circle centered at
    // (50, 50, 0) with radius 20, sweeping 90° from 0 to π/2.
    // Blend circle centered at (70, 50, 0) with radius 10.
    // The intersection should be where the arc crosses the blend circle.

    RVec center{50.0, 50.0, 0.0};
    double radius = 20.0;
    RVec axis1{1.0, 0.0, 0.0};
    RVec axis2{0.0, 1.0, 0.0};

    auto arc = NurbsCurve::fromArc(center, radius, axis1, axis2,
                                    0.0, M_PI / 2.0);

    // The arc starts at (70, 50, 0) and ends at (50, 70, 0).
    // Blend circle centered at the arc's start point (70, 50, 0) with r=10.
    RVec blendCenter{70.0, 50.0, 0.0};
    double blendRadius = 10.0;

    auto sOpt = OutsideCircleBlender::arcCircleIntersection(
        arc, blendCenter, blendRadius, false);

    // There should be an intersection near the start of the arc
    ASSERT_TRUE(sOpt.has_value());
    EXPECT_GE(*sOpt, 0.0);
    EXPECT_LE(*sOpt, arc.length());

    // Verify: evaluate the arc at s and check distance to blend center
    double u = arc.invertLength(*sOpt);
    RVec p = arc.evaluate(u);
    EXPECT_NEAR(p.distanceTo(blendCenter), blendRadius, 0.5);
}
