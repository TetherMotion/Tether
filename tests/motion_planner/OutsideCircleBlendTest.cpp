/**
 * @file OutsideCircleBlendTest.cpp
 * @brief Tests for the OutsideCircleBlender (negative G64 outside blend).
 */

#include "tether/motion_planner/blend/OutsideCircleBlender.hpp"
#include "tether/motion_planner/blend/BoundaryConditions.hpp"
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
    // G2 mode (default transitionFraction = 0.15)

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_GT(result.blendedCount, 0);
    EXPECT_EQ(result.skippedCount, 0);

    // G2: 4 lines + 3 circle arcs + 6 quintic transitions = 13 pieces
    EXPECT_EQ(result.path->numPieces(), 13u);

    // Check piece types
    int arcCount = 0;    // degree-2: circle arcs
    int lineCount = 0;   // degree-1: lines
    int transCount = 0;  // degree-5: quintic transitions
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        const auto& p = result.path->piece(i);
        if (p.degree() == 2) {
            arcCount++;
            RVec center, axis1, axis2;
            double radius;
            ASSERT_TRUE(OutsideCircleBlender::extractCircleFromArc(
                p, center, radius, axis1, axis2));
            EXPECT_NEAR(radius, 5.0, 0.1);
        } else if (p.degree() == 1) {
            lineCount++;
        } else if (p.degree() == 5) {
            transCount++;
        }
    }
    EXPECT_EQ(arcCount, 3);
    EXPECT_EQ(lineCount, 4);
    EXPECT_EQ(transCount, 6);

    // G0 continuity
    EXPECT_TRUE(result.path->isG0Connected(1e-6));
}

TEST(OutsideCircleBlendTest, LPath45Degrees) {
    // L-shaped path with a 45° corner, blend radius = 3
    auto path = makeLPath(50.0, 50.0, M_PI / 4.0);  // 45° turn
    EXPECT_EQ(path.numPieces(), 2u);

    OutsideCircleBlendConfig config;
    config.radius = 3.0;
    // G2 mode (default transitionFraction = 0.15)

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.blendedCount, 1);
    EXPECT_EQ(result.skippedCount, 0);

    // G2: 2 lines + 1 circle arc + 2 quintic transitions = 5 pieces
    EXPECT_EQ(result.path->numPieces(), 5u);

    // Check piece types
    int arcCount = 0, lineCount = 0, transCount = 0;
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        int d = result.path->piece(i).degree();
        if (d == 2) arcCount++;
        else if (d == 1) lineCount++;
        else if (d == 5) transCount++;
    }
    EXPECT_EQ(arcCount, 1);
    EXPECT_EQ(lineCount, 2);
    EXPECT_EQ(transCount, 2);

    // G0 continuity
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
    // G1 mode (no transitions): verify arc starts where trimmed line ends
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);  // 90° corner

    OutsideCircleBlendConfig config;
    config.radius = 10.0;
    config.transitionFraction = 0.0;  // G1 mode

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.path->numPieces(), 3u);

    const auto& p0 = result.path->piece(0);
    const auto& p1 = result.path->piece(1);
    const auto& p2 = result.path->piece(2);

    RVec p0end = p0.endPoint();
    RVec p1start = p1.startPoint();
    RVec p1end = p1.endPoint();
    RVec p2start = p2.startPoint();

    // G0 continuity
    EXPECT_NEAR(p0end.distanceTo(p1start), 0.0, 1e-6);
    EXPECT_NEAR(p1end.distanceTo(p2start), 0.0, 1e-6);

    // Arc endpoints at distance r=10 from corner (100,0)
    RVec corner{100.0, 0.0, 0.0};
    EXPECT_NEAR(p1start.distanceTo(corner), 10.0, 0.1);
    EXPECT_NEAR(p1end.distanceTo(corner), 10.0, 0.1);

    // Trimmed line ends at (90, 0)
    EXPECT_NEAR(p0end[0], 90.0, 0.1);
    EXPECT_NEAR(p0end[1], 0.0, 0.1);

    // Second trimmed line starts at (100, 10)
    EXPECT_NEAR(p2start[0], 100.0, 0.1);
    EXPECT_NEAR(p2start[1], 10.0, 0.1);
}

TEST(OutsideCircleBlendTest, ArcIsOutsideMajorArc) {
    // G1 mode: verify the arc is the MAJOR arc (outside), sweeping 270°
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);

    OutsideCircleBlendConfig config;
    config.radius = 10.0;
    config.transitionFraction = 0.0;  // G1 mode

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.blendedCount, 1);

    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        const auto& p = result.path->piece(i);
        if (p.degree() == 2) {
            double expectedMajor = 10.0 * 3.0 * M_PI / 2.0;
            double expectedMinor = 10.0 * M_PI / 2.0;
            double arcLen = p.length();
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

// ============================================================================
// G2 continuity tests
// ============================================================================

TEST(OutsideCircleBlendTest, G2ContinuityAtTransitions) {
    // Verify G2 (curvature) continuity at the junctions between
    // line → transition → circle arc → transition → line
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);  // 90° corner

    OutsideCircleBlendConfig config;
    config.radius = 10.0;
    config.transitionFraction = 0.15;  // G2 mode

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.blendedCount, 1);
    // 5 pieces: line, transition, arc, transition, line
    ASSERT_EQ(result.path->numPieces(), 5u);

    // Check G2 continuity at each junction.
    // G2 means: position match (G0), tangent match (G1), curvature match (G2).
    const double curvTol = 0.05;  // relative tolerance for curvature matching

    for (std::size_t i = 0; i + 1 < result.path->numPieces(); ++i) {
        const auto& pA = result.path->piece(i);
        const auto& pB = result.path->piece(i + 1);

        // G0: positions match
        RVec endA = pA.endPoint();
        RVec startB = pB.startPoint();
        EXPECT_NEAR(endA.distanceTo(startB), 0.0, 1e-5)
            << "G0 failed at junction " << i;

        // G1: tangents match
        auto derivA = pA.arcDerivatives(pA.knotMax(), 1);
        auto derivB = pB.arcDerivatives(pB.knotMin(), 1);
        RVec tA = derivA.tangent;
        RVec tB = derivB.tangent;
        EXPECT_NEAR(tA.distanceTo(tB), 0.0, 1e-4)
            << "G1 failed at junction " << i;

        // G2: curvatures match
        // Skip curvature check at line→transition and transition→line
        // junctions where the curvature should be 0 on both sides.
        // For all junctions, the curvature vectors should match.
        RVec kA = derivA.curvature;
        RVec kB = derivB.curvature;
        double kAmag = kA.norm();
        double kBmag = kB.norm();

        // For line junctions (κ=0 on the line side), the transition
        // should also have κ≈0 at that end.
        if (kAmag < 1e-8 && kBmag < 1e-8) {
            // Both zero — G2 satisfied
            SUCCEED();
        } else if (kAmag < 1e-8 || kBmag < 1e-8) {
            // One side is zero, other isn't — G2 failure
            ADD_FAILURE()
                << "G2 curvature mismatch at junction " << i
                << ": |κ_A|=" << kAmag << ", |κ_B|=" << kBmag;
        } else {
            // Both non-zero — check relative match
            double relErr = std::abs(kAmag - kBmag) / std::max(kAmag, kBmag);
            EXPECT_LT(relErr, curvTol)
                << "G2 curvature magnitude mismatch at junction " << i
                << ": |κ_A|=" << kAmag << ", |κ_B|=" << kBmag;
        }
    }
}

TEST(OutsideCircleBlendTest, G2ContinuitySquare) {
    // Verify G2 continuity for a square path with 3 corners
    auto path = makeSquarePath(100.0);

    OutsideCircleBlendConfig config;
    config.radius = 5.0;
    config.transitionFraction = 0.15;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_TRUE(result.path->isG0Connected(1e-5));

    // Check G1 (tangent) continuity at all junctions
    for (std::size_t i = 0; i + 1 < result.path->numPieces(); ++i) {
        const auto& pA = result.path->piece(i);
        const auto& pB = result.path->piece(i + 1);

        auto dA = pA.arcDerivatives(pA.knotMax(), 1);
        auto dB = pB.arcDerivatives(pB.knotMin(), 1);
        EXPECT_NEAR(dA.tangent.distanceTo(dB.tangent), 0.0, 1e-4)
            << "G1 failed at junction " << i;
    }
}

TEST(OutsideCircleBlendTest, G1ModeNoTransitions) {
    // With transitionFraction=0, should produce G1-only output (no quintic pieces)
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);

    OutsideCircleBlendConfig config;
    config.radius = 10.0;
    config.transitionFraction = 0.0;  // G1 mode

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.blendedCount, 1);
    // G1: 2 lines + 1 arc = 3 pieces (no transitions)
    EXPECT_EQ(result.path->numPieces(), 3u);

    // No degree-5 (quintic) pieces should exist
    for (std::size_t i = 0; i < result.path->numPieces(); ++i) {
        EXPECT_NE(result.path->piece(i).degree(), 5)
            << "Quintic transition found in G1 mode";
    }
}

TEST(OutsideCircleBlendTest, G2CurvatureMatchesCircleAtArcJunctions) {
    // Verify that the curvature at the transition→arc junction matches
    // the circle's curvature (1/r) on both sides.
    auto path = makeLPath(100.0, 100.0, M_PI / 2.0);

    OutsideCircleBlendConfig config;
    config.radius = 10.0;
    config.transitionFraction = 0.15;

    auto result = OutsideCircleBlender::blend(path, config);
    ASSERT_TRUE(result.path.has_value());
    ASSERT_EQ(result.path->numPieces(), 5u);

    // Pieces: [line, transition1, arc, transition2, line]
    // Check curvature at transition1→arc junction (index 1→2)
    // and arc→transition2 junction (index 2→3)
    double expectedCurv = 1.0 / 10.0;  // 1/r

    for (int idx : {1, 2}) {
        const auto& pA = result.path->piece(idx);
        const auto& pB = result.path->piece(idx + 1);

        auto dA = pA.arcDerivatives(pA.knotMax(), 2);
        auto dB = pB.arcDerivatives(pB.knotMin(), 2);

        double kA = dA.curvature.norm();
        double kB = dB.curvature.norm();

        // Both should be close to 1/r
        EXPECT_NEAR(kA, expectedCurv, expectedCurv * 0.1)
            << "Curvature at end of piece " << idx << " = " << kA
            << ", expected " << expectedCurv;
        EXPECT_NEAR(kB, expectedCurv, expectedCurv * 0.1)
            << "Curvature at start of piece " << (idx + 1) << " = " << kB
            << ", expected " << expectedCurv;
    }

    // Check curvature at line→transition junctions (should be ≈0)
    for (int idx : {0, 3}) {
        const auto& pA = result.path->piece(idx);
        const auto& pB = result.path->piece(idx + 1);

        auto dA = pA.arcDerivatives(pA.knotMax(), 2);
        auto dB = pB.arcDerivatives(pB.knotMin(), 2);

        // Line side: κ = 0
        // Transition side: κ should also be ≈0 (matching the line)
        EXPECT_NEAR(dA.curvature.norm(), 0.0, 1e-4)
            << "Line curvature at junction " << idx << " should be 0";
        EXPECT_NEAR(dB.curvature.norm(), 0.0, 1e-4)
            << "Transition curvature at junction " << idx << " should be ~0";
    }
}
