/// @file test_PlanningSegmentBuilder.cpp
/// @brief Tests for PlanningSegmentBuilder, computeTimeFromFeedRate,
///        and CornerAnalysis::deviationPercentage.

#include "tether/gcode/PlanningSegmentBuilder.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/gcode/motion/G64CornerMode.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace GCode;

// ── Test G-code snippets ─────────────────────────────────────────────────────

static const char* SQUARE_GCODE =
    "G21\n"           // mm units
    "G90\n"           // absolute positioning
    "G0 X0 Y0 Z5\n"   // rapid to start
    "G1 X100 Y0 Z5 F3000\n"
    "G1 X100 Y100 Z5 F3000\n"
    "G1 X0 Y100 Z5 F3000\n"
    "G1 X0 Y0 Z5 F3000\n"
    "M30\n";

static const char* ARC_GCODE =
    "G21\n"
    "G90\n"
    "G17\n"           // XY plane
    "G0 X0 Y0 Z5\n"
    "G2 X50 Y0 I25 J0 F1500\n"  // CW arc, radius 25
    "M30\n";

static const char* INCREMENTAL_GCODE =
    "G21\n"
    "G91\n"           // incremental
    "G0 X0 Y0 Z5\n"
    "G1 X50 Y0 F3000\n"   // move +50 X
    "G1 X50 Y0 F3000\n"   // move +50 X (total 100)
    "M30\n";

static const char* INCH_GCODE =
    "G20\n"           // inches
    "G90\n"
    "G0 X0 Y0 Z1\n"   // 1 inch = 25.4mm
    "G1 X1 Y0 F100\n"  // 1 inch move
    "M30\n";

// ── PlanningSegmentBuilder tests ─────────────────────────────────────────────

TEST(PlanningSegmentBuilder, ParsesSquareToolpath) {
    auto result = PlanningSegmentBuilder::fromText(SQUARE_GCODE);
    EXPECT_TRUE(result.error.ok()) << "Parse error";
    EXPECT_FALSE(result.segments.empty());
    EXPECT_GT(result.segments.size(), 0u);
}

TEST(PlanningSegmentBuilder, SquareToolpathBounds) {
    auto result = PlanningSegmentBuilder::fromText(SQUARE_GCODE);
    ASSERT_TRUE(result.error.ok());
    ASSERT_FALSE(result.segments.empty());

    double minX = 1e9, maxX = -1e9;
    double minY = 1e9, maxY = -1e9;
    for (const auto& s : result.segments) {
        minX = std::min(minX, s.start[0]);
        maxX = std::max(maxX, s.start[0]);
        minY = std::min(minY, s.start[1]);
        maxY = std::max(maxY, s.start[1]);
    }
    // Also check end positions
    for (const auto& s : result.segments) {
        minX = std::min(minX, s.end[0]);
        maxX = std::max(maxX, s.end[0]);
        minY = std::min(minY, s.end[1]);
        maxY = std::max(maxY, s.end[1]);
    }

    EXPECT_NEAR(minX, 0.0, 0.1);
    EXPECT_NEAR(maxX, 100.0, 0.1);
    EXPECT_NEAR(minY, 0.0, 0.1);
    EXPECT_NEAR(maxY, 100.0, 0.1);
}

TEST(PlanningSegmentBuilder, ArcIsPreserved) {
    auto result = PlanningSegmentBuilder::fromText(ARC_GCODE);
    ASSERT_TRUE(result.error.ok());
    ASSERT_FALSE(result.segments.empty());

    // The arc should be a single ArcCW segment (not tessellated)
    int arcCount = 0;
    for (const auto& s : result.segments) {
        if (s.isArc()) ++arcCount;
    }
    EXPECT_GE(arcCount, 1) << "Arc should be preserved as a single segment";

    // Check arc geometry
    for (const auto& s : result.segments) {
        if (s.isArc()) {
            EXPECT_NEAR(s.arcRadius, 25.0, 0.1);
            // Path length = |sweep| * radius ≈ π * 25 ≈ 78.5
            EXPECT_GT(s.segmentLength, 70.0);
            EXPECT_LT(s.segmentLength, 90.0);
        }
    }
}

TEST(PlanningSegmentBuilder, IncrementalPositioning) {
    auto result = PlanningSegmentBuilder::fromText(INCREMENTAL_GCODE);
    ASSERT_TRUE(result.error.ok());
    ASSERT_FALSE(result.segments.empty());

    // Final X should be 100 (two +50 moves)
    double finalX = result.segments.back().end[0];
    EXPECT_NEAR(finalX, 100.0, 0.5);
}

TEST(PlanningSegmentBuilder, InchUnitsConversion) {
    auto result = PlanningSegmentBuilder::fromText(INCH_GCODE);
    ASSERT_TRUE(result.error.ok());
    ASSERT_FALSE(result.segments.empty());

    // Final X should be 1 inch = 25.4mm
    double finalX = result.segments.back().end[0];
    EXPECT_NEAR(finalX, 25.4, 0.5);
}

TEST(PlanningSegmentBuilder, EmptyGcodeReturnsError) {
    auto result = PlanningSegmentBuilder::fromText("; just a comment\n\n  \n");
    // Should either return error or return empty segments
    if (result.error.ok()) {
        EXPECT_TRUE(result.segments.empty());
    }
}

TEST(PlanningSegmentBuilder, BlockMetadataPopulated) {
    auto result = PlanningSegmentBuilder::fromText(SQUARE_GCODE);
    ASSERT_TRUE(result.error.ok());
    EXPECT_FALSE(result.blocks.empty());

    // Should have blocks with line numbers
    bool hasLineNumber = false;
    for (const auto& blk : result.blocks) {
        if (blk.lineNumber > 0) hasLineNumber = true;
    }
    EXPECT_TRUE(hasLineNumber);
}

// ── computeTimeFromFeedRate tests ────────────────────────────────────────────

TEST(PlanningSegmentComputeTime, LinearMove) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.feedRate = 3000.0;  // mm/min = 50 mm/s
    seg.segmentLength = 100.0;  // mm
    seg.computeTimeFromFeedRate();
    // time = 100 / 50 = 2.0 seconds
    EXPECT_NEAR(seg.segmentTime, 2.0, 0.001);
}

TEST(PlanningSegmentComputeTime, RapidMove) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Rapid;
    seg.isRapid = true;
    seg.feedRate = 100.0;  // mm/min = 1.67 mm/s (below 200 minimum)
    seg.segmentLength = 100.0;
    seg.computeTimeFromFeedRate();
    // Should use minimum 200 mm/s for rapid
    // time = 100 / 200 = 0.5 seconds
    EXPECT_NEAR(seg.segmentTime, 0.5, 0.001);
}

TEST(PlanningSegmentComputeTime, ZeroLengthSegment) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Linear;
    seg.feedRate = 3000.0;
    seg.segmentLength = 0.0;
    seg.computeTimeFromFeedRate();
    // Should get minimum 1ms
    EXPECT_NEAR(seg.segmentTime, 0.001, 1e-9);
}

TEST(PlanningSegmentComputeTime, CustomParameters) {
    PlanningSegment seg;
    seg.motionType = SegmentMotionType::Rapid;
    seg.isRapid = true;
    seg.feedRate = 100.0;
    seg.segmentLength = 100.0;
    seg.computeTimeFromFeedRate(500.0, 0.01);  // min rapid 500 mm/s, min time 10ms
    // time = 100 / 500 = 0.2 seconds
    EXPECT_NEAR(seg.segmentTime, 0.2, 0.001);
}

// ── CornerAnalysis::deviationPercentage tests ────────────────────────────────

TEST(CornerAnalysisDeviationPercentage, StraightSegments) {
    // Two collinear segments (0° turn) → 100%
    PlanningSegment seg1, seg2;
    seg1.start = Position{};  // (0,0,0)
    seg1.end = Position{};
    seg1.end[0] = 10.0;  // (10,0,0)

    seg2.start = seg1.end;
    seg2.end = Position{};
    seg2.end[0] = 20.0;  // (20,0,0)

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.deviationPercentage, 100.0, 0.5);
}

TEST(CornerAnalysisDeviationPercentage, RightAngle) {
    // 90° turn → cos(45°) × 100 ≈ 70.7%
    PlanningSegment seg1, seg2;
    seg1.start = Position{};
    seg1.end = Position{};
    seg1.end[0] = 10.0;  // (10,0,0)

    seg2.start = seg1.end;
    seg2.end = Position{};
    seg2.end[0] = 10.0;  // keep X=10
    seg2.end[1] = 10.0;  // (10,10,0)

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.deviationPercentage, 70.71, 1.0);
}

TEST(CornerAnalysisDeviationPercentage, FullReversal) {
    // 180° reversal → cos(90°) × 100 = 0%
    PlanningSegment seg1, seg2;
    seg1.start = Position{};
    seg1.end = Position{};
    seg1.end[0] = 10.0;  // (10,0,0)

    seg2.start = seg1.end;
    seg2.end = Position{};
    // Move back to origin
    seg2.end[0] = 0.0;  // (0,0,0)

    auto analysis = CornerAnalyzer::analyze(seg1, seg2);
    EXPECT_NEAR(analysis.deviationPercentage, 0.0, 0.5);
}
