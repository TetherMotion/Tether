/// @file PlanningSegmentConverterTests.cpp
/// @brief Tests for tether::motion::piecewiseNurbsFromSegments()

#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace tether::motion;
using GCode::PlanningSegment;
using GCode::SegmentMotionType;
using GCode::InterpolationPlane;
using GCode::Position;

namespace {

/// Build a simple square toolpath: 4 linear segments
std::vector<PlanningSegment> makeSquare() {
    std::vector<PlanningSegment> segs;
    // (0,0,0) → (100,0,0) → (100,100,0) → (0,100,0) → (0,0,0)
    Position pts[] = {
        Position{},       // (0,0,0)
        Position{},       // (100,0,0)
        Position{},       // (100,100,0)
        Position{},       // (0,100,0)
        Position{},       // (0,0,0)
    };
    pts[1][0] = 100.0;
    pts[2][0] = 100.0; pts[2][1] = 100.0;
    pts[3][1] = 100.0;

    for (int i = 0; i < 4; ++i) {
        PlanningSegment seg;
        seg.start = pts[i];
        seg.end = pts[i + 1];
        seg.motionType = SegmentMotionType::Linear;
        seg.feedRate = 3000.0;
        seg.segmentLength = seg.start.linearDistance(seg.end);
        segs.push_back(seg);
    }
    return segs;
}

/// Build a single arc segment: semicircle from (0,0) to (50,0), center (25,0), r=25
PlanningSegment makeArc() {
    PlanningSegment seg;
    seg.start = Position{};  // (0,0,0)
    seg.end = Position{};
    seg.end[0] = 50.0;  // (50,0,0)
    seg.center = Position{};
    seg.center[0] = 25.0;  // center at (25,0,0)
    seg.motionType = SegmentMotionType::ArcCW;
    seg.plane = InterpolationPlane::XY;
    seg.arcRadius = 25.0;
    seg.arcSweep = -M_PI;  // CW semicircle (negative sweep)
    seg.segmentLength = M_PI * 25.0;  // ≈ 78.5
    return seg;
}

} // anonymous namespace

// ── Linear path tests ────────────────────────────────────────────────────────

TEST(PlanningSegmentConverter, SquarePath) {
    auto segs = makeSquare();
    auto result = piecewiseNurbsFromSegments(segs);

    EXPECT_EQ(result.path.numPieces(), 4u);
    // Total length = 4 × 100 = 400mm
    EXPECT_NEAR(result.path.totalLength(), 400.0, 0.5);
}

TEST(PlanningSegmentConverter, DeviationsAndSpeedsPopulated) {
    auto segs = makeSquare();
    // Set some deviation and extruder speed values
    segs[0].entryVelocity = 80.0;  // deviation %
    segs[0].exitVelocity = 3.0;    // extruder speed mm/s

    auto result = piecewiseNurbsFromSegments(segs);

    EXPECT_EQ(result.deviations.size(), 4u);
    EXPECT_EQ(result.extruderSpeeds.size(), 4u);
    EXPECT_FLOAT_EQ(result.deviations[0], 80.0f);
    EXPECT_FLOAT_EQ(result.extruderSpeeds[0], 3.0f);
}

TEST(PlanningSegmentConverter, SkipsZeroLengthSegments) {
    auto segs = makeSquare();
    // Insert a zero-length segment in the middle
    PlanningSegment zero;
    zero.start = segs[1].end;
    zero.end = segs[1].end;  // same start and end
    zero.motionType = SegmentMotionType::Linear;
    segs.insert(segs.begin() + 2, zero);

    auto result = piecewiseNurbsFromSegments(segs);

    // Should have 4 pieces (zero-length skipped)
    EXPECT_EQ(result.path.numPieces(), 4u);
}

// ── Arc path tests ───────────────────────────────────────────────────────────

TEST(PlanningSegmentConverter, ArcPath) {
    auto seg = makeArc();
    auto result = piecewiseNurbsFromSegments({seg});

    EXPECT_EQ(result.path.numPieces(), 1u);
    // Arc length = π × 25 ≈ 78.54
    EXPECT_NEAR(result.path.totalLength(), 78.54, 1.0);
}

TEST(PlanningSegmentConverter, ArcSamplesFollowCircularPath) {
    auto seg = makeArc();
    auto result = piecewiseNurbsFromSegments({seg});

    ASSERT_EQ(result.path.numPieces(), 1u);

    // Sample the path at several points and check they lie on the circle
    double totalLen = result.path.totalLength();
    for (int i = 0; i <= 10; ++i) {
        double s = totalLen * i / 10.0;
        RVec p = result.path.evaluatePosition(s);
        double dx = p[0] - 25.0;  // center at (25,0)
        double dy = p[1] - 0.0;
        double r = std::sqrt(dx*dx + dy*dy);
        EXPECT_NEAR(r, 25.0, 0.5);  // within 0.5mm tolerance
    }
}

// ── Mixed path tests ─────────────────────────────────────────────────────────

TEST(PlanningSegmentConverter, MixedLineAndArc) {
    auto segs = makeSquare();
    // Replace the 3rd segment with an arc
    segs[2] = makeArc();
    segs[2].start = segs[1].end;  // connect from previous end

    auto result = piecewiseNurbsFromSegments(segs);

    // Should have 4 pieces (3 lines + 1 arc)
    EXPECT_EQ(result.path.numPieces(), 4u);
    EXPECT_GT(result.path.totalLength(), 0.0);
}

// ── Edge case tests ──────────────────────────────────────────────────────────

TEST(PlanningSegmentConverter, EmptyInputThrows) {
    EXPECT_THROW(
        piecewiseNurbsFromSegments({}),
        std::invalid_argument
    );
}

TEST(PlanningSegmentConverter, AllZeroLengthThrows) {
    std::vector<PlanningSegment> segs;
    PlanningSegment zero;
    zero.start = Position{};
    zero.end = Position{};
    segs.push_back(zero);

    EXPECT_THROW(
        piecewiseNurbsFromSegments(segs),
        std::invalid_argument
    );
}
