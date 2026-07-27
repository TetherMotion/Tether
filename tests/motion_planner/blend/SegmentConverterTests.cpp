/**
 * @file SegmentConverterTests.cpp
 * @brief Tests for SegmentConverter (MotionSegment → NurbsCurve).
 */

#include "tether/motion_planner/blend/SegmentConverter.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tether::motion::testing::expectVecNear;

namespace {

constexpr double kPi = 3.14159265358979323846;
using MotionPlanner::MotionSegment;
using MotionPlanner::MotionSegmentType;
using MotionPlanner::ArcPlane;
using MotionPlanner::MAX_MOTION_AXES;

} // namespace

TEST(SegmentConverter, LinearSegmentProducesLineNurbs) {
    std::array<double, MAX_MOTION_AXES> start{}, end{};
    start[0] = 0.0; start[1] = 0.0;
    end[0] = 10.0; end[1] = 0.0;
    MotionSegment seg = MotionSegment::linear(start, end, 100.0);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->degree(), 1);
    EXPECT_EQ(curve->dim(), 1u); // only X is active
    expectVecNear(curve->startPoint(), tether::motion::RVec{0.0}, 1e-9);
    expectVecNear(curve->endPoint(), tether::motion::RVec{10.0}, 1e-9);
    EXPECT_NEAR(curve->length(), 10.0, 1e-9);
}

TEST(SegmentConverter, LinearSegment2D) {
    std::array<double, MAX_MOTION_AXES> start{}, end{};
    start[0] = 0.0; start[1] = 0.0;
    end[0] = 3.0; end[1] = 4.0;
    MotionSegment seg = MotionSegment::linear(start, end, 100.0);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->dim(), 2u);
    EXPECT_NEAR(curve->length(), 5.0, 1e-9);
}

TEST(SegmentConverter, RapidSegmentProducesLine) {
    std::array<double, MAX_MOTION_AXES> start{}, end{};
    start[0] = 0.0; end[0] = 5.0;
    MotionSegment seg = MotionSegment::rapid(start, end);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->degree(), 1);
    EXPECT_EQ(seg.type, MotionSegmentType::Rapid); // sanity
}

TEST(SegmentConverter, ArcSegmentProducesQuadraticNurbs) {
    // 90° CCW arc in the XY plane, center (0,0), radius 5, from (5,0) to (0,5).
    std::array<double, MAX_MOTION_AXES> start{}, end{}, center{};
    start[0] = 5.0; start[1] = 0.0;
    end[0] = 0.0; end[1] = 5.0;
    center[0] = 0.0; center[1] = 0.0;
    MotionSegment seg = MotionSegment::arcCCW(start, end, center, 5.0, 100.0,
                                              ArcPlane::XY);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->degree(), 2); // rational quadratic arc
    EXPECT_EQ(curve->dim(), 2u);
    // Arc length = π/2 × 5 ≈ 7.854
    EXPECT_NEAR(curve->length(), kPi / 2.0 * 5.0, 1e-6);
    // Endpoints match.
    expectVecNear(curve->startPoint(),
                  tether::motion::RVec{5.0, 0.0}, 1e-9);
    expectVecNear(curve->endPoint(),
                  tether::motion::RVec{0.0, 5.0}, 1e-9);
}

TEST(SegmentConverter, ArcSegmentXZPlane) {
    // 90° arc in the XZ plane.
    std::array<double, MAX_MOTION_AXES> start{}, end{}, center{};
    start[0] = 5.0; start[2] = 0.0;
    end[0] = 0.0; end[2] = 5.0;
    center[0] = 0.0; center[2] = 0.0;
    MotionSegment seg = MotionSegment::arcCCW(start, end, center, 5.0, 100.0,
                                              ArcPlane::XZ);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->dim(), 2u); // X and Z are active
    EXPECT_NEAR(curve->length(), kPi / 2.0 * 5.0, 1e-6);
}

TEST(SegmentConverter, DwellSegmentReturnsNullopt) {
    std::array<double, MAX_MOTION_AXES> pos{};
    pos[0] = 1.0;
    MotionSegment seg = MotionSegment::dwell(0.5, pos);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    EXPECT_FALSE(curve.has_value());
}

TEST(SegmentConverter, NurbsSegmentPassThrough) {
    // Build a degree-2 NURBS with 3 control points. Both X and Y move
    // between the first and last control points so the active-axis
    // detection picks up both.
    std::array<double, MAX_MOTION_AXES> start{}, end{};
    start[0] = 0.0; start[1] = 0.0;
    end[0] = 10.0; end[1] = 5.0;
    std::vector<std::array<double, MAX_MOTION_AXES>> poles(3);
    poles[0] = start;
    poles[1] = start; poles[1][0] = 5.0; poles[1][1] = 6.0;
    poles[2] = end;
    std::vector<double> weights{1.0, 1.0, 1.0};
    std::vector<double> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    MotionSegment seg = MotionSegment::nurbs(start, end, poles, weights,
                                             knots, 2, 100.0);

    auto curve = tether::motion::SegmentConverter::convert(seg);
    ASSERT_TRUE(curve.has_value());
    EXPECT_EQ(curve->degree(), 2);
    EXPECT_EQ(curve->numControlPoints(), 3u);
    EXPECT_EQ(curve->dim(), 2u);
    // The control points should match (in the active subspace).
    auto cps = curve->controlPoints();
    ASSERT_EQ(cps.size(), 3u);
    expectVecNear(cps[0], tether::motion::RVec{0.0, 0.0}, 1e-9);
    expectVecNear(cps[1], tether::motion::RVec{5.0, 6.0}, 1e-9);
    expectVecNear(cps[2], tether::motion::RVec{10.0, 5.0}, 1e-9);
}

TEST(SegmentConverter, ConvertAllSkipsDwell) {
    std::array<double, MAX_MOTION_AXES> a{}, b{}, c{};
    a[0] = 0.0; b[0] = 5.0; c[0] = 10.0;

    std::vector<MotionSegment> segs;
    segs.push_back(MotionSegment::linear(a, b, 100.0));
    segs.push_back(MotionSegment::dwell(0.1, b)); // skipped
    segs.push_back(MotionSegment::linear(b, c, 100.0));

    auto curves = tether::motion::SegmentConverter::convertAll(segs);
    EXPECT_EQ(curves.size(), 2u);
    EXPECT_NEAR(curves[0].length(), 5.0, 1e-9);
    EXPECT_NEAR(curves[1].length(), 5.0, 1e-9);
}
