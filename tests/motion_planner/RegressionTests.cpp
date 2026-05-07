#include <gtest/gtest.h>

#include <tether/motion_planner/PathBuilder.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/PiecewisePath.hpp>
#include <tether/motion_planner/BezierCurve.hpp>
#include <tether/motion_planner/SourceReference.hpp>

using namespace MotionPlanner;

// Regression tests

TEST(PathBuilderRegression, RapidConvertedToCurve) {
    MotionSegmentList segments;

    std::array<double, MAX_MOTION_AXES> start{};
    std::array<double, MAX_MOTION_AXES> end{};
    start.fill(0.0);
    end.fill(0.0);
    start[0] = 0.0; start[1] = 0.0;
    end[0] = 10.0; end[1] = 0.0;

    segments.append(MotionSegment::rapid(start, end));

    PathBuilder2D builder;
    auto result = builder.build(segments);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.numSegments(), 0u);

    auto eval = result.path.evaluateAtArcLength(0.0);
    EXPECT_NEAR(eval.position[0], start[0], 1e-9);
    EXPECT_NEAR(eval.position[1], start[1], 1e-9);
}

TEST(PiecewisePathRegression, AppendPreservesCurveSourceRef) {
    auto file = std::make_shared<SourceFile>("trace.gcode");
    auto src = SourceReference::fromLine(42, file);

    Vec2 p0{0.0, 0.0}, p1{10.0, 0.0};
    auto curve = createLinearBezier(p0, p1, src);

    PiecewiseBezierPath2D path;
    path.appendSegment(curve); // don't pass a SourceReference explicitly

    const auto& segInfo = path.getSegment(0);
    EXPECT_EQ(segInfo.sourceRef.type(), SourceReference::Type::Single);
    EXPECT_EQ(segInfo.sourceRef.lineNumber(), 42u);
}
