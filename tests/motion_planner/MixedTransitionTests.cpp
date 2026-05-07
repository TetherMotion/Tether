#include <gtest/gtest.h>
#include "tether/motion_planner/CornerBlending.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include <cmath>
#include <array>

namespace MotionPlanner {
namespace test {

using Arr = std::array<double, MAX_MOTION_AXES>;
using Analyzer = CornerAnalyzer2D;
constexpr double PI = 3.14159265358979323846;

static Arr makePos(double x, double y, double z = 0) {
    Arr a{};
    a[0] = x; a[1] = y; a[2] = z;
    return a;
}

static MotionSegment makeLine(double x0, double y0, double x1, double y1) {
    return MotionSegment::linear(makePos(x0, y0), makePos(x1, y1), 1000.0);
}

static MotionSegment makeArcCW(double sx, double sy,
                                double ex, double ey,
                                double cx, double cy) {
    return MotionSegment::arcCW(makePos(sx, sy), makePos(ex, ey),
                                makePos(cx, cy), 1000.0, ArcPlane::XY);
}

// 1. Zero Tolerance - Should NOT blend
TEST(MixedTransition, LineLine_ZeroTolerance) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10); // 90 deg corner
    BlendConfig config;
    config.tolerance = 0.0;
    
    auto analysis = Analyzer::analyze(seg1, seg2, config);
    EXPECT_FALSE(analysis.canBlend) << "Zero tolerance should prevent blending";
    EXPECT_EQ(analysis.blendReason, "Zero tolerance");
}

// 2. Zero Length Segment - Should NOT blend
TEST(MixedTransition, ZeroLengthSegment) {
    auto seg1 = makeLine(0, 0, 0, 0); // Zero length
    auto seg2 = makeLine(0, 0, 10, 0);
    BlendConfig config;
    config.tolerance = 1.0;
    
    auto analysis = Analyzer::analyze(seg1, seg2, config);
    EXPECT_FALSE(analysis.canBlend);
    EXPECT_EQ(analysis.blendReason, "Zero-length segment");
}

// 3. Cusp (180 deg turn) - Should NOT blend
TEST(MixedTransition, CuspTurn) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 0, 0); // Go back exactly
    BlendConfig config;
    config.tolerance = 1.0;
    
    auto analysis = Analyzer::analyze(seg1, seg2, config);
    EXPECT_FALSE(analysis.canBlend);
    EXPECT_EQ(analysis.blendReason, "Cusp angle");
}

// 4. Smooth Arc (Tangent Match) - Should NOT blend (near straight)
TEST(MixedTransition, LineToSmoothArc_ShouldNotBlend) {
    double x1 = 20.0, y1 = 0.0;
    double x2 = 30.0, y2 = 10.0;
    // Arc Center (37.071, 2.929)
    double cx = 37.071;
    double cy = 2.929;
    double ex = 44.14; 
    double ey = 2.93;

    auto line = makeLine(x1, y1, x2, y2);
    auto arc = makeArcCW(x2, y2, ex, ey, cx, cy);

    BlendConfig config;
    config.tolerance = 5.0;
    
    auto analysis = Analyzer::analyze(line, arc, config);
    EXPECT_FALSE(analysis.canBlend) << "Should not blend a smooth transition";
}

// 5. Normal Corner - Should Blend
TEST(MixedTransition, NormalCorner_ShouldBlend) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);
    BlendConfig config;
    config.tolerance = 2.0;
    
    auto analysis = Analyzer::analyze(seg1, seg2, config);
    EXPECT_TRUE(analysis.canBlend);
    EXPECT_GT(analysis.blendRadius, 0.0);
}

// 6. Outside Blend (Negative Tolerance)
TEST(MixedTransition, OutsideBlend_ExpectedBehavior) {
    auto seg1 = makeLine(0, 0, 10, 0);
    auto seg2 = makeLine(10, 0, 10, 10);
    BlendConfig config;
    config.tolerance = -2.0; // Outside
    
    auto analysis = Analyzer::analyze(seg1, seg2, config);
    EXPECT_TRUE(analysis.canBlend);
    EXPECT_TRUE(analysis.isOutsideBlend);
    EXPECT_GT(analysis.blendRadius, 0.0);
}

// 7. Short Segments - Should scale down blend or reject
TEST(MixedTransition, ShortSegment_Constraints) {
    auto seg1 = makeLine(0, 0, 1.0, 0); // Very short
    auto seg2 = makeLine(1.0, 0, 1.0, 10.0);
    BlendConfig config;
    config.tolerance = 5.0; // Huge tolerance for short seg
    config.minSegmentLength = 0.1; // Default
    // maxBlendFraction is 0.5 default

    auto analysis = Analyzer::analyze(seg1, seg2, config);
    
    if (analysis.canBlend) {
        // Entry distance must be <= 0.5 * segLength (0.5)
        EXPECT_LE(analysis.entryDistance, 0.500001);
    }
}

} // namespace test
} // namespace MotionPlanner
