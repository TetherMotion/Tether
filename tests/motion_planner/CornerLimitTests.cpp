#include <gtest/gtest.h>

#include "tether/motion_planner/CornerBlending.hpp"
#include "tether/motion_planner/MotionSegment.hpp"

using namespace MotionPlanner;

TEST(CornerLimitTests, InsideApproximateRespectsInsideTolerance) {
    // Create two perpendicular segments (90 degree corner)
    MotionSegment seg1, seg2;
    seg1.startPosition = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg1.endPosition = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg1.segmentLength = 1.0;

    seg2.startPosition = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg2.endPosition = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    seg2.segmentLength = 1.0;

    BlendConfig config;
    config.tolerance = 0.1; // default
    config.cornerMode = BlendConfig::CornerLimitMode::InsideApproximate;
    config.insideTolerance = 0.02; // smaller than path tolerance

    auto analysis = CornerAnalyzer<2>::analyzeLineLine(seg1, seg2, config);

    // Compute expected radius for insideTolerance (90-degree corner)
    double halfAngle = (analysis.angle * 0.5);
    double cosHalf = std::cos(halfAngle);
    // Use the same formula as CornerBlending: r = tol / (1 - cosHalf)
    double expectedRadius = config.insideTolerance / (1.0 - cosHalf);

    // analyzeLineLine already computed the blend geometry; use that result
    EXPECT_NEAR(analysis.blendRadius, expectedRadius, 1e-6);
}

TEST(CornerLimitTests, OutsideApproximateRespectsOutsideTolerance) {
    // Create two perpendicular segments (90 degree corner) turning right (convex)
    MotionSegment seg1, seg2;
    seg1.startPosition = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg1.endPosition = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg1.segmentLength = 1.0;

    seg2.startPosition = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    seg2.endPosition = {1.0, -1.0, 0.0, 0.0, 0.0, 0.0};
    seg2.segmentLength = 1.0;

    BlendConfig config;
    config.tolerance = 0.1; // default
    config.cornerMode = BlendConfig::CornerLimitMode::OutsideApproximate;
    config.outsideTolerance = 0.03; // smaller than path tolerance

    auto analysis = CornerAnalyzer<2>::analyzeLineLine(seg1, seg2, config);

    double halfAngle = (analysis.angle * 0.5);
    double cosHalf = std::cos(halfAngle);
    // Use the same formula as CornerBlending: r = tol / (1 - cosHalf)
    double expectedRadius = config.outsideTolerance / (1.0 - cosHalf);

    // analyzeLineLine already computed the blend geometry; use that result
    EXPECT_NEAR(analysis.blendRadius, expectedRadius, 1e-6);
}
