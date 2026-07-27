/**
 * @file test_CertifiedReplannerHeader.cpp
 * @brief Tests for the unified CertifiedReplanner.hpp header and namespace compat
 */

#include "tether/motion_replanner/CertifiedReplanner.hpp"

#include <gtest/gtest.h>
#include <vector>

using namespace tether::motion::replanner;
using GCodeExport::TrajectorySample;

//=============================================================================
// Verify the unified header compiles and all types are accessible
//=============================================================================

TEST(CertifiedReplannerHeader, AllTypesAccessible) {
    // Verify all types from Phases 1-7 are accessible via the unified header.

    // Phase 1
    ConverterConfig config;
    SegmentToPieceMap map;
    (void)config;
    (void)map;

    // Phase 2
    CertifiedContourError err;
    (void)err;

    // Phase 3
    CertifiedCornerDetection detection;
    (void)detection;

    // Phase 4
    CurvatureLimiterConfig limiterConfig;
    CurvatureAwareFeedLimits feedLimits;
    (void)limiterConfig;
    (void)feedLimits;

    // Phase 5
    SuggestionSolverConfig solverConfig;
    CertifiedSuggestion suggestion;
    (void)solverConfig;
    (void)suggestion;

    // Phase 6
    ReblenderConfig reblenderConfig;
    ReblendResult reblendResult;
    (void)reblenderConfig;
    (void)reblendResult;

    // Phase 7
    ProfileLimits profileLimits;
    ProfileReplanResult profileResult;
    (void)profileLimits;
    (void)profileResult;

    SUCCEED();
}

//=============================================================================
// Verify backward-compat namespace (MotionReplanner::)
//=============================================================================

TEST(CertifiedReplannerHeader, BackwardCompatNamespace) {
    // Verify the new types are accessible via the legacy MotionReplanner
    // namespace (via the using declarations in CertifiedReplanner.hpp).

    // Phase 1
    MotionReplanner::ConverterConfig config1;
    (void)config1;

    // Phase 2
    MotionReplanner::CertifiedContourError err2;
    (void)err2;

    // Phase 3
    MotionReplanner::CertifiedCornerDetection det3;
    (void)det3;

    // Phase 4
    MotionReplanner::CurvatureAwareFeedLimits fl4;
    (void)fl4;

    // Phase 5
    MotionReplanner::CertifiedSuggestion s5;
    (void)s5;

    // Phase 6
    MotionReplanner::ReblendResult r6;
    (void)r6;

    // Phase 7
    MotionReplanner::ProfileReplanResult p7;
    (void)p7;

    SUCCEED();
}

//=============================================================================
// Verify functions are callable via both namespaces
//=============================================================================

TEST(CertifiedReplannerHeader, FunctionsCallableBothNamespaces) {
    // Phase 4: computeFeedRateLimit is a simple function to test
    CurvatureLimiterConfig config;
    double feed1 = tether::motion::replanner::computeFeedRateLimit(0.0, config);
    double feed2 = MotionReplanner::computeFeedRateLimit(0.0, config);
    EXPECT_NEAR(feed1, feed2, 1e-12);

    // Phase 5: solveCertifiedFeedRate
    auto s1 = tether::motion::replanner::solveCertifiedFeedRate(6000.0, 0.0);
    auto s2 = MotionReplanner::solveCertifiedFeedRate(6000.0, 0.0);
    EXPECT_NEAR(s1.suggestedFeedRate, s2.suggestedFeedRate, 1e-12);
}
