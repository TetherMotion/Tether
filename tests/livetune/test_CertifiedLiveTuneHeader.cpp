/**
 * @file test_CertifiedLiveTuneHeader.cpp
 * @brief Tests for the unified CertifiedLiveTune.hpp header and namespace compat
 */

#include "tether/livetune/CertifiedLiveTune.hpp"

#include <gtest/gtest.h>
#include <vector>

using namespace tether::motion::livetune;
using GCodeExport::TrajectorySample;

//=============================================================================
// Verify the unified header compiles and all types are accessible
//=============================================================================

TEST(CertifiedLiveTuneHeader, AllTypesAccessible) {
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
// Verify backward-compat namespace (LiveTune::)
//=============================================================================

TEST(CertifiedLiveTuneHeader, BackwardCompatNamespace) {
    // Verify the new types are accessible via the legacy OnlineTuner
    // namespace (via the using declarations in CertifiedLiveTune.hpp).

    // Phase 1
    LiveTune::ConverterConfig config1;
    (void)config1;

    // Phase 2
    LiveTune::CertifiedContourError err2;
    (void)err2;

    // Phase 3
    LiveTune::CertifiedCornerDetection det3;
    (void)det3;

    // Phase 4
    LiveTune::CurvatureAwareFeedLimits fl4;
    (void)fl4;

    // Phase 5
    LiveTune::CertifiedSuggestion s5;
    (void)s5;

    // Phase 6
    LiveTune::ReblendResult r6;
    (void)r6;

    // Phase 7
    LiveTune::ProfileReplanResult p7;
    (void)p7;

    SUCCEED();
}

//=============================================================================
// Verify functions are callable via both namespaces
//=============================================================================

TEST(CertifiedLiveTuneHeader, FunctionsCallableBothNamespaces) {
    // Phase 4: computeFeedRateLimit is a simple function to test
    CurvatureLimiterConfig config;
    double feed1 = tether::motion::livetune::computeFeedRateLimit(0.0, config);
    double feed2 = LiveTune::computeFeedRateLimit(0.0, config);
    EXPECT_NEAR(feed1, feed2, 1e-12);

    // Phase 5: solveCertifiedFeedRate
    auto s1 = tether::motion::livetune::solveCertifiedFeedRate(6000.0, 0.0);
    auto s2 = LiveTune::solveCertifiedFeedRate(6000.0, 0.0);
    EXPECT_NEAR(s1.suggestedFeedRate, s2.suggestedFeedRate, 1e-12);
}
