/**
 * @file test_OnlineReblender.cpp
 * @brief Tests for online re-blending via PathBlender
 */

#include "tether/motion_replanner/OnlineReblender.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion::replanner;
using namespace tether::motion;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    return s;
}

/// Build an L-shaped path with a 90° corner: (0,0)→(50,0)→(50,50)
PiecewiseNurbsPath makeLPath() {
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 5.0, 0, 0, 1, i * 5.0));
    }
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 50.0, i * 5.0,
                                     1, 1, 50.0 + i * 5.0));
    }
    return convertTrajectory(samples);
}

} // anonymous namespace

//=============================================================================
// Basic reblend tests
//=============================================================================

TEST(OnlineReblender, ReblendLPath) {
    PiecewiseNurbsPath path = makeLPath();
    ASSERT_EQ(path.numPieces(), 2u);

    ReblenderConfig config;
    config.defaultTolerance = 0.5; // 0.5mm tolerance
    ReblendResult result = reblend(path, config);

    // The 90° corner should be blendable with a 0.5mm tolerance.
    EXPECT_TRUE(result.reblended);
    EXPECT_GT(result.blendedCount, 0);
    EXPECT_FALSE(result.blendedPath.pieces.empty());

    // The audit trail should have one entry (one junction).
    EXPECT_EQ(result.blendedPath.audit.size(), 1u);

    // The audit entry should show a Blended outcome.
    EXPECT_EQ(result.blendedPath.audit[0].geometry.outcome,
              BlendOutcome::Blended);
}

TEST(OnlineReblender, ReblendWithSpec) {
    PiecewiseNurbsPath path = makeLPath();

    BlendSpec spec;
    spec.mode = PathMode::Blend;
    spec.tolerance = 1.0; // 1mm tolerance
    spec.continuity = Continuity::G2;
    spec.validate();

    ReblendResult result = reblendWithSpec(path, spec);
    EXPECT_GT(result.blendedCount, 0);
    EXPECT_EQ(result.blendedPath.audit.size(), 1u);
}

TEST(OnlineReblender, TightToleranceSmallerDeviation) {
    PiecewiseNurbsPath path = makeLPath();

    // Blend with a loose tolerance.
    ReblenderConfig looseConfig;
    looseConfig.defaultTolerance = 1.0;
    ReblendResult looseResult = reblend(path, looseConfig);

    // Blend with a tight tolerance.
    ReblenderConfig tightConfig;
    tightConfig.defaultTolerance = 0.1;
    ReblendResult tightResult = reblend(path, tightConfig);

    ASSERT_GT(looseResult.blendedPath.audit.size(), 0u);
    ASSERT_GT(tightResult.blendedPath.audit.size(), 0u);

    // Both should be blended.
    EXPECT_EQ(looseResult.blendedPath.audit[0].geometry.outcome,
              BlendOutcome::Blended);
    EXPECT_EQ(tightResult.blendedPath.audit[0].geometry.outcome,
              BlendOutcome::Blended);

    // The tight tolerance should produce a smaller (or equal) deviation.
    double looseDev = looseResult.blendedPath.audit[0].geometry.deviation.upper;
    double tightDev = tightResult.blendedPath.audit[0].geometry.deviation.upper;
    EXPECT_LE(tightDev, looseDev + 1e-9);

    // The tight deviation should be within its tolerance.
    EXPECT_LE(tightDev, tightConfig.defaultTolerance + 1e-9);
}

TEST(OnlineReblender, AuditTrailPopulated) {
    PiecewiseNurbsPath path = makeLPath();

    ReblenderConfig config;
    config.defaultTolerance = 0.5;
    ReblendResult result = reblend(path, config);

    ASSERT_EQ(result.blendedPath.audit.size(), 1u);
    const auto& entry = result.blendedPath.audit[0];

    // The audit entry should have all fields populated.
    EXPECT_EQ(entry.cornerIndex, 0u);
    EXPECT_GT(entry.angleRad, 0.0);
    EXPECT_EQ(entry.spec.tolerance, config.defaultTolerance);
    // Note: the note field may be empty for successful blends (it's
    // typically populated for fallbacks/diagnostics).
}

TEST(OnlineReblender, ExtractPath) {
    PiecewiseNurbsPath path = makeLPath();

    ReblenderConfig config;
    config.defaultTolerance = 0.5;
    ReblendResult result = reblend(path, config);

    auto newPath = extractPath(result.blendedPath);
    ASSERT_TRUE(newPath.has_value());
    EXPECT_GT(newPath->numPieces(), 0u);
    EXPECT_EQ(newPath->dim(), path.dim());
}

TEST(OnlineReblender, ExtractPathEmpty) {
    BlendedPath empty;
    empty.pieces = {};
    auto result = extractPath(empty);
    EXPECT_FALSE(result.has_value());
}

//=============================================================================
// Per-junction reblend tests
//=============================================================================

TEST(OnlineReblender, ReblendSpecificJunctions) {
    // 3-piece path: (0,0)→(50,0)→(50,50)→(100,50)
    std::vector<TrajectorySample> samples;
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample(i * 0.01, i * 5.0, 0, 0, 1, i * 5.0));
    }
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample((10 + i) * 0.01, 50.0, i * 5.0,
                                     1, 1, 50.0 + i * 5.0));
    }
    for (int i = 0; i <= 10; ++i) {
        samples.push_back(makeSample((20 + i) * 0.01, 50.0 + i * 5.0, 50.0,
                                     2, 1, 100.0 + i * 5.0));
    }
    PiecewiseNurbsPath path = convertTrajectory(samples);
    ASSERT_EQ(path.numPieces(), 3u);

    // Reblend only junction 0 (between piece 0 and 1).
    ReblenderConfig config;
    config.defaultTolerance = 1.0;
    config.tightTolerance = 0.1;
    ReblendResult result = reblendJunctions(path, {0}, config);

    // Both junctions should be blended (tight spec applied to all).
    EXPECT_EQ(result.blendedPath.audit.size(), 2u);
    EXPECT_GT(result.blendedCount, 0);
}

TEST(OnlineReblender, InvalidJunctionIndex) {
    PiecewiseNurbsPath path = makeLPath(); // 2 pieces, 1 junction (index 0)

    ReblenderConfig config;
    EXPECT_THROW(reblendJunctions(path, {5}, config), std::invalid_argument);
}

TEST(OnlineReblender, NoProblematicJunctions) {
    PiecewiseNurbsPath path = makeLPath();

    ReblenderConfig config;
    config.defaultTolerance = 0.5;
    ReblendResult result = reblendJunctions(path, {}, config);

    // Empty list → use default spec.
    EXPECT_GT(result.blendedCount, 0);
}

//=============================================================================
// Summary string
//=============================================================================

TEST(OnlineReblender, SummaryString) {
    PiecewiseNurbsPath path = makeLPath();

    ReblenderConfig config;
    config.defaultTolerance = 0.5;
    ReblendResult result = reblend(path, config);

    EXPECT_FALSE(result.summary.empty());
    // Summary should contain "Blended:" and a count.
    EXPECT_NE(result.summary.find("Blended:"), std::string::npos);
}
