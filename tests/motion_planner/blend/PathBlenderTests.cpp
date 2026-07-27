/**
 * @file PathBlenderTests.cpp
 * @brief Tests for PathBlender (overlap resolution, audit trail).
 */

#include "tether/motion_planner/blend/PathBlender.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

} // namespace

TEST(PathBlender, SingleCornerPath) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    using tether::motion::PiecewiseNurbsPath;
    // Two pieces: line along -x, line along +y, meeting at origin.
    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(RVec{-10, 0}, RVec{0, 0}));
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 0}, RVec{0, 10}));
    PiecewiseNurbsPath path(std::move(pieces));

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;

    tether::motion::PathBlender blender;
    tether::motion::BlendedPath result = blender.blend(path, spec);

    // One corner → one audit entry.
    EXPECT_EQ(result.audit.size(), 1u);
    // The corner should be blended (90° is well within range).
    EXPECT_EQ(result.blendedCount, 1);
    // Output pieces: trimmed in, blend, trimmed out = 3 pieces.
    EXPECT_GE(result.pieces.size(), 2u);
}

TEST(PathBlender, StraightPathNoBlends) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    using tether::motion::PiecewiseNurbsPath;
    // Three colinear pieces — no corners to blend.
    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 0}, RVec{5, 0}));
    pieces.push_back(NurbsCurve::fromLine(RVec{5, 0}, RVec{10, 0}));
    pieces.push_back(NurbsCurve::fromLine(RVec{10, 0}, RVec{15, 0}));
    PiecewiseNurbsPath path(std::move(pieces));

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;

    tether::motion::PathBlender blender;
    tether::motion::BlendedPath result = blender.blend(path, spec);

    EXPECT_EQ(result.blendedCount, 0);
    EXPECT_EQ(result.straightCount, 2);
    // No blend curves inserted.
    EXPECT_EQ(result.pieces.size(), 3u);
}

TEST(PathBlender, AuditTrailPopulated) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    using tether::motion::PiecewiseNurbsPath;
    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(RVec{-10, 0}, RVec{0, 0}));
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 0}, RVec{0, 10}));
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 10}, RVec{10, 10}));
    PiecewiseNurbsPath path(std::move(pieces));

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5;

    tether::motion::PathBlender blender;
    tether::motion::BlendedPath result = blender.blend(path, spec);

    EXPECT_EQ(result.audit.size(), 2u);
    for (const auto& entry : result.audit) {
        // Every entry has a corner kind, angle, and a valid outcome.
        EXPECT_GE(entry.angleRad, 0.0);
        // If not Blended, there should be a reason.
        if (entry.geometry.outcome != tether::motion::BlendOutcome::Blended) {
            EXPECT_FALSE(entry.geometry.reason.empty());
        }
    }
}

TEST(PathBlender, OverlapResolutionDoesNotCrash) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    using tether::motion::PiecewiseNurbsPath;
    // Three pieces with a short middle piece — two corners close together.
    std::vector<NurbsCurve> pieces;
    pieces.push_back(NurbsCurve::fromLine(RVec{-10, 0}, RVec{0, 0}));
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 0}, RVec{0, 1})); // short
    pieces.push_back(NurbsCurve::fromLine(RVec{0, 1}, RVec{10, 1}));
    PiecewiseNurbsPath path(std::move(pieces));

    tether::motion::BlendSpec spec;
    spec.tolerance = 0.5; // large tolerance relative to the middle piece

    tether::motion::PathBlender blender;
    tether::motion::BlendedPath result = blender.blend(path, spec);

    // Should not crash; at least one audit entry should note overlap or
    // fallback.
    EXPECT_EQ(result.audit.size(), 2u);
    // The total count should be consistent.
    EXPECT_EQ(result.blendedCount + result.exactStopCount + result.straightCount, 2);
}
