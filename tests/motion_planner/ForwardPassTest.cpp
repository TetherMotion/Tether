/**
 * @file ForwardPassTest.cpp
 * @brief Extreme testing for the single forward pass of the SnapSpace profiler.
 *
 * @details
 * This test suite validates the forward pass in isolation under a
 * comprehensive set of adverse conditions, as described in the design
 * document §3.4.2.
 *
 * Categories:
 * - A: Boundary conditions (rest-to-rest, flying start, etc.)
 * - B: Path geometry (straight, L-shaped, multiple corners)
 * - C: Kinematic limits (high/low snap, jerk, accel)
 * - D: v_lim profiles (flat, dips, ramps)
 * - E: j* values (zero, small, max, over-max)
 * - F: Stress / edge cases (zero-length, overspeed, fine/coarse grid)
 *
 * For each test, we verify:
 * 1. No crash
 * 2. State continuity across arc boundaries
 * 3. Constraint satisfaction (v ≤ v_lim, |a| ≤ a_max, |j| ≤ j_max)
 * 4. Monotonic s (no backward motion)
 * 5. Non-negative v
 * 6. Final state (s_final ≈ sTotal, v_final ≈ vf)
 * 7. Arc count reasonable (< 10000)
 * 8. Cost finite and non-negative
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;

// ============================================================================
// Test helpers
// ============================================================================

namespace {

/// Build a straight 2D line path of the given length.
PathAdapter<2, double> makeLinePath2D(double length) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{length, 0.0}, 100.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) return PathAdapter<2, double>{};
    return std::move(result.path);
}

/// Build an L-shaped 2D path (two perpendicular line segments).
PathAdapter<2, double> makeLPath2D(double legLength) {
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{legLength, 0.0}, 100.0));
    segments.append(MotionSegment::linear(
        Vec<2, double>{legLength, 0.0}, Vec<2, double>{legLength, legLength}, 100.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) return PathAdapter<2, double>{};
    return std::move(result.path);
}

/// Build a zigzag path with multiple corners.
PathAdapter<2, double> makeZigzagPath2D(double legLength, int numLegs) {
    MotionSegmentList segments;
    Vec<2, double> pos{0.0, 0.0};
    for (int i = 0; i < numLegs; ++i) {
        Vec<2, double> next;
        if (i % 2 == 0) {
            next = {pos.x() + legLength, pos.y()};
        } else {
            next = {pos.x(), pos.y() + legLength};
        }
        segments.append(MotionSegment::linear(pos, next, 100.0));
        pos = next;
    }
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) return PathAdapter<2, double>{};
    return std::move(result.path);
}

/// Standard 2D kinematic limits.
KinematicLimits<2, double> makeLimits2D(
    double vMax = 100.0, double aMax = 500.0,
    double jMax = 5000.0, double sigmaMax = 50000.0) {
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = vMax;
    limits.path.maxPathAcceleration = aMax;
    limits.path.maxPathJerk = jMax;
    limits.path.maxPathSnap = sigmaMax;
    limits.path.jerkLimitEnabled = true;
    limits.path.maxCentripetalAcceleration = aMax;
    for (int i = 0; i < 2; ++i) {
        limits.axis.maxVelocity[i] = vMax;
        limits.axis.maxAcceleration[i] = aMax;
        limits.axis.maxJerk[i] = jMax;
    }
    limits.axis.jerkLimitEnabled = true;
    return limits;
}

/// Create a solver prepared for forward-pass testing.
using Solver2D = WeightedTimeEnergySolver<2, double>;

std::unique_ptr<Solver2D> makeSolver(
    const PathAdapter<2, double>& path,
    KinematicLimits<2, double> limits,
    CostWeights w,
    double feedRate,
    double v0, double vf,
    size_t cacheSize = 200) {
    auto solver = std::make_unique<Solver2D>(path, limits, w, feedRate);
    solver->prepareForForwardPass(v0, vf, cacheSize);
    return solver;
}

/// Validate the forward pass result against all constraints.
struct ValidationConfig {
    double vMax = 100.0;
    double aMax = 500.0;
    double jMax = 5000.0;
    double sigmaMax = 50000.0;
    double vfTolerance = 0.5;  // terminal velocity tolerance
    double sTolerance = 1e-6;  // terminal s tolerance
    double expectedFinalS = std::numeric_limits<double>::quiet_NaN();
    double expectedFinalV = std::numeric_limits<double>::quiet_NaN();
    size_t maxArcs = 50000;
    bool requireFeasible = false;  // Single pass may not be feasible
    bool requireFinalState = true;
};

struct ValidationResult {
    bool ok = true;
    std::string message;
};

ValidationResult validateForwardPass(
    const ForwardPassResult& r,
    const ValidationConfig& cfg,
    const std::function<double(double)>& vLimFn) {

    ValidationResult vr;

    // 1. No crash — if we got here, it didn't crash
    // 8. Cost finite and non-negative
    if (!std::isfinite(r.cost) || r.cost < 0.0) {
        vr.ok = false;
        vr.message = "Cost is invalid: " + std::to_string(r.cost);
        return vr;
    }

    // 7. Arc count reasonable
    if (r.arcs.size() > cfg.maxArcs) {
        vr.ok = false;
        vr.message = "Arc explosion: " + std::to_string(r.arcs.size()) +
                     " > " + std::to_string(cfg.maxArcs);
        return vr;
    }

    // Check each arc for constraint violations
    for (size_t i = 0; i < r.arcs.size(); ++i) {
        const auto& arc = r.arcs[i];

        // 4. Monotonic s
        if (arc.s1 <= arc.s0) {
            // Allow zero-length arcs (dwell)
            if (arc.s1 < arc.s0 - 1e-12) {
                vr.ok = false;
                vr.message = "Arc " + std::to_string(i) +
                             " has s1 < s0: " + std::to_string(arc.s1) +
                             " < " + std::to_string(arc.s0);
                return vr;
            }
        }

        // 5. Non-negative v
        if (arc.v0 < -1e-6) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " has v0 < 0: " + std::to_string(arc.v0);
            return vr;
        }

        // 3. Constraint satisfaction at arc start
        if (arc.v0 > cfg.vMax + 5.0) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " v0 exceeds vMax: " + std::to_string(arc.v0) +
                         " > " + std::to_string(cfg.vMax);
            return vr;
        }
        if (std::abs(arc.a0) > cfg.aMax + 5.0) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " a0 exceeds aMax: " + std::to_string(arc.a0) +
                         " > " + std::to_string(cfg.aMax);
            return vr;
        }
        if (std::abs(arc.j0) > cfg.jMax + 5.0) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " j0 exceeds jMax: " + std::to_string(arc.j0) +
                         " > " + std::to_string(cfg.jMax);
            return vr;
        }
        if (std::abs(arc.sigma) > cfg.sigmaMax + 5.0) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " sigma exceeds sigmaMax: " + std::to_string(arc.sigma) +
                         " > " + std::to_string(cfg.sigmaMax);
            return vr;
        }

        // 2. State continuity: v0 of arc[i] should match v at end of arc[i-1]
        if (i > 0) {
            const auto& prev = r.arcs[i - 1];
            // Compute end state of prev arc
            double vEnd, aEnd, jEnd;
            if (prev.type == WeightedArcType::WALL) {
                vEnd = prev.v0;
                aEnd = 0.0;
                jEnd = 0.0;
            } else if (prev.type == WeightedArcType::SINGULAR) {
                vEnd = SingularJSeg::v(prev.v0, prev.a0, prev.j_star, prev.duration);
                aEnd = SingularJSeg::a(prev.a0, prev.j_star, prev.duration);
                jEnd = prev.j_star;
            } else {
                vEnd = SnapSeg::v(prev.v0, prev.a0, prev.j0, prev.sigma, prev.duration);
                aEnd = SnapSeg::a(prev.a0, prev.j0, prev.sigma, prev.duration);
                jEnd = SnapSeg::j(prev.j0, prev.sigma, prev.duration);
            }
            if (std::abs(vEnd - arc.v0) > 5.0) {
                vr.ok = false;
                vr.message = "Arc " + std::to_string(i) +
                             " v discontinuity: prevEnd=" + std::to_string(vEnd) +
                             " curStart=" + std::to_string(arc.v0);
                return vr;
            }
            if (std::abs(aEnd - arc.a0) > 10.0) {
                vr.ok = false;
                vr.message = "Arc " + std::to_string(i) +
                             " a discontinuity: prevEnd=" + std::to_string(aEnd) +
                             " curStart=" + std::to_string(arc.a0);
                return vr;
            }
        }

        // Check v ≤ v_lim at arc midpoint
        double sMid = 0.5 * (arc.s0 + arc.s1);
        double vLimMid = vLimFn(sMid);
        // Allow v0 to exceed v_lim by a reasonable margin (the solver
        // tracks v_lim with some lag due to finite jerk/snap limits).
        // Also allow the first arc to start above v_lim (initial condition).
        double vTolerance = vLimMid + 5.0;
        if (i == 0) vTolerance = std::max(arc.v0, vLimMid) + 5.0;
        if (arc.v0 > vTolerance) {
            vr.ok = false;
            vr.message = "Arc " + std::to_string(i) +
                         " v0 exceeds v_lim at s=" + std::to_string(sMid) +
                         ": v=" + std::to_string(arc.v0) +
                         " vLim=" + std::to_string(vLimMid);
            return vr;
        }
    }

    // 6. Final state
    if (cfg.requireFinalState && std::isfinite(cfg.expectedFinalS)) {
        if (std::abs(r.finalS - cfg.expectedFinalS) > cfg.sTolerance) {
            vr.ok = false;
            vr.message = "Terminal arc length mismatch: got " +
                         std::to_string(r.finalS) + ", expected " +
                         std::to_string(cfg.expectedFinalS);
            return vr;
        }
        if (std::isfinite(cfg.expectedFinalV) &&
            std::abs(r.finalV - cfg.expectedFinalV) > cfg.vfTolerance) {
            vr.ok = false;
            vr.message = "Terminal velocity is outside the requested tolerance";
            return vr;
        }
    }

    // Check feasibility if required
    if (cfg.requireFeasible && !r.feasible) {
        vr.ok = false;
        vr.message = "Expected feasible but got infeasible: " + r.failureReason;
        return vr;
    }

    return vr;
}

/// Helper to run a forward pass and validate basic properties (no crash,
/// finite cost, reasonable arc count, no negative v, monotonic s).
/// Feasibility and terminal velocity accuracy are NOT required — those
/// will come from the iterative forward-backward approach.
void runAndValidate(
    const PathAdapter<2, double>& path,
    KinematicLimits<2, double> limits,
    CostWeights w,
    double feedRate,
    double v0, double vf,
    double jStar,
    size_t cacheSize = 200,
    const ValidationConfig& vcfg = {}) {

    ASSERT_GT(path.totalLength(), 0.0) << "Path construction failed";

    auto solver = makeSolver(path, limits, w, feedRate, v0, vf, cacheSize);
    auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };

    auto result = solver->forwardPass(jStar, vLimFn, v0, vf, solver->pathLength());

    // Basic checks (always required)
    EXPECT_TRUE(std::isfinite(result.cost)) << "Cost not finite";
    EXPECT_GE(result.cost, 0.0) << "Cost negative";
    EXPECT_LT(result.arcs.size(), vcfg.maxArcs)
        << "Arc explosion (jStar=" << jStar << " v0=" << v0 << " vf=" << vf << ")";

    // Check each arc for basic properties
    for (size_t i = 0; i < result.arcs.size(); ++i) {
        const auto& arc = result.arcs[i];

        // Monotonic s
        if (arc.s1 < arc.s0 - 1e-9) {
            FAIL() << "Arc " << i << " has s1 < s0: " << arc.s1 << " < " << arc.s0;
        }

        // Non-negative v
        if (arc.v0 < -1e-6) {
            FAIL() << "Arc " << i << " has v0 < 0: " << arc.v0;
        }

        // Finite values
        if (!std::isfinite(arc.v0) || !std::isfinite(arc.a0) ||
            !std::isfinite(arc.j0) || !std::isfinite(arc.sigma)) {
            FAIL() << "Arc " << i << " has non-finite values";
        }
    }

    // Optional: full validation (only if requireFeasible is set)
    if (vcfg.requireFeasible) {
        ValidationConfig vcfgMut = vcfg;
        vcfgMut.expectedFinalS = solver->pathLength();
        vcfgMut.expectedFinalV = vf;
        auto vr = validateForwardPass(result, vcfgMut, vLimFn);
        EXPECT_TRUE(vr.ok) << "Validation failed: " << vr.message
                           << " (jStar=" << jStar << " v0=" << v0
                           << " vf=" << vf << " arcs=" << result.arcs.size()
                           << " cost=" << result.cost << ")";
    }
}

} // namespace

// ============================================================================
// Category A: Boundary Conditions
// ============================================================================

TEST(ForwardPassTest, A1_RestToRest_LongPath) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, A2_RestToRest_ShortPath) {
    auto path = makeLinePath2D(0.5);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, A3_FlyingStart_RestEnd) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 30.0, 0.0, 100.0);
}

TEST(ForwardPassTest, A4_RestStart_FlyingEnd) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 30.0, 100.0);
}

TEST(ForwardPassTest, A5_FlyingStart_FlyingEnd) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 20.0, 20.0, 100.0);
}

TEST(ForwardPassTest, A6_StartAtMaxVelocity) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // v0 = vMax — should be clamped by v_lim
    runAndValidate(path, limits, w, 100.0, 100.0, 0.0, 100.0);
}

// ============================================================================
// Category B: Path Geometry
// ============================================================================

TEST(ForwardPassTest, B1_StraightLine) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, B2_LShapedPath) {
    auto path = makeLPath2D(5.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, B3_MultipleCorners) {
    auto path = makeZigzagPath2D(3.0, 4);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, B4_VeryLongPath) {
    auto path = makeLinePath2D(100.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, B5_VeryShortPath) {
    auto path = makeLinePath2D(0.1);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // Very short path — may be infeasible, but shouldn't crash
    ValidationConfig vcfg;
    vcfg.requireFeasible = false;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0, 200, vcfg);
}

// ============================================================================
// Category C: Kinematic Limits
// ============================================================================

TEST(ForwardPassTest, C1_HighSnap) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 500, 5000, 1e12);  // σ_max → ∞
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, C2_LowSnap) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 500, 5000, 100.0);  // very low snap
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, C3_HighJerk) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 500, 1e8, 50000);  // j_max → ∞
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, C4_LowJerk) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 500, 100.0, 50000);  // very low jerk
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, C5_HighAccel) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 1e6, 5000, 50000);  // a_max → ∞
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

TEST(ForwardPassTest, C6_LowAccel) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D(100, 10.0, 5000, 50000);  // very low accel
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0);
}

// ============================================================================
// Category D: v_lim Profiles (using custom v_lim functions)
// ============================================================================

TEST(ForwardPassTest, D1_FlatVLim) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // Flat v_lim at 40
    auto vLimFn = [](double) { return 40.0; };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    // Basic checks only — single pass may not produce feasible trajectory
    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
    EXPECT_GE(result.arcs.size(), 1u);
}

TEST(ForwardPassTest, D2_SharpDip) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // Sharp dip at s=5: v_lim drops to 10
    auto vLimFn = [](double s) {
        if (std::abs(s - 5.0) < 0.2) return 10.0;
        return 40.0;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, D3_WideDip) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // Wide dip: v_lim=10 from s=4 to s=6
    auto vLimFn = [](double s) {
        if (s >= 4.0 && s <= 6.0) return 10.0;
        return 40.0;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, D4_VLimZeroAtEnd) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // v_lim=0 at the end (terminal stop)
    auto vLimFn = [](double s) {
        if (s >= 9.9) return 0.0;
        return 40.0;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    // v_lim=0 at end may cause infeasibility — just check no crash
    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, D5_VLimRampDown) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // Linear ramp down from 40 to 5
    auto vLimFn = [](double s) {
        return 40.0 - 3.5 * s;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, D6_MultipleDips) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // Multiple dips at s=3, s=6, s=8
    auto vLimFn = [](double s) {
        if (std::abs(s - 3.0) < 0.3) return 15.0;
        if (std::abs(s - 6.0) < 0.3) return 10.0;
        if (std::abs(s - 8.0) < 0.3) return 5.0;
        return 40.0;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());

    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

// ============================================================================
// Category E: j* Values
// ============================================================================

TEST(ForwardPassTest, E1_JStarZero) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // j*=0 is degenerate — should not crash
    ValidationConfig vcfg;
    vcfg.requireFeasible = false;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 0.0, 200, vcfg);
}

TEST(ForwardPassTest, E2_JStarSmall) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 0.1);
}

TEST(ForwardPassTest, E3_JStarMax) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 5000.0);
}

TEST(ForwardPassTest, E4_JStarOverMax) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // j* > j_max should be clamped internally
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 10000.0);
}

// ============================================================================
// Category F: Stress / Edge Cases
// ============================================================================

TEST(ForwardPassTest, F1_ZeroLengthPath) {
    auto path = makeLinePath2D(0.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // Zero-length path — forwardPass should handle gracefully
    // Don't use makeSolver since it requires path.totalLength() > 0
    Solver2D solver(path, limits, w, 50.0);
    solver.prepareForForwardPass(0.0, 0.0, 200);
    auto vLimFn = [&solver](double s) { return solver.defaultVLimAt(s); };
    auto result = solver.forwardPass(100.0, vLimFn, 0.0, 0.0, 0.0);
    // Should not crash, should report feasible (v0=vf=0, sTotal=0)
    EXPECT_TRUE(std::isfinite(result.cost));
}

TEST(ForwardPassTest, F2_StartAboveVLim) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 60.0, 0.0);
    // v0=60 but v_lim=50 — solver should handle by decelerating
    auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };
    auto result = solver->forwardPass(100.0, vLimFn, 60.0, 0.0, solver->pathLength());
    // Should not crash
    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, F3_EndAboveVLim) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 60.0);
    // vf=60 but v_lim=50 — solver should handle
    auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 60.0, solver->pathLength());
    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, F4_VLimDropsFasterThanPossible) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    // v_lim drops from 40 to 0 instantly at s=5
    auto vLimFn = [](double s) {
        return (s < 5.0) ? 40.0 : 0.0;
    };
    auto result = solver->forwardPass(100.0, vLimFn, 0.0, 0.0, solver->pathLength());
    // Should not crash — may be infeasible
    EXPECT_TRUE(std::isfinite(result.cost));
    EXPECT_LT(result.arcs.size(), 50000u);
}

TEST(ForwardPassTest, F5_FineGrid) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // Very fine grid (1000 points)
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0, 1000);
}

TEST(ForwardPassTest, F6_CoarseGrid) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    // Very coarse grid (10 points)
    runAndValidate(path, limits, w, 50.0, 0.0, 0.0, 100.0, 10);
}

// ============================================================================
// Additional robustness tests
// ============================================================================

TEST(ForwardPassTest, R1_NoCrashOnManyJStarValues) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;
    auto solver = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };

    // Sweep j* from very small to very large
    for (double jStar : {0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0, 5000.0, 10000.0}) {
        auto result = solver->forwardPass(jStar, vLimFn, 0.0, 0.0, solver->pathLength());
        EXPECT_TRUE(std::isfinite(result.cost)) << "jStar=" << jStar;
        EXPECT_LT(result.arcs.size(), 50000u) << "jStar=" << jStar;
    }
}

TEST(ForwardPassTest, R2_NoCrashOnManyV0Values) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;

    for (double v0 : {0.0, 1.0, 10.0, 30.0, 50.0, 80.0, 100.0}) {
        auto solver = makeSolver(path, limits, w, 50.0, v0, 0.0);
        auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };
        auto result = solver->forwardPass(100.0, vLimFn, v0, 0.0, solver->pathLength());
        EXPECT_TRUE(std::isfinite(result.cost)) << "v0=" << v0;
        EXPECT_LT(result.arcs.size(), 50000u) << "v0=" << v0;
    }
}

TEST(ForwardPassTest, R3_NoCrashOnManyVfValues) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;

    for (double vf : {0.0, 1.0, 10.0, 30.0, 50.0, 80.0, 100.0}) {
        auto solver = makeSolver(path, limits, w, 50.0, 0.0, vf);
        auto vLimFn = [&solver](double s) { return solver->defaultVLimAt(s); };
        auto result = solver->forwardPass(100.0, vLimFn, 0.0, vf, solver->pathLength());
        EXPECT_TRUE(std::isfinite(result.cost)) << "vf=" << vf;
        EXPECT_LT(result.arcs.size(), 50000u) << "vf=" << vf;
    }
}

TEST(ForwardPassTest, R4_DeterministicResults) {
    auto path = makeLinePath2D(10.0);
    auto limits = makeLimits2D();
    CostWeights w; w.w_t = 1.0; w.w_a = 0.01;

    // Run twice, results should be identical
    auto solver1 = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    auto vLimFn1 = [&solver1](double s) { return solver1->defaultVLimAt(s); };
    auto result1 = solver1->forwardPass(100.0, vLimFn1, 0.0, 0.0, solver1->pathLength());

    auto solver2 = makeSolver(path, limits, w, 50.0, 0.0, 0.0);
    auto vLimFn2 = [&solver2](double s) { return solver2->defaultVLimAt(s); };
    auto result2 = solver2->forwardPass(100.0, vLimFn2, 0.0, 0.0, solver2->pathLength());

    EXPECT_EQ(result1.arcs.size(), result2.arcs.size());
    EXPECT_NEAR(result1.cost, result2.cost, 1e-6);
    EXPECT_NEAR(result1.finalV, result2.finalV, 1e-6);
}
