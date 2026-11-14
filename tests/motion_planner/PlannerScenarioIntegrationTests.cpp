/**
 * @file PlannerScenarioIntegrationTests.cpp
 * @brief End-to-end scenario tests for the path & velocity planner.
 *
 * @details
 * Coverage: ~26 scenarios ordered easy → superhard, driven by realistic
 * 3D-printing and 3-axis CNC toolpaths. Each scenario runs the production
 * pipeline
 *
 *   G-code text → GCode::PlanningSegmentBuilder::fromText
 *               → tether::motion::piecewiseNurbsFromSegments
 *               → PathAdapter (per-segment feeds, corner velocities, dwells)
 *               → ParetoTimeEnergyOptimalVelocityPlanner
 *               → VelocityProfile + WeightedSwitchingStructure (WSS)
 *
 * and validates the resulting trajectory for
 *   - arc-length / time / velocity continuity of the WSS
 *   - velocity, acceleration, jerk, snap bound satisfaction (path + per-axis)
 *   - centripetal bound (v²κ ≤ a_cent) on curved pieces
 *   - monotone forward progress s(t), endpoints reached, start/end at rest
 *   - dwell insertion (G4), per-segment feed caps, junction-velocity dips
 *   - MotionPlan-level queries (evaluateAt, feed override)
 *
 * Machine models:
 *   printerLimits(): cartesian FDM printer — fast XY, slow Z leadscrew.
 *   cncLimits():     3-axis CNC mill — moderate feed, strong spindle axes.
 *
 * Some tests assert *desired* properties that probe known-risky corners of
 * the pipeline (helical arcs, G5 splines, dense polylines). Failures are
 * real planner findings, not test noise.
 */

#include <gtest/gtest.h>

#include "tether/gcode/PlanningSegmentBuilder.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"
#include "tether/motion_planner/MotionPlanner.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/MotionPlan.hpp"
#include "tether/motion_planner/PathAdapter.hpp"
#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace MotionPlanner;
namespace mpa = MotionPlanner::analytical;
using tether::motion::piecewiseNurbsFromSegments;

namespace {

// ============================================================================
// Machine models
// ============================================================================

/// Cartesian FDM printer: fast XY (200 mm/s, 2 m/s²), slow Z leadscrew.
KinematicLimits<3, double> printerLimits() {
    KinematicLimits<3, double> L;
    L.path.maxPathVelocity = 200.0;
    L.path.maxPathAcceleration = 2000.0;
    L.path.maxPathJerk = 20000.0;
    L.path.maxPathSnap = 2.0e5;
    L.path.maxCentripetalAcceleration = 2000.0;
    L.path.jerkLimitEnabled = true;
    L.path.snapLimitEnabled = true;
    L.axis.maxVelocity = {200.0, 200.0, 20.0};
    L.axis.maxAcceleration = {2000.0, 2000.0, 500.0};
    L.axis.maxJerk = {20000.0, 20000.0, 5000.0};
    L.axis.jerkLimitEnabled = true;
    // Per-axis snap requires 4th-order path derivatives PathAdapter
    // does not expose → planner rejects it. Path-level snap is used.
    return L;
}

/// 3-axis CNC mill: 80 mm/s feed, 600 mm/s², spindle-capable Z.
KinematicLimits<3, double> cncLimits() {
    KinematicLimits<3, double> L;
    L.path.maxPathVelocity = 80.0;
    L.path.maxPathAcceleration = 600.0;
    L.path.maxPathJerk = 8000.0;
    L.path.maxPathSnap = 8.0e4;
    L.path.maxCentripetalAcceleration = 400.0;
    L.path.jerkLimitEnabled = true;
    L.path.snapLimitEnabled = true;
    L.axis.maxVelocity = {80.0, 80.0, 40.0};
    L.axis.maxAcceleration = {600.0, 600.0, 300.0};
    L.axis.maxJerk = {8000.0, 8000.0, 4000.0};
    L.axis.jerkLimitEnabled = true;
    // Per-axis snap limits unsupported on PathAdapter (needs 4th-order
    // path derivatives) — path-level snap limit used instead.
    return L;
}

/// Lighter CNC (wood router): high feed, low stiffness.
KinematicLimits<3, double> routerLimits() {
    auto L = cncLimits();
    L.path.maxPathVelocity = 150.0;
    L.axis.maxVelocity = {150.0, 150.0, 50.0};
    L.path.maxPathAcceleration = 1000.0;
    L.axis.maxAcceleration = {1000.0, 1000.0, 400.0};
    return L;
}

// ============================================================================
// Pipeline runner
// ============================================================================

struct PlannedRun {
    bool ok = false;
    std::string failedAt;          ///< pipeline stage that failed, if any
    std::size_t numPlanningSegments = 0;
    std::size_t numNurbsPieces = 0;
    double pathLength = 0.0;
    std::size_t numDwellPoints = 0;
    PathAdapter<3, double> path;   ///< adapter (planner owns an internal copy)
    std::vector<double> pieceFeedRates;  ///< mm/s per piece (inf = uncapped)
    std::vector<tether::motion::DwellPoint> dwellPoints;
    std::unique_ptr<VelocityProfile> profile;
    std::shared_ptr<mpa::WeightedSwitchingStructure<3, double>> wss;
    double solveMs = 0.0;
};

/**
 * Run the full production pipeline on G-code text.
 * @param gcode            Program text (may contain \n lines).
 * @param limits           Machine kinematic limits.
 * @param feedRateMmS      Global feed cap (mm/s) for the profile.
 * @param junctionDev      Junction deviation (mm) for corner velocity model.
 * @param cacheSize        Constraint cache samples; 0 → auto (pieces*20).
 */
PlannedRun runGcode(const std::string& gcode,
                    const KinematicLimits<3, double>& limits,
                    double feedRateMmS,
                    double junctionDev = 0.05,
                    std::size_t cacheSize = 0,
                    mpa::CostWeights weights = {}) {
    PlannedRun r;

    auto parsed = GCode::PlanningSegmentBuilder::fromText(gcode);
    if (!parsed.error.ok()) {
        r.failedAt = "parse";
        return r;
    }
    r.numPlanningSegments = parsed.segments.size();
    if (parsed.segments.empty()) {
        r.failedAt = "no motion segments";
        return r;
    }

    std::optional<tether::motion::PlanningSegmentNurbsResult> nurbsOpt;
    try {
        nurbsOpt.emplace(piecewiseNurbsFromSegments(parsed.segments));
    } catch (const std::exception& e) {
        r.failedAt = std::string("nurbs: ") + e.what();
        return r;
    }
    auto& nurbs = *nurbsOpt;
    r.numNurbsPieces = nurbs.path.numPieces();
    if (r.numNurbsPieces == 0) {
        r.failedAt = "empty nurbs path";
        return r;
    }
    r.pathLength = nurbs.path.totalLength();
    r.pieceFeedRates = nurbs.feedRates;
    r.dwellPoints = nurbs.dwellPoints;
    r.numDwellPoints = nurbs.dwellPoints.size();

    r.path = PathAdapter<3, double>(nurbs.path);
    if (!nurbs.feedRates.empty()) {
        r.path.setSegmentVelocityLimits(nurbs.feedRates);
    }
    r.path.computeCornerVelocities(
        junctionDev, limits.path.maxCentripetalAcceleration);
    if (!nurbs.dwellPoints.empty()) {
        std::vector<std::pair<double, double>> dwells;
        dwells.reserve(nurbs.dwellPoints.size());
        for (const auto& dp : nurbs.dwellPoints) {
            dwells.emplace_back(dp.arcLength, dp.duration);
        }
        r.path.setDwellPoints(dwells);
    }

    if (cacheSize == 0) {
        cacheSize = std::min<std::size_t>(
            40000, std::max<std::size_t>(400, r.numNurbsPieces * 20));
    }

    mpa::ParetoTimeEnergyOptimalVelocityPlanner<3, double> planner(
        limits, weights);
    const auto t0 = std::chrono::steady_clock::now();
    r.profile = planner.computeProfile(
        r.path, feedRateMmS, 0.0, 0.0, cacheSize);
    r.solveMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    r.wss = planner.weightedSource();
    r.ok = (r.profile != nullptr) && (r.profile->totalTime() > 0.0);
    if (!r.ok) {
        r.failedAt = std::string("profile: ") + planner.failureReason();
    }
    return r;
}

// ============================================================================
// Metrics + shared validation
// ============================================================================

struct Metrics {
    double T = 0.0;                  ///< total trajectory time [s]
    double maxV = 0.0;               ///< max path velocity [mm/s]
    double maxAbsA = 0.0;            ///< max |tangential accel| [mm/s²]
    double maxAbsJ = 0.0;            ///< max |tangential jerk| [mm/s³]
    double maxCentripetal = 0.0;     ///< max v²·κ [mm/s²]
    double minInteriorV = 0.0;       ///< min v over interior 10%..90% of T
    std::array<double, 3> maxAxisV{0, 0, 0};
    std::array<double, 3> maxAxisA{0, 0, 0};
    std::array<double, 3> endPos{0, 0, 0};
    double maxStepS = 0.0;           ///< largest positive ds between samples
    bool monotoneS = true;
    bool finite = true;
};

/// Sample the WSS (exact analytical source) at uniform time steps.
/// Null-safe: returns a finite=false metrics struct when wss is null.
Metrics sampleWss(const std::shared_ptr<
                      mpa::WeightedSwitchingStructure<3, double>>& wss,
                  std::size_t n = 4000) {
    Metrics m;
    if (!wss) { m.finite = false; return m; }
    m.T = wss->totalTime();
    if (!(m.T > 0.0)) { m.finite = false; return m; }
    m.minInteriorV = std::numeric_limits<double>::max();

    double prevS = wss->arcLength(0.0);
    for (std::size_t i = 0; i <= n; ++i) {
        const double t = m.T * static_cast<double>(i) / n;
        const double s = wss->arcLength(t);
        const double v = wss->pathVelocity(t);
        const double a = wss->pathAcceleration(t);
        const double j = wss->pathJerk(t);
        const double k = wss->curvature(t);
        const auto vv = wss->velocity(t);
        const auto aa = wss->acceleration(t);

        if (!std::isfinite(s) || !std::isfinite(v) || !std::isfinite(a) ||
            !std::isfinite(j) || !std::isfinite(k)) {
            m.finite = false;
        }
        for (int ax = 0; ax < 3; ++ax) {
            if (!std::isfinite(vv[ax]) || !std::isfinite(aa[ax]))
                m.finite = false;
            m.maxAxisV[ax] = std::max(m.maxAxisV[ax], std::abs(vv[ax]));
            m.maxAxisA[ax] = std::max(m.maxAxisA[ax], std::abs(aa[ax]));
        }
        m.maxV = std::max(m.maxV, v);
        m.maxAbsA = std::max(m.maxAbsA, std::abs(a));
        m.maxAbsJ = std::max(m.maxAbsJ, std::abs(j));
        m.maxCentripetal = std::max(m.maxCentripetal, v * v * k);
        if (t > 0.1 * m.T && t < 0.9 * m.T)
            m.minInteriorV = std::min(m.minInteriorV, v);

        const double ds = s - prevS;
        if (ds < -1e-6) m.monotoneS = false;
        m.maxStepS = std::max(m.maxStepS, ds);
        prevS = s;
    }
    if (m.minInteriorV == std::numeric_limits<double>::max())
        m.minInteriorV = 0.0;
    const auto p = wss->position(m.T);
    for (int ax = 0; ax < 3; ++ax) m.endPos[ax] = p[ax];
    return m;
}

void printMetrics(const char* name, const PlannedRun& r, const Metrics& m) {
    std::cout << std::fixed << std::setprecision(3)
              << "[" << name << "]"
              << " segs=" << r.numPlanningSegments
              << " pieces=" << r.numNurbsPieces
              << " L=" << r.pathLength << "mm"
              << " T=" << m.T << "s"
              << " vMax=" << m.maxV
              << " aMax=" << m.maxAbsA
              << " jMax=" << m.maxAbsJ
              << " aCent=" << m.maxCentripetal
              << " minV=" << m.minInteriorV
              << " axisV={" << m.maxAxisV[0] << "," << m.maxAxisV[1]
              << "," << m.maxAxisV[2] << "}"
              << " axisA={" << m.maxAxisA[0] << "," << m.maxAxisA[1]
              << "," << m.maxAxisA[2] << "}"
              << " solve=" << r.solveMs << "ms"
              << " arcs=" << (r.wss ? r.wss->arcs().size() : 0)
              << "\n";
    if (!r.pieceFeedRates.empty()) {
        std::cout << "    feeds={";
        for (double f : r.pieceFeedRates)
            std::cout << (std::isinf(f) ? 9999.0 : f) << ",";
        std::cout << "}\n";
    }
}

/**
 * Shared soundness assertions on the WSS arc list:
 *  - s/t/v continuity between consecutive arcs
 *  - |a| ≤ aMax on all arcs (WALL included — wall accel is slaved to the
 *    velocity limit and must respect the accel bound by construction)
 *  - |j| ≤ jMax on non-WALL arcs (WALL jerk follows the geometry)
 *  - v ≥ 0, endpoints at rest, total s-span ≈ path length
 */
void expectWssSound(const PlannedRun& r,
                    const KinematicLimits<3, double>& L,
                    double feedMmS,
                    double aTol = 0.05,
                    double jTol = 0.05) {
    ASSERT_TRUE(r.ok) << "pipeline failed at: " << r.failedAt;
    ASSERT_NE(r.wss, nullptr);
    const auto& arcs = r.wss->arcs();
    ASSERT_FALSE(arcs.empty());

    const double aMax = L.path.maxPathAcceleration;
    const double jMax = L.path.maxPathJerk;

    for (std::size_t i = 1; i < arcs.size(); ++i) {
        EXPECT_NEAR(arcs[i].s0, arcs[i - 1].s1, 1e-6)
            << "s gap at arc " << i;
        const double prevTEnd = arcs[i - 1].t0 + arcs[i - 1].duration;
        EXPECT_NEAR(arcs[i].t0, prevTEnd, 1e-6) << "t gap at arc " << i;
        if (arcs[i - 1].type != mpa::WeightedArcType::DWELL &&
            arcs[i].type != mpa::WeightedArcType::DWELL) {
            EXPECT_NEAR(arcs[i].v0, arcs[i - 1].v1, 1e-3)
                << "v gap at arc " << i;
        }
    }
    for (std::size_t i = 0; i < arcs.size(); ++i) {
        const auto& arc = arcs[i];
        EXPECT_GE(arc.v0, -1e-6) << "negative v at arc " << i;
        EXPECT_GE(arc.v1, -1e-6) << "negative v1 at arc " << i;
        EXPECT_LE(arc.v0, feedMmS * 1.05 + 0.5)
            << "v0 exceeds feed at arc " << i << ": " << arc.v0;
        EXPECT_LE(std::abs(arc.a0), aMax * (1 + aTol))
            << "a0 over limit at arc " << i << ": " << arc.a0;
        EXPECT_LE(std::abs(arc.a1), aMax * (1 + aTol))
            << "a1 over limit at arc " << i << ": " << arc.a1;
        if (arc.type != mpa::WeightedArcType::WALL) {
            EXPECT_LE(std::abs(arc.j0), jMax * (1 + jTol))
                << "j0 over limit at arc " << i << ": " << arc.j0;
            EXPECT_LE(std::abs(arc.j1), jMax * (1 + jTol))
                << "j1 over limit at arc " << i << ": " << arc.j1;
        }
    }
    EXPECT_NEAR(arcs.front().v0, 0.0, 1e-3) << "not starting at rest";
    EXPECT_NEAR(arcs.back().v1, 0.0, 1e-3) << "not ending at rest";
    EXPECT_NEAR(arcs.back().s1 - arcs.front().s0,
                r.path.totalLength(), 0.5)
        << "WSS s-span != path length";
}

/// Sampled validation: per-axis velocity/acceleration bounds + monotonic s.
void expectAxisBounds(const Metrics& m,
                      const KinematicLimits<3, double>& L,
                      double feedMmS,
                      double vTol = 0.08,
                      double aTol = 0.15) {
    EXPECT_TRUE(m.finite) << "non-finite sample in trajectory";
    EXPECT_TRUE(m.monotoneS) << "s(t) not monotone";
    EXPECT_LE(m.maxV, feedMmS * (1 + vTol) + 0.5)
        << "path velocity exceeds feed: " << m.maxV;
    EXPECT_LE(m.maxV, L.path.maxPathVelocity * (1 + vTol) + 0.5)
        << "path velocity exceeds path vmax: " << m.maxV;
    for (int ax = 0; ax < 3; ++ax) {
        EXPECT_LE(m.maxAxisV[ax], L.axis.maxVelocity[ax] * (1 + vTol) + 0.5)
            << "axis " << ax << " velocity over limit: " << m.maxAxisV[ax];
        EXPECT_LE(m.maxAxisA[ax],
                  L.axis.maxAcceleration[ax] * (1 + aTol) + 1.0)
            << "axis " << ax << " accel over limit: " << m.maxAxisA[ax];
    }
}

/// Check that velocity at the arc-length position s respects the local cap.
double velocityAtS(const mpa::WeightedSwitchingStructure<3, double>& wss,
                   double s) {
    return wss.pathVelocity(wss.timeAtArcLength(s));
}

// ============================================================================
// G-code generators
// ============================================================================

std::string header() { return "G21\nG90\n"; }

/// N×N square perimeter (closed) at feed F mm/min.
std::string squareGcode(double size, int feed) {
    std::ostringstream os;
    os << header();
    os << "G0 X0 Y0 F18000\n";
    os << "G1 X" << size << " Y0 F" << feed << "\n";
    os << "G1 X" << size << " Y" << size << "\n";
    os << "G1 X0 Y" << size << "\n";
    os << "G1 X0 Y0\n";
    return os.str();
}

/// Zigzag raster infill: `lines` horizontal passes of `len` mm.
std::string rasterGcode(int lines, double len, double spacing, int feed) {
    std::ostringstream os;
    os << header() << "G0 X0 Y0 F18000\n";
    for (int i = 0; i < lines; ++i) {
        const double y = i * spacing;
        os << "G0 X" << (i % 2 ? len : 0.0) << " Y" << y << " F18000\n";
        os << "G1 X" << (i % 2 ? 0.0 : len) << " Y" << y
           << " E" << 0.05 * len << " F" << feed << "\n";
    }
    return os.str();
}

/// Realistic small print layer: 2 perimeter loops + crosshatch infill
/// + travels + Z hop + dwell. ~700+ motion commands for `segments`≈600.
std::string benchLayerGcode(double size = 60.0) {
    std::ostringstream os;
    os << header() << "M83\n";
    const int F_TRAVEL = 18000, F_PERIM = 6000, F_INFILL = 9000,
              F_Z = 900, F_SLOW = 1500;
    const double z = 0.3;

    // Skirt + 3 perimeter loops, each with a few dozen points (rounded).
    for (int loop = 0; loop < 3; ++loop) {
        const double inset = 2.0 * loop;
        const double r = std::max(4.0, size * 0.12 - inset * 0.4);
        const double s = size - 2 * inset;
        os << "G0 Z" << (z + 0.4) << " F" << F_Z << "\n";
        // Rounded rectangle built from many small chords (like a slicer).
        const int perCorner = 12;
        const double cx[4] = {r, s - r, s - r, r};
        const double cy[4] = {r, r, s - r, s - r};
        const double a0[4] = {M_PI, 1.5 * M_PI, 0.0, 0.5 * M_PI};
        bool first = true;
        for (int c = 0; c < 4; ++c) {
            for (int j = 0; j < perCorner; ++j) {
                const double a = a0[c] + (j + 1) *
                    (M_PI / 2.0) / perCorner;
                const double x = cx[c] + r * std::cos(a);
                const double y = cy[c] + r * std::sin(a);
                if (first) {
                    os << "G0 X" << x << " Y" << y << " F" << F_TRAVEL
                       << "\nG1 Z" << z << " F" << F_Z << "\n";
                    first = false;
                }
                os << "G1 X" << x << " Y" << y << " E0.05 F" << F_PERIM
                   << "\n";
            }
        }
        os << "G4 P100 ; wipe\n";
    }

    // Crosshatch infill — dense 45°-ish rasters.
    for (int pass = 0; pass < 2; ++pass) {
        for (double d = -size; d < size; d += 4.0) {
            double x0 = std::max(0.0, d), x1 = std::min(size, d + size);
            double y0 = x0 - d, y1 = x1 - d;
            if (pass == 1) { std::swap(x0, x1); std::swap(y0, y1); }
            os << "G0 X" << x0 << " Y" << y0 << " F" << F_TRAVEL << "\n";
            os << "G1 X" << x1 << " Y" << y1 << " E0.06 F" << F_INFILL
               << "\n";
            if (static_cast<int>(d) % 12 == 0)
                os << "G4 P50 ; pause\n";
        }
    }

    // A few slow "bridge" lines at low feed.
    for (double y = 10; y < size; y += 20) {
        os << "G0 X4 Y" << y << " F" << F_TRAVEL << "\n";
        os << "G1 X" << (size - 4) << " Y" << y << " E0.08 F" << F_SLOW
           << "\n";
    }
    os << "G0 Z" << (z + 2) << " F" << F_Z << "\nM84\n";
    return os.str();
}

/// Dense polyline: n tiny chords following a wiggle (organic coastline).
std::string densePolylineGcode(int n, double step, double amp,
                               double period, int feed) {
    std::ostringstream os;
    os << header() << "G0 X0 Y0 F18000\n";
    for (int i = 1; i <= n; ++i) {
        const double x = i * step;
        const double y = amp * std::sin(2 * M_PI * i / period);
        os << "G1 X" << x << " Y" << y << " E0.002 F" << feed << "\n";
    }
    return os.str();
}

/// Spiral-vase style contour: polygonal circle climbing in Z via G1.
std::string spiralGcode(int steps, double radius, double zClimb,
                        int feed) {
    std::ostringstream os;
    os << header();
    os << "G0 X" << radius << " Y0 Z0 F18000\n";
    for (int i = 1; i <= steps; ++i) {
        const double a = 2 * M_PI * i / steps;
        const double z = zClimb * i / steps;
        os << "G1 X" << radius * std::cos(a) << " Y" << radius * std::sin(a)
           << " Z" << z << " E0.004 F" << feed << "\n";
    }
    return os.str();
}

/// CNC drilling pattern: n holes on a grid, plunge/dwell/retract.
std::string drillGcode(int nx, int ny, double pitch, double depth) {
    std::ostringstream os;
    os << header() << "G0 Z5 F3000\n";
    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            os << "G0 X" << ix * pitch << " Y" << iy * pitch << "\n";
            os << "G1 Z" << -depth << " F600\n";
            os << "G4 P150\n";
            os << "G0 Z5\n";
        }
    }
    return os.str();
}

} // namespace

// ============================================================================
// EASY — fundamental building blocks
// ============================================================================

// 1. Single extrusion line: the simplest possible plan.
TEST(PlannerScenarios_Easy, SingleExtrusionLine) {
    const auto L = printerLimits();
    auto r = runGcode(header() + "G1 X40 Y0 E1.5 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("SingleLine", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);

    // Feed cap F3000 = 50 mm/s must bind on the segment.
    EXPECT_LE(m.maxV, 50.0 * 1.05 + 0.1);
    EXPECT_GT(m.maxV, 40.0) << "planner never reached feed cap";
    // Endpoint reached.
    EXPECT_NEAR(m.endPos[0], 40.0, 0.2);
    EXPECT_NEAR(m.endPos[1], 0.0, 0.2);
    // Time lower bound: cannot beat constant-feed traversal.
    EXPECT_GE(m.T, 40.0 / 50.0);
    EXPECT_LT(m.T, 2.5) << "implausibly slow for a 40mm line";
}

// 2. L-shaped two-line corner (90°).
TEST(PlannerScenarios_Easy, TwoLineCorner) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G1 X20 Y0 E0.6 F6000\n"
        "G1 X20 Y20 E0.6 F6000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("LCorner", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);

    EXPECT_LT(m.minInteriorV, 95.0)
        << "no slowdown at the 90° corner";
    EXPECT_NEAR(m.endPos[0], 20.0, 0.2);
    EXPECT_NEAR(m.endPos[1], 20.0, 0.2);
}

// 3. Closed square perimeter — 4 symmetric corners.
TEST(PlannerScenarios_Easy, SquarePerimeter) {
    const auto L = printerLimits();
    auto r = runGcode(squareGcode(25.0, 7200), L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("Square", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);

    // Junction-velocity model must dip below feed at corners.
    EXPECT_LT(m.minInteriorV, 110.0);
    // Closed loop: endpoint = start point.
    EXPECT_NEAR(m.endPos[0], 0.0, 0.3);
    EXPECT_NEAR(m.endPos[1], 0.0, 0.3);
}

// 4. Rapid travel between extrusions: F on G0 is ignored, path vmax binds.
//    FINDING (conservative bounds): the Pareto planner collapses each
//    dwell-separated interval to the interval's *minimum* velocity
//    limit (globalBounds → min_s vLim). The uncapped rapid shares an
//    interval with 50 mm/s extrusion moves, so it is clamped to 50 too.
TEST(PlannerScenarios_Easy, RapidTravelUsesMachineMax) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G1 X10 Y0 E0.3 F3000\n"
        "G0 X60 Y0 F3000\n"          // rapid, F3000 → uncapped
        "G1 X70 Y0 E0.3 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("Rapid", r, m);
    expectWssSound(r, L, 200.0);
    // The rapid leg must exceed the G1 feed (50 mm/s) — path vmax = 200.
    EXPECT_GT(m.maxV, 120.0) << "rapid move did not use machine vmax";
    EXPECT_LE(m.maxV, 200.0 * 1.05 + 0.5);
}

// 5. G2 semicircle: centripetal limit decides velocity on the arc.
TEST(PlannerScenarios_Easy, ArcSemicircle) {
    const auto L = printerLimits();
    // R15, F3000 (50 mm/s). Centripetal cap = sqrt(2000·15) ≈ 173 → feed binds.
    auto r = runGcode(
        header() +
        "G1 X5 Y15 F3000\n"
        "G2 X35 Y15 I15 J0 E2.0 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("ArcSemi", r, m);
    expectWssSound(r, L, 200.0);
    EXPECT_LE(m.maxV, 50.0 * 1.08 + 0.2);
    EXPECT_LE(m.maxCentripetal,
              L.path.maxCentripetalAcceleration * 1.10 + 1.0);
    EXPECT_NEAR(m.endPos[0], 35.0, 0.3);
    EXPECT_NEAR(m.endPos[1], 15.0, 0.3);
}

// 6. Full circle in one G2 — closed arc, zero net displacement.
TEST(PlannerScenarios_Easy, FullCircleArc) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G0 X20 Y0 F18000\n"
        "G2 X20 Y0 I-5 J0 E3.0 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("FullCircle", r, m);
    // DESIRED: the arc becomes a NURBS piece (2 pieces total incl. G0).
    // FINDING: pieces == 1 — the coincident start/end makes the chord
    // length 0, so the converter's zero-length guard silently drops the
    // full circle before the arc branch is reached.
    EXPECT_GE(r.numNurbsPieces, 2u)
        << "full-circle G2 dropped by zero-chord-length check";
}

// 6b. Same circle as two semicircle arcs — nonzero chords, so both
//     arcs survive conversion. Feed (50) < centripetal cap
//     (sqrt(2000·10) ≈ 141) → feed binds on the arcs.
TEST(PlannerScenarios_Easy, TwoSemicircleArcs) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G0 X10 Y0 F18000\n"
        "G2 X30 Y0 I10 J0 E1.5 F3000\n"
        "G2 X10 Y0 I-10 J0 E1.5 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("SemiCircle2", r, m);
    expectWssSound(r, L, 200.0);
    ASSERT_GE(r.numNurbsPieces, 3u);
    EXPECT_LE(m.maxV, 50.0 * 1.08 + 0.2);
    EXPECT_GT(m.T, 2 * M_PI * 10.0 / 50.0 * 0.8);
    EXPECT_NEAR(m.endPos[0], 10.0, 0.3);
}

// 7. Tiny-radius circle (two semicircle arcs): centripetal dominates,
//    velocity must collapse below the feed cap.
TEST(PlannerScenarios_Easy, TinyRadiusCircle) {
    const auto L = printerLimits();
    // R1: centripetal cap = sqrt(2000·1) ≈ 44.7 mm/s < feed 50.
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G2 X2 Y0 I1 J0 E0.3 F3000\n"
        "G2 X0 Y0 I-1 J0 E0.3 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("TinyCircle", r, m);
    expectWssSound(r, L, 200.0);
    EXPECT_LE(m.maxV, 45.5) << "centripetal limit not enforced on R1";
    EXPECT_LE(m.maxCentripetal,
              L.path.maxCentripetalAcceleration * 1.10 + 1.0);
    EXPECT_GE(m.T, 2 * M_PI * 1.0 / 44.7 * 0.9);
}

// 8. Z-only move with slow Z axis: axis limit binds, not feed.
TEST(PlannerScenarios_Easy, SlowZAxisMove) {
    const auto L = printerLimits();   // Z vmax = 20 mm/s
    auto r = runGcode(
        header() +
        "G1 Z5 F3000\n"               // F=50mm/s but Z axis capped at 20
        "G1 X20 Y0 E0.6 F3000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("SlowZ", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);
    // Z leg alone ≥ 5mm / 20mm/s = 0.25s (plus ramps).
    EXPECT_GE(m.T, 0.25 + 20.0 / 50.0);
    EXPECT_LE(m.maxAxisV[2], 20.0 * 1.08 + 0.2);
}

// ============================================================================
// MEDIUM — realistic print patterns
// ============================================================================

// 9. Raster infill with rapids between (classic rectilinear fill).
//    Stepover is 60 mm so the travel hops have enough distance to reach
//    machine speed even with jerk-limited acceleration (a 4 mm hop
//    physically cannot exceed ~90 mm/s at a=2000/j=20000).
TEST(PlannerScenarios_Medium, InfillRaster) {
    const auto L = printerLimits();
    auto r = runGcode(rasterGcode(8, 60.0, 60.0, 5400), L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("Infill", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);
    // Infill feed = 90 mm/s; travels reach 200.
    EXPECT_GT(m.maxV, 150.0) << "travels not using machine speed";
}

// 10. Mixed per-segment feed rates on one loop (perimeter vs bridge).
TEST(PlannerScenarios_Medium, MixedFeedRatesHonored) {
    const auto L = printerLimits();
    // Two adjacent segments: slow F1200 (20mm/s) and fast F12000 (200mm/s).
    // FINDING: the fast leg inherits the slow leg's cap — the interval
    // velocity ceiling is min over the whole solve interval.
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G1 X30 Y0 E0.5 F1200\n"      // slow: 20 mm/s
        "G1 X90 Y0 E1.0 F12000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("MixedFeed", r, m);
    expectWssSound(r, L, 200.0);
    ASSERT_TRUE(r.ok);
    ASSERT_NE(r.wss, nullptr);

    for (double s : {1.0, 29.0, 31.0, 60.0, 89.0})
        std::cout << "  segVLim@" << s << " = "
                  << r.path.maxVelocityAtArcLength(s) << "\n";
    for (const auto& si : r.path.segments())
        std::cout << "  segment cumLen=" << si.cumulativeArcLength
                  << " arcLen=" << si.arcLength
                  << " vMax=" << si.maxVelocity << "\n";

    // Find the slow segment's arc-length interior (s in (5, 25)mm) and
    // verify the velocity cap of 20 mm/s is respected there.
    double slowMaxV = 0.0;
    for (double s = 6.0; s < 25.0; s += 0.5) {
        slowMaxV = std::max(slowMaxV, velocityAtS(*r.wss, s));
    }
    EXPECT_LE(slowMaxV, 20.0 * 1.15 + 0.5)
        << "per-segment feed ignored (v=" << slowMaxV << ")";
    // Fast leg must exceed the slow feed.
    EXPECT_GT(m.maxV, 60.0);
}

// 11. G4 dwell mid-path: must decelerate to rest, pause, resume.
TEST(PlannerScenarios_Medium, DwellMidPath) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G1 X25 Y0 E0.5 F6000\n"
        "G4 P400\n"                    // 0.4 s dwell
        "G1 X50 Y0 E0.5 F6000\n", L, 200.0);
    auto m = sampleWss(r.wss, 8000);
    printMetrics("Dwell", r, m);
    expectWssSound(r, L, 200.0);
    ASSERT_TRUE(r.ok);
    ASSERT_NE(r.wss, nullptr);
    ASSERT_EQ(r.numDwellPoints, 1u);

    // A DWELL arc exists with the right duration.
    int dwellArcs = 0;
    double dwellTime = 0.0;
    for (const auto& arc : r.wss->arcs()) {
        if (arc.type == mpa::WeightedArcType::DWELL) {
            ++dwellArcs;
            dwellTime += arc.duration;
            EXPECT_NEAR(arc.v0, 0.0, 1e-9);
            EXPECT_NEAR(arc.v1, 0.0, 1e-9);
            EXPECT_NEAR(arc.s1 - arc.s0, 0.0, 1e-9);
        }
    }
    EXPECT_EQ(dwellArcs, 1);
    EXPECT_NEAR(dwellTime, 0.4, 1e-3);
    // Total time includes motion + dwell.
    EXPECT_GE(m.T, 0.4 + 2 * (25.0 / 100.0));
}

// 12. Consecutive dwells (back-to-back G4s).
TEST(PlannerScenarios_Medium, ConsecutiveDwells) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G1 X20 Y0 F6000\n"
        "G4 P200\n"
        "G4 P300\n"
        "G1 X40 Y0 F6000\n", L, 200.0);
    auto m = sampleWss(r.wss, 8000);
    printMetrics("Dwell2", r, m);
    expectWssSound(r, L, 200.0);
    ASSERT_TRUE(r.ok);
    ASSERT_NE(r.wss, nullptr);

    int dwellArcs = 0;
    double dwellTime = 0.0;
    for (const auto& arc : r.wss->arcs()) {
        if (arc.type == mpa::WeightedArcType::DWELL) {
            ++dwellArcs;
            dwellTime += arc.duration;
        }
    }
    EXPECT_EQ(dwellArcs, 2);
    EXPECT_NEAR(dwellTime, 0.5, 1e-3);
}

// 13. Spiral-vase contour: polygonal circle with simultaneous Z climb.
TEST(PlannerScenarios_Medium, SpiralVaseLayer) {
    const auto L = printerLimits();
    auto r = runGcode(spiralGcode(96, 25.0, 0.9, 4800), L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("SpiralVase", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);
    // Continuous spiral — velocity should stay near feed (80 mm/s),
    // no full stop in the interior.
    EXPECT_GT(m.minInteriorV, 20.0)
        << "spiral stalls — junction model over-braking polyline";
    EXPECT_NEAR(m.endPos[2], 0.9, 0.1);
}

// 14. Acute 30° corner — deeper slowdown than 90°.
TEST(PlannerScenarios_Medium, AcuteCorner) {
    const auto L = printerLimits();
    const double th = M_PI / 6.0;  // 30°
    const double ex = 30.0 + 20.0 * std::cos(th);
    const double ey = 20.0 * std::sin(th);
    std::ostringstream os;
    os << header()
       << "G1 X30 Y0 F9000\n"
       << "G1 X" << ex << " Y" << ey << " F9000\n";
    auto r = runGcode(os.str(), L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("Acute30", r, m);
    expectWssSound(r, L, 200.0);

    // Compare with the 90° case: junction v = sqrt(a·δ/sin(θ/2)):
    // sin(75°)≈0.966 vs sin(45°)≈0.707 → similar magnitude, both deep dips.
    EXPECT_LT(m.minInteriorV, 110.0);
    EXPECT_NEAR(m.endPos[0], ex, 0.3);
    EXPECT_NEAR(m.endPos[1], ey, 0.3);
}

// 15. Line + arc + line — arc lead-in profile (CNC-style but printable).
//    FINDING: the line→arc tangent discontinuity collapses the global
//    accel/jerk bounds for the whole interval (observed a≈3 mm/s²,
//    j≈1 mm/s³ → T≈12 s for 56 mm). Sound, but pathologically slow.
TEST(PlannerScenarios_Medium, LineArcLineBlend) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G1 X20 Y0 E0.4 F6000\n"
        "G3 X30 Y10 I0 J10 E0.6 F6000\n"  // quarter arc R10 CCW
        "G1 X30 Y30 E0.6 F6000\n", L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("LineArcLine", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0);
    EXPECT_LE(m.maxCentripetal,
              L.path.maxCentripetalAcceleration * 1.10 + 1.0);
    EXPECT_NEAR(m.endPos[0], 30.0, 0.3);
    EXPECT_NEAR(m.endPos[1], 30.0, 0.3);
}

// 16. Dense polyline — 250 × 0.3mm chords (textured-wall / organic path).
TEST(PlannerScenarios_Medium, DensePolylineWiggle) {
    const auto L = printerLimits();
    auto r = runGcode(densePolylineGcode(250, 0.3, 0.4, 40, 9000),
                      L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("DensePoly", r, m);
    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0, 0.10, 0.30);
    EXPECT_LT(r.solveMs, 30000.0) << "solver too slow on dense path";
}

// ============================================================================
// CNC — 3-axis machining patterns
// ============================================================================

// 17. Square pocket with Z plunge: line path at depth.
TEST(PlannerScenarios_CNC, SquarePocketAtDepth) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G0 X0 Y0 Z2 F4800\n"
        "G1 Z-2 F600\n"                 // plunge
        "G1 X40 Y0 F2400\n"
        "G1 X40 Y40 F2400\n"
        "G1 X0 Y40 F2400\n"
        "G1 X0 Y0 F2400\n"
        "G0 Z5\n", L, 80.0);
    auto m = sampleWss(r.wss);
    printMetrics("Pocket", r, m);
    expectWssSound(r, L, 80.0);
    expectAxisBounds(m, L, 80.0);
    EXPECT_NEAR(m.endPos[2], 5.0, 0.3);
    // Cutting feed = 40 mm/s on XY legs.
    EXPECT_LE(m.maxV, 80.0 * 1.08 + 0.5);
}

// 18. Drilling pattern: 6 holes, plunge + dwell + retract each.
TEST(PlannerScenarios_CNC, DrillingPattern) {
    const auto L = cncLimits();
    auto r = runGcode(drillGcode(3, 2, 25.0, 3.0), L, 80.0);
    auto m = sampleWss(r.wss, 8000);
    printMetrics("Drill", r, m);
    expectWssSound(r, L, 80.0);
    ASSERT_TRUE(r.ok);
    ASSERT_NE(r.wss, nullptr);

    int dwellArcs = 0;
    double dwellTime = 0.0;
    for (const auto& arc : r.wss->arcs()) {
        if (arc.type == mpa::WeightedArcType::DWELL) {
            ++dwellArcs; dwellTime += arc.duration;
        }
    }
    EXPECT_EQ(dwellArcs, 6);
    EXPECT_NEAR(dwellTime, 6 * 0.15, 1e-3);
    EXPECT_LE(m.maxAxisV[2], 40.0 * 1.10 + 0.5);
}

// 19. Helical arc (G2 with Z descent) — probe: does the arc keep the Z
//     ramp? The PlanningSegment→NURBS converter builds planar arcs only;
//     a helical G2 drops Z silently. This test documents the real behavior.
TEST(PlannerScenarios_CNC, HelicalArcZDrop) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G0 X10 Y0 Z0 F4800\n"
        "G2 X10 Y0 I-5 J0 Z-3 F2400\n", L, 80.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("HelixArc", r, m);

    // DESIRED: endpoint Z reaches -3 (helix preserved).
    // If this fails, the converter flattened the helix to a planar arc.
    EXPECT_NEAR(m.endPos[2], -3.0, 0.2)
        << "helical G2 lost its Z ramp (planar-arc conversion defect)";

    // Quantify helix fidelity: a true helix has Z linear in the in-plane
    // angle. The liftArcToHelix ramp is exact at control points; inside
    // each ≤120° span it deviates slightly. Center = (5,0).
    double maxDev = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        const double s = 10.5 + (r.pathLength - 10.5) * i / 2000.0;
        const auto p = r.path.evaluateAtArcLength(s);
        const double ang = std::atan2(p.position[1] - 0.0,
                                      p.position[0] - 5.0);
        double frac = -ang / (2.0 * M_PI);  // CW: angle decreases from 0
        while (frac < 0.0) frac += 1.0;
        const double zExpected = -3.0 * frac;
        maxDev = std::max(maxDev, std::abs(p.position[2] - zExpected));
    }
    std::cout << "[HelixArc] max Z deviation from true helix: "
              << maxDev << " mm\n";
    // ~30° lift spans keep the ramp within a few µm of linear-in-angle.
    EXPECT_LT(maxDev, 0.01) << "helix Z deviates too far from linear";
}

// 20. Full 3-axis diagonal move: all axis limits could bind.
TEST(PlannerScenarios_CNC, Diagonal3DMove) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() + "G1 X60 Y40 Z-20 F4800\n", L, 80.0);
    auto m = sampleWss(r.wss);
    printMetrics("Diagonal3D", r, m);
    expectWssSound(r, L, 80.0);
    expectAxisBounds(m, L, 80.0);
    // Tangent ≈ (0.76,0.50,0.40): Z axis vmax=40 → path v ≤ 40/0.40 ≈ 100.
    // Feed = 80 mm/s binds.
    EXPECT_NEAR(m.endPos[0], 60.0, 0.3);
    EXPECT_NEAR(m.endPos[1], 40.0, 0.3);
    EXPECT_NEAR(m.endPos[2], -20.0, 0.3);
}

// 21. Facing passes: long parallel passes joined by 180° hairpins —
//     velocity must nearly stall at each turn.
TEST(PlannerScenarios_CNC, FacingPassesHairpins) {
    const auto L = cncLimits();
    std::ostringstream os;
    os << header() << "G0 X0 Y0 Z-1 F4800\n";
    for (int i = 0; i < 5; ++i) {
        const double y = i * 8.0;
        const double xs = (i % 2) ? 100.0 : 0.0;
        const double xe = (i % 2) ? 0.0 : 100.0;
        os << "G1 X" << xs << " Y" << y << " F3000\n";
        os << "G1 X" << xe << " Y" << y << " F3000\n";
    }
    auto r = runGcode(os.str(), L, 80.0);
    auto m = sampleWss(r.wss);
    printMetrics("Facing", r, m);
    expectWssSound(r, L, 80.0);
    expectAxisBounds(m, L, 80.0);
    // 180° turns: sin(θ/2)≈1 → v_corner = sqrt(400·0.05) ≈ 4.5 mm/s.
    EXPECT_LT(m.minInteriorV, 40.0)
        << "no slowdown at 180° hairpins";
}

// 22. G18 arc in the XZ plane — off-XY arc plane.
TEST(PlannerScenarios_CNC, XZPlaneArc) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G18\n"
        "G0 X10 Y0 Z0 F4800\n"
        "G2 X20 Z0 I5 K0 F2400\n"      // semicircle in XZ: center (15,0)
        "G17\n", L, 80.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("XZArc", r, m);
    EXPECT_NEAR(m.endPos[0], 20.0, 0.3);
    EXPECT_NEAR(m.endPos[2], 0.0, 0.3);
    // Previously the arc was built in the wrong plane giving T~6.7e5 s.
    EXPECT_LT(m.T, 60.0);
    // The rapid->arc seam must not collapse to a full stop: the bound
    // pair is kept self-consistent (stoppable within the piece), so the
    // junction holds ~38 mm/s (was 0 when the starved accel bound made
    // every transition look infeasible and got equalized to a stop).
    EXPECT_GT(m.minInteriorV, 1.0);
    EXPECT_LT(m.T, 2.5);
}

// 23. G91 incremental mode — relative moves must accumulate correctly.
TEST(PlannerScenarios_CNC, IncrementalMode) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G91\n"
        "G1 X20 Y0 F2400\n"
        "G1 X0 Y20 F2400\n"
        "G1 X-20 Y0 F2400\n"
        "G90\n", L, 80.0);
    auto m = sampleWss(r.wss);
    printMetrics("G91", r, m);
    expectWssSound(r, L, 80.0);
    EXPECT_NEAR(m.endPos[0], 0.0, 0.3);
    EXPECT_NEAR(m.endPos[1], 20.0, 0.3);
}

// ============================================================================
// HARD / SUPERHARD — stress + defect probes
// ============================================================================

// 24. 500 micro-segments (0.1mm each): solver piece-count stress.
TEST(PlannerScenarios_Hard, MicroSegments500) {
    const auto L = printerLimits();
    auto r = runGcode(densePolylineGcode(500, 0.1, 0.05, 25, 6000),
                      L, 200.0);
    auto m = sampleWss(r.wss);
    printMetrics("Micro500", r, m);
    expectWssSound(r, L, 200.0);
    EXPECT_LT(r.solveMs, 60000.0);
}

// 25. Full realistic bench layer: ~700 mixed segments (perimeters with
//     dense chords, crosshatch infill, travels, dwells, Z hops).
TEST(PlannerScenarios_Hard, BenchLayerFullValidation) {
    const auto L = printerLimits();
    const std::string gcode = benchLayerGcode(60.0);
    auto r = runGcode(gcode, L, 200.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at: " << r.failedAt;
    auto m = sampleWss(r.wss, 12000);
    printMetrics("BenchLayer", r, m);

    // Diagnostic: planning-segment composition.
    {
        auto parsed = GCode::PlanningSegmentBuilder::fromText(gcode);
        std::map<int, int> typeCount;
        int zeroLen = 0;
        for (const auto& s : parsed.segments) {
            ++typeCount[static_cast<int>(s.motionType)];
            if (s.segmentLength < 1e-6) ++zeroLen;
        }
        for (auto& [t, n] : typeCount)
            std::cout << "  motionType " << t << " x" << n << "\n";
        std::cout << "  zeroLength=" << zeroLen << "\n";
    }

    expectWssSound(r, L, 200.0);
    expectAxisBounds(m, L, 200.0, 0.10, 0.25);
    // Every planning segment is accounted for: it becomes a NURBS piece,
    // a dwell point, or is skipped as a sub-micron zero-length motion.
    {
        auto parsed = GCode::PlanningSegmentBuilder::fromText(gcode);
        std::size_t expectedSkipped = 0;
        for (const auto& s : parsed.segments) {
            if (s.motionType != GCode::SegmentMotionType::Dwell &&
                s.segmentLength < 1e-6) {
                ++expectedSkipped;
            }
        }
        EXPECT_EQ(r.numPlanningSegments - expectedSkipped,
                  r.numNurbsPieces + r.numDwellPoints)
            << "unaccounted planning segments";
        EXPECT_EQ(expectedSkipped, 5u);
    }
    EXPECT_GE(r.numDwellPoints, 3u);
    // Sanity: the whole layer can't take less time than length/feed.
    EXPECT_GT(m.T, r.pathLength / 200.0);
}

// 26. Spline flattening probe: G5 cubic spline through the production
//     pipeline — the converter emits SPLINE segments but
//     piecewiseNurbsFromSegments turns them into straight chords.
TEST(PlannerScenarios_Hard, SplinePreservesShape) {
    const auto L = printerLimits();
    // LinuxCNC G5: cubic Bezier via I,J (first ctrl pt offset) and P,Q
    // (second ctrl pt offset) relative to start/end.
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G5 X40 Y0 I10 J20 P30 Q-20 E1.0 F6000\n", L, 200.0);
    if (!r.ok) {
        GTEST_SKIP() << "pipeline rejected spline at: " << r.failedAt;
    }
    auto m = sampleWss(r.wss);
    printMetrics("Spline", r, m);
    // DESIRED: path deviates from the chord (control points pull it).
    // Sample path midpoint — should be well off the X axis.
    const auto mid = r.path.evaluateAtArcLength(r.pathLength * 0.5);
    EXPECT_GT(std::abs(mid.position[1]), 1.0)
        << "G5 spline flattened to a straight chord (shape lost)";
}

// 27. MotionPlanBuilder pipeline (the other entry point): blended path +
//     evaluateAt sampling + feed override scaling.
TEST(PlannerScenarios_Hard, MotionPlanBuilderPath) {
    auto L = printerLimits();

    MotionSegmentList segs;
    segs.append(MotionSegment::linear(
        Vec<3, double>{0, 0, 0}, Vec<3, double>{30, 0, 0}, 100.0));
    segs.append(MotionSegment::linear(
        Vec<3, double>{30, 0, 0}, Vec<3, double>{30, 30, 0}, 100.0));
    segs.append(MotionSegment::linear(
        Vec<3, double>{30, 30, 0}, Vec<3, double>{0, 30, 0}, 100.0));

    MotionPlanBuilder3D builder(L);
    auto plan = builder.build(segs, 100.0);
    ASSERT_GT(plan.numSegments(), 0u);
    ASSERT_GT(plan.totalDuration(), 0.0);

    // Sample the whole plan; check bounds + continuity.
    const double T = plan.totalDuration();
    double maxV = 0, maxAxV[3] = {0, 0, 0}, prevX = 0;
    for (int i = 0; i <= 4000; ++i) {
        const double t = T * i / 4000.0;
        const auto s = plan.evaluateAt(t);
        maxV = std::max(maxV, std::abs(s.pathVelocity));
        for (int ax = 0; ax < 3; ++ax)
            maxAxV[ax] = std::max(maxAxV[ax], std::abs(s.velocity[ax]));
        if (i > 0) {
            EXPECT_FALSE(std::isnan(s.position[0])) << "NaN at t=" << t;
            EXPECT_LE(std::abs(s.position[0] - prevX), 30.0);
        }
        prevX = s.position[0];
    }
    EXPECT_LE(maxV, 100.0 * 1.10 + 0.5);
    for (int ax = 0; ax < 3; ++ax)
        EXPECT_LE(maxAxV[ax], L.axis.maxVelocity[ax] * 1.10 + 0.5);

    // Feed override halves throughput.
    const double t0 = plan.totalDuration();
    plan.setFeedOverride(0.5);
    EXPECT_NEAR(plan.totalDuration(), 2.0 * t0, 0.02 * t0);

    std::cout << "[MotionPlanBuilder] T=" << T
              << "s pieces=" << plan.numSegments()
              << " maxV=" << maxV << "\n";
}

// 28. Dwell through MotionPlanBuilder: PathBuilderAdapter now keeps
//     G4 dwells as path dwell points (previously silently dropped),
//     so the plan's total time includes the 0.5s pause.
TEST(PlannerScenarios_Hard, DwellViaMotionPlanBuilder) {
    auto L = printerLimits();

    MotionSegmentList segs;
    segs.append(MotionSegment::linear(
        Vec<3, double>{0, 0, 0}, Vec<3, double>{25, 0, 0}, 100.0));
    segs.append(MotionSegment::dwell(
        0.5, std::array<double, MAX_MOTION_AXES>{25, 0, 0}));
    segs.append(MotionSegment::linear(
        Vec<3, double>{25, 0, 0}, Vec<3, double>{50, 0, 0}, 100.0));

    MotionPlanBuilder3D builder(L);
    auto plan = builder.build(segs, 100.0);
    ASSERT_GT(plan.numSegments(), 0u);

    const double total = plan.totalDuration();
    std::cout << "[DwellBuilder] totalDuration=" << total << "s\n";
    // Motion alone would take well under 1s; with the dwell the plan
    // must total at least motion + 0.5s.
    EXPECT_GT(total, 0.9)
        << "dwell still dropped through MotionPlanBuilder";
}

// 39. Mixed per-segment feeds through MotionPlanBuilder: per-segment
//     velocity limits are now plumbed through PathBuilderAdapter, so
//     the fast leg must reach its own feed (junction split) instead of
//     collapsing to the global minimum.
TEST(PlannerScenarios_Hard, MixedFeedsViaMotionPlanBuilder) {
    auto L = printerLimits();

    MotionSegmentList segs;
    segs.append(MotionSegment::linear(
        Vec<3, double>{0, 0, 0}, Vec<3, double>{100, 0, 0}, 20.0));
    segs.append(MotionSegment::linear(
        Vec<3, double>{100, 0, 0}, Vec<3, double>{200, 0, 0}, 150.0));

    MotionPlanBuilder3D builder(L);
    auto plan = builder.build(segs, 150.0);
    ASSERT_GT(plan.numSegments(), 0u);
    const double T = plan.totalDuration();
    double maxV = 0;
    for (int i = 0; i <= 4000; ++i) {
        const auto s = plan.evaluateAt(T * i / 4000.0);
        maxV = std::max(maxV, std::abs(s.pathVelocity));
    }
    std::cout << "[MixedFeedBuilder] T=" << T << "s maxV=" << maxV
              << "\n";
    EXPECT_GT(maxV, 60.0)
        << "fast leg collapsed to the slow leg's feed (no split)";
    EXPECT_LE(maxV, 150.0 * 1.10 + 0.5);
}

// 29. Cost-weight sweep: heavier energy weight must slow the trajectory
//     monotonically (Pareto front sanity).
TEST(PlannerScenarios_Hard, EnergyWeightSlowsTrajectory) {
    const auto L = printerLimits();
    const std::string gcode = squareGcode(30.0, 6000);

    mpa::CostWeights fast;  fast.w_t = 1.0; fast.w_j = 0.0; fast.w_a = 0.0;
    mpa::CostWeights heavy; heavy.w_t = 1.0; heavy.w_j = 1e-6; heavy.w_a = 1e-3;

    auto rFast = runGcode(gcode, L, 200.0, 0.05, 0, fast);
    auto rHeavy = runGcode(gcode, L, 200.0, 0.05, 0, heavy);
    ASSERT_TRUE(rFast.ok && rHeavy.ok);
    ASSERT_NE(rFast.wss, nullptr);
    ASSERT_NE(rHeavy.wss, nullptr);
    const double tFast = rFast.wss->totalTime();
    const double tHeavy = rHeavy.wss->totalTime();
    std::cout << "[ParetoSweep] w=time-optimal T=" << tFast
              << "s  w=energy-heavy T=" << tHeavy << "s\n";
    EXPECT_GT(tHeavy, tFast * 1.001)
        << "energy weight did not slow the trajectory";
}

// 30. The same path under printer vs CNC limits — total time must grow
//     when limits tighten. High-feed raster so the machine velocity cap
//     (200 vs 80 mm/s) is the binding constraint, not the F-word.
TEST(PlannerScenarios_Hard, TighterLimitsSlowerTime) {
    const std::string gcode = rasterGcode(6, 60.0, 4.0, 18000);
    auto rPrint = runGcode(gcode, printerLimits(), 200.0);
    auto rCnc = runGcode(gcode, cncLimits(), 80.0);
    ASSERT_TRUE(rPrint.ok && rCnc.ok);
    ASSERT_NE(rPrint.wss, nullptr);
    ASSERT_NE(rCnc.wss, nullptr);
    const double tPrint = rPrint.wss->totalTime();
    const double tCnc = rCnc.wss->totalTime();
    std::cout << "[LimitSweep] printer T=" << tPrint
              << "s  cnc T=" << tCnc << "s\n";
    EXPECT_GT(tCnc, tPrint * 1.5)
        << "CNC limits should slow the same path substantially";
}

// 31. Multi-turn arcs (P word): a P3 full-circle move must cover three
//     turns of arc length; helical P-turns split the Z travel evenly.
TEST(PlannerScenarios_Hard, MultiTurnArcPWord) {
    const auto L = printerLimits();
    // 3 full turns of an R10 circle: L = 3*2π*10 ≈ 188.5 mm.
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G2 X0 Y0 Z-9 I10 J0 P3 F6000\n", L, 200.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("MultiTurn", r, m);
    EXPECT_NEAR(r.pathLength, 3.0 * 2.0 * M_PI * 10.0, 2.0);
    // Helical: Z drops 9 mm over 3 turns → end Z = -9.
    EXPECT_NEAR(m.endPos[2], -9.0, 0.3);
    // Mid-way through the motion Z should be about half-way down.
    const auto mid = r.path.evaluateAtArcLength(r.pathLength * 0.5);
    EXPECT_NEAR(mid.position[2], -4.5, 0.6);
}

// 32. Arc direction fidelity: the NURBS path must traverse each arc in
//     the same direction as the interpreter's own tessellation. G18's
//     (a1,a2) = (Z,X) ordering historically mirrored XZ arcs.
TEST(PlannerScenarios_Hard, ArcDirectionMatchesTessellation) {
    struct Case { const char* name; const char* gcode; };
    const Case cases[] = {
        {"XY-G2",  "G0 X0 Y0\nG2 X20 Y0 I10 J0 F3000\n"},
        {"XY-G3",  "G0 X0 Y0\nG3 X20 Y0 I10 J0 F3000\n"},
        {"XZ-G2",  "G18\nG0 X0 Z0\nG2 X20 Z0 I10 K0 F3000\n"},
        {"XZ-G3",  "G18\nG0 X0 Z0\nG3 X20 Z0 I10 K0 F3000\n"},
        {"YZ-G2",  "G19\nG0 Y0 Z0\nG2 Y20 Z0 J10 K0 F3000\n"},
        {"YZ-G3",  "G19\nG0 Y0 Z0\nG3 Y20 Z0 J10 K0 F3000\n"},
    };
    for (const auto& c : cases) {
        // Tessellated reference (interpreter's own convention)
        GCode::Interpreter interp;
        interp.setEmitArcSegments(false);
        std::vector<GCode::MotionSegment> msegs;
        interp.setMotionCallback([&](const GCode::MotionSegment& s) {
            msegs.push_back(s); return GCode::Error{};
        });
        ASSERT_TRUE(interp.loadString(c.gcode).ok()) << c.name;
        ASSERT_TRUE(interp.run().ok()) << c.name;
        ASSERT_GT(msegs.size(), 4u) << c.name;
        const auto& mid = msegs[msegs.size() / 2].endPosition;

        // NURBS arc path
        auto parsed = GCode::PlanningSegmentBuilder::fromText(c.gcode);
        ASSERT_TRUE(parsed.error.ok()) << c.name;
        auto nurbs = piecewiseNurbsFromSegments(parsed.segments);
        const auto p = nurbs.path.evaluatePosition(
            nurbs.path.totalLength() * 0.5);
        for (int ax = 0; ax < 3; ++ax) {
            EXPECT_NEAR(p[ax], mid[ax], 0.5)
                << c.name << " axis " << ax
                << " — NURBS arc goes the wrong way around";
        }
    }
}

// 33. Inconsistent IJK arc (end not on the commanded circle): the
//     interpreter rebuilds a circle through start+end with the
//     requested radius. The repaired arc must reach the commanded end
//     exactly and keep the commanded direction/shape.
TEST(PlannerScenarios_Hard, InconsistentArcRepaired) {
    const auto L = printerLimits();
    // Commanded center (5,0), r=5; end (9,1) is sqrt(17)=4.12 away —
    // off the circle. Repair picks the candidate circle whose CW arc
    // matches the commanded minor-arc shape (bulges upward).
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G2 X9 Y1 I5 J0 F3000\n", L, 200.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("InconsistentArc", r, m);
    EXPECT_NEAR(m.endPos[0], 9.0, 0.15);
    EXPECT_NEAR(m.endPos[1], 1.0, 0.15);
    // Repaired minor arc passes above the chord (peak y > 2).
    double maxY = -1e9;
    for (int i = 0; i <= 100; ++i) {
        const auto p = r.path.evaluateAtArcLength(r.pathLength * i / 100);
        maxY = std::max(maxY, p.position[1]);
    }
    EXPECT_GT(maxY, 2.0)
        << "arc took the major-arc/major-bulge side — wrong candidate";
}

// 34. Inconsistent arc where no circle fits (chord > diameter):
//     fall back to a straight line reaching the commanded end.
TEST(PlannerScenarios_Hard, InconsistentArcFallsBackToLine) {
    const auto L = printerLimits();
    // r=5 but the 20mm chord exceeds the 10mm diameter.
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G2 X20 Y0 I5 J0 F3000\n", L, 200.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("InconsistentArcLine", r, m);
    EXPECT_NEAR(m.endPos[0], 20.0, 0.15);
    EXPECT_NEAR(m.endPos[1], 0.0, 0.15);
    // Straight-line fallback, not a curved path.
    EXPECT_NEAR(r.pathLength, 20.0, 0.5);
}

// 35. Inconsistent *helical* arc: feasibility must use the in-plane
//     chord (9.06mm < 10mm diameter → repairable), not the 3D chord
//     (10.34mm → would wrongly fall back to a line).
TEST(PlannerScenarios_Hard, InconsistentHelicalArcRepaired) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G0 X0 Y0 Z0 F18000\n"
        "G2 X9 Y1 Z-5 I5 J0 F2000\n", L, 80.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("InconsistentHelix", r, m);
    EXPECT_NEAR(m.endPos[0], 9.0, 0.15);
    EXPECT_NEAR(m.endPos[1], 1.0, 0.15);
    EXPECT_NEAR(m.endPos[2], -5.0, 0.15);
    // Arc (not line fallback) → bulges above the chord.
    double maxY = -1e9;
    for (int i = 0; i <= 100; ++i) {
        const auto p = r.path.evaluateAtArcLength(r.pathLength * i / 100);
        maxY = std::max(maxY, p.position[1]);
    }
    EXPECT_GT(maxY, 2.0)
        << "helical repair used the 3D chord and fell back to a line";
}

// 36. G5.1 quadratic spline: one control point (I,J offset from
//     start), degree-2 Bézier — must not flatten to the chord.
TEST(PlannerScenarios_Hard, QuadraticSplineG51) {
    const auto L = printerLimits();
    auto r = runGcode(
        header() +
        "G0 X0 Y0 F18000\n"
        "G5.1 X40 Y0 I10 J30 E1.0 F6000\n", L, 200.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("SplineG51", r, m);
    // Bézier(0.5) = (P0 + 2·P1 + P2)/4 = (15, 15): symmetric curve, so
    // the arc-length midpoint sits near the parametric midpoint.
    const auto mid = r.path.evaluateAtArcLength(r.pathLength * 0.5);
    EXPECT_GT(mid.position[1], 5.0)
        << "G5.1 quadratic flattened to a chord";
    EXPECT_NEAR(m.endPos[0], 40.0, 0.15);
    EXPECT_NEAR(m.endPos[1], 0.0, 0.15);
}

// 37. R-format arcs: center reconstructed from the radius. R+ picks
//     the minor arc, R- the major arc — both must reach the commanded
//     end on the correct side of the chord.
TEST(PlannerScenarios_Hard, RFormatArcs) {
    const auto L = printerLimits();
    {   // R+12: minor CW arc, center below chord → bulges +Y.
        auto r = runGcode(
            header() +
            "G0 X0 Y0 F18000\n"
            "G2 X20 Y0 R12 F3000\n", L, 200.0);
        ASSERT_TRUE(r.ok) << "minor R-arc failed at " << r.failedAt;
        auto m = sampleWss(r.wss);
        printMetrics("RArcMinor", r, m);
        EXPECT_NEAR(m.endPos[0], 20.0, 0.15);
        EXPECT_NEAR(m.endPos[1], 0.0, 0.15);
        // Regression (jerk-floor poisoning): once bounds.jerk dropped
        // below jerkHeadroom, the headroom bisection could never succeed
        // and floored the piece to j=1 mm/s³ (T ~ 9.1 s here). And when
        // the bisection only ran on sym<=0, a thin-but-positive headroom
        // sample could still crush jerk to ~0.8 (T ~ 5.9 s). A plain
        // circular arc must keep real jerk authority and finish in ~1-2 s.
        EXPECT_GT(m.maxAbsJ, 2.0)
            << "jerk bound collapsed to the 1.0 fallback floor";
        EXPECT_LT(m.T, 3.0)
            << "jerk bound collapsed — a 23mm arc should take ~1.2s";
        double maxY = -1e9;
        for (int i = 0; i <= 100; ++i) {
            maxY = std::max(maxY,
                r.path.evaluateAtArcLength(r.pathLength * i / 100)
                    .position[1]);
        }
        EXPECT_GT(maxY, 3.0) << "minor R-arc bulged the wrong way";
    }
    {   // R-12: major CW arc — center above chord (10,+6.63), so the
        // >180° traversal goes over the top of the circle (peak
        // y = 6.63+12 ≈ 18.6) and never dips below the chord.
        auto r = runGcode(
            header() +
            "G0 X0 Y0 F18000\n"
            "G2 X20 Y0 R-12 F3000\n", L, 200.0);
        ASSERT_TRUE(r.ok) << "major R-arc failed at " << r.failedAt;
        auto m = sampleWss(r.wss);
        printMetrics("RArcMajor", r, m);
        EXPECT_NEAR(m.endPos[0], 20.0, 0.15);
        EXPECT_NEAR(m.endPos[1], 0.0, 0.15);
        // Major arc: L = r · sweep ≈ 12 · 4.31 ≈ 51.7 mm.
        EXPECT_NEAR(r.pathLength, 51.7, 1.5);
        double maxY = -1e9, minY = 1e9;
        for (int i = 0; i <= 100; ++i) {
            const auto p =
                r.path.evaluateAtArcLength(r.pathLength * i / 100);
            maxY = std::max(maxY, p.position[1]);
            minY = std::min(minY, p.position[1]);
        }
        EXPECT_GT(maxY, 15.0) << "major R-arc took the minor-arc side";
        EXPECT_GT(minY, -0.5) << "major R-arc dipped below the chord";
        // Same regression guard as the minor arc: the worst observed
        // collapse took 12.8 s; correct plan is ~1.9 s.
        EXPECT_LT(m.T, 4.0)
            << "jerk bound collapsed — a 52mm arc should take ~2s";
    }
}

// 38. Negative-direction helix: CCW (G3) full turn descending in Z —
//     exercises helixDelta < 0 on a positive-sweep arc.
TEST(PlannerScenarios_Hard, NegativeHelixCCW) {
    const auto L = cncLimits();
    auto r = runGcode(
        header() +
        "G0 X10 Y0 Z0 F18000\n"
        "G3 X10 Y0 Z-4 I-10 J0 F2000\n", L, 80.0);
    ASSERT_TRUE(r.ok) << "pipeline failed at " << r.failedAt;
    auto m = sampleWss(r.wss);
    printMetrics("NegHelix", r, m);
    EXPECT_NEAR(m.endPos[0], 10.0, 0.15);
    EXPECT_NEAR(m.endPos[1], 0.0, 0.15);
    EXPECT_NEAR(m.endPos[2], -4.0, 0.15);
    // 10mm rapid lead-in + full-turn helix hypot(2π·10, 4) ≈ 62.96 mm.
    EXPECT_NEAR(r.pathLength,
                10.0 + std::hypot(2.0 * M_PI * 10.0, 4.0), 3.0);
}
