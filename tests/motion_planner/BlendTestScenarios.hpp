/**
 * @file BlendTestScenarios.hpp
 * @brief Parameterized test scenarios for both MotionPlanner and G64 blenders
 *
 * Provides structured test case definitions that can be run against either
 * blending system, with automated SVG output for visual verification.
 */

#pragma once

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <gtest/gtest.h>

#include "tether/motion_planner/CornerBlending.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/BezierCurve.hpp"
#include "tether/motion_planner/BlendCore.hpp"
#include "BlendTestVisualizer.hpp"

namespace BlendTest {

// ============================================================================
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;

using Arr = std::array<double, MotionPlanner::MAX_MOTION_AXES>;
using Vec2 = MotionPlanner::Vec<2, double>;
using Curve2D = MotionPlanner::BezierCurve<2, double>;
using Analysis2D = MotionPlanner::CornerAnalysis<2, double>;
using Analyzer2D = MotionPlanner::CornerAnalyzer<2, double>;
using Builder2D = MotionPlanner::BlendCurveBuilder<2, double>;
using BlendConfig = MotionPlanner::BlendConfig;

// ============================================================================
// Segment factory helpers
// ============================================================================

inline Arr makePos(double x, double y) {
    Arr a{};
    a[0] = x; a[1] = y;
    return a;
}

inline MotionPlanner::MotionSegment makeLine(double x0, double y0, double x1, double y1) {
    return MotionPlanner::MotionSegment::linear(makePos(x0, y0), makePos(x1, y1), 1000.0);
}

inline MotionPlanner::MotionSegment makeArcCW(double sx, double sy,
                                               double ex, double ey,
                                               double cx, double cy) {
    return MotionPlanner::MotionSegment::arcCW(makePos(sx, sy), makePos(ex, ey),
                                                makePos(cx, cy), 1000.0,
                                                MotionPlanner::ArcPlane::XY);
}

inline MotionPlanner::MotionSegment makeArcCCW(double sx, double sy,
                                                double ex, double ey,
                                                double cx, double cy) {
    return MotionPlanner::MotionSegment::arcCCW(makePos(sx, sy), makePos(ex, ey),
                                                 makePos(cx, cy), 1000.0,
                                                 MotionPlanner::ArcPlane::XY);
}

// ============================================================================
// Test scenario definitions
// ============================================================================

enum class SegmentType { Line, ArcCW, ArcCCW };

struct SegmentDef {
    SegmentType type = SegmentType::Line;
    double sx, sy, ex, ey;
    double cx = 0, cy = 0;  // Arc center (only for arcs)
};

struct BlendScenario {
    std::string name;
    SegmentDef seg1;
    SegmentDef seg2;
    double tolerance = 0.5;
    double maxBlendFraction = 0.5;
    BlendConfig::CornerLimitMode cornerMode = BlendConfig::CornerLimitMode::Centered;
    std::string expectedTransition;  // "Line-Line", "Line-Arc", etc.
};

inline MotionPlanner::MotionSegment makeSegment(const SegmentDef& d) {
    switch (d.type) {
        case SegmentType::Line:   return makeLine(d.sx, d.sy, d.ex, d.ey);
        case SegmentType::ArcCW:  return makeArcCW(d.sx, d.sy, d.ex, d.ey, d.cx, d.cy);
        case SegmentType::ArcCCW: return makeArcCCW(d.sx, d.sy, d.ex, d.ey, d.cx, d.cy);
    }
    return makeLine(d.sx, d.sy, d.ex, d.ey);
}

inline std::string transitionType(const SegmentDef& s1, const SegmentDef& s2) {
    auto typeStr = [](SegmentType t) -> std::string {
        switch (t) {
            case SegmentType::Line:   return "Line";
            case SegmentType::ArcCW:  return "ArcCW";
            case SegmentType::ArcCCW: return "ArcCCW";
        }
        return "Unknown";
    };
    return typeStr(s1.type) + "-" + typeStr(s2.type);
}

// ============================================================================
// Scenario library
// ============================================================================

inline std::vector<BlendScenario> standardScenarios() {
    std::vector<BlendScenario> scenarios;

    // --- Line-Line at various angles ---

    // 90 degree corner
    scenarios.push_back({
        "LL_90deg_std", 
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // 45 degree corner
    scenarios.push_back({
        "LL_45deg_std",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10 + 10 * std::tan(45 * DEG2RAD), 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // 30 degree corner (shallow)
    scenarios.push_back({
        "LL_30deg_std",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10 + 10 * std::cos(30 * DEG2RAD), 10 * std::sin(30 * DEG2RAD)},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // 120 degree corner (obtuse)
    double a120 = 120 * DEG2RAD;
    scenarios.push_back({
        "LL_120deg_std",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10 + 10 * std::cos(a120), 10 * std::sin(a120)},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // 150 degree corner (very obtuse)
    double a150 = 150 * DEG2RAD;
    scenarios.push_back({
        "LL_150deg_std",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10 + 10 * std::cos(a150), 10 * std::sin(a150)},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // --- Line-Line with different tolerances ---

    scenarios.push_back({
        "LL_90deg_tol01",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        0.1, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    scenarios.push_back({
        "LL_90deg_tol2",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        2.0, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // --- Line-Line with short segments (half-length constraint) ---

    scenarios.push_back({
        "LL_90deg_short",
        {SegmentType::Line, 0, 0, 2, 0},
        {SegmentType::Line, 2, 0, 2, 2},
        5.0, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    scenarios.push_back({
        "LL_90deg_frac30",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        5.0, 0.3, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // --- Line-Line with asymmetric segments ---

    scenarios.push_back({
        "LL_90deg_asym",
        {SegmentType::Line, 0, 0, 20, 0},
        {SegmentType::Line, 20, 0, 20, 5},
        1.0, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-Line"
    });

    // --- Corner limit modes ---

    scenarios.push_back({
        "LL_90deg_inside",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::InsideStrict, "Line-Line"
    });

    scenarios.push_back({
        "LL_90deg_outside",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::OutsideStrict, "Line-Line"
    });

    // --- Line-Arc transitions ---

    scenarios.push_back({
        "LA_90deg_std",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::ArcCCW, 10, 0, 20, 10, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-ArcCCW"
    });

    // --- Arc-Line transitions ---

    scenarios.push_back({
        "AL_90deg_std",
        {SegmentType::ArcCCW, 0, 10, 10, 0, 0, 0},
        {SegmentType::Line, 10, 0, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "ArcCCW-Line"
    });

    // --- Arc-Arc transitions ---

    scenarios.push_back({
        "AA_90deg_std",
        {SegmentType::ArcCCW, 0, 10, 10, 0, 0, 0},
        {SegmentType::ArcCCW, 10, 0, 20, 10, 10, 10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "ArcCCW-ArcCCW"
    });

    // --- CW arc transitions ---

    scenarios.push_back({
        "LA_CW_90deg",
        {SegmentType::Line, 0, 0, 10, 0},
        {SegmentType::ArcCW, 10, 0, 20, -10, 10, -10},
        0.5, 0.5, BlendConfig::CornerLimitMode::Centered, "Line-ArcCW"
    });

    return scenarios;
}

// ============================================================================
// Multi-corner path scenarios
// ============================================================================

struct MultiCornerScenario {
    std::string name;
    std::vector<Arr> waypoints;
    double tolerance = 0.5;
    double maxBlendFraction = 0.5;
};

inline std::vector<MultiCornerScenario> multiCornerScenarios() {
    std::vector<MultiCornerScenario> scenarios;

    // Square path (4 corners)
    {
        MultiCornerScenario s;
        s.name = "square_path";
        s.waypoints = {makePos(0, 0), makePos(10, 0), makePos(10, 10), makePos(0, 10), makePos(0, 0)};
        s.tolerance = 0.5;
        s.maxBlendFraction = 0.5;
        scenarios.push_back(std::move(s));
    }

    // Zigzag path (5 corners, 60° turns)
    {
        MultiCornerScenario s;
        s.name = "zigzag_60deg";
        s.waypoints = {makePos(0, 0), makePos(10, 0),
            makePos(10 + 10*std::cos(60*DEG2RAD), 10*std::sin(60*DEG2RAD)),
            makePos(10 + 10*std::cos(60*DEG2RAD) + 10, 0),
            makePos(10 + 10*std::cos(60*DEG2RAD) + 10 + 10*std::cos(60*DEG2RAD),
                    10*std::sin(60*DEG2RAD))};
        s.tolerance = 0.3;
        s.maxBlendFraction = 0.5;
        scenarios.push_back(std::move(s));
    }

    // Star path (sharp 36° corners)
    {
        MultiCornerScenario s;
        s.name = "star_path";
        std::vector<Arr> star;
        double cx = 15, cy = 15, rOuter = 10, rInner = 4;
        for (int i = 0; i <= 10; ++i) {
            double angle = i * 36.0 * DEG2RAD - PI / 2;
            double r = (i % 2 == 0) ? rOuter : rInner;
            star.push_back(makePos(cx + r * std::cos(angle), cy + r * std::sin(angle)));
        }
        s.waypoints = std::move(star);
        s.tolerance = 0.2;
        s.maxBlendFraction = 0.4;
        scenarios.push_back(std::move(s));
    }

    // Short segments chain (tests overlap protection)
    {
        MultiCornerScenario s;
        s.name = "short_chain";
        s.waypoints = {makePos(0, 0), makePos(3, 0), makePos(3, 3), makePos(0, 3), makePos(0, 0)};
        s.tolerance = 1.0;
        s.maxBlendFraction = 0.5;
        scenarios.push_back(std::move(s));
    }

    // Varying angles path
    {
        MultiCornerScenario s;
        s.name = "varying_angles";
        s.waypoints = {makePos(0, 0), makePos(10, 0),
            makePos(10 + 10*std::cos(30*DEG2RAD), 10*std::sin(30*DEG2RAD)),
            makePos(10 + 10*std::cos(30*DEG2RAD) + 10*std::cos(90*DEG2RAD),
                    10*std::sin(30*DEG2RAD) + 10*std::sin(90*DEG2RAD)),
            makePos(10 + 10*std::cos(30*DEG2RAD) + 10*std::cos(90*DEG2RAD) + 10*std::cos(135*DEG2RAD),
                    10*std::sin(30*DEG2RAD) + 10*std::sin(90*DEG2RAD) + 10*std::sin(135*DEG2RAD))};
        s.tolerance = 0.5;
        s.maxBlendFraction = 0.5;
        scenarios.push_back(std::move(s));
    }

    return scenarios;
}

// ============================================================================
// Run a single blend scenario and generate visualization
// ============================================================================

struct BlendTestResult {
    bool canBlend = false;
    bool c1Pass = false;
    bool c2Pass = false;
    double angleDeg = 0;
    double blendRadius = 0;
    double entryDist = 0;
    double exitDist = 0;
    double maxCurvature = 0;
    std::vector<Curve2D> blendCurves;
    Analysis2D analysis;
    BlendVisualizationData vizData;
};

inline BlendTestResult runBlendScenario(const BlendScenario& scenario,
                                        const std::string& svgOutputPath = "") {
    BlendTestResult result;

    auto seg1 = makeSegment(scenario.seg1);
    auto seg2 = makeSegment(scenario.seg2);

    BlendConfig config;
    config.tolerance = scenario.tolerance;
    config.maxBlendFraction = scenario.maxBlendFraction;
    config.cornerMode = scenario.cornerMode;

    result.analysis = Analyzer2D::analyze(seg1, seg2, config);
    result.canBlend = result.analysis.canBlend;
    result.angleDeg = result.analysis.angle * 180.0 / PI;
    result.blendRadius = result.analysis.blendRadius;
    result.entryDist = result.analysis.entryDistance;
    result.exitDist = result.analysis.exitDistance;

    if (result.canBlend) {
        result.blendCurves = Builder2D::buildG2BlendCurve(result.analysis);

        // Check C1 continuity
        if (!result.blendCurves.empty()) {
            double entryErr = std::abs(
                Builder2D::computeTangentAt(result.blendCurves.front(), 0.0)
                    .dot(result.analysis.incomingDir) - 1.0);
            double exitErr = std::abs(
                Builder2D::computeTangentAt(result.blendCurves.back(), 1.0)
                    .dot(result.analysis.outgoingDir) - 1.0);
            result.c1Pass = (entryErr < 0.1 && exitErr < 0.1);

            // Check C2 continuity
            double entryK = std::abs(Builder2D::computeCurvatureAt(
                result.blendCurves.front(), 0.0) - result.analysis.incomingCurvature);
            double exitK = std::abs(Builder2D::computeCurvatureAt(
                result.blendCurves.back(), 1.0) - result.analysis.outgoingCurvature);
            result.c2Pass = (entryK < 5.0 && exitK < 5.0);

            // Max curvature
            result.maxCurvature = Builder2D::computeMaxCurvature(
                result.blendCurves.front(), 100);
        }
    }

    // Build visualization data
    BlendVisualizationData& vd = result.vizData;
    vd.testName = scenario.name;
    vd.transitionType = transitionType(scenario.seg1, scenario.seg2);
    vd.cornerAngleDeg = result.angleDeg;
    vd.tolerance = scenario.tolerance;
    vd.blendRadius = result.blendRadius;
    vd.entryDistance = result.entryDist;
    vd.exitDistance = result.exitDist;
    vd.maxCurvature = result.maxCurvature;
    vd.canBlend = result.canBlend;
    vd.c1Continuous = result.c1Pass;
    vd.c2Continuous = result.c2Pass;

    // Path points
    namespace bc = tether::blend;
    vd.pathStart = bc::BlendVec(scenario.seg1.sx, scenario.seg1.sy, 0.0);
    vd.cornerPoint = bc::BlendVec(scenario.seg1.ex, scenario.seg1.ey, 0.0);
    vd.pathEnd = bc::BlendVec(scenario.seg2.ex, scenario.seg2.ey, 0.0);

    if (result.canBlend) {
        vd.blendEntry = tether::blend::BlendVec(result.analysis.blendEntry[0], result.analysis.blendEntry[1], 0.0);
        vd.blendExit = tether::blend::BlendVec(result.analysis.blendExit[0], result.analysis.blendExit[1], 0.0);

        // Sample blend curve
        for (const auto& curve : result.blendCurves) {
            for (int i = 0; i <= 100; ++i) {
                double t = static_cast<double>(i) / 100.0;
                auto p = curve.evaluate(t);
                vd.blendPoints.push_back(tether::blend::BlendVec(p[0], p[1], 0.0));
            }
        }

        // Control points from first curve
        if (!result.blendCurves.empty()) {
            const auto& cp = result.blendCurves[0].controlPoints();
            for (const auto& p : cp) {
                vd.controlPoints.push_back(tether::blend::BlendVec(p[0], p[1], 0.0));
            }
        }

        // Curvature profile
        for (int i = 0; i <= 50; ++i) {
            double t = static_cast<double>(i) / 50.0;
            double k = 0;
            for (const auto& curve : result.blendCurves) {
                k = Builder2D::computeCurvatureAt(curve, t);
            }
            vd.curvatureProfile.emplace_back(t, k);
        }
    }

    if (!svgOutputPath.empty()) {
        generateBlendSVG(vd, svgOutputPath);
    }

    return result;
}

// ============================================================================
// Run a multi-corner path scenario
// ============================================================================

inline MultiBlendVisualizationData runMultiCornerScenario(
    const MultiCornerScenario& scenario,
    const std::string& svgOutputPath = "") {

    MultiBlendVisualizationData vd;
    vd.testName = scenario.name;

    // Build original path
    for (const auto& wp : scenario.waypoints) {
        vd.originalPath.push_back(tether::blend::BlendVec(wp[0], wp[1], 0.0));
    }

    // Build segments and blend
    std::vector<MotionPlanner::MotionSegment> segments;
    for (size_t i = 0; i + 1 < scenario.waypoints.size(); ++i) {
        segments.push_back(makeLine(
            scenario.waypoints[i][0], scenario.waypoints[i][1],
            scenario.waypoints[i+1][0], scenario.waypoints[i+1][1]));
    }

    BlendConfig config;
    config.tolerance = scenario.tolerance;
    config.maxBlendFraction = scenario.maxBlendFraction;

    // Seed blended path with first waypoint
    if (!scenario.waypoints.empty()) {
        vd.blendedPath.push_back(tether::blend::BlendVec(
            scenario.waypoints[0][0], scenario.waypoints[0][1], 0.0));
    }

    // Process each corner
    for (size_t i = 0; i + 1 < segments.size(); ++i) {
        auto analysis = Analyzer2D::analyze(segments[i], segments[i+1], config);

        if (analysis.canBlend) {
            // Add blend entry, curve, and exit
            vd.blendedPath.push_back(tether::blend::BlendVec(analysis.blendEntry[0], analysis.blendEntry[1], 0.0));

            auto curves = Builder2D::buildG2BlendCurve(analysis);
            for (const auto& curve : curves) {
                for (int j = 0; j <= 50; ++j) {
                    double t = static_cast<double>(j) / 50.0;
                    auto p = curve.evaluate(t);
                    vd.blendedPath.push_back(tether::blend::BlendVec(p[0], p[1], 0.0));
                }
            }

            vd.blendRegions.push_back(tether::blend::BlendVec(analysis.blendEntry[0], analysis.blendEntry[1], 0.0));
            vd.blendRegions.push_back(tether::blend::BlendVec(analysis.blendExit[0], analysis.blendExit[1], 0.0));

            vd.blendedPath.push_back(tether::blend::BlendVec(analysis.blendExit[0], analysis.blendExit[1], 0.0));
        } else {
            // No blend — just add corner point
            vd.blendedPath.push_back(tether::blend::BlendVec(
                scenario.waypoints[i+1][0], scenario.waypoints[i+1][1], 0.0));
        }
    }

    // Add final waypoint
    if (!scenario.waypoints.empty()) {
        vd.blendedPath.push_back(tether::blend::BlendVec(
            scenario.waypoints.back()[0], scenario.waypoints.back()[1], 0.0));
    }

    // Labels
    for (size_t i = 0; i < scenario.waypoints.size(); ++i) {
        vd.labels.push_back("W" + std::to_string(i));
    }

    if (!svgOutputPath.empty()) {
        generateMultiBlendSVG(vd, svgOutputPath);
    }

    return vd;
}

} // namespace BlendTest
