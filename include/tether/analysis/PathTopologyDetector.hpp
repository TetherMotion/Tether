/**
 * @file PathTopologyDetector.hpp
 * @brief Detect toolpath loops, self-intersections, overlaps, and symmetry.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Detected toolpath loop.
struct LoopInfo {
    int startLine = 0;
    int endLine = 0;
    int iterations = 0;
    double loopDistance = 0;
    enum class Type { Pattern, Spiral, Rectangular, Circular } type = Type::Pattern;
};

/// @brief Self-intersection of two path segments.
struct SelfIntersection {
    int line1 = 0;
    int line2 = 0;
    double pointX = 0, pointY = 0;
    bool isRapid = false;
    enum class Severity { Warning, Error } severity = Severity::Warning;
};

/// @brief Overlapping extrusion paths.
struct OverlapRegion {
    int line1 = 0;
    int line2 = 0;
    double area = 0;       ///< mm²
    double percentage = 0;
};

/// @brief Combined topology analysis result.
struct PathTopologyResult {
    // Loops
    std::vector<LoopInfo> loops;
    int loopCount = 0;
    double totalLoopDistance = 0;
    int maxIterations = 0;
    double loopEfficiencyScore = 100;

    // Self-intersections
    std::vector<SelfIntersection> intersections;
    int intersectionCount = 0;
    bool hasCuttingIntersections = false;
    bool hasRapidIntersections = false;

    // Overlaps
    std::vector<OverlapRegion> overlaps;
    int overlapCount = 0;
    double totalOverlapArea = 0;
    bool hasSignificantOverlaps = false;

    // Symmetry
    bool isSymmetric = false;
    enum class SymmetryType { None, MirrorX, MirrorY, Rotational, Bilateral } symmetryType = SymmetryType::None;
    double symmetryScore = 0;
    double symmetryAxisValue = 0;
    bool symmetryAxisIsX = false;
    double matchedPercentage = 0;

    std::vector<std::string> recommendations;
};

/// @brief Detect toolpath topology: loops, self-intersections, overlaps, symmetry.
class PathTopologyDetector {
public:
    struct Params {
        double loopToleranceMm = 0.1;
        double intersectionZThresholdMm = 0.5;
        double overlapExtrusionWidthMm = 0.48;
        double overlapTolerancePercent = 20;
        double symmetryToleranceMm = 0.5;
        int maxSegmentsForIntersection = 500;
        int maxSegmentsForOverlap = 200;
    };

    explicit PathTopologyDetector() : params_() {}
    explicit PathTopologyDetector(Params params) : params_(params) {}

    /// @brief Detect all topology features in G-code.
    PathTopologyResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;

    void detectLoops(const std::vector<ParsedMove>& moves, PathTopologyResult& result) const;
    void detectSelfIntersections(const std::vector<ParsedMove>& moves, PathTopologyResult& result) const;
    void detectOverlaps(const std::vector<ParsedMove>& moves, PathTopologyResult& result) const;
    void detectSymmetry(const std::vector<ParsedMove>& moves, PathTopologyResult& result) const;

    /// Segment-segment intersection test (2D).
    static bool segmentIntersection(
        double x1, double y1, double x2, double y2,
        double x3, double y3, double x4, double y4,
        double& ix, double& iy);
};

} // namespace tether::analysis
