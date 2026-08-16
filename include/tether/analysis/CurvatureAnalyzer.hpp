/**
 * @file CurvatureAnalyzer.hpp
 * @brief Analyze toolpath curvature, generate heatmap, and compute cornering speed.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Per-segment curvature data.
struct CurvatureSegment {
    int lineNumber = 0;
    double curvature = 0;   ///< 1/radius [1/mm]
    double radius = 0;      ///< radius of curvature [mm]
    double angleChange = 0; ///< direction change [degrees]
    enum class Severity { Smooth, Moderate, Sharp } severity = Severity::Smooth;
};

/// @brief Curvature heatmap point.
struct CurvatureHeatmapPoint {
    double x = 0, y = 0;
    double curvature = 0;
    enum class Category { Straight, Gentle, Moderate, Sharp, VerySharp } category = Category::Straight;
};

/// @brief Cornering speed recommendation point.
struct CorneringSpeedPoint {
    int line = 0;
    double cornerAngle = 0;   ///< degrees
    double recommendedSpeed = 0; ///< mm/min
    double currentSpeed = 0;     ///< mm/min
    double speedReduction = 0;   ///< percent
    enum class Severity { Gentle, Moderate, Sharp, VerySharp } severity = Severity::Gentle;
};

/// @brief Combined curvature analysis result.
struct CurvatureResult {
    std::vector<CurvatureSegment> segments;
    double maxCurvature = 0;
    double minRadius = 0;
    double avgCurvature = 0;
    int sharpTurnCount = 0;
    double smoothnessScore = 100; ///< 0-100

    std::vector<CurvatureHeatmapPoint> heatmap;
    std::vector<CorneringSpeedPoint> corneringPoints;
    int cornerCount = 0;
    double avgSpeedReduction = 0;
    int overspeedCount = 0;
    double corneringEfficiencyScore = 100;

    std::vector<std::string> recommendations;
};

/// @brief Analyze toolpath curvature, generate heatmap, and compute cornering speed.
class CurvatureAnalyzer {
public:
    struct Params {
        double sharpThresholdDeg = 60;  ///< Angle above which a turn is "sharp"
        double maxFeedRateMmPerMin = 3000; ///< Max feed for cornering speed calc
    };

    explicit CurvatureAnalyzer() : params_() {}
    explicit CurvatureAnalyzer(Params params) : params_(params) {}

    /// @brief Analyze curvature of G-code toolpath.
    CurvatureResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;
};

} // namespace tether::analysis
