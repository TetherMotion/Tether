/**
 * @file ArcAnalyzer.hpp
 * @brief Analyze arc interpolation quality, detect arc-fitting candidates, compute arc length.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief An arc fitting candidate (linear segments that could be replaced by G2/G3).
struct ArcFittingCandidate {
    int startLine = 0;
    int endLine = 0;
    double centerX = 0, centerY = 0;
    double radius = 0;
    bool isCW = false; ///< true=G2, false=G3
    int segmentCount = 0;
    double estimatedSavings = 0; ///< bytes
    double fitError = 0;
};

/// @brief Arc quality info for an existing G2/G3 command.
struct ArcQualityInfo {
    int line = 0;
    double radius = 0;
    double angle = 0;  ///< degrees
    double length = 0; ///< mm
    bool isCW = false;
    bool hasIssue = false;
    std::string issue;
};

/// @brief Arc data with full geometry.
struct ArcData {
    int line = 0;
    bool isCW = false;
    double startX = 0, startY = 0;
    double endX = 0, endY = 0;
    double centerX = 0, centerY = 0;
    double radius = 0;
    double arcLength = 0;
    double sweepAngle = 0; ///< degrees
};

/// @brief Combined arc analysis result.
struct ArcAnalysisResult {
    // Arc fitting candidates
    std::vector<ArcFittingCandidate> candidates;
    int totalFittableSegments = 0;
    double totalEstimatedSavings = 0;

    // Arc quality
    std::vector<ArcQualityInfo> arcs;
    int arcCount = 0;
    double avgRadius = 0;
    double minRadius = 0;
    double totalArcLength = 0;
    int issueCount = 0;
    double qualityScore = 100;

    // Arc length data
    std::vector<ArcData> arcData;
    double avgArcLength = 0;
    double maxArcLength = 0;
    int cwCount = 0, ccwCount = 0;
    double totalLinearDistance = 0;
    double arcPercentage = 0;

    std::vector<std::string> recommendations;
};

/// @brief Analyze arc interpolation: quality, fitting candidates, arc length.
class ArcAnalyzer {
public:
    struct Params {
        double fitToleranceMm = 0.01;
        int minSegments = 5;
    };

    explicit ArcAnalyzer() : params_() {}
    explicit ArcAnalyzer(Params params) : params_(params) {}

    /// @brief Analyze arc-related properties of G-code.
    ArcAnalysisResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;

    /// Fit a circle through 3 points. Returns nullopt if collinear.
    struct CircleFit { double cx, cy, radius; };
    static std::optional<CircleFit> fitCircle(
        double x1, double y1, double x2, double y2,
        double x3, double y3);
};

} // namespace tether::analysis
