/**
 * @file PathContinuityChecker.hpp
 * @brief Check toolpath continuity: gaps, jumps, retracts, plunges, per-layer.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief A single continuity issue.
struct ContinuityIssue {
    int line = 0;
    enum class Type { Gap, Jump, Retract, Plunge, DirectionChange } type = Type::Gap;
    double gapDistance = 0;
    std::string description;
    enum class Severity { Low, Medium, High } severity = Severity::Low;
};

/// @brief Per-layer continuity data.
struct LayerContinuity {
    int layer = 0;
    double zHeight = 0;
    bool isContinuous = false;
    int gapCount = 0;
    double maxGap = 0;
    double totalGapDistance = 0;
};

/// @brief Combined continuity analysis result.
struct PathContinuityResult {
    // Overall continuity
    std::vector<ContinuityIssue> issues;
    int issueCount = 0;
    int highSeverityCount = 0;
    int mediumSeverityCount = 0;
    int lowSeverityCount = 0;
    double totalGapDistance = 0;
    double continuityScore = 100;
    bool isContinuous = true;

    // Per-layer continuity
    std::vector<LayerContinuity> layers;
    int layerCount = 0;
    int continuousLayerCount = 0;
    int totalGaps = 0;
    double maxGap = 0;

    std::vector<std::string> recommendations;
};

/// @brief Check toolpath continuity: gaps, jumps, retracts, plunges, per-layer.
class PathContinuityChecker {
public:
    struct Params {
        double gapThresholdMm = 1.0;       ///< For overall continuity
        double layerGapThresholdMm = 5.0;  ///< For per-layer continuity
    };

    explicit PathContinuityChecker() : params_() {}
    explicit PathContinuityChecker(Params params) : params_(params) {}

    /// @brief Check continuity of G-code toolpath.
    PathContinuityResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;
};

} // namespace tether::analysis
