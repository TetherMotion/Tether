/**
 * @file ToolpathEfficiencyAnalyzer.hpp
 * @brief Analyze toolpath length, efficiency, air cutting, and rapid travel.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Combined toolpath efficiency result.
struct ToolpathEfficiencyResult {
    // Toolpath length breakdown
    double cuttingDistance = 0;  ///< mm
    double travelDistance = 0;   ///< mm (rapid + non-cutting feed)
    double arcDistance = 0;      ///< mm
    double totalDistance = 0;    ///< mm
    double cuttingPercentage = 0;
    double travelPercentage = 0;
    double cuttingTime = 0;      ///< seconds
    double travelTime = 0;       ///< seconds
    double efficiencyRatio = 0;  ///< cutting / travel

    // Toolpath efficiency metrics
    double totalTime = 0;        ///< seconds
    double cuttingRatio = 0;     ///< cutting / total distance
    double engagementRatio = 0;  ///< cutting time / total time
    double efficiencyScore = 0;  ///< 0-100
    double wastedTravelPercentage = 0;
    double optimizationPotential = 0;

    // Air cutting
    double airCuttingTime = 0;       ///< seconds
    double airCuttingDistance = 0;   ///< mm
    double airCuttingPercentage = 0;
    int airCuttingCount = 0;
    double longestAirCut = 0;        ///< mm
    double airCuttingEfficiencyScore = 100;

    // Rapid travel
    double rapidDistance = 0;        ///< mm
    int rapidCount = 0;
    double avgRapidLength = 0;       ///< mm
    double longestRapid = 0;         ///< mm
    double rapidPercentage = 0;
    double rapidEfficiencyScore = 100;

    std::vector<std::string> recommendations;
};

/// @brief Analyze toolpath efficiency: length breakdown, air cutting, rapid travel.
class ToolpathEfficiencyAnalyzer {
public:
    struct Params {
        double cuttingZThreshold = 0; ///< Z values below this are "cutting" (CNC)
    };

    explicit ToolpathEfficiencyAnalyzer() : params_() {}
    explicit ToolpathEfficiencyAnalyzer(Params params) : params_(params) {}

    /// @brief Analyze toolpath efficiency from G-code.
    ToolpathEfficiencyResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;
};

} // namespace tether::analysis
