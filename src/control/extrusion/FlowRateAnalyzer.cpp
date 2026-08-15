/**
 * @file FlowRateAnalyzer.cpp
 * @brief Analyse extrusion flow-rate consistency from G-code.
 */

#include "tether/control/extrusion/FlowRateAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numeric>
#include <regex>

namespace tether::control::extrusion {

// ── Main analysis ───────────────────────────────────────────────────

FlowRateAnalysisResult FlowRateAnalyzer::analyse(
    const std::vector<std::string>& gcodeLines) const {
    FlowRateAnalysisResult result;
    result.currentFlowRatePercent = 100.0;
    result.expectedBeadWidth = params_.nozzleDiameterMm * 1.2;

    const double filamentArea = M_PI * (params_.filamentDiameterMm / 2.0) *
                                (params_.filamentDiameterMm / 2.0);

    // Regexes for parsing G-code words.
    const std::regex xRegex(R"([Xx](-?\d*\.?\d+))");
    const std::regex yRegex(R"([Yy](-?\d*\.?\d+))");
    const std::regex zRegex(R"([Zz](-?\d*\.?\d+))");
    const std::regex eRegex(R"([Ee](-?\d*\.?\d+))");
    const std::regex fRegex(R"([Ff](\d*\.?\d+))");

    double prevX = 0.0, prevY = 0.0, prevE = 0.0;
    double feedRate = 0.0;       // mm/min
    double currentZ = params_.defaultLayerHeightMm;

    std::vector<double> flowRates;
    std::vector<double> beadWidths;
    flowRates.reserve(1024);
    beadWidths.reserve(1024);

    for (const auto& line : gcodeLines) {
        // Strip comments.
        auto commentPos = line.find_first_of(";(");
        std::string code = (commentPos != std::string::npos)
            ? line.substr(0, commentPos) : line;

        // Only process G0/G1 moves.
        if (code.find("G0") == std::string::npos &&
            code.find("G1") == std::string::npos &&
            code.find("g0") == std::string::npos &&
            code.find("g1") == std::string::npos)
            continue;

        std::smatch m;

        // Parse feed rate.
        if (std::regex_search(code, m, fRegex)) {
            feedRate = std::stod(m[1].str());
        }

        // Parse coordinates.
        double x = prevX, y = prevY, e = prevE;
        if (std::regex_search(code, m, xRegex)) x = std::stod(m[1].str());
        if (std::regex_search(code, m, yRegex)) y = std::stod(m[1].str());
        if (std::regex_search(code, m, zRegex)) currentZ = std::stod(m[1].str());
        if (std::regex_search(code, m, eRegex)) {
            e = std::stod(m[1].str());
        }

        // Compute flow rate for extruding moves.
        if (e > prevE && feedRate > 0.0) {
            const double dx = x - prevX;
            const double dy = y - prevY;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 1e-9) {
                const double eDelta = e - prevE;
                const double volume = eDelta * filamentArea; // mm³
                const double timeSec = dist / (feedRate / 60.0); // s
                const double flowRate = (timeSec > 0.0)
                    ? volume / timeSec : 0.0; // mm³/s

                flowRates.push_back(flowRate);

                // Bead width = volume / (distance × layer height).
                if (currentZ > 0.0) {
                    const double width = volume / (dist * currentZ);
                    beadWidths.push_back(width);
                }

                // Store sample (up to cap).
                if (result.samples.size() < params_.maxStoredSamples) {
                    result.samples.push_back({flowRate, 0.0, feedRate, dist});
                }
            }
        }

        prevX = x;
        prevY = y;
        prevE = e;
    }

    if (flowRates.empty()) {
        result.recommendations.push_back("No extrusion data for flow-rate analysis");
        result.consistencyScore = 100.0;
        result.calibrationScore = 100.0;
        return result;
    }

    // Compute flow-rate statistics.
    computeStats(flowRates, result);

    // Compute bead-width statistics.
    computeBeadWidth(beadWidths, result);

    // Generate recommendations.
    generateRecommendations(result);

    return result;
}

// ── Statistics ──────────────────────────────────────────────────────

void FlowRateAnalyzer::computeStats(std::vector<double>& flowRates,
                                     FlowRateAnalysisResult& result) const {
    result.sampleCount = static_cast<int>(flowRates.size());

    const double sum = std::accumulate(flowRates.begin(), flowRates.end(), 0.0);
    result.avgFlowRate = sum / flowRates.size();

    auto [minIt, maxIt] = std::minmax_element(flowRates.begin(), flowRates.end());
    result.minFlowRate = *minIt;
    result.maxFlowRate = *maxIt;

    double sqSum = 0.0;
    for (double q : flowRates) {
        sqSum += (q - result.avgFlowRate) * (q - result.avgFlowRate);
    }
    result.stdDev = std::sqrt(sqSum / flowRates.size());
    result.coefficientOfVariation = (result.avgFlowRate > 0.0)
        ? result.stdDev / result.avgFlowRate : 0.0;

    // Outliers: > 2σ from mean.
    result.outlierCount = static_cast<int>(
        std::count_if(flowRates.begin(), flowRates.end(),
                      [&](double q) {
                          return std::abs(q - result.avgFlowRate) >
                                 2.0 * result.stdDev;
                      }));

    // Consistency score: 100 - CV×100, clamped to [0, 100].
    result.consistencyScore = std::max(0.0, 100.0 -
                                       result.coefficientOfVariation * 100.0);
}

// ── Bead-width analysis ─────────────────────────────────────────────

void FlowRateAnalyzer::computeBeadWidth(std::vector<double>& widths,
                                         FlowRateAnalysisResult& result) const {
    if (widths.empty()) {
        result.calibrationScore = 100.0;
        return;
    }

    const double sum = std::accumulate(widths.begin(), widths.end(), 0.0);
    result.avgBeadWidth = sum / widths.size();

    result.widthDeviationPercent = (result.expectedBeadWidth > 0.0)
        ? ((result.avgBeadWidth - result.expectedBeadWidth) /
           result.expectedBeadWidth) * 100.0
        : 0.0;

    result.overExtrusion = result.widthDeviationPercent > 5.0;
    result.underExtrusion = result.widthDeviationPercent < -5.0;

    // Calibration score: 100 - |deviation| × 2, clamped to [0, 100].
    result.calibrationScore = std::max(0.0, 100.0 -
                                       std::abs(result.widthDeviationPercent) * 2.0);

    // Flow-rate adjustment advice.
    result.currentFlowRatePercent = 100.0;
    result.recommendedFlowRatePercent = std::clamp(
        100.0 - result.widthDeviationPercent, 80.0, 120.0);
    result.flowAdjustmentPercent =
        result.recommendedFlowRatePercent - result.currentFlowRatePercent;
}

// ── Recommendations ─────────────────────────────────────────────────

void FlowRateAnalyzer::generateRecommendations(
    FlowRateAnalysisResult& result) const {
    auto& recs = result.recommendations;

    recs.push_back(std::format("Avg flow rate: {:.2f} mm³/s, CV: {:.1f}%",
                               result.avgFlowRate,
                               result.coefficientOfVariation * 100.0));

    if (result.coefficientOfVariation > 0.3) {
        recs.push_back("High flow-rate variation — check extrusion settings");
    }
    if (result.outlierCount > 5) {
        recs.push_back(std::format("{} flow-rate outliers — inconsistent extrusion",
                                   result.outlierCount));
    }
    if (result.maxFlowRate > result.avgFlowRate * 2.0) {
        recs.push_back(std::format("Max flow {:.2f} mm³/s is 2× average — possible over-extrusion",
                                   result.maxFlowRate));
    }

    // Bead-width / calibration advice.
    if (result.overExtrusion) {
        recs.push_back(std::format("Over-extrusion: bead width {:.3f} mm vs expected {:.3f} mm",
                                   result.avgBeadWidth, result.expectedBeadWidth));
        recs.push_back(std::format("Reduce flow rate by {:.0f}% (to {:.0f}%)",
                                   std::abs(result.flowAdjustmentPercent),
                                   result.recommendedFlowRatePercent));
    } else if (result.underExtrusion) {
        recs.push_back(std::format("Under-extrusion: bead width {:.3f} mm vs expected {:.3f} mm",
                                   result.avgBeadWidth, result.expectedBeadWidth));
        recs.push_back(std::format("Increase flow rate by {:.0f}% (to {:.0f}%)",
                                   std::abs(result.flowAdjustmentPercent),
                                   result.recommendedFlowRatePercent));
    } else {
        recs.push_back("Flow rate is well-calibrated — no adjustment needed");
    }

    if (result.consistencyScore > 85.0) {
        recs.push_back("Consistent flow rate — good extrusion quality");
    }
}

} // namespace tether::control::extrusion
