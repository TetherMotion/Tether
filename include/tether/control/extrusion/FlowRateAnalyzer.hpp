/**
 * @file FlowRateAnalyzer.hpp
 * @brief Analyse extrusion flow-rate consistency from G-code.
 *
 * @details
 * Tether's extrusion models (PowerLawPressureAdvance,
 * CrossWlfPressureAdvance, FlowAdaptiveHeaterController) all assume a
 * consistent volumetric flow rate Q.  In practice, slicer-generated
 * G-code can exhibit significant flow-rate variation due to:
 *
 *   - Feed-rate changes at corners and layers
 *   - Pressure advance not yet stabilised
 *   - Slicer bugs or inconsistent line-width settings
 *   - Extruder hardware limitations (slip, grinding)
 *
 * This analyser scans G-code moves, computes the instantaneous
 * volumetric flow rate Q = (ΔE · A_filament) / Δt for each extruding
 * segment, and reports statistical measures of consistency:
 *
 *   - Mean, min, max, standard deviation
 *   - Coefficient of variation (CV = σ / μ)
 *   - Outlier count (> 2σ from mean)
 *   - Consistency score (0-100, higher is better)
 *
 * It also computes the **expected bead width** for each segment from
 * the volume / (distance × layer-height) relation and compares it to
 * the nozzle diameter, detecting over- and under-extrusion.
 *
 * @see ExtrusionPressureModels.hpp for the models that consume Q.
 * @see docs/extrusion/FlowAdaptiveTemperatureControl.md
 */

#pragma once

#include <string>
#include <vector>

namespace tether::control::extrusion {

/// @brief Per-segment flow-rate sample.
struct FlowRateSample {
    double flowRateMm3PerS = 0.0;  ///< Volumetric flow Q [mm³/s]
    double beadWidthMm = 0.0;      ///< Estimated bead width [mm]
    double feedRateMmPerMin = 0.0; ///< G-code feed rate [mm/min]
    double distanceMm = 0.0;       ///< XY travel distance [mm]
};

/// @brief Result of a flow-rate consistency analysis.
struct FlowRateAnalysisResult {
    // --- Flow-rate statistics ---
    double avgFlowRate = 0.0;       ///< Mean Q [mm³/s]
    double minFlowRate = 0.0;       ///< Min Q [mm³/s]
    double maxFlowRate = 0.0;       ///< Max Q [mm³/s]
    double stdDev = 0.0;            ///< Standard deviation [mm³/s]
    double coefficientOfVariation = 0.0; ///< CV = σ/μ (dimensionless)

    // --- Bead-width statistics ---
    double avgBeadWidth = 0.0;      ///< Mean estimated bead width [mm]
    double expectedBeadWidth = 0.0; ///< Expected width = nozzle × 1.2 [mm]
    double widthDeviationPercent = 0.0; ///< (avg - expected) / expected × 100
    bool overExtrusion = false;     ///< Width deviation > +5%
    bool underExtrusion = false;    ///< Width deviation < -5%

    // --- Quality metrics ---
    int sampleCount = 0;            ///< Number of extruding segments
    int outlierCount = 0;           ///< Samples > 2σ from mean
    double consistencyScore = 0.0;  ///< 0-100, higher is better
    double calibrationScore = 0.0;  ///< 0-100, based on width deviation

    // --- Flow-rate adjustment advice ---
    double currentFlowRatePercent = 100.0;  ///< Current flow rate [%]
    double recommendedFlowRatePercent = 100.0; ///< Recommended flow [%]
    double flowAdjustmentPercent = 0.0;     ///< recommended - current [%]

    // --- Per-segment data (optional, may be empty for large files) ---
    std::vector<FlowRateSample> samples;

    std::vector<std::string> recommendations;
};

/// @brief Parameters for the analyser.
struct FlowRateAnalyzerParams {
    double nozzleDiameterMm = 0.4;   ///< Nozzle diameter [mm]
    double filamentDiameterMm = 1.75; ///< Filament diameter [mm]
    double defaultLayerHeightMm = 0.2; ///< Fallback layer height [mm]
    /// Maximum samples to store (0 = unlimited).  Large files may
    /// produce hundreds of thousands of samples; set a cap to
    /// limit memory.
    size_t maxStoredSamples = 10000;
};

/// @brief Flow-rate consistency analyser.
///
/// Scans G-code for extruding moves and computes volumetric flow-rate
/// statistics, bead-width estimates, and calibration advice.
class FlowRateAnalyzer {
public:
    using Params = FlowRateAnalyzerParams;

    explicit FlowRateAnalyzer(Params params = Params{})
        : params_(params) {}

    /// @brief Analyse a G-code program for flow-rate consistency.
    /// @param gcodeLines The G-code text, one entry per line.
    FlowRateAnalysisResult analyse(const std::vector<std::string>& gcodeLines) const;

    // --- Accessors ---
    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;

    /// @brief Compute statistics from a list of flow rates.
    void computeStats(std::vector<double>& flowRates,
                      FlowRateAnalysisResult& result) const;

    /// @brief Compute bead-width statistics and calibration advice.
    void computeBeadWidth(std::vector<double>& widths,
                          FlowRateAnalysisResult& result) const;

    /// @brief Generate human-readable recommendations.
    void generateRecommendations(FlowRateAnalysisResult& result) const;
};

} // namespace tether::control::extrusion
