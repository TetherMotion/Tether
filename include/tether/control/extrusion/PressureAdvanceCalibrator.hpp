/**
 * @file PressureAdvanceCalibrator.hpp
 * @brief Calibration advisor for pressure-advance settings.
 *
 * @details
 * Tether already implements three pressure-advance *models* — linear
 * (classic Klipper), power-law, and Cross-WLF — in
 * ExtrusionPressureModels.hpp.  Those models answer "given a PA gain,
 * what is the extruder position offset?"  This header answers the
 * complementary question: "given a G-code program and a machine
 * configuration, what PA gain *should* I use?"
 *
 * The calibrator works in two modes:
 *
 * 1. **Heuristic mode** — recommends a PA value from a lookup table
 *    keyed by extruder type (direct-drive vs Bowden) and filament
 *    material.  The table encodes community-tuned starting points;
 *    the calibrator refines them using measured retraction statistics
 *    from the G-code (long retractions → Bowden → higher PA).
 *
 * 2. **Tuning-curve mode** — given a series of (PA, measured-error)
 *    pairs from a calibration print (e.g. the Marlin linear-advance
 *    tower), the calibrator fits a parabola to find the PA that
 *    minimises the error.  This is the same approach used by
 *    Marlin's `M900` auto-tune and Klipper's `TUNING_TOWER` command.
 *
 * The calibrator also detects the current PA setting from G-code
 * comments and commands (M900, SET_PRESSURE_ADVANCE, M572) so it can
 * report whether the configured value matches the recommendation.
 *
 * @see ExtrusionPressureModels.hpp for the runtime PA models.
 * @see docs/extrusion/NonNewtonianPressureAdvance.md
 */

#pragma once

#include <string>
#include <vector>

namespace tether::control::extrusion {

/// @brief Extruder drive type.
enum class ExtruderType {
    DirectDrive, ///< Direct-drive extruder (short filament path)
    Bowden,      ///< Bowden extruder (long PTFE tube)
    Unknown      ///< Not yet classified
};

/// @brief Filament material categories used by the heuristic table.
enum class FilamentMaterial {
    PLA,
    ABS,
    PETG,
    TPU,
    Nylon,
    ASA,
    Unknown
};

/// @brief A single observation from a calibration print.
struct CalibrationPoint {
    double pressureAdvance; ///< PA value tested [s]
    double measuredError;   ///< Measured error metric (lower = better)
};

/// @brief Detected PA setting from G-code.
struct DetectedPaSetting {
    int lineNumber = 0;     ///< 1-based line number
    double value = 0.0;     ///< PA value [s]
    std::string command;    ///< Source command (e.g. "M900", "SET_PRESSURE_ADVANCE")
};

/// @brief Result of a pressure-advance calibration analysis.
struct PaCalibrationResult {
    // --- Detected state ---
    bool paEnabled = false;          ///< Whether PA is currently enabled
    double currentValue = 0.0;       ///< Detected PA value [s] (0 if none)
    std::vector<DetectedPaSetting> settings; ///< All PA commands found

    // --- Recommendation ---
    double recommendedValue = 0.0;   ///< Recommended PA [s]
    ExtruderType detectedExtruder = ExtruderType::Unknown;
    FilamentMaterial material = FilamentMaterial::Unknown;

    // --- Risk assessment ---
    double oozeRisk = 0.0;           ///< 0-100, high when PA too low
    double underExtrusionRisk = 0.0; ///< 0-100, high when PA too high
    double consistencyScore = 0.0;   ///< 0-100, how close current is to recommended

    // --- Tuning curve (populated only in tuning-curve mode) ---
    bool hasTuningCurve = false;
    double fittedOptimum = 0.0;      ///< Parabola-fitted optimum PA [s]
    double fittedMinimum = 0.0;      ///< Error at the optimum

    // --- Diagnostics ---
    double avgRetractionDistance = 0.0; ///< Average retraction length [mm]
    int retractionCount = 0;            ///< Number of retractions detected

    std::vector<std::string> recommendations;
};

/// @brief Parameters for the calibrator.
struct PressureAdvanceCalibratorParams {
    FilamentMaterial material = FilamentMaterial::PLA;
    ExtruderType extruderType = ExtruderType::Unknown;
    double nozzleDiameterMm = 0.4;
    double filamentDiameterMm = 1.75;
    /// Bowden detection threshold: avg retraction > this → Bowden [mm].
    double bowdenRetractionThreshold = 3.0;
    /// Direct-drive detection threshold: avg retraction > this → direct [mm].
    double directRetractionThreshold = 0.5;
};

/// @brief Pressure-advance calibration advisor.
///
/// Recommends PA values from machine configuration and G-code analysis,
/// and fits tuning curves from calibration-print data.
class PressureAdvanceCalibrator {
public:
    using Params = PressureAdvanceCalibratorParams;

    explicit PressureAdvanceCalibrator(Params params = Params{})
        : params_(params) {}

    /// @brief Analyse a G-code program and produce a PA recommendation.
    /// @param gcodeLines The G-code text, one entry per line.
    PaCalibrationResult analyse(const std::vector<std::string>& gcodeLines) const;

    /// @brief Fit a tuning curve to calibration-print data.
    ///
    /// Given a series of (PA, error) points from a calibration tower,
    /// fit a parabola y = a·x² + b·x + c and return the PA that
    /// minimises the error (vertex at x = -b/(2a)).
    ///
    /// @param points Calibration observations (must be ≥ 3).
    /// @return Fitted optimum PA [s], or 0 if fitting fails.
    static double fitTuningCurve(const std::vector<CalibrationPoint>& points);

    /// @brief Convert a filament name string to the enum.
    static FilamentMaterial parseMaterial(const std::string& name);

    /// @brief Get the heuristic PA recommendation for a material + extruder combo.
    /// @return Recommended PA [s].
    static double heuristicPa(FilamentMaterial material, ExtruderType extruder);

    // --- Accessors ---
    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;

    /// @brief Detect the current PA setting from G-code commands.
    void detectCurrentPa(const std::vector<std::string>& lines,
                         PaCalibrationResult& result) const;

    /// @brief Classify extruder type from retraction statistics.
    ExtruderType classifyExtruder(double avgRetractionMm) const;

    /// @brief Compute risk scores from current vs recommended PA.
    void computeRisk(PaCalibrationResult& result) const;
};

} // namespace tether::control::extrusion
