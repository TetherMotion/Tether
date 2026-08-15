/**
 * @file PressureAdvanceCalibrator.cpp
 * @brief Calibration advisor for pressure-advance settings.
 */

#include "tether/control/extrusion/PressureAdvanceCalibrator.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <regex>

namespace tether::control::extrusion {

// ── Heuristic PA table ──────────────────────────────────────────────

double PressureAdvanceCalibrator::heuristicPa(
    FilamentMaterial material, ExtruderType extruder) {
    // Community-tuned starting points [s].
    // Rows: material, Columns: direct-drive / Bowden
    struct PaRange { double direct; double bowden; };
    static constexpr PaRange table[] = {
        /* PLA   */ {0.04, 0.60},
        /* ABS   */ {0.05, 0.70},
        /* PETG  */ {0.06, 0.80},
        /* TPU   */ {0.02, 0.30},
        /* Nylon */ {0.05, 0.70},
        /* ASA   */ {0.05, 0.70},
        /* Unknown */ {0.04, 0.55},
    };
    const auto idx = static_cast<size_t>(material);
    const auto& r = table[idx < std::size(table) ? idx
                                                  : static_cast<size_t>(FilamentMaterial::Unknown)];
    return (extruder == ExtruderType::Bowden) ? r.bowden : r.direct;
}

// ── Material parsing ────────────────────────────────────────────────

FilamentMaterial PressureAdvanceCalibrator::parseMaterial(
    const std::string& name) {
    // Case-insensitive substring match.
    auto upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    if (upper.find("PLA") != std::string::npos)  return FilamentMaterial::PLA;
    if (upper.find("ABS") != std::string::npos)  return FilamentMaterial::ABS;
    if (upper.find("PETG") != std::string::npos) return FilamentMaterial::PETG;
    if (upper.find("TPU") != std::string::npos ||
        upper.find("FLEX") != std::string::npos)  return FilamentMaterial::TPU;
    if (upper.find("NYLON") != std::string::npos) return FilamentMaterial::Nylon;
    if (upper.find("ASA") != std::string::npos)   return FilamentMaterial::ASA;
    return FilamentMaterial::Unknown;
}

// ── Extruder classification ─────────────────────────────────────────

ExtruderType PressureAdvanceCalibrator::classifyExtruder(
    double avgRetractionMm) const {
    if (avgRetractionMm > params_.bowdenRetractionThreshold)
        return ExtruderType::Bowden;
    if (avgRetractionMm > params_.directRetractionThreshold)
        return ExtruderType::DirectDrive;
    return params_.extruderType; // fall back to user-specified
}

// ── Current-PA detection ────────────────────────────────────────────

void PressureAdvanceCalibrator::detectCurrentPa(
    const std::vector<std::string>& lines,
    PaCalibrationResult& result) const {
    // Marlin: M900 K0.045
    const std::regex m900(R"(M900\s+K(\d*\.?\d+))", std::regex::icase);
    // Klipper: SET_PRESSURE_ADVANCE ADVANCE=0.045
    const std::regex klipper(R"(SET_PRESSURE_ADVANCE\s+ADVANCE=(\d*\.?\d+))",
                              std::regex::icase);
    // RepRap: M572 D0 S0.045
    const std::regex m572(R"(M572\s+D\d+\s+S(\d*\.?\d+))", std::regex::icase);

    for (int i = 0; i < static_cast<int>(lines.size()); ++i) {
        const auto& line = lines[static_cast<size_t>(i)];
        std::smatch m;
        if (std::regex_search(line, m, m900)) {
            result.currentValue = std::stod(m[1].str());
            result.paEnabled = result.currentValue > 0.0;
            result.settings.push_back({i + 1, result.currentValue, "M900"});
        } else if (std::regex_search(line, m, klipper)) {
            result.currentValue = std::stod(m[1].str());
            result.paEnabled = result.currentValue > 0.0;
            result.settings.push_back({i + 1, result.currentValue,
                                       "SET_PRESSURE_ADVANCE"});
        } else if (std::regex_search(line, m, m572)) {
            result.currentValue = std::stod(m[1].str());
            result.paEnabled = result.currentValue > 0.0;
            result.settings.push_back({i + 1, result.currentValue, "M572"});
        }
    }
}

// ── Risk computation ────────────────────────────────────────────────

void PressureAdvanceCalibrator::computeRisk(
    PaCalibrationResult& result) const {
    const double delta = result.currentValue - result.recommendedValue;
    // PA too low → ooze risk; PA too high → under-extrusion risk.
    result.oozeRisk = delta < 0.0
        ? std::min(100.0, std::abs(delta) * 1000.0) : 0.0;
    result.underExtrusionRisk = delta > 0.0
        ? std::min(100.0, delta * 1000.0) : 0.0;
    result.consistencyScore = std::max(0.0, 100.0 - std::abs(delta) * 500.0);
}

// ── Main analysis ───────────────────────────────────────────────────

PaCalibrationResult PressureAdvanceCalibrator::analyse(
    const std::vector<std::string>& gcodeLines) const {
    PaCalibrationResult result;
    result.material = params_.material;
    result.detectedExtruder = params_.extruderType;

    // 1. Detect current PA setting from G-code commands.
    detectCurrentPa(gcodeLines, result);

    // 2. Measure retraction statistics to classify extruder type.
    double totalRetraction = 0.0;
    int retractionCount = 0;
    double prevE = 0.0;
    bool hasE = false;

    // Regex to extract E value from G0/G1 lines.
    const std::regex eRegex(R"(E(-?\d*\.?\d+))", std::regex::icase);

    for (const auto& line : gcodeLines) {
        // Strip comments.
        auto commentPos = line.find_first_of(";(");
        std::string code = (commentPos != std::string::npos)
            ? line.substr(0, commentPos) : line;
        // Only look at G0/G1 moves.
        if (code.find("G0") == std::string::npos &&
            code.find("G1") == std::string::npos &&
            code.find("g0") == std::string::npos &&
            code.find("g1") == std::string::npos)
            continue;

        std::smatch m;
        if (std::regex_search(code, m, eRegex)) {
            double e = std::stod(m[1].str());
            if (hasE && e < prevE) {
                // Retraction detected (E decreasing).
                retractionCount++;
                totalRetraction += (prevE - e);
            }
            prevE = e;
            hasE = true;
        }
    }

    result.retractionCount = retractionCount;
    result.avgRetractionDistance = (retractionCount > 0)
        ? totalRetraction / retractionCount : 0.0;

    // 3. Classify extruder type if unknown.
    if (params_.extruderType == ExtruderType::Unknown && retractionCount > 0) {
        result.detectedExtruder = classifyExtruder(result.avgRetractionDistance);
    }

    // 4. Get heuristic recommendation.
    result.recommendedValue = heuristicPa(params_.material,
                                           result.detectedExtruder);

    // 5. Compute risk scores.
    computeRisk(result);

    // 6. Generate recommendations.
    auto& recs = result.recommendations;
    const char* extruderName =
        (result.detectedExtruder == ExtruderType::Bowden)   ? "Bowden"
      : (result.detectedExtruder == ExtruderType::DirectDrive) ? "direct-drive"
                                                              : "unknown";

    recs.push_back(std::format("{} extruder, current PA {:.4f}s, recommended {:.4f}s",
                               extruderName, result.currentValue,
                               result.recommendedValue));

    if (!result.paEnabled) {
        recs.push_back("Pressure advance not configured — consider enabling for better print quality");
        recs.push_back(std::format("Typical values: {:.2f}-{:.2f} for Bowden, {:.2f}-{:.2f} for direct drive",
                                   0.02, 0.08, 0.4, 0.8));
    }
    if (result.oozeRisk > 30.0) {
        recs.push_back(std::format("High ooze risk ({:.0f}%) — increase PA", result.oozeRisk));
    }
    if (result.underExtrusionRisk > 30.0) {
        recs.push_back(std::format("High under-extrusion risk ({:.0f}%) — decrease PA",
                                   result.underExtrusionRisk));
    }
    if (retractionCount == 0) {
        recs.push_back("No retractions detected — PA may not be needed");
    }
    if (result.consistencyScore > 80.0) {
        recs.push_back("PA is well-tuned — good pressure control");
    }
    if (result.avgRetractionDistance > 0.0) {
        recs.push_back(std::format("Average retraction: {:.2f} mm ({} retractions)",
                                   result.avgRetractionDistance, retractionCount));
    }

    return result;
}

// ── Tuning-curve fit ────────────────────────────────────────────────

double PressureAdvanceCalibrator::fitTuningCurve(
    const std::vector<CalibrationPoint>& points) {
    if (points.size() < 3) return 0.0;

    // Least-squares parabola fit: y = a·x² + b·x + c
    // Normal equations: [Σx⁴ Σx³ Σx²] [a]   [Σx²y]
    //                   [Σx³ Σx² Σx ] [b] = [Σxy ]
    //                   [Σx² Σx  n  ] [c]   [Σy  ]
    double Sx = 0, Sx2 = 0, Sx3 = 0, Sx4 = 0;
    double Sy = 0, Sxy = 0, Sx2y = 0;
    const double n = static_cast<double>(points.size());

    for (const auto& p : points) {
        const double x = p.pressureAdvance;
        const double y = p.measuredError;
        const double x2 = x * x;
        const double x3 = x2 * x;
        const double x4 = x3 * x;
        Sx += x; Sx2 += x2; Sx3 += x3; Sx4 += x4;
        Sy += y; Sxy += x * y; Sx2y += x2 * y;
    }

    // Solve 3×3 system via Cramer's rule.
    const double det = Sx4 * (Sx2 * n - Sx * Sx)
                     - Sx3 * (Sx3 * n - Sx * Sx2)
                     + Sx2 * (Sx3 * Sx - Sx2 * Sx2);
    if (std::abs(det) < 1e-18) return 0.0;

    const double detA = Sx2y * (Sx2 * n - Sx * Sx)
                     - Sx3 * (Sxy * n - Sx * Sy)
                     + Sx2 * (Sxy * Sx - Sx2 * Sy);
    const double a = detA / det;

    // If curvature is negligible (near-linear data), there's no meaningful
    // optimum.  Use a relative threshold based on the data scale.
    const double yRange = [&] {
        double yMin = points[0].measuredError, yMax = yMin;
        for (const auto& p : points) {
            yMin = std::min(yMin, p.measuredError);
            yMax = std::max(yMax, p.measuredError);
        }
        return yMax - yMin;
    }();
    if (std::abs(a) < 1e-9 * std::max(1.0, yRange)) return 0.0;

    // Vertex of parabola: x = -b / (2a)
    const double detB = Sx4 * (Sxy * n - Sx * Sy)
                     - Sx2y * (Sx3 * n - Sx * Sx2)
                     + Sx2 * (Sx3 * Sy - Sx2 * Sxy);
    const double b = detB / det;

    return -b / (2.0 * a);
}

} // namespace tether::control::extrusion
