// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file PathQualityGrader.hpp
 * @brief Qualitative grading logic extracted from PathEvaluator
 *
 * @details
 * Encapsulates the qualitative assessment sub-responsibility of PathEvaluator:
 *  - Mapping metric values to letter grades (A–F) via thresholds
 *  - Converting grades to numeric scores [0, 1]
 *  - Building QualitativeAssessment objects with descriptions and recommendations
 *  - Aggregating per-aspect assessments into an overall grade
 *  - Generating diagnostic messages from quantitative results
 */

#include "tether/motion_replanner/PathEvaluatorTypes.hpp"

#include <string>
#include <vector>

namespace tether::motion::replanner {

// Forward declarations (defined in PathEvaluator.hpp / PathRelativeFFT.hpp)
struct QuantitativeEvaluation;
struct SpectralEvaluation;

class PathQualityGrader {
public:
    explicit PathQualityGrader(const EvaluatorConfig& config)
        : config_(config) {}

    /// @brief Generate qualitative grades from quantitative results.
    /// @param quant The quantitative evaluation.
    /// @param spectral Pointer to spectral evaluation (may be nullptr).
    QualitativeEvaluation evaluateQualitative(
        const QuantitativeEvaluation& quant,
        const SpectralEvaluation* spectral) const;

    //--- Grading helpers (public for testing and external use) ---

    /// Map a value to a grade given A/B/C/D thresholds (lower = better).
    Grade gradeFromThresholds(double value,
                              double threshA, double threshB,
                              double threshC, double threshD) const;

    /// Map a grade to a score in [0, 1].
    double gradeToScore(Grade g) const;

    /// Build a QualitativeAssessment from a grade and a description template.
    QualitativeAssessment makeAssessment(
        Grade g, const std::string& aspect,
        double value, const std::string& unit) const;

private:
    EvaluatorConfig config_;
};

} // namespace tether::motion::replanner
