// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file PathEvaluatorTypes.hpp
 * @brief Shared types for PathEvaluator and PathQualityGrader
 *
 * @details
 * Extracted from PathEvaluator.hpp to break the circular dependency between
 * PathEvaluator (which owns a PathQualityGrader) and PathQualityGrader
 * (which needs the Grade/QualitativeAssessment/QualitativeEvaluation/
 * EvaluatorConfig types).
 */

#include <string>
#include <vector>
#include <cstddef>

namespace tether::motion::replanner {

//=============================================================================
// Qualitative assessment types
//=============================================================================

/// Letter grade for a single aspect of path fidelity.
enum class Grade { A, B, C, D, F };

/// Convert a grade to a string ("A", "B", etc.).
inline std::string gradeToString(Grade g) {
    switch (g) {
        case Grade::A: return "A";
        case Grade::B: return "B";
        case Grade::C: return "C";
        case Grade::D: return "D";
        case Grade::F: return "F";
    }
    return "?";
}

/// Qualitative assessment for one aspect: grade, score, description, and
/// actionable recommendations.
struct QualitativeAssessment {
    Grade grade = Grade::A;
    /// Score in [0, 1], where 1 = perfect. Used for weighted aggregation.
    double score = 1.0;
    /// Human-readable description of what this grade means.
    std::string description;
    /// Actionable recommendations to improve this aspect.
    std::vector<std::string> recommendations;
};

/// Complete qualitative evaluation covering all aspects.
struct QualitativeEvaluation {
    QualitativeAssessment pathFidelity;        ///< Based on contour error
    QualitativeAssessment surfaceFinish;       ///< Based on Ra/Rq
    QualitativeAssessment timingFidelity;      ///< Based on lag error
    QualitativeAssessment smoothness;          ///< Based on jerk
    QualitativeAssessment oscillationSeverity; ///< Based on FFT oscillation index
    QualitativeAssessment cornerPreservation;  ///< Based on corner error
    QualitativeAssessment overall;             ///< Weighted combination

    /// Diagnostic messages highlighting specific issues.
    std::vector<std::string> diagnosticMessages;
};

//=============================================================================
// Configuration
//=============================================================================

/// Configuration for the PathEvaluator.
struct EvaluatorConfig {
    //--- Contour error thresholds (mm) for grading ---
    double contourErrorThresholdA = 0.005; ///< 5 µm for grade A
    double contourErrorThresholdB = 0.010; ///< 10 µm for grade B
    double contourErrorThresholdC = 0.020; ///< 20 µm for grade C
    double contourErrorThresholdD = 0.050; ///< 50 µm for grade D

    //--- Surface finish thresholds (µm) for grading ---
    double surfaceFinishRaA = 0.5;
    double surfaceFinishRaB = 1.6;
    double surfaceFinishRaC = 3.2;
    double surfaceFinishRaD = 6.3;

    //--- Lag error thresholds (mm) for grading ---
    double lagErrorThresholdA = 0.01;
    double lagErrorThresholdB = 0.05;
    double lagErrorThresholdC = 0.1;
    double lagErrorThresholdD = 0.5;

    //--- Jerk thresholds (mm/s³) for grading ---
    double jerkThresholdA = 100.0;
    double jerkThresholdB = 500.0;
    double jerkThresholdC = 1000.0;
    double jerkThresholdD = 5000.0;

    //--- Oscillation index thresholds for grading ---
    double oscillationThresholdA = 0.1;
    double oscillationThresholdB = 0.2;
    double oscillationThresholdC = 0.3;
    double oscillationThresholdD = 0.5;

    //--- Settling ---
    double settlingTolerance = 0.01; ///< 10 µm

    //--- Algorithm selection ---
    /// Use certified contour error (Bernstein root isolation) vs tangent
    /// projection. Certified is more accurate but slower.
    bool useCertifiedContourError = true;

    //--- Shape distance computation ---
    /// Downsample factor for O(n²) shape distances (Hausdorff/Frechet/DTW).
    /// 1 = use all samples; 10 = use every 10th sample.
    std::size_t shapeDistanceDownsample = 1;

    //--- Active axes ---
    /// Which axes to include in the evaluation (X=0, Y=1, Z=2, ...).
    /// Empty = auto-detect from the desired trajectory.
    std::vector<int> activeAxes;
};

} // namespace tether::motion::replanner
