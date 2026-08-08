/**
 * @file PathEvaluator.hpp
 * @brief Extensive quantitative and qualitative evaluators for desired vs
 *        actual path comparison.
 *
 * @details
 * Provides a comprehensive evaluation framework that compares a desired
 * trajectory against the actual measured trajectory and produces:
 *
 * - **Quantitative metrics**: integral errors (IAE/ISE/ITAE/ITSE), norm
 *   errors (L1/L2/L∞), shape distances (Hausdorff/Frechet/DTW), kinematic
 *   tracking, surface-finish estimates (Ra/Rq/Rz), and following-error
 *   metrics.
 *
 * - **Qualitative assessments**: letter grades (A–F) with scores (0–1),
 *   human-readable descriptions, and actionable recommendations for each
 *   aspect of path fidelity.
 *
 * ## Error decomposition
 *
 * All metrics are computed on the Frenet-frame decomposition of the
 * position error relative to the desired path:
 *
 * - **Contour error** (e_c): perpendicular distance to the path — the
 *   surface-finish-critical component.
 * - **Lag error** (e_l): signed arc-length offset along the path — the
 *   timing/following-error component.
 * - **Binormal error** (e_b): out-of-plane component (3D paths only).
 * - **Combined error** (|e|): full 3D Euclidean distance to the desired
 *   position.
 *
 * When `useCertifiedContourError` is true (default), the contour error is
 * computed via `computeCertifiedContourError` (Bernstein root isolation,
 * M8/M9). When false, the cheaper tangent-projection method is used.
 *
 * @see CertifiedContourError.hpp for the certified contour error algorithm.
 * @see PathRelativeFFT.hpp for spectral/oscillation evaluators.
 * @see SvgExporter.hpp for SVG visualization of these results.
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_replanner/MotionReplanner.hpp" // ErrorStatistics
#include "tether/motion_replanner/PathEvaluatorTypes.hpp"
#include "tether/motion_replanner/PathQualityGrader.hpp"

#include <vector>
#include <string>
#include <cstddef>
#include <array>

namespace tether::motion::replanner {

//=============================================================================
// Forward declarations
//=============================================================================

struct SpectralEvaluation; // defined in PathRelativeFFT.hpp

//=============================================================================
// Quantitative metric structs
//=============================================================================

/// Integral-based error metrics.
///
/// Computed via trapezoidal integration over arc length (spatial) or time
/// (temporal). These are the classical control-theory performance indices,
/// adapted to path-following analysis.
///
/// @note Units: integrals over arc length have units of mm² (IAE) or mm³
///       (ISE); integrals over time have units of mm·s (ITAE) or mm²·s
///       (ITSE).
struct IntegralErrorMetrics {
    double iae_s = 0.0;   ///< ∫|e_contour(s)| ds — spatial IAE (mm²)
    double ise_s = 0.0;   ///< ∫e_contour(s)² ds — spatial ISE (mm³)
    double itae_t = 0.0;  ///< ∫t·|e_combined(t)| dt — time-weighted IAE (mm·s)
    double itse_t = 0.0;  ///< ∫t·e_combined(t)² dt — time-weighted ISE (mm²·s)
    double iae_lag = 0.0; ///< ∫|e_lag(s)| ds — lag IAE (mm²)
    double ise_lag = 0.0; ///< ∫e_lag(s)² ds — lag ISE (mm³)
};

/// Norm-based error metrics for each error component.
///
/// L1 = ∫|e|ds, L2 = sqrt(∫e²ds), L∞ = max|e|.
/// These are the standard Lebesgue norms of the error function over the
/// path arc length.
struct NormMetrics {
    double l1_contour = 0.0;    ///< ∫|e_contour| ds (mm²)
    double l2_contour = 0.0;    ///< sqrt(∫e_contour² ds) (mm^3/2)
    double linf_contour = 0.0;  ///< max|e_contour| (mm)
    double l1_lag = 0.0;
    double l2_lag = 0.0;
    double linf_lag = 0.0;
    double l1_combined = 0.0;
    double l2_combined = 0.0;
    double linf_combined = 0.0;
};

/// Shape-distance metrics measuring geometric fidelity between the desired
/// and actual paths.
///
/// These metrics treat the trajectories as geometric curves and measure
/// how different their shapes are, independent of timing.
struct ShapeDistanceMetrics {
    /// Hausdorff distance: max over actual points of min distance to
    /// desired path. Measures the worst-case geometric deviation (mm).
    double hausdorff = 0.0;

    /// Discrete Frechet distance: the minimum over all monotone
    /// reparameterizations of the max pointwise distance (mm).
    double frechet = 0.0;

    /// Dynamic time warping distance: the minimum total cost of aligning
    /// the two sequences, normalized by path length (mm).
    double dtw = 0.0;

    /// Ratio of actual path length to desired path length.
    /// 1.0 = perfect length match; >1 = actual is longer (overshoot);
    /// <1 = actual is shorter (cutting corners).
    double pathLengthRatio = 0.0;

    /// Maximum absolute curvature difference between actual and desired
    /// at corresponding arc-length positions (1/mm).
    double curvatureErrorMax = 0.0;

    /// RMS curvature difference (1/mm).
    double curvatureErrorRms = 0.0;
};

/// Kinematic tracking metrics: how well the actual velocity, acceleration,
/// and jerk profiles match the desired.
struct KinematicTrackingMetrics {
    /// RMS of |v_actual - v_desired| (mm/s).
    double velocityTrackingRms = 0.0;
    /// Max of |v_actual - v_desired| (mm/s).
    double velocityTrackingMax = 0.0;
    /// RMS of |a_actual - a_desired| (mm/s²).
    double accelTrackingRms = 0.0;
    /// Max of |a_actual - a_desired| (mm/s²).
    double accelTrackingMax = 0.0;
    /// Maximum actual jerk magnitude (mm/s³).
    double jerkActualMax = 0.0;
    /// RMS actual jerk magnitude (mm/s³).
    double jerkActualRms = 0.0;
    /// Smoothness index: ∫jerk² dt — the jerk-cost functional (mm²/s⁵).
    /// Lower = smoother motion.
    double smoothnessIndex = 0.0;
};

/// Surface finish estimates derived from the contour error profile.
///
/// These approximate the surface roughness parameters from ISO 4287,
/// treating the contour error as the surface profile. Values are in
/// micrometers (µm).
struct SurfaceFinishMetrics {
    /// Arithmetic mean roughness Ra = (1/L)∫|e_contour|ds (µm).
    double ra = 0.0;
    /// Root-mean-square roughness Rq = sqrt((1/L)∫e_contour²ds) (µm).
    double rq = 0.0;
    /// Maximum peak-to-valley Rz = max(e) - min(e) of detrended contour (µm).
    double rz = 0.0;
    /// Number of zero-crossings of the detrended contour error.
    std::size_t peakCount = 0;
};

/// Following/timing error metrics.
struct FollowingErrorMetrics {
    /// Maximum |lag error| (mm) — the worst following error.
    double maxFollowingError = 0.0;
    /// Mean |lag error| (mm).
    double meanFollowingError = 0.0;
    /// Arc length to settle within tolerance after the worst corner (mm).
    /// 0 if no corner or never settles.
    double settlingDistance = 0.0;
    /// Peak cross-correlation value between desired and actual velocity
    /// profiles (0–1).
    double crossCorrelationPeak = 0.0;
    /// Lag at peak cross-correlation (seconds).
    double crossCorrelationLag = 0.0;
};

/// Aggregate of all quantitative metrics.
struct QuantitativeEvaluation {
    IntegralErrorMetrics integrals;
    NormMetrics norms;
    ShapeDistanceMetrics shape;
    KinematicTrackingMetrics kinematic;
    SurfaceFinishMetrics surface;
    FollowingErrorMetrics following;

    /// Statistical summaries (reuses the existing ErrorStatistics struct).
    MotionReplanner::ErrorStatistics contourStats;
    MotionReplanner::ErrorStatistics lagStats;
    MotionReplanner::ErrorStatistics combinedStats;

    /// Number of samples evaluated.
    std::size_t sampleCount = 0;
    /// Total desired path length (mm).
    double pathLength = 0.0;
    /// Total duration (seconds).
    double duration = 0.0;
};

//=============================================================================
// PathEvaluator class
//=============================================================================

/// Computes quantitative and qualitative evaluations of actual vs desired
/// path.
///
/// Usage:
/// ```cpp
/// PathEvaluator evaluator;
/// auto quant = evaluator.evaluateQuantitative(desired, actual);
/// // spectral eval from PathRelativeFFT...
/// auto qual = evaluator.evaluateQualitative(quant, spectral);
/// ```
class PathEvaluator {
public:
    explicit PathEvaluator(EvaluatorConfig config = {});

    /// Evaluate quantitative metrics from raw desired/actual trajectory
    /// samples.
    ///
    /// Internally:
    /// 1. Builds a PiecewiseNurbsPath from the desired samples.
    /// 2. Computes per-sample Frenet-decomposed errors (contour/lag/combined).
    /// 3. Aggregates all metric groups.
    ///
    /// @param desired The desired trajectory samples (ordered by time).
    /// @param actual The actual measured trajectory samples (ordered by time).
    /// @return The complete quantitative evaluation.
    QuantitativeEvaluation evaluateQuantitative(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual) const;

    /// Generate qualitative grades from quantitative results and (optional)
    /// spectral evaluation.
    ///
    /// @param quant The quantitative evaluation.
    /// @param spectral Pointer to spectral evaluation (may be nullptr if
    ///        FFT analysis was not performed; oscillation grade will be
    ///        "not assessed").
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
    PathQualityGrader grader_;

    //--- Internal helpers ---

    /// Determine active axes from the desired trajectory.
    std::vector<int> detectActiveAxes(
        const std::vector<GCodeExport::TrajectorySample>& desired) const;

    /// Compute per-sample Frenet-decomposed errors.
    struct PerSampleErrors {
        std::vector<double> arcLengths;  ///< Desired arc length per sample
        std::vector<double> times;       ///< Time per sample
        std::vector<double> contour;     ///< Contour error per sample
        std::vector<double> lag;         ///< Lag error per sample
        std::vector<double> combined;    ///< Combined 3D error per sample
        std::vector<double> actualPos3d; ///< Actual 3D positions (interleaved x,y,z)
        std::vector<double> desiredPos3d;///< Desired 3D positions
        std::vector<double> actualVel;   ///< Actual velocity magnitudes
        std::vector<double> desiredVel;  ///< Desired velocity magnitudes
        std::vector<double> actualAccel; ///< Actual accel magnitudes
        std::vector<double> desiredAccel;///< Desired accel magnitudes
        std::vector<double> actualJerk;  ///< Actual jerk magnitudes
        std::vector<double> desiredCurv; ///< Desired curvature
        std::vector<double> actualCurv;  ///< Actual curvature (finite diff)
    };

    PerSampleErrors computeErrors(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual,
        const PiecewiseNurbsPath& path) const;

    //--- Metric computation helpers ---

    IntegralErrorMetrics computeIntegrals(const PerSampleErrors& errs) const;
    NormMetrics computeNorms(const PerSampleErrors& errs) const;
    ShapeDistanceMetrics computeShapeDistances(
        const PerSampleErrors& errs,
        const PiecewiseNurbsPath& path) const;
    KinematicTrackingMetrics computeKinematic(
        const PerSampleErrors& errs) const;
    SurfaceFinishMetrics computeSurfaceFinish(
        const PerSampleErrors& errs) const;
    FollowingErrorMetrics computeFollowing(
        const PerSampleErrors& errs) const;
};

//=============================================================================
// Combined evaluation (convenience)
//=============================================================================

/// Complete evaluation result: quantitative + qualitative + spectral.
struct CompleteEvaluation {
    QuantitativeEvaluation quantitative;
    QualitativeEvaluation qualitative;
    // SpectralEvaluation is included via PathRelativeFFT.hpp; kept as a
    // pointer to avoid a circular include dependency. The caller fills
    // this field after running PathRelativeFFT::evaluate.
    // In practice, users include both headers and use:
    //   CompleteEvaluation eval;
    //   eval.quantitative = pathEval.evaluateQuantitative(desired, actual);
    //   eval.spectral = fftEval.evaluate(desired, actual);
    //   eval.qualitative = pathEval.evaluateQualitative(eval.quantitative, &eval.spectral);
};

} // namespace tether::motion::replanner
