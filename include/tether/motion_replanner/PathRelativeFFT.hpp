/**
 * @file PathRelativeFFT.hpp
 * @brief FFT-based oscillation detection relative to the desired path.
 *
 * @details
 * Performs spectral analysis of tracking errors in both the spatial domain
 * (arc-length resampled, frequency in cycles/mm) and the temporal domain
 * (time resampled, frequency in Hz), with the errors decomposed into the
 * Frenet frame of the desired path.
 *
 * ## Why path-relative FFT?
 *
 * A standard time-domain FFT of the tracking error is smeared by feed-rate
 * variations: if the machine slows down at a corner, the time-domain
 * spectrum spreads the corner's error over a wider frequency band. By
 * resampling the error signal uniformly by arc length, we decouple the
 * spectral analysis from the feed-rate profile. A spatial oscillation
 * that repeats every corner (e.g., 0.05 cycles/mm on a path with 20mm
 * corner spacing) appears as a sharp spectral peak regardless of how fast
 * the machine was moving.
 *
 * ## Algorithm
 *
 * 1. **Frenet decomposition**: For each actual sample, compute the contour
 *    error (perpendicular to path), lag error (along path), and binormal
 *    error (out-of-plane) using `computeCertifiedContourError`.
 *
 * 2. **Resampling**: Collect (s_i, e_i) pairs and resample to a uniform
 *    grid s_k = k·Δs using PCHIP (monotone cubic) interpolation. This
 *    produces a uniformly-sampled signal suitable for FFT. The temporal
 *    domain uses (t_i, e_i) resampled to t_k = k·Δt.
 *
 * 3. **Detrending**: Remove the DC component (mean) and optionally a
 *    linear trend (least-squares regression) to prevent spectral leakage
 *    from low-frequency drift.
 *
 * 4. **Windowing**: Apply a Hann window to reduce spectral leakage from
 *    the finite observation interval.
 *
 * 5. **FFT**: Cooley-Tukey radix-2 FFT, zero-padded to the next power of 2.
 *
 * 6. **Metric derivation**: From the magnitude/phase spectra, compute:
 *    - Dominant frequency and magnitude
 *    - Spectral entropy (normalized to [0,1])
 *    - Oscillation index (power concentration at dominant frequency)
 *    - Band power (low/mid/high)
 *    - Harmonic distortion (2nd + 3rd harmonic relative to fundamental)
 *    - Peak detection (top N peaks with prominence)
 *
 * 7. **Path-geometry correlation**: Compute the expected frequencies from
 *    path geometry (corner spacing, arc lengths, segment lengths) and
 *    match them to detected spectral peaks.
 *
 * 8. **Cross-domain comparison**: Compare spatial and temporal spectra to
 *    determine whether oscillations are path-correlated (spatial) or
 *    machine-dynamic (temporal), using the feed-rate modulation index.
 *
 * ## Mathematical details
 *
 * ### Frenet frame decomposition
 *
 * Given the desired path C(s) parameterized by arc length s, the tangent
 * vector is T = C'(s), the normal is N = C''(s)/|C''(s)|, and the binormal
 * is B = T × N. The position error e = p_actual - C(s_desired) is decomposed:
 *
 *   e_contour = e · N      (perpendicular to path)
 *   e_lag = e · T           (along path, signed)
 *   e_binormal = e · B      (out-of-plane, 3D only)
 *
 * ### Arc-length resampling
 *
 * Given non-uniformly sampled (s_i, e_i), the uniformly resampled signal
 * e_k at s_k = s_min + k·Δs (k = 0..N-1) is computed via PCHIP
 * (Piecewise Cubic Hermite Interpolating Polynomial), which is monotonic
 * between data points and does not overshoot — critical for error signals
 * that may have sharp transitions.
 *
 * ### Spectral entropy
 *
 * H = -Σ p_k log₂(p_k), where p_k = PSD_k / ΣPSD.
 * Normalized: H_norm = H / log₂(N).
 * Low entropy = concentrated spectrum (strong oscillation).
 * High entropy = broadband noise.
 *
 * ### Oscillation index
 *
 * OI = PSD(f_dominant) / ΣPSD.
 * High OI = strong single-frequency oscillation.
 *
 * ### Feed-rate modulation index
 *
 * MI = f_temporal / (f_spatial × v_avg).
 * If MI ≈ 1, the oscillation is purely spatial (path-correlated).
 * If MI ≠ 1, there is temporal modulation (machine dynamics).
 *
 * @see CertifiedContourError.hpp for the Frenet decomposition.
 * @see PathEvaluator.hpp for the quantitative/qualitative evaluators.
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <vector>
#include <string>
#include <complex>
#include <cstddef>

namespace tether::motion::replanner {

//=============================================================================
// Enums
//=============================================================================

/// Which Frenet-frame error component to analyze.
enum class SpectralComponent {
    Contour,   ///< Normal to path (surface-finish-critical)
    Lag,       ///< Along path (timing)
    Binormal,  ///< Out-of-plane (3D only)
    Combined   ///< 3D position error magnitude
};

/// Domain for spectral analysis.
enum class SpectralDomain {
    Spatial,   ///< Arc-length resampled, frequency in cycles/mm
    Temporal   ///< Time resampled, frequency in Hz
};

//=============================================================================
// Result structs
//=============================================================================

/// A single detected spectral peak.
struct SpectralPeak {
    double frequency = 0.0;   ///< cycles/mm or Hz
    double magnitude = 0.0;   ///< Amplitude
    double power = 0.0;       ///< Magnitude²
    double phase = 0.0;       ///< Radians
    double prominence = 0.0;  ///< Height above surrounding valleys
};

/// Spectral metrics for one component in one domain.
struct ComponentSpectrum {
    SpectralComponent component = SpectralComponent::Contour;
    SpectralDomain domain = SpectralDomain::Spatial;

    /// Frequency axis (one-sided, 0 to Nyquist).
    std::vector<double> frequencies;
    /// Magnitude spectrum |FFT| (one-sided).
    std::vector<double> magnitudes;
    /// Phase spectrum (radians).
    std::vector<double> phases;
    /// Power spectral density |FFT|².
    std::vector<double> powerSpectralDensity;

    //--- Derived metrics ---

    /// Frequency of the highest peak (cycles/mm or Hz).
    double dominantFrequency = 0.0;
    /// Magnitude at the dominant frequency.
    double dominantMagnitude = 0.0;
    /// Normalized spectral entropy [0, 1]. Low = concentrated.
    double spectralEntropy = 0.0;
    /// Oscillation index: PSD(dominant) / ΣPSD. High = strong oscillation.
    double oscillationIndex = 0.0;
    /// Total power (sum of PSD).
    double totalPower = 0.0;
    /// RMS amplitude: sqrt(totalPower / N).
    double rmsAmplitude = 0.0;

    //--- Band power ---
    double lowBandPower = 0.0;    ///< Below lowBandCutoff
    double midBandPower = 0.0;    ///< Between low and high cutoffs
    double highBandPower = 0.0;   ///< Above highBandCutoff

    //--- Harmonic analysis ---
    /// (P_2nd + P_3rd) / P_fundamental — harmonic distortion ratio.
    double harmonicDistortion = 0.0;

    /// Detected peaks (top N, sorted by magnitude descending).
    std::vector<SpectralPeak> peaks;

    /// Whether certified contour error was used (vs tangent projection).
    bool isCertified = true;
};

/// Cross-domain comparison result (spatial vs temporal).
struct CrossDomainComparison {
    /// Feed-rate modulation index = f_temporal / (f_spatial × v_avg).
    /// If ≈ 1, oscillation is purely spatial (path-correlated).
    double feedRateModulationIndex = 0.0;

    /// Spectral coherence: Pearson correlation between normalized spatial
    /// and temporal spectra. High = similar spectral shape.
    double spectralCoherence = 0.0;

    /// True if modulation index is near 1 (path-correlated oscillation).
    bool isPathCorrelated = false;

    /// Human-readable interpretation.
    std::string interpretation;
};

/// Path-geometry correlation: matches spectral peaks to path features.
struct PathGeometryCorrelation {
    /// 1 / average corner spacing (cycles/mm). 0 if no corners.
    double cornerFrequency = 0.0;
    /// 1 / average arc segment length (cycles/mm).
    double arcFrequency = 0.0;
    /// 1 / average segment length (cycles/mm).
    double segmentFrequency = 0.0;

    /// Matched peaks: (peak frequency, geometry source description).
    std::vector<std::pair<double, std::string>> matchedPeaks;
};

/// Complete spectral evaluation.
struct SpectralEvaluation {
    //--- Spatial domain (arc-length, cycles/mm) ---
    ComponentSpectrum spatialContour;
    ComponentSpectrum spatialLag;
    ComponentSpectrum spatialBinormal;
    ComponentSpectrum spatialCombined;

    //--- Temporal domain (time, Hz) ---
    ComponentSpectrum temporalContour;
    ComponentSpectrum temporalLag;
    ComponentSpectrum temporalBinormal;
    ComponentSpectrum temporalCombined;

    //--- Cross-domain comparisons ---
    CrossDomainComparison contourComparison;
    CrossDomainComparison lagComparison;

    //--- Path geometry correlation (spatial only) ---
    PathGeometryCorrelation geometryCorrelation;

    //--- Overall oscillation assessment ---
    /// True if any component's oscillation index exceeds the threshold.
    bool oscillationDetected = false;
    /// Overall severity [0, 1] — max oscillation index across all components.
    double oscillationSeverity = 0.0;
    /// Human-readable description of detected oscillations.
    std::string oscillationDescription;
};

//=============================================================================
// Configuration
//=============================================================================

/// Configuration for the path-relative FFT evaluator.
struct FFTConfig {
    //--- Resampling ---
    /// Interpolation method for resampling to uniform grid.
    enum class Interpolation { Linear, CubicPCHIP };
    Interpolation interpolation = Interpolation::CubicPCHIP;

    /// Minimum number of samples for FFT (pad if fewer).
    std::size_t minSamples = 16;

    /// Maximum number of resampled points (cap for memory).
    std::size_t maxSamples = 65536;

    //--- Windowing ---
    /// Window function to apply before FFT.
    enum class Window { Hann, Hamming, Blackman, Rectangular };
    Window window = Window::Hann;

    //--- Detrending ---
    /// Remove the DC (mean) component before FFT.
    bool removeDC = true;
    /// Remove a linear trend (least-squares) before FFT.
    bool removeLinearTrend = true;

    //--- Peak detection ---
    /// Maximum number of peaks to report per spectrum.
    std::size_t maxPeaks = 5;
    /// Peak prominence threshold (relative to max peak magnitude).
    double peakProminenceThreshold = 0.05;

    //--- Band boundaries ---
    /// Spatial domain (cycles/mm): low/mid/high band cutoffs.
    double spatialLowBandCutoff = 0.01;
    double spatialHighBandCutoff = 0.1;
    /// Temporal domain (Hz): low/mid/high band cutoffs.
    double temporalLowBandCutoff = 5.0;
    double temporalHighBandCutoff = 100.0;

    //--- Oscillation detection ---
    /// Oscillation index above this = oscillation detected.
    double oscillationIndexThreshold = 0.3;

    //--- Algorithm selection ---
    /// Use certified contour error (Bernstein root isolation) vs tangent
    /// projection. Certified is more accurate but slower.
    bool useCertifiedContourError = true;

    //--- Active axes ---
    /// Which axes to include (X=0, Y=1, Z=2, ...). Empty = auto-detect.
    std::vector<int> activeAxes;
};

//=============================================================================
// PathRelativeFFT class
//=============================================================================

/// Computes path-relative spectral analysis for oscillation detection.
///
/// Usage:
/// ```cpp
/// PathRelativeFFT fftEval;
/// SpectralEvaluation spectral = fftEval.evaluate(desired, actual);
///
/// if (spectral.oscillationDetected) {
///     std::cout << spectral.oscillationDescription << std::endl;
/// }
/// ```
class PathRelativeFFT {
public:
    explicit PathRelativeFFT(FFTConfig config = {});

    /// Compute the full spectral evaluation from desired/actual trajectories.
    ///
    /// Internally:
    /// 1. Builds a PiecewiseNurbsPath from the desired samples.
    /// 2. Computes Frenet-decomposed errors per sample.
    /// 3. Resamples in both spatial and temporal domains.
    /// 4. Runs FFT for each component in each domain.
    /// 5. Derives metrics, detects peaks, correlates with path geometry.
    ///
    /// @param desired The desired trajectory samples (ordered by time).
    /// @param actual The actual measured trajectory samples (ordered by time).
    /// @return The complete spectral evaluation.
    SpectralEvaluation evaluate(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual) const;

    /// Compute a single component spectrum from a pre-computed error signal.
    ///
    /// This is useful for incremental/online analysis where the caller
    /// has already computed the error signal.
    ///
    /// @param errorSignal The error values at each sample.
    /// @param abscissa The abscissa values (arc lengths for spatial,
    ///        timestamps for temporal).
    /// @param component Which component this is (for labeling).
    /// @param domain Which domain (spatial or temporal).
    /// @return The component spectrum.
    ComponentSpectrum computeComponentSpectrum(
        const std::vector<double>& errorSignal,
        const std::vector<double>& abscissa,
        SpectralComponent component,
        SpectralDomain domain) const;

private:
    FFTConfig config_;

    //--- Internal helpers ---

    /// Resample (x, y) pairs to a uniform grid using PCHIP or linear.
    std::vector<double> resampleUniform(
        const std::vector<double>& values,
        const std::vector<double>& abscissa,
        double xMin, double xMax,
        std::size_t numPoints) const;

    /// PCHIP interpolation at a single query point.
    double pchipInterpolate(
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        double xq) const;

    /// Linear interpolation at a single query point.
    double linearInterpolate(
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        double xq) const;

    /// Detrend: remove DC and/or linear trend.
    void detrend(std::vector<double>& signal,
                 bool removeDC, bool removeLinear) const;

    /// Apply a window function.
    void applyWindow(std::vector<double>& signal,
                     FFTConfig::Window window) const;

    /// Cooley-Tukey radix-2 FFT (in-place, complex).
    void fft(std::vector<std::complex<double>>& data) const;

    /// Find the top N spectral peaks with prominence filtering.
    std::vector<SpectralPeak> findPeaks(
        const std::vector<double>& freqs,
        const std::vector<double>& mags,
        const std::vector<double>& phases,
        const std::vector<double>& psd,
        std::size_t maxPeaks,
        double prominenceThreshold) const;

    /// Compute path-geometry frequencies and match to spectral peaks.
    PathGeometryCorrelation computeGeometryCorrelation(
        const PiecewiseNurbsPath& path,
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<SpectralPeak>& spatialPeaks) const;

    /// Compute cross-domain comparison for one component.
    CrossDomainComparison computeCrossDomain(
        const ComponentSpectrum& spatial,
        const ComponentSpectrum& temporal,
        double avgVelocity) const;

    /// Detect active axes from the desired trajectory.
    std::vector<int> detectActiveAxes(
        const std::vector<GCodeExport::TrajectorySample>& desired) const;

    /// Compute Frenet-decomposed errors for all samples.
    struct FrenetErrors {
        std::vector<double> arcLengths;
        std::vector<double> times;
        std::vector<double> contour;
        std::vector<double> lag;
        std::vector<double> binormal;
        std::vector<double> combined;
    };

    FrenetErrors computeFrenetErrors(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual,
        const PiecewiseNurbsPath& path) const;
};

} // namespace tether::motion::replanner
