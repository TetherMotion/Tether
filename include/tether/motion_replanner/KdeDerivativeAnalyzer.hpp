/**
 * @file KdeDerivativeAnalyzer.hpp
 * @brief Kernel density estimation of 1st-derivative vs deviation relationships.
 *
 * @details
 * Estimates the joint probability density p(derivative, deviation) from the
 * individual (di, ei) sample points of a desired-vs-actual trajectory, using
 * 2D kernel density estimation (KDE). The resulting density surface reveals
 * the statistical relationship between how fast the machine was moving (or
 * accelerating, or jerking) and how much it deviated from the desired path —
 * information that is invisible in scatter plots for large N and impossible
 * to extract from summary statistics alone.
 *
 * ## Why derivative vs deviation?
 *
 * Tracking error typically scales with kinematic demands:
 *
 * - **Velocity vs contour error** — reveals velocity-dependent servo lag,
 *   resonance excitation bands, and feed-rate safety envelopes.
 * - **Acceleration vs contour error** — reveals inertia-induced overshoot,
 *   structural compliance, and acceleration limits.
 * - **Jerk vs contour error** — reveals discontinuity-induced vibration and
 *   the jerk threshold above which surface finish degrades.
 * - **Curvature vs contour error** — reveals corner-rounding and the
 *   curvature threshold where the controller can no longer hold tolerance.
 *
 * The KDE surface p(d, e) makes these relationships visible as ridges, bands,
 * or bifurcations that a simple scatter plot would obscure for large N.
 *
 * ## Algorithm
 *
 * 1. **Sample extraction**: For each (desired[i], actual[i]) pair, compute
 *    the kinematic derivative d_i (velocity / acceleration / jerk / curvature
 *    magnitude) and the deviation e_i (contour / lag / combined error).
 *
 * 2. **Bandwidth selection**: The kernel bandwidth (h_d, h_e) controls the
 *    smoothness of the estimate. Supported methods:
 *    - **Silverman's rule of thumb** (default): h = 1.06·σ·n^(-1/5)
 *    - **Scott's rule**: h = n^(-1/5)·σ (multivariate generalization)
 *    - **ISJ (Improved Sheather-Jones)**: data-driven, pilot-estimate based
 *    - **Fixed**: user-specified bandwidth
 *    - **Cross-validation**: least-squares or likelihood CV (expensive)
 *
 * 3. **KDE evaluation**: The 2D density at grid point (d, e) is
 *
 *      p(d, e) = (1/(n·h_d·h_e)) Σ_i K((d - d_i)/h_d) · K((e - e_i)/h_e)
 *
 *    Supported kernels:
 *    - **Gaussian** (default): K(u) = exp(-u²/2) / √(2π)
 *    - **Epanechnikov**: K(u) = (3/4)(1 - u²) for |u| ≤ 1
 *    - **Uniform**: K(u) = 1/2 for |u| ≤ 1
 *    - **Triangular**: K(u) = (1 - |u|) for |u| ≤ 1
 *    - **Quartic (biweight)**: K(u) = (15/16)(1 - u²)² for |u| ≤ 1
 *    - **Cosine**: K(u) = (π/4)cos(πu/2) for |u| ≤ 1
 *
 *    For Gaussian kernels, the entire grid is evaluated via a fast Gaussian
 *    sum using separable 1D convolutions (O(n·G) instead of O(n·G²) where G
 *    is the grid size per axis). For compact kernels, a binned approximation
 *    is used: samples are histogrammed first, then the kernel is convolved
 *    with the histogram.
 *
 * 4. **Derived metrics**: From the density surface, compute:
 *    - Marginal densities p(d) and p(e)
 *    - Conditional density p(e | d) = p(d, e) / p(d)
 *    - Conditional mean E[e | d] and variance Var[e | d]
 *    - Quantile contours (5%, 25%, 50%, 75%, 95%) of p(e | d)
 *    - Mutual information I(d; e) = ΣΣ p(d,e)·log₂(p(d,e)/(p(d)·p(e)))
 *    - Correlation ratio η² = Var[E[e|d]] / Var[e]
 *    - Maximum density location (mode of the joint distribution)
 *    - Tail indices (fraction of mass above deviation thresholds)
 *
 * 5. **Threshold extraction**: From the conditional density, extract the
 *    feed-rate / acceleration / jerk / curvature threshold at which the
 *    deviation exceeds a specified tolerance with a given probability.
 *    This yields actionable machine tuning limits.
 *
 * ## Mathematical details
 *
 * ### Silverman's rule of thumb
 *
 *   h = 1.06 · σ̂ · n^(-1/5)
 *
 * where σ̂ is the sample standard deviation. This is optimal for Gaussian
 * data but oversmooths multimodal distributions.
 *
 * ### Improved Sheather-Jones (ISJ)
 *
 * Solves the fixed-point equation h = 1.06·(t(h))^(1/5)·n^(-1/5) where
 * t(h) is a functional of the pilot estimate. More robust for multimodal
 * and skewed data. See Sheather & Jones (1991).
 *
 * ### Mutual information
 *
 *   I(d; e) = Σ_d Σ_e p(d,e) · log₂( p(d,e) / (p(d)·p(e)) )
 *
 * High I indicates strong dependence between derivative and deviation.
 * I = 0 for independent variables.
 *
 * ### Correlation ratio
 *
 *   η² = Var[E[e | d]] / Var[e]
 *
 * η² ∈ [0, 1]. η² = 0 means e is independent of d; η² = 1 means e is a
 * deterministic function of d. Unlike Pearson correlation, η² captures
 * nonlinear relationships.
 *
 * @see PathEvaluator.hpp for the underlying error computation.
 * @see SvgExporter.hpp for KDE heatmap visualization.
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <vector>
#include <string>
#include <cstddef>
#include <array>
#include <functional>

namespace tether::motion::replanner {

//=============================================================================
// Enums
//=============================================================================

/// Which kinematic derivative to use as the X-axis of the KDE.
enum class DerivativeAxis {
    Velocity,       ///< Speed magnitude (mm/s)
    Acceleration,   ///< Acceleration magnitude (mm/s²)
    Jerk,           ///< Jerk magnitude (mm/s³)
    Curvature,      ///< Path curvature (1/mm)
    FeedRate,       ///< Commanded feed rate (mm/s) — same as velocity for desired
    ArcLength,      ///< Arc length position (mm) — for spatial distribution
    Time,           ///< Time (s) — for temporal distribution
};

/// Which deviation measure to use as the Y-axis of the KDE.
enum class DeviationAxis {
    ContourError,   ///< Perpendicular distance to path (mm)
    LagError,       ///< Signed arc-length offset (mm)
    CombinedError,  ///< 3D Euclidean distance (mm)
    BinormalError,  ///< Out-of-plane component (mm)
    TrackingError,  ///< Position error magnitude (mm)
    VelocityError,  ///< Velocity tracking error (mm/s)
    AccelerationError, ///< Acceleration tracking error (mm/s²)
};

/// Kernel function for KDE.
enum class KernelType {
    Gaussian,       ///< K(u) = exp(-u²/2) / √(2π)  [unbounded]
    Epanechnikov,   ///< K(u) = (3/4)(1 - u²)       [|u| ≤ 1]
    Uniform,        ///< K(u) = 1/2                 [|u| ≤ 1]
    Triangular,     ///< K(u) = (1 - |u|)           [|u| ≤ 1]
    Quartic,        ///< K(u) = (15/16)(1 - u²)²    [|u| ≤ 1]  (biweight)
    Cosine,         ///< K(u) = (π/4)cos(πu/2)      [|u| ≤ 1]
};

/// Bandwidth selection method.
enum class BandwidthMethod {
    Silverman,      ///< Rule of thumb: h = 1.06·σ·n^(-1/5)
    Scott,          ///< Scott's rule: h = n^(-1/5)·σ
    ISJ,            ///< Improved Sheather-Jones (data-driven)
    Fixed,          ///< User-specified bandwidth
    LeastSquaresCV, ///< Least-squares cross-validation (expensive)
    LikelihoodCV,   ///< Likelihood cross-validation (expensive)
};

/// Colormap for heatmap rendering.
enum class KdeColormap {
    Viridis,        ///< Perceptually uniform, purple-to-yellow
    Inferno,        ///< Perceptually uniform, black-to-yellow
    Plasma,         ///< Perceptually uniform, purple-to-yellow
    Magma,          ///< Perceptually uniform, black-to-white
    Jet,            ///< Classic rainbow (not perceptually uniform)
    Hot,            ///< Black-to-red-to-yellow-to-white
    Cool,           ///< Cyan-to-magenta
    Grayscale,      ///< Black-to-white
    BlueRed,        ///< Diverging blue-to-red
};

//=============================================================================
// Result structs
//=============================================================================

/// A 2D KDE grid (density surface).
struct KdeGrid {
    /// X-axis (derivative) bin centers.
    std::vector<double> xBins;
    /// Y-axis (deviation) bin centers.
    std::vector<double> yBins;
    /// Density values, row-major: density[y * xBins.size() + x].
    std::vector<double> density;
    /// X-axis bandwidth used.
    double bandwidthX = 0.0;
    /// Y-axis bandwidth used.
    double bandwidthY = 0.0;
    /// Number of samples used.
    std::size_t sampleCount = 0;

    /// 2D index lookup.
    std::size_t index(std::size_t ix, std::size_t iy) const {
        return iy * xBins.size() + ix;
    }

    /// Get density at bin (ix, iy).
    double at(std::size_t ix, std::size_t iy) const {
        return density[index(ix, iy)];
    }

    /// Maximum density value.
    double maxDensity() const;

    /// Total mass (should be ≈ 1.0 for a proper PDF).
    double totalMass() const;

    /// Marginal density along X (integrate out Y).
    std::vector<double> marginalX() const;

    /// Marginal density along Y (integrate out X).
    std::vector<double> marginalY() const;
};

/// Marginal distribution statistics.
struct MarginalStats {
    double mean = 0.0;
    double stdDev = 0.0;
    double skewness = 0.0;
    double kurtosis = 0.0;
    double min = 0.0;
    double max = 0.0;
    double median = 0.0;
    double p05 = 0.0;  ///< 5th percentile
    double p25 = 0.0;  ///< 25th percentile
    double p75 = 0.0;  ///< 75th percentile
    double p95 = 0.0;  ///< 95th percentile
    double mode = 0.0; ///< Most likely value (peak of marginal density)
};

/// Conditional distribution p(e | d) statistics for a single d bin.
struct ConditionalStats {
    double xValue = 0.0;       ///< The derivative value d
    double meanY = 0.0;        ///< E[e | d]
    double stdY = 0.0;         ///< sqrt(Var[e | d])
    double medianY = 0.0;      ///< Median of p(e | d)
    double p05Y = 0.0;         ///< 5th percentile
    double p25Y = 0.0;         ///< 25th percentile
    double p75Y = 0.0;         ///< 75th percentile
    double p95Y = 0.0;         ///< 95th percentile
    double modeY = 0.0;        ///< Mode of p(e | d)
    double mass = 0.0;         ///< Total mass (proportion of samples at this d)
    bool valid = false;        ///< True if there is enough mass for reliable stats
};

/// Threshold extraction result: the derivative value at which deviation
/// exceeds a tolerance with a given probability.
struct DeviationThreshold {
    double derivativeValue = 0.0;  ///< The threshold derivative value
    double probability = 0.0;      ///< P(e > tolerance | d = threshold)
    double tolerance = 0.0;        ///< The deviation tolerance (mm)
    DerivativeAxis derivativeType = DerivativeAxis::Velocity;
    DeviationAxis deviationType = DeviationAxis::ContourError;
    bool found = false;            ///< True if a threshold was found
    std::string description;       ///< Human-readable interpretation
};

/// Complete KDE derivative-vs-deviation evaluation.
struct KdeEvaluation {
    //--- Input configuration echo ---
    DerivativeAxis derivativeAxis = DerivativeAxis::Velocity;
    DeviationAxis deviationAxis = DeviationAxis::ContourError;
    KernelType kernel = KernelType::Gaussian;
    BandwidthMethod bandwidthMethod = BandwidthMethod::Silverman;

    //--- Raw sample data ---
    /// (d_i, e_i) pairs extracted from the trajectory.
    std::vector<double> derivatives;
    std::vector<double> deviations;
    std::vector<double> arcLengths;  ///< Arc length for each sample
    std::vector<double> times;       ///< Time for each sample

    //--- KDE grid ---
    KdeGrid grid;

    //--- Marginal statistics ---
    MarginalStats derivativeMarginal;
    MarginalStats deviationMarginal;

    //--- Conditional statistics p(e | d) ---
    std::vector<ConditionalStats> conditional;

    //--- Dependence metrics ---
    /// Pearson correlation coefficient ∈ [-1, 1].
    double pearsonCorrelation = 0.0;
    /// Spearman rank correlation ∈ [-1, 1].
    double spearmanCorrelation = 0.0;
    /// Kendall's tau ∈ [-1, 1].
    double kendallTau = 0.0;
    /// Mutual information I(d; e) in bits.
    double mutualInformation = 0.0;
    /// Correlation ratio η² ∈ [0, 1] (nonlinear dependence).
    double correlationRatio = 0.0;
    /// Distance correlation ∈ [0, 1] (captures any dependence).
    double distanceCorrelation = 0.0;
    /// Hirschberg-Goodman dependence index (normalized MI).
    double dependenceIndex = 0.0;

    //--- Density surface features ---
    /// (d, e) at the peak of the joint density.
    double modeDerivative = 0.0;
    double modeDeviation = 0.0;
    /// Maximum density value.
    double maxDensity = 0.0;
    /// Entropy of the joint distribution H(d, e) in bits.
    double jointEntropy = 0.0;
    /// Conditional entropy H(e | d) in bits.
    double conditionalEntropy = 0.0;
    /// H(d) - H(d|e) normalized to [0, 1] — symmetric dependence.
    double normalizedMutualInfo = 0.0;

    //--- Tail / risk analysis ---
    /// Fraction of samples with deviation > tolerance.
    double tailFraction = 0.0;
    /// Value-at-risk (VaR): deviation at the 95th percentile.
    double var95 = 0.0;
    /// Expected tail deviation (ETD): E[e | e > VaR95].
    double expectedTailDeviation = 0.0;
    /// Conditional value-at-risk (CVaR): E[e | e > VaR95].
    double conditionalVar95 = 0.0;

    //--- Thresholds ---
    /// Extracted thresholds for various tolerances.
    std::vector<DeviationThreshold> thresholds;

    //--- Quality flags ---
    /// True if the KDE has enough samples for reliable estimates.
    bool hasSufficientData = false;
    /// Recommended bandwidth for X (may differ from used if auto-selected).
    double recommendedBandwidthX = 0.0;
    /// Recommended bandwidth for Y.
    double recommendedBandwidthY = 0.0;
    /// Human-readable summary.
    std::string summary;
};

//=============================================================================
// Configuration
//=============================================================================

/// Configuration for the KDE derivative analyzer.
struct KdeConfig {
    //--- Axis selection ---
    DerivativeAxis derivativeAxis = DerivativeAxis::Velocity;
    DeviationAxis deviationAxis = DeviationAxis::ContourError;

    //--- Kernel ---
    KernelType kernel = KernelType::Gaussian;

    //--- Bandwidth ---
    BandwidthMethod bandwidthMethod = BandwidthMethod::Silverman;
    /// Fixed bandwidth for X (used when method = Fixed).
    double fixedBandwidthX = 0.0;
    /// Fixed bandwidth for Y (used when method = Fixed).
    double fixedBandwidthY = 0.0;
    /// Bandwidth scale factor (multiplies the auto-selected bandwidth).
    /// 1.0 = default, <1 = less smoothing, >1 = more smoothing.
    double bandwidthScale = 1.0;

    //--- Grid ---
    /// Number of bins along the X (derivative) axis.
    std::size_t gridX = 128;
    /// Number of bins along the Y (deviation) axis.
    std::size_t gridY = 128;
    /// X-axis range [min, max]. If empty, auto-computed from data.
    std::array<double, 2> xRange = {0.0, 0.0};
    /// Y-axis range [min, max]. If empty, auto-computed from data.
    std::array<double, 2> yRange = {0.0, 0.0};
    /// Pad the auto-computed range by this fraction (e.g., 0.05 = 5%).
    double rangePadding = 0.05;
    /// If true, use logarithmic spacing for the X axis bins.
    bool logX = false;
    /// If true, use logarithmic spacing for the Y axis bins.
    bool logY = false;

    //--- Conditional statistics ---
    /// Minimum mass per X bin for conditional stats to be valid.
    double minConditionalMass = 0.01;
    /// Number of quantile levels to compute for conditional distributions.
    std::vector<double> quantileLevels = {0.05, 0.25, 0.50, 0.75, 0.95};

    //--- Threshold extraction ---
    /// Deviation tolerances for which to extract derivative thresholds (mm).
    std::vector<double> tolerances = {0.005, 0.01, 0.02, 0.05, 0.1, 0.2};
    /// Probability threshold for "exceeds tolerance" (e.g., 0.05 = 5% chance).
    double thresholdProbability = 0.05;

    //--- Algorithm ---
    /// Use certified contour error (Bernstein root isolation) vs tangent
    /// projection. Certified is more accurate but slower.
    bool useCertifiedContourError = true;

    /// Use binned KDE approximation for large N (faster, slightly less accurate).
    /// If sample count exceeds this threshold, binned estimation is used.
    std::size_t binnedThreshold = 5000;

    /// Number of bins for the intermediate histogram in binned KDE.
    std::size_t binnedHistogramSize = 512;

    //--- Active axes ---
    /// Which axes to include (X=0, Y=1, Z=2, ...). Empty = auto-detect.
    std::vector<int> activeAxes;
};

//=============================================================================
// KdeDerivativeAnalyzer class
//=============================================================================

/// Computes kernel density estimates of derivative-vs-deviation relationships.
///
/// Usage:
/// ```cpp
/// KdeConfig config;
/// config.derivativeAxis = DerivativeAxis::Velocity;
/// config.deviationAxis = DeviationAxis::ContourError;
/// KdeDerivativeAnalyzer analyzer(config);
///
/// KdeEvaluation eval = analyzer.evaluate(desired, actual);
///
/// if (eval.hasSufficientData) {
///     std::cout << "Mutual information: " << eval.mutualInformation << " bits\n";
///     std::cout << "Correlation ratio: " << eval.correlationRatio << "\n";
///     for (const auto& t : eval.thresholds) {
///         if (t.found) {
///             std::cout << t.description << "\n";
///         }
///     }
/// }
/// ```

/// @brief Dependence metrics between two sample vectors.
/// @see KdeDerivativeAnalyzer::computeDependence
/// @see DependenceAnalyzer::compute
struct DependenceMetrics {
    double pearson = 0.0;
    double spearman = 0.0;
    double kendall = 0.0;
    double mutualInformation = 0.0;
    double correlationRatio = 0.0;
    double distanceCorrelation = 0.0;
    double dependenceIndex = 0.0;
    double jointEntropy = 0.0;
    double conditionalEntropy = 0.0;
    double normalizedMutualInfo = 0.0;
};

/// @brief Tail risk metrics (VaR, CVaR, ETD).
/// @see KdeDerivativeAnalyzer::computeTailRisk
/// @see KdeStatistics::computeTailRisk
struct TailRisk {
    double tailFraction = 0.0;
    double var95 = 0.0;
    double expectedTailDeviation = 0.0;
    double conditionalVar95 = 0.0;
};

class KdeDerivativeAnalyzer {
public:
    explicit KdeDerivativeAnalyzer(KdeConfig config = {});

    //--- Main evaluation ---

    /// Compute the full KDE evaluation from desired/actual trajectories.
    ///
    /// Internally:
    /// 1. Builds a PiecewiseNurbsPath from the desired samples.
    /// 2. Computes per-sample derivatives and deviations.
    /// 3. Selects bandwidths (auto or fixed).
    /// 4. Evaluates the 2D KDE on a regular grid.
    /// 5. Computes marginal, conditional, and dependence statistics.
    /// 6. Extracts deviation thresholds.
    ///
    /// @param desired The desired trajectory samples (ordered by time).
    /// @param actual The actual measured trajectory samples (ordered by time).
    /// @return The complete KDE evaluation.
    KdeEvaluation evaluate(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual) const;

    //--- Low-level API (for advanced use cases) ---

    /// Extract (derivative, deviation) pairs from trajectories.
    ///
    /// @param desired Desired trajectory samples.
    /// @param actual Actual trajectory samples.
    /// @return (derivatives, deviations, arcLengths, times) vectors.
    struct SamplePairs {
        std::vector<double> derivatives;
        std::vector<double> deviations;
        std::vector<double> arcLengths;
        std::vector<double> times;
    };
    SamplePairs extractPairs(
        const std::vector<GCodeExport::TrajectorySample>& desired,
        const std::vector<GCodeExport::TrajectorySample>& actual) const;

    /// Compute bandwidths from data using the configured method.
    /// @param data The sample values.
    /// @param method Bandwidth selection method.
    /// @param fixedValue Fixed value (used when method = Fixed).
    /// @param scale Multiplier applied to the result.
    /// @return The selected bandwidth.
    double computeBandwidth(const std::vector<double>& data,
                            BandwidthMethod method,
                            double fixedValue,
                            double scale) const;

    /// Evaluate a 1D kernel function at a normalized distance.
    /// @param kernel Kernel type.
    /// @param u Normalized distance (x - x_i) / h.
    /// @return Kernel weight K(u).
    double kernelValue(KernelType kernel, double u) const;

    /// Evaluate the 2D KDE on a regular grid.
    /// @param derivatives X-axis sample values.
    /// @param deviations Y-axis sample values.
    /// @param hX X bandwidth.
    /// @param hY Y bandwidth.
    /// @param xMin, xMax, yMin, yMax Grid bounds.
    /// @param nX, nY Grid resolution.
    /// @param kernel Kernel type.
    /// @return The KDE grid.
    KdeGrid evaluateKde(
        const std::vector<double>& derivatives,
        const std::vector<double>& deviations,
        double hX, double hY,
        double xMin, double xMax,
        double yMin, double yMax,
        std::size_t nX, std::size_t nY,
        KernelType kernel) const;

    /// Compute marginal statistics from a 1D sample.
    MarginalStats computeMarginalStats(const std::vector<double>& data) const;

    /// Compute conditional statistics p(e | d) from the KDE grid.
    std::vector<ConditionalStats> computeConditionalStats(
        const KdeGrid& grid,
        const std::vector<double>& quantileLevels,
        double minMass) const;

    /// Compute dependence metrics between two samples.
    /// @see DependenceMetrics (defined at namespace scope)
    DependenceMetrics computeDependence(
        const std::vector<double>& x,
        const std::vector<double>& y,
        const KdeGrid& grid) const;

    /// Extract deviation thresholds from conditional statistics.
    std::vector<DeviationThreshold> extractThresholds(
        const std::vector<ConditionalStats>& conditional,
        const std::vector<double>& tolerances,
        double probability,
        DerivativeAxis derType,
        DeviationAxis devType) const;

    /// Compute tail risk metrics (VaR, CVaR, ETD).
    /// @see TailRisk (defined at namespace scope)
    TailRisk computeTailRisk(
        const std::vector<double>& deviations,
        double varPercentile = 0.95) const;

    //--- Public helpers (for testing and external use) ---

    /// Compute the mode (peak) of a 1D density estimate.
    static double densityMode(const std::vector<double>& bins,
                              const std::vector<double>& density);

    /// Compute the quantile of a 1D density estimate.
    static double densityQuantile(const std::vector<double>& bins,
                                  const std::vector<double>& density,
                                  double q);

    /// Compute the entropy of a 1D density estimate (bits).
    static double densityEntropy(const std::vector<double>& density);

    /// Convert a colormap enum to an RGB color for a value in [0, 1].
    /// @return (r, g, b) in [0, 255].
    static std::array<int, 3> colormapColor(KdeColormap cmap, double value);

private:
    KdeConfig config_;

    //--- Internal helpers ---

    /// Detect active axes from the desired trajectory.
    std::vector<int> detectActiveAxes(
        const std::vector<GCodeExport::TrajectorySample>& desired) const;

    /// Compute the kinematic derivative for a single sample.
    double computeDerivative(
        const GCodeExport::TrajectorySample& desired,
        const GCodeExport::TrajectorySample& actual,
        std::size_t index,
        const std::vector<GCodeExport::TrajectorySample>& allActual,
        DerivativeAxis type) const;

    /// Compute the deviation for a single sample.
    double computeDeviation(
        const GCodeExport::TrajectorySample& desired,
        const GCodeExport::TrajectorySample& actual,
        const PiecewiseNurbsPath& path,
        double sPath,
        DeviationAxis type) const;

    /// Improved Sheather-Jones bandwidth (data-driven).
    double isjBandwidth(const std::vector<double>& data) const;

    /// Least-squares cross-validation bandwidth.
    double lscvBandwidth(const std::vector<double>& data) const;

    /// Likelihood cross-validation bandwidth.
    double likelihoodCvBandwidth(const std::vector<double>& data) const;

    /// Compute Spearman rank correlation.
    double spearmanRank(const std::vector<double>& x,
                        const std::vector<double>& y) const;

    /// Compute Kendall's tau.
    double kendallTau(const std::vector<double>& x,
                      const std::vector<double>& y) const;

    /// Compute distance correlation (Szekely & Rizzo, 2007).
    double distanceCorrelation(const std::vector<double>& x,
                               const std::vector<double>& y) const;

    /// Compute mutual information from the KDE grid.
    double mutualInformationFromGrid(const KdeGrid& grid) const;

    /// Compute entropy values from the KDE grid.
    struct GridEntropy {
        double joint = 0.0;
        double conditional = 0.0;
        double normalizedMI = 0.0;
    };
    GridEntropy entropyFromGrid(const KdeGrid& grid) const;

    /// Binned KDE for large N (histogram + kernel convolution).
    KdeGrid binnedKde(
        const std::vector<double>& derivatives,
        const std::vector<double>& deviations,
        double hX, double hY,
        double xMin, double xMax,
        double yMin, double yMax,
        std::size_t nX, std::size_t nY,
        KernelType kernel) const;

    /// Generate a human-readable summary.
    std::string generateSummary(const KdeEvaluation& eval) const;
};

//=============================================================================
// Utility functions
//=============================================================================

/// Convert a DerivativeAxis to a human-readable string.
std::string toString(DerivativeAxis axis);

/// Convert a DeviationAxis to a human-readable string.
std::string toString(DeviationAxis axis);

/// Convert a KernelType to a human-readable string.
std::string toString(KernelType kernel);

/// Convert a BandwidthMethod to a human-readable string.
std::string toString(BandwidthMethod method);

/// Convert a KdeColormap to a human-readable string.
std::string toString(KdeColormap cmap);

/// Get the unit string for a DerivativeAxis.
std::string unitString(DerivativeAxis axis);

/// Get the unit string for a DeviationAxis.
std::string unitString(DeviationAxis axis);

} // namespace tether::motion::replanner
