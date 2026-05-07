#pragma once
/// @file InstabilityMetrics.hpp
/// @brief Instability metric computations for the Destabilizer.
///
/// Each metric maps a closed-loop rollout trajectory to a scalar "how unstable"
/// score. The optimizer maximizes the combined weighted sum of selected metrics.

#include "DestabilizerTypes.hpp"
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <complex>

namespace Destabilizer {

/// Trajectory data from a single simulation rollout.
struct RolloutTrajectory {
    std::vector<double> times;
    std::vector<std::vector<double>> states;       ///< states[step][stateIdx]
    std::vector<std::vector<double>> outputs;      ///< outputs[step][outputIdx]
    std::vector<std::vector<double>> controlInputs;///< controlInputs[step][inputIdx]
    std::vector<double> referenceState;            ///< x_ref
    double dt = 0.001;
    int stateDim = 0;
    int outputDim = 0;
    int inputDim = 0;
};

/// Abstract base for instability metrics.
class InstabilityMetric {
public:
    virtual ~InstabilityMetric() = default;

    /// Compute the metric value from a rollout trajectory.
    virtual double compute(const RolloutTrajectory& traj) const = 0;

    /// Get the metric type.
    virtual MetricType type() const = 0;

    /// Human-readable name.
    virtual const char* name() const = 0;
};

/// Create a metric instance by type, with optional parameters.
std::unique_ptr<InstabilityMetric> createMetric(const WeightedMetric& wm);

// ---------------------------------------------------------------------------
// Concrete Metric Implementations
// ---------------------------------------------------------------------------

/// 1. Peak state deviation: max_t ||x(t) - x_ref||_2
class PeakStateDeviationMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::PeakStateDeviation; }
    const char* name() const override { return "Peak State Deviation"; }
};

/// 2. Terminal state deviation: ||x(T) - x_ref||_2
class TerminalStateDeviationMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::TerminalStateDeviation; }
    const char* name() const override { return "Terminal State Deviation"; }
};

/// 3. Integrated squared error: ∫ ||x - x_ref||² dt
class ISEMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::IntegratedSquaredError; }
    const char* name() const override { return "ISE"; }
};

/// 4. Integrated absolute error: ∫ ||x - x_ref|| dt
class IAEMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::IntegratedAbsoluteError; }
    const char* name() const override { return "IAE"; }
};

/// 5. Time-weighted ISE: ∫ t · ||x - x_ref||² dt
class ITSEMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::TimeWeightedISE; }
    const char* name() const override { return "ITSE"; }
};

/// 6. Exponential divergence rate (Lyapunov-like exponent estimate)
class ExponentialDivergenceMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::ExponentialDivergenceRate; }
    const char* name() const override { return "Exponential Divergence Rate"; }
};

/// 7. Region-of-attraction escape (soft)
class ROAEscapeMetric : public InstabilityMetric {
public:
    explicit ROAEscapeMetric(const std::vector<SafeSetBound>& bounds);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::RegionOfAttractionEscape; }
    const char* name() const override { return "ROA Escape"; }
private:
    std::vector<SafeSetBound> bounds_;
};

/// 8. Control effort saturation time
class ControlSaturationMetric : public InstabilityMetric {
public:
    ControlSaturationMetric(double satMin, double satMax);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::ControlSaturationTime; }
    const char* name() const override { return "Control Saturation Time"; }
private:
    double satMin_, satMax_;
};

/// 9. Oscillation amplitude growth
class OscillationGrowthMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::OscillationAmplitudeGrowth; }
    const char* name() const override { return "Oscillation Amplitude Growth"; }
};

/// 10. Limit-cycle escape count
class LimitCycleEscapeMetric : public InstabilityMetric {
public:
    explicit LimitCycleEscapeMetric(double threshold = 1.0);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::LimitCycleEscapeCount; }
    const char* name() const override { return "Limit-Cycle Escape Count"; }
private:
    double threshold_;
};

/// 11. Phase-space volume expansion (finite-time Lyapunov exponent)
class PhaseSpaceExpansionMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::PhaseSpaceVolumeExpansion; }
    const char* name() const override { return "Phase-Space Volume Expansion"; }
};

/// 12. Spectral radius of trajectory-sensitivity matrix
class SpectralRadiusMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::SpectralRadiusSensitivity; }
    const char* name() const override { return "Spectral Radius Sensitivity"; }
};

/// 13. Settling-time violation
class SettlingTimeViolationMetric : public InstabilityMetric {
public:
    explicit SettlingTimeViolationMetric(double settlingSpec = 1.0);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::SettlingTimeViolation; }
    const char* name() const override { return "Settling-Time Violation"; }
private:
    double settlingSpec_;
};

/// 14. Overshoot magnitude
class OvershootMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::OvershootMagnitude; }
    const char* name() const override { return "Overshoot Magnitude"; }
};

/// 15. Energy injected into plant
class EnergyInjectedMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::EnergyInjected; }
    const char* name() const override { return "Energy Injected"; }
};

/// 16. Constraint-violation integral
class ConstraintViolationMetric : public InstabilityMetric {
public:
    explicit ConstraintViolationMetric(const std::vector<SafeSetBound>& bounds);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::ConstraintViolationIntegral; }
    const char* name() const override { return "Constraint Violation Integral"; }
private:
    std::vector<SafeSetBound> bounds_;
};

/// 17. Time-to-instability: T - t* where t* is first divergence time
class TimeToInstabilityMetric : public InstabilityMetric {
public:
    explicit TimeToInstabilityMetric(double threshold = 10.0);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::TimeToInstability; }
    const char* name() const override { return "Time to Instability"; }
private:
    double threshold_;
};

/// 18. Controller bandwidth exceedance
class BandwidthExceedanceMetric : public InstabilityMetric {
public:
    explicit BandwidthExceedanceMetric(double bandwidth = 100.0);
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::ControllerBandwidthExceedance; }
    const char* name() const override { return "Controller Bandwidth Exceedance"; }
private:
    double bandwidth_;
};

/// 19. Covariance growth
class CovarianceGrowthMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::CovarianceGrowth; }
    const char* name() const override { return "Covariance Growth"; }
};

/// 20. Nonlinear distortion metric
class NonlinearDistortionMetric : public InstabilityMetric {
public:
    double compute(const RolloutTrajectory& traj) const override;
    MetricType type() const override { return MetricType::NonlinearDistortion; }
    const char* name() const override { return "Nonlinear Distortion"; }
};

// ---------------------------------------------------------------------------
// Combined Metric Evaluator
// ---------------------------------------------------------------------------

/// Evaluates a weighted combination of multiple metrics.
class CombinedMetricEvaluator {
public:
    CombinedMetricEvaluator() = default;

    /// Configure from weighted metric specifications and safe-set bounds.
    void configure(const std::vector<WeightedMetric>& metrics,
                   const std::vector<SafeSetBound>& safeSet,
                   double controlSatMin, double controlSatMax);

    /// Evaluate J(trajectory) = Σ w_i · m_i(trajectory).
    double evaluate(const RolloutTrajectory& traj) const;

    /// Evaluate and return per-metric breakdown.
    std::vector<double> evaluateBreakdown(const RolloutTrajectory& traj) const;

    int metricCount() const { return static_cast<int>(metrics_.size()); }

private:
    std::vector<std::pair<double, std::unique_ptr<InstabilityMetric>>> metrics_;
};

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------

/// Compute L2 norm of a vector.
double vectorNorm(const std::vector<double>& v);

/// Compute L-infinity norm.
double vectorNormInf(const std::vector<double>& v);

/// Compute difference vector: a - b.
std::vector<double> vectorDiff(const std::vector<double>& a,
                                const std::vector<double>& b);

/// Simple FFT (Cooley-Tukey) for power spectrum estimation.
/// Input: real-valued signal. Output: magnitude spectrum (positive frequencies).
std::vector<double> computeFFTMagnitude(const std::vector<double>& signal, double dt);

/// Compute power spectral density.
std::vector<std::pair<double, double>> computePSD(const std::vector<double>& signal,
                                                    double dt);

/// Linear least-squares fit: y = a + b*x. Returns {a, b}.
std::pair<double, double> linearFit(const std::vector<double>& x,
                                     const std::vector<double>& y);

/// Estimate dominant eigenvalue magnitude from a sequence of states.
double estimateSpectralRadius(const std::vector<std::vector<double>>& states, double dt);

} // namespace Destabilizer
