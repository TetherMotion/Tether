#pragma once
/// @file DestabilizerTypes.hpp
/// @brief Core types for the Destabilizer adversarial stability-testing subsystem.

#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <atomic>
#include <mutex>

namespace Destabilizer {

// ---------------------------------------------------------------------------
// Perturbation Parameterization
// ---------------------------------------------------------------------------

/// Identifies how the perturbation signal u_p(t) is parameterized.
enum class PerturbationType {
    PiecewiseConstant,  ///< N segments of constant amplitude
    PiecewiseLinear,    ///< N breakpoints with interpolation
    FourierSpectral,    ///< Sum of K sinusoids (amp, freq, phase)
    NeuralMLP,          ///< Small MLP mapping state → perturbation
    ImpulseTrain,       ///< M impulses with (time, amplitude, duration)
    BangBang            ///< Switching times between +max and -max
};

/// Identifies the optimization algorithm variant.
enum class OptimizerType {
    VanillaSGD,         ///< Vanilla SGD with finite-difference gradients
    SPSA,               ///< Simultaneous Perturbation Stochastic Approximation
    Adam,               ///< Adam optimizer with FD/SPSA gradients
    CMAES,              ///< Covariance Matrix Adaptation Evolution Strategy
    CrossEntropy,       ///< Cross-Entropy Method (CEM)
    RandomSearch,       ///< Random search baseline
    CoordinateDescent,  ///< Coordinate descent over θ components
    EvolutionaryStrategy ///< (μ, λ)-ES with mutation and selection
};

/// Identifies an instability metric.
enum class MetricType {
    PeakStateDeviation,          ///< max_t ||x(t) - x_ref||
    TerminalStateDeviation,      ///< ||x(T) - x_ref||
    IntegratedSquaredError,      ///< ∫ ||x - x_ref||² dt
    IntegratedAbsoluteError,     ///< ∫ ||x - x_ref|| dt
    TimeWeightedISE,             ///< ∫ t·||x - x_ref||² dt
    ExponentialDivergenceRate,   ///< Slope of log||x(t) - x_ref|| vs t
    RegionOfAttractionEscape,    ///< Soft escape from safe-set
    ControlSaturationTime,       ///< Fraction of time at saturation
    OscillationAmplitudeGrowth,  ///< Envelope growth rate
    LimitCycleEscapeCount,       ///< Threshold-crossing count
    PhaseSpaceVolumeExpansion,   ///< Finite-time Lyapunov exponent
    SpectralRadiusSensitivity,   ///< dx(T)/dx(0) spectral radius
    SettlingTimeViolation,       ///< Ratio of observed to spec settling time
    OvershootMagnitude,          ///< Max excursion past setpoint
    EnergyInjected,              ///< ∫ ||x||² dt plant energy
    ConstraintViolationIntegral, ///< ∫ max(0, g(x))² dt
    TimeToInstability,           ///< T - t* (first divergence time)
    ControllerBandwidthExceedance, ///< Fraction of energy above bandwidth
    CovarianceGrowth,            ///< Trace growth of propagated covariance
    NonlinearDistortion          ///< FFT harmonic distortion metric
};

// ---------------------------------------------------------------------------
// Configuration Structures
// ---------------------------------------------------------------------------

/// Hard constraint on a single perturbation channel.
struct ChannelConstraints {
    double amplitudeMax = 1.0;         ///< |u_p(t)| ≤ A_max per sample
    double rateMax = 1e6;              ///< |du_p/dt| ≤ Rate_max
    double energyMax = 1e6;            ///< ∫ u_p(t)² dt ≤ E_max
    double freqMin = 0.0;              ///< Minimum allowed frequency [Hz]
    double freqMax = 1e6;              ///< Maximum allowed frequency [Hz]
    double dutyCycleMax = 1.0;         ///< Fraction of time u_p ≠ 0 ≤ D_max
};

/// Perturbation channel definition.
struct PerturbationChannel {
    int inputIndex = 0;                ///< Which physical input/disturbance channel
    std::string name;                  ///< Human-readable channel name
    ChannelConstraints constraints;    ///< Hard constraints for this channel
    std::string tooltipRationale;      ///< Real-world rationale for defaults
};

/// Metric with user-specified weight.
struct WeightedMetric {
    MetricType type = MetricType::PeakStateDeviation;
    double weight = 1.0;

    // Metric-specific parameters
    double threshold = 1e6;            ///< For escape/threshold-based metrics
    double controllerBandwidth = 100.0; ///< For bandwidth exceedance metric [Hz]
    double settlingTimeSpec = 1.0;      ///< For settling-time violation [s]
};

/// Safe-set boundary for escape metrics.
struct SafeSetBound {
    int stateIndex = 0;
    double lowerBound = -1e9;
    double upperBound = 1e9;
    std::string stateName;
};

/// Optimizer hyperparameters.
struct OptimizerConfig {
    OptimizerType type = OptimizerType::Adam;
    double learningRate = 0.01;
    int batchSize = 1;

    // SGD / finite-difference
    double fdEpsilon = 1e-3;            ///< Finite difference step size
    bool centralDifferences = true;     ///< Use central (vs forward) differences

    // SPSA parameters
    double spsa_a = 0.01;
    double spsa_c = 0.1;
    double spsa_alpha = 0.602;
    double spsa_gamma = 0.101;

    // Adam parameters
    double adam_beta1 = 0.9;
    double adam_beta2 = 0.999;
    double adam_epsilon = 1e-8;

    // CMA-ES parameters
    double cmaes_sigma0 = 0.3;         ///< Initial step size
    int cmaes_populationSize = 0;      ///< 0 = auto (4 + 3*ln(N))

    // CEM parameters
    int cem_populationSize = 100;
    int cem_eliteCount = 10;

    // Evolutionary strategy
    int es_populationSize = 50;
    int es_parentCount = 10;
    double es_mutationRate = 0.1;

    // Coordinate descent
    int cd_sweepsPerIteration = 1;
};

/// Perturbation parameterization config.
struct PerturbationConfig {
    PerturbationType type = PerturbationType::PiecewiseConstant;
    int numSegments = 20;              ///< For piecewise-constant/linear
    int numHarmonics = 5;              ///< For Fourier/spectral (K sinusoids)
    int numImpulses = 5;               ///< For impulse train
    // MLP config
    int mlpHiddenSize = 16;
    int mlpNumLayers = 2;
};

/// Complete Destabilizer run configuration.
struct DestabilizerConfig {
    // System selection
    int systemId = -1;
    // Controller is set separately via the engine

    // Channels
    std::vector<PerturbationChannel> channels;

    // Perturbation parameterization
    PerturbationConfig perturbation;

    // Optimizer
    OptimizerConfig optimizer;

    // Metrics (weighted combination)
    std::vector<WeightedMetric> metrics;

    // Safe-set bounds for escape metrics
    std::vector<SafeSetBound> safeSet;

    // Simulation
    double horizon = 5.0;              ///< Simulation horizon T [s]
    double dt = 0.001;                 ///< Timestep Δt [s]

    // Computation budget
    int maxIterations = 500;
    double maxWallclockSeconds = 300.0;
    int maxFunctionEvaluations = 100000;

    // Early stopping
    double instabilityThreshold = 1e6; ///< Stop when J exceeds this
    bool earlyStop = true;

    // Reproducibility
    uint64_t seed = 42;
    bool deterministicMode = true;

    // Multi-start
    int numStarts = 1;                 ///< Independent searches
    bool warmStart = false;            ///< Warm-start from previous best θ

    // Parallelism
    int numThreads = 0;                ///< 0 = auto (hardware concurrency)

    // Reference state (setpoint)
    std::vector<double> referenceState;

    // Controller saturation limits (for saturation metrics)
    double controlSaturationMin = -1e6;
    double controlSaturationMax = 1e6;
};

// ---------------------------------------------------------------------------
// Result Structures
// ---------------------------------------------------------------------------

/// Per-iteration status during optimization.
struct IterationResult {
    int iteration = 0;
    double bestJ = 0.0;                  ///< Best instability metric so far
    double currentJ = 0.0;               ///< J at this iteration
    double gradientNorm = 0.0;
    double stepSize = 0.0;
    double elapsedSeconds = 0.0;
    std::vector<double> metricBreakdown;  ///< Per-metric contribution
    std::vector<bool> constraintActive;   ///< Which constraints were binding
};

/// Robustness verdict.
enum class Verdict {
    Destabilized,   ///< Controller broke within allowed envelope
    Robust,         ///< No instability found within budget
    Inconclusive    ///< Budget exhausted but borderline result
};

/// Complete result of a Destabilizer run.
struct DestabilizerResult {
    // Verdict
    Verdict verdict = Verdict::Inconclusive;
    int destabilizedAtIteration = -1;

    // Best perturbation found
    std::vector<double> bestTheta;         ///< Best parameter vector
    double bestJ = 0.0;                    ///< Best (highest) instability metric

    // Worst-case perturbation signal: u_p*(t)
    std::vector<double> worstCaseTimes;
    std::vector<std::vector<double>> worstCasePerturbation;  ///< Per-channel

    // Worst-case trajectory
    std::vector<double> trajectoryTimes;
    std::vector<std::vector<double>> trajectoryStates;
    std::vector<std::vector<double>> trajectoryOutputs;
    std::vector<std::vector<double>> trajectoryControlInputs;

    // Per-metric breakdown
    std::vector<double> metricValues;      ///< Final per-metric values

    // Convergence history
    std::vector<IterationResult> history;

    // Multi-start results (if numStarts > 1)
    std::vector<double> multiStartBestJ;   ///< Best J from each start

    // Timing
    double totalSeconds = 0.0;
    int totalFunctionEvaluations = 0;
    int totalIterations = 0;

    // Config snapshot for reproducibility
    uint64_t configHash = 0;
    uint64_t seed = 0;
};

// ---------------------------------------------------------------------------
// Progress Callback
// ---------------------------------------------------------------------------

/// Callback for reporting progress to the UI.
using ProgressCallback = std::function<void(const IterationResult&)>;

/// Callback to check if the run should be aborted.
using AbortCheck = std::function<bool()>;

} // namespace Destabilizer
