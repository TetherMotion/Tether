/**
 * @file SystemIdentifier.hpp
 * @brief Automatic system identification: delay, friction, PID tuning
 * 
 * This module provides tools for identifying key system parameters:
 * - System delay (transport delay between command and response)
 * - Friction models (Coulomb, viscous, Stribeck)
 * - PID controller tuning analysis and suggestions
 */

#pragma once

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

namespace MotionReplanner {

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief Sample with timing for identification
 */
struct IdentificationSample {
    double timestamp;        ///< Time in seconds
    double commanded;        ///< Commanded value
    double actual;           ///< Actual measured value
    double velocity;         ///< Velocity (if available)
    double acceleration;     ///< Acceleration (if available)
    double current;          ///< Motor current (if available)
    double torque;           ///< Torque estimate (if available)
};

/**
 * @brief System delay identification result
 */
struct DelayIdentificationResult {
    double transportDelay;              ///< Pure time delay (seconds)
    double delayConfidence;             ///< Confidence in estimate (0-1)
    double crossCorrelation;            ///< Peak cross-correlation value
    int crossCorrelationLag;            ///< Lag at peak correlation (samples)
    double samplingPeriod;              ///< Estimated sampling period
    
    // Statistical measures
    double delayStdDev;                 ///< Standard deviation across tests
    double delayMin;                    ///< Minimum detected delay
    double delayMax;                    ///< Maximum detected delay
    
    // Step response characteristics
    double riseTime;                    ///< 10-90% rise time
    double settlingTime;                ///< 2% settling time
    double overshoot;                   ///< Percent overshoot
    
    bool isValid() const { return delayConfidence > 0.5; }
};

/**
 * @brief Friction model types
 */
enum class FrictionModelType {
    Coulomb,            ///< Simple Coulomb (constant)
    Viscous,            ///< Linear viscous
    CoulombViscous,     ///< Combined Coulomb + viscous
    Stribeck,           ///< Full Stribeck model
    LuGre               ///< LuGre dynamic model
};

/**
 * @brief Friction model parameters
 */
struct FrictionModelParams {
    FrictionModelType type;
    
    // Coulomb parameters
    double coulombForce = 0;           ///< Static/kinetic Coulomb friction
    double staticFriction = 0;         ///< Breakaway friction
    
    // Viscous parameters
    double viscousCoeff = 0;           ///< Viscous friction coefficient
    
    // Stribeck parameters
    double stribeckVelocity = 0;       ///< Stribeck velocity
    double stribeckExponent = 2.0;     ///< Stribeck exponent
    
    // LuGre parameters
    double sigma0 = 0;                 ///< Bristle stiffness
    double sigma1 = 0;                 ///< Bristle damping
    double sigma2 = 0;                 ///< Viscous damping
    
    // Fit quality
    double rSquared = 0;               ///< R² fit quality
    double residualRMS = 0;            ///< RMS residual
    
    /**
     * @brief Calculate friction force for given velocity
     * @param velocity Signed velocity at which to evaluate the model [units/s]
     * @return Friction force predicted by this model [N or Nm, matching the fitted data]
     */
    double calculate(double velocity) const;

    /**
     * @brief Get model name
     * @return Human-readable name of `type` (e.g. "Stribeck"), for reporting
     */
    std::string modelName() const;
};

/**
 * @brief Friction identification result
 */
struct FrictionIdentificationResult {
    FrictionModelParams bestModel;      ///< Best fitting model
    std::vector<FrictionModelParams> allModels;  ///< All tried models
    
    // Per-direction results
    FrictionModelParams positiveDirection;
    FrictionModelParams negativeDirection;
    bool isSymmetric;                   ///< Similar in both directions
    double asymmetryRatio;              ///< Ratio between directions
    
    // Velocity-force curve
    std::vector<double> velocities;
    std::vector<double> forces;
    std::vector<double> fittedForces;
    
    bool isValid() const { return bestModel.rSquared > 0.7; }
};

/**
 * @brief PID tuning quality assessment
 */
struct PIDTuningAssessment {
    // Current gains (observed)
    double observedKp = 0;
    double observedKi = 0;
    double observedKd = 0;
    
    // Performance metrics
    double steadyStateError;            ///< Steady state error
    double riseTime;                    ///< 10-90% rise time
    double settlingTime;                ///< 2% settling time
    double overshoot;                   ///< Percent overshoot
    double dampingRatio;                ///< Estimated damping ratio
    double naturalFrequency;            ///< Estimated natural frequency
    
    // Quality scores (0-100)
    double stabilityScore;              ///< Stability margin
    double responseScore;               ///< Response speed quality
    double accuracyScore;               ///< Tracking accuracy
    double overallScore;                ///< Combined score
    
    // Suggested improvements
    double suggestedKp = 0;
    double suggestedKi = 0;
    double suggestedKd = 0;
    std::string tuningAdvice;
    
    // Issues detected
    std::vector<std::string> issues;
    std::vector<std::string> recommendations;
};

/**
 * @brief System dynamics identification result
 */
struct DynamicsIdentificationResult {
    // First-order model: G(s) = K / (Ts + 1)
    double gain;                        ///< DC gain K
    double timeConstant;                ///< Time constant T
    
    // Second-order model: G(s) = K*wn² / (s² + 2*zeta*wn*s + wn²)
    double naturalFrequency;            ///< Natural frequency wn
    double dampingRatio;                ///< Damping ratio zeta
    
    // Higher order approximation
    int systemOrder;                    ///< Estimated system order
    std::vector<double> poles;          ///< Identified poles
    std::vector<double> zeros;          ///< Identified zeros
    
    // Frequency response
    std::vector<double> frequencies;
    std::vector<double> magnitudeDb;
    std::vector<double> phaseDeg;
    
    double bandwidthHz;                 ///< -3dB bandwidth
    double phaseMarginDeg;              ///< Phase margin
    double gainMarginDb;                ///< Gain margin
    
    bool isValid() const { return systemOrder > 0; }
};

//=============================================================================
// System Identifier Class
//=============================================================================

/**
 * @brief Configuration for system identification
 */
struct IdentificationConfig {
    // Delay identification
    double maxDelay = 0.1;              ///< Maximum expected delay (seconds)
    int correlationWindow = 1000;       ///< Window size for correlation
    double stepThreshold = 0.1;         ///< Threshold for step detection
    
    // Friction identification
    double velocityRangeMin = 0.1;      ///< Min velocity for friction ID
    double velocityRangeMax = 1000;     ///< Max velocity for friction ID
    int velocityBins = 50;              ///< Number of velocity bins
    double steadyStateWindow = 0.5;     ///< Window for steady-state detection
    
    // PID analysis
    double settlingThreshold = 0.02;    ///< 2% settling threshold
    double riseTimeStart = 0.1;         ///< Rise time start (10%)
    double riseTimeEnd = 0.9;           ///< Rise time end (90%)
    
    // Frequency analysis
    double fftMinFreq = 0.1;            ///< Min frequency for analysis
    double fftMaxFreq = 1000;           ///< Max frequency for analysis
    int fftSize = 4096;                 ///< FFT size
};

/**
 * @brief Main system identification class
 */
class SystemIdentifier {
public:
    explicit SystemIdentifier(const IdentificationConfig& config = {});
    
    //-------------------------------------------------------------------------
    // Data Input
    //-------------------------------------------------------------------------
    
    /**
     * @brief Add identification sample
     * @param sample Sample to append; only fields relevant to the planned
     *        analysis need to be populated (e.g. delay ID needs `timestamp`,
     *        `commanded`, `actual`; friction ID also uses `velocity` and
     *        `torque`/`current`)
     */
    void addSample(const IdentificationSample& sample);

    /**
     * @brief Add batch of samples
     * @param samples Samples to append in order; equivalent to calling
     *        addSample() on each element
     */
    void addSamples(const std::vector<IdentificationSample>& samples);

    /**
     * @brief Clear all samples
     */
    void clearSamples();

    /**
     * @brief Get sample count
     * @return Number of samples currently buffered for identification
     */
    size_t sampleCount() const { return samples_.size(); }
    
    //-------------------------------------------------------------------------
    // Delay Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify system transport delay
     * Uses cross-correlation between commanded and actual signals
     * @return Delay estimate; check `isValid()` (confidence &gt; 0.5) before
     *         trusting `transportDelay`
     */
    DelayIdentificationResult identifyDelay();

    /**
     * @brief Identify delay from step response
     * @param stepTime Timestamp of the applied step [s]
     * @param stepMagnitude Signed amplitude of the applied step, in the
     *        command units
     * @return Delay estimate derived from the step's response onset; check
     *         `isValid()` before use
     */
    DelayIdentificationResult identifyDelayFromStep(
        double stepTime, double stepMagnitude);
    
    //-------------------------------------------------------------------------
    // Friction Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify friction model from constant velocity data
     * @return Result with the best-fitting model and all tried candidates;
     *         check `isValid()` (R² &gt; 0.7) before applying `bestModel`
     */
    FrictionIdentificationResult identifyFriction();

    /**
     * @brief Identify friction using velocity reversal method
     * @return Result comparing positive- and negative-direction fits;
     *         `isSymmetric`/`asymmetryRatio` indicate direction-dependent friction
     */
    FrictionIdentificationResult identifyFrictionFromReversals();

    /**
     * @brief Fit specific friction model to data
     * @param type Model form to fit (Coulomb, viscous, Stribeck, ...)
     * @param velocities Measured velocities; must match `forces` element-wise
     * @param forces Measured friction forces at `velocities`
     * @return Fitted parameters with `rSquared`/`residualRMS` fit quality
     */
    FrictionModelParams fitFrictionModel(
        FrictionModelType type,
        const std::vector<double>& velocities,
        const std::vector<double>& forces);
    
    //-------------------------------------------------------------------------
    // PID Analysis
    //-------------------------------------------------------------------------
    
    /**
     * @brief Analyze PID tuning from step response
     * @return Assessment with observed gains (if inferrable), performance
     *         metrics, 0-100 quality scores, and detected issues
     */
    PIDTuningAssessment analyzePIDTuning();

    /**
     * @brief Analyze tracking performance
     * @return Assessment focused on tracking accuracy over the buffered
     *         trajectory rather than a single step
     */
    PIDTuningAssessment analyzeTracking();

    /**
     * @brief Suggest PID gains using Ziegler-Nichols rules
     * @param[in,out] assessment Assessment to fill: `suggestedKp/Ki/Kd` and
     *        `tuningAdvice` are overwritten based on the identified dynamics
     */
    void suggestZieglerNichols(PIDTuningAssessment& assessment);

    /**
     * @brief Suggest PID gains using SIMC rules
     * @param[in,out] assessment Assessment to fill: `suggestedKp/Ki/Kd` and
     *        `tuningAdvice` are overwritten based on the identified dynamics
     */
    void suggestSIMC(PIDTuningAssessment& assessment);
    
    //-------------------------------------------------------------------------
    // Dynamics Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify system dynamics from step response
     * @return First/second-order model parameters; check `isValid()`
     *         (`systemOrder` &gt; 0) before using the fitted values
     */
    DynamicsIdentificationResult identifyDynamics();

    /**
     * @brief Perform frequency response analysis
     * @return Result with `frequencies`/`magnitudeDb`/`phaseDeg` Bode data
     *         plus bandwidth and stability margins
     */
    DynamicsIdentificationResult performFrequencyAnalysis();
    
    //-------------------------------------------------------------------------
    // Utility Functions
    //-------------------------------------------------------------------------
    
    /**
     * @brief Compute cross-correlation between two signals
     * @param signal1 First signal (typically the commanded value)
     * @param signal2 Second signal (typically the actual value)
     * @param maxLag Maximum lag to evaluate, in samples; result covers
     *        lags `-maxLag` .. `+maxLag`
     * @return Correlation values indexed by lag, where index `maxLag`
     *         corresponds to lag 0; positive lags mean `signal2` trails `signal1`
     */
    static std::vector<double> crossCorrelation(
        const std::vector<double>& signal1,
        const std::vector<double>& signal2,
        int maxLag);

    /**
     * @brief Find step transitions in signal
     * @param threshold Minimum change magnitude to count as a step
     * @return (sample index, step magnitude) pairs in chronological order
     */
    std::vector<std::pair<size_t, double>> findSteps(double threshold);

    /**
     * @brief Compute FFT of signal
     * @param[in] signal Time-domain samples
     * @param[out] frequencies Bin center frequencies [Hz], size fftSize/2
     * @param[out] magnitudes Per-bin magnitudes, same indexing as `frequencies`
     * @param[out] phases Per-bin phases [deg], same indexing as `frequencies`
     * @param sampleRate Sampling rate of `signal` [Hz]
     */
    static void computeFFT(
        const std::vector<double>& signal,
        std::vector<double>& frequencies,
        std::vector<double>& magnitudes,
        std::vector<double>& phases,
        double sampleRate);
    
private:
    IdentificationConfig config_;
    std::vector<IdentificationSample> samples_;
    
    // Internal methods
    double computeSamplingRate() const;
    void extractStepResponse(size_t stepIndex, double stepMagnitude,
                            std::vector<double>& time,
                            std::vector<double>& response);
    
    std::pair<double, double> fitFirstOrder(
        const std::vector<double>& time,
        const std::vector<double>& response,
        double finalValue);
    
    std::tuple<double, double, double> fitSecondOrder(
        const std::vector<double>& time,
        const std::vector<double>& response,
        double finalValue);
};

//=============================================================================
// Friction Model Calculator
//=============================================================================

/**
 * @brief Calculate Stribeck friction curve
 */
class StribeckCalculator {
public:
    /**
     * @brief Full Stribeck friction model
     * F = Fc + (Fs - Fc)*exp(-(v/vs)^delta) + sigma_v * v
     * @param velocity Signed velocity at which to evaluate [units/s]
     * @param coulomb Kinetic (Coulomb) friction level Fc
     * @param staticFriction Breakaway friction level Fs
     * @param stribeckVelocity Stribeck characteristic velocity vs
     * @param stribeckExponent Stribeck shape exponent delta (typically 2)
     * @param viscousCoeff Linear viscous coefficient sigma_v
     * @return Friction force at `velocity`, in the same force unit as the inputs
     */
    static double calculate(double velocity,
                           double coulomb, double staticFriction,
                           double stribeckVelocity, double stribeckExponent,
                           double viscousCoeff);

    /**
     * @brief Fit Stribeck model to velocity-force data
     * @param velocities Measured velocities; must match `forces` element-wise
     * @param forces Measured friction forces at `velocities`
     * @return Fitted `FrictionModelParams` with `type` = Stribeck and fit
     *         quality in `rSquared`/`residualRMS`
     */
    static FrictionModelParams fit(const std::vector<double>& velocities,
                                   const std::vector<double>& forces);
};

//=============================================================================
// Real-time Delay Estimator
//=============================================================================

/**
 * @brief Online delay estimation using recursive least squares
 */
class OnlineDelayEstimator {
public:
    explicit OnlineDelayEstimator(double maxDelay, double samplePeriod);
    
    /**
     * @brief Update estimate with new sample
     * @param commanded Commanded value at `timestamp`
     * @param actual Measured value at `timestamp`
     * @param timestamp Sample time [s]; must be monotonically increasing
     */
    void update(double commanded, double actual, double timestamp);

    /**
     * @brief Get current delay estimate
     * @return Estimated transport delay [s]
     */
    double getDelay() const { return currentDelay_; }

    /**
     * @brief Get delay confidence
     * @return Confidence in the current estimate, 0 (none) to 1 (certain)
     */
    double getConfidence() const { return confidence_; }
    
    /**
     * @brief Reset estimator
     */
    void reset();
    
private:
    double maxDelay_;
    double samplePeriod_;
    double currentDelay_;
    double confidence_;
    
    std::vector<double> commandHistory_;
    std::vector<double> actualHistory_;
    std::vector<double> timeHistory_;
    
    size_t historySize_;
    double forgettingFactor_;
};

//=============================================================================
// Auto-tuner
//=============================================================================

/**
 * @brief Automatic PID tuner using relay feedback
 */
class RelayAutoTuner {
public:
    struct TuningResult {
        double ultimateGain;            ///< Ultimate gain Ku
        double ultimatePeriod;          ///< Ultimate period Pu
        
        // Suggested gains (different rules)
        struct {
            double Kp, Ki, Kd;
        } zieglerNichols;
        
        struct {
            double Kp, Ki, Kd;
        } tyreus;
        
        struct {
            double Kp, Ki, Kd;
        } someOvershoot;
        
        struct {
            double Kp, Ki, Kd;
        } noOvershoot;
        
        int oscillationCount;           ///< Number of oscillations detected
        bool isValid;
    };
    
    RelayAutoTuner(double relayAmplitude, double hysteresis);
    
    /**
     * @brief Process sample during relay test
     * @param setpoint Target value the relay oscillates around
     * @param measurement Current measured process value
     * @param timestamp Sample time [s]; must be monotonically increasing
     * @return Control output (-amplitude or +amplitude) to apply this cycle
     */
    double process(double setpoint, double measurement, double timestamp);

    /**
     * @brief Check if enough oscillations collected
     * @return `true` once at least 3 oscillations were observed, i.e.
     *         computeTuning() has enough data for a meaningful result
     */
    bool isReady() const { return oscillationCount_ >= 3; }

    /**
     * @brief Compute tuning result
     * @return Ultimate gain/period plus suggested gains under the
     *         Ziegler-Nichols, Tyreus-Luyben, some-overshoot and no-overshoot
     *         rules; `isValid` is `false` if oscillation data was insufficient
     */
    TuningResult computeTuning();
    
    /**
     * @brief Reset tuner
     */
    void reset();
    
private:
    double relayAmplitude_;
    double hysteresis_;
    double currentOutput_;
    
    std::vector<double> peaks_;
    std::vector<double> peakTimes_;
    int oscillationCount_;
    double lastCrossing_;
    bool lastAbove_;
};

} // namespace MotionReplanner
