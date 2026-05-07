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
     */
    double calculate(double velocity) const;
    
    /**
     * @brief Get model name
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
     */
    void addSample(const IdentificationSample& sample);
    
    /**
     * @brief Add batch of samples
     */
    void addSamples(const std::vector<IdentificationSample>& samples);
    
    /**
     * @brief Clear all samples
     */
    void clearSamples();
    
    /**
     * @brief Get sample count
     */
    size_t sampleCount() const { return samples_.size(); }
    
    //-------------------------------------------------------------------------
    // Delay Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify system transport delay
     * Uses cross-correlation between commanded and actual signals
     */
    DelayIdentificationResult identifyDelay();
    
    /**
     * @brief Identify delay from step response
     */
    DelayIdentificationResult identifyDelayFromStep(
        double stepTime, double stepMagnitude);
    
    //-------------------------------------------------------------------------
    // Friction Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify friction model from constant velocity data
     */
    FrictionIdentificationResult identifyFriction();
    
    /**
     * @brief Identify friction using velocity reversal method
     */
    FrictionIdentificationResult identifyFrictionFromReversals();
    
    /**
     * @brief Fit specific friction model to data
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
     */
    PIDTuningAssessment analyzePIDTuning();
    
    /**
     * @brief Analyze tracking performance
     */
    PIDTuningAssessment analyzeTracking();
    
    /**
     * @brief Suggest PID gains using Ziegler-Nichols rules
     */
    void suggestZieglerNichols(PIDTuningAssessment& assessment);
    
    /**
     * @brief Suggest PID gains using SIMC rules
     */
    void suggestSIMC(PIDTuningAssessment& assessment);
    
    //-------------------------------------------------------------------------
    // Dynamics Identification
    //-------------------------------------------------------------------------
    
    /**
     * @brief Identify system dynamics from step response
     */
    DynamicsIdentificationResult identifyDynamics();
    
    /**
     * @brief Perform frequency response analysis
     */
    DynamicsIdentificationResult performFrequencyAnalysis();
    
    //-------------------------------------------------------------------------
    // Utility Functions
    //-------------------------------------------------------------------------
    
    /**
     * @brief Compute cross-correlation between two signals
     */
    static std::vector<double> crossCorrelation(
        const std::vector<double>& signal1,
        const std::vector<double>& signal2,
        int maxLag);
    
    /**
     * @brief Find step transitions in signal
     */
    std::vector<std::pair<size_t, double>> findSteps(double threshold);
    
    /**
     * @brief Compute FFT of signal
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
     */
    static double calculate(double velocity, 
                           double coulomb, double staticFriction,
                           double stribeckVelocity, double stribeckExponent,
                           double viscousCoeff);
    
    /**
     * @brief Fit Stribeck model to velocity-force data
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
     */
    void update(double commanded, double actual, double timestamp);
    
    /**
     * @brief Get current delay estimate
     */
    double getDelay() const { return currentDelay_; }
    
    /**
     * @brief Get delay confidence
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
     * @return Control output (-amplitude or +amplitude)
     */
    double process(double setpoint, double measurement, double timestamp);
    
    /**
     * @brief Check if enough oscillations collected
     */
    bool isReady() const { return oscillationCount_ >= 3; }
    
    /**
     * @brief Compute tuning result
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
