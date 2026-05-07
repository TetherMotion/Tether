/**
 * @file IndustrialAutotuners.hpp
 * @brief Industrial-Grade Autotuning Systems
 * 
 * @details
 * Complete autotuning systems suitable for industrial deployment.
 * 
 * ## Implemented Systems
 * 
 * ### Relay Feedback Autotuner
 * Industry standard for PID tuning
 * 
 * ### Step Response Autotuner  
 * Model extraction from step test
 * 
 * ### Pattern Recognition Autotuner
 * Rule-based from transient patterns
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include "ClassicalTuningMethods.hpp"
#include <deque>

namespace Control {
namespace Autotuning {

// ============================================================================
// Relay Feedback Autotuner
// ============================================================================

/**
 * @brief Industrial Relay Feedback Autotuner
 * 
 * Complete relay feedback autotuning system including:
 * - Automatic setpoint biasing
 * - Multiple relay types (standard, with hysteresis, asymmetric)
 * - Oscillation analysis
 * - Multiple tuning rule options
 * 
 * This is the most common industrial autotuning approach.
 */
class RelayFeedbackAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Relay Feedback Autotuner"; }
    std::string getDescription() const override {
        return "Industry-standard relay feedback autotuning. Determines "
               "ultimate gain and period from controlled oscillation.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Relay types
     */
    enum class RelayType {
        Standard,       ///< Symmetric on/off
        Hysteresis,     ///< With deadband
        Asymmetric,     ///< Different up/down amplitudes
        Integrating     ///< Biased for integrating processes
    };
    void setRelayType(RelayType type) { m_relayType = type; }
    
    /**
     * @brief Set relay amplitude
     */
    void setAmplitude(double d) { m_amplitude = d; }
    
    /**
     * @brief Set asymmetric amplitudes
     */
    void setAsymmetricAmplitudes(double dUp, double dDown);
    
    /**
     * @brief Set hysteresis width
     */
    void setHysteresis(double h) { m_hysteresis = h; }
    
    /**
     * @brief Set minimum cycles for identification
     */
    void setMinCycles(int n) { m_minCycles = n; }
    
    /**
     * @brief Set tuning rule
     */
    enum class TuningRule {
        ZieglerNichols,
        TyreusLuyben,
        SomeTimes,      ///< Some-times oscillatory
        NoOvershoot,    ///< Critically damped
        IMC_Aggressive,
        IMC_Moderate,
        IMC_Conservative
    };
    void setTuningRule(TuningRule rule) { m_rule = rule; }
    
    /**
     * @brief Get oscillation characteristics
     */
    struct OscillationData {
        double amplitude{0.0};     ///< Peak-to-peak amplitude
        double period{0.0};        ///< Period [s]
        double ultimateGain{0.0};  ///< Ku
        double ultimatePeriod{0.0}; ///< Tu
        int cycles{0};             ///< Number of complete cycles
        bool stable{false};        ///< Stable oscillation achieved
    };
    OscillationData getOscillationData() const { return m_oscData; }
    
    /**
     * @brief Enable automatic setpoint bias
     * Keeps oscillation around operating point
     */
    void enableSetpointBias(bool enable) { m_autoSetpoint = enable; }
    
    /**
     * @brief Set convergence tolerance
     */
    void setTolerance(double tol) { m_tolerance = tol; }
    
private:
    // Configuration
    RelayType m_relayType{RelayType::Hysteresis};
    double m_amplitude{1.0};
    double m_amplitudeUp{1.0}, m_amplitudeDown{1.0};
    double m_hysteresis{0.0};
    int m_minCycles{3};
    TuningRule m_rule{TuningRule::ZieglerNichols};
    bool m_autoSetpoint{true};
    double m_tolerance{0.05};
    
    // State
    enum class Phase {
        Initialize,
        Oscillating,
        Analyzing,
        Complete
    };
    Phase m_phase{Phase::Initialize};
    bool m_relayHigh{true};
    
    // Data collection
    std::deque<double> m_peaks;
    std::deque<double> m_valleys;
    std::deque<double> m_peakTimes;
    std::deque<double> m_valleyTimes;
    
    double m_time{0.0};
    double m_lastMeasured{0.0};
    double m_lastSlope{0.0};
    
    OscillationData m_oscData;
    
    // Methods
    double computeRelayOutput(double error);
    void detectPeaks(double measured, double dt);
    void analyzeOscillation();
    PIDGains computeGains();
};

// ============================================================================
// Step Response Autotuner
// ============================================================================

/**
 * @brief Step Response Autotuner
 * 
 * Identifies process model from open-loop step response.
 * 
 * ## Identification Methods
 * - Graphical (tangent method)
 * - Area method
 * - Optimization-based
 */
class StepResponseAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Step Response Autotuner"; }
    std::string getDescription() const override {
        return "Model identification from step test. Extracts FOPDT "
               "parameters and designs controller.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set step size
     */
    void setStepSize(double step) { m_stepSize = step; }
    
    /**
     * @brief Set identification method
     */
    enum class IdMethod {
        Tangent,        ///< Maximum slope tangent
        Area,           ///< Area-based
        TwoPoint,       ///< 28-63% or 35-85%
        Optimization    ///< Least squares fit
    };
    void setIdMethod(IdMethod method) { m_idMethod = method; }
    
    /**
     * @brief Set test duration (auto if 0)
     */
    void setTestDuration(double duration) { m_testDuration = duration; }
    
    /**
     * @brief Set tuning method after identification
     */
    void setTuningMethod(std::unique_ptr<OfflineAutotuner> tuner);
    
    /**
     * @brief Get identified model
     */
    FOPDTModel getModel() const { return m_model; }
    
    /**
     * @brief Get fit quality
     */
    double getFitQuality() const { return m_fitQuality; }
    
private:
    double m_stepSize{1.0};
    IdMethod m_idMethod{IdMethod::TwoPoint};
    double m_testDuration{0.0};  // Auto
    
    std::unique_ptr<OfflineAutotuner> m_tuner;
    
    // State
    enum class Phase {
        Steady,
        Stepping,
        Settling,
        Identifying,
        Complete
    };
    Phase m_phase{Phase::Steady};
    
    // Data
    std::vector<double> m_timeData;
    std::vector<double> m_responseData;
    double m_initialValue{0.0};
    double m_finalValue{0.0};
    double m_time{0.0};
    double m_stepTime{0.0};
    
    FOPDTModel m_model;
    double m_fitQuality{0.0};
    
    // Methods
    void identifyTangent();
    void identifyArea();
    void identifyTwoPoint();
    void identifyOptimization();
};

// ============================================================================
// Pattern Recognition Autotuner
// ============================================================================

/**
 * @brief Pattern Recognition Autotuner
 * 
 * Analyzes closed-loop transient response patterns and
 * adjusts gains based on recognized patterns.
 * 
 * ## Recognized Patterns
 * - Oscillatory (reduce gain)
 * - Sluggish (increase gain)
 * - Overshoot too high (adjust ratio)
 * - Good response (done)
 */
class PatternRecognitionAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Pattern Recognition Autotuner"; }
    std::string getDescription() const override {
        return "Rule-based tuning from response patterns. Automatically "
               "classifies response and adjusts gains.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Response pattern categories
     */
    enum class Pattern {
        Unknown,
        Oscillatory,    ///< Too much overshoot/oscillation
        Sluggish,       ///< Too slow response
        Underdamped,    ///< Acceptable but oscillatory
        Overdamped,     ///< Acceptable but slow
        Good,           ///< Within specs
        Unstable        ///< Growing oscillation
    };
    
    /**
     * @brief Set target specifications
     */
    struct Specifications {
        double maxOvershoot{0.1};     ///< Max overshoot (fraction)
        double maxSettlingTime{10.0}; ///< Max settling time [s]
        double maxRiseTime{2.0};      ///< Max rise time [s]
        double settleBand{0.02};      ///< Settling band (±fraction)
    };
    void setSpecifications(const Specifications& specs) { m_specs = specs; }
    
    /**
     * @brief Set gain adjustment factors
     */
    void setAdjustmentFactors(double kpFactor, double tiFactor, double tdFactor);
    
    /**
     * @brief Set maximum iterations
     */
    void setMaxIterations(int n) { m_maxIterations = n; }
    
    /**
     * @brief Get current detected pattern
     */
    Pattern getCurrentPattern() const { return m_currentPattern; }
    
    /**
     * @brief Get response metrics
     */
    struct ResponseMetrics {
        double riseTime{0.0};
        double settlingTime{0.0};
        double overshoot{0.0};
        double undershoot{0.0};
        int oscillations{0};
        double steadyStateError{0.0};
    };
    ResponseMetrics getMetrics() const { return m_metrics; }
    
private:
    Specifications m_specs;
    double m_kpFactor{1.5}, m_tiFactor{1.3}, m_tdFactor{1.2};
    int m_maxIterations{10};
    
    // State
    enum class Phase {
        WaitingForStep,
        Analyzing,
        Adjusting,
        Complete
    };
    Phase m_phase{Phase::WaitingForStep};
    int m_iteration{0};
    
    // Data collection
    std::deque<double> m_errorHistory;
    std::deque<double> m_refHistory;
    double m_stepStartTime{0.0};
    double m_stepMagnitude{0.0};
    double m_time{0.0};
    
    Pattern m_currentPattern{Pattern::Unknown};
    ResponseMetrics m_metrics;
    
    PIDGains m_currentGains;
    
    // Methods
    bool detectStepChange(double reference);
    void analyzeResponse();
    Pattern classifyPattern();
    void adjustGains();
};

// ============================================================================
// Bump Test Autotuner
// ============================================================================

/**
 * @brief Bump Test Autotuner
 * 
 * Manual-to-auto bumpless transfer with model identification.
 * Uses operator-induced bumps for process identification.
 */
class BumpTestAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Bump Test Autotuner"; }
    std::string getDescription() const override {
        return "Model identification from manual bumps. Uses operator "
               "actions for non-intrusive identification.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Report manual control change
     */
    void reportManualChange(double oldOutput, double newOutput);
    
    /**
     * @brief Set minimum bump size for identification
     */
    void setMinBumpSize(double size) { m_minBumpSize = size; }
    
    /**
     * @brief Set number of bumps required
     */
    void setBumpsRequired(int n) { m_bumpsRequired = n; }
    
    /**
     * @brief Get identification confidence
     */
    double getConfidence() const { return m_confidence; }
    
private:
    double m_minBumpSize{5.0};  // % change
    int m_bumpsRequired{2};
    
    // Bump data
    struct BumpData {
        double outputChange;
        double measuredBefore;
        double measuredAfter;
        double responseTime;
        FOPDTModel model;
    };
    std::vector<BumpData> m_bumps;
    
    double m_confidence{0.0};
    bool m_waitingForResponse{false};
    double m_time{0.0};
    
    FOPDTModel m_averageModel;
};

// ============================================================================
// Scheduled Autotuner
// ============================================================================

/**
 * @brief Scheduled/Periodic Autotuner
 * 
 * Automatically initiates tuning at specified intervals
 * or when performance degrades.
 */
class ScheduledAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Scheduled Autotuner"; }
    std::string getDescription() const override {
        return "Automatic periodic or triggered retuning. Maintains "
               "optimal performance as process changes.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }  // Always monitoring
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set tuning interval
     */
    void setTuningInterval(double hours) { m_intervalHours = hours; }
    
    /**
     * @brief Set performance degradation threshold
     */
    void setPerformanceThreshold(double threshold) { m_perfThreshold = threshold; }
    
    /**
     * @brief Set underlying autotuner
     */
    void setAutotuner(std::unique_ptr<OnlineAutotuner> tuner);
    
    /**
     * @brief Force immediate retuning
     */
    void triggerRetune() { m_forceTune = true; }
    
    /**
     * @brief Get time since last tune
     */
    double getTimeSinceLastTune() const;
    
    /**
     * @brief Get current performance metric
     */
    double getCurrentPerformance() const { return m_currentPerf; }
    
private:
    double m_intervalHours{24.0};
    double m_perfThreshold{1.5};  // 50% degradation
    
    std::unique_ptr<OnlineAutotuner> m_tuner;
    
    double m_lastTuneTime{0.0};
    double m_baselinePerf{0.0};
    double m_currentPerf{0.0};
    double m_time{0.0};
    bool m_forceTune{false};
    
    // Performance monitoring
    std::deque<double> m_errorBuffer;
    double m_errorSum{0.0};
};

// ============================================================================
// Safety-Critical Autotuner
// ============================================================================

/**
 * @brief Safety-Critical Autotuner
 * 
 * Autotuning with safety constraints and limits.
 * Suitable for critical processes.
 */
class SafetyAutotuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Safety-Critical Autotuner"; }
    std::string getDescription() const override {
        return "Autotuning with safety constraints. Limits excursions "
               "and prevents dangerous oscillations.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set process variable limits
     */
    void setPVLimits(double low, double high);
    
    /**
     * @brief Set control output limits
     */
    void setOutputLimits(double low, double high);
    
    /**
     * @brief Set rate of change limit
     */
    void setRateLimit(double maxRate);
    
    /**
     * @brief Set maximum oscillation amplitude
     */
    void setMaxOscillation(double amplitude) { m_maxOsc = amplitude; }
    
    /**
     * @brief Set underlying autotuner
     */
    void setAutotuner(std::unique_ptr<OnlineAutotuner> tuner);
    
    /**
     * @brief Check if safety limits were hit
     */
    bool wereLimitsHit() const { return m_limitsHit; }
    
    /**
     * @brief Abort tuning
     */
    void abort() { m_aborted = true; }
    
private:
    double m_pvLow{0.0}, m_pvHigh{100.0};
    double m_outLow{0.0}, m_outHigh{100.0};
    double m_maxRate{10.0};  // %/s
    double m_maxOsc{10.0};
    
    std::unique_ptr<OnlineAutotuner> m_tuner;
    
    bool m_limitsHit{false};
    bool m_aborted{false};
    double m_lastMeasured{0.0};
    double m_lastOutput{0.0};
    
    double checkSafetyLimits(double output, double measured, double dt);
};

// ============================================================================
// Auto-Select Autotuner
// ============================================================================

/**
 * @brief Auto-Select Autotuner
 * 
 * Automatically selects best tuning method based on
 * process characteristics.
 */
class AutoSelectTuner : public OnlineAutotuner {
public:
    std::string getName() const override { return "Auto-Select Autotuner"; }
    std::string getDescription() const override {
        return "Automatic method selection based on process type. "
               "Analyzes process and chooses optimal approach.";
    }
    bool isCompatible(const TunableController& controller) const override { return true; }
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Process categories
     */
    enum class ProcessCategory {
        Unknown,
        FastStable,      ///< Fast, well-damped
        SlowStable,      ///< Slow, well-damped
        Integrating,     ///< Integrating (level, etc)
        Oscillatory,     ///< Poorly damped
        DelayDominant,   ///< Large dead time
        Nonlinear        ///< Significant nonlinearity
    };
    
    /**
     * @brief Get detected process category
     */
    ProcessCategory getProcessCategory() const { return m_category; }
    
    /**
     * @brief Get selected method name
     */
    std::string getSelectedMethod() const;
    
    /**
     * @brief Override auto selection
     */
    void forceMethod(const std::string& methodName);
    
private:
    ProcessCategory m_category{ProcessCategory::Unknown};
    
    // Analysis data
    std::deque<double> m_measurementBuffer;
    std::deque<double> m_controlBuffer;
    double m_time{0.0};
    
    // Selected tuner
    std::unique_ptr<OnlineAutotuner> m_selectedTuner;
    std::string m_methodName;
    
    ProcessCategory analyzeProcess();
    void selectMethod();
};

} // namespace Autotuning
} // namespace Control
