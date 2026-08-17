/**
 * @file PIDAutotuning.hpp
 * @brief PID Autotuning Methods and Parameter Estimation
 * 
 * @details
 * Comprehensive collection of PID autotuning algorithms including:
 * 
 * ## Online Methods (Live Data)
 * - Relay Feedback (Åström-Hägglund)
 * - Pattern Search
 * - Extremum Seeking
 * - Model Reference Adaptive Control (MRAC)
 * 
 * ## Offline Methods (Recorded Data)
 * - Process Reaction Curve
 * - Frequency Response Analysis
 * - Step Response Fitting
 * - ARMAX Model Estimation
 * 
 * ## Tuning Rules
 * - Ziegler-Nichols (Open/Closed Loop)
 * - Cohen-Coon
 * - Lambda (IMC)
 * - SIMC (Skogestad IMC)
 * - AMIGO
 * - Tyreus-Luyben
 * - Chien-Hrones-Reswick
 * 
 * @see PIDControllers.hpp
 */

#pragma once

#include "control/ControllerBase.hpp"
#include <vector>
#include <array>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

namespace tether::control {

// ============================================================================
// Process Model Types
// ============================================================================

/**
 * @brief First Order Plus Dead Time (FOPDT) model parameters
 * 
 * G(s) = K * exp(-L*s) / (τ*s + 1)
 */
struct FOPDTModel {
    double K{1.0};      ///< Process gain
    double tau{1.0};    ///< Time constant
    double L{0.0};      ///< Dead time (delay)
    
    bool isValid() const {
        return K != 0 && tau > 0 && L >= 0;
    }
    
    /// Simulate step response at time t
    double stepResponse(double t) const {
        if (t < L) return 0.0;
        return K * (1.0 - std::exp(-(t - L) / tau));
    }
};

/**
 * @brief Second Order Plus Dead Time (SOPDT) model parameters
 */
struct SOPDTModel {
    double K{1.0};      ///< Process gain
    double tau1{1.0};   ///< First time constant
    double tau2{1.0};   ///< Second time constant
    double L{0.0};      ///< Dead time
    double zeta{1.0};   ///< Damping ratio (alternative form)
    
    bool isValid() const {
        return K != 0 && tau1 > 0 && tau2 > 0 && L >= 0;
    }
};

/**
 * @brief Integrating Plus Dead Time (IPDT) model
 * 
 * G(s) = K * exp(-L*s) / s
 */
struct IPDTModel {
    double K{1.0};      ///< Integrator gain
    double L{0.0};      ///< Dead time
    
    double rampResponse(double t) const {
        if (t < L) return 0.0;
        return K * (t - L);
    }
};

// ============================================================================
// Tuning Rule Enumeration
// ============================================================================

/**
 * @brief Available tuning rules/methods
 */
enum class TuningRule {
    // Classic methods
    ZieglerNichols,         ///< Z-N original (aggressive)
    ZieglerNicholsNoOvershoot, ///< Z-N with no overshoot
    ZieglerNicholsSomeOvershoot, ///< Z-N with 20% overshoot
    
    // Process reaction curve methods
    CohenCoon,              ///< Better for dead time
    ChienHronesReswick0,    ///< 0% overshoot setpoint
    ChienHronesReswick20,   ///< 20% overshoot setpoint
    
    // IMC-based methods
    Lambda,                 ///< Internal Model Control
    SIMC,                   ///< Skogestad IMC
    
    // Optimization-based
    AMIGO,                  ///< Approximate M-constrained
    TyreusLuyben,           ///< Conservative, no overshoot
    
    // Special purpose
    ISE,                    ///< Minimize integral squared error
    IAE,                    ///< Minimize integral absolute error
    ITAE                    ///< Minimize integral time-weighted absolute error
};

/**
 * @brief Tuning goal specification
 */
enum class TuningGoal {
    Regulatory,             ///< Disturbance rejection
    Setpoint,               ///< Setpoint tracking
    Balanced                ///< Balance between both
};

/**
 * @brief Controller type to tune
 */
enum class ControllerForm {
    PI,
    PID,
    PIDFiltered             ///< PID with derivative filter
};

// ============================================================================
// PID Gains Structure
// ============================================================================

/**
 * @brief Complete PID parameter set
 */
struct PIDGains {
    double Kp{0.0};         ///< Proportional gain
    double Ki{0.0};         ///< Integral gain
    double Kd{0.0};         ///< Derivative gain
    double Tf{0.0};         ///< Derivative filter time constant
    
    // Alternative representation
    double Ti() const { return (Ki != 0) ? Kp / Ki : 0.0; }  ///< Integral time
    double Td() const { return (Kp != 0) ? Kd / Kp : 0.0; }  ///< Derivative time
    
    void setFromTimeConstants(double kp, double ti, double td) {
        Kp = kp;
        Ki = (ti > 0) ? kp / ti : 0.0;
        Kd = kp * td;
    }
    
    bool isValid() const {
        return std::isfinite(Kp) && std::isfinite(Ki) && std::isfinite(Kd);
    }
};

// ============================================================================
// Tuning Rule Implementation
// ============================================================================

/**
 * @brief Calculate PID gains from FOPDT model using specified tuning rule
 */
class TuningRules {
public:
    /**
     * @brief Calculate gains for FOPDT model
     * @param model Process model
     * @param rule Tuning rule to apply
     * @param form Controller form
     * @param lambda Closed-loop time constant (for Lambda/IMC methods)
     * @return PID gains
     */
    static PIDGains calculate(const FOPDTModel& model, TuningRule rule,
                              ControllerForm form = ControllerForm::PID,
                              double lambda = -1.0);
    
    /**
     * @brief Ziegler-Nichols from critical gain and period
     * @param Ku Ultimate gain
     * @param Tu Ultimate period
     * @param form Controller form
     * @return PID gains
     */
    static PIDGains zieglerNicholsUltimate(double Ku, double Tu, ControllerForm form);
    
    /**
     * @brief Cohen-Coon tuning
     */
    static PIDGains cohenCoon(const FOPDTModel& model, ControllerForm form);
    
    /**
     * @brief Lambda/IMC tuning
     * @param model Process model
     * @param lambdaFactor Ratio of λ to dominant time constant (default 1.0)
     */
    static PIDGains lambdaTuning(const FOPDTModel& model, double lambdaFactor = 1.0);
    
    /**
     * @brief SIMC tuning (Skogestad IMC)
     */
    static PIDGains simcTuning(const FOPDTModel& model, double tauC = -1.0);
    
    /**
     * @brief AMIGO tuning
     */
    static PIDGains amigoTuning(const FOPDTModel& model, ControllerForm form);
    
    /**
     * @brief Tyreus-Luyben (conservative)
     */
    static PIDGains tyreusLuyben(double Ku, double Tu, ControllerForm form);
    
    /**
     * @brief Chien-Hrones-Reswick
     */
    static PIDGains chienHronesReswick(const FOPDTModel& model, 
                                        ControllerForm form,
                                        double overshootPercent = 0.0);
};

// ============================================================================
// Process Identification - Offline Methods
// ============================================================================

/**
 * @brief Process identification from step response data
 */
class ProcessIdentification {
public:
    /**
     * @brief Identify FOPDT model from step response
     * @param time Time vector
     * @param response Response vector
     * @param stepSize Magnitude of input step
     * @return Identified model
     */
    static FOPDTModel identifyFOPDT(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize = 1.0);
    
    /**
     * @brief Two-point method for FOPDT identification
     * Uses 28.3% and 63.2% of final value
     */
    static FOPDTModel twoPointMethod(const std::vector<double>& time,
                                     const std::vector<double>& response,
                                     double stepSize = 1.0);
    
    /**
     * @brief Area method for FOPDT identification
     */
    static FOPDTModel areaMethod(const std::vector<double>& time,
                                 const std::vector<double>& response,
                                 double stepSize = 1.0);
    
    /**
     * @brief Tangent (maximum slope) method
     */
    static FOPDTModel tangentMethod(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize = 1.0);
    
    /**
     * @brief Fit SOPDT model to step response
     */
    static SOPDTModel identifySOPDT(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize = 1.0);
    
    /**
     * @brief Estimate dead time from correlation
     */
    static double estimateDeadTime(const std::vector<double>& input,
                                   const std::vector<double>& output,
                                   double dt);
};

// ============================================================================
// Relay Feedback Autotuning (Online)
// ============================================================================

/**
 * @brief Relay feedback autotuner (Åström-Hägglund method)
 * 
 * Uses relay feedback to estimate critical gain and period,
 * then applies tuning rules.
 */
class RelayAutotuner {
public:
    /**
     * @brief Configuration for relay autotuning
     */
    struct Config {
        double relayAmplitude = 1.0;     ///< Relay output amplitude
        double hysteresis = 0.0;         ///< Relay hysteresis (for noise)
        int minCycles = 3;               ///< Minimum oscillation cycles
        int maxCycles = 20;              ///< Maximum cycles before timeout
        double stabilityMargin = 0.05;   ///< Amplitude variation tolerance
        TuningRule rule = TuningRule::ZieglerNichols;
        ControllerForm form = ControllerForm::PID;
        
        static Config getDefault() {
            return Config{1.0, 0.0, 3, 20, 0.05, TuningRule::ZieglerNichols, ControllerForm::PID};
        }
    };
    
    /**
     * @brief State of autotuning process
     */
    enum class State {
        Idle,
        WaitingForOscillation,
        Measuring,
        Complete,
        Failed
    };
    
    RelayAutotuner() : RelayAutotuner(Config::getDefault()) {}
    RelayAutotuner(const Config& config);
    
    /**
     * @brief Start autotuning
     * @param setpoint Operating point setpoint
     */
    void start(double setpoint);
    
    /**
     * @brief Process one sample
     * @param measured Current measurement
     * @param dt Time step
     * @return Control output (relay output)
     */
    double update(double measured, double dt);
    
    /**
     * @brief Get current state
     */
    State getState() const { return m_state; }
    
    /**
     * @brief Check if complete
     */
    bool isComplete() const { return m_state == State::Complete; }
    
    /**
     * @brief Get results
     */
    struct Results {
        double Ku{0.0};         ///< Ultimate gain
        double Tu{0.0};         ///< Ultimate period
        PIDGains gains;         ///< Calculated gains
        bool valid{false};
    };
    
    Results getResults() const { return m_results; }
    
    /**
     * @brief Reset autotuner
     */
    void reset();
    
private:
    Config m_config;
    State m_state{State::Idle};
    Results m_results;
    
    double m_setpoint{0.0};
    double m_relayOutput{0.0};
    double m_lastError{0.0};
    
    // Oscillation measurement
    std::vector<double> m_peaks;
    std::vector<double> m_valleys;
    std::vector<double> m_periodTimes;
    double m_timeSinceLastCrossing{0.0};
    bool m_lastAboveSetpoint{false};
    int m_cycleCount{0};
};

// ============================================================================
// Pattern Search Autotuning (Online)
// ============================================================================

/**
 * @brief Pattern search (Hooke-Jeeves) autotuning
 * 
 * Searches for optimal gains by perturbing and evaluating
 * performance in real-time.
 */
class PatternSearchAutotuner {
public:
    struct Config {
        PIDGains initialGains;
        double stepSize{0.1};           ///< Initial step as fraction of gain
        double stepReduction{0.5};      ///< Step reduction factor
        double minStep{0.01};           ///< Minimum step size
        double evaluationTime{10.0};    ///< Seconds per evaluation
        int maxIterations{50};
    };
    
    enum class State {
        Idle,
        Evaluating,
        Searching,
        Complete
    };
    
    PatternSearchAutotuner(const Config& config);
    
    void start(double setpoint);
    PIDGains update(double measured, double time);
    State getState() const { return m_state; }
    PIDGains getBestGains() const { return m_bestGains; }
    void reset();
    
private:
    double evaluatePerformance();
    
    Config m_config;
    State m_state{State::Idle};
    PIDGains m_currentGains;
    PIDGains m_bestGains;
    double m_bestCost{std::numeric_limits<double>::max()};
    
    std::vector<double> m_errors;
    double m_evalStartTime{0.0};
    int m_iteration{0};
    int m_paramIndex{0};  // 0=Kp, 1=Ki, 2=Kd
    double m_direction{1.0};
};

// ============================================================================
// Frequency Response Identification
// ============================================================================

/**
 * @brief Frequency response identification for offline tuning
 */
class FrequencyResponseID {
public:
    /**
     * @brief Identify frequency response from sine test data
     * @param time Time vector
     * @param input Sinusoidal input
     * @param output System response
     * @return Magnitude and phase at test frequency
     */
    struct FreqResponse {
        double frequency;
        double magnitude;
        double phase;
    };
    
    static FreqResponse identifySingleFrequency(
        const std::vector<double>& time,
        const std::vector<double>& input,
        const std::vector<double>& output);
    
    /**
     * @brief Identify Bode plot from multi-frequency data
     */
    static std::vector<FreqResponse> identifyBodePlot(
        const std::vector<double>& time,
        const std::vector<double>& input,
        const std::vector<double>& output,
        const std::vector<double>& frequencies);
    
    /**
     * @brief Estimate critical point (Ku, ωu) from frequency response
     */
    static std::pair<double, double> findCriticalPoint(
        const std::vector<FreqResponse>& bode);
    
    /**
     * @brief Fit FOPDT model to frequency response
     */
    static FOPDTModel fitModelToBode(const std::vector<FreqResponse>& bode);
};

// ============================================================================
// ARMAX Model Estimation
// ============================================================================

/**
 * @brief ARMAX model parameter estimation
 * 
 * y(t) = a1*y(t-1) + ... + an*y(t-n) + b1*u(t-d-1) + ... + bm*u(t-d-m) + e(t)
 */
class ARMAXEstimator {
public:
    struct Config {
        int na{2};          ///< Number of autoregressive terms
        int nb{2};          ///< Number of input terms
        int nk{1};          ///< Input delay (dead time in samples)
        double forgetting{0.99}; ///< Forgetting factor for RLS
    };
    
    ARMAXEstimator(const Config& config);
    
    /**
     * @brief Update model with new sample
     */
    void update(double input, double output);
    
    /**
     * @brief Get estimated parameters
     */
    std::vector<double> getARCoeffs() const;
    std::vector<double> getMACoeffs() const;
    
    /**
     * @brief Convert to continuous FOPDT approximation
     */
    FOPDTModel toFOPDT(double dt) const;
    
    /**
     * @brief Calculate PID gains from estimated model
     */
    PIDGains calculateGains(TuningRule rule, double dt) const;
    
private:
    Config m_config;
    std::vector<double> m_theta;  // Parameter vector
    std::vector<std::vector<double>> m_P;  // Covariance matrix
    std::vector<double> m_inputHistory;
    std::vector<double> m_outputHistory;
};

// ============================================================================
// Model Reference Adaptive Control (MRAC) Tuning
// ============================================================================

/**
 * @brief MRAC-based auto-tuning
 */
class MRACAutotuner {
public:
    struct Config {
        double referenceTimeConstant{1.0};  ///< Desired response speed
        double adaptationGain{0.01};
        double signalAmplitude{1.0};        ///< Excitation signal amplitude
    };
    
    MRACAutotuner(const Config& config);
    
    void start();
    PIDGains update(double measured, double control, double dt);
    PIDGains getCurrentGains() const { return m_gains; }
    void reset();
    
private:
    Config m_config;
    PIDGains m_gains;
    
    // Reference model state
    double m_refModelOutput{0.0};
    
    // Adaptation variables
    double m_adaptKp{0.0};
    double m_adaptKi{0.0};
};

// ============================================================================
// Inline Implementations
// ============================================================================

inline PIDGains TuningRules::zieglerNicholsUltimate(double Ku, double Tu, ControllerForm form) {
    PIDGains gains;
    
    switch (form) {
        case ControllerForm::PI:
            gains.Kp = 0.45 * Ku;
            gains.Ki = gains.Kp / (0.83 * Tu);
            gains.Kd = 0.0;
            break;
            
        case ControllerForm::PID:
        case ControllerForm::PIDFiltered:
            gains.Kp = 0.6 * Ku;
            gains.Ki = gains.Kp / (0.5 * Tu);
            gains.Kd = gains.Kp * 0.125 * Tu;
            gains.Tf = gains.Kd / (8.0 * gains.Kp);  // N = 8
            break;
    }
    
    return gains;
}

inline PIDGains TuningRules::cohenCoon(const FOPDTModel& model, ControllerForm form) {
    PIDGains gains;
    
    double r = model.L / model.tau;  // Dead time ratio
    
    switch (form) {
        case ControllerForm::PI:
            gains.Kp = (1.0 / model.K) * (model.tau / model.L) * 
                       (0.9 + r / 12.0);
            gains.Ki = gains.Kp / (model.L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r));
            gains.Kd = 0.0;
            break;
            
        case ControllerForm::PID:
        case ControllerForm::PIDFiltered:
            gains.Kp = (1.0 / model.K) * (model.tau / model.L) * 
                       (4.0 / 3.0 + r / 4.0);
            {
                double Ti = model.L * (32.0 + 6.0 * r) / (13.0 + 8.0 * r);
                double Td = model.L * 4.0 / (11.0 + 2.0 * r);
                gains.Ki = gains.Kp / Ti;
                gains.Kd = gains.Kp * Td;
                gains.Tf = Td / 8.0;
            }
            break;
    }
    
    return gains;
}

inline PIDGains TuningRules::lambdaTuning(const FOPDTModel& model, double lambdaFactor) {
    PIDGains gains;
    
    double lambda = (lambdaFactor > 0) ? lambdaFactor * model.tau 
                                       : std::max(0.25 * model.tau, model.L);
    
    gains.Kp = model.tau / (model.K * (lambda + model.L));
    gains.Ki = gains.Kp / model.tau;
    gains.Kd = 0.0;  // Pure PI for Lambda tuning
    
    return gains;
}

inline PIDGains TuningRules::simcTuning(const FOPDTModel& model, double tauC) {
    PIDGains gains;
    
    // Default: τc = max(τ/5, 0.2*L)
    if (tauC < 0) {
        tauC = std::max(model.tau / 5.0, 0.2 * model.L);
    }
    
    gains.Kp = model.tau / (model.K * (tauC + model.L));
    gains.Ki = gains.Kp / std::min(model.tau, 4.0 * (tauC + model.L));
    gains.Kd = gains.Kp * std::min(model.tau / 2.0, model.L);
    gains.Tf = gains.Kd / (10.0 * gains.Kp);
    
    return gains;
}

inline PIDGains TuningRules::amigoTuning(const FOPDTModel& model, ControllerForm form) {
    PIDGains gains;
    
    double r = model.L / model.tau;
    
    switch (form) {
        case ControllerForm::PI:
            gains.Kp = (0.15 / model.K) + (0.35 - model.L * model.tau / 
                       std::pow(model.L + model.tau, 2)) * model.tau / (model.K * model.L);
            gains.Ki = gains.Kp / (0.35 * model.L + 13.0 * model.L * model.tau * model.tau / 
                       std::pow(model.tau + model.L, 2));
            gains.Kd = 0.0;
            break;
            
        case ControllerForm::PID:
        case ControllerForm::PIDFiltered:
            gains.Kp = (1.0 / model.K) * (0.2 + 0.45 * model.tau / model.L);
            {
                double Ti = (0.4 * model.L + 0.8 * model.tau) * model.L / 
                            (model.L + 0.1 * model.tau);
                double Td = 0.5 * model.L * model.tau / (0.3 * model.L + model.tau);
                gains.Ki = gains.Kp / Ti;
                gains.Kd = gains.Kp * Td;
                gains.Tf = Td / 10.0;
            }
            break;
    }
    
    return gains;
}

inline PIDGains TuningRules::tyreusLuyben(double Ku, double Tu, ControllerForm form) {
    PIDGains gains;
    
    switch (form) {
        case ControllerForm::PI:
            gains.Kp = Ku / 3.2;
            gains.Ki = gains.Kp / (2.2 * Tu);
            gains.Kd = 0.0;
            break;
            
        case ControllerForm::PID:
        case ControllerForm::PIDFiltered:
            gains.Kp = Ku / 2.2;
            gains.Ki = gains.Kp / (2.2 * Tu);
            gains.Kd = gains.Kp * Tu / 6.3;
            gains.Tf = gains.Kd / (8.0 * gains.Kp);
            break;
    }
    
    return gains;
}

inline PIDGains TuningRules::chienHronesReswick(const FOPDTModel& model, 
                                                ControllerForm form,
                                                double overshootPercent) {
    PIDGains gains;
    
    // Coefficients depend on overshoot requirement
    double kpCoeff, tiCoeff, tdCoeff;
    
    if (overshootPercent <= 0.01) {  // 0% overshoot
        if (form == ControllerForm::PI) {
            kpCoeff = 0.35;
            tiCoeff = 1.2;
            tdCoeff = 0.0;
        } else {
            kpCoeff = 0.6;
            tiCoeff = 1.0;
            tdCoeff = 0.5;
        }
    } else {  // 20% overshoot
        if (form == ControllerForm::PI) {
            kpCoeff = 0.6;
            tiCoeff = 1.0;
            tdCoeff = 0.0;
        } else {
            kpCoeff = 0.95;
            tiCoeff = 1.4;
            tdCoeff = 0.47;
        }
    }
    
    gains.Kp = kpCoeff * model.tau / (model.K * model.L);
    gains.Ki = gains.Kp / (tiCoeff * model.tau);
    gains.Kd = gains.Kp * tdCoeff * model.L;
    
    if (form == ControllerForm::PIDFiltered) {
        gains.Tf = tdCoeff * model.L / 10.0;
    }
    
    return gains;
}

inline PIDGains TuningRules::calculate(const FOPDTModel& model, TuningRule rule,
                                       ControllerForm form, double lambda) {
    switch (rule) {
        case TuningRule::ZieglerNichols: {
            // Convert FOPDT to Ku, Tu approximation
            double Ku = 1.2 * model.tau / (model.K * model.L);
            double Tu = 4.0 * model.L;
            return zieglerNicholsUltimate(Ku, Tu, form);
        }
        
        case TuningRule::CohenCoon:
            return cohenCoon(model, form);
            
        case TuningRule::Lambda:
            return lambdaTuning(model, lambda);
            
        case TuningRule::SIMC:
            return simcTuning(model, lambda);
            
        case TuningRule::AMIGO:
            return amigoTuning(model, form);
            
        case TuningRule::ChienHronesReswick0:
            return chienHronesReswick(model, form, 0.0);
            
        case TuningRule::ChienHronesReswick20:
            return chienHronesReswick(model, form, 20.0);
            
        default:
            return cohenCoon(model, form);  // Default fallback
    }
}

// ============================================================================
// Process Identification Implementations
// ============================================================================

inline FOPDTModel ProcessIdentification::twoPointMethod(
    const std::vector<double>& time,
    const std::vector<double>& response,
    double stepSize) {
    
    FOPDTModel model;
    
    if (time.size() < 3 || time.size() != response.size()) {
        return model;
    }
    
    // Find steady-state value
    double y0 = response.front();
    double yss = response.back();
    double deltaY = yss - y0;
    
    model.K = deltaY / stepSize;
    
    // Find 28.3% and 63.2% points
    double y283 = y0 + 0.283 * deltaY;
    double y632 = y0 + 0.632 * deltaY;
    
    double t283 = 0, t632 = 0;
    
    for (size_t i = 1; i < response.size(); ++i) {
        if (response[i-1] < y283 && response[i] >= y283) {
            // Linear interpolation
            double frac = (y283 - response[i-1]) / (response[i] - response[i-1]);
            t283 = time[i-1] + frac * (time[i] - time[i-1]);
        }
        if (response[i-1] < y632 && response[i] >= y632) {
            double frac = (y632 - response[i-1]) / (response[i] - response[i-1]);
            t632 = time[i-1] + frac * (time[i] - time[i-1]);
            break;
        }
    }
    
    // Calculate τ and L
    model.tau = 1.5 * (t632 - t283);
    model.L = t632 - model.tau;
    
    if (model.L < 0) model.L = 0;
    
    return model;
}

inline FOPDTModel ProcessIdentification::tangentMethod(
    const std::vector<double>& time,
    const std::vector<double>& response,
    double stepSize) {
    
    FOPDTModel model;
    
    if (time.size() < 3 || time.size() != response.size()) {
        return model;
    }
    
    // Find steady-state
    double y0 = response.front();
    double yss = response.back();
    double deltaY = yss - y0;
    
    model.K = deltaY / stepSize;
    
    // Find maximum slope
    double maxSlope = 0;
    size_t maxSlopeIdx = 0;
    
    for (size_t i = 1; i < response.size() - 1; ++i) {
        double slope = (response[i+1] - response[i-1]) / (time[i+1] - time[i-1]);
        if (slope > maxSlope) {
            maxSlope = slope;
            maxSlopeIdx = i;
        }
    }
    
    // Tangent line: y = y_i + slope * (t - t_i)
    // Intersects y0 at: t = t_i - (y_i - y0) / slope
    // Intersects yss at: t = t_i + (yss - y_i) / slope
    
    double tInflection = time[maxSlopeIdx];
    double yInflection = response[maxSlopeIdx];
    
    double tL = tInflection - (yInflection - y0) / maxSlope;
    double tTau = tInflection + (yss - yInflection) / maxSlope;
    
    model.L = std::max(0.0, tL);
    model.tau = tTau - tL;
    
    return model;
}

inline FOPDTModel ProcessIdentification::identifyFOPDT(
    const std::vector<double>& time,
    const std::vector<double>& response,
    double stepSize) {
    // Use two-point method as default
    return twoPointMethod(time, response, stepSize);
}

} // namespace tether::control

