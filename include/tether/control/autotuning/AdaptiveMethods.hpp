/**
 * @file AdaptiveMethods.hpp
 * @brief Adaptive and Self-Tuning Controller Methods
 * 
 * @details
 * This file implements adaptive control methods that adjust controller
 * parameters online based on system behavior or parameter estimation.
 * 
 * ## Implemented Methods
 * 
 * ### Gain Scheduling
 * Pre-computed gains switched based on operating point
 * 
 * ### Model Reference Adaptive Control (MRAC)
 * Adapt controller to match reference model behavior
 * 
 * ### Self-Tuning Regulator (STR)
 * Online system identification + controller design
 * 
 * ### Extremum Seeking Control (ESC)
 * Real-time optimization without model
 * 
 * ### Fuzzy Logic Tuning
 * Rule-based parameter adjustment
 * 
 * ### Neural Network Tuning
 * Learn optimal parameters from data
 * 
 * ### Multiple Model Adaptive Control (MMAC)
 * Bank of models with switching/mixing
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include "ClassicalTuningMethods.hpp"
#include <map>
#include <deque>
#include <functional>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// Gain Scheduling
// ============================================================================

/**
 * @brief Gain Scheduling Controller
 * 
 * Uses lookup table or interpolation to adjust gains based on
 * scheduling variable (operating point).
 * 
 * ## Approaches
 * - **Table lookup**: Discrete operating points
 * - **Interpolation**: Linear/bilinear between points
 * - **Function**: Analytical function of scheduling variable
 */
class GainScheduler : public OnlineAutotuner {
public:
    std::string getName() const override { return "Gain Scheduling"; }
    std::string getDescription() const override {
        return "Pre-computed gains indexed by operating point. "
               "Fast adaptation for systems with known nonlinearities.";
    }
    bool isCompatible(const TunableController& controller) const override { return true; }
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }  // Always running
    void start() override {}
    void stop() override {}
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Add gain set at operating point
     */
    void addGainSet(double operatingPoint, const ParameterVector& gains);
    
    /**
     * @brief Set gains as function of scheduling variable
     */
    void setGainFunction(std::function<ParameterVector(double)> func);
    
    /**
     * @brief Set scheduling variable function
     * Maps (measured, reference) -> scheduling variable
     */
    void setSchedulingVariable(std::function<double(double, double)> func);
    
    /**
     * @brief Enable interpolation between gain sets
     */
    void enableInterpolation(bool enable) { m_interpolate = enable; }
    
    /**
     * @brief Get current gains
     */
    ParameterVector getCurrentGains() const { return m_currentGains; }
    
    /**
     * @brief Set rate limit for gain changes
     */
    void setGainRateLimit(double maxChangePerSecond) { m_rateLimit = maxChangePerSecond; }
    
private:
    std::map<double, ParameterVector> m_gainTable;
    std::function<ParameterVector(double)> m_gainFunction;
    std::function<double(double, double)> m_scheduleFunc;
    
    bool m_interpolate{true};
    bool m_useFunctionMap{false};
    double m_rateLimit{std::numeric_limits<double>::max()};
    
    ParameterVector m_currentGains;
    double m_lastScheduleVar{0.0};
};

// ============================================================================
// Model Reference Adaptive Control (MRAC)
// ============================================================================

/**
 * @brief Model Reference Adaptive Control
 * 
 * Adapts controller parameters so that closed-loop behavior
 * matches a reference model.
 * 
 * ## Structure
 * - Reference model: y_m = W_m(s) * r
 * - Plant: y = G(s, θ) * u  (unknown parameters θ)
 * - Controller: u = C(s, θ_hat) * e where e = r - y
 * - Adaptation: θ̂̇ = -Γ * e_m * φ (MIT rule or Lyapunov)
 */
class MRAC : public OnlineAutotuner {
public:
    std::string getName() const override { return "Model Reference Adaptive Control"; }
    std::string getDescription() const override {
        return "Adapts controller to match reference model behavior. "
               "Uses MIT rule or Lyapunov-based adaptation.";
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
     * @brief Set reference model (first order)
     * y_m = K_m / (τ_m * s + 1) * r
     */
    void setReferenceModel(double Km, double taum);
    
    /**
     * @brief Set reference model (second order)
     */
    void setReferenceModel(double Km, double wn, double zeta);
    
    /**
     * @brief Set adaptation gains
     */
    void setAdaptationGain(const std::vector<double>& gamma);
    
    /**
     * @brief Select adaptation law
     */
    enum class AdaptationLaw {
        MIT,        ///< MIT rule (gradient)
        Lyapunov,   ///< Lyapunov-based
        NormalizedMIT  ///< Normalized MIT rule
    };
    void setAdaptationLaw(AdaptationLaw law) { m_law = law; }
    
    /**
     * @brief Enable parameter projection (bounds)
     */
    void setParameterBounds(const std::vector<ParameterBounds>& bounds);
    
    /**
     * @brief Set sigma modification for robustness
     * Prevents parameter drift: θ̂̇ = -Γ*e_m*φ - σ*Γ*θ̂
     */
    void setSigmaModification(double sigma) { m_sigma = sigma; }
    
    /**
     * @brief Get model tracking error
     */
    double getModelError() const { return m_em; }
    
private:
    // Reference model
    double m_Km{1.0}, m_taum{1.0};
    bool m_secondOrderRef{false};
    double m_wn{1.0}, m_zeta{0.707};
    
    // Reference model state
    double m_ym{0.0}, m_ymDot{0.0};
    double m_ym2{0.0};  // For 2nd order
    
    // Adaptation
    AdaptationLaw m_law{AdaptationLaw::Lyapunov};
    std::vector<double> m_gamma;
    double m_sigma{0.0};
    std::vector<ParameterBounds> m_paramBounds;
    
    // State
    ParameterVector m_theta;  // Estimated parameters
    double m_em{0.0};  // Model error
    std::vector<double> m_phi;  // Regressor
    bool m_running{false};
};

// ============================================================================
// Self-Tuning Regulator (STR)
// ============================================================================

/**
 * @brief Self-Tuning Regulator
 * 
 * Online combination of:
 * 1. Recursive parameter estimation
 * 2. Controller design from estimated model
 * 
 * ## Estimation Methods
 * - Recursive Least Squares (RLS)
 * - Extended Least Squares (for colored noise)
 * - Recursive Prediction Error Method
 */
class SelfTuningRegulator : public OnlineAutotuner {
public:
    std::string getName() const override { return "Self-Tuning Regulator"; }
    std::string getDescription() const override {
        return "Online system identification + automatic controller design. "
               "Adapts to changing system dynamics.";
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
     * @brief Set model structure
     * @param na Number of output lags (A polynomial order)
     * @param nb Number of input lags (B polynomial order)  
     * @param nk Input delay in samples
     */
    void setModelStructure(int na, int nb, int nk);
    
    /**
     * @brief Set estimation method
     */
    enum class EstimationMethod {
        RLS,            ///< Recursive Least Squares
        RLSForgetting,  ///< RLS with forgetting factor
        ELS,            ///< Extended Least Squares
        RPEM            ///< Recursive Prediction Error
    };
    void setEstimationMethod(EstimationMethod method) { m_estimator = method; }
    
    /**
     * @brief Set forgetting factor (for RLS)
     * λ = 1: no forgetting, λ < 1: exponential forgetting
     */
    void setForgettingFactor(double lambda) { m_lambda = lambda; }
    
    /**
     * @brief Set controller design method
     */
    enum class DesignMethod {
        MinimumVariance,
        PolePlacement,
        PIDFromModel
    };
    void setDesignMethod(DesignMethod method) { m_designer = method; }
    
    /**
     * @brief Set tuning update interval
     * How often to recompute controller gains
     */
    void setUpdateInterval(int samples) { m_updateInterval = samples; }
    
    /**
     * @brief Get estimated model parameters
     */
    std::vector<double> getEstimatedA() const { return m_aHat; }
    std::vector<double> getEstimatedB() const { return m_bHat; }
    
    /**
     * @brief Get estimation covariance
     */
    double getEstimationCovariance() const;
    
private:
    // Model structure
    int m_na{2}, m_nb{2}, m_nk{1};
    
    // Estimation
    EstimationMethod m_estimator{EstimationMethod::RLSForgetting};
    double m_lambda{0.98};
    
    // Estimated parameters
    std::vector<double> m_aHat;
    std::vector<double> m_bHat;
    std::vector<double> m_cHat;  // For ELS
    std::vector<std::vector<double>> m_P;  // Covariance matrix
    
    // Design
    DesignMethod m_designer{DesignMethod::PIDFromModel};
    int m_updateInterval{10};
    int m_sampleCount{0};
    
    // Data buffers
    std::deque<double> m_yBuffer;
    std::deque<double> m_uBuffer;
    std::deque<double> m_eBuffer;  // For ELS
    
    // State
    bool m_running{false};
    ParameterVector m_currentGains;
    
    // Methods
    void updateEstimate(double y, double u);
    void updateController();
    std::vector<double> buildRegressor() const;
};

// ============================================================================
// Extremum Seeking Control (ESC)
// ============================================================================

/**
 * @brief Extremum Seeking Control
 * 
 * Model-free optimization that finds and tracks extremum of
 * unknown static map y = f(θ).
 * 
 * ## Algorithm
 * θ̂ = sin(ωt) * a        (perturbation)
 * θ = θ_avg + θ̂
 * ξ = HPF(y)             (remove DC)
 * θ_avg += k * ξ * sin(ωt)  (gradient estimate)
 */
class ExtremumSeekingControl : public OnlineAutotuner {
public:
    std::string getName() const override { return "Extremum Seeking Control"; }
    std::string getDescription() const override {
        return "Model-free real-time optimization. Finds and tracks "
               "optimum of unknown cost function.";
    }
    bool isCompatible(const TunableController& controller) const override { return true; }
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set perturbation parameters
     * @param amplitude Perturbation amplitude
     * @param frequency Perturbation frequency [rad/s]
     */
    void setPerturbation(double amplitude, double frequency);
    
    /**
     * @brief Set adaptation gain
     */
    void setAdaptationGain(double k) { m_k = k; }
    
    /**
     * @brief Set filter cutoff (should be < perturbation freq)
     */
    void setFilterCutoff(double wc) { m_filterCutoff = wc; }
    
    /**
     * @brief Set cost function
     * Maps (output, reference) -> cost to minimize
     */
    void setCostFunction(std::function<double(double, double)> cost);
    
    /**
     * @brief Set number of parameters to optimize
     */
    void setNumParameters(int n);
    
    /**
     * @brief Get current parameter estimates
     */
    ParameterVector getParameters() const { return m_theta; }
    
    /**
     * @brief Multi-parameter ESC with orthogonal perturbations
     */
    void enableMultiParameter(bool enable) { m_multiParam = enable; }
    
private:
    // Perturbation
    double m_amplitude{0.1};
    double m_omega{10.0};
    std::vector<double> m_phases;  // For multi-param
    
    // Adaptation
    double m_k{1.0};
    double m_filterCutoff{1.0};
    
    // Cost function
    std::function<double(double, double)> m_costFunc;
    
    // State
    ParameterVector m_theta;
    std::vector<double> m_xi;  // Filtered signals
    double m_time{0.0};
    bool m_multiParam{false};
    bool m_running{false};
    
    // High-pass filter states
    std::vector<double> m_hpfState;
};

// ============================================================================
// Fuzzy Logic Tuning
// ============================================================================

/**
 * @brief Fuzzy Logic PID Tuning
 * 
 * Uses fuzzy rules to adjust PID gains based on error and
 * error rate.
 * 
 * ## Fuzzy Sets
 * - Error: NB, NM, NS, ZO, PS, PM, PB
 * - Error rate: N, Z, P
 * 
 * ## Output
 * - Kp adjustment: ΔKp = f(e, ė)
 * - Similar for Ki, Kd
 */
class FuzzyTuning : public OnlineAutotuner {
public:
    std::string getName() const override { return "Fuzzy Logic Tuning"; }
    std::string getDescription() const override {
        return "Rule-based gain adjustment using fuzzy logic. "
               "Adapts gains based on error and error rate.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Fuzzy membership function
     */
    struct MembershipFunction {
        enum Type { Triangular, Gaussian, Trapezoidal };
        Type type{Triangular};
        double a{0}, b{0}, c{0}, d{0};  // Shape parameters
        
        double evaluate(double x) const;
    };
    
    /**
     * @brief Set error universe of discourse
     */
    void setErrorRange(double min, double max);
    
    /**
     * @brief Set error rate universe of discourse
     */
    void setErrorRateRange(double min, double max);
    
    /**
     * @brief Set gain adjustment ranges
     */
    void setGainRanges(double dKpMax, double dKiMax, double dKdMax);
    
    /**
     * @brief Add custom rule
     */
    void addRule(const std::string& errorSet, const std::string& errorRateSet,
                double dKp, double dKi, double dKd);
    
    /**
     * @brief Use default Mamdani rule base
     */
    void useDefaultRules();
    
    /**
     * @brief Set base (nominal) gains
     */
    void setBaseGains(double Kp, double Ki, double Kd);
    
private:
    // Universes
    double m_eMin{-1}, m_eMax{1};
    double m_deMin{-1}, m_deMax{1};
    double m_dKpMax{0.5}, m_dKiMax{0.5}, m_dKdMax{0.5};
    
    // Base gains
    double m_Kp0{1}, m_Ki0{0.1}, m_Kd0{0.05};
    
    // Membership functions
    std::map<std::string, MembershipFunction> m_errorMFs;
    std::map<std::string, MembershipFunction> m_rateMFs;
    
    // Rule base: (error_set, rate_set) -> (dKp, dKi, dKd)
    struct Rule {
        std::string errorSet;
        std::string rateSet;
        double dKp, dKi, dKd;
    };
    std::vector<Rule> m_rules;
    
    // State
    double m_lastError{0};
    bool m_running{false};
    
    // Methods
    double fuzzify(double value, const MembershipFunction& mf) const;
    void defuzzify(double& dKp, double& dKi, double& dKd);
};

// ============================================================================
// Neural Network Tuning
// ============================================================================

/**
 * @brief Neural Network-Based Controller Tuning
 * 
 * Uses feedforward neural network to learn mapping from
 * system state to optimal controller parameters.
 * 
 * ## Architecture
 * - Input: error, error integral, error derivative, operating point
 * - Hidden: 1-2 layers with ReLU/tanh activation
 * - Output: Kp, Ki, Kd (or parameter adjustments)
 */
class NeuralNetworkTuning : public OnlineAutotuner {
public:
    std::string getName() const override { return "Neural Network Tuning"; }
    std::string getDescription() const override {
        return "Learn optimal gains using neural network. "
               "Can approximate complex nonlinear gain schedules.";
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
     * @brief Set network architecture
     * @param layers Neurons in each hidden layer
     */
    void setArchitecture(const std::vector<int>& layers);
    
    /**
     * @brief Set activation function
     */
    enum class Activation { ReLU, Tanh, Sigmoid, Linear };
    void setActivation(Activation act) { m_activation = act; }
    
    /**
     * @brief Set learning rate
     */
    void setLearningRate(double lr) { m_learningRate = lr; }
    
    /**
     * @brief Load pre-trained weights
     */
    void loadWeights(const std::vector<std::vector<double>>& weights);
    
    /**
     * @brief Enable online learning
     */
    void enableOnlineLearning(bool enable) { m_onlineLearning = enable; }
    
    /**
     * @brief Train on batch of data
     * @param inputs Input features [samples x features]
     * @param targets Target outputs [samples x outputs]
     */
    void trainBatch(const std::vector<std::vector<double>>& inputs,
                   const std::vector<std::vector<double>>& targets,
                   int epochs = 100);
    
private:
    // Network structure
    std::vector<int> m_layers;
    Activation m_activation{Activation::Tanh};
    
    // Weights: m_W[layer][neuron][input]
    std::vector<std::vector<std::vector<double>>> m_W;
    std::vector<std::vector<double>> m_b;  // Biases
    
    // Learning
    double m_learningRate{0.01};
    bool m_onlineLearning{false};
    
    // State
    bool m_running{false};
    std::vector<double> m_lastOutput;
    
    // Forward pass
    std::vector<double> forward(const std::vector<double>& input);
    double activate(double x) const;
    double activateDerivative(double x) const;
};

// ============================================================================
// Multiple Model Adaptive Control (MMAC)
// ============================================================================

/**
 * @brief Multiple Model Adaptive Control
 * 
 * Maintains bank of models covering uncertainty set.
 * Switches or blends based on model fit.
 * 
 * ## Approaches
 * - **Switching**: Select best model's controller
 * - **Mixing**: Weighted average of controller outputs
 * - **Adaptive mixing**: Probabilistic weights (MMAE)
 */
class MMAC : public OnlineAutotuner {
public:
    std::string getName() const override { return "Multiple Model Adaptive Control"; }
    std::string getDescription() const override {
        return "Bank of models with switching or mixing. "
               "Fast adaptation for systems with large uncertainty.";
    }
    bool isCompatible(const TunableController& controller) const override { return true; }
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Add model with associated controller gains
     */
    void addModel(const FOPDTModel& model, const ParameterVector& gains);
    
    /**
     * @brief Set mixing mode
     */
    enum class MixingMode {
        Switching,      ///< Hard switch to best model
        BayesianMixing, ///< Posterior probability weighted
        SoftmaxMixing   ///< Softmax weighted
    };
    void setMixingMode(MixingMode mode) { m_mixingMode = mode; }
    
    /**
     * @brief Set switching hysteresis
     */
    void setSwitchingHysteresis(double h) { m_hysteresis = h; }
    
    /**
     * @brief Get current model probabilities
     */
    std::vector<double> getModelProbabilities() const { return m_probabilities; }
    
    /**
     * @brief Get active model index
     */
    int getActiveModel() const { return m_activeModel; }
    
private:
    struct ModelEntry {
        FOPDTModel model;
        ParameterVector gains;
        double yPred{0.0};      // Model prediction
        double residual{0.0};   // Prediction error
        double likelihood{1.0}; // For Bayesian mixing
    };
    
    std::vector<ModelEntry> m_models;
    MixingMode m_mixingMode{MixingMode::BayesianMixing};
    
    int m_activeModel{0};
    std::vector<double> m_probabilities;
    double m_hysteresis{0.1};
    
    // State
    double m_lastY{0.0}, m_lastU{0.0};
    bool m_running{false};
    
    void updateProbabilities(double y);
    ParameterVector computeMixedGains() const;
};

} // namespace Autotuning
} // namespace tether::control
