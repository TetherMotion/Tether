/**
 * @file AutotuningFramework.hpp
 * @brief Comprehensive Controller Autotuning Framework
 * 
 * @details
 * This file provides the foundational architecture for a comprehensive controller
 * autotuning framework. The framework supports tuning of arbitrary controller
 * implementations through a unified interface.
 * 
 * ## Architecture Overview
 * 
 * ```
 *                    ┌─────────────────────────────────────┐
 *                    │          AutotunerBase              │
 *                    │  (Abstract interface for all        │
 *                    │   autotuning algorithms)            │
 *                    └─────────────┬───────────────────────┘
 *                                  │
 *          ┌───────────────────────┼───────────────────────┐
 *          │                       │                       │
 *          ▼                       ▼                       ▼
 *   ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
 *   │  Classical   │      │ Optimization │      │  Adaptive    │
 *   │   Methods    │      │   -Based     │      │  Methods     │
 *   │ ZN,CC,IMC    │      │ GA,PSO,SA    │      │ MRAC,STR     │
 *   └──────────────┘      └──────────────┘      └──────────────┘
 *          │                       │                       │
 *          └───────────────────────┼───────────────────────┘
 *                                  │
 *                                  ▼
 *                    ┌─────────────────────────────────────┐
 *                    │       TunableController             │
 *                    │  (Interface for controllers that    │
 *                    │   can be tuned)                     │
 *                    └─────────────────────────────────────┘
 * ```
 * 
 * ## Design Principles
 * 
 * 1. **Controller Agnostic**: Any controller implementing TunableController can
 *    be tuned using any compatible autotuner.
 * 
 * 2. **Extensible**: New tuning methods can be added by inheriting from AutotunerBase.
 * 
 * 3. **Composable**: Tuners can be combined (e.g., Z-N + optimization refinement).
 * 
 * 4. **Real-time Capable**: Online tuning methods support real-time adjustment.
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 * @date 2026
 */

#pragma once

#include "../ControllerBase.hpp"
#include <vector>
#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <optional>
#include <array>
#include <cmath>
#include <complex>
#include <limits>

namespace Control {
namespace Autotuning {

// ============================================================================
// Forward Declarations
// ============================================================================

class AutotunerBase;
class TunableController;
class ProcessModel;
class OptimizationAlgorithm;

// ============================================================================
// Type Aliases
// ============================================================================

using AutotunerPtr = std::shared_ptr<AutotunerBase>;
using TunableControllerPtr = std::shared_ptr<TunableController>;
using OptimizationPtr = std::shared_ptr<OptimizationAlgorithm>;

// ============================================================================
// Common Types
// ============================================================================

/**
 * @brief Parameter bounds for optimization
 */
struct ParameterBounds {
    double min{-std::numeric_limits<double>::max()};
    double max{std::numeric_limits<double>::max()};
    
    bool contains(double value) const {
        return value >= min && value <= max;
    }
    
    double clamp(double value) const {
        return std::max(min, std::min(max, value));
    }
    
    double range() const { return max - min; }
    double center() const { return (min + max) / 2.0; }
};

/**
 * @brief Parameter descriptor for tunable parameters
 */
struct ParameterDescriptor {
    std::string name;
    double initialValue{0.0};
    ParameterBounds bounds;
    double scale{1.0};  // For normalization
    bool logarithmic{false};  // Use log scale for optimization
    
    double normalize(double value) const {
        if (logarithmic && value > 0) {
            return (std::log(value) - std::log(bounds.min)) / 
                   (std::log(bounds.max) - std::log(bounds.min));
        }
        return (value - bounds.min) / bounds.range();
    }
    
    double denormalize(double normalized) const {
        if (logarithmic) {
            return std::exp(std::log(bounds.min) + 
                          normalized * (std::log(bounds.max) - std::log(bounds.min)));
        }
        return bounds.min + normalized * bounds.range();
    }
};

/**
 * @brief Collection of parameters
 */
using ParameterVector = std::vector<double>;
using ParameterDescriptors = std::vector<ParameterDescriptor>;

/**
 * @brief Tuning result
 */
struct TuningResult {
    ParameterVector parameters;
    double cost{std::numeric_limits<double>::max()};
    bool success{false};
    std::string message;
    int iterations{0};
    int functionEvaluations{0};
    double elapsedTime{0.0};
    
    // Performance metrics
    double settlingTime{0.0};
    double overshoot{0.0};
    double riseTime{0.0};
    double steadyStateError{0.0};
    double gainMargin{0.0};
    double phaseMargin{0.0};
};

/**
 * @brief Process model types
 */
enum class ProcessModelType {
    FOPDT,      ///< First Order Plus Dead Time
    SOPDT,      ///< Second Order Plus Dead Time
    IPDT,       ///< Integrating Plus Dead Time
    IFOPDT,     ///< Integrating First Order Plus Dead Time
    Unstable,   ///< Unstable process
    Generic     ///< Generic state-space model
};

/**
 * @brief First Order Plus Dead Time model: G(s) = K * exp(-L*s) / (τ*s + 1)
 */
struct FOPDTModel {
    double K{1.0};      ///< Process gain
    double tau{1.0};    ///< Time constant
    double L{0.0};      ///< Dead time (delay)
    
    double normalizedDeadTime() const { return L / tau; }
    bool isValid() const { return tau > 0 && L >= 0; }
    
    std::complex<double> evaluate(double omega) const {
        std::complex<double> j(0, 1);
        return K * std::exp(-j * omega * L) / (tau * j * omega + 1.0);
    }
};

/**
 * @brief Second Order Plus Dead Time model
 */
struct SOPDTModel {
    double K{1.0};      ///< Process gain
    double tau1{1.0};   ///< First time constant
    double tau2{0.5};   ///< Second time constant
    double L{0.0};      ///< Dead time
    double zeta{0.7};   ///< Damping ratio (for underdamped form)
    
    bool isValid() const { return tau1 > 0 && tau2 > 0 && L >= 0; }
};

/**
 * @brief Integrating Plus Dead Time model: G(s) = K * exp(-L*s) / s
 */
struct IPDTModel {
    double K{1.0};      ///< Process gain
    double L{0.0};      ///< Dead time
    
    bool isValid() const { return L >= 0; }
};

/**
 * @brief Performance objective for tuning
 */
enum class PerformanceObjective {
    SetpointTracking,   ///< Optimize setpoint response
    DisturbanceRejection, ///< Optimize disturbance rejection
    Balanced,           ///< Balance both objectives
    MinimumTime,        ///< Minimize settling time
    MinimumOvershoot,   ///< Minimize overshoot
    Robust              ///< Maximize robustness margins
};

/**
 * @brief Autotuning mode
 */
enum class AutotuningMode {
    Offline,            ///< Offline tuning with recorded data
    Online,             ///< Online/adaptive tuning
    Batch               ///< Batch processing of multiple experiments
};

/**
 * @brief Controller form for PID-type controllers
 */
enum class PIDForm {
    Parallel,           ///< u = Kp*e + Ki*∫e + Kd*de/dt
    Standard,           ///< u = Kp*(e + 1/Ti*∫e + Td*de/dt)
    Series              ///< Interacting form
};

// ============================================================================
// Process Model Interface
// ============================================================================

/**
 * @brief Abstract process model interface
 * 
 * Represents the process to be controlled. Used by model-based
 * tuning methods.
 */
class ProcessModel {
public:
    virtual ~ProcessModel() = default;
    
    /**
     * @brief Get model type
     */
    virtual ProcessModelType getType() const = 0;
    
    /**
     * @brief Evaluate transfer function at frequency
     * @param omega Angular frequency [rad/s]
     * @return Complex frequency response
     */
    virtual std::complex<double> evaluate(double omega) const = 0;
    
    /**
     * @brief Simulate step response
     * @param stepMagnitude Step input magnitude
     * @param dt Time step
     * @param duration Simulation duration
     * @return Time-value pairs
     */
    virtual std::vector<std::pair<double, double>> stepResponse(
        double stepMagnitude, double dt, double duration) const = 0;
    
    /**
     * @brief Get approximate FOPDT model
     */
    virtual FOPDTModel toFOPDT() const = 0;
    
    /**
     * @brief Get ultimate gain and period
     * @return (Ku, Tu) pair
     */
    virtual std::pair<double, double> getUltimateParams() const = 0;
};

// ============================================================================
// Tunable Controller Interface
// ============================================================================

/**
 * @brief Interface for controllers that can be autotuned
 * 
 * Any controller implementing this interface can be tuned using
 * the autotuning framework.
 */
class TunableController {
public:
    virtual ~TunableController() = default;
    
    /**
     * @brief Get parameter descriptors
     * @return Vector of tunable parameters with bounds
     */
    virtual ParameterDescriptors getParameterDescriptors() const = 0;
    
    /**
     * @brief Get current parameter values
     */
    virtual ParameterVector getParameters() const = 0;
    
    /**
     * @brief Set parameter values
     * @param params New parameter values
     * @return true if parameters are valid and applied
     */
    virtual bool setParameters(const ParameterVector& params) = 0;
    
    /**
     * @brief Get controller type name
     */
    virtual std::string getControllerTypeName() const = 0;
    
    /**
     * @brief Evaluate controller performance on test data
     * @param input Input signal
     * @param reference Reference signal
     * @param dt Sample time
     * @return Output signal
     */
    virtual std::vector<double> evaluate(
        const std::vector<double>& input,
        const std::vector<double>& reference,
        double dt) = 0;
    
    /**
     * @brief Reset controller state
     */
    virtual void reset() = 0;
    
    /**
     * @brief Clone the controller
     */
    virtual std::shared_ptr<TunableController> clone() const = 0;
};

// ============================================================================
// Cost Function Interface
// ============================================================================

/**
 * @brief Cost function for optimization-based tuning
 */
class CostFunction {
public:
    virtual ~CostFunction() = default;
    
    /**
     * @brief Evaluate cost for given parameters
     * @param params Controller parameters
     * @return Cost value (lower is better)
     */
    virtual double evaluate(const ParameterVector& params) = 0;
    
    /**
     * @brief Get gradient (if available)
     * @param params Current parameters
     * @return Gradient vector
     */
    virtual std::optional<ParameterVector> gradient(const ParameterVector& params) {
        return std::nullopt;  // Default: not available
    }
    
    /**
     * @brief Check if gradient is available
     */
    virtual bool hasGradient() const { return false; }
};

using CostFunctionPtr = std::shared_ptr<CostFunction>;

/**
 * @brief Standard cost functions for control tuning
 */
class StandardCostFunctions {
public:
    /**
     * @brief Integral of Squared Error (ISE): ∫e²dt
     */
    static double ISE(const std::vector<double>& error, double dt);
    
    /**
     * @brief Integral of Absolute Error (IAE): ∫|e|dt
     */
    static double IAE(const std::vector<double>& error, double dt);
    
    /**
     * @brief Integral of Time-weighted Absolute Error (ITAE): ∫t|e|dt
     */
    static double ITAE(const std::vector<double>& error, double dt);
    
    /**
     * @brief Integral of Time-weighted Squared Error (ITSE): ∫te²dt
     */
    static double ITSE(const std::vector<double>& error, double dt);
    
    /**
     * @brief Combined cost with overshoot penalty
     */
    static double combinedCost(const std::vector<double>& response,
                               const std::vector<double>& reference,
                               double overshootWeight = 1.0,
                               double settlingWeight = 1.0,
                               double controlEffortWeight = 0.1,
                               double dt = 0.001);
};

// ============================================================================
// Autotuner Base Class
// ============================================================================

/**
 * @brief Abstract base class for all autotuning algorithms
 * 
 * All autotuning methods inherit from this class and implement
 * the tune() method for their specific algorithm.
 */
class AutotunerBase {
public:
    virtual ~AutotunerBase() = default;
    
    /**
     * @brief Get autotuner name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Get autotuner description
     */
    virtual std::string getDescription() const = 0;
    
    /**
     * @brief Get tuning mode (offline/online)
     * @return Default returns Offline mode; override in OnlineAutotuner
     */
    virtual AutotuningMode getMode() const { return AutotuningMode::Offline; }
    
    /**
     * @brief Check if tuner is compatible with controller type
     */
    virtual bool isCompatible(const TunableController& controller) const = 0;
    
    /**
     * @brief Tune controller parameters
     * @param controller Controller to tune
     * @param model Process model (optional, depends on method)
     * @return Tuning result with optimal parameters
     */
    virtual TuningResult tune(
        TunableController& controller,
        const ProcessModel* model = nullptr) = 0;
    
    /**
     * @brief Set performance objective
     */
    void setObjective(PerformanceObjective objective) { m_objective = objective; }
    
    /**
     * @brief Get performance objective
     */
    PerformanceObjective getObjective() const { return m_objective; }
    
    /**
     * @brief Set maximum iterations
     */
    void setMaxIterations(int maxIter) { m_maxIterations = maxIter; }
    
    /**
     * @brief Set convergence tolerance
     */
    void setTolerance(double tol) { m_tolerance = tol; }
    
    /**
     * @brief Set verbose output
     */
    void setVerbose(bool verbose) { m_verbose = verbose; }
    
protected:
    PerformanceObjective m_objective{PerformanceObjective::Balanced};
    int m_maxIterations{1000};
    double m_tolerance{1e-6};
    bool m_verbose{false};
};

/**
 * @brief Alias for offline autotuners (same as AutotunerBase but for semantic clarity)
 * 
 * Offline autotuners operate on batch data or models, not in real-time.
 */
using OfflineAutotuner = AutotunerBase;

// ============================================================================
// Online Autotuner Interface
// ============================================================================

/**
 * @brief Interface for online/adaptive autotuning methods
 */
class OnlineAutotuner : public AutotunerBase {
public:
    AutotuningMode getMode() const override { return AutotuningMode::Online; }
    
    /**
     * @brief Process one sample (for online tuning)
     * @param measured Current measurement
     * @param reference Current reference
     * @param control Applied control signal
     * @param dt Time step
     * @return Control output (if tuner produces control)
     */
    virtual double update(double measured, double reference, 
                         double control, double dt) = 0;
    
    /**
     * @brief Check if tuning is complete
     */
    virtual bool isComplete() const = 0;
    
    /**
     * @brief Start online tuning
     */
    virtual void start() = 0;
    
    /**
     * @brief Stop online tuning
     */
    virtual void stop() = 0;
    
    /**
     * @brief Get intermediate results
     */
    virtual TuningResult getIntermediateResult() const = 0;
};

// ============================================================================
// Simulation-Based Cost Function
// ============================================================================

/**
 * @brief Cost function that simulates closed-loop response
 */
class SimulationCostFunction : public CostFunction {
public:
    /**
     * @brief Constructor
     * @param controller Controller to evaluate
     * @param model Process model
     * @param referenceSignal Reference input for simulation
     * @param dt Simulation time step
     */
    SimulationCostFunction(
        std::shared_ptr<TunableController> controller,
        std::shared_ptr<ProcessModel> model,
        std::vector<double> referenceSignal,
        double dt = 0.001);
    
    double evaluate(const ParameterVector& params) override;
    
    /**
     * @brief Set cost weights
     */
    void setWeights(double ise = 1.0, double overshoot = 10.0, 
                    double settling = 1.0, double control = 0.01);
    
private:
    std::shared_ptr<TunableController> m_controller;
    std::shared_ptr<ProcessModel> m_model;
    std::vector<double> m_reference;
    double m_dt;
    
    double m_weightISE{1.0};
    double m_weightOvershoot{10.0};
    double m_weightSettling{1.0};
    double m_weightControl{0.01};
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Calculate step response characteristics
 */
struct StepResponseMetrics {
    double riseTime{0.0};       ///< 10% to 90%
    double settlingTime{0.0};   ///< To within 2%
    double overshoot{0.0};      ///< Percentage overshoot
    double peakTime{0.0};       ///< Time to first peak
    double steadyStateValue{0.0};
    double steadyStateError{0.0};
};

StepResponseMetrics analyzeStepResponse(
    const std::vector<double>& response,
    double finalValue,
    double dt,
    double settlingThreshold = 0.02);

/**
 * @brief Calculate frequency response characteristics
 */
struct FrequencyResponseMetrics {
    double gainMargin{0.0};         ///< [dB]
    double phaseMargin{0.0};        ///< [degrees]
    double crossoverFrequency{0.0}; ///< [rad/s]
    double bandwidth{0.0};          ///< [rad/s]
    double peakMagnitude{0.0};      ///< [dB]
};

FrequencyResponseMetrics analyzeFrequencyResponse(
    const ProcessModel& model,
    const TunableController& controller);

} // namespace Autotuning
} // namespace Control
