/**
 * @file ControllerBase.hpp
 * @brief Base controller interface and common types for the control framework
 * 
 * @details
 * This file provides the foundational abstractions for a comprehensive control
 * system framework. The design follows a modular, backend-agnostic architecture
 * that allows any controller to be composed with other controllers or systems.
 * 
 * ## Architecture Overview
 * 
 * ```
 *                    ┌─────────────────────────────────────┐
 *                    │         ControllerBase              │
 *                    │  (Abstract interface for all        │
 *                    │   control algorithms)               │
 *                    └─────────────┬───────────────────────┘
 *                                  │
 *          ┌───────────────────────┼───────────────────────┐
 *          │                       │                       │
 *          ▼                       ▼                       ▼
 *   ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
 *   │  PID Family  │      │ State-Space  │      │   Robust     │
 *   │ P,PD,PI,PID  │      │ LQR,LQG,LQI  │      │  H2, H∞      │
 *   │ FOPID, etc.  │      │              │      │              │
 *   └──────────────┘      └──────────────┘      └──────────────┘
 *          │                       │                       │
 *          └───────────────────────┼───────────────────────┘
 *                                  │
 *                                  ▼
 *                    ┌─────────────────────────────────────┐
 *                    │      CompositeController            │
 *                    │  (Combine controllers: cascade,     │
 *                    │   parallel, feedforward, etc.)      │
 *                    └─────────────────────────────────────┘
 * ```
 * 
 * ## Design Principles
 * 
 * 1. **Backend Agnostic**: Controllers can work with any system that provides
 *    the required interface (motor drives, simulations, etc.)
 * 
 * 2. **Composable**: Controllers can be nested, cascaded, or combined
 *    (e.g., LQR as inner loop, PID as outer loop)
 * 
 * 3. **Configurable**: Extensive configuration options with sensible defaults
 * 
 * 4. **Real-time Safe**: Designed for deterministic execution in real-time systems
 * 
 * ## Controller Categories
 * 
 * ### Classical Controllers
 * - **P, PD, PI, PID**: Proportional-Integral-Derivative family
 * - **Bang-Bang**: On-off control with hysteresis
 * - **PD+**: PD with gravity/friction compensation
 * 
 * ### Modern Control
 * - **LQR**: Linear Quadratic Regulator (optimal state feedback)
 * - **LQG**: LQR + Kalman filter (output feedback)
 * - **LQI**: LQR with integral action (zero steady-state error)
 * 
 * ### Robust Control
 * - **H2**: Minimize 2-norm of closed-loop transfer function
 * - **H∞**: Minimize infinity-norm (worst-case gain)
 * 
 * ### Advanced Controllers
 * - **FOPID**: Fractional-order PID (non-integer derivatives)
 * - **ILC**: Iterative Learning Control (repetitive tasks)
 * - **Dual-loop**: Cascaded position/velocity control
 * 
 * ## Usage Example
 * 
 * ```cpp
 * // Create a PID controller
 * auto pid = std::make_shared<PIDController>();
 * pid->setGains(1.0, 0.1, 0.05);
 * pid->setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
 * 
 * // Use it in a motion system
 * ElectronicGearing gearing;
 * gearing.setController(pid);
 * 
 * // Or create a cascade control
 * auto velocityPID = std::make_shared<PIDController>();
 * auto positionPID = std::make_shared<PIDController>();
 * 
 * CascadeController cascade;
 * cascade.setOuterController(positionPID);
 * cascade.setInnerController(velocityPID);
 * ```
 * 
 * @author Generated for ESP32 EtherCAT Motion Control
 * @version 1.0
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <memory>
#include <functional>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

namespace Control {

// ============================================================================
// Forward Declarations
// ============================================================================

class ControllerBase;
class StateEstimator;
class SystemModel;

// ============================================================================
// Type Aliases
// ============================================================================

using ControllerPtr = std::shared_ptr<ControllerBase>;
using ControllerWeakPtr = std::weak_ptr<ControllerBase>;

/**
 * @brief Maximum state dimension for fixed-size controllers
 */
constexpr size_t MAX_STATE_DIM = 12;

/**
 * @brief Maximum control input dimension
 */
constexpr size_t MAX_CONTROL_DIM = 6;

/**
 * @brief Maximum output dimension
 */
constexpr size_t MAX_OUTPUT_DIM = 6;

// ============================================================================
// Common Types
// ============================================================================

/**
 * @brief Fixed-size state vector
 */
using StateVector = std::array<double, MAX_STATE_DIM>;

/**
 * @brief Fixed-size control vector
 */
using ControlVector = std::array<double, MAX_CONTROL_DIM>;

/**
 * @brief Fixed-size output vector
 */
using OutputVector = std::array<double, MAX_OUTPUT_DIM>;

/**
 * @brief Fixed-size matrix (row-major)
 * 
 * For state-space systems: A is n×n, B is n×m, C is p×n, D is p×m
 */
template<size_t Rows, size_t Cols>
using Matrix = std::array<std::array<double, Cols>, Rows>;

/**
 * @brief State-space matrices type alias
 */
using StateMatrix = Matrix<MAX_STATE_DIM, MAX_STATE_DIM>;
using InputMatrix = Matrix<MAX_STATE_DIM, MAX_CONTROL_DIM>;
using OutputMatrix = Matrix<MAX_OUTPUT_DIM, MAX_STATE_DIM>;
using FeedthroughMatrix = Matrix<MAX_OUTPUT_DIM, MAX_CONTROL_DIM>;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief Controller type identifier
 */
enum class ControllerType {
    // Classical
    P,                      ///< Proportional only
    PD,                     ///< Proportional-Derivative
    PI,                     ///< Proportional-Integral
    PID,                    ///< Proportional-Integral-Derivative
    PID2DOF,                ///< Two degree-of-freedom PID
    
    // Nonlinear classical
    BangBang,               ///< On-off control
    PDPlus,                 ///< PD with compensation
    
    // Cascaded
    DualLoopPID,            ///< Position-velocity cascade
    
    // Fractional order
    FractionalPID,          ///< Fractional order PID (FOPID)
    
    // Optimal control
    LQR,                    ///< Linear Quadratic Regulator
    LQG,                    ///< Linear Quadratic Gaussian
    LQI,                    ///< LQR with Integral action
    
    // Robust control
    H2,                     ///< H2 optimal control
    HInfinity,              ///< H-infinity control
    
    // Learning control
    ILC,                    ///< Iterative Learning Control
    
    // Composite
    Cascade,                ///< Cascaded controllers
    Parallel,               ///< Parallel sum of controllers
    Feedforward,            ///< Controller with feedforward
    
    Custom                  ///< User-defined controller
};

/**
 * @brief Anti-windup methods for integral controllers
 * 
 * ## Method Descriptions
 * 
 * ### Clamping (Saturation)
 * Simply stops integrating when output saturates.
 * - Pros: Simple, no additional parameters
 * - Cons: Can be slow to recover
 * 
 * ### Back-Calculation
 * Feeds back the difference between saturated and unsaturated output.
 * - Pros: Smooth recovery, tunable via tracking time constant
 * - Cons: Requires saturation limits
 * 
 * ### Conditional Integration
 * Only integrates when error and output have same sign.
 * - Pros: Prevents overshoot
 * - Cons: May not integrate during transients
 * 
 * ### Tracking
 * Uses external feedback to track actual system state.
 * - Pros: Works with external actuator saturation
 * - Cons: Requires feedback signal
 * 
 * ### Observer-Based
 * Uses state observer to estimate true integrator state.
 * - Pros: Handles model uncertainties
 * - Cons: More complex implementation
 */
enum class AntiWindupMethod {
    None,                   ///< No anti-windup (not recommended)
    Clamping,               ///< Stop integration at limits
    BackCalculation,        ///< Back-calculate from saturation
    Conditional,            ///< Integrate only when appropriate
    Tracking,               ///< Track with external feedback
    ObserverBased           ///< Observer-based anti-windup
};

/**
 * @brief Controller operating mode
 */
enum class ControllerMode {
    Disabled,               ///< Controller output is zero
    Manual,                 ///< Manual output override
    Automatic,              ///< Normal automatic control
    Tracking,               ///< Track external setpoint
    Hold                    ///< Hold current output
};

/**
 * @brief Derivative filter type
 */
enum class DerivativeFilterType {
    None,                   ///< No filtering (noisy!)
    FirstOrder,             ///< First-order low-pass
    SecondOrder,            ///< Second-order low-pass
    MovingAverage,          ///< Moving average filter
    Median                  ///< Median filter (outlier rejection)
};

/**
 * @brief Setpoint weighting mode
 */
enum class SetpointWeight {
    Full,                   ///< Weight = 1.0 (full reference)
    Partial,                ///< 0 < weight < 1
    ErrorOnly               ///< Weight = 0 (error only)
};

// ============================================================================
// Controller Input/Output Structures
// ============================================================================

/**
 * @brief Controller input data
 * 
 * Contains all information needed for one control cycle.
 */
struct ControllerInput {
    double reference{0.0};          ///< Desired setpoint
    double measured{0.0};           ///< Measured process variable
    double dt{0.001};               ///< Time step [s]
    
    // Optional extended inputs
    double referenceDerivative{0.0};    ///< Derivative of reference (for 2DOF)
    double feedforward{0.0};            ///< Feedforward term
    double disturbance{0.0};            ///< Measured disturbance
    
    // State feedback (for state-space controllers)
    StateVector state{};                ///< Full state vector
    size_t stateDim{0};                 ///< Actual state dimension
    
    // Flags
    bool reset{false};                  ///< Reset controller state
    bool enable{true};                  ///< Enable controller
};

/**
 * @brief Controller output data
 */
struct ControllerOutput {
    double control{0.0};            ///< Primary control signal
    
    // Diagnostic outputs
    double proportional{0.0};       ///< P term contribution
    double integral{0.0};           ///< I term contribution  
    double derivative{0.0};         ///< D term contribution
    double feedforward{0.0};        ///< Feedforward contribution
    
    // Status
    bool saturated{false};          ///< Output is saturated
    bool integrating{false};        ///< Integrator is active
    double error{0.0};              ///< Current error
    
    // Extended outputs for state-space
    ControlVector controlVector{};  ///< Multi-dimensional control
    size_t controlDim{1};           ///< Control dimension
};

/**
 * @brief Controller saturation limits
 */
struct SaturationLimits {
    double outputMin{-std::numeric_limits<double>::max()};
    double outputMax{std::numeric_limits<double>::max()};
    double integralMin{-std::numeric_limits<double>::max()};
    double integralMax{std::numeric_limits<double>::max()};
    double derivativeMin{-std::numeric_limits<double>::max()};
    double derivativeMax{std::numeric_limits<double>::max()};
    double rateLimit{std::numeric_limits<double>::max()};  ///< Max output change rate
    
    bool isValid() const {
        return outputMin < outputMax && 
               integralMin < integralMax &&
               derivativeMin < derivativeMax;
    }
};

/**
 * @brief Controller tuning parameters (for auto-tuning)
 */
struct TuningParameters {
    double settlingTime{1.0};       ///< Desired settling time [s]
    double overshoot{0.05};         ///< Max overshoot (0-1)
    double bandwidth{10.0};         ///< Desired bandwidth [rad/s]
    double phaseMargin{60.0};       ///< Phase margin [degrees]
    double gainMargin{6.0};         ///< Gain margin [dB]
    bool aggressive{false};         ///< Aggressive vs conservative tuning
};

/**
 * @brief Controller diagnostic information
 */
struct ControllerDiagnostics {
    uint64_t cycleCount{0};         ///< Number of control cycles
    double maxError{0.0};           ///< Maximum error seen
    double rmsError{0.0};           ///< RMS error
    double integralValue{0.0};      ///< Current integral state
    double lastDerivative{0.0};     ///< Last derivative value
    uint32_t saturationCount{0};    ///< Times output saturated
    double computeTimeUs{0.0};      ///< Last compute time [µs]
    bool healthy{true};             ///< Controller health status
    std::string statusMessage;      ///< Status/error message
};

// ============================================================================
// Abstract Controller Base Class
// ============================================================================

/**
 * @brief Abstract base class for all controllers
 * 
 * This class defines the interface that all controllers must implement.
 * It provides common functionality for configuration, execution, and
 * diagnostics.
 * 
 * Implementing a Custom Controller:
 * 
 * Create a class that inherits from ControllerBase and implements
 * getType(), getName(), computeImpl(), and resetImpl() methods.
 * In computeImpl(), implement your control law and return the output.
 */
class ControllerBase {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~ControllerBase() = default;
    
    // ========================================================================
    // Type Information
    // ========================================================================
    
    /**
     * @brief Get controller type
     */
    virtual ControllerType getType() const = 0;
    
    /**
     * @brief Get controller name
     */
    virtual const char* getName() const = 0;
    
    /**
     * @brief Get detailed description
     */
    virtual const char* getDescription() const { return ""; }
    
    // ========================================================================
    // Main Interface
    // ========================================================================
    
    /**
     * @brief Compute control output
     * 
     * Main entry point for control computation. Handles mode switching,
     * saturation, and diagnostics before calling the implementation.
     * 
     * @param input Controller input data
     * @return Control output
     */
    ControllerOutput compute(const ControllerInput& input);
    
    /**
     * @brief Reset controller state
     * 
     * Clears integrators, filters, and other internal state.
     */
    void reset();
    
    /**
     * @brief Set controller operating mode
     */
    void setMode(ControllerMode mode) { m_mode = mode; }
    
    /**
     * @brief Get current operating mode
     */
    ControllerMode getMode() const { return m_mode; }
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set output saturation limits
     */
    void setSaturationLimits(const SaturationLimits& limits) {
        m_limits = limits;
    }
    
    /**
     * @brief Get current saturation limits
     */
    const SaturationLimits& getSaturationLimits() const { return m_limits; }
    
    /**
     * @brief Set manual output value (for Manual mode)
     */
    void setManualOutput(double output) { m_manualOutput = output; }
    
    /**
     * @brief Get manual output value
     */
    double getManualOutput() const { return m_manualOutput; }
    
    /**
     * @brief Enable/disable controller
     */
    void setEnabled(bool enabled) { 
        m_mode = enabled ? ControllerMode::Automatic : ControllerMode::Disabled;
    }
    
    /**
     * @brief Check if controller is enabled
     */
    bool isEnabled() const { return m_mode != ControllerMode::Disabled; }
    
    // ========================================================================
    // Backend Controller (for composition)
    // ========================================================================
    
    /**
     * @brief Set a backend controller
     * 
     * Allows this controller to use another controller as its backend,
     * enabling hierarchical/cascade control structures.
     * 
     * @param backend Shared pointer to backend controller
     */
    virtual void setBackendController(ControllerPtr backend) {
        m_backendController = backend;
    }
    
    /**
     * @brief Get backend controller
     */
    ControllerPtr getBackendController() const { return m_backendController; }
    
    /**
     * @brief Check if controller has a backend
     */
    bool hasBackend() const { return m_backendController != nullptr; }
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get diagnostic information
     */
    const ControllerDiagnostics& getDiagnostics() const { return m_diagnostics; }
    
    /**
     * @brief Reset diagnostics counters
     */
    void resetDiagnostics();
    
    /**
     * @brief Get last output
     */
    const ControllerOutput& getLastOutput() const { return m_lastOutput; }
    
protected:
    /**
     * @brief Implementation-specific compute (must override)
     */
    virtual ControllerOutput computeImpl(const ControllerInput& input) = 0;
    
    /**
     * @brief Implementation-specific reset (must override)
     */
    virtual void resetImpl() = 0;
    
    /**
     * @brief Apply saturation to output
     */
    double saturate(double value) const;
    
    /**
     * @brief Apply rate limiting
     */
    double rateLimit(double value, double dt);
    
    /**
     * @brief Update diagnostics
     */
    void updateDiagnostics(const ControllerInput& input, 
                          const ControllerOutput& output);
    
    // Member variables
    ControllerMode m_mode{ControllerMode::Automatic};
    SaturationLimits m_limits;
    double m_manualOutput{0.0};
    double m_lastOutputValue{0.0};
    ControllerOutput m_lastOutput;
    ControllerDiagnostics m_diagnostics;
    ControllerPtr m_backendController;
};

// ============================================================================
// State Estimator Interface
// ============================================================================

/**
 * @brief Abstract state estimator for observer-based control
 * 
 * Used by LQG and other observer-based controllers to estimate
 * unmeasured states from available measurements.
 */
class StateEstimator {
public:
    virtual ~StateEstimator() = default;
    
    /**
     * @brief Update state estimate
     * 
     * @param measurement Current measurement
     * @param control Control input applied
     * @param dt Time step
     * @return Updated state estimate
     */
    virtual StateVector estimate(const OutputVector& measurement,
                                 const ControlVector& control,
                                 double dt) = 0;
    
    /**
     * @brief Get current state estimate
     */
    virtual StateVector getState() const = 0;
    
    /**
     * @brief Reset estimator
     */
    virtual void reset() = 0;
    
    /**
     * @brief Get state dimension
     */
    virtual size_t getStateDim() const = 0;
};

// ============================================================================
// System Model Interface
// ============================================================================

/**
 * @brief Abstract system model for model-based control
 * 
 * Represents a linear time-invariant (LTI) system:
 *   ẋ = Ax + Bu
 *   y = Cx + Du
 */
class SystemModel {
public:
    virtual ~SystemModel() = default;
    
    /**
     * @brief Get state dimension
     */
    virtual size_t getStateDim() const = 0;
    
    /**
     * @brief Get input dimension
     */
    virtual size_t getInputDim() const = 0;
    
    /**
     * @brief Get output dimension
     */
    virtual size_t getOutputDim() const = 0;
    
    /**
     * @brief Get A matrix (state transition)
     */
    virtual const StateMatrix& getA() const = 0;
    
    /**
     * @brief Get B matrix (input)
     */
    virtual const InputMatrix& getB() const = 0;
    
    /**
     * @brief Get C matrix (output)
     */
    virtual const OutputMatrix& getC() const = 0;
    
    /**
     * @brief Get D matrix (feedthrough)
     */
    virtual const FeedthroughMatrix& getD() const = 0;
    
    /**
     * @brief Simulate one step
     * 
     * @param state Current state
     * @param control Control input
     * @param dt Time step
     * @return Next state
     */
    virtual StateVector simulate(const StateVector& state,
                                const ControlVector& control,
                                double dt) const = 0;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Matrix-vector multiplication
 */
template<size_t Rows, size_t Cols>
std::array<double, Rows> matVecMul(const Matrix<Rows, Cols>& mat,
                                    const std::array<double, Cols>& vec) {
    std::array<double, Rows> result{};
    for (size_t i = 0; i < Rows; ++i) {
        for (size_t j = 0; j < Cols; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}

/**
 * @brief Vector addition
 */
template<size_t N>
std::array<double, N> vecAdd(const std::array<double, N>& a,
                              const std::array<double, N>& b) {
    std::array<double, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

/**
 * @brief Vector subtraction
 */
template<size_t N>
std::array<double, N> vecSub(const std::array<double, N>& a,
                              const std::array<double, N>& b) {
    std::array<double, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

/**
 * @brief Scalar-vector multiplication
 */
template<size_t N>
std::array<double, N> vecScale(const std::array<double, N>& vec, double scale) {
    std::array<double, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = vec[i] * scale;
    }
    return result;
}

/**
 * @brief Vector dot product
 */
template<size_t N>
double vecDot(const std::array<double, N>& a, const std::array<double, N>& b) {
    double result = 0.0;
    for (size_t i = 0; i < N; ++i) {
        result += a[i] * b[i];
    }
    return result;
}

/**
 * @brief Vector norm (Euclidean)
 */
template<size_t N>
double vecNorm(const std::array<double, N>& vec) {
    return std::sqrt(vecDot(vec, vec));
}

/**
 * @brief Sign function
 */
inline double sign(double x) {
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

/**
 * @brief Clamp value to range
 */
inline double clamp(double value, double min, double max) {
    return std::max(min, std::min(max, value));
}

/**
 * @brief Dead band function
 */
inline double deadband(double value, double threshold) {
    if (std::abs(value) < threshold) return 0.0;
    return value - sign(value) * threshold;
}

} // namespace Control
