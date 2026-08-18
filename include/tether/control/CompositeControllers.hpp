/**
 * @file CompositeControllers.hpp
 * @brief Composite Controllers: Cascade, Parallel, Feedforward, Switching
 * 
 * @details
 * This file provides wrapper controllers that combine multiple controllers
 * in various configurations to achieve advanced control structures.
 * 
 * ## Cascade Control
 * 
 * ### Architecture
 * ```
 *                  ┌───────────────────────────────────────┐
 *   r(t) ─────────►│ Outer      u_outer    Inner     u    │───► Plant
 *                  │ Controller ────────► Controller      │
 *          ┌──────►│                ▲                     │
 *          │       └────────────────┼─────────────────────┘
 *          │                        │
 *   y2 ◄───┼────────────────────────┘ (inner measurement)
 *          │
 *   y1 ◄───┴─────────────────────────── (outer measurement)
 * ```
 * 
 * ### Benefits
 * - Disturbance rejection before affecting outer loop
 * - Faster response on inner loop
 * - Simplifies tuning (tune inner first, then outer)
 * 
 * ### Requirements
 * - Inner loop must be faster than outer (4-10× bandwidth)
 * - Need measurement of inner variable
 * 
 * ### Example: Position/Velocity Cascade
 * ```cpp
 * CascadeController cascade;
 * cascade.setOuterController(&positionPID);  // Slow position loop
 * cascade.setInnerController(&velocityPID);  // Fast velocity loop
 *
 * ControllerInput input;
 * input.reference = targetPosition;
 * input.measured = currentPosition;
 * input.auxMeasured[0] = currentVelocity;  // Inner measurement
 *
 * auto output = cascade.compute(input);
 * ```
 *
 * @warning Composite controllers store non-owning raw pointers to child
 *          controllers. The caller is responsible for ensuring that child
 *          controllers outlive the composite. Do NOT reconfigure (call
 *          setOuterController/setInnerController/etc.) while compute() is
 *          running from another thread — composite controllers are not
 *          thread-safe for concurrent reconfiguration + computation.
 * 
 * ## Feedforward Control
 * 
 * ### Architecture
 * ```
 *   r(t) ───┬──► Feedforward ──────┐
 *           │                      ▼
 *           └──► Feedback    ───► + ───► u(t)
 *                    ▲
 *                    │
 *   y(t) ◄───────────┘
 * ```
 * 
 * ### Benefits
 * - Immediate response to reference changes
 * - Feedback handles disturbances and model errors
 * - 2-DOF control structure
 * 
 * ## Parallel Control
 * 
 * Multiple controllers operating in parallel, outputs summed.
 * Useful for multi-rate control, combining fast/slow controllers.
 * 
 * ## Switching Control
 * 
 * Select different controllers based on operating conditions.
 * Includes bumpless transfer to prevent transients.
 * 
 * @see ControllerBase
 */

#pragma once

#include "ControllerBase.hpp"
#include <vector>
#include <functional>
#include <memory>

namespace tether::control {

// ============================================================================
// Cascade Controller
// ============================================================================

/**
 * @brief Cascade (Master-Slave) Controller
 * 
 * Implements two-loop cascade control where the outer controller's
 * output becomes the reference for the inner controller.
 * 
 * ## Configuration
 * - Outer controller: typically slower (position, level, etc.)
 * - Inner controller: typically faster (velocity, flow, etc.)
 * 
 * ## Usage
 * ```cpp
 * CascadeController cascade;
 * 
 * PIDController outerPID, innerPID;
 * outerPID.setGains(2.0, 0.1, 0.5);  // Position loop
 * innerPID.setGains(5.0, 1.0, 0.1);  // Velocity loop (faster)
 * 
 * cascade.setOuterController(&outerPID);
 * cascade.setInnerController(&innerPID);
 * 
 * // During control
 * ControllerInput input;
 * input.reference = targetPosition;
 * input.measured = currentPosition;
 * input.auxMeasured[0] = currentVelocity;
 * 
 * auto output = cascade.compute(input);
 * motor.setTorque(output.control);
 * ```
 */
class CascadeController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Cascade; }
    const char* getName() const override { return "Cascade Controller"; }
    const char* getDescription() const override {
        return "Two-loop cascade: outer loop output is inner loop reference. "
               "Use auxMeasured[0] for inner measurement. Inner loop should be 4-10× faster.";
    }
    
    /**
     * @brief Set outer (master) controller
     * @param controller Pointer to outer controller (ownership not transferred)
     */
    void setOuterController(ControllerBase* controller) { m_outer = controller; }
    
    /**
     * @brief Set inner (slave) controller
     * @param controller Pointer to inner controller (ownership not transferred)
     */
    void setInnerController(ControllerBase* controller) { m_inner = controller; }
    
    /**
     * @brief Set inner loop reference limits
     * @param min Minimum inner reference
     * @param max Maximum inner reference
     */
    void setInnerReferenceLimits(double min, double max);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ControllerBase* m_outer{nullptr};
    ControllerBase* m_inner{nullptr};
    double m_innerRefMin{-1e10};
    double m_innerRefMax{1e10};
};

// ============================================================================
// Feedforward Controller
// ============================================================================

/**
 * @brief Feedforward + Feedback Controller
 * 
 * Combines model-based feedforward with feedback for optimal tracking.
 * 
 * ## Design
 * Feedforward: u_ff = G_ff(s) × r(s)
 * For perfect tracking: G_ff = P⁻¹ (plant inverse)
 * 
 * ## Usage
 * ```cpp
 * FeedforwardController ff;
 * ff.setFeedback(&pid);
 * 
 * // Custom feedforward function (e.g., acceleration feedforward)
 * ff.setFeedforwardFunction([](const ControllerInput& in) {
 *     return in.referenceDerivative * inertia;  // τ_ff = J × α_desired
 * });
 * 
 * // Or use gain-based feedforward
 * ff.setFeedforwardGain(1.5);  // Simple proportional feedforward
 * ```
 */
class FeedforwardController : public ControllerBase {
public:
    using FeedforwardFunction = std::function<double(const ControllerInput&)>;
    
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Feedforward + Feedback"; }
    const char* getDescription() const override {
        return "2-DOF control: model-based feedforward for tracking + feedback for "
               "disturbance rejection. Set custom FF function or use simple gain.";
    }
    
    /**
     * @brief Set feedback controller
     * @param controller Pointer to feedback controller
     */
    void setFeedback(ControllerBase* controller) { m_feedback = controller; }
    
    /**
     * @brief Set feedforward function
     * @param func Function computing feedforward from input
     */
    void setFeedforwardFunction(FeedforwardFunction func) { m_ffFunc = func; }
    
    /**
     * @brief Set simple proportional feedforward gain
     * @param gain Feedforward gain (u_ff = gain × reference)
     */
    void setFeedforwardGain(double gain) { m_ffGain = gain; m_useSimpleFF = true; }
    
    /**
     * @brief Set velocity feedforward gain
     * @param gain Velocity FF gain (u_ff += gain × referenceDerivative)
     */
    void setVelocityFeedforward(double gain) { m_velFFGain = gain; }
    
    /**
     * @brief Set acceleration feedforward gain
     * @param gain Acceleration FF gain (u_ff += gain × referenceAccel)
     */
    void setAccelerationFeedforward(double gain) { m_accelFFGain = gain; }
    
    /**
     * @brief Set feedforward/feedback mixing ratio
     * @param ratio 0=pure FB, 1=pure FF
     */
    void setMixingRatio(double ratio) { m_mixRatio = std::clamp(ratio, 0.0, 1.0); }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ControllerBase* m_feedback{nullptr};
    FeedforwardFunction m_ffFunc{nullptr};
    bool m_useSimpleFF{false};
    double m_ffGain{1.0};
    double m_velFFGain{0.0};
    double m_accelFFGain{0.0};
    double m_mixRatio{0.5};
};

// ============================================================================
// Parallel Controller
// ============================================================================

/**
 * @brief Parallel Controller Combination
 * 
 * Runs multiple controllers in parallel and sums their outputs.
 * Useful for:
 * - Multi-rate control (fast + slow controller)
 * - Adding specialized controllers (notch filter + PID)
 * - Redundancy
 * 
 * ## Usage
 * ```cpp
 * ParallelController parallel;
 * 
 * PIDController slow, fast;
 * slow.setGains(1.0, 0.1, 0.0);  // Slow integral action
 * fast.setGains(5.0, 0.0, 1.0);  // Fast PD action
 * 
 * parallel.addController(&slow, 1.0);  // Weight 1.0
 * parallel.addController(&fast, 0.5);  // Weight 0.5
 * ```
 */
class ParallelController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Parallel Controller"; }
    const char* getDescription() const override {
        return "Runs multiple controllers in parallel, weighted sum of outputs. "
               "Good for multi-rate control, combining specialized controllers.";
    }
    
    /**
     * @brief Add a controller to the parallel structure
     * @param controller Pointer to controller
     * @param weight Output weight for this controller
     */
    void addController(ControllerBase* controller, double weight = 1.0);
    
    /**
     * @brief Remove all controllers
     */
    void clearControllers();
    
    /**
     * @brief Set weight for specific controller
     * @param index Controller index
     * @param weight New weight
     */
    void setWeight(size_t index, double weight);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    struct ControllerEntry {
        ControllerBase* controller;
        double weight;
    };
    std::vector<ControllerEntry> m_controllers;
};

// ============================================================================
// Switching Controller
// ============================================================================

/**
 * @brief Switching Controller with Bumpless Transfer
 * 
 * Selects between multiple controllers based on operating conditions.
 * Implements bumpless transfer to avoid output discontinuities.
 * 
 * ## Use Cases
 * - Gain scheduling based on operating point
 * - Different controllers for different modes
 * - Fault-tolerant control (switch on sensor failure)
 * 
 * ## Bumpless Transfer Methods
 * - Output tracking: backup controllers track active output
 * - State initialization: transfer internal states
 * 
 * ## Usage
 * ```cpp
 * SwitchingController sw;
 * 
 * sw.addController(&lowSpeedPID, [](const ControllerInput& in) {
 *     return in.measured < 100.0;  // Use for speed < 100
 * });
 * 
 * sw.addController(&highSpeedPID, [](const ControllerInput& in) {
 *     return in.measured >= 100.0;  // Use for speed >= 100
 * });
 * 
 * sw.enableBumplessTransfer(true);
 * ```
 */
class SwitchingController : public ControllerBase {
public:
    using SelectionFunction = std::function<bool(const ControllerInput&)>;
    
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "Switching Controller"; }
    const char* getDescription() const override {
        return "Selects between controllers based on conditions. Supports bumpless "
               "transfer. Good for gain scheduling, mode-dependent control.";
    }
    
    /**
     * @brief Add controller with selection condition
     * @param controller Pointer to controller
     * @param condition Function returning true when this controller should be active
     * @param priority Priority (higher = checked first)
     */
    void addController(ControllerBase* controller, SelectionFunction condition, int priority = 0);
    
    /**
     * @brief Set default controller (when no conditions match)
     */
    void setDefaultController(ControllerBase* controller) { m_default = controller; }
    
    /**
     * @brief Enable/disable bumpless transfer
     */
    void enableBumplessTransfer(bool enable) { m_bumpless = enable; }
    
    /**
     * @brief Set bumpless transfer time constant
     * @param tau Time constant [s] for output blending
     */
    void setBumplessTimeConstant(double tau) { m_bumplessTau = tau; }
    
    /**
     * @brief Get index of currently active controller
     */
    int getActiveIndex() const { return m_activeIndex; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    struct ControllerEntry {
        ControllerBase* controller;
        SelectionFunction condition;
        int priority;
        
        ControllerEntry& operator=(const ControllerEntry&) = default;
    };
    std::vector<ControllerEntry> m_controllers;
    ControllerBase* m_default{nullptr};
    
    bool m_bumpless{true};
    double m_bumplessTau{0.1};
    int m_activeIndex{-1};
    int m_prevActiveIndex{-1};
    double m_blendFactor{1.0};
    double m_prevOutput{0.0};
};

// ============================================================================
// Rate Limiter Wrapper
// ============================================================================

/**
 * @brief Rate Limiter Wrapper
 * 
 * Wraps any controller and limits output rate of change.
 * Useful for protecting actuators, reducing wear.
 * 
 * ## Usage
 * ```cpp
 * RateLimiterWrapper limited;
 * limited.setController(&myPID);
 * limited.setRateLimits(-10.0, 10.0);  // ±10 units/second
 * ```
 */
class RateLimiterWrapper : public ControllerBase {
public:
    ControllerType getType() const override { 
        return m_inner ? m_inner->getType() : ControllerType::Custom; 
    }
    const char* getName() const override { return "Rate-Limited Controller"; }
    const char* getDescription() const override {
        return "Wraps any controller with output rate limiting. "
               "Protects actuators from excessive rate of change.";
    }
    
    /**
     * @brief Set inner controller
     */
    void setController(ControllerBase* controller) { m_inner = controller; }
    
    /**
     * @brief Set rate limits
     * @param minRate Minimum rate of change [units/s]
     * @param maxRate Maximum rate of change [units/s]
     */
    void setRateLimits(double minRate, double maxRate);
    
    /**
     * @brief Set symmetric rate limit
     * @param maxRate Maximum absolute rate
     */
    void setRateLimit(double maxRate) { setRateLimits(-maxRate, maxRate); }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ControllerBase* m_inner{nullptr};
    double m_minRate{-1e10};
    double m_maxRate{1e10};
    double m_prevOutput{0.0};
    bool m_firstSample{true};
};

// ============================================================================
// Deadband Wrapper
// ============================================================================

/**
 * @brief Deadband/Hysteresis Wrapper
 * 
 * Adds deadband to error or output to reduce chatter.
 * 
 * ## Deadband Types
 * - Error deadband: ignore small errors
 * - Output deadband: zero output for small values
 * - Hysteresis: different thresholds for increasing/decreasing
 * 
 * ## Usage
 * ```cpp
 * DeadbandWrapper db;
 * db.setController(&myPID);
 * db.setErrorDeadband(0.1);     // Ignore errors < 0.1
 * db.setOutputDeadband(0.5);    // Zero output if |u| < 0.5
 * ```
 */
class DeadbandWrapper : public ControllerBase {
public:
    ControllerType getType() const override { 
        return m_inner ? m_inner->getType() : ControllerType::Custom; 
    }
    const char* getName() const override { return "Deadband Controller"; }
    const char* getDescription() const override {
        return "Adds deadband to error or output to reduce actuator chatter. "
               "Useful for digital valves, motors with stiction.";
    }
    
    void setController(ControllerBase* controller) { m_inner = controller; }
    
    /**
     * @brief Set error deadband
     * @param deadband Errors smaller than this are treated as zero
     */
    void setErrorDeadband(double deadband) { m_errorDeadband = std::fabs(deadband); }
    
    /**
     * @brief Set output deadband
     * @param deadband Outputs smaller than this become zero
     */
    void setOutputDeadband(double deadband) { m_outputDeadband = std::fabs(deadband); }
    
    /**
     * @brief Set hysteresis band
     * @param hysteresis Additional band for direction reversal
     */
    void setHysteresis(double hysteresis) { m_hysteresis = std::fabs(hysteresis); }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ControllerBase* m_inner{nullptr};
    double m_errorDeadband{0.0};
    double m_outputDeadband{0.0};
    double m_hysteresis{0.0};
    double m_prevOutput{0.0};
};

// ============================================================================
// Filter Wrapper
// ============================================================================

/**
 * @brief Low-Pass Filter Wrapper
 * 
 * Adds low-pass filtering to controller output.
 * Reduces high-frequency noise, smooths actuator commands.
 * 
 * ## Usage
 * ```cpp
 * FilterWrapper filtered;
 * filtered.setController(&myPID);
 * filtered.setFilterCutoff(100.0);  // 100 Hz cutoff
 * filtered.setFilterOrder(2);        // Second-order filter
 * ```
 */
class FilterWrapper : public ControllerBase {
public:
    ControllerType getType() const override { 
        return m_inner ? m_inner->getType() : ControllerType::Custom; 
    }
    const char* getName() const override { return "Filtered Controller"; }
    const char* getDescription() const override {
        return "Adds low-pass filtering to controller output. "
               "Reduces noise, smooths actuator commands.";
    }
    
    void setController(ControllerBase* controller) { m_inner = controller; }
    
    /**
     * @brief Set filter cutoff frequency
     * @param cutoffHz Cutoff frequency [Hz]
     */
    void setFilterCutoff(double cutoffHz) { m_cutoff = cutoffHz; }
    
    /**
     * @brief Set filter order (1 or 2)
     */
    void setFilterOrder(int order) { m_order = std::clamp(order, 1, 2); }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ControllerBase* m_inner{nullptr};
    double m_cutoff{100.0};
    int m_order{1};
    
    // Filter states
    double m_state1{0.0};
    double m_state2{0.0};
};

// ============================================================================
// Controller Factory
// ============================================================================

/**
 * @brief Factory for creating controllers by type
 * 
 * Provides a simple interface for creating controllers.
 * Useful for configuration-driven systems.
 */
class ControllerFactory {
public:
    /**
     * @brief Create controller by type
     * @param type Controller type
     * @return Unique pointer to new controller
     */
    static std::unique_ptr<ControllerBase> create(ControllerType type);
    
    /**
     * @brief Create controller by name
     * @param name Controller name (e.g., "PID", "LQR", "P-Type ILC")
     * @return Unique pointer to new controller
     */
    static std::unique_ptr<ControllerBase> create(const char* name);
};

} // namespace tether::control
