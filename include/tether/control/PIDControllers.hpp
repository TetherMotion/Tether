/**
 * @file PIDControllers.hpp
 * @brief PID Controller Family - P, PD, PI, PID, PID-2DOF, PD+
 * 
 * @details
 * This file implements the complete family of PID-based controllers with
 * extensive documentation, multiple anti-windup methods, and advanced features.
 * 
 * ## PID Control Theory
 * 
 * The PID controller is the most widely used feedback controller in industry.
 * It computes a control signal based on error (e = setpoint - measured):
 * 
 * ```
 * u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·(de/dt)
 * 
 * Where:
 *   Kp = Proportional gain (immediate response)
 *   Ki = Integral gain (eliminate steady-state error)
 *   Kd = Derivative gain (predictive/damping action)
 * ```
 * 
 * ## Discrete Implementation
 * 
 * For digital control, we use discrete approximations:
 * 
 * ### Integral (Tustin/Bilinear)
 * ```
 * I[k] = I[k-1] + Ki·(T/2)·(e[k] + e[k-1])
 * ```
 * 
 * ### Derivative (Backward Difference with Filter)
 * ```
 * D[k] = (Tf·D[k-1] + Kd·(e[k] - e[k-1])) / (Tf + T)
 * 
 * Where Tf = derivative filter time constant
 * ```
 * 
 * ## Controller Variants
 * 
 * ### P Controller (Proportional Only)
 * - Simplest controller
 * - Cannot eliminate steady-state error
 * - Good for systems with natural damping
 * 
 * ### PD Controller
 * - Adds predictive capability via derivative
 * - Improves transient response and stability
 * - Still has steady-state error
 * 
 * ### PI Controller  
 * - Eliminates steady-state error
 * - Most common in process control
 * - Can cause overshoot/oscillation
 * 
 * ### PID Controller
 * - Full three-term control
 * - Best overall performance
 * - Requires careful tuning
 * 
 * ### PID-2DOF (Two Degrees of Freedom)
 * - Separate response to setpoint vs disturbance
 * - Uses setpoint weights (b for P, c for D)
 * - Reduces overshoot on setpoint changes
 * 
 * ### PD+ (PD with Compensation)
 * - PD control with feedforward compensation
 * - Commonly adds gravity/friction compensation
 * - Popular in robotics
 * 
 * ## Anti-Windup Methods
 * 
 * ### Problem: Integrator Windup
 * When the actuator saturates, the integrator continues accumulating error,
 * causing large overshoot when the error changes sign.
 * 
 * ### Solutions Implemented:
 * 
 * 1. **Clamping**: Stop integration when output saturates
 *    ```cpp
 *    if (!saturated) integral += Ki * error * dt;
 *    ```
 * 
 * 2. **Back-Calculation**: Feed back saturation difference
 *    ```cpp
 *    integral += (Ki * error + Kt * (saturated_u - u)) * dt;
 *    ```
 * 
 * 3. **Conditional Integration**: Integrate only when helpful
 *    ```cpp
 *    if (sign(error) != sign(output)) integral += Ki * error * dt;
 *    ```
 * 
 * 4. **Tracking**: Track external feedback signal
 * 
 * ## Tuning Guidelines
 * 
 * ### Ziegler-Nichols (Ultimate Gain Method)
 * 1. Set Ki = Kd = 0
 * 2. Increase Kp until sustained oscillation
 * 3. Record Ku (ultimate gain) and Tu (period)
 * 
 * | Type | Kp        | Ki           | Kd          |
 * |------|-----------|--------------|-------------|
 * | P    | 0.5·Ku    | -            | -           |
 * | PI   | 0.45·Ku   | 1.2·Kp/Tu    | -           |
 * | PID  | 0.6·Ku    | 2·Kp/Tu      | Kp·Tu/8     |
 * 
 * ### Cohen-Coon (Process Reaction Curve)
 * Better for processes with significant dead time.
 * 
 * ### Lambda Tuning (IMC-based)
 * ```
 * Kp = τ / (K·λ)
 * Ki = Kp / τ  
 * Kd = 0 (or small)
 * 
 * Where:
 *   K = process gain
 *   τ = process time constant
 *   λ = desired closed-loop time constant
 * ```
 * 
 * ## Usage Examples
 * 
 * ### Basic PID
 * ```cpp
 * PIDController pid;
 * pid.setGains(1.0, 0.1, 0.05);
 * pid.setDerivativeFilter(0.01);  // 10ms filter
 * pid.setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
 * pid.setSaturationLimits({-100, 100, -50, 50});
 * 
 * // In control loop
 * ControllerInput input;
 * input.reference = targetPosition;
 * input.measured = actualPosition;
 * input.dt = 0.001;  // 1ms
 * 
 * ControllerOutput output = pid.compute(input);
 * motor.setTorque(output.control);
 * ```
 * 
 * ### Two Degrees of Freedom PID
 * ```cpp
 * PID2DOFController pid2dof;
 * pid2dof.setGains(1.0, 0.1, 0.05);
 * pid2dof.setSetpointWeights(0.7, 0.0);  // b=0.7, c=0
 * // b < 1 reduces overshoot on setpoint changes
 * // c = 0 applies derivative only to measurement
 * ```
 * 
 * ### PD+ for Robot Joint
 * ```cpp
 * PDPlusController pdplus;
 * pdplus.setGains(100.0, 10.0);  // Kp, Kd
 * pdplus.setCompensationCallback([](double pos) {
 *     // Gravity compensation for robot arm
 *     return 9.81 * mass * linkLength * sin(pos);
 * });
 * ```
 * 
 * @see ControllerBase
 * @see DualLoopPIDController
 * @see FractionalPIDController
 */

#pragma once

#include "ControllerBase.hpp"

namespace tether::control {

// ============================================================================
// P Controller
// ============================================================================

/**
 * @brief Pure Proportional Controller
 * 
 * ## Control Law
 * ```
 * u = Kp · e
 * ```
 * 
 * ## Characteristics
 * - Simplest controller
 * - Output proportional to error
 * - Cannot eliminate steady-state error (unless Kp → ∞)
 * - Very stable, easy to tune
 * 
 * ## When to Use
 * - Systems with inherent damping
 * - When steady-state error is acceptable
 * - As inner loop of cascade control
 * - When simplicity is paramount
 * 
 * ## Example
 * ```cpp
 * PController p;
 * p.setGain(2.0);  // Proportional gain
 * 
 * ControllerInput input{.reference = 100, .measured = 90, .dt = 0.001};
 * auto output = p.compute(input);
 * // output.control = 2.0 * (100 - 90) = 20
 * ```
 */
class PController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::P; }
    const char* getName() const override { return "P Controller"; }
    const char* getDescription() const override {
        return "Proportional controller: u = Kp·e. Simple, stable, but has "
               "steady-state error. Use for systems with natural damping.";
    }
    
    /**
     * @brief Set proportional gain
     * @param kp Proportional gain (positive)
     */
    void setGain(double kp) { m_kp = kp; }
    
    /**
     * @brief Get proportional gain
     */
    double getGain() const { return m_kp; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override {}
    
private:
    double m_kp{1.0};
};

// ============================================================================
// PD Controller
// ============================================================================

/**
 * @brief Proportional-Derivative Controller
 * 
 * ## Control Law
 * ```
 * u = Kp · e + Kd · (de/dt)
 * ```
 * 
 * ## With Derivative Filter (recommended)
 * ```
 *           Kd · s
 * D(s) = ──────────
 *         τf·s + 1
 * ```
 * 
 * ## Characteristics
 * - Proportional gives immediate response
 * - Derivative provides prediction/damping
 * - No integral → steady-state error remains
 * - More responsive than P alone
 * - Can amplify high-frequency noise (use filter!)
 * 
 * ## Derivative on Measurement vs Error
 * - **On Error**: Responds to setpoint changes (derivative kick)
 * - **On Measurement**: Smoother response, no kick on setpoint change
 * 
 * ## When to Use
 * - Position control without integral windup concerns
 * - Fast response needed, some error acceptable
 * - Systems where derivative action improves damping
 * 
 * ## Example
 * ```cpp
 * PDController pd;
 * pd.setGains(10.0, 0.5);        // Kp=10, Kd=0.5
 * pd.setDerivativeFilter(0.01);   // 10ms filter
 * pd.setDerivativeOnMeasurement(true);  // Avoid derivative kick
 * ```
 */
class PDController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::PD; }
    const char* getName() const override { return "PD Controller"; }
    const char* getDescription() const override {
        return "Proportional-Derivative controller: u = Kp·e + Kd·de/dt. "
               "Adds predictive action. Good for position control.";
    }
    
    /**
     * @brief Set gains
     * @param kp Proportional gain
     * @param kd Derivative gain
     */
    void setGains(double kp, double kd);
    
    /**
     * @brief Get proportional gain
     */
    double getKp() const { return m_kp; }
    
    /**
     * @brief Get derivative gain
     */
    double getKd() const { return m_kd; }
    
    /**
     * @brief Set derivative filter time constant
     * @param tf Filter time constant [s] (0 = no filter)
     * 
     * Recommended: tf = Kd / (5 to 10 × Kp)
     */
    void setDerivativeFilter(double tf) { m_tf = tf; }
    
    /**
     * @brief Get derivative filter time constant
     */
    double getDerivativeFilter() const { return m_tf; }
    
    /**
     * @brief Set derivative to act on measurement
     * 
     * When true, derivative is calculated from measurement only,
     * avoiding "derivative kick" on setpoint changes.
     * 
     * @param onMeasurement true = derivative on measurement
     */
    void setDerivativeOnMeasurement(bool onMeasurement) {
        m_derivOnMeasurement = onMeasurement;
    }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    double m_kp{1.0};
    double m_kd{0.0};
    double m_tf{0.0};  // Derivative filter time constant
    
    double m_lastError{0.0};
    double m_lastMeasurement{0.0};
    double m_filteredDerivative{0.0};
    bool m_derivOnMeasurement{true};
    bool m_firstRun{true};
};

// ============================================================================
// PI Controller
// ============================================================================

/**
 * @brief Proportional-Integral Controller
 * 
 * ## Control Law
 * ```
 * u = Kp · e + Ki · ∫e dt
 * ```
 * 
 * ## Transfer Function
 * ```
 *              Ki
 * C(s) = Kp + ──
 *              s
 * ```
 * 
 * ## Characteristics
 * - Proportional provides immediate response
 * - Integral eliminates steady-state error
 * - Most common controller in process industry
 * - Prone to integrator windup
 * 
 * ## Integral Time (Ti) vs Integral Gain (Ki)
 * ```
 * Ki = Kp / Ti
 * 
 * Ti = time for integral to equal proportional contribution
 *      given constant error
 * ```
 * 
 * ## Anti-Windup is Critical!
 * Without anti-windup, integral can "wind up" during saturation,
 * causing large overshoot when error changes sign.
 * 
 * ## When to Use
 * - Zero steady-state error required
 * - Derivative action not needed or too noisy
 * - Level, flow, and pressure control
 * 
 * ## Example
 * ```cpp
 * PIController pi;
 * pi.setGains(1.0, 0.2);  // Kp=1.0, Ki=0.2
 * 
 * // Or using time constant form
 * pi.setGainsFromTi(1.0, 5.0);  // Kp=1.0, Ti=5s → Ki=0.2
 * 
 * pi.setAntiWindup(AntiWindupMethod::BackCalculation, 0.5);
 * pi.setIntegralLimits(-100, 100);
 * ```
 */
class PIController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::PI; }
    const char* getName() const override { return "PI Controller"; }
    const char* getDescription() const override {
        return "Proportional-Integral controller: u = Kp·e + Ki·∫e·dt. "
               "Eliminates steady-state error. Most common industrial controller.";
    }
    
    /**
     * @brief Set gains directly
     * @param kp Proportional gain
     * @param ki Integral gain
     */
    void setGains(double kp, double ki);
    
    /**
     * @brief Set gains using integral time constant
     * @param kp Proportional gain  
     * @param ti Integral time [s] (Ki = Kp/Ti)
     */
    void setGainsFromTi(double kp, double ti);
    
    /**
     * @brief Get proportional gain
     */
    double getKp() const { return m_kp; }
    
    /**
     * @brief Get integral gain
     */
    double getKi() const { return m_ki; }
    
    /**
     * @brief Set integral limits
     */
    void setIntegralLimits(double min, double max);
    
    /**
     * @brief Set anti-windup method
     * @param method Anti-windup method
     * @param param Method-specific parameter (e.g., tracking gain for back-calc)
     */
    void setAntiWindup(AntiWindupMethod method, double param = 0.0);
    
    /**
     * @brief Get current integral value
     */
    double getIntegral() const { return m_integral; }
    
    /**
     * @brief Set integral to specific value (for bumpless transfer)
     */
    void setIntegral(double value) { m_integral = value; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    double m_kp{1.0};
    double m_ki{0.0};
    double m_integral{0.0};
    double m_integralMin{-std::numeric_limits<double>::max()};
    double m_integralMax{std::numeric_limits<double>::max()};
    
    AntiWindupMethod m_antiWindup{AntiWindupMethod::Clamping};
    double m_antiWindupParam{0.0};
    double m_lastError{0.0};
    double m_lastOutput{0.0};
};

// ============================================================================
// PID Controller
// ============================================================================

/**
 * @brief Full Proportional-Integral-Derivative Controller
 * 
 * ## Control Law
 * ```
 * u = Kp · e + Ki · ∫e dt + Kd · (de/dt)
 * ```
 * 
 * ## Transfer Function (Ideal Form)
 * ```
 *              Ki
 * C(s) = Kp + ── + Kd·s
 *              s
 * ```
 * 
 * ## Transfer Function (Standard/ISA Form)
 * ```
 *              1        Td·s
 * C(s) = Kp·(1 + ──── + ────────)
 *              Ti·s   αTd·s + 1
 * 
 * Where:
 *   Ti = Integral time = Kp/Ki
 *   Td = Derivative time = Kd/Kp
 *   α = Derivative filter coefficient (typically 0.1)
 * ```
 * 
 * ## Discrete Implementation (Velocity Form)
 * 
 * The velocity (incremental) form has advantages:
 * - Bumpless transfer between modes
 * - Natural anti-windup
 * 
 * ```
 * Δu[k] = Kp·(e[k] - e[k-1])
 *       + Ki·T·e[k]  
 *       + Kd/T·(e[k] - 2e[k-1] + e[k-2])
 * 
 * u[k] = u[k-1] + Δu[k]
 * ```
 * 
 * ## Tuning Methods Implemented
 * 
 * ### Ziegler-Nichols
 * Classic method, tends to be aggressive
 * 
 * ### Cohen-Coon  
 * Better for processes with dead time
 * 
 * ### Lambda/IMC
 * Specify desired closed-loop time constant
 * 
 * ### AMIGO
 * Approximate M-constrained Integral Gain Optimization
 * 
 * ## Anti-Windup Methods
 * 
 * All methods from AntiWindupMethod enum are supported:
 * - Clamping: Simple, stops integration when saturated
 * - Back-Calculation: Smooth, uses tracking time constant
 * - Conditional: Only integrates when appropriate
 * - Tracking: Uses external feedback
 * - Observer-Based: For model-based anti-windup
 * 
 * ## Example
 * ```cpp
 * PIDController pid;
 * 
 * // Method 1: Direct gains
 * pid.setGains(1.0, 0.1, 0.05);
 * 
 * // Method 2: Time constant form
 * pid.setGainsFromTimeConstants(1.0, 10.0, 0.5);  // Kp, Ti, Td
 * 
 * // Method 3: Auto-tune from process model
 * pid.autoTune(processGain, timeConstant, deadTime, 
 *              TuningMethod::Lambda);
 * 
 * // Configure derivative
 * pid.setDerivativeFilter(0.01);
 * pid.setDerivativeOnMeasurement(true);
 * 
 * // Configure anti-windup
 * pid.setAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
 * 
 * // Set limits
 * SaturationLimits limits;
 * limits.outputMin = -100;
 * limits.outputMax = 100;
 * limits.integralMin = -50;
 * limits.integralMax = 50;
 * pid.setSaturationLimits(limits);
 * ```
 * 
 * @see PIController
 * @see PDController
 * @see PID2DOFController
 */
class PIDController : public ControllerBase {
public:
    /**
     * @brief Tuning method for auto-tuning
     */
    enum class TuningMethod {
        ZieglerNichols,     ///< Classic, aggressive
        ZieglerNicholsPI,   ///< Z-N for PI only
        ZieglerNicholsPID,  ///< Z-N for full PID
        CohenCoon,          ///< Better for dead time
        Lambda,             ///< IMC-based (specify lambda)
        AMIGO,              ///< Robustness-optimized
        SIMC,               ///< Skogestad IMC
        TyreusLuyben        ///< Conservative
    };
    
    ControllerType getType() const override { return ControllerType::PID; }
    const char* getName() const override { return "PID Controller"; }
    const char* getDescription() const override {
        return "Full PID controller: u = Kp·e + Ki·∫e·dt + Kd·de/dt. "
               "Most versatile classical controller. Supports multiple "
               "anti-windup methods and auto-tuning.";
    }
    
    // ========================================================================
    // Gain Configuration
    // ========================================================================
    
    /**
     * @brief Set gains directly (parallel form)
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(double kp, double ki, double kd);
    
    /**
     * @brief Set gains using time constants (ISA/standard form)
     * @param kp Proportional gain
     * @param ti Integral time [s] (Ki = Kp/Ti)
     * @param td Derivative time [s] (Kd = Kp*Td)
     */
    void setGainsFromTimeConstants(double kp, double ti, double td);
    
    /**
     * @brief Get gains
     */
    double getKp() const { return m_kp; }
    double getKi() const { return m_ki; }
    double getKd() const { return m_kd; }
    
    // ========================================================================
    // Auto-Tuning
    // ========================================================================
    
    /**
     * @brief Auto-tune from process model parameters
     * 
     * @param processGain Process steady-state gain (K)
     * @param timeConstant Process time constant (τ) [s]
     * @param deadTime Process dead time (θ) [s]
     * @param method Tuning method
     * @param lambda Closed-loop time constant for Lambda method [s]
     */
    void autoTune(double processGain, double timeConstant, double deadTime,
                  TuningMethod method = TuningMethod::Lambda,
                  double lambda = 0.0);
    
    /**
     * @brief Auto-tune from ultimate gain experiment
     * 
     * Use this after determining Ku and Tu experimentally:
     * 1. Set Ki = Kd = 0
     * 2. Increase Kp until sustained oscillation
     * 3. Record Ku (gain at oscillation) and Tu (period)
     * 
     * @param ultimateGain Ultimate gain Ku
     * @param ultimatePeriod Ultimate period Tu [s]
     * @param method Tuning method (Z-N variants)
     */
    void autoTuneFromUltimate(double ultimateGain, double ultimatePeriod,
                              TuningMethod method = TuningMethod::ZieglerNicholsPID);
    
    // ========================================================================
    // Derivative Configuration
    // ========================================================================
    
    /**
     * @brief Set derivative filter time constant
     * @param tf Filter time constant [s] (0 = no filter, not recommended)
     * 
     * Rule of thumb: tf = Td / 8 to Td / 20
     */
    void setDerivativeFilter(double tf) { m_tf = tf; }
    
    /**
     * @brief Get derivative filter time constant
     */
    double getDerivativeFilter() const { return m_tf; }
    
    /**
     * @brief Set derivative to act on measurement only
     * @param onMeasurement true = avoid derivative kick
     */
    void setDerivativeOnMeasurement(bool onMeasurement) {
        m_derivOnMeasurement = onMeasurement;
    }
    
    /**
     * @brief Set derivative filter type
     */
    void setDerivativeFilterType(DerivativeFilterType type) {
        m_derivFilterType = type;
    }
    
    // ========================================================================
    // Anti-Windup Configuration
    // ========================================================================
    
    /**
     * @brief Set anti-windup method
     * 
     * @param method Anti-windup method
     * @param param Method-specific parameter:
     *              - BackCalculation: tracking time constant (Tt)
     *              - Tracking: tracking gain
     */
    void setAntiWindup(AntiWindupMethod method, double param = 0.0);
    
    /**
     * @brief Get anti-windup method
     */
    AntiWindupMethod getAntiWindup() const { return m_antiWindup; }
    
    /**
     * @brief Set integral limits
     */
    void setIntegralLimits(double min, double max);
    
    // ========================================================================
    // State Access
    // ========================================================================
    
    /**
     * @brief Get current integral value
     */
    double getIntegral() const { return m_integral; }
    
    /**
     * @brief Set integral (for bumpless transfer)
     */
    void setIntegral(double value) { m_integral = value; }
    
    /**
     * @brief Get last derivative value
     */
    double getLastDerivative() const { return m_filteredDerivative; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    // Gains
    double m_kp{1.0};
    double m_ki{0.0};
    double m_kd{0.0};
    
    // Derivative filter
    double m_tf{0.0};
    DerivativeFilterType m_derivFilterType{DerivativeFilterType::FirstOrder};
    bool m_derivOnMeasurement{true};
    
    // Anti-windup
    AntiWindupMethod m_antiWindup{AntiWindupMethod::Clamping};
    double m_antiWindupParam{0.0};
    double m_integralMin{-std::numeric_limits<double>::max()};
    double m_integralMax{std::numeric_limits<double>::max()};
    
    // State
    double m_integral{0.0};
    double m_lastError{0.0};
    double m_prevError{0.0};  // For velocity form
    double m_lastMeasurement{0.0};
    double m_filteredDerivative{0.0};
    double m_lastOutput{0.0};
    bool m_firstRun{true};
    
    // Internal methods
    double computeDerivative(const ControllerInput& input);
    void applyAntiWindup(double error, double output, double saturatedOutput, double dt);
};

// ============================================================================
// PID-2DOF Controller
// ============================================================================

/**
 * @brief Two Degrees of Freedom PID Controller
 * 
 * ## Motivation
 * 
 * Standard PID responds the same way to setpoint changes and disturbances.
 * 2DOF PID allows separate tuning for each:
 * - Response to setpoint changes (command tracking)
 * - Response to disturbances (disturbance rejection)
 * 
 * ## Control Law
 * ```
 * u = Kp·(b·r - y) + Ki·∫(r - y)dt + Kd·(c·dr/dt - dy/dt)
 * 
 * Where:
 *   r = reference (setpoint)
 *   y = measurement
 *   b = proportional setpoint weight (0 to 1)
 *   c = derivative setpoint weight (0 to 1)
 * ```
 * 
 * ## Effect of Weights
 * 
 * ### Proportional Weight (b)
 * - b = 1: Full reference in P term (like standard PID)
 * - b = 0: Only error in P term (I-PD)
 * - b < 1: Reduced overshoot on setpoint changes
 * 
 * ### Derivative Weight (c)
 * - c = 1: Full reference derivative (like standard PID)
 * - c = 0: Derivative on measurement only (most common)
 * - c < 1: Reduced derivative kick
 * 
 * ## Common Configurations
 * 
 * | Name  | b   | c   | Description                          |
 * |-------|-----|-----|--------------------------------------|
 * | PID   | 1.0 | 1.0 | Standard PID                         |
 * | PI-D  | 1.0 | 0.0 | Derivative on measurement            |
 * | I-PD  | 0.0 | 0.0 | Minimize overshoot                   |
 * | 2DOF  | 0.5 | 0.0 | Balanced (typical starting point)    |
 * 
 * ## Example
 * ```cpp
 * PID2DOFController pid2dof;
 * pid2dof.setGains(1.0, 0.1, 0.05);
 * pid2dof.setSetpointWeights(0.7, 0.0);  // b=0.7 reduces overshoot
 *                                         // c=0 derivative on measurement
 * 
 * // For step input, acts like PID with reduced proportional kick
 * // For disturbance rejection, full PID action
 * ```
 */
class PID2DOFController : public PIDController {
public:
    ControllerType getType() const override { return ControllerType::PID2DOF; }
    const char* getName() const override { return "PID-2DOF Controller"; }
    const char* getDescription() const override {
        return "Two Degrees of Freedom PID. Separate setpoint and disturbance "
               "response via weights b (proportional) and c (derivative).";
    }
    
    /**
     * @brief Set setpoint weights
     * @param b Proportional weight (0 to 1)
     * @param c Derivative weight (0 to 1, typically 0)
     */
    void setSetpointWeights(double b, double c);
    
    /**
     * @brief Get proportional setpoint weight
     */
    double getB() const { return m_b; }
    
    /**
     * @brief Get derivative setpoint weight
     */
    double getC() const { return m_c; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    
private:
    double m_b{1.0};  // Proportional setpoint weight
    double m_c{0.0};  // Derivative setpoint weight
    double m_lastReference{0.0};
    double m_lastMeasurement2DOF{0.0};
    bool m_firstIteration{true};
};

// ============================================================================
// Bang-Bang Controller
// ============================================================================

/**
 * @brief Bang-Bang (On-Off) Controller with Hysteresis
 * 
 * ## Control Law
 * ```
 * if (e > +h/2): u = u_max
 * if (e < -h/2): u = u_min
 * else: u = u_prev (hysteresis)
 * 
 * Where:
 *   e = error
 *   h = hysteresis band
 * ```
 * 
 * ## Characteristics
 * - Simplest nonlinear controller
 * - Only two output states (on/off)
 * - Fast response, but oscillates around setpoint
 * - Hysteresis prevents chattering
 * 
 * ## Limit Cycling
 * Bang-bang control creates limit cycles (oscillation) around setpoint.
 * The amplitude and frequency depend on:
 * - System dynamics
 * - Hysteresis band
 * - Output levels
 * 
 * ## When to Use
 * - Simple temperature control (heater on/off)
 * - Systems where oscillation is acceptable
 * - When actuator only has two states
 * - Time-optimal control (theoretical)
 * 
 * ## Example
 * ```cpp
 * BangBangController bb;
 * bb.setOutputLevels(-1.0, 1.0);  // Min/max output
 * bb.setHysteresis(2.0);          // ±1 degree band
 * 
 * // For temperature control
 * ControllerInput input;
 * input.reference = 50.0;   // 50°C setpoint
 * input.measured = 48.0;    // Current temp
 * 
 * auto output = bb.compute(input);
 * // If error > 1 (hysteresis/2), output = 1.0 (heat on)
 * // If error < -1, output = -1.0 (cool on)
 * // Otherwise, maintain previous state
 * ```
 */
class BangBangController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::BangBang; }
    const char* getName() const override { return "Bang-Bang Controller"; }
    const char* getDescription() const override {
        return "On-off controller with hysteresis. Simple, fast, but oscillates. "
               "Use for systems where limit cycling is acceptable.";
    }
    
    /**
     * @brief Set output levels
     * @param uMin Output when error < -h/2
     * @param uMax Output when error > +h/2
     */
    void setOutputLevels(double uMin, double uMax);
    
    /**
     * @brief Set hysteresis band
     * @param h Hysteresis band width (total, switching at ±h/2)
     */
    void setHysteresis(double h) { m_hysteresis = h; }
    
    /**
     * @brief Get hysteresis band
     */
    double getHysteresis() const { return m_hysteresis; }
    
    /**
     * @brief Set neutral output (for 3-state bang-bang)
     * @param neutral Output when within hysteresis band
     * @param enable Enable 3-state mode
     */
    void setNeutralOutput(double neutral, bool enable = true);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    double m_uMin{-1.0};
    double m_uMax{1.0};
    double m_uNeutral{0.0};
    double m_hysteresis{1.0};
    bool m_threeState{false};
    double m_lastOutput{0.0};
    bool m_state{false};  // Current on/off state
};

// ============================================================================
// PD+ Controller (PD with Compensation)
// ============================================================================

/**
 * @brief PD Controller with Feedforward Compensation
 * 
 * ## Control Law
 * ```
 * u = Kp·e + Kd·ė + g(x)
 * 
 * Where g(x) is a compensation term for:
 * - Gravity compensation in robotics
 * - Friction compensation
 * - Coriolis compensation
 * - Any known nonlinearity
 * ```
 * 
 * ## Why PD+ Instead of PID?
 * 
 * In many robotic systems, using integral action is problematic:
 * - Slow response to disturbances
 * - Can cause instability with contact
 * - Windup during collisions
 * 
 * PD+ provides:
 * - Fast, stable response
 * - Zero steady-state error (through compensation)
 * - Natural handling of contact forces
 * 
 * ## Gravity Compensation Example
 * 
 * For a robot arm joint with angle θ:
 * ```
 * τ_gravity = m·g·L·sin(θ)
 * 
 * PD+: τ = Kp·(θd - θ) + Kd·(θ̇d - θ̇) + m·g·L·sin(θd)
 * ```
 * 
 * Note: Compensation based on desired position (θd) for stability.
 * 
 * ## Example
 * ```cpp
 * PDPlusController pdplus;
 * pdplus.setGains(100.0, 10.0);  // Stiff gains for robot
 * 
 * // Set gravity compensation callback
 * double mass = 1.5;  // kg
 * double length = 0.3; // m
 * pdplus.setCompensationCallback([mass, length](double position) {
 *     return mass * 9.81 * length * std::sin(position);
 * });
 * 
 * // In control loop
 * ControllerInput input;
 * input.reference = desiredAngle;
 * input.measured = actualAngle;
 * input.referenceDerivative = desiredVelocity;  // For Kd term
 * 
 * auto output = pdplus.compute(input);
 * motor.setTorque(output.control);
 * ```
 */
class PDPlusController : public ControllerBase {
public:
    /**
     * @brief Compensation function type
     * 
     * Takes position (and optionally velocity) and returns
     * the feedforward compensation term.
     */
    using CompensationFunction = std::function<double(double position)>;
    using FullCompensationFunction = std::function<double(double position, 
                                                          double velocity)>;
    
    ControllerType getType() const override { return ControllerType::PDPlus; }
    const char* getName() const override { return "PD+ Controller"; }
    const char* getDescription() const override {
        return "PD with feedforward compensation. Common in robotics for "
               "gravity, friction, or Coriolis compensation. Zero steady-state "
               "error without integral action.";
    }
    
    /**
     * @brief Set gains
     */
    void setGains(double kp, double kd);
    
    /**
     * @brief Set simple compensation callback
     * @param func Function: position → compensation torque
     */
    void setCompensationCallback(CompensationFunction func) {
        m_compensationFunc = func;
    }
    
    /**
     * @brief Set full compensation callback
     * @param func Function: (position, velocity) → compensation torque
     */
    void setFullCompensationCallback(FullCompensationFunction func) {
        m_fullCompensationFunc = func;
    }
    
    /**
     * @brief Set compensation based on desired (true) or actual (false) state
     * 
     * Using desired state is more stable but less accurate.
     * Using actual state gives better accuracy but can cause instability.
     */
    void setCompensationOnDesired(bool onDesired) {
        m_compOnDesired = onDesired;
    }
    
    /**
     * @brief Set derivative filter
     */
    void setDerivativeFilter(double tf) { m_tf = tf; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    double m_kp{1.0};
    double m_kd{0.0};
    double m_tf{0.0};
    
    CompensationFunction m_compensationFunc;
    FullCompensationFunction m_fullCompensationFunc;
    bool m_compOnDesired{true};
    
    double m_lastMeasurement{0.0};
    double m_filteredDerivative{0.0};
    bool m_firstRun{true};
};

// ============================================================================
// Dual Loop PID Controller
// ============================================================================

/**
 * @brief Dual-Loop (Cascade) PID Position Controller
 * 
 * ## Architecture
 * ```
 *   Position      ┌─────────┐  Velocity   ┌─────────┐  Torque
 *   Setpoint  ──►│ Outer   │  Setpoint  ─►│ Inner   │──────►
 *                │ PID     │              │ PID     │
 *   Position  ──►│         │  Velocity ──►│         │
 *   Feedback     └─────────┘  Feedback    └─────────┘
 * ```
 * 
 * ## Why Cascade Control?
 * 
 * 1. **Faster Disturbance Rejection**: Inner loop rejects disturbances
 *    before they significantly affect the outer variable.
 * 
 * 2. **Linearization**: Inner loop makes the outer loop see a more
 *    linear system (velocity response is simpler than position).
 * 
 * 3. **Better Tuning**: Each loop can be tuned independently.
 * 
 * 4. **Safety**: Inner loop can have its own limits (max velocity).
 * 
 * ## Design Rules
 * 
 * 1. Inner loop must be faster (3-5x) than outer loop
 * 2. Tune inner loop first, then outer
 * 3. Inner loop often needs only PI (velocity control)
 * 4. Outer loop often needs only P or PD (position control)
 * 
 * ## Typical Configuration
 * 
 * | Loop     | Type | Typical Gains                    |
 * |----------|------|----------------------------------|
 * | Outer    | P    | Kp only, generates velocity cmd  |
 * | Inner    | PI   | Fast velocity tracking           |
 * 
 * ## Example
 * ```cpp
 * DualLoopPIDController cascade;
 * 
 * // Configure outer (position) loop
 * cascade.setOuterGains(10.0, 0.0, 0.0);  // P-only is common
 * 
 * // Configure inner (velocity) loop
 * cascade.setInnerGains(1.0, 5.0, 0.0);   // PI
 * cascade.setInnerAntiWindup(AntiWindupMethod::BackCalculation, 0.1);
 * 
 * // Set velocity limits (inner loop saturation)
 * cascade.setVelocityLimits(-1000, 1000);  // rpm or similar
 * 
 * // In control loop
 * ControllerInput input;
 * input.reference = targetPosition;
 * input.measured = actualPosition;
 * 
 * // Need velocity feedback too
 * cascade.setVelocityFeedback(actualVelocity);
 * 
 * auto output = cascade.compute(input);
 * motor.setTorque(output.control);
 * ```
 * 
 * @see PIDController
 * @see CascadeController
 */
class DualLoopPIDController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::DualLoopPID; }
    const char* getName() const override { return "Dual-Loop PID Position Controller"; }
    const char* getDescription() const override {
        return "Cascade position-velocity PID. Outer loop (position) generates "
               "velocity setpoint for inner loop. Better disturbance rejection "
               "and easier tuning than single-loop.";
    }
    
    /**
     * @brief Set outer loop (position) gains
     */
    void setOuterGains(double kp, double ki = 0.0, double kd = 0.0);
    
    /**
     * @brief Set inner loop (velocity) gains
     */
    void setInnerGains(double kp, double ki, double kd = 0.0);
    
    /**
     * @brief Set velocity limits (inner loop command saturation)
     */
    void setVelocityLimits(double min, double max);
    
    /**
     * @brief Set torque limits (output saturation)
     */
    void setTorqueLimits(double min, double max);
    
    /**
     * @brief Set velocity feedback
     * 
     * Must be called each cycle before compute() to provide
     * the inner loop measurement.
     */
    void setVelocityFeedback(double velocity) { m_velocity = velocity; }
    
    /**
     * @brief Set outer loop anti-windup
     */
    void setOuterAntiWindup(AntiWindupMethod method, double param = 0.0);
    
    /**
     * @brief Set inner loop anti-windup
     */
    void setInnerAntiWindup(AntiWindupMethod method, double param = 0.0);
    
    /**
     * @brief Get velocity command (outer loop output)
     */
    double getVelocityCommand() const { return m_velocityCommand; }
    
    /**
     * @brief Access outer loop controller
     */
    PIDController& getOuterController() { return m_outerPID; }
    
    /**
     * @brief Access inner loop controller
     */
    PIDController& getInnerController() { return m_innerPID; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    PIDController m_outerPID;  // Position loop
    PIDController m_innerPID;  // Velocity loop
    
    double m_velocity{0.0};
    double m_velocityCommand{0.0};
    double m_velocityMin{-std::numeric_limits<double>::max()};
    double m_velocityMax{std::numeric_limits<double>::max()};
};

} // namespace tether::control
