/**
 * @file PIDController.hpp
 * @brief PID controller implementation for CiA 402 axes
 * 
 * @details
 * Provides configurable PID control with:
 * - Position and velocity loops
 * - Anti-windup protection
 * - Output limiting
 * - Feed-forward support
 * - Digital filtering (low-pass, notch)
 * 
 * ## Control Loop Architecture
 * 
 * ```
 *                    ┌───────────────────────────────────────────┐
 *                    │            Position Loop                   │
 *  Position  ┌───┐   │   ┌─────┐    ┌─────────────┐              │
 *  Setpoint─►│ + │───┼──►│ Kp  │───►│             │              │
 *            │   │   │   └─────┘    │             │   ┌───────┐  │
 *            │ - │◄──┼──────────────│   Velocity  │──►│Output │──┼──► Torque/
 *            └───┘   │              │   Loop (opt)│   │Limit  │  │    Current
 *               │    │   ┌─────┐    │             │   └───────┘  │    Command
 *  Actual ◄─────┴────┼───│Filt.│◄───│             │              │
 *  Position          │   └─────┘    └─────────────┘              │
 *                    └───────────────────────────────────────────┘
 * 
 *                    ┌───────────────────────────────────────────┐
 *                    │            Velocity Loop                   │
 *  Velocity  ┌───┐   │   ┌─────┐    ┌─────┐    ┌─────┐          │
 *  Setpoint─►│ + │───┼──►│ Kp  │───►│     │───►│Anti │          │
 *            │   │   │   └─────┘    │  +  │    │Wind │──────────┼──► Output
 *            │ - │◄──┼───┐          │     │    │up   │          │
 *            └───┘   │   │ ┌─────┐  │     │    └─────┘          │
 *               │    │   │ │ Ki  │──►     │                      │
 *               │    │   │ │∫dt  │  │     │                      │
 *  Actual ◄─────┴────┼───┤ └─────┘  └─────┘                      │
 *  Velocity          │   │                                       │
 *                    │   │ ┌─────┐                               │
 *                    │   └─│ Kd  │──────────────────────────────►│
 *                    │     │d/dt │                               │
 *                    │     └─────┘                               │
 *                    └───────────────────────────────────────────┘
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>

namespace CiA402 {

/**
 * @brief PID gains structure
 */
struct PIDGains {
    double Kp{0.0};     ///< Proportional gain
    double Ki{0.0};     ///< Integral gain
    double Kd{0.0};     ///< Derivative gain
    double Kff{0.0};    ///< Feed-forward gain
};

/**
 * @brief PID controller limits
 */
struct PIDLimits {
    double outputMin{-1000.0};  ///< Minimum output
    double outputMax{1000.0};   ///< Maximum output
    double integralMin{-100.0}; ///< Minimum integral accumulator
    double integralMax{100.0};  ///< Maximum integral accumulator
};

/**
 * @brief Anti-windup method
 */
enum class AntiWindupMethod {
    None,           ///< No anti-windup
    Clamping,       ///< Simple integral clamping
    BackCalculation,///< Back-calculation method
    ConditionalIntegration  ///< Disable integration when saturated
};

/**
 * @brief PID controller implementation
 */
class PIDController {
public:
    PIDController();
    ~PIDController() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set PID gains
     * @param gains Gain set to apply.
     */
    void setGains(const PIDGains& gains);
    
    /**
     * @brief Set gains individually
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     * @param Kff Velocity feedforward gain.
     */
    void setGains(double Kp, double Ki, double Kd, double Kff = 0.0);
    
    /**
     * @brief Get current gains
     * @return Gains currently in use.
     */
    PIDGains getGains() const { return m_gains; }
    
    /**
     * @brief Set output limits
     * @param limits Output/integral bounds and rate limit.
     */
    void setLimits(const PIDLimits& limits);
    
    /**
     * @brief Get output limits
     * @return Limits currently enforced.
     */
    PIDLimits getLimits() const { return m_limits; }
    
    /**
     * @brief Set sample time
     * 
     * @param sampleTimeS Sample time in seconds
     */
    void setSampleTime(double sampleTimeS);
    
    /**
     * @brief Set anti-windup method
     * @param method Anti-windup strategy to apply while saturated.
     */
    void setAntiWindup(AntiWindupMethod method) { m_antiWindup = method; }
    
    /**
     * @brief Set back-calculation gain (for BackCalculation anti-windup)
     * @param Kb Back-calculation gain (1/Tt); larger unwinds faster.
     */
    void setBackCalcGain(double Kb) { m_Kb = Kb; }
    
    /**
     * @brief Enable/disable derivative filter
     * 
     * @param enable Enable filter
     * @param tau Filter time constant (seconds)
     */
    void setDerivativeFilter(bool enable, double tau = 0.01);
    
    // ========================================================================
    // Operation
    // ========================================================================
    
    /**
     * @brief Reset controller state
     */
    void reset();
    
    /**
     * @brief Calculate control output
     * 
     * @param setpoint Desired value
     * @param measurement Current value
     * @param feedforward Feed-forward input (optional)
     * @return Control output
     */
    double calculate(double setpoint, double measurement, double feedforward = 0.0);
    
    /**
     * @brief Calculate with velocity feed-forward
     * 
     * @param setpoint Position setpoint
     * @param measurement Position measurement
     * @param velocitySetpoint Velocity setpoint for feed-forward
     * @return Control output
     */
    double calculateWithVelocityFF(double setpoint, double measurement, 
                                   double velocitySetpoint);
    
    // ========================================================================
    // Status
    // ========================================================================
    
    /**
     * @brief Get current error
     * @return Error from the last update.
     */
    double getError() const { return m_error; }
    
    /**
     * @brief Get integral term
     * @return Integral contribution of the last update.
     */
    double getIntegralTerm() const { return m_integral; }
    
    /**
     * @brief Get derivative term
     * @return Derivative contribution of the last update.
     */
    double getDerivativeTerm() const { return m_derivative; }
    
    /**
     * @brief Get proportional term
     * @return Proportional contribution of the last update.
     */
    double getProportionalTerm() const { return m_proportional; }
    
    /**
     * @brief Get last output
     * @return Control output produced by the last update.
     */
    double getOutput() const { return m_output; }
    
    /**
     * @brief Check if output is saturated
     * @return true when the output hit a limit in the last update.
     */
    bool isSaturated() const { return m_saturated; }
    
private:
    /**
     * @brief Apply anti-windup
     */
    void applyAntiWindup(double unsaturatedOutput);
    
    // Gains
    PIDGains m_gains;
    PIDLimits m_limits;
    
    // State
    double m_integral{0.0};
    double m_lastError{0.0};
    double m_lastMeasurement{0.0};
    double m_derivative{0.0};
    double m_filteredDerivative{0.0};
    
    // Output
    double m_proportional{0.0};
    double m_error{0.0};
    double m_output{0.0};
    bool m_saturated{false};
    
    // Configuration
    double m_sampleTime{0.001};
    AntiWindupMethod m_antiWindup{AntiWindupMethod::Clamping};
    double m_Kb{1.0};  // Back-calculation gain
    
    // Derivative filter
    bool m_derivativeFilterEnabled{false};
    double m_derivativeFilterTau{0.01};
    double m_derivativeFilterAlpha{1.0};
    
    bool m_firstRun{true};
};

// ============================================================================
// Digital Filters
// ============================================================================

/**
 * @brief First-order low-pass filter
 * 
 * H(s) = 1 / (tau*s + 1)
 */
class LowPassFilter {
public:
    /**
     * @brief Constructor
     * 
     * @param cutoffHz Cutoff frequency in Hz
     * @param sampleTimeS Sample time in seconds
     */
    LowPassFilter(double cutoffHz = CIA402_FILTER_POSITION_CUTOFF_HZ, 
                  double sampleTimeS = 0.001);
    
    /**
     * @brief Set cutoff frequency
     * @param cutoffHz -3 dB cutoff frequency [Hz].
     */
    void setCutoffFrequency(double cutoffHz);
    
    /**
     * @brief Set sample time
     * @param sampleTimeS Filter sample period [s].
     */
    void setSampleTime(double sampleTimeS);
    
    /**
     * @brief Reset filter
     *
     * Clears the internal state; the next sample starts from rest.
     */
    void reset();
    
    /**
     * @brief Reset with initial value
     * @param initialValue Output the filter starts from.
     */
    void reset(double initialValue);
    
    /**
     * @brief Apply filter
     * @param input New sample.
     * @return Filtered sample.
     */
    double filter(double input);
    
    /**
     * @brief Get last output
     * @return Most recent filtered value.
     */
    double getOutput() const { return m_output; }
    
private:
    void updateCoefficients();
    
    double m_cutoffHz;
    double m_sampleTime;
    double m_alpha{0.0};
    double m_output{0.0};
};

/**
 * @brief Second-order notch filter
 * 
 * Removes specific frequency (e.g., mechanical resonance)
 * 
 * H(s) = (s² + wn²) / (s² + (wn/Q)*s + wn²)
 */
class NotchFilter {
public:
    /**
     * @brief Constructor
     * 
     * @param notchHz Notch center frequency in Hz
     * @param Q Quality factor (higher = narrower notch)
     * @param sampleTimeS Sample time in seconds
     */
    NotchFilter(double notchHz = CIA402_FILTER_NOTCH_FREQ_HZ,
                double Q = CIA402_FILTER_NOTCH_Q,
                double sampleTimeS = 0.001);
    
    /**
     * @brief Set notch frequency
     * @param notchHz Notch centre frequency [Hz].
     */
    void setNotchFrequency(double notchHz);
    
    /**
     * @brief Set Q factor
     * @param Q Quality factor (higher = narrower notch).
     */
    void setQ(double Q);
    
    /**
     * @brief Set sample time
     * @param sampleTimeS Filter sample period [s].
     */
    void setSampleTime(double sampleTimeS);
    
    /**
     * @brief Reset filter
     *
     * Clears the biquad history; the next sample starts from rest.
     */
    void reset();
    
    /**
     * @brief Apply filter
     * @param input New sample.
     * @return Filtered sample.
     */
    double filter(double input);
    
    /**
     * @brief Get last output
     * @return Most recent filtered value.
     */
    double getOutput() const { return m_y1; }
    
private:
    void updateCoefficients();
    
    double m_notchHz;
    double m_Q;
    double m_sampleTime;
    
    // Filter coefficients (biquad)
    double m_b0{1.0}, m_b1{0.0}, m_b2{0.0};
    double m_a1{0.0}, m_a2{0.0};
    
    // State
    double m_x1{0.0}, m_x2{0.0};  // Input history
    double m_y1{0.0}, m_y2{0.0};  // Output history
};

/**
 * @brief Moving average filter
 */
class MovingAverageFilter {
public:
    /**
     * @brief Constructor
     * 
     * @param windowSize Number of samples to average
     */
    explicit MovingAverageFilter(size_t windowSize = 10);
    
    /**
     * @brief Set window size
     * @param size Number of samples averaged.
     */
    void setWindowSize(size_t size);
    
    /**
     * @brief Reset filter
     *
     * Clears the window; the average restarts from the next sample.
     */
    void reset();
    
    /**
     * @brief Apply filter
     * @param input New sample.
     * @return Mean of the samples in the window.
     */
    double filter(double input);
    
    /**
     * @brief Get last output
     * @return Most recent window average.
     */
    double getOutput() const { return m_output; }
    
private:
    std::vector<double> m_buffer;
    size_t m_index{0};
    double m_sum{0.0};
    double m_output{0.0};
    bool m_filled{false};
};

// ============================================================================
// Cascaded Position-Velocity Controller
// ============================================================================

/**
 * @brief Cascaded position and velocity PID controller
 * 
 * Outer position loop generates velocity setpoint for inner velocity loop.
 */
class CascadedPIDController {
public:
    CascadedPIDController();
    
    /**
     * @brief Set position loop gains
     * @param gains Outer (position) loop gains.
     */
    void setPositionGains(const PIDGains& gains);
    
    /**
     * @brief Set velocity loop gains
     * @param gains Inner (velocity) loop gains.
     */
    void setVelocityGains(const PIDGains& gains);
    
    /**
     * @brief Set position loop limits
     * @param limits Output limits of the position loop (velocity command).
     */
    void setPositionLimits(const PIDLimits& limits);
    
    /**
     * @brief Set velocity loop limits
     * @param limits Output limits of the velocity loop (torque command).
     */
    void setVelocityLimits(const PIDLimits& limits);
    
    /**
     * @brief Set sample time for both loops
     * @param sampleTimeS Sample period [s].
     */
    void setSampleTime(double sampleTimeS);
    
    /**
     * @brief Reset both controllers
     *
     * Clears integrators and filter state in both loops.
     */
    void reset();
    
    /**
     * @brief Calculate cascaded output
     * 
     * @param positionSetpoint Position setpoint
     * @param actualPosition Current position
     * @param actualVelocity Current velocity
     * @param velocityFF Velocity feed-forward (optional)
     * @return Torque/current command
     */
    double calculate(double positionSetpoint, double actualPosition,
                    double actualVelocity, double velocityFF = 0.0);
    
    /**
     * @brief Get position controller
     * @return Outer (position) loop controller for live tuning.
     */
    PIDController& getPositionController() { return m_positionPID; }
    
    /**
     * @brief Get velocity controller
     * @return Inner (velocity) loop controller for live tuning.
     */
    PIDController& getVelocityController() { return m_velocityPID; }
    
    /**
     * @brief Get position error
     * @return Position error from the last calculate() call.
     */
    double getPositionError() const { return m_positionPID.getError(); }
    
    /**
     * @brief Get velocity error
     * @return Velocity error from the last calculate() call.
     */
    double getVelocityError() const { return m_velocityPID.getError(); }
    
    /**
     * @brief Get velocity setpoint (output of position loop)
     * @return Velocity command produced by the position loop.
     */
    double getVelocitySetpoint() const { return m_velocitySetpoint; }
    
private:
    PIDController m_positionPID;
    PIDController m_velocityPID;
    double m_velocitySetpoint{0.0};
};

} // namespace CiA402
