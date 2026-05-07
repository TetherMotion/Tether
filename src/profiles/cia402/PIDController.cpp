/**
 * @file PIDController.cpp
 * @brief PID controller and filter implementations
 */

#include "profiles/cia402/PIDController.hpp"
#include <cstring>

namespace CiA402 {

// ============================================================================
// PID Controller
// ============================================================================

PIDController::PIDController() {
    m_gains.Kp = CIA402_DEFAULT_KP_POSITION;
    m_gains.Ki = CIA402_DEFAULT_KI_POSITION;
    m_gains.Kd = CIA402_DEFAULT_KD_POSITION;
}

void PIDController::setGains(const PIDGains& gains) {
    m_gains = gains;
}

void PIDController::setGains(double Kp, double Ki, double Kd, double Kff) {
    m_gains.Kp = Kp;
    m_gains.Ki = Ki;
    m_gains.Kd = Kd;
    m_gains.Kff = Kff;
}

void PIDController::setLimits(const PIDLimits& limits) {
    m_limits = limits;
}

void PIDController::setSampleTime(double sampleTimeS) {
    m_sampleTime = sampleTimeS;
    
    // Update derivative filter coefficient
    if (m_derivativeFilterEnabled && m_derivativeFilterTau > 0) {
        m_derivativeFilterAlpha = m_sampleTime / (m_derivativeFilterTau + m_sampleTime);
    }
}

void PIDController::setDerivativeFilter(bool enable, double tau) {
    m_derivativeFilterEnabled = enable;
    m_derivativeFilterTau = tau;
    
    if (enable && tau > 0) {
        m_derivativeFilterAlpha = m_sampleTime / (tau + m_sampleTime);
    }
}

void PIDController::reset() {
    m_integral = 0.0;
    m_lastError = 0.0;
    m_lastMeasurement = 0.0;
    m_derivative = 0.0;
    m_filteredDerivative = 0.0;
    m_proportional = 0.0;
    m_error = 0.0;
    m_output = 0.0;
    m_saturated = false;
    m_firstRun = true;
}

double PIDController::calculate(double setpoint, double measurement, double feedforward) {
    // Calculate error
    m_error = setpoint - measurement;
    
    // Proportional term
    m_proportional = m_gains.Kp * m_error;
    
    // Integral term with anti-windup
    if (m_antiWindup == AntiWindupMethod::ConditionalIntegration && m_saturated) {
        // Don't integrate when saturated
    } else {
        m_integral += m_gains.Ki * m_error * m_sampleTime;
        
        // Clamping anti-windup
        if (m_antiWindup == AntiWindupMethod::Clamping) {
            m_integral = std::clamp(m_integral, m_limits.integralMin, m_limits.integralMax);
        }
    }
    
    // Derivative term (derivative on measurement to avoid setpoint kick)
    if (m_firstRun) {
        m_derivative = 0.0;
        m_filteredDerivative = 0.0;
        m_firstRun = false;
    } else {
        // Use derivative on measurement to avoid derivative kick
        double rawDerivative = -(measurement - m_lastMeasurement) / m_sampleTime;
        
        // Apply filter if enabled
        if (m_derivativeFilterEnabled) {
            m_filteredDerivative = m_derivativeFilterAlpha * rawDerivative + 
                                   (1.0 - m_derivativeFilterAlpha) * m_filteredDerivative;
            m_derivative = m_gains.Kd * m_filteredDerivative;
        } else {
            m_derivative = m_gains.Kd * rawDerivative;
        }
    }
    
    // Feed-forward
    double ff = m_gains.Kff * feedforward;
    
    // Calculate unsaturated output
    double unsaturatedOutput = m_proportional + m_integral + m_derivative + ff;
    
    // Apply output limits
    m_output = std::clamp(unsaturatedOutput, m_limits.outputMin, m_limits.outputMax);
    
    // Check saturation
    m_saturated = (m_output != unsaturatedOutput);
    
    // Back-calculation anti-windup
    if (m_antiWindup == AntiWindupMethod::BackCalculation && m_saturated) {
        double saturationError = m_output - unsaturatedOutput;
        m_integral += m_Kb * saturationError * m_sampleTime;
    }
    
    // Save state
    m_lastError = m_error;
    m_lastMeasurement = measurement;
    
    return m_output;
}

double PIDController::calculateWithVelocityFF(double setpoint, double measurement, 
                                              double velocitySetpoint) {
    return calculate(setpoint, measurement, velocitySetpoint);
}

void PIDController::applyAntiWindup(double unsaturatedOutput) {
    switch (m_antiWindup) {
        case AntiWindupMethod::Clamping:
            m_integral = std::clamp(m_integral, m_limits.integralMin, m_limits.integralMax);
            break;
            
        case AntiWindupMethod::BackCalculation:
            if (m_saturated) {
                double error = m_output - unsaturatedOutput;
                m_integral += m_Kb * error * m_sampleTime;
            }
            break;
            
        case AntiWindupMethod::ConditionalIntegration:
            // Handled in calculate()
            break;
            
        default:
            break;
    }
}

// ============================================================================
// Low-Pass Filter
// ============================================================================

LowPassFilter::LowPassFilter(double cutoffHz, double sampleTimeS)
    : m_cutoffHz(cutoffHz)
    , m_sampleTime(sampleTimeS)
    , m_output(0.0)
{
    updateCoefficients();
}

void LowPassFilter::setCutoffFrequency(double cutoffHz) {
    m_cutoffHz = cutoffHz;
    updateCoefficients();
}

void LowPassFilter::setSampleTime(double sampleTimeS) {
    m_sampleTime = sampleTimeS;
    updateCoefficients();
}

void LowPassFilter::updateCoefficients() {
    // Calculate filter coefficient
    // alpha = dt / (tau + dt) where tau = 1 / (2*pi*fc)
    double tau = 1.0 / (2.0 * M_PI * m_cutoffHz);
    m_alpha = m_sampleTime / (tau + m_sampleTime);
}

void LowPassFilter::reset() {
    m_output = 0.0;
}

void LowPassFilter::reset(double initialValue) {
    m_output = initialValue;
}

double LowPassFilter::filter(double input) {
    m_output = m_alpha * input + (1.0 - m_alpha) * m_output;
    return m_output;
}

// ============================================================================
// Notch Filter
// ============================================================================

NotchFilter::NotchFilter(double notchHz, double Q, double sampleTimeS)
    : m_notchHz(notchHz)
    , m_Q(Q)
    , m_sampleTime(sampleTimeS)
{
    updateCoefficients();
}

void NotchFilter::setNotchFrequency(double notchHz) {
    m_notchHz = notchHz;
    updateCoefficients();
}

void NotchFilter::setQ(double Q) {
    m_Q = Q;
    updateCoefficients();
}

void NotchFilter::setSampleTime(double sampleTimeS) {
    m_sampleTime = sampleTimeS;
    updateCoefficients();
}

void NotchFilter::updateCoefficients() {
    // Bilinear transform of notch filter
    // Using direct form I biquad
    
    double wn = 2.0 * M_PI * m_notchHz;
    double k = std::tan(wn * m_sampleTime / 2.0);
    double k2 = k * k;
    double denom = 1.0 + k / m_Q + k2;
    
    m_b0 = (1.0 + k2) / denom;
    m_b1 = 2.0 * (k2 - 1.0) / denom;
    m_b2 = (1.0 + k2) / denom;
    m_a1 = 2.0 * (k2 - 1.0) / denom;
    m_a2 = (1.0 - k / m_Q + k2) / denom;
}

void NotchFilter::reset() {
    m_x1 = m_x2 = 0.0;
    m_y1 = m_y2 = 0.0;
}

double NotchFilter::filter(double input) {
    // Direct form I biquad
    double output = m_b0 * input + m_b1 * m_x1 + m_b2 * m_x2 
                   - m_a1 * m_y1 - m_a2 * m_y2;
    
    // Update state
    m_x2 = m_x1;
    m_x1 = input;
    m_y2 = m_y1;
    m_y1 = output;
    
    return output;
}

// ============================================================================
// Moving Average Filter
// ============================================================================

MovingAverageFilter::MovingAverageFilter(size_t windowSize) {
    setWindowSize(windowSize);
}

void MovingAverageFilter::setWindowSize(size_t size) {
    m_buffer.resize(size, 0.0);
    reset();
}

void MovingAverageFilter::reset() {
    std::fill(m_buffer.begin(), m_buffer.end(), 0.0);
    m_index = 0;
    m_sum = 0.0;
    m_output = 0.0;
    m_filled = false;
}

double MovingAverageFilter::filter(double input) {
    // Subtract old value from sum
    m_sum -= m_buffer[m_index];
    
    // Add new value
    m_buffer[m_index] = input;
    m_sum += input;
    
    // Advance index
    m_index = (m_index + 1) % m_buffer.size();
    
    // Check if buffer is filled
    if (m_index == 0) {
        m_filled = true;
    }
    
    // Calculate average
    size_t count = m_filled ? m_buffer.size() : m_index;
    m_output = (count > 0) ? m_sum / count : 0.0;
    
    return m_output;
}

// ============================================================================
// Cascaded PID Controller
// ============================================================================

CascadedPIDController::CascadedPIDController() {
    // Default position gains
    m_positionPID.setGains(
        CIA402_DEFAULT_KP_POSITION,
        CIA402_DEFAULT_KI_POSITION,
        CIA402_DEFAULT_KD_POSITION
    );
    
    // Default velocity gains
    m_velocityPID.setGains(
        CIA402_DEFAULT_KP_VELOCITY,
        CIA402_DEFAULT_KI_VELOCITY,
        CIA402_DEFAULT_KD_VELOCITY
    );
}

void CascadedPIDController::setPositionGains(const PIDGains& gains) {
    m_positionPID.setGains(gains);
}

void CascadedPIDController::setVelocityGains(const PIDGains& gains) {
    m_velocityPID.setGains(gains);
}

void CascadedPIDController::setPositionLimits(const PIDLimits& limits) {
    m_positionPID.setLimits(limits);
}

void CascadedPIDController::setVelocityLimits(const PIDLimits& limits) {
    m_velocityPID.setLimits(limits);
}

void CascadedPIDController::setSampleTime(double sampleTimeS) {
    m_positionPID.setSampleTime(sampleTimeS);
    m_velocityPID.setSampleTime(sampleTimeS);
}

void CascadedPIDController::reset() {
    m_positionPID.reset();
    m_velocityPID.reset();
    m_velocitySetpoint = 0.0;
}

double CascadedPIDController::calculate(double positionSetpoint, double actualPosition,
                                        double actualVelocity, double velocityFF) {
    // Position loop generates velocity setpoint
    m_velocitySetpoint = m_positionPID.calculate(positionSetpoint, actualPosition);
    
    // Add velocity feed-forward
    double totalVelocitySetpoint = m_velocitySetpoint + velocityFF;
    
    // Velocity loop generates torque command
    double torqueCommand = m_velocityPID.calculate(totalVelocitySetpoint, actualVelocity);
    
    return torqueCommand;
}

} // namespace CiA402
