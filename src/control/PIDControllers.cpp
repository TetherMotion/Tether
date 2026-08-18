/**
 * @file PIDControllers.cpp
 * @brief Implementation of PID Controller Family
 */

#include "control/PIDControllers.hpp"
#include <algorithm>
#include <cmath>

namespace tether::control {

// ============================================================================
// P Controller
// ============================================================================

ControllerOutput PController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    
    ControllerOutput output;
    output.control = m_kp * error;
    output.proportional = output.control;
    output.error = error;
    
    return output;
}

// ============================================================================
// PD Controller
// ============================================================================

void PDController::setGains(double kp, double kd) {
    m_kp = kp;
    m_kd = kd;
}

ControllerOutput PDController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double dt = input.dt;
    
    // Compute derivative term
    double derivative = 0.0;
    
    if (!m_firstRun && dt > 0) {
        double derivInput;
        if (m_derivOnMeasurement) {
            // Derivative on measurement (avoids derivative kick)
            derivInput = -(input.measured - m_lastMeasurement) / dt;
        } else {
            // Derivative on error
            derivInput = (error - m_lastError) / dt;
        }
        
        // Apply derivative filter
        if (m_tf > 0) {
            double alpha = dt / (m_tf + dt);
            m_filteredDerivative = alpha * derivInput + (1.0 - alpha) * m_filteredDerivative;
            derivative = m_filteredDerivative;
        } else {
            derivative = derivInput;
        }
    }
    
    m_lastError = error;
    m_lastMeasurement = input.measured;
    m_firstRun = false;
    
    ControllerOutput output;
    output.proportional = m_kp * error;
    output.derivative = m_kd * derivative;
    output.control = output.proportional + output.derivative;
    output.error = error;
    
    return output;
}

void PDController::resetImpl() {
    m_lastError = 0.0;
    m_lastMeasurement = 0.0;
    m_filteredDerivative = 0.0;
    m_firstRun = true;
}

// ============================================================================
// PI Controller
// ============================================================================

void PIController::setGains(double kp, double ki) {
    m_kp = kp;
    m_ki = ki;
}

void PIController::setGainsFromTi(double kp, double ti) {
    m_kp = kp;
    m_ki = (ti > 0) ? kp / ti : 0.0;
}

void PIController::setIntegralLimits(double min, double max) {
    m_integralMin = min;
    m_integralMax = max;
}

void PIController::setAntiWindup(AntiWindupMethod method, double param) {
    m_antiWindup = method;
    m_antiWindupParam = param;
}

ControllerOutput PIController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double dt = input.dt;
    
    // Proportional term
    double pTerm = m_kp * error;
    
    // Compute raw output (before anti-windup)
    double rawOutput = pTerm + m_integral;
    
    // Apply saturation for anti-windup calculation
    double saturatedOutput = std::clamp(rawOutput,
                                        m_limits.outputMin,
                                        m_limits.outputMax);
    bool isSaturated = (rawOutput != saturatedOutput);
    
    // Update integral with anti-windup
    double integralIncrement = m_ki * error * dt;
    
    switch (m_antiWindup) {
        case AntiWindupMethod::None:
            m_integral += integralIncrement;
            break;
            
        case AntiWindupMethod::Clamping:
            // Only integrate if not saturated or integration would help
            if (!isSaturated || (error * rawOutput < 0)) {
                m_integral += integralIncrement;
            }
            break;
            
        case AntiWindupMethod::BackCalculation: {
            // Track back saturation error
            double trackingGain = (m_antiWindupParam > 0) ? 
                                   m_antiWindupParam : 1.0 / m_kp;
            double trackingTerm = trackingGain * (saturatedOutput - rawOutput);
            m_integral += (integralIncrement + trackingTerm * dt);
            break;
        }
            
        case AntiWindupMethod::Conditional:
            // Only integrate when error and output have opposite signs
            // or when we're not saturated
            if (!isSaturated || (error * m_integral) < 0) {
                m_integral += integralIncrement;
            }
            break;
            
        default:
            m_integral += integralIncrement;
            break;
    }
    
    // Apply integral limits
    m_integral = std::clamp(m_integral, m_integralMin, m_integralMax);
    
    m_lastError = error;
    m_lastOutput = saturatedOutput;
    
    ControllerOutput output;
    output.proportional = pTerm;
    output.integral = m_integral;
    output.control = pTerm + m_integral;
    output.error = error;
    output.saturated = isSaturated;
    
    return output;
}

void PIController::resetImpl() {
    m_integral = 0.0;
    m_lastError = 0.0;
    m_lastOutput = 0.0;
}

// ============================================================================
// PID Controller
// ============================================================================

void PIDController::setGains(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PIDController::setGainsFromTimeConstants(double kp, double ti, double td) {
    m_kp = kp;
    m_ki = (ti > 0) ? kp / ti : 0.0;
    m_kd = kp * td;
}

void PIDController::setAntiWindup(AntiWindupMethod method, double param) {
    m_antiWindup = method;
    m_antiWindupParam = param;
}

void PIDController::setIntegralLimits(double min, double max) {
    m_integralMin = min;
    m_integralMax = max;
}

void PIDController::autoTune(double processGain, double timeConstant, 
                             double deadTime, TuningMethod method,
                             double lambda) {
    double K = processGain;
    double tau = timeConstant;
    double theta = deadTime;

    // Guard against division by zero — NaN/Inf in gains would propagate
    // through the control loop and could damage machinery.
    if (std::abs(K) < 1e-10 || std::abs(theta) < 1e-10) {
        return;
    }
    
    switch (method) {
        case TuningMethod::ZieglerNichols:
        case TuningMethod::ZieglerNicholsPID: {
            // Classic Z-N from process reaction curve
            // For FOPTD: Kp = 1.2*tau/(K*theta), Ti = 2*theta, Td = 0.5*theta
            m_kp = (1.2 * tau) / (K * theta);
            double ti = 2.0 * theta;
            double td = 0.5 * theta;
            m_ki = (ti > 0) ? m_kp / ti : 0.0;
            m_kd = m_kp * td;
            break;
        }
            
        case TuningMethod::ZieglerNicholsPI: {
            m_kp = (0.9 * tau) / (K * theta);
            double ti = 3.33 * theta;
            m_ki = (ti > 0) ? m_kp / ti : 0.0;
            m_kd = 0.0;
            break;
        }
            
        case TuningMethod::CohenCoon: {
            // Better for dead-time dominant processes
            double r = theta / tau;
            m_kp = (1.35 / K) * (tau / theta + 0.185);
            double ti = theta * (2.5 - 2.0 * r) / (1.0 - 0.39 * r);
            double td = 0.37 * theta / (1.0 - 0.81 * r);
            m_ki = (ti > 0) ? m_kp / ti : 0.0;
            m_kd = m_kp * td;
            break;
        }
            
        case TuningMethod::Lambda: {
            // IMC-based Lambda tuning
            double lam = (lambda > 0) ? lambda : tau;  // Default: lambda = tau
            m_kp = tau / (K * (lam + theta));
            m_ki = m_kp / tau;
            m_kd = 0.0;  // Often zero for Lambda tuning
            break;
        }
            
        case TuningMethod::SIMC: {
            // Skogestad IMC - good balance of performance and robustness
            double tc = std::max(tau, 8.0 * theta);  // Closed-loop time constant
            m_kp = tau / (K * (tc + theta));
            m_ki = m_kp / std::min(tau, 4.0 * (tc + theta));
            m_kd = 0.0;
            break;
        }
            
        case TuningMethod::AMIGO: {
            // AMIGO - robustness optimized
            m_kp = (0.2 + 0.45 * tau / theta) / K;
            double ti = (0.4 * theta + 0.8 * tau) * theta / (theta + 0.1 * tau);
            double td = 0.5 * theta * tau / (0.3 * theta + tau);
            m_ki = (ti > 0) ? m_kp / ti : 0.0;
            m_kd = m_kp * td;
            break;
        }
            
        case TuningMethod::TyreusLuyben: {
            // Conservative, from ultimate gain
            // Requires Ku and Tu - this is a fallback
            m_kp = 0.5 * tau / (K * theta);
            m_ki = m_kp / (2.2 * std::max(tau, theta));
            m_kd = 0.0;
            break;
        }
    }
    
    // Set reasonable derivative filter
    if (m_kd > 0) {
        m_tf = m_kd / (8.0 * m_kp);
    }
}

void PIDController::autoTuneFromUltimate(double ultimateGain, double ultimatePeriod,
                                         TuningMethod method) {
    double Ku = ultimateGain;
    double Tu = ultimatePeriod;
    
    switch (method) {
        case TuningMethod::ZieglerNichols:
        case TuningMethod::ZieglerNicholsPID:
            m_kp = 0.6 * Ku;
            m_ki = 2.0 * m_kp / Tu;
            m_kd = m_kp * Tu / 8.0;
            break;
            
        case TuningMethod::ZieglerNicholsPI:
            m_kp = 0.45 * Ku;
            m_ki = 1.2 * m_kp / Tu;
            m_kd = 0.0;
            break;
            
        case TuningMethod::TyreusLuyben:
            m_kp = Ku / 3.2;
            m_ki = m_kp / (2.2 * Tu);
            m_kd = m_kp * Tu / 6.3;
            break;
            
        default:
            // For methods that don't use ultimate gain, use Z-N
            m_kp = 0.6 * Ku;
            m_ki = 2.0 * m_kp / Tu;
            m_kd = m_kp * Tu / 8.0;
            break;
    }
    
    // Set derivative filter
    if (m_kd > 0) {
        m_tf = m_kd / (8.0 * m_kp);
    }
}

double PIDController::computeDerivative(const ControllerInput& input) {
    double dt = input.dt;
    if (m_firstRun || dt <= 0) {
        return 0.0;
    }
    
    double derivInput;
    if (m_derivOnMeasurement) {
        // Derivative on measurement only
        derivInput = -(input.measured - m_lastMeasurement) / dt;
    } else {
        // Derivative on error
        double error = input.reference - input.measured;
        derivInput = (error - m_lastError) / dt;
    }
    
    // Apply derivative filter
    if (m_tf > 0) {
        switch (m_derivFilterType) {
            case DerivativeFilterType::None:
                return derivInput;
                
            case DerivativeFilterType::FirstOrder: {
                double alpha = dt / (m_tf + dt);
                m_filteredDerivative = alpha * derivInput + 
                                       (1.0 - alpha) * m_filteredDerivative;
                break;
            }
                
            case DerivativeFilterType::SecondOrder: {
                // Second-order filter for smoother response
                double omega = 1.0 / m_tf;
                double alpha = omega * dt;
                double beta = alpha * alpha;
                double gamma = 2.0 * alpha;
                m_filteredDerivative = (beta * derivInput + 
                                       gamma * m_filteredDerivative - 
                                       0.5 * beta * m_filteredDerivative) /
                                       (1.0 + gamma + beta);
                break;
            }
                
            case DerivativeFilterType::MovingAverage:
                // Simple exponential moving average
                m_filteredDerivative = 0.9 * m_filteredDerivative + 
                                       0.1 * derivInput;
                break;
                
            case DerivativeFilterType::Median:
                // For median filter, we just use first-order as fallback
                // (proper median would need a buffer)
                m_filteredDerivative = 0.5 * derivInput + 0.5 * m_filteredDerivative;
                break;
        }
        return m_filteredDerivative;
    }
    
    return derivInput;
}

void PIDController::applyAntiWindup(double error, double output, 
                                    double saturatedOutput, double dt) {
    bool isSaturated = (output != saturatedOutput);
    double integralIncrement = m_ki * error * dt;
    
    switch (m_antiWindup) {
        case AntiWindupMethod::None:
            m_integral += integralIncrement;
            break;
            
        case AntiWindupMethod::Clamping:
            // Stop integration when saturated and integration would make it worse
            if (!isSaturated || (error * output < 0)) {
                m_integral += integralIncrement;
            }
            break;
            
        case AntiWindupMethod::BackCalculation: {
            // Track back the saturation error with time constant Tt
            double Tt = (m_antiWindupParam > 0) ? m_antiWindupParam : 
                        std::sqrt(m_kd / m_ki);  // Default: sqrt(Td/Ti)
            if (Tt <= 0) Tt = 1.0 / m_kp;  // Fallback
            
            double trackingTerm = (saturatedOutput - output) / Tt;
            m_integral += (integralIncrement + trackingTerm * dt);
            break;
        }
            
        case AntiWindupMethod::Conditional: {
            // Conditional integration - multiple criteria
            bool shouldIntegrate = true;
            
            // Criterion 1: Don't integrate if saturated AND integral would grow
            if (isSaturated && (error * m_integral > 0)) {
                shouldIntegrate = false;
            }
            
            // Criterion 2: Don't integrate if error is large
            // (optional, based on param)
            if (m_antiWindupParam > 0 && std::abs(error) > m_antiWindupParam) {
                shouldIntegrate = false;
            }
            
            if (shouldIntegrate) {
                m_integral += integralIncrement;
            }
            break;
        }
            
        case AntiWindupMethod::Tracking:
            // For tracking anti-windup, use external tracking signal
            // (provided via input.feedforward as tracking error)
            m_integral += integralIncrement;
            break;
            
        case AntiWindupMethod::ObserverBased:
            // Observer-based anti-windup - model-based approach
            // Simplified: use back-calculation with estimated dynamics
            {
                double Tt = (m_antiWindupParam > 0) ? m_antiWindupParam : 0.1;
                double trackingTerm = (saturatedOutput - output) / Tt;
                m_integral += (integralIncrement + trackingTerm * dt);
            }
            break;
    }
    
    // Apply integral limits
    m_integral = std::clamp(m_integral, m_integralMin, m_integralMax);
}

ControllerOutput PIDController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double dt = input.dt;
    
    // Proportional term
    double pTerm = m_kp * error;
    
    // Derivative term
    double derivative = computeDerivative(input);
    double dTerm = m_kd * derivative;
    
    // Compute raw output
    double rawOutput = pTerm + m_integral + dTerm;
    
    // Apply saturation
    double saturatedOutput = std::clamp(rawOutput,
                                        m_limits.outputMin,
                                        m_limits.outputMax);
    
    // Update integral with anti-windup
    applyAntiWindup(error, rawOutput, saturatedOutput, dt);
    
    // Update state
    m_prevError = m_lastError;
    m_lastError = error;
    m_lastMeasurement = input.measured;
    m_lastOutput = saturatedOutput;
    m_firstRun = false;
    
    ControllerOutput output;
    output.proportional = pTerm;
    output.integral = m_integral;
    output.derivative = dTerm;
    output.control = pTerm + m_integral + dTerm;
    output.error = error;
    output.saturated = (rawOutput != saturatedOutput);
    
    return output;
}

void PIDController::resetImpl() {
    m_integral = 0.0;
    m_lastError = 0.0;
    m_prevError = 0.0;
    m_lastMeasurement = 0.0;
    m_filteredDerivative = 0.0;
    m_lastOutput = 0.0;
    m_firstRun = true;
}

// ============================================================================
// PID-2DOF Controller
// ============================================================================

void PID2DOFController::setSetpointWeights(double b, double c) {
    m_b = std::clamp(b, 0.0, 1.0);
    m_c = std::clamp(c, 0.0, 1.0);
}

ControllerOutput PID2DOFController::computeImpl(const ControllerInput& input) {
    double dt = input.dt;
    
    // Proportional term with setpoint weight
    double pError = m_b * input.reference - input.measured;
    double pTerm = getKp() * pError;
    
    // Integral term (always uses full error)
    // Integral is handled by base class accumulation logic
    
    // Derivative term with setpoint weight
    double dTerm = 0.0;
    double derivative = 0.0;
    
    if (dt > 0 && !m_firstIteration) {
        // Compute derivative on weighted reference minus measurement
        double dRef = (input.reference - m_lastReference) / dt;
        double dMeas = (input.measured - m_lastMeasurement2DOF) / dt;
        
        // Apply setpoint weight to derivative
        // c=0: derivative on measurement only (classic)
        // c=1: derivative on reference and measurement (full 2DOF)
        derivative = m_c * dRef - dMeas;
        
        dTerm = getKd() * derivative;
    }
    
    m_lastReference = input.reference;
    m_lastMeasurement2DOF = input.measured;
    m_firstIteration = false;
    
    // Compute using base class for integral and anti-windup
    ControllerOutput baseOutput = PIDController::computeImpl(input);
    
    // Adjust output for 2DOF
    baseOutput.proportional = pTerm;
    baseOutput.derivative = dTerm;
    baseOutput.control = pTerm + baseOutput.integral + dTerm;
    
    return baseOutput;
}

// ============================================================================
// Bang-Bang Controller
// ============================================================================

void BangBangController::setOutputLevels(double uMin, double uMax) {
    m_uMin = uMin;
    m_uMax = uMax;
}

void BangBangController::setNeutralOutput(double neutral, bool enable) {
    m_uNeutral = neutral;
    m_threeState = enable;
}

ControllerOutput BangBangController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double halfHyst = m_hysteresis / 2.0;
    
    double output;
    
    if (m_threeState) {
        // Three-state bang-bang (with neutral zone)
        if (error > halfHyst) {
            output = m_uMax;
            m_state = true;
        } else if (error < -halfHyst) {
            output = m_uMin;
            m_state = false;
        } else {
            output = m_uNeutral;
        }
    } else {
        // Standard two-state bang-bang with hysteresis
        if (error > halfHyst) {
            m_state = true;
        } else if (error < -halfHyst) {
            m_state = false;
        }
        // Else keep previous state (hysteresis)
        
        output = m_state ? m_uMax : m_uMin;
    }
    
    m_lastOutput = output;
    
    ControllerOutput result;
    result.control = output;
    result.error = error;
    
    return result;
}

void BangBangController::resetImpl() {
    m_state = false;
    m_lastOutput = m_uMin;
}

// ============================================================================
// PD+ Controller
// ============================================================================

void PDPlusController::setGains(double kp, double kd) {
    m_kp = kp;
    m_kd = kd;
}

ControllerOutput PDPlusController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double dt = input.dt;
    
    // Proportional term
    double pTerm = m_kp * error;
    
    // Derivative term
    double derivative = 0.0;
    double dTerm = 0.0;
    
    if (!m_firstRun && dt > 0) {
        // Use velocity error if reference derivative is provided
        if (std::abs(input.referenceDerivative) > 1e-10) {
            // Estimate measured derivative from position
            double measuredDeriv = (input.measured - m_lastMeasurement) / dt;
            // Direct velocity error
            derivative = input.referenceDerivative - measuredDeriv;
        } else {
            // Estimate from position
            derivative = -(input.measured - m_lastMeasurement) / dt;
        }
        
        // Apply filter
        if (m_tf > 0) {
            double alpha = dt / (m_tf + dt);
            m_filteredDerivative = alpha * derivative + 
                                   (1.0 - alpha) * m_filteredDerivative;
            derivative = m_filteredDerivative;
        }
        
        dTerm = m_kd * derivative;
    }
    
    // Compensation term
    double compensation = 0.0;
    double position = m_compOnDesired ? input.reference : input.measured;
    double velocity = m_compOnDesired ? input.referenceDerivative : derivative;
    
    if (m_fullCompensationFunc) {
        compensation = m_fullCompensationFunc(position, velocity);
    } else if (m_compensationFunc) {
        compensation = m_compensationFunc(position);
    }
    
    m_lastMeasurement = input.measured;
    m_firstRun = false;
    
    ControllerOutput output;
    output.proportional = pTerm;
    output.derivative = dTerm;
    output.feedforward = compensation;
    output.control = pTerm + dTerm + compensation;
    output.error = error;
    
    return output;
}

void PDPlusController::resetImpl() {
    m_lastMeasurement = 0.0;
    m_filteredDerivative = 0.0;
    m_firstRun = true;
}

// ============================================================================
// Dual Loop PID Controller
// ============================================================================

void DualLoopPIDController::setOuterGains(double kp, double ki, double kd) {
    m_outerPID.setGains(kp, ki, kd);
}

void DualLoopPIDController::setInnerGains(double kp, double ki, double kd) {
    m_innerPID.setGains(kp, ki, kd);
}

void DualLoopPIDController::setVelocityLimits(double min, double max) {
    m_velocityMin = min;
    m_velocityMax = max;
    
    SaturationLimits outerLimits;
    outerLimits.outputMin = min;
    outerLimits.outputMax = max;
    m_outerPID.setSaturationLimits(outerLimits);
}

void DualLoopPIDController::setTorqueLimits(double min, double max) {
    SaturationLimits innerLimits;
    innerLimits.outputMin = min;
    innerLimits.outputMax = max;
    m_innerPID.setSaturationLimits(innerLimits);
}

void DualLoopPIDController::setOuterAntiWindup(AntiWindupMethod method, double param) {
    m_outerPID.setAntiWindup(method, param);
}

void DualLoopPIDController::setInnerAntiWindup(AntiWindupMethod method, double param) {
    m_innerPID.setAntiWindup(method, param);
}

ControllerOutput DualLoopPIDController::computeImpl(const ControllerInput& input) {
    // Outer loop: position → velocity command
    ControllerInput outerInput = input;
    ControllerOutput outerOutput = m_outerPID.compute(outerInput);
    
    // Saturate velocity command
    m_velocityCommand = std::clamp(outerOutput.control, m_velocityMin, m_velocityMax);
    
    // Inner loop: velocity → torque
    ControllerInput innerInput;
    innerInput.reference = m_velocityCommand;
    innerInput.measured = m_velocity;
    innerInput.dt = input.dt;
    
    ControllerOutput innerOutput = m_innerPID.compute(innerInput);
    
    // Combine outputs
    ControllerOutput output;
    output.control = innerOutput.control;
    output.error = input.reference - input.measured;  // Position error
    output.proportional = outerOutput.proportional;
    output.integral = outerOutput.integral + innerOutput.integral;
    output.derivative = outerOutput.derivative + innerOutput.derivative;
    output.saturated = innerOutput.saturated;
    
    return output;
}

void DualLoopPIDController::resetImpl() {
    m_outerPID.reset();
    m_innerPID.reset();
    m_velocity = 0.0;
    m_velocityCommand = 0.0;
}

} // namespace tether::control
