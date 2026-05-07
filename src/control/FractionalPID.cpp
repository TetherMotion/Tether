/**
 * @file FractionalPID.cpp
 * @brief Implementation of Fractional-Order PID Controller
 */

#include "control/FractionalPID.hpp"
#include <algorithm>
#include <cmath>

namespace Control {

// ============================================================================
// Helper: Gamma function approximation for binomial coefficients
// ============================================================================

static double gammaApprox(double x) {
    // Stirling's approximation for gamma function
    // Γ(x) ≈ √(2π/x) * (x/e)^x
    if (x <= 0) return 1.0;
    if (x < 0.5) {
        // Reflection formula for small x
        return M_PI / (std::sin(M_PI * x) * gammaApprox(1.0 - x));
    }
    
    // Lanczos approximation coefficients
    static const double g = 7;
    static const double c[9] = {
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7
    };
    
    double z = x - 1;
    double ag = c[0];
    for (int k = 1; k < 9; k++) {
        ag += c[k] / (z + k);
    }
    
    double t = z + g + 0.5;
    return std::sqrt(2 * M_PI) * std::pow(t, z + 0.5) * std::exp(-t) * ag;
}

double FractionalPIDController::binomialCoeff(double alpha, int k) {
    // Generalized binomial coefficient using gamma function
    // (α k) = Γ(α+1) / (Γ(k+1) * Γ(α-k+1))
    
    if (k == 0) return 1.0;
    if (k < 0) return 0.0;
    
    // Use recursive formula for efficiency:
    // (α k) = (α k-1) * (α-k+1) / k
    double coeff = 1.0;
    for (int i = 0; i < k; i++) {
        coeff *= (alpha - i) / (i + 1.0);
    }
    return coeff;
}

// ============================================================================
// FractionalPIDController Implementation
// ============================================================================

void FractionalPIDController::setGains(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void FractionalPIDController::setOrders(double lambda, double mu) {
    // Constrain to reasonable ranges
    m_lambda = std::clamp(lambda, 0.01, 2.0);
    m_mu = std::clamp(mu, 0.01, 2.0);
    
    // Recompute GL coefficients
    computeGLCoefficients();
}

void FractionalPIDController::setApproximationMethod(FractionalApproximation method) {
    m_approxMethod = method;
    computeGLCoefficients();
}

void FractionalPIDController::setMemoryLength(size_t length) {
    m_memoryLength = std::max(length, size_t(10));
    
    // Resize error history
    while (m_errorHistory.size() > m_memoryLength) {
        m_errorHistory.pop_front();
    }
    
    // Recompute coefficients
    computeGLCoefficients();
}

void FractionalPIDController::setOustaloupParams(double omegaLow, double omegaHigh, 
                                                  int order) {
    m_omegaLow = omegaLow;
    m_omegaHigh = omegaHigh;
    m_oustaloupOrder = std::clamp(order, 1, 10);
    
    // Resize Oustaloup state vectors
    m_integralState.resize(m_oustaloupOrder, 0.0);
    m_derivState.resize(m_oustaloupOrder, 0.0);
}

void FractionalPIDController::setAntiWindup(AntiWindupMethod method, double param) {
    m_antiWindup = method;
    m_antiWindupParam = param;
}

void FractionalPIDController::setIntegralLimits(double min, double max) {
    m_integralMin = min;
    m_integralMax = max;
}

void FractionalPIDController::computeGLCoefficients() {
    size_t n = m_memoryLength;
    
    // Compute integral coefficients (for s^(-λ))
    // GL coefficients: c_j = (-1)^j * (-λ j)
    m_integralCoeffs.resize(n);
    for (size_t j = 0; j < n; j++) {
        m_integralCoeffs[j] = std::pow(-1.0, static_cast<double>(j)) * 
                             binomialCoeff(-m_lambda, static_cast<int>(j));
    }
    
    // Compute derivative coefficients (for s^μ)
    // GL coefficients: c_j = (-1)^j * (μ j)
    m_derivCoeffs.resize(n);
    for (size_t j = 0; j < n; j++) {
        m_derivCoeffs[j] = std::pow(-1.0, static_cast<double>(j)) * 
                          binomialCoeff(m_mu, static_cast<int>(j));
    }
}

double FractionalPIDController::computeFractionalIntegral(double dt) {
    if (m_errorHistory.empty() || dt <= 0) return 0.0;
    
    // GL discretization: D^(-λ)[f] ≈ h^λ * Σ c_j * f(t - j*h)
    double sum = 0.0;
    size_t n = std::min(m_errorHistory.size(), m_integralCoeffs.size());
    
    for (size_t j = 0; j < n; j++) {
        sum += m_integralCoeffs[j] * m_errorHistory[m_errorHistory.size() - 1 - j];
    }
    
    return std::pow(dt, m_lambda) * sum;
}

double FractionalPIDController::computeFractionalDerivative(double dt) {
    if (m_errorHistory.size() < 2 || dt <= 0) return 0.0;
    
    // GL discretization: D^μ[f] ≈ h^(-μ) * Σ c_j * f(t - j*h)
    double sum = 0.0;
    size_t n = std::min(m_errorHistory.size(), m_derivCoeffs.size());
    
    for (size_t j = 0; j < n; j++) {
        sum += m_derivCoeffs[j] * m_errorHistory[m_errorHistory.size() - 1 - j];
    }
    
    return std::pow(dt, -m_mu) * sum;
}

void FractionalPIDController::autoTuneIsoDamping(double processGain, double processLag,
                                                  double bandwidth, double phaseMargin) {
    double K = processGain;
    double T = processLag;
    double wc = bandwidth;
    double phi_m = phaseMargin * M_PI / 180.0;  // Convert to radians
    
    // For a first-order process G(s) = K / (1 + Ts):
    // At crossover frequency:
    //   |C(jωc)| * |G(jωc)| = 1
    //   ∠C(jωc) + ∠G(jωc) = -π + φm
    
    // For iso-damping, we want:
    //   d(∠C + ∠G)/dω |ωc = 0
    
    // Simplified design rules (Monje et al.):
    
    // Process phase at crossover
    double phi_G = -std::atan(wc * T);
    
    // Required controller phase
    // double phi_C = -M_PI + phi_m - phi_G; // Not used
    
    // Assume λ and μ are related to achieve iso-damping
    // Rule of thumb: μ + λ ≈ 1 for iso-damping
    
    // From phase condition:
    // φC = -λ*π/2 + μ*π/2 + atan2(...)
    
    // Simplified approach: use analytical formulas for FOPTD
    double alpha = 0.5;  // Start with balanced fractional orders
    
    // Compute λ for desired phase
    // Phase contribution of integrator: -λ*π/2
    m_lambda = 0.8;  // Typical starting value
    
    // Compute μ for iso-damping (phase flatness)
    // For iso-damping: slope of phase should be zero at ωc
    m_mu = 1.0 - m_lambda + 0.5;  // Heuristic relationship
    m_mu = std::clamp(m_mu, 0.3, 1.0);
    
    // Gain magnitude at crossover
    double magG = K / std::sqrt(1 + wc * wc * T * T);
    
    // Compute controller gains for |C(jωc)G(jωc)| = 1
    // |C(jωc)| = √(Kp² + (Ki/ωc^λ)² + (Kd*ωc^μ)²)
    
    // Simplified: use proportional to set magnitude, others for dynamics
    m_kp = 1.0 / magG;
    m_ki = m_kp * std::pow(wc, m_lambda) * 0.3;  // Integral contribution
    m_kd = m_kp / std::pow(wc, m_mu) * 0.1;      // Derivative contribution
    
    computeGLCoefficients();
}

void FractionalPIDController::autoTuneSIMC(double processGain, double timeConstant,
                                           double deadTime) {
    double K = processGain;
    double tau = timeConstant;
    double theta = deadTime;
    
    // SIMC-like rules adapted for FOPID
    // Closed-loop time constant
    double tc = std::max(tau / 2.0, 2.0 * theta);
    
    // Start with classical SIMC PID
    m_kp = tau / (K * (tc + theta));
    double ti = std::min(tau, 4.0 * (tc + theta));
    m_ki = m_kp / ti;
    m_kd = m_kp * theta / 2.0;
    
    // Adjust fractional orders for better robustness
    // Higher lambda for processes with dead time
    double deadTimeRatio = theta / (theta + tau);
    m_lambda = 1.0 - 0.2 * deadTimeRatio;  // Reduce lambda for large dead time
    
    // Fractional derivative helps with noise
    m_mu = 0.8;  // Typically use μ < 1 for noise reduction
    
    computeGLCoefficients();
}

ControllerOutput FractionalPIDController::computeImpl(const ControllerInput& input) {
    double error = input.reference - input.measured;
    double dt = input.dt;
    
    // Add error to history
    m_errorHistory.push_back(error);
    while (m_errorHistory.size() > m_memoryLength) {
        m_errorHistory.pop_front();
    }
    
    // Proportional term (standard)
    double pTerm = m_kp * error;
    
    // Fractional integral term
    double iTerm = 0.0;
    if (m_ki != 0) {
        switch (m_approxMethod) {
            case FractionalApproximation::GrunwaldLetnikov:
            case FractionalApproximation::ShortMemory:
                m_fractionalIntegral = computeFractionalIntegral(dt);
                break;
            case FractionalApproximation::Oustaloup:
            case FractionalApproximation::Matsuda:
                m_fractionalIntegral = computeOustaloupIntegral(error, dt);
                break;
        }
        iTerm = m_ki * m_fractionalIntegral;
    }
    
    // Fractional derivative term
    double dTerm = 0.0;
    if (m_kd != 0) {
        switch (m_approxMethod) {
            case FractionalApproximation::GrunwaldLetnikov:
            case FractionalApproximation::ShortMemory:
                m_fractionalDerivative = computeFractionalDerivative(dt);
                break;
            case FractionalApproximation::Oustaloup:
            case FractionalApproximation::Matsuda:
                m_fractionalDerivative = computeOustaloupDerivative(error, dt);
                break;
        }
        dTerm = m_kd * m_fractionalDerivative;
    }
    
    // Compute raw output
    double rawOutput = pTerm + iTerm + dTerm;
    
    // Apply saturation
    double saturatedOutput = std::clamp(rawOutput, 
                                        m_limits.outputMin,
                                        m_limits.outputMax);
    bool isSaturated = (rawOutput != saturatedOutput);
    
    // Anti-windup for fractional integral
    if (isSaturated && m_antiWindup != AntiWindupMethod::None) {
        // For fractional integral, we scale down the accumulated history
        if (m_antiWindup == AntiWindupMethod::Clamping) {
            // Don't add to history if saturated and error would make it worse
            if ((error > 0 && rawOutput > m_limits.outputMax) ||
                (error < 0 && rawOutput < m_limits.outputMin)) {
                if (!m_errorHistory.empty()) {
                    m_errorHistory.back() = 0;  // Zero out last error contribution
                }
            }
        }
    }
    
    // Clamp integral term
    iTerm = std::clamp(iTerm, m_integralMin, m_integralMax);
    
    m_lastOutput = saturatedOutput;
    
    ControllerOutput output;
    output.proportional = pTerm;
    output.integral = iTerm;
    output.derivative = dTerm;
    output.control = rawOutput;
    output.error = error;
    output.saturated = isSaturated;
    
    return output;
}

double FractionalPIDController::computeOustaloupIntegral(double error, double dt) {
    // Oustaloup approximation for s^(-λ)
    // Simplified implementation using recursive filter sections
    
    if (m_integralState.size() != static_cast<size_t>(m_oustaloupOrder)) {
        m_integralState.resize(m_oustaloupOrder, 0.0);
    }
    
    // Compute pole and zero frequencies
    double omegaRatio = m_omegaHigh / m_omegaLow;
    double output = error;
    
    for (int k = 0; k < m_oustaloupOrder; k++) {
        // Zero and pole for this section
        double kNorm = (k - (m_oustaloupOrder - 1) / 2.0 + 0.5) / m_oustaloupOrder;
        double omegaZero = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 + m_lambda) / 2.0);
        double omegaPole = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 - m_lambda) / 2.0);
        
        // First-order section: (s + ωz) / (s + ωp)
        // Tustin: y[n] = a*y[n-1] + b*(x[n] + x[n-1])
        double alpha = (2.0 - omegaPole * dt) / (2.0 + omegaPole * dt);
        double beta = dt * omegaZero / (2.0 + omegaPole * dt);
        
        double newState = alpha * m_integralState[k] + beta * (output + output);
        output = newState;
        m_integralState[k] = newState;
    }
    
    // Apply gain factor
    double gain = std::pow(m_omegaHigh, -m_lambda);
    return output * gain;
}

double FractionalPIDController::computeOustaloupDerivative(double error, double dt) {
    // Oustaloup approximation for s^μ
    
    if (m_derivState.size() != static_cast<size_t>(m_oustaloupOrder)) {
        m_derivState.resize(m_oustaloupOrder, 0.0);
    }
    
    double omegaRatio = m_omegaHigh / m_omegaLow;
    double output = error;
    
    for (int k = 0; k < m_oustaloupOrder; k++) {
        double kNorm = (k - (m_oustaloupOrder - 1) / 2.0 + 0.5) / m_oustaloupOrder;
        double omegaZero = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 - m_mu) / 2.0);
        double omegaPole = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 + m_mu) / 2.0);
        
        double alpha = (2.0 - omegaPole * dt) / (2.0 + omegaPole * dt);
        double beta = (2.0 * omegaZero) / (2.0 + omegaPole * dt);
        
        double newState = alpha * m_derivState[k] + beta * (output - m_derivState[k]);
        output = newState;
        m_derivState[k] = newState;
    }
    
    double gain = std::pow(m_omegaHigh, m_mu);
    return output * gain;
}

void FractionalPIDController::resetImpl() {
    m_errorHistory.clear();
    m_fractionalIntegral = 0.0;
    m_fractionalDerivative = 0.0;
    m_lastOutput = 0.0;
    
    std::fill(m_integralState.begin(), m_integralState.end(), 0.0);
    std::fill(m_derivState.begin(), m_derivState.end(), 0.0);
}

// ============================================================================
// OustaloupFilter Implementation
// ============================================================================

void OustaloupFilter::configure(double alpha, double omegaLow, double omegaHigh, 
                                 int order) {
    m_alpha = std::clamp(alpha, -0.99, 0.99);
    m_omegaLow = omegaLow;
    m_omegaHigh = omegaHigh;
    m_order = std::clamp(order, 1, 10);
    
    // Compute poles and zeros
    m_poles.resize(m_order);
    m_zeros.resize(m_order);
    
    double omegaRatio = m_omegaHigh / m_omegaLow;
    
    for (int k = 0; k < m_order; k++) {
        double kNorm = (2.0 * k - m_order + 1) / (2.0 * m_order);
        
        // Zeros: ω'_k = ωb * (ωh/ωb)^((k+N+(1+α)/2)/(2N+1))
        m_zeros[k] = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 + m_alpha) / 2.0);
        
        // Poles: ω_k = ωb * (ωh/ωb)^((k+N+(1-α)/2)/(2N+1))
        m_poles[k] = m_omegaLow * std::pow(omegaRatio, kNorm + (1.0 - m_alpha) / 2.0);
    }
    
    // Gain
    m_gain = std::pow(m_omegaHigh, m_alpha);
    
    // Initialize state
    m_state.resize(m_order, 0.0);
}

double OustaloupFilter::process(double input, double dt) {
    double output = input;
    
    for (int k = 0; k < m_order; k++) {
        // Each section: (s + ω'_k) / (s + ω_k)
        // Discretized using Tustin
        double wz = m_zeros[k];
        double wp = m_poles[k];
        
        double a = (2.0 - wp * dt) / (2.0 + wp * dt);
        double b0 = (2.0 + wz * dt) / (2.0 + wp * dt);
        double b1 = (-2.0 + wz * dt) / (2.0 + wp * dt);
        
        double newOutput = a * m_state[k] + b0 * output;
        m_state[k] = newOutput;
        output = newOutput;
    }
    
    return output * m_gain;
}

void OustaloupFilter::reset() {
    std::fill(m_state.begin(), m_state.end(), 0.0);
}

} // namespace Control
