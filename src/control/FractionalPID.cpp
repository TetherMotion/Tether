/**
 * @file FractionalPID.cpp
 * @brief Implementation of Fractional-Order PID Controller
 */

#include "control/FractionalPID.hpp"
#include <algorithm>
#include <cmath>

namespace tether::control {

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
    m_integralState.assign(m_oustaloupOrder, 0.0);
    m_derivState.assign(m_oustaloupOrder, 0.0);
    m_integralInPrev.assign(m_oustaloupOrder, 0.0);
    m_derivInPrev.assign(m_oustaloupOrder, 0.0);
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

namespace {

// Oustaloup recursive approximation (ORA) of s^alpha over
// [omegaLow, omegaHigh] with N first-order sections:
//   s^alpha ≈ omegaHigh^alpha · Prod_k (s + wz_k)/(s + wp_k)
//   wz_k = omegaLow·r^((2k-1-alpha)/(2N)),  wp_k = omegaLow·r^((2k-1+alpha)/(2N))
//   r = omegaHigh/omegaLow,  k = 1..N
// Verified exact at the band center for alpha = 1.
struct OustaloupSection {
    double a{0.0};    // y[n-1] coefficient
    double b0{0.0};   // x[n] coefficient
    double b1{0.0};   // x[n-1] coefficient
};

std::vector<OustaloupSection> oustaloupSections(double alpha, double omegaLow,
                                               double omegaHigh, int order,
                                               double dt) {
    std::vector<OustaloupSection> sections(order);
    if (omegaLow <= 0.0 || omegaHigh <= omegaLow || dt <= 0.0) {
        for (auto& s : sections) { s.a = 0.0; s.b0 = 1.0; s.b1 = 0.0; }
        return sections;
    }
    const double r = omegaHigh / omegaLow;
    const double twoN = 2.0 * order;
    for (int k = 0; k < order; ++k) {
        // k is 0-based; formulas use k' = k+1.
        const double eZ = (2.0 * (k + 1) - 1.0 - alpha) / twoN;
        const double eP = (2.0 * (k + 1) - 1.0 + alpha) / twoN;
        const double wz = omegaLow * std::pow(r, eZ);
        const double wp = omegaLow * std::pow(r, eP);

        // Tustin discretization of (s + wz)/(s + wp):
        //   a  = (2 - wp·dt)/(2 + wp·dt)
        //   b0 = (2 + wz·dt)/(2 + wp·dt)
        //   b1 = (2 - wz·dt)/(2 + wp·dt)
        const double den = 2.0 + wp * dt;
        sections[k].a  = (2.0 - wp * dt) / den;
        sections[k].b0 = (2.0 + wz * dt) / den;
        sections[k].b1 = (2.0 - wz * dt) / den;
    }
    return sections;
}

// Cascade the sections: x feeds section 0, each section's output feeds
// the next. Per-section states hold previous output (yPrev) and
// previous input (xPrev).
double oustaloupCascade(const std::vector<OustaloupSection>& sections,
                        double input,
                        std::vector<double>& yPrev,
                        std::vector<double>& xPrev) {
    if (yPrev.size() != sections.size()) yPrev.assign(sections.size(), 0.0);
    if (xPrev.size() != sections.size()) xPrev.assign(sections.size(), 0.0);

    double x = input;
    for (size_t k = 0; k < sections.size(); ++k) {
        const auto& s = sections[k];
        const double y = s.a * yPrev[k] + s.b0 * x + s.b1 * xPrev[k];
        xPrev[k] = x;
        yPrev[k] = y;
        x = y;
    }
    return x;
}

} // namespace

double FractionalPIDController::computeOustaloupIntegral(double error, double dt) {
    // ORA approximation of s^(-lambda)
    const auto sections = oustaloupSections(-m_lambda, m_omegaLow,
                                            m_omegaHigh, m_oustaloupOrder, dt);
    const double out = oustaloupCascade(sections, error,
                                        m_integralState, m_integralInPrev);
    return out * std::pow(m_omegaHigh, -m_lambda);
}

double FractionalPIDController::computeOustaloupDerivative(double error, double dt) {
    // ORA approximation of s^mu
    const auto sections = oustaloupSections(m_mu, m_omegaLow,
                                            m_omegaHigh, m_oustaloupOrder, dt);
    const double out = oustaloupCascade(sections, error,
                                        m_derivState, m_derivInPrev);
    return out * std::pow(m_omegaHigh, m_mu);
}

void FractionalPIDController::resetImpl() {
    m_errorHistory.clear();
    m_fractionalIntegral = 0.0;
    m_fractionalDerivative = 0.0;
    m_lastOutput = 0.0;
    
    std::fill(m_integralState.begin(), m_integralState.end(), 0.0);
    std::fill(m_derivState.begin(), m_derivState.end(), 0.0);
    std::fill(m_integralInPrev.begin(), m_integralInPrev.end(), 0.0);
    std::fill(m_derivInPrev.begin(), m_derivInPrev.end(), 0.0);
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
    
    // Oustaloup recursive approximation (ORA), k = 0..N-1:
    //   zeros: wz_k = ωb·r^((2k+1-α)/(2N))
    //   poles: wp_k = ωb·r^((2k+1+α)/(2N))
    //   gain  = ωh^α
    m_poles.resize(m_order);
    m_zeros.resize(m_order);
    const double omegaRatio = m_omegaHigh / m_omegaLow;
    const double twoN = 2.0 * m_order;
    
    for (int k = 0; k < m_order; k++) {
        m_zeros[k] = m_omegaLow * std::pow(omegaRatio,
            (2.0 * k + 1.0 - m_alpha) / twoN);
        m_poles[k] = m_omegaLow * std::pow(omegaRatio,
            (2.0 * k + 1.0 + m_alpha) / twoN);
    }
    
    // Gain
    m_gain = std::pow(m_omegaHigh, m_alpha);
    
    // Initialize state
    m_state.assign(m_order, 0.0);
    m_inPrev.assign(m_order, 0.0);
}

double OustaloupFilter::process(double input, double dt) {
    double output = input;
    
    for (int k = 0; k < m_order; k++) {
        // Each section: (s + wz_k) / (s + wp_k), Tustin-discretized:
        //   y[n] = a·y[n-1] + b0·x[n] + b1·x[n-1]
        const double wz = m_zeros[k];
        const double wp = m_poles[k];
        const double den = 2.0 + wp * dt;
        const double a  = (2.0 - wp * dt) / den;
        const double b0 = (2.0 + wz * dt) / den;
        const double b1 = (2.0 - wz * dt) / den;
        
        const double y = a * m_state[k] + b0 * output + b1 * m_inPrev[k];
        m_inPrev[k] = output;
        m_state[k] = y;
        output = y;
    }
    
    return output * m_gain;
}

void OustaloupFilter::reset() {
    std::fill(m_state.begin(), m_state.end(), 0.0);
    std::fill(m_inPrev.begin(), m_inPrev.end(), 0.0);
}

} // namespace tether::control
