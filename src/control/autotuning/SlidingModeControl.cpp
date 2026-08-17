/**
 * @file SlidingModeControl.cpp
 * @brief Implementation of Sliding Mode Control methods
 */

#include "tether/control/autotuning/SlidingModeControl.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tether::control {

// ============================================================================
// SlidingModeController Implementation
// ============================================================================

void SlidingModeController::setSystemModel(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    
    for (int i = 0; i < n * n; i++) {
        m_A[i] = A[i];
    }
    for (int i = 0; i < n * m; i++) {
        m_B[i] = B[i];
    }
    m_hasLinearModel = true;
}

void SlidingModeController::setSecondOrderModel(double naturalFreq, double damping,
                                                 double inputGain) {
    // ẋ = [x2; -wn^2*x1 - 2*zeta*wn*x2] + [0; b]*u
    m_n = 2;
    m_m = 1;
    
    double wn2 = naturalFreq * naturalFreq;
    double twoZetaWn = 2.0 * damping * naturalFreq;
    
    // A = [0, 1; -wn^2, -2*zeta*wn]
    m_A[0] = 0.0;     m_A[1] = 1.0;
    m_A[2] = -wn2;    m_A[3] = -twoZetaWn;
    
    // B = [0; b]
    m_B[0] = 0.0;
    m_B[1] = inputGain;
    
    m_hasLinearModel = true;
}

void SlidingModeController::setNonlinearModel(
    std::function<StateVector(const StateVector&)> f,
    std::function<StateVector(const StateVector&)> g) {
    m_f = f;
    m_g = g;
    m_hasNonlinearModel = true;
}

void SlidingModeController::setSlidingSurface(const double* C, int n) {
    for (int i = 0; i < n && i < MAX_STATE_DIM; i++) {
        m_C[i] = C[i];
    }
    m_surfaceParams.type = SurfaceType::Linear;
    m_surfaceParams.coefficients.assign(C, C + n);
}

void SlidingModeController::setSlidingSurface(double lambda, int order) {
    // For tracking: σ = e + λė + λ²ë/2 + ... (for higher orders)
    // Simple case: σ = e + λė (second order)
    m_surfaceParams.type = SurfaceType::Linear;
    m_surfaceParams.coefficients.clear();
    
    // Coefficients for error and its derivatives
    // σ = c₀e + c₁ė + c₂ë + ...
    // For stable sliding: eigenvalue at -λ
    for (int i = 0; i < order; i++) {
        double coeff = std::pow(lambda, order - 1 - i);
        m_surfaceParams.coefficients.push_back(coeff);
        if (i < MAX_STATE_DIM) {
            m_C[i] = coeff;
        }
    }
}

void SlidingModeController::setSurfaceParams(const SlidingSurfaceParams& params) {
    m_surfaceParams = params;
    for (size_t i = 0; i < params.coefficients.size() && i < MAX_STATE_DIM; i++) {
        m_C[i] = params.coefficients[i];
    }
}

void SlidingModeController::enableEquivalentEstimation(bool enable, double filterConst) {
    m_estimateUeq = enable;
    m_filterConst = filterConst;
}

void SlidingModeController::enableAdaptiveGain(bool enable, double gamma, double threshold) {
    m_adaptiveGain = enable;
    m_adaptiveGamma = gamma;
    m_adaptiveThreshold = threshold;
}

bool SlidingModeController::isReachable() const {
    // Check reachability: σσ̇ < 0
    return m_sigma * m_sigmaDot < 0;
}

double SlidingModeController::estimateReachingTime(double sigma0) const {
    double K = m_gains.switchingGain;
    double lambda = m_gains.proportionalGain;
    
    if (lambda > 0) {
        // σ̇ = -K·sign(σ) - λσ
        // Reaching time: t_r = (1/λ)ln(1 + λ|σ₀|/K)
        return (1.0 / lambda) * std::log(1.0 + lambda * std::abs(sigma0) / K);
    } else {
        // σ̇ = -K·sign(σ)
        // Reaching time: t_r = |σ₀|/K
        return std::abs(sigma0) / K;
    }
}

double SlidingModeController::computeSigma(const StateVector& error) const {
    double sigma = 0.0;
    
    switch (m_surfaceParams.type) {
        case SurfaceType::Linear:
            // σ = Σ cᵢ·eᵢ
            for (size_t i = 0; i < m_surfaceParams.coefficients.size() && i < error.size(); i++) {
                sigma += m_surfaceParams.coefficients[i] * error[i];
            }
            break;
            
        case SurfaceType::Integral:
            // σ = Σ cᵢ·eᵢ + Ki·∫e
            for (size_t i = 0; i < m_surfaceParams.coefficients.size() && i < error.size(); i++) {
                sigma += m_surfaceParams.coefficients[i] * error[i];
            }
            sigma += m_surfaceParams.integralGain * m_sigmaIntegral;
            break;
            
        case SurfaceType::Terminal:
            // σ = ė + β·e^(p/q)
            if (error.size() >= 2) {
                double e = error[0];
                double eDot = error[1];
                double pq = m_surfaceParams.terminalP / m_surfaceParams.terminalQ;
                sigma = eDot + m_surfaceParams.terminalBeta * 
                        std::pow(std::abs(e), pq) * (e >= 0 ? 1 : -1);
            }
            break;
            
        case SurfaceType::NonsingularTerminal:
            // Modified to avoid singularity
            if (error.size() >= 2) {
                double e = error[0];
                double eDot = error[1];
                double qp = m_surfaceParams.terminalQ / m_surfaceParams.terminalP;
                sigma = e + (1.0 / m_surfaceParams.terminalBeta) * 
                        std::pow(std::abs(eDot), qp) * (eDot >= 0 ? 1 : -1);
            }
            break;
            
        default:
            // Default linear
            for (int i = 0; i < m_n && i < static_cast<int>(error.size()); i++) {
                sigma += m_C[i] * error[i];
            }
            break;
    }
    
    return sigma;
}

double SlidingModeController::computeSigmaDot(const StateVector& error, 
                                               const StateVector& errorDot) const {
    double sigmaDot = 0.0;
    
    // For linear surface: σ̇ = Σ cᵢ·ėᵢ
    for (size_t i = 0; i < m_surfaceParams.coefficients.size() && i < errorDot.size(); i++) {
        sigmaDot += m_surfaceParams.coefficients[i] * errorDot[i];
    }
    
    // Add integral term if present
    if (m_surfaceParams.type == SurfaceType::Integral && !error.empty()) {
        sigmaDot += m_surfaceParams.integralGain * error[0];
    }
    
    return sigmaDot;
}

double SlidingModeController::computeEquivalentControl(const StateVector& state,
                                                        const StateVector& reference) const {
    // Equivalent control: u_eq such that σ̇ = 0 (stays on surface)
    // For ẋ = Ax + Bu: σ̇ = CA(x-x_d) + CBu - Cẋ_d = 0
    // u_eq = -(CB)^(-1)(CAe - Cẋ_d)
    
    if (!m_hasLinearModel) return 0.0;
    
    // Simplified for SISO: u_eq = -CAe / CB (assuming scalar CB)
    double CAe = 0.0;
    double CB = 0.0;
    
    for (int i = 0; i < m_n; i++) {
        double Ai_dot_e = 0.0;
        for (int j = 0; j < m_n; j++) {
            double error = state[j] - reference[j];
            Ai_dot_e += m_A[i * m_n + j] * error;
        }
        CAe += m_C[i] * Ai_dot_e;
        CB += m_C[i] * m_B[i];
    }
    
    if (std::abs(CB) < 1e-10) return 0.0;
    
    return -CAe / CB;
}

double SlidingModeController::computeSwitchingControl(double sigma) const {
    double u_sw = 0.0;
    double K = m_adaptiveGain ? m_adaptedK : m_gains.switchingGain;
    double sign_sigma = (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
    
    switch (m_reachingLaw) {
        case ReachingLaw::Constant:
            // σ̇ = -K·sign(σ)
            u_sw = K * sign_sigma;
            break;
            
        case ReachingLaw::ConstantPlusProportional:
            // σ̇ = -K·sign(σ) - λσ
            u_sw = K * sign_sigma + m_gains.proportionalGain * sigma;
            break;
            
        case ReachingLaw::PowerRate:
            // σ̇ = -K|σ|^α·sign(σ), 0 < α < 1
            u_sw = K * std::pow(std::abs(sigma), m_gains.powerAlpha) * sign_sigma;
            break;
            
        case ReachingLaw::Exponential:
            // σ̇ = -K·exp(ε|σ|)·sign(σ)
            u_sw = K * std::exp(-m_gains.proportionalGain * std::abs(sigma)) * sign_sigma;
            break;
    }
    
    return u_sw;
}

double SlidingModeController::switchingFunction(double sigma) const {
    double phi = m_gains.boundaryWidth;
    
    switch (m_chatterReduction) {
        case ChatteringReduction::None:
            return (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
            
        case ChatteringReduction::Saturation:
            // sat(σ/φ)
            if (std::abs(sigma) > phi) {
                return (sigma > 0) ? 1.0 : -1.0;
            }
            return sigma / phi;
            
        case ChatteringReduction::Sigmoid:
            // 2/(1 + exp(-k·σ)) - 1
            return 2.0 / (1.0 + std::exp(-10.0 * sigma / phi)) - 1.0;
            
        case ChatteringReduction::Hyperbolic:
            // tanh(σ/φ)
            return std::tanh(sigma / phi);
            
        case ChatteringReduction::SuperTwisting:
            // Handled by SuperTwistingController
            return (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
    }
    
    return (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
}

ControllerOutput SlidingModeController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    // Build error state (simplified for tracking control)
    StateVector error;
    error[0] = r - y;
    
    // Compute error derivative
    StateVector errorDot;
    errorDot[0] = (error[0] - m_lastError[0]) / dt;
    error[1] = errorDot[0];  // For second-order systems
    
    // Compute sliding variable
    m_sigma = computeSigma(error);
    m_sigmaDot = computeSigmaDot(error, errorDot);
    
    // Update integral for integral SMC
    if (m_surfaceParams.type == SurfaceType::Integral) {
        m_sigmaIntegral += error[0] * dt;
    }
    
    // Compute equivalent control
    StateVector state;
    state[0] = y;
    state[1] = (y - m_lastState[0]) / dt;
    
    StateVector reference;
    reference[0] = r;
    reference[1] = 0;  // Assume constant reference
    
    m_uEq = computeEquivalentControl(state, reference);
    
    // If estimating u_eq from σ dynamics
    if (m_estimateUeq) {
        // Low-pass filter equivalent control
        m_uEqFiltered += (m_uEq - m_uEqFiltered) * dt / (m_filterConst + dt);
        m_uEq = m_uEqFiltered;
    }
    
    // Compute switching control
    double u_sw_raw = computeSwitchingControl(m_sigma);
    m_uSw = u_sw_raw * switchingFunction(m_sigma);
    
    // Adapt gain if enabled
    if (m_adaptiveGain) {
        if (std::abs(m_sigma) > m_adaptiveThreshold) {
            m_adaptedK += m_adaptiveGamma * std::abs(m_sigma) * dt;
            m_adaptedK = std::min(m_adaptedK, m_gains.switchingGain * 10.0);
        } else {
            m_adaptedK -= m_adaptiveGamma * 0.1 * dt;
            m_adaptedK = std::max(m_adaptedK, m_gains.switchingGain * 0.1);
        }
    }
    
    // Total control
    double u = m_uEq + m_uSw;
    
    // Apply saturation
    u = std::max(m_uMin, std::min(m_uMax, u));
    
    // Update state memory
    m_lastState = state;
    m_lastError = error;
    m_lastSigma = m_sigma;
    
    output.control = u;
    output.error = error[0];
    
    return output;
}

void SlidingModeController::resetImpl() {
    m_sigma = 0.0;
    m_sigmaDot = 0.0;
    m_sigmaIntegral = 0.0;
    m_uEq = 0.0;
    m_uSw = 0.0;
    m_uEqFiltered = 0.0;
    m_adaptedK = m_gains.switchingGain;
    m_lastState.fill(0.0);
    m_lastError.fill(0.0);
    m_lastSigma = 0.0;
}

// ============================================================================
// SuperTwistingController Implementation
// ============================================================================

void SuperTwistingController::setSystemModel(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
}

void SuperTwistingController::setSlidingSurface(const double* C, int n) {
    for (int i = 0; i < n && i < MAX_STATE_DIM; i++) {
        m_C[i] = C[i];
    }
}

void SuperTwistingController::setGains(double K1, double K2) {
    m_K1 = K1;
    m_K2 = K2;
}

void SuperTwistingController::setGainsFromDisturbance(double dMax, double dDotMax) {
    // Sufficient conditions for finite-time convergence:
    // K1 > 2√D, K2 > D (for σ̈ bounded by D)
    // With safety margin
    double D = dMax + dDotMax;
    m_K1 = 3.0 * std::sqrt(D);
    m_K2 = 1.5 * D;
}

ControllerOutput SuperTwistingController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    // Compute error
    double e = r - y;
    
    // Compute sliding variable (simple proportional)
    m_sigma = e;
    
    // Super-twisting algorithm:
    // u = -K1|σ|^(1/2)·sign(σ) + v
    // v̇ = -K2·sign(σ)
    
    double sign_sigma = (m_sigma > 0) ? 1.0 : ((m_sigma < 0) ? -1.0 : 0.0);
    double sqrt_sigma = std::sqrt(std::abs(m_sigma));
    
    // Proportional term
    double u_prop = -m_K1 * sqrt_sigma * sign_sigma;
    
    // Integral term (integrating -K2·sign(σ))
    m_v += -m_K2 * sign_sigma * dt;
    
    double u = u_prop + m_v;
    
    output.control = u;
    output.error = e;
    
    return output;
}

void SuperTwistingController::resetImpl() {
    m_v = 0.0;
    m_sigma = 0.0;
}

// ============================================================================
// TerminalSlidingModeController Implementation
// ============================================================================

void TerminalSlidingModeController::setTerminalParameters(double beta, int p, int q) {
    m_beta = beta;
    m_p = p;
    m_q = q;
    
    // Validate: p and q must be odd, p > q
    if (m_p % 2 == 0) m_p++;
    if (m_q % 2 == 0) m_q++;
    if (m_p <= m_q) m_p = m_q + 2;
}

void TerminalSlidingModeController::setSystemModel(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
}

double TerminalSlidingModeController::estimateConvergenceTime(double error) const {
    // For terminal SMC: t_s = (q/(β(p-q)))|e(0)|^((p-q)/q)
    double pq_diff = static_cast<double>(m_p - m_q);
    double exponent = pq_diff / m_q;
    return (m_q / (m_beta * pq_diff)) * std::pow(std::abs(error), exponent);
}

ControllerOutput TerminalSlidingModeController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    double e = r - y;
    m_errorDot = (e - m_lastError) / dt;
    m_lastError = e;
    
    // Terminal sliding surface: σ = ė + β·e^(p/q)
    double pq = static_cast<double>(m_p) / m_q;
    double e_pq = std::pow(std::abs(e), pq);
    double sign_e = (e >= 0) ? 1.0 : -1.0;
    
    if (m_nonsingular) {
        // Non-singular: σ = e + (1/β)·ė^(q/p)
        double qp = static_cast<double>(m_q) / m_p;
        double eDot_qp = std::pow(std::abs(m_errorDot), qp);
        double sign_eDot = (m_errorDot >= 0) ? 1.0 : -1.0;
        m_sigma = e + (1.0 / m_beta) * eDot_qp * sign_eDot;
    } else {
        // Standard: σ = ė + β·e^(p/q)
        m_sigma = m_errorDot + m_beta * e_pq * sign_e;
    }
    
    // Control law
    double sign_sigma = (m_sigma > 0) ? 1.0 : ((m_sigma < 0) ? -1.0 : 0.0);
    
    // Use boundary layer for chattering reduction
    if (std::abs(m_sigma) < m_phi) {
        sign_sigma = m_sigma / m_phi;
    }
    
    double u = m_K * sign_sigma;
    
    output.control = u;
    output.error = e;
    
    return output;
}

void TerminalSlidingModeController::resetImpl() {
    m_sigma = 0.0;
    m_lastError = 0.0;
    m_errorDot = 0.0;
}

// ============================================================================
// IntegralSlidingModeController Implementation
// ============================================================================

void IntegralSlidingModeController::setSystemModel(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
}

void IntegralSlidingModeController::setNominalControl(
    std::function<double(const StateVector&, double)> uNom) {
    m_uNom = uNom;
}

void IntegralSlidingModeController::setIntegralGain(const double* Ki, int n) {
    for (int i = 0; i < n && i < MAX_STATE_DIM; i++) {
        m_Ki[i] = Ki[i];
    }
}

void IntegralSlidingModeController::initialize(const StateVector& x0, const StateVector& xd0) {
    // Initialize integral state for smooth start (σ(0) = 0)
    m_integralState.fill(0.0);
    // The integral is set such that σ(0) = x0 - xd0 + integral = 0
    for (int i = 0; i < m_n; i++) {
        m_integralState[i] = -(x0[i] - xd0[i]);
    }
}

ControllerOutput IntegralSlidingModeController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    // Build state (simplified)
    StateVector state;
    state[0] = y;
    
    // Compute error
    double e = r - y;
    
    // Update integral
    m_integralState[0] += e * dt;
    
    // Compute sliding variable: σ = e + Ki·∫e
    m_sigma = e;
    for (int i = 0; i < m_n; i++) {
        m_sigma += m_Ki[i] * m_integralState[i];
    }
    
    // Nominal control
    double u_nom = 0.0;
    if (m_uNom) {
        u_nom = m_uNom(state, r);
    }
    
    // Discontinuous control
    double sign_sigma = (m_sigma > 0) ? 1.0 : ((m_sigma < 0) ? -1.0 : 0.0);
    
    // Use boundary layer
    if (std::abs(m_sigma) < m_phi) {
        sign_sigma = m_sigma / m_phi;
    }
    
    double u_disc = m_K * sign_sigma;
    
    double u = u_nom + u_disc;
    
    output.control = u;
    output.error = e;
    
    return output;
}

void IntegralSlidingModeController::resetImpl() {
    m_integralState.fill(0.0);
    m_sigma = 0.0;
}

// ============================================================================
// AdaptiveSlidingModeController Implementation
// ============================================================================

void AdaptiveSlidingModeController::setSystemModel(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
}

void AdaptiveSlidingModeController::setSlidingSurface(const double* C, int n) {
    for (int i = 0; i < n && i < MAX_STATE_DIM; i++) {
        m_C[i] = C[i];
    }
}

void AdaptiveSlidingModeController::setAdaptationParams(double gamma, double Kmin, double Kmax) {
    m_gamma = gamma;
    m_Kmin = Kmin;
    m_Kmax = Kmax;
    m_K = Kmin;
}

ControllerOutput AdaptiveSlidingModeController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    double e = r - y;
    
    // Compute sliding variable
    m_sigma = m_C[0] * e;
    
    // Adapt gain
    // K̇ = γ|σ| when outside boundary layer
    if (std::abs(m_sigma) > m_phi) {
        m_K += m_gamma * std::abs(m_sigma) * dt;
    } else {
        // Slowly decrease gain when on surface
        m_K -= m_gamma * 0.01 * dt;
    }
    m_K = std::max(m_Kmin, std::min(m_Kmax, m_K));
    
    // Estimate disturbance (simplified)
    m_dHat = m_K * 0.1;  // Rough estimate
    
    // Switching control
    double sign_sigma = (m_sigma > 0) ? 1.0 : ((m_sigma < 0) ? -1.0 : 0.0);
    
    // Boundary layer
    if (std::abs(m_sigma) < m_phi) {
        sign_sigma = m_sigma / m_phi;
    }
    
    double u = m_K * sign_sigma;
    
    output.control = u;
    output.error = e;
    
    return output;
}

void AdaptiveSlidingModeController::resetImpl() {
    m_sigma = 0.0;
    m_K = m_Kmin;
    m_dHat = 0.0;
}

// ============================================================================
// SMCDesignUtils Implementation
// ============================================================================

std::vector<double> SMCDesignUtils::designSurface(
    const double* A, const double* B, int n,
    const std::vector<std::complex<double>>& desiredPoles) {
    
    std::vector<double> C(n, 0.0);
    
    // For controllable systems, design surface using Ackermann's formula
    // Simplified: for second-order systems
    if (n == 2 && desiredPoles.size() >= 1) {
        // Surface σ = c1*x1 + c2*x2 = 0
        // Sliding dynamics: ẋ1 = x2, sliding mode at desired pole
        double p = desiredPoles[0].real();
        C[0] = -p;
        C[1] = 1.0;
    } else {
        // Default: C = [1, 1, ..., 1]
        for (int i = 0; i < n; i++) {
            C[i] = 1.0;
        }
    }
    
    return C;
}

double SMCDesignUtils::minimumSwitchingGain(double disturbanceBound, double safetyMargin) {
    return disturbanceBound * (1.0 + safetyMargin);
}

double SMCDesignUtils::reachingTime(double sigma0, double K, double lambda) {
    if (lambda > 0) {
        return (1.0 / lambda) * std::log(1.0 + lambda * std::abs(sigma0) / K);
    }
    return std::abs(sigma0) / K;
}

bool SMCDesignUtils::checkLyapunovCondition(double sigma, double sigmaDot, double boundaryLayer) {
    if (std::abs(sigma) <= boundaryLayer) {
        return true;  // Inside boundary layer
    }
    return sigma * sigmaDot < 0;  // V̇ = σσ̇ < 0
}

double SMCDesignUtils::chatteringAmplitude(double K, double phi, double systemBandwidth) {
    // Chattering amplitude ≈ K / (system_bandwidth * φ)
    if (phi > 0 && systemBandwidth > 0) {
        return K / (systemBandwidth * phi);
    }
    return K;  // Without boundary layer, full amplitude
}

// ============================================================================
// HigherOrderSMC Implementation
// ============================================================================

void HigherOrderSMC::setOrder(int order) {
    m_order = order;
    m_integralStates.resize(order - 1, 0.0);
}

void HigherOrderSMC::setGains(const std::vector<double>& gains) {
    m_gains = gains;
}

double HigherOrderSMC::compute(double sigma, const std::vector<double>& sigmaDerivatives,
                                double dt) {
    double u = 0.0;
    
    // Higher-order SMC drives σ, σ̇, ..., σ^(r-1) to zero
    // Using nested structure or super-twisting extensions
    
    if (m_order == 1) {
        // First order: u = -K·sign(σ)
        double sign_sigma = (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
        u = -m_gains[0] * sign_sigma;
    } else if (m_order == 2) {
        // Second order (super-twisting)
        double sign_sigma = (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
        double sqrt_sigma = std::sqrt(std::abs(sigma));
        
        u = -m_gains[0] * sqrt_sigma * sign_sigma;
        if (!m_integralStates.empty()) {
            m_integralStates[0] += -m_gains[1] * sign_sigma * dt;
            u += m_integralStates[0];
        }
    } else {
        // General higher-order: recursive structure
        // Simplified implementation
        double sign_sigma = (sigma > 0) ? 1.0 : ((sigma < 0) ? -1.0 : 0.0);
        for (size_t i = 0; i < m_gains.size(); i++) {
            double power = 1.0 - static_cast<double>(i) / m_order;
            u -= m_gains[i] * std::pow(std::abs(sigma), power) * sign_sigma;
        }
    }
    
    return u;
}

} // namespace tether::control
