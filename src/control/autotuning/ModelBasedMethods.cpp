// Implementation moved to individual files under
// src/control/autotuning/model_based/*.cpp

#include "tether/control/autotuning/ModelBasedMethods.hpp"

// This file intentionally left as a stub to avoid duplicate symbols.

// ============================================================================
// IMCDesign Implementation
// ============================================================================

bool IMCDesign::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("PI") != std::string::npos ||
           name.find("IMC") != std::string::npos;
}

TuningResult IMCDesign::tune(TunableController& controller,
                             const ProcessModel* model) {
    TuningResult result;
    
    // Get model from either provided model or stored model
    FOPDTModel fopdt;
    
    if (model) {
        fopdt = model->toFOPDT();
    } else if (m_useFOPDT) {
        fopdt = m_fopdtModel;
    } else {
        // Convert SOPDT to equivalent FOPDT
        fopdt.K = m_sopdtModel.K;
        fopdt.tau = m_sopdtModel.tau1 + m_sopdtModel.tau2;
        fopdt.L = m_sopdtModel.L;
    }
    
    if (fopdt.K == 0 || fopdt.tau <= 0) {
        result.success = false;
        result.message = "Invalid model parameters";
        return result;
    }
    
    // Design IMC-PID
    PIDGains gains;
    if (m_useFOPDT || model) {
        gains = designForFOPDT(fopdt, m_lambda, m_filterOrder);
    } else {
        gains = designForSOPDT(m_sopdtModel, m_lambda);
    }
    
    // Convert to parameter vector
    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;
    
    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "IMC design complete";
    result.settlingTime = m_lambda * 4;
    
    controller.setParameters(result.parameters);
    
    return result;
}

PIDGains IMCDesign::designForFOPDT(const FOPDTModel& model, 
                                   double lambda, int filterOrder) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    // IMC filter: f(s) = 1 / (λs + 1)^n
    // For FOPDT with first-order filter:
    // Q = (τs + 1) / (K(λs + 1))
    // C = Q / (1 - Gm*Q) where Gm = K*exp(-Ls)/(τs+1)
    
    // Using first-order Padé approximation for dead time:
    // exp(-Ls) ≈ (1 - Ls/2) / (1 + Ls/2)
    
    // Resulting PID parameters (IMC-PID):
    if (filterOrder == 1) {
        // First-order IMC filter
        double lambdaEff = lambda + L/2;  // Effective lambda with half dead time
        
        gains.Kp = tau / (K * lambdaEff);
        gains.Ti = tau;
        gains.Td = L / 2;
    } else {
        // Second-order IMC filter
        double lambdaEff = lambda;
        
        gains.Kp = (tau + L/2) / (K * (lambdaEff + L/2));
        gains.Ti = tau + L/2;
        gains.Td = tau * L / (2*tau + L);
    }
    
    return gains;
}

PIDGains IMCDesign::designForSOPDT(const SOPDTModel& model, double lambda) {
    PIDGains gains;
    
    double K = model.K;
    double tau1 = model.tau1;
    double tau2 = model.tau2;
    double L = model.L;
    
    // For SOPDT: G = K*exp(-Ls) / ((τ1s+1)(τ2s+1))
    // IMC-based PID:
    double tauSum = tau1 + tau2;
    double tauProd = tau1 * tau2;
    
    gains.Kp = tauSum / (K * (lambda + L));
    gains.Ti = tauSum;
    gains.Td = tauProd / tauSum;
    
    return gains;
}

PIDGains IMCDesign::designForIPDT(const IPDTModel& model, double lambda) {
    PIDGains gains;
    
    double K = model.K;  // Integrator gain
    double L = model.L;  // Dead time
    
    // For integrating process: G = K*exp(-Ls)/s
    // IMC-based PI:
    gains.Kp = 2 / (K * (2*lambda + L));
    gains.Ti = 2*lambda + L;
    gains.Td = 0.0;  // No derivative for integrating processes
    
    return gains;
}

// ============================================================================
// PolePlacement Implementation
// ============================================================================

bool PolePlacement::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("State") != std::string::npos ||
           name.find("PID") != std::string::npos ||
           name.find("LQR") != std::string::npos;
}

TuningResult PolePlacement::tune(TunableController& controller,
                                 const ProcessModel* model) {
    TuningResult result;
    
    if (m_desiredPoles.empty()) {
        result.success = false;
        result.message = "No desired poles specified";
        return result;
    }
    
    // If model provided and no system matrices set, convert model
    if (model && m_A.empty()) {
        auto fopdt = model->toFOPDT();
        
        // Create state-space from FOPDT (2nd order approximation)
        // Using Padé approximation for delay
        m_n = 2;
        m_m = 1;
        m_A.resize(m_n * m_n);
        m_B.resize(m_n * m_m);
        
        double tau = fopdt.tau;
        double K = fopdt.K;
        double L = fopdt.L;
        
        // State-space with Padé delay approximation
        // A = [0, 1; -2/(τL), -(2τ+L)/(τL)]
        // B = [0; 2K/(τL)]
        m_A[0] = 0.0;
        m_A[1] = 1.0;
        m_A[2] = -2.0 / (tau * L + 0.01);
        m_A[3] = -(2.0*tau + L) / (tau * L + 0.01);
        
        m_B[0] = 0.0;
        m_B[1] = 2.0 * K / (tau * L + 0.01);
    }
    
    if (m_A.empty() || m_B.empty()) {
        result.success = false;
        result.message = "No system matrices specified";
        return result;
    }
    
    // Compute state feedback gain using Ackermann's formula
    std::vector<double> K_fb = computeGain(m_A.data(), m_B.data(), 
                                           m_n, m_desiredPoles);
    
    if (K_fb.empty()) {
        result.success = false;
        result.message = "Pole placement computation failed";
        return result;
    }
    
    // Convert to PID if system is 2nd or 3rd order
    PIDGains gains = stateFeedbackToPID(K_fb, m_n);
    
    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;
    
    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "Pole placement complete";
    
    controller.setParameters(result.parameters);
    
    return result;
}

void PolePlacement::setDesiredPoles(const std::vector<std::complex<double>>& poles) {
    m_desiredPoles = poles;
}

void PolePlacement::setPolesFromSpecs(double settlingTime, double overshoot) {
    // For 2% settling: ts ≈ 4 / (ζωn)
    // Overshoot: Mp = exp(-πζ/√(1-ζ²))
    
    // Solve for damping ratio from overshoot
    double zeta;
    if (overshoot <= 0) {
        zeta = 1.0;  // Critically damped
    } else {
        // Mp = exp(-πζ/√(1-ζ²))
        // ln(Mp) = -πζ/√(1-ζ²)
        // Solve iteratively
        zeta = 0.5;
        for (int i = 0; i < 20; i++) {
            double mp_calc = std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta*zeta + 0.001));
            double error = mp_calc - overshoot;
            zeta += 0.05 * error;
            zeta = std::clamp(zeta, 0.1, 0.999);
        }
    }
    
    // Natural frequency from settling time
    double wn = 4.0 / (zeta * settlingTime);
    
    // Complex conjugate poles: p = -ζωn ± jωn√(1-ζ²)
    double realPart = -zeta * wn;
    double imagPart = wn * std::sqrt(1.0 - zeta*zeta);
    
    m_desiredPoles.clear();
    m_desiredPoles.push_back({realPart, imagPart});
    m_desiredPoles.push_back({realPart, -imagPart});
}

std::vector<double> PolePlacement::computeGain(const double* A, const double* B,
                                                int n, const std::vector<std::complex<double>>& poles) {
    if (n <= 0 || poles.size() < static_cast<size_t>(n)) {
        return {};
    }
    
    // Ackermann's formula for SISO systems
    // K = [0 0 ... 0 1] * [B AB A²B ... A^(n-1)B]^(-1) * α(A)
    // where α(s) = (s-p1)(s-p2)...(s-pn) is desired characteristic polynomial
    
    // For simplicity, implement for n = 2
    if (n != 2) {
        return {};
    }
    
    // Build controllability matrix Wc = [B AB]
    // For n=2: Wc = [b1 a11*b1+a12*b2; b2 a21*b1+a22*b2]
    double b1 = B[0], b2 = B[1];
    double a11 = A[0], a12 = A[1], a21 = A[2], a22 = A[3];
    
    double Wc[4];
    Wc[0] = b1;
    Wc[1] = a11*b1 + a12*b2;
    Wc[2] = b2;
    Wc[3] = a21*b1 + a22*b2;
    
    // Check controllability
    double det = Wc[0]*Wc[3] - Wc[1]*Wc[2];
    if (std::abs(det) < 1e-10) {
        return {};  // Not controllable
    }
    
    // Invert Wc
    double WcInv[4];
    WcInv[0] = Wc[3] / det;
    WcInv[1] = -Wc[1] / det;
    WcInv[2] = -Wc[2] / det;
    WcInv[3] = Wc[0] / det;
    
    // Characteristic polynomial coefficients from desired poles
    // α(s) = s² + α1*s + α0
    // For poles p1, p2: α(s) = (s-p1)(s-p2) = s² - (p1+p2)s + p1*p2
    std::complex<double> sum = poles[0] + poles[1];
    std::complex<double> prod = poles[0] * poles[1];
    double alpha1 = -sum.real();
    double alpha0 = prod.real();
    
    // Compute α(A) = A² + α1*A + α0*I
    double A2[4];
    A2[0] = a11*a11 + a12*a21;
    A2[1] = a11*a12 + a12*a22;
    A2[2] = a21*a11 + a22*a21;
    A2[3] = a21*a12 + a22*a22;
    
    double alphaA[4];
    alphaA[0] = A2[0] + alpha1*a11 + alpha0;
    alphaA[1] = A2[1] + alpha1*a12;
    alphaA[2] = A2[2] + alpha1*a21;
    alphaA[3] = A2[3] + alpha1*a22 + alpha0;
    
    // K = [0 1] * Wc^-1 * α(A)
    // First: temp = [0 1] * Wc^-1 = [WcInv[2] WcInv[3]]
    // Then: K = temp * α(A)
    double k1 = WcInv[2]*alphaA[0] + WcInv[3]*alphaA[2];
    double k2 = WcInv[2]*alphaA[1] + WcInv[3]*alphaA[3];
    
    return {k1, k2};
}

PIDGains PolePlacement::stateFeedbackToPID(const std::vector<double>& K, int order) {
    PIDGains gains;
    
    if (order == 2 && K.size() >= 2) {
        // For 2nd order: u = -K1*x1 - K2*x2
        // Approximate PID: Kp ≈ K1, Kd ≈ K2
        gains.Kp = K[0];
        gains.Ti = 0.0;  // No integral from state feedback alone
        gains.Td = K[1] / (K[0] + 0.01);
        
        // Add integral action for steady-state tracking
        gains.Ti = 5.0 / K[0];
    }
    
    return gains;
}

void PolePlacement::setSystemMatrices(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    m_A.assign(A, A + n*n);
    m_B.assign(B, B + n*m);
}

// ============================================================================
// LoopShaping Implementation
// ============================================================================

bool LoopShaping::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("Lead") != std::string::npos ||
           name.find("Lag") != std::string::npos;
}

TuningResult LoopShaping::tune(TunableController& controller,
                               const ProcessModel* model) {
    TuningResult result;
    
    if (!model) {
        result.success = false;
        result.message = "Process model required for loop shaping";
        return result;
    }
    
    auto fopdt = model->toFOPDT();
    
    // Design lead-lag compensator to achieve desired crossover and phase margin
    // At crossover: |L(jωc)| = 1, ∠L(jωc) = -180° + PM
    
    // Plant phase at crossover
    double plantPhase = -std::atan(m_omegaC * fopdt.tau) - m_omegaC * fopdt.L;
    plantPhase *= 180.0 / M_PI;
    
    // Required phase boost
    double phaseBoost = m_phaseMargin - (180.0 + plantPhase);
    
    // Lead compensator design: C(s) = Kc * (s/ω_z + 1) / (s/ω_p + 1)
    // Phase boost: φ = arctan(ω/ω_z) - arctan(ω/ω_p)
    // Maximum boost at ω = sqrt(ω_z * ω_p)
    
    double maxBoostRad = phaseBoost * M_PI / 180.0;
    if (maxBoostRad > 0) {
        // For lead: sin(φmax) = (α-1)/(α+1) where α = ωp/ωz
        double sinPhi = std::sin(maxBoostRad);
        double alpha = (1.0 + sinPhi) / (1.0 - sinPhi + 0.01);
        
        // Place maximum boost at crossover
        double omegaZ = m_omegaC / std::sqrt(alpha);
        double omegaP = m_omegaC * std::sqrt(alpha);
        
        // Calculate gain at crossover to make |L(jωc)| = 1
        double plantMag = fopdt.K / std::sqrt(1.0 + std::pow(m_omegaC * fopdt.tau, 2));
        double compMag = std::sqrt((1.0 + std::pow(m_omegaC/omegaZ, 2)) / 
                                   (1.0 + std::pow(m_omegaC/omegaP, 2)));
        double Kc = 1.0 / (plantMag * compMag);
        
        // Convert to PID approximation
        // Lead: (s/ωz + 1)/(s/ωp + 1) ≈ Kp(1 + Td*s) for high gain
        double Kp = Kc;
        double Td = 1.0 / omegaZ - 1.0 / omegaP;
        double Ti = 10.0 / m_omegaC;  // Add integral for low-freq gain
        
        double Ki = Kp / Ti;
        double Kd_val = Kp * Td;
        
        result.parameters = {Kp, Ki, Kd_val};
        result.success = true;
        result.message = "Loop shaping complete";
        result.phaseMargin = m_phaseMargin;
        
        controller.setParameters(result.parameters);
    } else {
        // Lag compensator for gain adjustment
        double Kp = 1.0 / fopdt.K;
        double Ti = 10.0 / m_omegaC;
        
        result.parameters = {Kp, Kp/Ti, 0.0};
        result.success = true;
        result.message = "Loop shaping (lag only)";
        
        controller.setParameters(result.parameters);
    }
    
    return result;
}

TransferFunction LoopShaping::designLeadLag(const ProcessModel& plant,
                                            double omegaC, double phaseMargin) {
    TransferFunction C;
    
    auto fopdt = plant.toFOPDT();
    
    // Calculate required phase boost
    double plantPhase = -std::atan(omegaC * fopdt.tau) - omegaC * fopdt.L;
    plantPhase *= 180.0 / M_PI;
    double phaseBoost = phaseMargin - (180.0 + plantPhase);
    
    if (phaseBoost > 0) {
        double maxBoostRad = phaseBoost * M_PI / 180.0;
        double sinPhi = std::sin(maxBoostRad);
        double alpha = (1.0 + sinPhi) / (1.0 - sinPhi + 0.01);
        
        double omegaZ = omegaC / std::sqrt(alpha);
        double omegaP = omegaC * std::sqrt(alpha);
        
        double plantMag = fopdt.K / std::sqrt(1.0 + std::pow(omegaC * fopdt.tau, 2));
        double compMag = std::sqrt((1.0 + std::pow(omegaC/omegaZ, 2)) / 
                                   (1.0 + std::pow(omegaC/omegaP, 2)));
        double Kc = 1.0 / (plantMag * compMag);
        
        C.num = {Kc / omegaZ, Kc};
        C.den = {1.0 / omegaP, 1.0};
    } else {
        C.num = {1.0};
        C.den = {1.0};
    }
    
    return C;
}

// ============================================================================
// DirectSynthesis Implementation
// ============================================================================

bool DirectSynthesis::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos;
}

TuningResult DirectSynthesis::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;
    
    if (!model) {
        result.success = false;
        result.message = "Process model required for direct synthesis";
        return result;
    }
    
    auto fopdt = model->toFOPDT();
    
    // Design using direct synthesis
    PIDGains gains = designForFOPDT(fopdt, m_tauCL);
    
    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;
    
    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "Direct synthesis complete";
    result.settlingTime = 4.0 * m_tauCL;
    
    controller.setParameters(result.parameters);
    
    return result;
}

PIDGains DirectSynthesis::designForFOPDT(const FOPDTModel& model, double tauCL) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    // Desired closed-loop: T(s) = exp(-Ls) / (τc*s + 1)
    // Controller: C = T / (G * (1-T))
    // For FOPDT: C = (τs + 1) / (K * τc * s)
    
    // This gives PI controller:
    // Kp = τ / (K * τc)
    // Ti = τ
    
    // With dead-time compensation (first-order Padé):
    gains.Kp = tau / (K * (tauCL + L/2));
    gains.Ti = tau;
    gains.Td = L / 2;
    
    return gains;
}

PIDGains DirectSynthesis::designForSOPDT(const SOPDTModel& model, 
                                          double tauCL, double zetaCL) {
    PIDGains gains;
    
    double K = model.K;
    double tau1 = model.tau1;
    double tau2 = model.tau2;
    double L = model.L;
    
    // For SOPDT with desired second-order response:
    // T(s) = exp(-Ls) / (τc²s² + 2ζτc*s + 1)
    
    double tauSum = tau1 + tau2;
    double tauProd = tau1 * tau2;
    
    gains.Kp = tauSum / (K * 2 * zetaCL * tauCL);
    gains.Ti = tauSum;
    gains.Td = tauProd / tauSum;
    
    return gains;
}

// ============================================================================
// SmithPredictor Implementation
// ============================================================================

bool SmithPredictor::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("Smith") != std::string::npos;
}

TuningResult SmithPredictor::tune(TunableController& controller,
                                  const ProcessModel* model) {
    TuningResult result;
    
    FOPDTModel fopdt;
    if (model) {
        fopdt = model->toFOPDT();
    } else {
        fopdt = m_model;
    }
    
    if (fopdt.K == 0 || fopdt.tau <= 0) {
        result.success = false;
        result.message = "Invalid model parameters";
        return result;
    }
    
    // Design Smith predictor
    auto sp = design(fopdt, m_lambda);
    
    // Return inner controller gains
    double Ki = (sp.innerController.Ti > 0) ? 
                sp.innerController.Kp / sp.innerController.Ti : 0.0;
    double Kd = sp.innerController.Kp * sp.innerController.Td;
    
    result.parameters = {sp.innerController.Kp, Ki, Kd};
    result.success = true;
    result.message = "Smith predictor design complete";
    result.settlingTime = 4.0 * m_lambda;
    
    controller.setParameters(result.parameters);
    
    return result;
}

SmithPredictor::SmithPredictorStructure SmithPredictor::design(
    const FOPDTModel& model, double lambda) {
    
    SmithPredictorStructure sp;
    sp.processModel = model;
    sp.delay = model.L;
    
    // Inner controller designed for delay-free model
    // G_df(s) = K / (τs + 1)
    // Use IMC tuning for inner controller
    sp.innerController.Kp = model.tau / (model.K * lambda);
    sp.innerController.Ti = model.tau;
    sp.innerController.Td = 0.0;  // PI for Smith predictor
    
    return sp;
}

// ============================================================================
// DahlinAlgorithm Implementation
// ============================================================================

bool DahlinAlgorithm::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("PID") != std::string::npos ||
           name.find("Discrete") != std::string::npos;
}

TuningResult DahlinAlgorithm::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;
    
    if (!model) {
        result.success = false;
        result.message = "Process model required for Dahlin's algorithm";
        return result;
    }
    
    auto fopdt = model->toFOPDT();
    
    // Design digital controller
    auto coeffs = designDigital(fopdt, m_Ts, m_lambda);
    
    // Convert to PID-like parameters (approximate)
    // Digital PID: C(z) = Kp + Ki*Ts/(1-z^-1) + Kd*(1-z^-1)/Ts
    double b0 = coeffs[0];
    double b1 = coeffs[1];
    double a1 = coeffs[2];
    
    // Approximate continuous PID
    double Kp = b0;
    double Ki = (b0 + b1) / m_Ts;
    double Kd = 0.0;
    
    result.parameters = {Kp, Ki, Kd};
    result.success = true;
    result.message = "Dahlin controller design complete";
    result.settlingTime = 4.0 * m_lambda;
    
    controller.setParameters(result.parameters);
    
    return result;
}

std::array<double, 3> DahlinAlgorithm::designDigital(const FOPDTModel& model,
                                                      double Ts, double lambda) {
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    // Discrete plant (ZOH): G(z) = K(1-a)z^(-N-1) / (1 - az^-1)
    // where a = exp(-Ts/τ), N = floor(L/Ts)
    double a = std::exp(-Ts / tau);
    int N = static_cast<int>(L / Ts);
    
    // Desired closed-loop: T(z) = (1-α)z^(-N-1) / (1 - αz^-1)
    // where α = exp(-Ts/λ)
    double alpha = std::exp(-Ts / lambda);
    
    // Controller: C(z) = T / (G * (1-T))
    // C(z) = (1-α)(1-az^-1) / (K(1-a)(1-αz^-1) - (1-α)(1-az^-1))
    
    // Simplify to standard form: C(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
    double num0 = (1 - alpha) * 1.0;
    double num1 = (1 - alpha) * (-a);
    double den0 = K * (1 - a) * 1.0 - (1 - alpha) * 1.0;
    double den1 = K * (1 - a) * (-alpha) - (1 - alpha) * (-a);
    
    double b0 = num0 / (den0 + 0.001);
    double b1 = num1 / (den0 + 0.001);
    double a1 = den1 / (den0 + 0.001);
    
    return {b0, b1, a1};
}

// ============================================================================
// DeadbeatControl Implementation
// ============================================================================

bool DeadbeatControl::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("Discrete") != std::string::npos ||
           name.find("State") != std::string::npos;
}

TuningResult DeadbeatControl::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;
    
    if (!model) {
        result.success = false;
        result.message = "Process model required for deadbeat control";
        return result;
    }
    
    auto fopdt = model->toFOPDT();
    
    // For FOPDT, convert to discrete state-space
    // x(k+1) = Ad*x(k) + Bd*u(k)
    // y(k) = Cd*x(k)
    
    double a = std::exp(-m_Ts / fopdt.tau);
    double b = fopdt.K * (1 - a);
    
    // Deadbeat: place all poles at z = 0
    // For first-order: K = (1 - desired_pole) / b = 1/b for deadbeat
    double K_db = (1.0 - 0.0) / (b + 0.001);  // All poles at 0
    
    // Add settling samples for robustness
    if (m_settlingN > 0) {
        double desiredPole = std::pow(0.1, 1.0 / m_settlingN);
        K_db = (a - desiredPole) / (b + 0.001);
    }
    
    // Convert to PID approximation
    double Kp = K_db;
    double Ki = K_db / fopdt.tau;
    
    result.parameters = {Kp, Ki, 0.0};
    result.success = true;
    result.message = "Deadbeat control design complete";
    result.settlingTime = m_Ts * (m_settlingN > 0 ? m_settlingN : 1);
    
    controller.setParameters(result.parameters);
    
    return result;
}

std::vector<double> DeadbeatControl::design(const double* A, const double* B,
                                            const double* C, int n, int m, int p,
                                            double Ts, int settlingN) {
    // For general state-space, use Ackermann's formula
    // with all poles at z = 0 (or slightly inside unit circle for robustness)
    
    std::vector<std::complex<double>> desiredPoles(n);
    double poleLocation = (settlingN > 0) ? std::pow(0.1, 1.0 / settlingN) : 0.0;
    
    for (int i = 0; i < n; i++) {
        desiredPoles[i] = {poleLocation, 0.0};
    }
    
    return PolePlacement::computeGain(A, B, n, desiredPoles);
}

// ============================================================================
// MinimumVarianceControl Implementation
// ============================================================================

bool MinimumVarianceControl::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("MVC") != std::string::npos ||
           name.find("Stochastic") != std::string::npos;
}

TuningResult MinimumVarianceControl::tune(TunableController& controller,
                                          const ProcessModel* model) {
    TuningResult result;
    
    if (m_A.empty() || m_B.empty() || m_C.empty()) {
        result.success = false;
        result.message = "ARMAX model required. Call setARMAXModel() first.";
        return result;
    }
    
    // MVC design
    // For ARMAX: A(q)y(t) = B(q)u(t-k) + C(q)e(t)
    // MVC solves: A = C*R + q^(-k)*S (Diophantine equation)
    // Controller: u = -(S/BR)*y + (T/BR)*r
    
    // Simplified for first-order system
    // A(q) = 1 + a1*q^-1
    // B(q) = b0
    // C(q) = 1 + c1*q^-1
    
    double a1 = m_A.size() > 1 ? m_A[1] : 0.0;
    double b0 = m_B.size() > 0 ? m_B[0] : 1.0;
    double c1 = m_C.size() > 1 ? m_C[1] : 0.0;
    
    // For k=1 (one-step delay):
    // R = 1, S = a1 - c1
    double s0 = a1 - c1;
    
    // Generalized MVC with control weight λ
    // J = E[(y - r)² + λu²]
    // Modified controller gain:
    double K_mvc = s0 / (b0 + m_lambda * std::abs(b0));
    
    // Convert to PID approximation
    double Kp = K_mvc;
    double Ki = K_mvc / 10.0;  // Approximate integral
    
    result.parameters = {Kp, Ki, 0.0};
    result.success = true;
    result.message = "Minimum variance control design complete";
    
    controller.setParameters(result.parameters);
    
    return result;
}

void MinimumVarianceControl::setARMAXModel(const std::vector<double>& A,
                                            const std::vector<double>& B,
                                            const std::vector<double>& C,
                                            int k) {
    m_A = A;
    m_B = B;
    m_C = C;
    m_k = k;
}

} // namespace Autotuning
} // namespace tether::control
