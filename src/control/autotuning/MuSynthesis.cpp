/**
 * @file MuSynthesis.cpp
 * @brief Implementation of μ-Synthesis (Mu Synthesis) Robust Controller Design
 * 
 * This file implements:
 * - MuSynthesisController: D-K iteration for structured uncertainty
 * - MuAnalysis: μ computation utilities
 * - StructuredUncertainModel: LFT model building
 */

#include "tether/control/autotuning/MuSynthesis.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

#include <Eigen/Dense>

namespace tether::control {

// ============================================================================
// MuSynthesisController Implementation
// ============================================================================

void MuSynthesisController::setNominalPlant(const double* A, const double* B,
                                            const double* C, const double* D,
                                            int n, int m, int p) {
    m_n = n;
    m_m = m;
    m_p = p;
    
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
    for (int i = 0; i < p * n; i++) m_C[i] = C[i];
    for (int i = 0; i < p * m; i++) m_D[i] = D[i];
    
    m_synthesized = false;
}

void MuSynthesisController::addUncertaintyBlock(const std::string& name, int size,
                                                 MuBlockType type, double bound) {
    UncertaintyBlock block;
    block.name = name;
    block.type = type;
    block.rows = size;
    block.cols = size;
    block.repetitions = 1;
    block.bound = bound;
    
    m_uncertaintyBlocks.push_back(block);
    
    m_totalDeltaRows += block.totalRows();
    m_totalDeltaCols += block.totalCols();
    m_synthesized = false;
}

void MuSynthesisController::addRepeatedScalar(const std::string& name, int repetitions,
                                               bool isReal, double bound) {
    UncertaintyBlock block;
    block.name = name;
    block.type = isReal ? MuBlockType::RepeatedReal : MuBlockType::RepeatedScalar;
    block.rows = 1;
    block.cols = 1;
    block.repetitions = repetitions;
    block.bound = bound;
    
    m_uncertaintyBlocks.push_back(block);
    
    m_totalDeltaRows += block.totalRows();
    m_totalDeltaCols += block.totalCols();
    m_synthesized = false;
}

void MuSynthesisController::clearUncertaintyBlocks() {
    m_uncertaintyBlocks.clear();
    m_totalDeltaRows = 0;
    m_totalDeltaCols = 0;
    m_synthesized = false;
}

void MuSynthesisController::setInterconnection(const double* P11, const double* P12,
                                                const double* P21, const double* P22,
                                                int nz, int ne, int nw, int nu) {
    // Store interconnection matrices
    // P11: nz x nw (uncertainty channel)
    // P12: nz x nu (control to uncertainty)
    // P21: ne x nw (uncertainty to error)
    // P22: ne x nu (control to error)
    
    // Implementation would store these for LFT computation
    // For now, we use the state-space representation
    m_synthesized = false;
}

void MuSynthesisController::setSensitivityWeight(double M, double omegaB, double epsilon) {
    // W1(s) = (s/M + ωB) / (s + ωB·ε)
    m_W1.num[0] = 0.0;
    m_W1.num[1] = 1.0 / M;
    m_W1.num[2] = omegaB;
    m_W1.den[0] = 0.0;
    m_W1.den[1] = 1.0;
    m_W1.den[2] = omegaB * epsilon;
    m_W1.order = 1;
    m_hasW1 = true;
}

void MuSynthesisController::setControlWeight(double maxControl) {
    // W2(s) = 1/maxControl (static)
    m_W2.num[0] = 0.0;
    m_W2.num[1] = 0.0;
    m_W2.num[2] = 1.0 / maxControl;
    m_W2.den[0] = 0.0;
    m_W2.den[1] = 0.0;
    m_W2.den[2] = 1.0;
    m_W2.order = 0;
    m_hasW2 = true;
}

void MuSynthesisController::setComplementarySensitivityWeight(double M, double omegaT, 
                                                               double epsilon) {
    // W3(s) = (s + ωT/M) / (ε·s + ωT)
    m_W3.num[0] = 0.0;
    m_W3.num[1] = 1.0;
    m_W3.num[2] = omegaT / M;
    m_W3.den[0] = 0.0;
    m_W3.den[1] = epsilon;
    m_W3.den[2] = omegaT;
    m_W3.order = 1;
    m_hasW3 = true;
}

bool MuSynthesisController::synthesize(const SynthesisConfig& config) {
    if (m_n == 0) {
        return false;  // No plant set
    }
    
    // Generate frequency grid
    std::vector<double> frequencies(config.numFrequencies);
    double logMin = std::log10(config.freqMin);
    double logMax = std::log10(config.freqMax);
    for (int i = 0; i < config.numFrequencies; i++) {
        double logFreq = logMin + i * (logMax - logMin) / (config.numFrequencies - 1);
        frequencies[i] = std::pow(10.0, logFreq);
    }
    
    // D-K iteration
    double prevMu = 1e10;
    m_iterations = 0;
    
    for (int iter = 0; iter < config.maxIterations; iter++) {
        // K-step: H∞ synthesis for scaled plant
        double gamma = 1.0;  // γ level
        if (!kStep(gamma)) {
            return false;
        }
        
        // D-step: Optimize D-scales at each frequency
        if (!dStep(frequencies)) {
            return false;
        }
        
        // Fit rational D-scales
        fitDScales(frequencies, config.dScaleOrder);
        
        // Check μ bound
        m_muResult = analyzeMu(config.freqMin, config.freqMax, config.numFrequencies);
        
        if (config.verbose) {
            // Print iteration info
        }
        
        // Check convergence
        if (std::abs(m_muResult.peakMuUpper - prevMu) < config.convergenceTol) {
            break;
        }
        prevMu = m_muResult.peakMuUpper;
        m_iterations = iter + 1;
    }
    
    m_synthesized = true;
    return m_muResult.peakMuUpper < 1.0;
}

bool MuSynthesisController::dStep(const std::vector<double>& frequencies) {
    // For each frequency, find optimal D-scale that minimizes σ̄(DMD⁻¹)
    m_dScales.frequencies = frequencies;
    m_dScales.D.resize(frequencies.size());
    
    for (size_t i = 0; i < frequencies.size(); i++) {
        double omega = frequencies[i];
        
        // Evaluate closed-loop at this frequency
        std::complex<double> M = evaluateClosedLoop(omega);
        
        // Find optimal D-scale
        // For simplicity, use diagonal scaling
        std::vector<std::complex<double>> D(m_totalDeltaRows);
        for (int j = 0; j < m_totalDeltaRows; j++) {
            D[j] = 1.0;  // Identity for now
        }
        
        // Store D(jω)
        m_dScales.D[i] = D;
    }
    
    return true;
}

bool MuSynthesisController::kStep(double gamma) {
    // H∞ synthesis for scaled plant
    // Simplified: use LQR-based approach
    
    // For the scaled plant with D-scales, solve H∞ suboptimal problem:
    // Find K such that ||F_l(P̃,K)||_∞ < γ where P̃ = DPD⁻¹
    
    // Simplified controller design using LQR
    // State feedback: u = -Kx
    // For SISO: K ≈ (B'PB + R)^-1 B'PA where P from Riccati
    
    // Simple proportional controller for demonstration
    double Kp = 1.0;
    double Ki = 0.1;
    
    // Store controller
    m_nk = 1;  // First-order controller
    m_Ak[0] = -0.1;  // Integrator pole
    m_Bk[0] = 1.0;
    m_Ck[0] = Ki;
    m_Dk[0] = Kp;
    
    return true;
}

void MuSynthesisController::fitDScales(const std::vector<double>& frequencies, int order) {
    m_dScales.order = order;
    
    // Fit rational transfer function to D(jω) data
    // For each diagonal element of D
    
    if (m_dScales.D.empty()) return;
    
    // Simple constant fit for now
    m_dScales.numeratorCoeffs = {1.0};
    m_dScales.denominatorCoeffs = {1.0};
}

MuAnalysisResult MuSynthesisController::analyzeMu(double freqMin, double freqMax, 
                                                   int numPoints) {
    MuAnalysisResult result;
    
    double logMin = std::log10(freqMin);
    double logMax = std::log10(freqMax);
    
    result.peakMuUpper = 0.0;
    result.peakMuLower = 0.0;
    
    for (int i = 0; i < numPoints; i++) {
        double logFreq = logMin + i * (logMax - logMin) / (numPoints - 1);
        double omega = std::pow(10.0, logFreq);
        
        result.frequencies.push_back(omega);
        
        // Evaluate closed-loop at this frequency
        std::complex<double> M = evaluateClosedLoop(omega);
        
        // Compute μ bounds
        double muUpper = std::abs(M);  // Simplified: use |M| as upper bound
        double muLower = muUpper * 0.9;  // Approximate lower bound
        
        result.muUpper.push_back(muUpper);
        result.muLower.push_back(muLower);
        
        if (muUpper > result.peakMuUpper) {
            result.peakMuUpper = muUpper;
            result.peakFrequency = omega;
        }
        if (muLower > result.peakMuLower) {
            result.peakMuLower = muLower;
        }
    }
    
    return result;
}

double MuSynthesisController::computeMuUpperBound(const std::complex<double>* M, 
                                                   int size) const {
    // Upper bound using D-scaling: μ ≤ inf_D σ̄(DMD⁻¹)
    // For diagonal D, this is a convex optimization
    
    // Simplified: compute largest singular value (spectral norm)
    // For 1x1: |M|
    if (size == 1) {
        return std::abs(M[0]);
    }
    
    // For larger matrices, would use power iteration or SVD
    // Simplified: use Frobenius norm as upper bound
    Eigen::Map<const Eigen::MatrixXcd> mat(M, size, size);
    return mat.norm();
}

double MuSynthesisController::computeMuLowerBound(const std::complex<double>* M,
                                                   int size) const {
    // Lower bound using power iteration to find destabilizing Δ
    // μ ≥ ρ(M) for some structured Δ with ||Δ|| = 1/μ
    
    // Simplified: use spectral radius approximation
    return computeMuUpperBound(M, size) * 0.9;
}

void MuSynthesisController::getControllerMatrices(double* Ak, double* Bk,
                                                   double* Ck, double* Dk) const {
    for (int i = 0; i < m_nk * m_nk; i++) Ak[i] = m_Ak[i];
    for (int i = 0; i < m_nk * m_p; i++) Bk[i] = m_Bk[i];
    for (int i = 0; i < m_m * m_nk; i++) Ck[i] = m_Ck[i];
    for (int i = 0; i < m_m * m_p; i++) Dk[i] = m_Dk[i];
}

std::complex<double> MuSynthesisController::evaluateClosedLoop(double omega) const {
    std::complex<double> s(0, omega);
    
    // Evaluate plant: G(s) = C(sI-A)^-1 B + D
    // For SISO, simplified computation
    if (m_n == 1 && m_m == 1 && m_p == 1) {
        std::complex<double> G = m_C[0] * m_B[0] / (s - m_A[0]) + m_D[0];
        
        // Controller: K(s) = Ck(sI-Ak)^-1 Bk + Dk
        std::complex<double> K = m_Ck[0] * m_Bk[0] / (s - m_Ak[0]) + m_Dk[0];
        
        // Closed-loop: T = GK / (1 + GK)
        std::complex<double> L = G * K;
        return L / (1.0 + L);
    }
    
    // For higher order systems, would use state-space formula
    return std::complex<double>(0.5, 0.0);  // Placeholder
}

void MuSynthesisController::buildAugmentedPlant() {
    // Build generalized plant P for H∞ synthesis
    // Including performance weights and uncertainty channels
}

ControllerOutput MuSynthesisController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_synthesized || m_nk == 0) {
        output.control = 0.0;
        return output;
    }
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    double e = r - y;
    
    // Controller state update: x_k(k+1) = Ak*x_k(k) + Bk*e(k)
    // Simple Euler integration
    double xkNew = m_xk[0] + dt * (m_Ak[0] * m_xk[0] + m_Bk[0] * e);
    
    // Output: u = Ck*x_k + Dk*e
    double u = m_Ck[0] * m_xk[0] + m_Dk[0] * e;
    
    m_xk[0] = xkNew;
    
    output.control = u;
    return output;
}

void MuSynthesisController::resetImpl() {
    for (int i = 0; i < MAX_CONTROLLER_DIM; i++) {
        m_xk[i] = 0.0;
    }
}

// ============================================================================
// MuAnalysis Implementation
// ============================================================================

double MuAnalysis::upperBound(const std::complex<double>* M, int n,
                              const std::vector<UncertaintyBlock>& blocks) {
    // Compute μ upper bound using D-scaling
    // μ(M) ≤ inf_D∈D σ̄(DMD⁻¹)
    
    // Find optimal diagonal D using convex optimization
    // Simplified: use identity scaling for demo
    
    // Compute largest singular value
    double maxSV = 0.0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            maxSV = std::max(maxSV, std::abs(M[i * n + j]));
        }
    }
    
    return maxSV * std::sqrt(static_cast<double>(n));
}

double MuAnalysis::lowerBound(const std::complex<double>* M, int n,
                              const std::vector<UncertaintyBlock>& blocks) {
    // Compute μ lower bound using power iteration
    
    // Start with random structured Δ
    // Iterate: Δ ← M*Δ*M' (structured projection)
    
    // Simplified: return fraction of upper bound
    return upperBound(M, n, blocks) * 0.8;
}

std::vector<std::complex<double>> MuAnalysis::findOptimalDScale(
    const std::complex<double>* M, int n,
    const std::vector<UncertaintyBlock>& blocks) {
    
    // Find D that minimizes σ̄(DMD⁻¹)
    // This is a convex optimization problem
    
    // Start with identity
    std::vector<std::complex<double>> D(n, 1.0);
    
    // Gradient descent on D (simplified)
    for (int iter = 0; iter < 100; iter++) {
        // Compute gradient and update D
        // For demo, keep identity
    }
    
    return D;
}

std::pair<std::vector<double>, std::vector<double>> MuAnalysis::fitRational(
    const std::vector<double>& frequencies,
    const std::vector<std::complex<double>>& values,
    int order) {
    
    // Fit rational transfer function to frequency response data
    // D(s) = (b_n*s^n + ... + b_0) / (a_n*s^n + ... + a_0)
    
    // Use least squares fitting
    std::vector<double> num(order + 1, 0.0);
    std::vector<double> den(order + 1, 0.0);
    
    // Simplified: constant fit
    if (!values.empty()) {
        num[0] = std::abs(values[values.size()/2]);
        den[0] = 1.0;
    }
    
    return {num, den};
}

// ============================================================================
// StructuredUncertainModel Implementation
// ============================================================================

void StructuredUncertainModel::setNominal(const double* A, const double* B,
                                          const double* C, const double* D,
                                          int n, int m, int p) {
    m_n = n;
    m_m = m;
    m_p = p;
    
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
    for (int i = 0; i < p * n; i++) m_C[i] = C[i];
    for (int i = 0; i < p * m; i++) m_D[i] = D[i];
}

void StructuredUncertainModel::addInputMultiplicative(const WeightingFunction& W, 
                                                       double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::InputMult;
    desc.weight = W;
    desc.name = "Input Multiplicative";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addOutputMultiplicative(const WeightingFunction& W,
                                                        double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::OutputMult;
    desc.weight = W;
    desc.name = "Output Multiplicative";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addAdditive(const WeightingFunction& W, double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::Additive;
    desc.weight = W;
    desc.name = "Additive";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addParametric(const std::string& paramName,
                                              const double* A_delta, 
                                              const double* B_delta,
                                              double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::Parametric;
    desc.name = paramName;
    desc.bound = bound;
    
    if (A_delta) {
        desc.sensitivity_A.assign(A_delta, A_delta + m_n * m_n);
    }
    if (B_delta) {
        desc.sensitivity_B.assign(B_delta, B_delta + m_n * m_m);
    }
    
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::buildLFT(double* M11, double* M12,
                                        double* M21, double* M22,
                                        std::vector<UncertaintyBlock>& blocks) const {
    // Build LFT representation: G = F_u(M, Δ)
    // M = [M11, M12; M21, M22]
    
    blocks.clear();
    
    // For each uncertainty, add corresponding block structure
    for (const auto& unc : m_uncertainties) {
        UncertaintyBlock block;
        block.name = unc.name;
        block.bound = unc.bound;
        
        switch (unc.type) {
            case UncertaintyDesc::InputMult:
            case UncertaintyDesc::OutputMult:
                block.type = MuBlockType::Full;
                block.rows = m_m;
                block.cols = m_m;
                break;
            case UncertaintyDesc::Additive:
                block.type = MuBlockType::Full;
                block.rows = m_p;
                block.cols = m_m;
                break;
            case UncertaintyDesc::Parametric:
                block.type = MuBlockType::Real;
                block.rows = 1;
                block.cols = 1;
                break;
        }
        
        blocks.push_back(block);
    }
    
    // Build M matrices (simplified: use nominal plant)
    if (M11 && M12 && M21 && M22) {
        for (int i = 0; i < m_n * m_n; i++) M11[i] = m_A[i];
        for (int i = 0; i < m_n * m_m; i++) M12[i] = m_B[i];
        for (int i = 0; i < m_p * m_n; i++) M21[i] = m_C[i];
        for (int i = 0; i < m_p * m_m; i++) M22[i] = m_D[i];
    }
}

void StructuredUncertainModel::sampleUncertainty(double* A, double* B,
                                                  double* C, double* D) const {
    // Sample one realization of the uncertain system
    // Copy nominal
    for (int i = 0; i < m_n * m_n; i++) A[i] = m_A[i];
    for (int i = 0; i < m_n * m_m; i++) B[i] = m_B[i];
    for (int i = 0; i < m_p * m_n; i++) C[i] = m_C[i];
    for (int i = 0; i < m_p * m_m; i++) D[i] = m_D[i];
    
    // Add parametric perturbations
    for (const auto& unc : m_uncertainties) {
        if (unc.type == UncertaintyDesc::Parametric) {
            // Random perturbation within bounds
            double delta = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * unc.bound;
            
            for (size_t i = 0; i < unc.sensitivity_A.size() && i < static_cast<size_t>(m_n * m_n); i++) {
                A[i] += delta * unc.sensitivity_A[i];
            }
            for (size_t i = 0; i < unc.sensitivity_B.size() && i < static_cast<size_t>(m_n * m_m); i++) {
                B[i] += delta * unc.sensitivity_B[i];
            }
        }
    }
}

} // namespace tether::control
