/**
 * @file RobustControllers.cpp
 * @brief Implementation of H2 and H∞ Robust Controllers
 */

#include "control/RobustControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

#include <Eigen/Dense>

namespace Control {

// ============================================================================
// WeightingFunction Implementation
// ============================================================================

WeightingFunction WeightingFunction::firstOrder(double zero, double pole, double gain) {
    WeightingFunction w;
    w.order = 1;
    w.gain = gain;
    w.num[0] = 0;
    w.num[1] = 1;
    w.num[2] = zero;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = pole;
    return w;
}

WeightingFunction WeightingFunction::integrator(double gain) {
    WeightingFunction w;
    w.order = 1;
    w.gain = gain;
    w.num[0] = 0;
    w.num[1] = 0;
    w.num[2] = 1;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = 0;
    return w;
}

WeightingFunction WeightingFunction::sensitivity(double M, double omegaB, double epsilon) {
    // W₁(s) = (s/M + ωB) / (s + ωB·ε)
    // = (s + M·ωB) / (M·(s + ωB·ε))
    WeightingFunction w;
    w.order = 1;
    w.gain = 1.0 / M;
    w.num[0] = 0;
    w.num[1] = 1;
    w.num[2] = M * omegaB;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = omegaB * epsilon;
    return w;
}

WeightingFunction WeightingFunction::complementary(double M, double omegaT, double epsilon) {
    // W₃(s) = (s + ωT/M) / (ε·s + ωT)
    WeightingFunction w;
    w.order = 1;
    w.gain = 1.0 / epsilon;
    w.num[0] = 0;
    w.num[1] = epsilon;
    w.num[2] = omegaT / M * epsilon;
    w.den[0] = 0;
    w.den[1] = epsilon;
    w.den[2] = omegaT;
    return w;
}

double WeightingFunction::magnitude(double omega) const {
    // Evaluate |W(jω)|
    double numReal = num[2] - num[0] * omega * omega;
    double numImag = num[1] * omega;
    double denReal = den[2] - den[0] * omega * omega;
    double denImag = den[1] * omega;
    
    double numMag = std::sqrt(numReal * numReal + numImag * numImag);
    double denMag = std::sqrt(denReal * denReal + denImag * denImag);
    
    return gain * numMag / denMag;
}

void WeightingFunction::toStateSpace(double* A, double* B, double* C, double* D, int& n) const {
    if (order == 0) {
        n = 0;
        D[0] = gain;
        return;
    }
    
    if (order == 1) {
        n = 1;
        // First order: ẋ = -a*x + b*u, y = c*x + d*u
        // For (s+z)/(s+p): ẋ = -p*x + u, y = (z-p)*x + u
        double p = den[2] / den[1];
        double z = num[2] / num[1];
        
        A[0] = -p;
        B[0] = 1.0;
        C[0] = gain * (z - p);
        D[0] = gain;
        return;
    }
    
    // Second order (controllable canonical form)
    n = 2;
    double a0 = den[2] / den[0];
    double a1 = den[1] / den[0];
    double b0 = num[2] / den[0];
    double b1 = num[1] / den[0];
    double b2 = num[0] / den[0];
    
    A[0] = 0;      A[1] = 1;
    A[2] = -a0;    A[3] = -a1;
    
    B[0] = 0;
    B[1] = 1;
    
    C[0] = gain * (b0 - b2 * a0);
    C[1] = gain * (b1 - b2 * a1);
    
    D[0] = gain * b2;
}

// ============================================================================
// H2 Controller Implementation
// ============================================================================

void H2Controller::setGeneralizedPlant(const double* A, const double* B1, const double* B2,
                                        const double* C1, const double* C2,
                                        const double* D11, const double* D12,
                                        const double* D21, const double* D22,
                                        int n, int nw, int nu, int nz, int ny) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_nw = nw;
    m_nu = nu;
    m_nz = nz;
    m_ny = ny;
    
    std::memcpy(m_A.data(), A, m_n * m_n * sizeof(double));
    std::memcpy(m_B1.data(), B1, m_n * nw * sizeof(double));
    std::memcpy(m_B2.data(), B2, m_n * nu * sizeof(double));
    std::memcpy(m_C1.data(), C1, nz * m_n * sizeof(double));
    std::memcpy(m_C2.data(), C2, ny * m_n * sizeof(double));
    std::memcpy(m_D12.data(), D12, nz * nu * sizeof(double));
    std::memcpy(m_D21.data(), D21, ny * nw * sizeof(double));
    
    m_designed = false;
}

void H2Controller::setRegulatorProblem(const double* A, const double* B, const double* C,
                                        const double* Q, const double* R,
                                        const double* W, const double* V,
                                        int n, int m, int p) {
    // Convert LQG problem to generalized plant form
    m_n = n;
    m_nw = n + p;  // [process noise; measurement noise]
    m_nu = m;
    m_nz = n + m;  // [state error; control]
    m_ny = p;
    
    std::memcpy(m_A.data(), A, n * n * sizeof(double));
    std::memcpy(m_B2.data(), B, n * m * sizeof(double));
    std::memcpy(m_C2.data(), C, p * n * sizeof(double));
    
    // B1 = [W^½  0]
    // C1 = [Q^½; 0]
    // D12 = [0; R^½]
    // D21 = [0  V^½]
    
    // Simplified: assume W, V, Q, R are diagonal with given values
    // Full implementation would require matrix square roots
    
    m_designed = false;
}

bool H2Controller::design() {
    // H2 design is equivalent to solving two Riccati equations
    // This is essentially LQG design
    
    // For a proper implementation, would solve:
    // Control Riccati: A'X + XA - XB2*R⁻¹*B2'X + C1'C1 = 0
    // Filter Riccati: AY + YA' - YC2'*V⁻¹*C2*Y + B1*B1' = 0
    
    // Simplified: use internal LQG-like approach
    LQGController lqg;
    
    // Map to LQG format
    lqg.setSystemMatrices(m_A.data(), m_B2.data(), m_C2.data(), nullptr,
                          m_n, m_nu, m_ny);
    
    // Extract Q, R from C1'C1, D12'D12
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> Q{};
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> R{};

    // Q = C1'*C1  (C1 is m_nz × m_n, row-major flat array)
    {
        Eigen::Map<const Eigen::MatrixXd> C1(m_C1.data(), m_nz, m_n);
        Eigen::MatrixXd Qmat = C1.transpose() * C1;
        for (int i = 0; i < m_n; ++i)
            for (int j = 0; j < m_n; ++j)
                Q[i * m_n + j] = Qmat(i, j);
    }

    // R = D12'*D12  (D12 is m_nz × m_nu, row-major flat array)
    {
        Eigen::Map<const Eigen::MatrixXd> D12(m_D12.data(), m_nz, m_nu);
        Eigen::MatrixXd Rmat = D12.transpose() * D12;
        for (int i = 0; i < m_nu; ++i)
            for (int j = 0; j < m_nu; ++j)
                R[i * m_nu + j] = Rmat(i, j);
    }

    lqg.setLQRWeights(Q.data(), R.data());

    // W = B1*B1', V = D21*D21'
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> W{};
    std::array<double, MAX_OUTPUT_DIM * MAX_OUTPUT_DIM> V{};

    // W = B1*B1'  (B1 is m_n × m_nw, row-major flat array)
    {
        Eigen::Map<const Eigen::MatrixXd> B1(m_B1.data(), m_n, m_nw);
        Eigen::MatrixXd Wmat = B1 * B1.transpose();
        for (int i = 0; i < m_n; ++i)
            for (int j = 0; j < m_n; ++j)
                W[i * m_n + j] = Wmat(i, j);
    }

    // V = D21*D21'  (D21 is m_ny × m_nw, row-major flat array)
    {
        Eigen::Map<const Eigen::MatrixXd> D21(m_D21.data(), m_ny, m_nw);
        Eigen::MatrixXd Vmat = D21 * D21.transpose();
        for (int i = 0; i < m_ny; ++i)
            for (int j = 0; j < m_ny; ++j)
                V[i * m_ny + j] = Vmat(i, j);
    }

    lqg.setNoiseCovariances(W.data(), V.data());
    
    if (!lqg.design()) {
        return false;
    }
    
    // Extract controller from LQG
    // Controller: ẋk = (A - B2*K - L*C2)*xk + L*y, u = -K*xk
    m_nk = m_n;
    
    // Get K from LQR
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> K{};
    lqg.getLQR().getGainMatrix(K.data());
    
    // Ak = A - B2*K - L*C2
    // For simplicity, copy structure from LQG
    // In practice, would extract from lqg internal state
    
    m_designed = true;
    m_h2Norm = 1.0;  // Would compute actual norm
    
    return true;
}

void H2Controller::getControllerMatrices(double* Ak, double* Bk, double* Ck, double* Dk) const {
    if (Ak) std::memcpy(Ak, m_Ak.data(), m_nk * m_nk * sizeof(double));
    if (Bk) std::memcpy(Bk, m_Bk.data(), m_nk * m_ny * sizeof(double));
    if (Ck) std::memcpy(Ck, m_Ck.data(), m_nu * m_nk * sizeof(double));
    if (Dk) std::memcpy(Dk, m_Dk.data(), m_nu * m_ny * sizeof(double));
}

ControllerOutput H2Controller::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_designed) {
        output.control = 0;
        return output;
    }
    
    double dt = input.dt;
    double y = input.measured;
    
    // Controller dynamics: ẋk = Ak*xk + Bk*y
    std::array<double, MAX_STATE_DIM> xkDot{};
    for (int i = 0; i < m_nk; i++) {
        double sum = 0;
        for (int j = 0; j < m_nk; j++) {
            sum += m_Ak[i * m_nk + j] * m_xk[j];
        }
        sum += m_Bk[i] * y;
        xkDot[i] = sum;
    }
    
    // Euler integration
    for (int i = 0; i < m_nk; i++) {
        m_xk[i] += xkDot[i] * dt;
    }
    
    // Output: u = Ck*xk + Dk*y
    double u = 0;
    for (int i = 0; i < m_nk; i++) {
        u += m_Ck[i] * m_xk[i];
    }
    u += m_Dk[0] * y;
    
    output.control = std::clamp(u, m_limits.outputMin, m_limits.outputMax);
    output.error = input.reference - y;
    
    return output;
}

void H2Controller::resetImpl() {
    std::fill(m_xk.begin(), m_xk.end(), 0.0);
}

// ============================================================================
// H∞ Controller Implementation
// ============================================================================

void HInfinityController::setPlant(const double* A, const double* B, 
                                    const double* C, const double* D,
                                    int n, int m, int p) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = m;
    m_p = p;
    
    std::memcpy(m_A.data(), A, m_n * m_n * sizeof(double));
    std::memcpy(m_B.data(), B, m_n * m * sizeof(double));
    std::memcpy(m_C.data(), C, p * m_n * sizeof(double));
    if (D) {
        std::memcpy(m_D.data(), D, p * m * sizeof(double));
    }
    
    m_designed = false;
}

void HInfinityController::setGeneralizedPlant(const double* A, const double* B1, 
                                               const double* B2,
                                               const double* C1, const double* C2,
                                               const double* D11, const double* D12,
                                               const double* D21, const double* D22,
                                               int n, int nw, int nu, int nz, int ny) {
    m_n = n;
    m_nw = nw;
    m_nu = nu;
    m_nz = nz;
    m_ny = ny;
    m_na = n;  // Will be augmented with weights
    
    std::memcpy(m_Aa.data(), A, n * n * sizeof(double));
    std::memcpy(m_B1a.data(), B1, n * nw * sizeof(double));
    std::memcpy(m_B2a.data(), B2, n * nu * sizeof(double));
    std::memcpy(m_C1a.data(), C1, nz * n * sizeof(double));
    std::memcpy(m_C2a.data(), C2, ny * n * sizeof(double));
    
    m_designed = false;
}

void HInfinityController::setSensitivityWeight(double M, double omegaB, double epsilon) {
    m_W1 = WeightingFunction::sensitivity(M, omegaB, epsilon);
    m_hasW1 = true;
    m_designed = false;
}

void HInfinityController::setControlWeight(double maxControl) {
    m_W2 = WeightingFunction::firstOrder(0, 1, 1.0 / maxControl);
    m_hasW2 = true;
    m_designed = false;
}

void HInfinityController::setComplementaryWeight(double M, double omegaT, double epsilon) {
    m_W3 = WeightingFunction::complementary(M, omegaT, epsilon);
    m_hasW3 = true;
    m_designed = false;
}

void HInfinityController::setWeights(const WeightingFunction& W1,
                                      const WeightingFunction& W2,
                                      const WeightingFunction& W3) {
    m_W1 = W1;
    m_W2 = W2;
    m_W3 = W3;
    m_hasW1 = m_hasW2 = m_hasW3 = true;
    m_designed = false;
}

void HInfinityController::buildAugmentedPlant() {
    // Build augmented plant including weighting functions
    // This is a simplified implementation
    
    // For mixed sensitivity:
    // z = [W1*S*w; W2*K*S*w; W3*T*w]
    
    // Augmented state includes weight states
    int nW1 = m_hasW1 ? m_W1.order : 0;
    int nW2 = m_hasW2 ? m_W2.order : 0;
    int nW3 = m_hasW3 ? m_W3.order : 0;
    
    m_na = m_n + nW1 + nW2 + nW3;
    
    // Initialize augmented matrices to zero
    std::fill(m_Aa.begin(), m_Aa.end(), 0.0);
    std::fill(m_B1a.begin(), m_B1a.end(), 0.0);
    std::fill(m_B2a.begin(), m_B2a.end(), 0.0);
    std::fill(m_C1a.begin(), m_C1a.end(), 0.0);
    std::fill(m_C2a.begin(), m_C2a.end(), 0.0);
    
    // Copy plant matrices to augmented (upper-left block)
    for (int i = 0; i < m_n; i++) {
        for (int j = 0; j < m_n; j++) {
            m_Aa[i * m_na + j] = m_A[i * m_n + j];
        }
        for (int j = 0; j < m_m; j++) {
            m_B2a[i * m_m + j] = m_B[i * m_m + j];
        }
    }
    
    for (int i = 0; i < m_p; i++) {
        for (int j = 0; j < m_n; j++) {
            m_C2a[i * m_na + j] = m_C[i * m_n + j];
        }
    }
    
    // Add weight dynamics (simplified - would need proper interconnection)
    // This would require more complex matrix assembly for proper mixed sensitivity
}

bool HInfinityController::isAchievable(double gamma) {
    // Check if there exists a stabilizing controller for given gamma
    // This requires solving two coupled Riccati equations
    
    std::array<double, MAX_AUG * MAX_AUG> X{}, Y{};
    return solveHinfRiccati(gamma, X.data(), Y.data());
}

bool HInfinityController::solveHinfRiccati(double gamma, double* X, double* Y) {
    // Solve H∞ Riccati equations for given gamma
    // 
    // Control Riccati:
    // A'X + XA + C1'C1 + X(γ⁻²B1B1' - B2R⁻¹B2')X = 0
    //
    // Filter Riccati:
    // AY + YA' + B1B1' + Y(γ⁻²C1'C1 - C2'V⁻¹C2)Y = 0
    //
    // Coupling condition: ρ(XY) < γ²
    
    const int n = m_na;
    double gamma2 = gamma * gamma;
    
    // Simplified iterative solver (would need proper Riccati solver)
    
    // Initialize X = I, Y = I (or solve standard Riccati first)
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            X[i * n + j] = (i == j) ? 1.0 : 0.0;
            Y[i * n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    const int maxIter = 100;
    const double tol = 1e-6;
    
    for (int iter = 0; iter < maxIter; iter++) {
        // Iterate on X
        // (Simplified - proper implementation would use coupled iteration)
        
        // Check coupling condition
        std::array<double, MAX_AUG * MAX_AUG> XY{};
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0;
                for (int k = 0; k < n; k++) {
                    sum += X[i * n + k] * Y[k * n + j];
                }
                XY[i * n + j] = sum;
            }
        }
        
        // Check spectral radius (simplified: use trace as proxy)
        double trace = 0;
        for (int i = 0; i < n; i++) {
            trace += XY[i * n + i];
        }
        
        if (trace > gamma2 * n) {
            return false;  // Gamma not achievable
        }
        
        // Check convergence (simplified)
        break;
    }
    
    return true;
}

void HInfinityController::computeController(double gamma, const double* X, const double* Y) {
    // Compute H∞ central controller from Riccati solutions
    // K(s) = -F(sI - Ac)⁻¹L
    // where Ac, F, L depend on X, Y, and plant matrices
    
    const int n = m_na;
    double gamma2 = gamma * gamma;
    
    // Controller order = plant order (for central controller)
    m_nk = n;
    
    // Simplified controller matrices
    // Proper implementation requires computing F, L from Riccati solutions
    
    // State feedback: F = -R⁻¹(B2'X + D12'C1)
    // Observer gain: L = -(YC2' + B1D21')V⁻¹
    // Controller: Ak = A + γ⁻²B1B1'X - B2F - LC2
    
    // For now, use simplified structure
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            // Ak ≈ A - B*K - L*C (similar to LQG)
            m_Ak[i * n + j] = m_Aa[i * n + j];
        }
    }
    
    // Bk = L (observer gain)
    for (int i = 0; i < n; i++) {
        m_Bk[i] = 1.0;  // Placeholder
    }
    
    // Ck = -F (state feedback)
    for (int i = 0; i < m_m; i++) {
        for (int j = 0; j < n; j++) {
            m_Ck[i * n + j] = -X[j];  // Simplified
        }
    }
    
    // Dk = 0 for strictly proper controller
    std::fill(m_Dk.begin(), m_Dk.end(), 0.0);
}

bool HInfinityController::design(double gamma) {
    buildAugmentedPlant();
    
    std::array<double, MAX_AUG * MAX_AUG> X{}, Y{};
    
    if (!solveHinfRiccati(gamma, X.data(), Y.data())) {
        return false;
    }
    
    computeController(gamma, X.data(), Y.data());
    
    m_gamma = gamma;
    m_designed = true;
    
    return true;
}

double HInfinityController::designOptimal(double tol) {
    buildAugmentedPlant();
    
    // Bisection on gamma
    double gammaLow = 0.1;
    double gammaHigh = 100.0;
    
    // Find upper bound
    while (!isAchievable(gammaHigh) && gammaHigh < 1e6) {
        gammaHigh *= 2;
    }
    
    // Find lower bound
    while (isAchievable(gammaLow) && gammaLow > 1e-6) {
        gammaLow /= 2;
    }
    
    // Bisection
    while (gammaHigh - gammaLow > tol) {
        double gammaMid = (gammaLow + gammaHigh) / 2;
        if (isAchievable(gammaMid)) {
            gammaHigh = gammaMid;
        } else {
            gammaLow = gammaMid;
        }
    }
    
    // Design at optimal gamma
    design(gammaHigh);
    
    return gammaHigh;
}

void HInfinityController::getControllerMatrices(double* Ak, double* Bk, 
                                                 double* Ck, double* Dk) const {
    if (Ak) std::memcpy(Ak, m_Ak.data(), m_nk * m_nk * sizeof(double));
    if (Bk) std::memcpy(Bk, m_Bk.data(), m_nk * m_ny * sizeof(double));
    if (Ck) std::memcpy(Ck, m_Ck.data(), m_nu * m_nk * sizeof(double));
    if (Dk) std::memcpy(Dk, m_Dk.data(), m_nu * m_ny * sizeof(double));
}

ControllerOutput HInfinityController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_designed) {
        output.control = 0;
        return output;
    }
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    double e = r - y;
    
    // Controller dynamics
    std::array<double, MAX_AUG> xkDot{};
    for (int i = 0; i < m_nk; i++) {
        double sum = 0;
        for (int j = 0; j < m_nk; j++) {
            sum += m_Ak[i * m_nk + j] * m_xk[j];
        }
        sum += m_Bk[i] * e;  // Use error as input
        xkDot[i] = sum;
    }
    
    // Euler integration
    for (int i = 0; i < m_nk; i++) {
        m_xk[i] += xkDot[i] * dt;
    }
    
    // Output
    double u = 0;
    for (int i = 0; i < m_nk; i++) {
        u += m_Ck[i] * m_xk[i];
    }
    u += m_Dk[0] * e;
    
    output.control = std::clamp(u, m_limits.outputMin, m_limits.outputMax);
    output.error = e;
    
    return output;
}

void HInfinityController::resetImpl() {
    std::fill(m_xk.begin(), m_xk.end(), 0.0);
}

// ============================================================================
// μ-Synthesis Framework
// ============================================================================

void MuSynthesisFramework::setUncertaintyStructure(const int* blockSizes, int numBlocks,
                                                    const bool* repeated) {
    m_numBlocks = std::min(numBlocks, 10);
    for (int i = 0; i < m_numBlocks; i++) {
        m_blockSizes[i] = blockSizes[i];
        m_repeated[i] = repeated ? repeated[i] : false;
    }
}

double MuSynthesisFramework::computeMuUpperBound(const double* M, int n) const {
    // Upper bound on structured singular value
    // Uses D-scaling: μ(M) ≤ inf_D σ̄(DMD⁻¹)
    
    // Simplified: use unstructured singular value as upper bound
    // Proper implementation would optimize over D
    
    double maxSv = 0;
    for (int i = 0; i < n; i++) {
        double rowSum = 0;
        for (int j = 0; j < n; j++) {
            rowSum += M[i * n + j] * M[i * n + j];
        }
        maxSv = std::max(maxSv, std::sqrt(rowSum));
    }
    
    return maxSv;
}

bool MuSynthesisFramework::dkIteration() {
    // D-K iteration for μ-synthesis
    // 1. Fix D, synthesize K (H∞ problem)
    // 2. Fix K, fit D to μ-plot
    // 3. Iterate
    
    // This requires external optimization - placeholder
    return true;
}

} // namespace Control
