/**
 * @file LQRController.cpp
 * @brief LQR Controller implementation
 * 
 * Split from StateSpaceControllers.cpp for maintainability.
 */

#include "control/StateSpaceControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace tether::control {

// External matrix functions from MatrixUtils.cpp
extern void matMul(const double* A, const double* B, double* C, int m, int n, int p);
extern void matTranspose(const double* A, double* B, int m, int n);
extern void matAdd(const double* A, const double* B, double* C, int m, int n);
extern void matCopy(const double* src, double* dst, int m, int n);
extern void matIdentity(double* A, int n);
extern bool matInverse(const double* A, double* Ainv, int n);

// ============================================================================
// LQR Controller
// ============================================================================

void LQRController::setSystemMatrices(const double* A, const double* B, int n, int m) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    m_isDiscrete = false;
    m_gainComputed = false;
    
    std::memcpy(m_A.data(), A, m_n * m_n * sizeof(double));
    std::memcpy(m_B.data(), B, m_n * m_m * sizeof(double));
}

void LQRController::setDiscreteSystemMatrices(const double* Ad, const double* Bd, 
                                               int n, int m) {
    setSystemMatrices(Ad, Bd, n, m);
    m_isDiscrete = true;
}

void LQRController::setWeightMatrices(const double* Q, const double* R) {
    std::memcpy(m_Q.data(), Q, m_n * m_n * sizeof(double));
    std::memcpy(m_R.data(), R, m_m * m_m * sizeof(double));
    m_gainComputed = false;
}

void LQRController::setGainMatrix(const double* K) {
    std::memcpy(m_K.data(), K, m_m * m_n * sizeof(double));
    m_gainComputed = true;
}

void LQRController::getGainMatrix(double* K) const {
    std::memcpy(K, m_K.data(), m_m * m_n * sizeof(double));
}

void LQRController::setReferenceState(const double* ref) {
    std::memcpy(m_xRef.data(), ref, m_n * sizeof(double));
}

bool LQRController::computeGain() {
    if (m_n == 0 || m_m == 0) return false;
    
    if (m_isDiscrete) {
        return solveDiscreteRiccati();
    } else {
        return solveRiccati();
    }
}

bool LQRController::solveRiccati() {
    const int n = m_n;
    const int m = m_m;
    
    matCopy(m_Q.data(), m_P.data(), n, n);
    
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> Rinv{};
    if (!matInverse(m_R.data(), Rinv.data(), m)) {
        return false;
    }
    
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> BRinv{};
    matMul(m_B.data(), Rinv.data(), BRinv.data(), n, m, m);
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> BRinvBt{};
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> Bt{};
    matTranspose(m_B.data(), Bt.data(), n, m);
    matMul(BRinv.data(), Bt.data(), BRinvBt.data(), n, m, n);
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> Pnew{};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> At{};
    matTranspose(m_A.data(), At.data(), n, n);
    
    const int maxIter = 1000;
    const double tol = 1e-10;
    double dt = 0.01;
    
    for (int iter = 0; iter < maxIter; iter++) {
        std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> AtP{}, PA{}, PBRBtP{};
        
        matMul(At.data(), m_P.data(), AtP.data(), n, n, n);
        matMul(m_P.data(), m_A.data(), PA.data(), n, n, n);
        
        std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> PBRinvBt{};
        matMul(m_P.data(), BRinvBt.data(), PBRinvBt.data(), n, n, n);
        matMul(PBRinvBt.data(), m_P.data(), PBRBtP.data(), n, n, n);
        
        for (int i = 0; i < n * n; i++) {
            double Pdot = AtP[i] + PA[i] - PBRBtP[i] + m_Q[i];
            Pnew[i] = m_P[i] + dt * Pdot;
        }
        
        double diff = 0;
        for (int i = 0; i < n * n; i++) {
            diff += (Pnew[i] - m_P[i]) * (Pnew[i] - m_P[i]);
        }
        
        matCopy(Pnew.data(), m_P.data(), n, n);
        
        if (std::sqrt(diff) < tol) {
            break;
        }
    }
    
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> BtP{};
    matMul(Bt.data(), m_P.data(), BtP.data(), m, n, n);
    matMul(Rinv.data(), BtP.data(), m_K.data(), m, m, n);
    
    m_gainComputed = true;
    return true;
}

bool LQRController::solveDiscreteRiccati() {
    const int n = m_n;
    const int m = m_m;
    
    matCopy(m_Q.data(), m_P.data(), n, n);
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> Pnew{};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> At{};
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> Bt{};
    
    matTranspose(m_A.data(), At.data(), n, n);
    matTranspose(m_B.data(), Bt.data(), n, m);
    
    const int maxIter = 1000;
    const double tol = 1e-10;
    
    for (int iter = 0; iter < maxIter; iter++) {
        std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> PA{}, AtPA{};
        matMul(m_P.data(), m_A.data(), PA.data(), n, n, n);
        matMul(At.data(), PA.data(), AtPA.data(), n, n, n);
        
        std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> PB{};
        std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> BtPB{};
        matMul(m_P.data(), m_B.data(), PB.data(), n, n, m);
        matMul(Bt.data(), PB.data(), BtPB.data(), m, n, m);
        
        std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> RpBtPB{};
        matAdd(m_R.data(), BtPB.data(), RpBtPB.data(), m, m);
        
        std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> RpBtPBinv{};
        if (!matInverse(RpBtPB.data(), RpBtPBinv.data(), m)) {
            return false;
        }
        
        std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> AtPB{};
        matMul(At.data(), PB.data(), AtPB.data(), n, n, m);
        
        std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> AtPBRinv{};
        matMul(AtPB.data(), RpBtPBinv.data(), AtPBRinv.data(), n, m, m);
        
        std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> BtPA{};
        matMul(Bt.data(), PA.data(), BtPA.data(), m, n, n);
        
        std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> term{};
        matMul(AtPBRinv.data(), BtPA.data(), term.data(), n, m, n);
        
        for (int i = 0; i < n * n; i++) {
            Pnew[i] = AtPA[i] - term[i] + m_Q[i];
        }
        
        double diff = 0;
        for (int i = 0; i < n * n; i++) {
            diff += (Pnew[i] - m_P[i]) * (Pnew[i] - m_P[i]);
        }
        
        matCopy(Pnew.data(), m_P.data(), n, n);
        
        if (std::sqrt(diff) < tol) {
            break;
        }
    }
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> PA{};
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> PB{};
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> BtPB{};
    
    matMul(m_P.data(), m_A.data(), PA.data(), n, n, n);
    matMul(m_P.data(), m_B.data(), PB.data(), n, n, m);
    matMul(Bt.data(), PB.data(), BtPB.data(), m, n, m);
    
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> RpBtPB{};
    matAdd(m_R.data(), BtPB.data(), RpBtPB.data(), m, m);
    
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROL_DIM> RpBtPBinv{};
    matInverse(RpBtPB.data(), RpBtPBinv.data(), m);
    
    std::array<double, MAX_CONTROL_DIM * MAX_STATE_DIM> BtPA{};
    matMul(Bt.data(), PA.data(), BtPA.data(), m, n, n);
    matMul(RpBtPBinv.data(), BtPA.data(), m_K.data(), m, m, n);
    
    m_gainComputed = true;
    return true;
}

ControllerOutput LQRController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_gainComputed || m_n == 0) {
        output.control = 0;
        return output;
    }
    
    std::array<double, MAX_STATE_DIM> xerr{};
    for (int i = 0; i < m_n; i++) {
        xerr[i] = input.state[i] - m_xRef[i];
    }
    
    std::array<double, MAX_CONTROL_DIM> u{};
    for (int i = 0; i < m_m; i++) {
        double sum = 0;
        for (int j = 0; j < m_n; j++) {
            sum += m_K[i * m_n + j] * xerr[j];
        }
        u[i] = -sum;
    }
    
    output.control = std::clamp(u[0], m_limits.outputMin, m_limits.outputMax);
    
    for (int i = 0; i < m_m; i++) {
        output.controlVector[i] = u[i];
    }
    
    output.error = xerr[0];
    
    return output;
}

void LQRController::resetImpl() {
    std::fill(m_xRef.begin(), m_xRef.end(), 0.0);
}

} // namespace tether::control
