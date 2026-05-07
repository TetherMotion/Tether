/**
 * @file KalmanFilter.cpp
 * @brief Kalman Filter implementation
 * 
 * Split from StateSpaceControllers.cpp for maintainability.
 */

#include "control/StateSpaceControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace Control {

// External matrix functions from MatrixUtils.cpp
extern void matMul(const double* A, const double* B, double* C, int m, int n, int p);
extern void matTranspose(const double* A, double* B, int m, int n);
extern void matCopy(const double* src, double* dst, int m, int n);
extern bool matInverse(const double* A, double* Ainv, int n);

// ============================================================================
// Kalman Filter
// ============================================================================

void KalmanFilter::setSystemMatrices(const double* A, const double* B, const double* C,
                                      int n, int m, int p) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    m_p = std::min(p, static_cast<int>(MAX_OUTPUT_DIM));
    
    std::memcpy(m_A.data(), A, m_n * m_n * sizeof(double));
    std::memcpy(m_B.data(), B, m_n * m_m * sizeof(double));
    std::memcpy(m_C.data(), C, m_p * m_n * sizeof(double));
    
    m_gainComputed = false;
}

void KalmanFilter::setNoiseCovariances(const double* W, const double* V) {
    std::memcpy(m_W.data(), W, m_n * m_n * sizeof(double));
    std::memcpy(m_V.data(), V, m_p * m_p * sizeof(double));
    m_gainComputed = false;
}

void KalmanFilter::setKalmanGain(const double* L) {
    std::memcpy(m_L.data(), L, m_n * m_p * sizeof(double));
    m_gainComputed = true;
}

void KalmanFilter::setInitialState(const double* x0) {
    std::memcpy(m_xHat.data(), x0, m_n * sizeof(double));
}

void KalmanFilter::getEstimatedState(double* x) const {
    std::memcpy(x, m_xHat.data(), m_n * sizeof(double));
}

bool KalmanFilter::computeGain() {
    return solveFilterRiccati();
}

bool KalmanFilter::solveFilterRiccati() {
    const int n = m_n;
    const int p = m_p;
    
    matCopy(m_W.data(), m_P.data(), n, n);
    
    std::array<double, MAX_OUTPUT_DIM * MAX_OUTPUT_DIM> Vinv{};
    if (!matInverse(m_V.data(), Vinv.data(), p)) {
        return false;
    }
    
    std::array<double, MAX_STATE_DIM * MAX_OUTPUT_DIM> Ct{};
    matTranspose(m_C.data(), Ct.data(), p, n);
    
    std::array<double, MAX_STATE_DIM * MAX_OUTPUT_DIM> CtVinv{};
    matMul(Ct.data(), Vinv.data(), CtVinv.data(), n, p, p);
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> CtVinvC{};
    matMul(CtVinv.data(), m_C.data(), CtVinvC.data(), n, p, n);
    
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> Pnew{};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> At{};
    matTranspose(m_A.data(), At.data(), n, n);
    
    const int maxIter = 1000;
    const double tol = 1e-10;
    double dt = 0.01;
    
    for (int iter = 0; iter < maxIter; iter++) {
        std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> AP{}, PAt{}, PCtVC{}, PCtVCP{};
        
        matMul(m_A.data(), m_P.data(), AP.data(), n, n, n);
        matMul(m_P.data(), At.data(), PAt.data(), n, n, n);
        matMul(m_P.data(), CtVinvC.data(), PCtVC.data(), n, n, n);
        matMul(PCtVC.data(), m_P.data(), PCtVCP.data(), n, n, n);
        
        for (int i = 0; i < n * n; i++) {
            double Pdot = AP[i] + PAt[i] - PCtVCP[i] + m_W[i];
            Pnew[i] = m_P[i] + dt * Pdot;
        }
        
        double diff = 0;
        for (int i = 0; i < n * n; i++) {
            diff += (Pnew[i] - m_P[i]) * (Pnew[i] - m_P[i]);
        }
        
        matCopy(Pnew.data(), m_P.data(), n, n);
        
        if (std::sqrt(diff) < tol) break;
    }
    
    std::array<double, MAX_STATE_DIM * MAX_OUTPUT_DIM> PCt{};
    matMul(m_P.data(), Ct.data(), PCt.data(), n, n, p);
    matMul(PCt.data(), Vinv.data(), m_L.data(), n, p, p);
    
    m_gainComputed = true;
    return true;
}

void KalmanFilter::predict(const double* u, double dt) {
    std::array<double, MAX_STATE_DIM> xNew{};
    
    for (int i = 0; i < m_n; i++) {
        double sum = 0;
        for (int j = 0; j < m_n; j++) {
            sum += m_A[i * m_n + j] * m_xHat[j];
        }
        xNew[i] = sum;
    }
    
    for (int i = 0; i < m_n; i++) {
        for (int j = 0; j < m_m; j++) {
            xNew[i] += m_B[i * m_m + j] * u[j];
        }
    }
    
    matCopy(xNew.data(), m_xHat.data(), m_n, 1);
}

void KalmanFilter::update(const double* y) {
    if (!m_gainComputed) return;
    
    std::array<double, MAX_OUTPUT_DIM> innovation{};
    for (int i = 0; i < m_p; i++) {
        double Cxhat = 0;
        for (int j = 0; j < m_n; j++) {
            Cxhat += m_C[i * m_n + j] * m_xHat[j];
        }
        innovation[i] = y[i] - Cxhat;
    }
    
    for (int i = 0; i < m_n; i++) {
        double correction = 0;
        for (int j = 0; j < m_p; j++) {
            correction += m_L[i * m_p + j] * innovation[j];
        }
        m_xHat[i] += correction;
    }
}

void KalmanFilter::getState(double* x) const {
    std::memcpy(x, m_xHat.data(), m_n * sizeof(double));
}

StateVector KalmanFilter::estimate(const OutputVector& measurement,
                                   const ControlVector& control,
                                   double dt) {
    predict(control.data(), dt);
    update(measurement.data());
    StateVector result{};
    getState(result.data());
    return result;
}

StateVector KalmanFilter::getState() const {
    StateVector result{};
    for (int i = 0; i < m_n && i < static_cast<int>(MAX_STATE_DIM); ++i) {
        result[i] = m_xHat[i];
    }
    return result;
}

size_t KalmanFilter::getStateDim() const {
    return static_cast<size_t>(m_n);
}

void KalmanFilter::reset() {
    std::fill(m_xHat.begin(), m_xHat.end(), 0.0);
}

} // namespace Control
