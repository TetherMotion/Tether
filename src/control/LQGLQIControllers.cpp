/**
 * @file LQGLQIControllers.cpp
 * @brief LQG, LQI controllers and StateSpace helper functions
 * 
 * Split from StateSpaceControllers.cpp for maintainability.
 */

#include "control/StateSpaceControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Dense>

namespace Control {

// External matrix functions from MatrixUtils.cpp
extern void matMul(const double* A, const double* B, double* C, int m, int n, int p);
extern void matTranspose(const double* A, double* B, int m, int n);
extern void matAdd(const double* A, const double* B, double* C, int m, int n);
extern void matSub(const double* A, const double* B, double* C, int m, int n);
extern void matScale(const double* A, double alpha, double* B, int m, int n);
extern void matCopy(const double* src, double* dst, int m, int n);
extern void matIdentity(double* A, int n);
extern double matNorm(const double* A, int m, int n);
extern bool matInverse(const double* A, double* Ainv, int n);

// ============================================================================
// LQG Controller
// ============================================================================

void LQGController::setSystemMatrices(const double* A, const double* B,
                                       const double* C, const double* D,
                                       int n, int m, int p) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    m_p = std::min(p, static_cast<int>(MAX_OUTPUT_DIM));

    m_lqr.setSystemMatrices(A, B, m_n, m_m);

    // Convert raw row-major arrays to Eigen matrices for the Eigen-based KF
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m_n, m_n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, m_n, m_m);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenC(C, m_p, m_n);

    m_kf.setSystemMatrices(eigenA, eigenB, eigenC);

    std::memcpy(m_C.data(), C, static_cast<size_t>(m_p) * static_cast<size_t>(m_n) * sizeof(double));
}

void LQGController::setLQRWeights(const double* Q, const double* R) {
    m_lqr.setWeightMatrices(Q, R);
}

void LQGController::setNoiseCovariances(const double* W, const double* V) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenW(W, m_n, m_n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenV(V, m_p, m_p);
    m_kf.setNoiseCovariances(eigenW, eigenV);
}

bool LQGController::design() {
    bool lqrOk = m_lqr.computeGain();
    bool kfOk = m_kf.computeGain();
    return lqrOk && kfOk;
}

ControllerOutput LQGController::computeImpl(const ControllerInput& input) {
    Eigen::VectorXd u(static_cast<Eigen::Index>(m_m));
    u.setZero();
    u(0) = m_lastControl;
    m_kf.predict(u);

    Eigen::VectorXd y(static_cast<Eigen::Index>(m_p));
    y.setZero();
    y(0) = input.measured;
    m_kf.update(y);

    Eigen::VectorXd xHat = m_kf.getStateVector();

    ControllerInput lqrInput = input;
    for (int i = 0; i < m_n && i < static_cast<int>(MAX_STATE_DIM); ++i) {
        lqrInput.state[i] = xHat(i);
    }

    double xRef[MAX_STATE_DIM] = {m_reference};
    m_lqr.setReferenceState(xRef);

    ControllerOutput output = m_lqr.compute(lqrInput);

    m_lastControl = output.control;

    return output;
}

void LQGController::resetImpl() {
    m_lqr.reset();
    m_kf.reset();
    m_lastControl = 0;
}

// ============================================================================
// LQI Controller
// ============================================================================

void LQIController::setSystemMatrices(const double* A, const double* B, const double* C,
                                       int n, int m, int p) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    m_p = std::min(p, static_cast<int>(MAX_OUTPUT_DIM));
    
    std::memcpy(m_A.data(), A, m_n * m_n * sizeof(double));
    std::memcpy(m_B.data(), B, m_n * m_m * sizeof(double));
    std::memcpy(m_C.data(), C, m_p * m_n * sizeof(double));
    
    int na = m_n + m_p;
    
    std::fill(m_Aa.begin(), m_Aa.end(), 0.0);
    std::fill(m_Ba.begin(), m_Ba.end(), 0.0);
    
    for (int i = 0; i < m_n; i++) {
        for (int j = 0; j < m_n; j++) {
            m_Aa[i * na + j] = m_A[i * m_n + j];
        }
    }
    
    for (int i = 0; i < m_p; i++) {
        for (int j = 0; j < m_n; j++) {
            m_Aa[(m_n + i) * na + j] = -m_C[i * m_n + j];
        }
    }
    
    for (int i = 0; i < m_n; i++) {
        for (int j = 0; j < m_m; j++) {
            m_Ba[i * m_m + j] = m_B[i * m_m + j];
        }
    }
    
    m_designed = false;
}

void LQIController::setAugmentedWeights(const double* Qa, const double* R) {
    int na = std::min(m_n + m_p, static_cast<int>(MAX_AUG));
    std::memcpy(m_Qa.data(), Qa, static_cast<size_t>(na) * static_cast<size_t>(na) * sizeof(double));
    std::memcpy(m_R.data(), R, static_cast<size_t>(m_m) * static_cast<size_t>(m_m) * sizeof(double));
    m_designed = false;
}

void LQIController::setWeights(const double* Qx, const double* Qi, const double* R) {
    int na = m_n + m_p;
    
    std::fill(m_Qa.begin(), m_Qa.end(), 0.0);
    
    for (int i = 0; i < m_n; i++) {
        for (int j = 0; j < m_n; j++) {
            m_Qa[i * na + j] = Qx[i * m_n + j];
        }
    }
    
    for (int i = 0; i < m_p; i++) {
        for (int j = 0; j < m_p; j++) {
            m_Qa[(m_n + i) * na + (m_n + j)] = Qi[i * m_p + j];
        }
    }
    
    std::memcpy(m_R.data(), R, m_m * m_m * sizeof(double));
    m_designed = false;
}

void LQIController::setIntegralLimits(double min, double max) {
    m_integralMin = min;
    m_integralMax = max;
}

bool LQIController::design() {
    int na = m_n + m_p;
    
    LQRController lqrAug;
    lqrAug.setSystemMatrices(m_Aa.data(), m_Ba.data(), na, m_m);
    lqrAug.setWeightMatrices(m_Qa.data(), m_R.data());
    
    if (!lqrAug.computeGain()) {
        return false;
    }
    
    std::array<double, MAX_AUG * MAX_CONTROL_DIM> Ka{};
    lqrAug.getGainMatrix(Ka.data());
    
    for (int i = 0; i < m_m; i++) {
        for (int j = 0; j < m_n; j++) {
            m_Kx[i * m_n + j] = Ka[i * na + j];
        }
        for (int j = 0; j < m_p; j++) {
            m_Ki[i * m_p + j] = Ka[i * na + m_n + j];
        }
    }
    
    m_designed = true;
    return true;
}

void LQIController::getStateGain(double* Kx) const {
    std::memcpy(Kx, m_Kx.data(), m_m * m_n * sizeof(double));
}

void LQIController::getIntegralGain(double* Ki) const {
    std::memcpy(Ki, m_Ki.data(), m_m * m_p * sizeof(double));
}

void LQIController::getIntegralState(double* xi) const {
    std::memcpy(xi, m_xi.data(), m_p * sizeof(double));
}

ControllerOutput LQIController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_designed || m_n == 0) {
        output.control = 0;
        return output;
    }
    
    double dt = input.dt;
    
    std::array<double, MAX_OUTPUT_DIM> y{};
    // y = C * x  (C is m_p × m_n, row-major)
    {
        Eigen::Map<const Eigen::MatrixXd> C(m_C.data(), m_p, m_n);
        Eigen::Map<const Eigen::VectorXd> state(input.state.data(), m_n);
        Eigen::VectorXd yVec = C * state;
        for (int i = 0; i < m_p; ++i) y[i] = yVec(i);
    }

    for (int i = 0; i < m_p; i++) {
        double error = input.reference - y[i];
        m_xi[i] += error * dt;
        m_xi[i] = std::clamp(m_xi[i], m_integralMin, m_integralMax);
    }

    std::array<double, MAX_CONTROL_DIM> u{};
    // u = -Kx * x - Ki * xi  (Kx is m_m × m_n, Ki is m_m × m_p, row-major)
    {
        Eigen::Map<const Eigen::MatrixXd> Kx(m_Kx.data(), m_m, m_n);
        Eigen::Map<const Eigen::MatrixXd> Ki(m_Ki.data(), m_m, m_p);
        Eigen::Map<const Eigen::VectorXd> state(input.state.data(), m_n);
        Eigen::Map<Eigen::VectorXd> xi(m_xi.data(), m_p);
        Eigen::VectorXd uVec = -(Kx * state + Ki * xi);
        for (int i = 0; i < m_m; ++i) u[i] = uVec(i);
    }
    
    output.control = std::clamp(u[0], m_limits.outputMin, m_limits.outputMax);
    output.error = input.reference - y[0];
    output.integral = m_xi[0];
    
    for (int i = 0; i < m_m; i++) {
        output.controlVector[i] = u[i];
    }
    
    return output;
}

void LQIController::resetImpl() {
    std::fill(m_xi.begin(), m_xi.end(), 0.0);
}

// ============================================================================
// State-Space Helper Functions
// ============================================================================

namespace StateSpace {

void discretize(const double* A, const double* B, double dt,
                double* Ad, double* Bd, int n, int m) {
    matIdentity(Ad, n);
    for (int i = 0; i < n * n; i++) {
        Ad[i] += A[i] * dt;
    }
    
    for (int i = 0; i < n * m; i++) {
        Bd[i] = B[i] * dt;
    }
}

bool isControllable(const double* A, const double* B, int n, int m) {
    std::vector<double> C(n * n * m);
    std::vector<double> temp(n * m);
    std::vector<double> Apow(n * n);
    
    matIdentity(Apow.data(), n);
    
    for (int k = 0; k < n; k++) {
        matMul(Apow.data(), B, temp.data(), n, n, m);
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                C[i * n * m + k * m + j] = temp[i * m + j];
            }
        }
        
        std::vector<double> newApow(n * n);
        matMul(Apow.data(), A, newApow.data(), n, n, n);
        matCopy(newApow.data(), Apow.data(), n, n);
    }
    
    return true;
}

bool isObservable(const double* A, const double* C, int n, int p) {
    return true;
}

void matrixExponential(const double* A, double t, double* expAt, int n) {
    std::vector<double> At(n * n);
    for (int i = 0; i < n * n; i++) {
        At[i] = A[i] * t;
    }
    
    int s = 0;
    double norm = matNorm(At.data(), n, n);
    while (norm > 0.5) {
        for (int i = 0; i < n * n; i++) {
            At[i] /= 2;
        }
        norm /= 2;
        s++;
    }
    
    std::vector<double> I(n * n), X2(n * n), num(n * n), den(n * n);
    matIdentity(I.data(), n);
    
    matScale(At.data(), 0.5, X2.data(), n, n);
    
    matAdd(I.data(), X2.data(), num.data(), n, n);
    
    matSub(I.data(), X2.data(), den.data(), n, n);
    
    std::vector<double> denInv(n * n);
    matInverse(den.data(), denInv.data(), n);
    matMul(denInv.data(), num.data(), expAt, n, n, n);
    
    for (int i = 0; i < s; i++) {
        std::vector<double> temp(n * n);
        matMul(expAt, expAt, temp.data(), n, n, n);
        matCopy(temp.data(), expAt, n, n);
    }
}

} // namespace StateSpace

} // namespace Control
