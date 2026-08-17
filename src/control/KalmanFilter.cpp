/**
 * @file KalmanFilter.cpp
 * @brief Eigen-based discrete-time Kalman filter implementation
 */

#include "control/KalmanFilter.hpp"
#include <cmath>
#include <limits>

namespace tether::control {

KalmanFilter::KalmanFilter(size_t n, size_t m, size_t p)
    : n_(n), m_(m), p_(p) {
    resize();
}

void KalmanFilter::resize() {
    A_.setZero(static_cast<Eigen::Index>(n_), static_cast<Eigen::Index>(n_));
    B_.setZero(static_cast<Eigen::Index>(n_), static_cast<Eigen::Index>(m_));
    C_.setZero(static_cast<Eigen::Index>(p_), static_cast<Eigen::Index>(n_));
    W_.setZero(static_cast<Eigen::Index>(n_), static_cast<Eigen::Index>(n_));
    V_.setZero(static_cast<Eigen::Index>(p_), static_cast<Eigen::Index>(p_));
    P_.setZero(static_cast<Eigen::Index>(n_), static_cast<Eigen::Index>(n_));
    L_.setZero(static_cast<Eigen::Index>(n_), static_cast<Eigen::Index>(p_));
    x_.setZero(static_cast<Eigen::Index>(n_));
}

void KalmanFilter::setSystemMatrices(const Eigen::MatrixXd& A,
                                     const Eigen::MatrixXd& B,
                                     const Eigen::MatrixXd& C) {
    n_ = static_cast<size_t>(A.rows());
    m_ = static_cast<size_t>(B.cols());
    p_ = static_cast<size_t>(C.rows());
    resize();
    A_ = A;
    B_ = B;
    C_ = C;
    gain_computed_ = false;
}

void KalmanFilter::setNoiseCovariances(const Eigen::MatrixXd& W,
                                       const Eigen::MatrixXd& V) {
    W_ = W;
    V_ = V;
    gain_computed_ = false;
}

void KalmanFilter::setInitialState(const Eigen::VectorXd& x0,
                                   const Eigen::MatrixXd& P0) {
    x_ = x0;
    if (P0.size() > 0) {
        P_ = P0;
    }
}

void KalmanFilter::setKalmanGain(const Eigen::MatrixXd& L) {
    L_ = L;
    gain_computed_ = true;
}

bool KalmanFilter::computeGain() {
    return solveDiscreteRiccati();
}

bool KalmanFilter::solveDiscreteRiccati() {
    if (n_ == 0 || p_ == 0) {
        return false;
    }

    // Iterative solution of the discrete-time algebraic Riccati equation:
    // P = A*P*A' - A*P*C'*(C*P*C' + V)^-1*C*P*A' + W
    const int max_iter = 1000;
    const double tol = 1e-12;

    Eigen::MatrixXd P = W_;
    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::MatrixXd S = C_ * P * C_.transpose() + V_;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
        if (!lu.isInvertible()) {
            return false;
        }
        Eigen::MatrixXd K = P * C_.transpose() * lu.inverse();
        Eigen::MatrixXd P_new = A_ * (P - K * C_ * P) * A_.transpose() + W_;

        double diff = (P_new - P).norm();
        P = P_new;
        if (diff < tol) {
            L_ = A_ * P * C_.transpose() * lu.inverse();
            gain_computed_ = true;
            return true;
        }
    }

    return false;
}

Eigen::VectorXd KalmanFilter::predict(const Eigen::VectorXd& u) {
    x_ = A_ * x_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + W_;
    return x_;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd& y) {
    if (!gain_computed_) {
        // Time-varying update: compute gain from current P_
        Eigen::MatrixXd S = C_ * P_ * C_.transpose() + V_;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
        if (!lu.isInvertible()) {
            return x_;
        }
        L_ = P_ * C_.transpose() * lu.inverse();
    }

    Eigen::VectorXd innovation = y - C_ * x_;
    x_ = x_ + L_ * innovation;

    // Joseph form covariance update for numerical stability
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(n_),
                                                   static_cast<Eigen::Index>(n_));
    P_ = (I - L_ * C_) * P_ * (I - L_ * C_).transpose() + L_ * V_ * L_.transpose();

    return x_;
}

Eigen::VectorXd KalmanFilter::estimate(const Eigen::VectorXd& y,
                                       const Eigen::VectorXd& u) {
    predict(u);
    return update(y);
}

StateVector KalmanFilter::estimate(const OutputVector& measurement,
                                   const ControlVector& control,
                                   double /*dt*/) {
    Eigen::VectorXd y(static_cast<Eigen::Index>(p_));
    Eigen::VectorXd u(static_cast<Eigen::Index>(m_));
    for (size_t i = 0; i < p_; ++i) y(static_cast<Eigen::Index>(i)) = measurement[i];
    for (size_t i = 0; i < m_; ++i) u(static_cast<Eigen::Index>(i)) = control[i];

    predict(u);
    update(y);

    return getState();
}

StateVector KalmanFilter::getState() const {
    StateVector result{};
    const Eigen::Index dim = static_cast<Eigen::Index>(n_);
    for (Eigen::Index i = 0; i < dim && i < static_cast<Eigen::Index>(MAX_STATE_DIM); ++i) {
        result[static_cast<size_t>(i)] = x_(i);
    }
    return result;
}

void KalmanFilter::reset() {
    x_.setZero();
    P_ = W_;
    gain_computed_ = false;
}

} // namespace tether::control
