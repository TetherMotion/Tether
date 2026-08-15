/**
 * @file ExtendedKalmanFilter.cpp
 * @brief Eigen-based Extended Kalman filter implementation
 */

#include "control/ExtendedKalmanFilter.hpp"
#include <cmath>
#include <limits>

namespace Control {

namespace {

constexpr double kNumericalDiffEpsilon = 1e-7;

} // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(size_t n, size_t m, size_t p)
    : n_(n), m_(m), p_(p) {
    resize();
}

void ExtendedKalmanFilter::resize() {
    const Eigen::Index n = static_cast<Eigen::Index>(n_);
    const Eigen::Index m = static_cast<Eigen::Index>(m_);
    const Eigen::Index p = static_cast<Eigen::Index>(p_);

    x_.setZero(n);
    P_.setZero(n, n);
    W_.setZero(n, n);
    V_.setZero(p, p);
}

void ExtendedKalmanFilter::setStateDim(size_t n) {
    n_ = n;
    resize();
}

void ExtendedKalmanFilter::setInputDim(size_t m) {
    m_ = m;
    resize();
}

void ExtendedKalmanFilter::setOutputDim(size_t p) {
    p_ = p;
    resize();
}

void ExtendedKalmanFilter::setDims(size_t n, size_t m, size_t p) {
    n_ = n;
    m_ = m;
    p_ = p;
    resize();
}

void ExtendedKalmanFilter::setInitialState(const Eigen::VectorXd& x0,
                                           const Eigen::MatrixXd& P0) {
    x_ = x0;
    if (P0.size() > 0) {
        P_ = P0;
    }
}

void ExtendedKalmanFilter::setProcessNoise(const Eigen::MatrixXd& W) {
    W_ = W;
}

void ExtendedKalmanFilter::setMeasurementNoise(const Eigen::MatrixXd& V) {
    V_ = V;
}

void ExtendedKalmanFilter::setModelFunctions(StateFunction f,
                                             MeasurementFunction h) {
    f_ = std::move(f);
    h_ = std::move(h);
}

void ExtendedKalmanFilter::setJacobianFunctions(StateJacobian F,
                                                MeasurementJacobian H) {
    F_ = std::move(F);
    H_ = std::move(H);
    use_numdiff_ = false;
}

Eigen::MatrixXd ExtendedKalmanFilter::numericalJacobianF(const Eigen::VectorXd& x,
                                                         const Eigen::VectorXd& u,
                                                         double dt) const {
    const Eigen::Index n = static_cast<Eigen::Index>(n_);
    Eigen::MatrixXd J(n, n);
    Eigen::VectorXd f0 = f_(x, u, dt);
    for (Eigen::Index j = 0; j < n; ++j) {
        Eigen::VectorXd x_pert = x;
        x_pert(j) += kNumericalDiffEpsilon;
        Eigen::VectorXd f_pert = f_(x_pert, u, dt);
        J.col(j) = (f_pert - f0) / kNumericalDiffEpsilon;
    }
    return J;
}

Eigen::MatrixXd ExtendedKalmanFilter::numericalJacobianH(const Eigen::VectorXd& x) const {
    const Eigen::Index n = static_cast<Eigen::Index>(n_);
    const Eigen::Index p = static_cast<Eigen::Index>(p_);
    Eigen::MatrixXd J(p, n);
    Eigen::VectorXd h0 = h_(x);
    for (Eigen::Index j = 0; j < n; ++j) {
        Eigen::VectorXd x_pert = x;
        x_pert(j) += kNumericalDiffEpsilon;
        Eigen::VectorXd h_pert = h_(x_pert);
        J.col(j) = (h_pert - h0) / kNumericalDiffEpsilon;
    }
    return J;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::VectorXd& u, double dt) {
    x_ = f_(x_, u, dt);

    Eigen::MatrixXd F = F_ ? F_(x_, u, dt)
                           : numericalJacobianF(x_, u, dt);
    P_ = F * P_ * F.transpose() + W_;

    return x_;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd& y) {
    Eigen::MatrixXd H = H_ ? H_(x_)
                           : numericalJacobianH(x_);

    Eigen::MatrixXd S = H * P_ * H.transpose() + V_;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
    if (!lu.isInvertible()) {
        return x_;
    }

    Eigen::MatrixXd K = P_ * H.transpose() * lu.inverse();
    Eigen::VectorXd innovation = y - h_(x_);
    x_ = x_ + K * innovation;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(n_),
                                                   static_cast<Eigen::Index>(n_));
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * V_ * K.transpose();

    return x_;
}

Eigen::VectorXd ExtendedKalmanFilter::estimate(const Eigen::VectorXd& y,
                                               const Eigen::VectorXd& u,
                                               double dt) {
    predict(u, dt);
    return update(y);
}

StateVector ExtendedKalmanFilter::estimate(const OutputVector& measurement,
                                           const ControlVector& control,
                                           double dt) {
    Eigen::VectorXd y(static_cast<Eigen::Index>(p_));
    Eigen::VectorXd u(static_cast<Eigen::Index>(m_));
    for (size_t i = 0; i < p_; ++i) y(static_cast<Eigen::Index>(i)) = measurement[i];
    for (size_t i = 0; i < m_; ++i) u(static_cast<Eigen::Index>(i)) = control[i];

    predict(u, dt);
    update(y);

    return getState();
}

StateVector ExtendedKalmanFilter::getState() const {
    StateVector result{};
    const Eigen::Index dim = static_cast<Eigen::Index>(n_);
    for (Eigen::Index i = 0; i < dim && i < static_cast<Eigen::Index>(MAX_STATE_DIM); ++i) {
        result[static_cast<size_t>(i)] = x_(i);
    }
    return result;
}

void ExtendedKalmanFilter::reset() {
    x_.setZero();
    P_ = W_;
}

} // namespace Control
