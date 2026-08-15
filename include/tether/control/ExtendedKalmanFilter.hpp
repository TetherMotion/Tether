/**
 * @file ExtendedKalmanFilter.hpp
 * @brief Eigen-based Extended Kalman filter for nonlinear systems
 *
 * Discrete-time EKF using Eigen matrix types. The user supplies nonlinear
 * process and measurement models, and optionally their Jacobians. If Jacobians
 * are not supplied, the filter falls back to numerical differentiation.
 *
 * Process model:  x[k+1] = f(x[k], u[k], dt) + w[k]
 * Measurement model: y[k] = h(x[k]) + v[k]
 */

#pragma once

#include "control/ControllerBase.hpp"
#include <Eigen/Dense>
#include <functional>

namespace Control {

/**
 * @brief Extended Kalman filter for nonlinear state estimation
 *
 * The filter linearizes the nonlinear process and measurement models around
 * the current state estimate at each step. Process/measurement Jacobians may
 * be supplied explicitly; otherwise numerical differentiation is used.
 */
class ExtendedKalmanFilter : public StateEstimator {
public:
    using StateFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                                        const Eigen::VectorXd&,
                                                        double)>;
    using MeasurementFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using StateJacobian = std::function<Eigen::MatrixXd(const Eigen::VectorXd&,
                                                        const Eigen::VectorXd&,
                                                        double)>;
    using MeasurementJacobian = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;

    ExtendedKalmanFilter() = default;

    /**
     * @brief Construct with dimensions
     * @param n State dimension
     * @param m Input dimension
     * @param p Output dimension
     */
    ExtendedKalmanFilter(size_t n, size_t m, size_t p);

    /**
     * @brief Set state, input, and output dimensions
     */
    void setStateDim(size_t n);
    void setInputDim(size_t m);
    void setOutputDim(size_t p);
    void setDims(size_t n, size_t m, size_t p);

    /**
     * @brief Set initial state estimate and covariance
     */
    void setInitialState(const Eigen::VectorXd& x0,
                         const Eigen::MatrixXd& P0 = Eigen::MatrixXd());

    /**
     * @brief Set process and measurement noise covariances
     */
    void setProcessNoise(const Eigen::MatrixXd& W);
    void setMeasurementNoise(const Eigen::MatrixXd& V);

    /**
     * @brief Set nonlinear process and measurement functions
     * @param f Process function f(x, u, dt)
     * @param h Measurement function h(x)
     */
    void setModelFunctions(StateFunction f, MeasurementFunction h);

    /**
     * @brief Set analytic Jacobian functions
     * @param F State Jacobian F = df/dx
     * @param H Measurement Jacobian H = dh/dx
     */
    void setJacobianFunctions(StateJacobian F, MeasurementJacobian H);

    /**
     * @brief Enable or disable numerical differentiation fallback
     */
    void enableNumericalDifferentiation(bool enable) { use_numdiff_ = enable; }

    /**
     * @brief Prediction step
     * @param u Control input
     * @param dt Time step
     * @return Predicted state estimate
     */
    Eigen::VectorXd predict(const Eigen::VectorXd& u, double dt);

    /**
     * @brief Update step
     * @param y Measurement
     * @return Updated state estimate
     */
    Eigen::VectorXd update(const Eigen::VectorXd& y);

    /**
     * @brief Combined prediction + update step
     */
    Eigen::VectorXd estimate(const Eigen::VectorXd& y,
                             const Eigen::VectorXd& u,
                             double dt);

    /**
     * @brief StateEstimator interface implementation
     */
    StateVector estimate(const OutputVector& measurement,
                         const ControlVector& control,
                         double dt) override;

    /**
     * @brief Get current state estimate (StateEstimator interface)
     */
    StateVector getState() const override;

    /**
     * @brief Get current state estimate as Eigen vector
     */
    Eigen::VectorXd getStateVector() const { return x_; }

    /**
     * @brief Get current error covariance
     */
    const Eigen::MatrixXd& getCovariance() const { return P_; }

    /**
     * @brief Get state dimension
     */
    size_t getStateDim() const override { return n_; }

    /**
     * @brief Reset state estimate and covariance
     */
    void reset() override;

private:
    void resize();
    Eigen::MatrixXd numericalJacobianF(const Eigen::VectorXd& x,
                                       const Eigen::VectorXd& u,
                                       double dt) const;
    Eigen::MatrixXd numericalJacobianH(const Eigen::VectorXd& x) const;

    size_t n_{0};
    size_t m_{0};
    size_t p_{0};

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd W_;
    Eigen::MatrixXd V_;

    StateFunction f_;
    MeasurementFunction h_;
    StateJacobian F_;
    MeasurementJacobian H_;

    bool use_numdiff_{true};
};

} // namespace Control
