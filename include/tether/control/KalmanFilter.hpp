/**
 * @file KalmanFilter.hpp
 * @brief Eigen-based linear Kalman filter state estimator
 *
 * Discrete-time Kalman filter using Eigen matrix types. The filter assumes
 * the system matrices A, B, C are already discretized:
 *
 *   x[k+1] = A*x[k] + B*u[k] + w[k]   (w ~ N(0, W))
 *   y[k]   = C*x[k] + v[k]            (v ~ N(0, V))
 */

#pragma once

#include "control/ControllerBase.hpp"
#include <Eigen/Dense>
#include <functional>

namespace tether::control {

/**
 * @brief Discrete-time linear Kalman filter
 *
 * Estimates the state of an LTI system from noisy measurements. Internally
 * uses Eigen::MatrixXd / Eigen::VectorXd for all matrix operations.
 */
class KalmanFilter : public StateEstimator {
public:
    KalmanFilter() = default;

    /**
     * @brief Construct with dimensions
     * @param n State dimension
     * @param m Input dimension
     * @param p Output dimension
     */
    KalmanFilter(size_t n, size_t m, size_t p);

    /**
     * @brief Set discrete-time system matrices
     * @param A State transition matrix (n×n)
     * @param B Input matrix (n×m)
     * @param C Output matrix (p×n)
     */
    void setSystemMatrices(const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& C);

    /**
     * @brief Set process and measurement noise covariances
     * @param W Process noise covariance (n×n, positive semi-definite)
     * @param V Measurement noise covariance (p×p, positive definite)
     */
    void setNoiseCovariances(const Eigen::MatrixXd& W,
                             const Eigen::MatrixXd& V);

    /**
     * @brief Set the initial state estimate and covariance
     * @param x0 Initial state (n)
     * @param P0 Initial error covariance (n×n), optional
     */
    void setInitialState(const Eigen::VectorXd& x0,
                         const Eigen::MatrixXd& P0 = Eigen::MatrixXd());

    /**
     * @brief Set Kalman gain directly (bypasses computeGain)
     * @param L Kalman gain matrix (n×p)
     */
    void setKalmanGain(const Eigen::MatrixXd& L);

    /**
     * @brief Compute steady-state Kalman gain via discrete algebraic Riccati equation
     * @return true if the iterative solver converged
     */
    bool computeGain();

    /**
     * @brief Prediction step
     * @param u Control input (m)
     * @return Predicted state estimate
     */
    Eigen::VectorXd predict(const Eigen::VectorXd& u);

    /**
     * @brief Update step
     * @param y Measurement (p)
     * @return Updated state estimate
     */
    Eigen::VectorXd update(const Eigen::VectorXd& y);

    /**
     * @brief Combined prediction + update step
     * @param y Measurement (p)
     * @param u Control input (m)
     * @return Updated state estimate
     */
    Eigen::VectorXd estimate(const Eigen::VectorXd& y,
                             const Eigen::VectorXd& u);

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
     * @brief Get computed (or set) Kalman gain
     */
    const Eigen::MatrixXd& getGain() const { return L_; }

    /**
     * @brief Get state dimension
     */
    size_t getStateDim() const override { return n_; }

    /**
     * @brief Reset state estimate and covariance
     */
    void reset() override;

private:
    bool solveDiscreteRiccati();
    void resize();

    size_t n_{0};
    size_t m_{0};
    size_t p_{0};

    Eigen::MatrixXd A_; ///< State transition matrix
    Eigen::MatrixXd B_; ///< Input matrix
    Eigen::MatrixXd C_; ///< Output matrix
    Eigen::MatrixXd W_; ///< Process noise covariance
    Eigen::MatrixXd V_; ///< Measurement noise covariance
    Eigen::MatrixXd P_; ///< Error covariance
    Eigen::MatrixXd L_; ///< Kalman gain
    Eigen::VectorXd x_; ///< State estimate

    bool gain_computed_{false};
};

} // namespace tether::control
