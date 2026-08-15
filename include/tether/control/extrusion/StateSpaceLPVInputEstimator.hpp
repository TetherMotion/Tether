/**
 * @file StateSpaceLPVInputEstimator.hpp
 * @brief LPV state-space input estimation with Tikhonov-regularized inversion.
 *
 * @details
 * Represents the LPV system in state-space form:
 *
 *   v[n+1] = A(p[n]) v[n] + B(p[n]) x[n]
 *   y[n]   = C(p[n]) v[n] + D(p[n]) x[n]
 *
 * where v[n] is the internal state vector (e.g., melt-zone pressure).
 *
 * Deconvolution is equivalent to input estimation.  For strictly proper
 * systems (D = 0, relative degree d = 1), the input at step n is recovered
 * by looking one step ahead at the target:
 *
 *   y_tgt[n+1] = C(p[n+1]) (A(p[n]) v[n] + B(p[n]) x[n])
 *
 * Solving for x[n]:
 *
 *   x_req[n] = [C(p[n+1]) B(p[n])]^{+} (y_tgt[n+1] - C(p[n+1]) A(p[n]) v[n])
 *
 * The matrix pseudo-inverse [C·B]^{+} is computed with Tikhonov
 * regularization to handle ill-conditioned or near-singular cases:
 *
 *   x_req = (M^T M + λ I)^{-1} M^T b
 *
 * where M = C(p[n+1]) B(p[n]) and b = y_tgt[n+1] - C(p[n+1]) A(p[n]) v[n].
 *
 * For SISO systems (scalar M), this reduces to:
 *   x_req = M · b / (M² + λ)
 *
 * The internal state v[n] is propagated in a feedforward simulation:
 *   v[n+1] = A(p[n]) v[n] + B(p[n]) x_req[n]
 *
 * This approach is best for embedded systems with matrix algebra support
 * (e.g., Eigen).  It is the most mathematically rigorous time-domain method.
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md
 */

#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>

namespace tether::control::extrusion {

/// @brief State-space model at a single operating point.
struct StateSpaceLPVModelPoint {
    double parameter = 0.0;
    Eigen::MatrixXd A;  ///< State transition matrix [stateDim × stateDim]
    Eigen::MatrixXd B;  ///< Input matrix [stateDim × inputDim]
    Eigen::MatrixXd C;  ///< Output matrix [outputDim × stateDim]
    Eigen::MatrixXd D;  ///< Feedthrough matrix [outputDim × inputDim] (usually 0)

    StateSpaceLPVModelPoint() = default;
    StateSpaceLPVModelPoint(double p, Eigen::MatrixXd a, Eigen::MatrixXd b,
                            Eigen::MatrixXd c, Eigen::MatrixXd d = {})
        : parameter(p), A(std::move(a)), B(std::move(b)),
          C(std::move(c)), D(std::move(d)) {}
};

/// @brief Parameters for the state-space LPV input estimator.
struct StateSpaceLPVParams {
    /// @brief Tikhonov regularization λ for the matrix pseudo-inverse.
    double lambda = 1e-8;
};

/// @brief LPV state-space input estimator.
///
/// Propagates the internal state v[n] in a feedforward simulation and
/// recovers the required input x[n] by inverting the one-step-ahead
/// output equation with Tikhonov regularization.
class StateSpaceLPVInputEstimator {
public:
    /// @brief Construct with system dimensions.
    /// @param stateDim Dimension of the state vector v.
    /// @param inputDim Dimension of the input x (1 for SISO).
    /// @param outputDim Dimension of the output y (1 for SISO).
    StateSpaceLPVInputEstimator(int stateDim, int inputDim = 1,
                                int outputDim = 1,
                                StateSpaceLPVParams params = {});

    /// @brief Add a state-space model at operating point p.
    void addModelPoint(const StateSpaceLPVModelPoint& point);

    /// @brief Process one step with one-step-ahead lookahead.
    /// @param yTargetNext y_tgt[n+1] (the lookahead target).
    /// @param pCurrent Current scheduling parameter p[n].
    /// @param pNext Next scheduling parameter p[n+1].
    /// @return x_req[n] (required input at step n).
    double process(double yTargetNext, double pCurrent, double pNext);

    /// @brief Process a full trajectory.
    /// @param yTarget Target trajectory y_tgt[0..N-1].
    /// @param p Scheduling parameter trajectory p[0..N-1].
    /// @return Required input x_req[0..N-1].
    std::vector<double> process(const std::vector<double>& yTarget,
                                const std::vector<double>& p);

    /// @brief Reset the internal state to zero.
    void reset();

    /// @return Current state vector v[n].
    const Eigen::VectorXd& state() const { return v_; }

    /// @return Number of model points in the LUT.
    size_t numModelPoints() const { return modelLut_.size(); }

    /// @brief Set the Tikhonov regularization parameter.
    void setLambda(double lambda) { params_.lambda = lambda; }

private:
    int stateDim_;
    int inputDim_;
    int outputDim_;
    StateSpaceLPVParams params_;
    std::map<double, StateSpaceLPVModelPoint> modelLut_;
    Eigen::VectorXd v_;  ///< current state vector

    /// @brief Interpolate state-space matrices at scheduling parameter p.
    StateSpaceLPVModelPoint interpolateModel(double p) const;

    /// @brief Tikhonov-regularized solve: x = (M^T M + λI)^{-1} M^T b.
    /// For SISO (scalar M), uses the scalar form x = M·b / (M² + λ).
    Eigen::VectorXd regularizedSolve(const Eigen::MatrixXd& M,
                                     const Eigen::VectorXd& b,
                                     double lambda) const;
};

} // namespace tether::control::extrusion
