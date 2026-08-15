/**
 * @file StateSpaceLpvInputEstimator.cpp
 * @brief LPV state-space input estimation implementation.
 */

#include "tether/control/extrusion/StateSpaceLpvInputEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

StateSpaceLpvInputEstimator::StateSpaceLpvInputEstimator(
    int stateDim, int inputDim, int outputDim, StateSpaceLpvParams params)
    : stateDim_(stateDim), inputDim_(inputDim), outputDim_(outputDim),
      params_(std::move(params)), v_(Eigen::VectorXd::Zero(stateDim)) {}

void StateSpaceLpvInputEstimator::addModelPoint(
    const StateSpaceLpvModelPoint& point) {
    modelLut_[point.parameter] = point;
}

double StateSpaceLpvInputEstimator::process(
    double yTargetNext, double pCurrent, double pNext) {
    if (modelLut_.empty()) return 0.0;

    // Interpolate models at p[n] and p[n+1].
    auto modelCurrent = interpolateModel(pCurrent);
    auto modelNext = interpolateModel(pNext);

    // Compute the one-step-ahead inverse:
    //   x_req[n] = [C(p[n+1]) B(p[n])]^{+} (y_tgt[n+1] - C(p[n+1]) A(p[n]) v[n])
    //
    // M = C(p[n+1]) * B(p[n])   [outputDim × inputDim]
    // b = y_tgt[n+1] - C(p[n+1]) * A(p[n]) * v[n]   [outputDim × 1]

    const Eigen::MatrixXd& A = modelCurrent.A;
    const Eigen::MatrixXd& B = modelCurrent.B;
    const Eigen::MatrixXd& Cn1 = modelNext.C;

    // M = C(p[n+1]) * B(p[n])
    Eigen::MatrixXd M = Cn1 * B;  // [outputDim × inputDim]

    // predicted output without input: C(p[n+1]) * A(p[n]) * v[n]
    Eigen::VectorXd b = Eigen::VectorXd::Zero(outputDim_);
    if (outputDim_ == 1) {
        b(0) = yTargetNext - (Cn1 * A * v_)(0);
    } else {
        // For MIMO, yTargetNext would be a vector; here we handle SISO.
        b(0) = yTargetNext - (Cn1 * A * v_)(0);
    }

    // Tikhonov-regularized solve: x = (M^T M + λI)^{-1} M^T b
    Eigen::VectorXd xReq = regularizedSolve(M, b, params_.lambda);

    // Propagate state: v[n+1] = A(p[n]) v[n] + B(p[n]) x_req[n]
    v_ = A * v_ + B * xReq;

    // Return scalar input for SISO.
    return (inputDim_ == 1) ? xReq(0) : xReq(0);
}

std::vector<double> StateSpaceLpvInputEstimator::process(
    const std::vector<double>& yTarget, const std::vector<double>& p) {
    if (yTarget.empty() || yTarget.size() != p.size()) return {};
    reset();

    const int N = static_cast<int>(yTarget.size());
    std::vector<double> xReq(N, 0.0);

    for (int n = 0; n < N; ++n) {
        // Lookahead: y_tgt[n+1].  For the last sample, use y_tgt[N-1].
        const double yNext = (n + 1 < N) ? yTarget[n + 1] : yTarget[n];
        const double pNext = (n + 1 < N) ? p[n + 1] : p[n];
        xReq[n] = process(yNext, p[n], pNext);
    }
    return xReq;
}

void StateSpaceLpvInputEstimator::reset() {
    v_ = Eigen::VectorXd::Zero(stateDim_);
}

StateSpaceLpvModelPoint StateSpaceLpvInputEstimator::interpolateModel(
    double p) const {
    if (modelLut_.empty()) {
        return StateSpaceLpvModelPoint{};
    }

    auto it = modelLut_.find(p);
    if (it != modelLut_.end()) return it->second;

    auto upper = modelLut_.lower_bound(p);
    if (upper == modelLut_.begin()) return upper->second;
    if (upper == modelLut_.end()) return std::prev(upper)->second;

    auto lower = std::prev(upper);
    const double p0 = lower->first;
    const double p1 = upper->first;
    const double t = (p - p0) / (p1 - p0);

    StateSpaceLpvModelPoint result;
    result.parameter = p;
    result.A = lower->second.A + t * (upper->second.A - lower->second.A);
    result.B = lower->second.B + t * (upper->second.B - lower->second.B);
    result.C = lower->second.C + t * (upper->second.C - lower->second.C);
    result.D = lower->second.D + t * (upper->second.D - lower->second.D);
    return result;
}

Eigen::VectorXd StateSpaceLpvInputEstimator::regularizedSolve(
    const Eigen::MatrixXd& M, const Eigen::VectorXd& b, double lambda) const {
    // Tikhonov-regularized least squares: x = (M^T M + λI)^{-1} M^T b
    //
    // For SISO (M is 1×1 scalar m): x = m·b / (m² + λ)
    // For MIMO: use normal equations with regularization.

    if (M.rows() == 1 && M.cols() == 1) {
        // SISO scalar case.
        const double m = M(0, 0);
        const double bVal = b(0);
        Eigen::VectorXd x(1);
        x(0) = m * bVal / (m * m + lambda);
        return x;
    }

    // General case: x = (M^T M + λI)^{-1} M^T b
    const Eigen::MatrixXd MtM = M.transpose() * M;
    const Eigen::MatrixXd reg = MtM + lambda * Eigen::MatrixXd::Identity(
        MtM.rows(), MtM.cols());
    const Eigen::VectorXd Mtb = M.transpose() * b;
    return reg.ldlt().solve(Mtb);
}

} // namespace tether::control::extrusion
