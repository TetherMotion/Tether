/**
 * @file AnalyticalStateSpaceLPV.hpp
 * @brief Analytical state-space LPV input estimation on WSS arcs.
 *
 * @details
 * The continuous-time LPV state-space model:
 *
 *   dv/dt = A(p(t))·v(t) + B(p(t))·x(t)
 *   y(t)  = C(p(t))·v(t) + D(p(t))·x(t)
 *
 * For strictly proper systems (D = 0), differentiating the output:
 *
 *   ẏ(t) = C(p)·(A(p)·v + B(p)·x)
 *
 * Solving for x(t):
 *
 *   x(t) = [C(p)·B(p)]⁺·(ẏ(t) - C(p)·A(p)·v(t))
 *
 * with Tikhonov regularization on [CB]⁺.
 *
 * Within each arc, using arc-averaged p̄ (so A, B, C are constant):
 *
 *   dv/dt = F·v + G·ẏ(t)
 *
 * where F = A - B·M⁺·C·A, G = B·M⁺, M = C·B.
 *
 * This is a linear ODE with polynomial forcing ẏ(t), solved via matrix
 * exponential + polynomial integral formulas.
 *
 * The input is then:
 *   x(t) = M⁺·(ẏ(t) - C·A·v(t))
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §7
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief State-space model at a single operating point.
 */
struct AnalyticalStateSpaceModelPoint {
    double parameter = 0.0;
    Eigen::MatrixXd A;  ///< [stateDim × stateDim]
    Eigen::MatrixXd B;  ///< [stateDim × inputDim]
    Eigen::MatrixXd C;  ///< [outputDim × stateDim]
};

/**
 * @brief Parameters for analytical state-space LPV.
 */
struct AnalyticalStateSpaceLPVParams {
    /// Tikhonov regularization λ for matrix pseudo-inverse
    double lambda = 1e-8;

    /// State dimension
    int stateDim = 2;

    /// Input dimension (1 for SISO)
    int inputDim = 1;

    /// Output dimension (1 for SISO)
    int outputDim = 1;

    /// If true, y(t) = extruder position; if false, velocity
    bool usePosition = false;
};

/**
 * @brief Analytical state-space LPV input estimator.
 */
template<size_t Dim, typename T = double>
class AnalyticalStateSpaceLPV {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;

    /**
     * @brief Construct from trajectory and parameters.
     */
    AnalyticalStateSpaceLPV(const Traj& traj,
                             AnalyticalStateSpaceLPVParams params)
        : traj_(&traj), params_(params) {
        state_ = Eigen::VectorXd::Zero(params.stateDim);
    }

    /**
     * @brief Add a state-space model at operating point p.
     */
    void addModelPoint(const AnalyticalStateSpaceModelPoint& point) {
        modelLut_[point.parameter] = point;
    }

    /**
     * @brief Compute the required input x(t) at time t.
     */
    double inputAtTime(double t) const {
        if (modelLut_.empty()) return 0.0;
        const auto& arcs = traj_->arcs();
        if (arcs.empty()) return 0.0;

        size_t idx = traj_->findArc(t);
        const auto& a = arcs[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);

        // Get interpolated model at arc-averaged velocity
        double pBar = a.avgPathVelocity();
        auto model = interpolateModel(pBar);

        // M = C·B, M⁺ = M / (M² + λ) for SISO
        Eigen::MatrixXd M = model.C * model.B;
        double Mscalar = M(0, 0);
        double Mreg = Mscalar / (Mscalar * Mscalar + params_.lambda);

        // ẏ(t) — derivative of the target
        double ydot;
        if (params_.usePosition) {
            ydot = a.extruderVelocity(tau);
        } else {
            ydot = a.extrusionRatio * a.pathAcceleration(tau);
        }

        // y(t) — the target
        double y;
        if (params_.usePosition) {
            y = traj_->extruderPositionAtTime(t);
        } else {
            y = a.extruderVelocity(tau);
        }

        // Propagate state v(t) via the precomputed solution
        Eigen::VectorXd v = propagateState(idx, tau, a, model, Mreg);

        // x(t) = M⁺·(ẏ(t) - C·A·v(t))
        double CAv = (model.C * model.A * v)(0, 0);
        return Mreg * (ydot - CAv);
    }

    /**
     * @brief Compute the required input at multiple time points.
     */
    std::vector<double> inputSeries(const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(inputAtTime(t));
        return result;
    }

    /**
     * @brief Compute the adjusted extruder position at time t.
     */
    double adjustedExtruderPosition(double t) const {
        return traj_->extruderPositionAtTime(t) + inputAtTime(t);
    }

    /**
     * @brief Compute the adjusted extruder position at multiple times.
     */
    std::vector<double> adjustedExtruderPositionSeries(
        const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(adjustedExtruderPosition(t));
        return result;
    }

    /// Number of model points
    size_t numModelPoints() const { return modelLut_.size(); }

    /// Parameters
    const AnalyticalStateSpaceLPVParams& params() const { return params_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

private:
    const Traj* traj_;
    AnalyticalStateSpaceLPVParams params_;
    std::map<double, AnalyticalStateSpaceModelPoint> modelLut_;
    mutable Eigen::VectorXd state_;

    AnalyticalStateSpaceModelPoint interpolateModel(double p) const {
        if (modelLut_.empty()) return {};
        if (modelLut_.size() == 1) return modelLut_.begin()->second;

        auto upper = modelLut_.lower_bound(p);
        if (upper == modelLut_.begin()) return upper->second;
        if (upper == modelLut_.end()) return std::prev(upper)->second;

        auto lower = std::prev(upper);
        double p0 = lower->first, p1 = upper->first;
        double frac = (p - p0) / (p1 - p0);

        AnalyticalStateSpaceModelPoint result;
        result.parameter = p;
        result.A = lower->second.A + frac * (upper->second.A - lower->second.A);
        result.B = lower->second.B + frac * (upper->second.B - lower->second.B);
        result.C = lower->second.C + frac * (upper->second.C - lower->second.C);
        return result;
    }

    /**
     * @brief Propagate the state v through arcs up to (idx, tau).
     *
     * Uses matrix exponential per arc with polynomial forcing.
     */
    Eigen::VectorXd propagateState(size_t targetIdx, double targetTau,
                                    const ExtrusionArc& targetArc,
                                    const AnalyticalStateSpaceModelPoint& model,
                                    double Mreg) const {
        // F = A - B·M⁺·C·A
        Eigen::MatrixXd F = model.A - model.B * Mreg * model.C * model.A;
        // G = B·M⁺
        Eigen::MatrixXd G = model.B * Mreg;

        // Start from zero state and propagate through all arcs
        // (For efficiency, we could cache, but for correctness we recompute)
        Eigen::VectorXd v = Eigen::VectorXd::Zero(params_.stateDim);
        const auto& arcs = traj_->arcs();

        for (size_t i = 0; i <= targetIdx && i < arcs.size(); ++i) {
            const auto& a = arcs[i];
            double dt = (i == targetIdx) ? targetTau : a.duration;
            if (dt <= 0.0) continue;

            // Forcing: ẏ(τ) = α_e · d/dτ[v(τ)]
            // For y = velocity: ẏ = α_e · a(τ) = α_e · (c1 + 2·c2·τ + 3·c3·τ²)
            // For y = position: ẏ = α_e · v(τ) = α_e · (c0 + c1·τ + c2·τ² + c3·τ³)
            double alphaE = a.extrusionRatio;

            // v(dt) = exp(F·dt)·v0 + ∫₀^dt exp(F·(dt-s))·G·ẏ(s) ds
            Eigen::MatrixXd expFdt = (F * dt).exp();

            // Compute forcing integral using the recursive formula
            int n = params_.stateDim;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
            Eigen::MatrixXd Finv = F.colPivHouseholderQr().solve(I);

            // J_k = ∫₀^dt exp(Fu)·u^k du
            // J_0 = F⁻¹·(exp(F·dt) - I)
            // J_k = F⁻¹·(exp(F·dt)·dt^k - k·J_{k-1})
            std::vector<Eigen::MatrixXd> J(4);
            J[0] = Finv * (expFdt - I);
            J[1] = Finv * (expFdt * dt - J[0]);
            J[2] = Finv * (expFdt * dt * dt - 2.0 * J[1]);
            J[3] = Finv * (expFdt * dt * dt * dt - 3.0 * J[2]);

            // I_k = ∫₀^dt exp(F·(dt-s))·s^k ds = Σ C(k,j) dt^{k-j} (-1)^j J_j
            std::vector<Eigen::MatrixXd> Iint(4);
            for (int k = 0; k <= 3; ++k) {
                Iint[k] = Eigen::MatrixXd::Zero(n, n);
                for (int j = 0; j <= k; ++j) {
                    double binom = 1.0;
                    for (int l = 0; l < j; ++l)
                        binom *= static_cast<double>(k - l) / (l + 1.0);
                    Iint[k] += binom * std::pow(dt, k - j)
                               * (j % 2 == 0 ? 1.0 : -1.0) * J[j];
                }
            }

            // Forcing integral: ∫ exp(F·(dt-s))·G·ẏ(s) ds
            // ẏ(s) = α_e · (f0 + f1·s + f2·s² + f3·s³)
            Eigen::VectorXd forcing = Eigen::VectorXd::Zero(n);
            if (params_.usePosition) {
                // ẏ = α_e · v(s) = α_e · (c0 + c1·s + c2·s² + c3·s³)
                forcing = alphaE * (
                    a.c0 * (Iint[0] * G)
                    + a.c1 * (Iint[1] * G)
                    + a.c2 * (Iint[2] * G)
                    + a.c3 * (Iint[3] * G)
                );
            } else {
                // ẏ = α_e · a(s) = α_e · (c1 + 2·c2·s + 3·c3·s²)
                forcing = alphaE * (
                    a.c1 * (Iint[0] * G)
                    + 2.0 * a.c2 * (Iint[1] * G)
                    + 3.0 * a.c3 * (Iint[2] * G)
                );
            }

            v = expFdt * v + forcing;
        }

        return v;
    }
};

} // namespace MotionPlanner::analytical::extrusion
