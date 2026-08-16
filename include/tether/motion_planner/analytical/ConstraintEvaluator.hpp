/**
 * @file ConstraintEvaluator.hpp
 * @brief Computes feasible jerk (eta) bounds from NURBS geometry and limits.
 *
 * @details
 * The ConstraintEvaluator transforms kinematic constraints (per-axis
 * velocity, acceleration, jerk limits; path-level limits; centripetal
 * acceleration) into bounds on the control input eta = da/dt at a given
 * state (s, v, a).
 *
 * ## Mathematical Foundation
 *
 * Using the arc-length parameterization, the task-space quantities are:
 *
 *   qdot  = T * v                          (velocity)
 *   qddot = κ⃗ * v² + T * a                (acceleration)
 *   qddd  = j⃗ * v³ + 3*κ⃗*v*a + T * eta   (jerk)
 *
 * where T = dp/ds (unit tangent), κ⃗ = d²p/ds² (curvature vector),
 * j⃗ = d³p/ds³ (jounce vector), and eta = da/dt is the control input.
 *
 * ### Velocity Constraints
 *
 * Per-axis: |T_i * v| <= v_max_i  =>  v <= v_max_i / |T_i|
 * Path-level: v <= v_path_max
 * Feed rate: v <= v_feed
 * Centripetal: v² * κ <= a_cent_max  =>  v <= sqrt(a_cent_max / κ)
 *
 * These give a velocity limit v_lim(s) that is independent of (a, eta).
 *
 * ### Acceleration Constraints
 *
 * Per-axis: |κ⃗_i * v² + T_i * a| <= a_max_i
 *   If T_i > epsilon:  a ∈ [(−a_max_i − κ⃗_i*v²)/T_i, (a_max_i − κ⃗_i*v²)/T_i]
 *   If |T_i| < epsilon:  |κ⃗_i * v²| <= a_max_i  (constraint on v only)
 * Path-level: |a| <= a_path_max
 *
 * These give an acceleration bound [a_min, a_max] at (s, v).
 *
 * ### Jerk Constraints
 *
 * Per-axis: |j⃗_i * v³ + 3*κ⃗_i*v*a + T_i * eta| <= j_max_i
 *   Writing qddd_i = alpha_i * eta + beta_i where:
 *     alpha_i = T_i
 *     beta_i  = j⃗_i * v³ + 3*κ⃗_i * v * a
 *   If alpha_i > epsilon:
 *     eta ∈ [(−j_max_i − beta_i)/alpha_i, (j_max_i − beta_i)/alpha_i]
 *   If alpha_i < −epsilon: (flipped)
 *     eta ∈ [(j_max_i − beta_i)/alpha_i, (−j_max_i − beta_i)/alpha_i]
 *   If |alpha_i| < epsilon:  |beta_i| <= j_max_i  (constraint on v,a only)
 * Path-level: |eta| <= j_path_max
 *
 * The overall eta bounds are the intersection of all per-axis and
 * path-level jerk constraints.
 *
 * @see AnalyticalTypes.hpp for the EtaBounds and KinematicCoefficients types.
 * @see ../geometry/NurbsCurve.hpp for arcDerivatives (T, κ⃗, j⃗).
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "../VelocityProfile.hpp"
#include "../PathAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical {

/**
 * @brief Evaluates kinematic constraints to compute feasible eta bounds.
 *
 * The evaluator works with the existing PathAdapter (which wraps
 * PiecewiseNurbsPath) and KinematicLimits from the MotionPlanner namespace.
 *
 * It computes:
 * 1. Velocity limit v_lim(s) from curvature, feed rate, and per-axis limits
 * 2. Acceleration bounds [a_min, a_max] at (s, v)
 * 3. Jerk (eta) bounds [eta_min, eta_max] at (s, v, a)
 */
template<size_t Dim, typename T = double>
class ConstraintEvaluator {
public:
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;

    /**
     * @brief Construct with kinematic limits and feed rate.
     */
    ConstraintEvaluator(Limits limits, T feedRate)
        : limits_(std::move(limits))
        , feedRate_(feedRate) {}

    /**
     * @brief Compute the velocity limit v_lim at arc length s.
     *
     * The velocity is limited by:
     * - Feed rate
     * - Path-level max velocity
     * - Centripetal acceleration: v² * κ <= a_cent_max
     * - Per-axis velocity: |T_i * v| <= v_max_i
     *
     * @param s Arc length position
     * @param path The path to evaluate
     * @return Maximum feasible velocity at s
     */
    T velocityLimit(T s, const Path& path) const {
        T limit = feedRate_;
        limit = std::min(limit, limits_.path.maxPathVelocity);

        // Curvature (centripetal) limit
        T kappa = path.curvatureAtArcLength(s);
        if (kappa > MathConstants::EPSILON) {
            T curvatureLimit = std::sqrt(
                limits_.path.maxCentripetalAcceleration / kappa);
            limit = std::min(limit, curvatureLimit);
        }

        // Per-axis velocity limits
        auto eval = path.evaluateAtArcLength(s);
        limit = std::min(limit, limits_.maxVelocityForDirection(eval.tangent));

        return std::max(limit, T(0));
    }

    /**
     * @brief Compute the kinematic coefficients at (s, v, a).
     *
     * Evaluates the NURBS arc-length derivatives (T, κ⃗, j⃗) and computes
     * the alpha and beta coefficients for the jerk constraint.
     *
     * @param s Arc length position
     * @param v Current velocity ds/dt
     * @param a Current acceleration dv/dt
     * @param path The path to evaluate
     * @return Kinematic coefficients
     */
    KinematicCoefficients computeCoefficients(T s, T v, T a, const Path& path) const {
        KinematicCoefficients coeffs;

        // Get arc-length derivatives from the geometry core
        // We need order 3 for jounce (j⃗ = d³p/ds³)
        if (!path.hasInner()) {
            // Fallback: use the adapter's evaluateAtArcLength (order 2 only)
            auto eval = path.evaluateAtArcLength(s);
            const auto dim = Dim;
            coeffs.tangent.resize(dim);
            coeffs.curvature.resize(dim);
            coeffs.jounce.resize(dim, 0.0);
            for (size_t i = 0; i < dim; ++i) {
                coeffs.tangent[i] = static_cast<double>(eval.tangent[i]);
            }
            // curvature vector = curvature * normal
            T kappa = eval.curvature;
            coeffs.kappa = static_cast<double>(kappa);
            // We don't have the curvature vector directly from the adapter;
            // approximate: κ⃗ ≈ kappa * normal (if available)
            for (size_t i = 0; i < dim; ++i) {
                coeffs.curvature[i] = static_cast<double>(eval.normal[i]) *
                                      static_cast<double>(kappa);
            }
            // Jounce not available from adapter; set to zero
            // (this means jerk constraints will only use the T*eta term)
        } else {
            const auto& inner = path.inner();
            // Evaluate arc derivatives up to order 3
            tether::motion::ArcDerivatives derivs;
            try {
                derivs = inner.evaluate(static_cast<double>(s), 3);
            } catch (...) {
                // Fallback to order 2 if order 3 fails (degenerate)
                try {
                    derivs = inner.evaluate(static_cast<double>(s), 2);
                } catch (...) {
                    derivs = inner.evaluate(static_cast<double>(s), 0);
                }
            }

            const std::size_t dim = derivs.position.dim();
            coeffs.tangent.resize(dim);
            coeffs.curvature.resize(dim);
            coeffs.jounce.resize(dim, 0.0);

            for (std::size_t i = 0; i < dim; ++i) {
                coeffs.tangent[i] = derivs.tangent.unchecked(i);
                coeffs.curvature[i] = derivs.curvature.unchecked(i);
                if (derivs.jounce.dim() > i) {
                    coeffs.jounce[i] = derivs.jounce.unchecked(i);
                }
            }

            coeffs.kappa = derivs.curvature.norm();
            // Speed factor: g(u) = ||C'(u)||. In arc-length parameterization,
            // the tangent T = C'(u)/g(u) is unit, so g(u) = ||C'(u)||.
            // We can get it from the parametric derivative.
            auto loc = inner.locate(static_cast<double>(s));
            const auto& piece = inner.piece(loc.piece);
            double u = piece.invertLength(loc.localS);
            coeffs.speedFactor = piece.derivative(u, 1).norm();
        }

        // Compute alpha and beta for jerk constraint
        // qddd_i = alpha_i * eta + beta_i
        // alpha_i = T_i
        // beta_i  = j⃗_i * v³ + 3 * κ⃗_i * v * a
        const std::size_t dim = coeffs.tangent.size();
        coeffs.alpha_jerk.resize(dim);
        coeffs.beta_jerk.resize(dim);

        double v_d = static_cast<double>(v);
        double a_d = static_cast<double>(a);
        double v3 = v_d * v_d * v_d;
        double va = v_d * a_d;

        for (std::size_t i = 0; i < dim; ++i) {
            coeffs.alpha_jerk[i] = coeffs.tangent[i];
            coeffs.beta_jerk[i] = coeffs.jounce[i] * v3 +
                                  3.0 * coeffs.curvature[i] * va;
        }

        return coeffs;
    }

    /**
     * @brief Compute acceleration bounds [a_min, a_max] at (s, v).
     *
     * From per-axis acceleration constraints:
     *   |κ⃗_i * v² + T_i * a| <= a_max_i
     *
     * And path-level: |a| <= a_path_max
     *
     * @param s Arc length
     * @param v Current velocity
     * @param path The path
     * @return Pair (a_min, a_max)
     */
    std::pair<T, T> accelerationBounds(T s, T v, const Path& path) const {
        T a_min = -limits_.path.maxPathAcceleration;
        T a_max =  limits_.path.maxPathAcceleration;

        auto coeffs = computeCoefficients(s, v, T(0), path);
        double v2 = static_cast<double>(v) * static_cast<double>(v);

        const std::size_t dim = coeffs.tangent.size();
        for (std::size_t i = 0; i < dim && i < Dim; ++i) {
            double Ti = coeffs.tangent[i];
            double kappai_v2 = coeffs.curvature[i] * v2;

            double axMaxI = limits_.axis.maxAcceleration[i];

            if (std::abs(Ti) > MathConstants::EPSILON) {
                double a_lo = (-axMaxI - kappai_v2) / Ti;
                double a_hi = (axMaxI - kappai_v2) / Ti;
                if (a_lo > a_hi) std::swap(a_lo, a_hi);
                a_min = std::max(a_min, static_cast<T>(a_lo));
                a_max = std::min(a_max, static_cast<T>(a_hi));
            } else {
                // Tangent component ~ 0: constraint is |κ⃗_i * v²| <= a_max_i
                // This is a constraint on v, not on a. If violated, infeasible.
                if (std::abs(kappai_v2) > axMaxI) {
                    // Infeasible velocity — return empty interval
                    return {T(1), T(-1)};  // a_min > a_max signals infeasible
                }
            }
        }

        return {a_min, a_max};
    }

    /**
     * @brief Compute jerk (eta) bounds [eta_min, eta_max] at (s, v, a).
     *
     * From per-axis jerk constraints:
     *   |alpha_i * eta + beta_i| <= j_max_i
     *
     * And path-level: |eta| <= j_path_max
     *
     * @param s Arc length
     * @param v Current velocity
     * @param a Current acceleration
     * @param path The path
     * @return Eta bounds (feasible interval for eta)
     */
    EtaBounds etaBounds(T s, T v, T a, const Path& path) const {
        EtaBounds bounds;
        bounds.eta_min = -limits_.path.maxPathJerk;
        bounds.eta_max =  limits_.path.maxPathJerk;

        // If jerk limiting is not enabled, use large bounds
        if (!limits_.path.jerkLimitEnabled) {
            bounds.eta_min = -1e18;
            bounds.eta_max =  1e18;
        }

        auto coeffs = computeCoefficients(s, v, a, path);

        const std::size_t dim = coeffs.alpha_jerk.size();
        for (std::size_t i = 0; i < dim && i < Dim; ++i) {
            double alpha = coeffs.alpha_jerk[i];
            double beta = coeffs.beta_jerk[i];
            double jMaxI = limits_.axis.maxJerk[i];

            // Only apply per-axis jerk if enabled and positive
            if (!limits_.axis.jerkLimitEnabled || jMaxI <= 0) continue;

            if (std::abs(alpha) > MathConstants::EPSILON) {
                double eta_lo = (-jMaxI - beta) / alpha;
                double eta_hi = (jMaxI - beta) / alpha;
                if (eta_lo > eta_hi) std::swap(eta_lo, eta_hi);
                bounds.eta_min = std::max(bounds.eta_min, eta_lo);
                bounds.eta_max = std::min(bounds.eta_max, eta_hi);
            } else {
                // alpha ~ 0: constraint is |beta| <= j_max_i (on v, a only)
                // If violated, no eta can fix it — infeasible
                if (std::abs(beta) > jMaxI) {
                    bounds.eta_min = 1.0;
                    bounds.eta_max = -1.0;  // infeasible
                    return bounds;
                }
            }
        }

        return bounds;
    }

    /**
     * @brief Get the kinematic limits.
     */
    const Limits& limits() const { return limits_; }

    /**
     * @brief Get the feed rate.
     */
    T feedRate() const { return feedRate_; }

private:
    Limits limits_;
    T feedRate_;
};

} // namespace MotionPlanner::analytical
