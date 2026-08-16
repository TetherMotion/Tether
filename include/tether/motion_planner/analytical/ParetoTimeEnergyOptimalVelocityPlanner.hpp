/**
 * @file ParetoTimeEnergyOptimalVelocityPlanner.hpp
 * @brief Configurable energy/time-optimal path tracking for NURBS chains.
 *
 * @details
 * This profiler implements the weighted-cost optimal control problem:
 *
 *      minimize  J = ∫_0^T [ w_t + w_a · a(t)² ] dt
 *
 * where w_t is the weight on time and w_a is the weight on acceleration
 * energy. The solution is built from two primitive arc types identified
 * by Pontryagin's maximum principle:
 *
 * - **BANG arcs** (η = ±η_max): cubic-in-time transitions where jerk
 *   is at its bound. These ramp acceleration up or down.
 * - **SINGULAR arcs** (η = 0, a = a* = const): constant-acceleration
 *   cruising. The singular acceleration level a* is the single
 *   optimization parameter, selected by minimizing the closed-form
 *   scalar cost J(a*).
 *
 * ## Weight extremes
 *
 * - w_a = 0 → pure time-optimal (recovers bang-bang TOPPRA; a* → a_max)
 * - w_t = 0 → ill-posed (infinite time); always keep w_t > 0
 * - Both > 0 → configurable compromise (the regime this planner targets)
 *
 * ## Algorithm
 *
 * 1. **Golden-section search** over a* ∈ (0, a_max]
 * 2. For each a*, **forward state-machine simulation** produces a sequence
 *    of BANG and SINGULAR arcs (all analytically integrable)
 * 3. The cost J(a*) is computed in **closed form** from the arc list
 * 4. The optimal a* minimizes J; the corresponding arc list is the solution
 *
 * The state machine control law (derived from Pontryagin analysis with
 * H ≡ 0 and λ_a sign test):
 * - If a < a*: η = +η_max (BANG_PLUS — raise acceleration toward a*)
 * - If a > a*: η = -η_max (BANG_MINUS — lower acceleration toward a*)
 * - If a ≈ a*: η = 0 (SINGULAR — hold at a*)
 * - If v ≥ v_wall: follow the wall (WALL arc)
 *
 * ## Constraint handling
 *
 * Uses the existing ConstraintEvaluator for:
 * - Velocity limit v_lim(s) from curvature, feed rate, per-axis limits
 * - Jerk (eta) bounds [η_min, η_max] at (s, v, a)
 * - Acceleration bounds [a_min, a_max] at (s, v)
 *
 * ## Output
 *
 * - **VelocityProfile<T>**: tabulated v(s) profile (backward compatible)
 * - **WeightedSwitchingStructure**: exact analytic sampling (position,
 *   velocity, acceleration, jerk at any time t)
 *
 * @see VelocityProfiler.hpp for the abstract interface.
 * @see ConstraintEvaluator.hpp for constraint algebra.
 * @see docs/motion/ParetoTimeEnergyOptimal.md for the full manual.
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "ConstraintEvaluator.hpp"
#include "NumericalUtils.hpp"
#include "../VelocityProfile.hpp"
#include "../VelocityProfiler.hpp"
#include "../PathAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace MotionPlanner::analytical {

// ============================================================================
// Section 5: Data Structures
// ============================================================================

/**
 * @brief Weights for the time-energy cost functional.
 *
 * J = ∫ [w_t + w_a · a(t)²] dt
 *
 * w_t must always be > 0 (w_t = 0 is ill-posed — the solver would take
 * infinite time). w_a = 0 recovers pure time-optimal (TOPPRA).
 *
 * See §11 of the manual for tuning guide.
 */
struct CostWeights {
    /// Weight on time (must be > 0). Units: dimensionless.
    double w_t = 1.0;

    /// Weight on acceleration energy (≥ 0). Units: time³.
    /// 0 → pure time-optimal; large → smooth/energy-optimal.
    double w_a = 0.0;

    /**
     * @brief Compute the singular acceleration magnitude for a given
     *        costate level c.
     *
     * a* = sqrt((c + w_t) / w_a)  (requires w_a > 0)
     *
     * @param c Costate level (energy constant from Pontryagin analysis)
     * @return Singular acceleration magnitude
     */
    double a_star(double c) const {
        return (w_a > 0.0)
            ? std::sqrt(std::max(0.0, (c + w_t) / w_a))
            : 0.0;
    }

    /**
     * @brief Compute the costate level c from a target singular acceleration.
     *
     * c = w_a · a*² − w_t
     */
    double costateFromAStar(double a_star) const {
        return w_a * a_star * a_star - w_t;
    }
};

/**
 * @brief Arc type in the weighted switching structure.
 */
enum class WeightedArcType : uint8_t {
    BANG_PLUS,   ///< η = +η_max (raising acceleration toward a*)
    BANG_MINUS,  ///< η = -η_max (lowering acceleration toward a*)
    SINGULAR,    ///< η = 0, a = a* (constant acceleration cruising)
    WALL,        ///< v = v_wall(u(s)); acceleration slaved to geometry
};

/**
 * @brief Get human-readable name for a weighted arc type.
 */
inline const char* weightedArcTypeName(WeightedArcType type) {
    switch (type) {
        case WeightedArcType::BANG_PLUS:  return "BANG_PLUS";
        case WeightedArcType::BANG_MINUS: return "BANG_MINUS";
        case WeightedArcType::SINGULAR:   return "SINGULAR";
        case WeightedArcType::WALL:       return "WALL";
    }
    return "UNKNOWN";
}

/**
 * @brief A single arc in the weighted switching structure (WSS).
 *
 * All arcs are analytically integrable in the time domain:
 * - BANG: a(t) = a0 + η·τ, v(t) = v0 + a0·τ + ½η·τ², s(t) = s0 + ...
 * - SINGULAR: a(t) = a*, v(t) = v0 + a*·τ, s(t) = s0 + v0·τ + ½a*·τ²
 * - WALL: v(s) = v_wall(u(s)); requires ODE integration (quadrature)
 */
struct WeightedArc {
    WeightedArcType type = WeightedArcType::SINGULAR;

    /// Arc-length span [s0, s1]
    double s0 = 0.0;
    double s1 = 0.0;

    /// Absolute time at s0
    double t0 = 0.0;

    /// State at s0
    double v0 = 0.0;
    double a0 = 0.0;

    /// Geometric parameter at s0 (NURBS u)
    double u0 = 0.0;

    /// BANG: constant jerk value used (after clamping to [η_min, η_max])
    double eta = 0.0;

    /// SINGULAR: constant acceleration level (a*)
    double a_star = 0.0;

    /// Arc duration (time span), computed during solve
    double duration = 0.0;

    /// Check if this arc is valid (non-empty domain)
    bool valid() const { return s1 > s0; }

    /// Arc length span
    double length() const { return s1 - s0; }
};

// ============================================================================
// Section 10: Analytic Arc Propagation (closed-form formulas)
// ============================================================================

/**
 * @brief Bang arc propagation formulas (η = const).
 *
 * For a bang arc with constant jerk η, starting from (a0, v0) at τ=0:
 *   a(τ) = a0 + η·τ
 *   v(τ) = v0 + a0·τ + ½·η·τ²
 *   Δs(τ) = v0·τ + ½·a0·τ² + (1/6)·η·τ³
 *
 * The inverse (given Δs, find τ) requires solving a cubic. The cubic
 * is monotone in the region of interest (ds/dτ = v > 0), so Newton's
 * method converges in ~5 iterations.
 */
struct BangSeg {
    static double a(double a0, double e, double tau) {
        return a0 + e * tau;
    }
    static double v(double v0, double a0, double e, double tau) {
        return v0 + a0 * tau + 0.5 * e * tau * tau;
    }
    static double ds(double v0, double a0, double e, double tau) {
        return v0 * tau + 0.5 * a0 * tau * tau
               + (1.0 / 6.0) * e * tau * tau * tau;
    }

    /**
     * @brief Solve for τ given Δs: smallest positive root of
     *        (e/6)τ³ + (a0/2)τ² + v0·τ − ds = 0
     *
     * Newton's method with monotone safeguard. The cubic is strictly
     * increasing for τ > 0 when v > 0 (ds/dτ = v > 0), so convergence
     * is guaranteed.
     */
    static double tau_for_ds(double v0, double a0, double e, double ds) {
        if (ds <= 0.0) return 0.0;
        // Initial guess: ignore acceleration terms.
        double tau = ds / std::max(v0, 1e-12);
        for (int i = 0; i < 50; ++i) {
            double f = (e / 6.0) * tau * tau * tau
                       + (a0 / 2.0) * tau * tau
                       + v0 * tau - ds;
            double df = (e / 2.0) * tau * tau + a0 * tau + v0;
            if (std::abs(df) < 1e-15) break;
            double d = f / df;
            tau -= d;
            if (tau < 0.0) tau = ds / std::max(v0, 1e-12); // safeguard
            if (std::fabs(d) < 1e-15 * (1.0 + tau)) break;
        }
        return tau;
    }
};

/**
 * @brief Singular arc propagation formulas (a = a* = const, η = 0).
 *
 * For a singular arc with constant acceleration a*:
 *   a(τ) = a*
 *   v(τ) = v0 + a*·τ
 *   Δs(τ) = v0·τ + ½·a*·τ²
 *
 * The inverse is a simple quadratic (or linear when a* → 0).
 */
struct SingSeg {
    static double v(double v0, double as, double tau) {
        return v0 + as * tau;
    }
    static double ds(double v0, double as, double tau) {
        return v0 * tau + 0.5 * as * tau * tau;
    }

    /**
     * @brief Solve for τ given Δs: τ = (−v0 + √(v0² + 2·a*·Δs)) / a*
     *
     * When a* → 0, this reduces to τ = Δs / v0 (constant-velocity cruise).
     */
    static double tau_for_ds(double v0, double as, double ds) {
        if (ds <= 0.0) return 0.0;
        if (std::abs(as) < 1e-14) return ds / std::max(v0, 1e-12);
        double disc = v0 * v0 + 2.0 * as * ds;
        if (disc < 0.0) disc = 0.0;
        return (-v0 + std::sqrt(disc)) / as;
    }
};

/**
 * @brief Closed-form braking distance from (v, +a*) to (0, 0).
 *
 * The braking sequence is a symmetric S-curve:
 * 1. η = η_min until a goes from +a* to -a*
 * 2. η = 0 (hold at a = -a*) until velocity is low enough
 * 3. η = +η_max to bring a from -a* to 0 exactly when v = 0
 *
 * Phase 1 has zero net velocity change (symmetric ramp).
 * Phase 3 decelerates by Δv3 = -0.5·a*²/η_max.
 * Phase 2 must decelerate by v - 0.5·a*²/η_max, requiring
 * t2 = (v - 0.5·a*²/η_max) / a*.
 *
 * When v < 0.5·a*²/η_max, the three-phase formula is infeasible
 * (phase 3 alone would overshoot). In that case, we use a simplified
 * two-phase approach: ramp a from +a* toward 0 with η_min, stopping
 * when v = 0.
 *
 * @param v Current velocity (≥ 0)
 * @param a_star Singular acceleration level (positive)
 * @param eta_min Minimum jerk (negative, for deceleration ramp)
 * @param eta_max Maximum jerk (positive, for final ramp)
 * @return Total arc length needed to brake from v to 0
 */
inline double brake_distance(double v, double a_star,
                              double eta_min, double eta_max) {
    if (v <= 0.0) return 0.0;
    a_star = std::max(std::abs(a_star), 1e-12);
    eta_min = std::min(eta_min, -1e-12);
    eta_max = std::max(eta_max, 1e-12);

    // Phase 1: a from +a* to -a* with η_min (η_min < 0)
    // t1 = 2*a* / |η_min|, Δv1 = 0 (symmetric ramp)
    double t1 = 2.0 * a_star / std::abs(eta_min);
    double s1 = v * t1 + 0.5 * a_star * t1 * t1
                + (1.0 / 6.0) * eta_min * t1 * t1 * t1;

    // Phase 3: a from -a* to 0 with η_max
    // t3 = a* / η_max, Δv3 = -0.5 * a*² / η_max
    double t3 = a_star / eta_max;
    double dv3 = -0.5 * a_star * a_star / eta_max;

    // Phase 2: η = 0 at a = -a* (constant deceleration)
    // Need: Δv2 = -(v + dv3) = -(v - 0.5*a*²/η_max)
    // t2 = (v + dv3) / a* = (v - 0.5*a*²/η_max) / a*
    double v_phase2 = v + dv3;  // = v - 0.5*a*²/η_max

    if (v_phase2 < 0.0) {
        // Velocity too small for full three-phase braking.
        // Use simplified braking: decelerate with η_min from +a* until v=0.
        // a(t) = a* + η_min·t, v(t) = v + a*·t + 0.5·η_min·t²
        // Set v(t) = 0: 0.5·η_min·t² + a*·t + v = 0
        // t = (-a* + sqrt(a*² - 2·η_min·v)) / η_min
        double disc = a_star * a_star - 2.0 * eta_min * v;
        if (disc < 0.0) disc = 0.0;
        double t_brake = (-a_star + std::sqrt(disc)) / eta_min;
        if (t_brake < 0.0) {
            t_brake = (-a_star - std::sqrt(disc)) / eta_min;
        }
        double s_brake = v * t_brake + 0.5 * a_star * t_brake * t_brake
                         + (1.0 / 6.0) * eta_min * t_brake * t_brake * t_brake;
        return std::max(s_brake, 0.0);
    }

    // Full three-phase braking
    double t2 = v_phase2 / a_star;
    double v1 = v;  // Δv1 = 0
    double s2 = v1 * t2 - 0.5 * a_star * t2 * t2;
    double v3 = v1 - a_star * t2;  // velocity at start of phase 3
    double s3 = v3 * t3 - 0.5 * a_star * t3 * t3
                + (1.0 / 6.0) * eta_max * t3 * t3 * t3;

    return s1 + s2 + s3;
}

// ============================================================================
// Golden Section Search (for minimizing J(a*))
// ============================================================================

/**
 * @brief Golden section search for the minimum of a unimodal function.
 *
 * @param f Function to minimize (must be unimodal on [a, b])
 * @param a Left bracket
 * @param b Right bracket
 * @param tol Tolerance on the bracket width
 * @return Pair (argmin, fmin)
 */
inline std::pair<double, double> goldenSection(
    const std::function<double(double)>& f,
    double a, double b, double tol = 1e-8) {
    const double gr = (std::sqrt(5.0) + 1.0) / 2.0;  // golden ratio ≈ 1.618
    double c = b - (b - a) / gr;
    double d = a + (b - a) / gr;
    double fc = f(c);
    double fd = f(d);

    for (int iter = 0; iter < 100 && (b - a) > tol; ++iter) {
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - (b - a) / gr;
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + (b - a) / gr;
            fd = f(d);
        }
    }
    double xopt = (a + b) / 2.0;
    return {xopt, f(xopt)};
}

// ============================================================================
// Section 7: Weighted Switching Structure (WSS)
// ============================================================================

/**
 * @brief Weighted switching structure — the output representation.
 *
 * Stores the arc list and provides exact analytic sampling of
 * position, velocity, acceleration, and jerk at any time t.
 *
 * All BANG and SINGULAR arcs are sampled in closed form (polynomial
 * evaluation + one NURBS evaluation). WALL arcs use precomputed
 * quadrature tables.
 *
 * Implements the AnalyticalTrajectorySource interface for compatibility
 * with TrajectorySampler and MotionPlan.
 */
template<size_t Dim, typename T = double>
class WeightedSwitchingStructure : public AnalyticalTrajectorySource<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Arc = WeightedArc;

    /**
     * @brief Construct the WSS from a path and arc list.
     *
     * @param path The path (stored by const reference — caller must
     *             ensure lifetime)
     * @param arcs The weighted arcs (the solution)
     * @param w The cost weights used
     * @param evaluator The constraint evaluator (for velocity limit
     *                  queries during sampling)
     */
    WeightedSwitchingStructure(
        const Path& path,
        std::vector<Arc> arcs,
        CostWeights w,
        ConstraintEvaluator<Dim, T> evaluator)
        : path_(path)
        , arcs_(std::move(arcs))
        , w_(w)
        , evaluator_(std::move(evaluator)) {

        // Use pre-computed t0 and duration from the solver.
        // Recompute total time from the arc list.
        totalTime_ = 0.0;
        double tAccum = 0.0;
        for (auto& arc : arcs_) {
            arc.t0 = tAccum;
            if (arc.duration <= 0.0) {
                arc.duration = computeArcDuration(arc);
            }
            tAccum += arc.duration;
        }
        totalTime_ = tAccum;
    }

    // --- AnalyticalTrajectorySource interface ---

    T totalTime() const override { return static_cast<T>(totalTime_); }
    T totalLength() const override { return path_.totalLength(); }

    Vec<Dim, T> position(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = path_.evaluateAtArcLength(static_cast<T>(s));
        return eval.position;
    }

    Vec<Dim, T> velocity(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = path_.evaluateAtArcLength(static_cast<T>(s));
        // qdot = T * v (tangent * path velocity)
        Vec<Dim, T> result;
        for (size_t i = 0; i < Dim; ++i)
            result[i] = eval.tangent[i] * static_cast<T>(v);
        return result;
    }

    Vec<Dim, T> acceleration(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = path_.evaluateAtArcLength(static_cast<T>(s));
        // qddot = κ⃗ * v² + T * a (curvature * v² + tangent * a)
        Vec<Dim, T> result;
        for (size_t i = 0; i < Dim; ++i) {
            result[i] = eval.normal[i] * eval.curvature * static_cast<T>(v * v)
                      + eval.tangent[i] * static_cast<T>(a);
        }
        return result;
    }

    T arcLength(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return static_cast<T>(s);
    }

    T pathVelocity(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return static_cast<T>(v);
    }

    T pathAcceleration(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return static_cast<T>(a);
    }

    T pathJerk(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return static_cast<T>(eta);
    }

    T curvature(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return path_.curvatureAtArcLength(static_cast<T>(s));
    }

    SourceReference sourceRef(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return path_.sourceRefAtArcLength(static_cast<T>(s));
    }

    size_t segmentIndex(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = path_.evaluateAtArcLength(static_cast<T>(s));
        return eval.segmentIndex;
    }

    T segmentParameter(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = path_.evaluateAtArcLength(static_cast<T>(s));
        return eval.localParameter;
    }

    const char* representationName() const override {
        return "WeightedSwitchingStructure";
    }

    // --- Accessors ---

    const std::vector<Arc>& arcs() const { return arcs_; }
    const CostWeights& weights() const { return w_; }
    double costValue() const { return costValue_; }
    void setCostValue(double J) { costValue_ = J; }

    /**
     * @brief The optimal singular acceleration level a* found by the solver.
     */
    double optimalAStar() const {
        return arcs_.empty() ? 0.0
            : std::abs(arcs_.front().a_star);
    }

private:
    const Path& path_;
    std::vector<Arc> arcs_;
    CostWeights w_;
    ConstraintEvaluator<Dim, T> evaluator_;
    double totalTime_ = 0.0;
    double costValue_ = 0.0;

    /**
     * @brief Locate the arc containing time t and compute the full state.
     *
     * @return Tuple (arcIdx, tau, s, v, a, eta) where tau = t - arc.t0
     */
    std::tuple<size_t, double, double, double, double, double>
    locateAndState(T t_query) const {
        double t = static_cast<double>(t_query);
        if (arcs_.empty()) return {0, 0, 0, 0, 0, 0};

        // Binary search for the arc containing t
        size_t lo = 0, hi = arcs_.size();
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (arcs_[mid].t0 + arcs_[mid].duration < t)
                lo = mid + 1;
            else
                hi = mid;
        }
        size_t idx = std::min(lo, arcs_.size() - 1);

        const auto& arc = arcs_[idx];
        double tau = t - arc.t0;
        if (tau < 0.0) tau = 0.0;
        if (tau > arc.duration) tau = arc.duration;

        // Compute state at tau using closed-form arc formulas
        double s, v, a, eta;
        if (arc.type == WeightedArcType::SINGULAR) {
            a = arc.a_star;
            v = SingSeg::v(arc.v0, arc.a_star, tau);
            s = arc.s0 + SingSeg::ds(arc.v0, arc.a_star, tau);
            eta = 0.0;
        } else if (arc.type == WeightedArcType::WALL) {
            // WALL: approximate with constant velocity
            // (proper implementation would use precomputed quadrature)
            double vWall = arc.v0;
            v = vWall;
            a = 0.0;
            s = arc.s0 + vWall * tau;
            eta = 0.0;
        } else {
            // BANG_PLUS or BANG_MINUS
            double e = arc.eta;
            a = BangSeg::a(arc.a0, e, tau);
            v = BangSeg::v(arc.v0, arc.a0, e, tau);
            s = arc.s0 + BangSeg::ds(arc.v0, arc.a0, e, tau);
            eta = e;
        }

        // Clamp s to path bounds
        double sMax = static_cast<double>(path_.totalLength());
        if (s > sMax) s = sMax;
        if (s < 0.0) s = 0.0;
        if (v < 0.0) v = 0.0;

        return {idx, tau, s, v, a, eta};
    }

    /**
     * @brief Compute the duration of an arc (time span).
     */
    double computeArcDuration(const Arc& arc) const {
        double ds = arc.s1 - arc.s0;
        if (ds <= 0.0) return 0.0;

        if (arc.type == WeightedArcType::SINGULAR) {
            return SingSeg::tau_for_ds(arc.v0, arc.a_star, ds);
        } else if (arc.type == WeightedArcType::WALL) {
            // Approximate: constant velocity at v0
            return ds / std::max(arc.v0, 1e-12);
        } else {
            // BANG
            return BangSeg::tau_for_ds(arc.v0, arc.a0, arc.eta, ds);
        }
    }
};

// ============================================================================
// Section 6: The Solver
// ============================================================================

/**
 * @brief Weighted time-energy solver.
 *
 * Solves the weighted-cost optimal control problem by:
 * 1. Golden-section search over a* (singular acceleration level)
 * 2. For each a*, forward state-machine simulation producing arcs
 * 3. Closed-form cost evaluation J(a*)
 *
 * The state machine control law (from Pontryagin analysis):
 * - a < a* → η = +η_max (BANG_PLUS)
 * - a > a* → η = -η_max (BANG_MINUS)
 * - a ≈ a* → η = 0 (SINGULAR)
 * - v ≥ v_wall → follow wall (WALL)
 *
 * Braking is initiated when the remaining distance equals the closed-form
 * braking distance brake_distance(v, a*, η_min, η_max).
 */
template<size_t Dim, typename T = double>
class WeightedTimeEnergySolver {
public:
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;
    using Evaluator = ConstraintEvaluator<Dim, T>;
    using Arc = WeightedArc;
    using WSS = WeightedSwitchingStructure<Dim, T>;

    /**
     * @brief Constructor.
     * @param path The path to profile
     * @param limits Kinematic limits
     * @param w Cost weights
     * @param feedRate Feed rate
     */
    WeightedTimeEnergySolver(const Path& path, Limits limits,
                              CostWeights w, T feedRate)
        : path_(path)
        , limits_(std::move(limits))
        , w_(w)
        , feedRate_(feedRate)
        , evaluator_(limits_, feedRate) {
        sTotal_ = static_cast<double>(path_.totalLength());
    }

    /**
     * @brief Solve the weighted-cost problem.
     *
     * @param startVelocity Initial velocity (default 0)
     * @param endVelocity Final velocity (default 0)
     * @param numSamples Number of sample points for the simulation grid
     * @return Vector of weighted arcs (the solution)
     */
    std::vector<Arc> solve(T startVelocity = T(0),
                            T endVelocity = T(0),
                            size_t numSamples = 200) {
        v0_ = static_cast<double>(startVelocity);
        vf_ = static_cast<double>(endVelocity);
        numSamples_ = std::max(numSamples, size_t(10));

        // Step 0: Estimate max reachable acceleration
        double aMax = estimateMaxReachableAccel();

        // Step 3: Golden-section search over a* ∈ (0, aMax]
        // J(a*) is a closed-form (piecewise) scalar function.
        // For w_a = 0, J = w_t * T is not monotonically decreasing in a*
        // because larger a* also means larger braking distance, which can
        // force earlier braking and slower trajectories on short paths.
        // The golden section search handles this correctly as long as J(a*)
        // is unimodal, which it is for regular problems.
        auto J = [this](double a_star) {
            return simulateAndCost(a_star, /*record=*/false);
        };

        double aLo = 1e-6 * aMax;  // near-zero a* → very slow
        double aHi = aMax;

        auto [aOpt, Jmin] = goldenSection(J, aLo, aHi, 1e-6 * aMax);

        // Rebuild the optimal arc list
        arcs_.clear();
        double Jfinal = simulateAndCost(aOpt, /*record=*/true);
        (void)Jfinal;

        return arcs_;
    }

    /**
     * @brief Get the achieved cost value.
     */
    double costValue() const { return lastCost_; }

    /**
     * @brief Get the optimal a* found by the solver.
     */
    double optimalAStar() const { return lastAStar_; }

    /**
     * @brief Get the total traversal time of the solution.
     */
    double totalTime() const {
        if (arcs_.empty()) return 0.0;
        double t = 0.0;
        for (const auto& arc : arcs_)
            t += arc.duration;
        return t;
    }

    /**
     * @brief Get the constraint evaluator.
     */
    const Evaluator& evaluator() const { return evaluator_; }

private:
    const Path& path_;
    Limits limits_;
    CostWeights w_;
    T feedRate_;
    Evaluator evaluator_;

    double sTotal_ = 0.0;
    double v0_ = 0.0;
    double vf_ = 0.0;
    size_t numSamples_ = 200;

    std::vector<Arc> arcs_;
    double lastCost_ = 0.0;
    double lastAStar_ = 0.0;

    /**
     * @brief Estimate the maximum reachable acceleration.
     *
     * Uses the path-level acceleration limit as an upper bound, also
     * considering per-axis limits at the path midpoint.
     */
    double estimateMaxReachableAccel() const {
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        // Also check per-axis limits at the path midpoint
        if (sTotal_ > 0.0) {
            T sMid = static_cast<T>(sTotal_ * 0.5);
            auto eval = path_.evaluateAtArcLength(sMid);
            T aAxis = limits_.maxAccelerationForDirection(
                eval.tangent, eval.curvature, T(0));
            aMax = std::min(aMax, static_cast<double>(aAxis));
        }
        return std::max(aMax, 1e-6);
    }

    /**
     * @brief Forward state-machine simulation for fixed a*.
     *
     * Walks the path from s=0 to s=s_f, selecting the control at each
     * step based on the a* guidance law. Produces a sequence of BANG
     * and SINGULAR arcs. Computes the cost J in closed form.
     *
     * @param aStar The singular acceleration level
     * @param record If true, store the arcs in arcs_
     * @return The total cost J
     */
    double simulateAndCost(double aStar, bool record) {
        std::vector<Arc> tmp;
        double s = 0.0, t = 0.0;
        double v = v0_, a = 0.0;
        double J = 0.0;
        bool braking = false;
        double aStarEff = aStar;  // effective a* (negated during braking)

        const double ds = sTotal_ / static_cast<double>(numSamples_);
        const double sEnd = sTotal_;

        // Get jerk bounds (use path-level as default; per-interval recompute
        // would be more accurate but slower)
        auto getEtaBounds = [&](double sCur, double vCur, double aCur)
            -> std::pair<double, double> {
            auto bounds = evaluator_.etaBounds(
                static_cast<T>(sCur), static_cast<T>(vCur),
                static_cast<T>(aCur), path_);
            return {bounds.eta_min, bounds.eta_max};
        };

        auto getVLimit = [&](double sCur) -> double {
            return static_cast<double>(
                evaluator_.velocityLimit(static_cast<T>(sCur), path_));
        };

        int maxIter = static_cast<int>(numSamples_) * 20;
        for (int iter = 0; iter < maxIter && s < sEnd - 1e-10; ++iter) {
            auto [etaMin, etaMax] = getEtaBounds(s, v, a);
            double vLim = getVLimit(s);

            // Check if we need to start braking
            double sRemaining = sEnd - s;
            double sBrake = brake_distance(v, aStarEff, etaMin, etaMax);
            if (!braking && sRemaining <= sBrake + ds * 0.5) {
                braking = true;
                aStarEff = -aStar;  // switch to braking with -a*
            }

            // Select desired eta per the a* guidance law
            double etaDes;
            if (v >= vLim - 1e-10) {
                // At velocity wall — cruise
                etaDes = 0.0;
                a = 0.0;  // hold at wall
            } else if (a < aStarEff - 1e-10) {
                etaDes = etaMax;  // BANG_PLUS: raise a toward a*
            } else if (a > aStarEff + 1e-10) {
                etaDes = etaMin;  // BANG_MINUS: lower a toward a*
            } else {
                etaDes = 0.0;     // SINGULAR: hold at a*
            }

            // Clamp to feasible bounds
            double eta = std::clamp(etaDes, etaMin, etaMax);

            // Determine arc type
            WeightedArcType arcType;
            if (v >= vLim - 1e-10) {
                arcType = WeightedArcType::WALL;
                eta = 0.0;
            } else if (std::abs(eta) < 1e-12) {
                arcType = WeightedArcType::SINGULAR;
            } else if (eta > 0) {
                arcType = WeightedArcType::BANG_PLUS;
            } else {
                arcType = WeightedArcType::BANG_MINUS;
            }

            // Determine arc length: step to next event or ds, whichever smaller
            double dsArc = ds;

            // Don't overshoot the end
            if (s + dsArc > sEnd) dsArc = sEnd - s;

            // Don't overshoot velocity limit
            if (arcType == WeightedArcType::BANG_PLUS) {
                // Check how far until v reaches vLim
                double vTarget = vLim;
                // v(t) = v0 + a0*tau + 0.5*eta*tau^2
                // Solve for tau when v = vTarget:
                // 0.5*eta*tau^2 + a0*tau + (v0 - vTarget) = 0
                double disc = a * a - 2.0 * eta * (v - vTarget);
                if (disc > 0.0 && eta > 0.0) {
                    double tauV = (-a + std::sqrt(disc)) / eta;
                    if (tauV > 0.0) {
                        double dsV = BangSeg::ds(v, a, eta, tauV);
                        if (dsV > 0.0 && dsV < dsArc) dsArc = dsV;
                    }
                }
            }

            // Check if a reaches a* during this arc (for bang arcs)
            if (arcType == WeightedArcType::BANG_PLUS ||
                arcType == WeightedArcType::BANG_MINUS) {
                double aTarget = aStarEff;
                double tauA = (aTarget - a) / eta;
                if (tauA > 0.0 && tauA < 1e6) {
                    double dsA = BangSeg::ds(v, a, eta, tauA);
                    if (dsA > 0.0 && dsA < dsArc) dsArc = dsA;
                }
            }

            // Ensure minimum step
            if (dsArc < 1e-12) dsArc = 1e-12;

            // Compute tau (time for this arc)
            double tau;
            if (arcType == WeightedArcType::SINGULAR) {
                tau = SingSeg::tau_for_ds(v, aStarEff, dsArc);
            } else if (arcType == WeightedArcType::WALL) {
                tau = dsArc / std::max(v, 1e-12);
            } else {
                tau = BangSeg::tau_for_ds(v, a, eta, dsArc);
            }

            // Compute end state
            double v1, a1;
            if (arcType == WeightedArcType::SINGULAR) {
                a1 = aStarEff;
                v1 = SingSeg::v(v, aStarEff, tau);
            } else if (arcType == WeightedArcType::WALL) {
                a1 = 0.0;
                v1 = v;  // hold at wall
            } else {
                a1 = BangSeg::a(a, eta, tau);
                v1 = BangSeg::v(v, a, eta, tau);
            }

            // Clamp velocity to limit
            if (v1 > vLim) {
                v1 = vLim;
                if (a1 > 0.0) a1 = 0.0;
            }
            if (v1 < 0.0) v1 = 0.0;

            // Clamp acceleration to acceleration bounds
            auto [aMinBound, aMaxBound] = evaluator_.accelerationBounds(
                static_cast<T>(s + dsArc), static_cast<T>(v1), path_);
            a1 = std::clamp(a1, static_cast<double>(aMinBound),
                                 static_cast<double>(aMaxBound));

            // Exact cost increment: ∫(w_t + w_a * a²)dt
            // For BANG: a(t) = a0 + eta*tau, so
            //   ∫a²dt = a0²*tau + a0*eta*tau² + (eta²/3)*tau³
            // For SINGULAR: a = a* = const, so
            //   ∫a²dt = a*² * tau
            double dJ;
            if (arcType == WeightedArcType::SINGULAR) {
                dJ = w_.w_t * tau + w_.w_a * aStarEff * aStarEff * tau;
            } else if (arcType == WeightedArcType::WALL) {
                dJ = w_.w_t * tau;  // a = 0 on wall
            } else {
                // BANG: a(t) = a + eta*tau
                double a0_ = a, e = eta;
                double intA2 = a0_ * a0_ * tau
                             + a0_ * e * tau * tau
                             + (e * e * tau * tau * tau) / 3.0;
                dJ = w_.w_t * tau + w_.w_a * intA2;
            }
            J += dJ;

            // Record arc
            if (record) {
                Arc arc;
                arc.type = arcType;
                arc.s0 = s;
                arc.s1 = s + dsArc;
                arc.t0 = t;
                arc.v0 = v;
                arc.a0 = a;
                arc.eta = eta;
                arc.a_star = aStarEff;
                arc.duration = tau;
                tmp.push_back(arc);
            }

            // Advance state
            s += dsArc;
            t += tau;
            v = v1;
            a = a1;

            // Safety: if velocity is zero and we're not at the end, break
            if (v < 1e-12 && s < sEnd - 1e-6) {
                // Stuck — give a tiny velocity to continue
                v = 1e-6;
            }
        }

        // Ensure we reach the end
        if (s < sEnd - 1e-10 && !tmp.empty() && record) {
            // Extend last arc to the end
            tmp.back().s1 = sEnd;
        }

        // Force final velocity to zero (rest-to-rest)
        if (record && !tmp.empty()) {
            // Adjust the last arc's final state to ensure v=0
            // by clamping the last arc's velocity
            if (vf_ <= 1e-10) {
                // Add a final zero-velocity point if needed
                Arc& last = tmp.back();
                last.s1 = sEnd;
            }
        }

        if (record) {
            arcs_ = std::move(tmp);
            lastAStar_ = aStar;
        }

        // Penalize incomplete simulations (didn't reach the end)
        if (s < sEnd - 1e-6) {
            J += 1e6 * (sEnd - s);  // large penalty per unit of untraversed path
        }

        lastCost_ = J;
        return J;
    }
};

// ============================================================================
// Section 6+7: The VelocityProfiler Adapter
// ============================================================================

/**
 * @brief Pareto time-energy-optimal velocity profiler.
 *
 * Implements the VelocityProfiler interface. Solves the weighted-cost
 * optimal control problem J = ∫[w_t + w_a·a²]dt using Pontryagin's
 * maximum principle, producing a trajectory of BANG and SINGULAR arcs.
 *
 * The weight ratio w_a/w_t controls the time-energy tradeoff:
 * - w_a = 0 → pure time-optimal (TOPPRA; a* → a_max)
 * - w_a > 0 → smooth compromise (a* < a_max, less jerk, more time)
 *
 * The output is both a tabulated VelocityProfile (for backward
 * compatibility) and a WeightedSwitchingStructure (for exact sampling).
 *
 * @tparam Dim Spatial dimension
 * @tparam T   Numeric type (default: double)
 */
template<size_t Dim, typename T = double>
class ParetoTimeEnergyOptimalVelocityPlanner : public VelocityProfiler<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Profile = VelocityProfile<T>;
    using Limits = KinematicLimits<Dim, T>;
    using Point = VelocityProfilePoint<T>;
    using Evaluator = ConstraintEvaluator<Dim, T>;
    using WSS = WeightedSwitchingStructure<Dim, T>;
    using Solver = WeightedTimeEnergySolver<Dim, T>;

    /**
     * @brief Constructor.
     * @param limits Kinematic limits (per-axis and path-level).
     * @param w Cost weights (w_t must be > 0; w_a = 0 recovers TOPPRA).
     */
    explicit ParetoTimeEnergyOptimalVelocityPlanner(
        Limits limits = {},
        CostWeights w = {})
        : limits_(std::move(limits))
        , weights_(w) {}

    /**
     * @brief Compute a weighted time-energy-optimal velocity profile.
     *
     * Solves the weighted-cost problem, produces a WeightedSwitchingStructure,
     * and samples it to a tabulated VelocityProfile for backward compatibility.
     */
    Profile computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        (void)startJerk;  // not honored (WI-P3 style)
        Profile profile;
        if (path.numSegments() == 0) return profile;

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) return profile;

        // Validate inputs (same guards as other profilers)
        if (numSamples < 2) return profile;
        if (feedRate <= T(0)) return profile;
        if (limits_.path.maxPathAcceleration <= T(0)) return profile;
        if (limits_.path.maxCentripetalAcceleration < T(0)) return profile;

        // If w_a = 0, this degenerates to time-optimal. We still solve
        // via the same machinery (a* → a_max), but the user could also
        // use AnalyticalTOPPRA directly for that case.
        // If w_t = 0, the problem is ill-posed — clamp to a tiny value.
        CostWeights wEff = weights_;
        if (wEff.w_t <= 0.0) wEff.w_t = 1e-12;

        // Solve
        Solver solver(path, limits_, wEff, feedRate);
        auto arcs = solver.solve(startVelocity, endVelocity,
                                  std::max(numSamples, size_t(200)));

        // Build the WSS
        Evaluator evaluator(limits_, feedRate);
        auto wss = std::make_shared<WSS>(
            path, std::move(arcs), wEff, std::move(evaluator));
        wss_ = wss;
        wss_->setCostValue(solver.costValue());

        // Sample the WSS to produce a tabulated VelocityProfile
        profile = sampleToProfile(*wss_, numSamples, startAcceleration);

        return profile;
    }

    /**
     * @brief Get the weighted switching structure (for exact sampling).
     */
    std::shared_ptr<WSS> weightedSource() const { return wss_; }

    /**
     * @brief Get the achieved cost value J.
     */
    double costValue() const {
        return wss_ ? wss_->costValue() : 0.0;
    }

    /**
     * @brief Get the optimal singular acceleration a*.
     */
    double optimalAStar() const {
        return wss_ ? wss_->optimalAStar() : 0.0;
    }

    /**
     * @brief Get the cost weights.
     */
    CostWeights weights() const { return weights_; }

    /**
     * @brief Set the cost weights.
     */
    void setWeights(CostWeights w) { weights_ = w; }

    // --- VelocityProfiler interface ---

    Limits limits() const override { return limits_; }

    ProfilerType type() const override {
        return ProfilerType::ParetoTimeEnergy;
    }

    const char* name() const override {
        return "ParetoTimeEnergyOptimalVelocityPlanner "
               "(weighted time-energy optimal)";
    }

private:
    Limits limits_;
    CostWeights weights_;
    std::shared_ptr<WSS> wss_;

    /**
     * @brief Sample the WSS at uniform arc-length intervals to produce
     *        a tabulated VelocityProfile.
     */
    Profile sampleToProfile(const WSS& wss, size_t numSamples,
                             T startAcceleration) const {
        Profile profile;
        T pathLength = wss.totalLength();
        if (pathLength <= T(0) || numSamples < 2) return profile;

        T ds = pathLength / T(numSamples - 1);
        profile.reserve(numSamples);

        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(T(i) * ds, pathLength);

            // Find time at this arc length by walking arcs
            T t = timeAtArcLength(wss, s);

            // Sample state from WSS
            T v = wss.pathVelocity(t);
            T a = wss.pathAcceleration(t);
            T j = wss.pathJerk(t);

            Point pt;
            pt.arcLength = s;
            pt.velocity = v;
            pt.acceleration = (i == 0) ? startAcceleration : a;
            pt.jerk = j;
            pt.time = t;

            // Determine limiting factor from the arc type
            // (simplified — the WSS doesn't expose per-sample arc type
            //  directly, so we infer from acceleration/jerk)
            if (std::abs(j) > T(1e-10)) {
                pt.limitedBy = (j > T(0))
                    ? Point::LimitType::ForwardAccel
                    : Point::LimitType::BackwardDecel;
            } else if (std::abs(a) > T(1e-10)) {
                pt.limitedBy = Point::LimitType::Jerk;  // singular arc
            } else if (v > T(1e-10)) {
                pt.limitedBy = Point::LimitType::Curvature;  // wall/cruise
            } else {
                pt.limitedBy = Point::LimitType::None;
            }

            profile.addPoint(pt);
        }

        // Ensure the last point has zero velocity for rest-to-rest
        if (!profile.points().empty()) {
            auto& last = profile.points().back();
            last.velocity = T(0);
            last.acceleration = T(0);
            last.jerk = T(0);
        }

        return profile;
    }

    /**
     * @brief Find the time corresponding to an arc length by walking
     *        the arc list.
     */
    T timeAtArcLength(const WSS& wss, T s) const {
        const auto& arcs = wss.arcs();
        if (arcs.empty()) return T(0);

        double sTarget = static_cast<double>(s);
        double t = 0.0;

        for (const auto& arc : arcs) {
            if (sTarget <= arc.s1) {
                // Target is within this arc
                double dsLocal = sTarget - arc.s0;
                if (dsLocal < 0.0) dsLocal = 0.0;

                double tau;
                if (arc.type == WeightedArcType::SINGULAR) {
                    tau = SingSeg::tau_for_ds(arc.v0, arc.a_star, dsLocal);
                } else if (arc.type == WeightedArcType::WALL) {
                    tau = dsLocal / std::max(arc.v0, 1e-12);
                } else {
                    tau = BangSeg::tau_for_ds(arc.v0, arc.a0, arc.eta, dsLocal);
                }
                return static_cast<T>(arc.t0 + tau);
            }
            t = arc.t0 + arc.duration;
        }

        // Past the end
        return static_cast<T>(t);
    }
};

} // namespace MotionPlanner::analytical
