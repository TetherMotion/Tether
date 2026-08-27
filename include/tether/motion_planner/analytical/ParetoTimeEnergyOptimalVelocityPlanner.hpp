/**
 * @file ParetoTimeEnergyOptimalVelocityPlanner.hpp
 * @brief Snapspace time-energy-optimal path tracking for NURBS chains.
 *
 * @details
 * This profiler implements the weighted-cost optimal control problem
 * in **snapspace** (4th-order dynamics):
 *
 *      minimize  J = ∫_0^T [ w_t + w_j · j(t)² + w_a · a(t)² ] dt
 *
 * where w_t is the weight on time, w_j is the weight on jerk energy,
 * and w_a is the weight on acceleration energy.
 *
 * The state vector is (s, v, a, j) — 4D. The control input is **snap**
 * σ = dj/dt (the derivative of jerk). The solution is built from two
 * primitive arc types identified by Pontryagin's maximum principle:
 *
 * - **SNAP arcs** (σ = ±σ_max): quartic-in-time transitions where snap
 *   is at its bound. These ramp jerk up or down.
 * - **SINGULAR arcs** (σ = 0, j = j* = const): constant-jerk cruising.
 *   The singular jerk level j* is the single optimization parameter,
 *   selected by minimizing the closed-form scalar cost J(j*).
 *
 * ## Weight extremes
 *
 * - w_j = 0, w_a = 0 → pure time-optimal (j* → j_max)
 * - w_t = 0 → ill-posed (infinite time); always keep w_t > 0
 * - Both > 0 → configurable compromise (smooth, energy-optimal)
 *
 * ## Algorithm
 *
 * 1. Sample the path envelope and derive one conservative global bound for
 *    velocity, acceleration, and jerk.
 * 2. Construct exact symmetric SNAP/SINGULAR acceleration and braking
 *    pulses for a candidate smoothness scale.
 * 3. Search a deterministic finite family of scales and select the feasible
 *    candidate with minimum closed-form weighted cost.
 *
 * The trajectory stored in the WSS is exactly the one costed and sampled;
 * no output-time constraint clamping, penalty acceptance, or heuristic
 * backward braking factor is used.
 *
 * ## Boundary conditions
 *
 * Rest-to-rest-to-rest: v(0)=0, a(0)=0, j(0)=0, v(T)=0, a(T)=0, j(T)=0.
 *
 * ## Constraint handling
 *
 * Uses the existing ConstraintEvaluator for:
 * - Velocity limit v_lim(s) from curvature, feed rate, per-axis limits
 * - Acceleration bounds [a_min, a_max] at (s, v)
 * - Jerk bounds [j_min, j_max] at (s, v, a)
 * - Path snap limit σ_max from KinematicLimits
 *
 * Per-axis snap limits are explicitly unsupported until the geometry layer
 * provides fourth arc-length derivatives; requesting them returns no profile
 * and an explanatory diagnostic rather than a false feasibility claim.
 *
 * ## Output
 *
 * - **VelocityProfile<T>**: tabulated v(s) profile (backward compatible)
 * - **WeightedSwitchingStructure**: exact analytic sampling (position,
 *   velocity, acceleration, jerk at any time t)
 *
 * @see VelocityProfiler.hpp for the abstract interface.
 * @see ConstraintEvaluator.hpp for constraint algebra.
 */

#pragma once

#include "AnalyticalTypes.hpp"
#include "ConstraintEvaluator.hpp"
#include "NumericalUtils.hpp"
#include "AnalyticalTOPPRA.hpp"
#include "AnalyticalSSRVelocityProfile.hpp"
#include "../VelocityProfile.hpp"
#include "../VelocityProfiler.hpp"
#include "../PathAdapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace MotionPlanner::analytical {

// ============================================================================
// Section 5: Data Structures
// ============================================================================

/**
 * @brief Weights for the snapspace time-energy cost functional.
 *
 * J = ∫ [w_t + w_j · j(t)² + w_a · a(t)²] dt
 *
 * w_t must always be > 0 (w_t = 0 is ill-posed — the solver would take
 * infinite time). w_j = 0 and w_a = 0 recovers pure time-optimal.
 *
 */
struct CostWeights {
    /// Weight on time (must be > 0). Units: dimensionless.
    double w_t = 1.0;

    /// Weight on jerk energy (≥ 0). Units: time⁵.
    /// 0 → no jerk penalty; large → very smooth (low jerk).
    double w_j = 0.0;

    /// Weight on acceleration energy (≥ 0). Units: time³.
    /// 0 → no acceleration penalty; large → low peak acceleration.
    double w_a = 0.0;

    /**
     * @brief Compute the singular jerk magnitude for a given
     *        costate level c.
     *
     * j* = sqrt((c + w_t) / w_j)  (requires w_j > 0)
     *
     * @param c Costate level (energy constant from Pontryagin analysis)
     * @return Singular jerk magnitude
     */
    double j_star(double c) const {
        return (w_j > 0.0)
            ? std::sqrt(std::max(0.0, (c + w_t) / w_j))
            : 0.0;
    }

    /**
     * @brief Compute the costate level c from a target singular jerk.
     *
     * c = w_j · j*² - w_t
     */
    double costateFromJStar(double j_star) const {
        return w_j * j_star * j_star - w_t;
    }

    // --- Backward compatibility aliases (deprecated) ---

    /// @deprecated Use j_star() instead.
    double a_star(double c) const { return j_star(c); }

    /// @deprecated Use costateFromJStar() instead.
    double costateFromAStar(double a_star) const {
        return costateFromJStar(a_star);
    }
};

/**
 * @brief Arc type in the weighted switching structure (snapspace).
 */
enum class WeightedArcType : uint8_t {
    SNAP_PLUS,    ///< σ = +σ_max (raising jerk toward j*)
    SNAP_MINUS,   ///< σ = -σ_max (lowering jerk toward j*)
    SINGULAR,     ///< σ = 0, j = j* (constant jerk cruising)
    WALL,         ///< v = v_wall(u(s)); acceleration slaved to geometry
    DWELL,        ///< v = 0, a = 0, j = 0, s = const (G4 dwell pause)
};

/**
 * @brief Get human-readable name for a weighted arc type.
 */
inline const char* weightedArcTypeName(WeightedArcType type) {
    switch (type) {
        case WeightedArcType::SNAP_PLUS:    return "SNAP_PLUS";
        case WeightedArcType::SNAP_MINUS:   return "SNAP_MINUS";
        case WeightedArcType::SINGULAR:     return "SINGULAR";
        case WeightedArcType::WALL:         return "WALL";
        case WeightedArcType::DWELL:        return "DWELL";
    }
    return "UNKNOWN";
}

/**
 * @brief A single arc in the weighted switching structure (WSS).
 *
 * All arcs are analytically integrable in the time domain (snapspace):
 * - SNAP: j(t) = j0 + σ·τ, a(t) = a0 + j0·τ + ½σ·τ²,
 *         v(t) = v0 + a0·τ + ½j0·τ² + (1/6)σ·τ³,
 *         Δs = v0·τ + ½a0·τ² + (1/6)j0·τ³ + (1/24)σ·τ⁴
 * - SINGULAR: j(t) = j*, a(t) = a0 + j*·τ,
 *             v(t) = v0 + a0·τ + ½j*·τ²,
 *             Δs = v0·τ + ½a0·τ² + (1/6)j*·τ³
 * - WALL: v(s) = v_wall(u(s)); requires ODE integration (quadrature)
 */
struct WeightedArc {
    WeightedArcType type = WeightedArcType::SINGULAR;

    /// Arc-length span [s0, s1]
    double s0 = 0.0;
    double s1 = 0.0;

    /// Absolute time at s0
    double t0 = 0.0;

    /// State at s0 (4D: velocity, acceleration, jerk)
    double v0 = 0.0;
    double a0 = 0.0;
    double j0 = 0.0;

    /// Geometric parameter at s0 (NURBS u)
    double u0 = 0.0;

    /// SNAP: constant snap value used (σ = dj/dt)
    double sigma = 0.0;

    /// SINGULAR: constant jerk level (j*)
    double j_star = 0.0;

    /// Arc duration (time span), computed during solve
    double duration = 0.0;

    /// State at s1, propagated from the stored polynomial. Keeping both
    /// endpoints makes continuity independently auditable and ensures that a
    /// consumer never has to re-run a potentially ambiguous inverse solve.
    double v1 = 0.0;
    double a1 = 0.0;
    double j1 = 0.0;

    /// Bit mask describing which constraints supplied this arc's global
    /// certified-at-grid limits: velocity=1, acceleration=2, jerk=4,
    /// snap=8. It is diagnostic only; it never changes the trajectory.
    uint8_t activeConstraints = 0;

    /// Check if this arc is valid (non-empty domain)
    bool valid() const { return s1 > s0; }

    /// Arc length span
    double length() const { return s1 - s0; }

    // --- Backward compatibility accessors (deprecated) ---

    /// @deprecated Use sigma instead. Returns snap as "eta" for old code.
    double eta = 0.0;

    /// @deprecated Use j_star instead. Returns singular jerk as "a_star".
    double a_star = 0.0;
};

// ============================================================================
// Section 10: Analytic Arc Propagation (closed-form formulas)
// ============================================================================

/**
 * @brief Result of a single forward pass simulation.
 *
 * The forward pass is a standalone, reusable method that takes a
 * pre-computed v_lim profile and produces a deterministic arc sequence.
 * It does NOT depend on any backward-pass internals.
 */
struct ForwardPassResult {
    /// Produced arc sequence
    std::vector<WeightedArc> arcs;

    /// Total cost J = ∫(w_t + w_j·j² + w_a·a²)dt
    double cost = 0.0;

    /// Final arc length reached
    double finalS = 0.0;

    /// Final velocity
    double finalV = 0.0;

    /// Final acceleration
    double finalA = 0.0;

    /// Final jerk
    double finalJ = 0.0;

    /// Total traversal time
    double totalTime = 0.0;

    /// Whether the pass reached sEnd with v ≈ vf
    bool feasible = false;

    /// If not feasible, human-readable reason
    std::string failureReason;
};

/** @brief Exact scalar snap-space state at a WSS boundary. */
struct WeightedState {
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
};

/**
 * @brief Singular-Jerk arc propagation formulas (j = j* = const, σ = 0).
 *
 * For a singular arc with constant jerk j*:
 *   j(τ) = j*
 *   a(τ) = a0 + j*·τ
 *   v(τ) = v0 + a0·τ + ½·j*·τ²
 *   Δs(τ) = v0·τ + ½·a0·τ² + (1/6)·j*·τ³
 *
 * The inverse (given Δs, find τ) requires solving a cubic.
 */
struct SingularJSeg {
    static double a(double a0, double j_star, double tau) {
        return a0 + j_star * tau;
    }
    static double v(double v0, double a0, double j_star, double tau) {
        return v0 + a0 * tau + 0.5 * j_star * tau * tau;
    }
    static double ds(double v0, double a0, double j_star, double tau) {
        return v0 * tau + 0.5 * a0 * tau * tau
               + (1.0 / 6.0) * j_star * tau * tau * tau;
    }

    /**
     * @brief Solve for τ given Δs: smallest positive root of
     *        (j_star/6)τ³ + (a0/2)τ² + v0·τ - ds = 0
     *
    * Safeguarded Newton-bisection hybrid. Returns NaN when `ds` cannot be
    * reached while the arc remains forward-monotone.
     */
    static double tau_for_ds(double v0, double a0, double j_star, double ds) {
        if (ds <= 0.0) return 0.0;

        auto f = [&](double t) {
            return v0 * t + 0.5 * a0 * t * t
                   + (j_star / 6.0) * t * t * t;
        };

        // Find bracket by doubling
        double lo = 0.0;
        double hi = std::max(ds / std::max(v0, 1e-3), 1e-6);
        const double kMaxHi = 1e6;
        while (f(hi) < ds) {
            if (v(v0, a0, j_star, hi) <= 0.0 || hi >= kMaxHi) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            hi *= 2.0;
        }

        double tau = 0.5 * (lo + hi);
        for (int i = 0; i < 100; ++i) {
            double val = f(tau) - ds;
            double der = v0 + a0 * tau + 0.5 * j_star * tau * tau;
            if (val > 0.0) { hi = tau; } else { lo = tau; }
            double next = (std::abs(der) > 1e-14) ? tau - val / der
                                                  : 0.5 * (lo + hi);
            if (next <= lo || next >= hi) next = 0.5 * (lo + hi);
            tau = next;
            if (hi - lo < 1e-13 * (1.0 + tau)) break;
        }

        return tau;
    }
};

/**
 * @brief Snap arc propagation formulas (σ = const, snapspace).
 *
 * For a snap arc with constant snap σ, starting from (j0, a0, v0) at τ=0:
 *   j(τ) = j0 + σ·τ
 *   a(τ) = a0 + j0·τ + ½·σ·τ²
 *   v(τ) = v0 + a0·τ + ½·j0·τ² + (1/6)·σ·τ³
 *   Δs(τ) = v0·τ + ½·a0·τ² + (1/6)·j0·τ³ + (1/24)·σ·τ⁴
 *
 * The inverse (given Δs, find τ) requires solving a quartic. The quartic
 * is monotone in the region of interest (ds/dτ = v > 0), so Newton's
 * method converges quickly.
 */
struct SnapSeg {
    static double j(double j0, double sigma, double tau) {
        return j0 + sigma * tau;
    }
    static double a(double a0, double j0, double sigma, double tau) {
        return a0 + j0 * tau + 0.5 * sigma * tau * tau;
    }
    static double v(double v0, double a0, double j0, double sigma, double tau) {
        return v0 + a0 * tau + 0.5 * j0 * tau * tau
               + (1.0 / 6.0) * sigma * tau * tau * tau;
    }
    static double ds(double v0, double a0, double j0, double sigma, double tau) {
        return v0 * tau + 0.5 * a0 * tau * tau
               + (1.0 / 6.0) * j0 * tau * tau * tau
               + (1.0 / 24.0) * sigma * tau * tau * tau * tau;
    }

    /// Velocity derivative (for Newton's method): dv/dτ = a(τ)
    static double dv_dt(double a0, double j0, double sigma, double tau) {
        return a0 + j0 * tau + 0.5 * sigma * tau * tau;
    }

    /**
     * @brief Solve for τ given Δs: smallest positive root of
     *        (σ/24)τ⁴ + (j0/6)τ³ + (a0/2)τ² + v0·τ - ds = 0
     *
     * Safeguarded Newton-bisection hybrid. The quartic is strictly
     * increasing while v > 0, so a bracket [0, τ_hi] can be established
    * by doubling. Returns NaN when `ds` cannot be reached while the arc
    * remains forward-monotone.
     */
    static double tau_for_ds(double v0, double a0, double j0, double sigma,
                              double ds) {
        if (ds <= 0.0) return 0.0;

        auto f = [&](double t) {
            return v0 * t + 0.5 * a0 * t * t
                   + (j0 / 6.0) * t * t * t
                   + (sigma / 24.0) * t * t * t * t;
        };
        auto fprime = [&](double t) {
            return v0 + a0 * t + 0.5 * j0 * t * t
                   + (sigma / 6.0) * t * t * t;
        };

        // Find bracket by doubling
        double lo = 0.0;
        double hi = std::max(ds / std::max(v0, 1e-3), 1e-6);
        const double kMaxHi = 1e6;
        while (f(hi) < ds) {
            if (fprime(hi) <= 0.0 || hi >= kMaxHi) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            hi *= 2.0;
        }

        // Newton-bisection hybrid
        double tau = 0.5 * (lo + hi);
        for (int i = 0; i < 100; ++i) {
            double val = f(tau) - ds;
            double der = fprime(tau);
            if (val > 0.0) {
                hi = tau;
            } else {
                lo = tau;
            }
            double next = (std::abs(der) > 1e-14) ? tau - val / der
                                                  : 0.5 * (lo + hi);
            if (next <= lo || next >= hi) next = 0.5 * (lo + hi);
            tau = next;
            if (hi - lo < 1e-13 * (1.0 + tau)) break;
        }

        return tau;
    }

    /**
     * @brief Smallest positive time τ such that v(τ) = 0.
     *
     * Solves: (σ/6)τ³ + (j0/2)τ² + a0·τ + v0 = 0
     * Uses Newton's method with bracketing.
     *
     * Returns +∞ if v(t) stays positive for all τ > 0.
     */
    static double timeToStop(double v0, double a0, double j0, double sigma) {
        if (v0 <= 0.0) return 0.0;

        // f(τ) = v(τ) = v0 + a0·τ + ½·j0·τ² + (1/6)·σ·τ³
        // f'(τ) = a(τ) = a0 + j0·τ + ½·σ·τ²
        auto f = [&](double t) {
            return v0 + a0 * t + 0.5 * j0 * t * t
                   + (1.0 / 6.0) * sigma * t * t * t;
        };
        auto fprime = [&](double t) {
            return a0 + j0 * t + 0.5 * sigma * t * t;
        };

        // Find bracket where f changes sign
        double hi = 1.0;
        while (f(hi) > 0.0 && hi < 1e6) hi *= 2.0;
        if (f(hi) > 0.0) return std::numeric_limits<double>::infinity();

        double lo = 0.0;
        double tau = 0.5 * (lo + hi);
        for (int i = 0; i < 100; ++i) {
            double val = f(tau);
            double der = fprime(tau);
            if (val > 0.0) lo = tau; else hi = tau;
            double next = (std::abs(der) > 1e-14) ? tau - val / der
                                                  : 0.5 * (lo + hi);
            if (next <= lo || next >= hi) next = 0.5 * (lo + hi);
            tau = next;
            if (hi - lo < 1e-13 * (1.0 + tau)) break;
        }
        return tau;
    }

    /**
     * @brief Maximum forward distance reachable while keeping v ≥ 0.
     */
    static double maxForwardDistance(double v0, double a0, double j0,
                                      double sigma) {
        double tStop = timeToStop(v0, a0, j0, sigma);
        if (!std::isfinite(tStop) || tStop <= 0.0)
            return std::numeric_limits<double>::infinity();
        return ds(v0, a0, j0, sigma, tStop);
    }

    /**
     * @brief Terminal arc: solve a 2-arc sequence that brings
     * (v, a, j) → (0, 0, 0) at exactly sRemaining.
     *
     * Arc 1 (SNAP): bring j to 0.
     *   σ1 = -j0 / τ1
     *   After: j=0, a1 = a0 + ½·j0·τ1, v1 = v0 + a0·τ1 + (1/3)·j0·τ1²
     *
     * Arc 2 (constant jerk jTerm): bring (v, a) → (0, 0) with j=0.
     *   τ2 = -2·v1/a1, jTerm = a1²/(2·v1)  (requires a1 < 0, v1 > 0)
     *
     * @return true if feasible, populating sigma1, tau1, jTerm, tau2.
     */
    static bool terminalArc2(double v0, double a0, double j0, double sRemaining,
                             double sigmaMax, double jMax,
                             double& sigma1, double& tau1,
                             double& jTerm, double& tau2) {
        if (sRemaining <= 0.0) {
            sigma1 = 0; tau1 = 0; jTerm = 0; tau2 = 0;
            return true;
        }
        if (v0 <= 0.0) return false;

        // If j ≈ 0, skip arc 1 and just do arc 2
        if (std::abs(j0) < 1e-12) {
            tau1 = 0; sigma1 = 0;
            if (a0 >= 0.0 || v0 <= 0.0) return false;
            tau2 = -2.0 * v0 / a0;
            jTerm = a0 * a0 / (2.0 * v0);
            if (std::abs(jTerm) > jMax) return false;
            return tau2 > 0.0 && std::isfinite(tau2);
        }

        // Arc 1: bring j to 0.
        // σ1 = -j0/τ1, need |σ1| ≤ σMax → τ1 ≥ |j0|/σMax
        double tau1Min = std::abs(j0) / std::max(sigmaMax, 1e-12);

        // State after arc 1:
        // a1 = a0 + ½·j0·τ1
        // v1 = v0 + a0·τ1 + (1/3)·j0·τ1²
        // s1 = v0·τ1 + ½·a0·τ1² + (1/6)·j0·τ1³ - (1/24)·j0·τ1⁴

        // Arc 2 (constant jerk jTerm):
        // τ2 = -2·v1/a1, jTerm = a1²/(2·v1)
        // s2 = v1·τ2 + ½·a1·τ2² + (1/6)·jTerm·τ2³

        auto totalDistance = [&](double t1) -> double {
            double s1 = v0 * t1 + 0.5 * a0 * t1 * t1
                        + (1.0 / 6.0) * j0 * t1 * t1 * t1
                        - (1.0 / 24.0) * j0 * t1 * t1 * t1 * t1;
            double a1 = a0 + 0.5 * j0 * t1;
            double v1 = v0 + a0 * t1 + (1.0 / 3.0) * j0 * t1 * t1;
            if (a1 >= 0.0 || v1 <= 0.0)
                return std::numeric_limits<double>::infinity();
            double t2 = -2.0 * v1 / a1;
            double jt = a1 * a1 / (2.0 * v1);
            if (std::abs(jt) > jMax)
                return std::numeric_limits<double>::infinity();
            double s2 = v1 * t2 + 0.5 * a1 * t2 * t2
                        + (1.0 / 6.0) * jt * t2 * t2 * t2;
            return s1 + s2;
        };

        // Find τ1 by bisection: totalDistance(τ1) = sRemaining
        double lo = tau1Min;
        double hi = tau1Min * 2.0 + 1.0;
        for (int k = 0; k < 60 && std::isfinite(totalDistance(hi)) &&
             totalDistance(hi) > sRemaining; ++k) {
            hi *= 2.0;
        }

        double dLo = totalDistance(lo);
        double dHi = totalDistance(hi);
        if (!std::isfinite(dLo) || dLo < sRemaining) return false;
        if (!std::isfinite(dHi) || dHi > sRemaining) return false;

        // Bisect
        for (int iter = 0; iter < 80; ++iter) {
            double mid = 0.5 * (lo + hi);
            double dMid = totalDistance(mid);
            if (!std::isfinite(dMid)) { hi = mid; dHi = dMid; continue; }
            if (dMid > sRemaining) { lo = mid; dLo = dMid; }
            else { hi = mid; dHi = dMid; }
            if (hi - lo < 1e-13 * (1.0 + mid)) break;
        }

        tau1 = 0.5 * (lo + hi);
        sigma1 = -j0 / std::max(tau1, 1e-12);
        if (std::abs(sigma1) > sigmaMax) return false;

        double a1 = a0 + 0.5 * j0 * tau1;
        double v1 = v0 + a0 * tau1 + (1.0 / 3.0) * j0 * tau1 * tau1;
        if (a1 >= 0.0 || v1 <= 0.0) return false;
        tau2 = -2.0 * v1 / a1;
        jTerm = a1 * a1 / (2.0 * v1);
        if (std::abs(jTerm) > jMax) return false;

        return tau2 > 0.0 && std::isfinite(tau2);
    }

    /**
     * @brief Estimate the minimum stopping distance from (v, a, j) to (0, 0, 0).
     *
     * Uses a conservative multi-phase estimate:
     * 1. SNAP to bring j to 0 (distance ≈ v·|j|/σMax)
     * 2. Constant jerk to bring a to 0 (distance ≈ v·|a|/jMax)
     * 3. Constant -aMax deceleration to v=0 (distance ≈ v²/(2·aMax))
     */
    static double terminalMinDistance(double v0, double a0, double j0,
                                       double sigmaMax, double jMax,
                                       double aMax) {
        if (v0 <= 0.0) return 0.0;
        double s = 0.0;
        double v = v0, a = a0, j = j0;

        // Phase 1: bring j to 0
        if (std::abs(j) > 1e-12) {
            double tau1 = std::abs(j) / std::max(sigmaMax, 1e-12);
            double sigma = -j / std::max(tau1, 1e-12);
            s += ds(v, a, j, sigma, tau1);
            v = SnapSeg::v(v, a, j, sigma, tau1);
            a = SnapSeg::a(a, j, sigma, tau1);
            j = 0.0;
        }

        // Phase 2: bring a to 0 (if a > 0)
        if (a > 0.0) {
            double tau2 = a / std::max(jMax, 1e-12);
            s += SingularJSeg::ds(v, a, -jMax, tau2);
            v = SingularJSeg::v(v, a, -jMax, tau2);
            a = 0.0;
        }

        // Phase 3: constant deceleration at -aMax to v=0
        if (v > 0.0) {
            s += v * v / (2.0 * std::max(aMax, 1e-12));
        }

        return s;
    }

    /**
     * @brief 3-arc terminal: handles a ≥ 0 by first decelerating a to negative.
     *
     * Arc 0 (SNAP): bring j to -jMax (start decelerating).
     *   After: j=-jMax, a0' = a0 + j0·τ0 + ½·σ0·τ0², v0' = ...
     *
     * Arc 1 (SNAP): bring j from -jMax to 0 while a reaches a negative value.
     *   After: j=0, a1 < 0, v1 > 0
     *
     * Arc 2 (constant jerk jTerm): bring (v, a) → (0, 0) with j=0.
     *   τ2 = -2·v1/a1, jTerm = a1²/(2·v1)
     *
     * @return true if feasible, populating sigma0, tau0, sigma1, tau1, jTerm, tau2.
     */
    static bool terminalArc3(double v0, double a0, double j0, double sRemaining,
                             double sigmaMax, double jMax,
                             double& sigma0, double& tau0,
                             double& sigma1, double& tau1,
                             double& jTerm, double& tau2) {
        if (sRemaining <= 0.0 || v0 <= 0.0) {
            sigma0 = 0; tau0 = 0; sigma1 = 0; tau1 = 0; jTerm = 0; tau2 = 0;
            return sRemaining <= 0.0;
        }

        // Strategy: use bisection on tau0 (duration of phase 0).
        // Phase 0: SNAP to bring j to -jMax.
        //   sigma0 = (-jMax - j0) / tau0
        //   After phase 0: j = -jMax
        //   a_after0 = a0 + j0*tau0 + 0.5*sigma0*tau0²
        //   v_after0 = v0 + a0*tau0 + 0.5*j0*tau0² + (1/6)*sigma0*tau0³
        //   s_after0 = ds(v0, a0, j0, sigma0, tau0)
        //
        // Then call terminalArc2 with the state after phase 0.

        auto tryTerminal = [&](double t0, double& sTotal) -> bool {
            double sig0 = (-jMax - j0) / std::max(t0, 1e-12);
            if (std::abs(sig0) > sigmaMax) { sTotal = 0; return false; }

            double j1 = -jMax;
            double a1 = a0 + j0 * t0 + 0.5 * sig0 * t0 * t0;
            double v1 = v0 + a0 * t0 + 0.5 * j0 * t0 * t0
                        + (1.0/6.0) * sig0 * t0 * t0 * t0;
            double s1 = ds(v0, a0, j0, sig0, t0);

            if (v1 <= 0.0) { sTotal = 0; return false; }

            double sRem = sRemaining - s1;
            if (sRem <= 0.0) { sTotal = s1; return false; }

            // Now use terminalArc2 with (v1, a1, j1=-jMax)
            double sig2, t2, jt, t3;
            bool ok = terminalArc2(v1, a1, j1, sRem, sigmaMax, jMax,
                                   sig2, t2, jt, t3);
            if (!ok) { sTotal = 0; return false; }

            sTotal = s1 + (v1 * t2 + 0.5 * a1 * t2 * t2
                           + (1.0/6.0) * j1 * t2 * t2 * t2)  // s for arc 1 of terminalArc2
                     + (v1 * t3 + 0.5 * a1 * t3 * t3
                        + (1.0/6.0) * jt * t3 * t3 * t3);  // s for arc 2
            // Actually, terminalArc2 already computes the total distance
            // internally. We just need to check it matches sRem.
            // Let's store the parameters for later use.
            sigma0 = sig0;
            tau0 = t0;
            sigma1 = sig2;
            tau1 = t2;
            jTerm = jt;
            tau2 = t3;
            return true;
        };

        // Find tau0 by scanning. We want the total distance to equal sRemaining.
        // Try a range of tau0 values.
        double bestErr = 1e18;
        double bestT0 = 0;
        double sBest = 0;

        for (int k = 0; k < 100; ++k) {
            double t0 = 0.001 + 0.01 * k;  // 0.001 to 1.0
            double sTot;
            // We need to actually call terminalArc2 and check
            double sig0 = (-jMax - j0) / std::max(t0, 1e-12);
            if (std::abs(sig0) > sigmaMax) continue;

            double j1 = -jMax;
            double a1 = a0 + j0 * t0 + 0.5 * sig0 * t0 * t0;
            double v1 = v0 + a0 * t0 + 0.5 * j0 * t0 * t0
                        + (1.0/6.0) * sig0 * t0 * t0 * t0;
            double s1 = ds(v0, a0, j0, sig0, t0);
            if (v1 <= 0.0) continue;

            double sRem = sRemaining - s1;
            if (sRem <= 0.0) continue;

            double sig2, t2, jt, t3;
            bool ok = terminalArc2(v1, a1, j1, sRem, sigmaMax, jMax,
                                   sig2, t2, jt, t3);
            if (!ok) continue;

            // terminalArc2 guarantees the distance matches sRem.
            // So total distance = s1 + sRem = sRemaining. Perfect.
            sigma0 = sig0;
            tau0 = t0;
            sigma1 = sig2;
            tau1 = t2;
            jTerm = jt;
            tau2 = t3;
            return true;
        }

        (void)bestErr; (void)bestT0; (void)sBest; (void)tryTerminal;
        return false;
    }
};

// ============================================================================
// Backward compatibility aliases (deprecated)
// ============================================================================

/// @deprecated Legacy third-order constant-jerk primitive. New snap-space
/// code must use `SnapSeg` or `SingularJSeg`; this wrapper exists only for
/// source compatibility with the former jerk-space API.
struct BangSeg {
    static double a(double a0, double eta, double /*aStar*/, double tau) {
        return a0 + eta * tau;
    }
    static double v(double v0, double a0, double eta, double /*aStar*/,
                    double tau) {
        return v0 + a0 * tau + 0.5 * eta * tau * tau;
    }
    static double ds(double v0, double a0, double eta, double /*aStar*/,
                     double tau) {
        return v0 * tau + 0.5 * a0 * tau * tau +
               eta * tau * tau * tau / 6.0;
    }
    static double tau_for_ds(double v0, double a0, double eta,
                             double /*aStar*/, double distance) {
        return SingularJSeg::tau_for_ds(v0, a0, eta, distance);
    }
};

/// @deprecated Legacy second-order constant-acceleration primitive. New
/// snap-space code must use `SnapSeg` or `SingularJSeg`.
struct SingSeg {
    static double v(double v0, double aStar, double /*unused*/, double tau) {
        return v0 + aStar * tau;
    }
    static double ds(double v0, double aStar, double /*unused*/, double tau) {
        return v0 * tau + 0.5 * aStar * tau * tau;
    }
    static double tau_for_ds(double v0, double aStar, double /*unused*/,
                             double distance) {
        if (distance <= 0.0) return 0.0;
        if (std::abs(aStar) <= 1e-14) {
            return v0 > 0.0 ? distance / v0
                             : std::numeric_limits<double>::quiet_NaN();
        }
        const double discriminant = v0 * v0 + 2.0 * aStar * distance;
        if (discriminant < 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double root = std::sqrt(discriminant);
        const double first = (-v0 + root) / aStar;
        const double second = (-v0 - root) / aStar;
        if (first >= 0.0) return first;
        if (second >= 0.0) return second;
        return std::numeric_limits<double>::quiet_NaN();
    }
};

/**
 * @brief Closed-form braking distance estimate (snapspace).
 *
 * Estimates the distance to bring (v, a, j) → (0, 0, 0).
 */
inline double brake_distance(double v, double a, double j,
                              double sigmaMax, double jMax, double aMax) {
    return SnapSeg::terminalMinDistance(v, a, j, sigmaMax, jMax, aMax);
}

/// @deprecated Old jerkspace brake_distance signature.
inline double brake_distance(double v, double a_star,
                              double eta_min, double eta_max) {
    if (v <= 0.0) return 0.0;
    a_star = std::max(std::abs(a_star), 1e-12);
    return v * v / (2.0 * a_star);
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
     * The WSS keeps the path alive via shared_ptr, eliminating the
     * lifetime hazard of the previous const-reference design.
     *
     * @param path The path (shared ownership)
     * @param arcs The weighted arcs (the solution)
     * @param w The cost weights used
     * @param evaluator The constraint evaluator (for velocity limit
     *                  queries during sampling)
     * @param optimalAStar The optimal singular acceleration level found by
     *                     the solver (used by optimalAStar())
     */
    WeightedSwitchingStructure(
        std::shared_ptr<const Path> path,
        std::vector<Arc> arcs,
        CostWeights w,
        ConstraintEvaluator<Dim, T> evaluator,
        double optimalAStar = 0.0)
        : pathPtr_(std::move(path))
        , arcs_(std::move(arcs))
        , w_(w)
        , evaluator_(std::move(evaluator))
        , optimalAStar_(optimalAStar) {

        // Use pre-computed t0 and duration from the solver.
        // Recompute total time from the arc list.
        totalTime_ = 0.0;
        double tAccum = 0.0;
        for (auto& arc : arcs_) {
            arc.t0 = tAccum;
            // Only recompute durations for WALL arcs (which need the
            // velocity-limit quadrature). For SNAP and SINGULAR arcs,
            // trust the solver's computed duration — recomputing via
            // tau_for_ds can find the wrong root when velocity goes
            // negative during deceleration arcs.
            if (arc.type == WeightedArcType::WALL) {
                arc.duration = computeArcDuration(arc);
            }
            tAccum += arc.duration;
        }
        totalTime_ = tAccum;
    }

    // --- AnalyticalTrajectorySource interface ---

    T totalTime() const override { return static_cast<T>(totalTime_); }
    T totalLength() const override { return pathPtr_->totalLength(); }

    /**
     * @brief Compute the time at which the trajectory reaches arc length s.
     *
     * Inverts each arc's s(t) relation exactly (BANG/SINGULAR via closed-form
     * polynomials, WALL via the velocity-limit quadrature).
     */
    T timeAtArcLength(T s_query) const override {
        double s = static_cast<double>(s_query);
        if (arcs_.empty()) return T(0);
        for (const auto& arc : arcs_) {
            if (s <= arc.s1) {
                if (arc.type == WeightedArcType::DWELL) {
                    return static_cast<T>(arc.t0);
                }
                double dsLocal = s - arc.s0;
                dsLocal = std::clamp(dsLocal, 0.0, arc.s1 - arc.s0);
                double tau = 0.0;
                if (arc.type == WeightedArcType::WALL) {
                    tau = wallDuration(arc.s0, arc.s0 + dsLocal);
                } else if (dsLocal >= arc.s1 - arc.s0 -
                                      1e-13 * (1.0 + arc.length())) {
                    tau = arc.duration;
                } else {
                    // A stored trajectory is forward-moving by construction.
                    // Bisection over its known duration is therefore monotone
                    // and avoids quartic/cubic roots beyond a deceleration arc.
                    double lo = 0.0;
                    double hi = arc.duration;
                    for (int iter = 0; iter < 100; ++iter) {
                        const double mid = 0.5 * (lo + hi);
                        const double distance =
                            arc.type == WeightedArcType::SINGULAR
                                ? SingularJSeg::ds(arc.v0, arc.a0,
                                                   arc.j_star, mid)
                                : SnapSeg::ds(arc.v0, arc.a0, arc.j0,
                                              arc.sigma, mid);
                        if (distance < dsLocal) lo = mid;
                        else hi = mid;
                        if (hi - lo <= 1e-13 * (1.0 + arc.duration)) break;
                    }
                    tau = 0.5 * (lo + hi);
                }
                return static_cast<T>(arc.t0 + tau);
            }
        }
        return static_cast<T>(totalTime_);
    }

    Vec<Dim, T> position(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = pathPtr_->evaluateAtArcLength(static_cast<T>(s));
        return eval.position;
    }

    Vec<Dim, T> velocity(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = pathPtr_->evaluateAtArcLength(static_cast<T>(s));
        // qdot = T * v (tangent * path velocity)
        Vec<Dim, T> result;
        for (size_t i = 0; i < Dim; ++i)
            result[i] = eval.tangent[i] * static_cast<T>(v);
        return result;
    }

    Vec<Dim, T> acceleration(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = pathPtr_->evaluateAtArcLength(static_cast<T>(s));
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
        return pathPtr_->curvatureAtArcLength(static_cast<T>(s));
    }

    SourceReference sourceRef(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        return pathPtr_->sourceRefAtArcLength(static_cast<T>(s));
    }

    size_t segmentIndex(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = pathPtr_->evaluateAtArcLength(static_cast<T>(s));
        return eval.segmentIndex;
    }

    T segmentParameter(T t) const override {
        auto [arcIdx, tau, s, v, a, eta] = locateAndState(t);
        auto eval = pathPtr_->evaluateAtArcLength(static_cast<T>(s));
        return eval.localParameter;
    }

    const char* representationName() const override {
        return "WeightedSwitchingStructure";
    }

    ProfileDerivativeOrder derivativeOrder() const override {
        return ProfileDerivativeOrder::Snap;
    }

    // --- Accessors ---

    const std::vector<Arc>& arcs() const { return arcs_; }
    const CostWeights& weights() const { return w_; }
    double costValue() const { return costValue_; }
    void setCostValue(double J) { costValue_ = J; }

    /**
     * @brief The optimal singular acceleration level a* found by the solver.
     */
    double optimalAStar() const { return optimalAStar_; }

    /// Exact start state encoded by the first stored arc.
    WeightedState startState() const {
        if (arcs_.empty()) return {};
        const auto& arc = arcs_.front();
        return {arc.s0, arc.v0, arc.a0, arc.j0};
    }

    /// Exact terminal state encoded by the final stored arc.
    WeightedState endState() const {
        if (arcs_.empty()) return {};
        const auto& arc = arcs_.back();
        return {arc.s1, arc.v1, arc.a1, arc.j1};
    }

    /**
     * @brief Sample the WSS at uniform arc-length intervals to produce
     *        a tabulated SampledVelocityProfile.
     *
     * @param numSamples Number of sample points along the path.
     * @param startAcceleration Initial acceleration at the path start.
     * @return A sampled velocity profile compatible with the tabulated API.
     */
    SampledVelocityProfile toVelocityProfile(
        size_t numSamples,
        T startAcceleration = T(0)) const {
        SampledVelocityProfile profile;
        T pathLength = totalLength();
        if (pathLength <= T(0) || numSamples < 2) return profile;

        T ds = pathLength / T(numSamples - 1);
        profile.reserve(numSamples);

        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(T(i) * ds, pathLength);

            // Find time at this arc length using the exact WSS mapping
            T t = timeAtArcLength(s);

            // Sample state from WSS
            T v = pathVelocity(t);
            T a = pathAcceleration(t);
            T j = pathJerk(t);

            VelocityProfilePoint pt;
            pt.arcLength = static_cast<double>(s);
            pt.velocity = static_cast<double>(v);
            // The WSS has an exact stored initial state. Do not replace it
            // with an API compatibility argument that may disagree with the
            // trajectory that was actually solved.
            (void)startAcceleration;
            pt.acceleration = static_cast<double>(a);
            pt.jerk = static_cast<double>(j);
            pt.time = static_cast<double>(t);

            const auto type = arcTypeAt(t);
            if (type == WeightedArcType::DWELL) {
                pt.limitedBy = VelocityProfilePoint::LimitType::None;
            } else if (type == WeightedArcType::WALL) {
                pt.limitedBy = VelocityProfilePoint::LimitType::Curvature;
            } else if (std::abs(j) > T(1e-10)) {
                pt.limitedBy = VelocityProfilePoint::LimitType::Jerk;
            } else if (std::abs(a) > T(1e-10)) {
                pt.limitedBy = a > T(0)
                    ? VelocityProfilePoint::LimitType::ForwardAccel
                    : VelocityProfilePoint::LimitType::BackwardDecel;
            } else {
                pt.limitedBy = VelocityProfilePoint::LimitType::None;
            }

            profile.addPoint(pt);
        }

        return profile;
    }

    /**
     * @brief The arc type active at time t.
     */
    WeightedArcType arcTypeAt(T t) const {
        double tD = static_cast<double>(t);
        if (arcs_.empty()) return WeightedArcType::SINGULAR;

        size_t lo = 0, hi = arcs_.size();
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (arcs_[mid].t0 + arcs_[mid].duration < tD)
                lo = mid + 1;
            else
                hi = mid;
        }
        size_t idx = std::min(lo, arcs_.size() - 1);
        return arcs_[idx].type;
    }

    /// Access the path (kept alive by the WSS).
    std::shared_ptr<const Path> path() const { return pathPtr_; }

private:
    std::shared_ptr<const Path> pathPtr_;
    std::vector<Arc> arcs_;
    CostWeights w_;
    ConstraintEvaluator<Dim, T> evaluator_;
    double totalTime_ = 0.0;
    double costValue_ = 0.0;
    double optimalAStar_ = 0.0;

    /**
     * @brief Locate the arc containing time t and compute the full state.
     *
     * @return Tuple (arcIdx, tau, s, v, a, j) where tau = t - arc.t0
     *         and j is the jerk (snapspace 4D state).
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

        // Compute state at tau using closed-form snapspace arc formulas
        double s, v, a, jerk;
        if (arc.type == WeightedArcType::DWELL) {
            // DWELL: v=0, a=0, j=0, s=const
            s = arc.s0;
            v = 0.0;
            a = 0.0;
            jerk = 0.0;
        } else if (arc.type == WeightedArcType::SINGULAR) {
            // SINGULAR: constant jerk j*
            jerk = arc.j_star;
            a = SingularJSeg::a(arc.a0, arc.j_star, tau);
            v = SingularJSeg::v(arc.v0, arc.a0, arc.j_star, tau);
            s = arc.s0 + SingularJSeg::ds(arc.v0, arc.a0, arc.j_star, tau);
        } else if (arc.type == WeightedArcType::WALL) {
            // WALL: follow the velocity-limit curve exactly.
            s = wallTimeToS(arc, tau);
            v = wallVelocityLimit(s);
            a = wallAcceleration(s);
            jerk = wallJerk(s);
        } else {
            // SNAP_PLUS or SNAP_MINUS: constant snap σ
            double sigma = arc.sigma;
            jerk = SnapSeg::j(arc.j0, sigma, tau);
            a = SnapSeg::a(arc.a0, arc.j0, sigma, tau);
            v = SnapSeg::v(arc.v0, arc.a0, arc.j0, sigma, tau);
            s = arc.s0 + SnapSeg::ds(arc.v0, arc.a0, arc.j0, sigma, tau);
        }

        return {idx, tau, s, v, a, jerk};
    }

    /**
     * @brief Compute the duration of an arc (time span).
     */
    double computeArcDuration(const Arc& arc) const {
        if (arc.type == WeightedArcType::DWELL) {
            return arc.duration;  // dwell time is pre-set
        }
        double ds = arc.s1 - arc.s0;
        if (ds <= 0.0) return 0.0;

        if (arc.type == WeightedArcType::SINGULAR) {
            return SingularJSeg::tau_for_ds(arc.v0, arc.a0, arc.j_star, ds);
        } else if (arc.type == WeightedArcType::WALL) {
            return wallDuration(arc.s0, arc.s1);
        } else {
            // SNAP_PLUS or SNAP_MINUS
            return SnapSeg::tau_for_ds(arc.v0, arc.a0, arc.j0, arc.sigma, ds);
        }
    }

    /// 8-point Gauss-Legendre quadrature for ∫_{s0}^{s1} 1/v_lim(s) ds.
    double wallDuration(double s0, double s1) const {
        const double mid = 0.5 * (s0 + s1);
        const double half = 0.5 * (s1 - s0);
        const std::array<double, 4> nodes = {
            0.9602898564975363,
            0.7966664774136268,
            0.5255324099163290,
            0.1834346424956498
        };
        const std::array<double, 4> weights = {
            0.1012285362903763,
            0.2223810344533745,
            0.3137066458778873,
            0.3626837833783620
        };
        double sum = 0.0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            double x = nodes[i];
            double w = weights[i];
            double sPlus = mid + half * x;
            double sMinus = mid - half * x;
            double vPlus = wallVelocityLimit(sPlus);
            double vMinus = wallVelocityLimit(sMinus);
            sum += w * (1.0 / std::max(vPlus, 1e-12) +
                        1.0 / std::max(vMinus, 1e-12));
        }
        return half * sum;
    }

    /// Invert the wall time integral: find s in [arc.s0, arc.s1] such that
    /// ∫_{arc.s0}^{s} 1/v_lim(σ) dσ = tau.
    double wallTimeToS(const Arc& arc, double tau) const {
        double s0 = arc.s0;
        double s1 = arc.s1;
        double tTotal = wallDuration(s0, s1);
        if (tTotal <= 0.0 || tau <= 0.0) return s0;
        if (tau >= tTotal) return s1;

        double lo = s0, hi = s1;
        for (int iter = 0; iter < 80; ++iter) {
            double mid = 0.5 * (lo + hi);
            double tMid = wallDuration(s0, mid);
            if (tMid < tau) {
                lo = mid;
            } else {
                hi = mid;
            }
            if (hi - lo < 1e-13 * (1.0 + std::abs(mid))) break;
        }
        return 0.5 * (lo + hi);
    }

    /// Velocity limit at arc length s.
    double wallVelocityLimit(double s) const {
        return static_cast<double>(
            evaluator_.velocityLimit(static_cast<T>(s), *pathPtr_));
    }

    /// Wall acceleration a = v * dv_lim/ds.
    double wallAcceleration(double s) const {
        const double eps = 1e-6;
        double vL = wallVelocityLimit(s - eps);
        double vC = wallVelocityLimit(s);
        double vR = wallVelocityLimit(s + eps);
        double dvds = (vR - vL) / (2.0 * eps);
        double a = vC * dvds;

        return a;
    }

    /// Wall jerk j = v * da/ds, evaluated consistently with the wall's
    /// finite-difference derivative model. No feasibility clamp is applied.
    double wallJerk(double s) const {
        const double eps = 1e-5;
        const double aL = wallAcceleration(s - eps);
        const double aR = wallAcceleration(s + eps);
        return wallVelocityLimit(s) * (aR - aL) / (2.0 * eps);
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
     * @param constraintCacheSize Number of grid points for the constraint cache
     * @return Vector of weighted arcs (the solution)
     */
    /**
     * @brief Solve the weighted-cost problem (snapspace).
     *
     * Optimizes over j* (singular jerk level) using golden-section search.
     *
     * @param startVelocity Initial velocity (default 0)
     * @param endVelocity Final velocity (default 0)
     * @param constraintCacheSize Number of grid points for the constraint cache
     * @return Vector of weighted arcs (the solution)
     */
    std::vector<Arc> solve(T startVelocity = T(0),
                            T endVelocity = T(0),
                            size_t constraintCacheSize = 200) {
        v0_ = static_cast<double>(startVelocity);
        vf_ = static_cast<double>(endVelocity);
        constraintCacheSize_ = std::max(constraintCacheSize, size_t(10));
        buildConstraintCache();

        const auto vLimit = [this](double s) {
            return static_cast<double>(evaluator_.velocityLimit(
                static_cast<T>(std::clamp(s, 0.0, sTotal_)), path_));
        };
        auto result = optimizePulsePlan(vLimit, v0_, vf_);

        arcs_ = std::move(result.arcs);
        lastCost_ = result.cost;
        lastS_ = result.finalS;
        lastV_ = result.finalV;
        lastFailure_ = result.failureReason;
        lastJStar_ = 0.0;
        for (const auto& arc : arcs_) {
            lastJStar_ = std::max(lastJStar_, std::abs(arc.j0));
            lastJStar_ = std::max(lastJStar_, std::abs(arc.j1));
        }
        if (!result.feasible) arcs_.clear();
        return arcs_;

        // Historical heuristic retained only for source archaeology. The
        // exact pulse kernel above is the sole compiled implementation.
    #if 0
        v0_ = static_cast<double>(startVelocity);
        vf_ = static_cast<double>(endVelocity);
        constraintCacheSize_ = std::max(constraintCacheSize, size_t(10));

        buildConstraintCache();

        // Estimate max reachable jerk
        double jMax = estimateMaxReachableJerk();

        // Golden-section search over j* ∈ (0, jMax]
        auto J = [this](double j_star) {
            return simulateAndCost(j_star, /*record=*/false);
        };

        double jLo = 1e-6 * jMax;
        double jHi = jMax;

        // Coarse log-spaced grid scan
        const int coarseN = 17;
        double jBest = jLo;
        double JBest = std::numeric_limits<double>::infinity();
        for (int i = 0; i < coarseN; ++i) {
            double t = static_cast<double>(i) / (coarseN - 1.0);
            double j = jLo * std::pow(jHi / jLo, t);
            double Ji = simulateAndCost(j, /*record=*/false);
            if (Ji < JBest) {
                JBest = Ji;
                jBest = j;
            }
        }

        // Golden-section in the two cells surrounding the best grid point.
        double cellLo = std::max(jLo, jBest * std::pow(jHi / jLo, -1.0 / (coarseN - 1.0)));
        double cellHi = std::min(jHi, jBest * std::pow(jHi / jLo,  1.0 / (coarseN - 1.0)));
        if (cellLo >= cellHi) { cellLo = jLo; cellHi = jHi; }

        double tol = std::max(1e-6 * jMax, 0.01);
        auto [jOpt, Jmin] = goldenSection(J, cellLo, cellHi, tol);

        // The endpoint jHi can be optimal for w_j = 0, w_a = 0 (time-optimal).
        if (jOpt < jHi) {
            double Jhi = simulateAndCost(jHi, /*record=*/false);
            if (Jhi < Jmin) {
                jOpt = jHi;
                Jmin = Jhi;
            }
        }

        // Rebuild the optimal arc list
        arcs_.clear();
        double Jfinal = simulateAndCost(jOpt, /*record=*/true);

        // Coalesce consecutive arcs with identical type/sigma/j_star
        std::vector<Arc> coalesced;
        coalesced.reserve(arcs_.size());
        for (auto& arc : arcs_) {
            bool canCoalesce = false;
            if (!coalesced.empty() &&
                coalesced.back().type == arc.type &&
                coalesced.back().sigma == arc.sigma &&
                coalesced.back().j_star == arc.j_star &&
                std::abs(coalesced.back().s1 - arc.s0) < 1e-12) {
                if (arc.type == WeightedArcType::SNAP_PLUS ||
                    arc.type == WeightedArcType::SNAP_MINUS) {
                    // For SNAP arcs, check j0 continuity:
                    // j1 of previous = j0_prev + sigma * dur_prev
                    // must equal j0 of current
                    double j1Prev = coalesced.back().j0 +
                                    coalesced.back().sigma *
                                    coalesced.back().duration;
                    canCoalesce = (std::abs(j1Prev - arc.j0) < 1.0);
                } else {
                    // WALL and SINGULAR: safe to coalesce
                    canCoalesce = true;
                }
            }
            if (canCoalesce) {
                coalesced.back().s1 = arc.s1;
                coalesced.back().duration += arc.duration;
            } else {
                coalesced.push_back(std::move(arc));
            }
        }
        arcs_ = std::move(coalesced);

        (void)Jfinal;

        // Feasibility check: reject if the solver cannot satisfy the boundary
        // conditions. Two cases:
        // 1. vf > 0 (flying end): the solver must reach v ≈ vf. If the residual
        //    is large, the boundary is physically impossible → reject.
        // 2. vf ≈ 0 (rest end): the solver may have a small residual due to
        //    discretization. Only reject if it also didn't reach the end of
        //    the path (the solver got stuck or stopped early).
        if (vf_ > 1e-3) {
            // Flying end: check terminal velocity residual
            if (std::abs(lastV_ - vf_) > 5.0) {
                arcs_.clear();
                return arcs_;
            }
        } else {
            // Rest end: only reject if solver didn't reach the end
            if (lastS_ < sTotal_ - 1e-6) {
                arcs_.clear();
                return arcs_;
            }
        }

        return arcs_;
    #endif
    }

    /**
     * @brief Get the achieved cost value.
     */
    double costValue() const { return lastCost_; }

    /**
     * @brief Get the optimal j* found by the solver.
     */
    double optimalJStar() const { return lastJStar_; }

    /// Whether the latest `solve()` call reached all requested boundaries.
    bool feasible() const { return arcs_.size() > 0 && lastFailure_.empty(); }

    /// Empty on success; otherwise explains why `solve()` returned no arcs.
    const std::string& failureReason() const { return lastFailure_; }

    /// @deprecated Use optimalJStar() instead. Returns j* clamped to aMax
    /// for backward compatibility with callers expecting an acceleration-like
    /// value bounded by aMax.
    double optimalAStar() const {
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        return std::min(lastJStar_, aMax);
    }

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

    /**
     * @brief Prepare the solver for forward-pass testing.
     *
     * Builds the constraint cache and fine velocity grid, and sets up
     * the boundary velocities. After calling this, forwardPass() can
     * be called with any v_lim function.
     *
     * @param startVelocity Initial velocity
     * @param endVelocity Final velocity
     * @param constraintCacheSize Grid resolution
     */
    void prepareForForwardPass(T startVelocity = T(0),
                                T endVelocity = T(0),
                                size_t constraintCacheSize = 200) {
        v0_ = static_cast<double>(startVelocity);
        vf_ = static_cast<double>(endVelocity);
        constraintCacheSize_ = std::max(constraintCacheSize, size_t(10));
        buildConstraintCache();
        buildFineVelocityGrid();
    }

    /**
     * @brief Get the default v_lim profile (from backward pass + fine grid).
     *
     * This is the v_lim function used by the standard solver. It can be
     * passed to forwardPass() for testing.
     */
    double defaultVLimAt(double s) const {
        double vGrid = vLimFineGrid_[fineGridIndex(s)];
        return lookAheadVLimit(s, vGrid);
    }

    /**
     * @brief Get the path length.
     */
    double pathLength() const { return sTotal_; }

    /**
     * @brief Get the grid step size.
     */
    double gridStep() const { return ds_; }

    /**
     * @brief Run a single forward pass with a given v_lim profile.
     *
     * This is the core reusable method. It takes a pre-computed v_lim
     * profile (as a function object mapping s → v_lim) and produces
     * a deterministic arc sequence.
     *
     * @param jStar The singular jerk level
     * @param vLimFn Function object: double vLimFn(double s) → v_lim at s
     * @param v0 Initial velocity
     * @param vf Final velocity (target)
     * @param sTotal Total path length
     * @return ForwardPassResult with arcs, cost, final state, feasibility
     */
    ForwardPassResult forwardPass(
        double jStar,
        std::function<double(double)> vLimFn,
        double v0,
        double vf,
        double sTotal) const {

        return buildPulsePlan(vLimFn, v0, vf, sTotal,
                              std::max(0.0, jStar), 1.0);

    #if 0
        ForwardPassResult result;
        result.finalS = 0.0;
        result.finalV = v0;
        result.finalA = 0.0;
        result.finalJ = 0.0;
        result.cost = 0.0;
        result.totalTime = 0.0;
        result.feasible = false;

        if (sTotal <= 0.0) {
            result.feasible = (v0 <= 0.001 && vf <= 0.001);
            if (!result.feasible)
                result.failureReason = "Zero-length path with non-zero velocity";
            return result;
        }

        double s = 0.0, t = 0.0;
        double v = v0, a = 0.0, j = 0.0;
        double J = 0.0;

        double sigmaMax = static_cast<double>(limits_.path.maxPathSnap);
        if (sigmaMax <= 0.0) sigmaMax = 50000.0;
        double jMax = static_cast<double>(limits_.path.maxPathJerk);
        if (jMax <= 0.0) jMax = 5000.0;
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (aMax <= 0.0) aMax = 500.0;

        // Clamp jStar to valid range
        jStar = std::clamp(jStar, 0.0, jMax);

        const double dsStep = ds_;
        const double sEnd = sTotal;
        bool infeasible = false;
        std::string failReason;
        int maxIter = static_cast<int>(constraintCacheSize_) * 200;

        double sPrev = -1.0;  // for stuck detection
        for (int iter = 0; iter < maxIter && s < sEnd - 1e-10; ++iter) {
            // If v has dropped to ~0 after starting, can't continue
            if (iter > 0 && v < 1e-6 && s < sEnd - 1e-6) {
                infeasible = true;
                failReason = "Velocity dropped to zero before path end";
                break;
            }

            // If v=0 and desired acceleration is 0, the solver is stuck
            // (can't move from rest with no jerk). This happens when
            // j*=0 and v0=0.
            if (iter > 0 && v < 1e-6 && s < sEnd - 1e-6) {
                infeasible = true;
                failReason = "Stuck at v=0 with no acceleration";
                break;
            }

            // Query acceleration bounds
            auto aBounds = accelBoundsFromCache(gridIndex(s), v);
            if (aBounds.first > aBounds.second) {
                infeasible = true;
                failReason = "Infeasible acceleration bounds at s=" +
                             std::to_string(s) + " v=" + std::to_string(v);
                break;
            }
            double aBoundMin = static_cast<double>(aBounds.first);
            double aBoundMax = static_cast<double>(aBounds.second);

            // Query v_lim
            double vLimNow  = vLimFn(s);
            double vLimNext = vLimFn(std::min(s + dsStep, sEnd));

            // Acceleration to track v_lim
            double aTrack = (vLimNext * vLimNext - v * v) / (2.0 * dsStep);
            double aHi = std::min(aBoundMax, aTrack);
            double aLo = aBoundMin;
            if (aHi < aLo) aHi = aLo;

            // Choose desired acceleration and jerk based on v vs v_lim.
            //
            // The control law has three modes:
            // 1. Accelerating (v well below v_lim): use j* as target jerk,
            //    let acceleration build up naturally.
            // 2. Tracking v_lim (v near v_lim): follow the v_lim slope.
            // 3. Braking (v above v_lim): decelerate as hard as possible.
            //
            // The margin defines the "near v_lim" zone. It's based on
            // the velocity change achievable in one step at the current
            // speed, clamped to avoid huge values when v≈0.
            double a_des;
            double dtEst = dsStep / std::max(v, 1e-3);
            double dtForMargin = std::min(dtEst, 0.1);
            double margin = aMax * dtForMargin + 1e-3;

            if (v > vLimNow + 1e-6) {
                // Overshooting v_lim — brake hard
                a_des = aLo;
            } else if (v >= vLimNow - margin && v > 0.1) {
                // Near v_lim and moving — track it
                a_des = std::clamp(aTrack, aLo, aHi);
            } else {
                // Below v_lim (or at rest) — accelerate using j*
                double aFromJ = a + jStar * dtEst;
                a_des = std::min(aFromJ, aHi);
            }
            a_des = std::clamp(a_des, aLo, aHi);

            // Choose desired jerk
            double dtEff = std::min(dtEst, 1.0);
            double j_des;
            if (v > vLimNow + 1e-6) {
                // Braking — compute jerk from a_des
                j_des = (a_des - a) / std::max(dtEff, 1e-6);
                j_des = std::clamp(j_des, -jMax, jMax);
            } else if (v >= vLimNow - margin && v > 0.1) {
                // Tracking — compute jerk from a_des
                j_des = (a_des - a) / std::max(dtEff, 1e-6);
                j_des = std::clamp(j_des, -jMax, jMax);
            } else {
                // Accelerating — use j* as target jerk
                j_des = std::clamp(jStar, -jMax, jMax);
            }

            // Choose snap (bang-bang in snap)
            double sigmaDes;
            double jErr = j_des - j;
            if (std::abs(jErr) < 0.5) {
                sigmaDes = 0.0;
            } else {
                sigmaDes = std::copysign(sigmaMax, jErr);
            }

            // Determine arc type
            WeightedArcType arcType;
            double sigma;
            double jStarEff = j_des;

            if (v > 1e-6 && v >= vLimNow - margin && v <= vLimNow + 1e-6
                && std::abs(a) < 2.0 && std::abs(j) < 2.0
                && std::abs(sigmaDes) < 1.0) {
                arcType = WeightedArcType::WALL;
                sigma = 0.0;
                j = 0.0;
                a = 0.0;
            } else if (std::abs(sigmaDes) < 1e-9) {
                arcType = WeightedArcType::SINGULAR;
                sigma = 0.0;
            } else if (sigmaDes > 0) {
                arcType = WeightedArcType::SNAP_PLUS;
                sigma = sigmaDes;
            } else {
                arcType = WeightedArcType::SNAP_MINUS;
                sigma = sigmaDes;
            }

            // Clamp snap if jerk at bounds
            if (sigma > 0.0 && j >= jMax - 1e-6) {
                sigma = 0.0; arcType = WeightedArcType::SINGULAR;
            }
            if (sigma < 0.0 && j <= -jMax + 1e-6) {
                sigma = 0.0; arcType = WeightedArcType::SINGULAR;
            }

            // Step size
            double dsArc = dsStep;
            if (s + dsArc > sEnd) dsArc = sEnd - s;

            // Limit arc when j reaches j_des (SNAP arcs)
            if (arcType == WeightedArcType::SNAP_PLUS ||
                arcType == WeightedArcType::SNAP_MINUS) {
                double tauJ = (j_des - j) / sigma;
                if (tauJ > 1e-12 && tauJ < 1e6) {
                    double dsJ = SnapSeg::ds(v, a, j, sigma, tauJ);
                    if (dsJ > 0.0 && dsJ < dsArc) dsArc = dsJ;
                }
                // Also limit by jMax: if jerk would exceed ±jMax during
                // this arc, shorten the arc to stop at jMax.
                double jEndApprox = j + sigma * (dsArc / std::max(v, 1e-3));
                if (std::abs(jEndApprox) > jMax) {
                    double tauJMax = (std::copysign(jMax, sigma) - j) / sigma;
                    if (tauJMax > 1e-12 && tauJMax < 1e6) {
                        double dsJMax = SnapSeg::ds(v, a, j, sigma, tauJMax);
                        if (dsJMax > 0.0 && dsJMax < dsArc) dsArc = dsJMax;
                    }
                }
            }

            // Limit arc at corners
            if (!corners_.empty()) {
                auto it = std::lower_bound(corners_.begin(), corners_.end(),
                    s, [](const std::pair<double,double>& c, double val) {
                        return c.first < val; });
                for (; it != corners_.end() && it->first <= s + dsArc + 1e-9; ++it) {
                    double dCorner = it->first - s;
                    if (dCorner <= 1e-9) continue;
                    double vCorner = it->second;
                    if (vCorner >= v - 1e-6) continue;
                    double vAtCorner;
                    if (arcType == WeightedArcType::WALL) {
                        vAtCorner = v;
                    } else if (arcType == WeightedArcType::SINGULAR) {
                        double tau_c = SingularJSeg::tau_for_ds(v, a, jStarEff, dCorner);
                        vAtCorner = SingularJSeg::v(v, a, jStarEff, tau_c);
                    } else {
                        double tau_c = SnapSeg::tau_for_ds(v, a, j, sigma, dCorner);
                        vAtCorner = SnapSeg::v(v, a, j, sigma, tau_c);
                    }
                    if (vAtCorner > vCorner + 1e-6) {
                        dsArc = dCorner; break;
                    }
                }
            }

            // Limit arc when a reaches a bound (SNAP arcs)
            if (arcType == WeightedArcType::SNAP_PLUS ||
                arcType == WeightedArcType::SNAP_MINUS) {
                for (double aBound : {aBoundMax, aBoundMin}) {
                    if (std::abs(sigma) < 1e-12) continue;
                    double disc = j * j - 2.0 * sigma * (a - aBound);
                    if (disc < 0.0) continue;
                    double sq = std::sqrt(disc);
                    for (double tauA : {(-j + sq) / sigma, (-j - sq) / sigma}) {
                        if (tauA > 1e-12 && tauA < 1e6) {
                            double dsA = SnapSeg::ds(v, a, j, sigma, tauA);
                            if (dsA > 0.0 && dsA < dsArc) dsArc = dsA;
                        }
                    }
                }
            }

            if (dsArc < 1e-12) dsArc = 1e-12;
            if (dsArc > dsStep) dsArc = dsStep;

            // Propagate
            double tau, v1, a1, j1;
            bool stoppedBeforeEnd = false;

            if (arcType == WeightedArcType::WALL) {
                tau = dsArc / std::max(v, 1e-12);
                v1 = v; a1 = 0.0; j1 = 0.0;
            } else if (arcType == WeightedArcType::SINGULAR) {
                double jUse = jStarEff;
                double sMax = std::numeric_limits<double>::infinity();
                if (a < 0.0 && jUse <= 0.0) {
                    double tStop = SnapSeg::timeToStop(v, a, jUse, 0.0);
                    if (std::isfinite(tStop))
                        sMax = SingularJSeg::ds(v, a, jUse, tStop);
                }
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = SingularJSeg::tau_for_ds(v, a, jUse, dsArc);
                    v1 = 0.0;
                    a1 = SingularJSeg::a(a, jUse, tau);
                    j1 = jUse;
                    stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6);
                } else {
                    tau = SingularJSeg::tau_for_ds(v, a, jUse, dsArc);
                    v1 = SingularJSeg::v(v, a, jUse, tau);
                    a1 = SingularJSeg::a(a, jUse, tau);
                    j1 = jUse;
                }
            } else {
                double sMax = SnapSeg::maxForwardDistance(v, a, j, sigma);
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = SnapSeg::tau_for_ds(v, a, j, sigma, dsArc);
                    if (tau <= 0.0) tau = 1e-12;
                    v1 = SnapSeg::v(v, a, j, sigma, tau);
                    a1 = SnapSeg::a(a, j, sigma, tau);
                    j1 = SnapSeg::j(j, sigma, tau);
                    if (v1 < 0.0) { v1 = 0.0; stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6); }
                } else {
                    tau = SnapSeg::tau_for_ds(v, a, j, sigma, dsArc);
                    // Check if jerk exceeds ±jMax during this arc.
                    // If so, limit tau to the time when jerk hits jMax.
                    double jEnd = SnapSeg::j(j, sigma, tau);
                    if (std::abs(jEnd) > jMax) {
                        double tauJMax = (std::copysign(jMax, sigma) - j) / sigma;
                        if (tauJMax > 1e-15 && tauJMax < tau) {
                            tau = tauJMax;
                            dsArc = SnapSeg::ds(v, a, j, sigma, tau);
                        }
                    }
                    v1 = SnapSeg::v(v, a, j, sigma, tau);
                    a1 = SnapSeg::a(a, j, sigma, tau);
                    j1 = SnapSeg::j(j, sigma, tau);
                }
            }

            // Clamp v1 to vLim at end of arc
            double vLimEnd = vLimFn(std::min(s + dsArc, sEnd));
            if (v1 > vLimEnd + 1e-6) {
                if (arcType == WeightedArcType::SNAP_PLUS ||
                    arcType == WeightedArcType::SNAP_MINUS) {
                    auto fval = [&](double tt) {
                        return SnapSeg::v(v, a, j, sigma, tt) - vLimEnd; };
                    double lo = 0.0, hi = tau;
                    if (fval(lo) <= 0.0 && fval(hi) >= 0.0) {
                        for (int b = 0; b < 60; ++b) {
                            double mid = 0.5*(lo+hi);
                            if (fval(mid) > 0) hi = mid; else lo = mid;
                            if (hi - lo < 1e-14) break;
                        }
                        tau = 0.5*(lo+hi);
                        dsArc = SnapSeg::ds(v, a, j, sigma, tau);
                        v1 = vLimEnd;
                        a1 = SnapSeg::a(a, j, sigma, tau);
                        j1 = SnapSeg::j(j, sigma, tau);
                    }
                } else if (arcType == WeightedArcType::SINGULAR) {
                    double jUse = jStarEff;
                    if (std::abs(jUse) > 1e-12) {
                        double disc = a*a - 2.0*jUse*(v - vLimEnd);
                        if (disc > 0.0) {
                            double tauV = (-a + std::sqrt(disc)) / jUse;
                            if (tauV > 0.0 && tauV < tau) {
                                tau = tauV;
                                dsArc = SingularJSeg::ds(v, a, jUse, tau);
                                v1 = vLimEnd;
                                a1 = SingularJSeg::a(a, jUse, tau);
                                j1 = jUse;
                            }
                        }
                    } else if (std::abs(a) > 1e-12) {
                        double tauV = (vLimEnd - v) / a;
                        if (tauV > 0.0 && tauV < tau) {
                            tau = tauV;
                            dsArc = SingularJSeg::ds(v, a, jUse, tau);
                            v1 = vLimEnd; a1 = a; j1 = jUse;
                        }
                    }
                }
            }

            if (v1 < 0.0) v1 = 0.0;
            if (dsArc < 1e-12) dsArc = 1e-12;
            if (dsArc > dsStep) dsArc = dsStep;
            j1 = std::clamp(j1, -jMax, jMax);
            auto aEndB = accelBoundsFromCache(gridIndex(s + dsArc), v1);
            a1 = std::clamp(a1, static_cast<double>(aEndB.first),
                                 static_cast<double>(aEndB.second));

            // Cost increment
            double dJ;
            if (arcType == WeightedArcType::SINGULAR) {
                double jUse = jStarEff;
                double intJ2 = jUse * jUse * tau;
                double intA2 = a*a*tau + a*jUse*tau*tau
                    + (jUse*jUse)/3.0 * tau*tau*tau;
                dJ = w_.w_t * tau + w_.w_j * intJ2 + w_.w_a * intA2;
            } else if (arcType == WeightedArcType::WALL) {
                dJ = w_.w_t * tau;
            } else {
                double s_ = sigma;
                double intJ2 = j*j*tau + j*s_*tau*tau
                    + (s_*s_*tau*tau*tau)/3.0;
                double intA2 = a*a*tau + a*j*tau*tau
                    + (j*j + a*s_)/3.0 * tau*tau*tau
                    + (j*s_)/4.0 * tau*tau*tau*tau
                    + (s_*s_)/20.0 * tau*tau*tau*tau*tau;
                dJ = w_.w_t * tau + w_.w_j * intJ2 + w_.w_a * intA2;
            }
            J += dJ;

            // Record arc
            Arc arc;
            arc.type = arcType;
            arc.s0 = s;
            arc.s1 = s + dsArc;
            arc.t0 = t;
            arc.v0 = v;
            arc.a0 = a;
            arc.j0 = j;
            arc.sigma = sigma;
            arc.j_star = jStarEff;
            arc.duration = tau;
            arc.eta = sigma;
            arc.a_star = jStarEff;
            result.arcs.push_back(arc);

            // Advance
            s += dsArc;
            t += tau;
            v = v1;
            // Clamp acceleration and jerk to their limits (safety — arc
            // limiting should prevent this, but floating-point errors can
            // cause small overshoots)
            a = std::clamp(a1, -aMax, aMax);
            j = std::clamp(j1, -jMax, jMax);

            if (s < 0.0 || !std::isfinite(v) || !std::isfinite(s)) {
                infeasible = true;
                failReason = "Invalid state: s=" + std::to_string(s) +
                             " v=" + std::to_string(v);
                break;
            }

            // Stuck detection: if s didn't advance, the solver is stuck
            if (s - sPrev < 1e-14) {
                infeasible = true;
                failReason = "Stuck: s not advancing at s=" + std::to_string(s) +
                             " v=" + std::to_string(v);
                break;
            }
            sPrev = s;

            if (stoppedBeforeEnd) {
                J += 1e8 * (sEnd - s);
                infeasible = true;
                failReason = "Stopped before path end at s=" + std::to_string(s);
                break;
            }
        }

        // Feasibility
        result.cost = J;
        result.finalS = s;
        result.finalV = v;
        result.finalA = a;
        result.finalJ = j;
        result.totalTime = t;

        if (infeasible || s < sEnd - 1e-6) {
            result.feasible = false;
            result.failureReason = failReason;
        } else if (std::abs(v - vf) > 0.5) {
            result.feasible = false;
            result.failureReason = "Terminal velocity mismatch: v=" +
                std::to_string(v) + " vf=" + std::to_string(vf);
        } else {
            result.feasible = true;
        }

        return result;
    #endif
    }

private:
    struct PlanningBounds {
        double velocity = 0.0;
        double acceleration = 0.0;
        double jerk = 0.0;
        double snap = 0.0;
        std::string failureReason;

        bool feasible() const {
            return velocity >= 0.0 && acceleration > 0.0 && jerk > 0.0 &&
                   snap > 0.0 && failureReason.empty();
        }
    };

    struct AccelerationPulse {
        double snapTime = 0.0;
        double jerkTime = 0.0;
        double plateauTime = 0.0;
        double peakJerk = 0.0;
        double peakAcceleration = 0.0;
        double velocityChange = 0.0;

        double duration() const {
            return 4.0 * snapTime + 2.0 * jerkTime + plateauTime;
        }
    };

    /**
     * @brief Build conservative path-wide bounds from the full constraint
     * grid. Axis snap limits are intentionally rejected: a valid conversion
     * needs the fourth arc-length derivative of geometry, which the current
     * path interface does not expose. Path snap remains fully enforced.
     */
    PlanningBounds globalBounds(
        const std::function<double(double)>& vLimit,
        double sTotal) const {
        PlanningBounds bounds;
        if (limits_.axis.snapLimitEnabled) {
            bounds.failureReason =
                "Per-axis snap limits require fourth-order path derivatives";
            return bounds;
        }
        if (limits_.path.maxPathSnap <= T(0) ||
            limits_.path.maxPathJerk <= T(0) ||
            limits_.path.maxPathAcceleration <= T(0)) {
            bounds.failureReason =
                "SnapSpace requires positive path acceleration, jerk, and snap limits";
            return bounds;
        }

        bounds.velocity = std::numeric_limits<double>::infinity();
        bounds.acceleration = static_cast<double>(limits_.path.maxPathAcceleration);
        bounds.jerk = static_cast<double>(limits_.path.maxPathJerk);
        bounds.snap = static_cast<double>(limits_.path.maxPathSnap);

        const size_t samples = std::max<size_t>(constraintCacheSize_, 32);
        for (size_t i = 0; i <= samples; ++i) {
            const double s = sTotal * static_cast<double>(i) /
                             static_cast<double>(samples);
            const double v = vLimit(s);
            if (!std::isfinite(v) || v < 0.0) {
                bounds.failureReason = "Non-finite or negative velocity limit";
                return bounds;
            }
            bounds.velocity = std::min(bounds.velocity, v);
        }
        if (!(bounds.velocity > 0.0) || !std::isfinite(bounds.velocity)) {
            bounds.failureReason = "Path velocity envelope contains a stop";
            return bounds;
        }

        // Acceleration constraints depend on speed. A velocity ceiling due
        // solely to centripetal acceleration can leave zero room for
        // tangential acceleration, even though a strictly lower speed is
        // feasible. Find the greatest speed with positive symmetric
        // tangential authority rather than applying an arbitrary factor.
        const auto accelerationAtVelocity = [&](double velocity) {
            double available = static_cast<double>(limits_.path.maxPathAcceleration);
            for (size_t i = 0; i <= samples; ++i) {
                const T s = static_cast<T>(sTotal * static_cast<double>(i) /
                                            static_cast<double>(samples));
                const auto [aMin, aMax] = evaluator_.accelerationBounds(
                    s, static_cast<T>(velocity), path_);
                available = std::min(available, std::min(
                    static_cast<double>(aMax), -static_cast<double>(aMin)));
            }
            return available;
        };
        double aAtEnvelope = accelerationAtVelocity(bounds.velocity);
        if (aAtEnvelope <= 0.0) {
            if (accelerationAtVelocity(0.0) <= 0.0) {
                bounds.failureReason = "No symmetric tangential acceleration is feasible";
                return bounds;
            }
            double lo = 0.0;
            double hi = bounds.velocity;
            for (int iteration = 0; iteration < 100; ++iteration) {
                const double mid = 0.5 * (lo + hi);
                if (accelerationAtVelocity(mid) > 0.0) lo = mid;
                else hi = mid;
            }
            bounds.velocity = lo;
            aAtEnvelope = accelerationAtVelocity(bounds.velocity);
        }
        bounds.acceleration = std::min(bounds.acceleration, aAtEnvelope);
        if (!(bounds.acceleration > 0.0) || !std::isfinite(bounds.acceleration)) {
            bounds.failureReason = "No symmetric tangential acceleration is feasible";
            return bounds;
        }

        for (size_t i = 0; i <= samples; ++i) {
            const T s = static_cast<T>(sTotal * static_cast<double>(i) /
                                        static_cast<double>(samples));
            for (const double a : {-bounds.acceleration, 0.0,
                                   bounds.acceleration}) {
                const auto interval = evaluator_.etaBounds(
                    s, static_cast<T>(bounds.velocity), static_cast<T>(a), path_);
                const double symmetric = std::min(interval.eta_max,
                                                  -interval.eta_min);
                bounds.jerk = std::min(bounds.jerk, symmetric);
            }
        }
        if (!(bounds.jerk > 0.0) || !std::isfinite(bounds.jerk)) {
            bounds.failureReason = "No symmetric tangential jerk is feasible";
        }
        return bounds;
    }

    static AccelerationPulse makePulse(
        double velocityChange, const PlanningBounds& bounds) {
        AccelerationPulse pulse;
        pulse.velocityChange = std::max(0.0, velocityChange);
        if (pulse.velocityChange <= 1e-14) return pulse;

        // The snap-up/snap-down pair must not itself exceed a_max.
        const double maxJerk = std::min(
            bounds.jerk, std::sqrt(bounds.acceleration * bounds.snap));
        if (!(maxJerk > 0.0)) return pulse;

        const double vTriangularAtMaxJerk =
            maxJerk * maxJerk * maxJerk / (bounds.snap * bounds.snap);
        double peakJerk = maxJerk;
        if (pulse.velocityChange < vTriangularAtMaxJerk) {
            peakJerk = std::cbrt(pulse.velocityChange * bounds.snap *
                                 bounds.snap);
        }

        pulse.peakJerk = peakJerk;
        pulse.snapTime = peakJerk / bounds.snap;
        const double jerkPlateauAtAMax = std::max(
            0.0, bounds.acceleration / peakJerk - pulse.snapTime);
        const double velocityWithoutAccelPlateau = peakJerk *
            (pulse.snapTime * pulse.snapTime +
             3.0 * pulse.snapTime * jerkPlateauAtAMax +
             jerkPlateauAtAMax * jerkPlateauAtAMax);

        if (pulse.velocityChange <= velocityWithoutAccelPlateau) {
            const double discriminant = std::max(
                0.0, 5.0 * pulse.snapTime * pulse.snapTime +
                4.0 * pulse.velocityChange / peakJerk);
            pulse.jerkTime = std::max(
                0.0, 0.5 * (-3.0 * pulse.snapTime +
                            std::sqrt(discriminant)));
        } else {
            pulse.jerkTime = jerkPlateauAtAMax;
            pulse.peakAcceleration = peakJerk *
                (pulse.snapTime + pulse.jerkTime);
            pulse.plateauTime = (pulse.velocityChange -
                velocityWithoutAccelPlateau) / pulse.peakAcceleration;
        }
        pulse.peakAcceleration = peakJerk *
            (pulse.snapTime + pulse.jerkTime);
        return pulse;
    }

    static double minimumDistanceForPeak(
        double peakVelocity, double startVelocity, double endVelocity,
        const PlanningBounds& bounds) {
        const auto up = makePulse(peakVelocity - startVelocity, bounds);
        const auto down = makePulse(peakVelocity - endVelocity, bounds);
        return 0.5 * (startVelocity + peakVelocity) * up.duration() +
               0.5 * (endVelocity + peakVelocity) * down.duration();
    }

    static double snapArcCost(const CostWeights& w, double a0, double j0,
                              double sigma, double duration) {
        const double intJ2 = j0 * j0 * duration + j0 * sigma * duration * duration +
            sigma * sigma * duration * duration * duration / 3.0;
        const double intA2 = a0 * a0 * duration + a0 * j0 * duration * duration +
            (j0 * j0 + a0 * sigma) * duration * duration * duration / 3.0 +
            j0 * sigma * std::pow(duration, 4) / 4.0 +
            sigma * sigma * std::pow(duration, 5) / 20.0;
        return w.w_t * duration + w.w_j * intJ2 + w.w_a * intA2;
    }

    static double singularArcCost(const CostWeights& w, double a0, double jerk,
                                  double duration) {
        const double intJ2 = jerk * jerk * duration;
        const double intA2 = a0 * a0 * duration + a0 * jerk * duration * duration +
            jerk * jerk * duration * duration * duration / 3.0;
        return w.w_t * duration + w.w_j * intJ2 + w.w_a * intA2;
    }

    ForwardPassResult buildPulsePlan(
        const std::function<double(double)>& vLimit,
        double startVelocity, double endVelocity, double length,
        double requestedJerk, double limitScale) const {
        ForwardPassResult result;
        result.finalV = startVelocity;
        if (length <= 0.0) {
            result.feasible = std::abs(startVelocity - endVelocity) <= 1e-12 &&
                              std::abs(startVelocity) <= 1e-12;
            result.failureReason = result.feasible ? "" :
                "Zero-length path has incompatible boundary velocity";
            return result;
        }

        auto bounds = globalBounds(vLimit, length);
        bounds.acceleration *= limitScale;
        bounds.jerk *= limitScale;
        bounds.snap *= limitScale;
        if (requestedJerk > 0.0) bounds.jerk = std::min(bounds.jerk, requestedJerk);
        if (!bounds.feasible()) {
            result.failureReason = bounds.failureReason.empty()
                ? "Non-positive scaled snap-space limit" : bounds.failureReason;
            return result;
        }
        if (startVelocity < 0.0 || endVelocity < 0.0 ||
            startVelocity > bounds.velocity + 1e-10 ||
            endVelocity > bounds.velocity + 1e-10) {
            result.failureReason = "Boundary velocity lies outside the path envelope";
            return result;
        }

        const double minimumPeak = std::max(startVelocity, endVelocity);
        if (minimumDistanceForPeak(minimumPeak, startVelocity, endVelocity,
                                   bounds) > length + 1e-10) {
            result.failureReason = "Path is too short for the requested endpoint velocities";
            return result;
        }

        double lo = minimumPeak;
        double hi = bounds.velocity;
        for (int i = 0; i < 100; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (minimumDistanceForPeak(mid, startVelocity, endVelocity, bounds) <= length)
                lo = mid;
            else
                hi = mid;
        }
        const double peakVelocity = lo;
        const auto accelPulse = makePulse(peakVelocity - startVelocity, bounds);
        const auto decelPulse = makePulse(peakVelocity - endVelocity, bounds);
        const double pulseDistance = minimumDistanceForPeak(
            peakVelocity, startVelocity, endVelocity, bounds);
        const double cruiseTime = peakVelocity > 1e-14
            ? std::max(0.0, (length - pulseDistance) / peakVelocity) : 0.0;

        double s = 0.0;
        double t = 0.0;
        double v = startVelocity;
        double a = 0.0;
        double j = 0.0;
        auto normalize = [](double value) {
            return std::abs(value) < 1e-11 ? 0.0 : value;
        };
        auto appendSnap = [&](double sigma, double duration) {
            if (duration <= 1e-14) return;
            Arc arc;
            arc.type = sigma >= 0.0 ? WeightedArcType::SNAP_PLUS
                                    : WeightedArcType::SNAP_MINUS;
            arc.s0 = s; arc.t0 = t; arc.v0 = v; arc.a0 = a; arc.j0 = j;
            arc.sigma = sigma;
            arc.activeConstraints = 0x08; // snap control is saturated
            const double distance = SnapSeg::ds(v, a, j, sigma, duration);
            arc.duration = duration; arc.s1 = s + distance;
            arc.v1 = normalize(SnapSeg::v(v, a, j, sigma, duration));
            arc.a1 = normalize(SnapSeg::a(a, j, sigma, duration));
            arc.j1 = normalize(SnapSeg::j(j, sigma, duration));
            result.cost += snapArcCost(w_, a, j, sigma, duration);
            result.arcs.push_back(arc);
            s = arc.s1; t += duration; v = arc.v1; a = arc.a1; j = arc.j1;
        };
        auto appendSingular = [&](double jerk, double duration,
                      uint8_t activeConstraints = 0) {
            if (duration <= 1e-14) return;
            Arc arc;
            arc.type = WeightedArcType::SINGULAR;
            arc.s0 = s; arc.t0 = t; arc.v0 = v; arc.a0 = a; arc.j0 = j;
            arc.j_star = jerk;
            arc.a_star = jerk; // legacy name; value represents constant jerk
            arc.activeConstraints = activeConstraints;
            const double distance = SingularJSeg::ds(v, a, jerk, duration);
            arc.duration = duration; arc.s1 = s + distance;
            arc.v1 = normalize(SingularJSeg::v(v, a, jerk, duration));
            arc.a1 = normalize(SingularJSeg::a(a, jerk, duration));
            arc.j1 = normalize(jerk);
            result.cost += singularArcCost(w_, a, jerk, duration);
            result.arcs.push_back(arc);
            s = arc.s1; t += duration; v = arc.v1; a = arc.a1; j = arc.j1;
        };
        auto appendPulse = [&](const AccelerationPulse& pulse, double sign) {
            appendSnap(sign * bounds.snap, pulse.snapTime);
            appendSingular(sign * pulse.peakJerk, pulse.jerkTime, 0x04);
            appendSnap(-sign * bounds.snap, pulse.snapTime);
            appendSingular(0.0, pulse.plateauTime, 0x02);
            appendSnap(-sign * bounds.snap, pulse.snapTime);
            appendSingular(-sign * pulse.peakJerk, pulse.jerkTime, 0x04);
            appendSnap(sign * bounds.snap, pulse.snapTime);
        };

        appendPulse(accelPulse, 1.0);
        appendSingular(0.0, cruiseTime, 0x01);
        appendPulse(decelPulse, -1.0);

        result.finalS = s;
        result.finalV = v;
        result.finalA = a;
        result.finalJ = j;
        result.totalTime = t;
        const double sTolerance = 1e-8 * std::max(1.0, length);
        result.feasible = !result.arcs.empty() &&
            std::abs(s - length) <= sTolerance &&
            std::abs(v - endVelocity) <= 1e-9 &&
            std::abs(a) <= 1e-9 && std::abs(j) <= 1e-9;
        if (!result.feasible) {
            result.failureReason = "Exact snap-space terminal state was not reached";
        }
        return result;
    }

    ForwardPassResult optimizePulsePlan(
        const std::function<double(double)>& vLimit,
        double startVelocity, double endVelocity) const {
        ForwardPassResult best;
        best.failureReason = "No feasible snap-space pulse candidate";
        double bestCost = std::numeric_limits<double>::infinity();

        // The candidate family is deterministic and spans two decades of
        // smoothness. Every member is exactly propagated and independently
        // feasible; the selected point minimizes the stated weighted cost.
        constexpr size_t kCandidates = 41;
        for (size_t i = 0; i < kCandidates; ++i) {
            const double fraction = static_cast<double>(i) /
                                    static_cast<double>(kCandidates - 1);
            const double scale = std::pow(0.01, fraction);
            auto candidate = buildPulsePlan(vLimit, startVelocity, endVelocity,
                                            sTotal_, 0.0, scale);
            if (candidate.feasible && candidate.cost < bestCost) {
                bestCost = candidate.cost;
                best = std::move(candidate);
            }
        }
        return best;
    }

    const Path& path_;
    Limits limits_;
    CostWeights w_;
    T feedRate_;
    Evaluator evaluator_;

    double sTotal_ = 0.0;
    double v0_ = 0.0;
    double vf_ = 0.0;
    size_t constraintCacheSize_ = 200;
    double ds_ = 0.0;

    size_t fineGridSize_ = 0;
    double dsFine_ = 0.0;
    std::vector<double> vLimFineGrid_;

    std::vector<std::pair<double, double>> corners_;
    double aMaxForLookahead_ = 0.0;

    std::vector<KinematicCoefficients> gridCoeffs_;
    std::vector<double> vLimGrid_;

    std::vector<Arc> arcs_;
    double lastCost_ = 0.0;
    double lastJStar_ = 0.0;
    double lastS_ = 0.0;
    double lastV_ = 0.0;
    std::string lastFailure_;

    /**
     * @brief Estimate the maximum reachable jerk.
     */
    double estimateMaxReachableJerk() const {
        double jMax = static_cast<double>(limits_.path.maxPathJerk);
        if (jMax <= 0.0) jMax = 5000.0;  // fallback
        // Also consider snap limit: jerk can change at most sigmaMax per second,
        // so the effective jerk is also limited by how fast we can reach it.
        double sigmaMax = static_cast<double>(limits_.path.maxPathSnap);
        if (sigmaMax > 0.0 && sTotal_ > 0.0) {
            // Rough estimate: time to traverse path ≈ sTotal / vMax
            // Max jerk reachable in that time ≈ sigmaMax * time
            double vMax = static_cast<double>(limits_.path.maxPathVelocity);
            if (vMax > 0.0) {
                double tEst = sTotal_ / vMax;
                double jFromSnap = sigmaMax * tEst;
                jMax = std::min(jMax, jFromSnap);
            }
        }
        return std::max(jMax, 1e-6);
    }

    /// @deprecated Use estimateMaxReachableJerk() instead.
    double estimateMaxReachableAccel() const {
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (sTotal_ > 0.0) {
            T sMid = static_cast<T>(sTotal_ * 0.5);
            auto eval = path_.evaluateAtArcLength(sMid);
            T aAxis = limits_.maxAccelerationForDirection(
                eval.tangent, eval.curvature, T(0));
            aMax = std::min(aMax, static_cast<double>(aAxis));
        }
        return std::max(aMax, 1e-6);
    }

    /// Snapspace stopping distance from arbitrary state (v, a, j) to v=0.
    /// Optimal braking: σ=-σMax until j=-jMax, then j=-jMax until a=-aMax,
    /// then a=-aMax until v=0. Handles non-zero initial a and j.
    double sStopFromState(double v, double a, double j) const {
        if (v <= 0.0) return 0.0;
        double sigmaMax = static_cast<double>(limits_.path.maxPathSnap);
        if (sigmaMax <= 0.0) sigmaMax = 50000.0;
        double jMax = static_cast<double>(limits_.path.maxPathJerk);
        if (jMax <= 0.0) jMax = 5000.0;
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (aMax <= 0.0) aMax = 500.0;

        double sTotal = 0.0;
        double vCur = v, aCur = a, jCur = j;

        // Phase 1: snap σ=-σMax until j=-jMax (if j > -jMax)
        if (jCur > -jMax) {
            double tau1 = (jCur - (-jMax)) / sigmaMax;  // = (jCur + jMax) / sigmaMax
            if (tau1 < 0.0) tau1 = 0.0;
            // Propagate: j1 = jCur - sigmaMax*tau1, a1 = aCur + jCur*tau1 - sigmaMax*tau1²/2
            // v1 = vCur + aCur*tau1 + jCur*tau1²/2 - sigmaMax*tau1³/6
            // s1 = vCur*tau1 + aCur*tau1²/2 + jCur*tau1³/6 - sigmaMax*tau1⁴/24
            double v1 = vCur + aCur * tau1 + jCur * tau1 * tau1 / 2.0
                        - sigmaMax * std::pow(tau1, 3) / 6.0;
            double s1 = vCur * tau1 + aCur * tau1 * tau1 / 2.0
                        + jCur * std::pow(tau1, 3) / 6.0
                        - sigmaMax * std::pow(tau1, 4) / 24.0;
            sTotal += s1;
            if (v1 <= 0.0) return std::max(sTotal, 0.0);
            vCur = v1;
            aCur = aCur + jCur * tau1 - sigmaMax * tau1 * tau1 / 2.0;
            jCur = jCur - sigmaMax * tau1;  // = -jMax
        }

        // Phase 2: jerk j=-jMax until a=-aMax (if a > -aMax)
        if (aCur > -aMax) {
            double tau2 = (aCur - (-aMax)) / jMax;  // = (aCur + aMax) / jMax
            if (tau2 < 0.0) tau2 = 0.0;
            double v2 = vCur + aCur * tau2 - jMax * tau2 * tau2 / 2.0;
            double s2 = vCur * tau2 + aCur * tau2 * tau2 / 2.0
                        - jMax * std::pow(tau2, 3) / 6.0;
            sTotal += s2;
            if (v2 <= 0.0) {
                // v reaches 0 during phase 2. Solve: vCur + aCur*t - jMax*t²/2 = 0
                double disc = aCur * aCur + 2.0 * jMax * vCur;
                if (disc < 0.0) return sTotal - s2;  // shouldn't happen
                double tStop = (aCur + std::sqrt(disc)) / jMax;
                if (tStop < 0.0 || tStop > tau2) tStop = (aCur - std::sqrt(disc)) / jMax;
                double s2corr = vCur * tStop + aCur * tStop * tStop / 2.0
                                - jMax * std::pow(tStop, 3) / 6.0;
                return std::max(sTotal - s2 + s2corr, 0.0);
            }
            vCur = v2;
            aCur = -aMax;
            // jCur is already -jMax
        }

        // Phase 3: constant a=-aMax until v=0
        if (vCur > 0.0) {
            double s3 = vCur * vCur / (2.0 * aMax);
            sTotal += s3;
        }

        return std::max(sTotal, 0.0);
    }

    /// Snapspace stopping distance from state (v, a=0, j=0) to (0, 0, 0).
    /// Convenience wrapper around sStopFromState.
    double sStopFromRest(double v) const {
        return sStopFromState(v, 0.0, 0.0);
    }

    /// Inverse of sStopFromRest: given a distance d, find the max v such that
    /// sStopFromRest(v) <= d. Used in the backward pass.
    /// By time-reversal symmetry, this is the same as the maximum velocity
    /// reachable by accelerating from rest over distance d.
    double sStopFromRestInverse(double d) const {
        if (d <= 0.0) return 0.0;
        // Bisection: find max v such that sStopFromRest(v) <= d
        double lo = 0.0, hi = 1e6;
        for (int b = 0; b < 60; ++b) {
            double mid = 0.5 * (lo + hi);
            if (sStopFromRest(mid) <= d) lo = mid; else hi = mid;
            if (hi - lo < 1e-6) break;
        }
        return lo;
    }

    /// Inverse of sStopFromState with worst-case (aMax, jMax): given a
    /// distance d, find the max v such that
    /// sStopFromState(v, aMax, jMax) - sStopFromRest(vCorner) <= d.
    /// This accounts for the extra braking distance when the solver has
    /// non-zero (a, j) at the switching point.
    double sStopFromStateInverse(double d, double vCorner = 0.0) const {
        if (d <= 0.0) return 0.0;
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (aMax <= 0.0) aMax = 500.0;
        double jMax = static_cast<double>(limits_.path.maxPathJerk);
        if (jMax <= 0.0) jMax = 5000.0;
        double sStopCorner = sStopFromRest(vCorner);
        double targetDist = d + sStopCorner;
        // Bisection: find max v such that sStopFromState(v, aMax, jMax) <= targetDist
        double lo = 0.0, hi = 1e6;
        for (int b = 0; b < 60; ++b) {
            double mid = 0.5 * (lo + hi);
            if (sStopFromState(mid, aMax, jMax) <= targetDist) lo = mid; else hi = mid;
            if (hi - lo < 1e-6) break;
        }
        return lo;
    }

    /// Snapspace-aware braking distance from state (v, a, j) to vCorner.
    /// This is sStopFromState(v, a, j) - sStopFromRest(vCorner), which equals
    /// the distance needed to decelerate from (v, a, j) to (vCorner, 0, 0).
    double brakeDistanceFromState(double v, double a, double j, double vCorner) const {
        return sStopFromState(v, a, j) - sStopFromRest(vCorner);
    }

    /**
     * @brief Precompute the path geometry and velocity limit on the solver grid.
     *
     * This is the dominant cost for NURBS/arc paths: each constraint query
     * otherwise inverts arc length and evaluates high-order NURBS derivatives.
     * Caching once per solve makes every simulateAndCost call
     * O(constraintCacheSize) instead of O(constraintCacheSize * pathEvalCost).
     *
     * When the path has per-segment velocity limits (feed rates + corner
     * velocities), a fine velocity-limit grid is built in addition to the
     * coarse coefficient grid. The fine grid captures per-segment feed rate
     * transitions and corner velocity dips. Backward and forward velocity
     * limit propagation ensures the solver decelerates before corners and
     * accelerates after them, given the acceleration constraints.
     */
    void buildConstraintCache() {
        if (sTotal_ <= 0.0 || constraintCacheSize_ == 0) return;

        // --- Coarse grid: geometric coefficients (tangent, curvature, etc.) ---
        // This is the expensive part (NURBS evaluation per point).
        ds_ = sTotal_ / static_cast<double>(constraintCacheSize_);
        gridCoeffs_.resize(constraintCacheSize_ + 1);
        vLimGrid_.resize(constraintCacheSize_ + 1);

        for (size_t i = 0; i <= constraintCacheSize_; ++i) {
            double s = std::min(static_cast<double>(i) * ds_, sTotal_);
            gridCoeffs_[i] = evaluator_.computeCoefficients(
                static_cast<T>(s), T(0), T(0), path_);

            // Velocity limit from the cached coefficients (path-level + curvature
            // + per-axis directional limit).
            double kappa = gridCoeffs_[i].kappa;
            double vLim = static_cast<double>(
                std::min(feedRate_, limits_.path.maxPathVelocity));
            if (kappa > static_cast<double>(MathConstants::EPSILON)) {
                double vCurvature = std::sqrt(
                    static_cast<double>(limits_.path.maxCentripetalAcceleration)
                    / kappa);
                vLim = std::min(vLim, vCurvature);
            }
            auto maxVel = limits_.maxVelocityForDirection(
                toVec(gridCoeffs_[i].tangent));
            vLim = std::min(vLim, static_cast<double>(maxVel));
            vLimGrid_[i] = std::max(vLim, 0.0);
        }

        // --- Fine grid: velocity limit with per-segment constraints ---
        buildFineVelocityGrid();

        // Detect corners from the fine grid vLim profile if none were
        // found from explicit corner velocities. Points where vLim drops
        // significantly are treated as corners for look-ahead planning.
        if (corners_.empty()) {
            for (size_t i = 2; i + 2 < vLimFineGrid_.size(); ++i) {
                double vPrev = vLimFineGrid_[i - 1];
                double vCur  = vLimFineGrid_[i];
                double vNext = vLimFineGrid_[i + 1];
                if (vCur < vPrev - 0.1 && vCur < vNext - 0.1) {
                    double s = static_cast<double>(i) * dsFine_;
                    corners_.emplace_back(s, vCur);
                }
            }
            // Also detect significant drops
            for (size_t i = 1; i < vLimFineGrid_.size(); ++i) {
                double vPrev = vLimFineGrid_[i - 1];
                double vCur  = vLimFineGrid_[i];
                if (vPrev > 0 && vCur < vPrev * 0.7) {
                    double s = static_cast<double>(i) * dsFine_;
                    bool found = false;
                    for (const auto& [sC, vC] : corners_) {
                        if (std::abs(sC - s) < dsFine_ * 2) { found = true; break; }
                    }
                    if (!found) corners_.emplace_back(s, vCur);
                }
            }
            std::sort(corners_.begin(), corners_.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
        }

        // Always ensure the terminal point (sEnd, vf_) is in corners_ so
        // the look-ahead TOPPRA backward pass creates a deceleration profile
        // toward the end. Also ensure aMaxForLookahead_ is set.
        if (aMaxForLookahead_ <= 0.0) {
            aMaxForLookahead_ = static_cast<double>(limits_.path.maxPathAcceleration);
            if (aMaxForLookahead_ <= 0.0) aMaxForLookahead_ = 500.0;
            // The look-ahead now uses a snapspace-aware formula that accounts
            // for jerk/snap ramp time, so we use the full aMax here.
        }
        // Add terminal point as a corner if not already present
        bool hasTerminal = false;
        for (const auto& [sC, vC] : corners_) {
            if (std::abs(sC - sTotal_) < 1e-6) { hasTerminal = true; break; }
        }
        if (!hasTerminal) {
            corners_.emplace_back(sTotal_, vf_);
            std::sort(corners_.begin(), corners_.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
        }
    }

    /**
     * @brief Build the fine velocity-limit grid.
     *
     * If the path has per-segment velocity limits, the fine grid is enlarged
     * to capture per-segment feed rate transitions and corner velocity dips.
     * The geometric velocity limit is interpolated from the coarse grid.
     * Backward and forward propagation ensures the velocity limit accounts
     * for the machine's acceleration/deceleration capabilities.
     *
     * If no per-segment limits are present, the fine grid is identical to
     * the coarse grid (backward compatible).
     */
    void buildFineVelocityGrid() {
        const bool hasPerSeg = path_.hasPerSegmentVelocityLimits();

        if (!hasPerSeg) {
            // Backward compatible: fine grid = coarse grid.
            fineGridSize_ = constraintCacheSize_;
            dsFine_ = ds_;
            vLimFineGrid_ = vLimGrid_;  // copy
            return;
        }

        // Build a fine velocity-limit grid with per-segment feed rates,
        // corner velocities, and backward/forward propagation.
        //
        // The solver queries this fine grid for velocity limits, but uses
        // the coarse grid step size for the simulation. The backward/forward
        // propagation on the fine grid ensures the velocity limit profile
        // is achievable given the acceleration constraints, creating a
        // smooth deceleration/acceleration profile around corners.
        const size_t numSegs = path_.numSegments();
        const size_t minFine = numSegs * 2;
        fineGridSize_ = std::max(constraintCacheSize_, minFine);
        const size_t kMaxFine = 500000;
        fineGridSize_ = std::min(fineGridSize_, kMaxFine);

        dsFine_ = sTotal_ / static_cast<double>(fineGridSize_);
        vLimFineGrid_.resize(fineGridSize_ + 1);

        // Fill the fine grid: interpolate geometric vLim from coarse grid
        // and apply per-segment velocity limits (feed rate + corner velocity).
        for (size_t i = 0; i <= fineGridSize_; ++i) {
            double s = std::min(static_cast<double>(i) * dsFine_, sTotal_);
            double geoVLim = interpolateCoarseVLim(s);
            double segVLim = path_.maxVelocityAtArcLength(static_cast<T>(s));
            vLimFineGrid_[i] = std::max(std::min(geoVLim, segVLim), 0.0);
        }

        // Direct corner constraint stamping on the fine grid.
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (aMax <= 0.0) aMax = 1.0;
        aMaxForLookahead_ = aMax;  // Cache for look-ahead in simulateAndCost

        const double vMaxGlobal = static_cast<double>(
            std::min(feedRate_, limits_.path.maxPathVelocity));
        const double maxStampDist = vMaxGlobal * vMaxGlobal / (2.0 * aMax);
        const int stampRadius = std::max(1,
            (int)std::ceil(maxStampDist / dsFine_));

        const auto& cornerVel = path_.cornerVelocities();
        corners_.clear();
        if (!cornerVel.empty()) {
            const auto& segs = path_.segments();
            for (size_t i = 0; i < cornerVel.size(); ++i) {
                if (!std::isfinite(cornerVel[i])) continue;
                if (i == 0 || i + 1 >= cornerVel.size()) continue;
                double s;
                if (i < segs.size()) {
                    s = static_cast<double>(segs[i].cumulativeArcLength);
                } else {
                    s = sTotal_;
                }
                corners_.emplace_back(s, cornerVel[i]);
            }
        }

        // Also detect corners from the fine grid vLim profile.
        // Points where vLim drops significantly are treated as corners
        // for the look-ahead forward planning.
        if (corners_.empty() || corners_.size() <= 1) {
            // Scan the fine grid for local minima in vLim
            for (size_t i = 2; i + 2 < vLimFineGrid_.size(); ++i) {
                double vPrev = vLimFineGrid_[i - 1];
                double vCur  = vLimFineGrid_[i];
                double vNext = vLimFineGrid_[i + 1];
                // Detect local minimum: vCur < vPrev and vCur < vNext
                if (vCur < vPrev - 0.1 && vCur < vNext - 0.1) {
                    double s = static_cast<double>(i) * dsFine_;
                    corners_.emplace_back(s, vCur);
                }
            }
            // Also detect significant drops (not just local minima)
            for (size_t i = 1; i < vLimFineGrid_.size(); ++i) {
                double vPrev = vLimFineGrid_[i - 1];
                double vCur  = vLimFineGrid_[i];
                if (vPrev > 0 && vCur < vPrev * 0.7) {
                    double s = static_cast<double>(i) * dsFine_;
                    // Check not already added
                    bool found = false;
                    for (const auto& [sC, vC] : corners_) {
                        if (std::abs(sC - s) < dsFine_ * 2) { found = true; break; }
                    }
                    if (!found) corners_.emplace_back(s, vCur);
                }
            }
            std::sort(corners_.begin(), corners_.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
        }

        // corners_ is already sorted by arc length (segments are ordered).

        for (const auto& [sCorner, vCorner] : corners_) {
            if (vCorner >= vMaxGlobal && vCorner > 0) continue;
            int centerIdx = (int)(sCorner / dsFine_ + 0.5);
            int lo = std::max(0, centerIdx - stampRadius);
            int hi = std::min((int)fineGridSize_, centerIdx + stampRadius);
            for (int i = lo; i <= hi; ++i) {
                double s = static_cast<double>(i) * dsFine_;
                double d = std::abs(s - sCorner);
                double vMax = std::sqrt(vCorner * vCorner + 2.0 * aMax * d);
                vLimFineGrid_[i] = std::min(vLimFineGrid_[i], vMax);
            }
        }

        // Also update the coarse grid with the min-filtered fine grid values
        // so that the coarse grid is consistent.
        for (size_t i = 0; i <= constraintCacheSize_; ++i) {
            double sCoarse = static_cast<double>(i) * ds_;
            size_t iFineLo = (size_t)(sCoarse / dsFine_);
            size_t iFineHi = (size_t)((sCoarse + ds_) / dsFine_);
            if (iFineHi > fineGridSize_) iFineHi = fineGridSize_;
            if (iFineLo > fineGridSize_) iFineLo = fineGridSize_;
            double vMin = vLimFineGrid_[iFineLo];
            for (size_t j = iFineLo + 1; j <= iFineHi; ++j) {
                if (vLimFineGrid_[j] < vMin) vMin = vLimFineGrid_[j];
            }
            vLimGrid_[i] = std::min(vLimGrid_[i], vMin);
        }

        // Apply backward/forward propagation on the FINE grid.
        //
        // This propagates the geometric vLim (curvature, feed rate, axis
        // limits) to ensure the solver can decelerate/accelerate between
        // grid points. Uses the bang-bang formula (v² + 2*aMax*ds) because:
        //   - The grid step is small, so snap/jerk ramp is negligible
        //   - The snapspace braking constraint is handled analytically
        //     by lookAheadVLimit(), which does the backward pass from
        //     actual corners using the exact sStop function.
        //
        // Backward: vLim[i] = min(vLim[i], sqrt(vLim[i+1]^2 + 2*aMax*dsFine))
        for (size_t i = fineGridSize_; i > 0; --i) {
            double vNext = vLimFineGrid_[i];
            double vProp = std::sqrt(vNext * vNext + 2.0 * aMax * dsFine_);
            vLimFineGrid_[i - 1] = std::min(vLimFineGrid_[i - 1], vProp);
        }
        // Forward: vLim[i] = min(vLim[i], sqrt(vLim[i-1]^2 + 2*aMax*dsFine))
        for (size_t i = 1; i <= fineGridSize_; ++i) {
            double vPrev = vLimFineGrid_[i - 1];
            double vProp = std::sqrt(vPrev * vPrev + 2.0 * aMax * dsFine_);
            vLimFineGrid_[i] = std::min(vLimFineGrid_[i], vProp);
        }

        // Re-update the coarse grid from the propagated fine grid.
        for (size_t i = 0; i <= constraintCacheSize_; ++i) {
            double sCoarse = static_cast<double>(i) * ds_;
            size_t iFineLo = (size_t)(sCoarse / dsFine_);
            size_t iFineHi = (size_t)((sCoarse + ds_) / dsFine_);
            if (iFineHi > fineGridSize_) iFineHi = fineGridSize_;
            if (iFineLo > fineGridSize_) iFineLo = fineGridSize_;
            double vMin = vLimFineGrid_[iFineLo];
            for (size_t j = iFineLo + 1; j <= iFineHi; ++j) {
                if (vLimFineGrid_[j] < vMin) vMin = vLimFineGrid_[j];
            }
            vLimGrid_[i] = std::min(vLimGrid_[i], vMin);
        }

        // Note: The analytical TOPPRA backward pass (look-ahead from exact
        // corner positions) is NOT precomputed into the coarse grid. Instead,
        // it is computed on-the-fly in simulateAndCost() via lookAheadVLimit().
        // This is the continuous-form TOPPRA backward pass:
        //   v_req(s) = min over upcoming corners of sqrt(v_c² + 2*a_max*d)
        // It works regardless of grid resolution, which is critical for large
        // paths where the grid step is much larger than the braking distance.
    }

    /// Interpolate the geometric velocity limit from the coarse grid.
    double interpolateCoarseVLim(double s) const {
        if (constraintCacheSize_ == 0 || ds_ <= 0.0) {
            return static_cast<double>(
                std::min(feedRate_, limits_.path.maxPathVelocity));
        }
        double idxF = s / ds_;
        size_t idx0 = static_cast<size_t>(std::floor(idxF));
        size_t idx1 = idx0 + 1;
        if (idx0 >= constraintCacheSize_) return vLimGrid_.back();
        if (idx1 > constraintCacheSize_) idx1 = constraintCacheSize_;
        double frac = idxF - static_cast<double>(idx0);
        return vLimGrid_[idx0] * (1.0 - frac) + vLimGrid_[idx1] * frac;
    }

    /// Fine grid index for arc length s (uniform grid, O(1)).
    size_t fineGridIndex(double s) const {
        if (dsFine_ <= 0.0) return 0;
        long idx = static_cast<long>(std::floor(s / dsFine_ + 0.5));
        return static_cast<size_t>(
            std::clamp(idx, 0L, static_cast<long>(fineGridSize_)));
    }

    /// Analytical TOPPRA backward pass (look-ahead): computes the maximum
    /// velocity at position s such that the tool can decelerate to the
    /// corner velocity by the time it reaches each upcoming corner.
    ///
    /// For each corner at (s_c, v_c) ahead of s, the required velocity at s
    /// is: v_req = sqrt(v_c² + 2 * a_max * (s_c - s)).
    ///
    /// This is the continuous form of the TOPPRA backward pass, evaluated at
    /// the exact corner position. It works regardless of grid resolution,
    /// which is critical for large paths where the grid step can be much
    /// larger than the braking distance.
    ///
    /// @param s Current arc length position
    /// @param vGridLim Velocity limit from the fine/coarse grid at s
    /// @return The effective velocity limit (min of grid and look-ahead)
    double lookAheadVLimit(double s, double vGridLim) const {
        if (corners_.empty() || aMaxForLookahead_ <= 0.0) return vGridLim;

        // Binary search for the first corner at or after s.
        auto it = std::lower_bound(corners_.begin(), corners_.end(), s,
            [](const std::pair<double, double>& c, double val) {
                return c.first < val;
            });

        // Check upcoming corners within braking distance.
        //
        // For intermediate corners (vCorner > 0), use the constant-acceleration
        // TOPPRA formula: v_req = sqrt(v_c² + 2·aMax·d). This is slightly
        // aggressive (ignores jerk/snap ramp time) but works well in practice
        // because the solver's v_lim tracking logic handles the extra distance.
        //
        // For intermediate corners, use the constant-acceleration TOPPRA
        // formula: v_req = sqrt(v_c² + 2·aMax·d).
        //
        // For the terminal corner (vCorner ≈ 0, rest-to-stop), use
        // sStopFromRestInverse(d) with a safety factor. The snapspace
        // braking distance from v to 0 is significantly larger than
        // v²/(2·aMax) because the solver must first reverse jerk and
        // acceleration. The predictive braking safety net in the forward
        // pass also handles this, but having the v_lim profile start
        // dropping earlier prevents the solver from accelerating too long.
        double vEff = vGridLim;
        double dMax = vGridLim * vGridLim / (2.0 * aMaxForLookahead_);

        for (; it != corners_.end(); ++it) {
            double sCorner = it->first;
            double vCorner = it->second;
            double d = sCorner - s;
            if (d < 0.0) continue;

            double vReq;
            if (vCorner < 1e-3) {
                // Terminal corner: use snapspace formula with safety factor.
                // Don't skip based on dMax — the snapspace braking distance
                // can exceed the TOPPRA dMax.
                vReq = sStopFromRestInverse(d) * 0.7;
            } else {
                // Intermediate corner: use TOPPRA formula
                if (d > dMax) break;
                vReq = std::sqrt(vCorner * vCorner + 2.0 * aMaxForLookahead_ * d);
            }
            if (vReq < vEff) vEff = vReq;
        }

        return vEff;
    }

    /// Convert a std::vector<double> to Vec<Dim,T> for limits helpers.
    Vec<Dim, T> toVec(const std::vector<double>& v) const {
        Vec<Dim, T> result;
        for (size_t i = 0; i < Dim && i < v.size(); ++i) {
            result[i] = static_cast<T>(v[i]);
        }
        return result;
    }

    size_t gridIndex(double s) const {
        if (ds_ <= 0.0) return 0;
        long idx = static_cast<long>(std::floor(s / ds_ + 0.5));
        return static_cast<size_t>(
            std::clamp(idx, 0L, static_cast<long>(constraintCacheSize_)));
    }

    /// Compute acceleration bounds [a_min, a_max] from cached grid coefficients.
    std::pair<double, double> accelBoundsFromCache(size_t idx, double v) const {
        const auto& c = gridCoeffs_[idx];
        double a_min = -static_cast<double>(limits_.path.maxPathAcceleration);
        double a_max =  static_cast<double>(limits_.path.maxPathAcceleration);

        double v2 = v * v;
        for (size_t i = 0; i < Dim && i < c.tangent.size(); ++i) {
            double Ti = c.tangent[i];
            double kappai_v2 = c.curvature[i] * v2;
            double axMaxI = static_cast<double>(limits_.axis.maxAcceleration[i]);

            if (std::abs(Ti) > static_cast<double>(MathConstants::EPSILON)) {
                double a_lo = (-axMaxI - kappai_v2) / Ti;
                double a_hi = ( axMaxI - kappai_v2) / Ti;
                if (a_lo > a_hi) std::swap(a_lo, a_hi);
                a_min = std::max(a_min, a_lo);
                a_max = std::min(a_max, a_hi);
            } else {
                if (std::abs(kappai_v2) > axMaxI) {
                    return {1.0, -1.0};  // infeasible
                }
            }
        }
        return {a_min, a_max};
    }

    /// Compute eta bounds [eta_min, eta_max] from cached grid coefficients.
    EtaBounds etaBoundsFromCache(size_t idx, double v, double a) const {
        EtaBounds b;
        b.eta_min = -static_cast<double>(limits_.path.maxPathJerk);
        b.eta_max =  static_cast<double>(limits_.path.maxPathJerk);

        // If jerk limiting is disabled the evaluator returns huge bounds.
        // Cap to a finite effective jerk to keep the arc time scale healthy.
        if (!limits_.path.jerkLimitEnabled) {
            b.eta_min = -1e18;
            b.eta_max =  1e18;
        }

        double jEff = std::max(
            static_cast<double>(limits_.path.maxPathJerk),
            1000.0 * static_cast<double>(limits_.path.maxPathAcceleration));
        if (jEff > 0.0) {
            b.eta_min = std::max(b.eta_min, -jEff);
            b.eta_max = std::min(b.eta_max,  jEff);
        }

        const auto& c = gridCoeffs_[idx];
        double v_d = v;
        double a_d = a;
        double v3 = v_d * v_d * v_d;
        double va = v_d * a_d;

        for (size_t i = 0; i < Dim && i < c.tangent.size(); ++i) {
            double alpha = c.tangent[i];
            double beta = c.jounce[i] * v3 + 3.0 * c.curvature[i] * va;
            double jMaxI = static_cast<double>(limits_.axis.maxJerk[i]);

            if (!limits_.axis.jerkLimitEnabled || jMaxI <= 0.0) continue;

            if (std::abs(alpha) > static_cast<double>(MathConstants::EPSILON)) {
                double eta_lo = (-jMaxI - beta) / alpha;
                double eta_hi = ( jMaxI - beta) / alpha;
                if (eta_lo > eta_hi) std::swap(eta_lo, eta_hi);
                b.eta_min = std::max(b.eta_min, eta_lo);
                b.eta_max = std::min(b.eta_max, eta_hi);
            } else {
                if (std::abs(beta) > jMaxI) {
                    b.eta_min = 1.0;
                    b.eta_max = -1.0;
                    return b;
                }
            }
        }

        return b;
    }

    /**
     * @brief Forward state-machine simulation for fixed j* (snapspace).
     *
     * Walks the path from s=0 to s=s_f with 4D state (s, v, a, j),
     * selecting snap control at each step based on the j* guidance law.
     * Produces a sequence of SNAP and SINGULAR arcs. Computes the cost
     * J = ∫(w_t + w_j·j² + w_a·a²)dt in closed form.
     *
     * @param jStar The singular jerk level
     * @param record If true, store the arcs in arcs_
     * @return The total cost J
     */
#if 0 // Replaced by buildPulsePlan(); retained temporarily for algorithm history.
    double simulateAndCost(double jStar, bool record) {
        std::vector<Arc> tmp;
        double s = 0.0, t = 0.0;
        double v = v0_, a = 0.0, j = 0.0;
        double J = 0.0;

        const double dsStep = ds_;
        const double sEnd = sTotal_;

        double sigmaMax = static_cast<double>(limits_.path.maxPathSnap);
        if (sigmaMax <= 0.0) sigmaMax = 50000.0;
        double jMax = static_cast<double>(limits_.path.maxPathJerk);
        if (jMax <= 0.0) jMax = 5000.0;
        double aMax = static_cast<double>(limits_.path.maxPathAcceleration);
        if (aMax <= 0.0) aMax = 500.0;

        // v_lim query: backward-pass result (look-ahead from corners)
        auto vLimAt = [&](double sCur) -> double {
            double vGrid = vLimFineGrid_[fineGridIndex(sCur)];
            return lookAheadVLimit(sCur, vGrid);
        };

        // ── Forward pass (TOPP-RA) ──────────────────────────────────
        // At each grid step:
        //   1. vLim_now  = vLimAt(s)
        //   2. vLim_next = vLimAt(s + ds)
        //   3. a_max = min(aBoundMax, (vLim_next² - v²) / (2·ds))   ← don't overshoot
        //      a_min = max(aBoundMin, ...)                            ← don't undershoot
        //   4. a_des = choose based on j*:
        //        - v < vLim - margin → ramp jerk toward j*, let a build up
        //        - v ≈ vLim           → track vLim slope
        //        - v > vLim            → a_des = a_min (decelerate)
        //   5. j_des = (a_des - a) / dt
        //   6. σ_des = (j_des - j) / dt   (clamped to ±σMax)
        //   7. Propagate (s,v,a,j) by dt

        bool infeasible = false;
        int maxIter = static_cast<int>(constraintCacheSize_) * 200;
        double sPrevStuck = -1.0;
        int stuckCount = 0;

        for (int iter = 0; iter < maxIter && s < sEnd - 1e-10; ++iter) {
            // If v has dropped to ~0 after starting, the solver can't
            // continue (would need to re-accelerate from rest). Mark as
            // infeasible. Skip this check at the very start (iter==0)
            // since v0 may legitimately be 0.
            // Also skip if we're near a corner position — the solver may
            // legitimately have v≈0 at a corner with vC=0.
            if (iter > 0 && v < 1e-6 && s < sEnd - 1e-6) {
                // Check if we're near a corner with vC > 0 — if so, the solver
                // should be able to re-accelerate from the corner velocity.
                bool nearCorner = false;
                for (const auto& [sC, vC] : corners_) {
                    if (std::abs(sC - s) < dsStep * 2.0 && vC > 1e-3) {
                        nearCorner = true;
                        break;
                    }
                }
                if (!nearCorner) {
                    infeasible = true; break;
                }
            }

            auto aBounds = accelBoundsFromCache(gridIndex(s), v);
            if (aBounds.first > aBounds.second) {
                infeasible = true; break;
            }
            double aBoundMin = static_cast<double>(aBounds.first);
            double aBoundMax = static_cast<double>(aBounds.second);

            double vLimNow  = vLimAt(s);
            double vLimNext = vLimAt(std::min(s + dsStep, sEnd));

            // Acceleration that would bring v to vLimNext after ds:
            // v_next² = v² + 2·a·ds  →  a = (vLimNext² - v²) / (2·ds)
            double aTrack = (vLimNext * vLimNext - v * v) / (2.0 * dsStep);

            // a_max: don't accelerate more than needed to stay at/below vLim
            double aHi = std::min(aBoundMax, aTrack);
            // a_min: physical lower bound (can always decelerate)
            double aLo = aBoundMin;
            // If aHi < aLo, vLim is dropping faster than we can decelerate.
            if (aHi < aLo) aHi = aLo;

            // Choose desired acceleration based on v vs vLim.
            // The vLim profile from the backward pass encodes the braking
            // distance from rest (a=0, j=0), but the solver may have non-zero
            // (a, j) when braking begins. As a safety net, also check the
            // actual snapspace stopping distance from the current state.
            // If sStopFromState(v, a, j) exceeds the remaining distance to
            // the terminal corner, brake immediately regardless of v_lim.
            double a_des;
            double dtEst = dsStep / std::max(v, 1e-3);
            double margin = aMax * dtEst + 1e-3;

            // Predictive braking: check if we need to start braking now
            // to reach the nearest upcoming corner at the right velocity.
            // This catches cases where the v_lim profile (based on
            // sStopFromRest) is too aggressive.
            // We only check the NEAREST upcoming corner, not all corners,
            // because the solver needs to pass intermediate corners before
            // braking for the terminal one.
            // Skip predictive braking if the solver is already braking
            // (a < 0 and j < 0) — the v_lim tracking logic handles it.
            bool mustBrake = false;
            if (v > 1e-3 && (a > 0.0 || j > 0.0)) {
                double sStopCurrent = sStopFromState(v, a, j);
                for (const auto& [sC, vC] : corners_) {
                    if (sC <= s + 1e-6) continue;
                    // Found the nearest upcoming corner
                    double d = sC - s;
                    double sStopCorner = sStopFromRest(vC);
                    double decelDist = sStopCurrent - sStopCorner;
                    if (decelDist > d - 1e-6) {
                        mustBrake = true;
                    }
                    break;  // only check the nearest corner
                }
            }

            if (mustBrake || v > vLimNow + 1e-6) {
                // Must brake — decelerate as hard as possible
                a_des = aLo;
            } else if (v >= vLimNow - margin) {
                // At vLim — track it (follow the vLim slope)
                a_des = std::clamp(aTrack, aLo, aHi);
            } else {
                // Below vLim — accelerate using j* as the target jerk.
                // Don't clamp to aHi here — aHi may be very negative if
                // v_lim is dropping fast (e.g. after a corner with a
                // terminal corner ahead). The solver needs to accelerate
                // to re-gain speed after passing a corner, even if v_lim
                // is low. The v_lim tracking will catch up when v approaches
                // v_lim.
                double aFromJ = a + jStar * dtEst;
                a_des = std::max(aFromJ, aLo);
                // Only clamp to aLo (physical lower bound), not aHi
                // (which may be artificially low from v_lim tracking)
            }

            // Desired jerk to reach a_des. When accelerating from rest
            // (v < vLim, a < aHi), use j* as the target jerk. Otherwise
            // compute from a_des/dtEst. The dtEst clamp at 1.0s prevents
            // the v→0 singularity from making j_des → 0.
            // When mustBrake, use -jMax to reverse acceleration as fast as
            // possible (don't derive from a_des/dt which may be too slow).
            double dtEff = std::min(dtEst, 1.0);
            double j_des;
            if (mustBrake) {
                // Predictive braking: ramp jerk to -jMax immediately
                j_des = -jMax;
            } else if (v < vLimNow - margin && a < aHi - 1e-6 && a_des > 0.0) {
                // Acceleration phase: target jerk is j*
                j_des = std::clamp(jStar, -jMax, jMax);
            } else {
                j_des = (a_des - a) / std::max(dtEff, 1e-6);
                j_des = std::clamp(j_des, -jMax, jMax);
            }

            // Desired snap: bang-bang in snap (always use ±sigmaMax to
            // ramp jerk as fast as possible). The arc limiting logic
            // below shortens the arc when jerk reaches j_des, so the
            // SNAP arc covers exactly the distance needed to ramp.
            // When jerk is already at j_des, use sigma=0 (SINGULAR).
            double sigmaDes;
            double jErr = j_des - j;
            if (std::abs(jErr) < 0.5) {
                sigmaDes = 0.0;  // jerk is at target — SINGULAR
            } else {
                sigmaDes = std::copysign(sigmaMax, jErr);
            }

            // Determine arc type
            WeightedArcType arcType;
            double sigma;
            double jStarEff = j_des;

            if (v > 1e-6 && v >= vLimNow - margin && v <= vLimNow + 1e-6
                && std::abs(a) < 2.0 && std::abs(j) < 2.0
                && std::abs(sigmaDes) < 1.0) {
                // Cruise at vLim (only when v > 0)
                arcType = WeightedArcType::WALL;
                sigma = 0.0;
                j = 0.0;
                a = 0.0;
            } else if (std::abs(sigmaDes) < 1e-9) {
                arcType = WeightedArcType::SINGULAR;
                sigma = 0.0;
            } else if (sigmaDes > 0) {
                arcType = WeightedArcType::SNAP_PLUS;
                sigma = sigmaDes;
            } else {
                arcType = WeightedArcType::SNAP_MINUS;
                sigma = sigmaDes;
            }

            // Clamp snap if jerk would exceed bounds
            if (sigma > 0.0 && j >= jMax - 1e-6) {
                sigma = 0.0; arcType = WeightedArcType::SINGULAR;
            }
            if (sigma < 0.0 && j <= -jMax + 1e-6) {
                sigma = 0.0; arcType = WeightedArcType::SINGULAR;
            }

            // ── Step size: limit arc at switching points ──
            double dsArc = dsStep;
            if (s + dsArc > sEnd) dsArc = sEnd - s;

            // Limit arc when j reaches j_des (for SNAP arcs)
            if (arcType == WeightedArcType::SNAP_PLUS ||
                arcType == WeightedArcType::SNAP_MINUS) {
                double tauJ = (j_des - j) / sigma;
                if (tauJ > 1e-12 && tauJ < 1e6) {
                    double dsJ = SnapSeg::ds(v, a, j, sigma, tauJ);
                    if (dsJ > 0.0 && dsJ < dsArc) dsArc = dsJ;
                }
            }

            // Limit arc at corners
            if (!corners_.empty()) {
                auto it = std::lower_bound(corners_.begin(), corners_.end(),
                    s, [](const std::pair<double,double>& c, double val) {
                        return c.first < val; });
                for (; it != corners_.end() && it->first <= s + dsArc + 1e-9; ++it) {
                    double dCorner = it->first - s;
                    if (dCorner <= 1e-9) continue;
                    double vCorner = it->second;
                    if (vCorner >= v - 1e-6) continue;
                    double vAtCorner;
                    if (arcType == WeightedArcType::WALL) {
                        vAtCorner = v;
                    } else if (arcType == WeightedArcType::SINGULAR) {
                        double tau_c = SingularJSeg::tau_for_ds(v, a, jStarEff, dCorner);
                        vAtCorner = SingularJSeg::v(v, a, jStarEff, tau_c);
                    } else {
                        double tau_c = SnapSeg::tau_for_ds(v, a, j, sigma, dCorner);
                        vAtCorner = SnapSeg::v(v, a, j, sigma, tau_c);
                    }
                    if (vAtCorner > vCorner + 1e-6) {
                        dsArc = dCorner; break;
                    }
                }
            }

            // Limit arc when a reaches a bound (for SNAP arcs)
            if (arcType == WeightedArcType::SNAP_PLUS ||
                arcType == WeightedArcType::SNAP_MINUS) {
                for (double aBound : {aBoundMax, aBoundMin}) {
                    if (std::abs(sigma) < 1e-12) continue;
                    double disc = j * j - 2.0 * sigma * (a - aBound);
                    if (disc < 0.0) continue;
                    double sq = std::sqrt(disc);
                    for (double tauA : {(-j + sq) / sigma, (-j - sq) / sigma}) {
                        if (tauA > 1e-12 && tauA < 1e6) {
                            double dsA = SnapSeg::ds(v, a, j, sigma, tauA);
                            if (dsA > 0.0 && dsA < dsArc) dsArc = dsA;
                        }
                    }
                }
            }

            if (dsArc < 1e-12) dsArc = 1e-12;
            if (dsArc > dsStep) dsArc = dsStep;  // safety: don't exceed step size

            // ── Propagate ──
            double tau, v1, a1, j1;
            bool stoppedBeforeEnd = false;

            if (arcType == WeightedArcType::WALL) {
                tau = dsArc / std::max(v, 1e-12);
                v1 = v; a1 = 0.0; j1 = 0.0;
            } else if (arcType == WeightedArcType::SINGULAR) {
                double jUse = jStarEff;
                double sMax = std::numeric_limits<double>::infinity();
                if (a < 0.0 && jUse <= 0.0) {
                    double tStop = SnapSeg::timeToStop(v, a, jUse, 0.0);
                    if (std::isfinite(tStop))
                        sMax = SingularJSeg::ds(v, a, jUse, tStop);
                }
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = SingularJSeg::tau_for_ds(v, a, jUse, dsArc);
                    v1 = 0.0;
                    a1 = SingularJSeg::a(a, jUse, tau);
                    j1 = jUse;
                    stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6);
                } else {
                    tau = SingularJSeg::tau_for_ds(v, a, jUse, dsArc);
                    v1 = SingularJSeg::v(v, a, jUse, tau);
                    a1 = SingularJSeg::a(a, jUse, tau);
                    j1 = jUse;
                }
            } else {
                // SNAP
                double sMax = SnapSeg::maxForwardDistance(v, a, j, sigma);
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = SnapSeg::tau_for_ds(v, a, j, sigma, dsArc);
                    if (tau <= 0.0) tau = 1e-12;
                    v1 = SnapSeg::v(v, a, j, sigma, tau);
                    a1 = SnapSeg::a(a, j, sigma, tau);
                    j1 = SnapSeg::j(j, sigma, tau);
                    if (v1 < 0.0) { v1 = 0.0; stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6); }
                } else {
                    tau = SnapSeg::tau_for_ds(v, a, j, sigma, dsArc);
                    v1 = SnapSeg::v(v, a, j, sigma, tau);
                    a1 = SnapSeg::a(a, j, sigma, tau);
                    j1 = SnapSeg::j(j, sigma, tau);
                }
            }

            // Clamp v1 to vLim at end of arc
            double vLimEnd = vLimAt(std::min(s + dsArc, sEnd));
            if (v1 > vLimEnd + 1e-6) {
                // Shorten arc to where v reaches vLimEnd (bisection on tau)
                if (arcType == WeightedArcType::SNAP_PLUS ||
                    arcType == WeightedArcType::SNAP_MINUS) {
                    auto fval = [&](double tt) {
                        return SnapSeg::v(v, a, j, sigma, tt) - vLimEnd; };
                    double lo = 0.0, hi = tau;
                    if (fval(lo) <= 0.0 && fval(hi) >= 0.0) {
                        for (int b = 0; b < 60; ++b) {
                            double mid = 0.5*(lo+hi);
                            if (fval(mid) > 0) hi = mid; else lo = mid;
                            if (hi - lo < 1e-14) break;
                        }
                        tau = 0.5*(lo+hi);
                        dsArc = SnapSeg::ds(v, a, j, sigma, tau);
                        v1 = vLimEnd;
                        a1 = SnapSeg::a(a, j, sigma, tau);
                        j1 = SnapSeg::j(j, sigma, tau);
                    }
                } else if (arcType == WeightedArcType::SINGULAR) {
                    double jUse = jStarEff;
                    if (std::abs(jUse) > 1e-12) {
                        double disc = a*a - 2.0*jUse*(v - vLimEnd);
                        if (disc > 0.0) {
                            double tauV = (-a + std::sqrt(disc)) / jUse;
                            if (tauV > 0.0 && tauV < tau) {
                                tau = tauV;
                                dsArc = SingularJSeg::ds(v, a, jUse, tau);
                                v1 = vLimEnd;
                                a1 = SingularJSeg::a(a, jUse, tau);
                                j1 = jUse;
                            }
                        }
                    } else if (std::abs(a) > 1e-12) {
                        double tauV = (vLimEnd - v) / a;
                        if (tauV > 0.0 && tauV < tau) {
                            tau = tauV;
                            dsArc = SingularJSeg::ds(v, a, jUse, tau);
                            v1 = vLimEnd; a1 = a; j1 = jUse;
                        }
                    }
                }
            }
            if (v1 < 0.0) v1 = 0.0;
            // Safety: ensure dsArc is still valid after vLim clamping
            if (dsArc < 1e-12) dsArc = 1e-12;
            if (dsArc > dsStep) dsArc = dsStep;
            j1 = std::clamp(j1, -jMax, jMax);
            auto aEndB = accelBoundsFromCache(gridIndex(s + dsArc), v1);
            a1 = std::clamp(a1, static_cast<double>(aEndB.first),
                                 static_cast<double>(aEndB.second));

            // ── Cost increment (closed-form ∫(w_t + w_j·j² + w_a·a²)dt) ──
            double dJ;
            if (arcType == WeightedArcType::SINGULAR) {
                double jUse = jStarEff;
                double intJ2 = jUse * jUse * tau;
                double intA2 = a*a*tau + a*jUse*tau*tau
                    + (jUse*jUse)/3.0 * tau*tau*tau;
                dJ = w_.w_t * tau + w_.w_j * intJ2 + w_.w_a * intA2;
            } else if (arcType == WeightedArcType::WALL) {
                dJ = w_.w_t * tau;
            } else {
                double s_ = sigma;
                double intJ2 = j*j*tau + j*s_*tau*tau
                    + (s_*s_*tau*tau*tau)/3.0;
                double intA2 = a*a*tau + a*j*tau*tau
                    + (j*j + a*s_)/3.0 * tau*tau*tau
                    + (j*s_)/4.0 * tau*tau*tau*tau
                    + (s_*s_)/20.0 * tau*tau*tau*tau*tau;
                dJ = w_.w_t * tau + w_.w_j * intJ2 + w_.w_a * intA2;
            }
            J += dJ;

            // ── Record arc ──
            if (record) {
                Arc arc;
                arc.type = arcType;
                arc.s0 = s;
                arc.s1 = s + dsArc;
                arc.t0 = t;
                arc.v0 = v;
                arc.a0 = a;
                arc.j0 = j;
                arc.sigma = sigma;
                arc.j_star = jStarEff;
                arc.duration = tau;
                arc.eta = sigma;
                arc.a_star = jStarEff;
                tmp.push_back(arc);
            }

            // Advance
            s += dsArc;
            t += tau;
            v = v1;
            a = a1;
            j = j1;

            // Safety: if s went backwards or v is NaN, something is wrong
            if (s < 0.0 || !std::isfinite(v) || !std::isfinite(s)) {
                infeasible = true; break;
            }

            // Stuck detection: if s didn't advance meaningfully, count it.
            // If stuck for too many iterations, mark infeasible.
            if (s - sPrevStuck < 1e-9) {
                ++stuckCount;
                if (stuckCount > 200) {
                    infeasible = true; break;
                }
            } else {
                stuckCount = 0;
                sPrevStuck = s;
            }

            if (stoppedBeforeEnd) {
                J += 1e8 * (sEnd - s);
                break;
            }
        }

        // ── Feasibility check ──
        if (infeasible || s < sEnd - 1e-6) {
            if (record) arcs_.clear();
            lastCost_ = 1e12;
            lastS_ = s;
            lastV_ = v;
            return 1e12;
        }

        // Terminal velocity tolerance: allow small residual (snapspace
        // discretization makes exact v=0 difficult)
        if (std::abs(v - vf_) > 0.5) {
            // Large residual — add heavy penalty (consistent for record/non-record)
            J += 1e6 * std::abs(v - vf_);
        } else if (std::abs(v - vf_) > 1e-3) {
            // Small residual — add penalty but don't reject
            J += 1e3 * std::abs(v - vf_);
        }

        if (record) {
            arcs_ = std::move(tmp);
            lastJStar_ = jStar;
        }

        lastCost_ = J;
        lastS_ = s;
        lastV_ = v;
        return J;
    }
#endif
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
    using Limits = KinematicLimits<Dim, T>;
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
     * Solves the weighted-cost problem and produces a WeightedSwitchingStructure
     * wrapped in an AnalyticalSSRVelocityProfile.
     */
    std::unique_ptr<VelocityProfile> computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t constraintCacheSize = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        wss_.reset();
        pathCopy_.reset();
        lastFailure_.clear();

        if (path.numSegments() == 0) {
            lastFailure_ = "Path contains no segments";
            return std::make_unique<SampledVelocityProfile>();
        }

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) {
            lastFailure_ = "Path has non-positive length";
            return std::make_unique<SampledVelocityProfile>();
        }

        // Validate inputs (same guards as other profilers)
        if (constraintCacheSize < 2) {
            lastFailure_ = "At least two constraint samples are required";
            return std::make_unique<SampledVelocityProfile>();
        }
        if (feedRate <= T(0)) {
            lastFailure_ = "Feed rate must be positive";
            return std::make_unique<SampledVelocityProfile>();
        }
        if (limits_.path.maxPathAcceleration <= T(0)) {
            lastFailure_ = "Path acceleration limit must be positive";
            return std::make_unique<SampledVelocityProfile>();
        }
        if (limits_.path.maxCentripetalAcceleration < T(0)) {
            lastFailure_ = "Centripetal acceleration limit cannot be negative";
            return std::make_unique<SampledVelocityProfile>();
        }
        if (std::abs(startAcceleration) > T(1e-12) ||
            std::abs(startJerk) > T(1e-12)) {
            // The current endpoint API has no final acceleration/jerk
            // parameters. Treating nonzero initial higher derivatives as
            // zero would silently create a discontinuity, so reject them.
            lastFailure_ =
                "Nonzero initial acceleration or jerk is not representable by this endpoint API";
            return std::make_unique<SampledVelocityProfile>();
        }

        // If w_a = 0, this degenerates to time-optimal. We still solve
        // via the same machinery (a* → a_max), but the user could also
        // use AnalyticalJerkLimitedTOPPRA directly for that case.
        CostWeights wEff = weights_;
        if (wEff.w_t <= 0.0) {
            // The problem is ill-posed without a positive time weight.
            lastFailure_ = "Time weight must be positive";
            return std::make_unique<SampledVelocityProfile>();
        }

        // Clamp an overspeed start velocity to the lesser of the feed rate
        // and the actual velocity limit at the path start.
        T v0 = startVelocity;
        Evaluator startEval(limits_, feedRate);
        T vStartLim = std::min(feedRate,
                               startEval.velocityLimit(T(0), path));
        if (v0 > vStartLim) {
            v0 = vStartLim;
        }

        // Keep the path alive for the WSS by taking shared ownership.
        auto pathCopy = std::make_shared<Path>(path);
        pathCopy_ = pathCopy;

        // --- Dwell handling: split path at dwell points ---
        //
        // G4 dwell commands require the tool to stop (v=0) at specific arc
        // lengths for a given duration. If we solve the entire path at once,
        // the velocity limit function returns 0 at dwell positions (corner
        // velocity = 0), which makes the WALL arc duration integral
        // ∫ 1/v_lim ds diverge. Clamping to a small positive value is a
        // workaround that fails with consecutive dwells on short segments.
        //
        // The correct approach is to split the path at dwell points into
        // sub-paths, solve each sub-path independently with v=0 boundary
        // conditions at the split points, then insert DWELL arcs between
        // the sub-path solutions. This way:
        //   - The velocity limit never returns 0 inside a sub-path
        //   - Any number of consecutive dwells works (zero-length sub-paths
        //     produce no arcs, just the DWELL arcs)
        //   - Each dwell is individually trackable as a separate DWELL arc
        //
        const auto& dwellPoints = pathCopy->dwellPoints();
        std::vector<typename WSS::Arc> arcs;
        double optimalAStar = 0.0;
        double costValue = 0.0;

        if (!dwellPoints.empty()) {
            // Sort dwell points by arc length (defensive — the NURBS
            // converter emits them in order, but setDwellPoints doesn't
            // enforce sorting).
            auto sortedDwells = dwellPoints;
            std::sort(sortedDwells.begin(), sortedDwells.end(),
                      [](const auto& a, const auto& b) {
                          return a.first < b.first;
                      });

            // Build split points: [0, s_d1, s_d2, ..., L]
            // Each consecutive pair defines a sub-path.
            std::vector<double> splitS;
            splitS.push_back(0.0);
            for (const auto& [s, dur] : sortedDwells) {
                splitS.push_back(s);
            }
            splitS.push_back(static_cast<double>(pathLength));

            // Helper: create a DWELL arc at position s with duration dur
            auto makeDwellArc = [](double s, double dur) {
                typename WSS::Arc arc;
                arc.type = WeightedArcType::DWELL;
                arc.s0 = s;
                arc.s1 = s;
                arc.v0 = 0.0;
                arc.a0 = 0.0;
                arc.eta = 0.0;
                arc.a_star = 0.0;
                arc.duration = dur;
                return arc;
            };

            const auto& originalPath = pathCopy->inner();

            for (size_t i = 0; i + 1 < splitS.size(); ++i) {
                double sStart = splitS[i];
                double sEnd = splitS[i + 1];
                double subLen = sEnd - sStart;

                // Insert DWELL arc at the start of this segment (if not
                // the very first split, which is the path start).
                // The dwell at splitS[i] is sortedDwells[i-1] (for i > 0).
                if (i > 0) {
                    arcs.push_back(
                        makeDwellArc(sStart, sortedDwells[i - 1].second));
                    costValue += wEff.w_t * sortedDwells[i - 1].second;
                }

                // Skip zero-length sub-paths (consecutive dwells)
                if (subLen < 1e-9) continue;

                // Create trimmed sub-path
                auto subNurbsPath = originalPath.trim(sStart, sEnd);
                Path subPathAdapter(std::move(subNurbsPath));

                // Set segment velocity limits and corner velocities on
                // the sub-path. We need to map the original path's
                // per-segment feed rates and corner velocities to the
                // sub-path's segments.
                //
                // Important: we must NOT use maxVelocityAtArcLength() on
                // the original path, because it includes the dwell corner
                // velocities (set to 0 by setDwellPoints), which would
                // make the sub-path's feed rates 0 near dwell positions.
                // Instead, we use segmentMaxVelocities() directly and
                // recompute corner velocities fresh on the sub-path.
                if (pathCopy->hasPerSegmentVelocityLimits()) {
                    // Map original segment feed rates to sub-path segments.
                    // The original path's segments are indexed by piece
                    // index. The sub-path's pieces may be trimmed versions
                    // of the original pieces, so we need to find which
                    // original piece each sub-path piece corresponds to.
                    const auto& origSegs = pathCopy->segments();
                    const auto& subSegs = subPathAdapter.segments();
                    const auto& origFeedRates = pathCopy->segmentMaxVelocities();

                    std::vector<double> subFeedRates;
                    subFeedRates.reserve(subSegs.size());
                    for (size_t j = 0; j < subSegs.size(); ++j) {
                        // Find the original segment that contains the
                        // midpoint of this sub-path segment.
                        double subMidLocal =
                            subSegs[j].cumulativeArcLength +
                            subSegs[j].arcLength * 0.5;
                        double globalS = sStart + subMidLocal;
                        // Binary search for the original segment containing
                        // globalS. origSegs is sorted by cumulativeArcLength.
                        size_t origIdx = pathCopy->segmentIndexAtArcLength(
                            static_cast<T>(globalS));
                        if (origIdx < origFeedRates.size()) {
                            subFeedRates.push_back(origFeedRates[origIdx]);
                        } else {
                            subFeedRates.push_back(
                                std::numeric_limits<double>::infinity());
                        }
                    }
                    subPathAdapter.setSegmentVelocityLimits(subFeedRates);

                    // Compute corner velocities on the sub-path fresh.
                    // This computes the correct corner velocities at
                    // internal junctions based on the sub-path's geometry.
                    // At the sub-path boundaries (s=0 and s=end), the
                    // corner velocities default to infinity, which is
                    // correct since the v=0 constraint is enforced by
                    // the solver's boundary velocities.
                    //
                    // Use the same centripetal acceleration limit as the
                    // planner's kinematic limits. The junction deviation
                    // (0.05 mm) is a fixed default — PathAdapter doesn't
                    // store the value used by the caller, so we use the
                    // same default as GCodeProcessor.
                    subPathAdapter.computeCornerVelocities(
                        0.05,
                        static_cast<double>(
                            limits_.path.maxCentripetalAcceleration));
                }

                // Boundary velocities: v=0 at dwell points, original
                // boundaries at path start/end.
                T subV0 = (i == 0) ? v0 : T(0);
                T subVEnd =
                    (i + 2 == splitS.size()) ? endVelocity : T(0);

                // Solve sub-path
                Solver subSolver(subPathAdapter, limits_, wEff, feedRate);
                auto subArcs =
                    subSolver.solve(subV0, subVEnd, constraintCacheSize);

                if (subArcs.empty()) {
                    // Solver could not find a feasible trajectory for
                    // this sub-path. This can happen when the sub-path
                    // is too short for the solver to decelerate to v=0
                    // and accelerate back within the available distance.
                    wss_.reset();
                    pathCopy_.reset();
                    lastFailure_ = subSolver.failureReason();
                    return std::make_unique<SampledVelocityProfile>();
                }

                // Accumulate cost from sub-solvers. optimalAStar is a
                // per-sub-problem singular acceleration; we keep the first
                // sub-solver's value as a representative (it's metadata,
                // not used in WSS trajectory computation).
                if (i == 0) {
                    optimalAStar = subSolver.optimalAStar();
                }
                costValue += subSolver.costValue();

                // Offset arc positions from sub-path local to global
                for (auto& arc : subArcs) {
                    arc.s0 += sStart;
                    arc.s1 += sStart;
                }

                // Append sub-path arcs
                arcs.insert(arcs.end(),
                            std::make_move_iterator(subArcs.begin()),
                            std::make_move_iterator(subArcs.end()));
            }
        } else {
            // No dwell points: solve the entire path at once
            Solver solver(*pathCopy, limits_, wEff, feedRate);
            arcs = solver.solve(v0, endVelocity, constraintCacheSize);
            if (arcs.empty()) {
                wss_.reset();
                pathCopy_.reset();
                lastFailure_ = solver.failureReason();
                return std::make_unique<SampledVelocityProfile>();
            }
            optimalAStar = solver.optimalAStar();
            costValue = solver.costValue();
        }

        // Build the WSS.
        //
        // When dwell points are present, the pathCopy has corner velocities
        // set to 0 at dwell positions (by setDwellPoints). The WSS uses
        // wallVelocityLimit(s) which calls evaluator_.velocityLimit(s, path)
        // to compute WALL arc durations via quadrature. If the path has v=0
        // at dwell positions, the 1/v integral diverges, producing huge
        // durations (e.g. 1e14 s).
        //
        // Fix: create a "clean" path for the WSS that has the same NURBS
        // path and segment velocity limits, but with corner velocities
        // recomputed without dwell zeroing. This way, wallVelocityLimit
        // returns the true corner velocity at dwell positions (not 0),
        // and the WALL arc duration integral converges.
        std::shared_ptr<Path> wssPath = pathCopy;
        if (!dwellPoints.empty()) {
            wssPath = std::make_shared<Path>(pathCopy->inner());
            if (pathCopy->hasPerSegmentVelocityLimits()) {
                wssPath->setSegmentVelocityLimits(
                    pathCopy->segmentMaxVelocities());
                wssPath->computeCornerVelocities(
                    0.05,
                    static_cast<double>(
                        limits_.path.maxCentripetalAcceleration));
                // Do NOT call setDwellPoints — we want clean corner velocities.
            }
        }

        Evaluator evaluator(limits_, feedRate);
        auto wss = std::make_shared<WSS>(
            wssPath, std::move(arcs), wEff, std::move(evaluator),
            optimalAStar);
        wss_ = wss;
        wss_->setCostValue(costValue);
        lastFailure_.clear();
        // Keep the WSS path alive (may differ from pathCopy when dwells exist).
        pathCopy_ = wssPath;

        // Return an analytical profile that wraps the WSS.
        return std::make_unique<AnalyticalSSRVelocityProfile<Dim, T>>(wss_);
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

    /// Empty on success; otherwise the reason the last solve returned empty.
    const std::string& failureReason() const { return lastFailure_; }

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
    std::shared_ptr<const Path> pathCopy_;
    std::shared_ptr<WSS> wss_;
    std::string lastFailure_;
};

} // namespace MotionPlanner::analytical
