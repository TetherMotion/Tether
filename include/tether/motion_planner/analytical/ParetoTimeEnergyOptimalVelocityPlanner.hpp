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
#include "AnalyticalTOPPRA.hpp"
#include "AnalyticalSSRVelocityProfile.hpp"
#include "../VelocityProfile.hpp"
#include "../VelocityProfiler.hpp"
#include "../PathAdapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <tuple>
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
    DWELL,       ///< v = 0, a = 0, s = const (G4 dwell pause)
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
        case WeightedArcType::DWELL:      return "DWELL";
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
     * Safeguarded Newton-bisection hybrid. The cubic is strictly
     * increasing while v > 0, so a bracket [0, τ_hi] can be established
     * by doubling. The caller must only invoke this on arcs where v
     * stays positive; if no positive root exists, the returned τ is
     * clamped to the last bracket endpoint and the caller should treat
     * the arc as infeasible.
     */
    static double tau_for_ds(double v0, double a0, double e, double ds) {
        if (ds <= 0.0) return 0.0;

        auto f = [&](double t) {
            return v0 * t + 0.5 * a0 * t * t + (e / 6.0) * t * t * t;
        };

        // If the velocity would hit zero before we travel ds, return the
        // stopping time. Callers that ignore this will see v(tau)=0 and can
        // detect that the requested ds was infeasible.
        double tStop = timeToStop(v0, a0, e);
        if (std::isfinite(tStop) && tStop > 0.0) {
            double sMax = f(tStop);
            if (ds >= sMax) return tStop;

            // f is monotone increasing on [0, tStop] because v(t) >= 0.
            // Bisection/Newton on this bracket is safe and unique.
            double lo = 0.0, hi = tStop;
            double tau = 0.5 * (lo + hi);
            for (int i = 0; i < 100; ++i) {
                double val = f(tau) - ds;
                double der = v0 + a0 * tau + 0.5 * e * tau * tau;
                if (val > 0.0) hi = tau; else lo = tau;
                double next = (std::abs(der) > 1e-14) ? tau - val / der
                                                      : 0.5 * (lo + hi);
                if (next <= lo || next >= hi) next = 0.5 * (lo + hi);
                tau = next;
                if (hi - lo < 1e-13 * (1.0 + tau)) break;
            }
            return tau;
        }

        // No positive stopping time: v(t) stays positive, f is monotone.
        double lo = 0.0;
        double hi = std::max(ds / std::max(v0, 1e-3), 1e-6);
        const double kMaxHi = 1e6;
        while (f(hi) < ds) {
            hi *= 2.0;
            if (hi > kMaxHi) break;
        }

        double tau = 0.5 * (lo + hi);
        for (int i = 0; i < 100; ++i) {
            double val = f(tau) - ds;
            double der = v0 + a0 * tau + 0.5 * e * tau * tau;
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

    /// Smallest positive time τ such that v(τ) = 0.
    /// Returns +∞ if v(t) stays positive for all τ > 0.
    /// Returns 0 if the state is already at rest or any motion would make
    /// v negative immediately (e.g., v0 = 0 and a0 < 0).
    static double timeToStop(double v0, double a0, double e) {
        const double inf = std::numeric_limits<double>::infinity();

        // States that cannot move forward without v going negative first.
        if (v0 <= 0.0 && (a0 < 0.0 || (a0 == 0.0 && e < 0.0))) {
            return 0.0;
        }

        if (std::abs(e) > MathConstants::EPSILON) {
            double disc = a0 * a0 - 2.0 * e * v0;
            if (disc < 0.0) return inf;
            double sqrtDisc = std::sqrt(disc);
            double t1 = (-a0 - sqrtDisc) / e;
            double t2 = (-a0 + sqrtDisc) / e;
            double tPos = inf;
            if (t1 > 0.0) tPos = std::min(tPos, t1);
            if (t2 > 0.0) tPos = std::min(tPos, t2);
            return tPos;
        } else if (a0 < 0.0) {
            return -v0 / a0;
        } else {
            return inf;
        }
    }

    /// Maximum forward distance reachable while keeping v ≥ 0.
    static double maxForwardDistance(double v0, double a0, double e) {
        double tStop = timeToStop(v0, a0, e);
        if (!std::isfinite(tStop) || tStop <= 0.0) return std::numeric_limits<double>::infinity();
        return ds(v0, a0, e, tStop);
    }

    /**
     * @brief Shortest distance needed to bring the state to v=0 with a single
     * constant-η arc while respecting ηMin and aMinBound.
     *
     * The shortest stop uses the most negative feasible final acceleration.
     */
    static double terminalMinDistance(double v0, double a0, double etaMin,
                                      double etaMax, double aMinBound) {
        if (v0 <= 0.0) return 0.0;
        if (aMinBound > 0.0) aMinBound = 0.0;

        double jerkLimitTerm = a0 * a0 - 2.0 * v0 * etaMin;
        if (jerkLimitTerm < 0.0) return 0.0;
        double aEndJerkLim = -std::sqrt(jerkLimitTerm);
        double aEndLow = std::max(aMinBound, aEndJerkLim);
        if (aEndLow > 0.0) return 0.0;

        double aE = aEndLow;
        if (a0 + aE >= -1e-14) return 0.0;
        double t = -2.0 * v0 / (a0 + aE);
        if (!std::isfinite(t) || t < 0.0) return 0.0;
        double e = (aE * aE - a0 * a0) / (-2.0 * v0);
        if (e > etaMax) return 0.0;
        return ds(v0, a0, e, t);
    }

    /**
     * @brief Solve a single constant-η arc that brings the state to v=0 at
     * exactly sRemaining, with a final acceleration aEnd in [aMinBound, 0].
     *
     * Given (v0, a0), choose a final acceleration aEnd (≤ 0, ≥ aMinBound) such
     * that the arc length equals sRemaining and the final velocity is zero.
     * The jerk η and duration τ follow from the boundary conditions:
     *   aEnd = a0 + η·τ
     *   0    = v0 + a0·τ + ½·η·τ²
     * Solving yields τ = -2·v0 / (a0 + aEnd) and
     *   η    = (aEnd² - a0²) / (-2·v0).
     *
     * The chosen η must satisfy ηMin ≤ η ≤ 0 and aEnd must be within the
     * acceleration bounds. We bisect on aEnd.
     *
     * @return true if a feasible arc exists, populating eta, tau, and aEnd.
     */
    static bool terminalArc(double v0, double a0, double sRemaining,
                            double etaMin, double etaMax, double aMinBound,
                            double& eta, double& tau, double& aEnd) {
        if (sRemaining <= 0.0) {
            tau = 0.0;
            eta = 0.0;
            aEnd = a0;
            return true;
        }
        if (v0 <= 0.0) return false;

        // aEnd must be in [aMinBound, 0].
        if (aMinBound > 0.0) aMinBound = 0.0;

        auto sAt = [&](double aE) -> double {
            if (a0 + aE >= -1e-14) {
                // a never becomes negative enough to stop forward motion.
                return std::numeric_limits<double>::infinity();
            }
            double t = -2.0 * v0 / (a0 + aE);
            if (!std::isfinite(t) || t < 0.0) return 0.0;
            double e = (aE * aE - a0 * a0) / (-2.0 * v0);
            if (e > etaMax) return std::numeric_limits<double>::infinity();
            if (e < etaMin) return 0.0;
            return ds(v0, a0, e, t);
        };

        // Most negative feasible aEnd comes from the jerk lower bound and the
        // acceleration lower bound.
        double jerkLimitTerm = a0 * a0 - 2.0 * v0 * etaMin;
        if (jerkLimitTerm < 0.0) return false;
        double aEndJerkLim = -std::sqrt(jerkLimitTerm);
        double lo = std::max(aMinBound, aEndJerkLim);

        // Least negative feasible aEnd is constrained by the jerk upper bound.
        // For a0 >= 0 we also need aEnd < -a0, otherwise the stopping time
        // would be non-positive.
        double hi;
        if (a0 >= 0.0) {
            hi = -a0 - 1e-12;
        } else {
            double etaMaxTerm = a0 * a0 - 2.0 * v0 * etaMax;
            if (etaMaxTerm <= 0.0) {
                hi = 0.0;
            } else {
                hi = -std::sqrt(etaMaxTerm);
            }
        }
        if (hi > 0.0) hi = 0.0;
        if (lo > hi) return false;

        double sLo = sAt(lo);
        double sHi = sAt(hi);

        if (!std::isfinite(sLo)) {
            // lo should always give a finite stop distance if it is feasible.
            return false;
        }
        if (sRemaining < sLo - 1e-12) return false;

        // If the upper bound gives an infinite stopping distance, the required
        // aEnd lies very close to the critical value (aEnd -> -a0 for a0 >= 0
        // or aEnd -> 0 for a0 < 0).  In that case bisect against a bracketed
        // upper value by moving hi toward lo until sAt(hi) is finite and
        // larger than sRemaining.
        if (!std::isfinite(sHi) || sHi <= sRemaining) {
            double hiTry = hi;
            const double kHiMin = lo + 1e-12;
            for (int k = 0; k < 60; ++k) {
                double mid = 0.5 * (lo + hiTry);
                if (mid <= kHiMin) break;
                double sMid = sAt(mid);
                if (std::isfinite(sMid) && sMid > sRemaining) {
                    hiTry = mid;
                    sHi = sMid;
                    if (sMid - sRemaining > sRemaining - sLo) break;
                } else {
                    lo = mid;
                    sLo = std::isfinite(sMid) ? sMid : sLo;
                }
            }
            if (!std::isfinite(sHi) || sHi <= sRemaining) return false;
            hi = hiTry;
        }

        // Bisect on aEnd to match sRemaining.
        for (int iter = 0; iter < 80; ++iter) {
            double mid = 0.5 * (lo + hi);
            double sMid = sAt(mid);
            if (!std::isfinite(sMid)) {
                // mid is infeasible; move hi left.
                hi = mid;
                continue;
            }
            if (sMid > sRemaining) {
                // Need more deceleration (more negative aEnd).
                hi = mid;
                sHi = sMid;
            } else {
                lo = mid;
                sLo = sMid;
            }
            if (hi - lo < 1e-12 * (1.0 + std::abs(mid))) break;
        }

        aEnd = 0.5 * (lo + hi);
        tau = -2.0 * v0 / (a0 + aEnd);
        eta = (aEnd * aEnd - a0 * a0) / (-2.0 * v0);
        return std::isfinite(tau) && tau > 0.0 &&
               eta >= etaMin - 1e-12 && eta <= etaMax + 1e-12 &&
               aEnd >= aMinBound - 1e-12 && aEnd <= 1e-12;
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
            // Recompute durations so that WALL arcs use the velocity-limit
            // quadrature instead of any approximate constant-velocity value
            // supplied by the solver.
            arc.duration = computeArcDuration(arc);
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
                if (dsLocal < 0.0) dsLocal = 0.0;
                double tau = 0.0;
                if (arc.type == WeightedArcType::SINGULAR) {
                    tau = SingSeg::tau_for_ds(arc.v0, arc.a_star, dsLocal);
                } else if (arc.type == WeightedArcType::WALL) {
                    tau = wallDuration(arc.s0, arc.s0 + dsLocal);
                } else {
                    tau = BangSeg::tau_for_ds(arc.v0, arc.a0, arc.eta, dsLocal);
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

    // --- Accessors ---

    const std::vector<Arc>& arcs() const { return arcs_; }
    const CostWeights& weights() const { return w_; }
    double costValue() const { return costValue_; }
    void setCostValue(double J) { costValue_ = J; }

    /**
     * @brief The optimal singular acceleration level a* found by the solver.
     */
    double optimalAStar() const { return optimalAStar_; }

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
            pt.acceleration = (i == 0) ? static_cast<double>(startAcceleration) : static_cast<double>(a);
            pt.jerk = static_cast<double>(j);
            pt.time = static_cast<double>(t);

            // Determine limiting factor from the arc type
            if (std::abs(j) > T(1e-10)) {
                pt.limitedBy = (j > T(0))
                    ? VelocityProfilePoint::LimitType::ForwardAccel
                    : VelocityProfilePoint::LimitType::BackwardDecel;
            } else if (std::abs(a) > T(1e-10)) {
                pt.limitedBy = VelocityProfilePoint::LimitType::Jerk;  // singular arc
            } else if (v > T(1e-10)) {
                pt.limitedBy = VelocityProfilePoint::LimitType::Curvature;  // wall/cruise
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
        if (arc.type == WeightedArcType::DWELL) {
            // DWELL: v=0, a=0, s=const for the entire duration
            s = arc.s0;
            v = 0.0;
            a = 0.0;
            eta = 0.0;
        } else if (arc.type == WeightedArcType::SINGULAR) {
            a = arc.a_star;
            v = SingSeg::v(arc.v0, arc.a_star, tau);
            s = arc.s0 + SingSeg::ds(arc.v0, arc.a_star, tau);
            eta = 0.0;
        } else if (arc.type == WeightedArcType::WALL) {
            // WALL: follow the velocity-limit curve exactly.
            s = wallTimeToS(arc, tau);
            v = wallVelocityLimit(s);
            a = wallAcceleration(s);
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
        double sMax = static_cast<double>(pathPtr_->totalLength());
        if (s > sMax) s = sMax;
        if (s < 0.0) s = 0.0;
        if (v < 0.0) v = 0.0;

        // Hard guard: never report a velocity above the wall at this s.
        double vLim = static_cast<double>(evaluator_.velocityLimit(
            static_cast<T>(s), *pathPtr_));
        if (v > vLim) v = vLim;

        // Clamp acceleration and jerk to feasible limits at the current state.
        // This protects downstream consumers from spurious overshoots caused
        // by infeasible constant-eta arcs that the solver used to bridge
        // discrete sampling steps.
        auto [aMinB, aMaxB] = evaluator_.accelerationBounds(
            static_cast<T>(s), static_cast<T>(v), *pathPtr_);
        a = std::clamp(a, static_cast<double>(aMinB),
                            static_cast<double>(aMaxB));

        auto etaBounds = evaluator_.etaBounds(
            static_cast<T>(s), static_cast<T>(v), static_cast<T>(a),
            *pathPtr_);
        eta = std::clamp(eta,
                         static_cast<double>(etaBounds.eta_min),
                         static_cast<double>(etaBounds.eta_max));

        return {idx, tau, s, v, a, eta};
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
            return SingSeg::tau_for_ds(arc.v0, arc.a_star, ds);
        } else if (arc.type == WeightedArcType::WALL) {
            return wallDuration(arc.s0, arc.s1);
        } else {
            // BANG
            return BangSeg::tau_for_ds(arc.v0, arc.a0, arc.eta, ds);
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

    /// Wall acceleration a = v * dv_lim/ds, clamped to feasible bounds.
    double wallAcceleration(double s) const {
        const double eps = 1e-6;
        double vL = wallVelocityLimit(s - eps);
        double vC = wallVelocityLimit(s);
        double vR = wallVelocityLimit(s + eps);
        double dvds = (vR - vL) / (2.0 * eps);
        double a = vC * dvds;

        auto [aMin, aMax] = evaluator_.accelerationBounds(
            static_cast<T>(s), static_cast<T>(vC), *pathPtr_);
        return std::clamp(a, static_cast<double>(aMin),
                               static_cast<double>(aMax));
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
    std::vector<Arc> solve(T startVelocity = T(0),
                            T endVelocity = T(0),
                            size_t constraintCacheSize = 200) {
        v0_ = static_cast<double>(startVelocity);
        vf_ = static_cast<double>(endVelocity);
        constraintCacheSize_ = std::max(constraintCacheSize, size_t(10));

        // Precompute the path geometry and velocity limit on the solver grid.
        // This is done once per solve and reused by every simulateAndCost call.
        buildConstraintCache();

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

        // Harden the search: J(a*) is not guaranteed to be unimodal, so
        // scan a coarse log-spaced grid first, take the best bracket, and
        // only then run golden-section inside that bracket. This finds the
        // global trend and also handles the w_a = 0 endpoint optimum.
        const int coarseN = 17;
        double aBest = aLo;
        double JBest = std::numeric_limits<double>::infinity();
        for (int i = 0; i < coarseN; ++i) {
            double t = static_cast<double>(i) / (coarseN - 1.0);
            double a = aLo * std::pow(aHi / aLo, t);
            double Ji = simulateAndCost(a, /*record=*/false);
            if (Ji < JBest) {
                JBest = Ji;
                aBest = a;
            }
        }

        // Golden-section in the two cells surrounding the best grid point.
        double cellLo = std::max(aLo, aBest * std::pow(aHi / aLo, -1.0 / (coarseN - 1.0)));
        double cellHi = std::min(aHi, aBest * std::pow(aHi / aLo,  1.0 / (coarseN - 1.0)));
        if (cellLo >= cellHi) { cellLo = aLo; cellHi = aHi; }

        // A sub-millimetre tolerance is enough for the trajectory cost and
        // keeps the search bounded (≤ ~25 iterations for a 500-unit range).
        double tol = std::max(1e-6 * aMax, 0.01);
        auto [aOpt, Jmin] = goldenSection(J, cellLo, cellHi, tol);

        // The endpoint aHi can be optimal for w_a = 0 (time-optimal at the
        // largest feasible a*). Compare it explicitly.
        if (aOpt < aHi) {
            double Jhi = simulateAndCost(aHi, /*record=*/false);
            if (Jhi < Jmin) {
                aOpt = aHi;
                Jmin = Jhi;
            }
        }

        // Rebuild the optimal arc list
        arcs_.clear();
        double Jfinal = simulateAndCost(aOpt, /*record=*/true);

        // Coalesce consecutive arcs with identical type/eta/a_star that are
        // contiguous in arc length. This collapses the grid-dump into the
        // true switching structure and keeps the WSS small and exact.
        //
        // For BANG arcs (BANG_PLUS/BANG_MINUS), we additionally require that
        // a0 of the next arc matches a1 (= a0 + eta*duration) of the previous
        // arc. Without this check, coalescing BANG arcs with different a0
        // values produces an arc whose v1 (computed from v0, a0, eta, tau)
        // does not match the actual velocity at the end — creating velocity
        // discontinuities (GAPs) in the WSS.
        std::vector<Arc> coalesced;
        coalesced.reserve(arcs_.size());
        for (auto& arc : arcs_) {
            bool canCoalesce = false;
            if (!coalesced.empty() &&
                coalesced.back().type == arc.type &&
                coalesced.back().eta == arc.eta &&
                coalesced.back().a_star == arc.a_star &&
                std::abs(coalesced.back().s1 - arc.s0) < 1e-12) {
                if (arc.type == WeightedArcType::BANG_PLUS ||
                    arc.type == WeightedArcType::BANG_MINUS) {
                    // For BANG arcs, check a0 continuity:
                    // a1 of previous = a0_prev + eta * dur_prev
                    // must equal a0 of current
                    double a1Prev = coalesced.back().a0 +
                                    coalesced.back().eta *
                                    coalesced.back().duration;
                    canCoalesce = (std::abs(a1Prev - arc.a0) < 1.0);
                } else {
                    // WALL and SINGULAR: safe to coalesce (constant v or a)
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
    size_t constraintCacheSize_ = 200;  ///< Coarse grid size (for coefficients)
    double ds_ = 0.0;                   ///< Coarse grid spacing

    /// Fine grid for velocity limits (per-segment feed rates + cornering).
    /// When per-segment limits are present, the fine grid is larger than
    /// the coarse grid to capture per-segment velocity transitions.
    /// When no per-segment limits, fineGridSize_ == constraintCacheSize_
    /// and vLimFineGrid_ is identical to vLimGrid_.
    size_t fineGridSize_ = 0;
    double dsFine_ = 0.0;
    std::vector<double> vLimFineGrid_;

    /// Sorted list of corner constraints (arcLength, velocity) for the
    /// analytical TOPPRA backward pass (look-ahead).
    ///
    /// The look-ahead computes the exact deceleration profile from corner
    /// positions: v_req(s) = sqrt(v_corner² + 2*a_max*(s_corner - s)).
    /// This is the continuous form of the TOPPRA backward pass, evaluated
    /// at the exact corner position instead of at grid points. It works
    /// regardless of grid resolution, which is critical for large paths
    /// where the grid step (mm) can be much larger than the braking
    /// distance (sub-mm).
    std::vector<std::pair<double, double>> corners_;
    double aMaxForLookahead_ = 0.0;

    std::vector<KinematicCoefficients> gridCoeffs_;
    std::vector<double> vLimGrid_;

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
        // This is the TOPPRA backward/forward pass, computed at fine grid
        // resolution so that corner dips (which can be narrower than one
        // coarse grid step) are properly spread into smooth deceleration
        // profiles. The solver queries this fine grid for velocity limits,
        // so the deceleration points emerge naturally from the profile.
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
        double vEff = vGridLim;
        double dMax = vGridLim * vGridLim / (2.0 * aMaxForLookahead_);

        for (; it != corners_.end(); ++it) {
            double sCorner = it->first;
            double vCorner = it->second;
            double d = sCorner - s;
            if (d < 0.0) continue;
            if (d > dMax) break;

            double vReq = std::sqrt(vCorner * vCorner + 2.0 * aMaxForLookahead_ * d);
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

        // Keep the coarse grid step size for the simulation. The fine grid
        // is used only for velocity limit queries (per-segment + cornering).
        // Using the fine grid spacing for the step would make the solver
        // take too many steps and cause velocity discontinuities.
        const double ds = ds_;
        const double sEnd = sTotal_;

        // Constraint queries are served from the precomputed grid.  This
        // avoids the expensive arc-length inversion + high-order NURBS
        // derivative evaluation on every simulation step.
        // - Eta/accel bounds use the coarse grid (geometric coefficients).
        // - Velocity limit uses the fine grid (per-segment + cornering).
        auto getEtaBounds = [&](double sCur, double vCur, double aCur)
            -> EtaBounds {
            return etaBoundsFromCache(gridIndex(sCur), vCur, aCur);
        };

        auto getVLimit = [&](double sCur) -> double {
            // Analytical TOPPRA backward pass: compute the velocity limit
            // from exact corner positions, without any grid.
            //
            // v_req(s) = min over all upcoming corners (s_c, v_c) of:
            //   sqrt(v_c² + 2*a_max*(s_c - s))
            //
            // This is the continuous-form TOPPRA backward pass. It works
            // regardless of grid resolution. The grid vLimGrid_ is used
            // only for the geometric velocity limit (curvature, axis
            // limits, per-segment feed rates).
            double vGrid = vLimGrid_[gridIndex(sCur)];
            return lookAheadVLimit(sCur, vGrid);
        };

        bool infeasible = false;
        // Increase maxIter to account for sub-step arcs at corners
        // (WALL arcs limited to where vLim drops, BANG_MINUS arcs limited
        // to where v reaches vLim).
        int maxIter = static_cast<int>(constraintCacheSize_) * 50;
        for (int iter = 0; iter < maxIter && s < sEnd - 1e-10; ++iter) {
            EtaBounds etaBounds = getEtaBounds(s, v, a);
            if (!etaBounds.feasible()) {
                infeasible = true;
                break;
            }
            double vLim = getVLimit(s);

            // Acceleration bounds at this state.
            auto aBounds = accelBoundsFromCache(gridIndex(s), v);
            if (aBounds.first > aBounds.second) {
                infeasible = true;
                break;
            }
            double aMinBound = static_cast<double>(aBounds.first);
            double aMaxBound = static_cast<double>(aBounds.second);

            // Clamp eta bounds based on acceleration limits: if a is at or
            // near the acceleration bound, prevent jerk from pushing it
            // further beyond the bound. This avoids producing BANG arcs
            // whose end acceleration exceeds the limit.
            {
                double aTol = 1e-6 * std::max(1.0, std::abs(aMaxBound));
                if (a <= aMinBound + aTol) {
                    // Can't decrease a further — clamp eta_min to 0
                    etaBounds.eta_min = std::max(etaBounds.eta_min, 0.0);
                }
                if (a >= aMaxBound - aTol) {
                    // Can't increase a further — clamp eta_max to 0
                    etaBounds.eta_max = std::min(etaBounds.eta_max, 0.0);
                }
                if (!etaBounds.feasible()) {
                    infeasible = true;
                    break;
                }
            }

            double sRemaining = sEnd - s;

            // Terminal braking check: if the remaining distance is within the
            // shortest feasible stopping distance, solve a single constant-η arc
            // that lands at (sEnd, v=0). This guarantees rest-to-rest.
            double sStopMin = BangSeg::terminalMinDistance(
                v, a, etaBounds.eta_min, etaBounds.eta_max, aMinBound);
            if (sRemaining <= sStopMin * 1.2 + ds) {
                if (sRemaining < sStopMin - 1e-9) {
                    // We have already moved past the point where we can stop in
                    // the remaining distance: this a* is infeasible.
                    infeasible = true;
                    break;
                }
                double etaTerm, tauTerm, aEndTerm;
                bool terminalOk = BangSeg::terminalArc(
                    v, a, sRemaining, etaBounds.eta_min, etaBounds.eta_max,
                    aMinBound, etaTerm, tauTerm, aEndTerm);
                if (terminalOk) {
                    // Record the terminal arc and finish.
                    double a1 = std::clamp(aEndTerm, aMinBound, aMaxBound);

                    double intA2 = a * a * tauTerm
                                 + a * etaTerm * tauTerm * tauTerm
                                 + (etaTerm * etaTerm * tauTerm * tauTerm
                                    * tauTerm) / 3.0;
                    J += w_.w_t * tauTerm + w_.w_a * intA2;

                    if (record) {
                        Arc arc;
                        arc.type = (std::abs(etaTerm) > 1e-12)
                                       ? WeightedArcType::BANG_MINUS
                                       : WeightedArcType::SINGULAR;
                        arc.s0 = s;
                        arc.s1 = sEnd;
                        arc.t0 = t;
                        arc.v0 = v;
                        arc.a0 = a;
                        arc.eta = etaTerm;
                        arc.a_star = aStar;
                        arc.duration = tauTerm;
                        tmp.push_back(arc);
                    }
                    s = sEnd;
                    t += tauTerm;
                    v = 0.0;
                    a = a1;
                    break;
                }
            }

            // Select desired eta per the a* guidance law.
            //
            // The velocity limit from the analytical TOPPRA backward pass
            // (look-ahead) defines a smooth deceleration profile at corners.
            // The solver follows this profile:
            // - v > vLim: overshoot → BANG_MINUS to decelerate back to vLim
            // - v ≈ vLim: WALL (cruise) — but check if vLim is about to drop
            // - v < vLim: follow a* guidance (BANG_PLUS / SINGULAR / BANG_MINUS)
            double aStarEff = std::clamp(aStar, aMinBound, aMaxBound);
            double aTol = 1e-7 * std::max(1.0, std::abs(aStarEff));
            double etaDes;

            // Check if the look-ahead vLim is dropping (corner approaching).
            // If vLim at s+epsilon is less than v, we need to decelerate.
            //
            // Note: the jerk-limited braking distance early trigger was removed.
            // The corner-distance limiting + v1 shortening (for BANG arcs)
            // already prevents velocity discontinuities (GAPs) at corners.
            // The solver arrives at the corner with v > v_corner, then takes
            // a BANG_MINUS arc that correctly decelerates to v_corner using
            // the actual jerk-limited dynamics.
            bool vLimDropping = false;
            if (!corners_.empty() && v >= vLim - 1e-10) {
                double vLimAhead = lookAheadVLimit(
                    s + 1e-6, vLim);
                if (vLimAhead < v - 1e-6) vLimDropping = true;
            }

            if (v > vLim + 1e-6 || vLimDropping) {
                // Overshooting or about to overshoot — decelerate
                etaDes = etaBounds.eta_min;
            } else if (v >= vLim - 1e-10) {
                // At velocity wall — cruise
                etaDes = 0.0;
                a = 0.0;  // hold at wall
            } else if (a < aStarEff - aTol) {
                etaDes = etaBounds.eta_max;  // BANG_PLUS: raise a toward a*
            } else if (a > aStarEff + aTol) {
                etaDes = etaBounds.eta_min;  // BANG_MINUS: lower a toward a*
            } else {
                etaDes = 0.0;     // SINGULAR: hold at a*
            }

            // Clamp to feasible bounds
            double eta = etaBounds.clamp(etaDes);

            // Determine arc type
            WeightedArcType arcType;
            if (v > vLim + 1e-6 || vLimDropping) {
                // Overshooting or about to overshoot — BANG_MINUS
                arcType = WeightedArcType::BANG_MINUS;
            } else if (v >= vLim - 1e-10) {
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

            // Limit arc to the nearest corner where the arc dynamics would
            // violate the corner velocity. For each corner within the arc,
            // compute v at the corner from the arc dynamics. If v > v_corner,
            // shorten the arc to stop at the corner.
            //
            // This is more precise than blindly stopping at every corner with
            // v_corner < v: BANG_MINUS arcs that are already decelerating may
            // reach v_corner by the corner without needing to stop.
            if (!corners_.empty()) {
                auto it = std::lower_bound(corners_.begin(), corners_.end(),
                    s,
                    [](const std::pair<double, double>& c, double val) {
                        return c.first < val;
                    });
                for (; it != corners_.end() && it->first <= s + dsArc + 1e-9; ++it) {
                    double dCorner = it->first - s;
                    if (dCorner <= 1e-9) continue;
                    double vCorner = it->second;
                    if (vCorner >= v - 1e-6) continue;  // no constraint
                    // Compute v at the corner from the arc dynamics
                    double vAtCorner;
                    if (arcType == WeightedArcType::WALL) {
                        vAtCorner = v;  // constant velocity
                    } else if (arcType == WeightedArcType::SINGULAR) {
                        double tau_c = SingSeg::tau_for_ds(v, aStarEff, dCorner);
                        vAtCorner = SingSeg::v(v, aStarEff, tau_c);
                    } else {
                        double tau_c = BangSeg::tau_for_ds(v, a, eta, dCorner);
                        vAtCorner = BangSeg::v(v, a, eta, tau_c);
                    }
                    if (vAtCorner > vCorner + 1e-6) {
                        dsArc = dCorner;
                        break;
                    }
                }
            }

            // Don't overshoot velocity limit (BANG_PLUS: v reaching vLim)
            if (arcType == WeightedArcType::BANG_PLUS) {
                double vTarget = vLim;
                double disc = a * a - 2.0 * eta * (v - vTarget);
                if (disc > 0.0 && eta > 0.0) {
                    double tauV = (-a + std::sqrt(disc)) / eta;
                    if (tauV > 0.0) {
                        double dsV = BangSeg::ds(v, a, eta, tauV);
                        if (dsV > 0.0 && dsV < dsArc) dsArc = dsV;
                    }
                }
            }

            // Don't undershoot velocity limit (BANG_MINUS: v reaching vLim)
            // When decelerating from overshoot or from a dropping vLim, limit
            // the arc so the solver can follow the TOPPRA backward-pass profile.
            if (arcType == WeightedArcType::BANG_MINUS &&
                (v > vLim + 1e-6 || vLimDropping)) {
                // When vLimDropping, the nearest corner is within braking
                // distance. Limit the arc to the distance to that corner
                // so the solver takes small steps and doesn't overshoot
                // to v=0 over a large ds step.
                if (vLimDropping && !corners_.empty()) {
                    auto it = std::lower_bound(corners_.begin(), corners_.end(),
                        s,
                        [](const std::pair<double, double>& c, double val) {
                            return c.first < val;
                        });
                    for (; it != corners_.end(); ++it) {
                        double dCorner = it->first - s;
                        if (dCorner > 1e-9) {
                            if (dCorner < dsArc) dsArc = dCorner;
                            break;
                        }
                    }
                }

                // Limit the arc to the distance where v reaches vLim.
                // When overshooting (v > vLim), target the current vLim
                // (the corner velocity). When vLimDropping, use the
                // fixed-point iteration with the look-ahead vLim.
                double vTarget;
                if (v > vLim + 1e-6) {
                    // Overshooting: target the current vLim
                    vTarget = vLim;
                } else {
                    // vLimDropping: iterate with look-ahead vLim
                    vTarget = getVLimit(std::min(s + dsArc, sEnd));
                }

                if (vTarget < v - 1e-6) {
                    double disc = a * a - 2.0 * eta * (v - vTarget);
                    if (disc > 0.0 && eta < 0.0) {
                        double tauV = (-a - std::sqrt(disc)) / eta;
                        if (tauV > 0.0) {
                            double dsV = BangSeg::ds(v, a, eta, tauV);
                            if (dsV > 0.0 && dsV < dsArc) dsArc = dsV;
                        }
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

            // Compute tau (time for this arc). If the constant-eta control
            // would cause v to hit zero before we cover dsArc, we truncate the
            // step to the feasible stopping distance. This prevents the solver
            // from producing physically meaningless arcs with huge duration.
            bool stoppedBeforeEnd = false;
            double tau;
            double v1, a1;
            if (arcType == WeightedArcType::WALL) {
                tau = dsArc / std::max(v, 1e-12);
                a1 = 0.0;
                v1 = v;
            } else if (arcType == WeightedArcType::SINGULAR) {
                double as = aStarEff;
                double sMax = (as < 0.0) ? (-v * v / (2.0 * as))
                                         : std::numeric_limits<double>::infinity();
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = (as < 0.0 && v > 0.0) ? -v / as : 0.0;
                    v1 = 0.0;
                    a1 = as;
                    stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6);
                } else {
                    tau = SingSeg::tau_for_ds(v, as, dsArc);
                    a1 = as;
                    v1 = SingSeg::v(v, as, tau);
                }
            } else {
                double sMax = BangSeg::maxForwardDistance(v, a, eta);
                if (sMax < dsArc) {
                    dsArc = std::max(sMax, 0.0);
                    tau = BangSeg::timeToStop(v, a, eta);
                    v1 = 0.0;
                    a1 = BangSeg::a(a, eta, tau);
                    stoppedBeforeEnd = (s + dsArc < sEnd - 1e-6);
                } else {
                    tau = BangSeg::tau_for_ds(v, a, eta, dsArc);
                    a1 = BangSeg::a(a, eta, tau);
                    v1 = BangSeg::v(v, a, eta, tau);
                }
            }

            // If v exceeds the velocity limit at the end of the arc, shorten
            // the arc to where v reaches vLim exactly. This replaces the old
            // v1 clamping that created velocity discontinuities (GAPs) in the
            // WSS — the arc was recorded with the full duration but v1 was
            // clamped, so the arc parameters were inconsistent with the state.
            //
            // The velocity limit at the end of the arc is the look-ahead vLim
            // (TOPPRA backward pass), which accounts for corner velocities.
            double vLimEnd = getVLimit(std::min(s + dsArc, sEnd));
            if (v1 > vLimEnd + 1e-6 && !stoppedBeforeEnd) {
                if (arcType == WeightedArcType::SINGULAR) {
                    // v(t) = v0 + a* * t = vLimEnd → t = (vLimEnd - v0) / a*
                    double as = aStarEff;
                    if (std::abs(as) > 1e-12) {
                        double tauV = (vLimEnd - v) / as;
                        if (tauV > 0.0 && tauV < tau) {
                            tau = tauV;
                            dsArc = SingSeg::ds(v, as, tau);
                            v1 = vLimEnd;
                            a1 = as;
                        }
                    }
                } else if (arcType == WeightedArcType::WALL) {
                    // WALL: v = const. If v > vLimEnd, the solver should
                    // have decelerated earlier. Don't shorten to zero (that
                    // causes an infinite loop via the minimum-step guard).
                    // Instead, let the arc run to the corner — the next
                    // iteration will see v > vLim and take BANG_MINUS.
                    // The v1 shortening for BANG arcs will then produce a
                    // correct deceleration arc without a GAP.
                } else {
                    // BANG: v(t) = v0 + a0*tau + 0.5*eta*tau^2 = vLimEnd
                    // Solve for tau: 0.5*eta*tau^2 + a0*tau + (v0 - vLimEnd) = 0
                    // For eta < 0 (decelerating): tau = (-a - sqrt(disc)) / eta
                    // For eta > 0 (accelerating): tau = (-a + sqrt(disc)) / eta
                    double disc = a * a - 2.0 * eta * (v - vLimEnd);
                    if (disc > 0.0 && std::abs(eta) > 1e-12) {
                        double tauV = (eta < 0.0)
                            ? (-a - std::sqrt(disc)) / eta
                            : (-a + std::sqrt(disc)) / eta;
                        if (tauV > 0.0 && tauV < tau) {
                            tau = tauV;
                            dsArc = BangSeg::ds(v, a, eta, tau);
                            v1 = vLimEnd;
                            a1 = BangSeg::a(a, eta, tau);
                        }
                    }
                }
            }
            if (v1 < 0.0) v1 = 0.0;

            // Clamp acceleration to acceleration bounds
            auto aEndBounds = accelBoundsFromCache(
                gridIndex(s + dsArc), v1);
            if (aEndBounds.first > aEndBounds.second) {
                infeasible = true;
                break;
            }

            // For BANG arcs, if a1 = a0 + eta*tau would exceed the
            // acceleration bounds, shorten the arc so that a1 exactly
            // reaches the bound. This prevents the WSS from containing
            // arcs whose end acceleration exceeds the limit.
            if (arcType == WeightedArcType::BANG_PLUS ||
                arcType == WeightedArcType::BANG_MINUS) {
                double a1Raw = a + eta * tau;
                double aHi = static_cast<double>(aEndBounds.second);
                double aLo = static_cast<double>(aEndBounds.first);
                if (a1Raw > aHi + 1e-6 && std::abs(eta) > 1e-12) {
                    double tauA = (aHi - a) / eta;
                    if (tauA > 0.0 && tauA < tau) {
                        tau = tauA;
                        dsArc = BangSeg::ds(v, a, eta, tau);
                        v1 = BangSeg::v(v, a, eta, tau);
                        a1 = aHi;
                    }
                } else if (a1Raw < aLo - 1e-6 && std::abs(eta) > 1e-12) {
                    double tauA = (aLo - a) / eta;
                    if (tauA > 0.0 && tauA < tau) {
                        tau = tauA;
                        dsArc = BangSeg::ds(v, a, eta, tau);
                        v1 = BangSeg::v(v, a, eta, tau);
                        a1 = aLo;
                    }
                }
            }

            a1 = std::clamp(a1, static_cast<double>(aEndBounds.first),
                                 static_cast<double>(aEndBounds.second));

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

            // If we had to stop before reaching the end, the chosen a* is
            // infeasible for the remaining distance. Add a large penalty and exit.
            if (stoppedBeforeEnd) {
                J += 1e8 * (sEnd - s);
                break;
            }
        }

        // Surface infeasibility or failure to traverse the entire path.
        if (infeasible || s < sEnd - 1e-6) {
            if (record) arcs_.clear();
            lastCost_ = 1e12;
            return 1e12;
        }

        // The trajectory must also satisfy the final velocity boundary.
        if (std::abs(v - vf_) > 1e-3) {
            if (record) {
                arcs_.clear();
                lastCost_ = 1e12;
                return 1e12;
            }
            J += 1e8 * std::abs(v - vf_);
        }

        if (record) {
            arcs_ = std::move(tmp);
            lastAStar_ = aStar;
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

        (void)startJerk;  // not honored (WI-P3 style)
        if (path.numSegments() == 0) {
            return std::make_unique<SampledVelocityProfile>();
        }

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) return std::make_unique<SampledVelocityProfile>();

        // Validate inputs (same guards as other profilers)
        if (constraintCacheSize < 2) return std::make_unique<SampledVelocityProfile>();
        if (feedRate <= T(0)) return std::make_unique<SampledVelocityProfile>();
        if (limits_.path.maxPathAcceleration <= T(0)) return std::make_unique<SampledVelocityProfile>();
        if (limits_.path.maxCentripetalAcceleration < T(0)) return std::make_unique<SampledVelocityProfile>();

        // If w_a = 0, this degenerates to time-optimal. We still solve
        // via the same machinery (a* → a_max), but the user could also
        // use AnalyticalTOPPRA directly for that case.
        CostWeights wEff = weights_;
        if (wEff.w_t <= 0.0) {
            // The problem is ill-posed without a positive time weight.
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
};

} // namespace MotionPlanner::analytical
