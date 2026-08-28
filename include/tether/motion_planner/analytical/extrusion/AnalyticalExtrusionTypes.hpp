/**
 * @file AnalyticalExtrusionTypes.hpp
 * @brief Common types for analytical extrusion compensation on WSS arcs.
 *
 * @details
 * This header defines the shared infrastructure used by all analytical
 * extrusion compensation algorithms.  The key idea is that the
 * WeightedSwitchingStructure (WSS) from the ParetoTimeEnergyOptimalVelocityPlanner
 * provides piecewise-polynomial expressions for the path velocity v(t):
 *
 *   BANG:     v(τ) = v0 + a0·τ + ½·η·τ²   (quadratic)
 *   SINGULAR: v(τ) = v0 + a*·τ             (linear)
 *   WALL:     v(τ) = v_wall                 (constant)
 *
 * The extruder velocity is v_e(t) = α_e · v(t), where α_e is the extrusion
 * ratio (E-distance per unit path distance).  All extrusion compensation
 * quantities (flow Q, pressure P, feed-forward power, deconvolution input)
 * are derived from v_e(t) and are therefore also piecewise polynomial or
 * piecewise smooth.
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md
 * @see ParetoTimeEnergyOptimalVelocityPlanner.hpp for the WSS.
 */

#pragma once

#include "../ParetoTimeEnergyOptimalVelocityPlanner.hpp"
#include "../AnalyticalTypes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

// ============================================================================
// Extrusion Arc: a WSS arc annotated with extrusion-specific data
// ============================================================================

/**
 * @brief A WSS arc with precomputed extrusion-relevant quantities.
 *
 * Extends WeightedArc with the extrusion ratio and the polynomial
 * coefficients of v_e(τ) expressed in a normalized form:
 *
 *   v_e(τ) = α_e · (c0 + c1·τ + c2·τ² + c3·τ³)
 *
 * where:
 *   SNAP:     c0 = v0, c1 = a0, c2 = ½·j0, c3 = (1/6)·σ
 *   SINGULAR: c0 = v0, c1 = a0, c2 = ½·j*, c3 = 0
 *   WALL:     c0 = v0, c1 = c2 = c3 = 0 (non-polynomial, sampled)
 */
struct ExtrusionArc {
    /// The underlying WSS arc
    WeightedArc arc;

    /// Extrusion ratio (E per unit path distance). 0 for travel moves.
    double extrusionRatio = 0.0;

    /// Polynomial coefficients of v(τ) = c0 + c1·τ + c2·τ² + c3·τ³
    /// (c2 = c3 = 0 for WALL arcs; c3 = 0 for SINGULAR arcs)
    double c0 = 0.0;  ///< v at arc start
    double c1 = 0.0;  ///< acceleration at arc start
    double c2 = 0.0;  ///< ½·jerk at arc start (0 for SINGULAR/WALL)
    double c3 = 0.0;  ///< (1/6)·snap (0 for SINGULAR/WALL)

    /// Arc duration [s]
    double duration = 0.0;

    /// Arc start time [s]
    double t0 = 0.0;

    /**
     * @brief Evaluate the path velocity v(τ) at local time τ.
     */
    double pathVelocity(double tau) const {
        return c0 + c1 * tau + c2 * tau * tau + c3 * tau * tau * tau;
    }

    /**
     * @brief Evaluate the extruder velocity v_e(τ) at local time τ.
     */
    double extruderVelocity(double tau) const {
        return extrusionRatio * pathVelocity(tau);
    }

    /**
     * @brief Evaluate the path acceleration a(τ) at local time τ.
     */
    double pathAcceleration(double tau) const {
        return c1 + 2.0 * c2 * tau + 3.0 * c3 * tau * tau;
    }

    /**
     * @brief Evaluate the path jerk η(τ) at local time τ.
     *
     * jerk = d²v/dτ² = 2·c2 + 6·c3·τ
     * (constant 2·c2 when c3 = 0, i.e. for SINGULAR/WALL arcs)
     */
    double pathJerk(double tau = 0.0) const {
        return 2.0 * c2 + 6.0 * c3 * tau;
    }

    /**
     * @brief Antiderivative of v(τ): ∫₀^τ v(s) ds
     *   = c0·τ + ½·c1·τ² + ⅓·c2·τ³ + ¼·c3·τ⁴
     */
    double pathVelocityIntegral(double tau) const {
        return c0 * tau + 0.5 * c1 * tau * tau
               + (1.0 / 3.0) * c2 * tau * tau * tau
               + 0.25 * c3 * tau * tau * tau * tau;
    }

    /**
     * @brief Antiderivative of v_e(τ): α_e · ∫₀^τ v(s) ds
     */
    double extruderVelocityIntegral(double tau) const {
        return extrusionRatio * pathVelocityIntegral(tau);
    }

    /**
     * @brief Time-averaged path velocity over the arc.
     * ṽ = (1/Δt) ∫₀^Δt v(τ) dτ = Δs / Δt
     */
    double avgPathVelocity() const {
        if (duration <= 0.0) return c0;
        return arc.length() / duration;
    }

    /**
     * @brief Time-averaged extruder velocity over the arc.
     */
    double avgExtruderVelocity() const {
        return extrusionRatio * avgPathVelocity();
    }
};

// ============================================================================
// Extrusion Trajectory: a list of ExtrusionArcs built from a WSS
// ============================================================================

/**
 * @brief A trajectory of extrusion arcs built from a WeightedSwitchingStructure.
 *
 * This is the primary data structure consumed by all analytical extrusion
 * compensation algorithms.  It converts the WSS arc list into ExtrusionArcs
 * with precomputed polynomial coefficients and extrusion ratios.
 *
 * The extrusion ratio can be:
 * - Uniform (same α_e for all arcs) — for simple test cases
 * - Per-segment (from G-code move data) — for real trajectories
 */
template<size_t Dim, typename T = double>
class ExtrusionTrajectory {
public:
    using WSS = WeightedSwitchingStructure<Dim, T>;
    using Arc = ExtrusionArc;

    /**
     * @brief Build from a WSS with a uniform extrusion ratio.
     *
     * @param wss The weighted switching structure
     * @param extrusionRatio E per unit path distance (0 for travel)
     */
    explicit ExtrusionTrajectory(const WSS& wss, double extrusionRatio = 0.0)
        : wss_(&wss), totalLength_(static_cast<double>(wss.totalLength())) {
        buildUniform(extrusionRatio);
    }

    /**
     * @brief Build from a WSS with per-segment extrusion ratios.
     *
     * @param wss The weighted switching structure
     * @param segmentRatios Extrusion ratio per path segment (indexed by
     *                      segmentIndex).  Must cover all segments in the path.
     */
    ExtrusionTrajectory(const WSS& wss,
                        const std::vector<double>& segmentRatios)
        : wss_(&wss), totalLength_(static_cast<double>(wss.totalLength())) {
        buildPerSegment(segmentRatios);
    }

    /// Number of extrusion arcs
    size_t numArcs() const { return arcs_.size(); }

    /// Get all arcs
    const std::vector<Arc>& arcs() const { return arcs_; }

    /// Get arc by index
    const Arc& arc(size_t i) const { return arcs_.at(i); }

    /// Total trajectory time [s]
    double totalTime() const {
        return arcs_.empty() ? 0.0
            : arcs_.back().t0 + arcs_.back().duration;
    }

    /// Total path length [mm]
    double totalLength() const { return totalLength_; }

    /// Total extruded length [mm] (sum of α_e · Δs over all arcs)
    double totalExtrudedLength() const {
        double total = 0.0;
        for (const auto& a : arcs_)
            total += a.extrusionRatio * a.arc.length();
        return total;
    }

    /**
     * @brief Locate the arc containing time t.
     * @return Arc index (clamped to [0, numArcs-1])
     */
    size_t findArc(double t) const {
        if (arcs_.empty()) return 0;
        // Binary search
        size_t lo = 0, hi = arcs_.size();
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (arcs_[mid].t0 + arcs_[mid].duration < t)
                lo = mid + 1;
            else
                hi = mid;
        }
        return std::min(lo, arcs_.size() - 1);
    }

    /**
     * @brief Evaluate the path velocity at time t.
     */
    double pathVelocityAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        return a.pathVelocity(tau);
    }

    /**
     * @brief Evaluate the extruder velocity at time t.
     */
    double extruderVelocityAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        return a.extruderVelocity(tau);
    }

    /**
     * @brief Evaluate the path acceleration at time t.
     */
    double pathAccelerationAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        return a.pathAcceleration(tau);
    }

    /**
     * @brief Evaluate the path jerk at time t.
     */
    double pathJerkAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        return a.pathJerk(tau);
    }

    /**
     * @brief Evaluate the arc length at time t.
     */
    double arcLengthAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        return a.arc.s0 + a.pathVelocityIntegral(tau);
    }

    /**
     * @brief Evaluate the extruder position at time t (cumulative E).
     */
    double extruderPositionAtTime(double t) const {
        if (arcs_.empty()) return 0.0;
        size_t idx = findArc(t);
        double ePos = 0.0;
        for (size_t i = 0; i < idx; ++i)
            ePos += arcs_[i].extrusionRatio * arcs_[i].arc.length();
        const auto& a = arcs_[idx];
        double tau = std::clamp(t - a.t0, 0.0, a.duration);
        ePos += a.extruderVelocityIntegral(tau);
        return ePos;
    }

    /// The underlying WSS
    const WSS& wss() const { return *wss_; }

private:
    const WSS* wss_;
    std::vector<Arc> arcs_;
    double totalLength_ = 0.0;

    void buildUniform(double ratio) {
        const auto& wssArcs = wss_->arcs();
        arcs_.clear();
        arcs_.reserve(wssArcs.size());
        for (const auto& wa : wssArcs) {
            Arc ea;
            ea.arc = wa;
            ea.extrusionRatio = ratio;
            ea.t0 = wa.t0;
            ea.duration = wa.duration;
            ea.c0 = wa.v0;
            if (wa.type == WeightedArcType::SINGULAR) {
                // v(τ) = v0 + a0·τ + ½·j*·τ²
                ea.c1 = wa.a0;
                ea.c2 = 0.5 * wa.j_star;
                ea.c3 = 0.0;
            } else if (wa.type == WeightedArcType::WALL) {
                ea.c1 = 0.0;
                ea.c2 = 0.0;
                ea.c3 = 0.0;
            } else {
                // BANG_PLUS or BANG_MINUS (SNAP arc)
                // v(τ) = v0 + a0·τ + ½·j0·τ² + (1/6)·σ·τ³
                ea.c1 = wa.a0;
                ea.c2 = 0.5 * wa.j0;
                ea.c3 = (1.0 / 6.0) * wa.sigma;
            }
            arcs_.push_back(ea);
        }
    }

    void buildPerSegment(const std::vector<double>& segmentRatios) {
        const auto& wssArcs = wss_->arcs();
        arcs_.clear();
        arcs_.reserve(wssArcs.size());
        for (const auto& wa : wssArcs) {
            Arc ea;
            ea.arc = wa;
            ea.t0 = wa.t0;
            ea.duration = wa.duration;
            ea.c0 = wa.v0;
            if (wa.type == WeightedArcType::SINGULAR) {
                ea.c1 = wa.a0;
                ea.c2 = 0.5 * wa.j_star;
                ea.c3 = 0.0;
            } else if (wa.type == WeightedArcType::WALL) {
                ea.c1 = 0.0;
                ea.c2 = 0.0;
                ea.c3 = 0.0;
            } else {
                ea.c1 = wa.a0;
                ea.c2 = 0.5 * wa.j0;
                ea.c3 = (1.0 / 6.0) * wa.sigma;
            }
            // Determine the segment index at the arc midpoint
            double tMid = wa.t0 + wa.duration * 0.5;
            auto segIdx = wss_->segmentIndex(static_cast<T>(tMid));
            ea.extrusionRatio = (segIdx < segmentRatios.size())
                ? segmentRatios[segIdx] : 0.0;
            arcs_.push_back(ea);
        }
    }
};

// ============================================================================
// Continuous-time smoothing (rectangular window convolution)
// ============================================================================

/**
 * @brief Continuous-time centered moving average of the path velocity.
 *
 * Computes v_smooth(t) = (1/T_s) ∫_{t-T_s/2}^{t+T_s/2} v(τ) dτ
 *
 * Since v(τ) is piecewise polynomial, this integral is evaluated in closed
 * form by splitting at arc boundaries within the window.
 *
 * @param traj The extrusion trajectory
 * @param t Query time
 * @param smoothTime Smoothing window width T_s [s] (0 = no smoothing)
 * @return Smoothed path velocity at time t
 */
template<size_t Dim, typename T>
double smoothedPathVelocity(const ExtrusionTrajectory<Dim, T>& traj,
                            double t, double smoothTime) {
    if (smoothTime <= 0.0) {
        return traj.pathVelocityAtTime(t);
    }
    const double halfWin = smoothTime * 0.5;
    const double tLo = t - halfWin;
    const double tHi = t + halfWin;
    const auto& arcs = traj.arcs();
    if (arcs.empty()) return 0.0;

    double integral = 0.0;
    double coveredTime = 0.0;

    // Find arcs overlapping [tLo, tHi]
    for (const auto& a : arcs) {
        double arcStart = a.t0;
        double arcEnd = a.t0 + a.duration;
        double overlapStart = std::max(arcStart, tLo);
        double overlapEnd = std::min(arcEnd, tHi);
        if (overlapEnd <= overlapStart) continue;

        // Local time within this arc
        double tauStart = overlapStart - a.t0;
        double tauEnd = overlapEnd - a.t0;

        // ∫_{tauStart}^{tauEnd} v(τ) dτ = V(tauEnd) - V(tauStart)
        // where V(τ) = c0·τ + ½·c1·τ² + ⅓·c2·τ³ + ¼·c3·τ⁴
        double intEnd = a.c0 * tauEnd + 0.5 * a.c1 * tauEnd * tauEnd
                        + (1.0 / 3.0) * a.c2 * tauEnd * tauEnd * tauEnd
                        + 0.25 * a.c3 * tauEnd * tauEnd * tauEnd * tauEnd;
        double intStart = a.c0 * tauStart + 0.5 * a.c1 * tauStart * tauStart
                          + (1.0 / 3.0) * a.c2 * tauStart * tauStart * tauStart
                          + 0.25 * a.c3 * tauStart * tauStart * tauStart * tauStart;
        integral += intEnd - intStart;
        coveredTime += overlapEnd - overlapStart;
    }

    if (coveredTime <= 0.0) return 0.0;
    return integral / coveredTime;
}

/**
 * @brief Continuous-time centered moving average of the extruder velocity.
 *
 * v_e_smooth(t) = α_e · v_smooth(t)  (computed per-arc with correct α_e)
 *
 * For uniform extrusion ratio, this is just α_e · smoothedPathVelocity.
 * For per-segment ratios, the integral is split at segment boundaries.
 */
template<size_t Dim, typename T>
double smoothedExtruderVelocity(const ExtrusionTrajectory<Dim, T>& traj,
                                double t, double smoothTime) {
    if (smoothTime <= 0.0) {
        return traj.extruderVelocityAtTime(t);
    }
    const double halfWin = smoothTime * 0.5;
    const double tLo = t - halfWin;
    const double tHi = t + halfWin;
    const auto& arcs = traj.arcs();
    if (arcs.empty()) return 0.0;

    double integral = 0.0;
    double coveredTime = 0.0;

    for (const auto& a : arcs) {
        double arcStart = a.t0;
        double arcEnd = a.t0 + a.duration;
        double overlapStart = std::max(arcStart, tLo);
        double overlapEnd = std::min(arcEnd, tHi);
        if (overlapEnd <= overlapStart) continue;

        double tauStart = overlapStart - a.t0;
        double tauEnd = overlapEnd - a.t0;

        // ∫ v_e(τ) dτ = α_e · ∫ v(τ) dτ
        double intEnd = a.extrusionRatio * (a.c0 * tauEnd
                        + 0.5 * a.c1 * tauEnd * tauEnd
                        + (1.0 / 3.0) * a.c2 * tauEnd * tauEnd * tauEnd
                        + 0.25 * a.c3 * tauEnd * tauEnd * tauEnd * tauEnd);
        double intStart = a.extrusionRatio * (a.c0 * tauStart
                          + 0.5 * a.c1 * tauStart * tauStart
                          + (1.0 / 3.0) * a.c2 * tauStart * tauStart * tauStart
                          + 0.25 * a.c3 * tauStart * tauStart * tauStart * tauStart);
        integral += intEnd - intStart;
        coveredTime += overlapEnd - overlapStart;
    }

    if (coveredTime <= 0.0) return 0.0;
    return integral / coveredTime;
}

// ============================================================================
// Polynomial antiderivative helpers
// ============================================================================

/**
 * @brief Antiderivative of (c0 + c1·τ + c2·τ² + c3·τ³)^n dτ for integer n.
 *
 * For integer n, the integrand is a polynomial of degree 3n, and the
 * antiderivative is a polynomial of degree 3n+1.  This function expands
 * the polynomial and integrates term by term.
 *
 * @param c0, c1, c2, c3 Polynomial coefficients of v(τ)
 * @param n Power (must be a non-negative integer)
 * @param tau Upper limit of integration
 * @return ∫₀^τ (c0 + c1·s + c2·s² + c3·s³)^n ds
 */
inline double polynomialPowerIntegral(double c0, double c1, double c2,
                                       double c3, int n, double tau) {
    if (n == 0) return tau;
    if (n == 1) {
        return c0 * tau + 0.5 * c1 * tau * tau
               + (1.0 / 3.0) * c2 * tau * tau * tau
               + 0.25 * c3 * tau * tau * tau * tau;
    }
    // For n >= 2, expand (c0 + c1·τ + c2·τ² + c3·τ³)^n by multinomial
    // and integrate term by term.
    // We use a simple recursive multiplication approach.
    // Start with [c0, c1, c2, c3] (coefficients of v), multiply n times.
    // Result is a polynomial of degree 3n with 3n+1 coefficients.
    std::vector<double> poly = {c0, c1, c2, c3};
    for (int p = 1; p < n; ++p) {
        std::vector<double> next(poly.size() + 3, 0.0);
        for (size_t i = 0; i < poly.size(); ++i) {
            next[i]     += poly[i] * c0;
            next[i + 1] += poly[i] * c1;
            next[i + 2] += poly[i] * c2;
            next[i + 3] += poly[i] * c3;
        }
        poly = std::move(next);
    }
    // Integrate: ∫₀^τ Σ a_k · s^k ds = Σ a_k · τ^(k+1) / (k+1)
    double result = 0.0;
    for (size_t k = 0; k < poly.size(); ++k) {
        result += poly[k] * std::pow(tau, static_cast<int>(k) + 1)
                  / static_cast<double>(k + 1);
    }
    return result;
}

/**
 * @brief Backward-compatible overload (c3 = 0).
 */
inline double polynomialPowerIntegral(double c0, double c1, double c2,
                                       int n, double tau) {
    return polynomialPowerIntegral(c0, c1, c2, 0.0, n, tau);
}

/**
 * @brief Antiderivative of (c0 + c1·τ)^n dτ for real n (closed form).
 *
 * ∫₀^τ (c0 + c1·s)^n ds = [(c0 + c1·τ)^(n+1) - c0^(n+1)] / (c1·(n+1))
 *
 * For c1 = 0: ∫₀^τ c0^n ds = c0^n · τ
 *
 * @param c0, c1 Linear polynomial coefficients
 * @param n Power (any real number, n > -1)
 * @param tau Upper limit
 * @return ∫₀^τ (c0 + c1·s)^n ds
 */
inline double linearPowerIntegral(double c0, double c1, double n, double tau) {
    if (std::abs(c1) < 1e-15) {
        return std::pow(c0, n) * tau;
    }
    double np1 = n + 1.0;
    double valEnd = std::pow(c0 + c1 * tau, np1);
    double valStart = std::pow(c0, np1);
    return (valEnd - valStart) / (c1 * np1);
}

/**
 * @brief Antiderivative of (c0 + c1·τ + c2·τ²)^n dτ for real n via
 *        Gauss-Legendre quadrature.
 *
 * For non-integer n and quadratic v(τ), no elementary closed form exists.
 * We use 8-point Gauss-Legendre quadrature, which is exact for polynomials
 * up to degree 15 — more than sufficient since the integrand is smooth
 * (v > 0 on each arc) and the arc duration is short.
 *
 * @param c0, c1, c2, c3 Polynomial coefficients of v(τ)
 * @param n Power (real, n > -1)
 * @param tau Upper limit
 * @return ∫₀^τ (c0 + c1·s + c2·s² + c3·s³)^n ds (to quadrature precision)
 */
inline double quadraticPowerIntegralGL(double c0, double c1, double c2,
                                        double c3, double n, double tau) {
    if (tau <= 0.0) return 0.0;
    // 8-point Gauss-Legendre on [0, tau]
    // Nodes and weights for [-1, 1]:
    static const double nodes[] = {
        -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
        -0.1834346424956498,  0.1834346424956498,  0.5255324099163290,
         0.7966664774136267,  0.9602898564975363
    };
    static const double weights[] = {
        0.1012285362903763, 0.2223810344533745, 0.3137066456877887,
        0.3626837833783620, 0.3626837833783620, 0.3137066456877887,
        0.2223810344533745, 0.1012285362903763
    };
    double halfTau = tau * 0.5;
    double sum = 0.0;
    for (int i = 0; i < 8; ++i) {
        double s = halfTau * (nodes[i] + 1.0);  // map [-1,1] → [0, tau]
        double v = c0 + c1 * s + c2 * s * s + c3 * s * s * s;
        if (v > 0.0) {
            sum += weights[i] * std::pow(v, n);
        }
    }
    return halfTau * sum;
}

/**
 * @brief Unified antiderivative of v(τ)^n dτ.
 *
 * Dispatches to the appropriate closed-form or quadrature based on the
 * arc type (c2 = 0 → linear, c2 ≠ 0 → quadratic) and n (integer vs real).
 *
 * @param c0, c1, c2, c3 Polynomial coefficients of v(τ)
 * @param n Power
 * @param tau Upper limit
 * @return ∫₀^τ v(s)^n ds
 */
inline double velocityPowerIntegral(double c0, double c1, double c2,
                                     double c3, double n, double tau) {
    if (tau <= 0.0) return 0.0;

    // Constant velocity (c1 = c2 = c3 = 0)
    if (std::abs(c1) < 1e-15 && std::abs(c2) < 1e-15
        && std::abs(c3) < 1e-15) {
        return std::pow(std::abs(c0), n) * tau;
    }

    // n = 1: integral is just the antiderivative of v(τ)
    if (std::abs(n - 1.0) < 1e-10) {
        return c0 * tau + 0.5 * c1 * tau * tau
               + (1.0 / 3.0) * c2 * tau * tau * tau
               + 0.25 * c3 * tau * tau * tau * tau;
    }

    // Linear velocity (c2 = c3 = 0) — closed form for any real n
    if (std::abs(c2) < 1e-15 && std::abs(c3) < 1e-15) {
        return linearPowerIntegral(c0, c1, n, tau);
    }

    // Quadratic or cubic velocity — use Gauss-Legendre quadrature
    return quadraticPowerIntegralGL(c0, c1, c2, c3, n, tau);
}

/// @brief Backwards-compatible overload (c3 = 0).
inline double velocityPowerIntegral(double c0, double c1, double c2,
                                     double n, double tau) {
    return velocityPowerIntegral(c0, c1, c2, 0.0, n, tau);
}

} // namespace MotionPlanner::analytical::extrusion
