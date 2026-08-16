/**
 * @file SCurveProfile.hpp
 * @brief 7-Phase S-Curve Velocity Profile Implementation
 *
 * @details
 * This file implements jerk-limited motion profiles using the classic
 * 7-phase S-curve approach, inspired by the Ruckig library's methodology.
 *
 * ## 7 Phases
 *
 * 1. **Jerk+**: Increasing acceleration (j = j_max)
 * 2. **Const Accel**: Constant acceleration (j = 0, a = a_max)
 * 3. **Jerk-**: Decreasing acceleration (j = -j_max)
 * 4. **Cruise**: Constant velocity (j = 0, a = 0, v = v_max)
 * 5. **Jerk-**: Decreasing acceleration (j = -j_max), start decel
 * 6. **Const Decel**: Constant deceleration (j = 0, a = -a_max)
 * 7. **Jerk+**: Increasing acceleration (j = j_max), reducing decel
 *
 * ## Features
 *
 * - Analytical position/velocity/acceleration/jerk at any time t
 * - Time-optimal profile computation
 * - Handling of short moves (merged phases)
 * - Support for non-zero start/end velocities
 *
 * @see VelocityProfile.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "SourceReference.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace MotionPlanner {

// ============================================================================
// S-Curve Phase Definitions
// ============================================================================

/**
 * @brief S-curve phase type
 */
enum class SCurvePhase : uint8_t {
    JerkAccelUp = 0,    ///< Phase 1: j = +j_max (increasing accel)
    ConstAccel = 1,     ///< Phase 2: j = 0, a = +a_max
    JerkAccelDown = 2,  ///< Phase 3: j = -j_max (decreasing accel)
    Cruise = 3,         ///< Phase 4: j = 0, a = 0, v = v_max
    JerkDecelUp = 4,    ///< Phase 5: j = -j_max (start decel)
    ConstDecel = 5,     ///< Phase 6: j = 0, a = -a_max
    JerkDecelDown = 6   ///< Phase 7: j = +j_max (reducing decel)
};

constexpr size_t NUM_SCURVE_PHASES = 7;

/**
 * @brief Get phase name for debugging
 */
inline const char* phaseName(SCurvePhase phase) {
    switch (phase) {
        case SCurvePhase::JerkAccelUp:   return "JerkAccelUp";
        case SCurvePhase::ConstAccel:    return "ConstAccel";
        case SCurvePhase::JerkAccelDown: return "JerkAccelDown";
        case SCurvePhase::Cruise:        return "Cruise";
        case SCurvePhase::JerkDecelUp:   return "JerkDecelUp";
        case SCurvePhase::ConstDecel:    return "ConstDecel";
        case SCurvePhase::JerkDecelDown: return "JerkDecelDown";
    }
    return "Unknown";
}

// ============================================================================
// S-Curve Phase Data
// ============================================================================

/**
 * @brief Data for a single S-curve phase
 */
template<typename T = double>
struct SCurvePhaseData {
    /// Duration of this phase (seconds)
    T duration = T(0);
    
    /// Jerk during this phase (constant)
    T jerk = T(0);
    
    /// Acceleration at start of phase
    T startAccel = T(0);
    
    /// Velocity at start of phase
    T startVelocity = T(0);
    
    /// Position at start of phase
    T startPosition = T(0);
    
    /**
     * @brief Evaluate position at time t within this phase
     *
     * p(t) = p₀ + v₀·t + ½·a₀·t² + ⅙·j·t³
     */
    T positionAt(T t) const {
        t = clamp(t, T(0), duration);
        return startPosition + 
               startVelocity * t + 
               T(0.5) * startAccel * t * t + 
               (T(1)/T(6)) * jerk * t * t * t;
    }
    
    /**
     * @brief Evaluate velocity at time t within this phase
     *
     * v(t) = v₀ + a₀·t + ½·j·t²
     */
    T velocityAt(T t) const {
        t = clamp(t, T(0), duration);
        return startVelocity + 
               startAccel * t + 
               T(0.5) * jerk * t * t;
    }
    
    /**
     * @brief Evaluate acceleration at time t within this phase
     *
     * a(t) = a₀ + j·t
     */
    T accelerationAt(T t) const {
        t = clamp(t, T(0), duration);
        return startAccel + jerk * t;
    }
    
    /**
     * @brief Evaluate jerk at time t (constant within phase)
     */
    T jerkAt(T /*t*/) const {
        return jerk;
    }
    
    /**
     * @brief Get final values at end of phase
     */
    T endPosition() const { return positionAt(duration); }
    T endVelocity() const { return velocityAt(duration); }
    T endAcceleration() const { return accelerationAt(duration); }
    
    /**
     * @brief Is this phase active (non-zero duration)?
     */
    bool isActive() const { return duration > MathConstants::EPSILON; }
};

// ============================================================================
// S-Curve Constraints
// ============================================================================

/**
 * @brief Constraints for S-curve profile computation
 */
template<typename T = double>
struct SCurveConstraints {
    /// Maximum velocity (units/second)
    T maxVelocity = T(100);
    
    /// Maximum acceleration (units/second²)
    T maxAcceleration = T(500);
    
    /// Maximum jerk (units/second³)
    T maxJerk = T(5000);
    
    /// Minimum duration for a phase to be considered active
    T minPhaseDuration = T(1e-9);
    
    /**
     * @brief Validate constraints
     */
    bool isValid() const {
        return maxVelocity > T(0) && 
               maxAcceleration > T(0) && 
               maxJerk > T(0);
    }
};

// ============================================================================
// S-Curve Motion State
// ============================================================================

/**
 * @brief Complete motion state at a point in time
 */
template<typename T = double>
struct SCurveState {
    T position = T(0);
    T velocity = T(0);
    T acceleration = T(0);
    T jerk = T(0);
    T time = T(0);
    SCurvePhase phase = SCurvePhase::JerkAccelUp;
};

// ============================================================================
// S-Curve Profile
// ============================================================================

/**
 * @brief Complete 7-phase S-curve motion profile
 */
template<typename T = double>
class SCurveProfile {
public:
    using Phase = SCurvePhaseData<T>;
    using State = SCurveState<T>;
    using Constraints = SCurveConstraints<T>;

    SCurveProfile() = default;

    /**
     * @brief Compute S-curve profile for a point-to-point move
     *
     * @param distance Total distance to travel
     * @param startVelocity Initial velocity
     * @param endVelocity Target final velocity
     * @param constraints Kinematic constraints
     * @return true if profile computed successfully
     */
    bool compute(T distance, T startVelocity, T endVelocity,
                 const Constraints& constraints) {
        if (!constraints.isValid() || distance < T(0)) {
            return false;
        }
        
        constraints_ = constraints;
        totalDistance_ = distance;
        
        T vMax = constraints.maxVelocity;
        T aMax = constraints.maxAcceleration;
        T jMax = constraints.maxJerk;
        
        // Time to reach max acceleration from zero: t_j = a_max / j_max
        T tJerk = aMax / jMax;
        
        // Velocity change during jerk phase: Δv_j = ½ · j_max · t_j²
        T deltaVJerk = T(0.5) * jMax * tJerk * tJerk;
        
        // Check if we can reach max velocity
        // Velocity gained during accel phases: 2·Δv_j + a_max·t_const
        
        // Compute target cruise velocity (may be limited by distance)
        T vCruise = computeCruiseVelocity(distance, startVelocity, endVelocity,
                                           vMax, aMax, jMax);
        
        // Compute phase durations
        computePhaseDurations(distance, startVelocity, endVelocity, vCruise,
                              aMax, jMax);
        
        // Fill in phase data
        fillPhaseData(startVelocity, jMax, aMax);
        
        // Verify profile
        return verify();
    }

    /**
     * @brief Evaluate state at time t
     */
    State evaluateAt(T t) const {
        State state;
        state.time = t;
        
        // Find which phase we're in
        T phaseStart = T(0);
        
        for (size_t i = 0; i < NUM_SCURVE_PHASES; ++i) {
            if (t < phaseStart + phases_[i].duration || i == NUM_SCURVE_PHASES - 1) {
                T localT = t - phaseStart;
                localT = clamp(localT, T(0), phases_[i].duration);
                
                state.position = phases_[i].positionAt(localT);
                state.velocity = phases_[i].velocityAt(localT);
                state.acceleration = phases_[i].accelerationAt(localT);
                state.jerk = phases_[i].jerkAt(localT);
                state.phase = static_cast<SCurvePhase>(i);
                return state;
            }
            phaseStart += phases_[i].duration;
        }
        
        // Past end - return final state
        state.position = phases_[NUM_SCURVE_PHASES - 1].endPosition();
        state.velocity = phases_[NUM_SCURVE_PHASES - 1].endVelocity();
        state.acceleration = T(0);
        state.jerk = T(0);
        state.phase = SCurvePhase::JerkDecelDown;
        
        return state;
    }

    /**
     * @brief Get total duration of profile
     */
    T totalDuration() const {
        T total = T(0);
        for (const auto& phase : phases_) {
            total += phase.duration;
        }
        return total;
    }

    /**
     * @brief Get total distance of profile
     */
    T totalDistance() const { return totalDistance_; }

    /**
     * @brief Access individual phases
     */
    const Phase& phase(SCurvePhase p) const { 
        return phases_[static_cast<size_t>(p)]; 
    }
    
    const Phase& phase(size_t i) const { return phases_[i]; }

    /**
     * @brief Get all phases
     */
    const std::array<Phase, NUM_SCURVE_PHASES>& phases() const { return phases_; }

    /**
     * @brief Check if profile is valid
     */
    bool isValid() const { return valid_; }

    /**
     * @brief Get phase boundaries (times)
     */
    std::array<T, NUM_SCURVE_PHASES + 1> phaseBoundaries() const {
        std::array<T, NUM_SCURVE_PHASES + 1> boundaries;
        boundaries[0] = T(0);
        for (size_t i = 0; i < NUM_SCURVE_PHASES; ++i) {
            boundaries[i + 1] = boundaries[i] + phases_[i].duration;
        }
        return boundaries;
    }

    /**
     * @brief Find time at which position is reached
     *
     * Uses Newton-Raphson iteration within each phase.
     */
    std::optional<T> findTimeAtPosition(T targetPosition, T tolerance = T(1e-9)) const {
        if (targetPosition < T(0) || targetPosition > totalDistance_) {
            return std::nullopt;
        }
        
        T phaseStart = T(0);
        
        for (size_t i = 0; i < NUM_SCURVE_PHASES; ++i) {
            const Phase& p = phases_[i];
            
            if (!p.isActive()) {
                continue;
            }
            
            T phaseEnd = phaseStart + p.duration;
            T startPos = p.startPosition;
            T endPos = p.endPosition();
            
            if (targetPosition >= startPos - tolerance && 
                targetPosition <= endPos + tolerance) {
                // Target is in this phase - solve analytically
                T localT = solvePhaseForPosition(p, targetPosition - startPos, tolerance);
                if (localT >= T(0)) {
                    return phaseStart + localT;
                }
            }
            
            phaseStart = phaseEnd;
        }
        
        return std::nullopt;
    }

    /**
     * @brief Compute distance to accelerate from v0 to v1 with jerk limiting
     *
     * This is the jerk-limited analogue of the kinematic equation
     * v² = v₀² + 2·a·Δs. It computes the exact arc length needed to
     * change velocity from v0 to v1, respecting max acceleration and
     * max jerk. Used by the jerk-limited TOPP-RA forward/backward passes.
     *
     * @param v0 Starting velocity
     * @param v1 Ending velocity (must be >= v0)
     * @param aMax Maximum acceleration
     * @param jMax Maximum jerk
     * @return Distance required for the velocity change
     */
    static T computeAccelDistance(T v0, T v1, T aMax, T jMax) {
        if (v1 <= v0) return T(0);
        // Degenerate limits: the velocity change is infeasible within any
        // finite distance. Returning infinity (rather than 0) makes the
        // binary search in maxVelocityAfterDistance degrade to v0 instead
        // of vMax — i.e. the profiler stays at the current velocity instead
        // of jumping to the ceiling (which would imply infinite acceleration).
        if (aMax <= T(0) || jMax <= T(0)) return std::numeric_limits<T>::infinity();

        T tJerk = aMax / jMax;
        T deltaV = v1 - v0;
        T deltaVJerk = T(0.5) * jMax * tJerk * tJerk;

        if (deltaV <= T(2) * deltaVJerk) {
            // Can't reach max acceleration: symmetric jerk-only phases
            T t = std::sqrt(deltaV / jMax);
            return v0 * T(2) * t + jMax * t * t * t;
        }

        // Full acceleration profile
        T tConst = (deltaV - T(2) * deltaVJerk) / aMax;

        T d1 = v0 * tJerk + (T(1)/T(6)) * jMax * tJerk * tJerk * tJerk;
        T v1_temp = v0 + deltaVJerk;
        T d2 = v1_temp * tConst + T(0.5) * aMax * tConst * tConst;
        T v2_temp = v1_temp + aMax * tConst;
        T d3 = v2_temp * tJerk + T(0.5) * aMax * tJerk * tJerk -
               (T(1)/T(6)) * jMax * tJerk * tJerk * tJerk;

        return d1 + d2 + d3;
    }

    /**
     * @brief Compute distance to decelerate from v0 to v1 with jerk limiting
     *
     * Symmetric to computeAccelDistance. Computes the arc length needed
     * to reduce velocity from v0 to v1 (v1 <= v0).
     *
     * @param v0 Starting velocity
     * @param v1 Ending velocity (must be <= v0)
     * @param aMax Maximum deceleration magnitude
     * @param jMax Maximum jerk
     * @return Distance required for the velocity change
     */
    static T computeDecelDistance(T v0, T v1, T aMax, T jMax) {
        return computeAccelDistance(v1, v0, aMax, jMax);
    }

    // ========================================================================
    // WI-8: State-aware 3rd-order functions (carry acceleration as state)
    // ========================================================================
    //
    // These functions generalize computeAccelDistance / maxVelocityAfterDistance
    // to carry the acceleration as state, enabling a true 3rd-order TOPP-RA
    // that does NOT force a = 0 at every sample point. See
    // docs/motion/ToppraDerivation.md (T.5b, T.6b) for the derivation.

    /**
     * @brief Solve the cubic  d = v0·t + ½·a0·t² + (1/6)·j·t³  for t.
     *
     * Used when the available distance is too short to complete the jerk
     * ramp from a0 to aMax. The cubic is strictly monotone in t for
     * t > 0 (its derivative  v0 + a0·t + ½·j·t²  is positive when
     * v0 > 0), so Newton's method converges in 3–5 iterations.
     */
    static T solveAccelCubic(T v0, T a0, T jMax, T distance) {
        if (distance <= T(0)) return T(0);
        // Initial guess: ignore acceleration terms.
        T t = (v0 > T(0)) ? distance / v0 : std::cbrt(T(6) * distance / jMax);
        for (int iter = 0; iter < 20; ++iter) {
            T f = v0 * t + T(0.5) * a0 * t * t
                  + (T(1) / T(6)) * jMax * t * t * t - distance;
            T fp = v0 + a0 * t + T(0.5) * jMax * t * t;
            if (std::abs(fp) < T(1e-15)) break;
            T dt = f / fp;
            t -= dt;
            if (std::abs(dt) < T(1e-12) * std::max(t, T(1))) break;
        }
        return std::max(t, T(0));
    }

    /**
     * @brief Distance to accelerate from (v0, a0) to (v1, 0) with
     *        jerk-bang-bang control, |a| ≤ aMax, |j| ≤ jMax.
     *
     * Generalizes computeAccelDistance to handle non-zero initial
     * acceleration.
     *
     * **WI-8b.1 — Feasibility domain (a0 > 0):**
     * When a0 > 0, the acceleration must be shed to 0 before the velocity
     * reaches v1. Shedding a0 (ramping a from a0 to 0 with j = −jMax)
     * mandatorily gains Δv_shed = a0²/(2·jMax) of velocity. If
     * Δv = v1 − v0 < Δv_shed, reaching (v1, a = 0) is physically
     * impossible — shedding a0 alone overshoots v1. In that case the
     * function returns infinity (infeasible), so the binary search in
     * maxVelocityWithState degrades to v0 instead of producing a
     * garbage finite distance that implies infinite jerk.
     *
     * For the feasible regime (Δv ≥ Δv_shed), the distance-minimal
     * *feasible* control sheds a0 first (j = −jMax, a: a0 → 0), then
     * solves the a0 = 0 problem (computeAccelDistance) from
     * (v0 + Δv_shed, 0) to (v1, 0) for the remainder. This is slightly
     * more conservative than the "ramp up to aMax first" structure (which
     * is only valid for Δv ≥ (aMax²−a0²)/(2j) + aMax²/(2j)), but it is
     * always feasible and never produces negative phase times.
     *
     * For a0 ≤ 0, the existing triangular/trapezoidal structure is
     * correct (t1 = (a_peak − a0)/jMax > 0 always since a_peak ≥ 0 ≥ a0),
     * so it is retained.
     *
     * @param v0 Starting velocity
     * @param a0 Starting acceleration (may be non-zero, clamped to [-aMax, aMax])
     * @param v1 Ending velocity (must be >= v0)
     * @param aMax Maximum acceleration magnitude
     * @param jMax Maximum jerk magnitude
     * @return Distance required for the velocity change, or infinity if
     *         (v1, a = 0) is unreachable from (v0, a0) under the jerk bound
     */
    static T computeAccelDistanceWithState(T v0, T a0, T v1,
                                             T aMax, T jMax) {
        if (v1 <= v0) return T(0);
        if (aMax <= T(0) || jMax <= T(0))
            return std::numeric_limits<T>::infinity();

        a0 = std::clamp(a0, -aMax, aMax);
        T deltaV = v1 - v0;

        if (a0 > T(0)) {
            // WI-8b.1: shed a0 first (j = -jMax, a: a0 → 0).
            // Mandatory velocity gain: Δv_shed = a0²/(2·jMax).
            T deltaVShed = a0 * a0 / (T(2) * jMax);
            if (deltaV < deltaVShed - T(1e-12)) {
                // Infeasible: shedding a0 alone overshoots v1.
                return std::numeric_limits<T>::infinity();
            }
            // Phase A: ramp a from a0 to 0 with j = -jMax.
            T tA = a0 / jMax;
            T dA = v0 * tA + T(0.5) * a0 * tA * tA
                   - (T(1) / T(6)) * jMax * tA * tA * tA;
            T vA = v0 + deltaVShed;
            // Phase B: solve the a0 = 0 problem from (vA, 0) to (v1, 0).
            T dB = computeAccelDistance(vA, v1, aMax, jMax);
            return dA + dB;
        }

        // a0 ≤ 0: existing triangular/trapezoidal structure (t1 > 0 always).
        // Velocity gain in phase 1 (ramp a0 → aMax): (aMax² − a0²)/(2·jMax)
        T deltaV1_trap = (aMax * aMax - a0 * a0) / (T(2) * jMax);
        // Velocity gain in phase 3 (ramp aMax → 0): aMax²/(2·jMax)
        T deltaV3 = aMax * aMax / (T(2) * jMax);

        if (deltaV >= deltaV1_trap + deltaV3) {
            // Trapezoidal: a reaches aMax
            T t1 = (aMax - a0) / jMax;
            T t3 = aMax / jMax;
            T v_after_1 = v0 + deltaV1_trap;
            T deltaV2 = deltaV - deltaV1_trap - deltaV3;
            T t2 = deltaV2 / aMax;

            T d1 = v0 * t1 + T(0.5) * a0 * t1 * t1
                   + (T(1) / T(6)) * jMax * t1 * t1 * t1;
            T d2 = v_after_1 * t2 + T(0.5) * aMax * t2 * t2;
            T v_after_2 = v_after_1 + aMax * t2;
            T d3 = v_after_2 * t3 + T(0.5) * aMax * t3 * t3
                   - (T(1) / T(6)) * jMax * t3 * t3 * t3;
            return d1 + d2 + d3;
        } else {
            // Triangular: a doesn't reach aMax
            // 2·a_peak² = 2·jMax·Δv + a0²  →  a_peak = √(jMax·Δv + a0²/2)
            T a_peak_sq = jMax * deltaV + a0 * a0 / T(2);
            if (a_peak_sq <= T(0)) return T(0);
            T a_peak = std::sqrt(a_peak_sq);

            T t1 = (a_peak - a0) / jMax;
            T t3 = a_peak / jMax;
            T deltaV1 = (a_peak * a_peak - a0 * a0) / (T(2) * jMax);
            T v_after_1 = v0 + deltaV1;

            T d1 = v0 * t1 + T(0.5) * a0 * t1 * t1
                   + (T(1) / T(6)) * jMax * t1 * t1 * t1;
            T d3 = v_after_1 * t3 + T(0.5) * a_peak * t3 * t3
                   - (T(1) / T(6)) * jMax * t3 * t3 * t3;
            return d1 + d3;
        }
    }

    /**
     * @brief Distance to decelerate from (v0, a0) to (v1, 0) with
     *        jerk-bang-bang control.
     *
     * By time-reversal symmetry, this equals the acceleration distance
     * from (v1, 0) to (v0, -a0).
     */
    static T computeDecelDistanceWithState(T v0, T a0, T v1,
                                             T aMax, T jMax) {
        return computeAccelDistanceWithState(v1, -a0, v0, aMax, jMax);
    }

    /**
     * @brief Maximum velocity reachable from (v0, a0) over distance d,
     *        carrying acceleration state.
     *
     * Generalizes maxVelocityAfterDistance to carry the acceleration.
     * The optimal control is jerk-bang-bang: j = +jMax until a = aMax,
     * then hold a = aMax. If the resulting v would exceed vMax, the
     * function plans a jerk-limited approach that arrives at vMax with
     * a = 0 (cruise).
     *
     * **WI-8b.2 — Shed-acceleration ceiling constraint:**
     * A returned state (v1, a1) with a1 > 0 is only feasible w.r.t. the
     * ceiling if the acceleration can be shed before v exceeds vMax:
     *   a1² / (2·jMax) ≤ vMax − v1
     * (the LHS is the velocity gained while ramping a from a1 to 0 with
     * j = −jMax). If the uncapped result violates this, the function
     * finds the point where the trajectory crosses the shed boundary
     * (v + a²/(2·jMax) = vMax) and follows the boundary (j = −jMax)
     * for the remaining distance. This makes ceiling arrival tangent
     * (a → 0 as v → vMax) and carries a non-zero acceleration right up
     * to the shed boundary, avoiding the over-conservative "force a = 0"
     * behavior that would make the profile grid-dependent again.
     *
     * @param v0 Starting velocity
     * @param a0 Starting acceleration
     * @param distance Available distance
     * @param vMax Velocity ceiling (from v_lim or feed rate)
     * @param aMax Maximum acceleration
     * @param jMax Maximum jerk
     * @return Pair (max reachable velocity, acceleration at that velocity)
     */
    static std::pair<T, T> maxVelocityWithState(T v0, T a0, T distance,
                                                  T vMax, T aMax, T jMax) {
        if (distance <= T(0)) return {v0, a0};
        if (v0 >= vMax) return {vMax, T(0)};
        if (aMax <= T(0) || jMax <= T(0)) return {v0, a0};

        T a0c = std::clamp(a0, -aMax, aMax);

        // --- Uncapped max v1 (no vMax constraint) ---
        T v1_uncapped, a1_uncapped;
        if (a0c < aMax - T(1e-12)) {
            // Phase 1: ramp from a0 to aMax
            T t1 = (aMax - a0c) / jMax;
            T d1 = v0 * t1 + T(0.5) * a0c * t1 * t1
                   + (T(1) / T(6)) * jMax * t1 * t1 * t1;
            T v_after_1 = v0 + a0c * t1 + T(0.5) * jMax * t1 * t1;

            if (d1 >= distance) {
                // Can't complete the jerk ramp — solve cubic
                T t = solveAccelCubic(v0, a0c, jMax, distance);
                v1_uncapped = v0 + a0c * t + T(0.5) * jMax * t * t;
                a1_uncapped = a0c + jMax * t;
            } else {
                // Phase 2: constant a = aMax for remaining distance
                T d2 = distance - d1;
                T disc = v_after_1 * v_after_1 + T(2) * aMax * d2;
                T t2 = (-v_after_1 + std::sqrt(std::max(disc, T(0)))) / aMax;
                v1_uncapped = v_after_1 + aMax * t2;
                a1_uncapped = aMax;
            }
        } else {
            // a0 >= aMax — constant aMax
            T disc = v0 * v0 + T(2) * aMax * distance;
            T t = (-v0 + std::sqrt(std::max(disc, T(0)))) / aMax;
            v1_uncapped = v0 + aMax * t;
            a1_uncapped = aMax;
        }

        // WI-8b.2: Check both the velocity ceiling AND the shed-acceleration
        // constraint. A state (v1, a1) with a1 > 0 is feasible only if
        // a1²/(2·jMax) ≤ vMax − v1 (acceleration can be shed before v
        // exceeds the ceiling).
        T shedVel = (a1_uncapped > T(0))
            ? a1_uncapped * a1_uncapped / (T(2) * jMax) : T(0);
        if (v1_uncapped <= vMax && v1_uncapped + shedVel <= vMax + T(1e-9)) {
            return {std::min(v1_uncapped, vMax), a1_uncapped};
        }

        // --- Uncapped result violates the shed constraint or vMax ---
        // Find where the trajectory crosses the shed boundary
        //   v(t) + a(t)²/(2·jMax) = vMax
        // and follow the boundary (j = −jMax) for the remaining distance.
        //
        // The shed value along the trajectory in phase 1 (j = +jMax):
        //   v(t) + a(t)²/(2·jMax) = v0 + a0²/(2·jMax) + 2·a0·t + jMax·t²
        // which is a quadratic in t. In phase 2 (a = aMax):
        //   v(t) + aMax²/(2·jMax) = vMax  →  v(t) = vMax − aMax²/(2·jMax).

        T shed0 = v0 + a0c * a0c / (T(2) * jMax);
        if (shed0 >= vMax - T(1e-9)) {
            // Already on the shed boundary — follow it (j = −jMax) to
            // shed acceleration and gain velocity toward vMax. The
            // trajectory stays on the boundary v + a²/(2·jMax) = vMax.
            T tShed = solveAccelCubic(v0, a0c, -jMax, distance);
            T v1 = v0 + a0c * tShed - T(0.5) * jMax * tShed * tShed;
            T a1 = a0c - jMax * tShed;
            if (a1 <= T(1e-9)) {
                return {vMax, T(0)};
            }
            return {v1, a1};
        }

        // Try crossing in phase 1 (j = +jMax):
        // jMax·t² + 2·a0·t + (shed0 − vMax) = 0
        // t = (−a0 + √(a0²/2 + jMax·(vMax − v0))) / jMax
        T disc1 = a0c * a0c / T(2) + jMax * (vMax - v0);
        T t1_full = (a0c < aMax - T(1e-12))
            ? (aMax - a0c) / jMax : T(0);
        T d1_full = (t1_full > T(0))
            ? v0 * t1_full + T(0.5) * a0c * t1_full * t1_full
              + (T(1) / T(6)) * jMax * t1_full * t1_full * t1_full
            : T(0);
        T v_after_1 = v0 + a0c * t1_full + T(0.5) * jMax * t1_full * t1_full;

        T dCross, vCross, aCross;
        bool crossingFound = false;

        if (disc1 > T(0) && (a0c >= aMax - T(1e-12) ||
                             (-a0c + std::sqrt(disc1)) / jMax <= t1_full)) {
            // Crossing in phase 1 (or a0 >= aMax with constant aMax).
            if (a0c >= aMax - T(1e-12)) {
                // Constant aMax phase: v + aMax²/(2jMax) = vMax
                vCross = vMax - aMax * aMax / (T(2) * jMax);
                if (vCross <= v0) {
                    // Already past the crossing — follow the boundary.
                    T tShed = solveAccelCubic(v0, a0c, -jMax, distance);
                    T v1 = v0 + a0c * tShed - T(0.5) * jMax * tShed * tShed;
                    T a1 = a0c - jMax * tShed;
                    if (a1 <= T(1e-9)) return {vMax, T(0)};
                    return {v1, a1};
                }
                T tCross = (vCross - v0) / aMax;
                dCross = v0 * tCross + T(0.5) * aMax * tCross * tCross;
                aCross = aMax;
            } else {
                T tCross = (-a0c + std::sqrt(disc1)) / jMax;
                dCross = v0 * tCross + T(0.5) * a0c * tCross * tCross
                         + (T(1) / T(6)) * jMax * tCross * tCross * tCross;
                vCross = v0 + a0c * tCross + T(0.5) * jMax * tCross * tCross;
                aCross = a0c + jMax * tCross;
            }
            crossingFound = true;
        } else if (t1_full > T(0)) {
            // Crossing in phase 2 (constant a = aMax).
            vCross = vMax - aMax * aMax / (T(2) * jMax);
            if (vCross > v_after_1) {
                T t2Cross = (vCross - v_after_1) / aMax;
                dCross = d1_full + v_after_1 * t2Cross
                         + T(0.5) * aMax * t2Cross * t2Cross;
                aCross = aMax;
                crossingFound = true;
            }
        }

        if (!crossingFound) {
            // Fallback: binary search for max v1 with a = 0.
            T vLow = v0, vHigh = vMax;
            for (int iter = 0; iter < 60; ++iter) {
                T vMid = (vLow + vHigh) / T(2);
                T needed = computeAccelDistanceWithState(v0, a0, vMid, aMax, jMax);
                if (needed <= distance) vLow = vMid;
                else vHigh = vMid;
            }
            return {vLow, T(0)};
        }

        if (dCross >= distance) {
            // Crossing happens after distance d — the uncapped trajectory
            // at distance d hasn't reached the boundary yet. But the
            // uncapped result violated the shed constraint, which means
            // d < dCross is impossible. Use the uncapped result clamped
            // to vMax (shouldn't normally reach here).
            return {std::min(v1_uncapped, vMax), a1_uncapped};
        }

        // Follow the shed boundary (j = −jMax) for the remaining distance.
        T dRemaining = distance - dCross;
        // Solve: dRem = vCross·t + ½·aCross·t² − (1/6)·jMax·t³
        // (cubic with negative jerk — use solveAccelCubic with j = −jMax)
        T tShed = solveAccelCubic(vCross, aCross, -jMax, dRemaining);
        T v1 = vCross + aCross * tShed - T(0.5) * jMax * tShed * tShed;
        T a1 = aCross - jMax * tShed;

        if (a1 <= T(1e-9)) {
            // Fully shed — arrived at vMax with a = 0.
            return {vMax, T(0)};
        }
        return {v1, a1};
    }

    /**
     * @brief Maximum entry velocity that allows reaching (v1, a1) over
     *        distance d, carrying acceleration state (backward pass).
     *
     * By time-reversal symmetry, this is the forward problem from
     * (v1, -a1) with negated result acceleration.
     *
     * @param v1 Target ending velocity
     * @param a1 Target ending acceleration
     * @param distance Available distance
     * @param vMax Velocity ceiling
     * @param aMax Maximum acceleration magnitude
     * @param jMax Maximum jerk
     * @return Pair (max entry velocity, entry acceleration)
     */
    static std::pair<T, T> maxEntryVelocityWithState(T v1, T a1, T distance,
                                                       T vMax, T aMax,
                                                       T jMax) {
        auto [v0, a0_neg] = maxVelocityWithState(v1, -a1, distance,
                                                   vMax, aMax, jMax);
        return {v0, -a0_neg};
    }

    /**
     * @brief Derivative of computeAccelDistance with respect to v1.
     *
     * WI-P1: Used by Newton's method in maxVelocityAfterDistance.
     * The derivative is piecewise:
     * - Triangular case (Δv ≤ 2·ΔvJerk): dd/dv1 = (3·v1 − v0) / (2·jMax·t)
     *   where t = √(Δv / jMax)
     * - Trapezoidal case: dd/dv1 = (v1 − ΔvJerk) / aMax + tJerk
     */
    static T computeAccelDistanceDerivative(T v0, T v1, T aMax, T jMax) {
        if (v1 <= v0) return T(0);
        T tJerk = aMax / jMax;
        T deltaV = v1 - v0;
        T deltaVJerk = T(0.5) * jMax * tJerk * tJerk;

        if (deltaV <= T(2) * deltaVJerk) {
            // Triangular: d = 2·v0·t + jMax·t³, t = √(Δv/jMax)
            T t = std::sqrt(deltaV / jMax);
            if (t < T(1e-15)) return T(0);
            return (T(2) * v0 + T(3) * jMax * t * t) / (T(2) * jMax * t);
        } else {
            // Trapezoidal: dd/dv1 = (v1 − ΔvJerk) / aMax + tJerk
            return (v1 - deltaVJerk) / aMax + tJerk;
        }
    }

    /**
     * @brief Find the maximum velocity reachable from v0 over distance d,
     *        respecting jerk-limited acceleration.
     *
     * Used by the jerk-limited TOPP-RA forward pass: given the current
     * velocity v0 and the available distance Δs to the next sample,
     * compute the maximum v1 such that
     * computeAccelDistance(v0, v1, aMax, jMax) <= d.
     *
     * WI-P1: Uses Newton's method with the analytical derivative of
     * computeAccelDistance, converging in 2–4 iterations. Falls back to
     * bisection if Newton diverges or goes out of bounds.
     *
     * @param v0 Starting velocity
     * @param distance Available distance
     * @param vMax Velocity ceiling (from v_lim or feed rate)
     * @param aMax Maximum acceleration
     * @param jMax Maximum jerk
     * @return Maximum reachable velocity
     */
    static T maxVelocityAfterDistance(T v0, T distance, T vMax,
                                       T aMax, T jMax) {
        if (distance <= T(0)) return v0;
        if (v0 >= vMax) return vMax;
        if (aMax <= T(0) || jMax <= T(0)) return v0;

        // WI-P1: Newton's method with bisection fallback.
        // f(v1) = computeAccelDistance(v0, v1, aMax, jMax) - distance
        // f'(v1) = computeAccelDistanceDerivative(v0, v1, aMax, jMax)
        // We want the root of f in [v0, vMax].

        // Initial guess: linear approximation (ignore jerk ramp cost).
        T v1 = v0 + std::min(distance * aMax / (T(2) * v0 + T(1)),
                              vMax - v0);
        v1 = std::clamp(v1, v0, vMax);

        T vLow = v0, vHigh = vMax;
        for (int iter = 0; iter < 20; ++iter) {
            T needed = computeAccelDistance(v0, v1, aMax, jMax);
            T f = needed - distance;
            T fp = computeAccelDistanceDerivative(v0, v1, aMax, jMax);

            // Update bisection bracket.
            if (f <= T(0)) vLow = v1;
            else vHigh = v1;

            if (std::abs(f) < T(1e-10) * std::max(distance, T(1))) break;

            // Newton step.
            if (fp > T(1e-15)) {
                T vNew = v1 - f / fp;
                if (vNew > vLow && vNew < vHigh) {
                    v1 = vNew;
                    continue;
                }
            }
            // Fallback: bisection.
            v1 = (vLow + vHigh) / T(2);
        }
        return std::clamp(v1, vLow, vHigh);
    }

private:
    /**
     * @brief Compute cruise velocity for given distance
     */
    T computeCruiseVelocity(T distance, T v0, T vf, T vMax, T aMax, T jMax) const {
        // Time for jerk phases
        T tJerk = aMax / jMax;
        
        // Compute minimum distance needed to accelerate to vMax and decelerate
        T accelDist = computeAccelDistance(v0, vMax, aMax, jMax);
        T decelDist = computeDecelDistance(vMax, vf, aMax, jMax);
        T minDist = accelDist + decelDist;
        
        if (distance >= minDist) {
            // We can reach max velocity
            return vMax;
        }
        
        // Need to reduce cruise velocity
        // Use iterative approach to find appropriate cruise velocity
        T vLow = std::max(v0, vf);
        T vHigh = vMax;
        
        for (int iter = 0; iter < 50; ++iter) {
            T vMid = (vLow + vHigh) / T(2);
            
            T accel = computeAccelDistance(v0, vMid, aMax, jMax);
            T decel = computeDecelDistance(vMid, vf, aMax, jMax);
            T needed = accel + decel;
            
            if (std::abs(needed - distance) < T(1e-9)) {
                return vMid;
            }
            
            if (needed < distance) {
                vLow = vMid;
            } else {
                vHigh = vMid;
            }
        }
        
        return (vLow + vHigh) / T(2);
    }

    /**
     * @brief Compute durations for all phases
     */
    void computePhaseDurations(T distance, T v0, T vf, T vCruise, T aMax, T jMax) {
        T tJerk = aMax / jMax;
        T deltaVJerk = T(0.5) * jMax * tJerk * tJerk;
        
        // Acceleration phases (1, 2, 3)
        T accelDelta = vCruise - v0;
        
        if (accelDelta > T(0)) {
            if (accelDelta <= T(2) * deltaVJerk) {
                // Can't reach max acceleration - merge phases 1 and 3
                T t = std::sqrt(accelDelta / jMax);
                phaseDurations_[0] = t;
                phaseDurations_[1] = T(0);
                phaseDurations_[2] = t;
            } else {
                phaseDurations_[0] = tJerk;
                phaseDurations_[1] = (accelDelta - T(2) * deltaVJerk) / aMax;
                phaseDurations_[2] = tJerk;
            }
        } else {
            phaseDurations_[0] = T(0);
            phaseDurations_[1] = T(0);
            phaseDurations_[2] = T(0);
        }
        
        // Deceleration phases (5, 6, 7)
        T decelDelta = vCruise - vf;
        
        if (decelDelta > T(0)) {
            if (decelDelta <= T(2) * deltaVJerk) {
                T t = std::sqrt(decelDelta / jMax);
                phaseDurations_[4] = t;
                phaseDurations_[5] = T(0);
                phaseDurations_[6] = t;
            } else {
                phaseDurations_[4] = tJerk;
                phaseDurations_[5] = (decelDelta - T(2) * deltaVJerk) / aMax;
                phaseDurations_[6] = tJerk;
            }
        } else {
            phaseDurations_[4] = T(0);
            phaseDurations_[5] = T(0);
            phaseDurations_[6] = T(0);
        }
        
        // Cruise phase (4) - whatever distance is left
        T accelDist = computeAccelDistance(v0, vCruise, aMax, jMax);
        T decelDist = computeDecelDistance(vCruise, vf, aMax, jMax);
        T cruiseDist = distance - accelDist - decelDist;
        
        if (cruiseDist > MathConstants::EPSILON) {
            phaseDurations_[3] = cruiseDist / vCruise;
        } else {
            phaseDurations_[3] = T(0);
        }
    }

    /**
     * @brief Fill in complete phase data
     */
    void fillPhaseData(T v0, T jMax, T aMax) {
        T pos = T(0);
        T vel = v0;
        T accel = T(0);
        
        // Phase 1: Jerk+
        phases_[0].duration = phaseDurations_[0];
        phases_[0].jerk = jMax;
        phases_[0].startAccel = accel;
        phases_[0].startVelocity = vel;
        phases_[0].startPosition = pos;
        
        if (phases_[0].isActive()) {
            pos = phases_[0].endPosition();
            vel = phases_[0].endVelocity();
            accel = phases_[0].endAcceleration();
        }
        
        // Phase 2: Const Accel
        phases_[1].duration = phaseDurations_[1];
        phases_[1].jerk = T(0);
        phases_[1].startAccel = accel;
        phases_[1].startVelocity = vel;
        phases_[1].startPosition = pos;
        
        if (phases_[1].isActive()) {
            pos = phases_[1].endPosition();
            vel = phases_[1].endVelocity();
            accel = phases_[1].endAcceleration();
        }
        
        // Phase 3: Jerk-
        phases_[2].duration = phaseDurations_[2];
        phases_[2].jerk = -jMax;
        phases_[2].startAccel = accel;
        phases_[2].startVelocity = vel;
        phases_[2].startPosition = pos;
        
        if (phases_[2].isActive()) {
            pos = phases_[2].endPosition();
            vel = phases_[2].endVelocity();
            accel = T(0);  // Should be zero after accel
        }
        
        // Phase 4: Cruise
        phases_[3].duration = phaseDurations_[3];
        phases_[3].jerk = T(0);
        phases_[3].startAccel = T(0);
        phases_[3].startVelocity = vel;
        phases_[3].startPosition = pos;
        
        if (phases_[3].isActive()) {
            pos = phases_[3].endPosition();
            // vel stays same
        }
        
        // Phase 5: Jerk- (start decel)
        phases_[4].duration = phaseDurations_[4];
        phases_[4].jerk = -jMax;
        phases_[4].startAccel = T(0);
        phases_[4].startVelocity = vel;
        phases_[4].startPosition = pos;
        
        if (phases_[4].isActive()) {
            pos = phases_[4].endPosition();
            vel = phases_[4].endVelocity();
            accel = phases_[4].endAcceleration();
        }
        
        // Phase 6: Const Decel
        phases_[5].duration = phaseDurations_[5];
        phases_[5].jerk = T(0);
        phases_[5].startAccel = accel;
        phases_[5].startVelocity = vel;
        phases_[5].startPosition = pos;
        
        if (phases_[5].isActive()) {
            pos = phases_[5].endPosition();
            vel = phases_[5].endVelocity();
            accel = phases_[5].endAcceleration();
        }
        
        // Phase 7: Jerk+
        phases_[6].duration = phaseDurations_[6];
        phases_[6].jerk = jMax;
        phases_[6].startAccel = accel;
        phases_[6].startVelocity = vel;
        phases_[6].startPosition = pos;
    }

    /**
     * @brief Verify profile correctness
     */
    bool verify() {
        T finalPos = phases_[NUM_SCURVE_PHASES - 1].endPosition();
        T error = std::abs(finalPos - totalDistance_);
        
        valid_ = (error < totalDistance_ * T(0.001) + T(1e-9));
        return valid_;
    }

    /**
     * @brief Solve for time at position within a phase
     */
    T solvePhaseForPosition(const Phase& p, T targetDelta, T tolerance) const {
        // p(t) = v₀·t + ½·a₀·t² + ⅙·j·t³
        // Newton-Raphson to solve p(t) = targetDelta
        
        T t = p.duration / T(2);  // Initial guess
        
        for (int iter = 0; iter < 20; ++iter) {
            T pos = p.startVelocity * t + 
                    T(0.5) * p.startAccel * t * t + 
                    (T(1)/T(6)) * p.jerk * t * t * t;
            
            T vel = p.startVelocity + 
                    p.startAccel * t + 
                    T(0.5) * p.jerk * t * t;
            
            T error = pos - targetDelta;
            
            if (std::abs(error) < tolerance) {
                return clamp(t, T(0), p.duration);
            }
            
            if (std::abs(vel) > MathConstants::EPSILON) {
                t -= error / vel;
                t = clamp(t, T(0), p.duration);
            }
        }
        
        return t;
    }

    std::array<Phase, NUM_SCURVE_PHASES> phases_;
    std::array<T, NUM_SCURVE_PHASES> phaseDurations_{};
    Constraints constraints_;
    T totalDistance_ = T(0);
    bool valid_ = false;
};

// ============================================================================
// S-Curve Profile Builder
// ============================================================================

/**
 * @brief Builds S-curve profiles for path segments
 */
template<typename T = double>
class SCurveProfileBuilder {
public:
    using Profile = SCurveProfile<T>;
    using Constraints = SCurveConstraints<T>;

    /**
     * @brief Constructor
     */
    explicit SCurveProfileBuilder(Constraints constraints = {})
        : constraints_(std::move(constraints)) {}

    /**
     * @brief Build profile for a motion segment
     *
     * @param distance Segment length
     * @param startVelocity Entry velocity
     * @param endVelocity Exit velocity
     * @return Computed S-curve profile
     */
    Profile buildProfile(T distance, T startVelocity, T endVelocity) {
        Profile profile;
        profile.compute(distance, startVelocity, endVelocity, constraints_);
        return profile;
    }

    /**
     * @brief Build profiles for a sequence of segments
     *
     * Ensures velocity continuity between segments.
     *
     * @param distances Segment lengths
     * @param feedRates Target feed rates per segment
     * @return Vector of S-curve profiles
     */
    std::vector<Profile> buildProfiles(const std::vector<T>& distances,
                                        const std::vector<T>& feedRates) {
        std::vector<Profile> profiles;
        profiles.reserve(distances.size());
        
        T currentVelocity = T(0);
        
        for (size_t i = 0; i < distances.size(); ++i) {
            T targetVelocity = (i < feedRates.size()) ? 
                               std::min(feedRates[i], constraints_.maxVelocity) :
                               constraints_.maxVelocity;
            
            // Next segment's velocity (or zero at end)
            T nextVelocity = T(0);
            if (i + 1 < distances.size()) {
                nextVelocity = (i + 1 < feedRates.size()) ?
                               std::min(feedRates[i + 1], constraints_.maxVelocity) :
                               constraints_.maxVelocity;
            }
            
            // Constrain exit velocity for smooth transition
            T exitVelocity = std::min(targetVelocity, nextVelocity);
            
            Profile profile;
            profile.compute(distances[i], currentVelocity, exitVelocity, constraints_);
            profiles.push_back(std::move(profile));
            
            currentVelocity = profiles.back().evaluateAt(
                profiles.back().totalDuration()).velocity;
        }
        
        return profiles;
    }

    /**
     * @brief Get/set constraints
     */
    Constraints& constraints() { return constraints_; }
    const Constraints& constraints() const { return constraints_; }

private:
    Constraints constraints_;
};

// ============================================================================
// Type Aliases
// ============================================================================

using SCurvePhaseDataD = SCurvePhaseData<double>;
using SCurveConstraintsD = SCurveConstraints<double>;
using SCurveStateD = SCurveState<double>;
using SCurveProfileD = SCurveProfile<double>;
using SCurveProfileBuilderD = SCurveProfileBuilder<double>;

}  // namespace MotionPlanner
