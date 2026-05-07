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
#include <optional>
#include <string>

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
     * @brief Compute distance to accelerate from v0 to v1
     */
    T computeAccelDistance(T v0, T v1, T aMax, T jMax) const {
        if (v1 <= v0) return T(0);
        
        T tJerk = aMax / jMax;
        T deltaV = v1 - v0;
        T deltaVJerk = T(0.5) * jMax * tJerk * tJerk;
        
        if (deltaV <= T(2) * deltaVJerk) {
            // Can't reach max acceleration: symmetric jerk-only phases
            T t = std::sqrt(deltaV / jMax);
            // Distance for two jerk phases (phase1 + phase3): 2*v0*t + j*t^3
            return v0 * T(2) * t + jMax * t * t * t;
        }
        
        // Full acceleration profile
        T tConst = (deltaV - T(2) * deltaVJerk) / aMax;
        
        // Distance = jerk phases + const accel phase
        T d1 = v0 * tJerk + (T(1)/T(6)) * jMax * tJerk * tJerk * tJerk;  // First jerk
        T v1_temp = v0 + deltaVJerk;
        T d2 = v1_temp * tConst + T(0.5) * aMax * tConst * tConst;  // Const accel
        T v2_temp = v1_temp + aMax * tConst;
        T d3 = v2_temp * tJerk + T(0.5) * aMax * tJerk * tJerk - 
               (T(1)/T(6)) * jMax * tJerk * tJerk * tJerk;  // Final jerk
        
        return d1 + d2 + d3;
    }

    /**
     * @brief Compute distance to decelerate from v0 to v1
     */
    T computeDecelDistance(T v0, T v1, T aMax, T jMax) const {
        // Symmetric to acceleration
        return computeAccelDistance(v1, v0, aMax, jMax);
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
