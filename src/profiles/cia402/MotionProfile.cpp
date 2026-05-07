/**
 * @file MotionProfile.cpp
 * @brief Motion profile implementations
 */

#include "profiles/cia402/MotionProfile.hpp"
#include "tether/platform/EspCompat.hpp"
#include <cstring>

static const char* TAG = "MotionProfile";

namespace CiA402 {

// ============================================================================
// Base Class
// ============================================================================

void MotionProfile::setLimits(const MotionLimits& limits) {
    m_limits = limits;
}

// ============================================================================
// Linear Profile
// ============================================================================

double LinearProfile::plan(double startPos, double endPos, 
                          double startVel, double endVel) {
    (void)startVel;
    (void)endVel;
    
    m_startPos = startPos;
    m_endPos = endPos;
    
    double distance = std::abs(endPos - startPos);
    m_direction = (endPos >= startPos) ? 1 : -1;
    
    m_velocity = m_limits.maxVelocity * m_direction * m_speedFactor;
    
    if (std::abs(m_velocity) > 1e-9) {
        m_duration = distance / std::abs(m_velocity);
    } else {
        m_duration = 0.0;
    }
    
    return m_duration;
}

MotionState LinearProfile::evaluate(double time) const {
    MotionState state;
    state.time = time;
    
    if (time <= 0.0) {
        state.position = m_startPos;
        state.velocity = 0.0;
        state.complete = false;
    } else if (time >= m_duration) {
        state.position = m_endPos;
        state.velocity = 0.0;
        state.complete = true;
    } else {
        state.position = m_startPos + m_velocity * time;
        state.velocity = m_velocity;
        state.complete = false;
    }
    
    state.acceleration = 0.0;
    state.jerk = 0.0;
    
    return state;
}

// ============================================================================
// Trapezoidal Profile
// ============================================================================

double TrapezoidalProfile::plan(double startPos, double endPos, 
                               double startVel, double endVel) {
    m_startPos = startPos;
    m_endPos = endPos;
    m_startVel = startVel;
    m_endVel = endVel;
    
    double distance = std::abs(endPos - startPos);
    m_direction = (endPos >= startPos) ? 1 : -1;
    
    // Apply speed factor to limits
    double maxVel = m_limits.maxVelocity * std::abs(m_speedFactor);
    double accel = m_limits.maxAcceleration;
    double decel = m_limits.maxDeceleration;
    
    if (distance < 1e-9) {
        m_duration = 0.0;
        m_t1 = 0.0;
        m_t2 = 0.0;
        m_peakVelocity = 0.0;
        m_isTriangular = false;
        return m_duration;
    }
    
    // Time to accelerate to max velocity
    double t_accel = (maxVel - std::abs(startVel)) / accel;
    
    // Time to decelerate from max velocity
    double t_decel = (maxVel - std::abs(endVel)) / decel;
    
    // Distance during acceleration
    double d_accel = std::abs(startVel) * t_accel + 0.5 * accel * t_accel * t_accel;
    
    // Distance during deceleration
    double d_decel = maxVel * t_decel - 0.5 * decel * t_decel * t_decel;
    
    // Check if triangular profile is needed
    if (d_accel + d_decel > distance) {
        // Triangular profile - calculate peak velocity
        m_isTriangular = true;
        
        // Using: distance = v²/(2*a) + v²/(2*d)
        // Solve for v: v = sqrt(2*a*d*distance / (a+d))
        m_peakVelocity = std::sqrt(2.0 * accel * decel * distance / (accel + decel));
        
        m_t1 = (m_peakVelocity - std::abs(startVel)) / accel;
        m_t2 = m_t1;  // No coast phase
        m_duration = m_t1 + (m_peakVelocity - std::abs(endVel)) / decel;
    } else {
        // Trapezoidal profile
        m_isTriangular = false;
        m_peakVelocity = maxVel;
        
        // Coast distance
        double d_coast = distance - d_accel - d_decel;
        double t_coast = d_coast / maxVel;
        
        m_t1 = t_accel;
        m_t2 = t_accel + t_coast;
        m_duration = m_t2 + t_decel;
    }
    
    m_accel = accel;
    m_decel = decel;
    
    TETHER_LOGD(TAG, "Trapezoidal: d=%.2f, v_peak=%.2f, t1=%.3f, t2=%.3f, T=%.3f, tri=%d",
             distance, m_peakVelocity, m_t1, m_t2, m_duration, m_isTriangular);
    
    return m_duration;
}

MotionState TrapezoidalProfile::evaluate(double time) const {
    MotionState state;
    state.time = time;
    
    // Handle negative speed factor (reverse playback)
    double effectiveTime = time;
    if (m_speedFactor < 0) {
        effectiveTime = m_duration - time;
    }
    
    if (effectiveTime <= 0.0) {
        state.position = m_startPos;
        state.velocity = m_startVel;
        state.acceleration = m_accel * m_direction;
        state.jerk = 0.0;
        state.complete = false;
        return state;
    }
    
    if (effectiveTime >= m_duration) {
        state.position = m_endPos;
        state.velocity = m_endVel;
        state.acceleration = 0.0;
        state.jerk = 0.0;
        state.complete = true;
        return state;
    }
    
    double t = effectiveTime;
    double p, v, a;
    
    if (t < m_t1) {
        // Acceleration phase
        a = m_accel;
        v = m_startVel + a * t;
        p = m_startPos + m_direction * (m_startVel * t + 0.5 * a * t * t);
    } else if (t < m_t2) {
        // Coast phase
        double t_coast = t - m_t1;
        double p_at_t1 = m_startPos + m_direction * (m_startVel * m_t1 + 0.5 * m_accel * m_t1 * m_t1);
        
        a = 0.0;
        v = m_peakVelocity;
        p = p_at_t1 + m_direction * m_peakVelocity * t_coast;
    } else {
        // Deceleration phase
        double t_decel = t - m_t2;
        double p_at_t2 = m_startPos + m_direction * (
            m_startVel * m_t1 + 0.5 * m_accel * m_t1 * m_t1 +
            m_peakVelocity * (m_t2 - m_t1)
        );
        
        a = -m_decel;
        v = m_peakVelocity - m_decel * t_decel;
        p = p_at_t2 + m_direction * (m_peakVelocity * t_decel - 0.5 * m_decel * t_decel * t_decel);
    }
    
    state.position = p;
    state.velocity = v * m_direction;
    state.acceleration = a * m_direction;
    state.jerk = 0.0;
    state.complete = false;
    
    return state;
}

// ============================================================================
// Triangular Profile
// ============================================================================

double TriangularProfile::plan(double startPos, double endPos, 
                              double startVel, double endVel) {
    m_startPos = startPos;
    m_endPos = endPos;
    m_startVel = startVel;
    m_endVel = endVel;
    
    double distance = std::abs(endPos - startPos);
    m_direction = (endPos >= startPos) ? 1 : -1;
    
    double accel = m_limits.maxAcceleration;
    double decel = m_limits.maxDeceleration;
    
    if (distance < 1e-9) {
        m_duration = 0.0;
        m_t1 = 0.0;
        m_peakVelocity = 0.0;
        return m_duration;
    }
    
    // Calculate peak velocity from distance
    m_peakVelocity = std::sqrt(2.0 * accel * decel * distance / (accel + decel));
    
    // Limit to max velocity
    if (m_peakVelocity > m_limits.maxVelocity * std::abs(m_speedFactor)) {
        m_peakVelocity = m_limits.maxVelocity * std::abs(m_speedFactor);
    }
    
    m_t1 = m_peakVelocity / accel;
    m_duration = m_t1 + m_peakVelocity / decel;
    
    m_accel = accel;
    m_decel = decel;
    
    return m_duration;
}

MotionState TriangularProfile::evaluate(double time) const {
    MotionState state;
    state.time = time;
    
    if (time <= 0.0) {
        state.position = m_startPos;
        state.velocity = 0.0;
        state.acceleration = m_accel * m_direction;
        state.complete = false;
        return state;
    }
    
    if (time >= m_duration) {
        state.position = m_endPos;
        state.velocity = 0.0;
        state.acceleration = 0.0;
        state.complete = true;
        return state;
    }
    
    double p, v, a;
    
    if (time < m_t1) {
        // Acceleration phase
        a = m_accel;
        v = a * time;
        p = m_startPos + m_direction * 0.5 * a * time * time;
    } else {
        // Deceleration phase
        double t_decel = time - m_t1;
        double p_at_t1 = m_startPos + m_direction * 0.5 * m_accel * m_t1 * m_t1;
        
        a = -m_decel;
        v = m_peakVelocity - m_decel * t_decel;
        p = p_at_t1 + m_direction * (m_peakVelocity * t_decel - 0.5 * m_decel * t_decel * t_decel);
    }
    
    state.position = p;
    state.velocity = v * m_direction;
    state.acceleration = a * m_direction;
    state.jerk = 0.0;
    state.complete = false;
    
    return state;
}

// ============================================================================
// S-Curve Profile
// ============================================================================

double SCurveProfile::plan(double startPos, double endPos, 
                          double startVel, double endVel) {
    m_startPos = startPos;
    m_endPos = endPos;
    m_startVel = startVel;
    m_endVel = endVel;
    
    double distance = std::abs(endPos - startPos);
    m_direction = (endPos >= startPos) ? 1 : -1;
    
    if (distance < 1e-9) {
        m_duration = 0.0;
        std::memset(m_t, 0, sizeof(m_t));
        return m_duration;
    }
    
    m_jerk = m_limits.maxJerk;
    m_accel = m_limits.maxAcceleration;
    m_peakVel = m_limits.maxVelocity * std::abs(m_speedFactor);
    
    calculatePhases();
    
    // Store initial conditions for each phase
    m_p0[0] = 0.0;  // Relative position
    m_v0[0] = std::abs(startVel);
    m_a0[0] = 0.0;
    
    for (int i = 1; i < 7; i++) {
        MotionState s = evaluatePhase(i - 1, m_t[i] - m_t[i-1]);
        m_p0[i] = s.position;
        m_v0[i] = s.velocity;
        m_a0[i] = s.acceleration;
    }
    
    TETHER_LOGD(TAG, "S-Curve: d=%.2f, phases=%d, T=%.3f", distance, m_phaseCount, m_duration);
    
    return m_duration;
}

void SCurveProfile::calculatePhases() {
    double j = m_jerk;
    double a_max = m_accel;
    double v_max = m_peakVel;
    double d = std::abs(m_endPos - m_startPos);
    
    // Time for acceleration to reach a_max
    double tj = a_max / j;
    
    // Velocity gained during jerk phases
    double v_j = 0.5 * j * tj * tj;
    
    // Time at constant acceleration
    double ta = (v_max - 2.0 * v_j) / a_max;
    if (ta < 0) ta = 0;
    
    // Distance during acceleration (phases 1-3)
    double d_accel = v_j * tj + a_max * ta * (ta / 2 + tj) + v_j * tj + v_max * 0;
    
    // For simplicity, assume symmetric acceleration/deceleration
    double d_decel = d_accel;
    
    // Coast distance
    double d_coast = d - d_accel - d_decel;
    
    if (d_coast < 0) {
        // Need to reduce peak velocity
        // Simplified: use quadratic approximation
        d_coast = 0;
        
        // Recalculate for shorter distance
        // v_peak = sqrt(d * j) for full jerk limitation
        m_peakVel = std::sqrt(d * j * 0.5);
        if (m_peakVel > v_max) m_peakVel = v_max;
        
        ta = 0;
        tj = std::sqrt(m_peakVel / j);
    }
    
    // Phase times (cumulative)
    m_t[0] = 0;
    m_t[1] = tj;                          // End of jerk up (accel phase)
    m_t[2] = m_t[1] + ta;                 // End of const accel
    m_t[3] = m_t[2] + tj;                 // End of jerk down (accel phase)
    
    double t_coast = d_coast / m_peakVel;
    m_t[4] = m_t[3] + t_coast;            // End of coast
    
    m_t[5] = m_t[4] + tj;                 // End of jerk down (decel phase)
    m_t[6] = m_t[5] + ta;                 // End of const decel
    m_t[7] = m_t[6] + tj;                 // End of jerk up (decel phase)
    
    m_duration = m_t[7];
    
    // Jerk values for each phase
    m_j[0] = j;      // Increasing acceleration
    m_j[1] = 0;      // Constant acceleration
    m_j[2] = -j;     // Decreasing acceleration
    m_j[3] = 0;      // Constant velocity
    m_j[4] = -j;     // Increasing deceleration
    m_j[5] = 0;      // Constant deceleration
    m_j[6] = j;      // Decreasing deceleration
    
    m_phaseCount = 7;
}

MotionState SCurveProfile::evaluatePhase(int phase, double t) const {
    MotionState state;
    
    double p0 = (phase < 7) ? m_p0[phase] : m_p0[6];
    double v0 = (phase < 7) ? m_v0[phase] : m_v0[6];
    double a0 = (phase < 7) ? m_a0[phase] : m_a0[6];
    double j = (phase < 7) ? m_j[phase] : 0;
    
    // Kinematic equations with jerk
    state.jerk = j;
    state.acceleration = a0 + j * t;
    state.velocity = v0 + a0 * t + 0.5 * j * t * t;
    state.position = p0 + v0 * t + 0.5 * a0 * t * t + (1.0/6.0) * j * t * t * t;
    
    return state;
}

MotionState SCurveProfile::evaluate(double time) const {
    MotionState state;
    state.time = time;
    
    if (time <= 0.0) {
        state.position = m_startPos;
        state.velocity = m_startVel;
        state.acceleration = 0.0;
        state.jerk = m_j[0];
        state.complete = false;
        return state;
    }
    
    if (time >= m_duration) {
        state.position = m_endPos;
        state.velocity = m_endVel;
        state.acceleration = 0.0;
        state.jerk = 0.0;
        state.complete = true;
        return state;
    }
    
    // Find current phase
    int phase = 0;
    for (int i = 1; i < 8; i++) {
        if (time < m_t[i]) {
            phase = i - 1;
            break;
        }
    }
    
    // Time within phase
    double t_phase = time - m_t[phase];
    
    state = evaluatePhase(phase, t_phase);
    
    // Convert to absolute position
    state.position = m_startPos + m_direction * state.position;
    state.velocity *= m_direction;
    state.acceleration *= m_direction;
    state.jerk *= m_direction;
    state.time = time;
    state.complete = false;
    
    return state;
}

// ============================================================================
// Polynomial Profile
// ============================================================================

PolynomialProfile::PolynomialProfile(Order order) 
    : m_order(order) {
    std::memset(m_coeff, 0, sizeof(m_coeff));
}

double PolynomialProfile::plan(double startPos, double endPos, 
                              double startVel, double endVel) {
    m_startPos = startPos;
    m_endPos = endPos;
    m_startVel = startVel;
    m_endVel = endVel;
    
    double distance = std::abs(endPos - startPos);
    m_direction = (endPos >= startPos) ? 1 : -1;
    
    // Calculate duration if not specified
    if (m_desiredDuration <= 0.0) {
        // Estimate duration from velocity limit
        double avgVel = m_limits.maxVelocity * 0.5;
        m_duration = distance / avgVel;
        if (m_duration < 0.1) m_duration = 0.1;
    } else {
        m_duration = m_desiredDuration;
    }
    
    // Apply speed factor
    m_duration /= std::abs(m_speedFactor);
    
    // Calculate coefficients based on order
    switch (m_order) {
        case Order::Cubic:
            calculateCubic();
            break;
        case Order::Quintic:
            calculateQuintic();
            break;
        case Order::Septic:
            calculateSeptic();
            break;
    }
    
    return m_duration;
}

void PolynomialProfile::calculateCubic() {
    // Cubic polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³
    // Boundary conditions: p(0)=p0, v(0)=v0, p(T)=pf, v(T)=vf
    
    double T = m_duration;
    double T2 = T * T;
    double T3 = T2 * T;
    
    double dp = m_endPos - m_startPos;
    double dv0 = m_startVel;
    double dvf = m_endVel;
    
    m_coeff[0] = m_startPos;
    m_coeff[1] = dv0;
    m_coeff[2] = (3.0 * dp - (2.0 * dv0 + dvf) * T) / T2;
    m_coeff[3] = (-2.0 * dp + (dv0 + dvf) * T) / T3;
}

void PolynomialProfile::calculateQuintic() {
    // Quintic polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // Boundary conditions: p(0)=p0, v(0)=v0, a(0)=a0, p(T)=pf, v(T)=vf, a(T)=af
    
    double T = m_duration;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    double dp = m_endPos - m_startPos;
    
    m_coeff[0] = m_startPos;
    m_coeff[1] = m_startVel;
    m_coeff[2] = m_startAccel / 2.0;
    
    double a0 = m_startAccel;
    double af = m_endAccel;
    double v0 = m_startVel;
    double vf = m_endVel;
    
    m_coeff[3] = (20.0 * dp - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
    m_coeff[4] = (-30.0 * dp + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / (2.0 * T4);
    m_coeff[5] = (12.0 * dp - 6.0 * (vf + v0) * T + (af - a0) * T2) / (2.0 * T5);
}

void PolynomialProfile::calculateSeptic() {
    // Septic polynomial (7th order)
    // Full boundary conditions including jerk
    
    // Simplified: use quintic as base, extend with jerk terms
    calculateQuintic();
    
    // Additional terms for jerk (approximate)
    double T = m_duration;
    double T6 = std::pow(T, 6);
    double T7 = std::pow(T, 7);
    
    m_coeff[6] = (m_endJerk - m_startJerk) / (6.0 * T6);
    m_coeff[7] = -(m_endJerk - m_startJerk) / (6.0 * T7);
}

MotionState PolynomialProfile::evaluate(double time) const {
    MotionState state;
    state.time = time;
    
    if (time <= 0.0) {
        state.position = m_startPos;
        state.velocity = m_startVel;
        state.acceleration = m_startAccel;
        state.jerk = m_startJerk;
        state.complete = false;
        return state;
    }
    
    if (time >= m_duration) {
        state.position = m_endPos;
        state.velocity = m_endVel;
        state.acceleration = m_endAccel;
        state.jerk = m_endJerk;
        state.complete = true;
        return state;
    }
    
    double t = time;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    double t7 = t6 * t;
    
    int order = static_cast<int>(m_order);
    
    // Position
    state.position = m_coeff[0] + m_coeff[1] * t + m_coeff[2] * t2 + m_coeff[3] * t3;
    if (order >= 5) {
        state.position += m_coeff[4] * t4 + m_coeff[5] * t5;
    }
    if (order >= 7) {
        state.position += m_coeff[6] * t6 + m_coeff[7] * t7;
    }
    
    // Velocity (first derivative)
    state.velocity = m_coeff[1] + 2.0 * m_coeff[2] * t + 3.0 * m_coeff[3] * t2;
    if (order >= 5) {
        state.velocity += 4.0 * m_coeff[4] * t3 + 5.0 * m_coeff[5] * t4;
    }
    if (order >= 7) {
        state.velocity += 6.0 * m_coeff[6] * t5 + 7.0 * m_coeff[7] * t6;
    }
    
    // Acceleration (second derivative)
    state.acceleration = 2.0 * m_coeff[2] + 6.0 * m_coeff[3] * t;
    if (order >= 5) {
        state.acceleration += 12.0 * m_coeff[4] * t2 + 20.0 * m_coeff[5] * t3;
    }
    if (order >= 7) {
        state.acceleration += 30.0 * m_coeff[6] * t4 + 42.0 * m_coeff[7] * t5;
    }
    
    // Jerk (third derivative)
    state.jerk = 6.0 * m_coeff[3];
    if (order >= 5) {
        state.jerk += 24.0 * m_coeff[4] * t + 60.0 * m_coeff[5] * t2;
    }
    if (order >= 7) {
        state.jerk += 120.0 * m_coeff[6] * t3 + 210.0 * m_coeff[7] * t4;
    }
    
    state.complete = false;
    return state;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<MotionProfile> createProfile(ProfileType type) {
    switch (type) {
        case ProfileType::Linear:
            return std::make_unique<LinearProfile>();
        case ProfileType::Trapezoidal:
            return std::make_unique<TrapezoidalProfile>();
        case ProfileType::Triangular:
            return std::make_unique<TriangularProfile>();
        case ProfileType::SCurve:
            return std::make_unique<SCurveProfile>();
        case ProfileType::Polynomial:
            return std::make_unique<PolynomialProfile>();
        default:
            return std::make_unique<TrapezoidalProfile>();
    }
}

ProfileType selectOptimalProfile(double distance, const MotionLimits& limits) {
    // Calculate characteristic distances
    double accel = limits.maxAcceleration;
    double vel = limits.maxVelocity;
    double jerk = limits.maxJerk;
    
    // Distance needed for triangular profile at max acceleration
    double d_tri = vel * vel / accel;
    
    // If distance is very short, use linear
    if (distance < d_tri * 0.01) {
        return ProfileType::Linear;
    }
    
    // If jerk limit is significant, use S-curve
    double t_jerk = accel / jerk;
    if (t_jerk > 0.01) {
        return ProfileType::SCurve;
    }
    
    // Default to trapezoidal
    return ProfileType::Trapezoidal;
}

} // namespace CiA402
