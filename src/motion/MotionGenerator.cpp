/**
 * @file MotionGenerator.cpp
 * @brief Motion Generator Implementation
 */

#include "motion/MotionGenerator.hpp"
#include <cmath>
#include <algorithm>

namespace Motion {

// ============================================================================
// Trapezoidal Profile Generator Implementation
// ============================================================================

void TrapezoidalProfileGenerator::start() {
    calculateProfile();
    m_elapsed_time = 0;
    m_phase = Phase::Accel;
    m_current_position = m_start_position;
    m_current_velocity = 0;
    m_state = GeneratorState::Running;
}

void TrapezoidalProfileGenerator::stop() {
    m_state = GeneratorState::Idle;
    m_current_velocity = 0;
    m_phase = Phase::Complete;
}

void TrapezoidalProfileGenerator::calculateProfile() {
    int32_t distance = m_target_position - m_start_position;
    m_direction = (distance >= 0) ? 1 : -1;
    float abs_distance = std::abs(static_cast<float>(distance));
    
    float v_max = static_cast<float>(m_max_velocity);
    float a = static_cast<float>(m_acceleration);
    float d = static_cast<float>(m_deceleration > 0 ? m_deceleration : m_acceleration);
    
    // Time to accelerate to max velocity
    float t_accel = v_max / a;
    
    // Time to decelerate from max velocity
    float t_decel = v_max / d;
    
    // Distance during accel and decel phases
    float d_accel = 0.5f * a * t_accel * t_accel;
    float d_decel = 0.5f * d * t_decel * t_decel;
    
    if (d_accel + d_decel > abs_distance) {
        // Triangular profile - cannot reach max velocity
        // Solve for intersection velocity
        float v_peak = std::sqrt(2.0f * abs_distance * a * d / (a + d));
        m_peak_velocity = static_cast<int32_t>(v_peak);
        
        m_accel_time = v_peak / a;
        m_cruise_time = 0;
        m_decel_time = v_peak / d;
    } else {
        // Trapezoidal profile
        m_peak_velocity = m_max_velocity;
        m_accel_time = t_accel;
        m_decel_time = t_decel;
        
        float cruise_distance = abs_distance - d_accel - d_decel;
        m_cruise_time = cruise_distance / v_max;
    }
    
    m_total_time = m_accel_time + m_cruise_time + m_decel_time;
}

void TrapezoidalProfileGenerator::update(float dt_ms) {
    if (m_state != GeneratorState::Running) {
        return;
    }
    
    float dt_s = dt_ms / 1000.0f;
    m_elapsed_time += dt_s;
    
    float a = static_cast<float>(m_acceleration);
    float d = static_cast<float>(m_deceleration > 0 ? m_deceleration : m_acceleration);
    float v_peak = static_cast<float>(m_peak_velocity);
    
    float t = m_elapsed_time;
    float pos = 0;
    float vel = 0;
    
    if (t < m_accel_time) {
        // Acceleration phase
        m_phase = Phase::Accel;
        vel = a * t;
        pos = 0.5f * a * t * t;
    } else if (t < m_accel_time + m_cruise_time) {
        // Cruise phase
        m_phase = Phase::Cruise;
        float t_cruise = t - m_accel_time;
        vel = v_peak;
        pos = 0.5f * a * m_accel_time * m_accel_time + v_peak * t_cruise;
    } else if (t < m_total_time) {
        // Deceleration phase
        m_phase = Phase::Decel;
        float t_decel = t - m_accel_time - m_cruise_time;
        vel = v_peak - d * t_decel;
        
        float accel_dist = 0.5f * a * m_accel_time * m_accel_time;
        float cruise_dist = v_peak * m_cruise_time;
        pos = accel_dist + cruise_dist + v_peak * t_decel - 0.5f * d * t_decel * t_decel;
    } else {
        // Complete
        m_phase = Phase::Complete;
        m_state = GeneratorState::Complete;
        m_current_position = m_target_position;
        m_current_velocity = 0;
        return;
    }
    
    // Apply direction
    m_current_position = m_start_position + static_cast<int32_t>(pos * m_direction);
    m_current_velocity = static_cast<int32_t>(vel * m_direction);
}

// ============================================================================
// S-Curve Profile Generator Implementation
// ============================================================================

void SCurveProfileGenerator::start() {
    calculateProfile();
    m_elapsed_time = 0;
    m_current_phase = 0;
    m_position = static_cast<float>(m_start_position);
    m_velocity = 0;
    m_acceleration = 0;
    m_jerk = 0;
    m_state = GeneratorState::Running;
}

void SCurveProfileGenerator::stop() {
    m_state = GeneratorState::Idle;
    m_velocity = 0;
    m_acceleration = 0;
    m_jerk = 0;
}

void SCurveProfileGenerator::calculateProfile() {
    float distance = static_cast<float>(m_target_position - m_start_position);
    m_direction = (distance >= 0) ? 1 : -1;
    float abs_distance = std::abs(distance);
    
    float v_max = m_max_velocity;
    float a_max = m_max_acceleration;
    float j_max = m_max_jerk;
    
    // Time for jerk phase
    float t_j = a_max / j_max;
    
    // Velocity gained during jerk phases
    float v_j = 0.5f * j_max * t_j * t_j;
    
    // Distance during jerk phases
    float d_j = j_max * t_j * t_j * t_j / 6.0f;
    
    // Check if we can reach max acceleration
    float t_a = (v_max - 2.0f * v_j) / a_max;
    
    if (t_a < 0) {
        // Cannot reach max velocity with these constraints
        // Simplified: use triangular acceleration profile
        t_a = 0;
    }
    
    // Simplified S-curve calculation
    // Full implementation would handle all 7 phases
    
    m_t[0] = t_j;              // Increasing acceleration
    m_t[1] = t_a;              // Constant acceleration
    m_t[2] = t_j;              // Decreasing acceleration
    m_t[3] = 0;                // Constant velocity (calculated)
    m_t[4] = t_j;              // Increasing deceleration
    m_t[5] = t_a;              // Constant deceleration
    m_t[6] = t_j;              // Decreasing deceleration
    
    // Calculate distances for each phase
    float accel_distance = 2.0f * d_j + a_max * t_a * (t_j + 0.5f * t_a);
    float decel_distance = accel_distance;  // Symmetric
    
    float cruise_distance = abs_distance - accel_distance - decel_distance;
    if (cruise_distance > 0) {
        m_t[3] = cruise_distance / v_max;
    } else {
        m_t[3] = 0;
        // Would need to reduce max velocity - simplified for now
    }
    
    m_total_time = 0;
    for (int i = 0; i < 7; i++) {
        m_total_time += m_t[i];
    }
}

void SCurveProfileGenerator::update(float dt_ms) {
    if (m_state != GeneratorState::Running) {
        return;
    }
    
    float dt = dt_ms / 1000.0f;
    m_elapsed_time += dt;
    
    if (m_elapsed_time >= m_total_time) {
        m_state = GeneratorState::Complete;
        m_position = static_cast<float>(m_target_position);
        m_velocity = 0;
        m_acceleration = 0;
        m_jerk = 0;
        return;
    }
    
    // Determine current phase
    float t = m_elapsed_time;
    float phase_start = 0;
    
    for (int i = 0; i < 7; i++) {
        if (t < phase_start + m_t[i]) {
            m_current_phase = i;
            break;
        }
        phase_start += m_t[i];
    }
    
    // Calculate jerk based on phase
    float j = m_max_jerk * m_direction;
    switch (m_current_phase) {
        case 0: m_jerk = j; break;       // Increasing accel
        case 1: m_jerk = 0; break;       // Constant accel
        case 2: m_jerk = -j; break;      // Decreasing accel
        case 3: m_jerk = 0; break;       // Constant velocity
        case 4: m_jerk = -j; break;      // Increasing decel
        case 5: m_jerk = 0; break;       // Constant decel
        case 6: m_jerk = j; break;       // Decreasing decel
        default: m_jerk = 0; break;
    }
    
    // Integrate jerk -> acceleration -> velocity -> position
    m_acceleration += m_jerk * dt;
    m_velocity += m_acceleration * dt;
    m_position += m_velocity * dt;
    
    // Clamp values
    float a_max = m_max_acceleration;
    float v_max = m_max_velocity;
    m_acceleration = std::max(-a_max, std::min(a_max, m_acceleration));
    m_velocity = std::max(-v_max, std::min(v_max, m_velocity));
}

// ============================================================================
// Synchronized Motion Generator Implementation
// ============================================================================

int SynchronizedMotionGenerator::addAxis(MotionGenerator* generator) {
    if (m_axis_count >= kMaxSyncAxes || generator == nullptr) {
        return -1;
    }
    m_generators[m_axis_count] = generator;
    return static_cast<int>(m_axis_count++);
}

void SynchronizedMotionGenerator::startAll() {
    for (size_t i = 0; i < m_axis_count; i++) {
        if (m_generators[i]) {
            m_generators[i]->start();
        }
    }
}

void SynchronizedMotionGenerator::stopAll() {
    for (size_t i = 0; i < m_axis_count; i++) {
        if (m_generators[i]) {
            m_generators[i]->stop();
        }
    }
}

void SynchronizedMotionGenerator::updateAll(float dt_ms) {
    for (size_t i = 0; i < m_axis_count; i++) {
        if (m_generators[i]) {
            m_generators[i]->update(dt_ms);
        }
    }
}

bool SynchronizedMotionGenerator::allComplete() const {
    for (size_t i = 0; i < m_axis_count; i++) {
        if (m_generators[i] && !m_generators[i]->isComplete()) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Factory Function
// ============================================================================

MotionGenerator* createGenerator(GeneratorType type) {
    switch (type) {
        case GeneratorType::Sine:
            return new SineMotionGenerator();
        case GeneratorType::Trapezoidal:
            return new TrapezoidalProfileGenerator();
        case GeneratorType::SCurve:
            return new SCurveProfileGenerator();
        default:
            return nullptr;
    }
}

} // namespace Motion
