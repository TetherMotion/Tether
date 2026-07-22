/**
 * @file MotionGenerator.hpp
 * @brief Motion Generator Framework for CiA 402 Drives
 * 
 * @details
 * This module provides a framework for generating motion profiles including:
 * - Sine wave generator for testing and calibration
 * - Trapezoidal profile generator
 * - S-curve profile generator
 * - Custom profile interface
 * 
 * ## Architecture
 * 
 * ```
 *     ┌──────────────────────────────────────────────────────────────┐
 *     │                    MotionGenerator (Base)                    │
 *     │  - start(), stop(), update()                                 │
 *     │  - getPosition(), getVelocity(), getTorque()                 │
 *     └────────────────────────┬─────────────────────────────────────┘
 *                              │
 *     ┌────────────────────────┼────────────────────────────────────┐
 *     │                        │                                    │
 * ┌───┴───────────┐  ┌─────────┴────────┐  ┌────────────────────────┴──┐
 * │ SineGenerator │  │ TrapezoidProfile │  │ SCurveProfile             │
 * │ - amplitude   │  │ - max_velocity   │  │ - max_velocity            │
 * │ - frequency   │  │ - acceleration   │  │ - acceleration/jerk       │
 * │ - offset      │  │                  │  │                           │
 * └───────────────┘  └──────────────────┘  └───────────────────────────┘
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "MotionGenerator.hpp"
 * 
 * // Create sine generator
 * SineMotionGenerator sine;
 * sine.setAmplitude(10000);    // 10000 counts
 * sine.setFrequency(0.5);      // 0.5 Hz
 * sine.start();
 * 
 * // In cyclic loop
 * sine.update(cycle_time_ms);
 * drive.setTargetPosition(sine.getPosition());
 * drive.setTargetVelocity(sine.getVelocity());
 * ```
 */

#pragma once

#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Motion {

// ============================================================================
// Motion Generator Base Class
// ============================================================================

/**
 * @brief Motion generator state
 */
enum class GeneratorState : uint8_t {
    Idle,       ///< Generator not running
    Running,    ///< Generator actively producing motion
    Paused,     ///< Generator paused (can resume)
    Complete,   ///< Motion sequence complete
    Error       ///< Configuration invalid (e.g. zero acceleration/velocity)
};

/**
 * @brief Base class for all motion generators
 */
class MotionGenerator {
public:
    virtual ~MotionGenerator() = default;
    
    /**
     * @brief Start motion generation
     */
    virtual void start() = 0;
    
    /**
     * @brief Stop motion generation
     */
    virtual void stop() = 0;
    
    /**
     * @brief Pause motion generation
     */
    virtual void pause() { m_state = GeneratorState::Paused; }
    
    /**
     * @brief Resume motion generation
     */
    virtual void resume() { 
        if (m_state == GeneratorState::Paused) {
            m_state = GeneratorState::Running;
        }
    }
    
    /**
     * @brief Update motion generator state
     * 
     * @param dt_ms Time since last update in milliseconds
     */
    virtual void update(float dt_ms) = 0;
    
    /**
     * @brief Get current target position
     */
    virtual int32_t getPosition() const = 0;
    
    /**
     * @brief Get current target velocity
     */
    virtual int32_t getVelocity() const = 0;
    
    /**
     * @brief Get current target torque/force
     */
    virtual int16_t getTorque() const { return 0; }
    
    /**
     * @brief Get generator state
     */
    GeneratorState getState() const { return m_state; }
    
    /**
     * @brief Check if generator is running
     */
    bool isRunning() const { return m_state == GeneratorState::Running; }
    
    /**
     * @brief Check if motion is complete
     */
    bool isComplete() const { return m_state == GeneratorState::Complete; }

protected:
    GeneratorState m_state{GeneratorState::Idle};
};

// ============================================================================
// Sine Motion Generator
// ============================================================================

/**
 * @brief Sine wave motion generator
 * 
 * Generates sinusoidal position profiles, useful for:
 * - Drive testing and commissioning
 * - System identification
 * - Vibration/oscillation testing
 * - Demonstrating smooth continuous motion
 * 
 * Position equation: p(t) = offset + amplitude * sin(2π * frequency * t + phase)
 * Velocity equation: v(t) = amplitude * 2π * frequency * cos(2π * frequency * t + phase)
 */
class SineMotionGenerator : public MotionGenerator {
public:
    SineMotionGenerator() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set amplitude in position units
     */
    void setAmplitude(int32_t amplitude) { m_amplitude = amplitude; }
    
    /**
     * @brief Get amplitude
     */
    int32_t getAmplitude() const { return m_amplitude; }
    
    /**
     * @brief Set frequency in Hz
     */
    void setFrequency(float frequency) { m_frequency = frequency; }
    
    /**
     * @brief Get frequency
     */
    float getFrequency() const { return m_frequency; }
    
    /**
     * @brief Set DC offset (center position)
     */
    void setOffset(int32_t offset) { m_offset = offset; }
    
    /**
     * @brief Get offset
     */
    int32_t getOffset() const { return m_offset; }
    
    /**
     * @brief Set initial phase in radians
     */
    void setPhase(float phase) { m_initial_phase = phase; }
    
    /**
     * @brief Get phase
     */
    float getPhase() const { return m_current_phase; }
    
    /**
     * @brief Set number of cycles (0 = infinite)
     */
    void setCycles(uint32_t cycles) { m_target_cycles = cycles; }
    
    /**
     * @brief Configure generator with common parameters
     */
    void configure(int32_t amplitude, float frequency, int32_t offset = 0) {
        m_amplitude = amplitude;
        m_frequency = frequency;
        m_offset = offset;
    }
    
    // ========================================================================
    // Motion Generation
    // ========================================================================
    
    void start() override {
        m_elapsed_time = 0;
        m_current_phase = m_initial_phase;
        m_completed_cycles = 0;
        m_state = GeneratorState::Running;
        updateOutputs();
    }
    
    void stop() override {
        m_state = GeneratorState::Idle;
        m_current_position = m_offset;
        m_current_velocity = 0;
    }
    
    void update(float dt_ms) override {
        if (m_state != GeneratorState::Running) {
            return;
        }
        
        // Convert to seconds
        float dt_s = dt_ms / 1000.0f;
        m_elapsed_time += dt_s;
        
        // Update phase
        float omega = 2.0f * static_cast<float>(M_PI) * m_frequency;
        m_current_phase = m_initial_phase + omega * m_elapsed_time;
        
        // Check for cycle completion
        float cycles_completed = m_elapsed_time * m_frequency;
        uint32_t full_cycles = static_cast<uint32_t>(cycles_completed);
        if (full_cycles > m_completed_cycles) {
            m_completed_cycles = full_cycles;
            if (m_target_cycles > 0 && m_completed_cycles >= m_target_cycles) {
                m_state = GeneratorState::Complete;
                return;
            }
        }
        
        updateOutputs();
    }
    
    int32_t getPosition() const override {
        return m_current_position;
    }
    
    int32_t getVelocity() const override {
        return m_current_velocity;
    }
    
    /**
     * @brief Get position as floating point
     */
    float getPositionFloat() const { return m_position_float; }
    
    /**
     * @brief Get velocity as floating point
     */
    float getVelocityFloat() const { return m_velocity_float; }
    
    /**
     * @brief Get number of completed cycles
     */
    uint32_t getCompletedCycles() const { return m_completed_cycles; }

private:
    void updateOutputs() {
        float omega = 2.0f * static_cast<float>(M_PI) * m_frequency;
        
        // Position: offset + amplitude * sin(phase)
        m_position_float = static_cast<float>(m_offset) + 
                          static_cast<float>(m_amplitude) * std::sin(m_current_phase);
        m_current_position = static_cast<int32_t>(m_position_float);
        
        // Velocity: amplitude * omega * cos(phase)
        m_velocity_float = static_cast<float>(m_amplitude) * omega * std::cos(m_current_phase);
        m_current_velocity = static_cast<int32_t>(m_velocity_float);
    }
    
    // Configuration
    int32_t m_amplitude{0};
    float m_frequency{1.0f};
    int32_t m_offset{0};
    float m_initial_phase{0};
    uint32_t m_target_cycles{0};  // 0 = infinite
    
    // State
    float m_elapsed_time{0};
    float m_current_phase{0};
    uint32_t m_completed_cycles{0};
    
    // Outputs
    int32_t m_current_position{0};
    int32_t m_current_velocity{0};
    float m_position_float{0};
    float m_velocity_float{0};
};

// ============================================================================
// Trapezoidal Profile Generator
// ============================================================================

/**
 * @brief Trapezoidal velocity profile generator
 * 
 * Generates point-to-point motion with:
 * - Constant acceleration phase
 * - Constant velocity phase (if distance allows)
 * - Constant deceleration phase
 */
class TrapezoidalProfileGenerator : public MotionGenerator {
public:
    TrapezoidalProfileGenerator() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set maximum velocity
     */
    void setMaxVelocity(int32_t velocity) { m_max_velocity = velocity; }
    
    /**
     * @brief Set acceleration
     */
    void setAcceleration(int32_t acceleration) { m_acceleration = acceleration; }
    
    /**
     * @brief Set deceleration (defaults to acceleration if not set)
     */
    void setDeceleration(int32_t deceleration) { m_deceleration = deceleration; }
    
    /**
     * @brief Set target position
     */
    void setTargetPosition(int32_t position) { m_target_position = position; }
    
    /**
     * @brief Set start position
     */
    void setStartPosition(int32_t position) { m_start_position = position; }
    
    /**
     * @brief Configure profile with common parameters
     */
    void configure(int32_t start, int32_t target, int32_t max_vel, int32_t accel) {
        m_start_position = start;
        m_target_position = target;
        m_max_velocity = max_vel;
        m_acceleration = accel;
        m_deceleration = accel;
    }
    
    // ========================================================================
    // Motion Generation
    // ========================================================================
    
    void start() override;
    void stop() override;
    void update(float dt_ms) override;
    
    int32_t getPosition() const override { return m_current_position; }
    int32_t getVelocity() const override { return m_current_velocity; }
    
    /**
     * @brief Get motion phase
     */
    enum class Phase { Accel, Cruise, Decel, Complete };
    Phase getPhase() const { return m_phase; }

private:
    void calculateProfile();
    
    // Configuration
    int32_t m_max_velocity{0};
    int32_t m_acceleration{0};
    int32_t m_deceleration{0};
    int32_t m_start_position{0};
    int32_t m_target_position{0};
    
    // Profile parameters (calculated)
    float m_accel_time{0};
    float m_cruise_time{0};
    float m_decel_time{0};
    float m_total_time{0};
    int32_t m_peak_velocity{0};
    int8_t m_direction{1};
    
    // State
    float m_elapsed_time{0};
    Phase m_phase{Phase::Complete};
    
    // Outputs
    int32_t m_current_position{0};
    int32_t m_current_velocity{0};
};

// ============================================================================
// S-Curve Profile Generator
// ============================================================================

/**
 * @brief S-curve (jerk-limited) profile generator
 * 
 * Generates smooth motion profiles with limited jerk for:
 * - Reduced mechanical stress
 * - Better settling behavior
 * - Smoother trajectories
 */
class SCurveProfileGenerator : public MotionGenerator {
public:
    SCurveProfileGenerator() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set maximum velocity
     */
    void setMaxVelocity(float velocity) { m_max_velocity = velocity; }
    
    /**
     * @brief Set maximum acceleration
     */
    void setMaxAcceleration(float acceleration) { m_max_acceleration = acceleration; }
    
    /**
     * @brief Set maximum jerk
     */
    void setMaxJerk(float jerk) { m_max_jerk = jerk; }
    
    /**
     * @brief Set target position
     */
    void setTargetPosition(int32_t position) { m_target_position = position; }
    
    /**
     * @brief Set start position
     */
    void setStartPosition(int32_t position) { m_start_position = position; }
    
    /**
     * @brief Configure profile with common parameters
     */
    void configure(int32_t start, int32_t target, 
                   float max_vel, float max_accel, float max_jerk) {
        m_start_position = start;
        m_target_position = target;
        m_max_velocity = max_vel;
        m_max_acceleration = max_accel;
        m_max_jerk = max_jerk;
    }
    
    // ========================================================================
    // Motion Generation
    // ========================================================================
    
    void start() override;
    void stop() override;
    void update(float dt_ms) override;
    
    int32_t getPosition() const override { return static_cast<int32_t>(m_position); }
    int32_t getVelocity() const override { return static_cast<int32_t>(m_velocity); }
    
    /**
     * @brief Get floating point position
     */
    float getPositionFloat() const { return m_position; }
    
    /**
     * @brief Get floating point velocity
     */
    float getVelocityFloat() const { return m_velocity; }
    
    /**
     * @brief Get current acceleration
     */
    float getAcceleration() const { return m_acceleration; }
    
    /**
     * @brief Get current jerk
     */
    float getJerk() const { return m_jerk; }

private:
    void calculateProfile();
    
    // Configuration
    float m_max_velocity{0};
    float m_max_acceleration{0};
    float m_max_jerk{0};
    int32_t m_start_position{0};
    int32_t m_target_position{0};
    
    // Profile parameters (calculated)
    float m_t[8]{0};  // Phase times
    float m_total_time{0};
    int8_t m_direction{1};
    
    // State
    float m_elapsed_time{0};
    uint8_t m_current_phase{0};
    
    // Outputs
    float m_position{0};
    float m_velocity{0};
    float m_acceleration{0};
    float m_jerk{0};
};

// ============================================================================
// Multi-Axis Synchronized Generator
// ============================================================================

/**
 * @brief Maximum axes for synchronized motion
 */
constexpr size_t kMaxSyncAxes = 8;

/**
 * @brief Synchronized multi-axis motion generator
 * 
 * Coordinates multiple generators to ensure synchronized arrival
 * at target positions.
 */
class SynchronizedMotionGenerator {
public:
    SynchronizedMotionGenerator() = default;
    
    /**
     * @brief Add an axis generator
     * 
     * @param generator Pointer to axis generator (not owned)
     * @return Axis index, or -1 if full
     */
    int addAxis(MotionGenerator* generator);
    
    /**
     * @brief Start all axes synchronized
     */
    void startAll();
    
    /**
     * @brief Stop all axes
     */
    void stopAll();
    
    /**
     * @brief Update all axes
     */
    void updateAll(float dt_ms);
    
    /**
     * @brief Check if all axes are complete
     */
    bool allComplete() const;
    
    /**
     * @brief Get axis count
     */
    size_t getAxisCount() const { return m_axis_count; }
    
    /**
     * @brief Get axis generator
     */
    MotionGenerator* getAxis(size_t index) {
        return (index < m_axis_count) ? m_generators[index] : nullptr;
    }

private:
    MotionGenerator* m_generators[kMaxSyncAxes] = {nullptr};
    size_t m_axis_count{0};
};

// ============================================================================
// Motion Generator Factory
// ============================================================================

/**
 * @brief Motion generator types
 */
enum class GeneratorType {
    Sine,
    Trapezoidal,
    SCurve
};

/**
 * @brief Create a motion generator of the specified type
 * 
 * @param type Generator type
 * @return Pointer to new generator (caller owns)
 */
MotionGenerator* createGenerator(GeneratorType type);

} // namespace Motion
