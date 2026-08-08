/**
 * @file ParameterRamping.hpp
 * @brief Framework for smooth parameter transitions without causing jerk
 *
 * @details
 * Provides multiple strategies for smoothly transitioning parameter values:
 *
 * - **ConstantTimeRamper**: Reaches target in fixed time regardless of distance
 * - **SigmoidalRamper**: S-curve transition with configurable steepness
 * - **SCurveRamper**: Jerk-limited S-curve for motion profiles
 *
 * All rampers support:
 * - Immediate value queries
 * - Target changes mid-transition
 * - Velocity and acceleration queries (where applicable)
 * - Completion callbacks
 */

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>

namespace Control {

// ============================================================================
// Base Classes
// ============================================================================

/**
 * @brief Abstract base class for all parameter rampers
 */
class ParameterRamperBase {
public:
    virtual ~ParameterRamperBase() = default;
    
    /**
     * @brief Set new target value
     * @param target New target
     * @param immediate If true, jump immediately without ramping
     */
    virtual void setTarget(double target, bool immediate = false) = 0;
    
    /**
     * @brief Get current value
     */
    virtual double getValue() const = 0;
    
    /**
     * @brief Get current velocity (rate of change)
     */
    virtual double getVelocity() const { return 0.0; }
    
    /**
     * @brief Get current acceleration
     */
    virtual double getAcceleration() const { return 0.0; }
    
    /**
     * @brief Update ramper state
     * @param dt Time step
     * @return Current value after update
     */
    virtual double update(double dt) = 0;
    
    /**
     * @brief Check if ramping is complete
     */
    virtual bool isComplete() const = 0;
    
    /**
     * @brief Get progress (0.0 to 1.0)
     */
    virtual double getProgress() const = 0;
    
    /**
     * @brief Get target value
     */
    double getTarget() const { return m_target; }
    
    /**
     * @brief Set completion callback
     */
    void setOnComplete(std::function<void()> callback) {
        m_onComplete = std::move(callback);
    }
    
    /**
     * @brief Reset to value without transition
     */
    virtual void reset(double value) {
        m_current = value;
        m_target = value;
        m_startValue = value;
    }
    
protected:
    double m_current{0.0};
    double m_target{0.0};
    double m_startValue{0.0};
    std::function<void()> m_onComplete;
    
    void triggerComplete() {
        if (m_onComplete) {
            m_onComplete();
        }
    }
};

// ============================================================================
// Constant Time Ramper
// ============================================================================

/**
 * @brief Ramper that reaches target in a fixed time regardless of distance
 * 
 * Uses linear interpolation over a fixed duration.
 * Good for UI animations and parameter changes that should feel consistent.
 */
class ConstantTimeRamper : public ParameterRamperBase {
public:
    /**
     * @brief Configuration
     */
    struct Config {
        double rampTime = 1.0;           ///< Time to reach target (seconds)
        double minRampTime = 0.01;       ///< Minimum ramp time
        
        static Config getDefault() { return Config{1.0, 0.01}; }
    };
    
    ConstantTimeRamper() : m_config(Config::getDefault()) {}
    explicit ConstantTimeRamper(const Config& config)
        : m_config(config) {}
    
    void setTarget(double target, bool immediate = false) override {
        if (immediate) {
            m_current = target;
            m_target = target;
            m_startValue = target;
            m_elapsed = m_config.rampTime;
            return;
        }
        
        m_startValue = m_current;
        m_target = target;
        m_elapsed = 0.0;
        m_wasComplete = false;
    }
    
    double getValue() const override { return m_current; }
    
    double getVelocity() const override {
        if (isComplete()) return 0.0;
        return (m_target - m_startValue) / m_config.rampTime;
    }
    
    double update(double dt) override {
        if (isComplete()) return m_current;
        
        m_elapsed += dt;
        double progress = std::min(1.0, m_elapsed / m_config.rampTime);
        m_current = m_startValue + (m_target - m_startValue) * progress;
        
        if (progress >= 1.0 && !m_wasComplete) {
            m_current = m_target;
            m_wasComplete = true;
            triggerComplete();
        }
        
        return m_current;
    }
    
    bool isComplete() const override {
        return m_elapsed >= m_config.rampTime;
    }
    
    double getProgress() const override {
        return std::min(1.0, m_elapsed / m_config.rampTime);
    }
    
    void reset(double value) override {
        ParameterRamperBase::reset(value);
        m_elapsed = m_config.rampTime;
        m_wasComplete = true;
    }
    
    /// Set ramp time
    void setRampTime(double time) {
        m_config.rampTime = std::max(m_config.minRampTime, time);
    }
    
private:
    Config m_config;
    double m_elapsed{0.0};
    bool m_wasComplete{true};
};


// ============================================================================
// Sigmoidal Ramper
// ============================================================================

/**
 * @brief S-curve (sigmoid) ramper for smooth transitions
 * 
 * Uses logistic function for smooth acceleration and deceleration.
 * Provides natural-feeling transitions with zero velocity at endpoints.
 */
class SigmoidalRamper : public ParameterRamperBase {
public:
    struct Config {
        double duration = 1.0;           ///< Total transition time
        double steepness = 5.0;          ///< Sigmoid steepness (higher = sharper)
        double tolerance = 1e-6;
        
        static Config getDefault() { return Config{1.0, 5.0, 1e-6}; }
    };
    
    SigmoidalRamper() : m_config(Config::getDefault()) {}
    explicit SigmoidalRamper(const Config& config)
        : m_config(config) {}
    
    void setTarget(double target, bool immediate = false) override {
        if (immediate) {
            m_current = target;
            m_target = target;
            m_startValue = target;
            m_elapsed = m_config.duration;
            return;
        }
        
        m_startValue = m_current;
        m_target = target;
        m_elapsed = 0.0;
        m_wasComplete = false;
    }
    
    double getValue() const override { return m_current; }
    
    double getVelocity() const override {
        if (isComplete()) return 0.0;
        // Derivative of sigmoid
        double k = m_config.steepness;
        double x = (m_elapsed / m_config.duration) * 2.0 - 1.0;  // -1 to 1
        double ex = std::exp(-k * x);
        double sigmoid_deriv = k * ex / ((1.0 + ex) * (1.0 + ex));
        return (m_target - m_startValue) * sigmoid_deriv / m_config.duration;
    }
    
    double update(double dt) override {
        if (isComplete()) return m_current;
        
        m_elapsed += dt;
        
        if (m_elapsed >= m_config.duration) {
            m_current = m_target;
            if (!m_wasComplete) {
                m_wasComplete = true;
                triggerComplete();
            }
            return m_current;
        }
        
        // Map elapsed time to -1..1 range
        double x = (m_elapsed / m_config.duration) * 2.0 - 1.0;
        
        // Logistic sigmoid
        double k = m_config.steepness;
        double sigmoid = 1.0 / (1.0 + std::exp(-k * x));
        
        // Normalize to 0..1 range (account for sigmoid not starting at exactly 0)
        double sigmoidStart = 1.0 / (1.0 + std::exp(k));
        double sigmoidEnd = 1.0 / (1.0 + std::exp(-k));
        double normalizedSigmoid = (sigmoid - sigmoidStart) / (sigmoidEnd - sigmoidStart);
        
        m_current = m_startValue + (m_target - m_startValue) * normalizedSigmoid;
        
        return m_current;
    }
    
    bool isComplete() const override {
        return m_elapsed >= m_config.duration;
    }
    
    double getProgress() const override {
        return std::min(1.0, m_elapsed / m_config.duration);
    }
    
    void reset(double value) override {
        ParameterRamperBase::reset(value);
        m_elapsed = m_config.duration;
        m_wasComplete = true;
    }
    
    void setDuration(double duration) {
        m_config.duration = std::max(0.001, duration);
    }
    
    void setSteepness(double steepness) {
        m_config.steepness = std::max(1.0, steepness);
    }
    
private:
    Config m_config;
    double m_elapsed{0.0};
    bool m_wasComplete{true};
};


// ============================================================================
// S-Curve (Jerk-Limited) Ramper
// ============================================================================

/**
 * @brief Jerk-limited S-curve ramper for motion applications
 * 
 * Uses 7-phase S-curve profile with limited jerk, acceleration, and velocity.
 * Ideal for motion control where mechanical jerk limits exist.
 */
class SCurveRamper : public ParameterRamperBase {
public:
    struct Config {
        double maxVelocity = 10.0;       ///< Maximum velocity (rad/s) – default increased by 10x
        double maxAcceleration = 20.0;   ///< Maximum acceleration (rad/s²) – default increased by 10x
        double maxJerk = 100.0;          ///< Maximum jerk (rad/s³) – default increased by 10x
        double tolerance = 1e-9;
        
        // NOTE: original defaults (1/2/10) were too slow for many demo use-cases.
        //       Bump by a factor of ten so that "default" ramper is fast enough
        //       for typical motion profiles without requiring explicit overrides.
        static Config getDefault() { return Config{10.0, 20.0, 100.0, 1e-9}; }
    };
    
    SCurveRamper() : m_config(Config::getDefault()) {}
    explicit SCurveRamper(const Config& config)
        : m_config(config) {}
    
    void setTarget(double target, bool immediate = false) override {
        if (immediate) {
            m_current = target;
            m_target = target;
            m_velocity = 0.0;
            m_acceleration = 0.0;
            m_phase = Phase::Complete;
            return;
        }
        
        m_startValue = m_current;
        m_target = target;
        m_direction = (target > m_current) ? 1.0 : -1.0;
        m_phase = Phase::AccelUp;
        m_wasComplete = false;
        
        // Pre-calculate profile (simplified 3-phase for this implementation)
        calculateProfile();
    }
    
    double getValue() const override { return m_current; }
    double getVelocity() const override { return m_velocity; }
    double getAcceleration() const override { return m_acceleration; }
    
    double update(double dt) override {
        if (m_phase == Phase::Complete) return m_current;
        
        // Apply jerk
        double jerk = 0.0;
        double distRemaining = std::abs(m_target - m_current);
        double stoppingDist = std::abs(m_velocity * m_velocity) / (2.0 * m_config.maxAcceleration);
        
        switch (m_phase) {
            case Phase::AccelUp:
                jerk = m_config.maxJerk * m_direction;
                if (std::abs(m_acceleration) >= m_config.maxAcceleration) {
                    m_phase = Phase::AccelConst;
                }
                break;
                
            case Phase::AccelConst:
                jerk = 0.0;
                if (std::abs(m_velocity) >= m_config.maxVelocity || 
                    distRemaining <= stoppingDist * 2.0) {
                    m_phase = Phase::AccelDown;
                }
                break;
                
            case Phase::AccelDown:
                jerk = -m_config.maxJerk * m_direction;
                if (std::abs(m_acceleration) < 0.001) {
                    if (distRemaining <= stoppingDist * 1.2) {
                        m_phase = Phase::DecelUp;
                    } else {
                        m_phase = Phase::Cruise;
                    }
                }
                break;
                
            case Phase::Cruise:
                jerk = 0.0;
                if (distRemaining <= stoppingDist * 1.5) {
                    m_phase = Phase::DecelUp;
                }
                break;
                
            case Phase::DecelUp:
                jerk = -m_config.maxJerk * m_direction;
                if (std::abs(m_acceleration) >= m_config.maxAcceleration) {
                    m_phase = Phase::DecelConst;
                }
                break;
                
            case Phase::DecelConst:
                jerk = 0.0;
                if (distRemaining <= stoppingDist * 0.3) {
                    m_phase = Phase::DecelDown;
                }
                break;
                
            case Phase::DecelDown:
                jerk = m_config.maxJerk * m_direction;
                if (std::abs(m_velocity) < 0.001 || 
                    (m_direction > 0 && m_current >= m_target) ||
                    (m_direction < 0 && m_current <= m_target)) {
                    m_phase = Phase::Complete;
                    m_current = m_target;
                    m_velocity = 0.0;
                    m_acceleration = 0.0;
                    if (!m_wasComplete) {
                        m_wasComplete = true;
                        triggerComplete();
                    }
                    return m_current;
                }
                break;
                
            case Phase::Complete:
                return m_current;
        }
        
        // Integrate motion
        m_acceleration += jerk * dt;
        m_acceleration = std::clamp(m_acceleration, 
                                    -m_config.maxAcceleration, 
                                    m_config.maxAcceleration);
        
        m_velocity += m_acceleration * dt;
        m_velocity = std::clamp(m_velocity, 
                               -m_config.maxVelocity, 
                               m_config.maxVelocity);
        
        m_current += m_velocity * dt;
        
        // Overshoot protection
        if ((m_direction > 0 && m_current > m_target) ||
            (m_direction < 0 && m_current < m_target)) {
            m_current = m_target;
            m_velocity = 0.0;
            m_acceleration = 0.0;
            m_phase = Phase::Complete;
        }
        
        return m_current;
    }
    
    bool isComplete() const override {
        return m_phase == Phase::Complete;
    }
    
    double getProgress() const override {
        double totalDist = std::abs(m_target - m_startValue);
        if (totalDist < m_config.tolerance) return 1.0;
        return 1.0 - std::abs(m_target - m_current) / totalDist;
    }
    
    void reset(double value) override {
        ParameterRamperBase::reset(value);
        m_velocity = 0.0;
        m_acceleration = 0.0;
        m_phase = Phase::Complete;
        m_wasComplete = true;
    }
    
private:
    enum class Phase {
        AccelUp,        // Increase acceleration (jerk > 0)
        AccelConst,     // Constant acceleration
        AccelDown,      // Decrease acceleration
        Cruise,         // Constant velocity
        DecelUp,        // Increase deceleration
        DecelConst,     // Constant deceleration
        DecelDown,      // Decrease deceleration
        Complete
    };
    
    void calculateProfile() {
        // Pre-calculate profile parameters (simplified)
        // Full implementation would calculate all 7 phase durations
    }
    
    Config m_config;
    Phase m_phase{Phase::Complete};
    double m_velocity{0.0};
    double m_acceleration{0.0};
    double m_direction{1.0};
    bool m_wasComplete{true};
};



} // namespace Control

