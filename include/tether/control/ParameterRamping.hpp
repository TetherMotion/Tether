/**
 * @file ParameterRamping.hpp
 * @brief Framework for smooth parameter transitions without causing jerk
 * 
 * @details
 * Provides multiple strategies for smoothly transitioning parameter values:
 * 
 * - **ConstantTimeRamper**: Reaches target in fixed time regardless of distance
 * - **LimitedRateRamper**: Changes at maximum rate (limited change speed)
 * - **SigmoidalRamper**: S-curve transition with configurable steepness
 * - **ExponentialRamper**: First-order exponential approach
 * - **SCurveRamper**: Jerk-limited S-curve for motion profiles
 * - **CompositeRamper**: Combine multiple ramping strategies
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
// Limited Rate Ramper
// ============================================================================

/**
 * @brief Ramper with maximum rate of change (slew rate limiter)
 * 
 * Changes value at a constant rate until target is reached.
 * Time to reach target depends on distance.
 * Good for protecting physical systems from sudden changes.
 */
class LimitedRateRamper : public ParameterRamperBase {
public:
    struct Config {
        double maxRate = 1.0;            ///< Maximum change per second
        double tolerance = 1e-9;         ///< Completion tolerance
        
        static Config getDefault() { return Config{1.0, 1e-9}; }
    };
    
    LimitedRateRamper() : m_config(Config::getDefault()) {}
    explicit LimitedRateRamper(const Config& config)
        : m_config(config) {}
    
    void setTarget(double target, bool immediate = false) override {
        if (immediate) {
            m_current = target;
            m_target = target;
            m_velocity = 0.0;
            return;
        }
        
        m_startValue = m_current;
        m_target = target;
        m_wasComplete = false;
    }
    
    double getValue() const override { return m_current; }
    
    double getVelocity() const override { return m_velocity; }
    
    double update(double dt) override {
        double error = m_target - m_current;
        
        if (std::abs(error) <= m_config.tolerance) {
            m_current = m_target;
            m_velocity = 0.0;
            if (!m_wasComplete) {
                m_wasComplete = true;
                triggerComplete();
            }
            return m_current;
        }
        
        // Calculate desired change
        double maxChange = m_config.maxRate * dt;
        double change = std::clamp(error, -maxChange, maxChange);
        
        m_velocity = change / dt;
        m_current += change;
        
        return m_current;
    }
    
    bool isComplete() const override {
        return std::abs(m_target - m_current) <= m_config.tolerance;
    }
    
    double getProgress() const override {
        double totalDist = std::abs(m_target - m_startValue);
        if (totalDist < m_config.tolerance) return 1.0;
        double remaining = std::abs(m_target - m_current);
        return 1.0 - remaining / totalDist;
    }
    
    void reset(double value) override {
        ParameterRamperBase::reset(value);
        m_velocity = 0.0;
        m_wasComplete = true;
    }
    
    /// Set maximum rate
    void setMaxRate(double rate) {
        m_config.maxRate = std::abs(rate);
    }
    
    /// Get time remaining to reach target
    double getTimeRemaining() const {
        if (m_config.maxRate <= 0) return std::numeric_limits<double>::infinity();
        return std::abs(m_target - m_current) / m_config.maxRate;
    }
    
private:
    Config m_config;
    double m_velocity{0.0};
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
// Exponential Ramper
// ============================================================================

/**
 * @brief First-order exponential approach ramper
 * 
 * Approaches target asymptotically with configurable time constant.
 * Never technically "arrives" but gets within tolerance.
 */
class ExponentialRamper : public ParameterRamperBase {
public:
    struct Config {
        double timeConstant = 1.0;       ///< Time constant (63.2% of way there)
        double tolerance = 0.001;        ///< Fraction of distance to consider complete
        double minValue = -1e30;         ///< Optional value limits
        double maxValue = 1e30;
        
        static Config getDefault() { return Config{1.0, 0.001, -1e30, 1e30}; }
    };
    
    ExponentialRamper() : m_config(Config::getDefault()) {}
    explicit ExponentialRamper(const Config& config)
        : m_config(config) {}
    
    void setTarget(double target, bool immediate = false) override {
        if (immediate) {
            m_current = target;
            m_target = target;
            return;
        }
        
        m_startValue = m_current;
        m_target = target;
        m_wasComplete = false;
    }
    
    double getValue() const override { return m_current; }
    
    double getVelocity() const override {
        return (m_target - m_current) / m_config.timeConstant;
    }
    
    double update(double dt) override {
        double error = m_target - m_current;
        double totalError = m_target - m_startValue;
        
        // Check completion
        if (std::abs(totalError) > 1e-15) {
            if (std::abs(error / totalError) < m_config.tolerance) {
                m_current = m_target;
                if (!m_wasComplete) {
                    m_wasComplete = true;
                    triggerComplete();
                }
                return m_current;
            }
        } else {
            m_current = m_target;
            return m_current;
        }
        
        // Exponential approach
        double alpha = 1.0 - std::exp(-dt / m_config.timeConstant);
        m_current += alpha * error;
        
        // Apply limits
        m_current = std::clamp(m_current, m_config.minValue, m_config.maxValue);
        
        return m_current;
    }
    
    bool isComplete() const override {
        double totalError = std::abs(m_target - m_startValue);
        if (totalError < 1e-15) return true;
        return std::abs(m_target - m_current) / totalError < m_config.tolerance;
    }
    
    double getProgress() const override {
        double totalError = std::abs(m_target - m_startValue);
        if (totalError < 1e-15) return 1.0;
        return 1.0 - std::abs(m_target - m_current) / totalError;
    }
    
    void setTimeConstant(double tau) {
        m_config.timeConstant = std::max(0.001, tau);
    }
    
private:
    Config m_config;
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

// ============================================================================
// Polynomial Ramper (Quintic)
// ============================================================================

/**
 * @brief Quintic polynomial ramper for smooth motion
 * 
 * Uses 5th order polynomial that guarantees zero velocity and
 * acceleration at both endpoints.
 */
class QuinticRamper : public ParameterRamperBase {
public:
    struct Config {
        double duration = 1.0;
        
        static Config getDefault() { return Config{1.0}; }
    };
    
    QuinticRamper() : m_config(Config::getDefault()) {}
    explicit QuinticRamper(const Config& config)
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
        double t = m_elapsed / m_config.duration;
        // Derivative of quintic: 30t^4 - 60t^3 + 30t^2
        double vel_normalized = 30.0*t*t*t*t - 60.0*t*t*t + 30.0*t*t;
        return (m_target - m_startValue) * vel_normalized / m_config.duration;
    }
    
    double getAcceleration() const override {
        if (isComplete()) return 0.0;
        double t = m_elapsed / m_config.duration;
        // Second derivative: 120t^3 - 180t^2 + 60t
        double acc_normalized = 120.0*t*t*t - 180.0*t*t + 60.0*t;
        return (m_target - m_startValue) * acc_normalized / 
               (m_config.duration * m_config.duration);
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
        
        double t = m_elapsed / m_config.duration;
        
        // Quintic polynomial: 6t^5 - 15t^4 + 10t^3
        double blend = 6.0*t*t*t*t*t - 15.0*t*t*t*t + 10.0*t*t*t;
        
        m_current = m_startValue + (m_target - m_startValue) * blend;
        
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
    
private:
    Config m_config;
    double m_elapsed{0.0};
    bool m_wasComplete{true};
};

// ============================================================================
// Composite Ramper
// ============================================================================

/**
 * @brief Combines multiple ramping strategies
 * 
 * Can blend outputs or switch between rampers based on conditions.
 */
class CompositeRamper : public ParameterRamperBase {
public:
    enum class Mode {
        Blend,          ///< Weighted average of all rampers
        Sequential,     ///< Use rampers in sequence
        Fastest,        ///< Use whichever completes first
        Slowest         ///< Use whichever is slowest
    };
    
    struct Config {
        Mode mode = Mode::Blend;
        std::vector<double> weights;    ///< Weights for Blend mode
        
        static Config getDefault() { return Config{Mode::Blend, {}}; }
    };
    
    CompositeRamper() : m_config(Config::getDefault()) {}
    explicit CompositeRamper(const Config& config)
        : m_config(config) {}
    
    void addRamper(std::unique_ptr<ParameterRamperBase> ramper, double weight = 1.0) {
        m_rampers.push_back(std::move(ramper));
        m_config.weights.push_back(weight);
    }
    
    void setTarget(double target, bool immediate = false) override {
        m_target = target;
        for (auto& ramper : m_rampers) {
            ramper->setTarget(target, immediate);
        }
        if (immediate) {
            m_current = target;
        }
    }
    
    double getValue() const override { return m_current; }
    
    double update(double dt) override {
        if (m_rampers.empty()) return m_current;
        
        switch (m_config.mode) {
            case Mode::Blend: {
                double totalWeight = 0.0;
                double weightedSum = 0.0;
                for (size_t i = 0; i < m_rampers.size(); ++i) {
                    double val = m_rampers[i]->update(dt);
                    double w = (i < m_config.weights.size()) ? m_config.weights[i] : 1.0;
                    weightedSum += val * w;
                    totalWeight += w;
                }
                m_current = (totalWeight > 0) ? weightedSum / totalWeight : m_current;
                break;
            }
            
            case Mode::Fastest: {
                double closest = m_rampers[0]->update(dt);
                double minDist = std::abs(m_target - closest);
                for (size_t i = 1; i < m_rampers.size(); ++i) {
                    double val = m_rampers[i]->update(dt);
                    double dist = std::abs(m_target - val);
                    if (dist < minDist) {
                        minDist = dist;
                        closest = val;
                    }
                }
                m_current = closest;
                break;
            }
            
            case Mode::Slowest: {
                double farthest = m_rampers[0]->update(dt);
                double maxDist = std::abs(m_target - farthest);
                for (size_t i = 1; i < m_rampers.size(); ++i) {
                    double val = m_rampers[i]->update(dt);
                    double dist = std::abs(m_target - val);
                    if (dist > maxDist) {
                        maxDist = dist;
                        farthest = val;
                    }
                }
                m_current = farthest;
                break;
            }
            
            default:
                m_current = m_rampers[0]->update(dt);
        }
        
        return m_current;
    }
    
    bool isComplete() const override {
        if (m_rampers.empty()) return true;
        
        switch (m_config.mode) {
            case Mode::Blend:
            case Mode::Slowest:
                // Complete when all are complete
                for (const auto& ramper : m_rampers) {
                    if (!ramper->isComplete()) return false;
                }
                return true;
                
            case Mode::Fastest:
                // Complete when any is complete
                for (const auto& ramper : m_rampers) {
                    if (ramper->isComplete()) return true;
                }
                return false;
                
            default:
                return m_rampers[0]->isComplete();
        }
    }
    
    double getProgress() const override {
        if (m_rampers.empty()) return 1.0;
        
        double sum = 0.0;
        for (const auto& ramper : m_rampers) {
            sum += ramper->getProgress();
        }
        return sum / m_rampers.size();
    }
    
    void reset(double value) override {
        ParameterRamperBase::reset(value);
        for (auto& ramper : m_rampers) {
            ramper->reset(value);
        }
    }
    
private:
    Config m_config;
    std::vector<std::unique_ptr<ParameterRamperBase>> m_rampers;
};

// ============================================================================
// Multi-Parameter Ramper
// ============================================================================

/**
 * @brief Coordinates multiple parameters with the same ramping profile
 */
template<size_t N>
class MultiParameterRamper {
public:
    using RamperFactory = std::function<std::unique_ptr<ParameterRamperBase>()>;
    
    explicit MultiParameterRamper(RamperFactory factory) {
        for (size_t i = 0; i < N; ++i) {
            m_rampers[i] = factory();
        }
    }
    
    void setTargets(const std::array<double, N>& targets, bool immediate = false) {
        for (size_t i = 0; i < N; ++i) {
            m_rampers[i]->setTarget(targets[i], immediate);
        }
    }
    
    void setTarget(size_t index, double target, bool immediate = false) {
        if (index < N) {
            m_rampers[index]->setTarget(target, immediate);
        }
    }
    
    std::array<double, N> update(double dt) {
        std::array<double, N> values;
        for (size_t i = 0; i < N; ++i) {
            values[i] = m_rampers[i]->update(dt);
        }
        return values;
    }
    
    std::array<double, N> getValues() const {
        std::array<double, N> values;
        for (size_t i = 0; i < N; ++i) {
            values[i] = m_rampers[i]->getValue();
        }
        return values;
    }
    
    bool isComplete() const {
        for (const auto& ramper : m_rampers) {
            if (!ramper->isComplete()) return false;
        }
        return true;
    }
    
    void reset(const std::array<double, N>& values) {
        for (size_t i = 0; i < N; ++i) {
            m_rampers[i]->reset(values[i]);
        }
    }
    
    ParameterRamperBase& operator[](size_t index) {
        return *m_rampers[index];
    }
    
private:
    std::array<std::unique_ptr<ParameterRamperBase>, N> m_rampers;
};

} // namespace Control

