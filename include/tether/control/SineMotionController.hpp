/**
 * @file SineMotionController.hpp
 * @brief Sine wave motion controller with configurable parameters and phase offset
 * 
 * @details
 * Generates sinusoidal motion profiles with support for:
 * - Configurable amplitude, frequency, offset, and phase
 * - Parameter ramping using ParameterRamping framework
 * - Multi-axis coordination with phase offsets
 * - Smooth start/stop with envelope functions
 * - Frequency sweep capabilities
 * - Superposition of multiple frequencies
 */

#pragma once

#include "control/ParameterRamping.hpp"
#include <cmath>
#include <array>
#include <memory>
#include <vector>
#include <functional>

namespace Control {

// ============================================================================
// Sine Motion Controller
// ============================================================================

/**
 * @brief Sine wave motion generator with smooth parameter transitions
 */
class SineMotionController {
public:
    /**
     * @brief Configuration parameters
     */
    struct Config {
        double amplitude = 1.0;          ///< Peak amplitude
        double frequency = 1.0;          ///< Frequency in Hz
        double offset = 0.0;             ///< DC offset
        double phaseOffset = 0.0;        ///< Phase offset in radians
        double startPhase = 0.0;         ///< Initial phase when started
        
        // Ramping configuration
        double parameterRampTime = 1.0;  ///< Time to ramp parameters (seconds)
        bool useSmoothing = true;        ///< Use sigmoidal smoothing for params
        
        // Envelope
        bool useEnvelope = false;        ///< Apply amplitude envelope
        double attackTime = 0.5;         ///< Envelope attack time
        double releaseTime = 0.5;        ///< Envelope release time
        
        static Config getDefault() {
            return Config{1.0, 1.0, 0.0, 0.0, 0.0, 1.0, true, false, 0.5, 0.5};
        }
    };
    
    /**
     * @brief Current state
     */
    struct State {
        double position{0.0};           ///< Current position
        double velocity{0.0};           ///< Current velocity
        double acceleration{0.0};       ///< Current acceleration
        double phase{0.0};              ///< Current phase (radians)
        double time{0.0};               ///< Time since start
        bool running{false};            ///< Is oscillation active
    };
    
    SineMotionController() : SineMotionController(Config::getDefault()) {}
    explicit SineMotionController(const Config& config);
    
    /**
     * @brief Start oscillation
     * @param initialPhase Starting phase (radians)
     */
    void start(double initialPhase = 0.0);
    
    /**
     * @brief Stop oscillation with smooth envelope
     */
    void stop();
    
    /**
     * @brief Immediate stop
     */
    void stopImmediate();
    
    /**
     * @brief Update and get current position
     * @param dt Time step
     * @return Current position
     */
    double update(double dt);
    
    /**
     * @brief Get current state
     */
    const State& getState() const { return m_state; }
    
    /**
     * @brief Get current position
     */
    double getPosition() const { return m_state.position; }
    
    /**
     * @brief Get current velocity
     */
    double getVelocity() const { return m_state.velocity; }
    
    /**
     * @brief Get current acceleration
     */
    double getAcceleration() const { return m_state.acceleration; }

    /**
     * @brief Convenience helpers that apply a scaling factor to the output.
     *
     * Useful when the controller is operating in radians and the caller
     * wants values in encoder counts (scale = counts_per_radian).
     */
    int32_t getPositionScaled(double scale) const {
        return static_cast<int32_t>(m_state.position * scale);
    }
    int32_t getVelocityScaled(double scale) const {
        return static_cast<int32_t>(m_state.velocity * scale);
    }
    int32_t getAccelerationScaled(double scale) const {
        return static_cast<int32_t>(m_state.acceleration * scale);
    }
    
    // ========== Parameter Setters (with smooth ramping) ==========
    
    /**
     * @brief Set amplitude (smoothly ramped)
     */
    void setAmplitude(double amplitude, bool immediate = false);
    
    /**
     * @brief Set frequency (smoothly ramped)
     */
    void setFrequency(double frequency, bool immediate = false);
    
    /**
     * @brief Set DC offset (smoothly ramped)
     */
    void setOffset(double offset, bool immediate = false);
    
    /**
     * @brief Set phase offset (smoothly ramped)
     */
    void setPhaseOffset(double phase, bool immediate = false);
    
    /**
     * @brief Set all parameters at once
     */
    void setParameters(double amplitude, double frequency, 
                       double offset, double phaseOffset,
                       bool immediate = false);
    
    /**
     * @brief Set parameter ramp time
     */
    void setRampTime(double time);
    
    // ========== Getters ==========
    
    double getAmplitude() const { return m_amplitudeRamper->getValue(); }
    double getFrequency() const { return m_frequencyRamper->getValue(); }
    double getOffset() const { return m_offsetRamper->getValue(); }
    double getPhaseOffset() const { return m_phaseOffsetRamper->getValue(); }
    
    double getTargetAmplitude() const { return m_amplitudeRamper->getTarget(); }
    double getTargetFrequency() const { return m_frequencyRamper->getTarget(); }
    
    /**
     * @brief Check if all parameter transitions are complete
     */
    bool isParameterStable() const;
    
    /**
     * @brief Reset to initial state
     */
    void reset();
    
private:
    Config m_config;
    State m_state;
    
    // Parameter rampers
    std::unique_ptr<ParameterRamperBase> m_amplitudeRamper;
    std::unique_ptr<ParameterRamperBase> m_frequencyRamper;
    std::unique_ptr<ParameterRamperBase> m_offsetRamper;
    std::unique_ptr<ParameterRamperBase> m_phaseOffsetRamper;
    
    // Envelope state
    enum class EnvelopeState { Idle, Attack, Sustain, Release };
    EnvelopeState m_envelopeState{EnvelopeState::Idle};
    double m_envelope{0.0};
    double m_envelopeTime{0.0};
    
    std::unique_ptr<ParameterRamperBase> createRamper();
    void updateEnvelope(double dt);
};

// ============================================================================
// Multi-Axis Sine Controller
// ============================================================================

/**
 * @brief Coordinated multi-axis sine motion with phase offsets
 */
template<size_t NumAxes>
class MultiAxisSineController {
public:
    struct AxisConfig {
        double amplitude{1.0};
        double phaseOffset{0.0};        ///< Phase offset relative to master
        double offset{0.0};
        bool enabled{true};
    };
    
    struct Config {
        double frequency{1.0};          ///< Master frequency (shared)
        double rampTime{1.0};
        std::array<AxisConfig, NumAxes> axes;
    };
    
    explicit MultiAxisSineController(const Config& config = {})
        : m_config(config) {
        for (size_t i = 0; i < NumAxes; ++i) {
            SineMotionController::Config axisConfig;
            axisConfig.amplitude = config.axes[i].amplitude;
            axisConfig.frequency = config.frequency;
            axisConfig.offset = config.axes[i].offset;
            axisConfig.phaseOffset = config.axes[i].phaseOffset;
            axisConfig.parameterRampTime = config.rampTime;
            m_controllers[i] = std::make_unique<SineMotionController>(axisConfig);
        }
    }
    
    /**
     * @brief Start all axes
     */
    void start() {
        for (auto& ctrl : m_controllers) {
            ctrl->start();
        }
    }
    
    /**
     * @brief Stop all axes
     */
    void stop() {
        for (auto& ctrl : m_controllers) {
            ctrl->stop();
        }
    }
    
    /**
     * @brief Update all axes
     * @return Positions for all axes
     */
    std::array<double, NumAxes> update(double dt) {
        std::array<double, NumAxes> positions;
        for (size_t i = 0; i < NumAxes; ++i) {
            if (m_config.axes[i].enabled) {
                positions[i] = m_controllers[i]->update(dt);
            } else {
                positions[i] = m_config.axes[i].offset;
            }
        }
        return positions;
    }
    
    /**
     * @brief Set master frequency (affects all axes)
     */
    void setFrequency(double freq, bool immediate = false) {
        m_config.frequency = freq;
        for (auto& ctrl : m_controllers) {
            ctrl->setFrequency(freq, immediate);
        }
    }
    
    /**
     * @brief Set amplitude for specific axis
     */
    void setAxisAmplitude(size_t axis, double amplitude, bool immediate = false) {
        if (axis < NumAxes) {
            m_config.axes[axis].amplitude = amplitude;
            m_controllers[axis]->setAmplitude(amplitude, immediate);
        }
    }
    
    /**
     * @brief Set phase offset for specific axis
     */
    void setAxisPhaseOffset(size_t axis, double phase, bool immediate = false) {
        if (axis < NumAxes) {
            m_config.axes[axis].phaseOffset = phase;
            m_controllers[axis]->setPhaseOffset(phase, immediate);
        }
    }
    
    /**
     * @brief Configure circular motion (2D) using first two axes
     * @param radius Circle radius
     */
    void configureCircular(double radius) {
        static_assert(NumAxes >= 2, "Need at least 2 axes for circular motion");
        m_controllers[0]->setAmplitude(radius);
        m_controllers[0]->setPhaseOffset(0.0);
        m_controllers[1]->setAmplitude(radius);
        m_controllers[1]->setPhaseOffset(M_PI / 2.0);  // 90 degree offset
    }
    
    /**
     * @brief Configure elliptical motion
     */
    void configureElliptical(double majorAxis, double minorAxis, double rotation = 0.0) {
        static_assert(NumAxes >= 2, "Need at least 2 axes for elliptical motion");
        // TODO: Apply rotation matrix
        m_controllers[0]->setAmplitude(majorAxis);
        m_controllers[0]->setPhaseOffset(0.0);
        m_controllers[1]->setAmplitude(minorAxis);
        m_controllers[1]->setPhaseOffset(M_PI / 2.0);
    }
    
    /**
     * @brief Configure Lissajous pattern
     */
    void configureLissajous(double ampX, double ampY, int freqRatioX, int freqRatioY,
                            double phaseDiff = 0.0) {
        static_assert(NumAxes >= 2, "Need at least 2 axes for Lissajous");
        // For Lissajous, we need different frequencies per axis
        // This simplified version uses phase difference only
        m_controllers[0]->setAmplitude(ampX);
        m_controllers[0]->setPhaseOffset(0.0);
        m_controllers[1]->setAmplitude(ampY);
        m_controllers[1]->setPhaseOffset(phaseDiff);
    }
    
    SineMotionController& getAxis(size_t index) {
        return *m_controllers[index];
    }
    
private:
    Config m_config;
    std::array<std::unique_ptr<SineMotionController>, NumAxes> m_controllers;
};

// ============================================================================
// Frequency Sweep Generator
// ============================================================================

/**
 * @brief Sine sweep (chirp) generator for system identification
 */
class FrequencySweepGenerator {
public:
    enum class SweepType {
        Linear,             ///< Linear frequency sweep
        Logarithmic,        ///< Logarithmic (exponential) sweep
        Sine               ///< Sinusoidal frequency modulation
    };
    
    struct Config {
        double startFrequency = 0.1;
        double endFrequency = 10.0;
        double amplitude = 1.0;
        double sweepDuration = 60.0;     ///< Total sweep time
        SweepType type = SweepType::Logarithmic;
        int cycles = 0;                  ///< 0 = one-shot, >0 = repeat
        
        static Config getDefault() {
            return Config{0.1, 10.0, 1.0, 60.0, SweepType::Logarithmic, 0};
        }
    };
    
    FrequencySweepGenerator() : m_config(Config::getDefault()) {}
    explicit FrequencySweepGenerator(const Config& config)
        : m_config(config) {}
    
    void start() {
        m_time = 0.0;
        m_phase = 0.0;
        m_running = true;
        m_cycleCount = 0;
    }
    
    void stop() {
        m_running = false;
    }
    
    double update(double dt) {
        if (!m_running) return 0.0;
        
        double freq = getCurrentFrequency();
        double omega = 2.0 * M_PI * freq;
        
        m_phase += omega * dt;
        m_time += dt;
        
        // Check for cycle completion
        if (m_time >= m_config.sweepDuration) {
            m_cycleCount++;
            if (m_config.cycles > 0 && m_cycleCount >= m_config.cycles) {
                m_running = false;
            } else {
                m_time = 0.0;
                m_phase = 0.0;
            }
        }
        
        return m_config.amplitude * std::sin(m_phase);
    }
    
    double getCurrentFrequency() const {
        double t = m_time / m_config.sweepDuration;  // 0 to 1
        
        switch (m_config.type) {
            case SweepType::Linear:
                return m_config.startFrequency + 
                       t * (m_config.endFrequency - m_config.startFrequency);
                
            case SweepType::Logarithmic: {
                double logStart = std::log(m_config.startFrequency);
                double logEnd = std::log(m_config.endFrequency);
                return std::exp(logStart + t * (logEnd - logStart));
            }
            
            case SweepType::Sine: {
                double midFreq = (m_config.startFrequency + m_config.endFrequency) / 2.0;
                double freqRange = (m_config.endFrequency - m_config.startFrequency) / 2.0;
                return midFreq + freqRange * std::sin(2.0 * M_PI * t);
            }
        }
        
        return m_config.startFrequency;
    }
    
    double getProgress() const {
        return m_time / m_config.sweepDuration;
    }
    
    bool isRunning() const { return m_running; }
    
private:
    Config m_config;
    double m_time{0.0};
    double m_phase{0.0};
    bool m_running{false};
    int m_cycleCount{0};
};

// ============================================================================
// Superposition Generator
// ============================================================================

/**
 * @brief Generates sum of multiple sine waves
 */
class SuperpositionGenerator {
public:
    struct Harmonic {
        double frequency{1.0};
        double amplitude{1.0};
        double phase{0.0};
    };
    
    void addHarmonic(double frequency, double amplitude, double phase = 0.0) {
        m_harmonics.push_back({frequency, amplitude, phase});
    }
    
    void clearHarmonics() {
        m_harmonics.clear();
    }
    
    void start() {
        m_time = 0.0;
        m_running = true;
    }
    
    void stop() {
        m_running = false;
    }
    
    double update(double dt) {
        if (!m_running) return 0.0;
        
        m_time += dt;
        
        double output = 0.0;
        for (const auto& h : m_harmonics) {
            output += h.amplitude * std::sin(2.0 * M_PI * h.frequency * m_time + h.phase);
        }
        
        return output;
    }
    
    /**
     * @brief Configure as square wave approximation
     */
    void configureSquareWave(double fundamentalFreq, double amplitude, int harmonics = 5) {
        clearHarmonics();
        for (int n = 1; n <= harmonics * 2; n += 2) {
            addHarmonic(n * fundamentalFreq, amplitude * 4.0 / (M_PI * n));
        }
    }
    
    /**
     * @brief Configure as sawtooth wave approximation
     */
    void configureSawtoothWave(double fundamentalFreq, double amplitude, int harmonics = 10) {
        clearHarmonics();
        for (int n = 1; n <= harmonics; ++n) {
            addHarmonic(n * fundamentalFreq, amplitude * 2.0 / (M_PI * n) * (n % 2 == 0 ? -1 : 1));
        }
    }
    
    /**
     * @brief Configure as triangle wave approximation
     */
    void configureTriangleWave(double fundamentalFreq, double amplitude, int harmonics = 5) {
        clearHarmonics();
        for (int n = 1; n <= harmonics * 2; n += 2) {
            double sign = ((n - 1) / 2 % 2 == 0) ? 1.0 : -1.0;
            addHarmonic(n * fundamentalFreq, sign * amplitude * 8.0 / (M_PI * M_PI * n * n));
        }
    }
    
private:
    std::vector<Harmonic> m_harmonics;
    double m_time{0.0};
    bool m_running{false};
};

// ============================================================================
// Implementation
// ============================================================================

inline SineMotionController::SineMotionController(const Config& config)
    : m_config(config) {
    m_amplitudeRamper = createRamper();
    m_frequencyRamper = createRamper();
    m_offsetRamper = createRamper();
    m_phaseOffsetRamper = createRamper();
    
    m_amplitudeRamper->reset(config.amplitude);
    m_frequencyRamper->reset(config.frequency);
    m_offsetRamper->reset(config.offset);
    m_phaseOffsetRamper->reset(config.phaseOffset);
}

inline std::unique_ptr<ParameterRamperBase> SineMotionController::createRamper() {
    if (m_config.useSmoothing) {
        SigmoidalRamper::Config cfg;
        cfg.duration = m_config.parameterRampTime;
        cfg.steepness = 5.0;
        return std::make_unique<SigmoidalRamper>(cfg);
    } else {
        ConstantTimeRamper::Config cfg;
        cfg.rampTime = m_config.parameterRampTime;
        return std::make_unique<ConstantTimeRamper>(cfg);
    }
}

inline void SineMotionController::start(double initialPhase) {
    m_state.phase = initialPhase + m_config.startPhase;
    m_state.time = 0.0;
    m_state.running = true;
    
    if (m_config.useEnvelope) {
        m_envelopeState = EnvelopeState::Attack;
        m_envelope = 0.0;
        m_envelopeTime = 0.0;
    } else {
        m_envelopeState = EnvelopeState::Sustain;
        m_envelope = 1.0;
    }
}

inline void SineMotionController::stop() {
    if (m_config.useEnvelope) {
        m_envelopeState = EnvelopeState::Release;
        m_envelopeTime = 0.0;
    } else {
        m_state.running = false;
    }
}

inline void SineMotionController::stopImmediate() {
    m_state.running = false;
    m_envelopeState = EnvelopeState::Idle;
    m_envelope = 0.0;
}

inline double SineMotionController::update(double dt) {
    // Update parameter rampers
    double amplitude = m_amplitudeRamper->update(dt);
    double frequency = m_frequencyRamper->update(dt);
    double offset = m_offsetRamper->update(dt);
    double phaseOffset = m_phaseOffsetRamper->update(dt);
    
    if (!m_state.running && m_envelopeState == EnvelopeState::Idle) {
        m_state.position = offset;
        m_state.velocity = 0.0;
        m_state.acceleration = 0.0;
        return m_state.position;
    }
    
    // Update envelope
    updateEnvelope(dt);
    
    // Update phase
    double omega = 2.0 * M_PI * frequency;
    m_state.phase += omega * dt;
    m_state.time += dt;
    
    // Keep phase in reasonable range to avoid precision issues
    while (m_state.phase > 2.0 * M_PI) {
        m_state.phase -= 2.0 * M_PI;
    }
    
    // Calculate position with envelope
    double effectiveAmplitude = amplitude * m_envelope;
    double totalPhase = m_state.phase + phaseOffset;
    
    m_state.position = offset + effectiveAmplitude * std::sin(totalPhase);
    m_state.velocity = effectiveAmplitude * omega * std::cos(totalPhase);
    m_state.acceleration = -effectiveAmplitude * omega * omega * std::sin(totalPhase);
    
    return m_state.position;
}

inline void SineMotionController::updateEnvelope(double dt) {
    switch (m_envelopeState) {
        case EnvelopeState::Attack:
            m_envelopeTime += dt;
            if (m_envelopeTime >= m_config.attackTime) {
                m_envelope = 1.0;
                m_envelopeState = EnvelopeState::Sustain;
            } else {
                // Smooth attack using quintic
                double t = m_envelopeTime / m_config.attackTime;
                m_envelope = 6.0*t*t*t*t*t - 15.0*t*t*t*t + 10.0*t*t*t;
            }
            break;
            
        case EnvelopeState::Sustain:
            m_envelope = 1.0;
            break;
            
        case EnvelopeState::Release:
            m_envelopeTime += dt;
            if (m_envelopeTime >= m_config.releaseTime) {
                m_envelope = 0.0;
                m_envelopeState = EnvelopeState::Idle;
                m_state.running = false;
            } else {
                double t = m_envelopeTime / m_config.releaseTime;
                m_envelope = 1.0 - (6.0*t*t*t*t*t - 15.0*t*t*t*t + 10.0*t*t*t);
            }
            break;
            
        case EnvelopeState::Idle:
            m_envelope = 0.0;
            break;
    }
}

inline void SineMotionController::setAmplitude(double amplitude, bool immediate) {
    m_amplitudeRamper->setTarget(amplitude, immediate);
}

inline void SineMotionController::setFrequency(double frequency, bool immediate) {
    m_frequencyRamper->setTarget(frequency, immediate);
}

inline void SineMotionController::setOffset(double offset, bool immediate) {
    m_offsetRamper->setTarget(offset, immediate);
}

inline void SineMotionController::setPhaseOffset(double phase, bool immediate) {
    m_phaseOffsetRamper->setTarget(phase, immediate);
}

inline void SineMotionController::setParameters(double amplitude, double frequency,
                                                double offset, double phaseOffset,
                                                bool immediate) {
    setAmplitude(amplitude, immediate);
    setFrequency(frequency, immediate);
    setOffset(offset, immediate);
    setPhaseOffset(phaseOffset, immediate);
}

inline void SineMotionController::setRampTime(double time) {
    m_config.parameterRampTime = time;
    // Recreate rampers with new time
    double amp = m_amplitudeRamper->getValue();
    double freq = m_frequencyRamper->getValue();
    double off = m_offsetRamper->getValue();
    double phase = m_phaseOffsetRamper->getValue();
    
    m_amplitudeRamper = createRamper();
    m_frequencyRamper = createRamper();
    m_offsetRamper = createRamper();
    m_phaseOffsetRamper = createRamper();
    
    m_amplitudeRamper->reset(amp);
    m_frequencyRamper->reset(freq);
    m_offsetRamper->reset(off);
    m_phaseOffsetRamper->reset(phase);
}

inline bool SineMotionController::isParameterStable() const {
    return m_amplitudeRamper->isComplete() &&
           m_frequencyRamper->isComplete() &&
           m_offsetRamper->isComplete() &&
           m_phaseOffsetRamper->isComplete();
}

inline void SineMotionController::reset() {
    m_state = State{};
    m_amplitudeRamper->reset(m_config.amplitude);
    m_frequencyRamper->reset(m_config.frequency);
    m_offsetRamper->reset(m_config.offset);
    m_phaseOffsetRamper->reset(m_config.phaseOffset);
    m_envelopeState = EnvelopeState::Idle;
    m_envelope = 0.0;
    m_envelopeTime = 0.0;
}

} // namespace Control

