/**
 * @file StepMotionController.hpp
 * @brief Step-based motion sequence with S-curve ramps and hold times
 *
 * Provides a simple state machine for executing a list of discrete position
 * steps using a jerk-limited SCurveRamper.  Each step moves a relative
 * displacement, then optionally holds for a specified duration before
 * starting the next ramp.  Speeds and acceleration limits may be configured
 * per-step, and the entire sequence can be looped.
 *
 * This controller is intended for use in motion demos (e.g. CSP step
 * profiles) and simple automation tasks where a small number of sequential
 * moves is required.  It is intentionally lightweight and header-only so it
 * can be used in embedded examples without pulling in additional dependencies.
 */

#pragma once

#include "control/ParameterRamping.hpp"
#include "tether/common/ISetpointSource.hpp"
#include <vector>
#include <memory>
#include <algorithm>

namespace tether::control {

class StepMotionController : public tether::common::ISetpointSource {
public:
    /**
     * @brief Single motion step description
     *
     * Displacement is applied relative to the current position when the step
     * begins.  After the ramp completes the controller will remain at the
     * new position for 
     */
    struct Step {
        double displacement = 0.0;               ///< Relative move (radians)
        double holdTime = 0.5;                   ///< Wait time after move (s)
        SCurveRamper::Config profile{};          ///< SCurve parameters (0 = use default)
    };

    /**
     * @brief Global configuration for the controller
     */
    struct Config {
        std::vector<Step> steps;                 ///< Ordered list of steps
        double initialPosition = 0.0;            ///< Starting position (radians)
        bool loop = false;                       ///< Repeat sequence when finished

        SCurveRamper::Config defaultProfile = SCurveRamper::Config::getDefault();

        static Config getDefault() {
            Config cfg;
            cfg.loop = false;
            cfg.initialPosition = 0.0;
            cfg.defaultProfile = SCurveRamper::Config::getDefault();
            return cfg;
        }
    };

    StepMotionController() : StepMotionController(Config::getDefault()) {}
    explicit StepMotionController(const Config& cfg)
        : m_config(cfg)
    {
        // nothing else
    }

    /**
     * @brief Replace the step list (resetting internal state)
     */
    void setSteps(const std::vector<Step>& steps) {
        m_config.steps = steps;
        reset(m_config.initialPosition);
    }

    /**
     * @brief Start or restart the sequence
     */
    void start() override {
        m_position = m_config.initialPosition;
        m_currentStep = 0;
        m_holdTimer = 0.0;
        m_running = true;
        m_ramper.reset();
        if (!m_config.steps.empty()) {
            startStep();
        }
    }

    /**
     * @brief Stop the sequence immediately (ramper is discarded)
     */
    void stop() override {
        m_running = false;
        m_ramper.reset();
    }

    /**
     * @brief Immediate stop
     */
    void stopImmediate() override { stop(); }

    /**
     * @brief Reset controller to a given absolute position
     */
    void reset(double position) {
        m_position = position;
        m_holdTimer = 0.0;
        m_currentStep = 0;
        if (m_ramper) {
            m_ramper->reset(position);
        }
    }

    /**
     * @brief Advance the controller by dt seconds and return current position
     */
    double update(double dt) override {
        if (!m_running) {
            return m_position;
        }

        // if a ramper exists and is still running update it
        if (m_ramper && !m_ramper->isComplete()) {
            m_position = m_ramper->update(dt);
            if (m_ramper->isComplete()) {
                // begin hold after reaching target
                m_holdTimer = 0.0;
            }
            return m_position;
        }

        // either we finished a ramp or no ramp was configured
        if (m_config.steps.empty()) {
            m_running = false;
            return m_position;
        }

        // hold period
        m_holdTimer += dt;
        double hold = m_config.steps[m_currentStep].holdTime;
        if (m_holdTimer >= hold) {
            // advance to next step
            m_currentStep++;
            if (m_currentStep >= m_config.steps.size()) {
                if (m_config.loop) {
                    m_currentStep = 0;
                } else {
                    m_running = false;
                    return m_position;
                }
            }
            startStep();
        }

        return m_position;
    }

    double getPosition() const override { return m_position; }
    double getVelocity() const override { return 0.0; }
    double getAcceleration() const override { return 0.0; }
    bool isRunning() const override { return m_running; }

private:
    void startStep() {
        if (m_currentStep >= m_config.steps.size()) {
            m_running = false;
            return;
        }

        const Step& s = m_config.steps[m_currentStep];
        // choose profile (step-specific override wins)
        SCurveRamper::Config prof = s.profile;
        if (prof.maxVelocity <= 0.0) {
            prof = m_config.defaultProfile;
        }
        m_ramper = std::make_unique<SCurveRamper>(prof);
        m_ramper->reset(m_position);
        double tgt = m_position + s.displacement;
        m_ramper->setTarget(tgt);
    }

    Config m_config;
    size_t m_currentStep{0};
    double m_position{0.0};
    bool m_running{false};
    double m_holdTimer{0.0};
    std::unique_ptr<SCurveRamper> m_ramper;
};

} // namespace tether::control
