/**
 * @file ISetpointSource.hpp
 * @brief Abstract one-dimensional setpoint source interface
 *
 * A lightweight, generic interface for a single-axis setpoint generator that
 * can be used by motion controllers, CiA 402 drive wrappers, and other
 * components without depending on concrete control implementations.
 */

#pragma once

namespace tether::common {

/**
 * @brief Abstract one-dimensional setpoint source
 *
 * Implementations provide a stream of position/velocity/acceleration values
 * that can be queried once per control cycle. The interface is intentionally
 * minimal so that simple generators (sine waves, step sequences, etc.) can be
 * plugged into protocol-specific drive helpers without introducing a
 * dependency on a concrete controller class.
 */
class ISetpointSource {
public:
    virtual ~ISetpointSource() = default;

    /**
     * @brief Start generating setpoints
     */
    virtual void start() = 0;

    /**
     * @brief Stop generating setpoints smoothly
     */
    virtual void stop() = 0;

    /**
     * @brief Stop immediately with no ramp
     */
    virtual void stopImmediate() { stop(); }

    /**
     * @brief Advance the generator by @p dt seconds and return the new position
     * @param dt Time step in seconds
     * @return Current position
     */
    virtual double update(double dt) = 0;

    /**
     * @brief Get the current position
     */
    virtual double getPosition() const = 0;

    /**
     * @brief Get the current velocity
     */
    virtual double getVelocity() const { return 0.0; }

    /**
     * @brief Get the current acceleration
     */
    virtual double getAcceleration() const { return 0.0; }

    /**
     * @brief Check whether the source is currently running
     */
    virtual bool isRunning() const = 0;
};

} // namespace tether::common
