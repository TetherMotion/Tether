/**
 * @file IAxis.hpp
 * @brief Abstract drive-axis interface
 *
 * This is the neutral layer between concrete drive protocols
 * (e.g. CiA 402 over EtherCAT) and higher-level motion controllers.
 */

#pragma once

#include "tether/common/MotionTypes.hpp"

#include <cstdint>

namespace tether::common {

/**
 * @brief Drive-agnostic single-axis interface
 */
class IAxis {
public:
    using AxisId = uint32_t;

    virtual ~IAxis() = default;

    /**
     * @brief Enable the axis
     */
    virtual bool enable(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Disable the axis
     */
    virtual bool disable(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Stop motion immediately
     */
    virtual bool stop() = 0;

    /**
     * @brief Clear a fault condition
     */
    virtual bool clearFault() = 0;

    /**
     * @brief Whether the axis has a fault
     */
    virtual bool hasFault() const = 0;

    /**
     * @brief Whether the axis is enabled
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Whether the axis has been homed
     */
    virtual bool isHomed() const = 0;

    /**
     * @brief Actual position [internal units]
     */
    virtual int32_t getActualPosition() const = 0;

    /**
     * @brief Actual velocity [internal units/s]
     */
    virtual int32_t getActualVelocity() const = 0;

    /**
     * @brief Actual torque [0.1% of rated]
     */
    virtual int16_t getActualTorque() const = 0;

    /**
     * @brief Set position target
     */
    virtual bool setTargetPosition(int32_t target, const MotionCommand& mode) = 0;

    /**
     * @brief Set velocity target
     */
    virtual bool setTargetVelocity(int32_t target, const VelocityCommand& mode) = 0;

    /**
     * @brief Set torque target
     */
    virtual bool setTargetTorque(int16_t target, const TorqueCommand& mode) = 0;

    /**
     * @brief Start homing
     */
    virtual bool home(const HomingCommand& cmd) = 0;

    /**
     * @brief Wait for the current motion to complete
     */
    virtual bool waitMotionComplete(uint32_t timeoutMs = 60000) = 0;

    /**
     * @brief Wait for homing to complete
     */
    virtual bool waitHomingComplete(uint32_t timeoutMs = 30000) = 0;

    /**
     * @brief Set per-axis motion limits
     */
    virtual void setMotionLimits(const MotionLimits& limits) = 0;

    /**
     * @brief Get per-axis motion limits
     */
    virtual MotionLimits getMotionLimits() const = 0;

    /**
     * @brief Per-cycle update
     */
    virtual void update(double dtSeconds) = 0;
};

} // namespace tether::common
