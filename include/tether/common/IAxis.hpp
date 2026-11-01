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
     * @param timeoutMs Maximum time to wait for the drive to reach the
     *        enabled state [ms].
     * @return true when the axis is enabled.
     */
    virtual bool enable(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Disable the axis
     * @param timeoutMs Maximum time to wait for the drive to reach the
     *        disabled state [ms].
     * @return true when the axis is disabled.
     */
    virtual bool disable(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Stop motion immediately
     * @return true when the stop request was accepted.
     */
    virtual bool stop() = 0;

    /**
     * @brief Clear a fault condition
     * @return true when the fault was cleared.
     */
    virtual bool clearFault() = 0;

    /**
     * @brief Whether the axis has a fault
     * @return true while a fault is latched.
     */
    virtual bool hasFault() const = 0;

    /**
     * @brief Whether the axis is enabled
     * @return true when the drive accepts motion commands.
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Whether the axis has been homed
     * @return true after a successful homing sequence.
     */
    virtual bool isHomed() const = 0;

    /**
     * @brief Actual position [internal units]
     * @return Position feedback in drive units.
     */
    virtual int32_t getActualPosition() const = 0;

    /**
     * @brief Actual velocity [internal units/s]
     * @return Velocity feedback in drive units per second.
     */
    virtual int32_t getActualVelocity() const = 0;

    /**
     * @brief Actual torque [0.1% of rated]
     * @return Torque feedback in 0.1 % of rated torque.
     */
    virtual int16_t getActualTorque() const = 0;

    /**
     * @brief Set position target
     * @param target Target position in drive units.
     * @param mode Motion command parameters (velocity, acceleration, ...).
     * @return true when the drive accepted the target.
     */
    virtual bool setTargetPosition(int32_t target, const MotionCommand& mode) = 0;

    /**
     * @brief Set velocity target
     * @param target Target velocity in drive units per second.
     * @param mode Velocity command parameters (acceleration, ...).
     * @return true when the drive accepted the target.
     */
    virtual bool setTargetVelocity(int32_t target, const VelocityCommand& mode) = 0;

    /**
     * @brief Set torque target
     * @param target Target torque in 0.1 % of rated torque.
     * @param mode Torque command parameters (ramp, ...).
     * @return true when the drive accepted the target.
     */
    virtual bool setTargetTorque(int16_t target, const TorqueCommand& mode) = 0;

    /**
     * @brief Start homing
     * @param cmd Homing method, speeds and offsets.
     * @return true when the homing sequence was started.
     */
    virtual bool home(const HomingCommand& cmd) = 0;

    /**
     * @brief Wait for the current motion to complete
     * @param timeoutMs Maximum wait time [ms].
     * @return true when the target was reached before the timeout.
     */
    virtual bool waitMotionComplete(uint32_t timeoutMs = 60000) = 0;

    /**
     * @brief Wait for homing to complete
     * @param timeoutMs Maximum wait time [ms].
     * @return true when homing completed before the timeout.
     */
    virtual bool waitHomingComplete(uint32_t timeoutMs = 30000) = 0;

    /**
     * @brief Set per-axis motion limits
     * @param limits Position/velocity/acceleration limits enforced by the axis.
     */
    virtual void setMotionLimits(const MotionLimits& limits) = 0;

    /**
     * @brief Get per-axis motion limits
     * @return Limits currently enforced by the axis.
     */
    virtual MotionLimits getMotionLimits() const = 0;

    /**
     * @brief Per-cycle update
     * @param dtSeconds Time since the previous update [s].
     */
    virtual void update(double dtSeconds) = 0;
};

} // namespace tether::common
