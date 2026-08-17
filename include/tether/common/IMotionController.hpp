/**
 * @file IMotionController.hpp
 * @brief Abstract high-level motion controller interface
 *
 * This is the common definition of a multi-axis motion controller. Concrete
 * implementations (e.g. a CiA 402 multi-axis controller) live in their
 * respective components and implement this interface.
 */

#pragma once

#include <cstdint>

namespace tether::common {

/**
 * @brief Abstract high-level multi-axis motion controller interface
 *
 * Implementations handle lifecycle, per-cycle updates, and completion
 * waiting for one or more axes. Command-specific methods (position,
 * velocity, torque, homing, path execution) are intentionally left to
 * concrete classes so that this common base does not depend on any
 * protocol-specific command types.
 */
class IMotionController {
public:
    virtual ~IMotionController() = default;

    /**
     * @brief Enable all axes
     * @return true on success
     */
    virtual bool enableAll(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Disable all axes
     * @return true on success
     */
    virtual bool disableAll(uint32_t timeoutMs = 5000) = 0;

    /**
     * @brief Update all axes - call once per control cycle
     * @param dtSeconds Time since last update
     */
    virtual void update(double dtSeconds) = 0;
};

} // namespace tether::common
