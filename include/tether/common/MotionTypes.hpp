/**
 * @file MotionTypes.hpp
 * @brief Generic motion control types and command structures
 *
 * These types are deliberately protocol-agnostic. Concrete CiA 402, CANopen,
 * or simulated implementations can use them directly or translate them to
 * protocol-specific values internally.
 */

#pragma once

#include <cstdint>

namespace tether::common {

/**
 * @brief Motion profile shape
 */
enum class ProfileType : uint8_t {
    Linear     = 0,
    Trapezoidal = 1,
    Triangular  = 2,
    SCurve      = 3,
    Polynomial  = 4,
    Custom      = 5,
};

/**
 * @brief Kinematic limits for a single axis
 */
struct MotionLimits {
    int32_t maxPosition{0};      ///< Maximum position [internal units]
    int32_t minPosition{0};      ///< Minimum position [internal units]
    uint32_t maxVelocity{0};     ///< Maximum velocity [internal units/s]
    uint32_t maxAcceleration{0}; ///< Maximum acceleration [internal units/s^2]
    uint32_t maxDeceleration{0}; ///< Maximum deceleration [internal units/s^2]
    uint32_t maxJerk{0};         ///< Maximum jerk [internal units/s^3]
};

/**
 * @brief Motion state at a given time
 */
struct MotionState {
    double position{0.0};       ///< Position [user units]
    double velocity{0.0};       ///< Velocity [user units/s]
    double acceleration{0.0};   ///< Acceleration [user units/s^2]
    double jerk{0.0};           ///< Jerk [user units/s^3]
    double time{0.0};           ///< Time from profile start [s]
    bool complete{false};       ///< Profile complete flag
};

/**
 * @brief Point-to-point motion command
 */
struct MotionCommand {
    int32_t targetPosition{0};
    uint32_t velocity{0};        ///< 0 = use profile default
    uint32_t acceleration{0};    ///< 0 = use profile default
    uint32_t deceleration{0};    ///< 0 = use profile default
    uint32_t jerk{0};            ///< 0 = use default (for S-curve)
    bool relative{false};        ///< Relative to current position
    bool immediate{false};       ///< Start immediately (do not wait for previous)
    bool buffered{false};        ///< Add to motion buffer
    ProfileType profileType{ProfileType::Trapezoidal};
};

/**
 * @brief Velocity command
 */
struct VelocityCommand {
    int32_t targetVelocity{0};
    uint32_t acceleration{0};    ///< 0 = use profile default
    uint32_t deceleration{0};    ///< 0 = use profile default
    int32_t maxDuration{-1};     ///< -1 = indefinite
};

/**
 * @brief Torque command
 */
struct TorqueCommand {
    int16_t targetTorque{0};
    int16_t torqueSlope{0};      ///< Torque ramp rate
    int32_t maxDuration{-1};     ///< -1 = indefinite
};

/**
 * @brief Homing method identifier
 *
 * The interpretation of the numeric method id is left to the concrete axis
 * implementation (e.g. CiA 402 defines values 1-37).
 */
struct HomingCommand {
    uint16_t method{0};          ///< 0 = no homing
    uint32_t speedSwitch{1000};
    uint32_t speedZero{100};
    uint32_t acceleration{1000};
    int32_t offset{0};
    uint32_t timeoutMs{30000};   ///< Default homing timeout
};

} // namespace tether::common
