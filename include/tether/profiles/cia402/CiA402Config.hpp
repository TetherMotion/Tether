/**
 * @file CiA402Config.hpp
 * @brief Centralized configuration for CiA 402 motion control stack
 * 
 * @details
 * This file contains all user-configurable options for the CiA 402 motion
 * control implementation. The stack is designed to be backend-agnostic,
 * allowing use with different EtherCAT masters or other communication backends.
 * 
 * ## Architecture Overview
 * 
 * ```
 *                    ┌─────────────────────────────────────┐
 *                    │         User Application            │
 *                    └───────────────┬─────────────────────┘
 *                                    │
 *                    ┌───────────────┴─────────────────────┐
 *                    │     MotionController (Facade)       │
 *                    │  - Axis management                  │
 *                    │  - Coordinated motion               │
 *                    │  - Global speed control             │
 *                    └───────────────┬─────────────────────┘
 *                                    │
 *          ┌─────────────────────────┼─────────────────────────┐
 *          │                         │                         │
 *   ┌──────┴──────┐          ┌───────┴───────┐         ┌───────┴───────┐
 *   │ CiA402Axis  │          │ CiA402Axis    │         │ CiA402Axis    │
 *   │ (Drive 0)   │          │ (Drive 1)     │         │ (Drive N)     │
 *   └──────┬──────┘          └───────┬───────┘         └───────┬───────┘
 *          │                         │                         │
 *   ┌──────┴──────┐          ┌───────┴───────┐         ┌───────┴───────┐
 *   │ DriveBackend│          │ DriveBackend  │         │ DriveBackend  │
 *   │ (EtherCAT)  │          │ (EtherCAT)    │         │ (Other)       │
 *   └─────────────┘          └───────────────┘         └───────────────┘
 * ```
 * 
 * ## Motion Generation Pipeline
 * 
 * ```
 *   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
 *   │ Path Planner │───►│ Motion Prof  │───►│ Interpolator │───► Position Cmd
 *   │ (Trajectory) │    │ (S-curve)    │    │ (Per-cycle)  │
 *   └──────────────┘    └──────────────┘    └──────────────┘
 * ```
 * 
 * ## Configuration Categories
 * 
 * 1. **Feature Enables** - Enable/disable optional features
 * 2. **Axis Configuration** - Max axes, naming, limits
 * 3. **Motion Parameters** - Default velocities, accelerations, jerk
 * 4. **Profile Settings** - Motion profile types and parameters
 * 5. **Timing Configuration** - Sample rates, interpolation
 * 6. **PID/Filter Settings** - Control loop parameters
 * 7. **Homing Configuration** - Homing modes and parameters
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cfloat>

// ============================================================================
// FEATURE ENABLES
// ============================================================================

/**
 * @brief Enable S-curve motion profile support
 * 
 * S-curves provide smooth motion with continuous jerk, reducing mechanical
 * stress and improving positioning accuracy.
 * 
 * @note Recommendation: Enable for precision motion applications
 * Code size: ~4 KB
 */
#ifndef CIA402_FEATURE_SCURVE_ENABLED
#define CIA402_FEATURE_SCURVE_ENABLED       1
#endif

/**
 * @brief Enable multi-axis coordinated motion
 * 
 * Allows synchronized motion across multiple axes for path following
 * (linear, circular, helical, spline paths).
 * 
 * @note Recommendation: Enable for CNC, robotics, multi-axis systems
 * Code size: ~8 KB
 */
#ifndef CIA402_FEATURE_MULTIAXIS_ENABLED
#define CIA402_FEATURE_MULTIAXIS_ENABLED    1
#endif

/**
 * @brief Enable B-spline and NURBS path interpolation
 * 
 * Provides smooth curved paths using B-spline and NURBS representations.
 * 
 * @note Recommendation: Enable for complex curved paths (CAM, robotics)
 * Code size: ~6 KB
 */
#ifndef CIA402_FEATURE_SPLINE_ENABLED
#define CIA402_FEATURE_SPLINE_ENABLED       1
#endif

/**
 * @brief Enable electronic gearing
 * 
 * Allows slave axes to follow a master axis with configurable gear ratios.
 * 
 * @note Recommendation: Enable for gear trains, synchronized systems
 * Code size: ~2 KB
 */
#ifndef CIA402_FEATURE_GEARING_ENABLED
#define CIA402_FEATURE_GEARING_ENABLED      1
#endif

/**
 * @brief Enable homing support
 * 
 * Provides all CiA 402 homing methods (1-35+).
 * 
 * @note Recommendation: Enable unless drives handle homing internally
 * Code size: ~4 KB
 */
#ifndef CIA402_FEATURE_HOMING_ENABLED
#define CIA402_FEATURE_HOMING_ENABLED       1
#endif

/**
 * @brief Enable PID controller support
 * 
 * Built-in PID controller with anti-windup and filtering.
 * Use for drives requiring host-side position/velocity control.
 * 
 * @note Recommendation: Enable for simple drives without onboard control
 * Code size: ~2 KB
 */
#ifndef CIA402_FEATURE_PID_ENABLED
#define CIA402_FEATURE_PID_ENABLED          1
#endif

/**
 * @brief Enable polynomial motion profiles
 * 
 * Supports polynomial motion generation (3rd, 5th, 7th order).
 * 
 * @note Recommendation: Enable for custom motion profiles
 * Code size: ~2 KB
 */
#ifndef CIA402_FEATURE_POLYNOMIAL_ENABLED
#define CIA402_FEATURE_POLYNOMIAL_ENABLED   1
#endif

// ============================================================================
// AXIS CONFIGURATION
// ============================================================================

/**
 * @brief Maximum number of axes supported
 * 
 * Defines the maximum number of axes that can be managed simultaneously.
 * Each axis consumes ~2KB of RAM for state and buffers.
 * 
 * @note Typical values: 1-8 for most applications, up to 32 for large systems
 */
#ifndef CIA402_MAX_AXES
#define CIA402_MAX_AXES                     8
#endif

/**
 * @brief Maximum axis name length
 * 
 * Maximum characters for axis names (e.g., "X", "Y", "Spindle").
 */
#ifndef CIA402_MAX_AXIS_NAME
#define CIA402_MAX_AXIS_NAME                16
#endif

/**
 * @brief Maximum number of axes in coordinated motion group
 * 
 * Limits how many axes can participate in a single coordinated move.
 */
#ifndef CIA402_MAX_GROUP_AXES
#define CIA402_MAX_GROUP_AXES               6
#endif

// ============================================================================
// MOTION PARAMETERS - DEFAULTS
// ============================================================================

/**
 * @brief Default maximum velocity (user units per second)
 * 
 * Applied when no axis-specific limit is configured.
 * Units depend on application (mm/s, deg/s, counts/s, etc.)
 */
#ifndef CIA402_DEFAULT_MAX_VELOCITY
#define CIA402_DEFAULT_MAX_VELOCITY         1000.0
#endif

/**
 * @brief Default maximum acceleration (user units per second²)
 */
#ifndef CIA402_DEFAULT_MAX_ACCELERATION
#define CIA402_DEFAULT_MAX_ACCELERATION     10000.0
#endif

/**
 * @brief Default maximum deceleration (user units per second²)
 * 
 * Set to 0 to use same value as acceleration.
 */
#ifndef CIA402_DEFAULT_MAX_DECELERATION
#define CIA402_DEFAULT_MAX_DECELERATION     0.0
#endif

/**
 * @brief Default maximum jerk (user units per second³)
 * 
 * Used for S-curve and advanced motion profiles.
 * Higher values = faster ramps but more mechanical stress.
 */
#ifndef CIA402_DEFAULT_MAX_JERK
#define CIA402_DEFAULT_MAX_JERK             100000.0
#endif

/**
 * @brief Default position tolerance for "in position" detection
 */
#ifndef CIA402_DEFAULT_POSITION_TOLERANCE
#define CIA402_DEFAULT_POSITION_TOLERANCE   0.01
#endif

/**
 * @brief Default velocity tolerance for "at velocity" detection
 */
#ifndef CIA402_DEFAULT_VELOCITY_TOLERANCE
#define CIA402_DEFAULT_VELOCITY_TOLERANCE   0.1
#endif

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

/**
 * @brief Default motion cycle time in microseconds
 * 
 * This is the base sample rate for motion generation.
 * Common values:
 * - 1000 µs (1 kHz) - Standard motion control
 * - 500 µs (2 kHz) - High-performance servo
 * - 250 µs (4 kHz) - Ultra-high performance
 * 
 * @note Must match or be divisible by the communication cycle time
 */
#ifndef CIA402_DEFAULT_CYCLE_TIME_US
#define CIA402_DEFAULT_CYCLE_TIME_US        1000
#endif

/**
 * @brief Trajectory buffer depth (number of points)
 * 
 * How many motion points can be queued ahead.
 * Larger = smoother motion but more latency.
 */
#ifndef CIA402_TRAJECTORY_BUFFER_SIZE
#define CIA402_TRAJECTORY_BUFFER_SIZE       64
#endif

/**
 * @brief Interpolation lookahead points
 * 
 * Number of future points considered for interpolation.
 */
#ifndef CIA402_INTERPOLATION_LOOKAHEAD
#define CIA402_INTERPOLATION_LOOKAHEAD      8
#endif

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================

/**
 * @brief Default position loop P gain
 */
#ifndef CIA402_DEFAULT_KP_POSITION
#define CIA402_DEFAULT_KP_POSITION          10.0
#endif

/**
 * @brief Default position loop I gain
 */
#ifndef CIA402_DEFAULT_KI_POSITION
#define CIA402_DEFAULT_KI_POSITION          0.0
#endif

/**
 * @brief Default position loop D gain
 */
#ifndef CIA402_DEFAULT_KD_POSITION
#define CIA402_DEFAULT_KD_POSITION          0.0
#endif

/**
 * @brief Default velocity loop P gain
 */
#ifndef CIA402_DEFAULT_KP_VELOCITY
#define CIA402_DEFAULT_KP_VELOCITY          1.0
#endif

/**
 * @brief Default velocity loop I gain
 */
#ifndef CIA402_DEFAULT_KI_VELOCITY
#define CIA402_DEFAULT_KI_VELOCITY          0.1
#endif

/**
 * @brief Default velocity loop D gain
 */
#ifndef CIA402_DEFAULT_KD_VELOCITY
#define CIA402_DEFAULT_KD_VELOCITY          0.0
#endif

/**
 * @brief Default integrator anti-windup limit
 */
#ifndef CIA402_DEFAULT_INTEGRATOR_LIMIT
#define CIA402_DEFAULT_INTEGRATOR_LIMIT     1000.0
#endif

// ============================================================================
// FILTER CONFIGURATION
// ============================================================================

/**
 * @brief Enable low-pass filter on position command
 */
#ifndef CIA402_FILTER_POSITION_ENABLED
#define CIA402_FILTER_POSITION_ENABLED      0
#endif

/**
 * @brief Default position filter cutoff frequency (Hz)
 */
#ifndef CIA402_FILTER_POSITION_CUTOFF_HZ
#define CIA402_FILTER_POSITION_CUTOFF_HZ    100.0
#endif

/**
 * @brief Enable low-pass filter on velocity command
 */
#ifndef CIA402_FILTER_VELOCITY_ENABLED
#define CIA402_FILTER_VELOCITY_ENABLED      0
#endif

/**
 * @brief Default velocity filter cutoff frequency (Hz)
 */
#ifndef CIA402_FILTER_VELOCITY_CUTOFF_HZ
#define CIA402_FILTER_VELOCITY_CUTOFF_HZ    50.0
#endif

/**
 * @brief Enable notch filter for resonance suppression
 */
#ifndef CIA402_FILTER_NOTCH_ENABLED
#define CIA402_FILTER_NOTCH_ENABLED         0
#endif

/**
 * @brief Default notch filter center frequency (Hz)
 */
#ifndef CIA402_FILTER_NOTCH_FREQ_HZ
#define CIA402_FILTER_NOTCH_FREQ_HZ         100.0
#endif

/**
 * @brief Default notch filter Q factor
 */
#ifndef CIA402_FILTER_NOTCH_Q
#define CIA402_FILTER_NOTCH_Q               2.0
#endif

// ============================================================================
// HOMING CONFIGURATION
// ============================================================================

/**
 * @brief Default homing velocity (approach speed)
 */
#ifndef CIA402_DEFAULT_HOMING_VELOCITY
#define CIA402_DEFAULT_HOMING_VELOCITY      100.0
#endif

/**
 * @brief Default homing acceleration
 */
#ifndef CIA402_DEFAULT_HOMING_ACCELERATION
#define CIA402_DEFAULT_HOMING_ACCELERATION  1000.0
#endif

/**
 * @brief Default slow homing velocity (final approach)
 */
#ifndef CIA402_DEFAULT_HOMING_VELOCITY_SLOW
#define CIA402_DEFAULT_HOMING_VELOCITY_SLOW 10.0
#endif

/**
 * @brief Default homing offset from switch/index
 */
#ifndef CIA402_DEFAULT_HOMING_OFFSET
#define CIA402_DEFAULT_HOMING_OFFSET        0.0
#endif

/**
 * @brief Homing timeout in milliseconds
 */
#ifndef CIA402_HOMING_TIMEOUT_MS
#define CIA402_HOMING_TIMEOUT_MS            60000
#endif

// ============================================================================
// ELECTRONIC GEARING CONFIGURATION
// ============================================================================

/**
 * @brief Maximum number of geared slaves per master
 */
#ifndef CIA402_MAX_GEARED_SLAVES
#define CIA402_MAX_GEARED_SLAVES            4
#endif

/**
 * @brief Enable soft start for gear engagement
 * 
 * Gradually applies gear ratio to avoid sudden jumps.
 */
#ifndef CIA402_GEARING_SOFT_START
#define CIA402_GEARING_SOFT_START           1
#endif

/**
 * @brief Gear engagement ramp time in milliseconds
 */
#ifndef CIA402_GEARING_RAMP_TIME_MS
#define CIA402_GEARING_RAMP_TIME_MS         500
#endif

// ============================================================================
// PATH INTERPOLATION CONFIGURATION
// ============================================================================

/**
 * @brief Default path velocity for coordinated motion
 */
#ifndef CIA402_DEFAULT_PATH_VELOCITY
#define CIA402_DEFAULT_PATH_VELOCITY        500.0
#endif

/**
 * @brief Default path acceleration
 */
#ifndef CIA402_DEFAULT_PATH_ACCELERATION
#define CIA402_DEFAULT_PATH_ACCELERATION    5000.0
#endif

/**
 * @brief Default corner blending radius
 * 
 * Used for continuous path motion at corners.
 */
#ifndef CIA402_DEFAULT_BLEND_RADIUS
#define CIA402_DEFAULT_BLEND_RADIUS         1.0
#endif

/**
 * @brief Spline interpolation segments per curve
 */
#ifndef CIA402_SPLINE_SEGMENTS
#define CIA402_SPLINE_SEGMENTS              32
#endif

/**
 * @brief Maximum control points for B-spline/NURBS
 */
#ifndef CIA402_MAX_SPLINE_POINTS
#define CIA402_MAX_SPLINE_POINTS            64
#endif

// ============================================================================
// SAFETY AND LIMITS
// ============================================================================

/**
 * @brief Enable software position limits
 */
#ifndef CIA402_SOFTWARE_LIMITS_ENABLED
#define CIA402_SOFTWARE_LIMITS_ENABLED      1
#endif

/**
 * @brief Default following error limit
 * 
 * Maximum allowed position error before fault.
 */
#ifndef CIA402_DEFAULT_FOLLOWING_ERROR
#define CIA402_DEFAULT_FOLLOWING_ERROR      10.0
#endif

/**
 * @brief Quick stop deceleration multiplier
 * 
 * Applied to max deceleration during quick stop.
 */
#ifndef CIA402_QUICKSTOP_DECEL_FACTOR
#define CIA402_QUICKSTOP_DECEL_FACTOR       3.0
#endif

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

/**
 * @brief Motion task priority
 * 
 * Higher priority = lower latency for motion updates.
 */
#ifndef CIA402_MOTION_TASK_PRIORITY
#define CIA402_MOTION_TASK_PRIORITY         10
#endif

/**
 * @brief Motion task stack size
 */
#ifndef CIA402_MOTION_TASK_STACK_SIZE
#define CIA402_MOTION_TASK_STACK_SIZE       8192
#endif

/**
 * @brief Motion task CPU core (ESP32)
 * 
 * -1 = no affinity, 0 = PRO CPU, 1 = APP CPU
 */
#ifndef CIA402_MOTION_TASK_CORE
#define CIA402_MOTION_TASK_CORE             1
#endif

// ============================================================================
// LOGGING
// ============================================================================

/**
 * @brief Log tag for CiA 402 module
 */
#ifndef CIA402_LOG_TAG
#define CIA402_LOG_TAG                      "CIA402"
#endif

/**
 * @brief Enable verbose motion logging
 */
#ifndef CIA402_DEBUG_MOTION
#define CIA402_DEBUG_MOTION                 0
#endif

/**
 * @brief Enable state machine transition logging
 */
#ifndef CIA402_DEBUG_STATE
#define CIA402_DEBUG_STATE                  1
#endif

// ============================================================================
// CiA 402 PROTOCOL CONSTANTS
// ============================================================================

namespace CiA402 {

/**
 * @brief Control word bit definitions (0x6040)
 */
enum class ControlWordBit : uint16_t {
    SwitchOn            = 0x0001,   ///< Bit 0: Switch on
    EnableVoltage       = 0x0002,   ///< Bit 1: Enable voltage
    QuickStop           = 0x0004,   ///< Bit 2: Quick stop (active low!)
    EnableOperation     = 0x0008,   ///< Bit 3: Enable operation
    // Bits 4-6: Operation mode specific
    NewSetpoint         = 0x0010,   ///< Bit 4: New setpoint (PP mode)
    ChangeSetImmed      = 0x0020,   ///< Bit 5: Change set immediately (PP)
    AbsRel              = 0x0040,   ///< Bit 6: Absolute/Relative (PP)
    FaultReset          = 0x0080,   ///< Bit 7: Fault reset
    Halt                = 0x0100,   ///< Bit 8: Halt
    // Bits 9-10: Reserved
    HomingStart         = 0x0010,   ///< Bit 4: Start homing (HM mode)
};

/**
 * @brief Status word bit definitions (0x6041)
 */
enum class StatusWordBit : uint16_t {
    ReadyToSwitchOn     = 0x0001,   ///< Bit 0: Ready to switch on
    SwitchedOn          = 0x0002,   ///< Bit 1: Switched on
    OperationEnabled    = 0x0004,   ///< Bit 2: Operation enabled
    Fault               = 0x0008,   ///< Bit 3: Fault
    VoltageEnabled      = 0x0010,   ///< Bit 4: Voltage enabled
    QuickStop           = 0x0020,   ///< Bit 5: Quick stop (active low!)
    SwitchOnDisabled    = 0x0040,   ///< Bit 6: Switch on disabled
    Warning             = 0x0080,   ///< Bit 7: Warning
    // Bits 8-9: Mode specific
    Remote              = 0x0200,   ///< Bit 9: Remote (drive is in remote control)
    TargetReached       = 0x0400,   ///< Bit 10: Target reached
    InternalLimitActive = 0x0800,   ///< Bit 11: Internal limit active
    // Bits 12-13: Operation mode specific
    HomingAttained      = 0x1000,   ///< Bit 12: Homing attained (HM mode)
    HomingError         = 0x2000,   ///< Bit 13: Homing error (HM mode)
    SetpointAck         = 0x1000,   ///< Bit 12: Setpoint acknowledge (PP)
    FollowingError      = 0x2000,   ///< Bit 13: Following error (PP/PV)
};

/**
 * @brief CiA 402 state machine states
 */
enum class State : uint8_t {
    NotReadyToSwitchOn  = 0,    ///< Initial state after power on
    SwitchOnDisabled    = 1,    ///< High voltage may be present
    ReadyToSwitchOn     = 2,    ///< High voltage present, drive function disabled
    SwitchedOn          = 3,    ///< Drive function disabled, ready
    OperationEnabled    = 4,    ///< Drive function enabled
    QuickStopActive     = 5,    ///< Quick stop in progress
    FaultReactionActive = 6,    ///< Fault reaction in progress
    Fault               = 7,    ///< Fault state
};

/**
 * @brief Get string name for state
 */
inline const char* state_to_string(State state) {
    switch (state) {
        case State::NotReadyToSwitchOn:  return "NotReadyToSwitchOn";
        case State::SwitchOnDisabled:    return "SwitchOnDisabled";
        case State::ReadyToSwitchOn:     return "ReadyToSwitchOn";
        case State::SwitchedOn:          return "SwitchedOn";
        case State::OperationEnabled:    return "OperationEnabled";
        case State::QuickStopActive:     return "QuickStopActive";
        case State::FaultReactionActive: return "FaultReactionActive";
        case State::Fault:               return "Fault";
        default:                         return "Unknown";
    }
}

/**
 * @brief CiA 402 operating modes
 */
enum class OperatingMode : int8_t {
    NoMode              = 0,
    ProfilePosition     = 1,    ///< Profile position mode (PP)
    Velocity            = 2,    ///< Velocity mode (VL)
    ProfileVelocity     = 3,    ///< Profile velocity mode (PV)
    ProfileTorque       = 4,    ///< Profile torque mode (PT)
    Reserved            = 5,
    Homing              = 6,    ///< Homing mode (HM)
    InterpolatedPosition = 7,   ///< Interpolated position mode (IP)
    CyclicSyncPosition  = 8,    ///< Cyclic synchronous position mode (CSP)
    CyclicSyncVelocity  = 9,    ///< Cyclic synchronous velocity mode (CSV)
    CyclicSyncTorque    = 10,   ///< Cyclic synchronous torque mode (CST)
};

/**
 * @brief Get string name for operating mode
 */
inline const char* mode_to_string(OperatingMode mode) {
    switch (mode) {
        case OperatingMode::NoMode:              return "NoMode";
        case OperatingMode::ProfilePosition:     return "ProfilePosition";
        case OperatingMode::Velocity:            return "Velocity";
        case OperatingMode::ProfileVelocity:     return "ProfileVelocity";
        case OperatingMode::ProfileTorque:       return "ProfileTorque";
        case OperatingMode::Homing:              return "Homing";
        case OperatingMode::InterpolatedPosition:return "InterpolatedPosition";
        case OperatingMode::CyclicSyncPosition:  return "CyclicSyncPosition";
        case OperatingMode::CyclicSyncVelocity:  return "CyclicSyncVelocity";
        case OperatingMode::CyclicSyncTorque:    return "CyclicSyncTorque";
        default:                                 return "Unknown";
    }
}

/**
 * @brief Homing methods as per CiA 402
 */
enum class HomingMethod : int8_t {
    NoHoming                            = 0,
    
    // Methods with index pulse
    NegLimitIndex                       = 1,    ///< Negative limit switch & index
    PosLimitIndex                       = 2,    ///< Positive limit switch & index
    HomeSwitchPosIndex                  = 3,    ///< Home switch positive & index
    HomeSwitchPosIndex2                 = 4,    ///< Home switch positive & index (alt)
    HomeSwitchNegIndex                  = 5,    ///< Home switch negative & index
    HomeSwitchNegIndex2                 = 6,    ///< Home switch negative & index (alt)
    HomeSwitchPosIndexPos               = 7,    ///< Home switch pos, positive dir & index
    HomeSwitchPosIndexPos2              = 8,
    HomeSwitchPosIndexNeg               = 9,
    HomeSwitchPosIndexNeg2              = 10,
    HomeSwitchNegIndexPos               = 11,
    HomeSwitchNegIndexPos2              = 12,
    HomeSwitchNegIndexNeg               = 13,
    HomeSwitchNegIndexNeg2              = 14,
    
    // Methods without index pulse
    NegLimitOnly                        = 17,   ///< Negative limit switch only
    PosLimitOnly                        = 18,   ///< Positive limit switch only
    HomeSwitchPos                       = 19,   ///< Home switch positive only
    HomeSwitchPos2                      = 20,
    HomeSwitchNeg                       = 21,   ///< Home switch negative only
    HomeSwitchNeg2                      = 22,
    HomeSwitchPosPos                    = 23,
    HomeSwitchPosPos2                   = 24,
    HomeSwitchPosNeg                    = 25,
    HomeSwitchPosNeg2                   = 26,
    HomeSwitchNegPos                    = 27,
    HomeSwitchNegPos2                   = 28,
    HomeSwitchNegNeg                    = 29,
    HomeSwitchNegNeg2                   = 30,
    
    // Index pulse only
    NegDirIndexPulse                    = 33,   ///< Negative direction, index pulse
    PosDirIndexPulse                    = 34,   ///< Positive direction, index pulse
    
    // Current position
    CurrentPosition                     = 35,   ///< Set current position as home
    CurrentPositionIndex                = 37,   ///< Current position + index
};

/**
 * @brief Standard CiA 402 object dictionary indices
 */
enum class ObjIndex : uint16_t {
    // Device control
    ControlWord         = 0x6040,
    StatusWord          = 0x6041,
    QuickStopOption     = 0x605A,
    ShutdownOption      = 0x605B,
    DisableOperOption   = 0x605C,
    HaltOption          = 0x605D,
    FaultReactionOption = 0x605E,
    ModesOfOperation    = 0x6060,
    ModesOfOpDisplay    = 0x6061,
    
    // Position
    PositionActual      = 0x6064,
    VelocityActual      = 0x606C,
    TorqueActual        = 0x6077,
    TargetPosition      = 0x607A,
    PositionRangeLimit  = 0x607B,
    SoftwarePosLimit    = 0x607D,
    MaxProfileVelocity  = 0x607F,
    ProfileVelocity     = 0x6081,
    ProfileAcceleration = 0x6083,
    ProfileDeceleration = 0x6084,
    QuickStopDecel      = 0x6085,
    MotionProfileType   = 0x6086,
    
    // Velocity mode
    TargetVelocity      = 0x60FF,
    VelocityDemand      = 0x606B,
    VelocityOffset      = 0x60B1,
    
    // Torque mode
    TargetTorque        = 0x6071,
    TorqueOffset        = 0x60B2,
    MaxTorque           = 0x6072,
    
    // Homing
    HomingMethod        = 0x6098,
    HomingSpeed         = 0x6099,  // Subindex 1: fast, 2: slow
    HomingAcceleration  = 0x609A,
    HomeOffset          = 0x607C,
    
    // Factor group
    PositionFactor      = 0x6093,
    VelocityFactor      = 0x6094,
    AccelerationFactor  = 0x6097,
    
    // Following error
    FollowingErrorWindow= 0x6065,
    FollowingErrorTimeout = 0x6066,
    PositionWindow      = 0x6067,
    PositionWindowTime  = 0x6068,
    
    // Interpolation
    InterpolationMode   = 0x60C0,
    InterpolationData   = 0x60C1,
    InterpolationTimePeriod = 0x60C2,
    
    // Cyclic sync
    PositionOffset      = 0x60B0,
    TouchProbeFunction  = 0x60B8,
    TouchProbeStatus    = 0x60B9,
    TouchProbePos1Pos   = 0x60BA,
    TouchProbePos1Neg   = 0x60BB,
    TouchProbePos2Pos   = 0x60BC,
    TouchProbePos2Neg   = 0x60BD,
    
    // Digital I/O
    DigitalInputs       = 0x60FD,
    DigitalOutputs      = 0x60FE,
};

/**
 * @brief Error codes
 */
enum class ErrorCode : uint32_t {
    None                = 0x0000,
    GenericError        = 0x1000,
    OverCurrent         = 0x2310,
    OverVoltage         = 0x3210,
    UnderVoltage        = 0x3220,
    OverTemperature     = 0x4210,
    SupplyTemp          = 0x4310,
    EncoderError        = 0x5110,
    MotorBlocked        = 0x5441,
    FollowingError      = 0x8611,
    PositionLimit       = 0x8612,
    VelocityLimit       = 0x8613,
    CommunicationError  = 0x8100,
    HomingError         = 0x8620,
};

} // namespace CiA402
