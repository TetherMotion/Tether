/**
 * @file MotorModel.hpp
 * @brief Physical Motor Model for Virtual CiA 402 Drives
 *
 * Provides a comprehensive physics-based motor model including:
 * - Rotating inertial mass dynamics
 * - Torque-based motion with friction and damping
 * - Dual-loop PID control (position + velocity)
 * - Single-loop PID for velocity mode
 * - Configurable motor parameters (inertia, friction, limits)
 * - Endstop simulation with configurable behavior
 * - Braking resistor and regenerative braking
 * - Thermal modeling
 * - Encoder simulation with index pulse
 * - All CiA 402 homing modes
 *
 * ## Physical Model
 *
 * The motor model implements the following dynamics:
 *
 * ```
 * τ_net = τ_cmd - τ_friction - τ_damping - τ_load
 * 
 * α = τ_net / J                    (angular acceleration)
 * ω = ω_prev + α * dt              (angular velocity)
 * θ = θ_prev + ω * dt              (angular position)
 *
 * Where:
 *   τ_cmd     = commanded torque (from controller)
 *   τ_friction = μ * sign(ω)       (Coulomb friction)
 *   τ_damping  = B * ω             (viscous damping)
 *   τ_load    = external load torque
 *   J         = moment of inertia
 *   α         = angular acceleration
 *   ω         = angular velocity
 *   θ         = angular position
 * ```
 *
 * ## Control Architecture
 *
 * ```
 *                    Position Mode (Dual Loop)
 *     ┌─────────────────────────────────────────────────────────┐
 *     │                                                         │
 *     │  ┌─────────┐      ┌─────────┐      ┌─────────┐         │
 *     │  │Position │  vel │Velocity │ torq │ Motor   │  pos    │
 * ref─┼─►│   PID   │─────►│   PID   │─────►│ Physics │────┬────┼──►
 *     │  └────┬────┘      └────┬────┘      └─────────┘    │    │
 *     │       │                │                          │    │
 *     │       └────────────────┴──────────────────────────┘    │
 *     │                    (feedback)                          │
 *     └─────────────────────────────────────────────────────────┘
 *
 *                    Velocity Mode (Single Loop)
 *     ┌─────────────────────────────────────────────────────────┐
 *     │                                                         │
 *     │              ┌─────────┐      ┌─────────┐               │
 *     │          vel │Velocity │ torq │ Motor   │  vel          │
 * ref─┼─────────────►│   PID   │─────►│ Physics │────┬──────────┼──►
 *     │              └────┬────┘      └─────────┘    │          │
 *     │                   │                          │          │
 *     │                   └──────────────────────────┘          │
 *     │                        (feedback)                       │
 *     └─────────────────────────────────────────────────────────┘
 * ```
 */

#pragma once

#include "control/PIDControllers.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

namespace CiA402 {
namespace Motor {

// ============================================================================
// Motor Parameters
// ============================================================================

/**
 * @brief Physical motor parameters
 */
struct MotorParams {
    // Mechanical parameters
    double inertia = 0.001;              ///< Moment of inertia (kg·m²)
    double viscousDamping = 0.01;        ///< Viscous damping coefficient (N·m·s/rad)
    double coulombFriction = 0.1;        ///< Coulomb friction torque (N·m)
    double staticFriction = 0.15;        ///< Static friction torque (N·m)
    double stictionVelocity = 0.01;      ///< Velocity threshold for stiction (rad/s)
    
    // Electrical parameters (for torque calculation)
    double torqueConstant = 1.0;         ///< Torque constant (N·m/A)
    double backEMFConstant = 1.0;        ///< Back-EMF constant (V·s/rad)
    double windingResistance = 1.0;      ///< Winding resistance (Ω)
    double windingInductance = 0.001;    ///< Winding inductance (H)
    double ratedCurrent = 10.0;          ///< Rated current (A)
    double peakCurrent = 20.0;           ///< Peak current (A)
    
    // Limits
    double maxTorque = 10.0;             ///< Maximum torque (N·m)
    double maxVelocity = 100.0;          ///< Maximum velocity (rad/s)
    double maxAcceleration = 1000.0;     ///< Maximum acceleration (rad/s²)
    double maxPosition = 1e9;            ///< Maximum position (rad or counts)
    double minPosition = -1e9;           ///< Minimum position (rad or counts)
    
    // Encoder
    uint32_t encoderResolution = 131072; ///< Encoder counts per revolution
    bool hasIndexPulse = true;           ///< Encoder has index pulse
    int32_t indexPulsePosition = 0;      ///< Index pulse position (counts)
    
    // Gearing
    double gearRatio = 1.0;              ///< Gear ratio (motor:load)
    double gearEfficiency = 0.95;        ///< Gear efficiency
    double gearBacklash = 0.0;           ///< Gear backlash (rad)
};

/**
 * @brief Thermal parameters
 */
struct ThermalParams {
    double ambientTemp = 25.0;           ///< Ambient temperature (°C)
    double thermalResistance = 1.0;      ///< Thermal resistance (°C/W)
    double thermalCapacity = 100.0;      ///< Thermal capacity (J/°C)
    double maxWindingTemp = 150.0;       ///< Max winding temperature (°C)
    double maxMotorTemp = 100.0;         ///< Max motor case temperature (°C)
    bool enableThermalModel = true;      ///< Enable thermal simulation
};

/**
 * @brief Braking resistor parameters
 */
struct BrakingResistorParams {
    bool enabled = true;                 ///< Braking resistor installed
    double resistance = 10.0;            ///< Resistance (Ω)
    double maxPower = 100.0;             ///< Max power dissipation (W)
    double thermalTimeConstant = 10.0;   ///< Thermal time constant (s)
    double maxTemperature = 200.0;       ///< Max temperature (°C)
    double activationVoltage = 400.0;    ///< DC bus voltage for activation (V)
    bool enableOverheatProtection = true;
};

/**
 * @brief Endstop configuration
 */
struct EndstopConfig {
    // Positive limit
    bool positiveEnabled = true;
    double positivePosition = 1e9;       ///< Position (counts or rad)
    bool positiveNC = true;              ///< Normally closed switch
    bool positiveFault = false;          ///< Simulate disconnected
    
    // Negative limit
    bool negativeEnabled = true;
    double negativePosition = -1e9;
    bool negativeNC = true;
    bool negativeFault = false;
    
    // Home switch
    bool homeEnabled = true;
    double homePosition = 0.0;
    double homeWidth = 100.0;            ///< Width of home switch active zone
    bool homeNC = false;                 ///< Normally open
    bool homeFault = false;              ///< Simulate disconnected
    
    // Index pulse
    bool indexEnabled = true;
    double indexPosition = 0.0;
    bool indexFault = false;
    
    // Behavior
    double switchDebounceMs = 1.0;       ///< Switch debounce time
    double hardStopStiffness = 1e6;      ///< Stiffness at hard stop (N/m)
};

// ============================================================================
// Controller Parameters
// ============================================================================

/**
 * @brief Position controller parameters
 */
struct PositionControllerParams {
    // Outer loop (position)
    double posKp = 100.0;                ///< Position proportional gain
    double posKi = 0.0;                  ///< Position integral gain
    double posKd = 10.0;                 ///< Position derivative gain
    double posDerivativeFilter = 0.01;   ///< Derivative filter time constant (s)
    double posIntegralLimit = 1000.0;    ///< Integral windup limit
    
    // Inner loop (velocity)
    double velKp = 10.0;                 ///< Velocity proportional gain
    double velKi = 1.0;                  ///< Velocity integral gain
    double velKd = 0.0;                  ///< Velocity derivative gain
    double velDerivativeFilter = 0.001;  ///< Derivative filter time constant (s)
    double velIntegralLimit = 100.0;     ///< Integral windup limit
    
    // Feed-forward
    double velocityFeedforward = 0.0;    ///< Velocity feed-forward gain
    double accelerationFeedforward = 0.0; ///< Acceleration feed-forward gain
    double torqueFeedforward = 0.0;      ///< Torque feed-forward gain
    
    // Output limits
    double maxVelocityCmd = 100.0;       ///< Max velocity command from position loop
    double maxTorqueCmd = 10.0;          ///< Max torque command from velocity loop
};

/**
 * @brief Velocity controller parameters
 */
struct VelocityControllerParams {
    double kp = 10.0;                    ///< Proportional gain
    double ki = 1.0;                     ///< Integral gain
    double kd = 0.0;                     ///< Derivative gain
    double derivativeFilter = 0.001;     ///< Derivative filter time constant
    double integralLimit = 100.0;        ///< Integral windup limit
    double torqueFeedforward = 0.0;      ///< Torque feed-forward
    double maxTorqueCmd = 10.0;          ///< Max torque command
};

/**
 * @brief Torque controller parameters
 */
struct TorqueControllerParams {
    double maxTorque = 10.0;             ///< Maximum torque
    double torqueRampRate = 1000.0;      ///< Torque ramp rate (N·m/s)
    bool enableCurrentLimit = true;
};

// ============================================================================
// Motor State
// ============================================================================

/**
 * @brief Current motor state
 */
struct MotorState {
    // Position and velocity
    double position = 0.0;               ///< Current position (rad)
    double velocity = 0.0;               ///< Current velocity (rad/s)
    double acceleration = 0.0;           ///< Current acceleration (rad/s²)
    
    // Torques
    double commandedTorque = 0.0;        ///< Commanded torque (N·m)
    double actualTorque = 0.0;           ///< Actual torque (N·m)
    double loadTorque = 0.0;             ///< External load torque (N·m)
    double frictionTorque = 0.0;         ///< Friction torque (N·m)
    
    // Electrical
    double current = 0.0;                ///< Motor current (A)
    double voltage = 0.0;                ///< Motor voltage (V)
    double backEMF = 0.0;                ///< Back-EMF voltage (V)
    double dcBusVoltage = 0.0;           ///< DC bus voltage (V)
    
    // Thermal
    double windingTemperature = 25.0;    ///< Winding temperature (°C)
    double motorTemperature = 25.0;      ///< Motor case temperature (°C)
    double brakingResistorTemp = 25.0;   ///< Braking resistor temperature (°C)
    double powerDissipation = 0.0;       ///< Power dissipation (W)
    
    // Encoder
    int32_t encoderPosition = 0;         ///< Encoder position (counts)
    int32_t encoderVelocity = 0;         ///< Encoder velocity (counts/s)
    bool indexPulseDetected = false;     ///< Index pulse detected this cycle
    int32_t lastIndexPosition = 0;       ///< Last index pulse position
    
    // Limits
    bool positiveLimit = false;          ///< At positive limit
    bool negativeLimit = false;          ///< At negative limit
    bool homeSwitch = false;             ///< Home switch active
    bool indexPulse = false;             ///< Index pulse active
    
    // Faults
    bool overTorqueFault = false;
    bool overSpeedFault = false;
    bool overTempFault = false;
    bool followingErrorFault = false;
    bool encoderFault = false;
    bool brakingResistorFault = false;
    
    void reset() {
        *this = MotorState{};
    }
};

// ============================================================================
// Error Injection
// ============================================================================

/**
 * @brief Motor model error injection for testing
 */
struct MotorErrorInjection {
    bool enabled = false;
    
    // Endstop faults
    bool simulateEndstopDisconnect = false;
    bool simulateHomeDisconnect = false;
    bool simulateIndexDisconnect = false;
    
    // Electrical faults
    bool simulateOvercurrent = false;
    bool simulateOvervoltage = false;
    bool simulateUndervoltage = false;
    bool simulatePhaseOpen = false;
    bool simulateShortCircuit = false;
    
    // Mechanical faults
    bool simulateJam = false;              ///< Axis jammed
    bool simulateSlip = false;             ///< Coupling slip
    bool simulateBrokenBelt = false;
    bool simulateEncoderFault = false;
    
    // Thermal faults
    bool simulateOverheat = false;
    bool simulateBrakingResistorOverheat = false;
    
    // Timing faults
    bool simulateEncoderNoise = false;
    double encoderNoiseAmplitude = 0.0;    ///< Noise amplitude (counts)
    
    // Load disturbances
    bool injectLoadTorque = false;
    double injectedLoadTorque = 0.0;
    bool injectPositionStep = false;
    double positionStepAmount = 0.0;
    
    void reset() {
        enabled = false;
        simulateEndstopDisconnect = false;
        simulateHomeDisconnect = false;
        simulateIndexDisconnect = false;
        simulateOvercurrent = false;
        simulateOvervoltage = false;
        simulateUndervoltage = false;
        simulatePhaseOpen = false;
        simulateShortCircuit = false;
        simulateJam = false;
        simulateSlip = false;
        simulateBrokenBelt = false;
        simulateEncoderFault = false;
        simulateOverheat = false;
        simulateBrakingResistorOverheat = false;
        simulateEncoderNoise = false;
        injectLoadTorque = false;
        injectPositionStep = false;
    }
};

// ============================================================================
// Callback Types
// ============================================================================

using FaultCallback = std::function<void(uint16_t faultCode)>;
using LimitCallback = std::function<void(bool positive, bool negative)>;
using HomeSwitchCallback = std::function<void(bool active)>;
using IndexPulseCallback = std::function<void(int32_t position)>;

// ============================================================================
// Motor Model Class
// ============================================================================

/**
 * @brief Physical Motor Model
 *
 * Simulates a complete servo motor with:
 * - Physics-based dynamics (inertia, friction, damping)
 * - Dual-loop position control
 * - Single-loop velocity control
 * - Torque mode
 * - Thermal modeling
 * - Braking resistor
 * - Endstops and limits
 */
class MotorModel {
public:
    MotorModel();
    explicit MotorModel(const MotorParams& params);
    ~MotorModel() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setMotorParams(const MotorParams& params);
    const MotorParams& getMotorParams() const { return motorParams_; }
    
    void setThermalParams(const ThermalParams& params);
    const ThermalParams& getThermalParams() const { return thermalParams_; }
    
    void setBrakingResistorParams(const BrakingResistorParams& params);
    const BrakingResistorParams& getBrakingResistorParams() const { return brakingParams_; }
    
    void setEndstopConfig(const EndstopConfig& config);
    const EndstopConfig& getEndstopConfig() const { return endstopConfig_; }
    
    void setPositionControllerParams(const PositionControllerParams& params);
    const PositionControllerParams& getPositionControllerParams() const { return posCtrlParams_; }
    
    void setVelocityControllerParams(const VelocityControllerParams& params);
    const VelocityControllerParams& getVelocityControllerParams() const { return velCtrlParams_; }
    
    void setTorqueControllerParams(const TorqueControllerParams& params);
    const TorqueControllerParams& getTorqueControllerParams() const { return torqueCtrlParams_; }
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    /**
     * @brief Initialize motor model
     * @return true on success
     */
    bool initialize();
    
    /**
     * @brief Reset motor model to initial state
     */
    void reset();
    
    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return initialized_; }
    
    // ========================================================================
    // Control Modes
    // ========================================================================
    
    enum class ControlMode {
        Disabled,
        Position,
        Velocity,
        Torque
    };
    
    /**
     * @brief Set control mode
     */
    void setControlMode(ControlMode mode);
    
    /**
     * @brief Get control mode
     */
    ControlMode getControlMode() const { return controlMode_; }
    
    // ========================================================================
    // Setpoints
    // ========================================================================
    
    /**
     * @brief Set position setpoint (for position mode)
     * @param position Target position (counts)
     */
    void setTargetPosition(int32_t position);
    
    /**
     * @brief Set velocity setpoint (for velocity mode)
     * @param velocity Target velocity (counts/s)
     */
    void setTargetVelocity(int32_t velocity);
    
    /**
     * @brief Set torque setpoint (for torque mode)
     * @param torque Target torque (0.1% of rated torque)
     */
    void setTargetTorque(int16_t torque);
    
    /**
     * @brief Set external load torque
     */
    void setLoadTorque(double torque);
    
    // ========================================================================
    // State Access
    // ========================================================================
    
    const MotorState& getState() const { return state_; }
    MotorState& getState() { return state_; }
    
    /**
     * @brief Get position in encoder counts
     */
    int32_t getActualPosition() const { return state_.encoderPosition; }
    
    /**
     * @brief Get velocity in encoder counts/s
     */
    int32_t getActualVelocity() const { return state_.encoderVelocity; }
    
    /**
     * @brief Get torque in 0.1% of rated
     */
    int16_t getActualTorque() const;
    
    /**
     * @brief Get following error (target - actual position)
     */
    int32_t getFollowingError() const;
    
    /**
     * @brief Check if at positive limit
     */
    bool atPositiveLimit() const { return state_.positiveLimit; }
    
    /**
     * @brief Check if at negative limit
     */
    bool atNegativeLimit() const { return state_.negativeLimit; }
    
    /**
     * @brief Check if home switch active
     */
    bool isHomeSwitchActive() const { return state_.homeSwitch; }
    
    /**
     * @brief Check if index pulse detected
     */
    bool isIndexPulseDetected() const { return state_.indexPulseDetected; }
    
    /**
     * @brief Check if any fault active
     */
    bool hasFault() const;
    
    // ========================================================================
    // Simulation
    // ========================================================================
    
    /**
     * @brief Update motor model
     * @param dt Time step in seconds
     */
    void update(double dt);
    
    /**
     * @brief Run control loop
     * @param dt Time step in seconds
     * @return Commanded torque
     */
    double runControlLoop(double dt);
    
    /**
     * @brief Run physics simulation
     * @param commandedTorque Torque command (N·m)
     * @param dt Time step in seconds
     */
    void runPhysics(double commandedTorque, double dt);
    
    /**
     * @brief Update thermal model
     * @param dt Time step in seconds
     */
    void updateThermal(double dt);
    
    /**
     * @brief Update endstops
     */
    void updateEndstops();
    
    /**
     * @brief Update encoder
     */
    void updateEncoder();
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setFaultCallback(FaultCallback callback) { faultCallback_ = callback; }
    void setLimitCallback(LimitCallback callback) { limitCallback_ = callback; }
    void setHomeSwitchCallback(HomeSwitchCallback callback) { homeSwitchCallback_ = callback; }
    void setIndexPulseCallback(IndexPulseCallback callback) { indexPulseCallback_ = callback; }
    
    // ========================================================================
    // Error Injection
    // ========================================================================
    
    MotorErrorInjection& getErrorInjection() { return errorInjection_; }
    const MotorErrorInjection& getErrorInjection() const { return errorInjection_; }

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    double calculateFriction(double velocity);
    double calculateBackEMF(double velocity);
    double limitTorque(double torque);
    void checkLimits();
    void checkFaults();
    void triggerFault(uint16_t faultCode);
    
    // Position to radians conversion
    double countsToRadians(int32_t counts) const;
    int32_t radiansToCounts(double radians) const;
    
    // ========================================================================
    // Parameters
    // ========================================================================
    
    MotorParams motorParams_;
    ThermalParams thermalParams_;
    BrakingResistorParams brakingParams_;
    EndstopConfig endstopConfig_;
    PositionControllerParams posCtrlParams_;
    VelocityControllerParams velCtrlParams_;
    TorqueControllerParams torqueCtrlParams_;
    
    // ========================================================================
    // State
    // ========================================================================
    
    bool initialized_ = false;
    ControlMode controlMode_ = ControlMode::Disabled;
    MotorState state_;
    
    // Setpoints
    int32_t targetPosition_ = 0;
    int32_t targetVelocity_ = 0;
    int16_t targetTorque_ = 0;
    
    // ========================================================================
    // Controllers
    // ========================================================================
    
    // Position loop (dual loop)
    std::unique_ptr<Control::PIDController> positionPID_;
    std::unique_ptr<Control::PIDController> velocityPID_;
    
    // Velocity loop (single loop)
    std::unique_ptr<Control::PIDController> velocityOnlyPID_;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    FaultCallback faultCallback_;
    LimitCallback limitCallback_;
    HomeSwitchCallback homeSwitchCallback_;
    IndexPulseCallback indexPulseCallback_;
    
    // ========================================================================
    // Error Injection
    // ========================================================================
    
    MotorErrorInjection errorInjection_;
    
    // ========================================================================
    // Internal State
    // ========================================================================
    
    double prevPosition_ = 0.0;
    double prevVelocity_ = 0.0;
    bool wasAtPositiveLimit_ = false;
    bool wasAtNegativeLimit_ = false;
    bool wasHomeActive_ = false;
    
    mutable std::recursive_mutex mutex_;
};

} // namespace Motor
} // namespace CiA402
