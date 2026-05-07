/**
 * @file AdvancedMotorModel.hpp
 * @brief Advanced Physical Motor Model with Geartrain and Load Dynamics
 * 
 * This model implements:
 * - Motor with torque-speed characteristic curve
 * - Geartrain with backlash, inertia, and friction
 * - Output load with configurable inertia and friction
 * - Full friction models (static, kinetic, stiction, viscous)
 * - Backdrivability configuration
 * - Thermal modeling
 * 
 * The kinematic chain is:
 *   Motor -> Backlash -> Geartrain -> Load
 * 
 * Each component has its own inertia and friction characteristics.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <cmath>

namespace CiA402 {
namespace Motor {

// ============================================================================
// Friction Model
// ============================================================================

/**
 * @brief Comprehensive friction model parameters
 * 
 * Models the following friction components:
 * - Static friction (breakaway torque)
 * - Coulomb friction (constant kinetic friction)
 * - Viscous friction (velocity-proportional)
 * - Stribeck effect (friction reduction at low velocities)
 * 
 * Friction torque = sign(v) * [Fc + (Fs - Fc) * exp(-(v/vs)^2)] + Fv * v
 * 
 * Where:
 *   Fc = Coulomb friction
 *   Fs = Static friction  
 *   vs = Stribeck velocity
 *   Fv = Viscous coefficient
 */
struct FrictionParams {
    double staticFriction = 0.1;      ///< Static friction torque [Nm] (breakaway)
    double coulombFriction = 0.05;    ///< Coulomb (kinetic) friction torque [Nm]
    double viscousCoeff = 0.01;       ///< Viscous friction coefficient [Nm/(rad/s)]
    double stribeckVelocity = 0.1;    ///< Stribeck velocity [rad/s]
    double stictionVelocity = 0.001;  ///< Velocity threshold for stiction [rad/s]
    
    /**
     * @brief Calculate friction torque
     * @param velocity Current velocity [rad/s]
     * @param appliedTorque Applied torque (for stiction calculation) [Nm]
     * @return Friction torque [Nm]
     */
    double calculate(double velocity, double appliedTorque) const;
    
    /**
     * @brief Check if in stiction (stuck) state
     */
    bool isInStiction(double velocity) const {
        return std::abs(velocity) < stictionVelocity;
    }
};

// ============================================================================
// Motor Parameters
// ============================================================================

/**
 * @brief Torque-speed curve parameters
 * 
 * Models the motor's torque capability as a function of speed:
 * - Constant torque region (0 to corner speed)
 * - Constant power region (corner speed to max speed)
 * - Peak torque capability (short-term)
 * 
 * The available torque decreases above corner speed:
 *   T_available = T_rated * (cornerSpeed / speed) for speed > cornerSpeed
 */
struct TorqueSpeedCurve {
    double ratedTorque = 5.0;         ///< Continuous rated torque [Nm]
    double peakTorque = 15.0;         ///< Peak torque (short duration) [Nm]
    double stallTorque = 6.0;         ///< Stall torque at zero speed [Nm]
    double cornerSpeed = 300.0;       ///< Corner speed (start of field weakening) [rad/s]
    double maxSpeed = 600.0;          ///< Maximum speed [rad/s]
    double peakTorqueDuration = 2.0;  ///< Maximum peak torque duration [s]
    
    /**
     * @brief Get available torque at given speed
     * @param speed Current motor speed [rad/s]
     * @param isPeak Whether to use peak or continuous rating
     * @return Maximum available torque [Nm]
     */
    double getAvailableTorque(double speed, bool isPeak = false) const;
};

/**
 * @brief Motor electrical parameters
 */
struct MotorElectricalParams {
    double torqueConstant = 0.1;      ///< Torque constant Kt [Nm/A]
    double backEMFConstant = 0.1;     ///< Back-EMF constant Ke [V/(rad/s)]
    double windingResistance = 1.0;   ///< Winding resistance [Ohm]
    double windingInductance = 0.005; ///< Winding inductance [H]
    double ratedCurrent = 5.0;        ///< Rated continuous current [A]
    double peakCurrent = 15.0;        ///< Peak current [A]
    double supplyVoltage = 48.0;      ///< DC bus voltage [V]
};

/**
 * @brief Motor mechanical parameters
 */
struct MotorMechanicalParams {
    double rotorInertia = 0.001;      ///< Rotor inertia [kg·m²]
    FrictionParams friction;          ///< Motor bearing friction
    int32_t encoderResolution = 131072; ///< Encoder counts per revolution
    double maxAcceleration = 10000.0; ///< Maximum angular acceleration [rad/s²]
};

/**
 * @brief Complete motor parameters
 */
struct MotorParams {
    TorqueSpeedCurve torqueSpeed;
    MotorElectricalParams electrical;
    MotorMechanicalParams mechanical;
};

// ============================================================================
// Backlash Model
// ============================================================================

/**
 * @brief Backlash model between motor and geartrain
 * 
 * Models mechanical play (dead zone) in the coupling:
 * - Angular dead zone where no torque is transmitted
 * - Viscous damping within the backlash zone
 * - Contact stiffness when engaged
 * 
 * States:
 * - Contact positive: motor ahead of gear, transmitting positive torque
 * - Contact negative: motor behind gear, transmitting negative torque  
 * - In backlash: no contact, no torque transmission
 */
struct BacklashParams {
    double totalBacklash = 0.001;     ///< Total backlash angle [rad] (half on each side)
    double contactStiffness = 10000.0; ///< Stiffness when in contact [Nm/rad]
    double contactDamping = 10.0;     ///< Damping when in contact [Nm/(rad/s)]
    double backlashViscosity = 0.001; ///< Viscous damping in backlash zone [Nm/(rad/s)]
    bool enabled = true;              ///< Enable backlash modeling
};

/**
 * @brief Backlash state
 */
enum class BacklashState {
    ContactPositive,  ///< Motor driving forward
    InBacklash,       ///< In dead zone
    ContactNegative   ///< Motor driving backward
};

// ============================================================================
// Geartrain Parameters
// ============================================================================

/**
 * @brief Geartrain parameters
 * 
 * Models a gear reduction with:
 * - Gear ratio (input:output)
 * - Efficiency (forward and backward)
 * - Reflected inertia
 * - Internal friction
 */
struct GeartrainParams {
    double gearRatio = 10.0;          ///< Gear ratio (motor:output, >1 is reduction)
    double forwardEfficiency = 0.90;  ///< Forward efficiency (motor->load) [0-1]
    double backwardEfficiency = 0.85; ///< Backward efficiency (load->motor) [0-1]
    double gearInertia = 0.0001;      ///< Geartrain internal inertia [kg·m²] (at output)
    FrictionParams friction;          ///< Geartrain internal friction (at output)
    bool backdrivable = true;         ///< Whether load can drive motor
    double backdriveTorqueThreshold = 0.5; ///< Min torque to backdrive [Nm] (if not backdrivable)
};

// ============================================================================
// Load Parameters
// ============================================================================

/**
 * @brief Output load parameters
 */
struct LoadParams {
    double inertia = 0.01;            ///< Load inertia [kg·m²]
    FrictionParams friction;          ///< Load friction
    double externalTorque = 0.0;      ///< External applied torque [Nm] (gravity, process forces)
    double positionMin = -1e9;        ///< Minimum position [rad]
    double positionMax = 1e9;         ///< Maximum position [rad]
    double hardStopStiffness = 100000.0; ///< Hard stop spring constant [Nm/rad]
    double hardStopDamping = 1000.0;  ///< Hard stop damping [Nm/(rad/s)]
};

// ============================================================================
// Thermal Parameters
// ============================================================================

/**
 * @brief Thermal model parameters
 */
struct ThermalParams {
    bool enabled = false;             ///< Enable thermal simulation
    double motorThermalMass = 100.0;  ///< Motor thermal mass [J/K]
    double motorThermalResistance = 5.0; ///< Motor to ambient [K/W]
    double windingThermalMass = 10.0; ///< Winding thermal mass [J/K]
    double windingToMotorResistance = 1.0; ///< Winding to motor case [K/W]
    double ambientTemperature = 25.0; ///< Ambient temperature [°C]
    double maxWindingTemp = 120.0;    ///< Maximum winding temperature [°C]
    double maxMotorTemp = 80.0;       ///< Maximum motor case temperature [°C]
    double gearboxThermalMass = 50.0; ///< Gearbox thermal mass [J/K]
    double gearboxThermalResistance = 10.0; ///< Gearbox to ambient [K/W]
    double maxGearboxTemp = 80.0;     ///< Maximum gearbox temperature [°C]
};

// ============================================================================
// Endstop/Sensor Configuration
// ============================================================================

/**
 * @brief Endstop and sensor configuration
 */
struct SensorConfig {
    // Limit switches (at output)
    bool positiveLimitEnabled = false;
    double positiveLimitPosition = 1e9;  ///< [rad] at output
    bool positiveLimitNC = false;        ///< Normally closed
    
    bool negativeLimitEnabled = false;
    double negativeLimitPosition = -1e9; ///< [rad] at output
    bool negativeLimitNC = false;
    
    // Home switch
    bool homeEnabled = false;
    double homePosition = 0.0;           ///< [rad] at output
    double homeWidth = 0.01;             ///< Width of home zone [rad]
    bool homeNC = false;
    
    // Index pulse (at motor)
    bool indexEnabled = false;
    double indexPosition = 0.0;          ///< [rad] at motor
};

// ============================================================================
// System State
// ============================================================================

/**
 * @brief Motor state
 */
struct MotorState {
    double position = 0.0;            ///< Motor position [rad]
    double velocity = 0.0;            ///< Motor velocity [rad/s]
    double acceleration = 0.0;        ///< Motor acceleration [rad/s²]
    double torque = 0.0;              ///< Applied torque [Nm]
    double current = 0.0;             ///< Motor current [A]
    double backEMF = 0.0;             ///< Back-EMF voltage [V]
    double powerDissipation = 0.0;    ///< I²R losses [W]
    double frictionTorque = 0.0;      ///< Friction torque [Nm]
};

/**
 * @brief Geartrain state
 */
struct GeartrainState {
    double inputPosition = 0.0;       ///< Input (motor side) position [rad]
    double outputPosition = 0.0;      ///< Output (load side) position [rad]
    double inputVelocity = 0.0;       ///< Input velocity [rad/s]
    double outputVelocity = 0.0;      ///< Output velocity [rad/s]
    double transmittedTorque = 0.0;   ///< Torque transmitted through gear [Nm]
    double frictionTorque = 0.0;      ///< Internal friction torque [Nm]
    double efficiency = 0.0;          ///< Current efficiency
    BacklashState backlashState = BacklashState::InBacklash;
    double backlashAngle = 0.0;       ///< Current position within backlash [rad]
};

/**
 * @brief Load state
 */
struct LoadState {
    double position = 0.0;            ///< Load position [rad]
    double velocity = 0.0;            ///< Load velocity [rad/s]
    double acceleration = 0.0;        ///< Load acceleration [rad/s²]
    double appliedTorque = 0.0;       ///< Torque from geartrain [Nm]
    double frictionTorque = 0.0;      ///< Load friction [Nm]
    double externalTorque = 0.0;      ///< External forces [Nm]
    bool atPositiveLimit = false;     ///< At positive hard stop
    bool atNegativeLimit = false;     ///< At negative hard stop
};

/**
 * @brief Thermal state
 */
struct ThermalState {
    double windingTemp = 25.0;        ///< Winding temperature [°C]
    double motorCaseTemp = 25.0;      ///< Motor case temperature [°C]
    double gearboxTemp = 25.0;        ///< Gearbox temperature [°C]
    bool windingOvertemp = false;
    bool motorOvertemp = false;
    bool gearboxOvertemp = false;
};

/**
 * @brief Sensor state
 */
struct SensorState {
    bool positiveLimitActive = false;
    bool negativeLimitActive = false;
    bool homeActive = false;
    bool indexPulse = false;
    int32_t encoderPosition = 0;      ///< Motor encoder [counts]
    int32_t outputPosition = 0;       ///< Output encoder (if present) [counts]
};

/**
 * @brief Complete system state
 */
struct SystemState {
    MotorState motor;
    GeartrainState geartrain;
    LoadState load;
    ThermalState thermal;
    SensorState sensors;
    
    double simulationTime = 0.0;      ///< Total simulation time [s]
    bool hasFault = false;
    uint16_t faultCode = 0;
    
    void reset() {
        motor = MotorState();
        geartrain = GeartrainState();
        load = LoadState();
        thermal = ThermalState();
        sensors = SensorState();
        simulationTime = 0.0;
        hasFault = false;
        faultCode = 0;
    }
};

// ============================================================================
// Error Injection
// ============================================================================

/**
 * @brief Error injection for testing
 */
struct ErrorInjection {
    bool enabled = false;
    
    // Motor faults
    bool simulateMotorOverheat = false;
    bool simulateWindingOverheat = false;
    bool simulateEncoderFault = false;
    double encoderNoiseAmplitude = 0.0;
    
    // Mechanical faults
    bool simulateJam = false;
    bool simulateBrokenGear = false;
    double additionalBacklash = 0.0;
    double additionalFriction = 0.0;
    
    // Sensor faults
    bool positiveLimitDisconnect = false;
    bool negativeLimitDisconnect = false;
    bool homeDisconnect = false;
    bool indexDisconnect = false;
    
    // Load disturbances
    double injectedLoadTorque = 0.0;
    bool simulateLoadImpact = false;
    double impactTorque = 0.0;
    double impactDuration = 0.0;
};

// ============================================================================
// Control Interface
// ============================================================================

/**
 * @brief Control mode
 */
enum class ControlMode {
    Disabled,         ///< No control, motor freewheeling
    Torque,           ///< Direct torque command
    Velocity,         ///< Velocity control
    Position          ///< Position control (cascaded velocity)
};

/**
 * @brief Controller parameters
 */
struct ControllerParams {
    // Position loop (outer)
    double posKp = 100.0;
    double posKi = 10.0;
    double posKd = 5.0;
    double posIntegralLimit = 100.0;
    double maxVelocityCmd = 100.0;
    
    // Velocity loop (inner)
    double velKp = 1.0;
    double velKi = 0.5;
    double velKd = 0.01;
    double velIntegralLimit = 10.0;
    double maxTorqueCmd = 10.0;
    
    // Velocity feedforward
    double velocityFF = 0.0;
    double accelerationFF = 0.0;
};

// ============================================================================
// Advanced Motor Model Class
// ============================================================================

/**
 * @brief Advanced motor model with geartrain and load
 */
class AdvancedMotorModel {
public:
    AdvancedMotorModel();
    explicit AdvancedMotorModel(const MotorParams& motorParams);
    ~AdvancedMotorModel() = default;
    
    // =========================================================================
    // Configuration
    // =========================================================================
    
    void setMotorParams(const MotorParams& params);
    void setBacklashParams(const BacklashParams& params);
    void setGeartrainParams(const GeartrainParams& params);
    void setLoadParams(const LoadParams& params);
    void setThermalParams(const ThermalParams& params);
    void setSensorConfig(const SensorConfig& config);
    void setControllerParams(const ControllerParams& params);
    void setErrorInjection(const ErrorInjection& injection);
    
    const MotorParams& getMotorParams() const { return motorParams_; }
    const BacklashParams& getBacklashParams() const { return backlashParams_; }
    const GeartrainParams& getGeartrainParams() const { return geartrainParams_; }
    const LoadParams& getLoadParams() const { return loadParams_; }
    const ThermalParams& getThermalParams() const { return thermalParams_; }
    const SensorConfig& getSensorConfig() const { return sensorConfig_; }
    const ControllerParams& getControllerParams() const { return controllerParams_; }
    
    // =========================================================================
    // Initialization and Reset
    // =========================================================================
    
    bool initialize();
    void reset();
    void setInitialState(double motorPos, double loadPos);
    
    // =========================================================================
    // Control Interface
    // =========================================================================
    
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return controlMode_; }
    
    void setTargetTorque(double torque);       ///< [Nm] at motor
    void setTargetVelocity(double velocity);   ///< [rad/s] at motor
    void setTargetPosition(double position);   ///< [rad] at motor
    
    void setExternalLoadTorque(double torque); ///< [Nm] at output
    
    // =========================================================================
    // Simulation
    // =========================================================================
    
    /**
     * @brief Update simulation by one time step
     * @param dt Time step [s]
     */
    void update(double dt);
    
    /**
     * @brief Get current system state
     */
    const SystemState& getState() const { return state_; }
    
    // =========================================================================
    // Convenience Accessors
    // =========================================================================
    
    // Motor (all at motor shaft)
    double getMotorPosition() const { return state_.motor.position; }
    double getMotorVelocity() const { return state_.motor.velocity; }
    double getMotorTorque() const { return state_.motor.torque; }
    double getMotorCurrent() const { return state_.motor.current; }
    
    // Output (all at load/output shaft)
    double getOutputPosition() const { return state_.load.position; }
    double getOutputVelocity() const { return state_.load.velocity; }
    double getOutputTorque() const { return state_.load.appliedTorque; }
    
    // Encoder values (in counts)
    int32_t getMotorEncoderPosition() const { return state_.sensors.encoderPosition; }
    
    // Following error (at motor)
    double getFollowingError() const { return targetPosition_ - state_.motor.position; }
    
    // Fault status
    bool hasFault() const { return state_.hasFault; }
    uint16_t getFaultCode() const { return state_.faultCode; }
    void clearFault();
    
    // =========================================================================
    // Callbacks
    // =========================================================================
    
    using FaultCallback = std::function<void(uint16_t code)>;
    using LimitCallback = std::function<void(bool positive, bool negative)>;
    using HomeCallback = std::function<void(bool active)>;
    using IndexCallback = std::function<void(double position)>;
    
    void setFaultCallback(FaultCallback cb) { faultCallback_ = std::move(cb); }
    void setLimitCallback(LimitCallback cb) { limitCallback_ = std::move(cb); }
    void setHomeCallback(HomeCallback cb) { homeCallback_ = std::move(cb); }
    void setIndexCallback(IndexCallback cb) { indexCallback_ = std::move(cb); }

private:
    // =========================================================================
    // Internal Methods
    // =========================================================================
    
    // Control loops
    double runPositionControl(double dt);
    double runVelocityControl(double targetVel, double dt);
    double runTorqueLimit(double torqueCmd);
    
    // Physics simulation
    void simulateMotor(double torqueCmd, double dt);
    void simulateBacklash(double dt);
    void simulateGeartrain(double dt);
    void simulateLoad(double dt);
    void simulateThermal(double dt);
    
    // Sensor updates
    void updateSensors();
    void updateEncoder();
    
    // Fault detection
    void checkFaults();
    void triggerFault(uint16_t code);
    
    // Helper functions
    double calculateMotorFriction(double velocity, double appliedTorque);
    double calculateGearFriction(double velocity, double appliedTorque);
    double calculateLoadFriction(double velocity, double appliedTorque);
    double calculateBacklashTorque();
    double calculateHardStopTorque();
    
    // =========================================================================
    // Parameters
    // =========================================================================
    
    MotorParams motorParams_;
    BacklashParams backlashParams_;
    GeartrainParams geartrainParams_;
    LoadParams loadParams_;
    ThermalParams thermalParams_;
    SensorConfig sensorConfig_;
    ControllerParams controllerParams_;
    ErrorInjection errorInjection_;
    
    // =========================================================================
    // State
    // =========================================================================
    
    SystemState state_;
    ControlMode controlMode_ = ControlMode::Disabled;
    
    // Control targets
    double targetTorque_ = 0.0;
    double targetVelocity_ = 0.0;
    double targetPosition_ = 0.0;
    
    // Controller state
    double posIntegral_ = 0.0;
    double velIntegral_ = 0.0;
    double lastPosError_ = 0.0;
    double lastVelError_ = 0.0;
    
    // Previous state for derivative calculations
    double prevMotorPosition_ = 0.0;
    double prevLoadPosition_ = 0.0;
    double prevIndexPosition_ = 0.0;
    
    // Peak torque tracking
    double peakTorqueTimer_ = 0.0;
    bool usingPeakTorque_ = false;
    
    // Initialization flag
    bool initialized_ = false;
    
    // Thread safety
    mutable std::mutex mutex_;
    
    // Callbacks
    FaultCallback faultCallback_;
    LimitCallback limitCallback_;
    HomeCallback homeCallback_;
    IndexCallback indexCallback_;
};

// ============================================================================
// Factory Functions
// ============================================================================

namespace Factory {

/**
 * @brief Create parameters for a typical BLDC servo motor
 */
MotorParams createBLDCServoMotor(
    double ratedTorque = 1.0,
    double maxSpeed = 3000.0,  // RPM
    int encoderBits = 17
);

/**
 * @brief Create parameters for a stepper motor (modeled as servo)
 */
MotorParams createStepperMotor(
    double holdingTorque = 0.5,
    int stepsPerRev = 200,
    int microsteps = 256
);

/**
 * @brief Create planetary gearbox parameters
 */
GeartrainParams createPlanetaryGearbox(
    double ratio = 10.0,
    int stages = 1,
    bool highEfficiency = true
);

/**
 * @brief Create harmonic drive parameters
 */
GeartrainParams createHarmonicDrive(
    double ratio = 100.0
);

/**
 * @brief Create friction parameters from basic values
 */
FrictionParams createFrictionParams(
    double staticFriction,
    double coulombFriction,
    double viscousCoeff
);

} // namespace Factory

} // namespace Motor
} // namespace CiA402
