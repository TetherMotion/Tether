/**
 * @file CiA402Slave.hpp
 * @brief CiA 402 Drives and Motion Control Slave Implementation
 *
 * @details
 * Implements a complete CiA 402 compliant drive slave with:
 * - Full drive state machine (IEC 61800-7-201)
 * - All operating modes (PP, PV, PT, HM, IP, CSP, CSV, CST)
 * - Homing procedures
 * - Position, velocity, and torque control
 * - Motion profile generation
 *
 * ## Drive State Machine
 *
 * ```
 * NOT READY TO SWITCH ON  ──────►  SWITCH ON DISABLED
 *         │                               │
 *         ▼                               ▼
 *                              READY TO SWITCH ON
 *                                         │
 *                                         ▼
 *                                   SWITCHED ON
 *                                         │
 *                                         ▼
 *                               OPERATION ENABLED  ◄──►  QUICK STOP ACTIVE
 *                                         │
 *                                         ▼
 *                              FAULT REACTION ACTIVE
 *                                         │
 *                                         ▼
 *                                       FAULT
 * ```
 *
 * ## Operating Modes
 *
 * | Mode | Value | Description |
 * |------|-------|-------------|
 * | PP   | 1     | Profile Position |
 * | VL   | 2     | Velocity Mode |
 * | PV   | 3     | Profile Velocity |
 * | PT   | 4     | Profile Torque |
 * | HM   | 6     | Homing Mode |
 * | IP   | 7     | Interpolated Position |
 * | CSP  | 8     | Cyclic Synchronous Position |
 * | CSV  | 9     | Cyclic Synchronous Velocity |
 * | CST  | 10    | Cyclic Synchronous Torque |
 *
 * ## Usage Example
 *
 * @code
 * CiA402SlaveConfig config;
 * config.identity.vendorId = 0x1234;
 * config.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::CST;
 * config.positionEncoderResolution = 131072;  // 17-bit encoder
 *
 * auto drive = std::make_unique<CiA402Slave>(config);
 * drive->setHAL(hal);
 * drive->start();
 *
 * // Simulation: update motor physics
 * drive->setActualPosition(1000);
 * drive->setActualVelocity(100);
 * @endcode
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"
#include "profiles/cia402/CiA402Config.hpp"

#include <array>
#include <atomic>
#include <functional>
#include <magic_enum/magic_enum.hpp>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 402 Operating Modes
// ============================================================================

/**
 * @brief CiA 402 operating mode flags
 */
namespace CiA402Mode {
    constexpr uint32_t PP  = (1 << 0);   // Profile Position
    constexpr uint32_t VL  = (1 << 1);   // Velocity Mode
    constexpr uint32_t PV  = (1 << 2);   // Profile Velocity
    constexpr uint32_t PT  = (1 << 3);   // Profile Torque
    constexpr uint32_t HM  = (1 << 5);   // Homing Mode
    constexpr uint32_t IP  = (1 << 6);   // Interpolated Position
    constexpr uint32_t CSP = (1 << 7);   // Cyclic Synchronous Position
    constexpr uint32_t CSV = (1 << 8);   // Cyclic Synchronous Velocity
    constexpr uint32_t CST = (1 << 9);   // Cyclic Synchronous Torque
    
    constexpr uint32_t AllCyclic = CSP | CSV | CST;
    constexpr uint32_t AllProfile = PP | PV | PT | VL;
    constexpr uint32_t All = AllCyclic | AllProfile | HM | IP;
}

// ============================================================================
// CiA 402 Drive States
// ============================================================================

/**
 * @brief CiA 402 drive state machine states
 */
enum class CiA402State : uint8_t {
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled   = 1,
    ReadyToSwitchOn    = 2,
    SwitchedOn         = 3,
    OperationEnabled   = 4,
    QuickStopActive    = 5,
    FaultReactionActive= 6,
    Fault              = 7,
};

// ============================================================================
// CiA 402 Slave Configuration
// ============================================================================

/**
 * @brief Configuration for CiA 402 drive slave
 */
struct CiA402SlaveConfig {
    // Identity
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000192,  // Device type for drive
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 402 Drive",
    };
    
    // Supported operating modes
    uint32_t supportedModes = CiA402Mode::AllCyclic | CiA402Mode::PP | CiA402Mode::HM;
    
    // Motor/encoder parameters
    uint32_t positionEncoderResolution = 131072;  ///< Position units per revolution
    uint32_t velocityEncoderResolution = 131072;  ///< Velocity encoder resolution
    uint32_t encoderIncrements = 131072;          ///< Encoder increments per revolution
    uint32_t motorRevolutions = 1;                ///< Motor revolutions for above increments
    
    // Position limits
    int32_t softwarePosLimitMin = -2147483647;    ///< Minimum position (0x607D:1)
    int32_t softwarePosLimitMax = 2147483647;     ///< Maximum position (0x607D:2)
    int32_t homingOffset = 0;                     ///< Homing offset (0x607C)
    
    // Velocity limits
    uint32_t maxProfileVelocity = 1000000;        ///< Max profile velocity (0x607F)
    uint32_t maxMotorVelocity = 1200000;          ///< Max motor velocity (0x6080)
    
    // Acceleration limits
    uint32_t maxAcceleration = 10000000;          ///< Max acceleration (0x60C5)
    uint32_t maxDeceleration = 10000000;          ///< Max deceleration (0x60C6)
    uint32_t quickStopDeceleration = 20000000;    ///< Quick stop deceleration
    
    // Torque limits
    int16_t maxTorque = 1000;                     ///< Max torque in 0.1% units
    int16_t motorRatedTorque = 1000;              ///< Rated torque (0x6076)
    uint32_t motorRatedCurrent = 1000;            ///< Rated current in mA (0x6075)
    
    // Following error
    uint32_t followingErrorWindow = 65535;        ///< Following error window (0x6065)
    uint16_t followingErrorTimeout = 0;           ///< Following error timeout ms (0x6066)
    
    // Position window
    uint32_t positionWindow = 10;                 ///< Position window (0x6067)
    uint16_t positionWindowTime = 0;              ///< Position window time ms (0x6068)
    
    // Velocity threshold
    uint16_t velocityThreshold = 10;              ///< Velocity threshold (0x606F)
    uint16_t velocityThresholdTime = 0;           ///< Velocity threshold time ms (0x6070)
    
    // Homing parameters
    int8_t homingMethod = 35;                     ///< Homing method (0x6098)
    uint32_t homingSwitchSpeed = 10000;           ///< Speed for homing switch (0x6099:1)
    uint32_t homingZeroSpeed = 1000;              ///< Speed for zero (0x6099:2)
    uint32_t homingAcceleration = 100000;         ///< Homing acceleration (0x609A)
    
    // DC support
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;          // 1ms
    
    // Simulation parameters
    bool enableSimulation = true;                 ///< Enable motor simulation
    float simulatedInertia = 0.001f;              ///< Simulated inertia (kg·m²)
    float simulatedFriction = 0.1f;               ///< Simulated friction coefficient
    
    // Logging
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 402 Slave Class
// ============================================================================

/**
 * @brief CiA 402 Drives and Motion Control Slave
 *
 * Provides a complete implementation of a CiA 402 drive including:
 * - Full drive state machine
 * - All operating modes (PP, PV, PT, HM, IP, CSP, CSV, CST)
 * - Motion simulation
 * - Complete object dictionary
 */
class CiA402Slave : public ProfileSlave {
public:
    /**
     * @brief Constructor
     * @param config Drive slave configuration
     */
    explicit CiA402Slave(const CiA402SlaveConfig& config);
    
    /**
     * @brief Destructor
     */
    ~CiA402Slave() override;
    
    // ========================================================================
    // ProfileSlave Interface
    // ========================================================================
    
    const char* getProfileName() const override { return "CiA 402"; }
    uint32_t getDeviceType() const override { return 0x00000192; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // ========================================================================
    // Drive State Machine
    // ========================================================================
    
    /**
     * @brief Get current drive state
     */
    CiA402State getDriveState() const { return driveState_; }
    
    /**
     * @brief Get status word (0x6041)
     */
    uint16_t getStatusWord() const;
    
    /**
     * @brief Process control word (0x6040)
     */
    void processControlWord(uint16_t controlWord);
    
    /**
     * @brief Trigger fault
     * @param errorCode Error code (0x603F)
     */
    void triggerFault(uint16_t errorCode);
    
    /**
     * @brief Clear fault
     */
    void clearFault();
    
    /**
     * @brief Set drive state change callback
     */
    using DriveStateCallback = std::function<void(CiA402State oldState, CiA402State newState)>;
    void setDriveStateCallback(DriveStateCallback callback);
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    /**
     * @brief Get current operating mode (0x6061)
     */
    int8_t getOperatingMode() const { return operatingModeDisplay_; }
    
    /**
     * @brief Get target operating mode (0x6060)
     */
    int8_t getTargetOperatingMode() const { return operatingMode_; }
    
    /**
     * @brief Check if mode is supported
     */
    bool isModeSupported(int8_t mode) const;
    
    // ========================================================================
    // Position Control
    // ========================================================================
    
    /**
     * @brief Get actual position (0x6064)
     */
    int32_t getActualPosition() const { return actualPosition_; }
    
    /**
     * @brief Set actual position (for simulation)
     */
    void setActualPosition(int32_t position);
    
    /**
     * @brief Get target position (0x607A)
     */
    int32_t getTargetPosition() const { return targetPosition_; }
    
    /**
     * @brief Get position demand value (0x6062)
     */
    int32_t getPositionDemand() const { return positionDemand_; }
    
    /**
     * @brief Get following error (0x60F4)
     */
    int32_t getFollowingError() const;
    
    // ========================================================================
    // Velocity Control
    // ========================================================================
    
    /**
     * @brief Get actual velocity (0x606C)
     */
    int32_t getActualVelocity() const { return actualVelocity_; }
    
    /**
     * @brief Set actual velocity (for simulation)
     */
    void setActualVelocity(int32_t velocity);
    
    /**
     * @brief Get target velocity (0x60FF)
     */
    int32_t getTargetVelocity() const { return targetVelocity_; }
    
    /**
     * @brief Get velocity demand value (0x606B)
     */
    int32_t getVelocityDemand() const { return velocityDemand_; }
    
    // ========================================================================
    // Torque Control
    // ========================================================================
    
    /**
     * @brief Get actual torque (0x6077) in 0.1% of rated torque
     */
    int16_t getActualTorque() const { return actualTorque_; }
    
    /**
     * @brief Set actual torque (for simulation)
     */
    void setActualTorque(int16_t torque);
    
    /**
     * @brief Get target torque (0x6071)
     */
    int16_t getTargetTorque() const { return targetTorque_; }
    
    /**
     * @brief Get torque demand value (0x6074)
     */
    int16_t getTorqueDemand() const { return torqueDemand_; }
    
    // ========================================================================
    // Homing
    // ========================================================================
    
    /**
     * @brief Check if homing is complete
     */
    bool isHomingComplete() const { return homingComplete_; }
    
    /**
     * @brief Set homing complete flag
     */
    void setHomingComplete(bool complete);
    
    /**
     * @brief Set home position (for simulation)
     *
     * Simulates finding the home switch.
     *
     * @param position Position where home is found
     */
    void setHomePosition(int32_t position);
    
    /**
     * @brief Set homing callback
     *
     * Called during homing simulation to allow external homing logic.
     */
    using HomingCallback = std::function<bool(int8_t method, int32_t& homePosition)>;
    void setHomingCallback(HomingCallback callback);
    
    // ========================================================================
    // Touch Probe
    // ========================================================================
    
    /**
     * @brief Trigger touch probe event
     * @param probeNum Probe number (1 or 2)
     * @param position Captured position
     */
    void triggerTouchProbe(int probeNum, int32_t position);
    
    /**
     * @brief Get touch probe position
     */
    int32_t getTouchProbePosition(int probeNum) const;
    
    /**
     * @brief Get touch probe status (0x60B8)
     */
    uint16_t getTouchProbeStatus() const { return touchProbeStatus_; }
    
    // ========================================================================
    // Digital Inputs/Outputs
    // ========================================================================
    
    /**
     * @brief Set digital inputs (0x60FD)
     */
    void setDigitalInputs(uint32_t inputs);
    
    /**
     * @brief Get digital inputs (0x60FD)
     */
    uint32_t getDigitalInputs() const { return digitalInputs_; }
    
    /**
     * @brief Get digital outputs (0x60FE)
     */
    uint32_t getDigitalOutputs() const { return digitalOutputs_; }
    
    // ========================================================================
    // Supported Drive Functions (0x6502)
    // ========================================================================
    
    /**
     * @brief Get supported drive functions
     */
    uint32_t getSupportedDriveFunctions() const;
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    void onStateChange(SlaveState oldState, SlaveState newState) override;
    void onSync(int syncNum, uint64_t timestamp) override;
    
private:
    // Configuration
    CiA402SlaveConfig driveConfig_;
    
    // Drive state machine
    CiA402State driveState_ = CiA402State::SwitchOnDisabled;
    DriveStateCallback driveStateCallback_;
    
    // Control/Status words
    uint16_t controlWord_ = 0;
    uint16_t statusWord_ = 0;
    
    // Operating mode
    int8_t operatingMode_ = 0;        // Target (0x6060)
    int8_t operatingModeDisplay_ = 0;  // Actual (0x6061)
    
    // Error
    uint16_t errorCode_ = 0;          // 0x603F
    
    // Position
    int32_t actualPosition_ = 0;      // 0x6064
    int32_t targetPosition_ = 0;      // 0x607A
    int32_t positionDemand_ = 0;      // 0x6062
    int32_t positionOffset_ = 0;      // 0x60B0 (CSP)
    
    // Velocity
    int32_t actualVelocity_ = 0;      // 0x606C
    int32_t targetVelocity_ = 0;      // 0x60FF
    int32_t velocityDemand_ = 0;      // 0x606B
    int32_t velocityOffset_ = 0;      // 0x60B1 (CSV)
    uint32_t profileVelocity_ = 0;    // 0x6081
    
    // Acceleration
    uint32_t profileAcceleration_ = 0;     // 0x6083
    uint32_t profileDeceleration_ = 0;     // 0x6084
    
    // Torque
    int16_t actualTorque_ = 0;        // 0x6077
    int16_t targetTorque_ = 0;        // 0x6071
    int16_t torqueDemand_ = 0;        // 0x6074
    int16_t torqueOffset_ = 0;        // 0x60B2 (CST)
    uint16_t maxTorque_ = 1000;       // 0x6072
    
    // Homing
    bool homingComplete_ = false;
    bool homingActive_ = false;
    int8_t homingMethod_ = 35;
    uint32_t homingSwitchSpeed_ = 10000;
    uint32_t homingZeroSpeed_ = 1000;
    uint32_t homingAcceleration_ = 100000;
    int32_t homeOffset_ = 0;
    HomingCallback homingCallback_;
    
    // Touch probe
    uint16_t touchProbeFunction_ = 0;   // 0x60B8
    uint16_t touchProbeStatus_ = 0;     // 0x60B9
    int32_t touchProbe1Pos_ = 0;        // 0x60BA
    int32_t touchProbe2Pos_ = 0;        // 0x60BC
    
    // Digital I/O
    uint32_t digitalInputs_ = 0;        // 0x60FD
    uint32_t digitalOutputs_ = 0;       // 0x60FE
    uint32_t digitalOutputMask_ = 0;    // 0x60FE:2
    
    // Internal state
    bool targetReached_ = false;
    bool setPointAcknowledge_ = false;
    int32_t internalPosition_ = 0;
    int32_t internalVelocity_ = 0;
    uint64_t lastSimTime_ = 0;
    
    // PDO layout (for CSP mode)
    struct PDOLayout {
        // RxPDO (master → slave)
        size_t controlWordOffset = 0;
        size_t targetPositionOffset = 2;
        size_t targetVelocityOffset = 6;
        size_t targetTorqueOffset = 10;
        size_t modeOffset = 12;
        size_t digitalOutputsOffset = 13;
        
        // TxPDO (slave → master)
        size_t statusWordOffset = 0;
        size_t actualPositionOffset = 2;
        size_t actualVelocityOffset = 6;
        size_t actualTorqueOffset = 10;
        size_t modeDisplayOffset = 12;
        size_t digitalInputsOffset = 13;
    } pdoLayout_;
    
    // State machine methods
    void updateDriveState(uint16_t controlWord);
    void setDriveState(CiA402State newState);
    uint16_t computeStatusWord() const;
    
    // Motion methods
    void updateMotion(uint64_t deltaNs);
    void simulateMotion(uint64_t deltaNs);
    void executeProfilePosition();
    void executeProfileVelocity();
    void executeProfileTorque();
    void executeCyclicSyncPosition();
    void executeCyclicSyncVelocity();
    void executeCyclicSyncTorque();
    void executeHoming();
    
    // Position window check
    bool checkPositionReached();
    bool checkVelocityReached();
    
    // SDO handlers
    SDOAbortCode readMotionParam(uint16_t index, uint8_t subindex,
                                  uint8_t* data, size_t& len);
    SDOAbortCode writeMotionParam(uint16_t index, uint8_t subindex,
                                   const uint8_t* data, size_t len);
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a CiA 402 drive slave with specified configuration
 */
std::unique_ptr<CiA402Slave> createCiA402Slave(const CiA402SlaveConfig& config);

/**
 * @brief Create a simple servo drive (CSP mode)
 */
std::unique_ptr<CiA402Slave> createServoDrive(uint32_t encoderResolution = 131072);

/**
 * @brief Create a stepper drive (CSP mode)
 */
std::unique_ptr<CiA402Slave> createStepperDrive(uint32_t stepsPerRevolution = 200,
                                                  uint32_t microstepping = 256);

/**
 * @brief Create a frequency inverter (CSV mode)
 */
std::unique_ptr<CiA402Slave> createFrequencyInverter();

}  // namespace slave
}  // namespace EtherCAT
