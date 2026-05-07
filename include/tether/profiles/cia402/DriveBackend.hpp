/**
 * @file DriveBackend.hpp
 * @brief Abstract drive backend interface for CiA 402 motion control
 * 
 * @details
 * This abstraction layer allows the CiA 402 motion controller to work with
 * different communication backends (EtherCAT, CANopen, simulation, etc.)
 * 
 * ## Architecture
 * 
 * ```
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │                     MotionController                         │
 *  │  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐      │
 *  │  │  MotionGen    │ │  PathPlanner  │ │ Interpolator  │      │
 *  │  └───────────────┘ └───────────────┘ └───────────────┘      │
 *  └────────────────────────────┬────────────────────────────────┘
 *                               │ (Axis Commands)
 *  ┌────────────────────────────▼────────────────────────────────┐
 *  │                      CiA402Axis                              │
 *  │  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐      │
 *  │  │ StateMachine  │ │   PID Ctrl    │ │    Filter     │      │
 *  │  └───────────────┘ └───────────────┘ └───────────────┘      │
 *  └────────────────────────────┬────────────────────────────────┘
 *                               │ (Abstract Interface)
 *  ┌────────────────────────────▼────────────────────────────────┐
 *  │                     DriveBackend                             │
 *  │            (Abstract Interface - THIS FILE)                  │
 *  └────────────────────────────┬────────────────────────────────┘
 *                               │
 *        ┌──────────────────────┼──────────────────────┐
 *        │                      │                      │
 *  ┌─────▼─────┐         ┌──────▼─────┐        ┌──────▼──────┐
 *  │ EtherCAT  │         │  CANopen   │        │ Simulation  │
 *  │  Backend  │         │  Backend   │        │   Backend   │
 *  └───────────┘         └────────────┘        └─────────────┘
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * // Create backend for your communication type
 * auto backend = std::make_unique<EtherCATBackend>();
 * 
 * // Create axis with backend
 * CiA402Axis axis(0, std::move(backend));
 * 
 * // Use axis - all communication goes through backend
 * axis.enable();
 * axis.setTargetPosition(1000);
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include <cstdint>
#include <memory>
#include <functional>
#include <string>
#include <optional>
#include <vector>

namespace CiA402 {

/**
 * @brief Drive motion state (position, velocity, torque)
 */
struct DriveState {
    int32_t actualPosition{0};      ///< Current position [internal units]
    int32_t actualVelocity{0};      ///< Current velocity [internal units/s]
    int16_t actualTorque{0};        ///< Current torque [0.1% of rated]
    int16_t actualCurrent{0};       ///< Current [mA]
    
    int32_t targetPosition{0};      ///< Commanded position
    int32_t targetVelocity{0};      ///< Commanded velocity
    int16_t targetTorque{0};        ///< Commanded torque
    
    int32_t followingError{0};      ///< Following error [internal units]
    int32_t positionDemand{0};      ///< Position demand value
    int32_t velocityDemand{0};      ///< Velocity demand value
    
    uint16_t statusWord{0};         ///< CiA 402 status word
    uint16_t controlWord{0};        ///< CiA 402 control word
    
    int8_t modesOfOperation{0};     ///< Current operating mode
    int8_t modesOfOperationDisplay{0}; ///< Displayed operating mode
    
    uint16_t errorCode{0};          ///< Active error code
    uint32_t errorRegister{0};      ///< Error register
};

/**
 * @brief Drive configuration parameters
 */
struct DriveConfig {
    // Motor parameters
    uint32_t motorRatedCurrent{1000};   ///< Motor rated current [mA]
    uint32_t motorRatedTorque{1000};    ///< Motor rated torque [mNm]
    uint32_t motorRevolutions{1};       ///< Motor revolutions for gear ratio
    uint32_t shaftRevolutions{1};       ///< Shaft revolutions for gear ratio
    
    // Encoder parameters
    uint32_t encoderResolution{4096};   ///< Encoder increments per revolution
    int32_t positionEncoderOffset{0};   ///< Position encoder offset
    
    // Factor group
    int32_t positionFactor{1};          ///< Position factor numerator
    int32_t positionDivisor{1};         ///< Position factor divisor
    int32_t velocityFactor{1};          ///< Velocity factor numerator
    int32_t velocityDivisor{1};         ///< Velocity factor divisor
    
    // Limits
    int32_t maxPosition{INT32_MAX};     ///< Maximum position
    int32_t minPosition{INT32_MIN};     ///< Minimum position
    uint32_t maxVelocity{100000};       ///< Maximum velocity
    uint32_t maxAcceleration{100000};   ///< Maximum acceleration
    uint32_t maxDeceleration{100000};   ///< Maximum deceleration
    int16_t maxTorque{1000};            ///< Maximum torque [0.1%]
    int16_t maxCurrent{1000};           ///< Maximum current [mA]
    
    // Quick stop
    int16_t quickStopOptionCode{2};     ///< Quick stop behavior
    int16_t shutdownOptionCode{0};      ///< Shutdown behavior
    int16_t disableOperationCode{1};    ///< Disable operation behavior
    int16_t haltOptionCode{1};          ///< Halt behavior
    int16_t faultReactionCode{2};       ///< Fault reaction behavior
    
    // Following error
    uint32_t followingErrorWindow{1000};    ///< Following error window
    uint16_t followingErrorTimeout{100};    ///< Following error timeout [ms]
    uint32_t positionWindow{100};           ///< Position window for "target reached"
    uint16_t positionWindowTime{10};        ///< Time to be in window [ms]
};

/**
 * @brief Profile motion parameters
 */
struct ProfileParams {
    int32_t targetPosition{0};      ///< Target position
    int32_t targetVelocity{0};      ///< Profile velocity
    uint32_t acceleration{0};       ///< Acceleration
    uint32_t deceleration{0};       ///< Deceleration
    int16_t targetTorque{0};        ///< Target torque
    uint16_t torqueSlope{0};        ///< Torque slope
    int32_t positionOffset{0};      ///< Position offset (CSP)
    int32_t velocityOffset{0};      ///< Velocity offset (CSV)
    int16_t torqueOffset{0};        ///< Torque offset (CST)
};

/**
 * @brief Homing parameters
 */
struct HomingParams {
    int8_t method{0};               ///< Homing method (0-35)
    uint32_t speedSwitch{100};      ///< Speed during search for switch
    uint32_t speedZero{10};         ///< Speed during search for zero
    uint32_t acceleration{1000};    ///< Homing acceleration
    int32_t offset{0};              ///< Home offset
};

/**
 * @brief Interpolation parameters
 */
struct InterpolationParams {
    int8_t subModeSelect{0};        ///< Interpolation sub-mode
    uint8_t timePeriod{1};          ///< Time period value
    int8_t timeIndex{-3};           ///< Time period index (-3 = ms)
    int32_t positionBuffer[CIA402_TRAJECTORY_BUFFER_SIZE]{0};  ///< Position buffer
    size_t bufferLength{0};         ///< Number of points in buffer
};

/**
 * @brief SDO read/write result
 */
struct SDOResult {
    bool success{false};
    uint32_t abortCode{0};
    std::vector<uint8_t> data;
    std::string errorMessage;
};

/**
 * @brief Backend event callback types
 */
using StateChangeCallback = std::function<void(State oldState, State newState)>;
using ErrorCallback = std::function<void(uint16_t errorCode, const std::string& message)>;
using WarningCallback = std::function<void(uint16_t warningCode, const std::string& message)>;
using SyncCallback = std::function<void(uint64_t timestamp)>;

/**
 * @brief Abstract drive backend interface
 * 
 * Implement this interface for specific communication protocols
 * (EtherCAT, CANopen, simulation, etc.)
 */
class DriveBackend {
public:
    virtual ~DriveBackend() = default;
    
    // ========================================================================
    // Lifecycle Management
    // ========================================================================
    
    /**
     * @brief Initialize the backend
     * 
     * @return true if initialization successful
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Deinitialize the backend
     */
    virtual void deinitialize() = 0;
    
    /**
     * @brief Check if backend is connected and operational
     */
    virtual bool isConnected() const = 0;
    
    /**
     * @brief Get backend identifier/name
     */
    virtual std::string getName() const = 0;
    
    // ========================================================================
    // Process Data (PDO) - Cyclic Communication
    // ========================================================================
    
    /**
     * @brief Update process data (read from drive)
     * 
     * Call this cyclically to update drive state.
     * 
     * @return true if update successful
     */
    virtual bool updateInputs() = 0;
    
    /**
     * @brief Send process data (write to drive)
     * 
     * Call this cyclically to send commands to drive.
     * 
     * @return true if update successful
     */
    virtual bool updateOutputs() = 0;
    
    /**
     * @brief Get current drive state
     */
    virtual DriveState getState() const = 0;
    
    // ========================================================================
    // Control Word / Status Word
    // ========================================================================
    
    /**
     * @brief Read status word
     */
    virtual uint16_t readStatusWord() = 0;
    
    /**
     * @brief Write control word
     */
    virtual void writeControlWord(uint16_t controlWord) = 0;
    
    /**
     * @brief Read control word (last written)
     */
    virtual uint16_t readControlWord() const = 0;
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    /**
     * @brief Set operating mode
     * 
     * @param mode CiA 402 operating mode
     * @return true if mode change successful
     */
    virtual bool setOperatingMode(OperatingMode mode) = 0;
    
    /**
     * @brief Get current operating mode
     */
    virtual OperatingMode getOperatingMode() const = 0;
    
    /**
     * @brief Get displayed operating mode (from drive)
     */
    virtual OperatingMode getDisplayedMode() const = 0;
    
    // ========================================================================
    // Position Control
    // ========================================================================
    
    /**
     * @brief Set target position (PP, CSP modes)
     */
    virtual void setTargetPosition(int32_t position) = 0;
    
    /**
     * @brief Get actual position
     */
    virtual int32_t getActualPosition() const = 0;
    
    /**
     * @brief Get position demand
     */
    virtual int32_t getPositionDemand() const = 0;
    
    /**
     * @brief Get following error
     */
    virtual int32_t getFollowingError() const = 0;
    
    /**
     * @brief Set position offset (CSP mode)
     */
    virtual void setPositionOffset(int32_t offset) = 0;
    
    // ========================================================================
    // Velocity Control
    // ========================================================================
    
    /**
     * @brief Set target velocity (PV, CSV modes)
     */
    virtual void setTargetVelocity(int32_t velocity) = 0;
    
    /**
     * @brief Get actual velocity
     */
    virtual int32_t getActualVelocity() const = 0;
    
    /**
     * @brief Get velocity demand
     */
    virtual int32_t getVelocityDemand() const = 0;
    
    /**
     * @brief Set velocity offset (CSV mode)
     */
    virtual void setVelocityOffset(int32_t offset) = 0;
    
    // ========================================================================
    // Torque Control
    // ========================================================================
    
    /**
     * @brief Set target torque (PT, CST modes)
     * 
     * @param torque Torque in 0.1% of rated torque
     */
    virtual void setTargetTorque(int16_t torque) = 0;
    
    /**
     * @brief Get actual torque
     */
    virtual int16_t getActualTorque() const = 0;
    
    /**
     * @brief Set torque offset (CST mode)
     */
    virtual void setTorqueOffset(int16_t offset) = 0;
    
    // ========================================================================
    // Profile Parameters
    // ========================================================================
    
    /**
     * @brief Set profile velocity for position mode
     */
    virtual void setProfileVelocity(uint32_t velocity) = 0;
    
    /**
     * @brief Set profile acceleration
     */
    virtual void setProfileAcceleration(uint32_t acceleration) = 0;
    
    /**
     * @brief Set profile deceleration
     */
    virtual void setProfileDeceleration(uint32_t deceleration) = 0;
    
    /**
     * @brief Set motion profile type
     * 
     * @param type 0 = trapezoidal, 1 = S-curve (if supported)
     */
    virtual void setMotionProfileType(int16_t type) = 0;
    
    // ========================================================================
    // Homing
    // ========================================================================
    
    /**
     * @brief Configure homing parameters
     */
    virtual bool configureHoming(const HomingParams& params) = 0;
    
    /**
     * @brief Get homing parameters
     */
    virtual HomingParams getHomingParams() const = 0;
    
    // ========================================================================
    // Interpolated Position Mode
    // ========================================================================
    
    /**
     * @brief Configure interpolation parameters
     */
    virtual bool configureInterpolation(const InterpolationParams& params) = 0;
    
    /**
     * @brief Add point to interpolation buffer
     */
    virtual bool addInterpolationPoint(int32_t position) = 0;
    
    /**
     * @brief Clear interpolation buffer
     */
    virtual void clearInterpolationBuffer() = 0;
    
    // ========================================================================
    // SDO Access (Service Data Objects)
    // ========================================================================
    
    /**
     * @brief Read SDO
     * 
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param data Output buffer
     * @param size Maximum size to read
     * @return SDO result
     */
    virtual SDOResult readSDO(uint16_t index, uint8_t subindex, 
                              void* data, size_t size) = 0;
    
    /**
     * @brief Write SDO
     * 
     * @param index Object dictionary index
     * @param subindex Object dictionary subindex
     * @param data Data to write
     * @param size Size of data
     * @return SDO result
     */
    virtual SDOResult writeSDO(uint16_t index, uint8_t subindex,
                               const void* data, size_t size) = 0;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Apply drive configuration
     */
    virtual bool configure(const DriveConfig& config) = 0;
    
    /**
     * @brief Get current drive configuration
     */
    virtual DriveConfig getConfiguration() const = 0;
    
    /**
     * @brief Store parameters to drive NVM
     */
    virtual bool storeParameters() = 0;
    
    /**
     * @brief Restore parameters from drive NVM
     */
    virtual bool restoreParameters() = 0;
    
    // ========================================================================
    // Error Handling
    // ========================================================================
    
    /**
     * @brief Get active error code
     */
    virtual uint16_t getErrorCode() const = 0;
    
    /**
     * @brief Get error register
     */
    virtual uint8_t getErrorRegister() const = 0;
    
    /**
     * @brief Get error history
     */
    virtual std::vector<uint16_t> getErrorHistory() const = 0;
    
    /**
     * @brief Clear error history
     */
    virtual bool clearErrorHistory() = 0;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    /**
     * @brief Set state change callback
     */
    virtual void setStateChangeCallback(StateChangeCallback callback) = 0;
    
    /**
     * @brief Set error callback
     */
    virtual void setErrorCallback(ErrorCallback callback) = 0;
    
    /**
     * @brief Set warning callback
     */
    virtual void setWarningCallback(WarningCallback callback) = 0;
    
    /**
     * @brief Set sync callback (called on each cycle)
     */
    virtual void setSyncCallback(SyncCallback callback) = 0;
    
    // ========================================================================
    // Timing
    // ========================================================================
    
    /**
     * @brief Get cycle time in microseconds
     */
    virtual uint32_t getCycleTimeUs() const = 0;
    
    /**
     * @brief Set desired cycle time
     */
    virtual bool setCycleTimeUs(uint32_t cycleTimeUs) = 0;
    
    /**
     * @brief Get timestamp of last update
     */
    virtual uint64_t getLastUpdateTimestamp() const = 0;
};

/**
 * @brief Shared pointer type for backends
 */
using DriveBackendPtr = std::shared_ptr<DriveBackend>;
using DriveBackendUPtr = std::unique_ptr<DriveBackend>;

/**
 * @brief Factory function type for creating backends
 */
using BackendFactory = std::function<DriveBackendUPtr(uint32_t slaveId)>;

} // namespace CiA402
