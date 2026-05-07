/**
 * @file CiA417Lift.hpp
 * @brief CiA 417 Lift/Elevator Controller
 *
 * Provides comprehensive control over CiA 417 compliant lift controllers.
 *
 * Features:
 * - Position and floor management
 * - Multi-door control
 * - Call management (car and hall calls)
 * - Safety system monitoring
 * - Load weighing integration
 * - Multiple operating modes
 * - Comprehensive diagnostics
 */

#pragma once

#include "profiles/cia417/CiA417Defs.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <vector>

namespace CiA417 {

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Lift specifications
 */
struct LiftSpec {
    uint8_t  lift_type = 0;
    uint8_t  num_floors = 0;
    uint8_t  num_doors = 1;
    uint16_t nominal_speed = 0;     // mm/s
    uint16_t nominal_load = 0;      // kg
    uint8_t  drive_type = 0;
    uint8_t  safety_class = 0;
};

/**
 * @brief Current lift state
 */
struct LiftState {
    uint32_t statusword = 0;
    int32_t  position = 0;          // mm
    int16_t  velocity = 0;          // mm/s
    uint8_t  floor_actual = 0;
    uint8_t  floor_target = 0;
    uint8_t  motion_state = 0;
    uint8_t  operating_mode = 0;
    uint8_t  door_state_main = 0;
    uint8_t  door_state_counter = 0;
    uint16_t load_actual = 0;       // kg
    uint32_t safety_inputs = 0;
    uint16_t fault_code = 0;
    uint16_t warning_code = 0;
    
    // Status helpers
    bool isReady() const { return statusword & StatuswordBits::Ready; }
    bool isRunning() const { return statusword & StatuswordBits::Running; }
    bool isAtFloor() const { return statusword & StatuswordBits::AtFloor; }
    bool isInLevelingZone() const { return statusword & StatuswordBits::InLevelingZone; }
    bool isInDoorZone() const { return statusword & StatuswordBits::InDoorZone; }
    bool areDoorsOpen() const { return statusword & StatuswordBits::DoorsOpen; }
    bool areDoorsClosed() const { return statusword & StatuswordBits::DoorsClosed; }
    bool isOverloaded() const { return statusword & StatuswordBits::Overloaded; }
    bool isSafetyOK() const { return statusword & StatuswordBits::SafetyOK; }
    bool isInspectionMode() const { return statusword & StatuswordBits::InspectionMode; }
    bool hasFault() const { return statusword & StatuswordBits::Fault; }
    bool isDirectionUp() const { return statusword & StatuswordBits::DirectionUp; }
    bool isDirectionDown() const { return statusword & StatuswordBits::DirectionDown; }
};

/**
 * @brief Floor configuration
 */
struct FloorConfig {
    uint8_t  floor_number = 0;
    int32_t  height_mm = 0;
    bool     enabled = true;
    bool     has_main_door = true;
    bool     has_counter_door = false;
    uint8_t  landing_call_type = 0;  // 0=none, 1=up, 2=down, 3=both
};

/**
 * @brief Call status
 */
struct CallStatus {
    uint64_t car_calls = 0;
    uint64_t hall_calls_up = 0;
    uint64_t hall_calls_down = 0;
    uint8_t  next_stop = 0;
    uint8_t  direction = Direction::None;
};

/**
 * @brief Safety status
 */
struct SafetyStatus {
    uint32_t safety_inputs = 0;
    bool     governor_ok = false;
    bool     safety_circuit_ok = false;
    bool     final_limits_ok = false;
    bool     buffer_ok = false;
    bool     door_locks_ok = false;
    uint8_t  overspeed_status = 0;
    uint8_t  ucm_status = 0;
    bool     emergency_stop_active = false;
};

/**
 * @brief Diagnostics data
 */
struct LiftDiagnostics {
    uint32_t trip_count = 0;
    uint32_t operating_hours = 0;
    uint32_t door_cycles = 0;
    uint32_t energy_kwh = 0;
    int16_t  drive_temperature = 0;
    int16_t  motor_temperature = 0;
    uint16_t fault_history[10] = {0};
};

// ============================================================================
// Callback Types
// ============================================================================

using FloorReachedCallback = std::function<void(uint8_t floor)>;
using DoorStateCallback = std::function<void(uint8_t door_id, uint8_t state)>;
using SafetyCallback = std::function<void(uint32_t safety_bits)>;
using FaultCallback = std::function<void(uint16_t fault_code)>;
using CallCallback = std::function<void(const CallStatus& calls)>;

// ============================================================================
// PDO Mapping Presets
// ============================================================================

enum class PDOMappingPreset {
    Basic,
    Extended,
    Full,
    Custom
};

// ============================================================================
// Lift Controller Class
// ============================================================================

class LiftController {
public:
    explicit LiftController(uint16_t slave_addr, bool use_configured_addr = false);
    ~LiftController();
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    bool initialize();
    bool isInitialized() const { return initialized_; }
    const LiftSpec& getSpec() const { return spec_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    bool applyPDOMapping(PDOMappingPreset preset);
    
    // ========================================================================
    // Cyclic Update
    // ========================================================================
    
    void processTxPDO(const uint8_t* data, size_t len);
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    void update();
    
    // ========================================================================
    // Basic Control
    // ========================================================================
    
    bool enable();
    bool disable();
    bool resetFault();
    bool emergencyStop();
    
    // ========================================================================
    // Motion Control
    // ========================================================================
    
    bool goToFloor(uint8_t floor);
    bool moveUp();
    bool moveDown();
    bool stop();
    bool relevel();
    
    uint8_t getCurrentFloor() const { return state_.floor_actual; }
    uint8_t getTargetFloor() const { return state_.floor_target; }
    int32_t getPosition() const { return state_.position; }
    int16_t getVelocity() const { return state_.velocity; }
    uint8_t getMotionState() const { return state_.motion_state; }
    uint8_t getDirection() const;
    
    bool isAtFloor() const { return state_.isAtFloor(); }
    bool isMoving() const { return state_.isRunning(); }
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    bool setOperatingMode(uint8_t mode);
    uint8_t getOperatingMode() const { return state_.operating_mode; }
    
    bool enterInspectionMode();
    bool exitInspectionMode();
    bool enterFireServiceMode();
    bool enterRescueMode();
    bool enterMaintenanceMode();
    
    // ========================================================================
    // Door Control
    // ========================================================================
    
    bool openDoor(uint8_t door_id = 0);
    bool closeDoor(uint8_t door_id = 0);
    bool stopDoor(uint8_t door_id = 0);
    bool nudgeDoor(uint8_t door_id = 0);
    bool lockDoor(uint8_t door_id = 0);
    bool unlockDoor(uint8_t door_id = 0);
    
    uint8_t getDoorState(uint8_t door_id = 0) const;
    bool isDoorOpen(uint8_t door_id = 0) const;
    bool isDoorClosed(uint8_t door_id = 0) const;
    bool isDoorLocked(uint8_t door_id = 0) const;
    
    bool setDoorTiming(uint16_t open_time_ms, uint16_t close_time_ms, 
                       uint16_t nudging_delay_ms);
    bool setDoorForce(uint8_t force_percent);
    
    // ========================================================================
    // Call Management
    // ========================================================================
    
    bool registerCarCall(uint8_t floor);
    bool cancelCarCall(uint8_t floor);
    bool registerHallCall(uint8_t floor, uint8_t direction);
    bool cancelHallCall(uint8_t floor, uint8_t direction);
    bool clearAllCalls();
    
    bool isCarCallActive(uint8_t floor) const;
    bool isHallCallActive(uint8_t floor, uint8_t direction) const;
    CallStatus getCallStatus() const { return calls_; }
    
    // ========================================================================
    // Floor Configuration
    // ========================================================================
    
    bool setFloorHeight(uint8_t floor, int32_t height_mm);
    int32_t getFloorHeight(uint8_t floor);
    bool enableFloor(uint8_t floor, bool enable);
    bool lockFloor(uint8_t floor, bool lock);
    bool isFloorEnabled(uint8_t floor) const;
    bool isFloorLocked(uint8_t floor) const;
    
    // ========================================================================
    // Load Management
    // ========================================================================
    
    uint16_t getLoad() const { return state_.load_actual; }
    float getLoadPercent() const;
    bool isOverloaded() const { return state_.isOverloaded(); }
    bool setOverloadThreshold(uint16_t threshold_kg);
    bool setBypassThreshold(uint16_t threshold_kg);
    
    // ========================================================================
    // Safety Systems
    // ========================================================================
    
    SafetyStatus getSafetyStatus();
    bool isSafetyOK() const { return state_.isSafetyOK(); }
    uint32_t getSafetyInputs() const { return state_.safety_inputs; }
    
    bool isSafetyInputActive(uint32_t input_bit) const;
    bool isGovernorOK() const;
    bool isSafetyCircuitOK() const;
    bool areDoorLocksEngaged() const;
    bool isEmergencyStopActive() const;
    
    // ========================================================================
    // Brake Control
    // ========================================================================
    
    bool applyBrakes();
    bool releaseBrakes();
    bool testBrakes();
    uint8_t getBrakeStatus();
    
    // ========================================================================
    // Inspection Control
    // ========================================================================
    
    bool inspectionMoveUp();
    bool inspectionMoveDown();
    bool inspectionStop();
    bool carTopMoveUp();
    bool carTopMoveDown();
    bool pitMoveUp();
    bool pitMoveDown();
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    const LiftState& getState() const { return state_; }
    uint16_t getFaultCode() const { return state_.fault_code; }
    LiftDiagnostics getDiagnostics();
    std::string getDiagnosticsString() const;
    
    float getDriveTemperature() const;
    float getMotorTemperature() const;
    uint32_t getTripCount() const;
    uint32_t getOperatingHours() const;
    
    // ========================================================================
    // Display/Communication
    // ========================================================================
    
    bool setDisplayFloor(uint8_t floor);
    bool setDisplayDirection(uint8_t direction);
    bool setDisplayMessage(const std::string& message);
    bool playAnnouncement(uint8_t announcement_id);
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setFloorReachedCallback(FloorReachedCallback callback);
    void setDoorStateCallback(DoorStateCallback callback);
    void setSafetyCallback(SafetyCallback callback);
    void setFaultCallback(FaultCallback callback);
    void setCallCallback(CallCallback callback);

private:
    bool readSpec();
    void processBasicPDO(const uint8_t* data, size_t len);
    void processExtendedPDO(const uint8_t* data, size_t len);
    void processFullPDO(const uint8_t* data, size_t len);
    void checkStateChanges();
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    LiftSpec spec_;
    LiftState state_;
    CallStatus calls_;
    uint32_t prev_statusword_;
    uint8_t prev_floor_;
    uint8_t prev_door_state_main_;
    uint8_t prev_door_state_counter_;
    
    uint32_t controlword_;
    uint8_t target_floor_;
    uint8_t door_command_;
    PDOMappingPreset current_mapping_;
    
    // Callbacks
    FloorReachedCallback floor_reached_callback_;
    DoorStateCallback door_state_callback_;
    SafetyCallback safety_callback_;
    FaultCallback fault_callback_;
    CallCallback call_callback_;
};

} // namespace CiA417
