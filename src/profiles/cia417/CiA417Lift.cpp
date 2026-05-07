/**
 * @file CiA417Lift.cpp
 * @brief CiA 417 Lift Controller Implementation
 */

#include "profiles/cia417/CiA417Lift.hpp"
#include "tether/platform/EspCompat.hpp"
#include <cstring>

#define LOG_TAG "CiA417"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)

extern "C" {
    bool ecm_sdo_read(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                      void* data, size_t len, bool use_configured_addr);
    bool ecm_sdo_write(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                       const void* data, size_t len, bool use_configured_addr);
}

namespace CiA417 {

// ============================================================================
// Construction
// ============================================================================

LiftController::LiftController(uint16_t slave_addr, bool use_configured_addr)
    : slave_addr_(slave_addr)
    , use_configured_addr_(use_configured_addr)
    , initialized_(false)
    , prev_statusword_(0)
    , prev_floor_(0xFF)
    , prev_door_state_main_(0xFF)
    , prev_door_state_counter_(0xFF)
    , controlword_(0)
    , target_floor_(0)
    , door_command_(DoorCommand::None)
    , current_mapping_(PDOMappingPreset::Extended)
{
    std::memset(&spec_, 0, sizeof(spec_));
    std::memset(&state_, 0, sizeof(state_));
    std::memset(&calls_, 0, sizeof(calls_));
}

LiftController::~LiftController() = default;

// ============================================================================
// Initialization
// ============================================================================

bool LiftController::initialize() {
    if (initialized_) return true;
    
    LOGI("Initializing CiA 417 lift controller for slave %u", slave_addr_);
    
    if (!readSpec()) {
        LOGE("Failed to read lift specifications");
        return false;
    }
    
    update();
    initialized_ = true;
    
    LOGI("Lift initialized: %u floors, %u mm/s, %u kg",
         spec_.num_floors, spec_.nominal_speed, spec_.nominal_load);
    
    return true;
}

bool LiftController::readSpec() {
    readSDO(Object::LiftType, 0, &spec_.lift_type, sizeof(spec_.lift_type));
    readSDO(Object::NumberOfFloors, 0, &spec_.num_floors, sizeof(spec_.num_floors));
    readSDO(Object::NominalSpeed, 0, &spec_.nominal_speed, sizeof(spec_.nominal_speed));
    readSDO(Object::NominalLoad, 0, &spec_.nominal_load, sizeof(spec_.nominal_load));
    readSDO(Object::NumberOfDoors, 0, &spec_.num_doors, sizeof(spec_.num_doors));
    readSDO(Object::DriveType, 0, &spec_.drive_type, sizeof(spec_.drive_type));
    readSDO(Object::SafetyClass, 0, &spec_.safety_class, sizeof(spec_.safety_class));
    return true;
}

// ============================================================================
// PDO
// ============================================================================

bool LiftController::applyPDOMapping(PDOMappingPreset preset) {
    current_mapping_ = preset;
    return true;
}

void LiftController::processTxPDO(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Basic:
            processBasicPDO(data, len);
            break;
        case PDOMappingPreset::Extended:
            processExtendedPDO(data, len);
            break;
        case PDOMappingPreset::Full:
            processFullPDO(data, len);
            break;
        default:
            processExtendedPDO(data, len);
            break;
    }
    
    checkStateChanges();
}

void LiftController::processBasicPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Basic)) return;
    const auto* pdo = reinterpret_cast<const TxPDO_Basic*>(data);
    state_.statusword = pdo->statusword;
    state_.position = pdo->position_actual;
    state_.floor_actual = pdo->floor_actual;
    state_.door_state_main = pdo->door_state;
}

void LiftController::processExtendedPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Extended)) return;
    // Use memcpy instead of reinterpret_cast for safety
    TxPDO_Extended pdo;
    std::memcpy(&pdo, data, sizeof(pdo));
    state_.statusword = pdo.statusword;
    state_.position = pdo.position_actual;
    state_.velocity = pdo.velocity_actual;
    state_.floor_actual = pdo.floor_actual;
    state_.floor_target = pdo.floor_target;
    state_.door_state_main = pdo.door_state_main;
    state_.door_state_counter = pdo.door_state_counter;
    state_.load_actual = pdo.load_actual;
    state_.safety_inputs = pdo.safety_inputs;
}

void LiftController::processFullPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Full)) return;
    // Use memcpy instead of reinterpret_cast for safety
    TxPDO_Full pdo;
    std::memcpy(&pdo, data, sizeof(pdo));
    state_.statusword = pdo.statusword;
    state_.position = pdo.position_actual;
    state_.velocity = pdo.velocity_actual;
    state_.floor_actual = pdo.floor_actual;
    state_.floor_target = pdo.floor_target;
    state_.motion_state = pdo.motion_state;
    state_.operating_mode = pdo.operating_mode;
    state_.door_state_main = pdo.door_state_main;
    state_.door_state_counter = pdo.door_state_counter;
    state_.load_actual = pdo.load_actual;
    state_.safety_inputs = pdo.safety_inputs;
    calls_.car_calls = pdo.car_calls;
    calls_.hall_calls_up = pdo.hall_calls_up;
    calls_.hall_calls_down = pdo.hall_calls_down;
    state_.fault_code = pdo.fault_code;
}

size_t LiftController::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!data || max_len < sizeof(RxPDO_Basic)) return 0;
    
    auto* pdo = reinterpret_cast<RxPDO_Basic*>(data);
    pdo->controlword = controlword_;
    pdo->floor_target = target_floor_;
    pdo->door_command = door_command_;
    door_command_ = DoorCommand::None;  // Clear after sending
    return sizeof(RxPDO_Basic);
}

void LiftController::update() {
    readSDO(Object::Statusword, 0, &state_.statusword, sizeof(state_.statusword));
    readSDO(Object::PositionActual, 0, &state_.position, sizeof(state_.position));
    readSDO(Object::VelocityActual, 0, &state_.velocity, sizeof(state_.velocity));
    readSDO(Object::FloorActual, 0, &state_.floor_actual, sizeof(state_.floor_actual));
    readSDO(Object::FloorTarget, 0, &state_.floor_target, sizeof(state_.floor_target));
    readSDO(Object::MotionState, 0, &state_.motion_state, sizeof(state_.motion_state));
    readSDO(Object::OperatingMode, 0, &state_.operating_mode, sizeof(state_.operating_mode));
    readSDO(Object::DoorStatusMain, 0, &state_.door_state_main, sizeof(state_.door_state_main));
    readSDO(Object::DoorStatusCounter, 0, &state_.door_state_counter, sizeof(state_.door_state_counter));
    readSDO(Object::LoadActual, 0, &state_.load_actual, sizeof(state_.load_actual));
    readSDO(Object::SafetyInputs, 0, &state_.safety_inputs, sizeof(state_.safety_inputs));
    readSDO(Object::FaultCode, 0, &state_.fault_code, sizeof(state_.fault_code));
    
    checkStateChanges();
}

void LiftController::checkStateChanges() {
    // Floor change
    if (state_.floor_actual != prev_floor_ && state_.isAtFloor()) {
        if (floor_reached_callback_) {
            floor_reached_callback_(state_.floor_actual);
        }
        prev_floor_ = state_.floor_actual;
    }
    
    // Door state change
    if (state_.door_state_main != prev_door_state_main_) {
        if (door_state_callback_) {
            door_state_callback_(0, state_.door_state_main);
        }
        prev_door_state_main_ = state_.door_state_main;
    }
    
    if (state_.door_state_counter != prev_door_state_counter_) {
        if (door_state_callback_) {
            door_state_callback_(1, state_.door_state_counter);
        }
        prev_door_state_counter_ = state_.door_state_counter;
    }
    
    // Fault detection
    if (state_.statusword != prev_statusword_) {
        bool was_faulted = prev_statusword_ & StatuswordBits::Fault;
        bool is_faulted = state_.statusword & StatuswordBits::Fault;
        if (is_faulted && !was_faulted && fault_callback_) {
            fault_callback_(state_.fault_code);
        }
        
        // Safety change
        if ((state_.safety_inputs ^ (prev_statusword_ >> 16)) && safety_callback_) {
            safety_callback_(state_.safety_inputs);
        }
        
        prev_statusword_ = state_.statusword;
    }
}

// ============================================================================
// Basic Control
// ============================================================================

bool LiftController::enable() {
    controlword_ |= ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::disable() {
    controlword_ &= ~ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::resetFault() {
    uint32_t cw = controlword_ | ControlwordBits::ResetFault;
    if (!writeSDO(Object::Controlword, 0, &cw, sizeof(cw))) return false;
    controlword_ &= ~ControlwordBits::ResetFault;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::emergencyStop() {
    uint8_t estop = 1;
    return writeSDO(Object::EmergencyStop, 0, &estop, sizeof(estop));
}

// ============================================================================
// Motion Control
// ============================================================================

bool LiftController::goToFloor(uint8_t floor) {
    if (floor >= spec_.num_floors) return false;
    target_floor_ = floor;
    controlword_ |= ControlwordBits::Start;
    return writeSDO(Object::FloorTarget, 0, &floor, sizeof(floor));
}

bool LiftController::moveUp() {
    controlword_ |= ControlwordBits::GoUp;
    controlword_ &= ~ControlwordBits::GoDown;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::moveDown() {
    controlword_ |= ControlwordBits::GoDown;
    controlword_ &= ~ControlwordBits::GoUp;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::stop() {
    controlword_ |= ControlwordBits::Stop;
    controlword_ &= ~(ControlwordBits::GoUp | ControlwordBits::GoDown | ControlwordBits::Start);
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool LiftController::relevel() {
    controlword_ |= ControlwordBits::RelevelEnable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

uint8_t LiftController::getDirection() const {
    if (state_.isDirectionUp()) return Direction::Up;
    if (state_.isDirectionDown()) return Direction::Down;
    return Direction::None;
}

// ============================================================================
// Operating Mode
// ============================================================================

bool LiftController::setOperatingMode(uint8_t mode) {
    return writeSDO(Object::OperatingMode, 0, &mode, sizeof(mode));
}

bool LiftController::enterInspectionMode() {
    controlword_ |= ControlwordBits::InspectionEnable;
    return setOperatingMode(OperatingMode::Inspection);
}

bool LiftController::exitInspectionMode() {
    controlword_ &= ~ControlwordBits::InspectionEnable;
    return setOperatingMode(OperatingMode::Normal);
}

bool LiftController::enterFireServiceMode() {
    controlword_ |= ControlwordBits::FireServiceMode;
    return setOperatingMode(OperatingMode::FireService);
}

bool LiftController::enterRescueMode() {
    controlword_ |= ControlwordBits::RescueMode;
    return setOperatingMode(OperatingMode::Rescue);
}

bool LiftController::enterMaintenanceMode() {
    return setOperatingMode(OperatingMode::Maintenance);
}

// ============================================================================
// Door Control
// ============================================================================

bool LiftController::openDoor(uint8_t door_id) {
    door_command_ = DoorCommand::Open;
    uint8_t cmd = DoorCommand::Open;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

bool LiftController::closeDoor(uint8_t door_id) {
    door_command_ = DoorCommand::Close;
    uint8_t cmd = DoorCommand::Close;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

bool LiftController::stopDoor(uint8_t door_id) {
    door_command_ = DoorCommand::Stop;
    uint8_t cmd = DoorCommand::Stop;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

bool LiftController::nudgeDoor(uint8_t door_id) {
    door_command_ = DoorCommand::Nudge;
    uint8_t cmd = DoorCommand::Nudge;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

bool LiftController::lockDoor(uint8_t door_id) {
    uint8_t cmd = DoorCommand::Lock;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

bool LiftController::unlockDoor(uint8_t door_id) {
    uint8_t cmd = DoorCommand::Unlock;
    return writeSDO(Object::DoorCommand, door_id + 1, &cmd, sizeof(cmd));
}

uint8_t LiftController::getDoorState(uint8_t door_id) const {
    return door_id == 0 ? state_.door_state_main : state_.door_state_counter;
}

bool LiftController::isDoorOpen(uint8_t door_id) const {
    return getDoorState(door_id) == DoorState::FullyOpen;
}

bool LiftController::isDoorClosed(uint8_t door_id) const {
    return getDoorState(door_id) == DoorState::FullyClosed;
}

bool LiftController::isDoorLocked(uint8_t door_id) const {
    return getDoorState(door_id) == DoorState::Locked;
}

bool LiftController::setDoorTiming(uint16_t open_time, uint16_t close_time, 
                                   uint16_t nudging_delay) {
    return writeSDO(Object::DoorOpenTime, 0, &open_time, sizeof(open_time)) &&
           writeSDO(Object::DoorCloseTime, 0, &close_time, sizeof(close_time)) &&
           writeSDO(Object::DoorNudgingDelay, 0, &nudging_delay, sizeof(nudging_delay));
}

bool LiftController::setDoorForce(uint8_t force_percent) {
    return writeSDO(Object::DoorForce, 0, &force_percent, sizeof(force_percent));
}

// ============================================================================
// Call Management
// ============================================================================

bool LiftController::registerCarCall(uint8_t floor) {
    if (floor >= 64) return false;
    calls_.car_calls |= (1ULL << floor);
    return writeSDO(Object::CarCallsUp, 0, &calls_.car_calls, sizeof(calls_.car_calls));
}

bool LiftController::cancelCarCall(uint8_t floor) {
    if (floor >= 64) return false;
    calls_.car_calls &= ~(1ULL << floor);
    return writeSDO(Object::CarCallsUp, 0, &calls_.car_calls, sizeof(calls_.car_calls));
}

bool LiftController::registerHallCall(uint8_t floor, uint8_t direction) {
    if (floor >= 64) return false;
    if (direction == Direction::Up) {
        calls_.hall_calls_up |= (1ULL << floor);
        return writeSDO(Object::HallCallsUp, 0, &calls_.hall_calls_up, sizeof(calls_.hall_calls_up));
    } else {
        calls_.hall_calls_down |= (1ULL << floor);
        return writeSDO(Object::HallCallsDown, 0, &calls_.hall_calls_down, sizeof(calls_.hall_calls_down));
    }
}

bool LiftController::cancelHallCall(uint8_t floor, uint8_t direction) {
    if (floor >= 64) return false;
    if (direction == Direction::Up) {
        calls_.hall_calls_up &= ~(1ULL << floor);
        return writeSDO(Object::HallCallsUp, 0, &calls_.hall_calls_up, sizeof(calls_.hall_calls_up));
    } else {
        calls_.hall_calls_down &= ~(1ULL << floor);
        return writeSDO(Object::HallCallsDown, 0, &calls_.hall_calls_down, sizeof(calls_.hall_calls_down));
    }
}

bool LiftController::clearAllCalls() {
    calls_.car_calls = 0;
    calls_.hall_calls_up = 0;
    calls_.hall_calls_down = 0;
    return writeSDO(Object::CarCallsUp, 0, &calls_.car_calls, sizeof(calls_.car_calls)) &&
           writeSDO(Object::HallCallsUp, 0, &calls_.hall_calls_up, sizeof(calls_.hall_calls_up)) &&
           writeSDO(Object::HallCallsDown, 0, &calls_.hall_calls_down, sizeof(calls_.hall_calls_down));
}

bool LiftController::isCarCallActive(uint8_t floor) const {
    return floor < 64 && (calls_.car_calls & (1ULL << floor));
}

bool LiftController::isHallCallActive(uint8_t floor, uint8_t direction) const {
    if (floor >= 64) return false;
    return direction == Direction::Up ? 
           (calls_.hall_calls_up & (1ULL << floor)) :
           (calls_.hall_calls_down & (1ULL << floor));
}

// ============================================================================
// Floor Configuration
// ============================================================================

bool LiftController::setFloorHeight(uint8_t floor, int32_t height_mm) {
    return writeSDO(Object::FloorHeightTable, floor + 1, &height_mm, sizeof(height_mm));
}

int32_t LiftController::getFloorHeight(uint8_t floor) {
    int32_t height;
    if (readSDO(Object::FloorHeightTable, floor + 1, &height, sizeof(height))) {
        return height;
    }
    return 0;
}

bool LiftController::enableFloor(uint8_t floor, bool enable) {
    uint64_t mask;
    if (!readSDO(Object::FloorEnabled, 0, &mask, sizeof(mask))) return false;
    if (enable) {
        mask |= (1ULL << floor);
    } else {
        mask &= ~(1ULL << floor);
    }
    return writeSDO(Object::FloorEnabled, 0, &mask, sizeof(mask));
}

bool LiftController::lockFloor(uint8_t floor, bool lock) {
    uint64_t mask;
    if (!readSDO(Object::FloorLocked, 0, &mask, sizeof(mask))) return false;
    if (lock) {
        mask |= (1ULL << floor);
    } else {
        mask &= ~(1ULL << floor);
    }
    return writeSDO(Object::FloorLocked, 0, &mask, sizeof(mask));
}

bool LiftController::isFloorEnabled(uint8_t floor) const {
    uint64_t mask;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::FloorEnabled, 0, &mask, sizeof(mask))) {
        return mask & (1ULL << floor);
    }
    return false;
}

bool LiftController::isFloorLocked(uint8_t floor) const {
    uint64_t mask;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::FloorLocked, 0, &mask, sizeof(mask))) {
        return mask & (1ULL << floor);
    }
    return false;
}

// ============================================================================
// Load Management
// ============================================================================

float LiftController::getLoadPercent() const {
    if (spec_.nominal_load == 0) return 0.0f;
    return (state_.load_actual * 100.0f) / spec_.nominal_load;
}

bool LiftController::setOverloadThreshold(uint16_t threshold_kg) {
    return writeSDO(Object::OverloadThreshold, 0, &threshold_kg, sizeof(threshold_kg));
}

bool LiftController::setBypassThreshold(uint16_t threshold_kg) {
    return writeSDO(Object::BypassThreshold, 0, &threshold_kg, sizeof(threshold_kg));
}

// ============================================================================
// Safety Systems
// ============================================================================

SafetyStatus LiftController::getSafetyStatus() {
    SafetyStatus status;
    status.safety_inputs = state_.safety_inputs;
    status.governor_ok = state_.safety_inputs & SafetyInputBits::GovernorOK;
    status.safety_circuit_ok = state_.safety_inputs & SafetyInputBits::SafetyCircuitOK;
    status.final_limits_ok = (state_.safety_inputs & SafetyInputBits::UpperFinalLimit) &&
                             (state_.safety_inputs & SafetyInputBits::LowerFinalLimit);
    status.buffer_ok = state_.safety_inputs & SafetyInputBits::BufferSwitch;
    status.door_locks_ok = (state_.safety_inputs & SafetyInputBits::DoorLockMain) &&
                           (state_.safety_inputs & SafetyInputBits::DoorLockCounter);
    readSDO(Object::OverspeedStatus, 0, &status.overspeed_status, sizeof(status.overspeed_status));
    readSDO(Object::UCMStatus, 0, &status.ucm_status, sizeof(status.ucm_status));
    status.emergency_stop_active = !(state_.safety_inputs & SafetyInputBits::EmergencyStopCar);
    return status;
}

bool LiftController::isSafetyInputActive(uint32_t input_bit) const {
    return state_.safety_inputs & input_bit;
}

bool LiftController::isGovernorOK() const {
    return state_.safety_inputs & SafetyInputBits::GovernorOK;
}

bool LiftController::isSafetyCircuitOK() const {
    return state_.safety_inputs & SafetyInputBits::SafetyCircuitOK;
}

bool LiftController::areDoorLocksEngaged() const {
    return (state_.safety_inputs & SafetyInputBits::DoorLockMain) != 0;
}

bool LiftController::isEmergencyStopActive() const {
    return !(state_.safety_inputs & SafetyInputBits::EmergencyStopCar);
}

// ============================================================================
// Brake Control
// ============================================================================

bool LiftController::applyBrakes() {
    uint8_t cmd = 1;
    return writeSDO(Object::BrakeCommand, 0, &cmd, sizeof(cmd));
}

bool LiftController::releaseBrakes() {
    uint8_t cmd = 2;
    return writeSDO(Object::BrakeCommand, 0, &cmd, sizeof(cmd));
}

bool LiftController::testBrakes() {
    uint8_t cmd = 3;
    return writeSDO(Object::BrakeCommand, 0, &cmd, sizeof(cmd));
}

uint8_t LiftController::getBrakeStatus() {
    uint8_t status;
    if (readSDO(Object::BrakeStatus, 0, &status, sizeof(status))) {
        return status;
    }
    return 0xFF;
}

// ============================================================================
// Inspection Control
// ============================================================================

bool LiftController::inspectionMoveUp() {
    uint8_t cmd = 1;
    return writeSDO(Object::InspectionControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::inspectionMoveDown() {
    uint8_t cmd = 2;
    return writeSDO(Object::InspectionControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::inspectionStop() {
    uint8_t cmd = 0;
    return writeSDO(Object::InspectionControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::carTopMoveUp() {
    uint8_t cmd = 1;
    return writeSDO(Object::CarTopControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::carTopMoveDown() {
    uint8_t cmd = 2;
    return writeSDO(Object::CarTopControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::pitMoveUp() {
    uint8_t cmd = 1;
    return writeSDO(Object::PitControl, 0, &cmd, sizeof(cmd));
}

bool LiftController::pitMoveDown() {
    uint8_t cmd = 2;
    return writeSDO(Object::PitControl, 0, &cmd, sizeof(cmd));
}

// ============================================================================
// Diagnostics
// ============================================================================

LiftDiagnostics LiftController::getDiagnostics() {
    LiftDiagnostics diag;
    readSDO(Object::TripCount, 0, &diag.trip_count, sizeof(diag.trip_count));
    readSDO(Object::OperatingHours, 0, &diag.operating_hours, sizeof(diag.operating_hours));
    readSDO(Object::DoorCycles, 0, &diag.door_cycles, sizeof(diag.door_cycles));
    readSDO(Object::EnergyConsumption, 0, &diag.energy_kwh, sizeof(diag.energy_kwh));
    readSDO(Object::DriveTemperature, 0, &diag.drive_temperature, sizeof(diag.drive_temperature));
    readSDO(Object::MotorTemperature, 0, &diag.motor_temperature, sizeof(diag.motor_temperature));
    return diag;
}

std::string LiftController::getDiagnosticsString() const {
    std::string result;
    result += "Lift Status:\n";
    result += "  Floor: " + std::to_string(state_.floor_actual) + "\n";
    result += "  Position: " + std::to_string(state_.position) + " mm\n";
    result += "  Velocity: " + std::to_string(state_.velocity) + " mm/s\n";
    result += "  Load: " + std::to_string(state_.load_actual) + " kg\n";
    result += "  Safety OK: " + std::string(state_.isSafetyOK() ? "Yes" : "No") + "\n";
    if (state_.fault_code != 0) {
        result += "  Fault: 0x" + std::to_string(state_.fault_code) + "\n";
    }
    return result;
}

float LiftController::getDriveTemperature() const {
    int16_t temp;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::DriveTemperature, 0, &temp, sizeof(temp))) {
        return rawToTempC(temp);
    }
    return 0.0f;
}

float LiftController::getMotorTemperature() const {
    int16_t temp;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::MotorTemperature, 0, &temp, sizeof(temp))) {
        return rawToTempC(temp);
    }
    return 0.0f;
}

uint32_t LiftController::getTripCount() const {
    uint32_t count;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::TripCount, 0, &count, sizeof(count))) {
        return count;
    }
    return 0;
}

uint32_t LiftController::getOperatingHours() const {
    uint32_t hours;
    LiftController* self = const_cast<LiftController*>(this);
    if (self->readSDO(Object::OperatingHours, 0, &hours, sizeof(hours))) {
        return hours;
    }
    return 0;
}

// ============================================================================
// Display
// ============================================================================

bool LiftController::setDisplayFloor(uint8_t floor) {
    return writeSDO(Object::DisplayFloor, 0, &floor, sizeof(floor));
}

bool LiftController::setDisplayDirection(uint8_t direction) {
    return writeSDO(Object::DisplayDirection, 0, &direction, sizeof(direction));
}

bool LiftController::setDisplayMessage(const std::string& message) {
    return writeSDO(Object::DisplayMessage, 0, message.c_str(), message.size() + 1);
}

bool LiftController::playAnnouncement(uint8_t announcement_id) {
    return writeSDO(Object::VoiceAnnouncement, 0, &announcement_id, sizeof(announcement_id));
}

// ============================================================================
// Callbacks
// ============================================================================

void LiftController::setFloorReachedCallback(FloorReachedCallback callback) {
    floor_reached_callback_ = std::move(callback);
}

void LiftController::setDoorStateCallback(DoorStateCallback callback) {
    door_state_callback_ = std::move(callback);
}

void LiftController::setSafetyCallback(SafetyCallback callback) {
    safety_callback_ = std::move(callback);
}

void LiftController::setFaultCallback(FaultCallback callback) {
    fault_callback_ = std::move(callback);
}

void LiftController::setCallCallback(CallCallback callback) {
    call_callback_ = std::move(callback);
}

// ============================================================================
// SDO Helpers
// ============================================================================

bool LiftController::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    return ecm_sdo_read(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

bool LiftController::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    return ecm_sdo_write(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

} // namespace CiA417
