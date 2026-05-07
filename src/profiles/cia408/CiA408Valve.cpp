/**
 * @file CiA408Valve.cpp
 * @brief CiA 408 Fluid Power Valve Controller Implementation
 */

#include "profiles/cia408/CiA408Valve.hpp"
#include "tether/platform/EspCompat.hpp"
#include <cstring>
#include <cmath>

#ifndef UNIT_TEST_HOST
#define LOG_TAG "CiA408"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI(fmt, ...)
#define LOGW(fmt, ...)
#define LOGE(fmt, ...)
#endif

// External SDO functions (defined in EtherCAT master)
extern "C" {
    bool ecm_sdo_read(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                      void* data, size_t len, bool use_configured_addr);
    bool ecm_sdo_write(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                       const void* data, size_t len, bool use_configured_addr);
}

namespace CiA408 {

// ============================================================================
// Construction
// ============================================================================

ValveController::ValveController(uint16_t slave_addr, bool use_configured_addr)
    : slave_addr_(slave_addr)
    , use_configured_addr_(use_configured_addr)
    , initialized_(false)
    , current_mapping_(PDOMappingPreset::Basic)
    , prev_statusword_(0)
    , controlword_(0)
    , target_setpoint_(0)
    , target_position_(0)
    , target_velocity_(0)
    , target_pressure_(0)
    , current_mode_(0)
{
    std::memset(&capabilities_, 0, sizeof(capabilities_));
    std::memset(&valve_spec_, 0, sizeof(valve_spec_));
    std::memset(&state_, 0, sizeof(state_));
    std::memset(&dither_config_, 0, sizeof(dither_config_));
    std::memset(&controller_params_, 0, sizeof(controller_params_));
}

ValveController::~ValveController() = default;

// ============================================================================
// Initialization
// ============================================================================

bool ValveController::initialize() {
    if (initialized_) {
        return true;
    }
    
    LOGI("Initializing CiA 408 valve controller for slave %u", slave_addr_);
    
    // Detect capabilities
    if (!detectCapabilities()) {
        LOGE("Failed to detect valve capabilities");
        return false;
    }
    
    // Read valve specifications
    if (!readValveSpec()) {
        LOGW("Could not read full valve specifications");
    }
    
    // Read initial state
    update();
    
    initialized_ = true;
    LOGI("Valve initialized: type=%u, flow=%u L/min, pressure=%u bar",
         valve_spec_.valve_type, valve_spec_.nominal_flow / 10,
         valve_spec_.nominal_pressure / 10);
    
    return true;
}

bool ValveController::detectCapabilities() {
    uint8_t valve_type;
    if (readSDO(Object::ValveType, 0, &valve_type, sizeof(valve_type))) {
        capabilities_.valve_type = valve_type;
    }
    
    // Check for position feedback
    int32_t position;
    capabilities_.has_position_feedback = 
        readSDO(Object::PositionActual, 0, &position, sizeof(position));
    
    // Check for pressure sensors
    int16_t pressure;
    capabilities_.has_pressure_sensors = 
        readSDO(Object::PressureA, 0, &pressure, sizeof(pressure));
    
    // Check for dual coils
    int16_t current_b;
    capabilities_.has_dual_coils = 
        readSDO(Object::CurrentActualB, 0, &current_b, sizeof(current_b));
    
    // Determine closed loop support
    capabilities_.supports_closed_loop = capabilities_.has_position_feedback;
    capabilities_.supports_pressure_control = capabilities_.has_pressure_sensors;
    capabilities_.supports_force_control = capabilities_.has_pressure_sensors;
    
    // Check dither support
    uint16_t dither_amp;
    capabilities_.supports_dither = 
        readSDO(Object::DitherAmplitude, 0, &dither_amp, sizeof(dither_amp));
    
    capabilities_.num_channels = capabilities_.has_dual_coils ? 2 : 1;
    
    return true;
}

bool ValveController::readValveSpec() {
    readSDO(Object::ValveType, 0, &valve_spec_.valve_type, sizeof(valve_spec_.valve_type));
    readSDO(Object::NominalFlow, 0, &valve_spec_.nominal_flow, sizeof(valve_spec_.nominal_flow));
    readSDO(Object::NominalPressure, 0, &valve_spec_.nominal_pressure, sizeof(valve_spec_.nominal_pressure));
    readSDO(Object::NominalStroke, 0, &valve_spec_.nominal_stroke, sizeof(valve_spec_.nominal_stroke));
    readSDO(Object::ResponseTime, 0, &valve_spec_.response_time, sizeof(valve_spec_.response_time));
    readSDO(Object::Hysteresis, 0, &valve_spec_.hysteresis, sizeof(valve_spec_.hysteresis));
    readSDO(Object::Repeatability, 0, &valve_spec_.repeatability, sizeof(valve_spec_.repeatability));
    return true;
}

// ============================================================================
// PDO Configuration
// ============================================================================

bool ValveController::applyPDOMapping(PDOMappingPreset preset) {
    current_mapping_ = preset;
    // PDO mapping is typically configured via SM configuration
    // This stores the preset for processing
    return true;
}

// ============================================================================
// Cyclic Update
// ============================================================================

void ValveController::processTxPDO(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Basic:
            processBasicPDO(data, len);
            break;
        case PDOMappingPreset::Extended:
            processExtendedPDO(data, len);
            break;
        case PDOMappingPreset::Position:
            processPositionPDO(data, len);
            break;
        case PDOMappingPreset::Full:
            processFullPDO(data, len);
            break;
        default:
            processBasicPDO(data, len);
            break;
    }
    
    checkStateChanges();
}

void ValveController::processBasicPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Basic)) return;
    
    const auto* pdo = reinterpret_cast<const TxPDO_Basic*>(data);
    state_.statusword = pdo->statusword;
    state_.actual_value = pdo->actual_value;
}

void ValveController::processExtendedPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Extended)) return;
    
    const auto* pdo = reinterpret_cast<const TxPDO_Extended*>(data);
    state_.statusword = pdo->statusword;
    state_.actual_value = pdo->actual_value;
    state_.pressure_a = pdo->pressure_a;
    state_.pressure_b = pdo->pressure_b;
    // Extended PDO does not include current fields in the current definition.
}


void ValveController::processPositionPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Position)) return;
    
    const auto* pdo = reinterpret_cast<const TxPDO_Position*>(data);
    state_.statusword = pdo->statusword;
    state_.actual_value = pdo->actual_value;
    state_.position_actual = pdo->position_actual;
    // Older position PDOs do not include velocity/following fields; clear them
    state_.velocity_actual = 0;
    state_.following_error = 0;
}

void ValveController::processFullPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Full)) return;
    
    const auto* pdo = reinterpret_cast<const TxPDO_Full*>(data);
    state_.statusword = pdo->statusword;
    state_.actual_value = pdo->actual_value;
    state_.pressure_a = pdo->pressure_a;
    state_.pressure_b = pdo->pressure_b;
    state_.current_a = pdo->current_a;
    state_.current_b = pdo->current_b;
    state_.temperature = pdo->temperature;
    state_.fault_code = pdo->fault_code;
    // Full mapping does not include position/velocity; those are updated via SDO if present
}

size_t ValveController::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!data) return 0;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Basic: {
            if (max_len < sizeof(RxPDO_Basic)) return 0;
            auto* pdo = reinterpret_cast<RxPDO_Basic*>(data);
            pdo->controlword = controlword_;
            pdo->setpoint = target_setpoint_;
            return sizeof(RxPDO_Basic);
        }
        
        case PDOMappingPreset::Extended: {
            if (max_len < sizeof(RxPDO_Extended)) return 0;
            auto* pdo = reinterpret_cast<RxPDO_Extended*>(data);
            pdo->controlword = controlword_;
            pdo->setpoint = target_setpoint_;
            pdo->operating_mode = current_mode_;
            return sizeof(RxPDO_Extended);
        }
        
        case PDOMappingPreset::Position: {
            if (max_len < sizeof(RxPDO_Position)) return 0;
            auto* pdo = reinterpret_cast<RxPDO_Position*>(data);
            pdo->controlword = controlword_;
            pdo->position_setpoint = target_position_;
            pdo->velocity_limit = static_cast<int16_t>(target_velocity_);
            return sizeof(RxPDO_Position);
        }
        
        case PDOMappingPreset::Full: {
            if (max_len < sizeof(RxPDO_Full)) return 0;
            auto* pdo = reinterpret_cast<RxPDO_Full*>(data);
            pdo->controlword = controlword_;
            pdo->setpoint = target_setpoint_;
            pdo->setpoint2 = 0; // No dedicated position setpoint in Full mapping
            pdo->operating_mode = current_mode_;
            pdo->override_value = 0;
            return sizeof(RxPDO_Full);
        }
        
        default:
            return 0;
    }
}

void ValveController::update() {
    // Read via SDO
    readSDO(Object::Statusword, 0, &state_.statusword, sizeof(state_.statusword));
    readSDO(Object::ActualValue, 0, &state_.actual_value, sizeof(state_.actual_value));
    
    if (capabilities_.has_position_feedback) {
        readSDO(Object::PositionActual, 0, &state_.position_actual, sizeof(state_.position_actual));
        readSDO(Object::VelocityActual, 0, &state_.velocity_actual, sizeof(state_.velocity_actual));
    }
    
    if (capabilities_.has_pressure_sensors) {
        readSDO(Object::PressureA, 0, &state_.pressure_a, sizeof(state_.pressure_a));
        readSDO(Object::PressureB, 0, &state_.pressure_b, sizeof(state_.pressure_b));
        readSDO(Object::PressureP, 0, &state_.pressure_p, sizeof(state_.pressure_p));
    }
    
    readSDO(Object::CurrentActualA, 0, &state_.current_a, sizeof(state_.current_a));
    if (capabilities_.has_dual_coils) {
        readSDO(Object::CurrentActualB, 0, &state_.current_b, sizeof(state_.current_b));
    }
    
    readSDO(Object::Temperature, 0, &state_.temperature, sizeof(state_.temperature));
    readSDO(Object::FaultCode, 0, &state_.fault_code, sizeof(state_.fault_code));
    readSDO(Object::WarningCode, 0, &state_.warning_code, sizeof(state_.warning_code));
    
    checkStateChanges();
}

void ValveController::checkStateChanges() {
    if (state_.statusword != prev_statusword_) {
        // State changed
        if (state_change_callback_) {
            state_change_callback_(prev_statusword_, state_.statusword);
        }
        
        // Check for new fault
        bool was_faulted = prev_statusword_ & StatuswordBits::Fault;
        bool is_faulted = state_.statusword & StatuswordBits::Fault;
        if (is_faulted && !was_faulted) {
            fireFault(state_.fault_code);
        }
        
        // Check for target reached
        bool was_reached = prev_statusword_ & StatuswordBits::TargetReached;
        bool is_reached = state_.statusword & StatuswordBits::TargetReached;
        if (is_reached && !was_reached && target_reached_callback_) {
            target_reached_callback_();
        }
        
        prev_statusword_ = state_.statusword;
    }
}

void ValveController::fireFault(uint16_t fault) {
    LOGE("Valve fault: 0x%04X", fault);
    if (fault_callback_) {
        fault_callback_(fault);
    }
}

// ============================================================================
// Basic Control
// ============================================================================

bool ValveController::enable() {
    controlword_ |= ControlwordBits::Enable;
    controlword_ &= ~ControlwordBits::FastStop;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool ValveController::disable() {
    controlword_ &= ~ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool ValveController::resetFault() {
    uint16_t cw = controlword_ | ControlwordBits::FaultReset;
    if (!writeSDO(Object::Controlword, 0, &cw, sizeof(cw))) {
        return false;
    }
    
    // Clear the reset bit
    controlword_ &= ~ControlwordBits::FaultReset;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool ValveController::fastStop() {
    controlword_ |= ControlwordBits::FastStop;
    controlword_ &= ~ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool ValveController::isEnabled() const {
    return state_.statusword & StatuswordBits::Enabled;
}

bool ValveController::hasFault() const {
    return state_.statusword & StatuswordBits::Fault;
}

// ============================================================================
// Operating Mode
// ============================================================================

bool ValveController::setOperatingMode(uint8_t mode) {
    if (!writeSDO(Object::OperatingMode, 0, &mode, sizeof(mode))) {
        return false;
    }
    current_mode_ = mode;
    return true;
}

bool ValveController::enableClosedLoop(bool enable) {
    if (enable) {
        controlword_ |= ControlwordBits::ClosedLoop;
    } else {
        controlword_ &= ~ControlwordBits::ClosedLoop;
    }
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool ValveController::enablePressureCompensation(bool enable) {
    if (enable) {
        controlword_ |= ControlwordBits::PressureCompensation;
    } else {
        controlword_ &= ~ControlwordBits::PressureCompensation;
    }
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

// ============================================================================
// Setpoint Control
// ============================================================================

bool ValveController::setSetpoint(float percent) {
    if (percent < -100.0f) percent = -100.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int16_t raw = static_cast<int16_t>(percent * 100.0f);
    return setSetpointRaw(raw);
}

bool ValveController::setSetpointRaw(int16_t value) {
    if (value < -10000) value = -10000;
    if (value > 10000) value = 10000;
    
    target_setpoint_ = value;
    return writeSDO(Object::Setpoint, 0, &target_setpoint_, sizeof(target_setpoint_));
}

float ValveController::getSetpoint() const {
    return target_setpoint_ / 100.0f;
}

int16_t ValveController::getSetpointRaw() const {
    return target_setpoint_;
}

float ValveController::getActualValue() const {
    return state_.actual_value / 100.0f;
}

int16_t ValveController::getActualValueRaw() const {
    return state_.actual_value;
}

bool ValveController::setSetpointRamp(float rate_percent_per_sec) {
    int16_t rate = static_cast<int16_t>(rate_percent_per_sec * 100.0f);
    return writeSDO(Object::SetpointRampUp, 0, &rate, sizeof(rate)) &&
           writeSDO(Object::SetpointRampDown, 0, &rate, sizeof(rate));
}

// ============================================================================
// Position Control
// ============================================================================

bool ValveController::setPosition(int32_t position_um) {
    if (!capabilities_.has_position_feedback) {
        LOGW("Valve does not support position control");
        return false;
    }
    
    target_position_ = position_um;
    return writeSDO(Object::PositionSetpoint, 0, &target_position_, sizeof(target_position_));
}

int32_t ValveController::getPosition() const {
    return state_.position_actual;
}

bool ValveController::setPositionWindow(int32_t window_um, uint16_t time_ms) {
    return writeSDO(Object::PositionWindow, 0, &window_um, sizeof(window_um)) &&
           writeSDO(Object::PositionWindowTime, 0, &time_ms, sizeof(time_ms));
}

bool ValveController::isInPosition() const {
    return state_.statusword & StatuswordBits::InPosition;
}

// ============================================================================
// Velocity Control
// ============================================================================

bool ValveController::setVelocity(float velocity_mm_s) {
    target_velocity_ = static_cast<int16_t>(velocity_mm_s * 10.0f);
    return writeSDO(Object::VelocitySetpoint, 0, &target_velocity_, sizeof(target_velocity_));
}

float ValveController::getVelocity() const {
    return state_.velocity_actual / 10.0f;
}

// ============================================================================
// Pressure Control
// ============================================================================

bool ValveController::setPressureSetpoint(float pressure_bar) {
    target_pressure_ = static_cast<int16_t>(pressure_bar * 10.0f);
    return writeSDO(Object::PressureSetpoint, 0, &target_pressure_, sizeof(target_pressure_));
}

float ValveController::getPressureA() const {
    return state_.pressure_a / 10.0f;
}

float ValveController::getPressureB() const {
    return state_.pressure_b / 10.0f;
}

float ValveController::getSupplyPressure() const {
    return state_.pressure_p / 10.0f;
}

float ValveController::getDifferentialPressure() const {
    return (state_.pressure_a - state_.pressure_b) / 10.0f;
}

bool ValveController::setMaxPressure(float pressure_bar) {
    int16_t value = static_cast<int16_t>(pressure_bar * 10.0f);
    return writeSDO(Object::PressureLimit, 0, &value, sizeof(value));
}

// ============================================================================
// Controller Tuning
// ============================================================================

bool ValveController::setPositionGains(int32_t kp, int32_t ki, int32_t kd) {
    controller_params_.pos_kp = kp;
    controller_params_.pos_ki = ki;
    controller_params_.pos_kd = kd;
    
    return writeSDO(Object::PositionKp, 0, &kp, sizeof(kp)) &&
           writeSDO(Object::PositionKi, 0, &ki, sizeof(ki)) &&
           writeSDO(Object::PositionKd, 0, &kd, sizeof(kd));
}

bool ValveController::setVelocityGains(int32_t kp, int32_t ki) {
    controller_params_.vel_kp = kp;
    controller_params_.vel_ki = ki;
    
    return writeSDO(Object::VelocityKp, 0, &kp, sizeof(kp)) &&
           writeSDO(Object::VelocityKi, 0, &ki, sizeof(ki));
}

bool ValveController::setPressureGains(int32_t kp, int32_t ki) {
    controller_params_.prs_kp = kp;
    controller_params_.prs_ki = ki;
    
    return writeSDO(Object::PressureKp, 0, &kp, sizeof(kp)) &&
           writeSDO(Object::PressureKi, 0, &ki, sizeof(ki));
}

ControllerParams ValveController::getControllerParams() {
    ControllerParams params;
    
    readSDO(Object::PositionKp, 0, &params.pos_kp, sizeof(params.pos_kp));
    readSDO(Object::PositionKi, 0, &params.pos_ki, sizeof(params.pos_ki));
    readSDO(Object::PositionKd, 0, &params.pos_kd, sizeof(params.pos_kd));
    readSDO(Object::PositionKv, 0, &params.pos_kv, sizeof(params.pos_kv));
    readSDO(Object::PositionKa, 0, &params.pos_ka, sizeof(params.pos_ka));
    readSDO(Object::PositionOutputLimit, 0, &params.pos_limit, sizeof(params.pos_limit));
    
    readSDO(Object::VelocityKp, 0, &params.vel_kp, sizeof(params.vel_kp));
    readSDO(Object::VelocityKi, 0, &params.vel_ki, sizeof(params.vel_ki));
    readSDO(Object::VelocityOutputLimit, 0, &params.vel_limit, sizeof(params.vel_limit));
    
    readSDO(Object::PressureKp, 0, &params.prs_kp, sizeof(params.prs_kp));
    readSDO(Object::PressureKi, 0, &params.prs_ki, sizeof(params.prs_ki));
    readSDO(Object::PressureOutputLimit, 0, &params.prs_limit, sizeof(params.prs_limit));
    
    return params;
}

bool ValveController::setControllerParams(const ControllerParams& params) {
    controller_params_ = params;
    
    bool success = true;
    success &= writeSDO(Object::PositionKp, 0, &params.pos_kp, sizeof(params.pos_kp));
    success &= writeSDO(Object::PositionKi, 0, &params.pos_ki, sizeof(params.pos_ki));
    success &= writeSDO(Object::PositionKd, 0, &params.pos_kd, sizeof(params.pos_kd));
    success &= writeSDO(Object::PositionKv, 0, &params.pos_kv, sizeof(params.pos_kv));
    success &= writeSDO(Object::PositionKa, 0, &params.pos_ka, sizeof(params.pos_ka));
    success &= writeSDO(Object::PositionOutputLimit, 0, &params.pos_limit, sizeof(params.pos_limit));
    
    success &= writeSDO(Object::VelocityKp, 0, &params.vel_kp, sizeof(params.vel_kp));
    success &= writeSDO(Object::VelocityKi, 0, &params.vel_ki, sizeof(params.vel_ki));
    success &= writeSDO(Object::VelocityOutputLimit, 0, &params.vel_limit, sizeof(params.vel_limit));
    
    success &= writeSDO(Object::PressureKp, 0, &params.prs_kp, sizeof(params.prs_kp));
    success &= writeSDO(Object::PressureKi, 0, &params.prs_ki, sizeof(params.prs_ki));
    success &= writeSDO(Object::PressureOutputLimit, 0, &params.prs_limit, sizeof(params.prs_limit));
    
    return success;
}

// ============================================================================
// Dither Control
// ============================================================================

bool ValveController::configureDither(uint16_t amplitude, uint16_t frequency_hz) {
    dither_config_.amplitude = amplitude;
    dither_config_.frequency = frequency_hz;
    
    return writeSDO(Object::DitherAmplitude, 0, &amplitude, sizeof(amplitude)) &&
           writeSDO(Object::DitherFrequency, 0, &frequency_hz, sizeof(frequency_hz));
}

bool ValveController::enableDither(bool enable) {
    dither_config_.enabled = enable;
    uint8_t val = enable ? 1 : 0;
    return writeSDO(Object::DitherEnable, 0, &val, sizeof(val));
}

// ============================================================================
// Limits
// ============================================================================

bool ValveController::setPositionLimits(int32_t min_um, int32_t max_um) {
    return writeSDO(Object::PositionMin, 0, &min_um, sizeof(min_um)) &&
           writeSDO(Object::PositionMax, 0, &max_um, sizeof(max_um));
}

bool ValveController::setVelocityLimits(float max_mm_s, float accel_mm_s2) {
    int16_t vel = static_cast<int16_t>(max_mm_s * 10.0f);
    int32_t accel = static_cast<int32_t>(accel_mm_s2 * 10.0f);
    
    return writeSDO(Object::VelocityMax, 0, &vel, sizeof(vel)) &&
           writeSDO(Object::Acceleration, 0, &accel, sizeof(accel));
}

bool ValveController::setCurrentLimits(uint16_t max_current_a_ma, uint16_t max_current_b_ma) {
    return writeSDO(Object::CurrentLimitA, 0, &max_current_a_ma, sizeof(max_current_a_ma)) &&
           writeSDO(Object::CurrentLimitB, 0, &max_current_b_ma, sizeof(max_current_b_ma));
}

// ============================================================================
// Diagnostics
// ============================================================================

float ValveController::getCoilCurrentA() const {
    return state_.current_a / 10.0f;
}

float ValveController::getCoilCurrentB() const {
    return state_.current_b / 10.0f;
}

float ValveController::getCoilTemperature() const {
    return state_.temperature / 10.0f;
}

float ValveController::getSupplyVoltage() const {
    uint16_t voltage;
    ValveController* self = const_cast<ValveController*>(this);
    if (self->readSDO(Object::SupplyVoltage, 0, &voltage, sizeof(voltage))) {
        return voltage / 100.0f;
    }
    return 0.0f;
}

float ValveController::getFollowingError() const {
    return state_.following_error / 100.0f;
}

uint32_t ValveController::getOperatingHours() const {
    uint32_t hours;
    ValveController* self = const_cast<ValveController*>(this);
    if (self->readSDO(Object::OperatingHours, 0, &hours, sizeof(hours))) {
        return hours;
    }
    return 0;
}

uint32_t ValveController::getCycleCount() const {
    uint32_t count;
    ValveController* self = const_cast<ValveController*>(this);
    if (self->readSDO(Object::CycleCount, 0, &count, sizeof(count))) {
        return count;
    }
    return 0;
}

std::string ValveController::getDiagnostics() const {
    std::string result;
    result += "Valve Status:\n";
    result += "  Enabled: " + std::string(isEnabled() ? "Yes" : "No") + "\n";
    result += "  Fault: " + std::string(state_.hasFault() ? "Yes" : "No") + "\n";
    result += "  Setpoint: " + std::to_string(getSetpoint()) + "%\n";
    result += "  Actual: " + std::to_string(getActualValue()) + "%\n";
    
    if (capabilities_.has_position_feedback) {
        result += "  Position: " + std::to_string(state_.position_actual) + " um\n";
        result += "  Velocity: " + std::to_string(state_.velocity_actual / 10.0f) + " mm/s\n";
    }
    
    if (capabilities_.has_pressure_sensors) {
        result += "  Pressure A: " + std::to_string(getPressureA()) + " bar\n";
        result += "  Pressure B: " + std::to_string(getPressureB()) + " bar\n";
    }
    
    result += "  Current A: " + std::to_string(getCoilCurrentA()) + " mA\n";
    if (capabilities_.has_dual_coils) {
        result += "  Current B: " + std::to_string(getCoilCurrentB()) + " mA\n";
    }
    
    result += "  Temperature: " + std::to_string(getCoilTemperature()) + " C\n";
    
    if (state_.fault_code != 0) {
        result += "  Fault code: 0x" + std::to_string(state_.fault_code) + "\n";
    }
    
    return result;
}

// ============================================================================
// Calibration
// ============================================================================

bool ValveController::startNullCalibration() {
    uint8_t cmd = 1;  // Start null calibration
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool ValveController::startGainCalibration() {
    uint8_t cmd = 2;  // Start gain calibration
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool ValveController::startAutoTune() {
    uint8_t cmd = 3;  // Start auto-tune
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool ValveController::storeCalibration() {
    uint8_t cmd = 4;  // Store
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool ValveController::resetCalibration() {
    uint8_t cmd = 5;  // Reset to factory
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

uint8_t ValveController::getCalibrationStatus() {
    uint8_t status;
    if (readSDO(Object::CalibrationStatus, 0, &status, sizeof(status))) {
        return status;
    }
    return 0xFF;
}

bool ValveController::setNullOffset(int16_t offset) {
    return writeSDO(Object::NullOffset, 0, &offset, sizeof(offset));
}

bool ValveController::setDeadband(uint16_t deadband) {
    return writeSDO(Object::Deadband, 0, &deadband, sizeof(deadband));
}

// ============================================================================
// Callbacks
// ============================================================================

void ValveController::setStateChangeCallback(StateChangeCallback callback) {
    state_change_callback_ = std::move(callback);
}

void ValveController::setFaultCallback(FaultCallback callback) {
    fault_callback_ = std::move(callback);
}

void ValveController::setTargetReachedCallback(TargetReachedCallback callback) {
    target_reached_callback_ = std::move(callback);
}

// ============================================================================
// SDO Helpers
// ============================================================================

bool ValveController::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    return ecm_sdo_read(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

bool ValveController::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    return ecm_sdo_write(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

} // namespace CiA408
