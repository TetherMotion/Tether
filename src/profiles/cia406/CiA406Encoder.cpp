/**
 * @file CiA406Encoder.cpp
 * @brief Implementation of CiA 406 Encoder Device Controller
 * 
 * @details
 * This file implements the CiA 406 encoder controller including:
 * - Encoder initialization and capability detection
 * - PDO mapping configuration
 * - Position, velocity, and acceleration calculations
 * - Scaling and offset operations
 * - Alarm and warning handling
 * - Reference/homing procedures
 * - Diagnostic readings
 * 
 * @see CiA406Encoder.hpp for API documentation
 * @see CiA406Defs.hpp for object dictionary definitions
 */

#include "profiles/cia406/CiA406Encoder.hpp"
#include "profiles/cia406/CiA406Defs.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/SDOManager.hpp"

#include <cstring>
#include <algorithm>

namespace CiA406 {

static const char* TAG = "CiA406";

// ============================================================================
// Utility Function Implementations
// ============================================================================

const char* getEncoderClassName(EncoderClassEx cls) {
    switch (cls) {
        case EncoderClassEx::C1_AbsoluteSingleTurn: return "C1 (Absolute Single-Turn)";
        case EncoderClassEx::C2_AbsoluteMultiTurn:  return "C2 (Absolute Multi-Turn)";
        case EncoderClassEx::C3_Incremental:        return "C3 (Incremental)";
        case EncoderClassEx::C4_IncrementalWithLimits: return "C4 (Incremental with Limits)";
        default: return "Unknown";
    }
}

const char* getInterfaceTypeName(InterfaceType type) {
    switch (type) {
        case InterfaceType::Parallel:    return "Parallel";
        case InterfaceType::SSI:         return "SSI";
        case InterfaceType::BiSS_C:      return "BiSS-C";
        case InterfaceType::BiSS_B:      return "BiSS-B";
        case InterfaceType::EnDat21:     return "EnDat 2.1";
        case InterfaceType::EnDat22:     return "EnDat 2.2";
        case InterfaceType::SinCos_1Vpp: return "SinCos 1Vpp";
        case InterfaceType::TTL_RS422:   return "TTL/RS422";
        case InterfaceType::Hiperface:   return "Hiperface";
        case InterfaceType::DRIVE_CLiQ:  return "DRIVE-CLiQ";
        case InterfaceType::Tamagawa:    return "Tamagawa";
        default: return "Unknown";
    }
}

// ============================================================================
// OperatingStatus Implementation
// ============================================================================

OperatingStatus OperatingStatus::fromRaw(uint16_t raw) {
    OperatingStatus status;
    status.position_valid          = (raw & OperatingStatusBits::PositionValid) != 0;
    status.scaling_active          = (raw & OperatingStatusBits::ScalingActive) != 0;
    status.reference_done          = (raw & OperatingStatusBits::ReferenceDone) != 0;
    status.preset_executed         = (raw & OperatingStatusBits::PresetExecuted) != 0;
    status.overspeed_warning       = (raw & OperatingStatusBits::OverspeedWarning) != 0;
    status.counting_range_exceeded = (raw & OperatingStatusBits::CountingRangeExceeded) != 0;
    status.supply_voltage_low      = (raw & OperatingStatusBits::SupplyVoltageLow) != 0;
    status.supply_voltage_high     = (raw & OperatingStatusBits::SupplyVoltageHigh) != 0;
    return status;
}

uint16_t OperatingStatus::toRaw() const {
    uint16_t raw = 0;
    if (position_valid)          raw |= OperatingStatusBits::PositionValid;
    if (scaling_active)          raw |= OperatingStatusBits::ScalingActive;
    if (reference_done)          raw |= OperatingStatusBits::ReferenceDone;
    if (preset_executed)         raw |= OperatingStatusBits::PresetExecuted;
    if (overspeed_warning)       raw |= OperatingStatusBits::OverspeedWarning;
    if (counting_range_exceeded) raw |= OperatingStatusBits::CountingRangeExceeded;
    if (supply_voltage_low)      raw |= OperatingStatusBits::SupplyVoltageLow;
    if (supply_voltage_high)     raw |= OperatingStatusBits::SupplyVoltageHigh;
    return raw;
}

// ============================================================================
// AlarmFlags Implementation
// ============================================================================

AlarmFlags AlarmFlags::fromRaw(uint16_t raw) {
    AlarmFlags flags;
    flags.hardware_error          = (raw & AlarmBits::HardwareError) != 0;
    flags.temperature_exceeded    = (raw & AlarmBits::TemperatureExceeded) != 0;
    flags.light_source_error      = (raw & AlarmBits::LightSourceError) != 0;
    flags.battery_error           = (raw & AlarmBits::BatteryError) != 0;
    flags.position_error          = (raw & AlarmBits::PositionError) != 0;
    flags.communication_error     = (raw & AlarmBits::CommunicationError) != 0;
    flags.multi_turn_error        = (raw & AlarmBits::MultiTurnError) != 0;
    flags.initialization_error    = (raw & AlarmBits::InitializationError) != 0;
    flags.speed_exceeded          = (raw & AlarmBits::SpeedExceeded) != 0;
    flags.position_limit_exceeded = (raw & AlarmBits::PositionLimitExceeded) != 0;
    return flags;
}

bool AlarmFlags::hasAnyAlarm() const {
    return hardware_error || temperature_exceeded || light_source_error ||
           battery_error || position_error || communication_error ||
           multi_turn_error || initialization_error || speed_exceeded ||
           position_limit_exceeded;
}

std::string AlarmFlags::getDescription() const {
    std::string desc;
    if (hardware_error) desc += "Hardware Error, ";
    if (temperature_exceeded) desc += "Over Temperature, ";
    if (light_source_error) desc += "Light Source Error, ";
    if (battery_error) desc += "Battery Error, ";
    if (position_error) desc += "Position Error, ";
    if (communication_error) desc += "Communication Error, ";
    if (multi_turn_error) desc += "Multi-turn Error, ";
    if (initialization_error) desc += "Init Error, ";
    if (speed_exceeded) desc += "Over Speed, ";
    if (position_limit_exceeded) desc += "Position Limit, ";
    
    if (!desc.empty()) {
        // Remove trailing ", "
        desc = desc.substr(0, desc.length() - 2);
    } else {
        desc = "No alarms";
    }
    return desc;
}

// ============================================================================
// Encoder Class Implementation
// ============================================================================

Encoder::Encoder(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_position)
    : m_sdo(sdo)
    , slave_addr_(slave_position)
    , use_configured_addr_(false)
{
}

Encoder::Encoder(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_address, bool use_configured_addr)
    : m_sdo(sdo)
    , slave_addr_(slave_address)
    , use_configured_addr_(use_configured_addr)
{
}

bool Encoder::initialize() {
    TETHER_LOGI(TAG, "Initializing CiA 406 encoder at slave %u", slave_addr_);
    
    // Verify device type
    uint32_t device_type = 0;
    size_t len;
    
    if (!readObject(0x1000, 0, &device_type, sizeof(device_type), &len)) {
        TETHER_LOGE(TAG, "Failed to read device type");
        return false;
    }
    
    // Check for CiA 406 profile (406 = 0x196)
    uint16_t profile = device_type & 0xFFFF;
    if (profile != 406) {
        TETHER_LOGW(TAG, "Device profile is %u, expected 406", profile);
        // Continue anyway as some encoders may report differently
    }
    
    // Detect encoder class
    if (!detectEncoderClass()) {
        TETHER_LOGE(TAG, "Failed to detect encoder class");
        return false;
    }
    
    // Read capabilities
    if (!readCapabilities()) {
        TETHER_LOGW(TAG, "Failed to read all capabilities");
        // Continue with defaults
    }
    
    // Read diagnostic info
    readDiagnostics();
    
    TETHER_LOGI(TAG, "Encoder initialized: %s\n  Interface: %s\n  Resolution: %u bits (ST), %u revolutions (MT)",
             getEncoderClassName(capabilities_.encoder_class),
             getInterfaceTypeName(capabilities_.interface_type),
             capabilities_.single_turn_resolution,
             capabilities_.multi_turn_revolutions);
    
    initialized_ = true;
    return true;
}

bool Encoder::detectEncoderClass() {
    // Read operating parameters to determine class
    uint8_t encoder_class = 0;
    size_t len;
    
    if (readObject(OperatingParameters, OperatingParametersSub::EncoderClass,
                   &encoder_class, sizeof(encoder_class), &len)) {
        capabilities_.encoder_class = static_cast<EncoderClassEx>(encoder_class);
    } else {
        // Try to infer from available objects
        uint32_t multi_turn;
        if (readObject(MultiTurnValue, 0, &multi_turn, sizeof(multi_turn), &len)) {
            capabilities_.encoder_class = EncoderClassEx::C2_AbsoluteMultiTurn;
        } else {
            // Check for limit switch status (C4)
            uint8_t limit_status;
            if (readObject(LimitSwitchStatus, 0, &limit_status, sizeof(limit_status), &len)) {
                capabilities_.encoder_class = EncoderClassEx::C4_IncrementalWithLimits;
            } else {
                // Default to absolute single-turn
                capabilities_.encoder_class = EncoderClassEx::C1_AbsoluteSingleTurn;
            }
        }
    }
    
    // Read encoder type (rotary/linear)
    uint8_t enc_type;
    if (readObject(OperatingParameters, 0x04, &enc_type, sizeof(enc_type), &len)) {
        capabilities_.encoder_type = static_cast<EncoderTypeEx>(enc_type);
    }
    
    return true;
}

bool Encoder::readCapabilities() {
    size_t len;
    
    // Read single-turn resolution
    uint8_t st_res = 0;
    if (readObject(SingleTurnResolution, 0, &st_res, sizeof(st_res), &len)) {
        capabilities_.single_turn_resolution = st_res;
    }
    
    // Read multi-turn revolutions
    uint32_t mt_revs = 1;
    if (readObject(DistinguishableRevolutions, 0, &mt_revs, sizeof(mt_revs), &len)) {
        capabilities_.multi_turn_revolutions = mt_revs;
    } else {
        capabilities_.multi_turn_revolutions = 1;
    }
    
    // Read total measuring range
    uint32_t range = 0;
    if (readObject(TotalMeasuringRange, 0, &range, sizeof(range), &len)) {
        capabilities_.total_measuring_range = range;
    } else {
        // Calculate from resolution
        capabilities_.total_measuring_range = 1 << capabilities_.single_turn_resolution;
    }
    
    // Read interface type
    uint8_t iface = 0;
    if (readObject(InterfaceTypeObject, 0, &iface, sizeof(iface), &len)) {
        capabilities_.interface_type = static_cast<CiA406::InterfaceType>(iface);
    }
    
    // Probe for optional features
    int32_t dummy32;
    uint16_t dummy16;
    
    capabilities_.has_velocity_output = 
        readObject(VelocityActualValue, 0, &dummy32, sizeof(dummy32), &len);
    
    capabilities_.has_working_area_monitoring = 
        readObject(WorkingAreaLowLimit1, 0, &dummy32, sizeof(dummy32), &len);
    
    capabilities_.has_temperature_sensor = 
        readObject(Temperature, 0, &dummy16, sizeof(dummy16), &len);
    
    capabilities_.has_signal_quality = 
        readObject(SignalQuality, 0, &dummy16, sizeof(dummy16), &len);
    
    // Read supported alarms/warnings
    readObject(SupportedAlarms, 0, &capabilities_.supported_alarms, 
               sizeof(capabilities_.supported_alarms), &len);
    readObject(SupportedWarnings, 0, &capabilities_.supported_warnings,
               sizeof(capabilities_.supported_warnings), &len);
    
    return true;
}

bool Encoder::applyPDOMapping(PDOMappingPreset preset) {
    TETHER_LOGI(TAG, "Applying PDO mapping preset for slave %u", slave_addr_);
    
    current_mapping_ = preset;
    
    // Define mapping entries based on preset
    PDOMappingEntry entries[8];
    size_t entry_count = 0;
    
    switch (preset) {
        case PDOMappingPreset::Basic:
            entries[0] = {PositionValue, 0, 32};
            entries[1] = {OperatingStatusObject, 0, 16};
            entry_count = 2;
            pdo_size_ = sizeof(TxPDOBasic);
            break;
            
        case PDOMappingPreset::WithVelocity:
            entries[0] = {PositionValue, 0, 32};
            entries[1] = {VelocityActualValue, 0, 32};
            entries[2] = {OperatingStatusObject, 0, 16};
            entry_count = 3;
            pdo_size_ = sizeof(TxPDOWithVelocity);
            break;
            
        case PDOMappingPreset::Full:
            entries[0] = {PositionValue, 0, 32};
            entries[1] = {VelocityActualValue, 0, 32};
            entries[2] = {OperatingStatusObject, 0, 16};
            entries[3] = {AlarmObject, 0, 16};
            entries[4] = {Temperature, 0, 16};
            entries[5] = {SignalQuality, 0, 8};
            entry_count = 6;
            pdo_size_ = sizeof(TxPDOFull);
            break;
            
        case PDOMappingPreset::MultiTurn:
            entries[0] = {PositionValue, 0, 32};
            entries[1] = {MultiTurnValue, 0, 16};
            entries[2] = {SingleTurnValue, 0, 16};
            entries[3] = {OperatingStatusObject, 0, 16};
            entry_count = 4;
            pdo_size_ = sizeof(TxPDOMultiTurn);
            break;
            
        case PDOMappingPreset::HighSpeed:
            entries[0] = {PositionValue, 0, 32};
            entry_count = 1;
            pdo_size_ = sizeof(int32_t);
            break;
            
        case PDOMappingPreset::Diagnostic:
            // Same as Full
            entries[0] = {PositionValue, 0, 32};
            entries[1] = {VelocityActualValue, 0, 32};
            entries[2] = {OperatingStatusObject, 0, 16};
            entries[3] = {AlarmObject, 0, 16};
            entries[4] = {Temperature, 0, 16};
            entries[5] = {SignalQuality, 0, 8};
            entry_count = 6;
            pdo_size_ = sizeof(TxPDOFull);
            break;
            
        default:
            TETHER_LOGE(TAG, "Unknown PDO mapping preset");
            return false;
    }
    
    return applyCustomPDOMapping(entries, entry_count);
}

bool Encoder::applyCustomPDOMapping(const PDOMappingEntry* entries, size_t count) {
    // First, disable PDO mapping by writing 0 to subindex 0
    uint8_t zero = 0;
    if (!writeObject(0x1A00, 0, &zero, sizeof(zero))) {
        TETHER_LOGW(TAG, "Failed to disable PDO mapping");
    }
    
    // Write each mapping entry
    for (size_t i = 0; i < count; i++) {
        uint32_t mapping_value = entries[i].toMappingValue();
        if (!writeObject(0x1A00, i + 1, &mapping_value, sizeof(mapping_value))) {
            TETHER_LOGE(TAG, "Failed to write PDO mapping entry %zu", i);
            return false;
        }
    }
    
    // Enable mapping by writing entry count
    uint8_t entry_count = static_cast<uint8_t>(count);
    if (!writeObject(0x1A00, 0, &entry_count, sizeof(entry_count))) {
        TETHER_LOGE(TAG, "Failed to enable PDO mapping");
        return false;
    }
    
    current_mapping_ = PDOMappingPreset::Custom;
    return true;
}

void Encoder::update() {
    if (!initialized_) return;
    
    uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t dt_us = now - last_update_time_;
    
    // Read PDO data based on current mapping
    switch (current_mapping_) {
        case PDOMappingPreset::Basic:
        case PDOMappingPreset::HighSpeed: {
            TxPDOBasic pdo;
            // In real implementation, this would come from cyclic PDO data
            size_t len;
            readObject(PositionValue, 0, &pdo.position, sizeof(pdo.position), &len);
            readObject(OperatingStatusObject, 0, &pdo.status, sizeof(pdo.status), &len);
            
            raw_position_ = pdo.position;
            processStatus(pdo.status);
            break;
        }
        
        case PDOMappingPreset::WithVelocity: {
            TxPDOWithVelocity pdo;
            size_t len;
            readObject(PositionValue, 0, &pdo.position, sizeof(pdo.position), &len);
            readObject(VelocityActualValue, 0, &pdo.velocity, sizeof(pdo.velocity), &len);
            readObject(OperatingStatusObject, 0, &pdo.status, sizeof(pdo.status), &len);
            
            raw_position_ = pdo.position;
            velocity_ = pdo.velocity;
            processStatus(pdo.status);
            break;
        }
        
        case PDOMappingPreset::Full:
        case PDOMappingPreset::Diagnostic: {
            TxPDOFull pdo;
            size_t len;
            readObject(PositionValue, 0, &pdo.position, sizeof(pdo.position), &len);
            readObject(VelocityActualValue, 0, &pdo.velocity, sizeof(pdo.velocity), &len);
            readObject(OperatingStatusObject, 0, &pdo.status, sizeof(pdo.status), &len);
            readObject(AlarmObject, 0, &pdo.alarms, sizeof(pdo.alarms), &len);
            readObject(Temperature, 0, &pdo.temperature, sizeof(pdo.temperature), &len);
            readObject(SignalQuality, 0, &pdo.signal_quality, sizeof(pdo.signal_quality), &len);
            
            raw_position_ = pdo.position;
            velocity_ = pdo.velocity;
            processStatus(pdo.status);
            processAlarms(pdo.alarms);
            temperature_ = pdo.temperature;
            signal_quality_ = pdo.signal_quality;
            break;
        }
        
        case PDOMappingPreset::MultiTurn: {
            TxPDOMultiTurn pdo;
            size_t len;
            readObject(PositionValue, 0, &pdo.position, sizeof(pdo.position), &len);
            readObject(MultiTurnValue, 0, &pdo.multi_turn, sizeof(pdo.multi_turn), &len);
            readObject(SingleTurnValue, 0, &pdo.single_turn, sizeof(pdo.single_turn), &len);
            readObject(OperatingStatusObject, 0, &pdo.status, sizeof(pdo.status), &len);
            
            raw_position_ = pdo.position;
            multi_turn_value_ = pdo.multi_turn;
            single_turn_value_ = pdo.single_turn;
            processStatus(pdo.status);
            break;
        }
        
        default:
            break;
    }
    
    // Apply scaling
    scaled_position_ = scaling_.apply(raw_position_);
    
    // Calculate velocity if not provided by encoder
    if (current_mapping_ == PDOMappingPreset::Basic ||
        current_mapping_ == PDOMappingPreset::HighSpeed ||
        current_mapping_ == PDOMappingPreset::MultiTurn) {
        calculateVelocity(raw_position_, dt_us);
    }
    
    last_update_time_ = now;
    previous_position_ = raw_position_;
    
    // Fire position updated event
    fireEvent(EncoderEvent::PositionUpdated, static_cast<uint32_t>(raw_position_));
}

void Encoder::calculateVelocity(int32_t new_position, uint32_t dt_us) {
    if (dt_us == 0) return;
    
    int32_t delta = new_position - previous_position_;
    
    // Handle rollover for incremental/single-turn encoders
    int32_t max_value = capabilities_.total_measuring_range;
    if (max_value > 0) {
        if (delta > max_value / 2) {
            delta -= max_value;
        } else if (delta < -max_value / 2) {
            delta += max_value;
        }
    }
    
    // Calculate velocity (counts per second)
    velocity_ = (int64_t)delta * 1000000 / dt_us;
    
    // Calculate acceleration
    int32_t velocity_delta = velocity_ - previous_velocity_;
    acceleration_ = (int64_t)velocity_delta * 1000000 / dt_us;
    previous_velocity_ = velocity_;
}

void Encoder::processStatus(uint16_t status_word) {
    status_raw_ = status_word;
    status_ = OperatingStatus::fromRaw(status_word);
    
    // Check for working area state
    size_t len;
    uint8_t wa_state;
    if (readObject(WorkingAreaStateObject, 0, &wa_state, sizeof(wa_state), &len)) {
        WorkingAreaState new_state = static_cast<WorkingAreaState>(wa_state);
        if (new_state != working_area_state_) {
            if (isWithinWorkingArea() && !status_.position_valid) {
                fireEvent(EncoderEvent::WorkingAreaExited, wa_state);
            } else {
                fireEvent(EncoderEvent::WorkingAreaEntered, wa_state);
            }
            working_area_state_ = new_state;
        }
    }
    
    // Fire reference done event if just completed
    static bool prev_ref_done = false;
    if (status_.reference_done && !prev_ref_done) {
        fireEvent(EncoderEvent::ReferenceDone, 0);
    }
    prev_ref_done = status_.reference_done;
}

void Encoder::processAlarms(uint16_t alarm_word) {
    alarms_raw_ = alarm_word;
    alarms_ = AlarmFlags::fromRaw(alarm_word);
    
    // Check for new alarms
    uint16_t new_alarms = alarm_word & ~previous_alarms_;
    uint16_t cleared_alarms = previous_alarms_ & ~alarm_word;
    
    if (new_alarms != 0) {
        fireEvent(EncoderEvent::AlarmTriggered, new_alarms);
    }
    
    if (cleared_alarms != 0) {
        fireEvent(EncoderEvent::AlarmCleared, cleared_alarms);
    }
    
    previous_alarms_ = alarm_word;
}

void Encoder::fireEvent(EncoderEvent event, uint32_t data) {
    if (event_callback_) {
        event_callback_(event, slave_addr_, data);
    }
}

// ============================================================================
// Scaling and Offset
// ============================================================================

void Encoder::setScaling(int32_t numerator, int32_t denominator, int32_t offset) {
    scaling_.enabled = true;
    scaling_.numerator = numerator;
    scaling_.denominator = denominator;
    scaling_.offset = offset;
    
    // Also write to encoder if it supports internal scaling
    writeObject(ScalingNumerator, 0, &numerator, sizeof(numerator));
    writeObject(ScalingDenominator, 0, &denominator, sizeof(denominator));
    writeObject(ScalingOffset, 0, &offset, sizeof(offset));
    
    // Enable scaling function
    uint8_t enable = 1;
    writeObject(OperatingParameters, OperatingParametersSub::ScalingFunctionEnabled,
                &enable, sizeof(enable));
}

void Encoder::setScalingFromRange(int32_t encoder_counts, int32_t user_units) {
    setScaling(user_units, encoder_counts, 0);
}

void Encoder::disableScaling() {
    scaling_.enabled = false;
    
    uint8_t disable = 0;
    writeObject(OperatingParameters, OperatingParametersSub::ScalingFunctionEnabled,
                &disable, sizeof(disable));
}

void Encoder::setOffset(int32_t offset) {
    scaling_.offset = offset;
    writeObject(ScalingOffset, 0, &offset, sizeof(offset));
}

// ============================================================================
// Preset and Reference
// ============================================================================

bool Encoder::presetPosition(int32_t position) {
    TETHER_LOGI(TAG, "Presetting encoder %u position to %d", slave_addr_, position);
    
    // Write preset value
    if (!writeObject(PresetValue, 0, &position, sizeof(position))) {
        TETHER_LOGE(TAG, "Failed to write preset value");
        return false;
    }
    
    // Verify preset was applied
    size_t len;
    int32_t read_pos;
    if (readObject(PositionValue, 0, &read_pos, sizeof(read_pos), &len)) {
        if (read_pos == position) {
            TETHER_LOGI(TAG, "Preset successful");
            return true;
        }
    }
    
    TETHER_LOGW(TAG, "Preset may not have been applied correctly");
    return true; // Return true as the write succeeded
}

bool Encoder::startReference(const ReferenceConfig& config) {
    TETHER_LOGI(TAG, "Starting reference procedure for encoder %u", slave_addr_);
    
    // Write reference position
    writeObject(ReferencePosition, 0, &config.reference_position, 
                sizeof(config.reference_position));
    
    // Configure direction
    uint8_t direction = config.direction_positive ? 0 : 1;
    writeObject(CountingDirection, 0, &direction, sizeof(direction));
    
    // Start reference (implementation depends on encoder capabilities)
    // For now, just set current position as reference
    if (config.mode == ReferenceMode::CurrentPosition) {
        return presetPosition(config.reference_position);
    }
    
    // For other modes, would need to trigger reference procedure
    return true;
}

void Encoder::abortReference() {
    // Implementation depends on encoder
}

// ============================================================================
// Alarms and Working Area
// ============================================================================

bool Encoder::clearAlarms() {
    // Some encoders support clearing alarms by writing to alarm object
    uint16_t zero = 0;
    return writeObject(AlarmObject, 0, &zero, sizeof(zero));
}

bool Encoder::setWorkingArea(int32_t low_limit, int32_t high_limit, uint8_t area_index) {
    if (area_index == 1) {
        writeObject(WorkingAreaLowLimit1, 0, &low_limit, sizeof(low_limit));
        writeObject(WorkingAreaHighLimit1, 0, &high_limit, sizeof(high_limit));
    } else if (area_index == 2) {
        writeObject(WorkingAreaLowLimit2, 0, &low_limit, sizeof(low_limit));
        writeObject(WorkingAreaHighLimit2, 0, &high_limit, sizeof(high_limit));
    } else {
        return false;
    }
    return true;
}

bool Encoder::isWithinWorkingArea() const {
    return working_area_state_ == WorkingAreaState::WithinArea1 ||
           working_area_state_ == WorkingAreaState::WithinArea2;
}

// ============================================================================
// Diagnostics
// ============================================================================

bool Encoder::readDiagnostics() {
    size_t len;
    
    readObject(Temperature, 0, &temperature_, sizeof(temperature_), &len);
    readObject(SupplyVoltage, 0, &supply_voltage_, sizeof(supply_voltage_), &len);
    readObject(SignalQuality, 0, &signal_quality_, sizeof(signal_quality_), &len);
    readObject(OperatingTime, 0, &operating_time_, sizeof(operating_time_), &len);
    
    // Read strings
    char buffer[64];
    if (readObject(SerialNumber, 0, buffer, sizeof(buffer), &len)) {
        serial_number_ = std::string(buffer, len);
    }
    
    if (readObject(FirmwareVersion, 0, buffer, sizeof(buffer), &len)) {
        firmware_version_ = std::string(buffer, len);
    }
    
    return true;
}

// ============================================================================
// Interface Configuration
// ============================================================================

bool Encoder::configureSSI(uint16_t clock_khz, uint8_t data_bits, 
                           bool gray_code, bool msb_first) {
    writeObject(SSIConfiguration, SSIConfigSub::ClockFrequency, &clock_khz, sizeof(clock_khz));
    writeObject(SSIConfiguration, SSIConfigSub::DataBits, &data_bits, sizeof(data_bits));
    
    uint8_t gray = gray_code ? 1 : 0;
    writeObject(SSIConfiguration, SSIConfigSub::BinaryGray, &gray, sizeof(gray));
    
    uint8_t msb = msb_first ? 1 : 0;
    writeObject(SSIConfiguration, SSIConfigSub::MSBFirst, &msb, sizeof(msb));
    
    return true;
}

bool Encoder::configureBiSS(uint32_t clock_hz, uint8_t data_bits) {
    // BiSS configuration
    writeObject(BiSSConfiguration, 1, &clock_hz, sizeof(clock_hz));
    writeObject(BiSSConfiguration, 2, &data_bits, sizeof(data_bits));
    return true;
}

bool Encoder::configureEnDat(uint32_t clock_hz) {
    writeObject(EnDatConfiguration, 1, &clock_hz, sizeof(clock_hz));
    return true;
}

// ============================================================================
// Event Handling
// ============================================================================

void Encoder::setEventCallback(EncoderEventCallback callback) {
    event_callback_ = std::move(callback);
}

void Encoder::clearEventCallback() {
    event_callback_ = nullptr;
}

// ============================================================================
// SDO Access
// ============================================================================

bool Encoder::readObject(uint16_t index, uint8_t subindex, void* data, size_t size, size_t* out_size) {
    return m_sdo.readSync(slave_addr_, index, subindex, data, size, EtherCAT::SDO::kDefaultSDOTimeoutMs, out_size);
}

bool Encoder::writeObject(uint16_t index, uint8_t subindex, const void* data, size_t size) {
    return m_sdo.writeSync(slave_addr_, index, subindex, data, size, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<Encoder> createEncoder(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_position) {
    auto encoder = std::make_unique<Encoder>(sdo, slave_position);
    if (encoder->initialize()) {
        return encoder;
    }
    return nullptr;
}

std::vector<uint16_t> scanForEncoders(EtherCAT::SDO::SDOManager& sdo) {
    std::vector<uint16_t> encoders;
    
    // Scan first 16 slaves
    for (uint16_t i = 0; i < 16; i++) {
        uint32_t device_type = 0;
        size_t len;
        
        Encoder temp(sdo, i);
        if (temp.readObject(0x1000, 0, &device_type, sizeof(device_type), &len)) {
            uint16_t profile = device_type & 0xFFFF;
            if (profile == 406) {
                encoders.push_back(i);
            }
        }
    }
    
    return encoders;
}

} // namespace CiA406
