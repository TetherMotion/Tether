/**
 * @file CiA404Device.cpp
 * @brief CiA 404 Measuring Device and Closed Loop Controller Implementation
 */

#include "profiles/cia404/CiA404Device.hpp"
#include "tether/platform/EspCompat.hpp"
#include "SDOManager.hpp"

static const char* TAG = "CiA404";
#define LOG_I(fmt, ...) TETHER_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOG_W(fmt, ...) TETHER_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOG_E(fmt, ...) TETHER_LOGE(TAG, fmt, ##__VA_ARGS__)

#include <cstring>
#include <algorithm>

namespace CiA404 {

// ============================================================================
// Construction and Initialization
// ============================================================================

MeasuringDevice::MeasuringDevice(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr)
    : m_sdo(sdo)
    , slave_addr_(slave_addr)
    , use_configured_addr_(use_configured_addr)
    , initialized_(false)
    , capabilities_()
    , current_mapping_(PDOMappingPreset::Minimal)
    , controller_state_()
    , pid_params_()
{
}

MeasuringDevice::~MeasuringDevice() {
}

bool MeasuringDevice::initialize() {
    LOG_I("Initializing CiA 404 measuring device at address %u", slave_addr_);
    
    // Verify device type
    uint32_t device_type = 0;
    if (!readSDO(0x1000, 0, &device_type, sizeof(device_type))) {
        LOG_E("Failed to read device type");
        return false;
    }
    
    uint16_t profile = device_type & 0xFFFF;
    if (profile != PROFILE_NUMBER) {
        LOG_W("Device profile %u is not CiA 404 (%u)", profile, PROFILE_NUMBER);
    }
    
    if (!detectCapabilities()) {
        LOG_E("Failed to detect capabilities");
        return false;
    }
    
    LOG_I("Process inputs: %u, outputs: %u", 
          capabilities_.num_process_inputs, capabilities_.num_process_outputs);
    LOG_I("Controller: %s, Calibration: %s, Alarms: %s",
          capabilities_.has_controller ? "Yes" : "No",
          capabilities_.has_calibration ? "Yes" : "No",
          capabilities_.has_alarms ? "Yes" : "No");
    
    // Initialize state vectors
    input_states_.resize(capabilities_.num_process_inputs);
    input_configs_.resize(capabilities_.num_process_inputs);
    alarm_configs_.resize(capabilities_.num_process_inputs);
    calibration_data_.resize(capabilities_.num_process_inputs);
    prev_alarm_status_.resize(capabilities_.num_process_inputs, 0);
    output_values_.resize(capabilities_.num_process_outputs, 0);
    
    if (!applyDefaultConfiguration()) {
        LOG_W("Failed to apply default configuration");
    }
    
    initialized_ = true;
    fireEvent(DeviceEvent::ValueUpdated, 0, 0);
    
    return true;
}

bool MeasuringDevice::detectCapabilities() {
    // Check for process inputs
    uint8_t num = 0;
    if (readSDO(ProcessDataInput1, PDInputSub::NumberOfMappedObjects, &num, 1)) {
        capabilities_.num_process_inputs = 1;
        if (readSDO(ProcessDataInput2, PDInputSub::NumberOfMappedObjects, &num, 1)) {
            capabilities_.num_process_inputs++;
        }
        if (readSDO(ProcessDataInput3, PDInputSub::NumberOfMappedObjects, &num, 1)) {
            capabilities_.num_process_inputs++;
        }
        if (readSDO(ProcessDataInput4, PDInputSub::NumberOfMappedObjects, &num, 1)) {
            capabilities_.num_process_inputs++;
        }
    }
    
    // Check for process outputs
    if (readSDO(ProcessDataOutput1, PDOutputSub::NumberOfMappedObjects, &num, 1)) {
        capabilities_.num_process_outputs = 1;
        if (readSDO(ProcessDataOutput2, PDOutputSub::NumberOfMappedObjects, &num, 1)) {
            capabilities_.num_process_outputs++;
        }
    }
    
    // Check for controller
    uint8_t mode = 0;
    if (readSDO(ControllerMode, 0, &mode, 1)) {
        capabilities_.has_controller = true;
    }
    
    // Check for calibration
    uint8_t cal_status = 0;
    if (readSDO(CalibrationStatus, 1, &cal_status, 1)) {
        capabilities_.has_calibration = true;
    }
    
    // Check for alarms
    uint16_t alarm = 0;
    if (readSDO(AlarmStatus, 1, &alarm, 2)) {
        capabilities_.has_alarms = true;
    }
    
    // Check for diagnostics
    uint8_t sensor = 0;
    if (readSDO(SensorStatus, 1, &sensor, 1)) {
        capabilities_.has_diagnostics = true;
    }
    
    return true;
}

bool MeasuringDevice::applyDefaultConfiguration() {
    // Set default scaling (unity)
    for (size_t i = 0; i < input_configs_.size(); i++) {
        input_configs_[i].scaling_factor = 0x10000; // 1.0 in Q16
        input_configs_[i].offset = 0;
    }
    
    // Set default alarm limits
    for (size_t i = 0; i < alarm_configs_.size(); i++) {
        alarm_configs_[i].high_high_limit = 32767;
        alarm_configs_[i].high_limit = 30000;
        alarm_configs_[i].low_limit = -30000;
        alarm_configs_[i].low_low_limit = -32768;
    }
    
    return true;
}

// ============================================================================
// PDO Configuration
// ============================================================================

bool MeasuringDevice::applyPDOMapping(PDOMappingPreset preset) {
    current_mapping_ = preset;
    return true;
}

// ============================================================================
// Update Cycle
// ============================================================================

void MeasuringDevice::processTxPDO(const uint8_t* data, size_t len) {
    if (!initialized_ || len == 0) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Minimal:
        case PDOMappingPreset::InputWithStatus:
            processInputPDO(data, len);
            break;
            
        case PDOMappingPreset::Controller:
        case PDOMappingPreset::ControllerFull:
            processControllerPDO(data, len);
            break;
            
        case PDOMappingPreset::WithAlarms:
            processInputPDO(data, len);
            checkAlarms();
            break;
            
        default:
            processInputPDO(data, len);
            break;
    }
}

size_t MeasuringDevice::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!initialized_) return 0;
    
    size_t offset = 0;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Controller:
        case PDOMappingPreset::ControllerFull:
            if (max_len >= sizeof(RxPDO_ControllerSetpoint)) {
                RxPDO_ControllerSetpoint* pdo = reinterpret_cast<RxPDO_ControllerSetpoint*>(data);
                pdo->setpoint = controller_state_.setpoint;
                pdo->mode = controller_state_.mode;
                offset = sizeof(RxPDO_ControllerSetpoint);
            }
            break;
            
        default:
            // Output values
            for (size_t i = 0; i < output_values_.size() && offset + 4 <= max_len; i++) {
                memcpy(data + offset, &output_values_[i], 4);
                offset += 4;
            }
            break;
    }
    
    return offset;
}

void MeasuringDevice::update() {
    if (!initialized_) return;
    
    // Read process inputs
    for (size_t ch = 0; ch < capabilities_.num_process_inputs; ch++) {
        uint16_t index = ProcessDataInput1 + (ch * 0x10);
        int32_t val = 0;
        
        if (readSDO(index, PDInputSub::ProcessInputValue, &val, 4)) {
            input_states_[ch].raw_value = val;
            
            // Apply scaling
            const auto& cfg = input_configs_[ch];
            int64_t scaled = (static_cast<int64_t>(val) * cfg.scaling_factor) >> 16;
            input_states_[ch].scaled_value = static_cast<int32_t>(scaled) + cfg.offset;
            
            // Read status if available
            uint8_t status = 0;
            if (readSDO(index, PDInputSub::ProcessInputStatus, &status, 1)) {
                input_states_[ch].status = status;
            }
            
            // Fire value callback
            if (value_callback_) {
                value_callback_(slave_addr_, ch + 1, 
                               input_states_[ch].raw_value,
                               input_states_[ch].scaled_value);
            }
        }
    }
    
    // Read controller state if present
    if (capabilities_.has_controller) {
        readSDO(ControllerActualValue, 0, &controller_state_.actual_value, 4);
        readSDO(ControllerDeviation, 0, &controller_state_.deviation, 4);
        readSDO(ControllerOutput, 0, &controller_state_.output, 4);
        readSDO(ControllerStatus, 0, &controller_state_.status, 2);
    }
    
    // Check alarms
    if (capabilities_.has_alarms) {
        checkAlarms();
    }
}

void MeasuringDevice::processInputPDO(const uint8_t* data, size_t len) {
    size_t offset = 0;
    
    for (size_t ch = 0; ch < input_states_.size() && offset + 4 <= len; ch++) {
        int32_t val;
        memcpy(&val, data + offset, 4);
        input_states_[ch].raw_value = val;
        
        // Apply scaling
        const auto& cfg = input_configs_[ch];
        int64_t scaled = (static_cast<int64_t>(val) * cfg.scaling_factor) >> 16;
        input_states_[ch].scaled_value = static_cast<int32_t>(scaled) + cfg.offset;
        
        offset += 4;
        
        // Status byte if present
        if (offset < len) {
            input_states_[ch].status = data[offset++];
        }
    }
}

void MeasuringDevice::processControllerPDO(const uint8_t* data, size_t len) {
    if (len >= sizeof(TxPDO_ControllerStatus)) {
        const TxPDO_ControllerStatus* pdo = 
            reinterpret_cast<const TxPDO_ControllerStatus*>(data);
        controller_state_.actual_value = pdo->actual_value;
        controller_state_.output = pdo->output;
        controller_state_.deviation = pdo->deviation;
        controller_state_.status = pdo->status;
    }
}

void MeasuringDevice::checkAlarms() {
    for (size_t ch = 0; ch < input_states_.size(); ch++) {
        uint16_t alarm = 0;
        uint16_t warning = 0;
        int32_t val = input_states_[ch].scaled_value;
        const auto& cfg = alarm_configs_[ch];
        
        // Check alarm limits
        if (val >= cfg.high_high_limit) {
            alarm |= AlarmStatusBits::HighHigh;
        } else if (val >= cfg.high_limit) {
            alarm |= AlarmStatusBits::High;
        }
        
        if (val <= cfg.low_low_limit) {
            alarm |= AlarmStatusBits::LowLow;
        } else if (val <= cfg.low_limit) {
            alarm |= AlarmStatusBits::Low;
        }
        
        // Check warning limits
        if (val >= cfg.warning_high) {
            warning |= WarningStatusBits::High;
        }
        if (val <= cfg.warning_low) {
            warning |= WarningStatusBits::Low;
        }
        
        // Fire events for new alarms
        uint16_t new_alarms = alarm & ~prev_alarm_status_[ch];
        if (new_alarms) {
            if (new_alarms & AlarmStatusBits::HighHigh) {
                fireEvent(DeviceEvent::AlarmHighHigh, ch + 1, val);
            }
            if (new_alarms & AlarmStatusBits::High) {
                fireEvent(DeviceEvent::AlarmHigh, ch + 1, val);
            }
            if (new_alarms & AlarmStatusBits::Low) {
                fireEvent(DeviceEvent::AlarmLow, ch + 1, val);
            }
            if (new_alarms & AlarmStatusBits::LowLow) {
                fireEvent(DeviceEvent::AlarmLowLow, ch + 1, val);
            }
            
            if (alarm_callback_) {
                alarm_callback_(slave_addr_, ch + 1, alarm, val);
            }
        }
        
        prev_alarm_status_[ch] = alarm;
        input_states_[ch].alarm_status = alarm;
        input_states_[ch].warning_status = warning;
    }
}

void MeasuringDevice::fireEvent(DeviceEvent event, uint8_t channel, int32_t value) {
    if (event_callback_) {
        event_callback_(event, slave_addr_, channel, value);
    }
}

// ============================================================================
// Process Input Operations
// ============================================================================

int32_t MeasuringDevice::getRawValue(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].raw_value;
}

int32_t MeasuringDevice::getScaledValue(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].scaled_value;
}

int32_t MeasuringDevice::getFilteredValue(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].filtered_value;
}

uint8_t MeasuringDevice::getInputStatus(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].status;
}

const ProcessInputState& MeasuringDevice::getInputState(uint8_t channel) {
    static ProcessInputState empty;
    if (channel == 0 || channel > input_states_.size()) return empty;
    return input_states_[channel - 1];
}

bool MeasuringDevice::configureInput(uint8_t channel, uint8_t input_range,
                                     uint16_t unit, uint8_t decimal_places) {
    if (channel == 0 || channel > input_configs_.size()) return false;
    
    auto& cfg = input_configs_[channel - 1];
    cfg.input_range = input_range;
    cfg.engineering_unit = unit;
    cfg.decimal_places = decimal_places;
    
    bool result = writeSDO(AnalogInputRange, channel, &input_range, 1);
    result &= writeSDO(AnalogInputUnit, channel, &unit, 2);
    result &= writeSDO(AnalogInputDecimalPlaces, channel, &decimal_places, 1);
    
    return result;
}

bool MeasuringDevice::setInputScaling(uint8_t channel, int32_t raw_min, int32_t raw_max,
                                      int32_t eng_min, int32_t eng_max) {
    if (channel == 0 || channel > input_configs_.size()) return false;
    if (raw_max == raw_min) return false;
    
    auto& cfg = input_configs_[channel - 1];
    
    // Calculate Q16 scale factor
    int64_t scale = ((int64_t)(eng_max - eng_min) << 16) / (raw_max - raw_min);
    cfg.scaling_factor = static_cast<int32_t>(scale);
    cfg.offset = eng_min - ((raw_min * scale) >> 16);
    
    bool result = writeSDO(AnalogInputScalingFactor, channel, &cfg.scaling_factor, 4);
    result &= writeSDO(AnalogInputOffset, channel, &cfg.offset, 4);
    
    return result;
}

bool MeasuringDevice::setInputFilter(uint8_t channel, uint16_t filter_time_ms) {
    if (channel == 0 || channel > input_configs_.size()) return false;
    
    input_configs_[channel - 1].filter_time = filter_time_ms;
    return writeSDO(AnalogInputFilterTime, channel, &filter_time_ms, 2);
}

// ============================================================================
// Process Output Operations
// ============================================================================

bool MeasuringDevice::setOutputValue(uint8_t channel, int32_t value) {
    if (channel == 0 || channel > output_values_.size()) return false;
    
    output_values_[channel - 1] = value;
    
    uint16_t index = ProcessDataOutput1 + ((channel - 1) * 0x10);
    return writeSDO(index, PDOutputSub::ProcessOutputValue, &value, 4);
}

int32_t MeasuringDevice::getOutputValue(uint8_t channel) {
    if (channel == 0 || channel > output_values_.size()) return 0;
    return output_values_[channel - 1];
}

bool MeasuringDevice::configureOutput(uint8_t channel, uint8_t output_range) {
    return writeSDO(AnalogOutputRange, channel, &output_range, 1);
}

bool MeasuringDevice::setOutputErrorBehavior(uint8_t channel, uint8_t behavior, int32_t error_value) {
    bool result = writeSDO(AnalogOutputErrorBehavior, channel, &behavior, 1);
    result &= writeSDO(AnalogOutputErrorValue, channel, &error_value, 4);
    return result;
}

// ============================================================================
// Controller Operations
// ============================================================================

bool MeasuringDevice::setSetpoint(int32_t setpoint) {
    controller_state_.setpoint = setpoint;
    return writeSDO(ControllerSetpoint, 0, &setpoint, 4);
}

int32_t MeasuringDevice::getSetpoint() const {
    return controller_state_.setpoint;
}

bool MeasuringDevice::setControllerMode(uint8_t mode) {
    controller_state_.mode = mode;
    return writeSDO(ControllerMode, 0, &mode, 1);
}

uint8_t MeasuringDevice::getControllerMode() const {
    return controller_state_.mode;
}

bool MeasuringDevice::setPIDParameters(const PIDParameters& params) {
    pid_params_ = params;
    
    bool result = writeSDO(PID_Kp, 0, &params.kp, 4);
    result &= writeSDO(PID_Ti, 0, &params.ti, 4);
    result &= writeSDO(PID_Td, 0, &params.td, 4);
    result &= writeSDO(PID_SampleTime, 0, &params.sample_time, 2);
    result &= writeSDO(PID_OutputUpperLimit, 0, &params.output_max, 4);
    result &= writeSDO(PID_OutputLowerLimit, 0, &params.output_min, 4);
    result &= writeSDO(PID_AntiWindup, 0, &params.anti_windup, 4);
    result &= writeSDO(PID_DerivativeFilter, 0, &params.derivative_filter, 1);
    
    return result;
}

PIDParameters MeasuringDevice::getPIDParameters() {
    readSDO(PID_Kp, 0, &pid_params_.kp, 4);
    readSDO(PID_Ti, 0, &pid_params_.ti, 4);
    readSDO(PID_Td, 0, &pid_params_.td, 4);
    readSDO(PID_SampleTime, 0, &pid_params_.sample_time, 2);
    readSDO(PID_OutputUpperLimit, 0, &pid_params_.output_max, 4);
    readSDO(PID_OutputLowerLimit, 0, &pid_params_.output_min, 4);
    return pid_params_;
}

bool MeasuringDevice::setPIDGains(int32_t kp, int32_t ti, int32_t td) {
    pid_params_.kp = kp;
    pid_params_.ti = ti;
    pid_params_.td = td;
    
    bool result = writeSDO(PID_Kp, 0, &kp, 4);
    result &= writeSDO(PID_Ti, 0, &ti, 4);
    result &= writeSDO(PID_Td, 0, &td, 4);
    
    return result;
}

bool MeasuringDevice::setOutputLimits(int32_t min, int32_t max) {
    pid_params_.output_min = min;
    pid_params_.output_max = max;
    
    bool result = writeSDO(PID_OutputLowerLimit, 0, &min, 4);
    result &= writeSDO(PID_OutputUpperLimit, 0, &max, 4);
    
    return result;
}

bool MeasuringDevice::setSetpointRamp(int32_t rate) {
    return writeSDO(SetpointRampRate, 0, &rate, 4);
}

bool MeasuringDevice::setFeedforward(int32_t gain) {
    return writeSDO(FeedforwardGain, 0, &gain, 4);
}

const ControllerState& MeasuringDevice::getControllerState() const {
    return controller_state_;
}

int32_t MeasuringDevice::getDeviation() const {
    return controller_state_.deviation;
}

int32_t MeasuringDevice::getControllerOutput() const {
    return controller_state_.output;
}

bool MeasuringDevice::isControllerActive() const {
    return controller_state_.isActive();
}

bool MeasuringDevice::resetIntegrator() {
    controller_state_.integral_sum = 0;
    // Most implementations don't have a direct SDO for this
    // Usually done by toggling mode or special command
    return true;
}

// ============================================================================
// Alarm Operations
// ============================================================================

bool MeasuringDevice::configureAlarms(uint8_t channel, const AlarmConfig& config) {
    if (channel == 0 || channel > alarm_configs_.size()) return false;
    
    alarm_configs_[channel - 1] = config;
    
    bool result = setAlarmHighHigh(channel, config.high_high_limit);
    result &= setAlarmHigh(channel, config.high_limit);
    result &= setAlarmLow(channel, config.low_limit);
    result &= setAlarmLowLow(channel, config.low_low_limit);
    result &= setAlarmHysteresis(channel, config.hysteresis);
    result &= writeSDO(AlarmDelayTime, channel, &config.delay_time, 2);
    result &= writeSDO(WarningHighLimit, channel, &config.warning_high, 4);
    result &= writeSDO(WarningLowLimit, channel, &config.warning_low, 4);
    
    return result;
}

bool MeasuringDevice::setAlarmHighHigh(uint8_t channel, int32_t limit) {
    if (channel > 0 && channel <= alarm_configs_.size()) {
        alarm_configs_[channel - 1].high_high_limit = limit;
    }
    return writeSDO(AlarmHighHighLimit, channel, &limit, 4);
}

bool MeasuringDevice::setAlarmHigh(uint8_t channel, int32_t limit) {
    if (channel > 0 && channel <= alarm_configs_.size()) {
        alarm_configs_[channel - 1].high_limit = limit;
    }
    return writeSDO(AlarmHighLimit, channel, &limit, 4);
}

bool MeasuringDevice::setAlarmLow(uint8_t channel, int32_t limit) {
    if (channel > 0 && channel <= alarm_configs_.size()) {
        alarm_configs_[channel - 1].low_limit = limit;
    }
    return writeSDO(AlarmLowLimit, channel, &limit, 4);
}

bool MeasuringDevice::setAlarmLowLow(uint8_t channel, int32_t limit) {
    if (channel > 0 && channel <= alarm_configs_.size()) {
        alarm_configs_[channel - 1].low_low_limit = limit;
    }
    return writeSDO(AlarmLowLowLimit, channel, &limit, 4);
}

bool MeasuringDevice::setAlarmHysteresis(uint8_t channel, int32_t hysteresis) {
    if (channel > 0 && channel <= alarm_configs_.size()) {
        alarm_configs_[channel - 1].hysteresis = hysteresis;
    }
    return writeSDO(AlarmHysteresis, channel, &hysteresis, 4);
}

uint16_t MeasuringDevice::getAlarmStatus(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].alarm_status;
}

uint16_t MeasuringDevice::getWarningStatus(uint8_t channel) {
    if (channel == 0 || channel > input_states_.size()) return 0;
    return input_states_[channel - 1].warning_status;
}

bool MeasuringDevice::hasAlarm(uint8_t channel) {
    return getAlarmStatus(channel) != 0;
}

bool MeasuringDevice::acknowledgeAlarms(uint8_t channel) {
    // Implementation depends on device - some use command object
    return true;
}

// ============================================================================
// Calibration Operations
// ============================================================================

bool MeasuringDevice::calibrateZero(uint8_t channel, int32_t zero_eng) {
    if (channel == 0 || channel > calibration_data_.size()) return false;
    
    calibration_data_[channel - 1].zero_eng = zero_eng;
    
    bool result = writeSDO(CalibrationPoint1Eng, channel, &zero_eng, 4);
    uint8_t cmd = CalibrationCommands::CalibrateZero;
    result &= writeSDO(CalibrationCommand, channel, &cmd, 1);
    
    return result;
}

bool MeasuringDevice::calibrateSpan(uint8_t channel, int32_t span_eng) {
    if (channel == 0 || channel > calibration_data_.size()) return false;
    
    calibration_data_[channel - 1].span_eng = span_eng;
    
    bool result = writeSDO(CalibrationPoint2Eng, channel, &span_eng, 4);
    uint8_t cmd = CalibrationCommands::CalibrateSpan;
    result &= writeSDO(CalibrationCommand, channel, &cmd, 1);
    
    return result;
}

bool MeasuringDevice::calibrateTwoPoint(uint8_t channel, int32_t raw1, int32_t eng1,
                                        int32_t raw2, int32_t eng2) {
    if (channel == 0 || channel > calibration_data_.size()) return false;
    
    auto& cal = calibration_data_[channel - 1];
    cal.zero_raw = raw1;
    cal.zero_eng = eng1;
    cal.span_raw = raw2;
    cal.span_eng = eng2;
    
    bool result = writeSDO(CalibrationPoint1Raw, channel, &raw1, 4);
    result &= writeSDO(CalibrationPoint1Eng, channel, &eng1, 4);
    result &= writeSDO(CalibrationPoint2Raw, channel, &raw2, 4);
    result &= writeSDO(CalibrationPoint2Eng, channel, &eng2, 4);
    
    uint8_t cmd = CalibrationCommands::CalibrateTwoPoint;
    result &= writeSDO(CalibrationCommand, channel, &cmd, 1);
    
    return result;
}

bool MeasuringDevice::acceptCalibration(uint8_t channel) {
    uint8_t cmd = CalibrationCommands::AcceptCalibration;
    return writeSDO(CalibrationCommand, channel, &cmd, 1);
}

bool MeasuringDevice::rejectCalibration(uint8_t channel) {
    uint8_t cmd = CalibrationCommands::RejectCalibration;
    return writeSDO(CalibrationCommand, channel, &cmd, 1);
}

bool MeasuringDevice::restoreFactoryCalibration(uint8_t channel) {
    uint8_t cmd = CalibrationCommands::RestoreFactory;
    return writeSDO(CalibrationCommand, channel, &cmd, 1);
}

bool MeasuringDevice::saveCalibration(uint8_t channel) {
    uint8_t cmd = CalibrationCommands::SaveCalibration;
    return writeSDO(CalibrationCommand, channel, &cmd, 1);
}

uint8_t MeasuringDevice::getCalibrationStatus(uint8_t channel) {
    uint8_t status = 0;
    readSDO(CalibrationStatus, channel, &status, 1);
    if (channel > 0 && channel <= calibration_data_.size()) {
        calibration_data_[channel - 1].status = status;
    }
    return status;
}

bool MeasuringDevice::setTare(uint8_t channel) {
    uint8_t cmd = 1;
    return writeSDO(TareCommand, channel, &cmd, 1);
}

bool MeasuringDevice::clearTare(uint8_t channel) {
    int32_t zero = 0;
    return writeSDO(TareValue, channel, &zero, 4);
}

// ============================================================================
// Diagnostics
// ============================================================================

uint8_t MeasuringDevice::getSensorStatus(uint8_t channel) {
    uint8_t status = 0;
    readSDO(SensorStatus, channel, &status, 1);
    return status;
}

uint16_t MeasuringDevice::getSensorSupplyVoltage(uint8_t channel) {
    uint16_t voltage = 0;
    readSDO(SensorSupplyVoltage, channel, &voltage, 2);
    return voltage;
}

int16_t MeasuringDevice::getSensorTemperature(uint8_t channel) {
    int16_t temp = 0;
    readSDO(SensorTemperature, channel, &temp, 2);
    return temp;
}

uint8_t MeasuringDevice::getSignalQuality(uint8_t channel) {
    uint8_t quality = 0;
    readSDO(SignalQuality, channel, &quality, 1);
    return quality;
}

uint32_t MeasuringDevice::getOperatingHours() {
    uint32_t hours = 0;
    readSDO(OperatingHours, 0, &hours, 4);
    return hours;
}

std::string MeasuringDevice::getDiagnostics() const {
    std::string diag;
    diag += "CiA 404 Measuring Device\n";
    diag += "  Slave: " + std::to_string(slave_addr_) + "\n";
    diag += "  Process Inputs: " + std::to_string(capabilities_.num_process_inputs) + "\n";
    diag += "  Process Outputs: " + std::to_string(capabilities_.num_process_outputs) + "\n";
    diag += "  Controller: " + std::string(capabilities_.has_controller ? "Yes" : "No") + "\n";
    
    if (capabilities_.has_controller) {
        diag += "  Controller Mode: " + std::string(getControllerModeName(controller_state_.mode)) + "\n";
        diag += "  Setpoint: " + std::to_string(controller_state_.setpoint) + "\n";
        diag += "  Output: " + std::to_string(controller_state_.output) + "\n";
    }
    
    return diag;
}

// ============================================================================
// Event Handling
// ============================================================================

void MeasuringDevice::setEventCallback(DeviceEventCallback callback) {
    event_callback_ = callback;
}

void MeasuringDevice::setAlarmCallback(AlarmCallback callback) {
    alarm_callback_ = callback;
}

void MeasuringDevice::setValueCallback(ValueCallback callback) {
    value_callback_ = callback;
}

// ============================================================================
// Internal SDO Methods
// ============================================================================

bool MeasuringDevice::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    return m_sdo.readSync(slave_addr_, index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

bool MeasuringDevice::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    return m_sdo.writeSync(slave_addr_, index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

} // namespace CiA404
