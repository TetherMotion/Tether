/**
 * @file CiA410Inclinometer.cpp
 * @brief CiA 410 Inclinometer Controller Implementation
 */

#include "profiles/cia410/CiA410Inclinometer.hpp"
#include "tether/platform/EspCompat.hpp"
#include <cstring>
#include <cmath>

#define LOG_TAG "CiA410"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)

extern "C" {
    bool ecm_sdo_read(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                      void* data, size_t len, bool use_configured_addr);
    bool ecm_sdo_write(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                       const void* data, size_t len, bool use_configured_addr);
}

namespace CiA410 {

// ============================================================================
// Construction
// ============================================================================

InclinometerController::InclinometerController(uint16_t slave_addr, bool use_configured_addr)
    : slave_addr_(slave_addr)
    , use_configured_addr_(use_configured_addr)
    , initialized_(false)
    , prev_statusword_(0)
    , prev_alarm_status_(0)
    , controlword_(0)
    , operating_mode_(OperatingMode::Continuous)
    , filter_setting_(FilterSetting::LowPass_10Hz)
    , current_mapping_(PDOMappingPreset::DualAxis)
{
    std::memset(&capabilities_, 0, sizeof(capabilities_));
    std::memset(&state_, 0, sizeof(state_));
}

InclinometerController::~InclinometerController() = default;

// ============================================================================
// Initialization
// ============================================================================

bool InclinometerController::initialize() {
    if (initialized_) return true;
    
    LOGI("Initializing CiA 410 inclinometer for slave %u", slave_addr_);
    
    if (!detectCapabilities()) {
        LOGE("Failed to detect inclinometer capabilities");
        return false;
    }
    
    update();
    initialized_ = true;
    
    LOGI("Inclinometer initialized: %u axes, range ±%ld mdeg",
         capabilities_.num_axes, capabilities_.range_max);
    
    return true;
}

bool InclinometerController::detectCapabilities() {
    readSDO(Object::DeviceType, 0, &capabilities_.device_type, sizeof(capabilities_.device_type));
    readSDO(Object::SensorType, 0, &capabilities_.sensor_type, sizeof(capabilities_.sensor_type));
    readSDO(Object::NumberOfAxes, 0, &capabilities_.num_axes, sizeof(capabilities_.num_axes));
    readSDO(Object::MeasurementRange, 0, &capabilities_.range_max, sizeof(capabilities_.range_max));
    capabilities_.range_min = -capabilities_.range_max;
    readSDO(Object::Resolution, 0, &capabilities_.resolution, sizeof(capabilities_.resolution));
    readSDO(Object::Accuracy, 0, &capabilities_.accuracy, sizeof(capabilities_.accuracy));
    
    // Check for gyroscope
    int16_t gyro_test;
    capabilities_.has_gyroscope = readSDO(Object::GyroRateX, 0, &gyro_test, sizeof(gyro_test));
    
    // Check for temperature sensor
    int16_t temp_test;
    capabilities_.has_temperature = readSDO(Object::Temperature, 0, &temp_test, sizeof(temp_test));
    
    // Check for acceleration
    int32_t accel_test;
    capabilities_.has_acceleration = readSDO(Object::AccelerationX, 0, &accel_test, sizeof(accel_test));
    
    return true;
}

// ============================================================================
// PDO
// ============================================================================

bool InclinometerController::applyPDOMapping(PDOMappingPreset preset) {
    current_mapping_ = preset;
    return true;
}

void InclinometerController::processTxPDO(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::SingleAxis:
            processSingleAxisPDO(data, len);
            break;
        case PDOMappingPreset::DualAxis:
            processDualAxisPDO(data, len);
            break;
        case PDOMappingPreset::Extended:
            processExtendedPDO(data, len);
            break;
        case PDOMappingPreset::Full:
            processFullPDO(data, len);
            break;
        default:
            processDualAxisPDO(data, len);
            break;
    }
    
    checkStateChanges();
}

void InclinometerController::processSingleAxisPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_SingleAxis)) return;
    const auto* pdo = reinterpret_cast<const TxPDO_SingleAxis*>(data);
    state_.statusword = pdo->statusword;
    state_.angle.x = pdo->angle_x;
}

void InclinometerController::processDualAxisPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_DualAxis)) return;
    const auto* pdo = reinterpret_cast<const TxPDO_DualAxis*>(data);
    state_.statusword = pdo->statusword;
    state_.angle.x = pdo->angle_x;
    state_.angle.y = pdo->angle_y;
}

void InclinometerController::processExtendedPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Extended)) return;
    const auto* pdo = reinterpret_cast<const TxPDO_Extended*>(data);
    state_.statusword = pdo->statusword;
    state_.angle.x = pdo->angle_x;
    state_.angle.y = pdo->angle_y;
    state_.acceleration.x = pdo->accel_x;
    state_.acceleration.y = pdo->accel_y;
    state_.temperature = pdo->temperature;
}

void InclinometerController::processFullPDO(const uint8_t* data, size_t len) {
    if (len < sizeof(TxPDO_Full)) return;
    const auto* pdo = reinterpret_cast<const TxPDO_Full*>(data);
    state_.statusword = pdo->statusword;
    state_.angle.x = pdo->angle_x;
    state_.angle.y = pdo->angle_y;
    state_.angle.z = pdo->angle_z;
    state_.velocity.x = pdo->velocity_x;
    state_.velocity.y = pdo->velocity_y;
    state_.velocity.z = pdo->velocity_z;
    state_.acceleration.x = pdo->accel_x;
    state_.acceleration.y = pdo->accel_y;
    state_.acceleration.z = pdo->accel_z;
    state_.temperature = pdo->temperature;
    state_.alarm_status = pdo->alarm_status;
}

size_t InclinometerController::prepareRxPDO(uint8_t* data, size_t max_len) {
    if (!data) return 0;
    
    if (max_len < sizeof(RxPDO_Basic)) return 0;
    auto* pdo = reinterpret_cast<RxPDO_Basic*>(data);
    pdo->controlword = controlword_;
    return sizeof(RxPDO_Basic);
}

void InclinometerController::update() {
    readSDO(Object::Statusword, 0, &state_.statusword, sizeof(state_.statusword));
    readSDO(Object::AngleActualX, 0, &state_.angle.x, sizeof(state_.angle.x));
    
    if (capabilities_.num_axes >= 2) {
        readSDO(Object::AngleActualY, 0, &state_.angle.y, sizeof(state_.angle.y));
    }
    if (capabilities_.num_axes >= 3) {
        readSDO(Object::AngleActualZ, 0, &state_.angle.z, sizeof(state_.angle.z));
    }
    
    readSDO(Object::TotalAngle, 0, &state_.angle.total, sizeof(state_.angle.total));
    
    if (capabilities_.has_gyroscope) {
        readSDO(Object::AngleVelocityX, 0, &state_.velocity.x, sizeof(state_.velocity.x));
        readSDO(Object::AngleVelocityY, 0, &state_.velocity.y, sizeof(state_.velocity.y));
        readSDO(Object::AngleVelocityZ, 0, &state_.velocity.z, sizeof(state_.velocity.z));
    }
    
    if (capabilities_.has_acceleration) {
        readSDO(Object::AccelerationX, 0, &state_.acceleration.x, sizeof(state_.acceleration.x));
        readSDO(Object::AccelerationY, 0, &state_.acceleration.y, sizeof(state_.acceleration.y));
        readSDO(Object::AccelerationZ, 0, &state_.acceleration.z, sizeof(state_.acceleration.z));
    }
    
    if (capabilities_.has_temperature) {
        readSDO(Object::Temperature, 0, &state_.temperature, sizeof(state_.temperature));
    }
    
    readSDO(Object::AlarmStatus, 0, &state_.alarm_status, sizeof(state_.alarm_status));
    readSDO(Object::FaultCode, 0, &state_.fault_code, sizeof(state_.fault_code));
    
    checkStateChanges();
}

void InclinometerController::checkStateChanges() {
    if (state_.statusword != prev_statusword_) {
        bool was_faulted = prev_statusword_ & StatuswordBits::Fault;
        bool is_faulted = state_.statusword & StatuswordBits::Fault;
        if (is_faulted && !was_faulted && fault_callback_) {
            fault_callback_(state_.fault_code);
        }
        prev_statusword_ = state_.statusword;
    }
    
    if (state_.alarm_status != prev_alarm_status_) {
        if (state_.alarm_status && alarm_callback_) {
            alarm_callback_(state_.alarm_status);
        }
        prev_alarm_status_ = state_.alarm_status;
    }
    
    if (data_callback_) {
        data_callback_(state_);
    }
}

// ============================================================================
// Basic Control
// ============================================================================

bool InclinometerController::enable() {
    controlword_ |= ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool InclinometerController::disable() {
    controlword_ &= ~ControlwordBits::Enable;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool InclinometerController::resetFault() {
    uint16_t cw = controlword_ | ControlwordBits::ResetFault;
    if (!writeSDO(Object::Controlword, 0, &cw, sizeof(cw))) return false;
    controlword_ &= ~ControlwordBits::ResetFault;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool InclinometerController::setZero() {
    uint16_t cw = controlword_ | ControlwordBits::SetZero;
    if (!writeSDO(Object::Controlword, 0, &cw, sizeof(cw))) return false;
    controlword_ &= ~ControlwordBits::SetZero;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool InclinometerController::selfTest() {
    uint16_t cw = controlword_ | ControlwordBits::SelfTest;
    if (!writeSDO(Object::Controlword, 0, &cw, sizeof(cw))) return false;
    controlword_ &= ~ControlwordBits::SelfTest;
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

// ============================================================================
// Configuration
// ============================================================================

bool InclinometerController::setOperatingMode(uint8_t mode) {
    if (!writeSDO(Object::OperatingMode, 0, &mode, sizeof(mode))) return false;
    operating_mode_ = mode;
    return true;
}

bool InclinometerController::setFilterSetting(uint8_t filter) {
    if (!writeSDO(Object::FilterSetting, 0, &filter, sizeof(filter))) return false;
    filter_setting_ = filter;
    return true;
}

bool InclinometerController::setSampleRate(uint16_t rate_hz) {
    return writeSDO(Object::SampleRate, 0, &rate_hz, sizeof(rate_hz));
}

uint16_t InclinometerController::getSampleRate() const {
    uint16_t rate;
    InclinometerController* self = const_cast<InclinometerController*>(this);
    if (self->readSDO(Object::SampleRate, 0, &rate, sizeof(rate))) {
        return rate;
    }
    return 0;
}

bool InclinometerController::setAveragingCount(uint8_t count) {
    return writeSDO(Object::AveragingCount, 0, &count, sizeof(count));
}

// ============================================================================
// Angle Reading
// ============================================================================

float InclinometerController::getAngleX() const {
    return millidegToDeg(state_.angle.x);
}

float InclinometerController::getAngleY() const {
    return millidegToDeg(state_.angle.y);
}

float InclinometerController::getAngleZ() const {
    return millidegToDeg(state_.angle.z);
}

float InclinometerController::getTotalAngle() const {
    return millidegToDeg(state_.angle.total);
}

int32_t InclinometerController::getAngleXRaw() const { return state_.angle.x; }
int32_t InclinometerController::getAngleYRaw() const { return state_.angle.y; }
int32_t InclinometerController::getAngleZRaw() const { return state_.angle.z; }

// ============================================================================
// Velocity Reading
// ============================================================================

float InclinometerController::getVelocityX() const {
    return rawToDegsPerSec(state_.velocity.x);
}

float InclinometerController::getVelocityY() const {
    return rawToDegsPerSec(state_.velocity.y);
}

float InclinometerController::getVelocityZ() const {
    return rawToDegsPerSec(state_.velocity.z);
}

// ============================================================================
// Acceleration Reading
// ============================================================================

float InclinometerController::getAccelerationX() const {
    return milligToG(state_.acceleration.x);
}

float InclinometerController::getAccelerationY() const {
    return milligToG(state_.acceleration.y);
}

float InclinometerController::getAccelerationZ() const {
    return milligToG(state_.acceleration.z);
}

// ============================================================================
// Temperature
// ============================================================================

float InclinometerController::getTemperature() const {
    return rawToTempC(state_.temperature);
}

bool InclinometerController::enableTemperatureCompensation(bool enable) {
    if (enable) {
        controlword_ |= ControlwordBits::EnableTempComp;
    } else {
        controlword_ &= ~ControlwordBits::EnableTempComp;
    }
    return writeSDO(Object::Controlword, 0, &controlword_, sizeof(controlword_));
}

bool InclinometerController::setTempCompCoefficients(int16_t coeff_x, int16_t coeff_y) {
    return writeSDO(Object::TempCoeffX, 0, &coeff_x, sizeof(coeff_x)) &&
           writeSDO(Object::TempCoeffY, 0, &coeff_y, sizeof(coeff_y));
}

// ============================================================================
// Calibration
// ============================================================================

bool InclinometerController::startAutoZero() {
    uint8_t cmd = CalibrationCommand::AutoZero;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::startOnePointCalibration() {
    uint8_t cmd = CalibrationCommand::OnePoint;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::startTwoPointCalibration() {
    uint8_t cmd = CalibrationCommand::TwoPoint;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::startGyroBiasCalibration() {
    uint8_t cmd = CalibrationCommand::GyroBiasCalib;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::startCrossAxisCalibration() {
    uint8_t cmd = CalibrationCommand::CrossAxisCalib;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::storeCalibration() {
    uint8_t cmd = CalibrationCommand::StoreCalibration;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

bool InclinometerController::resetCalibration() {
    uint8_t cmd = CalibrationCommand::ResetCalibration;
    return writeSDO(Object::CalibrationCommand, 0, &cmd, sizeof(cmd));
}

uint8_t InclinometerController::getCalibrationStatus() {
    uint8_t status;
    if (readSDO(Object::CalibrationStatus, 0, &status, sizeof(status))) {
        return status;
    }
    return 0xFF;
}

CalibrationData InclinometerController::getCalibrationData() {
    CalibrationData data;
    readSDO(Object::ZeroOffsetX, 0, &data.zero_offset_x, sizeof(data.zero_offset_x));
    readSDO(Object::ZeroOffsetY, 0, &data.zero_offset_y, sizeof(data.zero_offset_y));
    readSDO(Object::ZeroOffsetZ, 0, &data.zero_offset_z, sizeof(data.zero_offset_z));
    readSDO(Object::ScaleFactorX, 0, &data.scale_factor_x, sizeof(data.scale_factor_x));
    readSDO(Object::ScaleFactorY, 0, &data.scale_factor_y, sizeof(data.scale_factor_y));
    readSDO(Object::ScaleFactorZ, 0, &data.scale_factor_z, sizeof(data.scale_factor_z));
    readSDO(Object::CrossAxisX, 0, &data.cross_axis_x, sizeof(data.cross_axis_x));
    readSDO(Object::CrossAxisY, 0, &data.cross_axis_y, sizeof(data.cross_axis_y));
    readSDO(Object::CrossAxisZ, 0, &data.cross_axis_z, sizeof(data.cross_axis_z));
    return data;
}

bool InclinometerController::setCalibrationData(const CalibrationData& data) {
    bool success = true;
    success &= writeSDO(Object::ZeroOffsetX, 0, &data.zero_offset_x, sizeof(data.zero_offset_x));
    success &= writeSDO(Object::ZeroOffsetY, 0, &data.zero_offset_y, sizeof(data.zero_offset_y));
    success &= writeSDO(Object::ZeroOffsetZ, 0, &data.zero_offset_z, sizeof(data.zero_offset_z));
    success &= writeSDO(Object::ScaleFactorX, 0, &data.scale_factor_x, sizeof(data.scale_factor_x));
    success &= writeSDO(Object::ScaleFactorY, 0, &data.scale_factor_y, sizeof(data.scale_factor_y));
    success &= writeSDO(Object::ScaleFactorZ, 0, &data.scale_factor_z, sizeof(data.scale_factor_z));
    return success;
}

bool InclinometerController::setZeroOffset(int32_t x, int32_t y, int32_t z) {
    return writeSDO(Object::ZeroOffsetX, 0, &x, sizeof(x)) &&
           writeSDO(Object::ZeroOffsetY, 0, &y, sizeof(y)) &&
           writeSDO(Object::ZeroOffsetZ, 0, &z, sizeof(z));
}

bool InclinometerController::setScaleFactor(int32_t x, int32_t y, int32_t z) {
    return writeSDO(Object::ScaleFactorX, 0, &x, sizeof(x)) &&
           writeSDO(Object::ScaleFactorY, 0, &y, sizeof(y)) &&
           writeSDO(Object::ScaleFactorZ, 0, &z, sizeof(z));
}

// ============================================================================
// Mounting Configuration
// ============================================================================

bool InclinometerController::setMountingOrientation(uint8_t orientation) {
    return writeSDO(Object::MountingOrientation, 0, &orientation, sizeof(orientation));
}

bool InclinometerController::setMountingRotation(int16_t rot_x, int16_t rot_y, int16_t rot_z) {
    return writeSDO(Object::MountingRotationX, 0, &rot_x, sizeof(rot_x)) &&
           writeSDO(Object::MountingRotationY, 0, &rot_y, sizeof(rot_y)) &&
           writeSDO(Object::MountingRotationZ, 0, &rot_z, sizeof(rot_z));
}

// ============================================================================
// Alarms
// ============================================================================

bool InclinometerController::setAlarmThresholds(const AlarmThresholds& thresholds) {
    bool success = true;
    success &= writeSDO(Object::AngleThresholdHigh, 0, &thresholds.angle_high, sizeof(thresholds.angle_high));
    success &= writeSDO(Object::AngleThresholdLow, 0, &thresholds.angle_low, sizeof(thresholds.angle_low));
    success &= writeSDO(Object::AccelThresholdHigh, 0, &thresholds.accel_high, sizeof(thresholds.accel_high));
    success &= writeSDO(Object::TempThresholdHigh, 0, &thresholds.temp_high, sizeof(thresholds.temp_high));
    success &= writeSDO(Object::TempThresholdLow, 0, &thresholds.temp_low, sizeof(thresholds.temp_low));
    success &= writeSDO(Object::VelocityThreshold, 0, &thresholds.velocity_threshold, sizeof(thresholds.velocity_threshold));
    success &= writeSDO(Object::Hysteresis, 0, &thresholds.hysteresis, sizeof(thresholds.hysteresis));
    return success;
}

AlarmThresholds InclinometerController::getAlarmThresholds() {
    AlarmThresholds thresholds;
    readSDO(Object::AngleThresholdHigh, 0, &thresholds.angle_high, sizeof(thresholds.angle_high));
    readSDO(Object::AngleThresholdLow, 0, &thresholds.angle_low, sizeof(thresholds.angle_low));
    readSDO(Object::AccelThresholdHigh, 0, &thresholds.accel_high, sizeof(thresholds.accel_high));
    readSDO(Object::TempThresholdHigh, 0, &thresholds.temp_high, sizeof(thresholds.temp_high));
    readSDO(Object::TempThresholdLow, 0, &thresholds.temp_low, sizeof(thresholds.temp_low));
    readSDO(Object::VelocityThreshold, 0, &thresholds.velocity_threshold, sizeof(thresholds.velocity_threshold));
    readSDO(Object::Hysteresis, 0, &thresholds.hysteresis, sizeof(thresholds.hysteresis));
    return thresholds;
}

bool InclinometerController::enableAlarms(uint16_t alarm_mask) {
    return writeSDO(Object::AlarmEnable, 0, &alarm_mask, sizeof(alarm_mask));
}

bool InclinometerController::isAlarmActive(uint16_t alarm_bit) const {
    return state_.alarm_status & alarm_bit;
}

// ============================================================================
// Diagnostics
// ============================================================================

uint8_t InclinometerController::getSensorHealth() {
    uint8_t health;
    if (readSDO(Object::SensorHealth, 0, &health, sizeof(health))) {
        return health;
    }
    return 0;
}

uint8_t InclinometerController::getSignalQuality() {
    uint8_t quality;
    if (readSDO(Object::SignalQuality, 0, &quality, sizeof(quality))) {
        return quality;
    }
    return 0;
}

uint32_t InclinometerController::getOperatingHours() {
    uint32_t hours;
    if (readSDO(Object::OperatingHours, 0, &hours, sizeof(hours))) {
        return hours;
    }
    return 0;
}

std::string InclinometerController::getDiagnostics() const {
    std::string result;
    result += "Inclinometer Status:\n";
    result += "  Ready: " + std::string(state_.isReady() ? "Yes" : "No") + "\n";
    result += "  Data Valid: " + std::string(state_.isDataValid() ? "Yes" : "No") + "\n";
    result += "  Calibrated: " + std::string(state_.isCalibrated() ? "Yes" : "No") + "\n";
    result += "  Angle X: " + std::to_string(getAngleX()) + " deg\n";
    result += "  Angle Y: " + std::to_string(getAngleY()) + " deg\n";
    if (capabilities_.num_axes >= 3) {
        result += "  Angle Z: " + std::to_string(getAngleZ()) + " deg\n";
    }
    if (capabilities_.has_temperature) {
        result += "  Temperature: " + std::to_string(getTemperature()) + " C\n";
    }
    if (state_.fault_code != 0) {
        result += "  Fault: 0x" + std::to_string(state_.fault_code) + "\n";
    }
    return result;
}

// ============================================================================
// Callbacks
// ============================================================================

void InclinometerController::setAlarmCallback(AlarmCallback callback) {
    alarm_callback_ = std::move(callback);
}

void InclinometerController::setFaultCallback(FaultCallback callback) {
    fault_callback_ = std::move(callback);
}

void InclinometerController::setDataCallback(DataCallback callback) {
    data_callback_ = std::move(callback);
}

// ============================================================================
// SDO Helpers
// ============================================================================

bool InclinometerController::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) {
    return ecm_sdo_read(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

bool InclinometerController::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) {
    return ecm_sdo_write(slave_addr_, index, subindex, data, len, use_configured_addr_);
}

} // namespace CiA410
