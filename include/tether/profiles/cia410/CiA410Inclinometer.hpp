/**
 * @file CiA410Inclinometer.hpp
 * @brief CiA 410 Inclinometer Controller
 *
 * Provides comprehensive control over CiA 410 compliant inclinometers
 * and tilt sensors.
 *
 * Features:
 * - Single, dual, and triple axis support
 * - MEMS accelerometer and gyroscope fusion
 * - Configurable filtering and sample rates
 * - Temperature compensation
 * - Alarm and threshold monitoring
 * - Comprehensive calibration support
 */

#pragma once

#include "profiles/cia410/CiA410Defs.hpp"
#include <cstdint>
#include <string>
#include <functional>

namespace CiA410 {

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Device capabilities
 */
struct InclinometerCapabilities {
    uint8_t  device_type = 0;
    uint8_t  sensor_type = 0;
    uint8_t  num_axes = 1;
    bool     has_gyroscope = false;
    bool     has_temperature = false;
    bool     has_acceleration = false;
    int32_t  range_min = 0;      // millidegrees
    int32_t  range_max = 0;
    uint16_t resolution = 0;     // millidegrees
    uint16_t accuracy = 0;       // millidegrees
};

/**
 * @brief Angle measurements
 */
struct AngleReading {
    int32_t x = 0;    // millidegrees
    int32_t y = 0;
    int32_t z = 0;
    int32_t total = 0;     // Resultant tilt
    int16_t azimuth = 0;   // Direction of tilt
    
    float getXDegrees() const { return millidegToDeg(x); }
    float getYDegrees() const { return millidegToDeg(y); }
    float getZDegrees() const { return millidegToDeg(z); }
    float getTotalDegrees() const { return millidegToDeg(total); }
};

/**
 * @brief Angular velocity
 */
struct VelocityReading {
    int16_t x = 0;    // 0.01 deg/s
    int16_t y = 0;
    int16_t z = 0;
    
    float getXDegsPerSec() const { return rawToDegsPerSec(x); }
    float getYDegsPerSec() const { return rawToDegsPerSec(y); }
    float getZDegsPerSec() const { return rawToDegsPerSec(z); }
};

/**
 * @brief Acceleration data
 */
struct AccelerationReading {
    int32_t x = 0;    // milli-g
    int32_t y = 0;
    int32_t z = 0;
    int32_t total = 0;
    
    float getXG() const { return milligToG(x); }
    float getYG() const { return milligToG(y); }
    float getZG() const { return milligToG(z); }
};

/**
 * @brief Current sensor state
 */
struct InclinometerState {
    uint16_t statusword = 0;
    AngleReading angle;
    VelocityReading velocity;
    AccelerationReading acceleration;
    int16_t temperature = 0;    // 0.1°C
    uint16_t alarm_status = 0;
    uint16_t fault_code = 0;
    uint16_t warning_code = 0;
    
    // Status helpers
    bool isReady() const { return statusword & StatuswordBits::Ready; }
    bool isDataValid() const { return statusword & StatuswordBits::DataValid; }
    bool isCalibrated() const { return statusword & StatuswordBits::Calibrated; }
    bool isMotionDetected() const { return statusword & StatuswordBits::MotionDetected; }
    bool hasAlarm() const { return statusword & StatuswordBits::AlarmActive; }
    bool hasFault() const { return statusword & StatuswordBits::Fault; }
    bool isOverRange() const { return statusword & StatuswordBits::OverRange; }
    bool isSettling() const { return statusword & StatuswordBits::Settling; }
    float getTemperatureCelsius() const { return rawToTempC(temperature); }
};

/**
 * @brief Calibration data
 */
struct CalibrationData {
    int32_t zero_offset_x = 0;
    int32_t zero_offset_y = 0;
    int32_t zero_offset_z = 0;
    int32_t scale_factor_x = 10000;  // 1.0000
    int32_t scale_factor_y = 10000;
    int32_t scale_factor_z = 10000;
    int16_t cross_axis_x = 0;
    int16_t cross_axis_y = 0;
    int16_t cross_axis_z = 0;
};

/**
 * @brief Alarm thresholds
 */
struct AlarmThresholds {
    int32_t angle_high = 450000;   // 45 degrees
    int32_t angle_low = -450000;
    int32_t accel_high = 2000;     // 2g
    int16_t temp_high = 850;       // 85°C
    int16_t temp_low = -400;       // -40°C
    int16_t velocity_threshold = 1000;  // 10 deg/s
    uint16_t hysteresis = 1000;    // 1 degree
};

// ============================================================================
// Callback Types
// ============================================================================

using AlarmCallback = std::function<void(uint16_t alarm_status)>;
using FaultCallback = std::function<void(uint16_t fault_code)>;
using DataCallback = std::function<void(const InclinometerState& state)>;

// ============================================================================
// PDO Mapping Presets
// ============================================================================

enum class PDOMappingPreset {
    SingleAxis,
    DualAxis,
    Extended,
    Full,
    Custom
};

// ============================================================================
// Inclinometer Controller Class
// ============================================================================

class InclinometerController {
public:
    explicit InclinometerController(uint16_t slave_addr, bool use_configured_addr = false);
    ~InclinometerController();
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    bool initialize();
    bool isInitialized() const { return initialized_; }
    const InclinometerCapabilities& getCapabilities() const { return capabilities_; }
    
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
    bool setZero();  // Tare
    bool selfTest();
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    bool setOperatingMode(uint8_t mode);
    uint8_t getOperatingMode() const { return operating_mode_; }
    
    bool setFilterSetting(uint8_t filter);
    uint8_t getFilterSetting() const { return filter_setting_; }
    
    bool setSampleRate(uint16_t rate_hz);
    uint16_t getSampleRate() const;
    
    bool setAveragingCount(uint8_t count);
    
    // ========================================================================
    // Angle Reading
    // ========================================================================
    
    float getAngleX() const;  // degrees
    float getAngleY() const;
    float getAngleZ() const;
    float getTotalAngle() const;
    
    int32_t getAngleXRaw() const;  // millidegrees
    int32_t getAngleYRaw() const;
    int32_t getAngleZRaw() const;
    
    const AngleReading& getAngle() const { return state_.angle; }
    
    // ========================================================================
    // Velocity Reading (if gyro equipped)
    // ========================================================================
    
    float getVelocityX() const;  // deg/s
    float getVelocityY() const;
    float getVelocityZ() const;
    
    const VelocityReading& getVelocity() const { return state_.velocity; }
    
    // ========================================================================
    // Acceleration Reading
    // ========================================================================
    
    float getAccelerationX() const;  // g
    float getAccelerationY() const;
    float getAccelerationZ() const;
    
    const AccelerationReading& getAcceleration() const { return state_.acceleration; }
    
    // ========================================================================
    // Temperature
    // ========================================================================
    
    float getTemperature() const;  // °C
    bool enableTemperatureCompensation(bool enable);
    bool setTempCompCoefficients(int16_t coeff_x, int16_t coeff_y);
    
    // ========================================================================
    // Calibration
    // ========================================================================
    
    bool startAutoZero();
    bool startOnePointCalibration();
    bool startTwoPointCalibration();
    bool startGyroBiasCalibration();
    bool startCrossAxisCalibration();
    bool storeCalibration();
    bool resetCalibration();
    
    uint8_t getCalibrationStatus();
    CalibrationData getCalibrationData();
    bool setCalibrationData(const CalibrationData& data);
    
    bool setZeroOffset(int32_t x, int32_t y, int32_t z = 0);
    bool setScaleFactor(int32_t x, int32_t y, int32_t z = 10000);
    
    // ========================================================================
    // Mounting Configuration
    // ========================================================================
    
    bool setMountingOrientation(uint8_t orientation);
    bool setMountingRotation(int16_t rot_x, int16_t rot_y, int16_t rot_z);
    
    // ========================================================================
    // Alarms
    // ========================================================================
    
    bool setAlarmThresholds(const AlarmThresholds& thresholds);
    AlarmThresholds getAlarmThresholds();
    
    bool enableAlarms(uint16_t alarm_mask);
    uint16_t getAlarmStatus() const { return state_.alarm_status; }
    bool isAlarmActive(uint16_t alarm_bit) const;
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    const InclinometerState& getState() const { return state_; }
    uint16_t getFaultCode() const { return state_.fault_code; }
    
    uint8_t getSensorHealth();
    uint8_t getSignalQuality();
    uint32_t getOperatingHours();
    std::string getDiagnostics() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setAlarmCallback(AlarmCallback callback);
    void setFaultCallback(FaultCallback callback);
    void setDataCallback(DataCallback callback);

private:
    bool detectCapabilities();
    void processSingleAxisPDO(const uint8_t* data, size_t len);
    void processDualAxisPDO(const uint8_t* data, size_t len);
    void processExtendedPDO(const uint8_t* data, size_t len);
    void processFullPDO(const uint8_t* data, size_t len);
    void checkStateChanges();
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    InclinometerCapabilities capabilities_;
    InclinometerState state_;
    uint16_t prev_statusword_;
    uint16_t prev_alarm_status_;
    
    uint16_t controlword_;
    uint8_t operating_mode_;
    uint8_t filter_setting_;
    PDOMappingPreset current_mapping_;
    
    AlarmCallback alarm_callback_;
    FaultCallback fault_callback_;
    DataCallback data_callback_;
};

} // namespace CiA410
