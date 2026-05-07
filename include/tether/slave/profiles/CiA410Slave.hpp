/**
 * @file CiA410Slave.hpp
 * @brief CiA 410 Inclinometer Slave Implementation
 *
 * @details
 * Implements a CiA 410 compliant inclinometer slave with:
 * - Single or dual axis inclination measurement
 * - Temperature compensation
 * - Calibration support
 * - Alarm handling
 *
 * ## Measurement Units
 *
 * - Angle: 0.001 degrees (milli-degrees)
 * - Temperature: 0.1 °C (deci-degrees)
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Inclinometer Type
// ============================================================================

enum class InclinometerType : uint8_t {
    SingleAxis = 1,
    DualAxis   = 2,
};

// ============================================================================
// CiA 410 Slave Configuration
// ============================================================================

struct CiA410SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x0000019A,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 410 Inclinometer",
    };
    
    InclinometerType type = InclinometerType::DualAxis;
    
    // Measurement range (milli-degrees)
    int32_t measurementRangeMin = -90000;   // -90°
    int32_t measurementRangeMax = 90000;    // +90°
    
    // Resolution (milli-degrees)
    int32_t resolution = 10;                 // 0.01° resolution
    
    // Temperature
    bool hasTemperatureSensor = true;
    int16_t temperatureMin = -400;           // -40°C
    int16_t temperatureMax = 850;            // +85°C
    
    // Filtering
    uint8_t filterSetting = 4;               // Filter strength (0-7)
    
    // Calibration
    bool supportsCalibration = true;
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 410 Slave Class
// ============================================================================

class CiA410Slave : public ProfileSlave {
public:
    explicit CiA410Slave(const CiA410SlaveConfig& config);
    ~CiA410Slave() override;
    
    const char* getProfileName() const override { return "CiA 410"; }
    uint32_t getDeviceType() const override { return 0x0000019A; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Inclination (in milli-degrees, 0.001°)
    int32_t getInclinationX() const { return inclinationX_; }
    int32_t getInclinationY() const { return inclinationY_; }
    void setInclination(int32_t x, int32_t y);  // For simulation
    
    // Temperature (in deci-degrees, 0.1°C)
    int16_t getTemperature() const { return temperature_; }
    void setTemperature(int16_t temp);  // For simulation
    
    // Operating status
    uint16_t getOperatingStatus() const { return operatingStatus_; }
    bool isReady() const { return (operatingStatus_ & 0x0001) != 0; }
    bool isCalibrating() const { return (operatingStatus_ & 0x0010) != 0; }
    
    // Calibration
    void startCalibration();
    void setCalibrationOffset(int32_t offsetX, int32_t offsetY);
    
    // Alarm
    bool isAlarmActive() const { return alarmStatus_ != 0; }
    uint8_t getAlarmStatus() const { return alarmStatus_; }
    void setAlarmLimits(int32_t lowX, int32_t highX, int32_t lowY, int32_t highY);
    
    // Filter
    void setFilterSetting(uint8_t setting);
    uint8_t getFilterSetting() const { return filterSetting_; }
    
    // Simulation callback (returns inclination based on external physics)
    using InclinationCallback = std::function<void(int32_t& x, int32_t& y)>;
    void setInclinationCallback(InclinationCallback callback) { inclinationCallback_ = callback; }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA410SlaveConfig incConfig_;
    
    // Measurements
    int32_t inclinationX_ = 0;        // X-axis inclination (milli-degrees)
    int32_t inclinationY_ = 0;        // Y-axis inclination (milli-degrees)
    int32_t rawInclinationX_ = 0;     // Unfiltered
    int32_t rawInclinationY_ = 0;
    int16_t temperature_ = 250;       // Temperature (deci-degrees, 25.0°C default)
    int16_t temperatureOffset_ = 0;   // Temperature offset for compensation
    
    // Filtered values (for EMA filter)
    float filteredInclinationX_ = 0.0f;
    float filteredInclinationY_ = 0.0f;
    
    // Calibration
    int32_t calibrationOffsetX_ = 0;
    int32_t calibrationOffsetY_ = 0;
    int32_t calibrationGainX_ = 1000;   // Gain factor (1000 = 1.000)
    int32_t calibrationGainY_ = 1000;
    bool calibrating_ = false;
    bool tempCompensationEnabled_ = false;
    
    // Calibration process state
    uint64_t calibrationAccumulator_ = 0;
    uint32_t calibrationSampleCount_ = 0;
    int64_t calibrationSumX_ = 0;
    int64_t calibrationSumY_ = 0;
    
    // Status
    uint16_t operatingStatus_ = 0x0001;  // Ready by default
    uint16_t controlWord_ = 0;           // Last received control word
    uint8_t alarmStatus_ = 0;
    
    // Alarm limits
    int32_t alarmLowX_ = -90000;
    int32_t alarmHighX_ = 90000;
    int32_t alarmLowY_ = -90000;
    int32_t alarmHighY_ = 90000;
    
    // Filter
    uint8_t filterSetting_ = 4;
    
    InclinationCallback inclinationCallback_;
    
    // Private methods
    void processControlWord(uint16_t controlWord);
    void applyFilter();
    void applyCalibration();
    void applyTemperatureCompensation();
    void simulateSensorNoise(uint64_t deltaNs);
    void simulateTemperature(uint64_t deltaNs);
    void handleCalibrationProcess(uint64_t deltaNs);
    void checkAlarms();
};

std::unique_ptr<CiA410Slave> createCiA410Slave(const CiA410SlaveConfig& config);

// Factory functions
std::unique_ptr<CiA410Slave> createSingleAxisInclinometer();
std::unique_ptr<CiA410Slave> createDualAxisInclinometer();

}  // namespace slave
}  // namespace EtherCAT
