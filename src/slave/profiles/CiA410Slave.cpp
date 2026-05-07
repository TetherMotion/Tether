/**
 * @file CiA410Slave.cpp
 * @brief CiA 410 Inclinometer Slave Implementation
 */

#include "slave/profiles/CiA410Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA410Slave Implementation
// ============================================================================

CiA410Slave::CiA410Slave(const CiA410SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA410, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , incConfig_(config)
{
    filterSetting_ = config.filterSetting;
}

CiA410Slave::~CiA410Slave() = default;

void CiA410Slave::initObjectDictionary() {
    ProfileSlave::registerCiA301Objects();
    // Minimal implementation - object dictionary registration would go here
}

void CiA410Slave::initPDOMappings() {
    // Minimal implementation - PDO mappings would go here
}

void CiA410Slave::updateTxPDO() {
    // Minimal implementation - transmit PDO update would go here
}

void CiA410Slave::processRxPDO() {
    // Minimal implementation - receive PDO processing would go here
}

void CiA410Slave::simulate(uint64_t deltaNs) {
    if (inclinationCallback_) {
        inclinationCallback_(rawInclinationX_, rawInclinationY_);
    }
    
    simulateSensorNoise(deltaNs);
    simulateTemperature(deltaNs);
    handleCalibrationProcess(deltaNs);
    applyFilter();
    applyCalibration();
    if (tempCompensationEnabled_) {
        applyTemperatureCompensation();
    }
    checkAlarms();
}

void CiA410Slave::setInclination(int32_t x, int32_t y) {
    rawInclinationX_ = x;
    rawInclinationY_ = y;
}

void CiA410Slave::setTemperature(int16_t temp) {
    temperature_ = temp;
}

void CiA410Slave::startCalibration() {
    calibrating_ = true;
    calibrationAccumulator_ = 0;
    calibrationSampleCount_ = 0;
    calibrationSumX_ = 0;
    calibrationSumY_ = 0;
    operatingStatus_ |= 0x0010;  // Set calibrating bit
}

void CiA410Slave::setCalibrationOffset(int32_t offsetX, int32_t offsetY) {
    calibrationOffsetX_ = offsetX;
    calibrationOffsetY_ = offsetY;
}

void CiA410Slave::setAlarmLimits(int32_t lowX, int32_t highX, int32_t lowY, int32_t highY) {
    alarmLowX_ = lowX;
    alarmHighX_ = highX;
    alarmLowY_ = lowY;
    alarmHighY_ = highY;
}

void CiA410Slave::setFilterSetting(uint8_t setting) {
    filterSetting_ = setting & 0x07;  // Limit to 0-7
}

void CiA410Slave::processControlWord(uint16_t controlWord) {
    controlWord_ = controlWord;
    // Minimal implementation
}

void CiA410Slave::applyFilter() {
    // Simple exponential moving average filter
    float alpha = 1.0f / (1.0f + filterSetting_);
    filteredInclinationX_ = alpha * rawInclinationX_ + (1.0f - alpha) * filteredInclinationX_;
    filteredInclinationY_ = alpha * rawInclinationY_ + (1.0f - alpha) * filteredInclinationY_;
    inclinationX_ = static_cast<int32_t>(filteredInclinationX_);
    inclinationY_ = static_cast<int32_t>(filteredInclinationY_);
}

void CiA410Slave::applyCalibration() {
    inclinationX_ = (inclinationX_ - calibrationOffsetX_) * calibrationGainX_ / 1000;
    inclinationY_ = (inclinationY_ - calibrationOffsetY_) * calibrationGainY_ / 1000;
}

void CiA410Slave::applyTemperatureCompensation() {
    // Minimal temperature compensation
    int16_t tempDelta = temperature_ - 250;  // Reference 25.0°C
    int32_t compensation = tempDelta * 10 / 100;  // Simple linear compensation
    inclinationX_ -= compensation;
    inclinationY_ -= compensation;
}

void CiA410Slave::simulateSensorNoise(uint64_t deltaNs) {
    (void)deltaNs;
    // Minimal implementation - no noise added
}

void CiA410Slave::simulateTemperature(uint64_t deltaNs) {
    (void)deltaNs;
    // Minimal implementation - temperature stays constant
}

void CiA410Slave::handleCalibrationProcess(uint64_t deltaNs) {
    if (!calibrating_) {
        return;
    }
    
    calibrationAccumulator_ += deltaNs;
    calibrationSumX_ += rawInclinationX_;
    calibrationSumY_ += rawInclinationY_;
    calibrationSampleCount_++;
    
    // Complete calibration after 1 second
    if (calibrationAccumulator_ >= 1000000000ULL && calibrationSampleCount_ > 0) {
        calibrationOffsetX_ = static_cast<int32_t>(calibrationSumX_ / calibrationSampleCount_);
        calibrationOffsetY_ = static_cast<int32_t>(calibrationSumY_ / calibrationSampleCount_);
        calibrating_ = false;
        operatingStatus_ &= ~0x0010;  // Clear calibrating bit
    }
}

void CiA410Slave::checkAlarms() {
    alarmStatus_ = 0;
    if (inclinationX_ < alarmLowX_ || inclinationX_ > alarmHighX_) {
        alarmStatus_ |= 0x01;
    }
    if (inclinationY_ < alarmLowY_ || inclinationY_ > alarmHighY_) {
        alarmStatus_ |= 0x02;
    }
}

std::unique_ptr<CiA410Slave> createCiA410Slave(const CiA410SlaveConfig& config) {
    return std::make_unique<CiA410Slave>(config);
}

std::unique_ptr<CiA410Slave> createSingleAxisInclinometer() {
    CiA410SlaveConfig config;
    config.type = InclinometerType::SingleAxis;
    return std::make_unique<CiA410Slave>(config);
}

std::unique_ptr<CiA410Slave> createDualAxisInclinometer() {
    CiA410SlaveConfig config;
    config.type = InclinometerType::DualAxis;
    return std::make_unique<CiA410Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
