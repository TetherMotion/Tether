/**
 * @file CiA404Slave.cpp
 * @brief CiA 404 Measuring Devices and Closed-Loop Controllers Slave Implementation
 */

#include "slave/profiles/CiA404Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA404Slave Implementation
// ============================================================================

CiA404Slave::CiA404Slave(const CiA404SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA404, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , config_(config)
{
    processValues_.fill(0);
    setpoints_.fill(0);
    controllerOutputs_.fill(0);
    
    for (auto& pid : pidParams_) {
        pid.kp = config.defaultKp;
        pid.ki = config.defaultKi;
        pid.kd = config.defaultKd;
        pid.integral = 0.0f;
        pid.prevError = 0.0f;
        pid.enabled = false;
    }
    
    for (auto& alarm : alarms_) {
        alarm.lowLimit = INT32_MIN;
        alarm.highLimit = INT32_MAX;
        alarm.active = false;
    }
}

CiA404Slave::~CiA404Slave() = default;

void CiA404Slave::initObjectDictionary() {
    ProfileSlave::registerCiA301Objects();
    // Minimal implementation - object dictionary registration would go here
}

void CiA404Slave::initPDOMappings() {
    // Minimal implementation - PDO mappings would go here
}

void CiA404Slave::updateTxPDO() {
    // Minimal implementation - transmit PDO update would go here
}

void CiA404Slave::processRxPDO() {
    // Minimal implementation - receive PDO processing would go here
}

void CiA404Slave::simulate(uint64_t deltaNs) {
    float dt = deltaNs / 1e9f;
    for (size_t i = 0; i < config_.controllerChannels && i < 8; ++i) {
        updateController(i, dt);
        checkAlarms(i);
    }
}

void CiA404Slave::setProcessValue(size_t channel, int32_t value) {
    if (channel < processValues_.size()) {
        processValues_[channel] = value;
    }
}

int32_t CiA404Slave::getProcessValue(size_t channel) const {
    if (channel < processValues_.size()) {
        return processValues_[channel];
    }
    return 0;
}

int32_t CiA404Slave::getSetpoint(size_t channel) const {
    if (channel < setpoints_.size()) {
        return setpoints_[channel];
    }
    return 0;
}

int32_t CiA404Slave::getControllerOutput(size_t channel) const {
    if (channel < controllerOutputs_.size()) {
        return controllerOutputs_[channel];
    }
    return 0;
}

void CiA404Slave::setPIDParameters(size_t channel, float kp, float ki, float kd) {
    if (channel < pidParams_.size()) {
        pidParams_[channel].kp = kp;
        pidParams_[channel].ki = ki;
        pidParams_[channel].kd = kd;
    }
}

void CiA404Slave::getPIDParameters(size_t channel, float& kp, float& ki, float& kd) const {
    if (channel < pidParams_.size()) {
        kp = pidParams_[channel].kp;
        ki = pidParams_[channel].ki;
        kd = pidParams_[channel].kd;
    } else {
        kp = ki = kd = 0.0f;
    }
}

void CiA404Slave::setControllerEnabled(size_t channel, bool enabled) {
    if (channel < pidParams_.size()) {
        pidParams_[channel].enabled = enabled;
        if (!enabled) {
            pidParams_[channel].integral = 0.0f;
            pidParams_[channel].prevError = 0.0f;
        }
    }
}

bool CiA404Slave::isControllerEnabled(size_t channel) const {
    if (channel < pidParams_.size()) {
        return pidParams_[channel].enabled;
    }
    return false;
}

void CiA404Slave::setAlarmLimits(size_t channel, int32_t low, int32_t high) {
    if (channel < alarms_.size()) {
        alarms_[channel].lowLimit = low;
        alarms_[channel].highLimit = high;
    }
}

bool CiA404Slave::isAlarmActive(size_t channel) const {
    if (channel < alarms_.size()) {
        return alarms_[channel].active;
    }
    return false;
}

void CiA404Slave::updateController(size_t channel, float dt) {
    if (channel >= pidParams_.size() || !pidParams_[channel].enabled) {
        return;
    }
    
    auto& pid = pidParams_[channel];
    float error = static_cast<float>(setpoints_[channel] - processValues_[channel]);
    
    pid.integral += error * dt;
    float derivative = (dt > 0) ? (error - pid.prevError) / dt : 0.0f;
    
    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    controllerOutputs_[channel] = static_cast<int32_t>(output);
    
    pid.prevError = error;
}

void CiA404Slave::processControlWord(uint16_t controlWord) {
    (void)controlWord;
    // Minimal implementation
}

void CiA404Slave::checkAlarms(size_t channel) {
    if (channel >= alarms_.size()) {
        return;
    }
    
    int32_t value = processValues_[channel];
    alarms_[channel].active = (value < alarms_[channel].lowLimit || 
                               value > alarms_[channel].highLimit);
}

std::unique_ptr<CiA404Slave> createCiA404Slave(const CiA404SlaveConfig& config) {
    return std::make_unique<CiA404Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
