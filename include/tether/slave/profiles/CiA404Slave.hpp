/**
 * @file CiA404Slave.hpp
 * @brief CiA 404 Measuring Devices and Closed-Loop Controllers Slave
 *
 * @details
 * Implements a CiA 404 compliant measuring device / PID controller slave with:
 * - Process value reading and scaling
 * - Setpoint input
 * - PID control with configurable parameters
 * - Alarm handling
 *
 * ## Object Dictionary Structure
 *
 * | Index Range | Description |
 * |-------------|-------------|
 * | 0x6000-0x60FF | Process input |
 * | 0x6100-0x61FF | Process input scaling |
 * | 0x6300-0x63FF | Setpoint |
 * | 0x6400-0x64FF | Controller |
 * | 0x6500-0x65FF | Controller output |
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 404 Slave Configuration
// ============================================================================

struct CiA404SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000194,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 404 Measuring Device",
    };
    
    // Number of process input channels
    uint8_t processInputChannels = 1;
    
    // Number of controller channels
    uint8_t controllerChannels = 1;
    
    // Default PID parameters
    float defaultKp = 1.0f;
    float defaultKi = 0.0f;
    float defaultKd = 0.0f;
    
    // Scaling
    int32_t inputScaleMin = 0;
    int32_t inputScaleMax = 32767;
    int32_t outputScaleMin = 0;
    int32_t outputScaleMax = 32767;
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 404 Slave Class
// ============================================================================

class CiA404Slave : public ProfileSlave {
public:
    explicit CiA404Slave(const CiA404SlaveConfig& config);
    ~CiA404Slave() override;
    
    const char* getProfileName() const override { return "CiA 404"; }
    uint32_t getDeviceType() const override { return 0x00000194; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Process input
    void setProcessValue(size_t channel, int32_t value);
    int32_t getProcessValue(size_t channel) const;
    
    // Setpoint
    int32_t getSetpoint(size_t channel) const;
    
    // Controller output
    int32_t getControllerOutput(size_t channel) const;
    
    // PID parameters
    void setPIDParameters(size_t channel, float kp, float ki, float kd);
    void getPIDParameters(size_t channel, float& kp, float& ki, float& kd) const;
    
    // Enable/disable controller
    void setControllerEnabled(size_t channel, bool enabled);
    bool isControllerEnabled(size_t channel) const;
    
    // Alarm
    void setAlarmLimits(size_t channel, int32_t low, int32_t high);
    bool isAlarmActive(size_t channel) const;
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA404SlaveConfig config_;
    
    // Process values
    std::array<int32_t, 8> processValues_;
    std::array<int32_t, 8> setpoints_;
    std::array<int32_t, 8> controllerOutputs_;
    
    // PID parameters
    struct PIDParams {
        float kp = 1.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float integral = 0.0f;
        float prevError = 0.0f;
        bool enabled = false;
    };
    std::array<PIDParams, 8> pidParams_;
    
    // Alarms
    struct AlarmConfig {
        int32_t lowLimit = INT32_MIN;
        int32_t highLimit = INT32_MAX;
        bool active = false;
    };
    std::array<AlarmConfig, 8> alarms_;
    
    void updateController(size_t channel, float dt);
    void processControlWord(uint16_t controlWord);
    void checkAlarms(size_t channel);
};

std::unique_ptr<CiA404Slave> createCiA404Slave(const CiA404SlaveConfig& config);

}  // namespace slave
}  // namespace EtherCAT
