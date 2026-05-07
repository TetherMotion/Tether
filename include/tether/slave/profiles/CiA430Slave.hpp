/**
 * @file CiA430Slave.hpp
 * @brief CiA 430 Power Supply Slave Implementation
 *
 * @details
 * Implements a CiA 430 compliant power supply slave with:
 * - DC power supply control
 * - Voltage and current regulation
 * - Output enable/disable
 * - Protection features (OVP, OCP, OTP)
 * - Multi-channel support
 *
 * ## Protection Features
 *
 * - OVP: Over-Voltage Protection
 * - OCP: Over-Current Protection
 * - OTP: Over-Temperature Protection
 * - UVP: Under-Voltage Protection
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Power Supply State
// ============================================================================

enum class PowerSupplyState : uint8_t {
    Off           = 0x00,
    Initializing  = 0x01,
    Ready         = 0x02,
    Running       = 0x03,
    Fault         = 0x04,
    Calibrating   = 0x05,
};

enum class ProtectionType : uint8_t {
    None          = 0x00,
    OVP           = 0x01,   // Over-voltage
    OCP           = 0x02,   // Over-current
    OTP           = 0x04,   // Over-temperature
    UVP           = 0x08,   // Under-voltage
};

// ============================================================================
// Channel Configuration
// ============================================================================

struct PowerChannelConfig {
    uint32_t nominalVoltage = 24000;    // mV (24V)
    uint32_t maxVoltage = 30000;        // mV (30V)
    uint32_t nominalCurrent = 5000;     // mA (5A)
    uint32_t maxCurrent = 6000;         // mA (6A)
    
    uint32_t ovpThreshold = 28000;      // mV
    uint32_t ocpThreshold = 5500;       // mA
    uint32_t uvpThreshold = 20000;      // mV
    
    bool hasRemoteSensing = false;
    bool hasSoftStart = true;
    uint16_t softStartTimeMs = 100;
};

// ============================================================================
// CiA 430 Slave Configuration
// ============================================================================

struct CiA430SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x000001AE,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 430 Power Supply",
    };
    
    uint8_t numberOfChannels = 1;
    std::array<PowerChannelConfig, 4> channels;  // Max 4 channels
    
    // Temperature monitoring
    bool hasTemperatureSensor = true;
    int16_t maxTemperature = 70;         // °C
    
    // AC input monitoring (if applicable)
    bool hasACMonitoring = false;
    uint16_t acVoltageNominal = 230;     // V RMS
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 430 Slave Class
// ============================================================================

class CiA430Slave : public ProfileSlave {
public:
    static constexpr uint8_t MAX_CHANNELS = 4;
    
    explicit CiA430Slave(const CiA430SlaveConfig& config);
    ~CiA430Slave() override;
    
    const char* getProfileName() const override { return "CiA 430"; }
    uint32_t getDeviceType() const override { return 0x000001AE; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Output enable/disable
    void setOutputEnable(uint8_t channel, bool enable);
    bool isOutputEnabled(uint8_t channel) const;
    void setAllOutputs(bool enable);
    
    // Voltage control (mV)
    void setTargetVoltage(uint8_t channel, uint32_t voltage);
    uint32_t getTargetVoltage(uint8_t channel) const;
    uint32_t getActualVoltage(uint8_t channel) const;
    
    // Current limit (mA)
    void setCurrentLimit(uint8_t channel, uint32_t current);
    uint32_t getCurrentLimit(uint8_t channel) const;
    uint32_t getActualCurrent(uint8_t channel) const;
    
    // Power (mW)
    uint32_t getActualPower(uint8_t channel) const;
    uint32_t getTotalPower() const;
    
    // State
    PowerSupplyState getState() const { return state_; }
    PowerSupplyState getChannelState(uint8_t channel) const;
    
    // Protection
    uint8_t getProtectionStatus(uint8_t channel) const;
    bool isProtectionTripped(uint8_t channel, ProtectionType type) const;
    void clearProtection(uint8_t channel);
    void clearAllProtection();
    
    // Temperature (°C)
    int16_t getTemperature() const { return temperature_; }
    void setTemperature(int16_t temp);  // For simulation
    
    // AC monitoring
    uint16_t getACVoltage() const { return acVoltage_; }
    bool isACOk() const { return acOk_; }
    
    // Simulation: Set actual values
    void setActualVoltage(uint8_t channel, uint32_t voltage);
    void setActualCurrent(uint8_t channel, uint32_t current);
    
    // Callbacks
    using ChannelCallback = std::function<void(uint8_t channel, uint32_t& voltage, uint32_t& current)>;
    void setChannelCallback(ChannelCallback callback) { channelCallback_ = callback; }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA430SlaveConfig psConfig_;
    
    // Channel data
    struct ChannelData {
        bool enabled = false;
        uint32_t targetVoltage = 0;      // mV
        uint32_t actualVoltage = 0;      // mV
        uint32_t currentLimit = 0;       // mA
        uint32_t actualCurrent = 0;      // mA
        uint8_t protectionStatus = 0;
        PowerSupplyState state = PowerSupplyState::Off;
        uint16_t softStartTimer = 0;
    };
    std::array<ChannelData, MAX_CHANNELS> channelData_;
    
    // Overall state
    PowerSupplyState state_ = PowerSupplyState::Off;
    int16_t temperature_ = 25;           // °C
    uint16_t acVoltage_ = 230;           // V RMS
    bool acOk_ = true;
    
    ChannelCallback channelCallback_;
    
    void updateChannelState(uint8_t channel, uint64_t deltaNs);
    void checkProtection(uint8_t channel);
    void updateOverallState();
    
    // Additional private helpers for control/status and simulation
    void processControlWord(uint16_t controlWord);
    uint16_t computeControlWord() const;
    uint16_t computeStatusWord() const;
    void registerChannelObjects();
    void checkGlobalProtection();
    void simulateTemperature(uint64_t deltaNs);
};

std::unique_ptr<CiA430Slave> createCiA430Slave(const CiA430SlaveConfig& config);

// Factory functions
std::unique_ptr<CiA430Slave> createSingleChannelPSU(uint32_t voltage, uint32_t current);
std::unique_ptr<CiA430Slave> createDualChannelPSU(uint32_t voltage1, uint32_t current1,
                                                   uint32_t voltage2, uint32_t current2);

}  // namespace slave
}  // namespace EtherCAT
