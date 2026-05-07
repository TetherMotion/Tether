/**
 * @file CiA430Slave.cpp
 * @brief CiA 430 Power Supply Slave Implementation
 */

#include "slave/profiles/CiA430Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA430Slave Implementation
// ============================================================================

CiA430Slave::CiA430Slave(const CiA430SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA430, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , psConfig_(config)
{
    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        channelData_[i] = ChannelData{};
        if (i < config.numberOfChannels) {
            channelData_[i].currentLimit = config.channels[i].nominalCurrent;
        }
    }
}

CiA430Slave::~CiA430Slave() = default;

void CiA430Slave::initObjectDictionary() {
    ProfileSlave::registerCiA301Objects();
    registerChannelObjects();
}

void CiA430Slave::initPDOMappings() {
    // Minimal implementation - PDO mappings would go here
}

void CiA430Slave::updateTxPDO() {
    // Minimal implementation - transmit PDO update would go here
}

void CiA430Slave::processRxPDO() {
    // Minimal implementation - receive PDO processing would go here
}

void CiA430Slave::simulate(uint64_t deltaNs) {
    for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
        updateChannelState(ch, deltaNs);
        checkProtection(ch);
    }
    checkGlobalProtection();
    simulateTemperature(deltaNs);
    updateOverallState();
    
    if (channelCallback_) {
        for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
            channelCallback_(ch, channelData_[ch].actualVoltage, channelData_[ch].actualCurrent);
        }
    }
}

void CiA430Slave::setOutputEnable(uint8_t channel, bool enable) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].enabled = enable;
        if (enable) {
            channelData_[channel].state = PowerSupplyState::Running;
        } else {
            channelData_[channel].state = PowerSupplyState::Ready;
            channelData_[channel].actualVoltage = 0;
            channelData_[channel].actualCurrent = 0;
        }
    }
}

bool CiA430Slave::isOutputEnabled(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].enabled;
    }
    return false;
}

void CiA430Slave::setAllOutputs(bool enable) {
    for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
        setOutputEnable(ch, enable);
    }
}

void CiA430Slave::setTargetVoltage(uint8_t channel, uint32_t voltage) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].targetVoltage = voltage;
    }
}

uint32_t CiA430Slave::getTargetVoltage(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].targetVoltage;
    }
    return 0;
}

uint32_t CiA430Slave::getActualVoltage(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].actualVoltage;
    }
    return 0;
}

void CiA430Slave::setCurrentLimit(uint8_t channel, uint32_t current) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].currentLimit = current;
    }
}

uint32_t CiA430Slave::getCurrentLimit(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].currentLimit;
    }
    return 0;
}

uint32_t CiA430Slave::getActualCurrent(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].actualCurrent;
    }
    return 0;
}

uint32_t CiA430Slave::getActualPower(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        // Power in mW = (mV * mA) / 1000
        return (channelData_[channel].actualVoltage * channelData_[channel].actualCurrent) / 1000;
    }
    return 0;
}

uint32_t CiA430Slave::getTotalPower() const {
    uint32_t total = 0;
    for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
        total += getActualPower(ch);
    }
    return total;
}

PowerSupplyState CiA430Slave::getChannelState(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].state;
    }
    return PowerSupplyState::Off;
}

uint8_t CiA430Slave::getProtectionStatus(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return channelData_[channel].protectionStatus;
    }
    return 0;
}

bool CiA430Slave::isProtectionTripped(uint8_t channel, ProtectionType type) const {
    if (channel < MAX_CHANNELS) {
        return (channelData_[channel].protectionStatus & static_cast<uint8_t>(type)) != 0;
    }
    return false;
}

void CiA430Slave::clearProtection(uint8_t channel) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].protectionStatus = 0;
        if (channelData_[channel].state == PowerSupplyState::Fault) {
            channelData_[channel].state = PowerSupplyState::Ready;
        }
    }
}

void CiA430Slave::clearAllProtection() {
    for (uint8_t ch = 0; ch < MAX_CHANNELS; ++ch) {
        clearProtection(ch);
    }
    if (state_ == PowerSupplyState::Fault) {
        state_ = PowerSupplyState::Ready;
    }
}

void CiA430Slave::setTemperature(int16_t temp) {
    temperature_ = temp;
}

void CiA430Slave::setActualVoltage(uint8_t channel, uint32_t voltage) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].actualVoltage = voltage;
    }
}

void CiA430Slave::setActualCurrent(uint8_t channel, uint32_t current) {
    if (channel < MAX_CHANNELS) {
        channelData_[channel].actualCurrent = current;
    }
}

void CiA430Slave::updateChannelState(uint8_t channel, uint64_t deltaNs) {
    if (channel >= MAX_CHANNELS) return;
    
    auto& ch = channelData_[channel];
    
    if (!ch.enabled) {
        ch.actualVoltage = 0;
        ch.actualCurrent = 0;
        return;
    }
    
    // Simulate voltage ramp
    if (ch.actualVoltage < ch.targetVoltage) {
        uint32_t rampRate = 1000000;  // 1V/ms in mV/s
        uint32_t delta = static_cast<uint32_t>((rampRate * deltaNs) / 1000000000ULL);
        ch.actualVoltage = std::min(ch.actualVoltage + delta, ch.targetVoltage);
    } else if (ch.actualVoltage > ch.targetVoltage) {
        uint32_t rampRate = 1000000;
        uint32_t delta = static_cast<uint32_t>((rampRate * deltaNs) / 1000000000ULL);
        if (delta > ch.actualVoltage) {
            ch.actualVoltage = ch.targetVoltage;
        } else {
            ch.actualVoltage = std::max(ch.actualVoltage - delta, ch.targetVoltage);
        }
    }
}

void CiA430Slave::checkProtection(uint8_t channel) {
    if (channel >= psConfig_.numberOfChannels) return;
    
    auto& ch = channelData_[channel];
    const auto& cfg = psConfig_.channels[channel];
    
    // Over-voltage protection
    if (ch.actualVoltage > cfg.ovpThreshold) {
        ch.protectionStatus |= static_cast<uint8_t>(ProtectionType::OVP);
        ch.state = PowerSupplyState::Fault;
        ch.enabled = false;
    }
    
    // Over-current protection
    if (ch.actualCurrent > cfg.ocpThreshold) {
        ch.protectionStatus |= static_cast<uint8_t>(ProtectionType::OCP);
        ch.state = PowerSupplyState::Fault;
        ch.enabled = false;
    }
    
    // Under-voltage protection
    if (ch.enabled && ch.actualVoltage < cfg.uvpThreshold && ch.targetVoltage >= cfg.uvpThreshold) {
        ch.protectionStatus |= static_cast<uint8_t>(ProtectionType::UVP);
    }
}

void CiA430Slave::updateOverallState() {
    bool anyFault = false;
    bool anyRunning = false;
    
    for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
        if (channelData_[ch].state == PowerSupplyState::Fault) {
            anyFault = true;
        }
        if (channelData_[ch].state == PowerSupplyState::Running) {
            anyRunning = true;
        }
    }
    
    if (anyFault) {
        state_ = PowerSupplyState::Fault;
    } else if (anyRunning) {
        state_ = PowerSupplyState::Running;
    } else {
        state_ = PowerSupplyState::Ready;
    }
}

void CiA430Slave::processControlWord(uint16_t controlWord) {
    (void)controlWord;
    // Minimal implementation
}

uint16_t CiA430Slave::computeControlWord() const {
    return 0;
}

uint16_t CiA430Slave::computeStatusWord() const {
    uint16_t status = static_cast<uint16_t>(state_);
    for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
        if (channelData_[ch].enabled) {
            status |= (1 << (8 + ch));
        }
    }
    return status;
}

void CiA430Slave::registerChannelObjects() {
    // Minimal implementation - channel-specific OD registration would go here
}

void CiA430Slave::checkGlobalProtection() {
    // Over-temperature protection
    if (psConfig_.hasTemperatureSensor && temperature_ > psConfig_.maxTemperature) {
        for (uint8_t ch = 0; ch < psConfig_.numberOfChannels; ++ch) {
            channelData_[ch].protectionStatus |= static_cast<uint8_t>(ProtectionType::OTP);
            channelData_[ch].state = PowerSupplyState::Fault;
            channelData_[ch].enabled = false;
        }
        state_ = PowerSupplyState::Fault;
    }
}

void CiA430Slave::simulateTemperature(uint64_t deltaNs) {
    (void)deltaNs;
    // Minimal implementation - temperature stays constant
}

std::unique_ptr<CiA430Slave> createCiA430Slave(const CiA430SlaveConfig& config) {
    return std::make_unique<CiA430Slave>(config);
}

std::unique_ptr<CiA430Slave> createSingleChannelPSU(uint32_t voltage, uint32_t current) {
    CiA430SlaveConfig config;
    config.numberOfChannels = 1;
    config.channels[0].nominalVoltage = voltage;
    config.channels[0].nominalCurrent = current;
    return std::make_unique<CiA430Slave>(config);
}

std::unique_ptr<CiA430Slave> createDualChannelPSU(uint32_t voltage1, uint32_t current1,
                                                   uint32_t voltage2, uint32_t current2) {
    CiA430SlaveConfig config;
    config.numberOfChannels = 2;
    config.channels[0].nominalVoltage = voltage1;
    config.channels[0].nominalCurrent = current1;
    config.channels[1].nominalVoltage = voltage2;
    config.channels[1].nominalCurrent = current2;
    return std::make_unique<CiA430Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
