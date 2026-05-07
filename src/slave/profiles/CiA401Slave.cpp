/**
 * @file CiA401Slave.cpp
 * @brief CiA 401 Digital and Analog I/O Slave Implementation
 */

#include "slave/profiles/CiA401Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Object Dictionary Indices
// ============================================================================

namespace {
    // Digital inputs
    constexpr uint16_t OD_DI_8BIT = 0x6000;
    constexpr uint16_t OD_DI_16BIT = 0x6010;
    constexpr uint16_t OD_DI_32BIT = 0x6020;
    
    // Digital outputs  
    constexpr uint16_t OD_DO_8BIT = 0x6200;
    constexpr uint16_t OD_DO_16BIT = 0x6210;
    constexpr uint16_t OD_DO_32BIT = 0x6220;
    
    // Analog inputs
    constexpr uint16_t OD_AI_16BIT = 0x6400;
    constexpr uint16_t OD_AI_32BIT = 0x6410;
    
    // Analog outputs
    constexpr uint16_t OD_AO_16BIT = 0x6510;
    constexpr uint16_t OD_AO_32BIT = 0x6520;
}

// ============================================================================
// CiA401Slave Implementation
// ============================================================================

CiA401Slave::CiA401Slave(const CiA401SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA401, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , ioConfig_(config)
{
    // Initialize I/O arrays to zero
    digitalInputs_.fill(0);
    digitalOutputs_.fill(0);
    digitalInputPolarity_.fill(0);
    digitalOutputPolarity_.fill(0);
    digitalOutputErrorMode_.fill(0);
    digitalOutputErrorValue_.fill(0);
    digitalInterruptMask_.fill(0);
    previousDigitalInputs_.fill(0);
    analogInputs_.fill(0);
    analogOutputs_.fill(0);
    analogInputOffset_.fill(0);
    analogInputGain_.fill(1.0f);
    analogOutputErrorValue_.fill(0);
}

CiA401Slave::~CiA401Slave() = default;

void CiA401Slave::initObjectDictionary() {
    ProfileSlave::registerCiA301Objects();
    
    auto& od = getObjectDictionary();
    
    // Digital inputs (grouped by 8 bits)
    for (uint8_t i = 0; i < ioConfig_.digitalInputs8; i++) {
        ODEntryInfo info;
        info.index = OD_DI_8BIT;
        info.subindex = i + 1;
        info.dataType = ObjectDictionaryDataType::Unsigned8;
        info.bitLength = 8;
        info.accessType = 0x01;  // Read-only
        info.name = "Digital Input 8-bit";
        info.defaultValue = 0;
        
        od.registerObject(info,
            [this, i](uint8_t* data, size_t& len) {
                return readDigitalInput(OD_DI_8BIT, i + 1, data, len);
            },
            nullptr);
    }
    
    // Digital outputs (grouped by 8 bits)
    for (uint8_t i = 0; i < ioConfig_.digitalOutputs8; i++) {
        ODEntryInfo info;
        info.index = OD_DO_8BIT;
        info.subindex = i + 1;
        info.dataType = ObjectDictionaryDataType::Unsigned8;
        info.bitLength = 8;
        info.accessType = 0x03;  // Read-write
        info.name = "Digital Output 8-bit";
        info.defaultValue = 0;
        
        od.registerObject(info,
            nullptr,
            [this, i](const uint8_t* data, size_t len) {
                return writeDigitalOutput(OD_DO_8BIT, i + 1, data, len);
            });
    }
    
    // Analog inputs
    for (uint8_t i = 0; i < ioConfig_.analogInputs; i++) {
        ODEntryInfo info;
        info.index = OD_AI_16BIT;
        info.subindex = i + 1;
        info.dataType = ObjectDictionaryDataType::Integer16;
        info.bitLength = 16;
        info.accessType = 0x01;  // Read-only
        info.name = "Analog Input 16-bit";
        info.defaultValue = 0;
        
        od.registerObject(info,
            [this, i](uint8_t* data, size_t& len) {
                return readAnalogInput(OD_AI_16BIT, i + 1, data, len);
            },
            nullptr);
    }
    
    // Analog outputs
    for (uint8_t i = 0; i < ioConfig_.analogOutputs; i++) {
        ODEntryInfo info;
        info.index = OD_AO_16BIT;
        info.subindex = i + 1;
        info.dataType = ObjectDictionaryDataType::Integer16;
        info.bitLength = 16;
        info.accessType = 0x03;  // Read-write
        info.name = "Analog Output 16-bit";
        info.defaultValue = 0;
        
        od.registerObject(info,
            nullptr,
            [this, i](const uint8_t* data, size_t len) {
                return writeAnalogOutput(OD_AO_16BIT, i + 1, data, len);
            });
    }
}

void CiA401Slave::initPDOMappings() {
    std::vector<uint32_t> txPdoEntries;
    std::vector<uint32_t> rxPdoEntries;
    
    size_t offset = 0;
    
    // TxPDO: Digital inputs
    pdoLayout_.digitalInputOffset = offset;
    pdoLayout_.digitalInputSize = ioConfig_.digitalInputs8;
    for (uint8_t i = 0; i < ioConfig_.digitalInputs8; i++) {
        txPdoEntries.push_back(PDOMapEntry(OD_DI_8BIT, i + 1, 8));
    }
    offset += ioConfig_.digitalInputs8;
    
    // TxPDO: Analog inputs
    pdoLayout_.analogInputOffset = offset;
    pdoLayout_.analogInputSize = ioConfig_.analogInputs * 2;
    for (uint8_t i = 0; i < ioConfig_.analogInputs; i++) {
        txPdoEntries.push_back(PDOMapEntry(OD_AI_16BIT, i + 1, 16));
    }
    offset += ioConfig_.analogInputs * 2;
    
    // Register TxPDO mapping (0x1A00)
    if (!txPdoEntries.empty()) {
        registerPDOMapping(0x1A00, txPdoEntries);
    }
    
    offset = 0;
    
    // RxPDO: Digital outputs
    pdoLayout_.digitalOutputOffset = offset;
    pdoLayout_.digitalOutputSize = ioConfig_.digitalOutputs8;
    for (uint8_t i = 0; i < ioConfig_.digitalOutputs8; i++) {
        rxPdoEntries.push_back(PDOMapEntry(OD_DO_8BIT, i + 1, 8));
    }
    offset += ioConfig_.digitalOutputs8;
    
    // RxPDO: Analog outputs
    pdoLayout_.analogOutputOffset = offset;
    pdoLayout_.analogOutputSize = ioConfig_.analogOutputs * 2;
    for (uint8_t i = 0; i < ioConfig_.analogOutputs; i++) {
        rxPdoEntries.push_back(PDOMapEntry(OD_AO_16BIT, i + 1, 16));
    }
    
    // Register RxPDO mapping (0x1600)
    if (!rxPdoEntries.empty()) {
        registerPDOMapping(0x1600, rxPdoEntries);
    }
}

void CiA401Slave::onStateChange(SlaveState oldState, SlaveState newState) {
    ProfileSlave::onStateChange(oldState, newState);
    
    if (newState == SlaveState::SAFE_OP || newState == SlaveState::OP) {
        clearCommunicationError();
    }
}

void CiA401Slave::updateTxPDO() {
    auto* txData = getCore().getTxPDOData();
    if (!txData) return;
    
    // Copy digital inputs
    if (pdoLayout_.digitalInputSize > 0) {
        std::memcpy(txData + pdoLayout_.digitalInputOffset, 
                    digitalInputs_.data(), pdoLayout_.digitalInputSize);
    }
    
    // Copy analog inputs
    if (pdoLayout_.analogInputSize > 0) {
        std::memcpy(txData + pdoLayout_.analogInputOffset, 
                    analogInputs_.data(), pdoLayout_.analogInputSize);
    }
}

void CiA401Slave::processRxPDO() {
    const auto* rxData = getCore().getRxPDOData();
    if (!rxData) return;
    
    // Copy digital outputs
    if (pdoLayout_.digitalOutputSize > 0) {
        std::memcpy(digitalOutputs_.data(), 
                    rxData + pdoLayout_.digitalOutputOffset, 
                    pdoLayout_.digitalOutputSize);
        
        if (digitalOutputCallback_) {
            digitalOutputCallback_(digitalOutputs_.data(), pdoLayout_.digitalOutputSize);
        }
    }
    
    // Copy analog outputs
    if (pdoLayout_.analogOutputSize > 0) {
        std::memcpy(analogOutputs_.data(),
                    rxData + pdoLayout_.analogOutputOffset,
                    pdoLayout_.analogOutputSize);
        
        if (analogOutputCallback_) {
            for (size_t i = 0; i < ioConfig_.analogOutputs; i++) {
                analogOutputCallback_(i, analogOutputs_[i]);
            }
        }
    }
}

void CiA401Slave::simulate(uint64_t deltaNs) {
    (void)deltaNs;
    checkInterrupts();
}

// ========================================================================
// Digital Input Access
// ========================================================================

void CiA401Slave::setDigitalInput8(size_t group, uint8_t value) {
    if (group < digitalInputs_.size()) {
        digitalInputs_[group] = value;
    }
}

uint8_t CiA401Slave::getDigitalInput8(size_t group) const {
    return (group < digitalInputs_.size()) ? digitalInputs_[group] : 0;
}

void CiA401Slave::setDigitalInput16(size_t group, uint16_t value) {
    size_t byteIdx = group * 2;
    if (byteIdx + 1 < digitalInputs_.size()) {
        digitalInputs_[byteIdx] = value & 0xFF;
        digitalInputs_[byteIdx + 1] = (value >> 8) & 0xFF;
    }
}

uint16_t CiA401Slave::getDigitalInput16(size_t group) const {
    size_t byteIdx = group * 2;
    if (byteIdx + 1 < digitalInputs_.size()) {
        return digitalInputs_[byteIdx] | (digitalInputs_[byteIdx + 1] << 8);
    }
    return 0;
}

void CiA401Slave::setDigitalInputBit(size_t bit, bool value) {
    size_t byteIdx = bit / 8;
    uint8_t bitMask = 1 << (bit % 8);
    if (byteIdx < digitalInputs_.size()) {
        if (value) {
            digitalInputs_[byteIdx] |= bitMask;
        } else {
            digitalInputs_[byteIdx] &= ~bitMask;
        }
    }
}

bool CiA401Slave::getDigitalInputBit(size_t bit) const {
    size_t byteIdx = bit / 8;
    uint8_t bitMask = 1 << (bit % 8);
    return (byteIdx < digitalInputs_.size()) && (digitalInputs_[byteIdx] & bitMask);
}

// ========================================================================
// Digital Output Access
// ========================================================================

uint8_t CiA401Slave::getDigitalOutput8(size_t group) const {
    return (group < digitalOutputs_.size()) ? digitalOutputs_[group] : 0;
}

uint16_t CiA401Slave::getDigitalOutput16(size_t group) const {
    size_t byteIdx = group * 2;
    if (byteIdx + 1 < digitalOutputs_.size()) {
        return digitalOutputs_[byteIdx] | (digitalOutputs_[byteIdx + 1] << 8);
    }
    return 0;
}

bool CiA401Slave::getDigitalOutputBit(size_t bit) const {
    size_t byteIdx = bit / 8;
    uint8_t bitMask = 1 << (bit % 8);
    return (byteIdx < digitalOutputs_.size()) && (digitalOutputs_[byteIdx] & bitMask);
}

void CiA401Slave::setDigitalOutputCallback(DigitalOutputCallback callback) {
    digitalOutputCallback_ = callback;
}

// ========================================================================
// Analog I/O Access
// ========================================================================

void CiA401Slave::setAnalogInput(size_t channel, int16_t value) {
    if (channel < analogInputs_.size()) {
        analogInputs_[channel] = value;
    }
}

int16_t CiA401Slave::getAnalogInput(size_t channel) const {
    return (channel < analogInputs_.size()) ? analogInputs_[channel] : 0;
}

void CiA401Slave::setAnalogInputScaling(size_t channel, int16_t offset, float gain) {
    if (channel < analogInputs_.size()) {
        analogInputOffset_[channel] = offset;
        analogInputGain_[channel] = gain;
    }
}

int16_t CiA401Slave::getAnalogOutput(size_t channel) const {
    return (channel < analogOutputs_.size()) ? analogOutputs_[channel] : 0;
}

void CiA401Slave::setAnalogOutputCallback(AnalogOutputCallback callback) {
    analogOutputCallback_ = callback;
}

// ========================================================================
// Interrupt Handling
// ========================================================================

void CiA401Slave::configureDigitalInterrupt(size_t group, uint8_t mask, uint8_t mode) {
    if (group < digitalInterruptMask_.size()) {
        digitalInterruptMask_[group] = mask;
        (void)mode;  // Mode stored elsewhere if needed
    }
}

void CiA401Slave::setInterruptCallback(InterruptCallback callback) {
    interruptCallback_ = callback;
}

void CiA401Slave::checkInterrupts() {
    if (!interruptCallback_) return;
    
    for (size_t i = 0; i < ioConfig_.digitalInputs8; i++) {
        uint8_t changed = (digitalInputs_[i] ^ previousDigitalInputs_[i]) & digitalInterruptMask_[i];
        if (changed) {
            interruptCallback_(i, changed);
        }
        previousDigitalInputs_[i] = digitalInputs_[i];
    }
}

// ========================================================================
// Error Handling
// ========================================================================

void CiA401Slave::setDigitalOutputErrorValue(size_t group, uint8_t value) {
    if (group < digitalOutputErrorValue_.size()) {
        digitalOutputErrorValue_[group] = value;
    }
}

void CiA401Slave::setDigitalOutputErrorMode(size_t group, uint8_t mode) {
    if (group < digitalOutputErrorMode_.size()) {
        digitalOutputErrorMode_[group] = mode;
    }
}

void CiA401Slave::triggerCommunicationError() {
    communicationError_ = true;
    applyErrorValues();
}

void CiA401Slave::clearCommunicationError() {
    communicationError_ = false;
}

void CiA401Slave::applyErrorValues() {
    if (!communicationError_) return;
    
    for (size_t i = 0; i < ioConfig_.digitalOutputs8; i++) {
        if (digitalOutputErrorMode_[i] == 0) {
            digitalOutputs_[i] = digitalOutputErrorValue_[i];
        }
        // Mode 1: keep last value (do nothing)
    }
    
    for (size_t i = 0; i < ioConfig_.analogOutputs; i++) {
        analogOutputs_[i] = analogOutputErrorValue_[i];
    }
}

// ========================================================================
// SDO Handlers
// ========================================================================

SDOAbortCode CiA401Slave::readDigitalInput(uint16_t index, uint8_t subindex,
                                            uint8_t* data, size_t& len) {
    (void)index;
    if (subindex == 0 || subindex > ioConfig_.digitalInputs8) {
        return SDOAbortCode::SubindexNotFound;
    }
    if (len < 1) {
        return SDOAbortCode::DataTypeMismatch;
    }
    data[0] = digitalInputs_[subindex - 1];
    len = 1;
    return SDOAbortCode::Success;
}

SDOAbortCode CiA401Slave::writeDigitalOutput(uint16_t index, uint8_t subindex,
                                              const uint8_t* data, size_t len) {
    (void)index;
    if (subindex == 0 || subindex > ioConfig_.digitalOutputs8) {
        return SDOAbortCode::SubindexNotFound;
    }
    if (len < 1) {
        return SDOAbortCode::DataTypeMismatch;
    }
    digitalOutputs_[subindex - 1] = data[0];
    return SDOAbortCode::Success;
}

SDOAbortCode CiA401Slave::readAnalogInput(uint16_t index, uint8_t subindex,
                                           uint8_t* data, size_t& len) {
    (void)index;
    if (subindex == 0 || subindex > ioConfig_.analogInputs) {
        return SDOAbortCode::SubindexNotFound;
    }
    if (len < 2) {
        return SDOAbortCode::DataTypeMismatch;
    }
    int16_t value = analogInputs_[subindex - 1];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    len = 2;
    return SDOAbortCode::Success;
}

SDOAbortCode CiA401Slave::writeAnalogOutput(uint16_t index, uint8_t subindex,
                                             const uint8_t* data, size_t len) {
    (void)index;
    if (subindex == 0 || subindex > ioConfig_.analogOutputs) {
        return SDOAbortCode::SubindexNotFound;
    }
    if (len < 2) {
        return SDOAbortCode::DataTypeMismatch;
    }
    analogOutputs_[subindex - 1] = static_cast<int16_t>(data[0] | (data[1] << 8));
    return SDOAbortCode::Success;
}

// ========================================================================
// Factory Functions
// ========================================================================

std::unique_ptr<CiA401Slave> createCiA401Slave(const CiA401SlaveConfig& config) {
    return std::make_unique<CiA401Slave>(config);
}

std::unique_ptr<CiA401Slave> createDigitalIOSlave(size_t inputs, size_t outputs) {
    CiA401SlaveConfig config;
    config.digitalInputs8 = static_cast<uint8_t>((inputs + 7) / 8);
    config.digitalOutputs8 = static_cast<uint8_t>((outputs + 7) / 8);
    config.analogInputs = 0;
    config.analogOutputs = 0;
    return createCiA401Slave(config);
}

std::unique_ptr<CiA401Slave> createAnalogIOSlave(size_t inputs, size_t outputs) {
    CiA401SlaveConfig config;
    config.digitalInputs8 = 0;
    config.digitalOutputs8 = 0;
    config.analogInputs = static_cast<uint8_t>(std::min(inputs, size_t(8)));
    config.analogOutputs = static_cast<uint8_t>(std::min(outputs, size_t(8)));
    return createCiA401Slave(config);
}

}  // namespace slave
}  // namespace EtherCAT
