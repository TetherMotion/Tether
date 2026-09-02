/**
 * @file EtherCATBackend.cpp
 * @brief EtherCAT drive backend implementation
 */

#include "profiles/cia402/EtherCATBackend.hpp"
#include "tether/platform/EspCompat.hpp"
#include <cstring>

static const char* TAG = "EtherCATBackend";

namespace CiA402 {

// Template implementations
template<typename T>
void EtherCATBackend::writeRxPDO(size_t offset, T value) {
    if (offset + sizeof(T) <= sizeof(m_rxPdo)) {
        std::memcpy(&m_rxPdo[offset], &value, sizeof(T));
    }
}

template<typename T>
T EtherCATBackend::readTxPDO(size_t offset) const {
    T value = 0;
    if (offset + sizeof(T) <= sizeof(m_txPdo)) {
        std::memcpy(&value, &m_txPdo[offset], sizeof(T));
    }
    return value;
}

// Explicit instantiations
template void EtherCATBackend::writeRxPDO<uint16_t>(size_t, uint16_t);
template void EtherCATBackend::writeRxPDO<int16_t>(size_t, int16_t);
template void EtherCATBackend::writeRxPDO<int32_t>(size_t, int32_t);
template void EtherCATBackend::writeRxPDO<uint32_t>(size_t, uint32_t);
template void EtherCATBackend::writeRxPDO<int8_t>(size_t, int8_t);
template void EtherCATBackend::writeRxPDO<uint8_t>(size_t, uint8_t);

template uint16_t EtherCATBackend::readTxPDO<uint16_t>(size_t) const;
template int16_t EtherCATBackend::readTxPDO<int16_t>(size_t) const;
template int32_t EtherCATBackend::readTxPDO<int32_t>(size_t) const;
template uint32_t EtherCATBackend::readTxPDO<uint32_t>(size_t) const;
template int8_t EtherCATBackend::readTxPDO<int8_t>(size_t) const;
template uint8_t EtherCATBackend::readTxPDO<uint8_t>(size_t) const;

EtherCATBackend::EtherCATBackend(EtherCATMasterPtr master, 
                                 const EtherCATSlaveConfig& config)
    : m_master(master)
    , m_config(config)
{
    std::memset(m_rxPdo, 0, sizeof(m_rxPdo));
    std::memset(m_txPdo, 0, sizeof(m_txPdo));
}

EtherCATBackend::~EtherCATBackend() {
    deinitialize();
}

bool EtherCATBackend::initialize() {
    if (m_initialized) {
        return true;
    }
    
    if (!m_master) {
        TETHER_LOGE(TAG, "No master interface");
        return false;
    }
    
    TETHER_LOGI(TAG, "Initializing EtherCAT backend for slave {}", 
             (unsigned long)m_config.slaveId);
    
    m_initialized = true;
    m_connected = m_master->isSlaveOperational(m_config.slaveId);
    
    return true;
}

void EtherCATBackend::deinitialize() {
    m_initialized = false;
    m_connected = false;
}

bool EtherCATBackend::isConnected() const {
    return m_connected && m_initialized;
}

std::string EtherCATBackend::getName() const {
    return m_config.name.empty() ? 
           "EtherCAT Slave " + std::to_string(m_config.slaveId) : 
           m_config.name;
}

bool EtherCATBackend::updateInputs() {
    if (!m_master || !m_initialized) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Read process data from slave
    if (!m_master->readProcessData(m_config.slaveId, m_txPdo, 
                                   m_config.pdoMapping.txPdoSize)) {
        m_connected = false;
        return false;
    }
    
    m_connected = true;
    m_lastUpdateTimestamp = esp_timer_get_time();
    
    // Update cached state
    const PDOMapping& pdo = m_config.pdoMapping;
    m_state.statusWord = readTxPDO<uint16_t>(pdo.statusWordOffset);
    m_state.actualPosition = readTxPDO<int32_t>(pdo.actualPositionOffset);
    m_state.actualVelocity = readTxPDO<int32_t>(pdo.actualVelocityOffset);
    m_state.actualTorque = readTxPDO<int16_t>(pdo.actualTorqueOffset);
    m_state.modesOfOperationDisplay = readTxPDO<int8_t>(pdo.modesDisplayOffset);
    m_state.errorCode = readTxPDO<uint16_t>(pdo.errorCodeOffset);
    
    // Check for state changes
    checkStateChange();
    
    // Call sync callback
    if (m_syncCallback) {
        m_syncCallback(m_lastUpdateTimestamp);
    }
    
    return true;
}

bool EtherCATBackend::updateOutputs() {
    if (!m_master || !m_initialized) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Write process data to slave
    return m_master->writeProcessData(m_config.slaveId, m_rxPdo, 
                                      m_config.pdoMapping.rxPdoSize);
}

DriveState EtherCATBackend::getState() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state;
}

uint16_t EtherCATBackend::readStatusWord() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.statusWord;
}

void EtherCATBackend::writeControlWord(uint16_t controlWord) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state.controlWord = controlWord;
    writeRxPDO<uint16_t>(m_config.pdoMapping.controlWordOffset, controlWord);
}

uint16_t EtherCATBackend::readControlWord() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.controlWord;
}

bool EtherCATBackend::setOperatingMode(OperatingMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_operatingMode = mode;
    int8_t modeValue = static_cast<int8_t>(mode);
    writeRxPDO<int8_t>(m_config.pdoMapping.modesOfOperationOffset, modeValue);
    m_state.modesOfOperation = modeValue;
    return true;
}

OperatingMode EtherCATBackend::getOperatingMode() const {
    return m_operatingMode;
}

OperatingMode EtherCATBackend::getDisplayedMode() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<OperatingMode>(m_state.modesOfOperationDisplay);
}

void EtherCATBackend::setTargetPosition(int32_t position) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state.targetPosition = position;
    writeRxPDO<int32_t>(m_config.pdoMapping.targetPositionOffset, position);
}

int32_t EtherCATBackend::getActualPosition() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.actualPosition;
}

int32_t EtherCATBackend::getPositionDemand() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.positionDemand;
}

int32_t EtherCATBackend::getFollowingError() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.followingError;
}

void EtherCATBackend::setPositionOffset(int32_t offset) {
    std::lock_guard<std::mutex> lock(m_mutex);
    writeRxPDO<int32_t>(m_config.pdoMapping.positionOffsetOffset, offset);
}

void EtherCATBackend::setTargetVelocity(int32_t velocity) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state.targetVelocity = velocity;
    writeRxPDO<int32_t>(m_config.pdoMapping.targetVelocityOffset, velocity);
}

int32_t EtherCATBackend::getActualVelocity() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.actualVelocity;
}

int32_t EtherCATBackend::getVelocityDemand() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.velocityDemand;
}

void EtherCATBackend::setVelocityOffset(int32_t offset) {
    std::lock_guard<std::mutex> lock(m_mutex);
    writeRxPDO<int32_t>(m_config.pdoMapping.velocityOffsetOffset, offset);
}

void EtherCATBackend::setTargetTorque(int16_t torque) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state.targetTorque = torque;
    writeRxPDO<int16_t>(m_config.pdoMapping.targetTorqueOffset, torque);
}

int16_t EtherCATBackend::getActualTorque() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.actualTorque;
}

void EtherCATBackend::setTorqueOffset(int16_t offset) {
    std::lock_guard<std::mutex> lock(m_mutex);
    writeRxPDO<int16_t>(m_config.pdoMapping.torqueOffsetOffset, offset);
}

void EtherCATBackend::setProfileVelocity(uint32_t velocity) {
    if (!m_master) return;
    
    m_master->writeSDO(m_config.slaveId, 
                      static_cast<uint16_t>(ObjIndex::ProfileVelocity),
                      0, &velocity, sizeof(velocity));
}

void EtherCATBackend::setProfileAcceleration(uint32_t acceleration) {
    if (!m_master) return;
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::ProfileAcceleration),
                      0, &acceleration, sizeof(acceleration));
}

void EtherCATBackend::setProfileDeceleration(uint32_t deceleration) {
    if (!m_master) return;
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::ProfileDeceleration),
                      0, &deceleration, sizeof(deceleration));
}

void EtherCATBackend::setMotionProfileType(int16_t type) {
    if (!m_master) return;
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::MotionProfileType),
                      0, &type, sizeof(type));
}

bool EtherCATBackend::configureHoming(const HomingParams& params) {
    if (!m_master) return false;
    
    m_homingParams = params;
    
    // Write homing parameters via SDO
    m_master->writeSDO(m_config.slaveId, 
                      static_cast<uint16_t>(ObjIndex::HomingMethod),
                      0, &params.method, sizeof(params.method));
    
    // HomingSpeed object uses subindex 1 for switch speed, 2 for zero speed
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::HomingSpeed),
                      1, &params.speedSwitch, sizeof(params.speedSwitch));
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::HomingSpeed),
                      2, &params.speedZero, sizeof(params.speedZero));
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::HomingAcceleration),
                      0, &params.acceleration, sizeof(params.acceleration));
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::HomeOffset),
                      0, &params.offset, sizeof(params.offset));
    
    return true;
}

HomingParams EtherCATBackend::getHomingParams() const {
    return m_homingParams;
}

bool EtherCATBackend::configureInterpolation(const InterpolationParams& params) {
    if (!m_master) return false;
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::InterpolationMode),
                      0, &params.subModeSelect, sizeof(params.subModeSelect));
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::InterpolationTimePeriod),
                      1, &params.timePeriod, sizeof(params.timePeriod));
    
    m_master->writeSDO(m_config.slaveId,
                      static_cast<uint16_t>(ObjIndex::InterpolationTimePeriod),
                      2, &params.timeIndex, sizeof(params.timeIndex));
    
    return true;
}

bool EtherCATBackend::addInterpolationPoint(int32_t position) {
    // Write to interpolation data buffer via SDO
    if (!m_master) return false;
    
    return m_master->writeSDO(m_config.slaveId,
                             static_cast<uint16_t>(ObjIndex::InterpolationData),
                             1, &position, sizeof(position)).success;
}

void EtherCATBackend::clearInterpolationBuffer() {
    // Implementation depends on drive
}

SDOResult EtherCATBackend::readSDO(uint16_t index, uint8_t subindex, 
                                   void* data, size_t size) {
    if (!m_master) {
        return {false, 0, {}, "No master"};
    }
    
    return m_master->readSDO(m_config.slaveId, index, subindex, data, size);
}

SDOResult EtherCATBackend::writeSDO(uint16_t index, uint8_t subindex,
                                    const void* data, size_t size) {
    if (!m_master) {
        return {false, 0, {}, "No master"};
    }
    
    return m_master->writeSDO(m_config.slaveId, index, subindex, data, size);
}

bool EtherCATBackend::configure(const DriveConfig& config) {
    if (!m_master) return false;
    
    m_driveConfig = config;
    
    // Write configuration via SDO
    // This would write to various object dictionary entries
    // Implementation depends on specific drive
    
    return true;
}

DriveConfig EtherCATBackend::getConfiguration() const {
    return m_driveConfig;
}

bool EtherCATBackend::storeParameters() {
    if (!m_master) return false;
    
    // Write store command (0x65766173 = "save")
    uint32_t storeCmd = 0x65766173;
    return m_master->writeSDO(m_config.slaveId, 0x1010, 1, 
                             &storeCmd, sizeof(storeCmd)).success;
}

bool EtherCATBackend::restoreParameters() {
    if (!m_master) return false;
    
    // Write restore command (0x64616F6C = "load")
    uint32_t restoreCmd = 0x64616F6C;
    return m_master->writeSDO(m_config.slaveId, 0x1011, 1,
                             &restoreCmd, sizeof(restoreCmd)).success;
}

uint16_t EtherCATBackend::getErrorCode() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state.errorCode;
}

uint8_t EtherCATBackend::getErrorRegister() const {
    // Read error register via SDO if needed
    return 0;
}

std::vector<uint16_t> EtherCATBackend::getErrorHistory() const {
    std::vector<uint16_t> history;
    
    if (!m_master) return history;
    
    // Read error history from 0x1003
    uint8_t numErrors = 0;
    m_master->readSDO(m_config.slaveId, 0x1003, 0, &numErrors, 1);
    
    for (uint8_t i = 1; i <= numErrors && i <= 10; i++) {
        uint32_t error = 0;
        auto result = m_master->readSDO(m_config.slaveId, 0x1003, i, &error, 4);
        if (result.success) {
            history.push_back(static_cast<uint16_t>(error));
        }
    }
    
    return history;
}

bool EtherCATBackend::clearErrorHistory() {
    if (!m_master) return false;
    
    uint8_t zero = 0;
    return m_master->writeSDO(m_config.slaveId, 0x1003, 0, &zero, 1).success;
}

void EtherCATBackend::setStateChangeCallback(StateChangeCallback callback) {
    m_stateCallback = callback;
}

void EtherCATBackend::setErrorCallback(ErrorCallback callback) {
    m_errorCallback = callback;
}

void EtherCATBackend::setWarningCallback(WarningCallback callback) {
    m_warningCallback = callback;
}

void EtherCATBackend::setSyncCallback(SyncCallback callback) {
    m_syncCallback = callback;
}

uint32_t EtherCATBackend::getCycleTimeUs() const {
    return m_cycleTimeUs;
}

bool EtherCATBackend::setCycleTimeUs(uint32_t cycleTimeUs) {
    m_cycleTimeUs = cycleTimeUs;
    return true;
}

uint64_t EtherCATBackend::getLastUpdateTimestamp() const {
    return m_lastUpdateTimestamp;
}

void EtherCATBackend::setPDOMapping(const PDOMapping& mapping) {
    m_config.pdoMapping = mapping;
}

State EtherCATBackend::decodeState() const {
    uint16_t sw = m_state.statusWord;
    
    // Decode according to CiA 402
    if ((sw & 0x004F) == 0x0008) return State::Fault;
    if ((sw & 0x004F) == 0x000F) return State::FaultReactionActive;
    if ((sw & 0x004F) == 0x0040) return State::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0007) return State::QuickStopActive;
    if ((sw & 0x006F) == 0x0027) return State::OperationEnabled;
    if ((sw & 0x006F) == 0x0023) return State::SwitchedOn;
    if ((sw & 0x006F) == 0x0021) return State::ReadyToSwitchOn;
    
    return State::NotReadyToSwitchOn;
}

void EtherCATBackend::checkStateChange() {
    State currentState = decodeState();
    
    if (currentState != m_lastState) {
        TETHER_LOGD(TAG, "Slave {} state: {} -> {}",
                (unsigned long)m_config.slaveId,
                static_cast<int>(m_lastState),
                static_cast<int>(currentState));
        
        if (m_stateCallback) {
            m_stateCallback(m_lastState, currentState);
        }
        
        // Check for fault
        if (currentState == State::Fault && m_errorCallback) {
            m_errorCallback(m_state.errorCode, "Drive fault");
        }
        
        // Check for warning
        if ((m_state.statusWord & static_cast<uint16_t>(StatusWordBit::Warning)) &&
            m_warningCallback) {
            m_warningCallback(0, "Drive warning");
        }
        
        m_lastState = currentState;
    }
    
    m_cia402State = currentState;
}

// ============================================================================
// Factory
// ============================================================================

DriveBackendUPtr EtherCATBackendFactory::createBackend(uint32_t slaveId) {
    EtherCATSlaveConfig config;
    config.slaveId = slaveId;
    config.name = "Slave " + std::to_string(slaveId);
    
    return createBackend(config);
}

DriveBackendUPtr EtherCATBackendFactory::createBackend(const EtherCATSlaveConfig& config) {
    if (!m_master) {
        return nullptr;
    }
    
    return std::make_unique<EtherCATBackend>(m_master, config);
}

} // namespace CiA402
