/**
 * @file ElectronicGearing.cpp
 * @brief Electronic gearing implementation
 */

#include "profiles/cia402/ElectronicGearing.hpp"
#include "tether/platform/EspCompat.hpp"
#include <algorithm>

static const char* TAG = "Gearing";

namespace CiA402 {

GearingController::GearingController() {
    m_lastUpdateTime = esp_timer_get_time();
}

void GearingController::setMasterBackend(DriveBackendPtr backend) {
    m_masterBackend = backend;
}

size_t GearingController::addSlave(DriveBackendPtr backend, 
                                   int32_t numerator, 
                                   int32_t denominator) {
    GearingSlave slave;
    slave.backend = backend;
    slave.config.numerator = numerator;
    slave.config.denominator = denominator;
    slave.state = GearingState::Disengaged;
    slave.currentRatio = 0.0;
    
    m_slaves.push_back(slave);
    
    TETHER_LOGI(TAG, "Added slave %d with ratio %ld:%ld", 
             (int)(m_slaves.size() - 1),
             (long)numerator, (long)denominator);
    
    return m_slaves.size() - 1;
}

void GearingController::configureSlave(size_t slaveIndex, 
                                       const GearingSlaveConfig& config) {
    if (slaveIndex >= m_slaves.size()) {
        return;
    }
    
    m_slaves[slaveIndex].config = config;
}

void GearingController::removeSlave(size_t slaveIndex) {
    if (slaveIndex >= m_slaves.size()) {
        return;
    }
    
    // Disengage first
    if (m_slaves[slaveIndex].state != GearingState::Disengaged) {
        disengageSlave(slaveIndex, false);
    }
    
    m_slaves.erase(m_slaves.begin() + slaveIndex);
}

void GearingController::clearSlaves() {
    disengage(false);
    m_slaves.clear();
}

void GearingController::setEventCallback(GearingEventCallback callback) {
    m_eventCallback = callback;
}

bool GearingController::engage(bool softStart) {
    if (!m_masterBackend) {
        TETHER_LOGE(TAG, "No master configured");
        return false;
    }
    
    bool success = true;
    for (size_t i = 0; i < m_slaves.size(); i++) {
        if (!engageSlave(i, softStart)) {
            success = false;
        }
    }
    
    return success;
}

bool GearingController::engageSlave(size_t slaveIndex, bool softStart) {
    if (slaveIndex >= m_slaves.size()) {
        return false;
    }
    
    GearingSlave& slave = m_slaves[slaveIndex];
    
    if (slave.state == GearingState::Engaged) {
        return true;  // Already engaged
    }
    
    if (!slave.backend) {
        TETHER_LOGE(TAG, "Slave %d has no backend", (int)slaveIndex);
        return false;
    }
    
    // Record sync position
    slave.syncPosition = slave.backend->getActualPosition();
    slave.lastMasterPos = m_masterBackend->getActualPosition();
    slave.engageStartTime = esp_timer_get_time();
    
    if (softStart && m_softStartEnabled && m_rampTimeMs > 0) {
        slave.state = GearingState::Engaging;
        slave.currentRatio = 0.0;
        TETHER_LOGI(TAG, "Slave %d engaging with soft start", (int)slaveIndex);
    } else {
        slave.state = GearingState::Engaged;
        slave.currentRatio = static_cast<double>(slave.config.numerator) / 
                            slave.config.denominator;
        TETHER_LOGI(TAG, "Slave %d engaged immediately", (int)slaveIndex);
    }
    
    notifyStateChange(slaveIndex, slave.state);
    return true;
}

void GearingController::disengage(bool softStop) {
    for (size_t i = 0; i < m_slaves.size(); i++) {
        disengageSlave(i, softStop);
    }
}

void GearingController::disengageSlave(size_t slaveIndex, bool softStop) {
    if (slaveIndex >= m_slaves.size()) {
        return;
    }
    
    GearingSlave& slave = m_slaves[slaveIndex];
    
    if (slave.state == GearingState::Disengaged) {
        return;
    }
    
    if (softStop && m_rampTimeMs > 0) {
        slave.state = GearingState::Disengaging;
        slave.engageStartTime = esp_timer_get_time();
        TETHER_LOGI(TAG, "Slave %d disengaging with soft stop", (int)slaveIndex);
    } else {
        slave.state = GearingState::Disengaged;
        slave.currentRatio = 0.0;
        TETHER_LOGI(TAG, "Slave %d disengaged immediately", (int)slaveIndex);
    }
    
    notifyStateChange(slaveIndex, slave.state);
}

void GearingController::update() {
    if (!m_masterBackend) {
        return;
    }
    
    // Get master position and velocity
    m_masterPosition = m_masterBackend->getActualPosition();
    
    uint64_t now = esp_timer_get_time();
    double dt = (now - m_lastUpdateTime) / 1e6;  // seconds
    
    if (dt > 0) {
        m_masterVelocity = static_cast<int32_t>(
            (m_masterPosition - m_lastMasterPosition) / dt
        );
    }
    
    m_lastMasterPosition = m_masterPosition;
    m_lastUpdateTime = now;
    
    // Update each slave
    for (size_t i = 0; i < m_slaves.size(); i++) {
        updateSlave(m_slaves[i], m_masterPosition, m_masterVelocity);
    }
}

void GearingController::updateSlave(GearingSlave& slave, 
                                    int32_t masterPos, 
                                    int32_t masterVel) {
    if (slave.state == GearingState::Disengaged) {
        return;
    }
    
    if (!slave.backend) {
        return;
    }
    
    // Calculate ramp factor for soft engage/disengage
    double rampFactor = calculateRampFactor(slave);
    
    // Handle state transitions
    if (slave.state == GearingState::Engaging && rampFactor >= 1.0) {
        slave.state = GearingState::Engaged;
        slave.currentRatio = static_cast<double>(slave.config.numerator) / 
                            slave.config.denominator;
        TETHER_LOGI(TAG, "Slave engaged (ramp complete)");
        notifyStateChange(m_slaves.size(), slave.state);  // Index not known here
    } else if (slave.state == GearingState::Disengaging && rampFactor <= 0.0) {
        slave.state = GearingState::Disengaged;
        slave.currentRatio = 0.0;
        TETHER_LOGI(TAG, "Slave disengaged (ramp complete)");
        notifyStateChange(m_slaves.size(), slave.state);
        return;
    }
    
    // Calculate effective ratio
    double targetRatio = static_cast<double>(slave.config.numerator) / 
                         slave.config.denominator;
    slave.currentRatio = targetRatio * rampFactor;
    
    // Calculate master delta since engage
    int32_t masterDelta = masterPos - slave.lastMasterPos;
    
    // Calculate slave target
    double slaveDelta = masterDelta * slave.currentRatio;
    slave.targetPosition = slave.syncPosition + 
                           static_cast<int32_t>(slaveDelta) + 
                           slave.config.offset;
    
    // Apply position command
    slave.backend->setTargetPosition(slave.targetPosition);
    
    // Feed-forward velocity if enabled
    if (slave.config.enableFeedForward) {
        int32_t ffVelocity = static_cast<int32_t>(
            masterVel * slave.currentRatio * slave.config.feedForwardGain
        );
        slave.backend->setVelocityOffset(ffVelocity);
    }
}

double GearingController::calculateRampFactor(const GearingSlave& slave) const {
    if (slave.state == GearingState::Engaged) {
        return 1.0;
    }
    if (slave.state == GearingState::Disengaged) {
        return 0.0;
    }
    
    uint64_t now = esp_timer_get_time();
    double elapsed = (now - slave.engageStartTime) / 1000.0;  // ms
    double factor = elapsed / m_rampTimeMs;
    
    if (slave.state == GearingState::Engaging) {
        return std::clamp(factor, 0.0, 1.0);
    } else if (slave.state == GearingState::Disengaging) {
        return std::clamp(1.0 - factor, 0.0, 1.0);
    }
    
    return 0.0;
}

void GearingController::notifyStateChange(size_t slaveIndex, GearingState newState) {
    if (m_eventCallback && slaveIndex < m_slaves.size()) {
        m_eventCallback(slaveIndex, newState);
    }
}

bool GearingController::isEngaged() const {
    for (const auto& slave : m_slaves) {
        if (slave.state != GearingState::Engaged) {
            return false;
        }
    }
    return !m_slaves.empty();
}

bool GearingController::isAnyEngaged() const {
    for (const auto& slave : m_slaves) {
        if (slave.state == GearingState::Engaged || 
            slave.state == GearingState::Engaging) {
            return true;
        }
    }
    return false;
}

GearingState GearingController::getSlaveState(size_t slaveIndex) const {
    if (slaveIndex >= m_slaves.size()) {
        return GearingState::Error;
    }
    return m_slaves[slaveIndex].state;
}

void GearingController::setGearRatio(size_t slaveIndex, 
                                     int32_t numerator, 
                                     int32_t denominator,
                                     bool ramp) {
    if (slaveIndex >= m_slaves.size() || denominator == 0) {
        return;
    }
    
    GearingSlave& slave = m_slaves[slaveIndex];
    
    // TODO: Implement ramped ratio change
    slave.config.numerator = numerator;
    slave.config.denominator = denominator;
    
    TETHER_LOGI(TAG, "Slave %d ratio changed to %ld:%ld", 
             (int)slaveIndex, (long)numerator, (long)denominator);
}

void GearingController::adjustOffset(size_t slaveIndex, int32_t offsetDelta) {
    if (slaveIndex >= m_slaves.size()) {
        return;
    }
    
    m_slaves[slaveIndex].config.offset += offsetDelta;
}

void GearingController::setOffset(size_t slaveIndex, int32_t offset) {
    if (slaveIndex >= m_slaves.size()) {
        return;
    }
    
    m_slaves[slaveIndex].config.offset = offset;
}

void GearingController::synchronize(size_t slaveIndex) {
    if (slaveIndex >= m_slaves.size() || !m_masterBackend) {
        return;
    }
    
    GearingSlave& slave = m_slaves[slaveIndex];
    
    if (slave.backend) {
        slave.syncPosition = slave.backend->getActualPosition();
        slave.lastMasterPos = m_masterBackend->getActualPosition();
    }
}

void GearingController::synchronizeAll() {
    for (size_t i = 0; i < m_slaves.size(); i++) {
        synchronize(i);
    }
}

int32_t GearingController::getSlaveTarget(size_t slaveIndex) const {
    if (slaveIndex >= m_slaves.size()) {
        return 0;
    }
    return m_slaves[slaveIndex].targetPosition;
}

int32_t GearingController::getSlaveFollowingError(size_t slaveIndex) const {
    if (slaveIndex >= m_slaves.size() || !m_slaves[slaveIndex].backend) {
        return 0;
    }
    
    const GearingSlave& slave = m_slaves[slaveIndex];
    int32_t actual = slave.backend->getActualPosition();
    return slave.targetPosition - actual;
}

double GearingController::getEffectiveRatio(size_t slaveIndex) const {
    if (slaveIndex >= m_slaves.size()) {
        return 0.0;
    }
    return m_slaves[slaveIndex].currentRatio;
}

// ============================================================================
// Multi-Master Gearing
// ============================================================================

void MultiMasterGearing::addMaster(DriveBackendPtr backend, double weight,
                                   int32_t num, int32_t den) {
    MasterInput input;
    input.backend = backend;
    input.weight = weight;
    input.numerator = num;
    input.denominator = den;
    m_masters.push_back(input);
}

void MultiMasterGearing::setSlaveBackend(DriveBackendPtr backend) {
    m_slaveBackend = backend;
}

void MultiMasterGearing::update() {
    if (!m_slaveBackend || m_masters.empty()) {
        return;
    }
    
    // Calculate weighted sum of master positions
    double totalWeight = 0.0;
    double weightedSum = 0.0;
    
    for (const auto& master : m_masters) {
        if (!master.backend) continue;
        
        int32_t masterPos = master.backend->getActualPosition();
        double ratio = static_cast<double>(master.numerator) / master.denominator;
        
        weightedSum += masterPos * ratio * master.weight;
        totalWeight += master.weight;
    }
    
    if (totalWeight > 0) {
        m_slaveTarget = static_cast<int32_t>(weightedSum / totalWeight);
        m_slaveBackend->setTargetPosition(m_slaveTarget);
    }
}

} // namespace CiA402
