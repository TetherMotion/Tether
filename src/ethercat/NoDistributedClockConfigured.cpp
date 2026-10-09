/**
 * @file NoDistributedClockConfigured.cpp
 * @brief NoDistributedClockConfigured sentinel implementation.
 *
 * TU split out of DCClass.cpp.
 */

#include "tether/ethercat/DCClass.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"

#include <cstring>
#include <algorithm>

namespace EtherCAT {

static const char* TAG = "DC sync";

// ============================================================================
// NoDistributedClockConfigured (sentinel)
// ============================================================================

NoDistributedClockConfigured::NoDistributedClockConfigured()
    : NoopDCTransport()
    , EtherCATDC(static_cast<NoopDCTransport&>(*this), 1, nullptr)
{
    // Keep the base class in Disabled state; do not initialize.
}

void NoDistributedClockConfigured::logCritical(const char* method) const {
    TETHER_LOGE(TAG, "CRITICAL: {}() called on uninitialized Distributed Clock. Call dc().init() first.", method);
}

bool NoDistributedClockConfigured::start() { logCritical("start"); return false; }
bool NoDistributedClockConfigured::start(std::function<bool()> ) { logCritical("start"); return false; }
bool NoDistributedClockConfigured::init() { logCritical("init"); return false; }
void NoDistributedClockConfigured::stop() { logCritical("stop"); }

DCState NoDistributedClockConfigured::getState() const { logCritical("getState"); return DCState::Disabled; }
DCLoopStats NoDistributedClockConfigured::getStats() const { logCritical("getStats"); return DCLoopStats{}; }

void NoDistributedClockConfigured::forceSync() { logCritical("forceSync"); }
void NoDistributedClockConfigured::setPDOEnabled(bool) { logCritical("setPDOEnabled"); }
bool NoDistributedClockConfigured::isPDOEnabled() const { logCritical("isPDOEnabled"); return false; }

bool NoDistributedClockConfigured::isSlaveSupported(uint16_t) const { logCritical("isSlaveSupported"); return false; }
int64_t NoDistributedClockConfigured::getSlaveOffset(uint16_t) const { logCritical("getSlaveOffset"); return 0; }

void NoDistributedClockConfigured::readSyncConfig(uint16_t) { logCritical("readSyncConfig"); }
bool NoDistributedClockConfigured::reconfigureSync(uint16_t) { logCritical("reconfigureSync"); return false; }

uint64_t NoDistributedClockConfigured::getMasterTimeNs() { logCritical("getMasterTimeNs"); return 0; }

bool NoDistributedClockConfigured::readRegister(uint16_t, DCRegisters, void*, uint16_t, unsigned int) { logCritical("readRegister"); return false; }
bool NoDistributedClockConfigured::writeRegister(uint16_t, DCRegisters, const void*, uint16_t, unsigned int) { logCritical("writeRegister"); return false; }

bool NoDistributedClockConfigured::sendSyncFrame() { logCritical("sendSyncFrame"); return false; }

} // namespace EtherCAT
