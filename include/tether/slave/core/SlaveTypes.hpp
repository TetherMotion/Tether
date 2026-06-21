/**
 * @file SlaveTypes.hpp
 * @brief Core type definitions for EtherCAT slave implementation (umbrella header)
 *
 * @details
 * This header has been split into focused, modular headers. It now serves as
 * a backward-compatible umbrella that includes all the split headers.
 *
 * Split headers:
 * - ALTypes.hpp          — AL state machine, ALStatus, ALControl, ALStatusCode
 * - FMMUConfigSlave.hpp  — FMMUConfig with bitstruct type/activate fields
 * - SMConfigSlave.hpp    — SyncManagerConfig with bitstruct control/status/activate
 * - DCStateSlave.hpp     — DCState with bitstruct syncActivation/latch fields
 * - WatchdogStateSlave.hpp — WatchdogState with bitstruct status
 * - SIIStateSlave.hpp    — SIIState with bitstruct control
 * - ESCConfig.hpp        — ESCConfig with bitstruct features
 * - SlaveIdentity.hpp    — SlaveIdentity struct
 * - SlaveCallbacks.hpp   — Callback type aliases
 *
 * Register address constants are in tether/ethercat/ESCRegisterMap.hpp
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>
#include <bit>

#include "tether/ethercat/ESCRegisterMap.hpp"
#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/ALRegisters.hpp"
#include "tether/ethercat/FMMURegisters.hpp"
#include "tether/ethercat/DCRegisters.hpp"
#include "tether/ethercat/WatchdogRegisters.hpp"
#include "tether/ethercat/SIIRegisters.hpp"
#include "tether/ethercat/ESCFeatureReg.hpp"

#include "tether/slave/core/ALTypes.hpp"
#include "tether/slave/core/FMMUConfigSlave.hpp"
#include "tether/slave/core/SMConfigSlave.hpp"
#include "tether/slave/core/DCStateSlave.hpp"
#include "tether/slave/core/WatchdogStateSlave.hpp"
#include "tether/slave/core/SIIStateSlave.hpp"
#include "tether/slave/core/ESCConfig.hpp"
#include "tether/slave/core/SlaveIdentity.hpp"
#include "tether/slave/core/SlaveCallbacks.hpp"

namespace EtherCAT {
namespace slave {

// Backward-compatible alias for ESCReg namespace
namespace ESCReg = ::EtherCAT::ESCReg;

}  // namespace slave
}  // namespace EtherCAT
