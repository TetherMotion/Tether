/**
 * @file SlaveGroup.cpp
 * @brief Single-packet AL state control/query for slave groups — see header.
 */

#include "tether/ethercat/SlaveGroup.hpp"

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include "raw/RawWireFormat.hpp"

#include "logging/Logger.hpp"

#include <cstring>
#include <format>
#include <string>

namespace EtherCAT {

static const char* TAG = "SlaveGroup";

// ============================================================================
// Factories
// ============================================================================

SlaveGroup SlaveGroup::all(Master& master)
{
    SlaveGroup g(master, std::span<const uint16_t>{});
    g.broadcast_ = true;
    return g;
}

SlaveGroup SlaveGroup::allAddressed(Master& master)
{
    const uint16_t n = master.getDiscoveredSlaveCount();
    std::vector<uint16_t> indices(n);
    for (uint16_t i = 0; i < n; ++i) indices[i] = i;
    return SlaveGroup(master, indices);
}

} // namespace EtherCAT
