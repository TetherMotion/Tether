/**
 * @file raw_internal_bridge.cpp
 * @brief Minimal bridge — all global state has been removed.
 *
 * The former BridgeContext singleton is gone.  All transport primitives
 * are now member functions on Master; call them directly.
 *
 * Only the stateless adp_for_slave_index helper remains here.
 */

#include "tether/ethercat/Master.hpp"
#include "raw/internal.hpp"

namespace EtherCAT {
namespace Raw {

// ============================================================================
// adp_for_slave_index — pure helper, no global state
// ============================================================================

uint16_t adp_for_slave_index(uint16_t slave_index) {
    return Master::adpForSlaveIndex(slave_index);
}

} // namespace Raw
} // namespace EtherCAT
