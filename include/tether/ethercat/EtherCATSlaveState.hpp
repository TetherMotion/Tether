/**
 * @file EtherCATSlaveState.hpp
 * @brief EtherCAT types: Slave State
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// Slave State
// ============================================================================

enum class SlaveState : uint8_t {
    INIT    = 0x01,
    PRE_OP  = 0x02,
    BOOT    = 0x03,
    SAFE_OP = 0x04,
    OP      = 0x08
};

inline const char* slaveStateToString(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "INIT";
        case SlaveState::PRE_OP:  return "PRE-OP";
        case SlaveState::BOOT:    return "BOOT";
        case SlaveState::SAFE_OP: return "SAFE-OP";
        case SlaveState::OP:      return "OP";
        default: return "UNKNOWN";
    }
}

} // namespace EtherCAT
