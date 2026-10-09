/**
 * @file DebugGateInternal.hpp
 * @brief Shared helpers for the DebugGate translation units.
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

// ============================================================================
// Helpers
// ============================================================================

inline const char* compareOpStr(CompareOp op) {
    switch (op) {
        case CompareOp::Eq:      return "==";
        case CompareOp::Ne:      return "!=";
        case CompareOp::Bitmask: return "&";
        case CompareOp::Ge:      return ">=";
        case CompareOp::Le:      return "<=";
        case CompareOp::Gt:      return ">";
        case CompareOp::Lt:      return "<";
        default:                 return "?";
    }
}

inline bool compareValues(uint64_t read_val, CompareOp op, uint64_t expected) {
    switch (op) {
        case CompareOp::Eq:      return read_val == expected;
        case CompareOp::Ne:      return read_val != expected;
        case CompareOp::Bitmask: return (read_val & expected) == expected;
        case CompareOp::Ge:      return read_val >= expected;
        case CompareOp::Le:      return read_val <= expected;
        case CompareOp::Gt:      return read_val >  expected;
        case CompareOp::Lt:      return read_val <  expected;
        default:                 return false;
    }
}

inline uint64_t readLEValue(const uint8_t* data, size_t len) {
    uint64_t val = 0;
    size_t copy_len = len < sizeof(uint64_t) ? len : sizeof(uint64_t);
    for (size_t i = 0; i < copy_len; ++i) {
        val |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return val;
}

inline const char* stateCheckpointName(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "state:init";
        case SlaveState::PRE_OP:  return "state:pre-op";
        case SlaveState::BOOT:    return "state:boot";
        case SlaveState::SAFE_OP: return "state:safe-op";
        case SlaveState::OP:      return "state:op";
        default:                  return "state:unknown";
    }
}

} // namespace EtherCAT
