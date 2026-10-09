/**
 * @file DebugConditions.cpp
 * @brief DebugGate — debug condition implementations.
 *
 * TU split out of DebugGate.cpp.
 */

#include "tether/ethercat/DebugGate.hpp"

#if TETHER_DEBUG_GATE_ENABLED

#include "tether/platform/Platform.hpp"
#include "DebugGateInternal.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

namespace EtherCAT {

// ============================================================================
// StateCondition
// ============================================================================

void StateCondition::onCheckpoint(const std::string& name, uint16_t slave_index) {
    if (fired_) return;
    if (name == stateCheckpointName(state_)) {
        if (slave_index_ == 0xFFFF || slave_index_ == slave_index) {
            fired_ = true;
        }
    }
}

// ============================================================================
// CheckpointCondition
// ============================================================================

void CheckpointCondition::onCheckpoint(const std::string& name, uint16_t slave_index) {
    if (fired_) return;
    if (name == checkpoint_name_) {
        if (slave_index_ == 0xFFFF || slave_index_ == slave_index) {
            fired_ = true;
        }
    }
}

// ============================================================================
// RegisterInterceptCondition
// ============================================================================

void RegisterInterceptCondition::onRegisterRead(uint16_t slave_index, uint16_t addr,
                                                 const uint8_t* data, uint16_t len) {
    if (fired_) return;
    if (addr != reg_addr_) return;
    if (slave_index_ != 0xFFFF && slave_index_ != slave_index) return;
    if (len == 0 || !data) return;

    uint64_t read_val = readLEValue(data, len);
    if (compareValues(read_val, op_, value_)) {
        fired_ = true;
    }
}

// ============================================================================
// CoEInterceptCondition
// ============================================================================

void CoEInterceptCondition::onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                                       const uint8_t* data, size_t len) {
    if (fired_) return;
    if (index != obj_index_) return;
    if (has_sub_ && sub != sub_index_) return;
    if (slave_index_ != 0xFFFF && slave_index_ != slave_index) return;
    if (len == 0 || !data) return;

    uint64_t read_val = readLEValue(data, len);
    if (compareValues(read_val, op_, value_)) {
        fired_ = true;
    }
}

// ============================================================================
// CustomCondition
// ============================================================================

bool CustomCondition::hasFired() const {
    if (fired_) return true;
    if (cb_ && cb_()) {
        const_cast<CustomCondition*>(this)->fired_ = true;
        return true;
    }
    return false;
}

} // namespace EtherCAT

#endif // TETHER_DEBUG_GATE_ENABLED
