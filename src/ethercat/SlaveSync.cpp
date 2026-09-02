// SPDX-License-Identifier: MIT
/**
 * @file SlaveSync.cpp
 * @brief Abstract multi-slave state-transition synchronization for EtherCAT
 */

#include "tether/ethercat/SlaveSync.hpp"

#include "tether/profiles/cia402/CiA402StateUtils.hpp"  // ECState, getECStateName

#include <algorithm>
#include <chrono>

namespace EtherCAT {
namespace Util {

namespace {
constexpr const char* TAG = "SlaveSync";
}

// ============================================================================
// SlaveSyncGate
// ============================================================================

void SlaveSyncGate::notify() {
    std::lock_guard<std::mutex> lk(mtx_);
    cv_.notify_all();
}

bool SlaveSyncGate::waitAll(std::stop_token st,
                            const std::vector<SlaveSyncSlot*>& slots) const {
    std::unique_lock<std::mutex> lk(mtx_);
    while (!st.stop_requested()) {
        // Check for any failure
        for (const auto* slot : slots) {
            if (slot && slot->failed.load(std::memory_order_relaxed)) {
                return false;
            }
        }
        // Check if all are in target state
        bool all_in_target = true;
        for (const auto* slot : slots) {
            if (slot && !slot->in_target_state.load(std::memory_order_relaxed)) {
                all_in_target = false;
                break;
            }
        }
        if (all_in_target) return true;
        cv_.wait_for(lk, std::chrono::seconds(1));
    }
    return false;
}

bool SlaveSyncGate::waitOne(std::stop_token st,
                            const SlaveSyncSlot& slot) const {
    std::unique_lock<std::mutex> lk(mtx_);
    while (!st.stop_requested()) {
        if (slot.failed.load(std::memory_order_relaxed)) {
            return false;
        }
        if (slot.in_target_state.load(std::memory_order_relaxed)) {
            return true;
        }
        cv_.wait_for(lk, std::chrono::seconds(1));
    }
    return false;
}

// ============================================================================
// SlaveSyncCoordinator
// ============================================================================

void SlaveSyncCoordinator::registerSlot(SlaveSyncSlot& slot) {
    slots_.push_back(&slot);
}

void SlaveSyncCoordinator::unregisterSlot(uint16_t slave_index) {
    slots_.erase(
        std::remove_if(slots_.begin(), slots_.end(),
                       [slave_index](SlaveSyncSlot* s) {
                           return s && s->slave_index == slave_index;
                       }),
        slots_.end());
}

SlaveSyncSlot* SlaveSyncCoordinator::findSlot(uint16_t slave_index) const {
    for (auto* slot : slots_) {
        if (slot && slot->slave_index == slave_index) {
            return slot;
        }
    }
    return nullptr;
}

void SlaveSyncCoordinator::clearAllReady() {
    for (auto* slot : slots_) {
        if (slot) slot->ready.store(false, std::memory_order_release);
    }
}

void SlaveSyncCoordinator::clearReady(uint16_t slave_index) {
    if (auto* slot = findSlot(slave_index)) {
        slot->ready.store(false, std::memory_order_release);
    }
}

bool SlaveSyncCoordinator::transitionAllTo(ECState target,
                                           const TransitionContext& ctx,
                                           bool call_on_reached) {
    bool all_ok = true;
    for (auto* slot : slots_) {
        if (!slot || !slot->handler) continue;

        const char* name = slot->name.empty() ? "?" : slot->name.c_str();
        TETHER_LOGI(TAG,
                    "Transitioning {} (slave {}) to {}{}...",
                    name, slot->slave_index,
                    getECStateName(target),
                    ctx.is_recovery ? " [recovery]" : "");

        const bool ok = slot->handler->transitionTo(target, ctx);
        if (ok) {
            slot->in_target_state.store(true, std::memory_order_release);
            slot->failed.store(false, std::memory_order_release);
            if (call_on_reached && slot->on_reached) {
                slot->on_reached();
            }
            TETHER_LOGI(TAG,
                        "{} (slave {}) reached {}",
                        name, slot->slave_index, getECStateName(target));
        } else {
            slot->failed.store(true, std::memory_order_release);
            slot->in_target_state.store(false, std::memory_order_release);
            all_ok = false;
            TETHER_LOGE(TAG,
                        "{} (slave {}) FAILED to reach {}",
                        name, slot->slave_index, getECStateName(target));
        }
    }
    return all_ok;
}

bool SlaveSyncCoordinator::transitionOne(uint16_t slave_index,
                                         ECState target,
                                         const TransitionContext& ctx,
                                         bool call_on_reached) {
    auto* slot = findSlot(slave_index);
    if (!slot || !slot->handler) {
        TETHER_LOGE(TAG,
                    "Cannot transition slave {} — not registered",
                    slave_index);
        return false;
    }

    const char* name = slot->name.empty() ? "?" : slot->name.c_str();
    TETHER_LOGI(TAG,
                "Transitioning {} (slave {}) to {}{}...",
                name, slot->slave_index,
                getECStateName(target),
                ctx.is_recovery ? " [recovery]" : "");

    const bool ok = slot->handler->transitionTo(target, ctx);
    if (ok) {
        slot->in_target_state.store(true, std::memory_order_release);
        slot->failed.store(false, std::memory_order_release);
        if (call_on_reached && slot->on_reached) {
            slot->on_reached();
        }
        TETHER_LOGI(TAG,
                    "{} (slave {}) reached {}",
                    name, slot->slave_index, getECStateName(target));
    } else {
        slot->failed.store(true, std::memory_order_release);
        slot->in_target_state.store(false, std::memory_order_release);
        TETHER_LOGE(TAG,
                    "{} (slave {}) FAILED to reach {}",
                    name, slot->slave_index, getECStateName(target));
    }
    return ok;
}

bool SlaveSyncCoordinator::transitionAllAndWait(ECState target,
                                                const TransitionContext& ctx,
                                                SlaveSyncGate& gate,
                                                std::stop_token st) {
    // Clear in_target_state for all slots before transitioning
    for (auto* slot : slots_) {
        if (slot) {
            slot->in_target_state.store(false, std::memory_order_release);
        }
    }

    if (!transitionAllTo(target, ctx, true)) {
        return false;
    }

    return gate.waitAll(st, slots_);
}

// ============================================================================
// SyncRecoveryHandler
// ============================================================================

SyncRecoveryHandler::SyncRecoveryHandler(SlaveSyncCoordinator& coord)
    : coord_(coord) {}

bool SyncRecoveryHandler::reinitializeSlave(uint16_t slave_index) {
    auto* slot = coord_.findSlot(slave_index);
    if (!slot || !slot->handler) {
        TETHER_LOGE(TAG,
                    "Recovery: slave {} not registered — cannot recover",
                    slave_index);
        return false;
    }

    const char* name = slot->name.empty() ? "?" : slot->name.c_str();
    TETHER_LOGI(TAG,
                "Recovery: reinitializing {} (slave {})...",
                name, slave_index);

    // Clear ready flag so the exchange thread stops cycling this group
    slot->ready.store(false, std::memory_order_release);
    slot->reset();

    // Full re-initialization (INIT → PRE_OP → SAFE_OP → OP)
    const bool ok = slot->handler->reinitialize();
    if (ok) {
        slot->in_target_state.store(true, std::memory_order_release);
        if (slot->on_reached) {
            slot->on_reached();
        }
        TETHER_LOGI(TAG,
                    "Recovery: {} (slave {}) reinitialized successfully",
                    name, slave_index);
    } else {
        slot->failed.store(true, std::memory_order_release);
        TETHER_LOGE(TAG,
                    "Recovery: {} (slave {}) reinitialization FAILED",
                    name, slave_index);
    }
    return ok;
}

} // namespace Util
} // namespace EtherCAT
