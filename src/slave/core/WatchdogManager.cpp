// SPDX-License-Identifier: MIT

#include "tether/slave/core/WatchdogManager.hpp"

namespace EtherCAT { namespace slave {

void WatchdogManager::update(SlaveState currentState, uint64_t /*deltaNs*/) {
    if (!enabled_) return;

    // Check SM watchdog (only active in OP state)
    if (currentState == SlaveState::OP) {
        state_->smCounter++;
        if (state_->smCounter >= state_->smTimeout) {
            state_->status.sm_triggered = 1;  // SM watchdog triggered
            if (callback_) {
                callback_(false, true);
            }
        }
    }
}

}} // namespace EtherCAT::slave
