// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

#include "tether/ethercat/WatchdogRegisters.hpp"

namespace EtherCAT { namespace slave {

struct WatchdogState {
    uint16_t divider = 2498;
    uint16_t pdiTimeout = 1000;
    uint16_t smTimeout = 1000;

    EtherCAT::Watchdog::WatchdogStatusReg status{};
    uint16_t smCounter = 0;
    uint8_t  pdiCounter = 0;

    uint64_t lastPdiAccess = 0;
    uint64_t lastSmAccess = 0;

    bool isPdiTriggered() const { return status.pdi_triggered; }
    bool isSmTriggered() const { return status.sm_triggered; }

    void resetPdiWatchdog() {
        pdiCounter = 0;
        status.pdi_triggered = 0;
    }

    void resetSmWatchdog() {
        smCounter = 0;
        status.sm_triggered = 0;
    }
};

}} // namespace EtherCAT::slave
