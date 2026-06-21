// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace Watchdog {

struct WatchdogStatusReg {
    uint8_t pdi_triggered : 1;
    uint8_t sm_triggered  : 1;
    uint8_t reserved      : 6;

    bool operator==(const WatchdogStatusReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const WatchdogStatusReg& o) const { return !(*this == o); }
};
static_assert(sizeof(WatchdogStatusReg) == 1, "WatchdogStatusReg must be 1 byte");

}} // namespace EtherCAT::Watchdog
