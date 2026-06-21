// SPDX-License-Identifier: MIT
#pragma once

#include <functional>
#include <cstdint>

#include "tether/slave/core/ALTypes.hpp"

namespace EtherCAT { namespace slave {

using StateChangeCallback = std::function<void(SlaveState oldState, SlaveState newState)>;
using SyncCallback = std::function<void(int syncNum, uint64_t timestamp)>;
using PDOExchangeCallback = std::function<void()>;
using WatchdogCallback = std::function<void(bool pdi, bool sm)>;

}} // namespace EtherCAT::slave
