// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <functional>
#include <bit>

#include "tether/ethercat/DCRegisters.hpp"

namespace EtherCAT { namespace slave {

namespace DCSyncActivation {
    constexpr uint16_t CyclicOperation    = 0x0001;
    constexpr uint16_t Sync0Enable        = 0x0100;
    constexpr uint16_t Sync1Enable        = 0x0200;
    constexpr uint16_t Sync0StartTime     = 0x0400;
}

struct DCState {
    uint64_t systemTime = 0;
    uint64_t receiveTimePort0 = 0;
    uint64_t receiveTimePort1 = 0;
    int64_t  systemTimeOffset = 0;
    int32_t  systemTimeDelay = 0;
    int32_t  systemTimeDiff = 0;

    uint16_t speedCounterStart = 0;
    uint16_t speedCounterDiff = 0;
    uint8_t  filterDepth = 4;
    uint8_t  controlLoop = 0;

    EtherCAT::DC::DCSyncActReg16 syncActivation{};
    uint64_t syncStartTime = 0;
    uint32_t sync0CycleTime = 0;
    uint32_t sync1CycleTime = 0;

    EtherCAT::DC::DCLatchCtrlReg   latchControl{};
    EtherCAT::DC::DCLatchStatusReg latchStatus{};
    uint64_t latch0TimePos = 0;
    uint64_t latch0TimeNeg = 0;
    uint64_t latch1TimePos = 0;
    uint64_t latch1TimeNeg = 0;

    bool     sync0Active = false;
    bool     sync1Active = false;
    uint64_t lastSync0Time = 0;
    uint64_t lastSync1Time = 0;

    bool isCyclicOperation() const { return syncActivation.cyclic_operation; }
    bool isSync0Enabled() const { return syncActivation.sync0_enable; }
    bool isSync1Enabled() const { return syncActivation.sync1_enable; }

    void advanceTime(uint64_t deltaNs) {
        systemTime += deltaNs;
    }

    bool checkSyncTrigger(std::function<void(int, uint64_t)> callback) {
        bool triggered = false;

        if (isSync0Enabled() && sync0CycleTime > 0) {
            if (lastSync0Time == 0) {
                // First trigger: initialize to current time to avoid
                // repeated triggers catching up from epoch 0.
                lastSync0Time = systemTime;
            }
            if ((systemTime - lastSync0Time) >= sync0CycleTime) {
                lastSync0Time += sync0CycleTime;
                if (callback) callback(0, systemTime);
                triggered = true;
            }
        }

        if (isSync1Enabled() && sync1CycleTime > 0) {
            if (lastSync1Time == 0) {
                lastSync1Time = systemTime;
            }
            if ((systemTime - lastSync1Time) >= sync1CycleTime) {
                lastSync1Time += sync1CycleTime;
                if (callback) callback(1, systemTime);
                triggered = true;
            }
        }

        return triggered;
    }
};

}} // namespace EtherCAT::slave
