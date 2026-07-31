/**
 * @file klipper_clock_sync.cpp
 * @brief Example: clock synchronisation between host and device.
 *
 * @details
 * Demonstrates the clock sync subsystem:
 *   1. Create an MCU clock at 180 MHz.
 *   2. Simulate periodic get_clock exchanges.
 *   3. Show the ClockSync converging on the correct slope.
 *   4. Convert host time delays to MCU clock ticks.
 */

#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/clock/ClockSync.hpp"

#include <cstdio>
#include <chrono>
#include <thread>

using namespace tether::klipper::clock;

int main() {
    const uint32_t clockFreq = 180000000; // 180 MHz
    McuClock mcu(clockFreq);
    ClockSync sync(0.1); // 10s decay time constant

    auto t0 = HostClock::now();

    // Simulate 20 get_clock exchanges, 100ms apart
    for (int i = 0; i < 20; ++i) {
        // Advance the MCU clock by 100ms worth of ticks
        mcu.advanceTo(mcu.ticks32() + clockFreq / 10);

        auto sendTime = t0 + std::chrono::milliseconds(i * 100);
        auto recvTime = t0 + std::chrono::milliseconds(i * 100 + 1); // 1ms RTT

        sync.addSample(sendTime, recvTime, mcu.ticks32());

        if (i % 5 == 0) {
            std::printf("Sample %d: mcuClock=%u synced=%d slope=%.1f\n",
                        i, mcu.ticks32(),
                        sync.isSynchronised(),
                        sync.slope());
        }
    }

    std::printf("\nFinal: synced=%d samples=%zu slope=%.1f\n",
                sync.isSynchronised(),
                sync.sampleCount(),
                sync.slope());

    // Convert a host delay to MCU ticks
    if (sync.isSynchronised()) {
        auto delay = std::chrono::milliseconds(10);
        uint32_t ticks = sync.hostDelayToMcuTicks(delay);
        std::printf("10ms host delay -> %u MCU ticks (expected ~%u)\n",
                    ticks, clockFreq / 100);
    }

    return 0;
}
