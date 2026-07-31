/**
 * @file klipper_multi_mcu.cpp
 * @brief Example: multi-MCU coordination and clock synchronization.
 *
 * @details
 * Demonstrates the MultiMcuManager for coordinating secondary MCUs:
 *   1. Create a MultiMcuManager and register secondary MCUs.
 *   2. Set serial paths and baud rates for each MCU.
 *   3. Configure clock frequencies for clock synchronization.
 *   4. Enable MCUs and verify connection status.
 *   5. Update statistics and report per-MCU status.
 *   6. Demonstrate clock sync between primary and secondary MCUs.
 */

#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/clock/ClockSync.hpp"

#include <cstdio>
#include <chrono>
#include <memory>

using namespace tether::klipper::klippy;
using namespace tether::klipper::clock;

int main() {
    // 1. Create a MultiMcuManager
    MultiMcuManager mgr;
    std::printf("MultiMcuManager created\n");

    // 2. Register secondary MCUs with serial paths and baud rates
    // MCU 1: toolhead MCU on /dev/ttyUSB0
    mgr.setSerialPath(1, "/dev/ttyUSB0");
    mgr.setBaudRate(1, 250000);
    mgr.setClockFreq(1, 180000000); // 180 MHz
    mgr.setFirmwareVersion(1, "tether-mcu-1.0");

    // MCU 2: bed/extras MCU on /dev/ttyACM0
    mgr.setSerialPath(2, "/dev/ttyACM0");
    mgr.setBaudRate(2, 115200);
    mgr.setClockFreq(2, 48000000);  // 48 MHz
    mgr.setFirmwareVersion(2, "tether-mcu-1.0");

    // MCU 3: expansion MCU on /dev/ttyUSB1
    mgr.setSerialPath(3, "/dev/ttyUSB1");
    mgr.setBaudRate(3, 250000);
    mgr.setClockFreq(3, 72000000);  // 72 MHz

    std::printf("Registered %zu secondary MCUs\n", mgr.mcuIds().size());

    // 3. Enable the MCUs (simulated connection)
    mgr.setEnabled(1, true);
    mgr.setEnabled(2, true);
    mgr.setEnabled(3, true);

    // 4. Update statistics for each MCU
    mgr.updateStats(1, 1024, 2048, 0);
    mgr.updateStats(2, 512, 1024, 1);
    mgr.updateStats(3, 256, 512, 0);

    // 5. Report per-MCU status
    std::printf("\n--- MCU Status ---\n");
    for (int id : mgr.mcuIds()) {
        std::printf("%s\n", mgr.getStatus(id).c_str());
        const auto* mcu = mgr.getMcu(id);
        if (mcu) {
            std::printf("  firmware=%s freq=%u Hz connected=%d\n",
                        mcu->firmwareVersion.c_str(),
                        mcu->clockFreq, mcu->connected);
        }
    }

    // 6. Clock synchronization between primary and a secondary MCU
    std::printf("\n--- Clock Synchronization ---\n");
    const uint32_t primaryFreq = 180000000;
    const uint32_t secondaryFreq = 48000000;

    McuClock primaryClock(primaryFreq);
    McuClock secondaryClock(secondaryFreq);
    ClockSync sync(0.1); // 10s decay

    auto t0 = HostClock::now();

    // Simulate 15 clock sync exchanges between primary and secondary
    for (int i = 0; i < 15; ++i) {
        primaryClock.advanceTo(primaryClock.ticks32() + primaryFreq / 10);
        secondaryClock.advanceTo(secondaryClock.ticks32() + secondaryFreq / 10);

        auto sendTime = t0 + std::chrono::milliseconds(i * 100);
        auto recvTime = t0 + std::chrono::milliseconds(i * 100 + 2); // 2ms RTT

        sync.addSample(sendTime, recvTime, secondaryClock.ticks32());

        if (i % 5 == 0) {
            std::printf("Sample %d: primary=%u secondary=%u synced=%d slope=%.1f\n",
                        i, primaryClock.ticks32(), secondaryClock.ticks32(),
                        sync.isSynchronised(), sync.slope());
        }
    }

    std::printf("\nFinal sync: synced=%d samples=%zu slope=%.1f\n",
                sync.isSynchronised(), sync.sampleCount(), sync.slope());

    // Convert a host delay to secondary MCU ticks
    if (sync.isSynchronised()) {
        auto delay = std::chrono::milliseconds(10);
        uint32_t ticks = sync.hostDelayToMcuTicks(delay);
        std::printf("10ms host delay -> %u secondary MCU ticks (expected ~%u)\n",
                    ticks, secondaryFreq / 100);
    }

    std::printf("\nDone\n");
    return 0;
}
