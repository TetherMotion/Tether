/**
 * @file klipper_loopback_demo.cpp
 * @brief Example: host connects to device over loopback, downloads dict, syncs clock.
 *
 * @details
 * Demonstrates the full Klipper protocol handshake:
 *   1. Build a device config with standard commands.
 *   2. Create a loopback transport pair.
 *   3. Start the device and connect the host.
 *   4. Download the data dictionary via the identify handshake.
 *   5. Synchronise the clock via get_clock.
 *   6. Send a command and receive a response.
 */

#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"

#include <cstdio>
#include <memory>

using namespace tether::klipper;

int main() {
    // 1. Build device config
    config::KlipperConfig cfg;
    config::withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();
    std::printf("Device dictionary: %zu messages\n", dict.messages().size());

    // 2. Create loopback transport pair
    auto hostToDev = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
    auto devToHost = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
    auto hostT = std::make_shared<transport::LoopbackTransport>();
    auto devT = std::make_shared<transport::LoopbackTransport>();
    hostT->wire(hostToDev, devToHost);
    devT->wire(devToHost, hostToDev);
    hostT->open();
    devT->open();

    // 3. Start device and connect host
    device::KlipperDeviceConfig dcfg;
    dcfg.clockFreqHz = 180000000;
    device::KlipperDevice dev(devT, dict, dcfg);
    dev.start();

    auto host = std::make_shared<klippy::KlippyHost>(hostT);
    host->connect();
    std::printf("Host connected\n");

    // 4. Download dictionary
    bool ok = host->downloadDictionary([&](){ dev.pump(); });
    std::printf("Dictionary download: %s (%zu messages)\n",
                ok ? "OK" : "FAIL", host->dictionary().messages().size());

    if (!ok) return 1;

    // 5. Sync clock
    for (int i = 0; i < 10; ++i) {
        dev.advanceClock(180000000); // 1 second
        host->syncClock([&](){ dev.pump(); });
    }
    std::printf("Clock sync: %s (%zu samples, slope=%.1f)\n",
                host->clockSync().isSynchronised() ? "OK" : "FAIL",
                host->clockSync().sampleCount(),
                host->clockSync().slope());

    // 6. Send a command
    ok = host->sendCommand("get_clock", {});
    std::printf("Send get_clock: %s\n", ok ? "OK" : "FAIL");
    for (int i = 0; i < 100; ++i) { dev.pump(); host->pump(); }

    std::printf("Done\n");
    return 0;
}
