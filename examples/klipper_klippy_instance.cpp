/**
 * @file klipper_klippy_instance.cpp
 * @brief Example: full KlippyInstance integration demo.
 *
 * @details
 * Demonstrates the integrated KlippyInstance facade:
 *   1. Create a KlippyInstance with custom config.
 *   2. Wire up heaters (extruder + bed) using Thermistor sensors.
 *   3. Wire up a Fan and a Probe.
 *   4. Execute G-code commands (G28 home, G1 move, M104 set temp, M109 wait).
 *   5. Query printer state via the UDS object model.
 *   6. Start and stop the server.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <cstdio>
#include <memory>

using namespace tether::klipper;
using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

int main() {
    // 1. Create a KlippyInstance with a custom config
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = "/tmp/tether_klippy_instance_uds";
    cfg.sdcardDir = "/tmp/tether_klippy_instance_sd";
    cfg.firmwareVersion = "tether-klippy-demo-1.0";
    KlippyInstance inst(cfg);

    // 2. Set up heaters (extruder + bed) with Thermistor sensors
    // Simulated ADC: returns a value corresponding to ~25°C at rest
    auto extruderAdc = []() -> double { return 4095.0 * 0.5; };
    auto bedAdc = []() -> double { return 4095.0 * 0.5; };

    Thermistor::Params thermParams;
    auto extruderSensor = std::make_shared<Thermistor>(0, thermParams, extruderAdc);
    auto bedSensor = std::make_shared<Thermistor>(1, thermParams, bedAdc);

    auto extruderHeater = std::make_shared<Heater>(
        0, [](double) {}, [&]() { return extruderSensor->read(); });
    extruderHeater->setPidParams({14.0, 0.1, 50.0, 100.0, 0.0, 1.0});

    auto bedHeater = std::make_shared<Heater>(
        1, [](double) {}, [&]() { return bedSensor->read(); });
    bedHeater->setPidParams({10.0, 0.05, 30.0, 100.0, 0.0, 1.0});

    inst.setExtruderHeater(extruderHeater);
    inst.setHeaterBed(bedHeater);

    // 3. Set up a Fan and a Probe
    auto fan = std::make_shared<Fan>(2, [](double) {});
    inst.setFan(fan);

    auto probe = std::make_shared<Probe>(3, []() { return false; });
    inst.setProbe(probe);

    // Register endstops for M119 / query_endstops
    auto xEndstop = std::make_shared<Endstop>(4, []() { return false; });
    auto yEndstop = std::make_shared<Endstop>(5, []() { return false; });
    auto zEndstop = std::make_shared<Endstop>(6, []() { return false; });
    inst.setEndstop("x", xEndstop);
    inst.setEndstop("y", yEndstop);
    inst.setEndstop("z", zEndstop);

    // 4. Execute G-code commands
    std::printf("Executing G-code commands...\n");
    inst.executeGcode("G28");                      // Home all axes
    inst.executeGcode("G1 X50 Y50 F3000");         // Move to (50,50)
    inst.executeGcode("M104 S200");                // Set extruder target to 200°C
    inst.executeGcode("M140 S60");                 // Set bed target to 60°C
    inst.executeGcode("M106 S128");                // Set fan to 50%
    inst.executeGcode("M109 S200");                // Wait for extruder temp (returns quickly)

    std::printf("Extruder target: %.1f°C\n", extruderHeater->target());
    std::printf("Bed target: %.1f°C\n", bedHeater->target());
    std::printf("Fan speed: %.2f\n", fan->speed());

    // 5. Query printer state via UDS objects
    auto& server = inst.server();
    auto status = server.queryObjects({{"toolhead", {}}, {"extruder", {}}, {"fan", {}}});
    for (const auto& [objName, fields] : status) {
        std::printf("Object '%s': %zu fields\n", objName.c_str(), fields.size());
    }

    // 6. Start and stop the server
    bool started = inst.start();
    std::printf("Server started: %s\n", started ? "OK" : "FAIL");
    inst.stop();
    std::printf("Server stopped\n");

    std::printf("Done\n");
    return 0;
}
