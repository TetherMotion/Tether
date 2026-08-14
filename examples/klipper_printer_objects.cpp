/**
 * @file klipper_printer_objects.cpp
 * @brief Example: printer object model and status queries.
 *
 * @details
 * This example demonstrates the printer object model exposed by Tether's
 * Klipper layer via the UDS server. It covers:
 *
 *   1. Core objects (toolhead, gcode_move, print_stats, virtual_sdcard)
 *   2. Heater objects (extruder, heater_bed, heaters)
 *   3. Fan objects (fan, controller_fan, heater_fan)
 *   4. Motion objects (motion_report, idle_timeout, stepper_enable)
 *   5. Probe and bed leveling objects (probe, bed_mesh, bed_tilt)
 *   6. Sensor objects (temperature_sensor, filament_switch_sensor)
 *   7. LED objects (led, neopixel, dotstar)
 *   8. TMC driver objects (tmc_driver, tmc_uart)
 *   9. Advanced objects (input_shaper, pressure_advance, skew_correction)
 *  10. Exclude object (exclude_object)
 *  11. Firmware retraction (firmware_retraction)
 *  12. Force move (force_move)
 *  13. Manual probe (manual_probe)
 *  14. G-code macro objects (gcode_macro)
 *  15. New D2 objects (load_cell, canbus_stats, angle, palette2, menu, gcode)
 *
 * Each section creates the object, sets some state, and queries the status
 * to show the JSON-like output that Moonraker would receive.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace tether::klipper;
using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

static void printSection(const char* title) {
    std::printf("\n--- %s ---\n\n", title);
}

static void printStatus(const std::string& objName,
                        const std::map<std::string, JsonValue>& status) {
    std::printf("  %s:\n", objName.c_str());
    for (const auto& [key, val] : status) {
        std::printf("    %s = ", key.c_str());
        if (val.isDouble()) {
            std::printf("%.4f\n", val.asDouble());
        } else if (val.isInt()) {
            std::printf("%ld\n", (long)val.asInt());
        } else if (val.isBool()) {
            std::printf("%s\n", val.asBool() ? "true" : "false");
        } else if (val.isString()) {
            std::printf("\"%s\"\n", val.asString().c_str());
        } else if (val.isArray()) {
            std::printf("[array with %zu elements]\n", val.asArray().size());
        } else if (val.isObject()) {
            std::printf("{object with %zu keys}\n", val.asObject().size());
        } else {
            std::printf("(unknown type)\n");
        }
    }
}

int main() {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = "/tmp/tether_printer_objects_uds";
    cfg.sdcardDir = "/tmp/tether_printer_objects_sd";
    KlippyInstance inst(cfg);

    // Wire up basic hardware
    Thermistor::Params thermParams;
    auto extruderSensor = std::make_shared<Thermistor>(0, thermParams, []() { return 4095.0 * 0.5; });
    auto bedSensor = std::make_shared<Thermistor>(1, thermParams, []() { return 4095.0 * 0.5; });
    auto extruderHeater = std::make_shared<Heater>(0, [](double) {}, [&]() { return extruderSensor->read(); });
    auto bedHeater = std::make_shared<Heater>(1, [](double) {}, [&]() { return bedSensor->read(); });
    inst.setExtruderHeater(extruderHeater);
    inst.setHeaterBed(bedHeater);
    auto fan = std::make_shared<Fan>(2, [](double) {});
    inst.setFan(fan);
    auto probe = std::make_shared<Probe>(3, []() { return false; });
    inst.setProbe(probe);

    std::printf("Tether Klipper Printer Object Model\n");
    std::printf("====================================\n");

    // ── 1. Core Objects ────────────────────────────────────────────────
    printSection("1. Core Objects");
    auto& server = inst.server();
    auto coreStatus = server.queryObjects({
        {"toolhead", {}}, {"gcode_move", {}}, {"print_stats", {}},
        {"virtual_sdcard", {}}, {"display_status", {}}, {"webhooks", {}}
    });
    for (const auto& [name, fields] : coreStatus) {
        printStatus(name, fields);
    }

    // ── 2. Heater Objects ──────────────────────────────────────────────
    printSection("2. Heater Objects");
    auto heaterStatus = server.queryObjects({
        {"extruder", {}}, {"heater_bed", {}}, {"heaters", {}}
    });
    for (const auto& [name, fields] : heaterStatus) {
        printStatus(name, fields);
    }

    // ── 3. Fan Objects ─────────────────────────────────────────────────
    printSection("3. Fan Objects");
    auto fanStatus = server.queryObjects({{"fan", {}}});
    for (const auto& [name, fields] : fanStatus) {
        printStatus(name, fields);
    }

    // ── 4. Motion Objects ──────────────────────────────────────────────
    printSection("4. Motion Objects");
    auto motionStatus = server.queryObjects({
        {"motion_report", {}}, {"idle_timeout", {}}, {"stepper_enable", {}}
    });
    for (const auto& [name, fields] : motionStatus) {
        printStatus(name, fields);
    }

    // ── 5. Probe & Bed Leveling ────────────────────────────────────────
    printSection("5. Probe & Bed Leveling");
    auto probeStatus = server.queryObjects({
        {"probe", {}}, {"bed_mesh", {}}, {"bed_tilt", {}}
    });
    for (const auto& [name, fields] : probeStatus) {
        printStatus(name, fields);
    }

    // ── 6. Sensor Objects ──────────────────────────────────────────────
    printSection("6. Sensor Objects");
    // Register a temperature sensor
    inst.registerTemperatureSensor("temp_sensor_1", extruderSensor);
    auto sensorStatus = server.queryObjects({{"temperature_sensor temp_sensor_1", {}}});
    for (const auto& [name, fields] : sensorStatus) {
        printStatus(name, fields);
    }

    // ── 7. LED Objects ─────────────────────────────────────────────────
    printSection("7. LED Objects (standalone)");
    LedObject ledObj("my_led");
    ledObj.setColor({1.0, 0.5, 0.0, 0.0});
    printStatus("led my_led", ledObj.status({}));

    // ── 8. TMC Driver Objects ──────────────────────────────────────────
    printSection("8. TMC Driver Objects");
    TmcDriverObject tmcObj("tmc2208 stepper_x");
    printStatus("tmc2208 stepper_x", tmcObj.status({}));

    // ── 9. Advanced Objects ────────────────────────────────────────────
    printSection("9. Advanced Objects");
    InputShaperObject isObj;
    isObj.setShaperFreqX(45.0);
    isObj.setShaperFreqY(55.0);
    printStatus("input_shaper", isObj.status({}));

#if TETHER_ENABLE_PRESSURE_ADVANCE
    PressureAdvanceObject paObj;
    paObj.setPressureAdvance(0.05);
    paObj.setSmoothTime(0.040);
    printStatus("pressure_advance", paObj.status({}));
#endif

    SkewCorrectionObject skewObj;
    skewObj.setSkew(0.123, 0.0, 0.0);
    printStatus("skew_correction", skewObj.status({}));

    // ── 10. Exclude Object ─────────────────────────────────────────────
    printSection("10. Exclude Object");
    ExcludeObjectObject exclObj;
    exclObj.setExcludedObjects({"part1", "part3"});
    exclObj.setObjects({"part1", "part2", "part3"});
    printStatus("exclude_object", exclObj.status({}));

    // ── 11. Firmware Retraction ────────────────────────────────────────
    printSection("11. Firmware Retraction");
    auto fr = std::make_shared<FirmwareRetraction>();
    FirmwareRetractionParams frParams;
    frParams.retractLength = 3.0;
    frParams.retractSpeed = 35.0;
    frParams.unretractSpeed = 20.0;
    frParams.zHop = 0.2;
    fr->setParams(frParams);
    FirmwareRetractionObject frObj(fr);
    printStatus("firmware_retraction", frObj.status({}));

    // ── 12. Force Move ─────────────────────────────────────────────────
    printSection("12. Force Move");
    ForceMoveObject fmObj;
    fmObj.setEnableForceMove(true);
    printStatus("force_move", fmObj.status({}));

    // ── 13. Safe Z Home ────────────────────────────────────────────────
    printSection("13. Safe Z Home");
    SafeZHomeObject szhObj;
    szhObj.setHomeXyPosition("100, 100");
    szhObj.setZHop(10.0);
    szhObj.setZHopSpeed(20.0);
    printStatus("safe_z_home", szhObj.status({}));

    // ── 14. Multi Pin ──────────────────────────────────────────────────
    printSection("14. Multi Pin");
    MultiPinObject mpObj("my_multi_pin");
    mpObj.setPins({"pin_a", "pin_b", "pin_c"});
    mpObj.setValue("1");
    printStatus("multi_pin my_multi_pin", mpObj.status({}));

    // ── 15. New D2 Objects ─────────────────────────────────────────────
    printSection("15. New D2 Printer Objects");

    ManualProbeObject mpObj2;
    mpObj2.setActive(true);
    mpObj2.setZPosition(0.5);
    printStatus("manual_probe", mpObj2.status({}));

    FilamentMotionSensorObject fmsObj("filament_motion_sensor");
    fmsObj.setFilamentDetected(true);
    fmsObj.setDistance(150.0);
    printStatus("filament_motion_sensor", fmsObj.status({}));

    LoadCellObject lcObj("load_cell");
    lcObj.setLoad(42.5);
    lcObj.setTareValue(1.0);
    printStatus("load_cell", lcObj.status({}));

    CanbusStatsObject cbObj("canbus_stats");
    cbObj.setRxError(5);
    cbObj.setTxError(3);
    cbObj.setBusState("error-active");
    printStatus("canbus_stats", cbObj.status({}));

    PwmCycleTimeObject pctObj("pwm_cycle_time");
    pctObj.setValue(0.75);
    pctObj.setCycleTime(0.050);
    printStatus("pwm_cycle_time", pctObj.status({}));

    ResonanceTesterObject rtObj;
    rtObj.setMinFreq(10.0);
    rtObj.setMaxFreq(150.0);
    printStatus("resonance_tester", rtObj.status({}));

    AngleObject angleObj("angle");
    angleObj.setAngle(90.0);
    angleObj.setVelocity(100.0);
    printStatus("angle", angleObj.status({}));

    Palette2Object p2Obj;
    p2Obj.setConnected(true);
    p2Obj.setLoading(false);
    printStatus("palette2", p2Obj.status({}));

    MenuObject menuObj;
    menuObj.setEnabled(true);
    menuObj.setTimeout(60);
    printStatus("menu", menuObj.status({}));

    GcodeObject gcodeObj;
    gcodeObj.setCommands(100);
    gcodeObj.setInfo("Running print job");
    printStatus("gcode", gcodeObj.status({}));

    // ── Summary ────────────────────────────────────────────────────────
    printSection("Summary");
    auto allObjects = server.listObjects();
    std::printf("Total registered printer objects: %zu\n", allObjects.size());
    std::printf("\nObject categories:\n");
    std::printf("  Core:        toolhead, gcode_move, print_stats, virtual_sdcard, display_status\n");
    std::printf("  Heaters:     extruder, heater_bed, heaters, heater_generic, temperature_fan\n");
    std::printf("  Fans:        fan, controller_fan, heater_fan, fan_generic\n");
    std::printf("  Motion:      motion_report, idle_timeout, stepper_enable, mcu\n");
    std::printf("  Probing:     probe, bed_mesh, bed_tilt, z_tilt, quad_gantry_level\n");
    std::printf("  Sensors:     temperature_sensor, filament_switch_sensor, filament_motion_sensor\n");
    std::printf("  LEDs:        led, neopixel, dotstar\n");
    std::printf("  TMC:         tmc_driver, tmc_uart\n");
    std::printf("  Advanced:    input_shaper, pressure_advance, skew_correction\n");
    std::printf("  Misc:        exclude_object, firmware_retraction, force_move, safe_z_home\n");
    std::printf("  D2 Objects:  manual_probe, load_cell, canbus_stats, pwm_cycle_time\n");
    std::printf("               resonance_tester, angle, palette2, menu, gcode\n");

    std::printf("\nDone\n");
    return 0;
}
