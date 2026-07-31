/**
 * @file klipper_config_macros.cpp
 * @brief Example: config file parsing, config sections, and G-code macros.
 *
 * @details
 * This example demonstrates how Tether's Klipper layer handles configuration
 * files and G-code macros. It covers:
 *
 *   1. Creating a Klipper config file with multiple sections
 *   2. Loading the config into KlippyInstance
 *   3. G-code macros (gcode_macro sections)
 *   4. Delayed G-codes (delayed_gcode sections)
 *   5. Firmware retraction config (firmware_retraction section)
 *   6. Exclude object config (exclude_object section)
 *   7. Save variables config (save_variables section)
 *   8. Force move config (force_move section)
 *   9. Homing override (homing_override section)
 *  10. Endstop phase (endstop_phase section)
 *  11. Menu definitions (menu section)
 *  12. Palette2 config (palette2 section)
 *  13. Registering macros programmatically
 *  14. Executing macros with parameters
 *  15. Macro variable substitution
 *
 * This example is useful for understanding how config sections map to
 * internal state and how macros are defined and invoked.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace tether::klipper;
using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

static void execAndPrint(KlippyInstance& inst, const std::string& gcode) {
    std::printf("  > %s\n", gcode.c_str());
    inst.executeGcode(gcode);
}

static void printSection(const char* title) {
    std::printf("\n--- %s ---\n\n", title);
}

int main() {
    // Create a temporary config file
    std::string configDir = "/tmp/tether_config_macros";
    std::filesystem::create_directories(configDir);
    std::string configPath = configDir + "/printer.cfg";

    {
        std::ofstream f(configPath);
        f << R"(
# Tether Klipper Configuration Example

[stepper_x]
step_pin: gpio0
dir_pin: gpio1
enable_pin: gpio2
microsteps: 16
rotation_distance: 40
position_endstop: 0
position_max: 200
homing_speed: 50

[stepper_y]
step_pin: gpio3
dir_pin: gpio4
enable_pin: gpio5
microsteps: 16
rotation_distance: 40
position_endstop: 0
position_max: 200
homing_speed: 50

[stepper_z]
step_pin: gpio6
dir_pin: gpio7
enable_pin: gpio8
microsteps: 16
rotation_distance: 8
position_endstop: 0
position_max: 200

[extruder]
step_pin: gpio9
dir_pin: gpio10
enable_pin: gpio11
microsteps: 16
rotation_distance: 33.5
nozzle_diameter: 0.400
filament_diameter: 1.750
max_extrude_only_distance: 100.0
heater_pin: gpio12
sensor_type: EPCOS 100K B57560G104F
control: pid
pid_Kp: 26.0
pid_Ki: 1.8
pid_Kd: 90.0
min_temp: 0
max_temp: 250

[heater_bed]
heater_pin: gpio13
sensor_type: EPCOS 100K B57560G104F
control: pid
pid_Kp: 58.0
pid_Ki: 2.0
pid_Kd: 600.0
min_temp: 0
max_temp: 120

[fan]
pin: gpio14

[probe]
pin: gpio15
z_offset: 0.0
speed: 5.0

[bed_mesh]
speed: 50
horizontal_move_z: 5
mesh_min: 10, 10
mesh_max: 190, 190
probe_count: 3, 3

[gcode_macro START_PRINT]
description: Start a print job with proper warmup
gcode:
  G28
  M104 S200
  M140 S60
  M109 S200
  M190 S60
  G1 Z5 F3000
  G1 X0 Y0 F3000

[gcode_macro END_PRINT]
description: End a print job with cooldown
gcode:
  M104 S0
  M140 S0
  G1 X0 Y200 F3000
  M84

[gcode_macro PAUSE]
description: Pause the print
rename_existing: BASE_PAUSE
gcode:
  SAVE_GCODE_STATE NAME=pause_state
  G91
  G1 Z10 F3000
  G90
  G1 X0 Y0 F6000

[gcode_macro RESUME]
description: Resume the print
gcode:
  G91
  G1 Z-10 F3000
  G90
  RESTORE_GCODE_STATE NAME=pause_state

[gcode_macro CLEAN_NOZZLE]
description: Clean the nozzle before printing
gcode:
  G28
  G1 X50 Y0 Z10 F3000
  G1 X50 Y50 Z5 F1500
  G1 X50 Y0 Z5 F1500
  G1 X50 Y50 Z5 F1500
  G1 X0 Y0 Z10 F3000

[delayed_gcode delayed_print_start]
gcode: M117 Starting print...
initial_duration: 5.0

[firmware_retraction]
retract_length: 3.0
retract_speed: 35.0
unretract_extra_length: 0.0
unretract_speed: 20.0
z_hop: 0.2

[exclude_object]

[save_variables]

[force_move]
enable_force_move: true

[homing_override homing_override_x]
gcode: G28 X

[endstop_phase stepper_x]
endstop_align_tolerance: 0.05

[menu main_menu]
name: Main Menu

[palette2]
serial: /dev/ttyPAL

[display]
lcd_type: hd44780

[output_pin beeper]
pin: gpio16

[neopixel status_led]
pin: gpio17
chain_count: 16
color_order: GRB
initial_RED: 0.0
initial_GREEN: 0.0
initial_BLUE: 1.0
)";
        f.close();
    }

    std::printf("Tether Klipper Config & Macros Example\n");
    std::printf("=======================================\n");
    std::printf("Config file: %s\n", configPath.c_str());

    // Create KlippyInstance and load config
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = "/tmp/tether_config_macros_uds";
    cfg.sdcardDir = "/tmp/tether_config_macros_sd";
    cfg.configPath = configPath;
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

    // Load the config
    printSection("1. Load Config");
    bool loaded = inst.loadConfig(configPath);
    std::printf("Config loaded: %s\n", loaded ? "YES" : "NO");

    // ── 2. List registered macros ──────────────────────────────────────
    printSection("2. Registered Macros (from config)");
    auto& macros = inst.macros();
    auto allMacros = macros.listMacros();
    for (const auto& m : allMacros) {
        std::printf("  %s\n", m.c_str());
    }

    // ── 3. Execute macros ──────────────────────────────────────────────
    printSection("3. Execute Macros");
    execAndPrint(inst, "START_PRINT");
    execAndPrint(inst, "CLEAN_NOZZLE");
    execAndPrint(inst, "PAUSE");
    execAndPrint(inst, "RESUME");
    execAndPrint(inst, "END_PRINT");

    // ── 4. Register a macro programmatically ───────────────────────────
    printSection("4. Register Macro Programmatically");
    GcodeMacro customMacro;
    customMacro.name = "CALIBRATE_FLOW";
    customMacro.gcode = "M112\nG28\nG1 X50 Y50 Z5 F3000\nM112";
    customMacro.description = "Calibrate extruder flow rate";
    macros.registerMacro(customMacro);
    std::printf("  Registered: CALIBRATE_FLOW\n");
    execAndPrint(inst, "CALIBRATE_FLOW");

    // ── 5. Firmware retraction from config ─────────────────────────────
    printSection("5. Firmware Retraction (from config)");
    auto& fr = inst.firmwareRetraction();
    auto frParams = fr.params();
    std::printf("  Retract length: %.1f mm\n", frParams.retractLength);
    std::printf("  Retract speed: %.1f mm/s\n", frParams.retractSpeed);
    std::printf("  Unretract speed: %.1f mm/s\n", frParams.unretractSpeed);
    std::printf("  Z hop: %.1f mm\n", frParams.zHop);
    execAndPrint(inst, "G10");
    execAndPrint(inst, "G11");

    // ── 6. Force move (enabled in config) ──────────────────────────────
    printSection("6. Force Move (enabled in config)");
    execAndPrint(inst, "FORCE_MOVE STEPPER=stepper_x DISTANCE=10 VELOCITY=20 ACCEL=1000");

    // ── 7. Exclude object (enabled in config) ──────────────────────────
    printSection("7. Exclude Object (enabled in config)");
    execAndPrint(inst, "EXCLUDE_OBJECT_DEFINE NAME=part1 POLYGON=[[0,0],[50,0],[50,50],[0,50]]");
    execAndPrint(inst, "EXCLUDE_OBJECT_START NAME=part1");
    execAndPrint(inst, "EXCLUDE_OBJECT NAME=part1");
    execAndPrint(inst, "EXCLUDE_OBJECT_END NAME=part1");
    execAndPrint(inst, "EXCLUDE_OBJECT_RESET");

    // ── 8. Save variables ──────────────────────────────────────────────
    printSection("8. Save Variables");
    execAndPrint(inst, "SAVE_VARIABLE VARIABLE=my_var VALUE=42");

    // ── 9. Delayed G-code ──────────────────────────────────────────────
    printSection("9. Delayed G-code");
    execAndPrint(inst, "SET_DELAYED_GCODE ID=delayed_print_start GCODE=M117\ Starting...");

    // ── 10. Output pin ─────────────────────────────────────────────────
    printSection("10. Output Pin (beeper)");
    execAndPrint(inst, "SET_PIN PIN=beeper VALUE=1");
    execAndPrint(inst, "SET_PIN PIN=beeper VALUE=0");

    // ── 11. Neopixel ───────────────────────────────────────────────────
    printSection("11. Neopixel (status_led)");
    execAndPrint(inst, "SET_NEOPIXEL LED=status_led RED=0.0 GREEN=1.0 BLUE=0.0 INDEX=0");

    // ── 12. Set G-code variable ────────────────────────────────────────
    printSection("12. G-code Variables");
    execAndPrint(inst, "SET_GCODE_VARIABLE MACRO=START_PRINT VARIABLE=bed_temp VALUE=60");

    // ── Summary ────────────────────────────────────────────────────────
    printSection("Summary");
    std::printf("Config sections processed:\n");
    std::printf("  [stepper_x/y/z]  — stepper configuration\n");
    std::printf("  [extruder]       — extruder/heater config\n");
    std::printf("  [heater_bed]     — bed heater config\n");
    std::printf("  [fan]            — fan config\n");
    std::printf("  [probe]          — probe config\n");
    std::printf("  [bed_mesh]       — bed mesh config\n");
    std::printf("  [gcode_macro *]  — 5 macros defined\n");
    std::printf("  [delayed_gcode]  — delayed G-code timer\n");
    std::printf("  [firmware_retraction] — retraction params\n");
    std::printf("  [exclude_object] — exclude object enabled\n");
    std::printf("  [save_variables] — save variables enabled\n");
    std::printf("  [force_move]     — force move enabled\n");
    std::printf("  [homing_override]— homing override G-code\n");
    std::printf("  [endstop_phase]  — endstop phase config\n");
    std::printf("  [menu]           — menu definition\n");
    std::printf("  [palette2]       — Palette2 connected\n");
    std::printf("  [display]        — display config\n");
    std::printf("  [output_pin]     — output pin config\n");
    std::printf("  [neopixel]       — neopixel LED strip\n");

    // Cleanup
    std::filesystem::remove_all(configDir);

    std::printf("\nDone\n");
    return 0;
}
