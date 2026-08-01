/**
 * @file KlippyUdsHelpText.hpp
 * @brief G-code help text table for the gcode/help UDS endpoint.
 *
 * @details
 * Extracted from KlippyUdsServerGcodeEndpoints.cpp to avoid rebuilding
 * the help map on every request and to centralise the help text in one
 * place. The table is a static constexpr array of {code, description}
 * pairs; getGcodeHelp() builds the JsonValue map lazily on first call
 * and caches it.
 */

#pragma once

#include "tether/klipper/klippy/JsonValue.hpp"

#include <array>
#include <map>
#include <string>

namespace tether::klipper::klippy {

/// @brief G-code help text entry.
struct GcodeHelpEntry {
    const char* code;
    const char* description;
};

/// @brief Static table of G-code help text.
inline constexpr std::array<GcodeHelpEntry, 85> GcodeHelpTable = {{
    // Motion
    {"G0",   "Rapid move"},
    {"G1",   "Linear move"},
    {"G2",   "Arc move clockwise"},
    {"G3",   "Arc move counter-clockwise"},
    {"G4",   "Dwell"},
    {"G5",   "Bezier spline move"},
    {"G12",  "Clean nozzle"},
    {"G17",  "XY plane select"},
    {"G18",  "XZ plane select"},
    {"G19",  "YZ plane select"},
    {"G20",  "Set units to inches"},
    {"G21",  "Set units to millimeters"},
    {"G28",  "Home axes"},
    {"G29",  "Bed mesh leveling"},
    {"G30",  "Probe"},
    {"G38",  "Probe toward target"},
    {"G60",  "Save position"},
    {"G61",  "Restore position"},
    {"G90",  "Absolute coordinates"},
    {"G91",  "Relative coordinates"},
    {"G92",  "Set position"},
    // Extrusion
    {"G10",  "Firmware retract"},
    {"G11",  "Firmware unretract"},
    {"M82",  "Absolute extrusion"},
    {"M83",  "Relative extrusion"},
    // Temperature
    {"M104", "Set hotend temperature"},
    {"M105", "Get temperatures"},
    {"M109", "Wait for hotend temperature"},
    {"M140", "Set bed temperature"},
    {"M190", "Wait for bed temperature"},
    {"M155", "Auto temperature reporting"},
    // Fan
    {"M106", "Set fan speed"},
    {"M107", "Fan off"},
    // Motors
    {"M17",  "Enable motors"},
    {"M18",  "Disable motors"},
    {"M84",  "Disable motors"},
    // SD card
    {"M20",  "List SD files"},
    {"M23",  "Select SD file"},
    {"M24",  "Start/resume SD print"},
    {"M25",  "Pause SD print"},
    {"M27",  "Report SD status"},
    // Display
    {"M73",  "Set display progress"},
    {"M117", "Set display message"},
    {"M118", "Output message"},
    // Status
    {"M114", "Get current position"},
    {"M119", "Get endstop status"},
    // Sync
    {"M400", "Wait for moves to finish"},
    // Overrides
    {"M220", "Set speed factor"},
    {"M221", "Set extrude factor"},
    // Advanced motion
    {"M205", "Advanced motion settings"},
    {"M900", "Set pressure advance"},
    {"M593", "Set input shaper"},
    // Settings
    {"M500", "Save settings"},
    {"M501", "Load settings"},
    {"M502", "Reset to factory defaults"},
    {"M503", "Report current settings"},
    // Emergency
    {"M112", "Emergency stop"},
    // Firmware info
    {"M115", "Get firmware version"},
    {"M116", "Wait for all temperatures"},
    // Stepper config
    {"M92",  "Set steps per mm"},
    {"M350", "Set microstepping"},
    {"M569", "Set stepper direction"},
    {"M906", "Set stepper driver current"},
    // Motion limits
    {"M200", "Set filament diameter"},
    {"M201", "Set print acceleration"},
    {"M203", "Set max feedrate"},
    {"M204", "Set acceleration"},
    // Offsets
    {"M206", "Set home offset"},
    {"M218", "Set tool offset"},
    {"M851", "Set probe Z offset"},
    // Retract
    {"M207", "Set retract parameters"},
    {"M208", "Set unretract parameters"},
    // PID
    {"M301", "Set hotend PID"},
    {"M303", "PID autotune"},
    {"M304", "Set bed PID"},
    // Probe
    {"M401", "Deploy probe"},
    {"M402", "Stow probe"},
    // Bed mesh
    {"M420", "Enable/disable bed mesh"},
    {"M421", "Set bed mesh point"},
    // Backlash
    {"M425", "Set backlash compensation"},
    // Misc
    {"M42",  "Set pin state"},
    {"M150", "Set LED color"},
    {"M280", "Servo control"},
    {"M300", "Beep"},
    {"M600", "Filament change"},
}};

/// @brief Build the G-code help JSON map from the static table.
inline JsonValue getGcodeHelpJson() {
    std::map<std::string, JsonValue> result;
    for (const auto& entry : GcodeHelpTable) {
        result[entry.code] = JsonValue(entry.description);
    }
    return JsonValue(result);
}

} // namespace tether::klipper::klippy
