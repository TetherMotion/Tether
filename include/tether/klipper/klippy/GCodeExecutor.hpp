#pragma once

/// @file GCodeExecutor.hpp
/// @brief G-code executor that dispatches commands to callbacks.
///
/// This file includes the parser, state, callbacks, and macro sub-components
/// and defines the main GCodeExecutor class.
///
/// @par Feature limitations
/// The Klipper G-code executor is a lightweight parser for 3D printer
/// control. It does **not** support the following features that the main
/// Tether RS274/NGC interpreter (tether::gcode::GCodeInterpreter) provides:
///
/// - O-code control flow (subroutines, if/else, while, repeat)
/// - # parameters and expression evaluation
/// - Tool compensation (G40-G42, G43-G49)
/// - Full canned cycle semantics (G73-G89 with rigid tapping)
/// - NURBS splines (G5.1, G5.2/G5.3)
/// - Execution modes (MDI, STEP, VERIFY)
///
/// Coordinate system features that ARE supported:
/// - Work coordinate systems G54-G59.3 (selection + offsets)
/// - G52 local coordinate offset
/// - G68/G69 coordinate rotation (2D plane + 3D Euler + axis-angle)
/// - G51/G50 per-axis and uniform scaling
/// - G53 machine coordinates (non-modal bypass)
/// - G92 position offset and G92.1/G92.2/G92.3 reset
///
/// Conversely, it provides 3D-printer-specific features not in the main
/// interpreter: temperature control, bed mesh leveling, input shaping,
/// TMC driver configuration, LED control, and 50+ Klipper extended
/// commands. See docs/KlipperGcodeCommands.md for the full reference.

#include "tether/klipper/klippy/GCodeParser.hpp"
#include "tether/klipper/klippy/PrinterMotionState.hpp"
#include "tether/klipper/klippy/GCodeCallbacks.hpp"
#include "tether/klipper/klippy/GCodeMacro.hpp"

#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/Thermal.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace tether::klipper::klippy {

// Forward declarations
class FirmwareRetraction;
class VirtualSdcard;

// ============================================================================
// G-code executor
// ============================================================================

/// @brief Executes G-code commands by dispatching to callbacks.
class GCodeExecutor {
public:
    GCodeExecutor(GcodeCallbacks callbacks, PrinterMotionState* state = nullptr)
        : callbacks_(std::move(callbacks))
        , state_(state ? state : &internalState_) {}

    /// @brief Set the macro registry for macro expansion.
    void setMacroRegistry(GcodeMacroRegistry* registry) { macroRegistry_ = registry; }

    /// @brief Execute a G-code script (possibly multiple lines).
    /// @param script Multi-line G-code string.
    /// @return True if all commands executed successfully.
    bool execute(const std::string& script) {
        bool success = true;
        size_t pos = 0;
        while (pos < script.size()) {
            auto newline = script.find('\n', pos);
            std::string line;
            if (newline == std::string::npos) {
                line = script.substr(pos);
                pos = script.size();
            } else {
                line = script.substr(pos, newline - pos);
                pos = newline + 1;
            }
            if (!executeLine(line)) {
                success = false;
            }
        }
        return success;
    }

    /// @brief Execute a single G-code line.
    bool executeLine(const std::string& line) {
        auto parsed = parseGcodeLine(line);
        if (!parsed) return true; // Empty/comment lines are OK

        auto& g = *parsed;

        // Dispatch based on code
        if (g.code == "G0" || g.code == "G1") {
            return executeMove(g);
        } else if (g.code == "G28") {
            return executeHome(g);
        } else if (g.code == "G30") {
            return executeProbe(g);
        } else if (g.code == "G92") {
            return executeSetPosition(g);
        } else if (g.code == "G4") {
            return executeDwell(g);
        } else if (g.code == "G90") {
            state_->distanceMode = GCode::DistanceMode::ABSOLUTE;
            return true;
        } else if (g.code == "G91") {
            state_->distanceMode = GCode::DistanceMode::INCREMENTAL;
            return true;
        } else if (g.code == "M82") {
            state_->absoluteExtrude = true;
            return true;
        } else if (g.code == "M83") {
            state_->absoluteExtrude = false;
            return true;
        } else if (g.code == "M104") {
            return executeSetHotendTemp(g, false);
        } else if (g.code == "M109") {
            return executeSetHotendTemp(g, true);
        } else if (g.code == "M140") {
            return executeSetBedTemp(g, false);
        } else if (g.code == "M190") {
            return executeSetBedTemp(g, true);
        } else if (g.code == "M106") {
            if (callbacks_.setFanSpeed) {
                callbacks_.setFanSpeed(g.get('S', 255.0) / 255.0);
            }
            return true;
        } else if (g.code == "M107") {
            if (callbacks_.setFanSpeed) {
                callbacks_.setFanSpeed(0.0);
            }
            return true;
        } else if (g.code == "M17") {
            if (callbacks_.setMotorEnable) {
                callbacks_.setMotorEnable("", true);
            }
            return true;
        } else if (g.code == "M18" || g.code == "M84") {
            if (callbacks_.setMotorEnable) {
                callbacks_.setMotorEnable("", false);
            }
            return true;
        } else if (g.code == "M112") {
            if (callbacks_.emergencyStop) {
                callbacks_.emergencyStop();
            }
            return true;
        } else if (g.code == "M118") {
            if (callbacks_.output && !g.comment.empty()) {
                callbacks_.output(g.comment);
            }
            return true;
        } else if (g.code == "M117") {
            // M117: Set display message (the message is the text after M117)
            if (callbacks_.setDisplayMessage) {
                callbacks_.setDisplayMessage(g.text.empty() ? g.comment : g.text);
            }
            return true;
        } else if (g.code == "M73") {
            // M73: Set/get progress
            if (callbacks_.setDisplayProgress) {
                callbacks_.setDisplayProgress(g.get('P', 0.0) / 100.0);
            }
            return true;
        } else if (g.code == "M220") {
            state_->speedFactor = g.get('S', 100.0) / 100.0;
            return true;
        } else if (g.code == "M221") {
            state_->extrudeFactor = g.get('S', 100.0) / 100.0;
            return true;
        } else if (g.code == "G10") {
            // G10 has multiple meanings depending on L word:
            // - No L word: firmware retract
            // - L2: set WCS offset (absolute)
            // - L20: set WCS offset from current position
            if (g.has('L')) {
                int lVal = static_cast<int>(g.get('L'));
                int pWord = static_cast<int>(g.get('P', 1));
                if (pWord < 1 || pWord > 9)
                    return false;
                int wcsIdx = pWord - 1;

                if (lVal == 2) {
                    // G10 L2: set WCS offset (absolute)
                    if (g.has('X'))
                        state_->coordSystemOffsets[wcsIdx][0] = g.get('X');
                    if (g.has('Y'))
                        state_->coordSystemOffsets[wcsIdx][1] = g.get('Y');
                    if (g.has('Z'))
                        state_->coordSystemOffsets[wcsIdx][2] = g.get('Z');
                    if (g.has('R'))
                        state_->coordSystemRotations[wcsIdx] = g.get('R');
                    state_->rebuildCoordTransform();
                    return true;
                } else if (lVal == 20) {
                    // G10 L20: set WCS so current position becomes specified
                    // coordinate. offset = machine - program.
                    auto machine = state_->coordTransform.toMachineXYZ(
                        state_->position[0], state_->position[1],
                        state_->position[2]);
                    if (g.has('X'))
                        state_->coordSystemOffsets[wcsIdx][0] =
                            machine[0] - g.get('X');
                    if (g.has('Y'))
                        state_->coordSystemOffsets[wcsIdx][1] =
                            machine[1] - g.get('Y');
                    if (g.has('Z'))
                        state_->coordSystemOffsets[wcsIdx][2] =
                            machine[2] - g.get('Z');
                    if (g.has('R'))
                        state_->coordSystemRotations[wcsIdx] = g.get('R');
                    state_->rebuildCoordTransform();
                    return true;
                }
            }

            // Firmware retract (no L word)
            if (callbacks_.retract) {
                double eMove = callbacks_.retract();
                if (!std::isnan(eMove)) {
                    state_->position[3] += eMove;
                    if (callbacks_.move) {
                        auto machine = state_->coordTransform.toMachineXYZ(
                            state_->position[0], state_->position[1],
                            state_->position[2]);
                        callbacks_.move(
                            machine[0], machine[1], machine[2],
                            state_->position[3], 20.0);
                    }
                }
            }
            return true;
        } else if (g.code == "G11") {
            // Firmware unretract
            if (callbacks_.unretract) {
                double eMove = callbacks_.unretract();
                if (!std::isnan(eMove)) {
                    state_->position[3] += eMove;
                    if (callbacks_.move) {
                        auto machine = state_->coordTransform.toMachineXYZ(
                            state_->position[0], state_->position[1],
                            state_->position[2]);
                        callbacks_.move(
                            machine[0], machine[1], machine[2],
                            state_->position[3], 10.0);
                    }
                }
            }
            return true;
        } else if (g.code == "G38") {
            // G38.2/G38.3: Probe toward target
            return executeProbe(g);
        } else if (g.code == "M23") {
            // Select SD file — filename is in the text or comment
            if (callbacks_.selectSdFile) {
                std::string filename = g.text.empty() ? g.comment : g.text;
                if (!filename.empty()) {
                    // Trim leading whitespace
                    auto start = filename.find_first_not_of(" \t");
                    if (start != std::string::npos) {
                        filename = filename.substr(start);
                    }
                    callbacks_.selectSdFile(filename);
                }
            }
            return true;
        } else if (g.code == "M24") {
            // Start/resume SD print
            if (callbacks_.startSdPrint) {
                callbacks_.startSdPrint();
            }
            return true;
        } else if (g.code == "M25") {
            // Pause SD print
            if (callbacks_.pauseSdPrint) {
                callbacks_.pauseSdPrint();
            }
            return true;
        } else if (g.code == "M27") {
            // Report SD status
            if (callbacks_.sdStatus) {
                std::string status = callbacks_.sdStatus();
                if (callbacks_.output) {
                    callbacks_.output(status);
                }
            }
            return true;
        } else if (g.code == "M20") {
            // List SD files — handled by custom callback
            if (callbacks_.custom) {
                callbacks_.custom(g);
            }
            return true;
        } else if (g.code == "G60") {
            // Save current position to slot
            if (callbacks_.savePosition) {
                callbacks_.savePosition(static_cast<int>(g.get('S', 0)));
            }
            return true;
        } else if (g.code == "G61") {
            // G61 can be either restore position (with S param) or exact stop (no param)
            if (g.has('S')) {
                if (callbacks_.restorePosition) {
                    callbacks_.restorePosition(static_cast<int>(g.get('S', 0)));
                }
            } else {
                // G61 without S = exact stop path control mode
                state_->pathControlMode = 0;
                if (callbacks_.setPathControl) callbacks_.setPathControl(0, 0.0);
            }
            return true;
        } else if (g.code == "G20") {
            // Set units to inches
            state_->units = GCode::Units::INCH;
            return true;
        } else if (g.code == "G21") {
            // Set units to millimeters (default)
            state_->units = GCode::Units::MM;
            return true;
        } else if (g.code == "G29") {
            // Bed mesh leveling — probe bed and fill mesh
            if (callbacks_.probeBed) {
                int n = callbacks_.probeBed();
                if (callbacks_.output) {
                    callbacks_.output("Bed probing complete: " +
                        std::to_string(n > 0 ? n : 0) + " points probed");
                }
            } else if (callbacks_.bedLevel) {
                callbacks_.bedLevel();
            }
            return true;
        } else if (g.code == "G32") {
            // Auto bed leveling — probe and apply correction
            if (callbacks_.autoBedLevel) {
                int n = callbacks_.autoBedLevel();
                if (callbacks_.output) {
                    callbacks_.output("Auto bed leveling complete: " +
                        std::to_string(n > 0 ? n : 0) + " points probed");
                }
            } else if (callbacks_.probeBed) {
                callbacks_.probeBed();
            }
            return true;
        } else if (g.code == "G33") {
            // Delta calibration
            if (callbacks_.deltaCalibrate) {
                int n = callbacks_.deltaCalibrate();
                if (callbacks_.output) {
                    callbacks_.output("Delta calibration complete: " +
                        std::to_string(n > 0 ? n : 0) + " iterations");
                }
            }
            return true;
        } else if (g.code == "G34") {
            // Z tilt leveling
            if (callbacks_.zTiltLevel) {
                bool ok = callbacks_.zTiltLevel();
                if (callbacks_.output) {
                    callbacks_.output(ok ? "Z tilt leveling complete" :
                                         "Z tilt leveling failed");
                }
            }
            return true;
        } else if (g.code == "G2" || g.code == "G3") {
            // Arc move — clockwise (G2) or counter-clockwise (G3)
            return executeArcMove(g);
        } else if (g.code == "G17") {
            state_->plane = GCode::Plane::XY;
            return true;
        } else if (g.code == "G18") {
            state_->plane = GCode::Plane::ZX;
            return true;
        } else if (g.code == "G19") {
            state_->plane = GCode::Plane::YZ;
            return true;
        } else if (g.code == "G12") {
            // Clean nozzle
            if (callbacks_.cleanNozzle) {
                callbacks_.cleanNozzle(g.get('P', 3.0), g.get('R', 0.0), g.get('S', 0.0));
            }
            return true;
        } else if (g.code == "M114") {
            // Get current position
            if (callbacks_.getPositionStatus) {
                std::string status = callbacks_.getPositionStatus();
                if (callbacks_.output) callbacks_.output(status);
            }
            return true;
        } else if (g.code == "M119") {
            // Get endstop status
            if (callbacks_.getEndstopStatus) {
                std::string status = callbacks_.getEndstopStatus();
                if (callbacks_.output) callbacks_.output(status);
            }
            return true;
        } else if (g.code == "M105") {
            // Get temperatures
            if (callbacks_.getTempStatus) {
                std::string status = callbacks_.getTempStatus();
                if (callbacks_.output) callbacks_.output(status);
            }
            return true;
        } else if (g.code == "M155") {
            // Auto temperature reporting
            if (callbacks_.setAutoTempReport) {
                callbacks_.setAutoTempReport(g.get('S', 0.0));
            }
            return true;
        } else if (g.code == "M400") {
            // Wait for current moves to finish
            if (callbacks_.waitForMoves) {
                callbacks_.waitForMoves();
            }
            return true;
        } else if (g.code == "M900") {
            // Set pressure advance
            if (callbacks_.setPressureAdvance) {
                int extruder = static_cast<int>(g.get('T', 0));
                double pa = g.get('K', g.get('S', 0.0));
                callbacks_.setPressureAdvance(extruder, pa);
            }
            return true;
        } else if (g.code == "M593") {
            // Set input shaper
            if (callbacks_.setInputShaperParams) {
                double freq = g.get('F', g.get('S', 0.0));
                std::string axis = g.has('X') ? "x" : (g.has('Y') ? "y" : "");
                std::string type;
                if (g.has('S')) {
                    int s = static_cast<int>(g.get('S'));
                    switch (s) {
                        case 0: type = "none"; break;
                        case 1: type = "ZV"; break;
                        case 2: type = "ZVD"; break;
                        case 3: type = "MZV"; break;
                        case 4: type = "EI"; break;
                        case 5: type = "damped_ei"; break;
                        default: type = "none"; break;
                    }
                }
                callbacks_.setInputShaperParams(axis, freq, type);
            }
            return true;
        } else if (g.code == "M500") {
            if (callbacks_.saveSettings) callbacks_.saveSettings();
            return true;
        } else if (g.code == "M501") {
            if (callbacks_.loadSettings) callbacks_.loadSettings();
            return true;
        } else if (g.code == "M502") {
            if (callbacks_.resetSettings) callbacks_.resetSettings();
            return true;
        } else if (g.code == "M503") {
            if (callbacks_.reportSettings) {
                std::string settings = callbacks_.reportSettings();
                if (callbacks_.output) callbacks_.output(settings);
            }
            return true;
        } else if (g.code == "M115") {
            // Get firmware version
            if (callbacks_.getFirmwareInfo) {
                std::string info = callbacks_.getFirmwareInfo();
                if (callbacks_.output) callbacks_.output(info);
            }
            return true;
        } else if (g.code == "M116") {
            // Wait for all temperatures to reach target
            if (callbacks_.waitForTemperatures) callbacks_.waitForTemperatures();
            return true;
        } else if (g.code == "M92") {
            // Set steps per mm
            if (callbacks_.setStepsPerMm) {
                if (g.has('X')) callbacks_.setStepsPerMm("x", g.get('X'));
                if (g.has('Y')) callbacks_.setStepsPerMm("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setStepsPerMm("z", g.get('Z'));
                if (g.has('E')) callbacks_.setStepsPerMm("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M200") {
            // Set filament diameter
            if (callbacks_.setFilamentDiameter) {
                callbacks_.setFilamentDiameter(g.get('D', 1.75));
            }
            return true;
        } else if (g.code == "M201") {
            // Set print acceleration
            if (callbacks_.setAcceleration) {
                callbacks_.setAcceleration(g.get('P', g.get('S', 0.0)), g.get('T', 0.0));
            }
            return true;
        } else if (g.code == "M203") {
            // Set max feedrate
            if (callbacks_.setMaxFeedrate) {
                if (g.has('X')) callbacks_.setMaxFeedrate("x", g.get('X'));
                if (g.has('Y')) callbacks_.setMaxFeedrate("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setMaxFeedrate("z", g.get('Z'));
                if (g.has('E')) callbacks_.setMaxFeedrate("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M204") {
            // Set acceleration
            if (callbacks_.setAcceleration) {
                double accel = g.get('P', g.get('S', 0.0));
                double travelAccel = g.get('T', accel);
                callbacks_.setAcceleration(accel, travelAccel);
            }
            return true;
        } else if (g.code == "M205") {
            // Advanced motion settings
            if (callbacks_.setAdvancedMotion) {
                callbacks_.setAdvancedMotion(g.get('X', 0.0), g.get('S', 0.0));
            }
            return true;
        } else if (g.code == "M206") {
            // Set home offset
            if (callbacks_.setHomeOffset) {
                if (g.has('X')) callbacks_.setHomeOffset("x", g.get('X'));
                if (g.has('Y')) callbacks_.setHomeOffset("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setHomeOffset("z", g.get('Z'));
            }
            return true;
        } else if (g.code == "M207") {
            // Set retract parameters
            if (callbacks_.setRetractParams) {
                callbacks_.setRetractParams(g.get('S', 0.0), g.get('F', 0.0), g.get('Z', 0.0));
            }
            return true;
        } else if (g.code == "M208") {
            // M208 can be either unretract params (S, F) or software endstops (X/Y/Z, S, P)
            if (g.has('X') || g.has('Y') || g.has('Z')) {
                // Software endstops mode
                std::string axis;
                if (g.has('X')) axis = "x";
                else if (g.has('Y')) axis = "y";
                else if (g.has('Z')) axis = "z";
                double mn = g.get('S', 0.0);
                double mx = g.get('P', 0.0);
                bool enable = g.get('S', 1.0) != 0;
                if (callbacks_.setSoftwareEndstops) callbacks_.setSoftwareEndstops(axis, mn, mx, enable);
            } else {
                // Unretract parameters
                if (callbacks_.setUnretractParams) {
                    callbacks_.setUnretractParams(g.get('S', 0.0), g.get('F', 0.0));
                }
            }
            return true;
        } else if (g.code == "M218") {
            // Set tool offset
            if (callbacks_.setToolOffset) {
                int tool = static_cast<int>(g.get('T', 0));
                if (g.has('X')) callbacks_.setToolOffset(tool, "x", g.get('X'));
                if (g.has('Y')) callbacks_.setToolOffset(tool, "y", g.get('Y'));
                if (g.has('Z')) callbacks_.setToolOffset(tool, "z", g.get('Z'));
            }
            return true;
        } else if (g.code == "M280") {
            // Servo control
            if (callbacks_.setServoAngle) {
                callbacks_.setServoAngle(static_cast<int>(g.get('P', 0)), g.get('S', 0.0));
            }
            return true;
        } else if (g.code == "M300") {
            // Beep
            if (callbacks_.beep) {
                callbacks_.beep(g.get('S', 1000.0), g.get('P', 100.0));
            }
            return true;
        } else if (g.code == "M301") {
            // Set hotend PID
            if (callbacks_.setHotendPid) {
                callbacks_.setHotendPid(g.get('P', 0.0), g.get('I', 0.0), g.get('D', 0.0));
            }
            return true;
        } else if (g.code == "M303") {
            // PID autotune
            if (callbacks_.runPidAutotune) {
                std::string result = callbacks_.runPidAutotune(g.get('S', 200.0),
                    static_cast<int>(g.get('C', 5)));
                if (callbacks_.output) callbacks_.output(result);
            }
            return true;
        } else if (g.code == "M304") {
            // Set bed PID
            if (callbacks_.setBedPid) {
                callbacks_.setBedPid(g.get('P', 0.0), g.get('I', 0.0), g.get('D', 0.0));
            }
            return true;
        } else if (g.code == "M350") {
            // Set microstepping
            if (callbacks_.setMicrostepping) {
                int ms = static_cast<int>(g.get('S', 16));
                if (g.has('X')) callbacks_.setMicrostepping("x", ms);
                if (g.has('Y')) callbacks_.setMicrostepping("y", ms);
                if (g.has('Z')) callbacks_.setMicrostepping("z", ms);
                if (g.has('E')) callbacks_.setMicrostepping("e", ms);
            }
            return true;
        } else if (g.code == "M401") {
            // Deploy probe
            if (callbacks_.deployProbe) callbacks_.deployProbe();
            return true;
        } else if (g.code == "M402") {
            // Stow probe
            if (callbacks_.stowProbe) callbacks_.stowProbe();
            return true;
        } else if (g.code == "M420") {
            // Enable/disable bed mesh
            if (callbacks_.setBedMeshEnabled) {
                callbacks_.setBedMeshEnabled(static_cast<int>(g.get('S', 1)) != 0);
            }
            return true;
        } else if (g.code == "M421") {
            // Set bed mesh point
            if (callbacks_.setBedMeshPoint) {
                int xIdx = static_cast<int>(g.get('I', 0));
                int yIdx = static_cast<int>(g.get('J', 0));
                double z = g.get('Z', 0.0);
                callbacks_.setBedMeshPoint(xIdx, yIdx, z);
            }
            return true;
        } else if (g.code == "M425") {
            // Backlash compensation
            if (callbacks_.setBacklash) {
                if (g.has('X')) callbacks_.setBacklash("x", g.get('X'));
                if (g.has('Y')) callbacks_.setBacklash("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setBacklash("z", g.get('Z'));
            }
            return true;
        } else if (g.code == "M42") {
            // Set pin state
            if (callbacks_.setPinState) {
                callbacks_.setPinState(static_cast<int>(g.get('P', 0)), g.get('S', 0.0));
            }
            return true;
        } else if (g.code == "M150") {
            // Set LED color
            if (callbacks_.setLedColor) {
                callbacks_.setLedColor(static_cast<int>(g.get('R', 0)),
                    static_cast<int>(g.get('G', 0)),
                    static_cast<int>(g.get('B', 0)),
                    static_cast<int>(g.get('W', 0)));
            }
            return true;
        } else if (g.code == "M569") {
            // Set stepper direction
            if (callbacks_.setStepperDirection) {
                int dir = static_cast<int>(g.get('S', 0));
                if (g.has('X')) callbacks_.setStepperDirection("x", dir);
                if (g.has('Y')) callbacks_.setStepperDirection("y", dir);
                if (g.has('Z')) callbacks_.setStepperDirection("z", dir);
                if (g.has('E')) callbacks_.setStepperDirection("e", dir);
            }
            return true;
        } else if (g.code == "M600") {
            // Filament change
            if (callbacks_.filamentChange) callbacks_.filamentChange();
            return true;
        } else if (g.code == "M851") {
            // Set probe Z offset
            if (callbacks_.setProbeOffset) {
                callbacks_.setProbeOffset(g.get('Z', 0.0));
            }
            return true;
        } else if (g.code == "M906") {
            // Set stepper driver current
            if (callbacks_.setStepperCurrent) {
                double current = g.get('T', g.get('S', 0.0));
                if (g.has('X')) callbacks_.setStepperCurrent("x", g.get('X'));
                if (g.has('Y')) callbacks_.setStepperCurrent("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setStepperCurrent("z", g.get('Z'));
                if (g.has('E')) callbacks_.setStepperCurrent("e", g.get('E'));
                (void)current;
            }
            return true;
        } else if (g.code == "M907") {
            // Set TMC driver current
            if (callbacks_.setTmcCurrent) {
                if (g.has('X')) callbacks_.setTmcCurrent("x", g.get('X'));
                if (g.has('Y')) callbacks_.setTmcCurrent("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setTmcCurrent("z", g.get('Z'));
                if (g.has('E')) callbacks_.setTmcCurrent("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M908") {
            // Set TMC driver run current
            if (callbacks_.setTmcRunCurrent) {
                if (g.has('X')) callbacks_.setTmcRunCurrent("x", g.get('X'));
                if (g.has('Y')) callbacks_.setTmcRunCurrent("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setTmcRunCurrent("z", g.get('Z'));
                if (g.has('E')) callbacks_.setTmcRunCurrent("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M909") {
            // Set TMC driver hold current
            if (callbacks_.setTmcHoldCurrent) {
                if (g.has('X')) callbacks_.setTmcHoldCurrent("x", g.get('X'));
                if (g.has('Y')) callbacks_.setTmcHoldCurrent("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setTmcHoldCurrent("z", g.get('Z'));
                if (g.has('E')) callbacks_.setTmcHoldCurrent("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M911") {
            // Set TMC stealthChop mode
            if (callbacks_.setTmcStealthChop) {
                bool enable = (g.get('S', 0.0) != 0);
                if (g.has('X')) callbacks_.setTmcStealthChop("x", enable);
                if (g.has('Y')) callbacks_.setTmcStealthChop("y", enable);
                if (g.has('Z')) callbacks_.setTmcStealthChop("z", enable);
                if (g.has('E')) callbacks_.setTmcStealthChop("e", enable);
            }
            return true;
        } else if (g.code == "M912") {
            // Set TMC spreadCycle threshold
            if (callbacks_.setTmcSpreadThreshold) {
                if (g.has('X')) callbacks_.setTmcSpreadThreshold("x", g.get('X'));
                if (g.has('Y')) callbacks_.setTmcSpreadThreshold("y", g.get('Y'));
                if (g.has('Z')) callbacks_.setTmcSpreadThreshold("z", g.get('Z'));
                if (g.has('E')) callbacks_.setTmcSpreadThreshold("e", g.get('E'));
            }
            return true;
        } else if (g.code == "M913") {
            // Set TMC bump sensitivity
            if (callbacks_.setTmcBumpSensitivity) {
                int sens = static_cast<int>(g.get('S', 0.0));
                if (g.has('X')) callbacks_.setTmcBumpSensitivity("x", sens);
                if (g.has('Y')) callbacks_.setTmcBumpSensitivity("y", sens);
                if (g.has('Z')) callbacks_.setTmcBumpSensitivity("z", sens);
                if (g.has('E')) callbacks_.setTmcBumpSensitivity("e", sens);
            }
            return true;
        } else if (g.code == "M914") {
            // Set TMC diag pin
            if (callbacks_.setTmcDiagPin) {
                int diag = static_cast<int>(g.get('S', 0.0));
                if (g.has('X')) callbacks_.setTmcDiagPin("x", diag);
                if (g.has('Y')) callbacks_.setTmcDiagPin("y", diag);
                if (g.has('Z')) callbacks_.setTmcDiagPin("z", diag);
                if (g.has('E')) callbacks_.setTmcDiagPin("e", diag);
            }
            return true;
        } else if (g.code == "M665") {
            // Set delta geometry parameters
            if (callbacks_.setDeltaGeometry) {
                callbacks_.setDeltaGeometry(
                    g.get('L', 0.0),  // Arm length
                    g.get('R', 0.0),  // Delta radius
                    g.get('A', 0.0),  // Tower A angle
                    g.get('B', 0.0),  // Tower B angle
                    g.get('C', 0.0)); // Tower C angle
            }
            return true;
        } else if (g.code == "M666") {
            // Set delta endstop adjustments
            if (callbacks_.setDeltaEndstopAdjust) {
                callbacks_.setDeltaEndstopAdjust(
                    g.get('X', 0.0), g.get('Y', 0.0), g.get('Z', 0.0));
            }
            return true;
        } else if (g.code == "M701") {
            // Load filament
            if (callbacks_.loadFilament) {
                callbacks_.loadFilament(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M702") {
            // Unload filament
            if (callbacks_.unloadFilament) {
                callbacks_.unloadFilament(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M703") {
            // Load filament into tool
            if (callbacks_.loadFilamentToTool) {
                callbacks_.loadFilamentToTool(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M704") {
            // Unload filament from tool
            if (callbacks_.unloadFilamentFromTool) {
                callbacks_.unloadFilamentFromTool(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M705") {
            // Purge filament
            if (callbacks_.purgeFilament) {
                callbacks_.purgeFilament(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M706") {
            // Retract filament
            if (callbacks_.retractFilament) {
                callbacks_.retractFilament(static_cast<int>(g.get('T', 0)));
            }
            return true;
        } else if (g.code == "M707") {
            // Set filament sensor state
            if (callbacks_.setFilamentSensorState) {
                callbacks_.setFilamentSensorState(
                    static_cast<int>(g.get('S', 0)),
                    g.get('P', 0.0) != 0);
            }
            return true;
        } else if (g.code == "M708") {
            // Report filament sensor state
            if (callbacks_.reportFilamentSensorState) {
                auto msg = callbacks_.reportFilamentSensorState();
                if (callbacks_.output && !msg.empty()) callbacks_.output(msg);
            }
            return true;
        } else if (g.code == "M860") {
            // Set secondary MCU serial path
            if (callbacks_.setSecondaryMcuSerial) {
                callbacks_.setSecondaryMcuSerial(
                    static_cast<int>(g.get('S', 0)),
                    std::to_string(static_cast<int>(g.get('P', 0))));
            }
            return true;
        } else if (g.code == "M861") {
            // Set secondary MCU baud rate
            if (callbacks_.setSecondaryMcuBaud) {
                callbacks_.setSecondaryMcuBaud(
                    static_cast<int>(g.get('S', 0)),
                    static_cast<int>(g.get('B', 250000)));
            }
            return true;
        } else if (g.code == "M862") {
            // Enable/disable secondary MCU
            if (callbacks_.setSecondaryMcuEnabled) {
                callbacks_.setSecondaryMcuEnabled(
                    static_cast<int>(g.get('S', 0)),
                    g.get('P', 0.0) != 0);
            }
            return true;
        } else if (g.code == "M863") {
            // Set secondary MCU clock frequency
            if (callbacks_.setSecondaryMcuFreq) {
                callbacks_.setSecondaryMcuFreq(
                    static_cast<int>(g.get('S', 0)),
                    static_cast<uint32_t>(g.get('F', 0)));
            }
            return true;
        } else if (g.code == "M876") {
            // Get secondary MCU status
            if (callbacks_.getSecondaryMcuStatus) {
                auto msg = callbacks_.getSecondaryMcuStatus(
                    static_cast<int>(g.get('S', 0)));
                if (callbacks_.output && !msg.empty()) callbacks_.output(msg);
            }
            return true;
        } else if (g.code == "G92.1" || g.code == "G92.2" || g.code == "G92.3") {
            // Reset G92 offsets.
            // G92.1: reset G92 offsets and set position to zero.
            // G92.2: reset G92 offsets but keep position (suspend).
            // G92.3: restore G92 offsets (resume from suspended).
            int mode = 1;
            if (g.code == "G92.2") mode = 2;
            else if (g.code == "G92.3") mode = 3;

            if (mode == 1) {
                // G92.1: zero out G92 offset and reset position to 0.
                state_->g92Offset = {0, 0, 0};
                state_->g92Active = false;
                state_->position[0] = 0;
                state_->position[1] = 0;
                state_->position[2] = 0;
                state_->rebuildCoordTransform();
            } else if (mode == 2) {
                // G92.2: suspend G92 offset (reset to zero, keep position).
                state_->g92Offset = {0, 0, 0};
                state_->g92Active = false;
                state_->rebuildCoordTransform();
            }
            // G92.3: restore — no-op for now (would need saved state).

            if (callbacks_.resetG92Offsets) {
                callbacks_.resetG92Offsets(mode);
            }
            return true;
        } else if (g.code == "M226") {
            // Wait for pin state
            int pin = static_cast<int>(g.get('P', 0));
            int pinState = static_cast<int>(g.get('S', 0));
            if (callbacks_.waitForPin) {
                callbacks_.waitForPin(pin, pinState, 60.0);
            } else if (callbacks_.waitForPinState) {
                callbacks_.waitForPinState(pin, pinState);
            }
            return true;
        } else if (g.code == "M240") {
            // Trigger camera
            if (callbacks_.triggerCamera) callbacks_.triggerCamera();
            return true;
        } else if (g.code == "M250") {
            // Set LCD contrast
            if (callbacks_.setLcdContrast) {
                callbacks_.setLcdContrast(static_cast<int>(g.get('S', 0)));
            }
            return true;
        } else if (g.code == "M260") {
            // Send I2C data
            if (callbacks_.sendI2cData) {
                uint8_t addr = static_cast<uint8_t>(g.get('A', 0));
                std::vector<uint8_t> data;
                // Parse data from B parameter (comma-separated hex)
                // For simplicity, accept a single byte via B
                if (g.has('B')) {
                    data.push_back(static_cast<uint8_t>(g.get('B', 0)));
                }
                callbacks_.sendI2cData(addr, data);
            }
            return true;
        } else if (g.code == "M261") {
            // Request I2C data
            if (callbacks_.requestI2cData) {
                uint8_t addr = static_cast<uint8_t>(g.get('A', 0));
                size_t len = static_cast<size_t>(g.get('B', 0));
                auto data = callbacks_.requestI2cData(addr, len);
                if (callbacks_.output) {
                    std::ostringstream ss;
                    ss << "I2C " << std::hex << static_cast<int>(addr) << ":";
                    for (auto b : data) {
                        ss << " " << std::hex << static_cast<int>(b);
                    }
                    callbacks_.output(ss.str());
                }
            }
            return true;
        } else if (g.code == "M355") {
            // Set case light
            if (callbacks_.setCaseLight) {
                bool on = (g.get('S', 0.0) != 0);
                double brightness = g.get('P', 100.0) / 100.0;
                callbacks_.setCaseLight(on, brightness);
            }
            return true;
        } else if (g.code == "M428") {
            // Set home offset from current position
            if (callbacks_.setHomeOffsetFromPosition) callbacks_.setHomeOffsetFromPosition();
            return true;
        } else if (g.code == "M524") {
            // Abort SD print
            if (callbacks_.abortSdPrint) callbacks_.abortSdPrint();
            return true;
        } else if (g.code == "M650") {
            // Clear bed mesh
            if (callbacks_.clearBedMesh) callbacks_.clearBedMesh();
            return true;
        } else if (g.code == "M852") {
            // Set skew correction
            if (callbacks_.setSkewCorrection) {
                callbacks_.setSkewCorrection(
                    g.get('X', 0.0), g.get('Y', 0.0), g.get('Z', 0.0));
            }
            return true;
        } else if (g.code == "M853") {
            // Probe calibration
            if (callbacks_.setProbeCalibration) {
                callbacks_.setProbeCalibration(g.get('Z', 0.0));
            }
            return true;
        } else if (g.code == "G5") {
            // Bezier spline move
            return executeBezierMove(g);
        } else if (g.code == "G53") {
            // Machine coordinates (non-modal)
            double x = g.has('X') ? g.get('X') : NAN;
            double y = g.has('Y') ? g.get('Y') : NAN;
            double z = g.has('Z') ? g.get('Z') : NAN;
            double speed = (g.has('F') ? g.get('F') : state_->feedrate) / 60.0;
            if (callbacks_.moveMachine) {
                callbacks_.moveMachine(x, y, z, speed);
            }
            return true;
        } else if (g.code == "G54" || g.code == "G55" || g.code == "G56" ||
                   g.code == "G57" || g.code == "G58" || g.code == "G59") {
            int sys = std::stoi(g.code.substr(1)) - 54;
            state_->activeCoordSystem = sys;
            if (callbacks_.selectCoordinateSystem) callbacks_.selectCoordinateSystem(sys);
            return true;
        } else if (g.code == "G59.1") {
            state_->activeCoordSystem = 6;
            if (callbacks_.selectCoordinateSystem) callbacks_.selectCoordinateSystem(6);
            return true;
        } else if (g.code == "G59.2") {
            state_->activeCoordSystem = 7;
            if (callbacks_.selectCoordinateSystem) callbacks_.selectCoordinateSystem(7);
            return true;
        } else if (g.code == "G59.3") {
            state_->activeCoordSystem = 8;
            if (callbacks_.selectCoordinateSystem) callbacks_.selectCoordinateSystem(8);
            return true;
        } else if (g.code == "G52") {
            // G52 — local coordinate offset. NaN means "unchanged" for
            // unspecified axes; no axis words means reset to zero.
            double x = g.has('X') ? g.get('X') : NAN;
            double y = g.has('Y') ? g.get('Y') : NAN;
            double z = g.has('Z') ? g.get('Z') : NAN;
            if (callbacks_.setLocalOffset) callbacks_.setLocalOffset(x, y, z);
            return true;
        } else if (g.code == "G68") {
            // G68 — coordinate rotation. Dispatch on which words are present.
            const bool hasA = g.has('A'), hasB = g.has('B'), hasC = g.has('C');
            const bool hasI = g.has('I'), hasJ = g.has('J'), hasK = g.has('K');
            const bool hasR = g.has('R');
            double px = g.get('X', 0.0), py = g.get('Y', 0.0), pz = g.get('Z', 0.0);
            if (hasI || hasJ || hasK) {
                // 3D axis-angle: I/J/K axis + R angle.
                double ix = g.get('I', 0.0), iy = g.get('J', 0.0), iz = g.get('K', 0.0);
                double angle = hasR ? g.get('R') : 0.0;
                if (callbacks_.setCoordinateRotationAxis)
                    callbacks_.setCoordinateRotationAxis(ix, iy, iz, angle, px, py, pz);
            } else if (hasA || hasB || hasC) {
                // 3D Euler XYZ: A/B/C angles.
                double a = g.get('A', 0.0), b = g.get('B', 0.0), c = g.get('C', 0.0);
                if (callbacks_.setCoordinateRotation3D)
                    callbacks_.setCoordinateRotation3D(a, b, c, px, py, pz);
            } else {
                // 2D plane rotation: R angle, X/Y pivot.
                double angle = hasR ? g.get('R') : 0.0;
                if (callbacks_.setCoordinateRotation2D)
                    callbacks_.setCoordinateRotation2D(angle, px, py);
            }
            return true;
        } else if (g.code == "G69") {
            // G69 — cancel coordinate rotation.
            if (callbacks_.cancelCoordinateRotation) callbacks_.cancelCoordinateRotation();
            return true;
        } else if (g.code == "G43") {
            // G43 — tool length offset from tool table (H word).
            // For now, use H value directly as Z offset (tool table lookup
            // would be done by the callback if available).
            double offset = g.get('H', 0.0);
            state_->toolLengthOffset = offset;
            state_->rebuildCoordTransform();
            return true;
        } else if (g.code == "G43.1") {
            // G43.1 — dynamic tool length offset (Z word specifies offset).
            double offset = g.get('Z', 0.0);
            state_->toolLengthOffset = offset;
            state_->rebuildCoordTransform();
            return true;
        } else if (g.code == "G49") {
            // G49 — cancel tool length offset.
            state_->toolLengthOffset = 0.0;
            state_->rebuildCoordTransform();
            return true;
        } else if (g.code == "G51") {
            // G51 — scaling. P word = uniform; X/Y/Z = per-axis.
            // Extended axes (A/B/C, U/V/W) can also be scaled.
            if (g.has('P')) {
                double s = g.get('P');
                if (callbacks_.setScaling) callbacks_.setScaling(s, s, s);
            } else {
                double sx = g.get('X', 1.0), sy = g.get('Y', 1.0), sz = g.get('Z', 1.0);
                if (callbacks_.setScaling) callbacks_.setScaling(sx, sy, sz);
                // Extended axis scaling (stored in state, applied by transform).
                state_->extScaleFactors = {
                    g.get('A', 1.0), g.get('B', 1.0), g.get('C', 1.0),
                    g.get('U', 1.0), g.get('V', 1.0), g.get('W', 1.0)
                };
                state_->rebuildCoordTransform();
            }
            return true;
        } else if (g.code == "G50") {
            // G50 — cancel scaling.
            if (callbacks_.cancelScaling) callbacks_.cancelScaling();
            state_->extScaleFactors = {1, 1, 1, 1, 1, 1};
            state_->rebuildCoordTransform();
            return true;
        } else if (g.code == "G61.1") {
            state_->pathControlMode = 1;
            if (callbacks_.setPathControl) callbacks_.setPathControl(1, 0.0);
            return true;
        } else if (g.code == "G64") {
            state_->pathControlMode = 2;
            double tol = g.get('P', 0.0);
            state_->pathBlendingTolerance = tol;
            if (callbacks_.setPathControl) callbacks_.setPathControl(2, tol);
            return true;
        } else if (g.code == "G80") {
            state_->cannedCycleActive = false;
            if (callbacks_.cancelCannedCycle) callbacks_.cancelCannedCycle();
            return true;
        } else if (g.code >= "G81" && g.code <= "G89") {
            // Canned cycles
            int type = std::stoi(g.code.substr(1));
            double x = g.has('X') ? g.get('X') : state_->position[0];
            double y = g.has('Y') ? g.get('Y') : state_->position[1];
            double z = g.has('Z') ? g.get('Z') : state_->position[2];
            double r = g.get('R', state_->cannedCycleRetractHeight);
            double f = g.has('F') ? g.get('F') : state_->cannedCycleFeedRate;
            state_->cannedCycleActive = true;
            state_->cannedCycleRetractHeight = r;
            state_->cannedCycleFeedRate = f;
            if (callbacks_.executeCannedCycle) {
                callbacks_.executeCannedCycle(type, x, y, z, r, f);
            }
            return true;
        } else if (g.code == "M0" || g.code == "M1") {
            std::string msg = g.comment;
            if (callbacks_.programStop) callbacks_.programStop(msg);
            return true;
        } else if (g.code == "M2" || g.code == "M30") {
            std::string msg = g.comment;
            if (callbacks_.programEnd) callbacks_.programEnd(msg);
            return true;
        } else if (g.code == "M3" || g.code == "M4") {
            double rpm = g.get('S', 0.0);
            if (g.code == "M4") rpm = -rpm;
            state_->spindleRpm = rpm;
            if (callbacks_.setSpindleSpeed) callbacks_.setSpindleSpeed(rpm);
            return true;
        } else if (g.code == "M5") {
            state_->spindleRpm = 0.0;
            if (callbacks_.setSpindleSpeed) callbacks_.setSpindleSpeed(0.0);
            return true;
        } else if (g.code == "M6") {
            int tool = static_cast<int>(g.get('T', 0));
            if (callbacks_.toolChange) callbacks_.toolChange(tool);
            return true;
        } else if (g.code == "M7") {
            state_->coolantMist = true;
            if (callbacks_.setCoolant) callbacks_.setCoolant(state_->coolantFlood, true);
            return true;
        } else if (g.code == "M8") {
            state_->coolantFlood = true;
            if (callbacks_.setCoolant) callbacks_.setCoolant(true, state_->coolantMist);
            return true;
        } else if (g.code == "M9") {
            state_->coolantFlood = false;
            state_->coolantMist = false;
            if (callbacks_.setCoolant) callbacks_.setCoolant(false, false);
            return true;
        } else if (g.code == "M211") {
            bool enable = g.get('S', 1.0) != 0;
            state_->softwareEndstopsEnabled = enable;
            if (callbacks_.setSoftwareEndstopEnable) callbacks_.setSoftwareEndstopEnable(enable);
            return true;
        } else if (g.code == "M305") {
            int sensor = static_cast<int>(g.get('P', 0));
            double r = g.get('R', 4700.0);
            double beta = g.get('B', 3950.0);
            double rNom = g.get('T', 100000.0);
            double tNom = 25.0;
            if (callbacks_.setThermistorParams) callbacks_.setThermistorParams(sensor, r, beta, rNom, tNom);
            return true;
        } else if (g.code == "M405") {
            state_->filamentWidthSensorEnabled = true;
            if (callbacks_.setFilamentWidthSensor) callbacks_.setFilamentWidthSensor(true);
            return true;
        } else if (g.code == "M406") {
            state_->filamentWidthSensorEnabled = false;
            if (callbacks_.setFilamentWidthSensor) callbacks_.setFilamentWidthSensor(false);
            return true;
        } else if (g.code == "M407") {
            if (callbacks_.getFilamentWidth) {
                std::string result = callbacks_.getFilamentWidth();
                if (callbacks_.output) callbacks_.output(result);
            }
            return true;
        } else if (g.code.substr(0, 1) == "T") {
            // Tool change: T0, T1, etc.
            int tool = 0;
            try { tool = std::stoi(g.code.substr(1)); } catch (...) { tool = 0; }
            if (callbacks_.toolChange) callbacks_.toolChange(tool);
        }

        // Try extended command handler (SET_SERVO, BED_MESH_CALIBRATE, etc.)
        if (g.isExtendedCommand() && callbacks_.extendedCommand) {
            if (callbacks_.extendedCommand(g)) {
                return true;
            }
        }

        // Try macro expansion
        if (macroRegistry_) {
            auto it = macroRegistry_->getMacro(g.code);
            if (it) {
                std::map<std::string, std::string> params;
                for (const auto& [key, value] : g.params) {
                    params[std::string(1, key)] = std::to_string(value);
                }
                std::string expanded = macroRegistry_->expandMacro(g.code, params);
                if (!expanded.empty()) {
                    execute(expanded);
                    return true;
                }
            }
        }

        // Try custom callback
        if (callbacks_.custom) {
            callbacks_.custom(g);
            return true;
        }

        // Unknown command - not an error, just ignore
        return true;
    }

    /// @brief Get the current printer state.
    const PrinterMotionState& state() const { return *state_; }

    /// @brief Get mutable printer state.
    PrinterMotionState& state() { return *state_; }

    /// @brief Get the callbacks (for extended command dispatch).
    GcodeCallbacks& callbacks() { return callbacks_; }
    const GcodeCallbacks& callbacks() const { return callbacks_; }

private:
    /// @brief Compute machine-space speed from program-space feed rate.
    ///
    /// When G51 scaling or G68 rotation is active, the machine-space distance
    /// differs from the program-space distance. This method computes the
    /// ratio by transforming the direction vector and scales the feed rate
    /// accordingly. If the transform is identity, the speed is unchanged.
    ///
    /// @param progStartX/Y/Z  Program-space start position.
    /// @param progEndX/Y/Z    Program-space end position.
    /// @param feedRate        Program-space feed rate (mm/min).
    /// @return Machine-space speed (mm/s).
    double computeScaledSpeed(
        double progStartX, double progStartY, double progStartZ,
        double progEndX, double progEndY, double progEndZ,
        double feedRate) const
    {
        double baseSpeed = feedRate / 60.0 * state_->speedFactor;
        if (state_->coordTransform.isIdentity())
            return baseSpeed;

        // Program-space direction vector.
        double dx = progEndX - progStartX;
        double dy = progEndY - progStartY;
        double dz = progEndZ - progStartZ;
        double progLen = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (progLen < 1e-12)
            return baseSpeed;

        // Transform the direction vector (rotation + scale, no translation).
        auto mv = state_->coordTransform.transformVelocity(dx, dy, dz);
        double machLen = std::sqrt(mv[0]*mv[0] + mv[1]*mv[1] + mv[2]*mv[2]);
        if (machLen < 1e-12)
            return baseSpeed;

        return baseSpeed * (machLen / progLen);
    }

    bool executeMove(const GcodeLine& g) {
        double x = g.has('X') ? g.get('X') : NAN;
        double y = g.has('Y') ? g.get('Y') : NAN;
        double z = g.has('Z') ? g.get('Z') : NAN;
        double e = g.has('E') ? g.get('E') : NAN;
        double f = g.has('F') ? g.get('F') : state_->feedrate;

        state_->feedrate = f;

        // Record start position for speed scaling.
        double startX = state_->position[0];
        double startY = state_->position[1];
        double startZ = state_->position[2];

        // Apply absolute/relative mode
        if (state_->distanceMode == GCode::DistanceMode::ABSOLUTE) {
            if (!std::isnan(x)) state_->position[0] = x;
            if (!std::isnan(y)) state_->position[1] = y;
            if (!std::isnan(z)) state_->position[2] = z;
        } else {
            if (!std::isnan(x)) state_->position[0] += x;
            if (!std::isnan(y)) state_->position[1] += y;
            if (!std::isnan(z)) state_->position[2] += z;
        }

        // E axis: absolute or relative
        if (!std::isnan(e)) {
            if (state_->absoluteExtrude) {
                state_->position[3] = e;
            } else {
                state_->position[3] += e;
            }
        }

        if (callbacks_.move) {
            double speed = computeScaledSpeed(
                startX, startY, startZ,
                state_->position[0], state_->position[1], state_->position[2],
                f);
            // Transform program-space position to machine space.
            auto machine = state_->coordTransform.toMachineXYZ(
                state_->position[0],
                state_->position[1],
                state_->position[2]);
            callbacks_.move(
                machine[0],
                machine[1],
                machine[2],
                state_->position[3],
                speed
            );
        }
        return true;
    }

    bool executeHome(const GcodeLine& g) {
        std::string axes;
        if (g.has('X')) axes += "x";
        if (g.has('Y')) axes += "y";
        if (g.has('Z')) axes += "z";
        if (axes.empty()) axes = "xyz"; // G28 with no params = all

        if (callbacks_.home) {
            callbacks_.home(axes);
        }

        // Update homed axes
        for (char c : axes) {
            std::string s(1, c);
            if (state_->homedAxes.find(s) == std::string::npos) {
                state_->homedAxes += s;
            }
        }
        return true;
    }

    bool executeProbe(const GcodeLine& g) {
        if (callbacks_.probe) {
            double z = callbacks_.probe();
            if (!std::isnan(z)) {
                state_->position[2] = z;
            }
        }
        return true;
    }

    bool executeSetPosition(const GcodeLine& g) {
        double x = g.has('X') ? g.get('X') : NAN;
        double y = g.has('Y') ? g.get('Y') : NAN;
        double z = g.has('Z') ? g.get('Z') : NAN;
        double e = g.has('E') ? g.get('E') : NAN;

        // G92 semantics: "the current position IS now <v>".
        // This sets a G92 offset so that the machine position stays the
        // same while the program position reads as the specified value.
        // newG92 = oldG92 + oldProgramPos - v
        // Then the program position is updated to v.
        bool anyLinear = false;
        if (!std::isnan(x)) {
            state_->g92Offset[0] += state_->position[0] - x;
            state_->position[0] = x;
            anyLinear = true;
        }
        if (!std::isnan(y)) {
            state_->g92Offset[1] += state_->position[1] - y;
            state_->position[1] = y;
            anyLinear = true;
        }
        if (!std::isnan(z)) {
            state_->g92Offset[2] += state_->position[2] - z;
            state_->position[2] = z;
            anyLinear = true;
        }
        if (anyLinear) {
            state_->g92Active = true;
            state_->rebuildCoordTransform();
        }

        // E axis: G92 E0 is common for resetting extruder position.
        // E is not part of the coordinate transform; set directly.
        if (!std::isnan(e)) {
            state_->position[3] = e;
        }

        if (callbacks_.setPosition) {
            callbacks_.setPosition(x, y, z, e);
        }
        return true;
    }

    bool executeDwell(const GcodeLine& g) {
        double seconds = 0.0;
        if (g.has('P')) seconds = g.get('P') / 1000.0; // P is in ms
        else if (g.has('S')) seconds = g.get('S');      // S is in seconds

        if (callbacks_.dwell) {
            callbacks_.dwell(seconds);
        }
        return true;
    }

    bool executeSetHotendTemp(const GcodeLine& g, bool wait) {
        double temp = g.get('S', 0.0);
        // Use -1 to indicate no T parameter was specified, so the
        // callback can fall back to the active extruder.
        int extruder = g.has('T') ? static_cast<int>(g.get('T', 0)) : -1;

        if (callbacks_.setHotendTemp) {
            callbacks_.setHotendTemp(extruder, temp, wait);
        }
        return true;
    }

    bool executeSetBedTemp(const GcodeLine& g, bool wait) {
        double temp = g.get('S', 0.0);

        if (callbacks_.setBedTemp) {
            callbacks_.setBedTemp(temp, wait);
        }
        return true;
    }

    /// @brief Execute an arc move (G2/G3).
    /// Decomposes the arc into line segments and calls move for each.
    ///
    /// @details
    /// The arc center and all interpolated points are computed in **program
    /// space**. The move callback applies the coordinate transform
    /// (WCS + G52 + G68 rotation + G51 scale) to each interpolated point,
    /// so a circle in program space correctly maps to an ellipse in machine
    /// space when rotation or non-uniform scaling is active.
    ///
    /// The I/J/K center offsets are relative to the start position in
    /// program space and define the arc geometry in program space. The
    /// transform is applied to each interpolated point via the move
    /// callback (through @ref CoordinateTransform::toMachineXYZ).
    ///
    /// For G17 (XY plane): I=X offset, J=Y offset
    /// For G18 (ZX plane): I=X offset, K=Z offset
    /// For G19 (YZ plane): J=Y offset, K=Z offset
    bool executeArcMove(const GcodeLine& g) {
        double x = g.has('X') ? g.get('X') : state_->position[0];
        double y = g.has('Y') ? g.get('Y') : state_->position[1];
        double z = g.has('Z') ? g.get('Z') : state_->position[2];
        double e = g.has('E') ? g.get('E') : NAN;
        double f = g.has('F') ? g.get('F') : state_->feedrate;
        state_->feedrate = f;

        // Convert inches to mm if needed
        double unitScale = (state_->units == GCode::Units::INCH) ? 25.4 : 1.0;
        x *= unitScale; y *= unitScale; z *= unitScale;

        bool clockwise = (g.code == "G2");

        // Get center offsets
        double cx, cy, cz;
        double startX = state_->position[0];
        double startY = state_->position[1];
        double startZ = state_->position[2];

        // Select in-plane axes based on the active plane (G17/G18/G19).
        // For G17 (XY): axisA=X, axisB=Y, normal=Z
        // For G18 (ZX): axisA=Z, axisB=X, normal=Y
        // For G19 (YZ): axisA=Y, axisB=Z, normal=X
        // We compute the arc in the (axisA, axisB) plane, then map back.
        double startA, startB, endA, endB; // in-plane coordinates
        double *pA, *pB;                   // pointers to the axis in state
        switch (state_->plane) {
            case GCode::Plane::ZX:
                startA = startZ; startB = startX;
                endA = z; endB = x;
                pA = &state_->position[2]; pB = &state_->position[0];
                break;
            case GCode::Plane::YZ:
                startA = startY; startB = startZ;
                endA = y; endB = z;
                pA = &state_->position[1]; pB = &state_->position[2];
                break;
            default: // XY
                startA = startX; startB = startY;
                endA = x; endB = y;
                pA = &state_->position[0]; pB = &state_->position[1];
                break;
        }

        if (g.has('R')) {
            // Radius mode — compute center from radius.
            double r = g.get('R') * unitScale;
            bool longArc = (r < 0);
            r = std::abs(r);

            double mA = (startA + endA) / 2.0;
            double mB = (startB + endB) / 2.0;
            double dA = (endA - startA) / 2.0;
            double dB = (endB - startB) / 2.0;
            double halfChord = std::sqrt(dA * dA + dB * dB);

            if (halfChord > r) halfChord = r;

            double h = std::sqrt(std::max(0.0, r * r - halfChord * halfChord));

            double perpA, perpB;
            if (halfChord > 1e-12) {
                perpA = -dB / halfChord;
                perpB = dA / halfChord;
            } else {
                perpA = 0;
                perpB = 0;
            }

            double sideSign;
            if (clockwise) {
                sideSign = longArc ? 1.0 : -1.0;
            } else {
                sideSign = longArc ? -1.0 : 1.0;
            }

            double centerA = mA + sideSign * perpA * h;
            double centerB = mB + sideSign * perpB * h;
            // Map center back to XYZ based on plane.
            switch (state_->plane) {
                case GCode::Plane::ZX: cx = centerB; cy = startY; cz = centerA; break;
                case GCode::Plane::YZ: cx = startX; cy = centerA; cz = centerB; break;
                default: cx = centerA; cy = centerB; cz = startZ; break;
            }
        } else {
            // IJK mode — center is relative to start in program space.
            // I/J/K map to the in-plane axes based on the active plane.
            double offA = 0, offB = 0;
            switch (state_->plane) {
                case GCode::Plane::ZX:
                    offA = g.get('K', 0.0) * unitScale; // Z offset
                    offB = g.get('I', 0.0) * unitScale; // X offset
                    cx = startX + offB; cy = startY; cz = startZ + offA;
                    break;
                case GCode::Plane::YZ:
                    offA = g.get('J', 0.0) * unitScale; // Y offset
                    offB = g.get('K', 0.0) * unitScale; // Z offset
                    cx = startX; cy = startY + offA; cz = startZ + offB;
                    break;
                default: // XY
                    offA = g.get('I', 0.0) * unitScale; // X offset
                    offB = g.get('J', 0.0) * unitScale; // Y offset
                    cx = startX + offA; cy = startY + offB; cz = startZ;
                    break;
            }
        }

        // Compute start and end angles in the active plane.
        double centerA, centerB;
        switch (state_->plane) {
            case GCode::Plane::ZX: centerA = cz; centerB = cx; break;
            case GCode::Plane::YZ: centerA = cy; centerB = cz; break;
            default: centerA = cx; centerB = cy; break;
        }
        double startAngle = std::atan2(startB - centerB, startA - centerA);
        double endAngle = std::atan2(endB - centerB, endA - centerA);
        double radius = std::sqrt((startA - centerA) * (startA - centerA) +
                                  (startB - centerB) * (startB - centerB));

        // Adjust angle range for clockwise/counter-clockwise.
        // In G18 (ZX) and G19 (YZ), the CW/CCW direction is reversed
        // when viewed from the positive normal axis (Y for ZX, X for YZ).
        bool effClockwise = clockwise;
        if (state_->plane == GCode::Plane::ZX || state_->plane == GCode::Plane::YZ)
            effClockwise = !clockwise; // RS274 convention

        if (effClockwise) {
            if (endAngle >= startAngle) endAngle -= 2.0 * M_PI;
        } else {
            if (endAngle <= startAngle) endAngle += 2.0 * M_PI;
        }

        // Decompose into segments (16 segments per full circle minimum)
        double totalAngle = std::abs(endAngle - startAngle);
        int segments = std::max(8, static_cast<int>(totalAngle / (M_PI / 16)));
        double angleStep = (endAngle - startAngle) / segments;

        // Z (or out-of-plane axis) linear interpolation
        double outOfPlaneStart, outOfPlaneEnd;
        double *pOut;
        switch (state_->plane) {
            case GCode::Plane::ZX: outOfPlaneStart = startY; outOfPlaneEnd = y; pOut = &state_->position[1]; break;
            case GCode::Plane::YZ: outOfPlaneStart = startX; outOfPlaneEnd = x; pOut = &state_->position[0]; break;
            default: outOfPlaneStart = startZ; outOfPlaneEnd = z; pOut = &state_->position[2]; break;
        }
        double outStep = (outOfPlaneEnd - outOfPlaneStart) / segments;

        double ePerSegment = 0;
        if (!std::isnan(e)) {
            double totalE = state_->absoluteExtrude ? (e - state_->position[3]) : e;
            ePerSegment = totalE / segments;
        }

        double baseSpeed = f / 60.0 * state_->speedFactor;
        for (int i = 1; i <= segments; ++i) {
            double angle = startAngle + angleStep * i;
            double pa = centerA + radius * std::cos(angle);
            double pb = centerB + radius * std::sin(angle);
            double pOutVal = outOfPlaneStart + outStep * i;
            double pe = std::isnan(e) ? state_->position[3] :
                        (state_->absoluteExtrude ? e : state_->position[3] + ePerSegment);

            // Record previous position for per-segment speed scaling.
            double prevX = state_->position[0];
            double prevY = state_->position[1];
            double prevZ = state_->position[2];

            // Map back to XYZ and update state position (program space).
            switch (state_->plane) {
                case GCode::Plane::ZX:
                    state_->position[0] = pb;  // X = axisB
                    state_->position[1] = pOutVal; // Y = out-of-plane
                    state_->position[2] = pa;  // Z = axisA
                    break;
                case GCode::Plane::YZ:
                    state_->position[0] = pOutVal; // X = out-of-plane
                    state_->position[1] = pa;  // Y = axisA
                    state_->position[2] = pb;  // Z = axisB
                    break;
                default: // XY
                    state_->position[0] = pa;  // X = axisA
                    state_->position[1] = pb;  // Y = axisB
                    state_->position[2] = pOutVal; // Z = out-of-plane
                    break;
            }
            if (!std::isnan(e)) {
                state_->position[3] = state_->absoluteExtrude ? e : state_->position[3] + ePerSegment;
            }

            if (callbacks_.move) {
                double speed = computeScaledSpeed(
                    prevX, prevY, prevZ,
                    state_->position[0], state_->position[1], state_->position[2],
                    f);
                // Transform program-space position to machine space.
                auto machine = state_->coordTransform.toMachineXYZ(
                    state_->position[0],
                    state_->position[1],
                    state_->position[2]);
                callbacks_.move(machine[0], machine[1],
                               machine[2], state_->position[3], speed);
            }
        }
        return true;
    }

    /// @brief Execute a Bezier spline move (G5).
    /// Uses cubic Bezier with I/J control point 1 and P/Q control point 2.
    /// Delegates the actual curve interpolation to the move callback.
    bool executeBezierMove(const GcodeLine& g) {
        // G5 X.. Y.. Z.. E.. F.. I.. J.. P.. Q..
        // First control point: (I, J) relative to start
        // Second control point: (P, Q) relative to end
        double endX = g.has('X') ? g.get('X') : state_->position[0];
        double endY = g.has('Y') ? g.get('Y') : state_->position[1];
        double endZ = g.has('Z') ? g.get('Z') : state_->position[2];
        double endE = g.has('E') ? g.get('E') : NAN;
        double f = g.has('F') ? g.get('F') : state_->feedrate;
        state_->feedrate = f;

        double unitScale = (state_->units == GCode::Units::INCH) ? 25.4 : 1.0;
        endX *= unitScale; endY *= unitScale; endZ *= unitScale;

        double startX = state_->position[0];
        double startY = state_->position[1];
        double startZ = state_->position[2];

        // Control points (relative to start/end respectively)
        double cp1X = startX + g.get('I', 0.0) * unitScale;
        double cp1Y = startY + g.get('J', 0.0) * unitScale;
        double cp2X = endX + g.get('P', 0.0) * unitScale;
        double cp2Y = endY + g.get('Q', 0.0) * unitScale;

        // Sample the Bezier curve
        int segments = 32;
        double startE = state_->position[3];

        for (int i = 1; i <= segments; ++i) {
            double t = static_cast<double>(i) / segments;
            double mt = 1.0 - t;
            // Cubic Bezier: B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
            double px = mt*mt*mt * startX + 3*mt*mt*t * cp1X +
                        3*mt*t*t * cp2X + t*t*t * endX;
            double py = mt*mt*mt * startY + 3*mt*mt*t * cp1Y +
                        3*mt*t*t * cp2Y + t*t*t * endY;
            // Linear interpolation for Z (Klipper doesn't do Z splines)
            double pz = startZ + (endZ - startZ) * t;

            double pe;
            if (!std::isnan(endE)) {
                if (state_->absoluteExtrude) {
                    pe = endE;
                } else {
                    pe = startE + (endE - startE) * t;
                }
            } else {
                pe = state_->position[3];
            }

            double prevX = state_->position[0];
            double prevY = state_->position[1];
            double prevZ = state_->position[2];

            if (state_->distanceMode == GCode::DistanceMode::ABSOLUTE) {
                state_->position[0] = px;
                state_->position[1] = py;
                state_->position[2] = pz;
                if (!std::isnan(endE)) state_->position[3] = pe;
            } else {
                state_->position[0] = px - (i == 1 ? startX : 0);
                state_->position[1] = py - (i == 1 ? startY : 0);
                if (!std::isnan(endE)) state_->position[3] = pe;
            }

            if (callbacks_.move) {
                double speed = computeScaledSpeed(
                    prevX, prevY, prevZ,
                    state_->position[0], state_->position[1], state_->position[2],
                    f);
                // Transform program-space position to machine space.
                auto machine = state_->coordTransform.toMachineXYZ(
                    state_->position[0],
                    state_->position[1],
                    state_->position[2]);
                callbacks_.move(machine[0], machine[1],
                               machine[2], state_->position[3], speed);
            }
        }
        return true;
    }

    GcodeCallbacks callbacks_;
    PrinterMotionState internalState_;
    PrinterMotionState* state_;
    GcodeMacroRegistry* macroRegistry_ = nullptr;
};

} // namespace tether::klipper::klippy

// Include AdvancedObjects after GCodeExecutor is fully defined (for macro support)
#include "tether/klipper/klippy/AdvancedObjects.hpp"
