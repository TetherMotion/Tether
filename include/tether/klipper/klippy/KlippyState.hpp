/**
 * @file KlippyState.hpp
 * @brief Extended command state for KlippyInstance.
 *
 * @details
 * Extracted from KlippyInstance.hpp to reduce the god-object problem.
 * All extended command state (servo positions, bed mesh profiles, LED
 * colors, probe calibration, idle timeout, delayed G-codes, exclude
 * object state, etc.) is grouped here in a single struct.
 *
 * KlippyInstance inherits privately from KlippyState so that the .ipp
 * callback files can reference the state fields by name without changes.
 * Future refactoring can add methods to KlippyState to encapsulate
 * state transitions.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Extended command state for KlippyInstance.
struct KlippyState {
    // --- Servo state (SET_SERVO) ---
    struct ServoState {
        double angle = 0.0;
        double pulseWidth = 0.0;
    };
    std::map<std::string, ServoState> servoStates_;

    // --- Bed mesh profiles (BED_MESH_PROFILE) ---
    struct BedMeshProfile {
        std::string name;
        bool loaded = false;
    };
    std::map<std::string, BedMeshProfile> bedMeshProfiles_;

    // --- Save config pending items (SAVE_CONFIG) ---
    std::map<std::string, std::string> saveConfigPendingItems_;

    // --- G-code offset (SET_GCODE_OFFSET) ---
    std::array<double, 4> gcodeOffset_ = {0, 0, 0, 0}; // X, Y, Z, E
    int gcodeOffsetMode_ = 0; // RESET=0, ADJUST=1

    // --- Extruder rotation distance (SET_EXTRUDER_ROTATION_DISTANCE) ---
    std::map<std::string, double> extruderRotationDistance_;

    // --- Named output pins (SET_PIN / SET_PWM_PIN) ---
    std::map<std::string, double> namedOutputPins_;
    std::map<std::string, double> namedPWMPins_;

    // --- Generic fan speeds (SET_FAN_SPEED) ---
    std::map<std::string, double> genericFanSpeeds_;

    // --- Temperature fan targets (SET_TEMPERATURE_FAN) ---
    std::map<std::string, double> temperatureFanTargets_;

    // --- ADC query results (QUERY_ADC) ---
    std::map<std::string, double> adcValues_;

    // --- LED state (SET_LED) ---
    struct LedState {
        std::array<double, 4> color = {0, 0, 0, 0}; // R, G, B, W
        bool white = false;
        int index = 0;
    };
    std::map<std::string, LedState> ledStates_;

    // --- Neopixel state (SET_NEOPIXEL) ---
    std::map<std::string, std::vector<std::array<double, 4>>> neopixelStates_;

    // --- Probe calibration state (PROBE_CALIBRATE) ---
    struct ProbeCalibState {
        bool active = false;
        double zPosition = 0.0;
        double zOffset = 0.0;
    };
    ProbeCalibState probeCalibState_;

    // --- Stepper enable overrides (SET_STEPPER_ENABLE) ---
    std::map<std::string, bool> stepperEnableOverrides_;

    // --- Idle timeout (SET_IDLE_TIMEOUT) ---
    double idleTimeout_ = 600.0; // 10 minutes default
    std::chrono::steady_clock::time_point lastActivityTime_ =
        std::chrono::steady_clock::now();
    std::string idleTimeoutState_ = "Ready";

    // --- Delayed G-code (SET_DELAYED_GCODE / UPDATE_DELAYED_GCODE) ---
    struct DelayedGcode {
        std::string gcode;
        double delay = 0.0;
        std::chrono::steady_clock::time_point scheduledTime;
        bool enabled = false;
    };
    std::map<std::string, DelayedGcode> delayedGcodes_;

    // --- Dual carriage (SET_DUAL_CARRIAGE) ---
    struct DualCarriageState {
        int mode = 0;
        double offset = 0.0;
    };
    std::map<std::string, DualCarriageState> dualCarriageStates_;

    // --- G-code variables (SET_GCODE_VARIABLE / SAVE_VARIABLE) ---
    std::map<std::string, std::map<std::string, std::string>> gcodeVariables_;
    std::map<std::string, std::string> savedVariables_;

    // --- TMC field overrides (SET_TMC_FIELD) ---
    std::map<std::string, std::map<std::string, uint32_t>> tmcFieldOverrides_;

    // --- Exclude object state (EXCLUDE_OBJECT_*) ---
    struct ExcludeObject {
        std::string name;
        std::vector<std::array<double, 2>> polygon; // XY outline
        bool excluded = false;
        bool started = false;
        bool finished = false;
    };
    std::vector<ExcludeObject> excludeObjects_;
    int currentExcludeObject_ = -1;
    std::set<std::string> excludedObjects_;
    bool excludeObjectStarted_ = false;
    bool excludeObjectEnabled_ = false;

    // --- Manual probe state (MANUAL_PROBE / ABORT / ACCEPT / ADJUSTED) ---
    struct ManualProbeState {
        bool active = false;
        double zPosition = 0.0;
        double zOffset = 0.0;
    };
    ManualProbeState manualProbeState_;

    // --- Sync extruder stepper (SYNC_EXTRUDER_STEPPER) ---
    std::map<std::string, std::string> extruderStepperSync_;

    // --- Print stats info (SET_PRINT_STATS_INFO) ---
    std::string printStatsInfoTotalLayer_;
    std::string printStatsInfoCurrentLayer_;

    // --- Display group (SET_DISPLAY_GROUP) ---
    std::string displayGroup_;

    // --- Config section state ---
    bool saveVariablesEnabled_ = false;
    bool forceMoveEnabled_ = false;
    bool palette2Configured_ = false;
    std::map<std::string, std::string> homingOverrides_;
    struct EndstopPhaseState {
        int endstopAlignTolerance = 0;
    };
    std::map<std::string, EndstopPhaseState> endstopPhases_;
    std::map<std::string, std::string> menuDefinitions_;

    // --- Accelerometer state (TEST_RESONANCES / SHAPER_CALIBRATE) ---
    bool accelerometerMeasuring_ = false;
    std::vector<std::array<double, 3>> accelerometerData_;

    // --- G-code state save/restore (SAVE_GCODE_STATE / RESTORE_GCODE_STATE) ---
    struct GcodeState {
        std::array<double, 4> position = {0, 0, 0, 0};
        std::array<double, 4> gcodeOffset = {0, 0, 0, 0};
        bool absoluteCoords = true;
        bool absoluteExtrude = false;
        double feedrate = 1500.0;
        double speedFactor = 1.0;
        double extrudeFactor = 1.0;
    };
    std::map<std::string, GcodeState> gcodeStates_;

    // --- Active extruder name (ACTIVATE_EXTRUDER) ---
    std::string activeExtruderName_ = "extruder";

    // --- Extruder step distance (SET_EXTRUDER_STEP_DISTANCE) ---
    std::map<std::string, double> extruderStepDistance_;

    // --- Digital pin states (SET_DIGITAL_PIN) ---
    std::map<std::string, bool> digitalPinStates_;

    // --- Dotstar LED states (SET_DOTSTAR) ---
    std::map<std::string, std::vector<std::array<double, 4>>> dotstarStates_;

    // --- Bed mesh offset (BED_MESH_OFFSET) ---
    double bedMeshOffsetX_ = 0.0;
    double bedMeshOffsetY_ = 0.0;

    // --- Stepper current (SET_CURRENT) ---
    std::map<std::string, double> stepperCurrents_;
    std::map<std::string, double> stepperHoldCurrents_;

    // --- Home position (SET_HOME_POSITION) ---
    std::array<double, 3> homePosition_ = {0, 0, 0};

    // --- Endstop home positions (ENDSTOP_HOME) ---
    std::map<std::string, double> endstopHomePositions_;

    // --- Peripheral state (for M42/M300/M280/M150) ---
    std::map<int, double> outputPins_;
    double lastBeepFreq_ = 0.0;
    double lastBeepDuration_ = 0.0;
    std::map<int, double> servos_;
    std::array<int, 4> ledColor_ = {0, 0, 0, 0};
    int lcdContrast_ = 100;
    std::array<double, 4> g92Offsets_ = {0, 0, 0, 0};
    std::array<double, 4> savedG92Offsets_ = {0, 0, 0, 0};
    std::chrono::steady_clock::time_point lastCameraTrigger_;

    // --- CNC state ---
    double spindleRpm_ = 0.0;
    int activeTool_ = 0;
    bool coolantFlood_ = false;
    bool coolantMist_ = false;

    // --- Thermistor parameters (M305) ---
    std::map<int, std::array<double, 4>> thermistorParams_;

    // --- Motion callback (for G53 machine moves and canned cycles) ---
    /// @brief Move callback that applies the coordinate transform (WCS,
    ///        G52, G68 rotation, G51 scale). Used for normal G-code moves.
    std::function<void(double, double, double, double, double)> moveCallback_;
    /// @brief Raw move callback that bypasses the coordinate transform.
    /// Used for G53 (machine coordinates) and internal repositioning.
    std::function<void(double, double, double, double, double)> moveCallbackRaw_;

    // --- Runtime state ---
    double autoTempInterval_ = 0.0;
    uint32_t moveQueueDepth_ = 0;
};

} // namespace tether::klipper::klippy
