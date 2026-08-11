#pragma once

/// @file GCodeCallbacks.hpp
/// @brief Callbacks for G-code execution

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

struct GcodeLine;

// ============================================================================
// G-code executor callbacks
// ============================================================================

/// @brief Callbacks for G-code execution.
struct GcodeCallbacks {
    /// @brief Move to a position. Called for G0/G1.
    /// @param x,y,z,e Target positions (NaN = no change).
    /// @param speed Feedrate in mm/s.
    std::function<void(double x, double y, double z, double e, double speed)> move;

    /// @brief Home axes. Called for G28.
    /// @param axes Empty = all, otherwise "x", "y", "z", etc.
    std::function<void(const std::string& axes)> home;

    /// @brief Set hotend temperature. Called for M104/M109.
    std::function<void(int extruder, double temp, bool wait)> setHotendTemp;

    /// @brief Set bed temperature. Called for M140/M190.
    std::function<void(double temp, bool wait)> setBedTemp;

    /// @brief Set fan speed. Called for M106/M107.
    std::function<void(double speed)> setFanSpeed;

    /// @brief Set motor enable. Called for M17/M18/M84.
    std::function<void(const std::string& axes, bool enable)> setMotorEnable;

    /// @brief Emergency stop. Called for M112.
    std::function<void()> emergencyStop;

    /// @brief Probe. Called for G30.
    /// @return Z position at probe trigger, or NaN if failed.
    std::function<double()> probe;

    /// @brief Set position. Called for G92.
    std::function<void(double x, double y, double z, double e)> setPosition;

    /// @brief Wait for a specified time. Called for G4.
    std::function<void(double seconds)> dwell;

    /// @brief Output a message. Called for M118 or G-code comments.
    std::function<void(const std::string& message)> output;

    /// @brief Execute a custom command.
    std::function<void(const GcodeLine& line)> custom;

    // --- Advanced callbacks ---

    /// @brief Firmware retract (G10). Return E movement.
    std::function<double()> retract;

    /// @brief Firmware unretract (G11). Return E movement.
    std::function<double()> unretract;

    /// @brief Select SD file (M23).
    std::function<bool(const std::string& filename)> selectSdFile;

    /// @brief Start/resume SD print (M24).
    std::function<void()> startSdPrint;

    /// @brief Pause SD print (M25).
    std::function<void()> pauseSdPrint;

    /// @brief Report SD status (M27).
    std::function<std::string()> sdStatus;

    /// @brief Set display progress (M73).
    std::function<void(double progress)> setDisplayProgress;

    /// @brief Set display message (M117).
    std::function<void(const std::string& message)> setDisplayMessage;

    /// @brief Set position with homing offset (G60.0/G61.0 save/restore).
    std::function<void(int slot)> savePosition;
    std::function<void(int slot)> restorePosition;

    // --- Status query callbacks ---

    /// @brief Get current position string (M114).
    std::function<std::string()> getPositionStatus;

    /// @brief Get endstop status string (M119).
    std::function<std::string()> getEndstopStatus;

    /// @brief Get temperature status string (M105).
    std::function<std::string()> getTempStatus;

    /// @brief Enable/disable auto temperature reporting (M155).
    std::function<void(double interval)> setAutoTempReport;

    // --- Sync ---

    /// @brief Wait for all queued moves to complete (M400).
    std::function<void()> waitForMoves;

    // --- Bed leveling ---

    /// @brief Start bed mesh leveling (G29).
    std::function<void()> bedLevel;

    // --- Advanced motion settings ---

    /// @brief Set pressure advance (M900).
    std::function<void(int extruder, double pa)> setPressureAdvance;

    /// @brief Set input shaper parameters (M593).
    std::function<void(const std::string& axis, double freq, const std::string& type)> setInputShaperParams;

    // --- Nozzle maintenance ---

    /// @brief Clean nozzle (G12).
    std::function<void(double iterations, double radius, double speed)> cleanNozzle;

    // --- Settings ---

    /// @brief Save settings (M500).
    std::function<void()> saveSettings;

    /// @brief Load settings (M501).
    std::function<void()> loadSettings;

    /// @brief Reset to factory defaults (M502).
    std::function<void()> resetSettings;

    /// @brief Report current settings (M503).
    std::function<std::string()> reportSettings;

    // --- Firmware info ---

    /// @brief Get firmware version string (M115).
    std::function<std::string()> getFirmwareInfo;

    /// @brief Wait for all temperatures to reach target (M116).
    std::function<void()> waitForTemperatures;

    // --- Stepper configuration ---

    /// @brief Set steps per mm (M92).
    std::function<void(const std::string& axis, double steps)> setStepsPerMm;

    /// @brief Set microstepping (M350).
    std::function<void(const std::string& axis, int microsteps)> setMicrostepping;

    /// @brief Set stepper driver current in mA (M906).
    std::function<void(const std::string& axis, double currentMa)> setStepperCurrent;

    /// @brief Set stepper driver direction/edge config (M569).
    std::function<void(const std::string& axis, int direction)> setStepperDirection;

    // --- Motion limits ---

    /// @brief Set max feedrate (M203).
    std::function<void(const std::string& axis, double feedrate)> setMaxFeedrate;

    /// @brief Set acceleration (M201/M204).
    std::function<void(double accel, double travelAccel)> setAcceleration;

    /// @brief Set advanced motion settings (M205).
    std::function<void(double jerk, double startAccel)> setAdvancedMotion;

    // --- Offsets ---

    /// @brief Set home offset (M206).
    std::function<void(const std::string& axis, double offset)> setHomeOffset;

    /// @brief Set tool offset (M218).
    std::function<void(int tool, const std::string& axis, double offset)> setToolOffset;

    /// @brief Set probe Z offset (M851).
    std::function<void(double offset)> setProbeOffset;

    // --- Retract settings ---

    /// @brief Set firmware retract parameters (M207).
    std::function<void(double length, double speed, double zLift)> setRetractParams;

    /// @brief Set firmware unretract parameters (M208).
    std::function<void(double length, double speed)> setUnretractParams;

    // --- PID ---

    /// @brief Set hotend PID (M301).
    std::function<void(double kp, double ki, double kd)> setHotendPid;

    /// @brief Set bed PID (M304).
    std::function<void(double kp, double ki, double kd)> setBedPid;

    /// @brief Run PID autotune (M303).
    std::function<std::string(double temp, int cycles)> runPidAutotune;

    // --- Probe control ---

    /// @brief Deploy probe (M401).
    std::function<void()> deployProbe;

    /// @brief Stow probe (M402).
    std::function<void()> stowProbe;

    // --- Bed mesh management ---

    /// @brief Enable/disable bed mesh (M420).
    std::function<void(bool enable)> setBedMeshEnabled;

    /// @brief Set bed mesh point (M421).
    std::function<void(int xIdx, int yIdx, double z)> setBedMeshPoint;

    // --- Backlash ---

    /// @brief Set backlash compensation (M425).
    std::function<void(const std::string& axis, double compensation)> setBacklash;

    // --- Filament ---

    /// @brief Set filament diameter (M200).
    std::function<void(double diameter)> setFilamentDiameter;

    /// @brief Filament change (M600).
    std::function<void()> filamentChange;

    // --- Misc ---

    /// @brief Set pin state (M42).
    std::function<void(int pin, double value)> setPinState;

    /// @brief Beep (M300).
    std::function<void(double freq, double duration)> beep;

    /// @brief Servo control (M280).
    std::function<void(int servo, double angle)> setServoAngle;

    /// @brief Set LED color (M150).
    std::function<void(int r, int g, int b, int w)> setLedColor;

    // --- Delta printer support (M665/M666) ---

    /// @brief Set delta geometry parameters (M665).
    /// Parameters: arm length, delta radius, tower angle offsets.
    std::function<void(double armLength, double deltaRadius,
                       double angleA, double angleB, double angleC)> setDeltaGeometry;

    /// @brief Set delta endstop adjustments (M666).
    /// Parameters: X, Y, Z endstop adjustments.
    std::function<void(double adjX, double adjY, double adjZ)> setDeltaEndstopAdjust;

    // --- TMC driver configuration (M907-M914) ---

    /// @brief Set TMC driver current (M907).
    std::function<void(const std::string& axis, double currentMa)> setTmcCurrent;

    /// @brief Set TMC driver run current (M908).
    std::function<void(const std::string& axis, double currentMa)> setTmcRunCurrent;

    /// @brief Set TMC driver hold current (M909).
    std::function<void(const std::string& axis, double currentMa)> setTmcHoldCurrent;

    /// @brief Set TMC driver stealthChop mode (M911).
    std::function<void(const std::string& axis, bool enable)> setTmcStealthChop;

    /// @brief Set TMC driver spreadCycle threshold (M912).
    std::function<void(const std::string& axis, double threshold)> setTmcSpreadThreshold;

    /// @brief Set TMC driver bump sensitivity (M913).
    std::function<void(const std::string& axis, int sensitivity)> setTmcBumpSensitivity;

    /// @brief Set TMC driver diag pin (M914).
    std::function<void(const std::string& axis, int diag)> setTmcDiagPin;

    // --- Filament load/unload (M701-M708) ---

    /// @brief Load filament (M701).
    std::function<void(int extruder)> loadFilament;

    /// @brief Unload filament (M702).
    std::function<void(int extruder)> unloadFilament;

    /// @brief Load filament into tool (M703).
    std::function<void(int tool)> loadFilamentToTool;

    /// @brief Unload filament from tool (M704).
    std::function<void(int tool)> unloadFilamentFromTool;

    /// @brief Purge filament (M705).
    std::function<void(int extruder)> purgeFilament;

    /// @brief Retract filament (M706).
    std::function<void(int extruder)> retractFilament;

    /// @brief Set filament sensor state (M707).
    std::function<void(int sensor, bool enabled)> setFilamentSensorState;

    /// @brief Report filament sensor state (M708).
    std::function<std::string()> reportFilamentSensorState;

    // --- Multi-MCU coordination (M860-M876) ---

    /// @brief Set secondary MCU serial path (M860).
    std::function<void(int mcuId, const std::string& serialPath)> setSecondaryMcuSerial;

    /// @brief Set secondary MCU baud rate (M861).
    std::function<void(int mcuId, int baudRate)> setSecondaryMcuBaud;

    /// @brief Enable/disable secondary MCU (M862).
    std::function<void(int mcuId, bool enable)> setSecondaryMcuEnabled;

    /// @brief Set secondary MCU clock frequency (M863).
    std::function<void(int mcuId, uint32_t freq)> setSecondaryMcuFreq;

    /// @brief Get secondary MCU status (M876).
    std::function<std::string(int mcuId)> getSecondaryMcuStatus;

    // --- Additional missing G-codes ---

    /// @brief Reset G92 offsets (G92.1/G92.2/G92.3).
    std::function<void(int mode)> resetG92Offsets;

    /// @brief Wait for pin state (M226).
    std::function<void(int pin, int state)> waitForPinState;

    /// @brief Trigger camera (M240).
    std::function<void()> triggerCamera;

    /// @brief Set LCD contrast (M250).
    std::function<void(int contrast)> setLcdContrast;

    /// @brief Send I2C data (M260).
    std::function<void(uint8_t addr, const std::vector<uint8_t>& data)> sendI2cData;

    /// @brief Request I2C data (M261).
    std::function<std::vector<uint8_t>(uint8_t addr, size_t len)> requestI2cData;

    /// @brief Set case light (M355).
    std::function<void(bool on, double brightness)> setCaseLight;

    /// @brief Set home offset from current position (M428).
    std::function<void()> setHomeOffsetFromPosition;

    /// @brief Abort SD print (M524).
    std::function<void()> abortSdPrint;

    /// @brief Clear bed mesh (M650).
    std::function<void()> clearBedMesh;

    /// @brief Set skew correction (M852).
    std::function<void(double xy, double xz, double yz)> setSkewCorrection;

    /// @brief Probe calibration (M853).
    std::function<void(double zOffset)> setProbeCalibration;

    // --- Spindle / tool / coolant (CNC) ---

    /// @brief Set spindle speed (M3/M4/M5). Positive=CW, negative=CCW, 0=off.
    std::function<void(double rpm)> setSpindleSpeed;

    /// @brief Tool change (M6). Tool number from T parameter.
    std::function<void(int tool)> toolChange;

    /// @brief Set coolant (M7/M8/M9). flood, mist, off.
    std::function<void(bool flood, bool mist)> setCoolant;

    // --- Coordinate systems (G54-G59.3) ---

    /// @brief Select coordinate system (G54-G59.3). System index 0-8.
    std::function<void(int system)> selectCoordinateSystem;

    /// @brief Set coordinate system offset (G10 L2/L20).
    std::function<void(int system, double x, double y, double z)> setCoordinateSystemOffset;

    // --- Local offset (G52) ---

    /// @brief Set local coordinate offset (G52). NaN axis = unchanged.
    std::function<void(double x, double y, double z)> setLocalOffset;

    // --- Coordinate rotation (G68/G69) ---

    /// @brief Set 2D coordinate rotation (G68 with R word only).
    /// @param angleDeg  Rotation angle in degrees.
    /// @param pivotX    Pivot X coordinate (in-plane first axis).
    /// @param pivotY    Pivot Y coordinate (in-plane second axis).
    std::function<void(double angleDeg, double pivotX, double pivotY)>
        setCoordinateRotation2D;

    /// @brief Set 3D coordinate rotation via Euler angles (G68 A/B/C).
    std::function<void(double aDeg, double bDeg, double cDeg,
                        double px, double py, double pz)>
        setCoordinateRotation3D;

    /// @brief Set 3D coordinate rotation via axis-angle (G68 I/J/K + R).
    std::function<void(double ix, double iy, double iz, double angleDeg,
                        double px, double py, double pz)>
        setCoordinateRotationAxis;

    /// @brief Cancel coordinate rotation (G69).
    std::function<void()> cancelCoordinateRotation;

    // --- Scaling (G51/G50) ---

    /// @brief Set per-axis scaling (G51).
    std::function<void(double sx, double sy, double sz)> setScaling;

    /// @brief Cancel scaling (G50).
    std::function<void()> cancelScaling;

    // --- Machine coordinates ---

    /// @brief Move in machine coordinates (G53). Non-modal.
    std::function<void(double x, double y, double z, double speed)> moveMachine;

    // --- Path control ---

    /// @brief Set path control mode (G61/G61.1/G64).
    /// mode: 0=exact stop, 1=exact path, 2=blending. tolerance for blending.
    std::function<void(int mode, double tolerance)> setPathControl;

    // --- Program flow ---

    /// @brief Programmed stop (M0/M1). Optional message.
    std::function<void(const std::string& message)> programStop;

    /// @brief Program end (M2/M30). Optional message.
    std::function<void(const std::string& message)> programEnd;

    // --- Software endstops ---

    /// @brief Set software endstops (M208). axis, min, max, enable.
    std::function<void(const std::string& axis, double min, double max, bool enable)> setSoftwareEndstops;

    /// @brief Enable/disable software endstops (M211).
    std::function<void(bool enable)> setSoftwareEndstopEnable;

    // --- Thermistor parameters ---

    /// @brief Set thermistor parameters (M305). sensor, R, beta, etc.
    std::function<void(int sensor, double rPullup, double beta, double rNominal, double tNominal)> setThermistorParams;

    // --- Filament sensor ---

    /// @brief Enable/disable filament width sensor (M405/M406).
    std::function<void(bool enable)> setFilamentWidthSensor;

    /// @brief Set filament width sensor measurement (M407).
    std::function<std::string()> getFilamentWidth;

    // --- Canned cycles ---

    /// @brief Execute canned cycle (G81-G89). type, x, y, z, r, feed, retract.
    std::function<void(int type, double x, double y, double z,
                       double retractHeight, double feedRate)> executeCannedCycle;

    /// @brief Cancel canned cycle mode (G80).
    std::function<void()> cancelCannedCycle;

    // --- Bed probing (G29) ---

    /// @brief Probe bed and fill mesh (G29).
    /// @return Number of points probed, or -1 on failure.
    std::function<int()> probeBed;

    // --- Auto bed leveling (G32) ---

    /// @brief Auto bed level: probe bed and apply correction (G32).
    /// @return Number of points probed, or -1 on failure.
    std::function<int()> autoBedLevel;

    // --- Delta calibration (G33) ---

    /// @brief Delta calibration: calibrate delta endstops and geometry (G33).
    /// @return Number of calibration iterations, or -1 on failure.
    std::function<int()> deltaCalibrate;

    // --- Z tilt leveling (G34) ---

    /// @brief Z tilt: level multiple Z steppers (G34).
    /// @return True if leveling succeeded.
    std::function<bool()> zTiltLevel;

    // --- Pin polling (M226) ---

    /// @brief Wait for pin to reach state (M226). Returns true if reached.
    std::function<bool(int pin, int state, double timeout)> waitForPin;

    // --- Print control (Moonraker-style) ---

    /// @brief Start print from SD file.
    std::function<void()> startPrint;

    /// @brief Cancel current print.
    std::function<void()> cancelPrint;

    /// @brief Pause current print.
    std::function<void()> pausePrint;

    /// @brief Resume current print.
    std::function<void()> resumePrint;

    // --- Extended (Klipper module) commands ---
    // These are commands like SET_SERVO, BED_MESH_CALIBRATE, etc.
    // The handler receives the full parsed GcodeLine with namedParams populated.
    // Return true if the command was handled, false if unrecognized.
    std::function<bool(const GcodeLine& line)> extendedCommand;
};

} // namespace tether::klipper::klippy
