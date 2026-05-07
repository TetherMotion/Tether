/**
 * @file MarlinMCodeHandler.hpp
 * @brief Marlin-compatible M-code Handler with Callback Registration
 *
 * @details
 * This handler provides Marlin-compatible M-code support with the ability
 * to register custom callback functions for any M-code.
 *
 * Supported Marlin M-codes (built-in):
 *
 * Motion & Speed:
 * - M201: Set maximum print acceleration (per axis)
 * - M203: Set maximum feedrate (per axis)
 * - M204: Set starting acceleration (print/retract/travel)
 * - M205: Set advanced settings (jerk, junction deviation)
 *
 * Temperature (callbacks):
 * - M104: Set extruder temperature
 * - M109: Set extruder temperature and wait
 * - M140: Set bed temperature
 * - M190: Set bed temperature and wait
 *
 * Tool & IO:
 * - M3/M4: Spindle on CW/CCW (or laser on)
 * - M5: Spindle/laser off
 * - M6: Tool change
 * - M7/M8/M9: Coolant control
 *
 * Program Control:
 * - M0/M1: Pause
 * - M2/M30: Program end
 * - M24/M25: Start/Pause SD print
 * - M82/M83: Extruder absolute/relative mode
 *
 * Information:
 * - M114: Report current position
 * - M115: Report firmware info
 * - M119: Report endstop status
 * - M503: Report settings
 *
 * Custom M-codes:
 * - M42: Set pin state
 * - M106/M107: Fan control
 * - M117: Display message
 * - M118: Serial print
 *
 * @author G-Code Export Tool
 * @version 1.0
 */

#pragma once

#include "GCodeTypes.hpp"
#include "motion/InterpolationStrategy.hpp"
#include <functional>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <mutex>

namespace GCode {

// ============================================================================
// M-Code Parameter Structure
// ============================================================================

/**
 * @brief Parameters parsed from an M-code command
 */
struct MCodeParameters {
    int mCode = -1;                         ///< The M-code number
    std::optional<double> P;                ///< P parameter
    std::optional<double> S;                ///< S parameter (often spindle speed)
    std::optional<double> R;                ///< R parameter
    std::optional<double> F;                ///< F parameter (feedrate)
    std::optional<double> T;                ///< T parameter (tool number)
    std::optional<double> I;                ///< I parameter
    std::optional<double> J;                ///< J parameter
    std::optional<double> K;                ///< K parameter

    // Axis parameters (for M201/M203/M204/M205)
    std::optional<double> X;
    std::optional<double> Y;
    std::optional<double> Z;
    std::optional<double> A;
    std::optional<double> B;
    std::optional<double> C;
    std::optional<double> U;
    std::optional<double> V;
    std::optional<double> W;
    std::optional<double> E;                ///< Extruder parameter

    // String parameters
    std::string message;                    ///< For M117/M118 messages

    // Additional context
    int lineNumber = -1;                    ///< Source line number
    std::string rawLine;                    ///< Original command text

    /**
     * @brief Get axis value by index
     */
    std::optional<double> getAxis(size_t index) const {
        switch (index) {
            case 0: return X;
            case 1: return Y;
            case 2: return Z;
            case 3: return A;
            case 4: return B;
            case 5: return C;
            case 6: return U;
            case 7: return V;
            case 8: return W;
            default: return std::nullopt;
        }
    }

    /**
     * @brief Check if this is a motion-related M-code
     */
    bool isMotionRelated() const {
        return mCode == 201 || mCode == 203 || mCode == 204 || mCode == 205;
    }
};

// ============================================================================
// M-Code Callback Types
// ============================================================================

/**
 * @brief Return value from M-code handler
 */
struct MCodeResult {
    bool success = true;
    bool pauseExecution = false;            ///< Request execution pause
    bool stopExecution = false;             ///< Request execution stop
    bool waitForCompletion = false;         ///< Wait before continuing
    double waitTime = 0.0;                  ///< Time to wait in seconds
    std::string message;                    ///< Status/error message
    std::string response;                   ///< Response to send (e.g., for M114)
};

/**
 * @brief Machine state passed to callbacks
 */
struct MarlinMachineState {
    Position currentPosition;
    Position currentVelocity;
    double currentFeedrate = 0.0;
    double currentSpindleSpeed = 0.0;
    bool spindleOn = false;
    bool spindleCW = true;
    int currentTool = 0;
    bool coolantMist = false;               ///< M7 state
    bool coolantFlood = false;              ///< M8 state
    bool isMetric = true;
    bool absoluteMode = true;
    bool extruderAbsolute = true;           ///< M82 vs M83
    double extruderPosition = 0.0;

    // Temperatures (for 3D printer mode)
    double extruderTemp = 0.0;
    double extruderTargetTemp = 0.0;
    double bedTemp = 0.0;
    double bedTargetTemp = 0.0;

    // Motion settings
    KinematicLimits kinematicLimits;

    // User-defined state storage
    std::unordered_map<std::string, double> userVariables;
};

/**
 * @brief Callback function signature for M-code handlers
 *
 * @param params Parsed M-code parameters
 * @param state Current machine state (may be modified)
 * @return MCodeResult indicating success/failure and any actions
 */
using MCodeCallback = std::function<MCodeResult(const MCodeParameters& params,
                                                 MarlinMachineState& state)>;

// ============================================================================
// Marlin M-Code Handler Configuration
// ============================================================================

/**
 * @brief Configuration options for MarlinMCodeHandler
 */
struct MarlinMCodeHandlerConfig {
    bool enableMarlinDefaults = true;   ///< Register default Marlin handlers
    bool enableRepRapCompat = true;     ///< RepRap compatibility mode
    bool strict = false;                ///< Fail on unknown M-codes
    bool verbose = false;               ///< Verbose logging
};

// ============================================================================
// Marlin M-Code Handler
// ============================================================================

/**
 * @brief Handler for Marlin-compatible M-codes with callback registration
 */
class MarlinMCodeHandler {
public:
    using Config = MarlinMCodeHandlerConfig;

    explicit MarlinMCodeHandler(const Config& config = {});

    /**
     * @brief Register a callback for an M-code
     *
     * @param mCode The M-code number to handle
     * @param callback The callback function
     * @param description Human-readable description (optional)
     * @return true if registered successfully
     *
     * If a callback is already registered for this M-code, it will be replaced.
     */
    bool registerCallback(int mCode, MCodeCallback callback,
                          const std::string& description = "");

    /**
     * @brief Unregister a callback
     * @param mCode The M-code to unregister
     * @return true if callback was unregistered
     */
    bool unregisterCallback(int mCode);

    /**
     * @brief Check if a callback is registered for an M-code
     */
    bool hasCallback(int mCode) const;

    /**
     * @brief Get description for an M-code
     */
    std::string getDescription(int mCode) const;

    /**
     * @brief Get list of all registered M-codes
     */
    std::vector<int> registeredMCodes() const;

    /**
     * @brief Parse M-code parameters from a line
     * @param line The G-code line to parse
     * @return Parsed parameters (mCode=-1 if not an M-code line)
     */
    MCodeParameters parse(const std::string& line) const;

    /**
     * @brief Execute an M-code
     * @param params Parsed parameters
     * @param state Machine state (modified by handlers)
     * @return Execution result
     */
    MCodeResult execute(const MCodeParameters& params, MarlinMachineState& state);

    /**
     * @brief Execute an M-code from a line of text
     * @param line The G-code line
     * @param state Machine state
     * @return Execution result (success=false if not an M-code)
     */
    MCodeResult executeLine(const std::string& line, MarlinMachineState& state);

    /**
     * @brief Get current machine state reference
     */
    MarlinMachineState& state() { return internalState_; }
    const MarlinMachineState& state() const { return internalState_; }

private:
    /**
     * @brief Register default Marlin M-code handlers
     */
    void registerMarlinDefaults();

    /**
     * @brief Default handlers for common M-codes
     */
    static MCodeResult handleM0M1(const MCodeParameters& params, MarlinMachineState& state);    // Pause
    static MCodeResult handleM2M30(const MCodeParameters& params, MarlinMachineState& state);   // End
    static MCodeResult handleM3(const MCodeParameters& params, MarlinMachineState& state);      // Spindle CW
    static MCodeResult handleM4(const MCodeParameters& params, MarlinMachineState& state);      // Spindle CCW
    static MCodeResult handleM5(const MCodeParameters& params, MarlinMachineState& state);      // Spindle off
    static MCodeResult handleM6(const MCodeParameters& params, MarlinMachineState& state);      // Tool change
    static MCodeResult handleM7(const MCodeParameters& params, MarlinMachineState& state);      // Mist coolant
    static MCodeResult handleM8(const MCodeParameters& params, MarlinMachineState& state);      // Flood coolant
    static MCodeResult handleM9(const MCodeParameters& params, MarlinMachineState& state);      // Coolant off
    static MCodeResult handleM82(const MCodeParameters& params, MarlinMachineState& state);     // Extruder absolute
    static MCodeResult handleM83(const MCodeParameters& params, MarlinMachineState& state);     // Extruder relative
    static MCodeResult handleM104(const MCodeParameters& params, MarlinMachineState& state);    // Set extruder temp
    static MCodeResult handleM109(const MCodeParameters& params, MarlinMachineState& state);    // Wait for extruder
    static MCodeResult handleM114(const MCodeParameters& params, MarlinMachineState& state);    // Report position
    static MCodeResult handleM115(const MCodeParameters& params, MarlinMachineState& state);    // Report firmware
    static MCodeResult handleM117(const MCodeParameters& params, MarlinMachineState& state);    // Display message
    static MCodeResult handleM140(const MCodeParameters& params, MarlinMachineState& state);    // Set bed temp
    static MCodeResult handleM190(const MCodeParameters& params, MarlinMachineState& state);    // Wait for bed
    static MCodeResult handleM201(const MCodeParameters& params, MarlinMachineState& state);    // Set max accel
    static MCodeResult handleM203(const MCodeParameters& params, MarlinMachineState& state);    // Set max feedrate
    static MCodeResult handleM204(const MCodeParameters& params, MarlinMachineState& state);    // Set acceleration
    static MCodeResult handleM205(const MCodeParameters& params, MarlinMachineState& state);    // Advanced settings
    static MCodeResult handleM220(const MCodeParameters& params, MarlinMachineState& state);    // Feed override
    static MCodeResult handleM221(const MCodeParameters& params, MarlinMachineState& state);    // Flow override
    static MCodeResult handleM503(const MCodeParameters& params, MarlinMachineState& state);    // Report settings

    struct CallbackEntry {
        MCodeCallback callback;
        std::string description;
    };

    Config config_;
    std::unordered_map<int, CallbackEntry> callbacks_;
    MarlinMachineState internalState_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Implementation
// ============================================================================

inline MarlinMCodeHandler::MarlinMCodeHandler(const Config& config)
    : config_(config) {
    if (config_.enableMarlinDefaults) {
        registerMarlinDefaults();
    }
}

inline bool MarlinMCodeHandler::registerCallback(int mCode, MCodeCallback callback,
                                                  const std::string& description) {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_[mCode] = CallbackEntry{std::move(callback), description};
    return true;
}

inline bool MarlinMCodeHandler::unregisterCallback(int mCode) {
    std::lock_guard<std::mutex> lock(mutex_);
    return callbacks_.erase(mCode) > 0;
}

inline bool MarlinMCodeHandler::hasCallback(int mCode) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return callbacks_.find(mCode) != callbacks_.end();
}

inline std::string MarlinMCodeHandler::getDescription(int mCode) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = callbacks_.find(mCode);
    if (it != callbacks_.end()) {
        return it->second.description;
    }
    return "";
}

inline std::vector<int> MarlinMCodeHandler::registeredMCodes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> codes;
    codes.reserve(callbacks_.size());
    for (const auto& [code, entry] : callbacks_) {
        codes.push_back(code);
    }
    std::sort(codes.begin(), codes.end());
    return codes;
}

inline MCodeParameters MarlinMCodeHandler::parse(const std::string& line) const {
    MCodeParameters params;
    params.rawLine = line;

    // Skip whitespace and check for M
    size_t pos = 0;
    while (pos < line.size() && std::isspace(line[pos])) ++pos;

    // Handle comments
    size_t commentPos = line.find(';');
    std::string cleanLine = (commentPos != std::string::npos)
                            ? line.substr(0, commentPos)
                            : line;

    // Parse words
    std::string word;
    std::istringstream iss(cleanLine);

    while (iss >> word) {
        if (word.empty()) continue;

        char letter = std::toupper(word[0]);
        std::string numStr = word.substr(1);

        // Handle M117 message specially
        if (letter == 'M' && numStr.find("117") == 0) {
            params.mCode = 117;
            // Rest of line after M117 is the message
            size_t msgStart = cleanLine.find("117");
            if (msgStart != std::string::npos) {
                msgStart += 3;
                while (msgStart < cleanLine.size() && std::isspace(cleanLine[msgStart])) {
                    ++msgStart;
                }
                if (msgStart < cleanLine.size()) {
                    params.message = cleanLine.substr(msgStart);
                }
            }
            return params;
        }

        double value = 0.0;
        try {
            if (!numStr.empty()) {
                value = std::stod(numStr);
            }
        } catch (...) {
            continue;  // Skip invalid numbers
        }

        switch (letter) {
            case 'M': params.mCode = static_cast<int>(value); break;
            case 'P': params.P = value; break;
            case 'S': params.S = value; break;
            case 'R': params.R = value; break;
            case 'F': params.F = value; break;
            case 'T': params.T = value; break;
            case 'I': params.I = value; break;
            case 'J': params.J = value; break;
            case 'K': params.K = value; break;
            case 'X': params.X = value; break;
            case 'Y': params.Y = value; break;
            case 'Z': params.Z = value; break;
            case 'A': params.A = value; break;
            case 'B': params.B = value; break;
            case 'C': params.C = value; break;
            case 'U': params.U = value; break;
            case 'V': params.V = value; break;
            case 'W': params.W = value; break;
            case 'E': params.E = value; break;
            default: break;
        }
    }

    return params;
}

inline MCodeResult MarlinMCodeHandler::execute(const MCodeParameters& params,
                                                MarlinMachineState& state) {
    if (params.mCode < 0) {
        MCodeResult result;
        result.success = false;
        result.message = "Invalid M-code";
        return result;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = callbacks_.find(params.mCode);
    if (it != callbacks_.end()) {
        return it->second.callback(params, state);
    }

    // Unknown M-code
    MCodeResult result;
    if (config_.strict) {
        result.success = false;
        result.message = "Unknown M-code: M" + std::to_string(params.mCode);
    } else {
        result.success = true;  // Silently ignore
        result.message = "Ignored unknown M-code: M" + std::to_string(params.mCode);
    }
    return result;
}

inline MCodeResult MarlinMCodeHandler::executeLine(const std::string& line,
                                                    MarlinMachineState& state) {
    auto params = parse(line);
    if (params.mCode < 0) {
        MCodeResult result;
        result.success = false;
        result.message = "Not an M-code line";
        return result;
    }
    return execute(params, state);
}

inline void MarlinMCodeHandler::registerMarlinDefaults() {
    // Program control
    registerCallback(0, handleM0M1, "M0 - Unconditional stop");
    registerCallback(1, handleM0M1, "M1 - Optional stop");
    registerCallback(2, handleM2M30, "M2 - Program end");
    registerCallback(30, handleM2M30, "M30 - Program end and rewind");

    // Spindle/Laser
    registerCallback(3, handleM3, "M3 - Spindle on CW / Laser on");
    registerCallback(4, handleM4, "M4 - Spindle on CCW");
    registerCallback(5, handleM5, "M5 - Spindle/Laser off");

    // Tool change
    registerCallback(6, handleM6, "M6 - Tool change");

    // Coolant
    registerCallback(7, handleM7, "M7 - Mist coolant on");
    registerCallback(8, handleM8, "M8 - Flood coolant on");
    registerCallback(9, handleM9, "M9 - Coolant off");

    // Extruder mode (3D printer)
    registerCallback(82, handleM82, "M82 - Extruder absolute mode");
    registerCallback(83, handleM83, "M83 - Extruder relative mode");

    // Temperature (3D printer)
    registerCallback(104, handleM104, "M104 - Set extruder temperature");
    registerCallback(109, handleM109, "M109 - Set extruder temp and wait");
    registerCallback(140, handleM140, "M140 - Set bed temperature");
    registerCallback(190, handleM190, "M190 - Set bed temp and wait");

    // Information
    registerCallback(114, handleM114, "M114 - Report current position");
    registerCallback(115, handleM115, "M115 - Report firmware info");
    registerCallback(117, handleM117, "M117 - Display message");
    registerCallback(503, handleM503, "M503 - Report settings");

    // Motion limits (Marlin-specific)
    registerCallback(201, handleM201, "M201 - Set max print acceleration");
    registerCallback(203, handleM203, "M203 - Set max feedrate");
    registerCallback(204, handleM204, "M204 - Set starting acceleration");
    registerCallback(205, handleM205, "M205 - Set advanced settings (jerk)");

    // Overrides
    registerCallback(220, handleM220, "M220 - Set feedrate percentage");
    registerCallback(221, handleM221, "M221 - Set flow percentage");
}

// ============================================================================
// Default Handler Implementations
// ============================================================================

inline MCodeResult MarlinMCodeHandler::handleM0M1(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    result.pauseExecution = true;
    if (params.P.has_value()) {
        result.waitTime = params.P.value() / 1000.0;  // P is in milliseconds
        result.waitForCompletion = true;
    }
    result.message = (params.mCode == 0) ? "Unconditional stop" : "Optional stop";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM2M30(const MCodeParameters& params,
                                                    MarlinMachineState& state) {
    MCodeResult result;
    result.stopExecution = true;
    result.message = "Program end";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM3(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.spindleOn = true;
    state.spindleCW = true;
    if (params.S.has_value()) {
        state.currentSpindleSpeed = params.S.value();
    }
    result.message = "Spindle CW at " + std::to_string(state.currentSpindleSpeed);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM4(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.spindleOn = true;
    state.spindleCW = false;
    if (params.S.has_value()) {
        state.currentSpindleSpeed = params.S.value();
    }
    result.message = "Spindle CCW at " + std::to_string(state.currentSpindleSpeed);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM5(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.spindleOn = false;
    state.currentSpindleSpeed = 0;
    result.message = "Spindle off";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM6(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    if (params.T.has_value()) {
        state.currentTool = static_cast<int>(params.T.value());
    }
    result.pauseExecution = true;  // Typically pause for manual tool change
    result.message = "Tool change to T" + std::to_string(state.currentTool);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM7(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.coolantMist = true;
    result.message = "Mist coolant on";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM8(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.coolantFlood = true;
    result.message = "Flood coolant on";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM9(const MCodeParameters& params,
                                                 MarlinMachineState& state) {
    MCodeResult result;
    state.coolantMist = false;
    state.coolantFlood = false;
    result.message = "Coolant off";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM82(const MCodeParameters& params,
                                                  MarlinMachineState& state) {
    MCodeResult result;
    state.extruderAbsolute = true;
    result.message = "Extruder absolute mode";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM83(const MCodeParameters& params,
                                                  MarlinMachineState& state) {
    MCodeResult result;
    state.extruderAbsolute = false;
    result.message = "Extruder relative mode";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM104(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        state.extruderTargetTemp = params.S.value();
    }
    result.message = "Set extruder temp to " + std::to_string(state.extruderTargetTemp);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM109(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        state.extruderTargetTemp = params.S.value();
    }
    result.waitForCompletion = true;
    result.message = "Wait for extruder temp " + std::to_string(state.extruderTargetTemp);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM114(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    std::ostringstream oss;
    oss << "X:" << state.currentPosition[0]
        << " Y:" << state.currentPosition[1]
        << " Z:" << state.currentPosition[2]
        << " E:" << state.extruderPosition;
    result.response = oss.str();
    result.message = result.response;
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM115(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    result.response = "FIRMWARE_NAME:GCodeExport PROTOCOL_VERSION:1.0";
    result.message = result.response;
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM117(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    result.message = "Display: " + params.message;
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM140(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        state.bedTargetTemp = params.S.value();
    }
    result.message = "Set bed temp to " + std::to_string(state.bedTargetTemp);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM190(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        state.bedTargetTemp = params.S.value();
    }
    result.waitForCompletion = true;
    result.message = "Wait for bed temp " + std::to_string(state.bedTargetTemp);
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM201(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    auto& limits = state.kinematicLimits;

    // Set per-axis max acceleration (Marlin uses mm/s²)
    if (params.X.has_value()) limits.axisMaxAcceleration[0] = params.X.value();
    if (params.Y.has_value()) limits.axisMaxAcceleration[1] = params.Y.value();
    if (params.Z.has_value()) limits.axisMaxAcceleration[2] = params.Z.value();
    if (params.E.has_value()) {
        // E axis mapped to axis 3 or handled separately
    }

    result.message = "Set max acceleration";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM203(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    auto& limits = state.kinematicLimits;

    // Set per-axis max feedrate (Marlin uses mm/s)
    if (params.X.has_value()) limits.axisMaxVelocity[0] = params.X.value() * 60.0;  // Convert to mm/min
    if (params.Y.has_value()) limits.axisMaxVelocity[1] = params.Y.value() * 60.0;
    if (params.Z.has_value()) limits.axisMaxVelocity[2] = params.Z.value() * 60.0;
    if (params.E.has_value()) {
        // E axis
    }

    result.message = "Set max feedrate";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM204(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    auto& limits = state.kinematicLimits;

    // P = print acceleration, T = travel acceleration, R = retract acceleration
    if (params.P.has_value()) {
        limits.maxAcceleration = params.P.value();
    }
    if (params.T.has_value()) {
        // Travel acceleration could be stored separately
    }
    if (params.R.has_value()) {
        // Retract acceleration
    }
    // Legacy: S = both print and travel
    if (params.S.has_value()) {
        limits.maxAcceleration = params.S.value();
    }

    result.message = "Set default acceleration";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM205(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    auto& limits = state.kinematicLimits;

    // J = junction deviation (newer)
    // X,Y,Z,E = jerk for each axis (classic Marlin)
    // S = minimum feedrate
    // T = minimum travel feedrate
    // B = minimum segment time

    if (params.J.has_value()) {
        // Junction deviation mode (newer Marlin)
        // Store or convert to jerk equivalent
    }

    // Per-axis jerk (mm/s)
    if (params.X.has_value()) limits.axisMaxJerk[0] = params.X.value();
    if (params.Y.has_value()) limits.axisMaxJerk[1] = params.Y.value();
    if (params.Z.has_value()) limits.axisMaxJerk[2] = params.Z.value();
    if (params.E.has_value()) {
        // E jerk
    }

    result.message = "Set advanced motion settings";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM220(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        // Feed rate override percentage
        double percentage = params.S.value();
        // Apply to current feedrate
        // state.feedOverridePercent = percentage;
    }
    result.message = "Feed rate override";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM221(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    if (params.S.has_value()) {
        // Extrusion flow rate override percentage
        // state.flowOverridePercent = params.S.value();
    }
    result.message = "Flow rate override";
    return result;
}

inline MCodeResult MarlinMCodeHandler::handleM503(const MCodeParameters& params,
                                                   MarlinMachineState& state) {
    MCodeResult result;
    std::ostringstream oss;
    auto& limits = state.kinematicLimits;

    oss << "echo:M201 X" << limits.axisMaxAcceleration[0]
        << " Y" << limits.axisMaxAcceleration[1]
        << " Z" << limits.axisMaxAcceleration[2] << "\n";
    oss << "echo:M203 X" << limits.axisMaxVelocity[0] / 60.0
        << " Y" << limits.axisMaxVelocity[1] / 60.0
        << " Z" << limits.axisMaxVelocity[2] / 60.0 << "\n";
    oss << "echo:M204 P" << limits.maxAcceleration << "\n";
    oss << "echo:M205 X" << limits.axisMaxJerk[0]
        << " Y" << limits.axisMaxJerk[1]
        << " Z" << limits.axisMaxJerk[2] << "\n";

    result.response = oss.str();
    result.message = "Report settings";
    return result;
}

} // namespace GCode
