/**
 * @file GCodeTypes.hpp
 * @brief Core G-Code Type Definitions and Enumerations
 * 
 * @details
 * This file defines the fundamental types used throughout the G-Code interpreter.
 * It provides a complete taxonomy of G-codes, M-codes, and auxiliary codes
 * conforming to RS274/NGC specification (LinuxCNC compatible).
 * 
 * ## G-Code Classification
 * 
 * G-codes are organized into modal groups, where only one code from each
 * group can be active at a time:
 * 
 * | Group | Name | Codes |
 * |-------|------|-------|
 * | 0 | Non-modal | G4, G10, G28, G30, G53, G92, G92.1, G92.2, G92.3 |
 * | 1 | Motion | G0, G1, G2, G3, G33, G38.x, G73, G76, G80-G89 |
 * | 2 | Plane | G17, G18, G19, G17.1, G18.1, G19.1 |
 * | 3 | Distance | G90, G91 |
 * | 4 | Arc Distance | G90.1, G91.1 |
 * | 5 | Feed Mode | G93, G94, G95 |
 * | 6 | Units | G20, G21 |
 * | 7 | Cutter Comp | G40, G41, G42, G41.1, G42.1 |
 * | 8 | Tool Length | G43, G43.1, G43.2, G49 |
 * | 10 | Canned Return | G98, G99 |
 * | 12 | Coord System | G54-G59.3 |
 * | 13 | Path Control | G61, G61.1, G64 |
 * | 14 | Spindle Mode | G96, G97 |
 * | 15 | Lathe Diam | G7, G8 |
 * 
 * ## Design Philosophy
 * 
 * 1. **Type Safety**: Strong typing prevents mixing incompatible values
 * 2. **Extensibility**: Easy to add custom G/M codes
 * 3. **Memory Efficiency**: Compact representations for embedded systems
 * 4. **LinuxCNC Compatibility**: Full RS274/NGC specification support
 * 
 * @see GCodeParser
 * @see GCodeInterpreter
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <array>
#include <variant>
#include <optional>
#include <functional>
#include <string>
#include <map>
#include <vector>
#include <charconv> // for from_chars parsing
#include <sstream> // string parsing utilities

namespace GCode {

// ============================================================================
// Configuration Constants
// ============================================================================

/// Maximum number of axes supported (XYZABCUVW)
constexpr size_t MAX_AXES = 9;

/// Maximum number of coordinate systems (G54-G59.3)
constexpr size_t MAX_COORD_SYSTEMS = 9;

/// Maximum number of tool table entries
constexpr size_t MAX_TOOLS = 256;

/// Maximum subroutine nesting depth
constexpr size_t MAX_CALL_DEPTH = 10;

/// Maximum number of local parameters per call level (#1-#30)
constexpr size_t MAX_LOCAL_PARAMS = 30;

/// Maximum number of global parameters (#31-#5999)
constexpr size_t MAX_GLOBAL_PARAMS = 6000;

/// Maximum lookahead buffer size (blocks)
constexpr size_t MAX_LOOKAHEAD = 50;

/// Maximum lookbehind buffer for reverse parsing
constexpr size_t MAX_LOOKBEHIND = 20;

/// Tolerance for arc radius matching (mm)
constexpr double ARC_RADIUS_TOLERANCE = 0.005;

/// Tolerance for comparing floating point values
constexpr double EPSILON = 1e-9;

// ============================================================================
// Axis Definitions
// ============================================================================

/**
 * @brief Axis indices for position arrays
 */
enum class Axis : uint8_t {
    X = 0,  ///< Primary linear axis
    Y = 1,  ///< Secondary linear axis
    Z = 2,  ///< Tertiary linear axis (spindle axis)
    A = 3,  ///< Rotary axis around X
    B = 4,  ///< Rotary axis around Y
    C = 5,  ///< Rotary axis around Z
    U = 6,  ///< Secondary X linear axis
    V = 7,  ///< Secondary Y linear axis
    W = 8,  ///< Secondary Z linear axis
    COUNT = 9
};

/**
 * @brief Compact position vector for all axes
 */
struct Position {
    std::array<double, MAX_AXES> coords{};
    
    double& operator[](Axis a) { return coords[static_cast<size_t>(a)]; }
    double operator[](Axis a) const { return coords[static_cast<size_t>(a)]; }
    double& operator[](size_t i) { return coords[i]; }
    double operator[](size_t i) const { return coords[i]; }
    
    double& x() { return coords[0]; }
    double& y() { return coords[1]; }
    double& z() { return coords[2]; }
    double x() const { return coords[0]; }
    double y() const { return coords[1]; }
    double z() const { return coords[2]; }
    
    Position operator+(const Position& other) const {
        Position result;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            result.coords[i] = coords[i] + other.coords[i];
        }
        return result;
    }
    
    Position operator-(const Position& other) const {
        Position result;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            result.coords[i] = coords[i] - other.coords[i];
        }
        return result;
    }
    
    Position operator*(double scalar) const {
        Position result;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            result.coords[i] = coords[i] * scalar;
        }
        return result;
    }
    
    double dot(const Position& other) const {
        double sum = 0;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            sum += coords[i] * other.coords[i];
        }
        return sum;
    }
    
    double magnitude() const {
        return std::sqrt(dot(*this));
    }
    
    Position normalized() const {
        double mag = magnitude();
        if (mag < EPSILON) return *this;
        return *this * (1.0 / mag);
    }
    
    /// Distance considering only linear axes
    double linearDistance(const Position& other) const {
        double dx = x() - other.x();
        double dy = y() - other.y();
        double dz = z() - other.z();
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

// ============================================================================
// Modal Group Definitions
// ============================================================================

/**
 * @brief Motion mode (Modal Group 1)
 */
enum class MotionMode : uint16_t {
    RAPID = 0,           ///< G0 - Rapid positioning
    LINEAR = 1,          ///< G1 - Linear interpolation
    CW_ARC = 2,          ///< G2 - Clockwise arc
    CCW_ARC = 3,         ///< G3 - Counter-clockwise arc
    DWELL = 4,           ///< G4 - Dwell (non-modal technically)
    CUBIC_SPLINE = 5,    ///< G5 - Cubic spline
    QUADRATIC_SPLINE = 51, ///< G5.1 - Quadratic B-spline
    NURBS = 52,          ///< G5.2/G5.3 - NURBS block
    THREADING = 33,      ///< G33 - Spindle synchronized
    RIGID_TAP = 331,     ///< G33.1 - Rigid tapping
    PROBE_TOWARD = 382,  ///< G38.2 - Probe toward, error on fail
    PROBE_TOWARD_NE = 383, ///< G38.3 - Probe toward, no error
    PROBE_AWAY = 384,    ///< G38.4 - Probe away, error on fail
    PROBE_AWAY_NE = 385, ///< G38.5 - Probe away, no error
    DRILL_PECK_BREAK = 73, ///< G73 - Peck drill with chip break
    THREAD_CYCLE = 76,   ///< G76 - Threading cycle
    CANNED_OFF = 80,     ///< G80 - Cancel canned cycle
    DRILL = 81,          ///< G81 - Drilling cycle
    DRILL_DWELL = 82,    ///< G82 - Drill with dwell
    DRILL_PECK = 83,     ///< G83 - Peck drilling
    TAP_RH = 84,         ///< G84 - Right-hand tapping
    BORE_FEED_OUT = 85,  ///< G85 - Boring, feed out
    BORE_STOP_RAPID = 86, ///< G86 - Boring, spindle stop, rapid out
    BORE_BACK = 87,      ///< G87 - Back boring
    BORE_MANUAL = 88,    ///< G88 - Boring, manual retract
    BORE_DWELL = 89,     ///< G89 - Boring with dwell
    TAP_LH = 74,         ///< G74 - Left-hand tapping
};

/**
 * @brief Plane selection (Modal Group 2)
 */
enum class Plane : uint8_t {
    XY = 17,      ///< G17 - XY plane (Z perpendicular)
    ZX = 18,      ///< G18 - ZX plane (Y perpendicular)
    YZ = 19,      ///< G19 - YZ plane (X perpendicular)
    UV = 171,     ///< G17.1 - UV plane
    WU = 181,     ///< G18.1 - WU plane
    VW = 191,     ///< G19.1 - VW plane
};

/**
 * @brief Distance mode (Modal Group 3)
 */
enum class DistanceMode : uint8_t {
    ABSOLUTE = 90,       ///< G90 - Absolute coordinates
    INCREMENTAL = 91,    ///< G91 - Incremental coordinates
};

/**
 * @brief Arc distance mode (Modal Group 4)
 */
enum class ArcDistanceMode : uint16_t {
    ABSOLUTE = 901,      ///< G90.1 - Absolute arc centers
    INCREMENTAL = 911,   ///< G91.1 - Incremental arc centers (default)
};

/**
 * @brief Feed rate mode (Modal Group 5)
 */
enum class FeedMode : uint8_t {
    INVERSE_TIME = 93,   ///< G93 - Inverse time mode
    UNITS_PER_MIN = 94,  ///< G94 - Units per minute (default)
    UNITS_PER_REV = 95,  ///< G95 - Units per revolution
};

/**
 * @brief Units (Modal Group 6)
 */
enum class Units : uint8_t {
    INCH = 20,           ///< G20 - Inches
    MM = 21,             ///< G21 - Millimeters
};

/**
 * @brief Cutter compensation mode (Modal Group 7)
 */
enum class CutterCompMode : uint16_t {
    OFF = 40,            ///< G40 - Compensation off
    LEFT = 41,           ///< G41 - Left of path
    RIGHT = 42,          ///< G42 - Right of path
    LEFT_DYNAMIC = 411,  ///< G41.1 - Dynamic left
    RIGHT_DYNAMIC = 421, ///< G42.1 - Dynamic right
};

/**
 * @brief Tool length offset mode (Modal Group 8)
 */
enum class ToolLengthMode : uint16_t {
    OFF = 49,            ///< G49 - Cancel tool length offset
    POSITIVE = 43,       ///< G43 - Apply from tool table
    DYNAMIC = 431,       ///< G43.1 - Dynamic offset
    ADDITIONAL = 432,    ///< G43.2 - Additional offset
};
/**
 * @brief Canned cycle return mode (Modal Group 10)
 */
enum class CannedReturnMode : uint8_t {
    INITIAL = 98,        ///< G98 - Return to initial Z
    R_PLANE = 99,        ///< G99 - Return to R plane
};

/**
 * @brief Active coordinate system (Modal Group 12)
 */
enum class CoordSystem : uint8_t {
    G54 = 1,   ///< Work coordinate system 1
    G55 = 2,   ///< Work coordinate system 2
    G56 = 3,   ///< Work coordinate system 3
    G57 = 4,   ///< Work coordinate system 4
    G58 = 5,   ///< Work coordinate system 5
    G59 = 6,   ///< Work coordinate system 6
    G59_1 = 7, ///< Work coordinate system 7
    G59_2 = 8, ///< Work coordinate system 8
    G59_3 = 9, ///< Work coordinate system 9
};

/**
 * @brief Path control mode (Modal Group 13)
 */
enum class PathMode : uint16_t {
    EXACT_PATH = 61,     ///< G61 - Exact path mode
    EXACT_STOP = 611,    ///< G61.1 - Exact stop mode
    BLEND = 64,          ///< G64 - Path blending
};

/**
 * @brief Spindle speed mode (Modal Group 14)
 */
enum class SpindleMode : uint8_t {
    CSS = 96,            ///< G96 - Constant surface speed
    RPM = 97,            ///< G97 - RPM mode (default)
};

/**
 * @brief Lathe diameter/radius mode (Modal Group 15)
 */
enum class LatheMode : uint8_t {
    DIAMETER = 7,        ///< G7 - Diameter mode
    RADIUS = 8,          ///< G8 - Radius mode (default)
};

// ============================================================================
// M-Code Definitions
// ============================================================================

/**
 * @brief Standard M-codes
 */
enum class MCode : uint16_t {
    // Program control
    PAUSE = 0,           ///< M0 - Program pause
    OPTIONAL_PAUSE = 1,  ///< M1 - Optional pause
    END = 2,             ///< M2 - Program end
    END_EXCHANGE = 30,   ///< M30 - End and exchange
    PALLET_PAUSE = 60,   ///< M60 - Pallet change pause
    
    // Spindle control
    SPINDLE_CW = 3,      ///< M3 - Spindle clockwise
    SPINDLE_CCW = 4,     ///< M4 - Spindle counter-clockwise
    SPINDLE_STOP = 5,    ///< M5 - Spindle stop
    SPINDLE_ORIENT = 19, ///< M19 - Spindle orient
    
    // Tool change
    TOOL_CHANGE = 6,     ///< M6 - Tool change
    SET_TOOL = 61,       ///< M61 - Set current tool number
    
    // Coolant
    COOLANT_MIST = 7,    ///< M7 - Mist coolant on
    COOLANT_FLOOD = 8,   ///< M8 - Flood coolant on
    COOLANT_OFF = 9,     ///< M9 - Coolant off
    
    // Feed/Speed override
    OVERRIDE_ON = 48,    ///< M48 - Enable overrides
    OVERRIDE_OFF = 49,   ///< M49 - Disable overrides
    FEED_OVERRIDE = 50,  ///< M50 - Feed override control
    SPINDLE_OVERRIDE = 51, ///< M51 - Spindle override control
    ADAPTIVE_FEED = 52,  ///< M52 - Adaptive feed control
    FEED_STOP = 53,      ///< M53 - Feed stop control
    
    // Digital I/O
    DOUT_SYNC_ON = 62,   ///< M62 - Digital out sync on
    DOUT_SYNC_OFF = 63,  ///< M63 - Digital out sync off
    DOUT_IMMED_ON = 64,  ///< M64 - Digital out immediate on
    DOUT_IMMED_OFF = 65, ///< M65 - Digital out immediate off
    WAIT_INPUT = 66,     ///< M66 - Wait on input
    AOUT_SYNC = 67,      ///< M67 - Analog out synchronized
    AOUT_IMMED = 68,     ///< M68 - Analog out immediate
    
    // Modal state save/restore
    SAVE_MODAL = 70,     ///< M70 - Save modal state
    INVALIDATE_MODAL = 71, ///< M71 - Invalidate saved modal
    RESTORE_MODAL = 72,  ///< M72 - Restore modal state
    AUTO_RESTORE = 73,   ///< M73 - Save and autorestore
    
    // Fanuc-style subroutines
    FANUC_CALL = 98,     ///< M98 - Call subprogram
    FANUC_RETURN = 99,   ///< M99 - Return from subprogram
    
    // User-defined M-codes (M100-M199)
    USER_BASE = 100,     ///< Start of user M-codes
    USER_END = 199,      ///< End of user M-codes
};

// ============================================================================
// O-Code Control Flow
// ============================================================================

/**
 * @brief O-code types for control flow
 */
enum class OCodeType : uint8_t {
    SUB,        ///< o<name> sub - Begin subroutine
    ENDSUB,     ///< o<name> endsub [return_value] - End subroutine
    CALL,       ///< o<name> call [args...] - Call subroutine
    RETURN,     ///< o<name> return [value] - Return from subroutine
    IF,         ///< o<num> if [condition]
    ELSEIF,     ///< o<num> elseif [condition]
    ELSE,       ///< o<num> else
    ENDIF,      ///< o<num> endif
    WHILE,      ///< o<num> while [condition]
    ENDWHILE,   ///< o<num> endwhile
    DO,         ///< o<num> do
    REPEAT,     ///< o<num> repeat [count]
    ENDREPEAT,  ///< o<num> endrepeat
    BREAK,      ///< o<num> break
    CONTINUE,   ///< o<num> continue
};

// ============================================================================
// Word Types
// ============================================================================

/**
 * @brief All possible word letters in G-code
 */
enum class WordLetter : uint8_t {
    A, B, C,    // Rotary axes
    D,          // Tool diameter/compensation number
    E,          // Analog/NURBS parameter
    F,          // Feed rate
    G,          // G-code
    H,          // Tool length offset number
    I, J, K,    // Arc centers / spline parameters
    L,          // Loop count / parameter
    M,          // M-code
    N,          // Line number
    O,          // O-code (subroutine number)
    P,          // Dwell time / parameter
    Q,          // Peck depth / parameter
    R,          // Arc radius / retract plane
    S,          // Spindle speed
    T,          // Tool number
    U, V, W,    // Secondary linear axes
    X, Y, Z,    // Primary linear axes
    DOLLAR,     // $ - Spindle selector
    COMMENT,    // ( ) or ; comment
    PERCENT,    // % program delimiter
    INVALID
};

/**
 * @brief A single parsed word (letter + value)
 */
struct Word {
    WordLetter letter{WordLetter::INVALID};
    double value{0.0};
    bool present{false};
    
    bool isAxis() const {
        return letter >= WordLetter::A && letter <= WordLetter::C ||
               letter >= WordLetter::U && letter <= WordLetter::Z;
    }
    
    bool isLinearAxis() const {
        return letter == WordLetter::X || letter == WordLetter::Y || 
               letter == WordLetter::Z || letter == WordLetter::U ||
               letter == WordLetter::V || letter == WordLetter::W;
    }
    
    bool isRotaryAxis() const {
        return letter == WordLetter::A || letter == WordLetter::B ||
               letter == WordLetter::C;
    }
};

// ============================================================================
// Block Structure
// ============================================================================

/**
 * @brief A complete parsed G-code block (one line)
 */
struct Block {
    /// Line number (N word), -1 if not present
    int32_t lineNumber{-1};
    
    /// Block delete flag (starts with /)
    bool blockDelete{false};
    
    /// All words present in this block
    std::array<Word, 26> words{};  // A-Z storage
    
    /// G-codes in this block (multiple allowed)
    std::array<int16_t, 4> gCodes{-1, -1, -1, -1};
    uint8_t gCodeCount{0};
    
    /// M-codes in this block (multiple allowed) 
    std::array<int16_t, 4> mCodes{-1, -1, -1, -1};
    uint8_t mCodeCount{0};
    
    /// O-code if present
    bool hasOCode{false};
    OCodeType oCodeType{OCodeType::SUB};
    int32_t oCodeNumber{-1};
    std::array<char, 64> oCodeName{};  // Named subroutine
    bool oCodeIsNamed{false};
    
    /// Comment text
    std::array<char, 128> comment{};
    bool hasComment{false};
    
    /// Original line text for error messages
    std::array<char, 256> originalText{};
    uint32_t sourceLineNumber{0};  // Actual file line
    
    /// Collected KEY=VALUE parameters found in the block (e.g., FOO=1)
    std::map<std::string, std::string> keyValues{};

    /// Helper to check if a word is present
    bool hasWord(WordLetter letter) const {
        return words[static_cast<size_t>(letter)].present;
    }

    /// Helper to get word value
    double getWord(WordLetter letter, double defaultVal = 0.0) const {
        const auto& w = words[static_cast<size_t>(letter)];
        return w.present ? w.value : defaultVal;
    }

    /// Check if block has a key/value parameter
    bool hasKeyValue(const std::string& key) const {
        return keyValues.find(key) != keyValues.end();
    }

    /// Get raw parameter string (if present) or default
    std::string getParamString(const std::string& key, const std::string& defaultVal = "") const {
        auto it = keyValues.find(key);
        return it != keyValues.end() ? it->second : defaultVal;
    }

    /// Convenience: parse an integer parameter; return default on parse failure
    int32_t getParamInt(const std::string& key, int32_t defaultVal = 0) const {
        auto it = keyValues.find(key);
        if (it == keyValues.end()) return defaultVal;
        std::string s = it->second;
        // trim whitespace
        auto trim = [](std::string& str) {
            size_t a = 0;
            while (a < str.size() && std::isspace(static_cast<unsigned char>(str[a]))) ++a;
            size_t b = str.size();
            while (b > a && std::isspace(static_cast<unsigned char>(str[b-1]))) --b;
            str = str.substr(a, b - a);
        };
        trim(s);
        if (s.empty()) return defaultVal;
        int val = 0;
        auto r = std::from_chars(s.data(), s.data() + s.size(), val);
        // success only if no error and entire string consumed
        if (r.ec == std::errc() && r.ptr == s.data() + s.size()) {
            return static_cast<int32_t>(val);
        }
        return defaultVal;
    }

    /// Convenience: parse a double parameter; return default on parse failure
    double getParamDouble(const std::string& key, double defaultVal = 0.0) const {
        auto it = keyValues.find(key);
        if (it == keyValues.end()) return defaultVal;
        std::string s = it->second;
        auto trim = [](std::string& str) {
            size_t a = 0;
            while (a < str.size() && std::isspace(static_cast<unsigned char>(str[a]))) ++a;
            size_t b = str.size();
            while (b > a && std::isspace(static_cast<unsigned char>(str[b-1]))) --b;
            str = str.substr(a, b - a);
        };
        trim(s);
        if (s.empty()) return defaultVal;
        double val = 0.0;
        auto r = std::from_chars(s.data(), s.data() + s.size(), val);
        // success only if no error and entire string consumed
        if (r.ec == std::errc() && r.ptr == s.data() + s.size()) {
            return val;
        }
        return defaultVal;
    }

    /// Convenience: parse a vector of doubles from a param value. Supports brackets [1,2,3],
    /// parentheses (1 2 3) or comma-separated unbracketed values. Returns empty vector on parse failure.
    std::vector<double> getParamVector(const std::string& key) const {
        std::vector<double> out;
        auto it = keyValues.find(key);
        if (it == keyValues.end()) return out;
        std::string s = it->second;
        if (s.empty()) return out;
        // strip surrounding [ ] or ( )
        if ((s.front() == '[' && s.back() == ']') || (s.front() == '(' && s.back() == ')')) {
            s = s.substr(1, s.size() - 2);
        }
        // split on commas or whitespace
        std::stringstream ss(s);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // token may still contain whitespace-separated numbers
            std::stringstream inner(token);
            std::string t2;
            while (inner >> t2) {
                try {
                    size_t idx = 0;
                    double v = std::stod(t2, &idx);
                    if (idx == 0) continue;
                    out.push_back(v);
                } catch (...) {
                    // skip unparsable tokens
                }
            }
        }
        return out;
    }
    
    /// Check if block contains motion
    bool hasMotion() const {
        return hasWord(WordLetter::X) || hasWord(WordLetter::Y) ||
               hasWord(WordLetter::Z) || hasWord(WordLetter::A) ||
               hasWord(WordLetter::B) || hasWord(WordLetter::C) ||
               hasWord(WordLetter::U) || hasWord(WordLetter::V) ||
               hasWord(WordLetter::W);
    }
    
    /// Check if a G-code is present in this block
    bool hasGCode(int16_t code) const {
        for (uint8_t i = 0; i < gCodeCount; ++i) {
            if (gCodes[i] == code) return true;
        }
        return false;
    }
    
    /// Check if a G-code (with decimal) is present (e.g., 382 for G38.2)
    bool hasGCodeDecimal(double code) const {
        int16_t intCode = static_cast<int16_t>(code * 10);
        for (uint8_t i = 0; i < gCodeCount; ++i) {
            if (gCodes[i] == intCode) return true;
        }
        return false;
    }
    
    /// Check if an M-code is present in this block
    bool hasMCode(int16_t code) const {
        for (uint8_t i = 0; i < mCodeCount; ++i) {
            if (mCodes[i] == code) return true;
        }
        return false;
    }
    
    /// Check if this is an O-code block
    bool isOCode() const {
        return hasOCode;
    }
};

// ============================================================================
// Tool Definitions
// ============================================================================

/**
 * @brief Tool table entry
 */
struct Tool {
    int32_t number{0};          ///< Tool number (T word)
    int32_t pocket{0};          ///< Pocket number for random TC
    
    Position offset{};          ///< Tool length offsets
    double diameter{0.0};       ///< Tool diameter
    double radius{0.0};         ///< Tool radius (diameter/2 or explicit)
    
    // Lathe-specific
    double frontAngle{0.0};     ///< Front angle (I word in G10 L1)
    double backAngle{0.0};      ///< Back angle (J word in G10 L1)
    uint8_t orientation{0};     ///< Tool orientation (Q word, 0-9)
    
    bool valid{false};
};

// ============================================================================
// Probe Results
// ============================================================================

/**
 * @brief Probe result (canonical shared definition)
 */
struct ProbeResult {
    /// Was probe tripped?
    bool tripped{false};

    /// Backwards-compatible success flag (true if tripped)
    bool success{false};

    /// Position at trip (machine coordinates)
    Position tripPosition;

    /// Position at trip (work coordinates)
    Position tripWorkPosition;

    /// Distance traveled
    double travelDistance{0};

    /// Timestamp of trip
    double tripTime{0};
};

#define GCODE_PROBE_RESULT_DEFINED

// ============================================================================
// Arc Parameters
// ============================================================================

/**
 * @brief Arc parameters after parsing/calculation
 */
struct ArcParams {
    Position center{};          ///< Arc center point
    Position startPoint{};      ///< Arc start point
    Position endPoint{};        ///< Arc end point
    double radius{0.0};         ///< Arc radius
    double startAngle{0.0};     ///< Start angle (radians)
    double endAngle{0.0};       ///< End angle (radians)
    double sweepAngle{0.0};     ///< Total sweep (radians)
    int32_t turns{1};           ///< Number of full turns (P word)
    double helixDelta{0.0};     ///< Helix height change
    bool clockwise{true};       ///< Direction
    Plane plane{Plane::XY};     ///< Arc plane
    bool valid{false};
};

// ============================================================================
// Spline Parameters
// ============================================================================

/**
 * @brief Cubic spline parameters (G5)
 */
struct CubicSplineParams {
    Position p0;                ///< Start point
    Position p1;                ///< First control point
    Position p2;                ///< Second control point
    Position p3;                ///< End point
    bool valid{false};
};

/**
 * @brief Quadratic spline parameters (G5.1)
 */
struct QuadSplineParams {
    Position p0;                ///< Start point
    Position p1;                ///< Control point
    Position p2;                ///< End point
    bool valid{false};
};

/**
 * @brief NURBS control point (G5.2)
 */
struct NurbsControlPoint {
    Position point{};
    double weight{1.0};
};

/**
 * @brief NURBS parameters (G5.2/G5.3)
 */
struct NurbsParams {
    std::array<NurbsControlPoint, 32> controlPoints{};
    size_t numPoints{0};
    uint8_t order{3};           ///< Curve order (L word)
    bool valid{false};
};

// ============================================================================
// Canned Cycle Parameters
// ============================================================================

/**
 * @brief Canned cycle parameters
 */
struct CannedCycleParams {
    MotionMode cycle{MotionMode::CANNED_OFF};
    double retractPlane{0.0};   ///< R - Retract/reference plane
    double depth{0.0};          ///< Z (or depth axis) target
    double clearPlane{0.0};     ///< Initial Z / clear height
    double feedRate{0.0};       ///< F - Feed rate
    double dwell{0.0};          ///< P - Dwell time (seconds)
    double peckDepth{0.0};      ///< Q - Peck increment
    int32_t repeatCount{1};     ///< L - Number of repeats
    bool useInitialZ{true};     ///< G98 vs G99
    
    // Tapping specific
    double pitch{0.0};          ///< Thread pitch for G33.1
    double retractMult{1.0};    ///< I - Retract speed multiplier
    
    // Threading specific (G76)
    double threadPitch{0.0};    ///< P - Thread pitch
    double threadDepth{0.0};    ///< K - Full thread depth
    double firstCut{0.0};       ///< J - First cut depth
    double peakOffset{0.0};     ///< I - Thread peak offset
    double degression{1.0};     ///< R - Depth degression
    double compoundAngle{0.0};  ///< Q - Compound slide angle
    int32_t springPasses{0};    ///< H - Spring passes
    double taperLength{0.0};    ///< E - Taper length
    uint8_t taperFlags{0};      ///< L - Entry/exit taper flags
};

#define GCODE_CANNED_CYCLE_PARAMS_DEFINED

// ============================================================================
// Machine State
// ============================================================================

/**
 * @brief Complete interpreter state
 */
struct MachineState {
    // Current position
    Position machinePosition{};  ///< Machine coordinates
    Position workPosition{};     ///< Work coordinates
    
    // Modal groups
    MotionMode motionMode{MotionMode::RAPID};
    Plane plane{Plane::XY};
    DistanceMode distanceMode{DistanceMode::ABSOLUTE};
    ArcDistanceMode arcDistanceMode{ArcDistanceMode::INCREMENTAL};
    FeedMode feedMode{FeedMode::UNITS_PER_MIN};
    Units units{Units::MM};
    CutterCompMode cutterComp{CutterCompMode::OFF};
    ToolLengthMode toolLengthMode{ToolLengthMode::OFF};
    CannedReturnMode cannedReturn{CannedReturnMode::INITIAL};
    CoordSystem coordSystem{CoordSystem::G54};
    PathMode pathMode{PathMode::BLEND};
    SpindleMode spindleMode{SpindleMode::RPM};
    LatheMode latheMode{LatheMode::RADIUS};
    
    // Feed and speed
    double feedRate{0.0};        ///< F word value
    double spindleSpeed{0.0};    ///< S word value
    bool spindleCW{true};        ///< Spindle direction
    bool spindleOn{false};       ///< Spindle running
    double maxSpindleSpeed{0.0}; ///< D word for CSS
    
    // Tool
    int32_t currentTool{0};      ///< Active tool number
    int32_t selectedTool{0};     ///< Next tool (T word)
    Position toolOffset{};       ///< Active tool length offset
    double cutterRadius{0.0};    ///< Active cutter radius
    
    // Coordinate system offsets
    std::array<Position, MAX_COORD_SYSTEMS> coordOffsets{};
    Position g92Offset{};        ///< G92 offset
    Position g52Offset{};        ///< G52 local offset
    double coordRotation{0.0};   ///< XY rotation (degrees)
    
    // Canned cycle state
    CannedCycleParams cannedCycle{};
    double cannedInitialZ{0.0};  ///< Z before canned cycle
    
    // Path blending
    double blendTolerance{0.0};  ///< G64 P value
    double naiveCamTolerance{0.0}; ///< G64 Q value
    
    // Coolant
    bool coolantMist{false};
    bool coolantFlood{false};
    
    // Overrides
    bool feedOverrideEnabled{true};
    bool spindleOverrideEnabled{true};
    bool adaptiveFeedEnabled{false};
    bool feedStopEnabled{false};
    double feedOverride{1.0};
    double spindleOverride{1.0};
    
    // Line number
    int32_t lineNumber{0};
    uint32_t blockCount{0};
    
    // Flags
    bool programRunning{false};
    bool feedHold{false};
    bool blockDelete{false};
    bool optionalStop{false};
};

// ============================================================================
// Error Types
// ============================================================================

/**
 * @brief G-code error codes
 */
enum class ErrorCode : uint16_t {
    OK = 0,

    /// End of input / end of program (used by streaming APIs)
    END = 1,
    
    // Parsing errors (100-199)
    SYNTAX_ERROR = 100,
    UNKNOWN_GCODE = 101,
    UNKNOWN_MCODE = 102,
    INVALID_WORD = 103,
    MISSING_VALUE = 104,
    INVALID_LINE_NUMBER = 105,
    EXPRESSION_ERROR = 106,
    PARAMETER_ERROR = 107,
    MISSING_BRACKET = 108,
    INVALID_OCODE = 109,
    SUBROUTINE_ERROR = 110,
    FILE_NOT_FOUND = 111,
    NESTED_TOO_DEEP = 112,
    
    // Execution errors (200-299)
    NO_FEED_RATE = 200,
    INVALID_MOTION = 201,
    ARC_RADIUS_ERROR = 202,
    AXIS_WORD_MISSING = 203,
    CONFLICTING_WORDS = 204,
    INVALID_PLANE = 205,
    SPINDLE_NOT_ON = 206,
    TOOL_ERROR = 207,
    PROBE_ERROR = 208,
    LIMIT_EXCEEDED = 209,
    INTERLOCK_ERROR = 210,
    CUTTER_COMP_ERROR = 211,
    QUEUE_FULL = 212,
    
    // Control flow errors (300-399)
    UNDEFINED_SUBROUTINE = 300,
    RETURN_WITHOUT_CALL = 301,
    BREAK_OUTSIDE_LOOP = 302,
    CONTINUE_OUTSIDE_LOOP = 303,
    ENDIF_WITHOUT_IF = 304,
    ELSE_WITHOUT_IF = 305,
    ENDWHILE_WITHOUT_WHILE = 306,
    DUPLICATE_LABEL = 307,
    
    // System errors (400-499)
    MEMORY_ERROR = 400,
    HARDWARE_ERROR = 401,
    TIMEOUT = 402,
    ESTOP = 403,
};

/**
 * @brief Error information structure
 */
struct Error {
    ErrorCode code{ErrorCode::OK};
    uint32_t line{0};
    std::array<char, 128> message{};
    std::array<char, 64> context{};
    
    bool ok() const { return code == ErrorCode::OK; }
    operator bool() const { return code != ErrorCode::OK; }
};

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Motion segment for output to motion controller
 */
struct MotionSegment {
    enum class Type : uint8_t {
        RAPID,
        LINEAR,
        ARC_CW,
        ARC_CCW,
        SPLINE,
        NURBS,
        DWELL,
        PROBE
    };
    
    Type type{Type::LINEAR};
    Position endPosition{};
    Position centerOffset{};     // For arcs
    double feedRate{0.0};
    double duration{0.0};        // For dwells
    double acceleration{0.0};
    int32_t lineNumber{0};
    
    // Spline data
    std::array<Position, 4> splinePoints{};
    
    // Arc data  
    ArcParams arc{};
};

/// Motion output callback
using MotionCallback = std::function<Error(const MotionSegment&)>;

/// User M-code handler (M100-M199)
using UserMCodeHandler = std::function<Error(int32_t mcode, double p, double q)>;

/// Message output callback
using MessageCallback = std::function<void(const std::string& message)>;

/// Input wait callback (M66)
using InputWaitCallback = std::function<bool(int32_t input, int32_t mode, 
                                             double timeout, double* value)>;

/// Output control callback (M62-M68)
using OutputCallback = std::function<Error(int32_t output, double value, bool sync)>;

} // namespace GCode
