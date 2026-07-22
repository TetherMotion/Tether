/**
 * @file GCodeVariables.hpp
 * @brief G-Code Variable and Expression System
 * 
 * @details
 * This file implements the G-code parameter (variable) system conforming to
 * RS274/NGC specification, including expression evaluation with mathematical
 * functions.
 * 
 * ## Parameter Types
 * 
 * ### Numbered Parameters
 * ```
 * #1 - #30     Local parameters (subroutine arguments)
 * #31 - #5000  Global parameters (persistent)
 * #5001 - #5999 System parameters (read-only)
 * ```
 * 
 * ### Named Parameters
 * ```
 * #<name>       Local named parameter
 * #<_name>      Global named parameter (persists across calls)
 * #<_name>      Predefined system parameters (underscore prefix)
 * ```
 * 
 * ### Predefined System Parameters
 * | Parameter | Description |
 * |-----------|-------------|
 * | #5061-#5069 | Probe trip coordinates |
 * | #5070 | Probe success flag |
 * | #5161-#5169 | G28 home position |
 * | #5181-#5189 | G30 home position |
 * | #5211-#5219 | G92 offset |
 * | #5220 | Current coordinate system |
 * | #5221-#5230 | G54 offsets + rotation |
 * | ... | Other coordinate systems |
 * | #5400 | Current tool number |
 * | #5401-#5409 | Tool length offsets |
 * | #5410 | Tool diameter |
 * | #5420-#5428 | Current position |
 * 
 * ## Expression Syntax
 * 
 * ### Operators (in precedence order)
 * ```
 * ** or ^       Power
 * * / MOD       Multiply, divide, modulo
 * + -           Add, subtract
 * EQ NE GT GE LT LE  Comparison
 * AND OR XOR NOT     Logical
 * ```
 * 
 * ### Functions
 * ```
 * ABS[x]    ACOS[x]   ASIN[x]   ATAN[x]/y]  COS[x]
 * EXP[x]    FIX[x]    FUP[x]    LN[x]       ROUND[x]
 * SIN[x]    SQRT[x]   TAN[x]    EXISTS[#<name>]
 * ```
 * 
 * ## Usage Examples
 * 
 * ### Basic Parameter Access
 * ```cpp
 * VariableSystem vars;
 * 
 * // Set a numbered parameter
 * vars.set(100, 42.5);
 * 
 * // Get a parameter
 * double val = vars.get(100);  // 42.5
 * 
 * // Set named parameter
 * vars.setNamed("my_var", 100.0);
 * double v = vars.getNamed("my_var");
 * ```
 * 
 * ### Expression Evaluation
 * ```cpp
 * ExpressionEvaluator eval(vars);
 * 
 * // Simple expressions
 * double result = eval.evaluate("[1 + 2 * 3]");  // 7.0
 * 
 * // With parameters
 * vars.set(1, 10.0);
 * result = eval.evaluate("[#1 * 2 + 5]");  // 25.0
 * 
 * // Functions
 * result = eval.evaluate("[SQRT[144] + SIN[30]]");  // 12.5
 * 
 * // Nested
 * result = eval.evaluate("[ABS[#1 - 20] * 2]");  // 20.0
 * ```
 * 
 * ## Implementation Notes
 * 
 * ### Memory Management
 * - Numbered parameters use fixed array storage
 * - Named parameters use hash map for O(1) access
 * - Local parameters stack per call level
 * 
 * ### Thread Safety
 * - VariableSystem is NOT thread-safe. All access must be from a single
 *   thread (typically the G-code execution thread). If multi-threaded
 *   access is needed, the caller must provide external synchronization.
 * - Local parameters are per-execution context
 * 
 * @see GCodeParser
 * @see GCodeInterpreter
 */

#pragma once

#include "GCodeTypes.hpp"
#include <cstring>
#include <cmath>
#include <unordered_map>
#include <string>
#include <vector>
#include <array>
#include <optional>

namespace GCode {

// ============================================================================
// Parameter Ranges
// ============================================================================

/// Start of local parameters
constexpr int32_t PARAM_LOCAL_START = 1;
/// End of local parameters
constexpr int32_t PARAM_LOCAL_END = 30;

/// Start of global user parameters
constexpr int32_t PARAM_GLOBAL_START = 31;
/// End of global user parameters
constexpr int32_t PARAM_GLOBAL_END = 5000;

/// System parameter ranges
constexpr int32_t PARAM_PROBE_RESULT = 5061;    // #5061-#5069 probe position
constexpr int32_t PARAM_PROBE_SUCCESS = 5070;   // #5070 probe success
constexpr int32_t PARAM_G28_HOME = 5161;        // #5161-#5169 G28 position
constexpr int32_t PARAM_G30_HOME = 5181;        // #5181-#5189 G30 position
constexpr int32_t PARAM_G92_OFFSET = 5211;      // #5211-#5219 G92 offset
constexpr int32_t PARAM_COORD_SYSTEM = 5220;    // Current coord system (1-9)
constexpr int32_t PARAM_G54_OFFSET = 5221;      // #5221-#5230 G54 offset+rotation
constexpr int32_t PARAM_TOOL_NUMBER = 5400;     // Current tool number
constexpr int32_t PARAM_TOOL_OFFSET = 5401;     // #5401-#5409 tool offsets
constexpr int32_t PARAM_TOOL_DIAMETER = 5410;   // Tool diameter
constexpr int32_t PARAM_CURRENT_POS = 5420;     // #5420-#5428 current position
constexpr int32_t PARAM_VALUE_RETURNED = 5599;  // _value_returned flag
constexpr int32_t PARAM_RETURN_VALUE = 5600;    // _value (return value)

// ============================================================================
// Predefined Named Parameters
// ============================================================================

/**
 * @brief Predefined named parameter names
 */
namespace PredefinedParams {
    constexpr const char* VALUE = "_value";
    constexpr const char* VALUE_RETURNED = "_value_returned";
    constexpr const char* VMAJOR = "_vmajor";
    constexpr const char* VMINOR = "_vminor";
    constexpr const char* LINE = "_line";
    constexpr const char* MOTION_MODE = "_motion_mode";
    constexpr const char* PLANE = "_plane";
    constexpr const char* CCOMP = "_ccomp";
    constexpr const char* METRIC = "_metric";
    constexpr const char* IMPERIAL = "_imperial";
    constexpr const char* ABSOLUTE = "_absolute";
    constexpr const char* INCREMENTAL = "_incremental";
    constexpr const char* INVERSE_TIME = "_inverse_time";
    constexpr const char* UNITS_PER_MINUTE = "_units_per_minute";
    constexpr const char* UNITS_PER_REV = "_units_per_rev";
    constexpr const char* COORD_SYSTEM = "_coord_system";
    constexpr const char* TOOL_OFFSET = "_tool_offset";
    constexpr const char* RETRACT_R = "_retract_r_plane";
    constexpr const char* RETRACT_OLD = "_retract_old_z";
    constexpr const char* SPINDLE_RPM_MODE = "_spindle_rpm_mode";
    constexpr const char* SPINDLE_CSS_MODE = "_spindle_css_mode";
    constexpr const char* IJK_ABSOLUTE = "_ijk_absolute_mode";
    constexpr const char* LATHE_DIAMETER = "_lathe_diameter_mode";
    constexpr const char* LATHE_RADIUS = "_lathe_radius_mode";
    constexpr const char* SPINDLE_ON = "_spindle_on";
    constexpr const char* SPINDLE_CW = "_spindle_cw";
    constexpr const char* MIST = "_mist";
    constexpr const char* FLOOD = "_flood";
    constexpr const char* SPEED_OVERRIDE = "_speed_override";
    constexpr const char* FEED_OVERRIDE = "_feed_override";
    constexpr const char* ADAPTIVE_FEED = "_adaptive_feed";
    constexpr const char* FEED_HOLD = "_feed_hold";
    constexpr const char* FEED = "_feed";
    constexpr const char* RPM = "_rpm";
    constexpr const char* CURRENT_TOOL = "_current_tool";
    constexpr const char* SELECTED_TOOL = "_selected_tool";
    constexpr const char* CURRENT_POCKET = "_current_pocket";
    constexpr const char* SELECTED_POCKET = "_selected_pocket";
    constexpr const char* X = "_x";
    constexpr const char* Y = "_y";
    constexpr const char* Z = "_z";
    constexpr const char* A = "_a";
    constexpr const char* B = "_b";
    constexpr const char* C = "_c";
    constexpr const char* U = "_u";
    constexpr const char* V = "_v";
    constexpr const char* W = "_w";
    constexpr const char* ABS_X = "_abs_x";
    constexpr const char* ABS_Y = "_abs_y";
    constexpr const char* ABS_Z = "_abs_z";
    constexpr const char* TASK = "_task";
}

// ============================================================================
// Variable Storage
// ============================================================================

/**
 * @brief Local parameter frame (per call level)
 */
struct LocalFrame {
    std::array<double, MAX_LOCAL_PARAMS> params{};
    std::array<bool, MAX_LOCAL_PARAMS> defined{};
    std::unordered_map<std::string, double> namedParams;
    
    void clear() {
        params.fill(0.0);
        defined.fill(false);
        namedParams.clear();
    }
};

/**
 * @brief Complete variable system
 */
class VariableSystem {
public:
    VariableSystem();
    
    // =======================================================================
    // Numbered Parameters
    // =======================================================================
    
    /**
     * @brief Get numbered parameter value
     * @param number Parameter number (1-5999)
     * @return Parameter value, 0.0 if undefined
     */
    double get(int32_t number) const;
    
    /**
     * @brief Set numbered parameter
     * @param number Parameter number
     * @param value Value to set
     * @return Error if parameter is read-only
     */
    Error set(int32_t number, double value);
    
    /**
     * @brief Check if parameter is defined
     */
    bool isDefined(int32_t number) const;
    
    /**
     * @brief Check if parameter is read-only
     */
    bool isReadOnly(int32_t number) const;
    
    // =======================================================================
    // Named Parameters
    // =======================================================================
    
    /**
     * @brief Get named parameter value
     * @param name Parameter name (without #<>)
     * @return Parameter value, nullopt if undefined
     */
    std::optional<double> getNamed(const std::string& name) const;
    
    /**
     * @brief Set named parameter
     * @param name Parameter name
     * @param value Value to set
     */
    Error setNamed(const std::string& name, double value);
    
    /**
     * @brief Check if named parameter exists
     */
    bool existsNamed(const std::string& name) const;
    
    /**
     * @brief Check if name is a global parameter (starts with _)
     */
    static bool isGlobalName(const std::string& name) {
        return !name.empty() && name[0] == '_';
    }
    
    // =======================================================================
    // Call Stack Management
    // =======================================================================
    
    /**
     * @brief Push new local frame for subroutine call
     * @param args Arguments to pass (#1, #2, etc.)
     */
    Error pushFrame(const std::vector<double>& args = {});
    
    /**
     * @brief Pop local frame on subroutine return
     */
    Error popFrame();
    
    /**
     * @brief Get current call depth
     */
    size_t getCallDepth() const { return m_callStack.size(); }
    
    /**
     * @brief Clear all variables
     */
    void clear();
    
    // =======================================================================
    // Machine State Integration
    // =======================================================================
    
    /**
     * @brief Update system parameters from machine state
     */
    void updateFromState(const MachineState& state);
    
    /**
     * @brief Set probe result
     */
    void setProbeResult(const ProbeResult& result);
    
    /**
     * @brief Set coordinate system offsets
     */
    void setCoordSystemOffset(CoordSystem cs, const Position& offset, 
                              double rotation = 0.0);
    
    /**
     * @brief Get coordinate system offset
     */
    Position getCoordSystemOffset(CoordSystem cs) const;
    
    /**
     * @brief Set return value (for subroutine returns)
     */
    void setReturnValue(double value);
    
    /**
     * @brief Get return value
     */
    std::optional<double> getReturnValue() const;
    
    /**
     * @brief Clear return value
     */
    void clearReturnValue();
    
private:
    // Global numbered parameters (#31-#5000)
    std::array<double, PARAM_GLOBAL_END - PARAM_GLOBAL_START + 1> m_globalParams{};
    std::array<bool, PARAM_GLOBAL_END - PARAM_GLOBAL_START + 1> m_globalDefined{};
    
    // System parameters (read-only, computed from state)
    std::array<double, 600> m_systemParams{};  // #5001-#5600
    
    // Global named parameters (start with _)
    std::unordered_map<std::string, double> m_globalNamed;
    
    // Call stack for local parameters
    std::vector<LocalFrame> m_callStack;
    
    // Return value handling
    bool m_valueReturned{false};
    double m_returnValue{0.0};
    
    // Current machine state reference (for predefined params)
    const MachineState* m_state{nullptr};
    
    // Helper to get local frame (current scope)
    LocalFrame& currentFrame();
    const LocalFrame& currentFrame() const;
    
    // System parameter helpers
    double getSystemParam(int32_t number) const;
    void updateSystemParams();
};

// ============================================================================
// Expression Tokens
// ============================================================================

/**
 * @brief Expression token types
 */
enum class TokenType : uint8_t {
    NUMBER,
    PARAMETER,      // #123 or #<name>
    OPEN_BRACKET,   // [
    CLOSE_BRACKET,  // ]
    PLUS,
    MINUS,
    MULTIPLY,
    DIVIDE,
    POWER,          // ** or ^
    MOD,
    EQ, NE, GT, GE, LT, LE,
    AND, OR, XOR, NOT,
    FUNC_ABS, FUNC_ACOS, FUNC_ASIN, FUNC_ATAN,
    FUNC_COS, FUNC_EXP, FUNC_FIX, FUNC_FUP,
    FUNC_LN, FUNC_ROUND, FUNC_SIN, FUNC_SQRT, FUNC_TAN,
    FUNC_EXISTS,
    END,
    ERROR
};

/**
 * @brief Expression token
 */
struct Token {
    TokenType type{TokenType::END};
    double value{0.0};
    std::string name;  // For named parameters
    int32_t paramNum{-1};  // For numbered parameters
};

// ============================================================================
// Expression Evaluator
// ============================================================================

/**
 * @brief G-code expression evaluator
 * 
 * Evaluates mathematical expressions in G-code following RS274/NGC syntax.
 * 
 * ## Supported Operations
 * 
 * ### Arithmetic
 * - Addition: +
 * - Subtraction: -
 * - Multiplication: *
 * - Division: /
 * - Power: ** or ^
 * - Modulo: MOD
 * 
 * ### Comparison (return 1.0 for true, 0.0 for false)
 * - Equal: EQ
 * - Not equal: NE
 * - Greater than: GT
 * - Greater or equal: GE
 * - Less than: LT
 * - Less or equal: LE
 * 
 * ### Logical (values > 0 are true)
 * - AND, OR, XOR, NOT
 * 
 * ### Functions (use [] not ())
 * - ABS[x]   - Absolute value
 * - ACOS[x]  - Arc cosine (result in degrees)
 * - ASIN[x]  - Arc sine (result in degrees)
 * - ATAN[x]/y] - Arc tangent with 2 args (degrees)
 * - COS[x]   - Cosine (x in degrees)
 * - EXP[x]   - e^x
 * - FIX[x]   - Round toward zero
 * - FUP[x]   - Round away from zero
 * - LN[x]    - Natural logarithm
 * - ROUND[x] - Round to nearest
 * - SIN[x]   - Sine (x in degrees)
 * - SQRT[x]  - Square root
 * - TAN[x]   - Tangent (x in degrees)
 * - EXISTS[#<name>] - Check if parameter exists
 */
class ExpressionEvaluator {
public:
    /**
     * @brief Constructor
     * @param vars Variable system for parameter access
     */
    explicit ExpressionEvaluator(VariableSystem& vars);
    
    /**
     * @brief Evaluate an expression
     * @param expr Expression string (including [] brackets)
     * @param result Output: evaluated result
     * @return Error if evaluation failed
     */
    Error evaluate(const char* expr, double& result);
    
    /**
     * @brief Evaluate expression returning result or error
     */
    std::pair<double, Error> evaluate(const char* expr);
    
    /**
     * @brief Get position after last evaluation
     */
    size_t getEndPosition() const { return m_pos; }
    
    /**
     * @brief Get last error message
     */
    const char* getErrorMessage() const { return m_error.data(); }
    
private:
    VariableSystem& m_vars;
    const char* m_expr{nullptr};
    size_t m_pos{0};
    size_t m_len{0};
    Token m_current;
    std::array<char, 64> m_error{};
    
    // Tokenizer
    Token nextToken();
    void skipWhitespace();
    Token parseNumber();
    Token parseParameter();
    Token parseIdentifier();
    
    // Recursive descent parser
    Error parseExpression(double& result);
    Error parseTernary(double& result);       // ? :
    Error parseLogicalOr(double& result);     // OR
    Error parseLogicalXor(double& result);    // XOR
    Error parseLogicalAnd(double& result);    // AND
    Error parseComparison(double& result);    // EQ NE GT GE LT LE
    Error parseAddSub(double& result);        // + -
    Error parseMulDiv(double& result);        // * / MOD
    Error parsePower(double& result);         // ** ^
    Error parseUnary(double& result);         // + - NOT
    Error parsePrimary(double& result);       // number, param, func, (expr)
    Error parseFunction(TokenType func, double& result);
    
    // Helper
    void setError(const char* msg);
    static double degToRad(double deg) { return deg * M_PI / 180.0; }
    static double radToDeg(double rad) { return rad * 180.0 / M_PI; }
};

// ============================================================================
// Parameter Substitution
// ============================================================================

/**
 * @brief Substitute parameter references in a string
 * 
 * Replaces #N and #<name> with their values.
 * 
 * @param input Input string with parameter references
 * @param vars Variable system
 * @param output Output buffer
 * @param outputSize Output buffer size
 * @return Error if substitution failed
 */
Error substituteParameters(const char* input, VariableSystem& vars,
                          char* output, size_t outputSize);

/**
 * @brief Parse a parameter reference
 * 
 * Parses #N, #<name>, or #<expr> from input.
 * 
 * @param input Input starting at #
 * @param vars Variable system (for expression evaluation)
 * @param value Output: parameter value
 * @param consumed Output: characters consumed
 * @return Error if parse failed
 */
Error parseParameterRef(const char* input, VariableSystem& vars,
                        double& value, size_t& consumed);

} // namespace GCode
