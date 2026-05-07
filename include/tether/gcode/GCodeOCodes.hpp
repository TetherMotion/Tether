/**
 * @file GCodeOCodes.hpp
 * @brief O-Code Control Flow Implementation
 * 
 * @details
 * This file implements the complete O-code (control flow) system conforming
 * to LinuxCNC RS274/NGC specification.
 * 
 * ## Supported O-Code Constructs
 * 
 * ### Subroutines
 * ```gcode
 * ; Numbered subroutine definition
 * O100 sub
 *     G0 X0 Y0
 *     G1 X#1 Y#2 F#3
 * O100 endsub
 * 
 * ; Call with parameters
 * O100 call [10] [20] [500]
 * 
 * ; Named subroutine
 * O<my_routine> sub
 *     (code here)
 * O<my_routine> endsub
 * 
 * O<my_routine> call [arg1] [arg2]
 * 
 * ; Return with value
 * O100 return [#1 * 2]
 * 
 * ; Access return value
 * #<_value>           ; The returned value
 * #<_value_returned>  ; 1 if value was returned
 * ```
 * 
 * ### Conditional (If/Else)
 * ```gcode
 * O101 if [#1 GT 10]
 *     G1 X100 F500
 * O101 elseif [#1 GT 5]
 *     G1 X50 F500
 * O101 else
 *     G1 X10 F500
 * O101 endif
 * ```
 * 
 * ### While Loop
 * ```gcode
 * #1 = 0
 * O102 while [#1 LT 10]
 *     G1 X[#1 * 10] Y0 F500
 *     #1 = [#1 + 1]
 * O102 endwhile
 * ```
 * 
 * ### Do-While Loop
 * ```gcode
 * #1 = 0
 * O103 do
 *     G1 X[#1 * 10] Y0 F500
 *     #1 = [#1 + 1]
 * O103 while [#1 LT 10]
 * ```
 * 
 * ### Repeat Loop
 * ```gcode
 * O104 repeat [10]
 *     G1 X[#<_repeat_count> * 5]
 * O104 endrepeat
 * ```
 * 
 * ### Break and Continue
 * ```gcode
 * O105 while [#1 LT 100]
 *     O110 if [#1 EQ 50]
 *         O105 break     ; Exit the while loop
 *     O110 endif
 *     O111 if [[#1 MOD 2] EQ 0]
 *         O105 continue  ; Skip to next iteration
 *     O111 endif
 *     ; (process odd numbers only)
 *     #1 = [#1 + 1]
 * O105 endwhile
 * ```
 * 
 * ### File Calling
 * ```gcode
 * ; Call subroutine from file myproc.ngc
 * O<myproc> call [arg1] [arg2]
 * 
 * ; File must contain:
 * ; O<myproc> sub
 * ;     (code)
 * ; O<myproc> endsub
 * ```
 * 
 * ### Fanuc-Style (M98/M99)
 * ```gcode
 * ; Main program
 * O1
 * M98 P100 L5     ; Call subprogram 100, repeat 5 times
 * M30
 * 
 * ; Subprogram (must follow call)
 * O100
 *     G1 X10 F500
 * M99
 * ```
 * 
 * ## Execution Model
 * 
 * O-codes are executed by maintaining:
 * 1. Call stack for subroutines
 * 2. Loop stack for while/repeat
 * 3. Conditional stack for if/else
 * 4. Return address for each call
 * 
 * ## Error Handling
 * 
 * | Error | Cause |
 * |-------|-------|
 * | Undefined subroutine | Call to non-existent O-number |
 * | Return without call | O-return outside subroutine |
 * | Break outside loop | O-break not inside while/repeat |
 * | Continue outside loop | O-continue not inside loop |
 * | Endif without if | O-endif without matching O-if |
 * | Nested too deep | Exceeds MAX_CALL_DEPTH |
 * 
 * @see GCodeParser
 * @see GCodeVariables
 */

#pragma once

#include "GCodeTypes.hpp"
#include "GCodeVariables.hpp"
#include "GCodeParser.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <stack>
#include <functional>

namespace GCode {

// ============================================================================
// O-Code Stack Frames
// ============================================================================

/**
 * @brief Call stack frame for subroutine execution
 */
struct CallFrame {
    /// O-code number or name
    int32_t oNumber{-1};
    std::string oName;
    bool isNamed{false};
    
    /// Return address (source position)
    size_t returnAddress{0};
    uint32_t returnLine{0};
    
    /// File context (for external file calls)
    std::string fileName;
    Parser* fileParser{nullptr};  // Parser for external file
    
    /// Local parameter frame index
    size_t paramFrameIndex{0};
    
    /// Modal state save (for M73)
    bool hasModalSave{false};
    MachineState savedModal{};
};

/**
 * @brief Loop stack frame
 */
struct LoopFrame {
    /// O-code number
    int32_t oNumber{-1};
    
    /// Loop type
    enum class Type : uint8_t {
        WHILE,      ///< o<n> while / o<n> endwhile
        DO_WHILE,   ///< o<n> do / o<n> while
        REPEAT      ///< o<n> repeat / o<n> endrepeat
    } type{Type::WHILE};
    
    /// Loop start address (for while) or repeat count
    size_t startAddress{0};
    uint32_t startLine{0};
    
    /// For repeat: current iteration and max
    int32_t currentIteration{0};
    int32_t maxIterations{0};
    
    /// Condition expression (for while)
    std::string condition;
};

/**
 * @brief Conditional (if/else) stack frame
 */
struct ConditionalFrame {
    /// O-code number
    int32_t oNumber{-1};
    
    /// Has a branch been taken?
    bool branchTaken{false};
    
    /// Currently in else block?
    bool inElse{false};
    
    /// Address of endif (for skipping)
    size_t endifAddress{0};
    uint32_t endifLine{0};
};

// ============================================================================
// Subroutine Registry
// ============================================================================

/**
 * @brief Information about a registered subroutine
 */
struct SubroutineInfo {
    /// O-code number or name
    int32_t oNumber{-1};
    std::string name;
    bool isNamed{false};
    
    /// Source file (empty if inline)
    std::string fileName;
    
    /// Position of 'sub' keyword
    size_t startAddress{0};
    uint32_t startLine{0};
    
    /// Position of 'endsub' keyword
    size_t endAddress{0};
    uint32_t endLine{0};
    
    /// Is this Fanuc-style (M98/M99)?
    bool isFanucStyle{false};
};

/**
 * @brief Subroutine registry
 */
class SubroutineRegistry {
public:
    /**
     * @brief Register a numbered subroutine
     */
    Error registerSubroutine(int32_t oNumber, const SubroutineInfo& info);
    
    /**
     * @brief Register a named subroutine
     */
    Error registerSubroutine(const std::string& name, const SubroutineInfo& info);
    
    /**
     * @brief Find numbered subroutine
     */
    const SubroutineInfo* find(int32_t oNumber) const;
    
    /**
     * @brief Find named subroutine
     */
    const SubroutineInfo* find(const std::string& name) const;
    
    /**
     * @brief Check if subroutine exists
     */
    bool exists(int32_t oNumber) const;
    bool exists(const std::string& name) const;
    
    /**
     * @brief Clear all registrations
     */
    void clear();
    
    /**
     * @brief Scan source for subroutines
     */
    Error scanSource(Parser& parser);
    
    /**
     * @brief Load subroutines from file
     */
    Error loadFromFile(const std::string& filename);
    
private:
    std::unordered_map<int32_t, SubroutineInfo> m_numbered;
    std::unordered_map<std::string, SubroutineInfo> m_named;
};

// ============================================================================
// O-Code Executor
// ============================================================================

/**
 * @brief Configuration for O-code execution
 */
struct OCodeConfig {
    /// Maximum call stack depth
    size_t maxCallDepth{MAX_CALL_DEPTH};
    
    /// Maximum loop iterations (infinite loop prevention)
    size_t maxLoopIterations{1000000};
    
    /// Subroutine search paths
    std::vector<std::string> searchPaths;
    
    /// Enable Fanuc-style M98/M99
    bool enableFanucStyle{true};
    
    /// Enable named subroutines
    bool enableNamedSubs{true};
    
    /// Case insensitive names
    bool caseInsensitive{true};
    
    /// File extension for subroutines
    std::string subFileExtension{".ngc"};
};

/**
 * @brief O-code execution engine
 */
class OCodeExecutor {
public:
    /**
     * @brief Constructor
     * @param vars Variable system
     * @param parser Parser for current source
     * @param config Configuration
     */
    OCodeExecutor(VariableSystem& vars, Parser& parser,
                  const OCodeConfig& config = OCodeConfig{});
    
    // ========================================================================
    // Execution Control
    // ========================================================================
    
    /**
     * @brief Execute O-code from block
     * @param block Block containing O-code
     * @param nextAction Output: what to do next
     * @return Error if execution failed
     */
    enum class NextAction {
        CONTINUE,       ///< Continue to next line
        JUMP,           ///< Jump to new address (use getJumpAddress)
        SKIP_TO_ENDIF,  ///< Skip to matching endif
        SKIP_TO_ELSE,   ///< Skip to matching else/elseif/endif
        SKIP_TO_ENDWHILE, ///< Skip to matching endwhile
        SKIP_TO_ENDREPEAT, ///< Skip to matching endrepeat
        EXIT_PROGRAM,   ///< Program end
        RETURN          ///< Return from subroutine
    };
    
    Error execute(const Block& block, NextAction& nextAction);
    
    /**
     * @brief Get jump address after execute() returns JUMP
     */
    size_t getJumpAddress() const { return m_jumpAddress; }
    uint32_t getJumpLine() const { return m_jumpLine; }
    
    /**
     * @brief Get return value (if any)
     */
    std::optional<double> getReturnValue() const;
    
    // ========================================================================
    // Subroutine Handling
    // ========================================================================
    
    /**
     * @brief Call a subroutine
     * @param oNumber O-code number
     * @param args Arguments to pass
     */
    Error callSubroutine(int32_t oNumber, 
                         const std::vector<double>& args = {});
    
    /**
     * @brief Call a named subroutine
     */
    Error callSubroutine(const std::string& name,
                         const std::vector<double>& args = {});
    
    /**
     * @brief Return from current subroutine
     * @param returnValue Optional return value
     */
    Error returnFromSub(std::optional<double> returnValue = std::nullopt);
    
    /**
     * @brief Check if in subroutine
     */
    bool inSubroutine() const { return !m_callStack.empty(); }
    
    /**
     * @brief Get current call depth
     */
    size_t getCallDepth() const { return m_callStack.size(); }
    
    // ========================================================================
    // Loop Handling
    // ========================================================================
    
    /**
     * @brief Begin while loop
     */
    Error beginWhile(int32_t oNumber, const std::string& condition);
    
    /**
     * @brief End while loop
     */
    Error endWhile(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Begin do block
     */
    Error beginDo(int32_t oNumber);
    
    /**
     * @brief End do-while (while after do)
     */
    Error doWhile(int32_t oNumber, const std::string& condition,
                  NextAction& nextAction);
    
    /**
     * @brief Begin repeat
     */
    Error beginRepeat(int32_t oNumber, int32_t count);
    
    /**
     * @brief End repeat
     */
    Error endRepeat(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Break out of loop
     */
    Error breakLoop(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Continue to next iteration
     */
    Error continueLoop(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Check if in loop
     */
    bool inLoop() const { return !m_loopStack.empty(); }
    
    /**
     * @brief Get current repeat count (for #<_repeat_count>)
     */
    int32_t getRepeatCount() const;
    
    // ========================================================================
    // Conditional Handling
    // ========================================================================
    
    /**
     * @brief Begin if block
     * @param oNumber O-code number
     * @param condition Condition expression
     * @param nextAction Output action
     */
    Error beginIf(int32_t oNumber, const std::string& condition,
                  NextAction& nextAction);
    
    /**
     * @brief Handle elseif
     */
    Error handleElseIf(int32_t oNumber, const std::string& condition,
                       NextAction& nextAction);
    
    /**
     * @brief Handle else
     */
    Error handleElse(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief End if block
     */
    Error endIf(int32_t oNumber);
    
    // ========================================================================
    // Fanuc Style (M98/M99)
    // ========================================================================
    
    /**
     * @brief Execute M98 (call subprogram)
     * @param pWord P parameter (subprogram number)
     * @param lWord L parameter (repeat count, default 1)
     */
    Error executeM98(int32_t pWord, int32_t lWord = 1);
    
    /**
     * @brief Execute M99 (return/end)
     * @param nextAction Output action
     */
    Error executeM99(NextAction& nextAction);
    
    // ========================================================================
    // Modal State Save/Restore (M70-M73)
    // ========================================================================
    
    /**
     * @brief Save modal state (M70)
     */
    Error saveModalState(const MachineState& state);
    
    /**
     * @brief Restore modal state (M72)
     */
    Error restoreModalState(MachineState& state);
    
    /**
     * @brief Invalidate saved state (M71)
     */
    Error invalidateModalState();
    
    /**
     * @brief Save and auto-restore on return (M73)
     */
    Error autoRestoreModalState(const MachineState& state);
    
    // ========================================================================
    // Registry Access
    // ========================================================================
    
    /**
     * @brief Get subroutine registry
     */
    SubroutineRegistry& getRegistry() { return m_registry; }
    const SubroutineRegistry& getRegistry() const { return m_registry; }
    
    /**
     * @brief Scan current source for subroutines
     */
    Error scanSubroutines();
    
    /**
     * @brief Add search path for subroutine files
     */
    void addSearchPath(const std::string& path);
    
    // ========================================================================
    // State
    // ========================================================================
    
    /**
     * @brief Reset all execution state
     */
    void reset();
    
    /**
     * @brief Get last error
     */
    const Error& getError() const { return m_error; }
    
private:
    OCodeConfig m_config;
    VariableSystem& m_vars;
    Parser& m_parser;
    ExpressionEvaluator m_evaluator;
    SubroutineRegistry m_registry;
    
    // Execution stacks
    std::vector<CallFrame> m_callStack;
    std::vector<LoopFrame> m_loopStack;
    std::vector<ConditionalFrame> m_condStack;
    
    // Modal state save
    std::optional<MachineState> m_savedModal;
    
    // Jump target
    size_t m_jumpAddress{0};
    uint32_t m_jumpLine{0};
    
    // Return value
    std::optional<double> m_returnValue;
    
    // Error
    Error m_error;
    
    // M98/M99 state
    int32_t m_m98RepeatRemaining{0};
    size_t m_m98ReturnAddress{0};
    
    // Helpers
    Error evaluateCondition(const std::string& condition, bool& result);
    Error findEndOfBlock(int32_t oNumber, OCodeType blockType, 
                         size_t& address, uint32_t& line);
    Error loadExternalSubroutine(const std::string& name);
    std::string findSubroutineFile(const std::string& name);
    
    void setError(ErrorCode code, const char* msg);
};

// ============================================================================
// O-Code Utilities
// ============================================================================

/**
 * @brief Get keyword string from O-code type
 */
const char* oCodeTypeToKeyword(OCodeType type);

/**
 * @brief Parse keyword to O-code type
 */
OCodeType keywordToOCodeType(const std::string& keyword);

/**
 * @brief Check if O-code type is block opener
 */
bool isBlockOpener(OCodeType type);

/**
 * @brief Check if O-code type is block closer
 */
bool isBlockCloser(OCodeType type);

/**
 * @brief Get matching closer for opener
 */
OCodeType getMatchingCloser(OCodeType opener);

/**
 * @brief Format O-code for display
 */
std::string formatOCode(int32_t oNumber, OCodeType type);
std::string formatOCode(const std::string& name, OCodeType type);

} // namespace GCode
