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
 * @brief Fanuc M98 call stack frame (subprogram call via M98 or G65/G66)
 */
struct M98Frame {
    /// Source offset of the subprogram body start (first line after label)
    size_t subStart{0};
    uint32_t subLine{0};

    /// Source offset to jump back to (line after the M98/G65 call)
    size_t returnAddress{0};

    /// Remaining L-word repetitions (first run already executing)
    int32_t remaining{0};
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
     * @param oNumber O-code number of the subroutine.
     * @param info Name, arguments and source location.
     * @return Error::OK, or a duplicate/validation error.
     */
    Error registerSubroutine(int32_t oNumber, const SubroutineInfo& info);
    
    /**
     * @brief Register a named subroutine
     * @param name Subroutine name as used by `o<name> call`.
     * @param info Name, arguments and source location.
     * @return Error::OK, or a duplicate/validation error.
     */
    Error registerSubroutine(const std::string& name, const SubroutineInfo& info);
    
    /**
     * @brief Find numbered subroutine
     * @param oNumber O-code number to look up.
     * @return Registered info, or nullptr when unknown.
     */
    const SubroutineInfo* find(int32_t oNumber) const;
    
    /**
     * @brief Find named subroutine
     * @param name Subroutine name to look up.
     * @return Registered info, or nullptr when unknown.
     */
    const SubroutineInfo* find(const std::string& name) const;
    
    /**
     * @brief Check if subroutine exists
     * @param oNumber O-code number to test.
     * @return true when a numbered subroutine is registered.
     */
    bool exists(int32_t oNumber) const;
    /// @param name Subroutine name to test.
    /// @return true when a named subroutine is registered.
    bool exists(const std::string& name) const;
    
    /**
     * @brief Clear all registrations
     */
    void clear();
    
    /**
     * @brief Scan source for subroutines
     * @param parser Parser positioned at the program source.
     * @return Error::OK, or the first parse error encountered.
     */
    Error scanSource(Parser& parser);
    
    /**
     * @brief Load subroutines from file
     * @param filename Path of the G-code source to scan.
     * @return Error::OK, or a file/parse error.
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
     * @return Block index the interpreter should continue at.
     */
    size_t getJumpAddress() const { return m_jumpAddress; }
    uint32_t getJumpLine() const { return m_jumpLine; }
    
    /**
     * @brief Get return value (if any)
     * @return Value passed to `o<sub> return`, or nullopt.
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
     * @param name Subroutine name to call.
     * @param args Arguments bound to the subroutine's parameter frame.
     * @return Error::OK, or an unknown-subroutine/argument-count error.
     */
    Error callSubroutine(const std::string& name,
                         const std::vector<double>& args = {});

    /**
     * @brief Call a numbered subprogram for G65/G66 (Fanuc macro call)
     *
     * Like callSubroutine() but also accepts bare `O<num>` program labels
     * (Fanuc-style subprograms terminated by M99). Arguments are bound to
     * local parameters #1..#30 via VariableSystem::pushFrame.
     *
     * @param oNumber Subprogram number.
     * @param args Arguments bound to #1..#30.
     * @return Error::OK, or an unknown-subprogram/argument-count error.
     */
    Error callSubprogram(int32_t oNumber, const std::vector<double>& args);
    
    /**
     * @brief Return from current subroutine
     * @param returnValue Optional return value
     */
    Error returnFromSub(std::optional<double> returnValue = std::nullopt);
    
    /**
     * @brief Check if in subroutine
     * @return true when at least one subroutine call is active.
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
     * @param oNumber O-code number of the while.
     * @param condition Loop condition expression.
     * @return Error::OK on success.
     */
    Error beginWhile(int32_t oNumber, const std::string& condition);
    
    /**
     * @brief End while loop
     * @param oNumber O-code number of the endwhile.
     * @param nextAction Output action (jump back when the condition holds).
     * @return Error::OK, or a nesting mismatch error.
     */
    Error endWhile(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Begin do block
     * @param oNumber O-code number of the do.
     * @return Error::OK on success.
     */
    Error beginDo(int32_t oNumber);
    
    /**
     * @brief End do-while (while after do)
     * @param oNumber O-code number of the trailing while.
     * @param condition Loop condition expression.
     * @param nextAction Output action (jump back when the condition holds).
     * @return Error::OK, or a nesting mismatch error.
     */
    Error doWhile(int32_t oNumber, const std::string& condition,
                  NextAction& nextAction);
    
    /**
     * @brief Begin repeat
     * @param oNumber O-code number of the repeat.
     * @param count Number of iterations to execute.
     * @return Error::OK on success.
     */
    Error beginRepeat(int32_t oNumber, int32_t count);
    
    /**
     * @brief End repeat
     * @param oNumber O-code number of the endrepeat.
     * @param nextAction Output action (jump back while iterations remain).
     * @return Error::OK, or a nesting mismatch error.
     */
    Error endRepeat(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Break out of loop
     * @param oNumber O-code number of the break.
     * @param nextAction Output action jumping past the loop.
     * @return Error::OK, or a break-outside-loop error.
     */
    Error breakLoop(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Continue to next iteration
     * @param oNumber O-code number of the continue.
     * @param nextAction Output action jumping to the loop condition.
     * @return Error::OK, or a continue-outside-loop error.
     */
    Error continueLoop(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief Check if in loop
     * @return true when at least one loop is active.
     */
    bool inLoop() const { return !m_loopStack.empty(); }
    
    /**
     * @brief Get current repeat count (for `#&lt;_repeat_count&gt;`)
     * @return 1-based iteration index of the innermost repeat block.
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
     * @return Error::OK on success.
     */
    Error beginIf(int32_t oNumber, const std::string& condition,
                  NextAction& nextAction);
    
    /**
     * @brief Handle elseif
     * @param oNumber O-code number of the elseif.
     * @param condition Condition expression.
     * @param nextAction Output action.
     * @return Error::OK on success.
     */
    Error handleElseIf(int32_t oNumber, const std::string& condition,
                       NextAction& nextAction);
    
    /**
     * @brief Handle else
     * @param oNumber O-code number of the else.
     * @param nextAction Output action.
     * @return Error::OK on success.
     */
    Error handleElse(int32_t oNumber, NextAction& nextAction);
    
    /**
     * @brief End if block
     * @param oNumber O-code number of the endif.
     * @return Error::OK, or a nesting mismatch error.
     */
    Error endIf(int32_t oNumber);
    
    // ========================================================================
    // Fanuc Style (M98/M99)
    // ========================================================================
    
    /**
     * @brief Execute M98 (call subprogram)
     * @param pWord P parameter (subprogram number)
     * @param lWord L parameter (repeat count, default 1)
     * @return Error::OK, or an unknown-subprogram error.
     */
    Error executeM98(int32_t pWord, int32_t lWord = 1);
    
    /**
     * @brief Execute M99 (return/end)
     * @param nextAction Output action
     * @return Error::OK, or a return-without-call error.
     */
    Error executeM99(NextAction& nextAction);
    
    // ========================================================================
    // Modal State Save/Restore (M70-M73)
    // ========================================================================
    
    /**
     * @brief Save modal state (M70)
     * @param state Machine state snapshot to store.
     * @return Error::OK on success.
     */
    Error saveModalState(const MachineState& state);
    
    /**
     * @brief Restore modal state (M72)
     * @param state Machine state overwritten with the saved snapshot.
     * @return Error::OK, or an error when no state was saved.
     */
    Error restoreModalState(MachineState& state);
    
    /**
     * @brief Invalidate saved state (M71)
     * @return Error::OK on success.
     */
    Error invalidateModalState();
    
    /**
     * @brief Save and auto-restore on return (M73)
     * @param state Machine state snapshot to store.
     * @return Error::OK on success.
     */
    Error autoRestoreModalState(const MachineState& state);
    
    // ========================================================================
    // Registry Access
    // ========================================================================
    
    /**
     * @brief Get subroutine registry
     * @return Mutable registry of known subroutines.
     */
    SubroutineRegistry& getRegistry() { return m_registry; }
    /// @return Read-only registry of known subroutines.
    const SubroutineRegistry& getRegistry() const { return m_registry; }
    
    /**
     * @brief Scan current source for subroutines
     * @return Error::OK, or the first parse error encountered.
     */
    Error scanSubroutines();
    
    /**
     * @brief Add search path for subroutine files
     * @param path Directory searched by M98/O-call file resolution.
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
     * @return Error produced by the most recent operation.
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
    
    // M98/M99 state (Fanuc subprogram calls)
    std::vector<M98Frame> m_m98Stack;
    
    // Helpers
    Error evaluateCondition(const std::string& condition, bool& result);
    Error findEndOfBlock(int32_t oNumber, OCodeType blockType, 
                         size_t& address, uint32_t& line);
    /// Scan forward (opener already consumed) for the matching block closer
    /// at depth 0 — `if`→`endif`, `while`→`endwhile`, `do`→`while`,
    /// `repeat`→`endrepeat`, `sub`→`endsub`. On return, getLastBlockStart/
    /// End describe the closer's line.
    Error scanForEnder(int32_t oNumber, OCodeType opener, Block& endBlock);
    /// Scan forward to the next same-depth ELSEIF/ELSE/ENDIF for oNumber and
    /// set the jump target to that block's start (it re-executes normally).
    Error scanToBranchPoint(int32_t oNumber);
    /// Scan forward to the matching ENDIF and jump to its start.
    Error scanToEndif(int32_t oNumber);
    /// Find the matching block closer and jump past it.
    Error jumpPastEnder(int32_t oNumber, OCodeType opener);
    /// Find the matching block closer and jump to its start (re-executes it).
    Error jumpToEnder(int32_t oNumber, OCodeType opener);
    Error loadExternalSubroutine(const std::string& name);
    std::string findSubroutineFile(const std::string& name);
    
    void setError(ErrorCode code, const char* msg);
};

// ============================================================================
// O-Code Utilities
// ============================================================================

/**
 * @brief Get keyword string from O-code type
 * @param type O-code type to name.
 * @return Static keyword (e.g. "while"), or nullptr for unknown types.
 */
const char* oCodeTypeToKeyword(OCodeType type);

/**
 * @brief Parse keyword to O-code type
 * @param keyword Keyword such as "while" or "if".
 * @return Matching type, or OCodeType::NONE when unrecognized.
 */
OCodeType keywordToOCodeType(const std::string& keyword);

/**
 * @brief Check if O-code type is block opener
 * @param type O-code type to test.
 * @return true for while/do/repeat/if and similar openers.
 */
bool isBlockOpener(OCodeType type);

/**
 * @brief Check if O-code type is block closer
 * @param type O-code type to test.
 * @return true for endwhile/enddo/endrepeat/endif.
 */
bool isBlockCloser(OCodeType type);

/**
 * @brief Get matching closer for opener
 * @param opener Block-opening type.
 * @return Corresponding closing type, or OCodeType::NONE when not an opener.
 */
OCodeType getMatchingCloser(OCodeType opener);

/**
 * @brief Format O-code for display
 * @param oNumber O-code number.
 * @param type O-code type.
 * @return Human-readable form such as "o100 while".
 */
std::string formatOCode(int32_t oNumber, OCodeType type);
/// @param name Subroutine name.
/// @param type O-code type.
/// @return Human-readable form such as "o<name> call".
std::string formatOCode(const std::string& name, OCodeType type);

} // namespace GCode
