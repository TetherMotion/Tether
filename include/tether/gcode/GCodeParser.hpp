/**
 * @file GCodeParser.hpp
 * @brief G-Code Parser - Converts tokens to Block structures
 * 
 * @details
 * The parser takes lexer tokens and produces Block structures containing
 * all information from a single G-code line. It handles:
 * 
 * - Modal group validation
 * - Word ordering and conflict detection
 * - Expression evaluation in word values
 * - O-code structure validation
 * - Block numbering
 * 
 * ## Parsing Pipeline
 * 
 * ```
 * Source Text
 *     │
 *     ▼
 * ┌─────────┐    ┌─────────┐    ┌─────────┐
 * │  Lexer  │───>│ Parser  │───>│  Block  │
 * └─────────┘    └─────────┘    └─────────┘
 *                     │
 *                     ▼
 *               ┌───────────┐
 *               │ Variables │
 *               └───────────┘
 * ```
 * 
 * ## Block Structure
 * 
 * A parsed block contains:
 * - Line number (N word)
 * - G-codes (modal and non-modal)
 * - M-codes
 * - Words (X, Y, Z, F, S, etc.)
 * - O-code information
 * - Comments
 * 
 * ## Modal Group Handling
 * 
 * The parser validates that only one G-code from each modal
 * group appears per block:
 * 
 * ```cpp
 * // Valid: G0 (group 1) and G17 (group 2)
 * G0 G17 X100
 * 
 * // Invalid: G0 and G1 both in group 1
 * G0 G1 X100  // ERROR
 * ```
 * 
 * ## Order of Execution
 * 
 * Within a block, items execute in this order:
 * 1. Comments (for display only)
 * 2. Set feed rate mode (G93, G94, G95)
 * 3. Set feed rate (F)
 * 4. Set spindle speed (S)
 * 5. Select tool (T)
 * 6. Change tool (M6)
 * 7. Spindle on/off (M3, M4, M5)
 * 8. Coolant on/off (M7, M8, M9)
 * 9. Enable/disable overrides (M48, M49)
 * 10. Dwell (G4)
 * 11. Set plane (G17, G18, G19)
 * 12. Set length units (G20, G21)
 * 13. Cutter radius compensation (G40, G41, G42)
 * 14. Tool length offset (G43, G43.1, G43.2, G49)
 * 15. Coordinate system selection (G54-G59.3)
 * 16. Set path control mode (G61, G61.1, G64)
 * 17. Set distance mode (G90, G91)
 * 18. Set retract mode (G98, G99)
 * 19. Home (G28, G30) or coordinate setting (G10, G92)
 * 20. Perform motion (G0-G3, G33, G38.x, G73, G76, G80-G89)
 * 21. Stop (M0, M1, M2, M30, M60)
 * 
 * ## Usage Example
 * 
 * ```cpp
 * GCode::VariableSystem vars;
 * GCode::Parser parser(vars);
 * 
 * // Parse a line
 * GCode::Block block;
 * Error err = parser.parseLine("G1 X100 Y50 F1000", block);
 * 
 * if (err.ok()) {
 *     // Access block data
 *     if (block.hasWord(WordLetter::X)) {
 *         double x = block.getWord(WordLetter::X);
 *     }
 *     
 *     for (int i = 0; i < block.gCodeCount; ++i) {
 *         printf("G%d ", block.gCodes[i]);
 *     }
 * }
 * ```
 * 
 * ## Multi-Block Lookahead
 * 
 * For motion planning, the parser supports reading ahead:
 * 
 * ```cpp
 * BlockQueue queue = parser.readAhead(50);  // 50 blocks
 * 
 * // Access blocks
 * for (const auto& block : queue) {
 *     processBlock(block);
 * }
 * ```
 * 
 * @see GCodeLexer
 * @see GCodeInterpreter
 */

#pragma once

#include "GCodeTypes.hpp"
#include "GCodeLexer.hpp"
#include "GCodeVariables.hpp"
#include <deque>
#include <memory>
#include <string>

namespace GCode {

// ============================================================================
// Modal Group Definitions
// ============================================================================

/**
 * @brief Modal groups for G-codes
 */
enum class ModalGroup : uint8_t {
    NON_MODAL = 0,      ///< G4, G10, G28, G30, G53, G92, etc.
    MOTION = 1,         ///< G0, G1, G2, G3, G33, G38.x, G73, G76, G80-G89
    PLANE = 2,          ///< G17, G18, G19, G17.1, G18.1, G19.1
    DISTANCE = 3,       ///< G90, G91
    ARC_DISTANCE = 4,   ///< G90.1, G91.1
    FEED_MODE = 5,      ///< G93, G94, G95
    UNITS = 6,          ///< G20, G21
    CUTTER_COMP = 7,    ///< G40, G41, G42, G41.1, G42.1
    TOOL_LENGTH = 8,    ///< G43, G43.1, G43.2, G49
    CANNED_RETURN = 10, ///< G98, G99
    COORD_SYSTEM = 12,  ///< G54-G59.3
    PATH_MODE = 13,     ///< G61, G61.1, G64
    SPINDLE_MODE = 14,  ///< G96, G97
    LATHE_DIAMETER = 15, ///< G7, G8
    LOCAL_OFFSET = 16,  ///< G52 (local coordinate offset)
    COORD_ROTATION = 17, ///< G68, G69 (coordinate system rotation)
    SCALING = 18        ///< G51, G50 (scaling on/off)
};

/**
 * @brief Get modal group for a G-code
 */
ModalGroup getModalGroup(int gcode);

/**
 * @brief Get modal group for a G-code with decimal (e.g., 5.1, 43.2)
 */
ModalGroup getModalGroup(int gcode, int decimal);

// ============================================================================
// Parser Configuration
// ============================================================================

/**
 * @brief Parser configuration
 */
struct ParserConfig {
    /// Maximum blocks in lookahead queue
    size_t maxLookahead{MAX_LOOKAHEAD};
    
    /// Maximum blocks in lookbehind queue
    size_t maxLookbehind{MAX_LOOKBEHIND};
    
    /// Strict modal group checking
    bool strictModalGroups{true};
    
    /// Allow multiple G-codes from same modal group
    bool allowMultipleSameModal{false};
    
    /// Allow omitting axis words when motion mode is active
    bool allowOmittedAxisWords{true};
    
    /// Enable expression evaluation in word values
    bool evaluateExpressions{true};
    
    /// Copy original line text into block
    bool preserveOriginalText{true};
    
    /// LinuxCNC compatibility mode
    bool linuxCNCMode{true};
    
    /// Fanuc compatibility mode
    bool fanucMode{false};
};

// ============================================================================
// Block Queue
// ============================================================================

/**
 * @brief Queue of parsed blocks for lookahead/lookbehind
 */
class BlockQueue {
public:
    BlockQueue(size_t maxSize = MAX_LOOKAHEAD);
    
    /// Add block to front (for lookbehind)
    void pushFront(const Block& block);
    
    /// Add block to back (for lookahead)
    void pushBack(const Block& block);
    
    /// Remove block from front
    Block popFront();
    
    /// Remove block from back
    Block popBack();
    
    /// Get block at index (0 = front)
    Block& operator[](size_t index);
    const Block& operator[](size_t index) const;
    
    /// Get front block
    Block& front();
    const Block& front() const;
    
    /// Get back block
    Block& back();
    const Block& back() const;
    
    /// Number of blocks in queue
    size_t size() const { return m_blocks.size(); }
    
    /// Check if empty
    bool empty() const { return m_blocks.empty(); }
    
    /// Check if full
    bool full() const { return m_blocks.size() >= m_maxSize; }
    
    /// Clear all blocks
    void clear() { m_blocks.clear(); }
    
    /// Get maximum size
    size_t maxSize() const { return m_maxSize; }
    
    /// Iterator support
    auto begin() { return m_blocks.begin(); }
    auto end() { return m_blocks.end(); }
    auto begin() const { return m_blocks.begin(); }
    auto end() const { return m_blocks.end(); }
    
private:
    std::deque<Block> m_blocks;
    size_t m_maxSize;
};

// ============================================================================
// Parser Class
// ============================================================================

/**
 * @brief G-code parser
 *
 * @note Thread safety: A Parser instance is NOT thread-safe and is NOT
 *       reentrant. All access must be performed from a single thread, or
 *       the caller must provide external synchronization. The referenced
 *       VariableSystem must not be modified by another thread during
 *       parsing.
 *
 * @note Lifetime: When input is provided via `setInput(const char*)` or
 *       `(const char*, size_t)`, the parser does NOT copy the buffer; the
 *       caller must keep it alive. Input provided via `loadFile()` is owned
 *       by the parser. Input provided as a temporary `std::string` to
 *       `setInput(std::string_view)` must outlive the parser.
 */
class Parser {
public:
    /**
     * @brief Constructor
     * @param vars Reference to variable system
     * @param config Parser configuration
     */
    explicit Parser(VariableSystem& vars, 
                    const ParserConfig& config = ParserConfig{});
    
    // ========================================================================
    // Input Source
    // ========================================================================
    
    /**
     * @brief Set input source
     * @param source G-code source text
     */
    void setInput(const char* source);
    void setInput(const char* source, size_t length);
    void setInput(std::string_view source);
    
    /**
     * @brief Load input from file
     * @param filename Path to G-code file
     * @return Error if file cannot be read
     */
    Error loadFile(const char* filename);
    
    // ========================================================================
    // Single Block Parsing
    // ========================================================================
    
    /**
     * @brief Parse a single line into a block
     * @param line G-code line to parse
     * @param block Output block
     * @return Error if parsing failed
     */
    Error parseLine(const char* line, Block& block);
    
    /**
     * @brief Parse next block from input
     * @param block Output block
     * @return Error if parsing failed, ErrorCode::END at end of input
     */
    Error parseNextBlock(Block& block);
    
    /**
     * @brief Parse previous block (reverse parsing)
     * @param block Output block
     * @return Error if parsing failed
     */
    Error parsePrevBlock(Block& block);
    
    // ========================================================================
    // Multi-Block Operations
    // ========================================================================
    
    /**
     * @brief Read multiple blocks ahead
     * @param count Number of blocks to read (up to maxLookahead)
     * @return Queue of parsed blocks
     */
    BlockQueue readAhead(size_t count);
    
    /**
     * @brief Read multiple blocks behind (reverse)
     * @param count Number of blocks to read (up to maxLookbehind)
     * @return Queue of parsed blocks (in reverse order)
     */
    BlockQueue readBehind(size_t count);
    
    /**
     * @brief Fill lookahead buffer
     *
     * @deprecated The parser no longer maintains a persistent lookahead
     *             buffer; use readAhead() instead. Kept as a no-op for API
     *             compatibility.
     */
    void fillLookahead();

    // ========================================================================
    // Position Control
    // ========================================================================
    
    /**
     * @brief Seek to specific line
     * @param lineNum 1-based line number
     */
    Error seekToLine(uint32_t lineNum);
    
    /**
     * @brief Seek to start of input
     */
    void seekToStart();
    
    /**
     * @brief Seek to end of input
     */
    void seekToEnd();
    
    /**
     * @brief Get current line number
     */
    uint32_t getCurrentLine() const;
    
    /**
     * @brief Check if at end of input
     */
    bool atEnd() const;
    
    /**
     * @brief Check if at start of input
     */
    bool atStart() const;

    /**
     * @brief Get the underlying lexer (for position tracking)
     */
    Lexer& getLexer() { return m_lexer; }
    const Lexer& getLexer() const { return m_lexer; }

    /**
     * @brief Get current block number (0-based count of parsed blocks)
     */
    size_t getCurrentBlockNumber() const { return m_currentBlockNum; }

    /**
     * @brief Set current block number (for restoring state after scans)
     */
    void setCurrentBlockNumber(size_t n) { m_currentBlockNum = n; }
    
    // ========================================================================
    // O-Code Handling
    // ========================================================================
    
    /**
     * @brief Find matching O-code block
     * 
     * Searches for matching endif, endwhile, endsub, etc.
     * 
     * @param oNumber O-code number
     * @param startType Starting O-code type (if, while, sub)
     * @param block Output: found block
     * @return Error if not found
     */
    Error findMatchingOCode(int32_t oNumber, OCodeType startType, Block& block);
    
    /**
     * @brief Find subroutine definition
     * @param oNumber O-code number
     * @param block Output: sub block
     * @return Error if not found
     */
    Error findSubroutine(int32_t oNumber, Block& block);
    
    /**
     * @brief Find named subroutine
     * @param name Subroutine name
     * @param block Output: sub block
     * @return Error if not found
     */
    Error findSubroutine(const std::string& name, Block& block);
    
    // ========================================================================
    // Validation
    // ========================================================================
    
    /**
     * @brief Validate a block
     * @param block Block to validate
     * @return Error if validation failed
     */
    Error validate(const Block& block) const;
    
    /**
     * @brief Get last error
     */
    const Error& getError() const { return m_error; }
    
    /**
     * @brief Clear error state
     */
    void clearError() { m_error = Error{}; }
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get total block count (after full scan)
     */
    uint32_t getTotalBlocks() const { return m_totalBlocks; }
    
    /**
     * @brief Scan entire input to count blocks
     */
    void scanBlockCount();
    
private:
    ParserConfig m_config;
    VariableSystem& m_vars;
    Lexer m_lexer;
    ExpressionEvaluator m_evaluator;

    // Reusable lexer for parseLine() (avoids per-line allocation).
    Lexer m_lineLexer;

    // File contents owned by the parser when loaded via loadFile(). Kept
    // alive so m_lexer.m_source remains valid across parse calls.
    std::string m_fileContent;

    // Statistics
    uint32_t m_totalBlocks{0};
    uint32_t m_currentBlockNum{0};

    // Error state
    Error m_error;

    // Parsing helpers
    Error parseBlockFromTokens(const std::vector<LexerToken>& tokens, Block& block);
    Error parseWord(const LexerToken& token, Block& block);
    Error parseGCode(double value, Block& block);
    Error parseMCode(double value, Block& block);
    Error parseOCode(const std::vector<LexerToken>& tokens,
                     size_t& index, Block& block);

    // Value evaluation
    Error evaluateWordValue(const LexerToken& token, double& value);

    // Modal group checking
    Error checkModalGroups(const Block& block) const;

    // Build a LexerConfig from the parser config.
    LexerConfig lexerConfigFromParser() const;

    // Error generation
    void setError(ErrorCode code, const char* msg, uint32_t line = 0);
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Parse G-code number from double
 * 
 * Handles G-codes with decimals like G5.1, G43.2
 * 
 * @param value G-code value (e.g., 5.1)
 * @param major Output: major number (e.g., 5)
 * @param minor Output: minor number (e.g., 1), -1 if none
 */
void parseGCodeNumber(double value, int& major, int& minor);

/**
 * @brief Format G-code for display
 */
std::string formatGCode(int major, int minor = -1);

/**
 * @brief Check if G-code is valid
 */
bool isValidGCode(int major, int minor = -1);

/**
 * @brief Check if M-code is valid
 */
bool isValidMCode(int mcode);

/**
 * @brief Get description for G-code
 */
const char* getGCodeDescription(int major, int minor = -1);

/**
 * @brief Get description for M-code
 */
const char* getMCodeDescription(int mcode);

} // namespace GCode
