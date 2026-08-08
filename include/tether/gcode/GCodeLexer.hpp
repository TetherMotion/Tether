/**
 * @file GCodeLexer.hpp
 * @brief G-Code Lexical Analyzer (Tokenizer)
 * 
 * @details
 * The lexer breaks G-code source text into tokens for parsing. It handles:
 * - Word letters (A-Z) with their values
 * - Line numbers (N word)
 * - Comments: parentheses ( ) and semicolon ;
 * - O-codes for control flow
 * - Block delete (/ at line start)
 * - Expression brackets []
 * - Parameter references #
 * 
 * ## Lexer States
 * 
 * ```
 * START ──> WORD_LETTER ──> WORD_VALUE ──> START
 *   │           │
 *   ├──> COMMENT ──> START
 *   │
 *   ├──> OCODE_NUM/NAME ──> OCODE_KEYWORD ──> START
 *   │
 *   └──> EXPRESSION ──> START
 * ```
 * 
 * ## Token Types
 * 
 * | Token | Description | Example |
 * |-------|-------------|---------|
 * | WORD | Letter + value | X100.5, G1, M3 |
 * | OCODE | O-word | O100, O<name> |
 * | KEYWORD | O-code keyword | sub, call, if |
 * | COMMENT | Comment text | (comment), ; text |
 * | EXPRESSION | Bracketed expr | [1+2], [#1*3] |
 * | BLOCK_DELETE | Line delete | / at start |
 * | EOL | End of line | \\n |
 * | END | End of input | EOF |
 * 
 * ## Usage Example
 * 
 * ```cpp
 * GCode::Lexer lexer;
 * lexer.setInput("G1 X100 Y50 F1000 ; rapid move");
 * 
 * while (auto token = lexer.nextToken()) {
 *     switch (token.type) {
 *         case TokenType::WORD:
 *             printf("%c%.4f ", token.letter, token.value);
 *             break;
 *         case TokenType::COMMENT:
 *             printf("// %s\n", token.text);
 *             break;
 *         // ...
 *     }
 * }
 * ```
 * 
 * ## Backward Parsing Support
 * 
 * The lexer supports reverse tokenization for lookbehind functionality:
 * 
 * ```cpp
 * lexer.setInput(programBuffer, programSize);
 * lexer.seekToEnd();  // Start from end
 * 
 * while (auto token = lexer.prevToken()) {
 *     // Process tokens in reverse order
 * }
 * ```
 * 
 * This is essential for:
 * - Negative feed rate lookahead
 * - Reverse path optimization
 * - Finding matching O-code blocks
 * 
 * @see GCodeParser
 */

#pragma once

#include "GCodeTypes.hpp"
#include "GCodeVariables.hpp"
#include <string>
#include <string_view>
#include <optional>
#include <magic_enum/magic_enum.hpp>

namespace GCode {

// ============================================================================
// Lexer Token
// ============================================================================

/**
 * @brief Token type enumeration
 */
enum class LexerTokenType : uint8_t {
    WORD,           ///< Letter + numeric value (G, M, X, Y, etc.)
    OCODE_NUMBER,   ///< O followed by number
    OCODE_NAME,     ///< O<name>
    OCODE_KEYWORD,  ///< sub, endsub, call, if, etc.
    PARAMETER,      ///< #number or #<name>
    EXPRESSION,     ///< [...] bracketed expression
    COMMENT,        ///< ( ) or ; comment
    BLOCK_DELETE,   ///< / at line start
    PERCENT,        ///< % program delimiter
    KEYVALUE,       ///< KEY=VALUE token (multi-letter key)
    EOL,            ///< End of line
    END,            ///< End of input
    ERROR           ///< Lexer error
};

/**
 * Add fields to support KEYVALUE tokens.
 */
struct LexerToken {
    LexerTokenType type{LexerTokenType::END};
    
    /// For WORD tokens: the letter (A-Z)
    WordLetter letter{WordLetter::INVALID};

    /// For WORD tokens: the numeric value
    double value{0.0};

    /// For WORD tokens whose value is an unevaluated expression ([...]) or
    /// parameter reference (#N / #<name>). When true, the parser must
    /// evaluate `expression` / `paramNumber` / `paramName` to obtain `value`.
    bool wordNeedsEval{false};
    
    /// For OCODE_NUMBER: the O-code number
    int32_t oNumber{-1};
    
    /// For OCODE_NAME, OCODE_KEYWORD, COMMENT: text content
    std::string text;
    
    /// For OCODE_KEYWORD: the keyword type
    OCodeType oKeyword{OCodeType::SUB};
    
    /// For PARAMETER: numbered or named flag
    bool isNamedParam{false};
    int32_t paramNumber{-1};
    std::string paramName;

    /// For KEYVALUE: key and value strings
    bool isKeyValue{false};
    std::string kvKey;
    std::string kvValue;
    
    /// For EXPRESSION: the expression text without brackets
    std::string expression;
    
    /// Source location
    uint32_t line{0};
    uint32_t column{0};
    size_t offset{0};  ///< Byte offset in source
    size_t length{0};  ///< Token length in bytes
    
    /// Check if token is valid
    bool valid() const { 
        return type != LexerTokenType::END && type != LexerTokenType::ERROR; 
    }
    
    /// Check if token represents end of line or input
    bool isTerminator() const {
        return type == LexerTokenType::EOL || type == LexerTokenType::END;
    }
};



// ============================================================================
// Syntax Highlighting
// ============================================================================

/**
 * @brief Token kind for syntax highlighting
 */
enum class HighlightKind : uint8_t {
    Unknown = 0,
    Whitespace,
    Comment,
    GCode,
    MCode,
    OCode,
    Axis,
    Word,
    Number,
    Variable,
    Expression,
    Operator,
    Error,
};

/**
 * @brief Highlight span returned by the lexer highlighter
 */
struct HighlightSpan {
    size_t start{0};
    size_t length{0};
    HighlightKind kind{HighlightKind::Unknown};
};

// ============================================================================
// Lexer Configuration
// ============================================================================

/**
 * @brief Lexer configuration options
 */
struct LexerConfig {
    /// Allow spaces within numbers (e.g., "10 00" = 1000)
    bool allowSpacesInNumbers{false};
    
    /// Case insensitive (convert to uppercase)
    bool caseInsensitive{true};
    
    /// Allow both () and ; comments
    bool allowBothCommentStyles{true};
    
    /// Treat % as program delimiter
    bool treatPercentAsDelimiter{true};
    
    /// Allow nested parentheses in comments
    bool allowNestedParentheses{false};
    
    /// Maximum token length
    size_t maxTokenLength{256};
    
    /// Skip block delete lines
    bool skipBlockDelete{false};
};

// ============================================================================
// Lexer Class
// ============================================================================

/**
 * @brief G-code lexical analyzer
 *
 * @note Thread safety: A Lexer instance is NOT thread-safe and is NOT
 *       reentrant. All access (including tokenization, seeking, and error
 *       inspection) must be performed from a single thread, or the caller
 *       must provide external synchronization.
 *
 * @note Lifetime: The lexer does NOT copy the input buffer. The caller must
 *       keep the buffer passed to setInput() alive for the entire lifetime
 *       of the lexer (or until the next setInput() call with a different
 *       buffer).
 */
class Lexer {
public:
    /**
     * @brief Constructor with default config
     */
    Lexer();
    
    /**
     * @brief Constructor with config
     */
    explicit Lexer(const LexerConfig& config);
    
    /**
     * @brief Set input source
     * @param source G-code source string
     */
    void setInput(const char* source);
    void setInput(const char* source, size_t length);
    void setInput(std::string_view source);
    
    /**
     * @brief Get next token
     * @return Next token, type=END at end of input
     */
    LexerToken nextToken();
    
    /**
     * @brief Peek at next token without consuming
     */
    LexerToken peekToken();
    
    /**
     * @brief Get previous token (reverse parsing)
     * @return Previous token, type=END at start of input
     */
    LexerToken prevToken();
    
    /**
     * @brief Read all tokens from current line
     * @return Vector of tokens until EOL
     */
    std::vector<LexerToken> tokenizeLine();
    
    // ========================================================================
    // Position Control
    // ========================================================================
    
    /**
     * @brief Seek to specific position
     * @param offset Byte offset in source
     */
    void seek(size_t offset);
    
    /**
     * @brief Seek to beginning of input
     */
    void seekToStart();
    
    /**
     * @brief Seek to end of input (for reverse parsing)
     */
    void seekToEnd();
    
    /**
     * @brief Seek to specific line number
     * @param lineNum 1-based line number
     * @return true if line found
     */
    bool seekToLine(uint32_t lineNum);
    
    /**
     * @brief Get current position
     */
    size_t getPosition() const { return m_pos; }
    
    /**
     * @brief Get current line number
     */
    uint32_t getLine() const { return m_line; }
    
    /**
     * @brief Get current column
     */
    uint32_t getColumn() const { return m_column; }
    
    /**
     * @brief Check if at end of input
     */
    bool atEnd() const { return m_pos >= m_len; }
    
    /**
     * @brief Check if at start of input
     */
    bool atStart() const { return m_pos == 0; }
    
    /**
     * @brief Get remaining input
     */
    std::string_view remaining() const;
    
    // ========================================================================
    // Line Operations
    // ========================================================================
    
    /**
     * @brief Skip to next line
     */
    void skipLine();
    
    /**
     * @brief Skip to previous line (for reverse parsing)
     */
    void prevLine();
    
    /**
     * @brief Get current line text
     */
    std::string getCurrentLineText() const;
    
    /**
     * @brief Check if current position is start of line
     */
    bool atLineStart() const;
    
    // ========================================================================
    // Error Handling
    // ========================================================================
    
    /**
     * @brief Get last error
     */
    const Error& getError() const { return m_error; }
    
    /**
     * @brief Check if lexer has error
     */
    bool hasError() const { return m_error.code != ErrorCode::OK; }
    
    /**
     * @brief Clear error state
     */
    void clearError() { m_error = Error{}; }
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get total line count (after full scan)
     */
    uint32_t getTotalLines() const { return m_totalLines; }
    
    /**
     * @brief Scan entire input to count lines
     */
    void scanLineCount();
    
private:
    LexerConfig m_config;
    
    // Source management
    const char* m_source{nullptr};
    size_t m_len{0};
    size_t m_pos{0};

    // Position tracking
    uint32_t m_line{1};
    uint32_t m_column{1};
    uint32_t m_totalLines{0};
    bool m_atLineStart{true};
    // Byte offset of the start of the current line. Maintained incrementally
    // in advance()/seek()/retreat() so that findLineStart(m_pos) is O(1) on
    // the hot path (forward parsing).
    size_t m_lineStartPos{0};
    // True when the previous token was OCODE_NUMBER or OCODE_NAME, so the
    // next token should be lexed as an O-code keyword (sub/call/if/etc.).
    bool m_afterOCode{false};

    // Lookahead token for peek
    std::optional<LexerToken> m_peeked;
    
    // Error state
    Error m_error;
    
    // Character access
    char current() const;
    char peek(size_t ahead = 1) const;
    void advance(size_t count = 1);
    void retreat(size_t count = 1);
    
    // Token scanning
    LexerToken scanWord();
    LexerToken scanNumber(bool allowSign = true);
    LexerToken scanOCode();
    LexerToken scanParameter();
    LexerToken scanExpression();
    LexerToken scanParenComment();
    LexerToken scanLineComment();
    LexerToken scanOCodeKeyword();
    
    // Helper functions
    void skipWhitespace();
    void skipToEndOfLine();
    bool isWordLetter(char c) const;
    bool isDigit(char c) const;
    bool isAlpha(char c) const;
    bool isAlnum(char c) const;
    WordLetter charToWordLetter(char c) const;
    OCodeType stringToOKeyword(const std::string& s) const;
    
    // Error generation
    LexerToken makeError(const char* msg);
    void setError(ErrorCode code, const char* msg);
    
    // Reverse scanning helpers
    void findPrevTokenStart();
    size_t findLineStart(size_t pos) const;
    size_t findLineEnd(size_t pos) const;
};

} // namespace GCode
