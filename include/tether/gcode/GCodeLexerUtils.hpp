// SPDX-License-Identifier: MIT
/**
 * @file GCodeLexerUtils.hpp
 * @brief G-code lexer utility functions (token helpers, line helpers, highlighting)
 *
 * @details
 * Provides stateless utility functions that build on top of the GCode::Lexer
 * class:
 * - Token utilities: word-letter/character conversion, axis mapping, formatting
 * - Line-based helpers: tokenizeLine, isEmptyOrComment, stripComments
 * - Syntax highlighting: highlightLine for editors/UIs
 *
 * Extracted from GCodeLexer.cpp to improve modularity.
 */

#pragma once

#include "GCodeLexer.hpp"  // Lexer, LexerToken, LexerConfig, WordLetter, HighlightSpan, Error, ErrorCode

#include <string>
#include <string_view>
#include <vector>

namespace GCode {

// ============================================================================
// Line-based tokenizer helpers
// ============================================================================

/**
 * Tokenize a single G-code line into tokens.
 * @param line   G-code line to tokenize
 * @param tokens Output: vector of tokens
 * @param config Optional lexer configuration
 * @return Error if tokenization failed
 */
Error tokenizeLine(const char* line, std::vector<LexerToken>& tokens,
                   const LexerConfig& config = LexerConfig{});

/**
 * @brief Check if a line is empty or comment-only.
 * @param line Raw G-code line
 * @return `true` when the line carries no commands
 */
bool isEmptyOrComment(const char* line);

/**
 * @brief Strip comments from a line (removes ; and ( ) comments).
 * @param line Raw G-code line
 * @return Line with comments removed
 */
std::string stripComments(const char* line);

// ============================================================================
// Token utilities
// ============================================================================

/**
 * @brief Convert a word letter to its character representation.
 * @param letter WordLetter enum value
 * @return Corresponding character ('X', 'F', ...)
 */
char wordLetterToChar(WordLetter letter);

/**
 * @brief Convert a character to a word letter.
 * @param c Letter character
 * @return Corresponding WordLetter enum value
 */
WordLetter charToWordLetter(char c);

/**
 * @brief Check if a word letter is an axis letter (X/Y/Z/A/B/C/U/V/W).
 * @param letter WordLetter to test
 * @return `true` for motion axis letters
 */
bool isAxisLetter(WordLetter letter);

/**
 * @brief Get the axis index (0-8) from a word letter, or -1 if not an axis.
 * @param letter WordLetter to map
 * @return Axis index or -1
 */
int axisFromWordLetter(WordLetter letter);

/**
 * @brief Format a token for display/debugging.
 * @param token Token to format
 * @return Human-readable token description
 */
std::string formatToken(const LexerToken& token);

// ============================================================================
// Syntax highlighting
// ============================================================================

/**
 * @brief Highlight a single G-code line for editors/UIs.
 * @param line G-code line to highlight
 * @param config Lexer configuration
 * @return Byte ranges into `line` with highlight kinds
 */
std::vector<HighlightSpan> highlightLine(std::string_view line,
                                        const LexerConfig& config = LexerConfig{});

} // namespace GCode
