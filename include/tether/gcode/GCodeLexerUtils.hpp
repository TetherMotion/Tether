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

/// Tokenize a single G-code line into tokens.
/// @param line   G-code line to tokenize
/// @param tokens Output: vector of tokens
/// @param config Optional lexer configuration
/// @return Error if tokenization failed
Error tokenizeLine(const char* line, std::vector<LexerToken>& tokens,
                   const LexerConfig& config = LexerConfig{});

/// Check if a line is empty or comment-only.
bool isEmptyOrComment(const char* line);

/// Strip comments from a line (removes ; and ( ) comments).
std::string stripComments(const char* line);

// ============================================================================
// Token utilities
// ============================================================================

/// Convert a word letter to its character representation.
char wordLetterToChar(WordLetter letter);

/// Convert a character to a word letter.
WordLetter charToWordLetter(char c);

/// Check if a word letter is an axis letter (X/Y/Z/A/B/C/U/V/W).
bool isAxisLetter(WordLetter letter);

/// Get the axis index (0-8) from a word letter, or -1 if not an axis.
int axisFromWordLetter(WordLetter letter);

/// Format a token for display/debugging.
std::string formatToken(const LexerToken& token);

// ============================================================================
// Syntax highlighting
// ============================================================================

/// Highlight a single G-code line for editors/UIs.
/// Returns byte ranges into the input line with highlight kinds.
std::vector<HighlightSpan> highlightLine(std::string_view line,
                                        const LexerConfig& config = LexerConfig{});

} // namespace GCode
