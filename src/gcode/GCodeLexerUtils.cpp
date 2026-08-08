// SPDX-License-Identifier: MIT
/**
 * @file GCodeLexerUtils.cpp
 * @brief G-code lexer utility functions (token helpers, line helpers, highlighting)
 *
 * @details
 * Extracted from GCodeLexer.cpp.  These functions are either fully stateless
 * (token utilities) or create temporary Lexer instances (line helpers,
 * highlighting) and have no dependency on Lexer instance state beyond the
 * public API.
 */

#include "tether/gcode/GCodeLexerUtils.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <sstream>

namespace {

inline char to_upper_ascii(char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

} // namespace

namespace GCode {

// ============================================================================
// Line-based tokenizer helpers
// ============================================================================

Error tokenizeLine(const char* line, std::vector<LexerToken>& tokens, const LexerConfig& config) {
    tokens.clear();
    if (!line) {
        Error err;
        err.code = ErrorCode::SYNTAX_ERROR;
        err.line = 0;
        err.message.fill(0);
        std::snprintf(err.message.data(), err.message.size(), "%s", "Null line");
        return err;
    }

    Lexer lexer(config);
    lexer.setInput(line);

    while (true) {
        LexerToken t = lexer.nextToken();
        if (t.type == LexerTokenType::END || t.type == LexerTokenType::EOL) {
            break;
        }
        if (t.type == LexerTokenType::ERROR) {
            return lexer.getError();
        }
        tokens.push_back(std::move(t));
    }

    return Error{};
}

bool isEmptyOrComment(const char* line) {
    if (!line) return true;
    Lexer lexer;
    lexer.setInput(line);

    while (true) {
        LexerToken t = lexer.nextToken();
        if (t.type == LexerTokenType::END || t.type == LexerTokenType::EOL) {
            return true;
        }
        if (t.type == LexerTokenType::COMMENT) {
            return true;
        }
        if (t.type == LexerTokenType::WORD || t.type == LexerTokenType::OCODE_NUMBER ||
            t.type == LexerTokenType::OCODE_NAME || t.type == LexerTokenType::PARAMETER ||
            t.type == LexerTokenType::EXPRESSION) {
            return false;
        }
    }
}

std::string stripComments(const char* line) {
    if (!line) return {};

    std::string s(line);

    // Remove semicolon comment
    const size_t semi = s.find(';');
    if (semi != std::string::npos) {
        s.resize(semi);
    }

    // Remove ( ... ) comments (non-nested). A stray ')' with no matching '('
    // is also removed so the output stays valid G-code.
    while (true) {
        const size_t open = s.find('(');
        const size_t close = s.find(')');
        if (open == std::string::npos && close == std::string::npos) break;
        if (open == std::string::npos) {
            // Stray ')' with no preceding '(' — drop it.
            s.erase(close, 1);
            continue;
        }
        if (close == std::string::npos || close < open) {
            // No closing ')' for this '(' (or a stray ')' before it): truncate
            // at the '(' and drop any stray ')' that precedes it.
            if (close != std::string::npos && close < open) {
                s.erase(close, 1);
                continue;
            }
            s.resize(open);
            break;
        }
        s.erase(open, close - open + 1);
    }

    return s;
}

// ============================================================================
// Token utilities
// ============================================================================

char wordLetterToChar(WordLetter letter) {
    if (letter == WordLetter::INVALID) return '?';
    const uint8_t v = static_cast<uint8_t>(letter);
    if (v < 26) {
        return static_cast<char>('A' + v);
    }
    // Non A-Z letters
    switch (letter) {
        case WordLetter::DOLLAR: return '$';
        case WordLetter::COMMENT: return ';';
        case WordLetter::PERCENT: return '%';
        default: return '?';
    }
}

WordLetter charToWordLetter(char c) {
    const char u = to_upper_ascii(c);
    if (u >= 'A' && u <= 'Z') {
        return static_cast<WordLetter>(static_cast<uint8_t>(u - 'A'));
    }
    if (u == '$') return WordLetter::DOLLAR;
    if (u == '%') return WordLetter::PERCENT;
    return WordLetter::INVALID;
}

bool isAxisLetter(WordLetter letter) {
    return letter == WordLetter::X || letter == WordLetter::Y || letter == WordLetter::Z ||
           letter == WordLetter::A || letter == WordLetter::B || letter == WordLetter::C ||
           letter == WordLetter::U || letter == WordLetter::V || letter == WordLetter::W;
}

int axisFromWordLetter(WordLetter letter) {
    switch (letter) {
        case WordLetter::X: return 0;
        case WordLetter::Y: return 1;
        case WordLetter::Z: return 2;
        case WordLetter::A: return 3;
        case WordLetter::B: return 4;
        case WordLetter::C: return 5;
        case WordLetter::U: return 6;
        case WordLetter::V: return 7;
        case WordLetter::W: return 8;
        default: return -1;
    }
}

std::string formatToken(const LexerToken& token) {
    std::ostringstream oss;
    oss << magic_enum::enum_name(token.type) << "@" << token.line << ":" << token.column;
    if (token.type == LexerTokenType::WORD) {
        oss << " " << wordLetterToChar(token.letter) << token.value;
    } else if (token.type == LexerTokenType::COMMENT) {
        oss << " (" << token.text << ")";
    } else if (token.type == LexerTokenType::PARAMETER) {
        if (token.isNamedParam) {
            oss << " #<" << token.paramName << ">";
        } else {
            oss << " #" << token.paramNumber;
        }
    }
    return oss.str();
}

// ============================================================================
// Syntax highlighting
// ============================================================================

std::vector<HighlightSpan> highlightLine(std::string_view line, const LexerConfig& config) {
    std::vector<HighlightSpan> spans;

    Lexer lexer(config);
    lexer.setInput(line);

    size_t lastEnd = 0;

    while (true) {
        LexerToken t = lexer.nextToken();
        if (t.type == LexerTokenType::END || t.type == LexerTokenType::EOL) {
            break;
        }

        if (t.type == LexerTokenType::ERROR) {
            spans.push_back(HighlightSpan{t.offset, std::max<size_t>(t.length, 1), HighlightKind::Error});
            break;
        }

        // Fill whitespace gaps
        if (t.offset > lastEnd) {
            spans.push_back(HighlightSpan{lastEnd, t.offset - lastEnd, HighlightKind::Whitespace});
        }

        HighlightKind kind = HighlightKind::Unknown;
        switch (t.type) {
            case LexerTokenType::COMMENT:
                kind = HighlightKind::Comment;
                break;
            case LexerTokenType::PARAMETER:
                kind = HighlightKind::Variable;
                break;
            case LexerTokenType::EXPRESSION:
                kind = HighlightKind::Expression;
                break;
            case LexerTokenType::OCODE_NUMBER:
            case LexerTokenType::OCODE_NAME:
            case LexerTokenType::OCODE_KEYWORD:
                kind = HighlightKind::OCode;
                break;
            case LexerTokenType::WORD:
                if (t.letter == WordLetter::G) kind = HighlightKind::GCode;
                else if (t.letter == WordLetter::M) kind = HighlightKind::MCode;
                else if (isAxisLetter(t.letter)) kind = HighlightKind::Axis;
                else kind = HighlightKind::Word;
                break;
            default:
                kind = HighlightKind::Unknown;
                break;
        }

        spans.push_back(HighlightSpan{t.offset, std::max<size_t>(t.length, 1), kind});
        lastEnd = t.offset + std::max<size_t>(t.length, 1);

        if (t.type == LexerTokenType::COMMENT) {
            // Comments consume rest of line; highlight trailing part as comment if any
            if (lastEnd < line.size()) {
                spans.back().length = line.size() - spans.back().start;
            }
            break;
        }
    }

    // Trailing whitespace
    if (lastEnd < line.size()) {
        spans.push_back(HighlightSpan{lastEnd, line.size() - lastEnd, HighlightKind::Whitespace});
    }

    // Remove empty spans
    spans.erase(std::remove_if(spans.begin(), spans.end(), [](const HighlightSpan& s) {
        return s.length == 0;
    }), spans.end());

    return spans;
}

} // namespace GCode
