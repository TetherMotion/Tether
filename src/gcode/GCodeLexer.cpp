#include "tether/gcode/GCodeLexer.hpp"
#include "tether/gcode/GCodeLexerUtils.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <sstream>

namespace {

inline char to_upper_ascii(char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

inline bool is_space(char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0;
}

} // namespace

namespace GCode {

// ============================================================================
// Lexer
// ============================================================================

Lexer::Lexer() : Lexer(LexerConfig{}) {}

Lexer::Lexer(const LexerConfig& config)
    : m_config(config) {
}

void Lexer::setInput(const char* source) {
    if (!source) {
        m_source = nullptr;
        m_len = 0;
        m_pos = 0;
        m_line = 1;
        m_column = 1;
        m_atLineStart = true;
        m_lineStartPos = 0;
        m_afterOCode = false;
        m_peeked.reset();
        clearError();
        return;
    }

    setInput(source, std::strlen(source));
}

void Lexer::setInput(const char* source, size_t length) {
    m_source = source;
    m_len = source ? length : 0;
    m_pos = 0;
    m_line = 1;
    m_column = 1;
    m_totalLines = 0;
    m_atLineStart = true;
    m_lineStartPos = 0;
    m_afterOCode = false;
    m_peeked.reset();
    clearError();
}

void Lexer::setInput(std::string_view source) {
    setInput(source.data(), source.size());
}

std::string_view Lexer::remaining() const {
    if (!m_source || m_pos >= m_len) {
        return std::string_view{};
    }
    return std::string_view{m_source + m_pos, m_len - m_pos};
}

void Lexer::seek(size_t offset) {
    if (!m_source) {
        m_pos = 0;
        m_line = 1;
        m_column = 1;
        m_atLineStart = true;
        m_lineStartPos = 0;
        m_afterOCode = false;
        m_peeked.reset();
        return;
    }

    const size_t target = std::min(offset, m_len);

    // Relative seek: move forward/backward by the delta only, maintaining
    // line/column incrementally. This keeps small rollbacks (the common case
    // in nextToken/scanWord) O(delta) instead of O(n).
    if (target >= m_pos) {
        // Forward: reuse advance() which maintains all position state.
        const size_t fwd = target - m_pos;
        for (size_t i = 0; i < fwd; ++i) {
            advance();
        }
    } else if (target < m_pos) {
        // Backward: walk back, counting newlines to decrement line, then
        // recompute column from the new line start.
        size_t back = m_pos - target;
        size_t p = m_pos;
        uint32_t line = m_line;
        while (back > 0 && p > 0) {
            --p;
            --back;
            if (m_source[p] == '\n') {
                --line;
            }
        }
        m_pos = p;
        m_line = line;
        // Recompute line start and column from the new position.
        size_t ls = p;
        while (ls > 0 && m_source[ls - 1] != '\n') --ls;
        m_lineStartPos = ls;
        m_column = static_cast<uint32_t>(p - ls) + 1;
        m_atLineStart = (p == ls);
    }

    m_afterOCode = false;
    m_peeked.reset();
}

void Lexer::seekToStart() {
    seek(0);
}

void Lexer::seekToEnd() {
    seek(m_len);
}

bool Lexer::seekToLine(uint32_t lineNum) {
    if (!m_source || lineNum < 1) {
        return false;
    }

    if (lineNum == 1) {
        seekToStart();
        return true;
    }

    uint32_t currentLine = 1;
    for (size_t i = 0; i < m_len; ++i) {
        if (m_source[i] == '\n') {
            ++currentLine;
            if (currentLine == lineNum) {
                seek(i + 1);
                return true;
            }
        }
    }

    return false;
}

void Lexer::skipLine() {
    if (!m_source) return;
    m_peeked.reset();
    while (m_pos < m_len && m_source[m_pos] != '\n') {
        advance();
    }
    if (m_pos < m_len && m_source[m_pos] == '\n') {
        advance();
    }
}

void Lexer::prevLine() {
    if (!m_source || m_pos == 0) return;
    m_peeked.reset();

    // Move to start of current line first (handles mid-line correctly).
    size_t start = findLineStart(m_pos);
    if (start == 0) {
        // Already on the first line; go to start of input.
        seekToStart();
        return;
    }

    // Move to start of previous line (the character before `start` is a '\n'
    // or belongs to the previous line).
    size_t prevStart = findLineStart(start - 1);
    seek(prevStart);
}

std::string Lexer::getCurrentLineText() const {
    if (!m_source) return {};
    const size_t start = findLineStart(m_pos);
    const size_t end = findLineEnd(m_pos);
    return std::string{m_source + start, end - start};
}

bool Lexer::atLineStart() const {
    return m_atLineStart;
}

void Lexer::scanLineCount() {
    if (!m_source) {
        m_totalLines = 0;
        return;
    }

    uint32_t lines = 1;
    for (size_t i = 0; i < m_len; ++i) {
        if (m_source[i] == '\n') {
            ++lines;
        }
    }
    m_totalLines = lines;
}

LexerToken Lexer::peekToken() {
    if (m_peeked.has_value()) {
        return *m_peeked;
    }

    const size_t savedPos = m_pos;
    const uint32_t savedLine = m_line;
    const uint32_t savedCol = m_column;
    const bool savedAtLineStart = m_atLineStart;
    const size_t savedLineStartPos = m_lineStartPos;

    LexerToken tok = nextToken();
    m_peeked = tok;

    // Restore state (without seeking, to keep this O(1)).
    m_pos = savedPos;
    m_line = savedLine;
    m_column = savedCol;
    m_atLineStart = savedAtLineStart;
    m_lineStartPos = savedLineStartPos;

    return tok;
}

LexerToken Lexer::nextToken() {
    if (m_peeked.has_value()) {
        LexerToken t = *m_peeked;
        m_peeked.reset();
        // Consume the same token by seeking to its end (O(delta) forward).
        if (t.length > 0) {
            seek(t.offset + t.length);
        }
        return t;
    }

    if (!m_source || m_pos >= m_len) {
        return LexerToken{};
    }

    // Loop here so that unknown characters are skipped iteratively rather
    // than recursively (avoids stack overflow on long runs of garbage).
    while (true) {
        skipWhitespace();

        if (m_pos >= m_len) {
            return LexerToken{};
        }

        const char c0 = current();

    // Newline -> EOL
    if (c0 == '\n') {
        m_afterOCode = false;
        LexerToken tok;
        tok.type = LexerTokenType::EOL;
        tok.offset = m_pos;
        tok.length = 1;
        tok.line = m_line;
        tok.column = m_column;
        advance();
        return tok;
    }

    // Block delete at line start
    if (m_atLineStart && c0 == '/') {
        m_afterOCode = false;
        LexerToken tok;
        tok.type = LexerTokenType::BLOCK_DELETE;
        tok.offset = m_pos;
        tok.length = 1;
        tok.line = m_line;
        tok.column = m_column;
        advance();
        return tok;
    }

    // Percent delimiter
    if (m_config.treatPercentAsDelimiter && c0 == '%') {
        m_afterOCode = false;
        LexerToken tok;
        tok.type = LexerTokenType::PERCENT;
        tok.offset = m_pos;
        tok.length = 1;
        tok.line = m_line;
        tok.column = m_column;
        advance();
        return tok;
    }

    // Line comment
    if (c0 == ';') {
        m_afterOCode = false;
        return scanLineComment();
    }

    // Parenthesis comment
    if (c0 == '(') {
        m_afterOCode = false;
        return scanParenComment();
    }

    // Expression
    if (c0 == '[') {
        m_afterOCode = false;
        return scanExpression();
    }

    // Parameter
    if (c0 == '#') {
        m_afterOCode = false;
        return scanParameter();
    }

    // Word, O-code or KEY=VALUE (multi-letter key)
    if (isAlpha(c0)) {
        // After an O-code number/name, the next identifier is the keyword
        if (m_afterOCode) {
            m_afterOCode = false;
            return scanOCodeKeyword();
        }
        const char c = m_config.caseInsensitive ? to_upper_ascii(c0) : c0;
        if (c == 'O') {
            LexerToken tok = scanOCode();
            m_afterOCode = (tok.type == LexerTokenType::OCODE_NUMBER ||
                            tok.type == LexerTokenType::OCODE_NAME);
            return tok;
        }
        // Try to detect a multi-letter KEY=VALUE token first (e.g., FOO=1, NAME="x")
        const size_t savePos = m_pos;
        // Read identifier (letters and underscores)
        size_t idStart = m_pos;
        while (m_pos < m_len && isAlpha(current())) {
            advance();
        }
        const size_t idEnd = m_pos;
        // Skip optional whitespace
        while (m_pos < m_len && is_space(current())) advance();
        if (m_pos < m_len && current() == '=') {
            // We have KEY = ... ; parse value similarly to editor rules
            LexerToken tok;
            tok.type = LexerTokenType::KEYVALUE;
            tok.offset = idStart;
            tok.line = m_line;
            tok.column = static_cast<uint32_t>(m_column);
            tok.isKeyValue = true;
            tok.kvKey.assign(m_source + idStart, idEnd - idStart);
            // consume '='
            advance();
            // skip whitespace
            while (m_pos < m_len && is_space(current())) advance();

            // parse value (allow empty value when '=' at end)
            if (m_pos >= m_len) {
                // empty value
                LexerToken tok2;
                tok2.type = LexerTokenType::KEYVALUE;
                tok2.offset = idStart;
                tok2.line = m_line;
                tok2.column = static_cast<uint32_t>(m_column);
                tok2.isKeyValue = true;
                tok2.kvKey.assign(m_source + idStart, idEnd - idStart);
                tok2.kvValue.clear();
                tok2.length = m_pos - idStart;
                return tok2;
            }

            char vc = current();
            if (vc == '"' || vc == '\'') {
                // quoted string
                const char quote = vc;
                size_t vstart = m_pos;
                advance(); // skip leading quote
                while (m_pos < m_len && current() != quote) advance();
                if (m_pos < m_len) advance(); // consume trailing quote
                size_t vend = m_pos;
                tok.kvValue.assign(m_source + vstart, vend - vstart);
                tok.length = vend - idStart;
                return tok;
            } else if (vc == '[') {
                // bracketed vector
                size_t vstart = m_pos;
                advance(); // skip '['
                while (m_pos < m_len && current() != ']') advance();
                if (m_pos < m_len) advance(); // skip ']'
                size_t vend = m_pos;
                tok.kvValue.assign(m_source + vstart, vend - vstart);
                tok.length = vend - idStart;
                return tok;
            } else if (vc == '(') {
                // parenthesis vector
                size_t vstart = m_pos;
                advance(); // skip '('
                while (m_pos < m_len && current() != ')') advance();
                if (m_pos < m_len) advance(); // skip ')'
                size_t vend = m_pos;
                tok.kvValue.assign(m_source + vstart, vend - vstart);
                tok.length = vend - idStart;
                return tok;
            } else {
                // unquoted scalar — consume until whitespace or comment start
                size_t vstart = m_pos;
                while (m_pos < m_len && !is_space(current()) && current() != ';' && current() != '(' && current() != ')') advance();
                size_t vend = m_pos;
                tok.kvValue.assign(m_source + vstart, vend - vstart);
                tok.length = vend - idStart;
                return tok;
            }
        }
        // Not a KEY=VALUE token — rollback and parse as a normal word
        seek(savePos);
        return scanWord();
    }

        // Unknown character: skip and continue the loop (iterative, no
        // recursion) to avoid stack overflow on long runs of garbage.
        advance();
    }
}

LexerToken Lexer::prevToken() {
    if (!m_source || m_pos == 0) {
        return LexerToken{};
    }

    // Simple (but correct) implementation: re-lex from start up to current pos.
    Lexer tmp(m_config);
    tmp.setInput(m_source, m_len);
    tmp.seekToStart();

    LexerToken last;
    while (true) {
        LexerToken t = tmp.nextToken();
        if (t.type == LexerTokenType::END) break;
        if (t.offset + t.length >= m_pos) break;
        last = t;
        if (t.type == LexerTokenType::EOL) {
            // continue
        }
    }

    if (last.type == LexerTokenType::END) {
        return LexerToken{};
    }

    seek(last.offset);
    return last;
}

std::vector<LexerToken> Lexer::tokenizeLine() {
    std::vector<LexerToken> tokens;

    while (true) {
        LexerToken t = nextToken();
        if (t.type == LexerTokenType::END) {
            break;
        }
        if (t.type == LexerTokenType::EOL) {
            break;
        }
        if (t.type == LexerTokenType::ERROR) {
            break;
        }
        if (t.type == LexerTokenType::BLOCK_DELETE && m_config.skipBlockDelete) {
            skipToEndOfLine();
            break;
        }
        tokens.push_back(std::move(t));
    }

    return tokens;
}

char Lexer::current() const {
    if (!m_source || m_pos >= m_len) return '\0';
    return m_source[m_pos];
}

char Lexer::peek(size_t ahead) const {
    if (!m_source) return '\0';
    const size_t p = m_pos + ahead;
    if (p >= m_len) return '\0';
    return m_source[p];
}

void Lexer::advance(size_t count) {
    for (size_t i = 0; i < count && m_pos < m_len; ++i) {
        const char c = m_source[m_pos++];
        if (c == '\n') {
            ++m_line;
            m_column = 1;
            m_atLineStart = true;
            // Newline just consumed: next position starts a new line.
            m_lineStartPos = m_pos;
        } else {
            ++m_column;
            m_atLineStart = false;
        }
    }
}

void Lexer::retreat(size_t count) {
    // Walk backward, maintaining line/column/lineStart incrementally.
    while (count > 0 && m_pos > 0) {
        --count;
        --m_pos;
        if (m_source[m_pos] == '\n') {
            --m_line;
            // Recompute line start for the (now current) line.
            size_t ls = m_pos;
            while (ls > 0 && m_source[ls - 1] != '\n') --ls;
            m_lineStartPos = ls;
            m_column = static_cast<uint32_t>(m_pos - ls) + 1;
            m_atLineStart = (m_pos == ls);
        } else {
            if (m_column > 1) --m_column;
            m_atLineStart = false;
        }
    }
}

void Lexer::skipWhitespace() {
    while (m_pos < m_len) {
        const char c = current();
        if (c == '\n') return;
        if (!is_space(c)) return;
        advance();
    }
}

void Lexer::skipToEndOfLine() {
    while (m_pos < m_len && current() != '\n') {
        advance();
    }
}

bool Lexer::isWordLetter(char c) const {
    const char u = to_upper_ascii(c);
    return u >= 'A' && u <= 'Z';
}

bool Lexer::isDigit(char c) const {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
}

bool Lexer::isAlpha(char c) const {
    return std::isalpha(static_cast<unsigned char>(c)) != 0;
}

bool Lexer::isAlnum(char c) const {
    return std::isalnum(static_cast<unsigned char>(c)) != 0;
}

WordLetter Lexer::charToWordLetter(char c) const {
    return GCode::charToWordLetter(c);
}

OCodeType Lexer::stringToOKeyword(const std::string& s) const {
    std::string u;
    u.reserve(s.size());
    for (char c : s) u.push_back(to_upper_ascii(c));

    if (u == "SUB") return OCodeType::SUB;
    if (u == "ENDSUB") return OCodeType::ENDSUB;
    if (u == "CALL") return OCodeType::CALL;
    if (u == "IF") return OCodeType::IF;
    if (u == "ELSEIF") return OCodeType::ELSEIF;
    if (u == "ELSE") return OCodeType::ELSE;
    if (u == "ENDIF") return OCodeType::ENDIF;
    if (u == "WHILE") return OCodeType::WHILE;
    if (u == "ENDWHILE") return OCodeType::ENDWHILE;
    if (u == "DO") return OCodeType::DO;
    if (u == "REPEAT") return OCodeType::REPEAT;
    if (u == "ENDREPEAT") return OCodeType::ENDREPEAT;
    if (u == "RETURN") return OCodeType::RETURN;
    if (u == "BREAK") return OCodeType::BREAK;
    if (u == "CONTINUE") return OCodeType::CONTINUE;

    return OCodeType::SUB;
}

LexerToken Lexer::makeError(const char* msg) {
    LexerToken tok;
    tok.type = LexerTokenType::ERROR;
    tok.offset = m_pos;
    tok.length = 1;
    tok.line = m_line;
    tok.column = m_column;
    setError(ErrorCode::SYNTAX_ERROR, msg);
    tok.text = msg ? msg : "Lexer error";
    return tok;
}

void Lexer::setError(ErrorCode code, const char* msg) {
    m_error.code = code;
    m_error.line = m_line;
    m_error.message.fill(0);
    if (msg) {
        std::snprintf(m_error.message.data(), m_error.message.size(), "%s", msg);
    }
}

LexerToken Lexer::scanNumber(bool allowSign) {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    bool hasDecimal = false;

    if (allowSign && (current() == '+' || current() == '-')) {
        advance();
    }

    while (m_pos < m_len) {
        char c = current();
        if (isDigit(c)) {
            advance();
            continue;
        }
        if (c == '.' && !hasDecimal) {
            hasDecimal = true;
            advance();
            continue;
        }
        if (m_config.allowSpacesInNumbers && is_space(c) && c != '\n') {
            advance();
            continue;
        }
        break;
    }

    const size_t end = m_pos;
    tok.length = end - start;

    // Build the numeric string, stripping any internal spaces consumed when
    // allowSpacesInNumbers is enabled (e.g. "1 000" -> "1000"). strtod/from_chars
    // stop at the first whitespace, so we must remove spaces before parsing.
    std::string s;
    s.reserve(end - start);
    for (size_t i = start; i < end; ++i) {
        char c = m_source[i];
        if (c != ' ' && c != '\t') s.push_back(c);
    }

    // Parse with from_chars (locale-independent, unlike strtod). Fall back to
    // strtod only if from_chars cannot handle the form (it does not accept a
    // leading '+' on all libstdc++ versions prior to C++23).
    double v = 0.0;
    const char* sbeg = s.c_str();
    const char* send = sbeg + s.size();
    auto r = std::from_chars(sbeg, send, v);
    if (r.ec != std::errc() || r.ptr != send) {
        // from_chars failed (e.g. leading '+' or hex); fall back to strtod.
        char* endPtr = nullptr;
        v = std::strtod(s.c_str(), &endPtr);
        if (endPtr == s.c_str()) {
            // No digits consumed at all (e.g. input was "+" or ".").
            tok.value = 0.0;
            tok.type = LexerTokenType::WORD; // caller overrides / checks length
            return tok;
        }
    }
    tok.value = v;
    tok.type = LexerTokenType::WORD; // caller overrides
    return tok;
}

LexerToken Lexer::scanWord() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    char c = current();
    c = m_config.caseInsensitive ? to_upper_ascii(c) : c;

    tok.type = LexerTokenType::WORD;
    tok.letter = charToWordLetter(c);

    advance();
    skipWhitespace();

    // A word value may be a bracketed expression ([...]) or a parameter
    // reference (#N / #<name>). In those cases the value is not yet
    // evaluated by the lexer; the token carries the raw expression/param
    // text and the parser evaluates it via ExpressionEvaluator.
    if (m_pos < m_len && current() == '[') {
        LexerToken exprTok = scanExpression();
        if (exprTok.type == LexerTokenType::ERROR) {
            return exprTok;
        }
        tok.expression = std::move(exprTok.expression);
        tok.wordNeedsEval = true;
        tok.value = 0.0; // resolved by parser
        tok.length = (m_pos - start);
        return tok;
    }
    if (m_pos < m_len && current() == '#') {
        LexerToken paramTok = scanParameter();
        if (paramTok.type == LexerTokenType::ERROR) {
            return paramTok;
        }
        tok.isNamedParam = paramTok.isNamedParam;
        tok.paramNumber = paramTok.paramNumber;
        tok.paramName = std::move(paramTok.paramName);
        tok.wordNeedsEval = true;
        tok.value = 0.0; // resolved by parser
        tok.length = (m_pos - start);
        return tok;
    }

    const size_t numStart = m_pos;
    LexerToken numTok = scanNumber(true);
    if (numTok.length == 0) {
        seek(numStart);
        return makeError("Missing value after word letter");
    }

    tok.value = numTok.value;
    tok.length = (m_pos - start);
    return tok;
}

LexerToken Lexer::scanOCode() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // Consume 'O'
    advance();
    skipWhitespace();

    if (current() == '<') {
        // Named O-code: O<name>
        advance();
        const size_t nameStart = m_pos;
        while (m_pos < m_len && current() != '>' && current() != '\n') {
            advance();
        }
        if (current() != '>') {
            return makeError("Unterminated O<name>");
        }
        const size_t nameEnd = m_pos;
        advance(); // consume '>'

        tok.type = LexerTokenType::OCODE_NAME;
        tok.text.assign(m_source + nameStart, nameEnd - nameStart);
        tok.length = m_pos - start;
        return tok;
    }

    // Numbered O-code: O123
    const size_t numStart = m_pos;
    LexerToken numTok = scanNumber(true);
    if (numTok.length == 0) {
        seek(numStart);
        return makeError("Missing O-code number");
    }

    tok.type = LexerTokenType::OCODE_NUMBER;
    tok.oNumber = static_cast<int32_t>(std::llround(numTok.value));
    tok.length = m_pos - start;
    return tok;
}

LexerToken Lexer::scanParameter() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // Consume '#'
    advance();

    if (current() == '<') {
        // Named: #<name>
        advance();
        const size_t nameStart = m_pos;
        while (m_pos < m_len && current() != '>' && current() != '\n') {
            advance();
        }
        if (current() != '>') {
            return makeError("Unterminated #<name>");
        }
        const size_t nameEnd = m_pos;
        advance(); // consume '>'

        tok.type = LexerTokenType::PARAMETER;
        tok.isNamedParam = true;
        tok.paramName.assign(m_source + nameStart, nameEnd - nameStart);
        tok.length = m_pos - start;
        return tok;
    }

    // Numbered: #123
    const size_t numStart = m_pos;
    LexerToken numTok = scanNumber(true);
    if (numTok.length == 0) {
        seek(numStart);
        return makeError("Missing parameter number");
    }

    tok.type = LexerTokenType::PARAMETER;
    tok.isNamedParam = false;
    tok.paramNumber = static_cast<int32_t>(std::llround(numTok.value));
    tok.length = m_pos - start;
    return tok;
}

LexerToken Lexer::scanExpression() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // Consume '['
    advance();

    const size_t exprStart = m_pos;
    int depth = 1;
    while (m_pos < m_len) {
        char c = current();
        if (c == '[') depth++;
        else if (c == ']') {
            depth--;
            if (depth == 0) break;
        }
        if (c == '\n') break;
        advance();
    }

    if (m_pos >= m_len || current() != ']') {
        return makeError("Unterminated expression");
    }

    const size_t exprEnd = m_pos;
    advance(); // consume ']'

    tok.type = LexerTokenType::EXPRESSION;
    tok.expression.assign(m_source + exprStart, exprEnd - exprStart);
    tok.length = m_pos - start;
    return tok;
}

LexerToken Lexer::scanParenComment() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // Consume '('
    advance();

    const size_t txtStart = m_pos;
    int depth = 1;
    while (m_pos < m_len) {
        char c = current();
        if (c == '\n') break;
        if (c == '(' && m_config.allowNestedParentheses) depth++;
        if (c == ')') {
            depth--;
            if (depth == 0) break;
        }
        advance();
    }

    if (m_pos >= m_len || current() != ')') {
        return makeError("Unterminated (comment)");
    }

    const size_t txtEnd = m_pos;
    advance(); // consume ')'

    tok.type = LexerTokenType::COMMENT;
    tok.text.assign(m_source + txtStart, txtEnd - txtStart);
    tok.length = m_pos - start;
    return tok;
}

LexerToken Lexer::scanLineComment() {
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // consume ';'
    advance();
    const size_t txtStart = m_pos;
    while (m_pos < m_len && current() != '\n') {
        advance();
    }
    const size_t txtEnd = m_pos;

    tok.type = LexerTokenType::COMMENT;
    tok.text.assign(m_source + txtStart, txtEnd - txtStart);
    tok.length = m_pos - start;
    return tok;
}

LexerToken Lexer::scanOCodeKeyword() {
    // Read an identifier (letters only) following an O-code number/name.
    // Produces an OCODE_KEYWORD token with the keyword type.
    LexerToken tok;
    const size_t start = m_pos;
    tok.offset = start;
    tok.line = m_line;
    tok.column = m_column;

    // Read letters [a-zA-Z]+
    while (m_pos < m_len && isAlpha(current())) {
        advance();
    }

    if (m_pos == start) {
        return makeError("Expected O-code keyword (sub, call, if, etc.)");
    }

    std::string kw(m_source + start, m_pos - start);
    tok.type = LexerTokenType::OCODE_KEYWORD;
    tok.text = kw;
    tok.oKeyword = stringToOKeyword(kw);
    tok.length = m_pos - start;
    return tok;
}

void Lexer::findPrevTokenStart() {
    // This lexer provides a fallback prevToken() implementation, so we don't
    // need an efficient backwards scanner here.
}

size_t Lexer::findLineStart(size_t pos) const {
    if (!m_source) return 0;
    if (pos > m_len) pos = m_len;
    // Fast path: the current position's line start is tracked incrementally.
    if (pos == m_pos) {
        return m_lineStartPos;
    }
    while (pos > 0) {
        if (m_source[pos - 1] == '\n') break;
        --pos;
    }
    return pos;
}

size_t Lexer::findLineEnd(size_t pos) const {
    if (!m_source) return 0;
    if (pos > m_len) pos = m_len;
    while (pos < m_len) {
        if (m_source[pos] == '\n') break;
        ++pos;
    }
    return pos;
}

} // namespace GCode
