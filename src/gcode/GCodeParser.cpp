#include "tether/gcode/GCodeParser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace {

static void set_error(GCode::Error& err, GCode::ErrorCode code, uint32_t line, const char* msg, const char* ctx = nullptr) {
    err.code = code;
    err.line = line;
    // snprintf null-terminates, so the explicit zero-fill is unnecessary.
    if (msg) {
        std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    } else {
        err.message[0] = '\0';
    }
    if (ctx) {
        std::snprintf(err.context.data(), err.context.size(), "%s", ctx);
    } else {
        err.context[0] = '\0';
    }
}

} // namespace

namespace GCode {

// ============================================================================
// Modal group helpers
// ============================================================================

ModalGroup getModalGroup(int gcode) {
    // The internal representation is usually major*10+minor. For grouping we
    // want the major number.
    const int major = gcode / 10;

    switch (major) {
        // Motion (modal group 1): G0-G3, G33, G38.x, G73, G74, G76, G80-G89
        case 0:
        case 1:
        case 2:
        case 3:
        case 33:
        case 38:
        case 73:
        case 74:
        case 76:
        case 80:
        case 81:
        case 82:
        case 83:
        case 84:
        case 85:
        case 86:
        case 87:
        case 88:
        case 89:
            return ModalGroup::MOTION;
        // Plane (modal group 2): G17-G19
        case 17:
        case 18:
        case 19:
            return ModalGroup::PLANE;
        // Distance (modal group 3): G90, G91
        case 90:
        case 91:
            return ModalGroup::DISTANCE;
        // Feed mode (modal group 5): G93, G94, G95
        case 93:
        case 94:
        case 95:
            return ModalGroup::FEED_MODE;
        // Units (modal group 6): G20, G21
        case 20:
        case 21:
            return ModalGroup::UNITS;
        // Cutter compensation (modal group 7): G40, G41, G42
        case 40:
        case 41:
        case 42:
            return ModalGroup::CUTTER_COMP;
        // Tool length (modal group 8): G43, G49
        case 43:
        case 49:
            return ModalGroup::TOOL_LENGTH;
        // Canned return (modal group 10): G98, G99
        case 98:
        case 99:
            return ModalGroup::CANNED_RETURN;
        // Coordinate system (modal group 12): G54-G59
        case 54:
        case 55:
        case 56:
        case 57:
        case 58:
        case 59:
            return ModalGroup::COORD_SYSTEM;
        // Path control (modal group 13): G61, G64
        case 61:
        case 64:
            return ModalGroup::PATH_MODE;
        // Spindle mode (modal group 14): G96, G97
        case 96:
        case 97:
            return ModalGroup::SPINDLE_MODE;
        // Lathe diameter (modal group 15): G7, G8
        case 7:
        case 8:
            return ModalGroup::LATHE_DIAMETER;
        // Local offset (modal group 16): G52
        case 52:
            return ModalGroup::LOCAL_OFFSET;
        // Coordinate rotation (modal group 17): G68, G69
        case 68:
        case 69:
            return ModalGroup::COORD_ROTATION;
        // Scaling (modal group 18): G51, G50
        case 51:
        case 50:
            return ModalGroup::SCALING;
        default:
            // G4 (dwell), G10, G28, G30, G53, G92, etc. are non-modal.
            return ModalGroup::NON_MODAL;
    }
}

ModalGroup getModalGroup(int gcode, int decimal) {
    // Honor the minor/decimal digit so decimal G-codes whose major collides
    // with another group are classified correctly (e.g. G90.1 vs G90).
    if (decimal > 0) {
        const int major = gcode / 10;
        // G90.1 / G91.1 -> arc distance mode (group 4)
        if ((major == 90 || major == 91)) {
            return ModalGroup::ARC_DISTANCE;
        }
        // G17.1 / G18.1 / G19.1 -> still plane selection.
        if (major == 17 || major == 18 || major == 19) {
            return ModalGroup::PLANE;
        }
        // G41.1 / G42.1 -> cutter comp; G43.1 / G43.2 -> tool length;
        // G61.1 -> path mode. These share their major's group already.
    }
    return getModalGroup(gcode);
}

// ============================================================================
// BlockQueue
// ============================================================================

BlockQueue::BlockQueue(size_t maxSize)
    : m_blocks()
    , m_maxSize(maxSize) {
}

void BlockQueue::pushFront(const Block& block) {
    if (m_blocks.size() >= m_maxSize) {
        m_blocks.pop_back();
    }
    m_blocks.push_front(block);
}

void BlockQueue::pushBack(const Block& block) {
    if (m_blocks.size() >= m_maxSize) {
        m_blocks.pop_front();
    }
    m_blocks.push_back(block);
}

Block BlockQueue::popFront() {
    if (m_blocks.empty()) {
        return Block{};
    }
    Block b = m_blocks.front();
    m_blocks.pop_front();
    return b;
}

Block BlockQueue::popBack() {
    if (m_blocks.empty()) {
        return Block{};
    }
    Block b = m_blocks.back();
    m_blocks.pop_back();
    return b;
}

Block& BlockQueue::operator[](size_t index) {
    if (index >= m_blocks.size()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks[index];
}

const Block& BlockQueue::operator[](size_t index) const {
    if (index >= m_blocks.size()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks[index];
}

Block& BlockQueue::front() {
    if (m_blocks.empty()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks.front();
}

const Block& BlockQueue::front() const {
    if (m_blocks.empty()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks.front();
}

Block& BlockQueue::back() {
    if (m_blocks.empty()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks.back();
}

const Block& BlockQueue::back() const {
    if (m_blocks.empty()) {
        static Block sentinel;
        return sentinel;
    }
    return m_blocks.back();
}

// ============================================================================
// Parser
// ============================================================================

Parser::Parser(VariableSystem& vars, const ParserConfig& config)
    : m_config(config)
    , m_vars(vars)
    , m_lexer(lexerConfigFromParser())
    , m_evaluator(vars)
    , m_lineLexer(lexerConfigFromParser()) {
}

LexerConfig Parser::lexerConfigFromParser() const {
    LexerConfig cfg;
    cfg.caseInsensitive = true;
    cfg.allowSpacesInNumbers = false;
    cfg.allowBothCommentStyles = true;
    cfg.treatPercentAsDelimiter = true;
    cfg.allowNestedParentheses = false;
    cfg.skipBlockDelete = false;
    (void)m_config; // currently no direct mapping; reserved for future dialect knobs
    return cfg;
}

void Parser::setInput(const char* source) {
    m_fileContent.clear(); // no longer owning file content
    m_lexer.setInput(source);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
    clearError();
}

void Parser::setInput(const char* source, size_t length) {
    m_fileContent.clear();
    m_lexer.setInput(source, length);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
    clearError();
}

void Parser::setInput(std::string_view source) {
    m_fileContent.clear();
    m_lexer.setInput(source);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
    clearError();
}

Error Parser::loadFile(const char* filename) {
    Error err;
    if (!filename) {
        set_error(err, ErrorCode::FILE_NOT_FOUND, 0, "Null filename");
        return err;
    }

    std::ifstream f(filename);
    if (!f.is_open()) {
        set_error(err, ErrorCode::FILE_NOT_FOUND, 0, "File not found", filename);
        return err;
    }

    std::ostringstream ss;
    ss << f.rdbuf();
    // Store contents in a member so the lexer's non-owning pointer stays
    // valid for the lifetime of the parser. Reset input from the owned buffer.
    m_fileContent = ss.str();
    m_lexer.setInput(m_fileContent.data(), m_fileContent.size());
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
    clearError();
    return Error{};
}

Error Parser::parseLine(const char* line, Block& block) {
    Error err;

    block = Block{};

    if (!line) {
        set_error(err, ErrorCode::SYNTAX_ERROR, 0, "Null input line");
        return err;
    }

    if (m_config.preserveOriginalText) {
        std::snprintf(block.originalText.data(), block.originalText.size(), "%s", line);
    }

    // Tokenize using a reusable member lexer (configured from ParserConfig)
    // so Parser::setInput/parseNextBlock don't interfere with this line's lex.
    m_lineLexer.setInput(line);

    const auto tokens = m_lineLexer.tokenizeLine();
    if (m_lineLexer.hasError()) {
        return m_lineLexer.getError();
    }

    return parseBlockFromTokens(tokens, block);
}

Error Parser::parseNextBlock(Block& block) {
    Error err;

    if (m_lexer.atEnd()) {
        err.code = ErrorCode::END;
        err.line = getCurrentLine();
        err.message.fill(0);
        err.context.fill(0);
        std::snprintf(err.message.data(), err.message.size(), "%s", "End of input");
        return err;
    }

    // Get current line slice
    const std::string lineText = m_lexer.getCurrentLineText();
    const uint32_t sourceLine = m_lexer.getLine();
    m_lexer.skipLine();

    err = parseLine(lineText.c_str(), block);
    block.sourceLineNumber = sourceLine;
    ++m_currentBlockNum;
    if (err.ok()) {
        clearError();
    } else {
        m_error = err;
    }

    return err;
}

Error Parser::parsePrevBlock(Block& block) {
    // Reverse parsing: move to the previous line start and parse it.
    // Return END when already at the start of input (no previous line).
    if (m_lexer.atStart()) {
        Error err;
        err.code = ErrorCode::END;
        err.line = getCurrentLine();
        err.message.fill(0);
        err.context.fill(0);
        std::snprintf(err.message.data(), err.message.size(), "%s", "Start of input");
        return err;
    }

    const size_t posBefore = m_lexer.getPosition();
    m_lexer.prevLine();
    // If prevLine didn't move us, we're at the first line — no previous block.
    if (m_lexer.getPosition() == posBefore) {
        Error err;
        err.code = ErrorCode::END;
        err.line = getCurrentLine();
        err.message.fill(0);
        err.context.fill(0);
        std::snprintf(err.message.data(), err.message.size(), "%s", "Start of input");
        return err;
    }

    const std::string lineText = m_lexer.getCurrentLineText();
    const uint32_t sourceLine = m_lexer.getLine();
    Error err = parseLine(lineText.c_str(), block);
    block.sourceLineNumber = sourceLine;
    if (m_currentBlockNum > 0) {
        --m_currentBlockNum;
    }
    if (err.ok()) {
        clearError();
    } else {
        m_error = err;
    }
    return err;
}

BlockQueue Parser::readAhead(size_t count) {
    BlockQueue q(std::min(count, m_config.maxLookahead));

    // Save full parser state so lookahead does not corrupt it.
    const size_t savedPos = m_lexer.getPosition();
    const uint32_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;

    for (size_t i = 0; i < count; ++i) {
        Block b;
        Error e = parseNextBlock(b);
        if (e) break; // includes END
        q.pushBack(b);
    }

    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
    return q;
}

BlockQueue Parser::readBehind(size_t count) {
    BlockQueue q(std::min(count, m_config.maxLookbehind));

    const size_t savedPos = m_lexer.getPosition();
    const uint32_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;

    for (size_t i = 0; i < count; ++i) {
        Block b;
        Error e = parsePrevBlock(b);
        if (e) break; // includes END
        q.pushBack(b);
    }

    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
    return q;
}

void Parser::fillLookahead() {
    // Deprecated no-op; readAhead() returns lookahead by value.
}

Error Parser::seekToLine(uint32_t lineNum) {
    Error err;
    if (!m_lexer.seekToLine(lineNum)) {
           set_error(err, ErrorCode::INVALID_LINE_NUMBER, lineNum, "Line not found");
    }
    return err;
}

void Parser::seekToStart() {
    m_lexer.seekToStart();
}

void Parser::seekToEnd() {
    m_lexer.seekToEnd();
}

uint32_t Parser::getCurrentLine() const {
    return m_lexer.getLine();
}

bool Parser::atEnd() const {
    return m_lexer.atEnd();
}

bool Parser::atStart() const {
    return m_lexer.atStart();
}

Error Parser::findMatchingOCode(int32_t oNumber, OCodeType startType, Block& block) {
    // Scan forward from the current block to find the matching closer/keyword
    // for the given O-code number and start type.
    Error err;
    OCodeType targetType;
    switch (startType) {
        case OCodeType::IF:       targetType = OCodeType::ENDIF; break;
        case OCodeType::WHILE:    targetType = OCodeType::ENDWHILE; break;
        case OCodeType::DO:       targetType = OCodeType::WHILE; break;
        case OCodeType::REPEAT:   targetType = OCodeType::ENDREPEAT; break;
        case OCodeType::SUB:      targetType = OCodeType::ENDSUB; break;
        default:
            set_error(err, ErrorCode::INVALID_OCODE, getCurrentLine(),
                      "Unknown O-code block type for matching");
            return err;
    }

    // Save lexer state so we can restore after scanning.
    const size_t savedPos = m_lexer.getPosition();
    const uint32_t savedLine = m_lexer.getLine();
    const size_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;

    int depth = 0;
    Block b;
    while (true) {
        Error e = parseNextBlock(b);
        if (e) {
            if (e.code == ErrorCode::END) {
                // Reached end of input — no match found
                break;
            }
            // Skip parse errors (e.g. standalone parameter assignments)
            // and continue scanning for the matching O-code block.
            continue;
        }
        if (b.hasOCode && b.oCodeNumber == oNumber) {
            if (b.oCodeType == startType) {
                ++depth;
            } else if (b.oCodeType == targetType) {
                --depth;
                if (depth == 0) {
                    block = b;
                    m_lexer.seek(savedPos);
                    m_currentBlockNum = savedBlockNum;
                    m_error = savedError;
                    return Error{};
                }
            }
        }
    }

    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
    set_error(err, ErrorCode::INVALID_OCODE, savedLine,
              "No matching O-code block found");
    return err;
}

Error Parser::findSubroutine(int32_t oNumber, Block& block) {
    Error err;
    const size_t savedPos = m_lexer.getPosition();
    const size_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;

    m_lexer.seekToStart();
    m_currentBlockNum = 0;
    Block b;
    while (true) {
        Error e = parseNextBlock(b);
        if (e) {
            if (e.code == ErrorCode::END) break;
            continue;  // skip parse errors
        }
        if (b.hasOCode && !b.oCodeIsNamed &&
            b.oCodeNumber == oNumber &&
            b.oCodeType == OCodeType::SUB) {
            block = b;
            m_lexer.seek(savedPos);
            m_currentBlockNum = savedBlockNum;
            m_error = savedError;
            return Error{};
        }
    }

    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
    set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, getCurrentLine(),
              "Subroutine not found");
    return err;
}

Error Parser::findSubroutine(const std::string& name, Block& block) {
    Error err;
    const size_t savedPos = m_lexer.getPosition();
    const size_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;

    m_lexer.seekToStart();
    m_currentBlockNum = 0;
    Block b;
    while (true) {
        Error e = parseNextBlock(b);
        if (e) {
            if (e.code == ErrorCode::END) break;
            continue;  // skip parse errors
        }
        if (b.hasOCode && b.oCodeIsNamed &&
            name == b.oCodeName.data() &&
            b.oCodeType == OCodeType::SUB) {
            block = b;
            m_lexer.seek(savedPos);
            m_currentBlockNum = savedBlockNum;
            m_error = savedError;
            return Error{};
        }
    }

    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
    set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, getCurrentLine(),
              "Named subroutine not found");
    return err;
}

Error Parser::validate(const Block& block) const {
    // Basic structural validation: ensure G/M code counts are within their
    // fixed array bounds (defensive — parseGCode/parseMCode already guard
    // this, but validate() may be called on externally-constructed blocks).
    if (block.gCodeCount > block.gCodes.size()) {
        Error err;
        set_error(err, ErrorCode::SYNTAX_ERROR, block.sourceLineNumber,
                  "Too many G-codes in block");
        return err;
    }
    if (block.mCodeCount > block.mCodes.size()) {
        Error err;
        set_error(err, ErrorCode::SYNTAX_ERROR, block.sourceLineNumber,
                  "Too many M-codes in block");
        return err;
    }
    return Error{};
}

void Parser::scanBlockCount() {
    const size_t savedPos = m_lexer.getPosition();
    const uint32_t savedBlockNum = m_currentBlockNum;
    const Error savedError = m_error;
    m_totalBlocks = 0;
    seekToStart();
    Block b;
    while (true) {
        Error e = parseNextBlock(b);
        if (e) break;
        ++m_totalBlocks;
    }
    m_lexer.seek(savedPos);
    m_currentBlockNum = savedBlockNum;
    m_error = savedError;
}

Error Parser::parseBlockFromTokens(const std::vector<LexerToken>& tokens, Block& block) {
    Error err;

    for (const auto& tok : tokens) {
        switch (tok.type) {
            case LexerTokenType::BLOCK_DELETE:
                block.blockDelete = true;
                break;
            case LexerTokenType::COMMENT:
                block.hasComment = true;
                std::snprintf(block.comment.data(), block.comment.size(), "%s", tok.text.c_str());
                break;
            case LexerTokenType::OCODE_NUMBER:
                block.hasOCode = true;
                block.oCodeIsNamed = false;
                block.oCodeNumber = tok.oNumber;
                break;
            case LexerTokenType::OCODE_NAME:
                block.hasOCode = true;
                block.oCodeIsNamed = true;
                std::snprintf(block.oCodeName.data(), block.oCodeName.size(), "%s", tok.text.c_str());
                break;
            case LexerTokenType::OCODE_KEYWORD:
                block.hasOCode = true;
                block.oCodeType = tok.oKeyword;
                break;
            case LexerTokenType::KEYVALUE:
                // store key/value string pair (kvValue contains raw value including brackets/quotes when present)
                if (tok.isKeyValue) {
                    // Trim quotes for stored string if value is quoted (keep inner content)
                    std::string v = tok.kvValue;
                    if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\''))) {
                        v = v.substr(1, v.size() - 2);
                    }
                    block.keyValues[tok.kvKey] = v;
                }
                break;
            case LexerTokenType::WORD:
                err = parseWord(tok, block);
                if (err) return err;
                break;
            case LexerTokenType::PARAMETER:
            case LexerTokenType::EXPRESSION:
                // A standalone parameter/expression (not attached to a word
                // letter) is not valid G-code — unless this block has an
                // O-code, in which case the expression is the condition or
                // call argument (handled by parseOCode below).
                if (!block.hasOCode) {
                    set_error(err, ErrorCode::SYNTAX_ERROR, getCurrentLine(),
                              "Parameter or expression without a preceding word letter");
                    return err;
                }
                // O-code conditions/args are extracted by parseOCode.
                break;
            case LexerTokenType::PERCENT:
                // Program delimiter — ignore.
                break;
            default:
                // OCODE_KEYWORD and any other token types are not handled here.
                break;
        }
    }

    if (m_config.strictModalGroups) {
        err = checkModalGroups(block);
        if (err) return err;
    }

    // If this block has an O-code, extract the condition/args expression.
    if (block.hasOCode) {
        size_t idx = 0;
        err = parseOCode(tokens, idx, block);
        if (err) return err;
    }

    return Error{};
}

Error Parser::parseWord(const LexerToken& token, Block& block) {
    if (token.type != LexerTokenType::WORD) {
        return Error{};
    }

    const WordLetter letter = token.letter;

    // Resolve the word value, evaluating expressions/parameters if present.
    double value = token.value;
    if (token.wordNeedsEval) {
        Error evalErr = evaluateWordValue(token, value);
        if (evalErr) return evalErr;
    }

    if (letter == WordLetter::G) {
        return parseGCode(value, block);
    }
    if (letter == WordLetter::M) {
        return parseMCode(value, block);
    }
    if (letter == WordLetter::N) {
        // Range-validate before narrowing to int32_t.
        const double rounded = std::llround(value);
        if (rounded < -2147483648.0 || rounded > 2147483647.0) {
            Error err;
            set_error(err, ErrorCode::INVALID_LINE_NUMBER, getCurrentLine(),
                      "Line number out of range");
            return err;
        }
        block.lineNumber = static_cast<int32_t>(rounded);
        return Error{};
    }

    const uint8_t idx = static_cast<uint8_t>(letter);
    if (idx < 26) {
        block.words[idx].letter = letter;
        block.words[idx].value = value;
        block.words[idx].present = true;
    }

    return Error{};
}

Error Parser::parseGCode(double value, Block& block) {
    Error err;

    if (block.gCodeCount >= block.gCodes.size()) {
        set_error(err, ErrorCode::SYNTAX_ERROR, getCurrentLine(), "Too many G-codes in one block");
        return err;
    }

    int major = 0;
    int minor = -1;
    parseGCodeNumber(value, major, minor);

    // Range-validate before narrowing. G-codes are encoded as major*10+minor
    // into an int16_t, so the encoded value must fit [-32768, 32767].
    const long long encoded = static_cast<long long>(major) * 10 + std::max(minor, 0);
    if (encoded < -32768 || encoded > 32767) {
        set_error(err, ErrorCode::UNKNOWN_GCODE, getCurrentLine(),
                  "G-code number out of range");
        return err;
    }

    const int16_t code = static_cast<int16_t>(encoded);
    block.gCodes[block.gCodeCount++] = code;
    return Error{};
}

Error Parser::parseMCode(double value, Block& block) {
    Error err;

    if (block.mCodeCount >= block.mCodes.size()) {
        set_error(err, ErrorCode::SYNTAX_ERROR, getCurrentLine(), "Too many M-codes in one block");
        return err;
    }

    // Range-validate before narrowing to int16_t.
    const double rounded = static_cast<double>(std::llround(value));
    if (rounded < -32768.0 || rounded > 32767.0) {
        set_error(err, ErrorCode::UNKNOWN_MCODE, getCurrentLine(),
                  "M-code number out of range");
        return err;
    }
    const int16_t code = static_cast<int16_t>(static_cast<long long>(rounded));
    block.mCodes[block.mCodeCount++] = code;
    return Error{};
}

Error Parser::parseOCode(const std::vector<LexerToken>& tokens, size_t& index, Block& block) {
    // parseOCode is called after the OCODE_NUMBER/OCODE_NAME and OCODE_KEYWORD
    // tokens have been consumed by parseBlockFromTokens. This method extracts
    // the condition expression (for if/elseif/while) or call arguments.
    //
    // The remaining tokens from index onward are scanned for:
    //   - EXPRESSION tokens (conditions for if/elseif/while, or call args)
    //   - WORD tokens (repeat count for O<n> repeat [count], or args for call)
    //
    // The condition/args are stored in block.oCodeCondition (as a string).
    (void)index; // parseBlockFromTokens already iterated all tokens

    Error err;

    // Find the expression token (if any) in the token list
    for (const auto& tok : tokens) {
        if (tok.type == LexerTokenType::EXPRESSION) {
            // Store the expression text as the O-code condition
            std::snprintf(block.oCodeCondition.data(),
                          block.oCodeCondition.size(),
                          "%s", tok.expression.c_str());
            break;
        }
    }

    return Error{};
}

Error Parser::evaluateWordValue(const LexerToken& token, double& value) {
    // Resolve a word value that the lexer left unevaluated because it was an
    // expression ([...]) or parameter reference (#N / #<name>).
    if (!token.wordNeedsEval) {
        value = token.value;
        return Error{};
    }

    if (!token.expression.empty()) {
        // Wrap in brackets; ExpressionEvaluator accepts an optional outer pair.
        std::string expr = "[" + token.expression + "]";
        double r = 0.0;
        Error e = m_evaluator.evaluate(expr.c_str(), r);
        if (e) return e;
        value = r;
        return Error{};
    }

    // Parameter reference.
    if (token.isNamedParam) {
        auto v = m_vars.getNamed(token.paramName);
        value = v.value_or(0.0);
        return Error{};
    }
    if (token.paramNumber >= 0) {
        value = m_vars.get(token.paramNumber);
        return Error{};
    }

    value = token.value;
    return Error{};
}

Error Parser::checkModalGroups(const Block& block) const {
    // Detect more than one G-code from the same modal group in a single block.
    // Non-modal codes (group NON_MODAL) are exempted.
    for (uint8_t i = 0; i < block.gCodeCount; ++i) {
        const int16_t code = block.gCodes[i];
        const ModalGroup g = getModalGroup(code);
        if (g == ModalGroup::NON_MODAL) continue;
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < block.gCodeCount; ++j) {
            if (getModalGroup(block.gCodes[j]) == g) {
                Error err;
                set_error(err, ErrorCode::CONFLICTING_WORDS, 0,
                          "Multiple G-codes from the same modal group in one block");
                return err;
            }
        }
    }
    return Error{};
}

void Parser::setError(ErrorCode code, const char* msg, uint32_t line) {
    set_error(m_error, code, line, msg);
}

// ============================================================================
// Utility functions
// ============================================================================

void parseGCodeNumber(double value, int& major, int& minor) {
    // Guard against overflow when narrowing a large double to int.
    if (!std::isfinite(value) || value < -2.0e9 || value > 2.0e9) {
        major = 0;
        minor = -1;
        return;
    }
    major = static_cast<int>(std::floor(value + 1e-9));
    const double frac = value - static_cast<double>(major);

    if (std::fabs(frac) < 1e-9) {
        minor = -1;
        return;
    }

    minor = static_cast<int>(std::llround(frac * 10.0));
}

std::string formatGCode(int major, int minor) {
    if (minor < 0) {
        return "G" + std::to_string(major);
    }
    return "G" + std::to_string(major) + "." + std::to_string(minor);
}

bool isValidGCode(int major, int minor) {
    if (major < 0) return false;
    // Reject clearly out-of-range major numbers. The known valid range is
    // 0..99 (with minor 0..9); anything beyond that is not a standard G-code.
    if (major > 999) return false;
    if (minor < -1 || minor > 9) return false;
    return true;
}

bool isValidMCode(int mcode) {
    // Standard M-codes are 0..199 (user codes 100..199). Negative or
    // out-of-range values are invalid.
    return mcode >= 0 && mcode <= 999;
}

const char* getGCodeDescription(int major, int minor) {
    (void)minor;
    switch (major) {
        case 0: return "Rapid positioning";
        case 1: return "Linear interpolation";
        case 2: return "Clockwise arc";
        case 3: return "Counter-clockwise arc";
        default: return "G-code";
    }
}

const char* getMCodeDescription(int mcode) {
    switch (mcode) {
        case 0: return "Program stop";
        case 2: return "Program end";
        case 3: return "Spindle on (CW)";
        case 5: return "Spindle stop";
        default: return "M-code";
    }
}

} // namespace GCode
