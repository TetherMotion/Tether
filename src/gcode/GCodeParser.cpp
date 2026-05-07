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
    err.message.fill(0);
    err.context.fill(0);
    if (msg) {
        std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    }
    if (ctx) {
        std::snprintf(err.context.data(), err.context.size(), "%s", ctx);
    }
}

} // namespace

namespace GCode {

// ============================================================================
// Modal group helpers
// ============================================================================

ModalGroup getModalGroup(int gcode) {
    // The internal representation is usually major*10+minor. For grouping we want major.
    const int major = gcode / 10;

    // Minimal modal grouping used by current host tooling.
    switch (major) {
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
        case 17:
        case 18:
        case 19:
            return ModalGroup::PLANE;
        case 90:
        case 91:
            return ModalGroup::DISTANCE;
        case 93:
        case 94:
        case 95:
            return ModalGroup::FEED_MODE;
        case 20:
        case 21:
            return ModalGroup::UNITS;
        case 61:
        case 64:
            return ModalGroup::PATH_MODE;
        default:
            return ModalGroup::NON_MODAL;
    }
}

ModalGroup getModalGroup(int gcode, int decimal) {
    (void)decimal;
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
    Block b = m_blocks.front();
    m_blocks.pop_front();
    return b;
}

Block BlockQueue::popBack() {
    Block b = m_blocks.back();
    m_blocks.pop_back();
    return b;
}

Block& BlockQueue::operator[](size_t index) {
    return m_blocks[index];
}

const Block& BlockQueue::operator[](size_t index) const {
    return m_blocks[index];
}

Block& BlockQueue::front() {
    return m_blocks.front();
}

const Block& BlockQueue::front() const {
    return m_blocks.front();
}

Block& BlockQueue::back() {
    return m_blocks.back();
}

const Block& BlockQueue::back() const {
    return m_blocks.back();
}

// ============================================================================
// Parser
// ============================================================================

Parser::Parser(VariableSystem& vars, const ParserConfig& config)
    : m_config(config)
    , m_vars(vars)
    , m_lexer(LexerConfig{})
    , m_evaluator(vars)
    , m_lookahead(config.maxLookahead)
    , m_lookbehind(config.maxLookbehind) {
}

void Parser::setInput(const char* source) {
    m_lexer.setInput(source);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
}

void Parser::setInput(const char* source, size_t length) {
    m_lexer.setInput(source, length);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
}

void Parser::setInput(std::string_view source) {
    m_lexer.setInput(source);
    m_currentBlockNum = 0;
    m_totalBlocks = 0;
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
    const std::string content = ss.str();
    setInput(content);
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

    // Tokenize using a local lexer so Parser::setInput/parseNextBlock don't interfere.
    LexerConfig lexCfg;
    lexCfg.caseInsensitive = true;
    lexCfg.allowSpacesInNumbers = false;
    Lexer lex(lexCfg);
    lex.setInput(line);

    const auto tokens = lex.tokenizeLine();
    if (lex.hasError()) {
        return lex.getError();
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

    return err;
}

Error Parser::parsePrevBlock(Block& block) {
    // Naive reverse: move to previous line start and parse.
    m_lexer.prevLine();
    const std::string lineText = m_lexer.getCurrentLineText();
    const uint32_t sourceLine = m_lexer.getLine();
    Error err = parseLine(lineText.c_str(), block);
    block.sourceLineNumber = sourceLine;
    return err;
}

BlockQueue Parser::readAhead(size_t count) {
    BlockQueue q(std::min(count, m_config.maxLookahead));

    const size_t savedPos = m_lexer.getPosition();

    for (size_t i = 0; i < count; ++i) {
        Block b;
        Error e = parseNextBlock(b);
        if (e) break;
        q.pushBack(b);
    }

    m_lexer.seek(savedPos);
    return q;
}

BlockQueue Parser::readBehind(size_t count) {
    BlockQueue q(std::min(count, m_config.maxLookbehind));

    const size_t savedPos = m_lexer.getPosition();

    for (size_t i = 0; i < count; ++i) {
        Block b;
        Error e = parsePrevBlock(b);
        if (e) break;
        q.pushBack(b);
    }

    m_lexer.seek(savedPos);
    return q;
}

void Parser::fillLookahead() {
    // Not needed for current host usage.
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
    (void)oNumber;
    (void)startType;
    (void)block;
    Error err;
        set_error(err, ErrorCode::INVALID_OCODE, getCurrentLine(), "O-code matching not implemented");
    return err;
}

Error Parser::findSubroutine(int32_t oNumber, Block& block) {
    (void)oNumber;
    (void)block;
    Error err;
    set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, getCurrentLine(), "Subroutine search not implemented");
    return err;
}

Error Parser::findSubroutine(const std::string& name, Block& block) {
    (void)name;
    (void)block;
    Error err;
    set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, getCurrentLine(), "Subroutine search not implemented");
    return err;
}

Error Parser::validate(const Block& block) const {
    (void)block;
    return Error{};
}

void Parser::scanBlockCount() {
    const size_t savedPos = m_lexer.getPosition();
    m_totalBlocks = 0;
    seekToStart();
    Block b;
    while (true) {
        Error e = parseNextBlock(b);
        if (e) break;
        ++m_totalBlocks;
    }
    m_lexer.seek(savedPos);
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
            default:
                break;
        }
    }

    if (m_config.strictModalGroups) {
        err = checkModalGroups(block);
        if (err) return err;
    }

    return Error{};
}

Error Parser::parseWord(const LexerToken& token, Block& block) {
    if (token.type != LexerTokenType::WORD) {
        return Error{};
    }

    const WordLetter letter = token.letter;

    if (letter == WordLetter::G) {
        return parseGCode(token.value, block);
    }
    if (letter == WordLetter::M) {
        return parseMCode(token.value, block);
    }
    if (letter == WordLetter::N) {
        block.lineNumber = static_cast<int32_t>(std::llround(token.value));
        return Error{};
    }

    const uint8_t idx = static_cast<uint8_t>(letter);
    if (idx < 26) {
        block.words[idx].letter = letter;
        block.words[idx].value = token.value;
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

    const int16_t code = static_cast<int16_t>(major * 10 + std::max(minor, 0));
    block.gCodes[block.gCodeCount++] = code;
    return Error{};
}

Error Parser::parseMCode(double value, Block& block) {
    Error err;

    if (block.mCodeCount >= block.mCodes.size()) {
        set_error(err, ErrorCode::SYNTAX_ERROR, getCurrentLine(), "Too many M-codes in one block");
        return err;
    }

    const int16_t code = static_cast<int16_t>(std::llround(value));
    block.mCodes[block.mCodeCount++] = code;
    return Error{};
}

Error Parser::parseOCode(const std::vector<LexerToken>& tokens, size_t& index, Block& block) {
    (void)tokens;
    (void)index;
    (void)block;
    Error err;
        set_error(err, ErrorCode::INVALID_OCODE, getCurrentLine(), "O-code parsing not implemented");
    return err;
}

Error Parser::evaluateWordValue(const LexerToken& token, double& value) {
    // For now, values are already numeric in lexer tokens.
    value = token.value;
    return Error{};
}

Error Parser::checkModalGroups(const Block& block) const {
    (void)block;
    return Error{};
}

void Parser::setError(ErrorCode code, const char* msg, uint32_t line) {
    set_error(m_error, code, line, msg);
}

// ============================================================================
// Utility functions
// ============================================================================

void parseGCodeNumber(double value, int& major, int& minor) {
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
    (void)minor;
    return major >= 0;
}

bool isValidMCode(int mcode) {
    return mcode >= 0;
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
