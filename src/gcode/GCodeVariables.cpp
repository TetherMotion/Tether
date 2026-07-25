#include "tether/gcode/GCodeVariables.hpp"
#include "tether/gcode/motion/GCodeProbing.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

inline char to_upper_ascii(char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

inline bool is_space(char c) {
    return std::isspace(static_cast<unsigned char>(c)) != 0;
}

static void set_error(GCode::Error& err, GCode::ErrorCode code, uint32_t line, const char* msg) {
    err.code = code;
    err.line = line;
    err.message.fill(0);
    err.context.fill(0);
    if (msg) {
        std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    }
}

} // namespace

namespace GCode {

// ============================================================================
// VariableSystem
// ============================================================================

VariableSystem::VariableSystem() {
    m_globalParams.fill(0.0);
    m_globalDefined.fill(false);
    m_systemParams.fill(0.0);
    m_callStack.clear();
    m_callStack.emplace_back();
    m_callStack.back().clear();

    m_globalNamed.clear();
    m_state = nullptr;
    m_valueReturned = false;
    m_returnValue = 0.0;
}

LocalFrame& VariableSystem::currentFrame() {
    if (m_callStack.empty()) {
        m_callStack.emplace_back();
        m_callStack.back().clear();
    }
    return m_callStack.back();
}

const LocalFrame& VariableSystem::currentFrame() const {
    if (m_callStack.empty()) {
        // Use a class-static rather than a function-local static so the
        // sentinel is initialized at namespace load time (no first-call
        // synchronization cost). It is never mutated; only read as a
        // default when no call frame is active.
        static const LocalFrame s_empty{};
        return s_empty;
    }
    return m_callStack.back();
}

double VariableSystem::get(int32_t number) const {
    if (number >= PARAM_LOCAL_START && number <= PARAM_LOCAL_END) {
        const size_t idx = static_cast<size_t>(number - PARAM_LOCAL_START);
        const auto& frame = currentFrame();
        return frame.defined[idx] ? frame.params[idx] : 0.0;
    }

    if (number >= PARAM_GLOBAL_START && number <= PARAM_GLOBAL_END) {
        const size_t idx = static_cast<size_t>(number - PARAM_GLOBAL_START);
        return m_globalDefined[idx] ? m_globalParams[idx] : 0.0;
    }

    if (number >= 5001 && number <= 5600) {
        return getSystemParam(number);
    }

    return 0.0;
}

Error VariableSystem::set(int32_t number, double value) {
    Error err;

    if (isReadOnly(number)) {
        set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Parameter is read-only");
        return err;
    }

    if (number >= PARAM_LOCAL_START && number <= PARAM_LOCAL_END) {
        auto& frame = currentFrame();
        const size_t idx = static_cast<size_t>(number - PARAM_LOCAL_START);
        frame.params[idx] = value;
        frame.defined[idx] = true;
        return err;
    }

    if (number >= PARAM_GLOBAL_START && number <= PARAM_GLOBAL_END) {
        const size_t idx = static_cast<size_t>(number - PARAM_GLOBAL_START);
        m_globalParams[idx] = value;
        m_globalDefined[idx] = true;
        return err;
    }

    set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Parameter number out of supported range");
    return err;
}

bool VariableSystem::isDefined(int32_t number) const {
    if (number >= PARAM_LOCAL_START && number <= PARAM_LOCAL_END) {
        const size_t idx = static_cast<size_t>(number - PARAM_LOCAL_START);
        return currentFrame().defined[idx];
    }

    if (number >= PARAM_GLOBAL_START && number <= PARAM_GLOBAL_END) {
        const size_t idx = static_cast<size_t>(number - PARAM_GLOBAL_START);
        return m_globalDefined[idx];
    }

    if (number >= 5001 && number <= 5600) {
        return true;
    }

    return false;
}

bool VariableSystem::isReadOnly(int32_t number) const {
    return (number >= 5001 && number <= 5600);
}

std::optional<double> VariableSystem::getNamed(const std::string& name) const {
    if (name.empty()) return std::nullopt;

    // Predefined named params (backed by machine state)
    if (m_state) {
        if (name == PredefinedParams::FEED) return m_state->feedRate;
        if (name == PredefinedParams::RPM) return m_state->spindleSpeed;
        if (name == PredefinedParams::CURRENT_TOOL) return static_cast<double>(m_state->currentTool);
        if (name == PredefinedParams::X) return m_state->workPosition.coords[0];
        if (name == PredefinedParams::Y) return m_state->workPosition.coords[1];
        if (name == PredefinedParams::Z) return m_state->workPosition.coords[2];
        if (name == PredefinedParams::A) return m_state->workPosition.coords[3];
        if (name == PredefinedParams::B) return m_state->workPosition.coords[4];
        if (name == PredefinedParams::C) return m_state->workPosition.coords[5];
        if (name == PredefinedParams::U) return m_state->workPosition.coords[6];
        if (name == PredefinedParams::V) return m_state->workPosition.coords[7];
        if (name == PredefinedParams::W) return m_state->workPosition.coords[8];
    }

    if (isGlobalName(name)) {
        auto it = m_globalNamed.find(name);
        if (it == m_globalNamed.end()) return std::nullopt;
        return it->second;
    }

    const auto& frame = currentFrame();
    auto it = frame.namedParams.find(name);
    if (it == frame.namedParams.end()) return std::nullopt;
    return it->second;
}

Error VariableSystem::setNamed(const std::string& name, double value) {
    Error err;
    if (name.empty()) {
        set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Empty named parameter");
        return err;
    }

    if (isGlobalName(name)) {
        m_globalNamed[name] = value;
    } else {
        currentFrame().namedParams[name] = value;
    }

    return err;
}

bool VariableSystem::existsNamed(const std::string& name) const {
    return getNamed(name).has_value();
}

Error VariableSystem::pushFrame(const std::vector<double>& args) {
    Error err;
    if (m_callStack.size() >= MAX_CALL_DEPTH) {
        set_error(err, ErrorCode::NESTED_TOO_DEEP, 0, "Call nesting too deep");
        return err;
    }

    LocalFrame frame;
    frame.clear();

    const size_t count = std::min(args.size(), static_cast<size_t>(MAX_LOCAL_PARAMS));
    for (size_t i = 0; i < count; ++i) {
        frame.params[i] = args[i];
        frame.defined[i] = true;
    }

    m_callStack.push_back(std::move(frame));
    return err;
}

Error VariableSystem::popFrame() {
    Error err;
    if (m_callStack.size() <= 1) {
        set_error(err, ErrorCode::RETURN_WITHOUT_CALL, 0, "Cannot pop base frame");
        return err;
    }

    m_callStack.pop_back();
    return err;
}

void VariableSystem::clear() {
    m_globalParams.fill(0.0);
    m_globalDefined.fill(false);
    m_systemParams.fill(0.0);
    m_globalNamed.clear();

    m_callStack.clear();
    m_callStack.emplace_back();
    m_callStack.back().clear();

    clearReturnValue();
}

void VariableSystem::updateFromState(const MachineState& state) {
    m_state = &state;
    updateSystemParams();
}

void VariableSystem::setProbeResult(const ProbeResult& result) {
    // #5061-#5069 = trip coordinates, #5070 = success flag
    for (size_t i = 0; i < MAX_AXES; ++i) {
        const int32_t num = PARAM_PROBE_RESULT + static_cast<int32_t>(i);
        if (num >= 5001 && num <= 5600) {
            m_systemParams[static_cast<size_t>(num - 5001)] = result.tripPosition.coords[i];
        }
    }
    m_systemParams[static_cast<size_t>(PARAM_PROBE_SUCCESS - 5001)] = (result.success || result.tripped) ? 1.0 : 0.0;
}

void VariableSystem::setCoordSystemOffset(CoordSystem cs, const Position& offset, double rotation) {
    const int idx = static_cast<int>(cs) - static_cast<int>(CoordSystem::G54);
    if (idx < 0 || idx >= static_cast<int>(MAX_COORD_SYSTEMS)) {
        return;
    }

    // Each coordinate system occupies a 10-slot block starting at #5221:
    //   G54   = #5221-#5230
    //   G55   = #5231-#5240
    //   G56   = #5241-#5250
    //   G57   = #5251-#5260
    //   G58   = #5261-#5270
    //   G59   = #5271-#5280
    //   G59.1 = #5281-#5290
    //   G59.2 = #5291-#5300
    //   G59.3 = #5301-#5310
    // Slots 0-8 hold X,Y,Z,A,B,C,U,V,W; slot 9 holds rotation.
    const size_t base = static_cast<size_t>(PARAM_G54_OFFSET - 5001)
                      + static_cast<size_t>(idx) * 10;
    if (base + 9 >= m_systemParams.size()) return;
    for (size_t i = 0; i < offset.coords.size() && i < 9; ++i) {
        m_systemParams[base + i] = offset.coords[i];
    }
    m_systemParams[base + 9] = rotation;
}

Position VariableSystem::getCoordSystemOffset(CoordSystem cs) const {
    Position out;
    out.coords.fill(0.0);

    const int idx = static_cast<int>(cs) - static_cast<int>(CoordSystem::G54);
    if (idx < 0 || idx >= static_cast<int>(MAX_COORD_SYSTEMS)) {
        return out;
    }

    const size_t base = static_cast<size_t>(PARAM_G54_OFFSET - 5001)
                      + static_cast<size_t>(idx) * 10;
    if (base + 9 >= m_systemParams.size()) return out;
    for (size_t i = 0; i < out.coords.size() && i < 9; ++i) {
        out.coords[i] = m_systemParams[base + i];
    }
    return out;
}

void VariableSystem::setReturnValue(double value) {
    m_returnValue = value;
    m_valueReturned = true;
}

std::optional<double> VariableSystem::getReturnValue() const {
    if (!m_valueReturned) return std::nullopt;
    return m_returnValue;
}

void VariableSystem::clearReturnValue() {
    m_valueReturned = false;
    m_returnValue = 0.0;
}

double VariableSystem::getSystemParam(int32_t number) const {
    if (number < 5001 || number > 5600) return 0.0;
    const size_t idx = static_cast<size_t>(number - 5001);
    if (idx >= m_systemParams.size()) return 0.0;
    return m_systemParams[idx];
}

void VariableSystem::updateSystemParams() {
    if (!m_state) return;

    // Current position (#5420-#5428)
    for (size_t i = 0; i < MAX_AXES; ++i) {
        const int32_t num = PARAM_CURRENT_POS + static_cast<int32_t>(i);
        if (num >= 5001 && num <= 5600) {
            m_systemParams[static_cast<size_t>(num - 5001)] = m_state->workPosition.coords[i];
        }
    }

    // Current tool number (#5400)
    m_systemParams[static_cast<size_t>(PARAM_TOOL_NUMBER - 5001)] =
        static_cast<double>(m_state->currentTool);

    // Current coordinate system (#5220): 1=G54 .. 9=G59.3
    m_systemParams[static_cast<size_t>(PARAM_COORD_SYSTEM - 5001)] =
        static_cast<double>(static_cast<int>(m_state->coordSystem));

    // Tool length offsets (#5401-#5409)
    for (size_t i = 0; i < MAX_AXES && i < 9; ++i) {
        const int32_t num = PARAM_TOOL_OFFSET + static_cast<int32_t>(i);
        if (num >= 5001 && num <= 5600) {
            m_systemParams[static_cast<size_t>(num - 5001)] =
                m_state->toolOffset.coords[i];
        }
    }

    // G28/G30 home positions (#5161-#5169, #5181-#5189) are not tracked in
    // MachineState yet; left at their last set values.

    // G92 offset (#5211-#5219)
    for (size_t i = 0; i < MAX_AXES && i < 9; ++i) {
        const int32_t num = PARAM_G92_OFFSET + static_cast<int32_t>(i);
        if (num >= 5001 && num <= 5600) {
            m_systemParams[static_cast<size_t>(num - 5001)] =
                m_state->g92Offset.coords[i];
        }
    }
}

// ============================================================================
// ExpressionEvaluator
// ============================================================================

ExpressionEvaluator::ExpressionEvaluator(VariableSystem& vars)
    : m_vars(vars) {
    m_error.fill(0);
}

void ExpressionEvaluator::setError(const char* msg) {
    m_error.fill(0);
    if (msg) {
        std::snprintf(m_error.data(), m_error.size(), "%s", msg);
    }
}

void ExpressionEvaluator::skipWhitespace() {
    while (m_pos < m_len && is_space(m_expr[m_pos])) {
        ++m_pos;
    }
}

Token ExpressionEvaluator::parseNumber() {
    Token t;
    t.type = TokenType::NUMBER;

    const size_t start = m_pos;
    bool hasDecimal = false;

    if (m_expr[m_pos] == '+' || m_expr[m_pos] == '-') {
        ++m_pos;
    }

    while (m_pos < m_len) {
        const char c = m_expr[m_pos];
        if (std::isdigit(static_cast<unsigned char>(c))) {
            ++m_pos;
            continue;
        }
        if (c == '.' && !hasDecimal) {
            hasDecimal = true;
            ++m_pos;
            continue;
        }
        break;
    }

    const std::string s(m_expr + start, m_pos - start);
    t.value = std::strtod(s.c_str(), nullptr);
    return t;
}

Token ExpressionEvaluator::parseParameter() {
    Token t;
    t.type = TokenType::PARAMETER;

    // consume '#'
    ++m_pos;
    skipWhitespace();

    if (m_pos < m_len && m_expr[m_pos] == '<') {
        ++m_pos;
        const size_t start = m_pos;
        while (m_pos < m_len && m_expr[m_pos] != '>' && m_expr[m_pos] != ']') {
            ++m_pos;
        }
        if (m_pos >= m_len || m_expr[m_pos] != '>') {
            t.type = TokenType::ERROR;
            setError("Unterminated #<name>");
            return t;
        }
        const size_t end = m_pos;
        ++m_pos; // consume '>'

        t.name.assign(m_expr + start, end - start);
        return t;
    }

    // numbered
    const size_t start = m_pos;
    if (m_pos < m_len && (m_expr[m_pos] == '+' || m_expr[m_pos] == '-')) {
        ++m_pos;
    }
    while (m_pos < m_len && std::isdigit(static_cast<unsigned char>(m_expr[m_pos]))) {
        ++m_pos;
    }

    if (m_pos == start) {
        t.type = TokenType::ERROR;
        setError("Missing parameter number");
        return t;
    }

    t.paramNum = std::atoi(std::string(m_expr + start, m_pos - start).c_str());
    return t;
}

Token ExpressionEvaluator::parseIdentifier() {
    Token t;

    const size_t start = m_pos;
    while (m_pos < m_len && (std::isalpha(static_cast<unsigned char>(m_expr[m_pos])) || m_expr[m_pos] == '_')) {
        ++m_pos;
    }

    std::string id(m_expr + start, m_pos - start);
    for (auto& c : id) c = to_upper_ascii(c);

    // Operators/keywords
    if (id == "MOD") { t.type = TokenType::MOD; return t; }
    if (id == "EQ") { t.type = TokenType::EQ; return t; }
    if (id == "NE") { t.type = TokenType::NE; return t; }
    if (id == "GT") { t.type = TokenType::GT; return t; }
    if (id == "GE") { t.type = TokenType::GE; return t; }
    if (id == "LT") { t.type = TokenType::LT; return t; }
    if (id == "LE") { t.type = TokenType::LE; return t; }
    if (id == "AND") { t.type = TokenType::AND; return t; }
    if (id == "OR") { t.type = TokenType::OR; return t; }
    if (id == "XOR") { t.type = TokenType::XOR; return t; }
    if (id == "NOT") { t.type = TokenType::NOT; return t; }

    // Functions
    if (id == "ABS") { t.type = TokenType::FUNC_ABS; return t; }
    if (id == "ACOS") { t.type = TokenType::FUNC_ACOS; return t; }
    if (id == "ASIN") { t.type = TokenType::FUNC_ASIN; return t; }
    if (id == "ATAN") { t.type = TokenType::FUNC_ATAN; return t; }
    if (id == "COS") { t.type = TokenType::FUNC_COS; return t; }
    if (id == "EXP") { t.type = TokenType::FUNC_EXP; return t; }
    if (id == "FIX") { t.type = TokenType::FUNC_FIX; return t; }
    if (id == "FUP") { t.type = TokenType::FUNC_FUP; return t; }
    if (id == "LN") { t.type = TokenType::FUNC_LN; return t; }
    if (id == "ROUND") { t.type = TokenType::FUNC_ROUND; return t; }
    if (id == "SIN") { t.type = TokenType::FUNC_SIN; return t; }
    if (id == "SQRT") { t.type = TokenType::FUNC_SQRT; return t; }
    if (id == "TAN") { t.type = TokenType::FUNC_TAN; return t; }
    if (id == "EXISTS") { t.type = TokenType::FUNC_EXISTS; return t; }

    t.type = TokenType::ERROR;
    t.name = id;
    setError("Unknown identifier");
    return t;
}

Token ExpressionEvaluator::nextToken() {
    if (!m_pushedBack.empty()) {
        Token t = m_pushedBack.front();
        m_pushedBack.pop_front();
        return t;
    }
    return nextTokenRaw();
}

void ExpressionEvaluator::pushBack(Token t) {
    m_pushedBack.push_front(t);
}

Token ExpressionEvaluator::nextTokenRaw() {
    skipWhitespace();

    Token t;
    t.type = TokenType::END;

    if (!m_expr || m_pos >= m_len) {
        return t;
    }

    const char c = m_expr[m_pos];

    if (c == '[') { ++m_pos; t.type = TokenType::OPEN_BRACKET; return t; }
    if (c == ']') { ++m_pos; t.type = TokenType::CLOSE_BRACKET; return t; }
    if (c == '+') { ++m_pos; t.type = TokenType::PLUS; return t; }
    if (c == '-') { ++m_pos; t.type = TokenType::MINUS; return t; }
    if (c == '/') { ++m_pos; t.type = TokenType::DIVIDE; return t; }
    if (c == '^') { ++m_pos; t.type = TokenType::POWER; return t; }
    if (c == '=') { ++m_pos; t.type = TokenType::ASSIGN; return t; }
    if (c == '?') { ++m_pos; t.type = TokenType::QUESTION; return t; }
    if (c == ':') { ++m_pos; t.type = TokenType::COLON; return t; }
    if (c == '*') {
        if (m_pos + 1 < m_len && m_expr[m_pos + 1] == '*') {
            m_pos += 2;
            t.type = TokenType::POWER;
            return t;
        }
        ++m_pos;
        t.type = TokenType::MULTIPLY;
        return t;
    }

    if (c == '#') {
        return parseParameter();
    }

    if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
        return parseNumber();
    }

    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
        return parseIdentifier();
    }

    t.type = TokenType::ERROR;
    setError("Unexpected character");
    ++m_pos;
    return t;
}

Error ExpressionEvaluator::evaluate(const char* expr, double& result) {
    Error err;
    m_error.fill(0);

    if (!expr) {
        set_error(err, ErrorCode::EXPRESSION_ERROR, 0, "Null expression");
        return err;
    }

    m_expr = expr;
    m_pos = 0;
    m_len = std::strlen(expr);
    m_depth = 0;
    m_pushedBack.clear();

    m_current = nextToken();

    // Parameter assignment: #<name> = expr  or  #N = expr
    // The assignment target is consumed as a PARAMETER token, followed by
    // an ASSIGN token. We evaluate the RHS and store it back to the variable
    // system, returning the assigned value as the result.
    if (m_current.type == TokenType::PARAMETER) {
        Token assignTarget = m_current;
        Token lookahead = nextToken();
        if (lookahead.type == TokenType::ASSIGN) {
            m_current = nextToken();
            double rhs = 0.0;
            err = parseExpression(rhs);
            if (err) return err;
            if (!std::isfinite(rhs)) {
                set_error(err, ErrorCode::EXPRESSION_ERROR, 0,
                          "Assignment of non-finite value (inf/nan)");
                return err;
            }
            if (assignTarget.paramNum >= 0) {
                m_vars.set(assignTarget.paramNum, rhs);
            } else if (!assignTarget.name.empty()) {
                m_vars.setNamed(assignTarget.name, rhs);
            }
            result = rhs;
            return Error{};
        }
        // Not an assignment — push both tokens back and parse normally.
        pushBack(lookahead);
        pushBack(assignTarget);
        m_current = nextToken();
    }

    double value = 0.0;
    err = parseExpression(value);
    if (err) {
        return err;
    }

    // Reject non-finite results (inf/nan) so downstream consumers don't
    // silently propagate them into motion commands.
    if (!std::isfinite(value)) {
        set_error(err, ErrorCode::EXPRESSION_ERROR, 0,
                  "Expression evaluated to non-finite value (inf/nan)");
        return err;
    }

    result = value;
    return Error{};
}

std::pair<double, Error> ExpressionEvaluator::evaluate(const char* expr) {
    double r = 0.0;
    Error e = evaluate(expr, r);
    return {r, e};
}

// --- Recursive descent ------------------------------------------------------

Error ExpressionEvaluator::parseExpression(double& result) {
    // Bound recursion to prevent stack overflow on deeply nested input.
    if (++m_depth > kMaxDepth) {
        --m_depth;
        Error err;
        set_error(err, ErrorCode::EXPRESSION_ERROR, 0,
                  "Expression recursion depth exceeded");
        return err;
    }
    Error err = parseTernary(result);
    --m_depth;
    return err;
}

Error ExpressionEvaluator::parseTernary(double& result) {
    // ternary: logicalOr ('?' expression ':' ternary)?
    // The true branch is a full expression; the false branch is a ternary
    // (right-associative for chained ternaries).
    Error err = parseLogicalOr(result);
    if (err) return err;

    if (m_current.type == TokenType::QUESTION) {
        m_current = nextToken();
        double condVal = result;
        double trueVal = 0.0;
        err = parseExpression(trueVal);
        if (err) return err;
        if (m_current.type != TokenType::COLON) {
            set_error(err, ErrorCode::EXPRESSION_ERROR, 0,
                      "Missing ':' in ternary expression");
            return err;
        }
        m_current = nextToken();
        double falseVal = 0.0;
        err = parseTernary(falseVal); // right-assoc for chained ternaries
        if (err) return err;
        result = (condVal != 0.0) ? trueVal : falseVal;
    }

    return Error{};
}

Error ExpressionEvaluator::parseLogicalOr(double& result) {
    Error err = parseLogicalXor(result);
    if (err) return err;

    while (m_current.type == TokenType::OR) {
        m_current = nextToken();
        double rhs = 0.0;
        err = parseLogicalXor(rhs);
        if (err) return err;
        result = ((result != 0.0) || (rhs != 0.0)) ? 1.0 : 0.0;
    }

    return Error{};
}

Error ExpressionEvaluator::parseLogicalXor(double& result) {
    Error err = parseLogicalAnd(result);
    if (err) return err;

    while (m_current.type == TokenType::XOR) {
        m_current = nextToken();
        double rhs = 0.0;
        err = parseLogicalAnd(rhs);
        if (err) return err;
        const bool a = (result != 0.0);
        const bool b = (rhs != 0.0);
        result = (a ^ b) ? 1.0 : 0.0;
    }

    return Error{};
}

Error ExpressionEvaluator::parseLogicalAnd(double& result) {
    Error err = parseComparison(result);
    if (err) return err;

    while (m_current.type == TokenType::AND) {
        m_current = nextToken();
        double rhs = 0.0;
        err = parseComparison(rhs);
        if (err) return err;
        result = ((result != 0.0) && (rhs != 0.0)) ? 1.0 : 0.0;
    }

    return Error{};
}

Error ExpressionEvaluator::parseComparison(double& result) {
    Error err = parseAddSub(result);
    if (err) return err;

    while (m_current.type == TokenType::EQ || m_current.type == TokenType::NE ||
           m_current.type == TokenType::GT || m_current.type == TokenType::GE ||
           m_current.type == TokenType::LT || m_current.type == TokenType::LE) {
        const TokenType op = m_current.type;
        m_current = nextToken();

        double rhs = 0.0;
        err = parseAddSub(rhs);
        if (err) return err;

        bool ok = false;
        switch (op) {
            case TokenType::EQ: ok = (result == rhs); break;
            case TokenType::NE: ok = (result != rhs); break;
            case TokenType::GT: ok = (result > rhs); break;
            case TokenType::GE: ok = (result >= rhs); break;
            case TokenType::LT: ok = (result < rhs); break;
            case TokenType::LE: ok = (result <= rhs); break;
            default: break;
        }
        result = ok ? 1.0 : 0.0;
    }

    return Error{};
}

Error ExpressionEvaluator::parseAddSub(double& result) {
    Error err = parseMulDiv(result);
    if (err) return err;

    while (m_current.type == TokenType::PLUS || m_current.type == TokenType::MINUS) {
        const TokenType op = m_current.type;
        m_current = nextToken();
        double rhs = 0.0;
        err = parseMulDiv(rhs);
        if (err) return err;
        result = (op == TokenType::PLUS) ? (result + rhs) : (result - rhs);
    }

    return Error{};
}

Error ExpressionEvaluator::parseMulDiv(double& result) {
    Error err = parsePower(result);
    if (err) return err;

    while (m_current.type == TokenType::MULTIPLY || m_current.type == TokenType::DIVIDE ||
           m_current.type == TokenType::MOD) {
        const TokenType op = m_current.type;
        m_current = nextToken();
        double rhs = 0.0;
        err = parsePower(rhs);
        if (err) return err;

        if (op == TokenType::MULTIPLY) {
            result *= rhs;
        } else if (op == TokenType::DIVIDE) {
            result /= rhs;
        } else {
            result = std::fmod(result, rhs);
        }
    }

    return Error{};
}

Error ExpressionEvaluator::parsePower(double& result) {
    // Right-associative: a ** b ** c == a ** (b ** c).
    Error err = parseUnary(result);
    if (err) return err;

    if (m_current.type == TokenType::POWER) {
        m_current = nextToken();
        double rhs = 0.0;
        err = parsePower(rhs); // recurse for right-assoc
        if (err) return err;
        result = std::pow(result, rhs);
    }

    return Error{};
}

Error ExpressionEvaluator::parseUnary(double& result) {
    if (m_current.type == TokenType::PLUS) {
        m_current = nextToken();
        return parseUnary(result);
    }
    if (m_current.type == TokenType::MINUS) {
        m_current = nextToken();
        Error err = parseUnary(result);
        if (err) return err;
        result = -result;
        return Error{};
    }
    if (m_current.type == TokenType::NOT) {
        m_current = nextToken();
        Error err = parseUnary(result);
        if (err) return err;
        result = (result == 0.0) ? 1.0 : 0.0;
        return Error{};
    }

    return parsePrimary(result);
}

Error ExpressionEvaluator::parsePrimary(double& result) {
    Error err;

    if (m_current.type == TokenType::NUMBER) {
        result = m_current.value;
        m_current = nextToken();
        return err;
    }

    if (m_current.type == TokenType::PARAMETER) {
        if (m_current.paramNum >= 0) {
            result = m_vars.get(m_current.paramNum);
        } else if (!m_current.name.empty()) {
            auto v = m_vars.getNamed(m_current.name);
            result = v.value_or(0.0);
        } else {
            result = 0.0;
        }
        m_current = nextToken();
        return err;
    }

    if (m_current.type == TokenType::OPEN_BRACKET) {
        m_current = nextToken();
        err = parseExpression(result);
        if (err) return err;
        if (m_current.type != TokenType::CLOSE_BRACKET) {
            set_error(err, ErrorCode::MISSING_BRACKET, 0, "Missing closing ]");
            return err;
        }
        m_current = nextToken();
        return Error{};
    }

    // Functions
    if (m_current.type >= TokenType::FUNC_ABS && m_current.type <= TokenType::FUNC_EXISTS) {
        const TokenType func = m_current.type;
        m_current = nextToken();
        return parseFunction(func, result);
    }

    set_error(err, ErrorCode::EXPRESSION_ERROR, 0, "Unexpected token in expression");
    return err;
}

Error ExpressionEvaluator::parseFunction(TokenType func, double& result) {
    Error err;

    if (m_current.type != TokenType::OPEN_BRACKET) {
        set_error(err, ErrorCode::EXPRESSION_ERROR, 0, "Function requires [arg] syntax");
        return err;
    }

    m_current = nextToken();

    if (func == TokenType::FUNC_EXISTS) {
        if (m_current.type != TokenType::PARAMETER) {
            set_error(err, ErrorCode::EXPRESSION_ERROR, 0, "EXISTS expects a parameter reference");
            return err;
        }

        bool exists = false;
        if (m_current.paramNum >= 0) {
            exists = m_vars.isDefined(m_current.paramNum);
        } else if (!m_current.name.empty()) {
            exists = m_vars.existsNamed(m_current.name);
        }

        m_current = nextToken();

        if (m_current.type != TokenType::CLOSE_BRACKET) {
            set_error(err, ErrorCode::MISSING_BRACKET, 0, "Missing closing ] after EXISTS arg");
            return err;
        }
        m_current = nextToken();

        result = exists ? 1.0 : 0.0;
        return Error{};
    }

    double a = 0.0;
    err = parseExpression(a);
    if (err) return err;

    double b = 0.0;
    bool hasSecond = false;

    if (func == TokenType::FUNC_ATAN && m_current.type == TokenType::DIVIDE) {
        m_current = nextToken();
        err = parseExpression(b);
        if (err) return err;
        hasSecond = true;
    }

    if (m_current.type != TokenType::CLOSE_BRACKET) {
        set_error(err, ErrorCode::MISSING_BRACKET, 0, "Missing closing ] after function args");
        return err;
    }

    m_current = nextToken();

    switch (func) {
        case TokenType::FUNC_ABS: result = std::fabs(a); break;
        case TokenType::FUNC_ACOS: result = radToDeg(std::acos(a)); break;
        case TokenType::FUNC_ASIN: result = radToDeg(std::asin(a)); break;
        case TokenType::FUNC_ATAN:
            result = hasSecond ? radToDeg(std::atan2(a, b)) : radToDeg(std::atan(a));
            break;
        case TokenType::FUNC_COS: result = std::cos(degToRad(a)); break;
        case TokenType::FUNC_EXP: result = std::exp(a); break;
        case TokenType::FUNC_FIX: result = (a >= 0.0) ? std::floor(a) : std::ceil(a); break;
        case TokenType::FUNC_FUP: result = (a >= 0.0) ? std::ceil(a) : std::floor(a); break;
        case TokenType::FUNC_LN: result = std::log(a); break;
        case TokenType::FUNC_ROUND: result = std::floor(a + 0.5); break;
        case TokenType::FUNC_SIN: result = std::sin(degToRad(a)); break;
        case TokenType::FUNC_SQRT: result = std::sqrt(a); break;
        case TokenType::FUNC_TAN: result = std::tan(degToRad(a)); break;
        default:
            set_error(err, ErrorCode::EXPRESSION_ERROR, 0, "Unknown function");
            return err;
    }

    return Error{};
}

// ============================================================================
// Parameter substitution helpers
// ============================================================================

Error parseParameterRef(const char* input, VariableSystem& vars, double& value, size_t& consumed) {
    Error err;
    consumed = 0;

    if (!input || input[0] != '#') {
        set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Expected #");
        return err;
    }

    consumed = 1;

    if (input[consumed] == '<') {
        ++consumed;
        const size_t start = consumed;
        while (input[consumed] && input[consumed] != '>') {
            ++consumed;
        }
        if (input[consumed] != '>') {
            set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Unterminated #<name>");
            return err;
        }
        const std::string name(input + start, consumed - start);
        ++consumed;
        value = vars.getNamed(name).value_or(0.0);
        return Error{};
    }

    const size_t start = consumed;
    while (input[consumed] && std::isdigit(static_cast<unsigned char>(input[consumed]))) {
        ++consumed;
    }
    if (consumed == start) {
        set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Missing parameter number");
        return err;
    }

    const int32_t num = std::atoi(std::string(input + start, consumed - start).c_str());
    value = vars.get(num);
    return Error{};
}

Error substituteParameters(const char* input, VariableSystem& vars, char* output, size_t outputSize) {
    Error err;

    if (!input || !output || outputSize == 0) {
        set_error(err, ErrorCode::PARAMETER_ERROR, 0, "Invalid buffers");
        return err;
    }

    size_t outPos = 0;
    for (size_t i = 0; input[i] != 0; ) {
        if (input[i] == '#') {
            double v = 0.0;
            size_t consumed = 0;
            Error perr = parseParameterRef(input + i, vars, v, consumed);
            if (perr) {
                return perr;
            }

            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.10g", v);
            const size_t n = std::strlen(buf);
            if (outPos + n >= outputSize) {
                set_error(err, ErrorCode::MEMORY_ERROR, 0, "Output buffer too small");
                return err;
            }
            std::memcpy(output + outPos, buf, n);
            outPos += n;
            i += consumed;
            continue;
        }

        if (outPos + 1 >= outputSize) {
            set_error(err, ErrorCode::MEMORY_ERROR, 0, "Output buffer too small");
            return err;
        }
        output[outPos++] = input[i++];
    }

    output[outPos] = 0;
    return Error{};
}

} // namespace GCode
