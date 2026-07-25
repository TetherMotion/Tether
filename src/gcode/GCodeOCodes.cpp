/**
 * @file GCodeOCodes.cpp
 * @brief O-Code Control Flow Implementation
 *
 * Implements the OCodeExecutor, SubroutineRegistry, and O-code utility
 * functions declared in GCodeOCodes.hpp.
 */

#include "tether/gcode/GCodeOCodes.hpp"
#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeVariables.hpp"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace GCode {

// ============================================================================
// Internal helpers
// ============================================================================

namespace {

void set_error(Error& err, ErrorCode code, uint32_t line, const char* msg) {
    err.code = code;
    err.line = line;
    err.message.fill(0);
    err.context.fill(0);
    if (msg) {
        std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    }
}

std::string to_lower(const std::string& s) {
    std::string r;
    r.reserve(s.size());
    for (char c : s) r.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    return r;
}

} // namespace

// ============================================================================
// SubroutineRegistry
// ============================================================================

Error SubroutineRegistry::registerSubroutine(int32_t oNumber, const SubroutineInfo& info) {
    Error err;
    auto it = m_numbered.find(oNumber);
    if (it != m_numbered.end()) {
        set_error(err, ErrorCode::SUBROUTINE_ERROR, 0,
                  "Duplicate numbered subroutine");
        return err;
    }
    m_numbered[oNumber] = info;
    return Error{};
}

Error SubroutineRegistry::registerSubroutine(const std::string& name, const SubroutineInfo& info) {
    Error err;
    auto it = m_named.find(name);
    if (it != m_named.end()) {
        set_error(err, ErrorCode::SUBROUTINE_ERROR, 0,
                  "Duplicate named subroutine");
        return err;
    }
    m_named[name] = info;
    return Error{};
}

const SubroutineInfo* SubroutineRegistry::find(int32_t oNumber) const {
    auto it = m_numbered.find(oNumber);
    if (it == m_numbered.end()) return nullptr;
    return &it->second;
}

const SubroutineInfo* SubroutineRegistry::find(const std::string& name) const {
    auto it = m_named.find(name);
    if (it == m_named.end()) return nullptr;
    return &it->second;
}

bool SubroutineRegistry::exists(int32_t oNumber) const {
    return m_numbered.count(oNumber) > 0;
}

bool SubroutineRegistry::exists(const std::string& name) const {
    return m_named.count(name) > 0;
}

void SubroutineRegistry::clear() {
    m_numbered.clear();
    m_named.clear();
}

Error SubroutineRegistry::scanSource(Parser& parser) {
    Error err;
    // Save parser state
    const size_t savedPos = parser.getLexer().getPosition();
    const size_t savedBlockNum = parser.getCurrentBlockNumber();

    parser.getLexer().seekToStart();
    parser.setCurrentBlockNumber(0);

    Block b;
    while (true) {
        Error e = parser.parseNextBlock(b);
        if (e) {
            if (e.code == ErrorCode::END) break;
            continue;  // skip parse errors
        }
        if (b.hasOCode && b.oCodeType == OCodeType::SUB) {
            SubroutineInfo info;
            info.oNumber = b.oCodeNumber;
            info.isNamed = b.oCodeIsNamed;
            if (b.oCodeIsNamed) {
                info.name = b.oCodeName.data();
            }
            info.startAddress = parser.getLexer().getPosition();
            info.startLine = b.sourceLineNumber;

            // Find matching endsub
            Block endB;
            Error endErr = parser.findMatchingOCode(
                b.oCodeIsNamed ? b.oCodeNumber : b.oCodeNumber,
                OCodeType::SUB, endB);
            if (!endErr) {
                info.endAddress = parser.getLexer().getPosition();
                info.endLine = endB.sourceLineNumber;
            }

            if (b.oCodeIsNamed) {
                err = registerSubroutine(info.name, info);
            } else {
                err = registerSubroutine(info.oNumber, info);
            }
            if (err) {
                parser.getLexer().seek(savedPos);
                parser.setCurrentBlockNumber(savedBlockNum);
                return err;
            }
        }
    }

    parser.getLexer().seek(savedPos);
    parser.setCurrentBlockNumber(savedBlockNum);
    return Error{};
}

Error SubroutineRegistry::loadFromFile(const std::string& filename) {
    Error err;
    std::ifstream f(filename);
    if (!f.is_open()) {
        set_error(err, ErrorCode::FILE_NOT_FOUND, 0, "Cannot open subroutine file");
        return err;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string content = ss.str();

    // Parse content for subroutines using a temporary parser.
    VariableSystem tmpVars;
    Parser parser(tmpVars);
    parser.setInput(content);
    return scanSource(parser);
}

// ============================================================================
// OCodeExecutor
// ============================================================================

OCodeExecutor::OCodeExecutor(VariableSystem& vars, Parser& parser,
                             const OCodeConfig& config)
    : m_config(config)
    , m_vars(vars)
    , m_parser(parser)
    , m_evaluator(vars) {
}

Error OCodeExecutor::execute(const Block& block, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (!block.hasOCode) {
        return Error{};
    }

    switch (block.oCodeType) {
        case OCodeType::SUB:
            // Subroutine definition — skip over the body when encountered
            // directly (the body is executed via callSubroutine).
            {
                Block endB;
                Error e = m_parser.findMatchingOCode(
                    block.oCodeNumber, OCodeType::SUB, endB);
                if (e) {
                    set_error(err, ErrorCode::INVALID_OCODE, block.sourceLineNumber,
                              "sub without matching endsub");
                    return e;
                }
                m_jumpAddress = m_parser.getLexer().getPosition();
                m_jumpLine = endB.sourceLineNumber;
                nextAction = NextAction::JUMP;
            }
            return Error{};

        case OCodeType::ENDSUB:
            return returnFromSub(std::nullopt);

        case OCodeType::CALL: {
            std::vector<double> args;
            // Parse call arguments from the condition expression
            // The condition may contain multiple bracketed expressions
            // For simplicity, parse the first expression as arg1, etc.
            // (Full multi-arg parsing would require tokenizing the line.)
            if (block.oCodeCondition[0] != '\0') {
                double val = 0.0;
                Error e = m_evaluator.evaluate(block.oCodeCondition.data(), val);
                if (!e) {
                    args.push_back(val);
                }
            }
            if (block.oCodeIsNamed) {
                return callSubroutine(std::string(block.oCodeName.data()), args);
            }
            return callSubroutine(block.oCodeNumber, args);
        }

        case OCodeType::RETURN: {
            std::optional<double> retVal;
            if (block.oCodeCondition[0] != '\0') {
                double val = 0.0;
                Error e = m_evaluator.evaluate(block.oCodeCondition.data(), val);
                if (!e) {
                    retVal = val;
                }
            }
            return returnFromSub(retVal);
        }

        case OCodeType::IF:
            return beginIf(block.oCodeNumber,
                           std::string(block.oCodeCondition.data()),
                           nextAction);

        case OCodeType::ELSEIF:
            return handleElseIf(block.oCodeNumber,
                                std::string(block.oCodeCondition.data()),
                                nextAction);

        case OCodeType::ELSE:
            return handleElse(block.oCodeNumber, nextAction);

        case OCodeType::ENDIF:
            return endIf(block.oCodeNumber);

        case OCodeType::WHILE:
            return beginWhile(block.oCodeNumber,
                              std::string(block.oCodeCondition.data()));

        case OCodeType::ENDWHILE:
            return endWhile(block.oCodeNumber, nextAction);

        case OCodeType::DO:
            return beginDo(block.oCodeNumber);

        case OCodeType::REPEAT: {
            int32_t count = 0;
            if (block.oCodeCondition[0] != '\0') {
                double val = 0.0;
                Error e = m_evaluator.evaluate(block.oCodeCondition.data(), val);
                if (e) {
                    set_error(err, ErrorCode::INVALID_OCODE, block.sourceLineNumber,
                              "Invalid repeat count");
                    return e;
                }
                count = static_cast<int32_t>(std::llround(val));
            }
            return beginRepeat(block.oCodeNumber, count);
        }

        case OCodeType::ENDREPEAT:
            return endRepeat(block.oCodeNumber, nextAction);

        case OCodeType::BREAK:
            return breakLoop(block.oCodeNumber, nextAction);

        case OCodeType::CONTINUE:
            return continueLoop(block.oCodeNumber, nextAction);

        default:
            set_error(err, ErrorCode::INVALID_OCODE, block.sourceLineNumber,
                      "Unknown O-code type");
            return err;
    }
}

std::optional<double> OCodeExecutor::getReturnValue() const {
    return m_returnValue;
}

// ============================================================================
// Subroutine Handling
// ============================================================================

Error OCodeExecutor::callSubroutine(int32_t oNumber,
                                    const std::vector<double>& args) {
    Error err;

    if (m_callStack.size() >= m_config.maxCallDepth) {
        set_error(err, ErrorCode::NESTED_TOO_DEEP, 0,
                  "Maximum call depth exceeded");
        return err;
    }

    // Find the subroutine
    const SubroutineInfo* info = m_registry.find(oNumber);
    if (!info) {
        // Try to find in source
        Block subBlock;
        Error e = m_parser.findSubroutine(oNumber, subBlock);
        if (e) {
            set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, 0,
                      "Undefined subroutine");
            return err;
        }
        // Register it
        SubroutineInfo newInfo;
        newInfo.oNumber = oNumber;
        newInfo.isNamed = false;
        newInfo.startAddress = m_parser.getLexer().getPosition();
        newInfo.startLine = subBlock.sourceLineNumber;
        m_registry.registerSubroutine(oNumber, newInfo);
        info = m_registry.find(oNumber);
    }

    // Push call frame
    CallFrame frame;
    frame.oNumber = oNumber;
    frame.isNamed = false;
    frame.returnAddress = m_parser.getLexer().getPosition();
    frame.returnLine = m_parser.getCurrentBlockNumber();
    m_callStack.push_back(std::move(frame));

    // Push variable frame with arguments
    m_vars.pushFrame(args);

    // Jump to subroutine start
    m_jumpAddress = info->startAddress;
    m_jumpLine = info->startLine;
    // The caller should use NextAction::JUMP — but execute() already returned.
    // For direct calls, we set the jump address and the caller checks it.
    return Error{};
}

Error OCodeExecutor::callSubroutine(const std::string& name,
                                    const std::vector<double>& args) {
    Error err;

    if (m_callStack.size() >= m_config.maxCallDepth) {
        set_error(err, ErrorCode::NESTED_TOO_DEEP, 0,
                  "Maximum call depth exceeded");
        return err;
    }

    std::string key = m_config.caseInsensitive ? to_lower(name) : name;
    const SubroutineInfo* info = m_registry.find(key);
    if (!info) {
        Block subBlock;
        Error e = m_parser.findSubroutine(name, subBlock);
        if (e) {
            set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, 0,
                      "Undefined named subroutine");
            return err;
        }
        SubroutineInfo newInfo;
        newInfo.oNumber = subBlock.oCodeNumber;
        newInfo.isNamed = true;
        newInfo.name = name;
        newInfo.startAddress = m_parser.getLexer().getPosition();
        newInfo.startLine = subBlock.sourceLineNumber;
        m_registry.registerSubroutine(key, newInfo);
        info = m_registry.find(key);
    }

    CallFrame frame;
    frame.oNumber = info->oNumber;
    frame.oName = name;
    frame.isNamed = true;
    frame.returnAddress = m_parser.getLexer().getPosition();
    frame.returnLine = m_parser.getCurrentBlockNumber();
    m_callStack.push_back(std::move(frame));

    m_vars.pushFrame(args);

    m_jumpAddress = info->startAddress;
    m_jumpLine = info->startLine;
    return Error{};
}

Error OCodeExecutor::returnFromSub(std::optional<double> returnValue) {
    Error err;

    if (m_callStack.empty()) {
        set_error(err, ErrorCode::RETURN_WITHOUT_CALL, 0,
                  "return/endsub without subroutine call");
        return err;
    }

    // Set return value
    m_returnValue = returnValue;
    if (returnValue) {
        m_vars.setNamed("_value", *returnValue);
        m_vars.setNamed("_value_returned", 1.0);
    } else {
        m_vars.setNamed("_value_returned", 0.0);
    }

    // Pop call frame
    CallFrame frame = std::move(m_callStack.back());
    m_callStack.pop_back();

    // Pop variable frame
    m_vars.popFrame();

    // Jump back to return address
    m_jumpAddress = frame.returnAddress;
    m_jumpLine = frame.returnLine;

    return Error{};
}

// ============================================================================
// Loop Handling
// ============================================================================

Error OCodeExecutor::beginWhile(int32_t oNumber, const std::string& condition) {
    Error err;
    LoopFrame frame;
    frame.oNumber = oNumber;
    frame.type = LoopFrame::Type::WHILE;
    frame.startAddress = m_parser.getLexer().getPosition();
    frame.startLine = m_parser.getCurrentBlockNumber();
    frame.condition = condition;
    m_loopStack.push_back(std::move(frame));
    return Error{};
}

Error OCodeExecutor::endWhile(int32_t oNumber, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_loopStack.empty() || m_loopStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "endwhile without matching while");
        return err;
    }

    LoopFrame& frame = m_loopStack.back();
    if (frame.type != LoopFrame::Type::WHILE) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "endwhile does not match while");
        return err;
    }

    // Evaluate condition
    bool result = false;
    Error e = evaluateCondition(frame.condition, result);
    if (e) return e;

    if (result) {
        // Jump back to start of loop body
        m_jumpAddress = frame.startAddress;
        m_jumpLine = frame.startLine;
        nextAction = NextAction::JUMP;
    } else {
        // Exit loop
        m_loopStack.pop_back();
    }

    return Error{};
}

Error OCodeExecutor::beginDo(int32_t oNumber) {
    Error err;
    LoopFrame frame;
    frame.oNumber = oNumber;
    frame.type = LoopFrame::Type::DO_WHILE;
    frame.startAddress = m_parser.getLexer().getPosition();
    frame.startLine = m_parser.getCurrentBlockNumber();
    m_loopStack.push_back(std::move(frame));
    return Error{};
}

Error OCodeExecutor::doWhile(int32_t oNumber, const std::string& condition,
                             NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_loopStack.empty() || m_loopStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "while (do-while) without matching do");
        return err;
    }

    LoopFrame& frame = m_loopStack.back();
    if (frame.type != LoopFrame::Type::DO_WHILE) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "do-while mismatch");
        return err;
    }

    bool result = false;
    Error e = evaluateCondition(condition, result);
    if (e) return e;

    if (result) {
        m_jumpAddress = frame.startAddress;
        m_jumpLine = frame.startLine;
        nextAction = NextAction::JUMP;
    } else {
        m_loopStack.pop_back();
    }

    return Error{};
}

Error OCodeExecutor::beginRepeat(int32_t oNumber, int32_t count) {
    Error err;
    if (count < 0) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "Negative repeat count");
        return err;
    }
    LoopFrame frame;
    frame.oNumber = oNumber;
    frame.type = LoopFrame::Type::REPEAT;
    frame.startAddress = m_parser.getLexer().getPosition();
    frame.startLine = m_parser.getCurrentBlockNumber();
    frame.currentIteration = 0;
    frame.maxIterations = count;
    m_loopStack.push_back(std::move(frame));
    return Error{};
}

Error OCodeExecutor::endRepeat(int32_t oNumber, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_loopStack.empty() || m_loopStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "endrepeat without matching repeat");
        return err;
    }

    LoopFrame& frame = m_loopStack.back();
    if (frame.type != LoopFrame::Type::REPEAT) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "endrepeat does not match repeat");
        return err;
    }

    ++frame.currentIteration;
    if (frame.currentIteration < frame.maxIterations) {
        m_jumpAddress = frame.startAddress;
        m_jumpLine = frame.startLine;
        nextAction = NextAction::JUMP;
    } else {
        m_loopStack.pop_back();
    }

    return Error{};
}

Error OCodeExecutor::breakLoop(int32_t oNumber, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_loopStack.empty()) {
        set_error(err, ErrorCode::BREAK_OUTSIDE_LOOP, 0,
                  "break outside loop");
        return err;
    }

    // Find the loop with matching oNumber
    // (break can target an outer loop by number)
    while (!m_loopStack.empty()) {
        LoopFrame& frame = m_loopStack.back();
        if (frame.oNumber == oNumber) {
            // Find matching end and jump past it
            m_loopStack.pop_back();
            break;
        }
        m_loopStack.pop_back();
    }

    // The caller should skip to the matching end keyword.
    // For simplicity, we set JUMP and the caller finds the end.
    nextAction = NextAction::JUMP;
    return Error{};
}

Error OCodeExecutor::continueLoop(int32_t oNumber, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_loopStack.empty()) {
        set_error(err, ErrorCode::CONTINUE_OUTSIDE_LOOP, 0,
                  "continue outside loop");
        return err;
    }

    // Find the loop with matching oNumber
    LoopFrame* target = nullptr;
    for (auto it = m_loopStack.rbegin(); it != m_loopStack.rend(); ++it) {
        if (it->oNumber == oNumber) {
            target = &(*it);
            break;
        }
    }

    if (!target) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "continue target loop not found");
        return err;
    }

    if (target->type == LoopFrame::Type::REPEAT) {
        ++target->currentIteration;
        if (target->currentIteration >= target->maxIterations) {
            // Exit loop
            while (!m_loopStack.empty() &&
                   m_loopStack.back().oNumber != oNumber) {
                m_loopStack.pop_back();
            }
            if (!m_loopStack.empty()) m_loopStack.pop_back();
            return Error{};
        }
    }

    m_jumpAddress = target->startAddress;
    m_jumpLine = target->startLine;
    nextAction = NextAction::JUMP;
    return Error{};
}

int32_t OCodeExecutor::getRepeatCount() const {
    if (m_loopStack.empty()) return 0;
    return m_loopStack.back().currentIteration;
}

// ============================================================================
// Conditional Handling
// ============================================================================

Error OCodeExecutor::beginIf(int32_t oNumber, const std::string& condition,
                             NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    bool result = false;
    Error e = evaluateCondition(condition, result);
    if (e) return e;

    ConditionalFrame frame;
    frame.oNumber = oNumber;
    frame.branchTaken = result;
    frame.inElse = false;

    if (!result) {
        // Skip to elseif/else/endif
        nextAction = NextAction::SKIP_TO_ELSE;
    }

    m_condStack.push_back(frame);
    return Error{};
}

Error OCodeExecutor::handleElseIf(int32_t oNumber, const std::string& condition,
                                  NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_condStack.empty() || m_condStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "elseif without matching if");
        return err;
    }

    ConditionalFrame& frame = m_condStack.back();

    if (frame.branchTaken) {
        // A previous branch was taken — skip to endif
        nextAction = NextAction::SKIP_TO_ENDIF;
    } else {
        bool result = false;
        Error e = evaluateCondition(condition, result);
        if (e) return e;

        if (result) {
            frame.branchTaken = true;
            // Execute this branch
        } else {
            // Skip to next elseif/else/endif
            nextAction = NextAction::SKIP_TO_ELSE;
        }
    }

    return Error{};
}

Error OCodeExecutor::handleElse(int32_t oNumber, NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_condStack.empty() || m_condStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "else without matching if");
        return err;
    }

    ConditionalFrame& frame = m_condStack.back();
    frame.inElse = true;

    if (frame.branchTaken) {
        // A previous branch was taken — skip to endif
        nextAction = NextAction::SKIP_TO_ENDIF;
    }
    // else: execute the else branch

    return Error{};
}

Error OCodeExecutor::endIf(int32_t oNumber) {
    Error err;

    if (m_condStack.empty() || m_condStack.back().oNumber != oNumber) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "endif without matching if");
        return err;
    }

    m_condStack.pop_back();
    return Error{};
}

// ============================================================================
// Fanuc Style (M98/M99)
// ============================================================================

Error OCodeExecutor::executeM98(int32_t pWord, int32_t lWord) {
    Error err;
    if (lWord < 1) lWord = 1;
    m_m98RepeatRemaining = lWord - 1;  // First call happens now
    m_m98ReturnAddress = m_parser.getLexer().getPosition();

    // Find subroutine P
    const SubroutineInfo* info = m_registry.find(pWord);
    if (!info) {
        set_error(err, ErrorCode::UNDEFINED_SUBROUTINE, 0,
                  "M98: undefined subprogram");
        return err;
    }

    m_jumpAddress = info->startAddress;
    m_jumpLine = info->startLine;
    return Error{};
}

Error OCodeExecutor::executeM99(NextAction& nextAction) {
    Error err;
    nextAction = NextAction::CONTINUE;

    if (m_m98RepeatRemaining > 0) {
        // Repeat the subprogram
        --m_m98RepeatRemaining;
        // Jump back to sub start (need to track it)
        nextAction = NextAction::JUMP;
    } else {
        // Return to caller
        m_jumpAddress = m_m98ReturnAddress;
        nextAction = NextAction::JUMP;
    }

    return Error{};
}

// ============================================================================
// Modal State Save/Restore (M70-M73)
// ============================================================================

Error OCodeExecutor::saveModalState(const MachineState& state) {
    m_savedModal = state;
    return Error{};
}

Error OCodeExecutor::restoreModalState(MachineState& state) {
    if (m_savedModal) {
        state = *m_savedModal;
        m_savedModal.reset();
    }
    return Error{};
}

Error OCodeExecutor::invalidateModalState() {
    m_savedModal.reset();
    return Error{};
}

Error OCodeExecutor::autoRestoreModalState(const MachineState& state) {
    m_savedModal = state;
    // Auto-restore on return is handled in returnFromSub
    if (!m_callStack.empty()) {
        m_callStack.back().hasModalSave = true;
        m_callStack.back().savedModal = state;
    }
    return Error{};
}

// ============================================================================
// Registry Access
// ============================================================================

Error OCodeExecutor::scanSubroutines() {
    return m_registry.scanSource(m_parser);
}

void OCodeExecutor::addSearchPath(const std::string& path) {
    m_config.searchPaths.push_back(path);
}

// ============================================================================
// State
// ============================================================================

void OCodeExecutor::reset() {
    m_callStack.clear();
    m_loopStack.clear();
    m_condStack.clear();
    m_savedModal.reset();
    m_jumpAddress = 0;
    m_jumpLine = 0;
    m_returnValue.reset();
    m_m98RepeatRemaining = 0;
    m_m98ReturnAddress = 0;
    m_registry.clear();
}

// ============================================================================
// Private helpers
// ============================================================================

Error OCodeExecutor::evaluateCondition(const std::string& condition, bool& result) {
    Error err;
    if (condition.empty()) {
        result = false;
        return Error{};
    }
    double val = 0.0;
    // Wrap in brackets if not already
    std::string expr = condition;
    if (expr.front() != '[') {
        expr = "[" + expr + "]";
    }
    Error e = m_evaluator.evaluate(expr.c_str(), val);
    if (e) return e;
    result = (val != 0.0);
    return Error{};
}

Error OCodeExecutor::findEndOfBlock(int32_t oNumber, OCodeType blockType,
                                    size_t& address, uint32_t& line) {
    Error err;
    Block endB;
    Error e = m_parser.findMatchingOCode(oNumber, blockType, endB);
    if (e) {
        set_error(err, ErrorCode::INVALID_OCODE, 0,
                  "Cannot find end of O-code block");
        return e;
    }
    address = m_parser.getLexer().getPosition();
    line = endB.sourceLineNumber;
    return Error{};
}

Error OCodeExecutor::loadExternalSubroutine(const std::string& name) {
    Error err;
    std::string filename = findSubroutineFile(name);
    if (filename.empty()) {
        set_error(err, ErrorCode::FILE_NOT_FOUND, 0,
                  "External subroutine file not found");
        return err;
    }
    return m_registry.loadFromFile(filename);
}

std::string OCodeExecutor::findSubroutineFile(const std::string& name) {
    for (const auto& path : m_config.searchPaths) {
        std::string full = path + "/" + name + m_config.subFileExtension;
        std::ifstream f(full);
        if (f.is_open()) {
            return full;
        }
    }
    return std::string{};
}

void OCodeExecutor::setError(ErrorCode code, const char* msg) {
    set_error(m_error, code, 0, msg);
}

// ============================================================================
// O-Code Utilities
// ============================================================================

const char* oCodeTypeToKeyword(OCodeType type) {
    switch (type) {
        case OCodeType::SUB:        return "sub";
        case OCodeType::ENDSUB:     return "endsub";
        case OCodeType::CALL:       return "call";
        case OCodeType::RETURN:     return "return";
        case OCodeType::IF:         return "if";
        case OCodeType::ELSEIF:     return "elseif";
        case OCodeType::ELSE:       return "else";
        case OCodeType::ENDIF:      return "endif";
        case OCodeType::WHILE:      return "while";
        case OCodeType::ENDWHILE:   return "endwhile";
        case OCodeType::DO:         return "do";
        case OCodeType::REPEAT:     return "repeat";
        case OCodeType::ENDREPEAT:  return "endrepeat";
        case OCodeType::BREAK:      return "break";
        case OCodeType::CONTINUE:   return "continue";
    }
    return "unknown";
}

OCodeType keywordToOCodeType(const std::string& keyword) {
    std::string u;
    u.reserve(keyword.size());
    for (char c : keyword) u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

    if (u == "SUB") return OCodeType::SUB;
    if (u == "ENDSUB") return OCodeType::ENDSUB;
    if (u == "CALL") return OCodeType::CALL;
    if (u == "RETURN") return OCodeType::RETURN;
    if (u == "IF") return OCodeType::IF;
    if (u == "ELSEIF") return OCodeType::ELSEIF;
    if (u == "ELSE") return OCodeType::ELSE;
    if (u == "ENDIF") return OCodeType::ENDIF;
    if (u == "WHILE") return OCodeType::WHILE;
    if (u == "ENDWHILE") return OCodeType::ENDWHILE;
    if (u == "DO") return OCodeType::DO;
    if (u == "REPEAT") return OCodeType::REPEAT;
    if (u == "ENDREPEAT") return OCodeType::ENDREPEAT;
    if (u == "BREAK") return OCodeType::BREAK;
    if (u == "CONTINUE") return OCodeType::CONTINUE;
    return OCodeType::SUB;
}

bool isBlockOpener(OCodeType type) {
    switch (type) {
        case OCodeType::SUB:
        case OCodeType::IF:
        case OCodeType::WHILE:
        case OCodeType::DO:
        case OCodeType::REPEAT:
            return true;
        default:
            return false;
    }
}

bool isBlockCloser(OCodeType type) {
    switch (type) {
        case OCodeType::ENDSUB:
        case OCodeType::ENDIF:
        case OCodeType::ENDWHILE:
        case OCodeType::ENDREPEAT:
            return true;
        default:
            return false;
    }
}

OCodeType getMatchingCloser(OCodeType opener) {
    switch (opener) {
        case OCodeType::IF:       return OCodeType::ENDIF;
        case OCodeType::WHILE:    return OCodeType::ENDWHILE;
        case OCodeType::DO:       return OCodeType::WHILE;  // do-while
        case OCodeType::REPEAT:   return OCodeType::ENDREPEAT;
        case OCodeType::SUB:      return OCodeType::ENDSUB;
        default:                  return OCodeType::ENDSUB;
    }
}

std::string formatOCode(int32_t oNumber, OCodeType type) {
    std::string s = "O";
    s += std::to_string(oNumber);
    s += " ";
    s += oCodeTypeToKeyword(type);
    return s;
}

std::string formatOCode(const std::string& name, OCodeType type) {
    std::string s = "O<";
    s += name;
    s += "> ";
    s += oCodeTypeToKeyword(type);
    return s;
}

} // namespace GCode
