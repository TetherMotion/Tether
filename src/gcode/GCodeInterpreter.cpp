/**
 * @file GCodeInterpreter.cpp
 * @brief Implementation of the main RS274/NGC G-code interpreter.
 *
 * @details
 * This file implements the core execution pipeline:
 *
 *   1. Parse line → Block
 *   2. Update modal state (plane, units, distance mode, WCS, etc.)
 *   3. Dispatch G-codes (motion, coordinate systems, non-modal actions)
 *   4. Dispatch M-codes (spindle, coolant, program control)
 *   5. Generate MotionSegments with coordinate transform applied
 *   6. Output segments via the motion callback
 *
 * The coordinate transform (WCS + G52 + G92 + G68 rotation + G51 scale)
 * is applied to all motion segments before output. Position reporting
 * uses the inverse transform to show program coordinates.
 */

#include "tether/gcode/GCodeInterpreter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace GCode {

namespace {

/// Helper to construct an Error with a message string.
Error makeError(ErrorCode code, const char* msg) {
    Error err;
    err.code = code;
    err.message.fill(0);
    err.context.fill(0);
    if (msg) std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    return err;
}

/**
 * @brief Collect Fanuc macro arguments (G65/G66) into a #1..#30 slot vector
 *
 * Fanuc letter→parameter mapping:
 *   A=1 B=2 C=3 I=4 J=5 K=6 D=7 E=8 F=9 H=11 M=13
 *   Q=17 R=18 S=19 T=20 U=21 V=22 W=23 X=24 Y=25 Z=26
 * (G, L, N, O, P are control words and not passed.)
 */
std::vector<double> collectMacroArgs(const Block& block) {
    static constexpr struct { WordLetter w; int param; } kMap[] = {
        {WordLetter::A, 1},  {WordLetter::B, 2},  {WordLetter::C, 3},
        {WordLetter::I, 4},  {WordLetter::J, 5},  {WordLetter::K, 6},
        {WordLetter::D, 7},  {WordLetter::E, 8},  {WordLetter::F, 9},
        {WordLetter::H, 11}, {WordLetter::M, 13}, {WordLetter::Q, 17},
        {WordLetter::R, 18}, {WordLetter::S, 19}, {WordLetter::T, 20},
        {WordLetter::U, 21}, {WordLetter::V, 22}, {WordLetter::W, 23},
        {WordLetter::X, 24}, {WordLetter::Y, 25}, {WordLetter::Z, 26},
    };
    std::vector<double> args(PARAM_LOCAL_END, 0.0);
    size_t used = 0;
    for (const auto& m : kMap) {
        if (block.hasWord(m.w)) {
            args[static_cast<size_t>(m.param) - 1] = block.getWord(m.w);
            used = std::max(used, static_cast<size_t>(m.param));
        }
    }
    args.resize(used);
    return args;
}

} // anonymous namespace

// ============================================================================
// Constructor / Destructor
// ============================================================================

Interpreter::Interpreter(const InterpreterConfig& config)
    : m_config(config)
    , m_lexer(std::make_unique<Lexer>())
    , m_parser(std::make_unique<Parser>(m_variables, config.parser))
    , m_oCodeExecutor(std::make_unique<OCodeExecutor>(m_variables, *m_parser))
    , m_toolLengthComp(m_toolTable)
    , m_cutterRadiusComp(m_toolTable, config.cutterComp)
{
    initializeDefaults();
}

Interpreter::~Interpreter() = default;

// ============================================================================
// Initialization
// ============================================================================

void Interpreter::initializeDefaults() {
    m_machineState = MachineState{};
    m_g66Active = false;
    m_oCodeExecutor->reset();
    m_coordinates.syncToVariables(m_variables);
}

// ============================================================================
// Program Loading
// ============================================================================

Error Interpreter::loadFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        return makeError(ErrorCode::FILE_NOT_FOUND, "Cannot open file");

    size_t size = file.tellg();
    if (size > m_config.maxProgramSize)
        return makeError(ErrorCode::LIMIT_EXCEEDED, "Program exceeds max size");

    file.seekg(0);
    m_programSource.assign(size, '\0');
    file.read(&m_programSource[0], size);
    m_filename = filename;

    m_parser->setInput(m_programSource);
    m_state = InterpreterState::READY;
    return Error{};
}

Error Interpreter::loadString(const std::string& program) {
    if (program.size() > m_config.maxProgramSize)
        return makeError(ErrorCode::LIMIT_EXCEEDED, "Program exceeds max size");

    m_programSource = program;
    m_filename = "<string>";
    m_parser->setInput(m_programSource);
    m_state = InterpreterState::READY;
    return Error{};
}

void Interpreter::unload() {
    m_programSource.clear();
    m_filename.clear();
    m_state = InterpreterState::IDLE;
}

bool Interpreter::isProgramLoaded() const {
    return !m_programSource.empty();
}

// ============================================================================
// Execution Control
// ============================================================================

Error Interpreter::run() {
    if (!isProgramLoaded())
        return makeError(ErrorCode::INVALID_MOTION, "No program loaded");

    m_state = InterpreterState::RUNNING;
    m_machineState.programRunning = true;

    Error err;
    while (m_state == InterpreterState::RUNNING) {
        err = step();
        if (!err.ok()) {
            m_state = InterpreterState::ERROR;
            m_lastError = err;
            m_errors.push_back(err);
            return err;
        }
        if (isFinished())
            break;
    }
    return Error{};
}

Error Interpreter::step() {
    if (!isProgramLoaded())
        return makeError(ErrorCode::INVALID_MOTION, "No program loaded");

    Block block;
    Error err = m_parser->parseNextBlock(block);
    if (err.code == ErrorCode::END)
        m_state = InterpreterState::FINISHED;
    if (!err.ok() && err.code != ErrorCode::END) {
        m_errors.push_back(err);
        m_lastError = err;
        return err;
    }

    if (err.code != ErrorCode::END) {
        err = executeBlock(block);
        if (!err.ok()) {
            m_errors.push_back(err);
            m_lastError = err;
            if (m_config.stopOnError)
                m_state = InterpreterState::ERROR;
            return err;
        }
        m_stats.linesProcessed++;
        m_stats.blocksExecuted++;
    }
    return Error{};
}

void Interpreter::pause() {
    if (m_state == InterpreterState::RUNNING)
        m_state = InterpreterState::PAUSED;
}

Error Interpreter::resume() {
    if (m_state == InterpreterState::PAUSED) {
        m_state = InterpreterState::RUNNING;
        return Error{};
    }
    return makeError(ErrorCode::INVALID_MOTION, "Not paused");
}

void Interpreter::stop() {
    m_state = InterpreterState::STOPPED;
    m_machineState.programRunning = false;
}

void Interpreter::reset() {
    m_state = InterpreterState::IDLE;
    m_machineState = MachineState{};
    m_errors.clear();
    m_lastError = Error{};
    m_stats = Statistics{};
    m_nurbsActive = false;
    m_cannedActive = false;
    m_compHasOffset = false;
    m_g66Active = false;
    initializeDefaults();
}

// ============================================================================
// Real-time commands (GRBL-style)
// ============================================================================

Error Interpreter::feedHold() {
    pause();
    m_machineState.feedHold = true;
    if (m_realtimeCallback) {
        Error err = m_realtimeCallback('!');
        if (!err.ok()) return err;
    }
    return Error{};
}

Error Interpreter::cycleResume() {
    m_machineState.feedHold = false;
    if (m_realtimeCallback) {
        Error err = m_realtimeCallback('~');
        if (!err.ok()) return err;
    }
    if (m_state == InterpreterState::PAUSED)
        return resume();
    return Error{};
}

Error Interpreter::softReset() {
    if (m_realtimeCallback)
        m_realtimeCallback('\x18');
    reset();
    return Error{};
}

Error Interpreter::jogCancel() {
    if (m_realtimeCallback)
        return m_realtimeCallback('\x85');
    return Error{};
}

std::string Interpreter::statusReport() const {
    const char* st = "Idle";
    switch (m_state) {
        case InterpreterState::RUNNING:  st = m_machineState.feedHold ? "Hold:0" : "Run"; break;
        case InterpreterState::PAUSED:   st = "Hold:0"; break;
        case InterpreterState::READY:    st = "Ready"; break;
        case InterpreterState::FINISHED: st = "Idle"; break;
        case InterpreterState::ERROR:    st = "Alarm"; break;
        case InterpreterState::STOPPED:  st = "Alarm"; break;
        default: break;
    }
    const Position& mp = m_machineState.machinePosition;
    std::ostringstream os;
    os << '<' << st << "|MPos:";
    os.precision(3);
    os << std::fixed << mp.x() << ',' << mp.y() << ',' << mp.z();
    os << "|FS:" << m_machineState.feedRate << ','
       << m_machineState.spindleSpeed << '>';
    return os.str();
}

Error Interpreter::systemCommand(const std::string& command) {
    // Strip leading '$'
    const std::string cmd = (!command.empty() && command[0] == '$')
        ? command.substr(1) : command;

    if (cmd == "G") {  // Modal group report
        std::ostringstream os;
        os << "[GC:";
        // Motion mode
        os << "G" << (static_cast<int>(m_machineState.motionMode) / 10);
        os << " G" << static_cast<int>(m_machineState.plane);
        os << " G" << static_cast<int>(m_machineState.distanceMode);
        os << " G" << static_cast<int>(m_machineState.feedMode);
        os << " G" << static_cast<int>(m_machineState.units);
        os << " G" << static_cast<int>(m_machineState.cutterComp);
        os << " G" << static_cast<int>(m_machineState.toolLengthMode);
        os << " T" << m_toolTable.getCurrentTool();
        os << " F" << m_machineState.feedRate;
        os << " S" << m_machineState.spindleSpeed << ']';
        if (m_messageCallback) m_messageCallback(os.str());
        return Error{};
    }
    if (cmd == "#") {  // Coordinate offset report
        const int wcs = m_coordinates.getActiveWCSNumber();
        const Position& off = m_coordinates.getWCS(wcs).offset;
        std::ostringstream os;
        os << "[G" << (53 + wcs) << ":" << off.x() << ',' << off.y()
           << ',' << off.z() << ']';
        if (m_messageCallback) m_messageCallback(os.str());
        return Error{};
    }
    if (cmd == "I") {  // Build info
        if (m_messageCallback)
            m_messageCallback("[Tether GCode Interpreter]");
        return Error{};
    }
    if (cmd == "X") {  // Unlock / clear alarm
        if (m_state == InterpreterState::ERROR ||
            m_state == InterpreterState::STOPPED) {
            m_state = InterpreterState::IDLE;
            m_lastError = Error{};
            m_errors.clear();
        }
        return Error{};
    }
    if (cmd == "H") {  // Home — host must supply homing motion
        if (m_realtimeCallback)
            return m_realtimeCallback('$');
        return Error{};
    }
    if (cmd.rfind("J=", 0) == 0) {  // Jog — defer to host
        if (m_realtimeCallback)
            return m_realtimeCallback('J');
        return Error{};
    }
    if (cmd == "C") {  // Toggle check mode (dry run)
        m_dryRun = !m_dryRun;
        if (m_messageCallback)
            m_messageCallback(m_dryRun ? "[Check mode enabled]"
                                       : "[Check mode disabled]");
        return Error{};
    }
    if (cmd == "N") {  // Report startup lines
        if (m_messageCallback) {
            for (size_t i = 0; i < m_startupLines.size(); ++i)
                m_messageCallback("[N" + std::to_string(i) + "=" +
                                  m_startupLines[i] + "]");
        }
        return Error{};
    }
    if (cmd.size() >= 3 && cmd[0] == 'N' && (cmd[1] == '0' || cmd[1] == '1')
        && cmd[2] == '=') {
        m_startupLines[cmd[1] - '0'] = cmd.substr(3);
        return Error{};
    }
    if (cmd == "$") {  // $$ — list all settings
        if (m_messageCallback) {
            for (const auto& [n, v] : m_grblSettings)
                m_messageCallback("$" + std::to_string(n) + "=" +
                                  std::to_string(v));
        }
        return Error{};
    }
    if (const auto eq = cmd.find('='); eq != std::string::npos && eq > 0 &&
        std::all_of(cmd.begin(), cmd.begin() + static_cast<long>(eq),
                    [](char c) { return std::isdigit(
                        static_cast<unsigned char>(c)); })) {
        // $n=value — settings write
        char* end = nullptr;
        const double value =
            std::strtod(cmd.c_str() + eq + 1, &end);
        if (end == cmd.c_str() + eq + 1 || *end != '\0')
            return makeError(ErrorCode::PARAMETER_ERROR,
                             "Invalid $ setting value");
        m_grblSettings[std::atoi(cmd.c_str())] = value;
        if (m_messageCallback) m_messageCallback("ok");
        return Error{};
    }
    return makeError(ErrorCode::UNKNOWN_GCODE, "Unknown $ command");
}

Error Interpreter::processRealtimeChar(char command) {
    switch (command) {
        case '!':    return feedHold();
        case '~':    return cycleResume();
        case '\x18': return softReset();
        case '\x85': return jogCancel();
        case '?': {
            if (m_messageCallback) m_messageCallback(statusReport());
            return Error{};
        }
        default:
            return makeError(ErrorCode::UNKNOWN_GCODE,
                             "Unknown real-time command");
    }
}

Error Interpreter::executeLine(const std::string& line) {
    Block block;
    Error err = m_parser->parseLine(line.c_str(), block);
    if (!err.ok()) {
        m_errors.push_back(err);
        m_lastError = err;
        return err;
    }
    return executeBlock(block);
}

Error Interpreter::verify() {
    auto savedState = m_state;
    auto savedDryRun = m_dryRun;
    m_dryRun = true;
    m_state = InterpreterState::READY;

    // Reset parser to beginning
    m_parser->setInput(m_programSource);

    Error err;
    while (m_state == InterpreterState::READY) {
        Block block;
        err = m_parser->parseNextBlock(block);
        if (err.code == ErrorCode::END)
            break;
        if (!err.ok()) {
            m_errors.push_back(err);
            m_lastError = err;
            break;
        }
        err = executeBlock(block);
        if (!err.ok()) {
            m_errors.push_back(err);
            m_lastError = err;
            break;
        }
    }

    m_dryRun = savedDryRun;
    m_state = savedState;
    m_parser->setInput(m_programSource);
    return err.code == ErrorCode::END ? Error{} : err;
}

// ============================================================================
// State Queries
// ============================================================================

bool Interpreter::isFinished() const {
    return m_state == InterpreterState::FINISHED ||
           m_state == InterpreterState::ERROR ||
           m_state == InterpreterState::STOPPED;
}

uint32_t Interpreter::getCurrentLine() const {
    return m_stats.linesProcessed;
}

uint32_t Interpreter::getTotalLines() const {
    return static_cast<uint32_t>(
        std::count(m_programSource.begin(), m_programSource.end(), '\n') + 1);
}

// ============================================================================
// Mode Control
// ============================================================================

void Interpreter::setBlockDelete(bool enabled) {
    m_config.skipOptionalBlocks = enabled;
    m_machineState.blockDelete = enabled;
}

bool Interpreter::isBlockDeleteEnabled() const {
    return m_config.skipOptionalBlocks;
}

void Interpreter::setOptionalStop(bool enabled) {
    m_config.m1OptionalStop = enabled;
    m_machineState.optionalStop = enabled;
}

bool Interpreter::isOptionalStopEnabled() const {
    return m_config.m1OptionalStop;
}

// ============================================================================
// Callbacks
// ============================================================================

void Interpreter::setMotionCallback(MotionCallback callback) { m_motionCallback = callback; }
void Interpreter::setMessageCallback(MessageCallback callback) { m_messageCallback = callback; }
void Interpreter::setMCodeCallback(MCodeCallback callback) { m_mcodeCallback = callback; }
void Interpreter::setSpindleCallback(SpindleCallback callback) { m_spindleCallback = callback; }
void Interpreter::setCoolantCallback(CoolantCallback callback) { m_coolantCallback = callback; }
void Interpreter::setDwellCallback(DwellCallback callback) { m_dwellCallback = callback; }
void Interpreter::setProgramControlCallback(ProgramControlCallback callback) { m_programCallback = callback; }
void Interpreter::setToolChangeCallback(ToolChangeCallback callback) { m_toolChangeCallback = callback; }
void Interpreter::setProbeCallback(ProbeMotionCallback callback) {
    m_probeCallback = std::move(callback);
}

// ============================================================================
// Position
// ============================================================================

Position Interpreter::getCurrentPosition() const {
    return m_machineState.workPosition;
}

Position Interpreter::getMachinePosition() const {
    return m_machineState.machinePosition;
}

void Interpreter::setPosition(const Position& pos, bool machineCoords) {
    if (machineCoords) {
        m_machineState.machinePosition = pos;
        m_machineState.workPosition = m_coordinates.toProgramCoords(pos);
    } else {
        m_machineState.workPosition = pos;
        m_machineState.machinePosition = m_coordinates.toMachineCoords(pos);
    }
    updatePositionVariables();
}

// ============================================================================
// Configuration
// ============================================================================

void Interpreter::setConfig(const InterpreterConfig& config) {
    m_config = config;
}

// ============================================================================
// Statistics
// ============================================================================

void Interpreter::resetStatistics() {
    m_stats = Statistics{};
}

// ============================================================================
// Block Execution
// ============================================================================

Error Interpreter::executeBlock(const Block& block) {
    // Skip block if block delete is enabled and block starts with /
    if (block.blockDelete && m_config.skipOptionalBlocks)
        return Error{};

    // Parameter assignment executes before everything else (order of
    // execution: assignments, then O-code flow control, then G/M words).
    if (block.hasParamAssign) {
        ExpressionEvaluator eval(m_variables);
        std::string expr = block.paramAssignExpr.data();
        if (expr.empty() || expr.front() != '[')
            expr = "[" + expr + "]";
        double value = 0.0;
        Error err = eval.evaluate(expr.c_str(), value);
        if (!err.ok())
            return err;
        err = block.paramAssignNamed
            ? m_variables.setNamed(block.paramAssignName.data(), value)
            : m_variables.set(block.paramAssignNumber, value);
        if (!err.ok())
            return err;
    }

    // O-code flow control runs before everything else — it may redirect
    // execution (subroutine call/return, loop, conditional branch).
    if (block.hasOCode) {
        // `o<n> debug/log/print, [expr]` — emit a message and continue.
        if (block.oCodeType == OCodeType::DEBUG ||
            block.oCodeType == OCodeType::LOG ||
            block.oCodeType == OCodeType::PRINT) {
            const char* tag = (block.oCodeType == OCodeType::DEBUG)
                ? "DEBUG" : (block.oCodeType == OCodeType::LOG)
                ? "LOG" : "PRINT";
            std::string msg = tag;
            msg += ": ";
            if (block.oCodeCondition[0] != '\0') {
                double val = 0.0;
                ExpressionEvaluator eval(m_variables);
                if (eval.evaluate(block.oCodeCondition.data(), val).ok()) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%g", val);
                    msg += buf;
                } else {
                    msg += block.oCodeCondition.data();
                }
            }
            if (m_messageCallback)
                m_messageCallback(msg);
            return Error{};
        }
        OCodeExecutor::NextAction action =
            OCodeExecutor::NextAction::CONTINUE;
        Error err = m_oCodeExecutor->execute(block, action);
        if (!err.ok())
            return err;
        if (action == OCodeExecutor::NextAction::JUMP)
            m_parser->getLexer().seek(m_oCodeExecutor->getJumpAddress());
        else if (action == OCodeExecutor::NextAction::EXIT_PROGRAM)
            m_state = InterpreterState::FINISHED;
        // O-code lines carry no G/M words.
        return Error{};
    }

    // Update modal state from non-motion G-codes
    updateModalState(block);

    // Process G-codes
    std::vector<MotionSegment> segments;
    Error err = processGCodes(block, segments);
    if (!err.ok())
        return err;

    // Process M-codes
    err = processMCodes(block);
    if (!err.ok())
        return err;

    // Output motion segments
    if (!segments.empty())
        err = outputSegments(segments);

    // G66 modal macro: re-invoke the subprogram on each block that carries
    // axis words (skipping blocks that set/cancel the modal call).
    if (err.ok() && m_g66Active && hasMotionWords(block)) {
        bool setsModalMacro = false;
        for (uint8_t i = 0; i < block.gCodeCount; ++i) {
            if (block.gCodes[i] == 660 || block.gCodes[i] == 670) {
                setsModalMacro = true;
                break;
            }
        }
        if (!setsModalMacro) {
            Error callErr = m_oCodeExecutor->callSubprogram(
                m_g66Program, collectMacroArgs(block));
            if (!callErr.ok())
                return callErr;
            m_parser->getLexer().seek(m_oCodeExecutor->getJumpAddress());
        }
    }

    // Update position variables
    updatePositionVariables();

    // Sync coordinate state to variables
    m_coordinates.syncToVariables(m_variables);

    return err;
}

// ============================================================================
} // namespace GCode
