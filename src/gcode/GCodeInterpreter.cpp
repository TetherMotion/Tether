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
// G-code Processing
// ============================================================================

Error Interpreter::processGCodes(const Block& block,
                                  std::vector<MotionSegment>& segments) {
    for (uint8_t i = 0; i < block.gCodeCount; ++i) {
        // G-codes are encoded as major*10+minor in int16_t.
        // E.g., G1 → 10, G10 → 100, G54 → 540, G38.2 → 382.
        // getModalGroup() expects the encoded value.
        int encoded = block.gCodes[i];

        Error err = dispatchGCode(encoded, block, segments);
        if (!err.ok())
            return err;
    }

    // Handle implicit motion (no G-code in block but motion words present)
    if (block.gCodeCount == 0 && hasMotionWords(block)) {
        Error err = handleMotion(block, segments);
        if (!err.ok())
            return err;
    }

    return Error{};
}

bool Interpreter::hasMotionWords(const Block& block) const {
    return block.hasWord(WordLetter::X) ||
           block.hasWord(WordLetter::Y) ||
           block.hasWord(WordLetter::Z) ||
           block.hasWord(WordLetter::A) ||
           block.hasWord(WordLetter::B) ||
           block.hasWord(WordLetter::C) ||
           block.hasWord(WordLetter::U) ||
           block.hasWord(WordLetter::V) ||
           block.hasWord(WordLetter::W);
}

Error Interpreter::dispatchGCode(double gcode, const Block& block,
                                  std::vector<MotionSegment>& segments) {
    // gcode is the encoded value (major*10+minor).
    int gi = static_cast<int>(gcode);

    // Determine modal group (expects encoded value).
    ModalGroup group = getModalGroup(gi);

    // Decode major number for switch statements.
    int major = gi / 10;
    int minor = gi % 10;
    // For non-decimal G-codes, use the major number directly.
    int gnum = (minor > 0) ? gi : major;

    switch (group) {
        case ModalGroup::MOTION:
            return handleMotion(block, segments);

        case ModalGroup::PLANE: {
            switch (gi) {
                case 170: m_machineState.plane = Plane::XY; break;
                case 180: m_machineState.plane = Plane::ZX; break;
                case 190: m_machineState.plane = Plane::YZ; break;
                case 171: m_machineState.plane = Plane::UV; break;
                case 181: m_machineState.plane = Plane::WU; break;
                case 191: m_machineState.plane = Plane::VW; break;
                default: break;
            }
            return Error{};
        }

        case ModalGroup::DISTANCE: {
            if (gnum == 90)
                m_machineState.distanceMode = DistanceMode::ABSOLUTE;
            else if (gnum == 91)
                m_machineState.distanceMode = DistanceMode::INCREMENTAL;
            return Error{};
        }

        case ModalGroup::ARC_DISTANCE: {
            if (gi == 901) // G90.1 → encoded 901
                m_machineState.arcDistanceMode = ArcDistanceMode::ABSOLUTE;
            else if (gi == 911) // G91.1 → encoded 911
                m_machineState.arcDistanceMode = ArcDistanceMode::INCREMENTAL;
            return Error{};
        }

        case ModalGroup::FEED_MODE: {
            if (gnum == 93)
                m_machineState.feedMode = FeedMode::INVERSE_TIME;
            else if (gnum == 94)
                m_machineState.feedMode = FeedMode::UNITS_PER_MIN;
            else if (gnum == 95)
                m_machineState.feedMode = FeedMode::UNITS_PER_REV;
            return Error{};
        }

        case ModalGroup::UNITS: {
            if (gnum == 20)
                m_machineState.units = Units::INCH;
            else if (gnum == 21)
                m_machineState.units = Units::MM;
            return Error{};
        }

        case ModalGroup::COORD_SYSTEM: {
            int32_t wcsNum = 1;
            if (gi == 541) {
                // G54.1 Pn (Haas/Fanuc ENS) — extended WCS 10+
                const int p = static_cast<int>(
                    block.getWord(WordLetter::P, 0));
                if (p < 1)
                    return makeError(ErrorCode::INVALID_MOTION,
                                     "G54.1 requires P word");
                wcsNum = 9 + p;
            } else if (gnum >= 54 && gnum <= 59) wcsNum = gnum - 53;
            else if (gi == 591) wcsNum = 7; // G59.1
            else if (gi == 592) wcsNum = 8; // G59.2
            else if (gi == 593) wcsNum = 9; // G59.3
            Error err = m_coordinates.selectWCS(wcsNum);
            if (!err.ok()) return err;
            m_coordinates.syncTransform(m_machineState);
            return Error{};
        }

        case ModalGroup::CANNED_RETURN: {
            if (gnum == 98)
                m_machineState.cannedReturn = CannedReturnMode::INITIAL;
            else if (gnum == 99)
                m_machineState.cannedReturn = CannedReturnMode::R_PLANE;
            return Error{};
        }

        case ModalGroup::PATH_MODE: {
            if (gnum == 61 || gi == 611)
                m_machineState.pathMode = PathMode::EXACT_STOP;
            else if (gnum == 64) {
                m_machineState.pathMode = PathMode::BLEND;
                if (block.hasWord(WordLetter::P))
                    m_machineState.blendTolerance = block.getWord(WordLetter::P);
                if (block.hasWord(WordLetter::Q))
                    m_machineState.naiveCamTolerance = block.getWord(WordLetter::Q);
            }
            return Error{};
        }

        case ModalGroup::SPINDLE_MODE: {
            if (gnum == 96)
                m_machineState.spindleMode = SpindleMode::CSS;
            else if (gnum == 97)
                m_machineState.spindleMode = SpindleMode::RPM;
            return Error{};
        }

        case ModalGroup::LATHE_DIAMETER: {
            if (gnum == 7)
                m_machineState.latheMode = LatheMode::DIAMETER;
            else if (gnum == 8)
                m_machineState.latheMode = LatheMode::RADIUS;
            return Error{};
        }

        case ModalGroup::CUTTER_COMP: {
            switch (gi) {
                case 400: // G40: cancel cutter compensation
                    m_machineState.cutterComp = CutterCompMode::OFF;
                    m_machineState.cutterRadius = 0.0;
                    return Error{};
                case 410:
                case 420: { // G41/G42 D<tool>: comp from tool table diameter
                    const int tool =
                        static_cast<int>(block.getWord(WordLetter::D, 0));
                    const ToolEntry* entry =
                        (tool > 0) ? m_toolTable.getTool(tool)
                                   : m_toolTable.getCurrentToolEntry();
                    if (!entry) {
                        Error err;
                        err.code = ErrorCode::TOOL_ERROR;
                        std::snprintf(err.message.data(), err.message.size(),
                                      "G%d: no tool entry for D%d",
                                      gnum, tool);
                        return err;
                    }
                    m_machineState.cutterComp = (gnum == 41)
                        ? CutterCompMode::LEFT : CutterCompMode::RIGHT;
                    m_machineState.cutterRadius =
                        entry->getEffectiveRadius();
                    return Error{};
                }
                case 411:
                case 421: { // G41.1/G42.1 D<diameter>: dynamic comp
                    m_machineState.cutterComp = (gi == 411)
                        ? CutterCompMode::LEFT_DYNAMIC
                        : CutterCompMode::RIGHT_DYNAMIC;
                    m_machineState.cutterRadius =
                        block.getWord(WordLetter::D, 0.0) / 2.0;
                    return Error{};
                }
                default:
                    return Error{};
            }
        }

        case ModalGroup::TOOL_LENGTH: {
            switch (gi) {
                case 430: { // G43 H<tool>: apply offset from tool table
                    const int tool =
                        static_cast<int>(block.getWord(WordLetter::H, 0));
                    const ToolEntry* entry =
                        (tool > 0) ? m_toolTable.getTool(tool)
                                   : m_toolTable.getCurrentToolEntry();
                    if (!entry) {
                        Error err;
                        err.code = ErrorCode::TOOL_ERROR;
                        std::snprintf(err.message.data(), err.message.size(),
                                      "G43: no tool entry for H%d", tool);
                        return err;
                    }
                    m_machineState.toolLengthMode = ToolLengthMode::POSITIVE;
                    m_machineState.toolOffset.x() = entry->xOffset + entry->xWear;
                    m_machineState.toolOffset.y() = entry->yOffset + entry->yWear;
                    m_machineState.toolOffset.z() = entry->getEffectiveZOffset();
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                case 431: { // G43.1: dynamic tool length offset
                    m_machineState.toolLengthMode = ToolLengthMode::DYNAMIC;
                    if (block.hasWord(WordLetter::X))
                        m_machineState.toolOffset.x() = block.getWord(WordLetter::X);
                    if (block.hasWord(WordLetter::Y))
                        m_machineState.toolOffset.y() = block.getWord(WordLetter::Y);
                    if (block.hasWord(WordLetter::Z))
                        m_machineState.toolOffset.z() = block.getWord(WordLetter::Z);
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                case 432: { // G43.2 H<tool>: add another tool's offset
                    const int tool =
                        static_cast<int>(block.getWord(WordLetter::H, 0));
                    const ToolEntry* entry = m_toolTable.getTool(tool);
                    if (!entry) {
                        Error err;
                        err.code = ErrorCode::TOOL_ERROR;
                        std::snprintf(err.message.data(), err.message.size(),
                                      "G43.2: no tool entry for H%d", tool);
                        return err;
                    }
                    m_machineState.toolLengthMode = ToolLengthMode::ADDITIONAL;
                    m_machineState.toolOffset.x() += entry->xOffset + entry->xWear;
                    m_machineState.toolOffset.y() += entry->yOffset + entry->yWear;
                    m_machineState.toolOffset.z() += entry->getEffectiveZOffset();
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                case 490: { // G49: cancel tool length offset
                    m_machineState.toolLengthMode = ToolLengthMode::OFF;
                    m_machineState.toolOffset = Position{};
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                default:
                    return Error{};
            }
        }

        case ModalGroup::LOCAL_OFFSET:
            return dispatchG52(block);

        case ModalGroup::COORD_ROTATION:
            if (gnum == 68) return dispatchG68(block);
            if (gnum == 69) return dispatchG69();
            return Error{};

        case ModalGroup::SCALING:
            if (gnum == 51) return dispatchG51(block);
            if (gnum == 50) return dispatchG50(block);
            if (gnum == 511) return dispatchG51_1(block);  // G51.1 mirror on
            if (gnum == 501) return dispatchG50_1(block);  // G50.1 mirror off
            return Error{};

        case ModalGroup::NON_MODAL: {
            switch (gnum) {
                case 4: { // G4 — Dwell
                    double seconds = 0;
                    if (block.hasWord(WordLetter::P))
                        seconds = block.getWord(WordLetter::P) / 1000.0;
                    else if (block.hasWord(WordLetter::S))
                        seconds = block.getWord(WordLetter::S);
                    if (m_dwellCallback)
                        m_dwellCallback(seconds);
                    MotionSegment seg;
                    seg.type = MotionSegment::Type::DWELL;
                    seg.duration = seconds;
                    seg.lineNumber = block.sourceLineNumber;
                    // Set endPosition to the current position so that
                    // downstream consumers (PlanningSegmentBuilder) can
                    // correctly track position across dwell segments.
                    seg.endPosition = m_machineState.workPosition;
                    segments.push_back(seg);
                    return Error{};
                }
                case 10: {
                    if (!block.hasWord(WordLetter::L)) {
                        // RepRap/Marlin G10 — firmware retract
                        if (m_userGCodeCallback)
                            return m_userGCodeCallback(10, block);
                        return Error{};
                    }
                    // G10 L2/L20 — set WCS data
                    int l = static_cast<int>(block.getWord(WordLetter::L, 2));
                    int p = static_cast<int>(block.getWord(WordLetter::P, 1));
                    if (l == 2)
                        return m_coordinates.processG10L2(p, block, m_variables);
                    else if (l == 20)
                        return m_coordinates.processG10L20(
                            p, block, m_machineState.machinePosition, m_variables);
                    return makeError(ErrorCode::INVALID_MOTION, "G10 L not supported");
                }
                case 11: // RepRap/Marlin G11 — firmware unretract
                    if (m_userGCodeCallback)
                        return m_userGCodeCallback(11, block);
                    return Error{};
                case 29: // RepRap/Marlin G29 — auto bed leveling
                    if (m_userGCodeCallback)
                        return m_userGCodeCallback(29, block);
                    return Error{};
                case 28: {
                    // G28 — Go to reference point 1
                    // Store current position, then rapid to reference
                    Position target = m_coordinates.getG28Reference();
                    MotionSegment seg;
                    seg.type = MotionSegment::Type::RAPID;
                    seg.endPosition = m_coordinates.toMachineCoords(target);
                    seg.lineNumber = block.sourceLineNumber;
                    segments.push_back(seg);
                    m_machineState.workPosition = target;
                    m_machineState.machinePosition =
                        m_coordinates.toMachineCoords(target);
                    return Error{};
                }
                case 30: {
                    // Feature::G30_PROBE — Marlin/RepRap single-point probe.
                    if (featureEnabled(Feature::G30_PROBE)) {
                        const double unitScale =
                            (m_machineState.units == Units::INCH) ? 25.4 : 1.0;
                        // Optional XY position first (rapid).
                        if (block.hasWord(WordLetter::X) ||
                            block.hasWord(WordLetter::Y)) {
                            Position xy = m_machineState.workPosition;
                            if (block.hasWord(WordLetter::X))
                                xy.x() = block.getWord(WordLetter::X) * unitScale;
                            if (block.hasWord(WordLetter::Y))
                                xy.y() = block.getWord(WordLetter::Y) * unitScale;
                            MotionSegment mv;
                            mv.type = MotionSegment::Type::RAPID;
                            mv.endPosition =
                                m_coordinates.toMachineCoords(xy);
                            mv.lineNumber = block.sourceLineNumber;
                            segments.push_back(mv);
                            m_machineState.workPosition = xy;
                            m_machineState.machinePosition = mv.endPosition;
                            ++m_stats.motionSegments;
                        }
                        // Probe toward Z (word value or bed plane z=0).
                        Position target = m_machineState.workPosition;
                        target.z() = block.hasWord(WordLetter::Z)
                            ? block.getWord(WordLetter::Z) * unitScale : 0.0;
                        if (block.hasWord(WordLetter::F))
                            m_machineState.feedRate =
                                block.getWord(WordLetter::F) * unitScale;
                        Error e = executeProbe(MotionMode::PROBE_TOWARD_NE,
                                               block, target, unitScale,
                                               segments);
                        if (!e.ok()) return e;
                        if (m_messageCallback) {
                            char buf[96];
                            std::snprintf(buf, sizeof(buf),
                                          "Bed X: %.3f Y: %.3f Z: %.3f",
                                          m_machineState.workPosition.x(),
                                          m_machineState.workPosition.y(),
                                          m_machineState.workPosition.z());
                            m_messageCallback(buf);
                        }
                        return Error{};
                    }
                    // G30 — Go to reference point 2
                    Position target = m_coordinates.getG30Reference(1);
                    MotionSegment seg;
                    seg.type = MotionSegment::Type::RAPID;
                    seg.endPosition = m_coordinates.toMachineCoords(target);
                    seg.lineNumber = block.sourceLineNumber;
                    segments.push_back(seg);
                    m_machineState.workPosition = target;
                    m_machineState.machinePosition =
                        m_coordinates.toMachineCoords(target);
                    return Error{};
                }
                case 53: {
                    // G53 — Move in machine coordinates (non-modal)
                    Position target = m_machineState.machinePosition;
                    if (block.hasWord(WordLetter::X))
                        target.x() = block.getWord(WordLetter::X);
                    if (block.hasWord(WordLetter::Y))
                        target.y() = block.getWord(WordLetter::Y);
                    if (block.hasWord(WordLetter::Z))
                        target.z() = block.getWord(WordLetter::Z);
                    MotionSegment seg;
                    seg.type = MotionSegment::Type::RAPID;
                    seg.endPosition = target; // Already machine coords
                    seg.lineNumber = block.sourceLineNumber;
                    segments.push_back(seg);
                    m_machineState.machinePosition = target;
                    m_machineState.workPosition =
                        m_coordinates.toProgramCoords(target);
                    return Error{};
                }
                case 92: {
                    // G92 — Set position offset
                    return m_coordinates.processG92(
                        block, m_machineState.machinePosition,
                        m_machineState, m_variables);
                }
                case 65: { // G65 P<n> — Macro call (non-modal)
                    if (!block.hasWord(WordLetter::P))
                        return makeError(ErrorCode::UNKNOWN_GCODE,
                                         "G65 requires P word");
                    int32_t p = static_cast<int32_t>(
                        block.getWord(WordLetter::P));
                    Error err = m_oCodeExecutor->callSubprogram(
                        p, collectMacroArgs(block));
                    if (!err.ok())
                        return err;
                    m_parser->getLexer().seek(
                        m_oCodeExecutor->getJumpAddress());
                    return Error{};
                }
                case 66: { // G66 P<n> — Modal macro call
                    if (!block.hasWord(WordLetter::P))
                        return makeError(ErrorCode::UNKNOWN_GCODE,
                                         "G66 requires P word");
                    m_g66Program = static_cast<int32_t>(
                        block.getWord(WordLetter::P));
                    m_g66Active = true;
                    Error err = m_oCodeExecutor->callSubprogram(
                        m_g66Program, collectMacroArgs(block));
                    if (!err.ok())
                        return err;
                    m_parser->getLexer().seek(
                        m_oCodeExecutor->getJumpAddress());
                    return Error{};
                }
                case 67: // G67 — Cancel modal macro call
                    m_g66Active = false;
                    return Error{};
                case 187: { // G187 — Haas accuracy/smoothing mode
                    // E<tolerance> sets blend tolerance; P1-P3 selects level.
                    m_machineState.pathMode = PathMode::BLEND;
                    if (block.hasWord(WordLetter::E))
                        m_machineState.blendTolerance =
                            block.getWord(WordLetter::E);
                    return Error{};
                }
                case 12:
                case 13: { // G12/G13 — Haas circular pocket mill (CW/CCW)
                    // I = circle radius; optional K = finished radius with
                    // Q stepover between passes; Z = depth; L = repeats.
                    const bool cw = (gnum == 12);
                    const double i0 = block.getWord(WordLetter::I, 0.0);
                    const double rk = block.getWord(WordLetter::K, 0.0);
                    const double q = block.getWord(WordLetter::Q, 0.0);
                    const int repeats = static_cast<int>(
                        block.getWord(WordLetter::L, 1.0));
                    if (i0 <= 0.0)
                        return makeError(ErrorCode::INVALID_MOTION,
                                         "G12/G13 requires positive I radius");
                    const Position center = m_machineState.workPosition;

                    // Optional plunge to Z depth at feed rate
                    if (block.hasWord(WordLetter::Z)) {
                        Position zt = center;
                        zt.z() = block.getWord(WordLetter::Z);
                        MotionSegment seg;
                        seg.type = MotionSegment::Type::LINEAR;
                        seg.endPosition = m_coordinates.toMachineCoords(zt);
                        seg.feedRate = m_machineState.feedRate;
                        seg.lineNumber = block.sourceLineNumber;
                        segments.push_back(seg);
                        ++m_stats.motionSegments;
                        m_machineState.workPosition = zt;
                    }

                    // Radius passes: I, then step by Q up to K if given
                    for (double r = i0; r <= (rk > 0 ? rk : i0) + 1e-9;
                         r += (q > 0 ? q : rk + 1)) {
                        for (int rep = 0; rep < repeats; ++rep) {
                            // Lead-in: feed from center to circle edge
                            Position edge = m_machineState.workPosition;
                            edge.x() = center.x() + r;
                            edge.y() = center.y();
                            MotionSegment lead;
                            lead.type = MotionSegment::Type::LINEAR;
                            lead.endPosition =
                                m_coordinates.toMachineCoords(edge);
                            lead.feedRate = m_machineState.feedRate;
                            lead.lineNumber = block.sourceLineNumber;
                            segments.push_back(lead);

                            // Full circle about center
                            MotionSegment arc;
                            arc.type = cw ? MotionSegment::Type::ARC_CW
                                          : MotionSegment::Type::ARC_CCW;
                            arc.endPosition = lead.endPosition;
                            arc.feedRate = m_machineState.feedRate;
                            arc.lineNumber = block.sourceLineNumber;
                            arc.centerOffset.x() = -r;
                            arc.centerOffset.y() = 0.0;
                            arc.arc.center = center;
                            arc.arc.startPoint = edge;
                            arc.arc.endPoint = edge;
                            arc.arc.radius = r;
                            arc.arc.startAngle = 0.0;
                            arc.arc.endAngle = cw ? -2.0 * M_PI : 2.0 * M_PI;
                            arc.arc.sweepAngle = arc.arc.endAngle;
                            arc.arc.clockwise = cw;
                            arc.arc.plane = m_machineState.plane;
                            arc.arc.valid = true;
                            segments.push_back(arc);
                            m_stats.motionSegments += 2;
                            m_machineState.workPosition = edge;
                        }
                        if (q <= 0.0) break;
                    }
                    // Return to center
                    MotionSegment ret;
                    ret.type = MotionSegment::Type::LINEAR;
                    Position cc = m_machineState.workPosition;
                    cc.x() = center.x(); cc.y() = center.y();
                    ret.endPosition = m_coordinates.toMachineCoords(cc);
                    ret.feedRate = m_machineState.feedRate;
                    ret.lineNumber = block.sourceLineNumber;
                    segments.push_back(ret);
                    ++m_stats.motionSegments;
                    m_machineState.workPosition = cc;
                    m_machineState.machinePosition = ret.endPosition;
                    return Error{};
                }
                case 150: // G150 — Haas generic pocket milling
                    return dispatchG150(block, segments);
                case 70: // G70 — Fanuc lathe finishing cycle
                    return dispatchG70(block, segments);
                case 71: // G71 — Fanuc lathe rough turning cycle
                    return dispatchG71(block, segments);
                case 72: // G72 — Fanuc lathe rough facing cycle
                    return dispatchG72(block, segments);
                case 921: // G92.1 — Reset G92, zero position
                    return m_coordinates.processG92_1(m_machineState, m_variables);
                case 922: // G92.2 — Reset G92, keep position
                    return m_coordinates.processG92_2(m_machineState);
                case 923: // G92.3 — Restore G92
                    return m_coordinates.processG92_3(m_machineState);
                default:
                    break;
            }
            return Error{};
        }

        default:
            // Unhandled modal group — ignore for now
            return Error{};
    }
}

// ============================================================================
// M-code Processing
// ============================================================================

Error Interpreter::processMCodes(const Block& block) {
    for (uint8_t i = 0; i < block.mCodeCount; ++i) {
        int mcode = block.mCodes[i];
        Error err = dispatchMCode(mcode, block);
        if (!err.ok())
            return err;
    }
    return Error{};
}

Error Interpreter::dispatchMCode(int32_t mcode, const Block& block) {
    switch (mcode) {
        case 0: // M0 — Program stop
            if (m_programCallback) m_programCallback(0);
            return Error{};
        case 1: // M1 — Optional stop
            if (m_config.m1OptionalStop && m_programCallback)
                m_programCallback(1);
            return Error{};
        case 2: // M2 — Program end
            m_state = InterpreterState::FINISHED;
            if (m_programCallback) m_programCallback(2);
            return Error{};
        case 30: // M30 — Program end + rewind
            m_state = InterpreterState::FINISHED;
            if (m_programCallback) m_programCallback(30);
            return Error{};
        case 3: case 4: { // M3/M4 — Spindle on
            bool cw = (mcode == 3);
            double rpm = block.getWord(WordLetter::S, m_machineState.spindleSpeed);
            if (m_machineState.maxSpindleSpeed > 0.0)
                rpm = std::min(rpm, m_machineState.maxSpindleSpeed);
            m_machineState.spindleSpeed = rpm;
            m_machineState.spindleCW = cw;
            m_machineState.spindleOn = true;
            if (m_spindleCallback)
                return m_spindleCallback(true, cw, rpm);
            return Error{};
        }
        case 5: { // M5 — Spindle off
            m_machineState.spindleOn = false;
            if (m_spindleCallback)
                return m_spindleCallback(false, true, 0);
            return Error{};
        }
        case 7: // M7 — Mist coolant
            m_machineState.coolantMist = true;
            if (m_coolantCallback)
                return m_coolantCallback(true, m_machineState.coolantFlood);
            return Error{};
        case 8: // M8 — Flood coolant
            m_machineState.coolantFlood = true;
            if (m_coolantCallback)
                return m_coolantCallback(m_machineState.coolantMist, true);
            return Error{};
        case 9: // M9 — Coolant off
            m_machineState.coolantMist = false;
            m_machineState.coolantFlood = false;
            if (m_coolantCallback)
                return m_coolantCallback(false, false);
            return Error{};
        case 6: { // M6 — Tool change
            int tool = static_cast<int>(block.getWord(WordLetter::T, 0));
            m_machineState.selectedTool = tool;
            if (m_toolChangeCallback)
                return m_toolChangeCallback(tool, m_machineState.currentTool,
                                            m_machineState);
            m_machineState.currentTool = tool;
            return Error{};
        }
        case 70: // M70 — Save modal state
            return m_oCodeExecutor->saveModalState(m_machineState);
        case 71: // M71 — Invalidate stored modal state
            return m_oCodeExecutor->invalidateModalState();
        case 72: // M72 — Restore modal state
            return m_oCodeExecutor->restoreModalState(m_machineState);
        case 73: // M73 — Save modal state, auto-restore on sub return
            return m_oCodeExecutor->autoRestoreModalState(m_machineState);
        case 98: { // M98 P<num> L<count> — Call subprogram
            if (!block.hasWord(WordLetter::P))
                return makeError(ErrorCode::UNKNOWN_MCODE,
                                 "M98 requires P word");
            int32_t p = static_cast<int32_t>(block.getWord(WordLetter::P));
            int32_t l = static_cast<int32_t>(
                block.getWord(WordLetter::L, 1));
            Error err = m_oCodeExecutor->executeM98(p, l);
            if (!err.ok())
                return err;
            m_parser->getLexer().seek(m_oCodeExecutor->getJumpAddress());
            return Error{};
        }
        case 99: { // M99 — Return from subprogram (or repeat)
            OCodeExecutor::NextAction action =
                OCodeExecutor::NextAction::CONTINUE;
            Error err = m_oCodeExecutor->executeM99(action);
            if (!err.ok())
                return err;
            if (action == OCodeExecutor::NextAction::JUMP)
                m_parser->getLexer().seek(m_oCodeExecutor->getJumpAddress());
            return Error{};
        }
        case 19: { // M19 — Spindle orient (Haas)
            m_machineState.spindleOn = true;
            if (m_spindleCallback)
                return m_spindleCallback(true, m_machineState.spindleCW,
                                         block.getWord(WordLetter::S, 0.0));
            return Error{};
        }
        // --- Marlin M-codes (3D-printer dialect) ---
        case 400: // M400 — Wait for all queued moves to finish
            // Barrier: downstream consumers see all prior segments first;
            // nothing to emit at interpreter level.
            if (m_mcodeCallback)
                return m_mcodeCallback(mcode, std::nullopt, std::nullopt);
            return Error{};
        case 600: // M600 — Filament change (pause + user hook)
            if (m_programCallback) m_programCallback(0);
            if (m_mcodeCallback)
                return m_mcodeCallback(mcode, std::nullopt, std::nullopt);
            return Error{};
        case 41:  // M41-M44 — Spindle gear range select (Haas)
        case 42:
        case 43:
        case 44:
        case 48:  // M48 — Enable feed/spindle overrides
            m_machineState.feedOverrideEnabled = true;
            m_machineState.spindleOverrideEnabled = true;
            return Error{};
        case 49:  // M49 — Disable overrides
            m_machineState.feedOverrideEnabled = false;
            m_machineState.spindleOverrideEnabled = false;
            return Error{};
        case 50:  // M50 — Feed override (P = scale, e.g. P1.1)
            if (block.hasWord(WordLetter::P))
                m_machineState.feedOverride =
                    block.getWord(WordLetter::P);
            return Error{};
        case 51:  // M51 — Spindle override (P = scale)
            if (block.hasWord(WordLetter::P))
                m_machineState.spindleOverride =
                    block.getWord(WordLetter::P);
            return Error{};
        case 52:  // M52 — Hold override (P0 = off, else on)
            m_machineState.feedHold =
                block.getWord(WordLetter::P, 1.0) == 0.0;
            return Error{};
        case 53:  // M53 — Feed/spindle override reset
            m_machineState.feedOverride = 1.0;
            m_machineState.spindleOverride = 1.0;
            return Error{};
        case 60:  // M60 — Pallet change
        case 17:  // M17 — Enable motors
        case 18:  // M18 — Disable motors
        case 84:  // M84 — Idle motors off
        case 104: // M104 — Set hotend temp (S)
        case 109: // M109 — Set hotend temp and wait
        case 140: // M140 — Set bed temp
        case 190: // M190 — Set bed temp and wait
        case 141: // M141 — Set chamber temp
        case 191: // M191 — Set chamber temp and wait
        case 106: // M106 — Fan on (S)
        case 107: // M107 — Fan off
        case 500: // M500 — Save settings to EEPROM
        case 501: // M501 — Load settings
        case 502: // M502 — Factory reset
        case 900: { // M900 — Linear advance (K factor)
            // Printer-domain commands: forward to the user M-code hook when
            // registered, otherwise accept silently.
            if (m_mcodeCallback) {
                std::optional<double> p, q;
                if (block.hasWord(WordLetter::P))
                    p = block.getWord(WordLetter::P);
                if (block.hasWord(WordLetter::Q))
                    q = block.getWord(WordLetter::Q);
                return m_mcodeCallback(mcode, p, q);
            }
            return Error{};
        }
        // --- Marlin/RepRap M-codes with interpreter-level state ---
        case 82:  // M82 — Extruder absolute mode
        case 83:  // M83 — Extruder relative mode
        case 92:  // M92 — Steps per mm (X Y Z E)
        case 114: // M114 — Report position
        case 115: // M115 — Report firmware info
        case 117: // M117 — Display message
        case 201: // M201 — Max acceleration per axis
        case 203: // M203 — Max feedrate per axis
        case 204: // M204 — Acceleration (P/T/R/S)
        case 205: // M205 — Advanced motion settings (jerk/JD)
        case 206: // M206 — Home offset
        case 218: // M218 — Hotend/tool offset
        case 220: // M220 — Feedrate override percent
        case 221: // M221 — Flow override percent
        case 280: // M280 — Servo position (P<n> S<angle>)
        case 301: // M301 — Hotend PID
        case 304: // M304 — Bed PID
        case 401: // M401 — Deploy probe
        case 402: // M402 — Stow probe
        case 420: // M420 — Bed leveling state
        case 421: // M421 — Set mesh point
        case 503: // M503 — Report settings
        case 851: // M851 — Probe offset
            return dispatchMarlinMCode(mcode, block);
        default:
            // Forward to user M-code callback
            if (m_mcodeCallback) {
                std::optional<double> p, q;
                if (block.hasWord(WordLetter::P))
                    p = block.getWord(WordLetter::P);
                if (block.hasWord(WordLetter::Q))
                    q = block.getWord(WordLetter::Q);
                return m_mcodeCallback(mcode, p, q);
            }
            return Error{};
    }
}

Error Interpreter::dispatchMarlinMCode(int32_t mcode, const Block& block) {
    // Sync the Marlin-side state snapshot from the core machine state.
    m_marlinState.currentPosition = m_machineState.machinePosition;
    m_marlinState.currentFeedrate = m_machineState.feedRate;
    m_marlinState.currentSpindleSpeed = m_machineState.spindleSpeed;
    m_marlinState.spindleOn = m_machineState.spindleOn;
    m_marlinState.spindleCW = m_machineState.spindleCW;
    m_marlinState.currentTool = m_machineState.currentTool;
    m_marlinState.coolantMist = m_machineState.coolantMist;
    m_marlinState.coolantFlood = m_machineState.coolantFlood;
    m_marlinState.isMetric = (m_machineState.units == Units::MM);
    m_marlinState.absoluteMode =
        (m_machineState.distanceMode == DistanceMode::ABSOLUTE);

    MCodeParameters params;
    params.mCode = mcode;
    auto grab = [&block](WordLetter w, std::optional<double>& out) {
        if (block.hasWord(w)) out = block.getWord(w);
    };
    grab(WordLetter::P, params.P);
    grab(WordLetter::S, params.S);
    grab(WordLetter::R, params.R);
    grab(WordLetter::F, params.F);
    grab(WordLetter::T, params.T);
    grab(WordLetter::D, params.D);
    grab(WordLetter::I, params.I);
    grab(WordLetter::J, params.J);
    grab(WordLetter::K, params.K);
    grab(WordLetter::X, params.X);
    grab(WordLetter::Y, params.Y);
    grab(WordLetter::Z, params.Z);
    grab(WordLetter::A, params.A);
    grab(WordLetter::B, params.B);
    grab(WordLetter::C, params.C);
    grab(WordLetter::U, params.U);
    grab(WordLetter::V, params.V);
    grab(WordLetter::W, params.W);
    grab(WordLetter::E, params.E);
    params.lineNumber = block.lineNumber;
    params.rawLine = std::string(block.originalText.data());

    if (mcode == 117) {
        // The parser stashes the free text after M117 in block.comment.
        if (block.hasComment)
            params.message = block.comment.data();
    }

    MCodeResult result = m_marlinHandler.execute(params, m_marlinState);
    if (!result.success) {
        return makeError(ErrorCode::PARAMETER_ERROR, result.message.c_str());
    }

    // Report: prefer response text, fall back to message.
    if (m_messageCallback) {
        if (!result.response.empty())
            m_messageCallback(result.response);
        else if (!result.message.empty())
            m_messageCallback(result.message);
    }

    // Apply interpreter-visible results.
    if (mcode == 220 && m_machineState.feedOverrideEnabled)
        m_machineState.feedOverride = m_marlinState.feedOverridePercent / 100.0;
    if (result.pauseExecution)
        m_state = InterpreterState::PAUSED;
    if (result.stopExecution)
        m_state = InterpreterState::FINISHED;

    // Still forward to the user M-code hook for hardware side effects.
    if (m_mcodeCallback) {
        std::optional<double> p, q;
        if (block.hasWord(WordLetter::P)) p = block.getWord(WordLetter::P);
        if (block.hasWord(WordLetter::Q)) q = block.getWord(WordLetter::Q);
        return m_mcodeCallback(mcode, p, q);
    }
    return Error{};
}

// ============================================================================
// G150 — Haas generic pocket milling
// ============================================================================
//
// Strategy: the pocket boundary is the closed XY profile defined by
// subprogram P (collected by walking its blocks, arcs tessellated). Each
// Z level is cleared with horizontal raster scanlines clipped to the
// polygon by even-odd fill; intervals are joined with retract-rapid
// between disjoint spans. A finish pass then traces the boundary itself
// (optionally inset by the K finish allowance applied as a uniform XY
// scale toward the polygon centroid — exact inward offsetting of
// arbitrary polygons is out of scope).

Error Interpreter::collectPocketBoundary(
        int32_t prog, double unitScale,
        std::vector<std::pair<double, double>>& pts) {
    // Locate the subprogram: Fanuc-style bare `O<num>` label first, then
    // LinuxCNC `O<num> sub`. findSubprogramLabel/findSubroutine restore the
    // lexer position; getLastBlockEnd() still points just past the label
    // line, which is where the boundary blocks begin.
    Block label;
    Error findErr = m_parser->findSubprogramLabel(prog, label);
    if (!findErr.ok())
        findErr = m_parser->findSubroutine(prog, label);
    if (!findErr.ok())
        return makeError(ErrorCode::UNDEFINED_SUBROUTINE,
                         "G150: pocket subprogram not found");

    Lexer& lex = m_parser->getLexer();
    const size_t saved = lex.getPosition();
    lex.seek(m_parser->getLastBlockEnd());

    Block b;
    double cx = 0.0, cy = 0.0;
    bool have = false;
    while (true) {
        Error e = m_parser->parseNextBlock(b);
        if (!e.ok()) break;
        if (b.hasOCode && b.oCodeHasKeyword &&
            (b.oCodeType == OCodeType::ENDSUB ||
             b.oCodeType == OCodeType::RETURN))
            break;
        bool endOfSub = false;
        for (uint8_t i = 0; i < b.mCodeCount && !endOfSub; ++i)
            if (b.mCodes[i] == 99 || b.mCodes[i] == 30) endOfSub = true;
        if (endOfSub) break;

        bool arc = false, cw = false;
        for (uint8_t i = 0; i < b.gCodeCount; ++i) {
            int major = b.gCodes[i] / 10;
            if (b.gCodes[i] % 10 == 0) {
                if (major == 2) { arc = true; cw = true; }
                if (major == 3) { arc = true; cw = false; }
            }
        }
        const bool hasXY = b.hasWord(WordLetter::X) || b.hasWord(WordLetter::Y);
        if (!hasXY && !arc) continue;

        const double nx = b.hasWord(WordLetter::X)
            ? b.getWord(WordLetter::X) * unitScale : cx;
        const double ny = b.hasWord(WordLetter::Y)
            ? b.getWord(WordLetter::Y) * unitScale : cy;

        if (arc && have &&
            (b.hasWord(WordLetter::I) || b.hasWord(WordLetter::J))) {
            const double ccx = cx +
                (b.hasWord(WordLetter::I) ? b.getWord(WordLetter::I) * unitScale : 0.0);
            const double ccy = cy +
                (b.hasWord(WordLetter::J) ? b.getWord(WordLetter::J) * unitScale : 0.0);
            const double r = std::hypot(cx - ccx, cy - ccy);
            if (r > 1e-9) {
                const double a0 = std::atan2(cy - ccy, cx - ccx);
                const double a1 = std::atan2(ny - ccy, nx - ccx);
                double sweep = cw ? a0 - a1 : a1 - a0;
                while (sweep <= 0.0) sweep += 2.0 * M_PI;
                const int n = std::max(4, static_cast<int>(
                    std::ceil(sweep / (M_PI / 8.0))));
                for (int i = 1; i <= n; ++i) {
                    const double ang =
                        a0 + (cw ? -1.0 : 1.0) * sweep * i / n;
                    pts.emplace_back(ccx + r * std::cos(ang),
                                     ccy + r * std::sin(ang));
                }
            } else {
                pts.emplace_back(nx, ny);
            }
        } else if (hasXY || arc) {
            pts.emplace_back(nx, ny);
        }
        cx = nx;
        cy = ny;
        have = true;
    }

    lex.seek(saved);
    if (pts.size() < 3)
        return makeError(ErrorCode::INVALID_MOTION,
                         "G150: pocket boundary needs >= 3 points");
    return Error{};
}

Error Interpreter::dispatchG150(const Block& block,
                                std::vector<MotionSegment>& segments) {
    if (!block.hasWord(WordLetter::P))
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G150 requires P (pocket subprogram O-number)");
    const int32_t prog = static_cast<int32_t>(block.getWord(WordLetter::P));
    const double unitScale =
        (m_machineState.units == Units::INCH) ? 25.4 : 1.0;

    const double startZ = m_machineState.workPosition.z();
    const double finalZ = block.hasWord(WordLetter::Z)
        ? block.getWord(WordLetter::Z) * unitScale : startZ;
    const double clearZ = block.hasWord(WordLetter::R)
        ? block.getWord(WordLetter::R) * unitScale : startZ;
    const double stepZ = block.hasWord(WordLetter::Q)
        ? std::abs(block.getWord(WordLetter::Q)) * unitScale : 0.0;
    const double stepover = block.hasWord(WordLetter::J)
        ? std::abs(block.getWord(WordLetter::J)) * unitScale
        : (block.hasWord(WordLetter::I)
               ? std::abs(block.getWord(WordLetter::I)) * unitScale : 1.0);
    if (stepover <= 0.0)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G150 stepover must be positive");
    if (finalZ > startZ)
        return makeError(ErrorCode::INVALID_MOTION,
                         "G150 Z must be below the current Z level");

    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F) * unitScale;
    const double feed = m_machineState.feedRate;

    // Optional spindle start (S word on the G150 line).
    if (block.hasWord(WordLetter::S)) {
        double rpm = block.getWord(WordLetter::S);
        if (m_machineState.maxSpindleSpeed > 0.0)
            rpm = std::min(rpm, m_machineState.maxSpindleSpeed);
        m_machineState.spindleSpeed = rpm;
        m_machineState.spindleCW = true;
        m_machineState.spindleOn = true;
        if (m_spindleCallback) {
            Error err = m_spindleCallback(true, true, rpm);
            if (!err.ok()) return err;
        }
    }

    // Pocket boundary polygon (program coords), closed.
    std::vector<std::pair<double, double>> poly;
    Error err = collectPocketBoundary(prog, unitScale, poly);
    if (!err.ok()) return err;
    if (std::hypot(poly.front().first - poly.back().first,
                   poly.front().second - poly.back().second) > 1e-6)
        poly.push_back(poly.front());

    double ymin = poly[0].second, ymax = poly[0].second;
    for (const auto& p : poly) {
        ymin = std::min(ymin, p.second);
        ymax = std::max(ymax, p.second);
    }

    // Emission helper: program coords -> machine coords segment.
    auto emit = [&](double x, double y, double z,
                    MotionSegment::Type type, double fr) {
        Position target = m_machineState.workPosition;
        target.x() = x;
        target.y() = y;
        target.z() = z;
        MotionSegment seg;
        seg.type = type;
        seg.endPosition = m_coordinates.toMachineCoords(target);
        seg.feedRate = fr;
        seg.lineNumber = block.sourceLineNumber;
        segments.push_back(seg);
        m_machineState.workPosition = target;
        m_machineState.machinePosition = seg.endPosition;
        ++m_stats.motionSegments;
    };

    // Roughing passes: step Z down from startZ to finalZ.
    double z = startZ;
    while (true) {
        z = (stepZ > 0.0) ? std::max(z - stepZ, finalZ) : finalZ;

        // Raster scanlines in Y, clipped to the boundary (even-odd fill).
        bool forward = true;
        for (double y = ymin; y <= ymax + 1e-9; y += stepover) {
            std::vector<double> xs;
            for (size_t i = 0; i + 1 < poly.size(); ++i) {
                const double y1 = poly[i].second, y2 = poly[i + 1].second;
                if ((y1 <= y) != (y2 <= y)) {
                    const double t = (y - y1) / (y2 - y1);
                    xs.push_back(poly[i].first +
                                 t * (poly[i + 1].first - poly[i].first));
                }
            }
            std::sort(xs.begin(), xs.end());
            if (!forward) std::reverse(xs.begin(), xs.end());
            forward = !forward;

            for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                const double xa = xs[k], xb = xs[k + 1];
                // Position at clearance, plunge, cut the span, retract.
                emit(xa, y, clearZ, MotionSegment::Type::RAPID, 0.0);
                emit(xa, y, z, MotionSegment::Type::LINEAR, feed);
                emit(xb, y, z, MotionSegment::Type::LINEAR, feed);
                emit(xb, y, clearZ, MotionSegment::Type::RAPID, 0.0);
            }
        }
        if (z <= finalZ) break;
    }

    // Finish pass: trace the boundary contour at final depth.
    if (feed > 0.0) {
        emit(poly[0].first, poly[0].second, clearZ,
             MotionSegment::Type::RAPID, 0.0);
        emit(poly[0].first, poly[0].second, finalZ,
             MotionSegment::Type::LINEAR, feed);
        for (size_t i = 1; i < poly.size(); ++i)
            emit(poly[i].first, poly[i].second, finalZ,
                 MotionSegment::Type::LINEAR, feed);
        emit(poly.back().first, poly.back().second, clearZ,
             MotionSegment::Type::RAPID, 0.0);
    }

    // Final X/Y end position if given on the G150 line.
    if (block.hasWord(WordLetter::X) || block.hasWord(WordLetter::Y)) {
        emit(block.hasWord(WordLetter::X)
                 ? block.getWord(WordLetter::X) * unitScale
                 : m_machineState.workPosition.x(),
             block.hasWord(WordLetter::Y)
                 ? block.getWord(WordLetter::Y) * unitScale
                 : m_machineState.workPosition.y(),
             clearZ, MotionSegment::Type::RAPID, 0.0);
    }
    return Error{};
}

// ============================================================================
// Fanuc lathe cycles (G70/G71/G72/G73)
// ============================================================================

Error Interpreter::collectLatheContour(
        int32_t seqStart, int32_t seqEnd, double unitScale,
        std::vector<std::pair<double, double>>& pts) {
    Lexer& lex = m_parser->getLexer();
    const size_t saved = lex.getPosition();
    lex.seekToStart();

    Block b;
    bool inRange = false, found = false;
    double cx = 0.0, cz = 0.0;
    bool have = false;
    while (true) {
        Error e = m_parser->parseNextBlock(b);
        if (!e.ok()) break;
        if (!inRange) {
            if (b.lineNumber == seqStart) {
                inRange = true;
                found = true;
            } else {
                continue;
            }
        }

        bool hasXZ =
            b.hasWord(WordLetter::X) || b.hasWord(WordLetter::Z);
        bool arc = false, cw = false;
        for (uint8_t i = 0; i < b.gCodeCount; ++i) {
            const int g = b.gCodes[i] / 10;
            if (g == 2 || g == 3) { arc = true; cw = (g == 2); }
        }

        const double nx = b.hasWord(WordLetter::X)
            ? b.getWord(WordLetter::X) * unitScale : cx;
        const double nz = b.hasWord(WordLetter::Z)
            ? b.getWord(WordLetter::Z) * unitScale : cz;

        if (arc && have &&
            (b.hasWord(WordLetter::I) || b.hasWord(WordLetter::K))) {
            const double ccx = cx +
                (b.hasWord(WordLetter::I) ? b.getWord(WordLetter::I) * unitScale : 0.0);
            const double ccz = cz +
                (b.hasWord(WordLetter::K) ? b.getWord(WordLetter::K) * unitScale : 0.0);
            const double r = std::hypot(cx - ccx, cz - ccz);
            if (r > 1e-9) {
                const double a0 = std::atan2(cz - ccz, cx - ccx);
                const double a1 = std::atan2(nz - ccz, nx - ccx);
                double sweep = cw ? a0 - a1 : a1 - a0;
                while (sweep <= 0.0) sweep += 2.0 * M_PI;
                const int n = std::max(4, static_cast<int>(
                    std::ceil(sweep / (M_PI / 8.0))));
                for (int i = 1; i <= n; ++i) {
                    const double ang =
                        a0 + (cw ? -1.0 : 1.0) * sweep * i / n;
                    pts.emplace_back(ccx + r * std::cos(ang),
                                     ccz + r * std::sin(ang));
                }
            } else {
                pts.emplace_back(nx, nz);
            }
        } else if (hasXZ || arc) {
            pts.emplace_back(nx, nz);
        }
        cx = nx;
        cz = nz;
        have = true;

        if (b.lineNumber == seqEnd) break;
    }

    lex.seek(saved);
    if (!found || pts.size() < 2)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G70-73: contour block range P..Q not found");
    return Error{};
}

Error Interpreter::dispatchG70(const Block& block,
                               std::vector<MotionSegment>& segments) {
    if (!block.hasWord(WordLetter::P) || !block.hasWord(WordLetter::Q))
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G70 requires P/Q contour block range");
    const double unitScale =
        (m_machineState.units == Units::INCH) ? 25.4 : 1.0;
    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F) * unitScale;

    std::vector<std::pair<double, double>> pts;
    Error err = collectLatheContour(
        static_cast<int32_t>(block.getWord(WordLetter::P)),
        static_cast<int32_t>(block.getWord(WordLetter::Q)), unitScale, pts);
    if (!err.ok()) return err;

    const double feed = m_machineState.feedRate;
    const double zStart = m_machineState.workPosition.z();
    auto emit = [&](double x, double z, MotionSegment::Type type, double fr) {
        Position target = m_machineState.workPosition;
        target.x() = x;
        target.z() = z;
        MotionSegment seg;
        seg.type = type;
        seg.endPosition = m_coordinates.toMachineCoords(target);
        seg.feedRate = fr;
        seg.lineNumber = block.sourceLineNumber;
        segments.push_back(seg);
        m_machineState.workPosition = target;
        m_machineState.machinePosition = seg.endPosition;
        ++m_stats.motionSegments;
    };

    emit(pts[0].first, pts[0].second, MotionSegment::Type::RAPID, 0.0);
    for (size_t i = 1; i < pts.size(); ++i)
        emit(pts[i].first, pts[i].second, MotionSegment::Type::LINEAR, feed);
    emit(pts.back().first, zStart, MotionSegment::Type::RAPID, 0.0);
    return Error{};
}

// Roughing engine shared by G71 (turning: levels in X, cuts along Z) and
// G72 (facing: levels in Z, cuts along X). The material region is the
// contour closed by the approach-side edges; each level is a scanline
// clipped to the polygon by even-odd fill, cut from the approach side.
static Error latheRoughing(const Block& block,
                           std::vector<MotionSegment>& segments,
                           MachineState& state, CoordinateSystemManager& coords,
                           uint32_t& segCount, bool turning,
                           const std::vector<std::pair<double, double>>& contour,
                           double depthOfCut, double allowU, double allowW,
                           double feed, int lineNo) {
    // Working axes: u = level axis (X for G71, Z for G72),
    //               v = cut axis  (Z for G71, X for G72).
    auto cu = [&](const std::pair<double, double>& p) {
        return turning ? p.first : p.second;
    };
    auto cv = [&](const std::pair<double, double>& p) {
        return turning ? p.second : p.first;
    };
    const double uStart = turning ? state.workPosition.x()
                                  : state.workPosition.z();
    const double vStart = turning ? state.workPosition.z()
                                  : state.workPosition.x();

    // Polygon: contour + closing edge along u = uStart (stock boundary on
    // the approach side of the level axis).
    std::vector<std::pair<double, double>> poly;
    poly.reserve(contour.size() + 2);
    for (const auto& p : contour) poly.emplace_back(cu(p), cv(p));
    poly.emplace_back(uStart, cv(contour.back()));
    poly.emplace_back(uStart, cv(contour.front()));
    poly.push_back(poly.front());  // close the polygon

    double umin = poly[0].first, umax = poly[0].first;
    for (const auto& p : poly) {
        umin = std::min(umin, p.first);
        umax = std::max(umax, p.first);
    }
    // Cut direction: from uStart toward the polygon interior.
    const double dir = (uStart >= (umin + umax) * 0.5) ? -1.0 : 1.0;
    // Finish allowances shift the profile away from the cut direction.
    const double allowLevel = turning ? allowU : allowW;
    const double uLimit = (dir < 0.0) ? umin + allowLevel
                                      : umax - allowLevel;

    auto emit = [&](double u, double v, MotionSegment::Type type,
                    double fr) {
        Position target = state.workPosition;
        if (turning) { target.x() = u; target.z() = v; }
        else         { target.z() = u; target.x() = v; }
        MotionSegment seg;
        seg.type = type;
        seg.endPosition = coords.toMachineCoords(target);
        seg.feedRate = fr;
        seg.lineNumber = lineNo;
        segments.push_back(seg);
        state.workPosition = target;
        state.machinePosition = seg.endPosition;
        ++segCount;
    };

    emit(uStart, vStart, MotionSegment::Type::RAPID, 0.0);
    for (int level = 0; level < 10000; ++level) {
        const double u = uStart + dir * depthOfCut * (level + 1);
        if ((dir < 0.0 && u <= uLimit) || (dir > 0.0 && u >= uLimit))
            break;

        // Intersect the scanline u = const with the polygon (even-odd).
        std::vector<double> vs;
        for (size_t i = 0; i + 1 < poly.size(); ++i) {
            const double u1 = poly[i].first, u2 = poly[i + 1].first;
            if ((u1 <= u) != (u2 <= u))
                vs.push_back(poly[i].second +
                             (u - u1) * (poly[i + 1].second - poly[i].second)
                                 / (u2 - u1));
        }
        if (vs.empty()) break;
        std::sort(vs.begin(), vs.end());

        for (size_t k = 0; k + 1 < vs.size(); k += 2) {
            const double va = vs[k], vb = vs[k + 1];
            // Enter at the approach side, cut to the far end, retract.
            emit(u, vStart, MotionSegment::Type::RAPID, 0.0);
            const double vNear = std::abs(va - vStart) < std::abs(vb - vStart)
                                     ? va : vb;
            const double vFar = (vNear == va) ? vb : va;
            emit(u, vFar, MotionSegment::Type::LINEAR, feed);
            emit(u, vStart, MotionSegment::Type::RAPID, 0.0);
        }
    }
    return Error{};
}

Error Interpreter::dispatchG71(const Block& block,
                               std::vector<MotionSegment>& segments) {
    return dispatchG71_G72Impl(block, segments, true);
}

Error Interpreter::dispatchG72(const Block& block,
                               std::vector<MotionSegment>& segments) {
    return dispatchG71_G72Impl(block, segments, false);
}

Error Interpreter::dispatchG71_G72Impl(
        const Block& block, std::vector<MotionSegment>& segments,
        bool turning) {
    if (!block.hasWord(WordLetter::P) || !block.hasWord(WordLetter::Q))
        return makeError(ErrorCode::PARAMETER_ERROR,
                         turning ? "G71 requires P/Q contour block range"
                                 : "G72 requires P/Q contour block range");
    const double unitScale =
        (m_machineState.units == Units::INCH) ? 25.4 : 1.0;
    const double depth = block.hasWord(WordLetter::D)
        ? std::abs(block.getWord(WordLetter::D)) * unitScale : 1.0;
    if (depth <= 0.0)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G71/G72 depth of cut must be positive");
    const double allowU = block.hasWord(WordLetter::U)
        ? block.getWord(WordLetter::U) * unitScale : 0.0;
    const double allowW = block.hasWord(WordLetter::W)
        ? block.getWord(WordLetter::W) * unitScale : 0.0;

    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F) * unitScale;
    if (block.hasWord(WordLetter::S)) {
        double rpm = block.getWord(WordLetter::S);
        if (m_machineState.maxSpindleSpeed > 0.0)
            rpm = std::min(rpm, m_machineState.maxSpindleSpeed);
        m_machineState.spindleSpeed = rpm;
        m_machineState.spindleCW = true;
        m_machineState.spindleOn = true;
        if (m_spindleCallback) {
            Error e = m_spindleCallback(true, true, rpm);
            if (!e.ok()) return e;
        }
    }

    std::vector<std::pair<double, double>> pts;
    Error err = collectLatheContour(
        static_cast<int32_t>(block.getWord(WordLetter::P)),
        static_cast<int32_t>(block.getWord(WordLetter::Q)), unitScale, pts);
    if (!err.ok()) return err;

    return latheRoughing(block, segments, m_machineState, m_coordinates,
                         m_stats.motionSegments, turning, pts, depth,
                         allowU, allowW, m_machineState.feedRate,
                         block.sourceLineNumber);
}

Error Interpreter::dispatchG73Lathe(const Block& block,
                                    std::vector<MotionSegment>& segments) {
    const double unitScale =
        (m_machineState.units == Units::INCH) ? 25.4 : 1.0;
    const double allowU = block.hasWord(WordLetter::U)
        ? block.getWord(WordLetter::U) * unitScale : 0.0;
    const double allowW = block.hasWord(WordLetter::W)
        ? block.getWord(WordLetter::W) * unitScale : 0.0;
    const int divs = block.hasWord(WordLetter::R)
        ? std::max(1, static_cast<int>(block.getWord(WordLetter::R))) : 1;
    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F) * unitScale;
    const double feed = m_machineState.feedRate;

    std::vector<std::pair<double, double>> pts;
    Error err = collectLatheContour(
        static_cast<int32_t>(block.getWord(WordLetter::P)),
        static_cast<int32_t>(block.getWord(WordLetter::Q)), unitScale, pts);
    if (!err.ok()) return err;

    const double x0 = m_machineState.workPosition.x();
    const double z0 = m_machineState.workPosition.z();
    auto emit = [&](double x, double z, MotionSegment::Type type,
                    double fr) {
        Position target = m_machineState.workPosition;
        target.x() = x;
        target.z() = z;
        MotionSegment seg;
        seg.type = type;
        seg.endPosition = m_coordinates.toMachineCoords(target);
        seg.feedRate = fr;
        seg.lineNumber = block.sourceLineNumber;
        segments.push_back(seg);
        m_machineState.workPosition = target;
        m_machineState.machinePosition = seg.endPosition;
        ++m_stats.motionSegments;
    };

    // Pass i traces the contour offset by the remaining relief share
    // (Fanuc: total U/W relief divided evenly across R passes).
    for (int i = 0; i < divs; ++i) {
        const double s = static_cast<double>(divs - 1 - i) / divs;
        const double ox = allowU * s, oz = allowW * s;
        emit(pts[0].first + ox, pts[0].second + oz,
             MotionSegment::Type::RAPID, 0.0);
        for (size_t k = 1; k < pts.size(); ++k)
            emit(pts[k].first + ox, pts[k].second + oz,
                 MotionSegment::Type::LINEAR, feed);
        emit(x0, z0, MotionSegment::Type::RAPID, 0.0);
    }
    return Error{};
}

// ============================================================================
// Spindle-synchronized moves (G33 threading, G33.1 rigid tapping)
// ============================================================================

Error Interpreter::executeThreading(const Block& block,
                                    const Position& target,
                                    double unitScale,
                                    std::vector<MotionSegment>& segments) {
    // K = pitch along Z; I = pitch along X (tapered threads).
    double pitch = 0.0;
    if (block.hasWord(WordLetter::K))
        pitch = block.getWord(WordLetter::K) * unitScale;
    else if (block.hasWord(WordLetter::I))
        pitch = block.getWord(WordLetter::I) * unitScale;
    if (pitch == 0.0)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G33 requires a thread pitch (K, or I for tapered)");
    if (!m_machineState.spindleOn || m_machineState.spindleSpeed <= 0.0)
        return makeError(ErrorCode::INVALID_MOTION,
                         "G33 requires the spindle to be running");

    MotionSegment seg;
    seg.type = MotionSegment::Type::THREADING;
    seg.endPosition = m_coordinates.toMachineCoords(target);
    seg.pitch = pitch;
    seg.feedRate = std::abs(pitch) * m_machineState.spindleSpeed;
    seg.lineNumber = block.sourceLineNumber;
    segments.push_back(seg);
    m_machineState.workPosition = target;
    m_machineState.machinePosition = seg.endPosition;
    ++m_stats.motionSegments;
    return Error{};
}

Error Interpreter::dispatchG76Threading(const Block& block,
                                        const Position& target,
                                        double unitScale,
                                        std::vector<MotionSegment>& segments) {
    // Single-line Fanuc form: X/Z thread end (X travel sets the total
    // depth), I taper, D first-cut depth, F pitch, Q minimum pass depth.
    if (!block.hasWord(WordLetter::F))
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G76 requires a thread pitch (F)");
    const double pitch = block.getWord(WordLetter::F) * unitScale;
    if (pitch == 0.0)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G76 thread pitch must be non-zero");
    if (!m_machineState.spindleOn || m_machineState.spindleSpeed <= 0.0)
        return makeError(ErrorCode::INVALID_MOTION,
                         "G76 requires the spindle to be running");

    const Position start = m_machineState.workPosition;
    const double totalDepth = std::abs(target.x() - start.x());
    if (totalDepth < 1e-9)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G76 thread depth (X travel) must be non-zero");
    const double dirX = (target.x() < start.x()) ? -1.0 : 1.0;
    const double taper = block.hasWord(WordLetter::I)
        ? block.getWord(WordLetter::I) * unitScale : 0.0;
    double firstCut = block.hasWord(WordLetter::D)
        ? std::abs(block.getWord(WordLetter::D)) * unitScale : totalDepth;
    const double minCut = block.hasWord(WordLetter::Q)
        ? std::abs(block.getWord(WordLetter::Q)) * unitScale : 0.0;
    if (firstCut <= 0.0) firstCut = totalDepth;

    const double feed = pitch * m_machineState.spindleSpeed;
    auto emit = [&](double x, double z, MotionSegment::Type type,
                    double fr, double pc = 0.0) {
        Position t = start;
        t.x() = x;
        t.z() = z;
        MotionSegment seg;
        seg.type = type;
        seg.endPosition = m_coordinates.toMachineCoords(t);
        seg.feedRate = fr;
        seg.pitch = pc;
        seg.lineNumber = block.sourceLineNumber;
        segments.push_back(seg);
        m_machineState.workPosition = t;
        m_machineState.machinePosition = seg.endPosition;
        ++m_stats.motionSegments;
    };

    // Pass depths follow the constant-area rule: cumulative depth
    // d_i = D*sqrt(i), pass increment clamped to >= Q, until full depth.
    double prev = 0.0;
    for (int i = 1; i <= 1000; ++i) {
        double d = firstCut * std::sqrt(static_cast<double>(i));
        if (minCut > 0.0 && d - prev < minCut) d = prev + minCut;
        if (d >= totalDepth) d = totalDepth;

        const double x = start.x() + dirX * d;
        // Infeed to this pass depth, synchronized cut along Z (I tapers
        // the end radius), retract in X, return in Z.
        emit(x, start.z(), MotionSegment::Type::RAPID, 0.0);
        emit(x + taper, target.z(), MotionSegment::Type::THREADING,
             feed, pitch);
        emit(start.x(), target.z(), MotionSegment::Type::RAPID, 0.0);
        emit(start.x(), start.z(), MotionSegment::Type::RAPID, 0.0);

        if (d >= totalDepth) break;
        prev = d;
    }
    return Error{};
}

Error Interpreter::executeRigidTap(const Block& block,
                                   const Position& target,
                                   double unitScale,
                                   std::vector<MotionSegment>& segments) {
    if (!block.hasWord(WordLetter::K))
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G33.1 requires a thread pitch (K)");
    const double pitch = block.getWord(WordLetter::K) * unitScale;
    if (pitch == 0.0)
        return makeError(ErrorCode::PARAMETER_ERROR,
                         "G33.1 thread pitch must be non-zero");
    if (!m_machineState.spindleOn || m_machineState.spindleSpeed <= 0.0)
        return makeError(ErrorCode::INVALID_MOTION,
                         "G33.1 requires the spindle to be running");

    const Position start = m_machineState.workPosition;
    const double feed = std::abs(pitch) * m_machineState.spindleSpeed;
    auto emit = [&](const Position& p, double pc) {
        MotionSegment seg;
        seg.type = MotionSegment::Type::THREADING;
        seg.endPosition = m_coordinates.toMachineCoords(p);
        seg.pitch = pc;
        seg.feedRate = feed;
        seg.lineNumber = block.sourceLineNumber;
        segments.push_back(seg);
        m_machineState.workPosition = p;
        m_machineState.machinePosition = seg.endPosition;
        ++m_stats.motionSegments;
    };

    // Tap down at pitch, spindle reverses, retract at pitch back to start.
    emit(target, pitch);
    emit(start, -pitch);
    return Error{};
}

// ============================================================================
// Motion Handling
// ============================================================================

Error Interpreter::handleMotion(const Block& block,
                                 std::vector<MotionSegment>& segments) {
    // Determine the active motion mode
    MotionMode mode = m_machineState.motionMode;

    // Check for explicit motion G-code in this block
    for (uint8_t i = 0; i < block.gCodeCount; ++i) {
        // Decode: G-codes are encoded as major*10+minor.
        int encoded = block.gCodes[i];
        ModalGroup group = getModalGroup(encoded);
        if (group == ModalGroup::MOTION) {
            int major = encoded / 10;
            int minor = encoded % 10;
            int gnum = (minor > 0) ? encoded : major;
            // Map G-code to motion mode
            switch (gnum) {
                case 0: mode = MotionMode::RAPID; break;
                case 1: mode = MotionMode::LINEAR; break;
                case 2: mode = MotionMode::CW_ARC; break;
                case 3: mode = MotionMode::CCW_ARC; break;
                case 4: mode = MotionMode::DWELL; break;
                case 5: mode = MotionMode::CUBIC_SPLINE; break;
                case 51: mode = MotionMode::QUADRATIC_SPLINE; break;
                case 52:
                case 53: mode = MotionMode::NURBS; break;
                case 33: mode = MotionMode::THREADING; break;
                case 331: mode = MotionMode::RIGID_TAP; break;
                case 382: mode = MotionMode::PROBE_TOWARD; break;
                case 383: mode = MotionMode::PROBE_TOWARD_NE; break;
                case 384: mode = MotionMode::PROBE_AWAY; break;
                case 385: mode = MotionMode::PROBE_AWAY_NE; break;
                case 73: mode = MotionMode::DRILL_PECK_BREAK; break;
                case 74: mode = MotionMode::TAP_LH; break;
                case 76: mode = MotionMode::THREAD_CYCLE; break;
                case 80: mode = MotionMode::CANNED_OFF; break;
                case 81: mode = MotionMode::DRILL; break;
                case 82: mode = MotionMode::DRILL_DWELL; break;
                case 83: mode = MotionMode::DRILL_PECK; break;
                case 84: mode = MotionMode::TAP_RH; break;
                case 85: mode = MotionMode::BORE_FEED_OUT; break;
                case 86: mode = MotionMode::BORE_STOP_RAPID; break;
                case 87: mode = MotionMode::BORE_BACK; break;
                case 88: mode = MotionMode::BORE_MANUAL; break;
                case 89: mode = MotionMode::BORE_DWELL; break;
                default: break;
            }
            m_machineState.motionMode = mode;
            break;
        }
    }

    // Fanuc lathe G73 (pattern repeat) shares the code number with the
    // RS274 peck-drill cycle — disambiguate by the P/Q contour range.
    if (mode == MotionMode::DRILL_PECK_BREAK &&
        block.hasWord(WordLetter::P) && block.hasWord(WordLetter::Q))
        return dispatchG73Lathe(block, segments);

    // Update feed rate if F word is present
    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F);

    // Canned/probe modes act on R/Z/Q/P words, not just axis words — they
    // are handled below even when no axis words are present.
    const bool cycleOrProbe = isCannedCycle(mode) || isProbeMode(mode) ||
                              mode == MotionMode::CANNED_OFF ||
                              mode == MotionMode::NURBS;

    // If no motion words are present, don't generate a segment.
    // The G-code only updates the modal motion mode and/or feed rate.
    if (!cycleOrProbe &&
        !hasMotionWords(block) &&
        !block.hasWord(WordLetter::I) &&
        !block.hasWord(WordLetter::J) &&
        !block.hasWord(WordLetter::K) &&
        !block.hasWord(WordLetter::R))
        return Error{};

    // Compute target position in program coordinates
    Position target = m_machineState.workPosition;
    double unitScale = (m_machineState.units == Units::INCH) ? 25.4 : 1.0;

    if (m_machineState.distanceMode == DistanceMode::ABSOLUTE) {
        if (block.hasWord(WordLetter::X))
            target.x() = block.getWord(WordLetter::X) * unitScale;
        if (block.hasWord(WordLetter::Y))
            target.y() = block.getWord(WordLetter::Y) * unitScale;
        if (block.hasWord(WordLetter::Z))
            target.z() = block.getWord(WordLetter::Z) * unitScale;
        if (block.hasWord(WordLetter::A))
            target[Axis::A] = block.getWord(WordLetter::A);
        if (block.hasWord(WordLetter::B))
            target[Axis::B] = block.getWord(WordLetter::B);
        if (block.hasWord(WordLetter::C))
            target[Axis::C] = block.getWord(WordLetter::C);
    } else {
        // Incremental mode
        if (block.hasWord(WordLetter::X))
            target.x() += block.getWord(WordLetter::X) * unitScale;
        if (block.hasWord(WordLetter::Y))
            target.y() += block.getWord(WordLetter::Y) * unitScale;
        if (block.hasWord(WordLetter::Z))
            target.z() += block.getWord(WordLetter::Z) * unitScale;
        if (block.hasWord(WordLetter::A))
            target[Axis::A] += block.getWord(WordLetter::A);
        if (block.hasWord(WordLetter::B))
            target[Axis::B] += block.getWord(WordLetter::B);
        if (block.hasWord(WordLetter::C))
            target[Axis::C] += block.getWord(WordLetter::C);
    }

    // Canned cycle cancel: G80 clears the modal cycle, emits no motion.
    if (mode == MotionMode::CANNED_OFF) {
        m_cannedActive = false;
        m_cannedParams = CannedParams{};
        // Feature::G80_CANCEL_LEVELING — Marlin/RepRap also cancels bed
        // leveling / mesh compensation.
        if (featureEnabled(Feature::G80_CANCEL_LEVELING))
            m_marlinState.bedLevelingEnabled = false;
        return Error{};
    }

    // Probe moves (G38.2–G38.5)
    if (isProbeMode(mode)) {
        return executeProbe(mode, block, target, unitScale, segments);
    }

    // Feature::G76_LATHE_THREADING — Fanuc threading cycle replaces the
    // RS274 fine-boring canned cycle.
    if (mode == MotionMode::THREAD_CYCLE &&
        featureEnabled(Feature::G76_LATHE_THREADING))
        return dispatchG76Threading(block, target, unitScale, segments);

    // Canned cycles (G73/G74/G76/G81–G89), including modal repeats
    if (isCannedCycle(mode)) {
        return executeCannedCycle(mode, block, target, unitScale, segments);
    }

    // Splines: G5 cubic, G5.1 quadratic, G5.2/G5.3 NURBS
    if (mode == MotionMode::CUBIC_SPLINE ||
        mode == MotionMode::QUADRATIC_SPLINE) {
        return executeSpline(mode, block, target, unitScale, segments);
    }
    if (mode == MotionMode::NURBS) {
        // Determine if this block is a G5.2/G5.3 command or a bare
        // control-point block.
        int nurbsG = 0;
        for (uint8_t i = 0; i < block.gCodeCount; ++i) {
            if (block.gCodes[i] == 52 || block.gCodes[i] == 53)
                nurbsG = block.gCodes[i];
        }
        return executeNurbs(nurbsG, block, target, unitScale, segments);
    }

    // Any non-NURBS motion abandons an unfinished NURBS block.
    m_nurbsActive = false;

    // Spindle-synchronized moves (G33 threading, G33.1 rigid tap)
    if (mode == MotionMode::THREADING)
        return executeThreading(block, target, unitScale, segments);
    if (mode == MotionMode::RIGID_TAP)
        return executeRigidTap(block, target, unitScale, segments);

    // Create motion segment
    MotionSegment seg;
    seg.lineNumber = block.sourceLineNumber;
    seg.feedRate = m_machineState.feedRate;

    // Handle arc moves (G2/G3) by decomposing into line segments
    if (mode == MotionMode::CW_ARC || mode == MotionMode::CCW_ARC) {
        return handleArc(block, target, mode, unitScale, segments);
    }

    switch (mode) {
        case MotionMode::RAPID:
            seg.type = MotionSegment::Type::RAPID;
            break;
        case MotionMode::LINEAR:
            seg.type = MotionSegment::Type::LINEAR;
            break;
        default:
            seg.type = MotionSegment::Type::LINEAR;
            break;
    }

    // Transform target from program coordinates to machine coordinates
    seg.endPosition = m_coordinates.toMachineCoords(target);

    // Update machine state positions
    m_machineState.workPosition = target;
    m_machineState.machinePosition = seg.endPosition;

    segments.push_back(seg);
    m_stats.motionSegments++;
    return Error{};
}

Error Interpreter::handleArc(const Block& block, const Position& target,
                              MotionMode mode, double unitScale,
                              std::vector<MotionSegment>& segments) {
    // Decompose arc into line segments.
    // Arc center is defined by I/J/K (offsets from start) or R (radius).
    Position start = m_machineState.workPosition;
    Position end = target;

    // Determine the active plane axes
    // G17 (XY): axis1=X, axis2=Y, helical=Z
    // G18 (ZX): axis1=Z, axis2=X, helical=Y
    // G19 (YZ): axis1=Y, axis2=Z, helical=X
    int a1, a2, ah;
    switch (m_machineState.plane) {
        case Plane::ZX: a1 = 2; a2 = 0; ah = 1; break; // ZX plane
        case Plane::YZ: a1 = 1; a2 = 2; ah = 0; break; // YZ plane
        default:        a1 = 0; a2 = 1; ah = 2; break; // XY plane
    }

    // Compute arc center
    double center1, center2, radius;
    bool haveCenter = false;

    // Get I/J/K offsets for the active plane
    // G17: I=X offset, J=Y offset
    // G18: K=Z offset, I=X offset
    // G19: J=Y offset, K=Z offset
    double offset1 = 0, offset2 = 0;
    bool hasOffset1 = false, hasOffset2 = false;

    if (m_machineState.plane == Plane::XY) {
        if (block.hasWord(WordLetter::I)) { offset1 = block.getWord(WordLetter::I) * unitScale; hasOffset1 = true; }
        if (block.hasWord(WordLetter::J)) { offset2 = block.getWord(WordLetter::J) * unitScale; hasOffset2 = true; }
    } else if (m_machineState.plane == Plane::ZX) {
        if (block.hasWord(WordLetter::K)) { offset1 = block.getWord(WordLetter::K) * unitScale; hasOffset1 = true; }
        if (block.hasWord(WordLetter::I)) { offset2 = block.getWord(WordLetter::I) * unitScale; hasOffset2 = true; }
    } else { // YZ
        if (block.hasWord(WordLetter::J)) { offset1 = block.getWord(WordLetter::J) * unitScale; hasOffset1 = true; }
        if (block.hasWord(WordLetter::K)) { offset2 = block.getWord(WordLetter::K) * unitScale; hasOffset2 = true; }
    }

    if (hasOffset1 || hasOffset2) {
        // I/J/K mode: center = start + offset
        center1 = start[a1] + offset1;
        center2 = start[a2] + offset2;
        radius = std::sqrt(offset1 * offset1 + offset2 * offset2);
        haveCenter = true;
    } else if (block.hasWord(WordLetter::R)) {
        // R mode: compute center from radius
        radius = std::abs(block.getWord(WordLetter::R) * unitScale);
        double r = block.getWord(WordLetter::R) * unitScale;
        // Midpoint between start and end in the plane
        double mid1 = (start[a1] + end[a1]) / 2.0;
        double mid2 = (start[a2] + end[a2]) / 2.0;
        // Distance from start to end in plane
        double d1 = end[a1] - start[a1];
        double d2 = end[a2] - start[a2];
        double dist = std::sqrt(d1 * d1 + d2 * d2);
        if (dist < 1e-12 || radius < dist / 2.0) {
            return makeError(ErrorCode::INVALID_MOTION, "Arc radius too small");
        }
        // Perpendicular distance from midpoint to center
        double h = std::sqrt(radius * radius - (dist / 2.0) * (dist / 2.0));
        // Perpendicular direction (normalized)
        double perp1 = -d2 / dist;
        double perp2 = d1 / dist;
        // For CW (G2) with positive R, center is to the right of start->end
        // For CCW (G3) with positive R, center is to the left
        // Negative R flips the side (major arc)
        int sign = (mode == MotionMode::CW_ARC) ? -1 : 1;
        if (r < 0) sign = -sign; // Negative R = major arc
        center1 = mid1 + sign * perp1 * h;
        center2 = mid2 + sign * perp2 * h;
        haveCenter = true;
    }

    if (!haveCenter)
        return makeError(ErrorCode::INVALID_MOTION, "No arc center specified");

    // Compute start and end angles
    double startAngle = std::atan2(start[a2] - center2, start[a1] - center1);
    double endAngle = std::atan2(end[a2] - center2, end[a1] - center1);

    // Compute sweep angle
    double sweep;
    if (mode == MotionMode::CW_ARC) {
        // CW: angle decreases
        sweep = startAngle - endAngle;
        while (sweep <= 0) sweep += 2.0 * M_PI;
        sweep = -sweep; // Negative for CW
    } else {
        // CCW: angle increases
        sweep = endAngle - startAngle;
        while (sweep <= 0) sweep += 2.0 * M_PI;
    }

    // Handle full circle (start == end in plane, with I/J/K)
    double planeDist = std::sqrt(
        (end[a1] - start[a1]) * (end[a1] - start[a1]) +
        (end[a2] - start[a2]) * (end[a2] - start[a2]));
    if (planeDist < 1e-12 && (hasOffset1 || hasOffset2)) {
        // Full circle
        sweep = (mode == MotionMode::CW_ARC) ? -2.0 * M_PI : 2.0 * M_PI;
    }

    // Programmable mirror (G51.1): reflection flips handedness, so an odd
    // number of mirrored in-plane axes swaps CW<->CCW.
    const bool mirrorFlip =
        m_machineState.axisMirror[a1] != m_machineState.axisMirror[a2];

    // When emit-arc-segments mode is active, emit a single arc MotionSegment
    // with full ArcParams instead of tessellating into line segments.
    if (m_emitArcSegments) {
        const bool cw = (mode == MotionMode::CW_ARC) != mirrorFlip;
        MotionSegment seg;
        seg.type = cw ? MotionSegment::Type::ARC_CW
                      : MotionSegment::Type::ARC_CCW;
        seg.endPosition = m_coordinates.toMachineCoords(end);
        seg.feedRate = m_machineState.feedRate;
        seg.lineNumber = block.sourceLineNumber;

        // centerOffset is relative to start (in program coordinates)
        Position centerPos = start;
        centerPos[a1] = center1;
        centerPos[a2] = center2;
        seg.centerOffset = centerPos - start;
        // Mirror negates the offset component along mirrored axes.
        if (m_machineState.axisMirror[a1])
            seg.centerOffset[a1] = -seg.centerOffset[a1];
        if (m_machineState.axisMirror[a2])
            seg.centerOffset[a2] = -seg.centerOffset[a2];

        // Populate ArcParams
        seg.arc.center = centerPos;
        seg.arc.startPoint = start;
        seg.arc.endPoint = end;
        seg.arc.radius = radius;
        seg.arc.startAngle = startAngle;
        seg.arc.endAngle = endAngle;
        seg.arc.sweepAngle = mirrorFlip ? -sweep : sweep;
        seg.arc.clockwise = cw;
        seg.arc.plane = m_machineState.plane;
        seg.arc.helixDelta = end[ah] - start[ah];
        seg.arc.valid = true;

        segments.push_back(seg);

        // Update machine state positions
        m_machineState.workPosition = end;
        m_machineState.machinePosition = m_coordinates.toMachineCoords(end);
        m_stats.motionSegments++;
        return Error{};
    }

    // Number of segments: aim for ~1mm chord deviation
    // chord deviation d = r * (1 - cos(theta/2))
    // theta = 2 * acos(1 - d/r)
    // numSegments = ceil(|sweep| / theta)
    int numSegments = 16; // Default
    if (radius > 0.001) {
        double maxDeviation = 0.1; // 0.1mm max chord deviation
        double maxAngle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - maxDeviation / radius)));
        numSegments = static_cast<int>(std::ceil(std::abs(sweep) / maxAngle));
        numSegments = std::max(numSegments, 4);
        numSegments = std::min(numSegments, 1000);
    }

    // Helical interpolation: Z (or helical axis) varies linearly
    double helicalStart = start[ah];
    double helicalEnd = end[ah];

    // Generate line segments
    for (int i = 1; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        double angle = startAngle + sweep * t;

        Position segEnd = start; // Copy non-plane axes
        segEnd[a1] = center1 + radius * std::cos(angle);
        segEnd[a2] = center2 + radius * std::sin(angle);
        segEnd[ah] = helicalStart + (helicalEnd - helicalStart) * t;

        MotionSegment seg;
        seg.type = MotionSegment::Type::LINEAR;
        seg.lineNumber = block.sourceLineNumber;
        seg.feedRate = m_machineState.feedRate;
        seg.endPosition = m_coordinates.toMachineCoords(segEnd);

        segments.push_back(seg);
    }

    // Update machine state positions
    m_machineState.workPosition = end;
    m_machineState.machinePosition = m_coordinates.toMachineCoords(end);
    m_stats.motionSegments += numSegments;
    return Error{};
}

// ============================================================================
// Output
// ============================================================================

Error Interpreter::outputSegments(const std::vector<MotionSegment>& segments) {
    if (m_dryRun)
        return Error{}; // No output in dry run mode

    for (const auto& seg : segments) {
        if (m_motionCallback) {
            Error err = m_motionCallback(seg);
            if (!err.ok())
                return err;
        }
    }
    return Error{};
}

// ============================================================================
// Modal State Update
// ============================================================================

void Interpreter::updateModalState(const Block& block) {
    // Update line number
    if (block.lineNumber >= 0)
        m_machineState.lineNumber = block.lineNumber;

    // Update feed rate
    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F);

    // Update spindle speed
    if (block.hasWord(WordLetter::S))
        m_machineState.spindleSpeed = block.getWord(WordLetter::S);

    // Update tool number
    if (block.hasWord(WordLetter::T))
        m_machineState.selectedTool = static_cast<int>(block.getWord(WordLetter::T));

    // G-code modal state is updated in dispatchGCode
    m_machineState.blockCount++;
}

// ============================================================================
// Position Variables
// ============================================================================

void Interpreter::updatePositionVariables() {
    // Current position: #5420-#5428
    for (size_t i = 0; i < MAX_AXES; ++i)
        m_variables.set(CURRENT_POS_PARAM_BASE + static_cast<int32_t>(i),
                        m_machineState.workPosition[i]);
}

// ============================================================================
// Canned Cycles (G73/G74/G76/G80–G89)
// ============================================================================

bool Interpreter::isCannedCycle(MotionMode m) {
    switch (m) {
        case MotionMode::DRILL_PECK_BREAK:
        case MotionMode::TAP_LH:
        case MotionMode::THREAD_CYCLE:
        case MotionMode::DRILL:
        case MotionMode::DRILL_DWELL:
        case MotionMode::DRILL_PECK:
        case MotionMode::TAP_RH:
        case MotionMode::BORE_FEED_OUT:
        case MotionMode::BORE_STOP_RAPID:
        case MotionMode::BORE_BACK:
        case MotionMode::BORE_MANUAL:
        case MotionMode::BORE_DWELL:
            return true;
        default:
            return false;
    }
}

bool Interpreter::isProbeMode(MotionMode m) {
    return m == MotionMode::PROBE_TOWARD || m == MotionMode::PROBE_TOWARD_NE ||
           m == MotionMode::PROBE_AWAY || m == MotionMode::PROBE_AWAY_NE;
}

Error Interpreter::executeCannedCycle(MotionMode cycle, const Block& block,
                                      const Position& target, double unitScale,
                                      std::vector<MotionSegment>& segments) {
    // Plane axes: a1/a2 = hole position, ah = drill axis.
    int a1, a2, ah;
    switch (m_machineState.plane) {
        case Plane::ZX: a1 = 2; a2 = 0; ah = 1; break;
        case Plane::YZ: a1 = 1; a2 = 2; ah = 0; break;
        default:        a1 = 0; a2 = 1; ah = 2; break;
    }
    static const WordLetter axisWord[3] = {
        WordLetter::X, WordLetter::Y, WordLetter::Z};

    // Merge words into modal cycle parameters (missing words keep the
    // previous values so bare `X20 Y30` lines repeat the cycle).
    CannedParams& p = m_cannedParams;
    if (block.hasWord(axisWord[ah])) {
        p.z = target[ah];
        p.zSet = true;
    }
    const double initialAH = m_machineState.workPosition[ah];
    if (block.hasWord(WordLetter::R)) {
        p.r = block.getWord(WordLetter::R) * unitScale;
        if (m_machineState.distanceMode == DistanceMode::INCREMENTAL)
            p.r += initialAH;
        p.rSet = true;
    }
    if (block.hasWord(WordLetter::Q)) {
        p.q = std::fabs(block.getWord(WordLetter::Q)) * unitScale;
    }
    if (block.hasWord(WordLetter::P)) {
        p.dwell = block.getWord(WordLetter::P);  // seconds (LinuxCNC)
    }
    if (block.hasWord(WordLetter::I))
        p.shiftI = block.getWord(WordLetter::I) * unitScale;
    if (block.hasWord(WordLetter::J))
        p.shiftJ = block.getWord(WordLetter::J) * unitScale;
    p.repeat = static_cast<int32_t>(block.getWord(WordLetter::L, 1));
    if (p.repeat < 1) p.repeat = 1;

    if (!p.zSet) {
        return makeError(ErrorCode::INVALID_MOTION,
                         "Canned cycle requires a depth word (Z)");
    }
    // Default R plane: current drill-axis position.
    if (!p.rSet)
        p.r = initialAH;

    m_cannedActive = true;

    const double retractAH =
        (m_machineState.cannedReturn == CannedReturnMode::INITIAL)
            ? initialAH : p.r;
    const double clearance = 0.5 * unitScale;  // peck re-approach clearance
    const double feed = m_machineState.feedRate;

    // Emit helpers — positions are in work coordinates; each segment
    // transforms to machine coords and advances machine state.
    auto emitMove = [&](MotionSegment::Type type, const Position& prog,
                        double fr) {
        MotionSegment s;
        s.type = type;
        s.endPosition = m_coordinates.toMachineCoords(prog);
        s.feedRate = fr;
        s.lineNumber = block.sourceLineNumber;
        segments.push_back(s);
        m_machineState.workPosition = prog;
        m_machineState.machinePosition = s.endPosition;
        ++m_stats.motionSegments;
    };
    auto emitDwell = [&](double seconds) {
        MotionSegment s;
        s.type = MotionSegment::Type::DWELL;
        s.duration = seconds;
        s.endPosition = m_coordinates.toMachineCoords(m_machineState.workPosition);
        s.lineNumber = block.sourceLineNumber;
        segments.push_back(s);
        if (m_dwellCallback) m_dwellCallback(seconds);
    };
    auto spindle = [&](bool on, bool cw) {
        if (m_spindleCallback)
            m_spindleCallback(on, cw, m_machineState.spindleSpeed);
        m_machineState.spindleOn = on;
        m_machineState.spindleCW = cw;
    };

    for (int32_t rep = 0; rep < p.repeat; ++rep) {
        Position pos = m_machineState.workPosition;
        // 1. Rapid to hole XY
        pos[a1] = target[a1];
        pos[a2] = target[a2];
        emitMove(MotionSegment::Type::RAPID, pos, 0.0);
        // 2. Rapid to R plane
        pos[ah] = p.r;
        emitMove(MotionSegment::Type::RAPID, pos, 0.0);

        switch (cycle) {
            case MotionMode::DRILL:  // G81
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                break;

            case MotionMode::DRILL_DWELL:  // G82
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                if (p.dwell > 0.0) emitDwell(p.dwell);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                break;

            case MotionMode::DRILL_PECK: {  // G83 — full retract per peck
                double depth = p.r;
                const double step = (p.q > 0.0) ? p.q
                                                : std::fabs(p.z - p.r);
                while (depth - step > p.z) {
                    depth -= step;
                    pos[ah] = depth;
                    emitMove(MotionSegment::Type::LINEAR, pos, feed);
                    pos[ah] = p.r;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                    pos[ah] = depth + clearance;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                }
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                break;
            }

            case MotionMode::DRILL_PECK_BREAK: {  // G73 — chip break
                double depth = p.r;
                const double step = (p.q > 0.0) ? p.q
                                                : std::fabs(p.z - p.r);
                while (depth - step > p.z) {
                    depth -= step;
                    pos[ah] = depth;
                    emitMove(MotionSegment::Type::LINEAR, pos, feed);
                    pos[ah] = depth + clearance;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                    pos[ah] = depth;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                }
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                break;
            }

            case MotionMode::TAP_RH:   // G84 — feed in CW, reverse out
            case MotionMode::TAP_LH: { // G74 — feed in CCW, reverse out
                const bool inCW = (cycle == MotionMode::TAP_RH);
                spindle(true, inCW);
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                spindle(true, !inCW);
                pos[ah] = p.r;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                spindle(true, inCW);
                if (retractAH > p.r) {
                    pos[ah] = retractAH;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                }
                break;
            }

            case MotionMode::BORE_FEED_OUT:  // G85
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                pos[ah] = p.r;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                if (retractAH > p.r) {
                    pos[ah] = retractAH;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                }
                break;

            case MotionMode::BORE_STOP_RAPID:  // G86
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                spindle(false, true);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                spindle(true, true);
                break;

            case MotionMode::BORE_MANUAL:  // G88 — dwell + stop, rapid out
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                if (p.dwell > 0.0) emitDwell(p.dwell);
                spindle(false, true);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                break;

            case MotionMode::BORE_DWELL:  // G89 — dwell, feed out
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                if (p.dwell > 0.0) emitDwell(p.dwell);
                pos[ah] = p.r;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                if (retractAH > p.r) {
                    pos[ah] = retractAH;
                    emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                }
                break;

            case MotionMode::THREAD_CYCLE:  // G76 — fine boring
            case MotionMode::BORE_BACK: {   // G87 — back boring
                // Approximation: bore to bottom, orient + shift off the wall,
                // rapid out, unshift, restart spindle.
                pos[ah] = p.z;
                emitMove(MotionSegment::Type::LINEAR, pos, feed);
                spindle(false, true);
                pos[a1] -= p.shiftI;
                pos[a2] -= p.shiftJ;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                pos[ah] = retractAH;
                emitMove(MotionSegment::Type::RAPID, pos, 0.0);
                pos[a1] += p.shiftI;
                pos[a2] += p.shiftJ;
                spindle(true, true);
                break;
            }

            default:
                break;
        }
    }
    return Error{};
}

// ============================================================================
// Probing (G38.2–G38.5)
// ============================================================================

Error Interpreter::executeProbe(MotionMode probeType, const Block& block,
                                const Position& target, double unitScale,
                                std::vector<MotionSegment>& segments) {
    const bool errorOnMiss = (probeType == MotionMode::PROBE_TOWARD ||
                              probeType == MotionMode::PROBE_AWAY);
    ProbeType pt = (probeType == MotionMode::PROBE_TOWARD ||
                    probeType == MotionMode::PROBE_TOWARD_NE)
                       ? ProbeType::TOWARD_WITH_ERROR
                       : ProbeType::AWAY_WITH_ERROR;

    ProbeResult result;
    if (m_probeCallback) {
        Error err = m_probeCallback(target, m_machineState.feedRate, pt,
                                    result);
        if (!err.ok())
            return err;
    } else {
        // No hardware probe attached — assume the probe trips at the
        // programmed target (dry-run / simulation semantics).
        result.tripped = true;
        result.tripPosition = m_coordinates.toMachineCoords(target);
        result.tripWorkPosition = target;
        const Position d = result.tripPosition -
                           m_machineState.machinePosition;
        result.travelDistance = std::sqrt(d.dot(d));
    }
    result.success = result.tripped;

    if (!result.tripped && errorOnMiss) {
        return makeError(ErrorCode::PROBE_ERROR,
                         "Probe move finished without contact");
    }

    // Emit the probe segment to the trip point (or programmed target on a
    // no-error miss).
    const Position progEnd =
        result.tripped ? result.tripWorkPosition : target;
    MotionSegment seg;
    seg.type = MotionSegment::Type::PROBE;
    seg.endPosition = m_coordinates.toMachineCoords(progEnd);
    seg.feedRate = m_machineState.feedRate;
    seg.lineNumber = block.sourceLineNumber;
    segments.push_back(seg);
    ++m_stats.motionSegments;

    m_machineState.workPosition = progEnd;
    m_machineState.machinePosition = seg.endPosition;

    // Publish probe results: #5061-#5068 trip position, #5070 success flag.
    m_variables.setProbeResult(result);
    (void)unitScale;
    return Error{};
}

Error Interpreter::executeSpline(MotionMode mode, const Block& block,
                                 const Position& target, double unitScale,
                                 std::vector<MotionSegment>& segments) {
    const Position& start = m_machineState.workPosition;

    // Plane axes (same convention as arcs): XY → I,J / ZX → K,I / YZ → J,K
    int a1, a2;
    switch (m_machineState.plane) {
        case Plane::ZX: a1 = 2; a2 = 0; break;
        case Plane::YZ: a1 = 1; a2 = 2; break;
        default:        a1 = 0; a2 = 1; break;
    }
    const WordLetter w1 = (m_machineState.plane == Plane::ZX)
        ? WordLetter::K : (m_machineState.plane == Plane::YZ)
        ? WordLetter::J : WordLetter::I;
    const WordLetter w2 = (m_machineState.plane == Plane::ZX)
        ? WordLetter::I : (m_machineState.plane == Plane::YZ)
        ? WordLetter::K : WordLetter::J;

    Position p1 = start;
    Position p2 = target;
    if (block.hasWord(w1))
        p1[a1] += block.getWord(w1) * unitScale;
    if (block.hasWord(w2))
        p1[a2] += block.getWord(w2) * unitScale;

    MotionSegment seg;
    seg.type = MotionSegment::Type::SPLINE;
    seg.lineNumber = block.sourceLineNumber;
    seg.feedRate = m_machineState.feedRate;

    if (mode == MotionMode::CUBIC_SPLINE) {
        // P/Q offset the second control point from the end point.
        if (block.hasWord(WordLetter::P))
            p2[a1] += block.getWord(WordLetter::P) * unitScale;
        if (block.hasWord(WordLetter::Q))
            p2[a2] += block.getWord(WordLetter::Q) * unitScale;
    }

    seg.splinePoints[0] = m_coordinates.toMachineCoords(start);
    seg.splinePoints[1] = m_coordinates.toMachineCoords(p1);
    seg.splinePoints[2] = (mode == MotionMode::CUBIC_SPLINE)
        ? m_coordinates.toMachineCoords(p2)
        : m_coordinates.toMachineCoords(target);
    seg.splinePoints[3] = m_coordinates.toMachineCoords(target);
    seg.endPosition = seg.splinePoints[3];

    segments.push_back(seg);
    ++m_stats.motionSegments;
    m_machineState.workPosition = target;
    m_machineState.machinePosition = seg.endPosition;
    return Error{};
}

Error Interpreter::executeNurbs(int nurbsG, const Block& block,
                                const Position& target, double unitScale,
                                std::vector<MotionSegment>& segments) {
    (void)unitScale;
    if (nurbsG == 52) {  // G5.2: begin NURBS block
        m_nurbsActive = true;
        m_nurbsPoints.clear();
        m_nurbsOrder = static_cast<int32_t>(block.getWord(WordLetter::L, 3));
        if (m_nurbsOrder < 2) m_nurbsOrder = 2;
        if (m_nurbsOrder > 8) m_nurbsOrder = 8;
        // Optional first control point on the same line (P = weight)
        if (block.hasWord(WordLetter::X) || block.hasWord(WordLetter::Y)) {
            NurbsControlPoint cp;
            cp.point = target;
            cp.weight = block.getWord(WordLetter::P, 1.0);
            m_nurbsPoints.push_back(cp);
        }
        m_machineState.motionMode = MotionMode::NURBS;
        return Error{};
    }

    if (nurbsG == 53) {  // G5.3: end block, tessellate
        if (!m_nurbsActive) {
            m_machineState.motionMode = MotionMode::LINEAR;
            return Error{};
        }
        m_nurbsActive = false;
        m_machineState.motionMode = MotionMode::LINEAR;

        const size_t n = m_nurbsPoints.size();
        const int order = m_nurbsOrder;
        if (n < static_cast<size_t>(order)) {
            return makeError(ErrorCode::INVALID_MOTION,
                             "NURBS: fewer control points than order");
        }

        // Clamped uniform knot vector: N + p + 1 knots for N points,
        // degree p. First p+1 knots are 0, last p+1 are 1, interior
        // knots evenly spaced.
        const int N = static_cast<int>(n);
        const int p = order - 1;
        const int knotCount = N + p + 1;
        std::vector<double> knots(knotCount);
        for (int i = 0; i < knotCount; ++i) {
            if (i <= p) knots[i] = 0.0;
            else if (i >= N) knots[i] = 1.0;
            else knots[i] = static_cast<double>(i - p) /
                            static_cast<double>(N - p);
        }
        const int degree = p;

        // de Boor evaluation in homogeneous coordinates (9 axes + weight)
        const int samples =
            std::max(16, static_cast<int>(n) * 8);
        for (int s = 1; s <= samples; ++s) {
            const double u = static_cast<double>(s) / samples;

            // Knot span: knots[k] <= u < knots[k+1]
            int k = degree;
            for (int i = degree; i < static_cast<int>(n); ++i) {
                if (u >= knots[i] && u < knots[i + 1]) { k = i; break; }
            }
            if (u >= 1.0) k = static_cast<int>(n) - 1;

            std::array<std::array<double, 10>, 9> d{};
            for (int j = 0; j <= degree; ++j) {
                const auto& cp = m_nurbsPoints[k - degree + j];
                for (int ax = 0; ax < 9; ++ax)
                    d[j][ax] = cp.weight * cp.point[ax];
                d[j][9] = cp.weight;
            }
            for (int r = 1; r <= degree; ++r) {
                for (int j = degree; j >= r; --j) {
                    const int i = k - degree + j;
                    const double denom =
                        knots[i + degree + 1 - r] - knots[i];
                    const double alpha =
                        (denom > 0.0) ? (u - knots[i]) / denom : 0.0;
                    for (int ax = 0; ax < 10; ++ax)
                        d[j][ax] = (1.0 - alpha) * d[j - 1][ax] +
                                   alpha * d[j][ax];
                }
            }

            Position pt{};
            const double w = d[degree][9];
            if (w > 1e-12)
                for (int ax = 0; ax < 9; ++ax) pt[ax] = d[degree][ax] / w;

            MotionSegment seg;
            seg.type = MotionSegment::Type::LINEAR;
            seg.endPosition = m_coordinates.toMachineCoords(pt);
            seg.feedRate = m_machineState.feedRate;
            seg.lineNumber = block.sourceLineNumber;
            segments.push_back(seg);
            ++m_stats.motionSegments;
        }

        if (!m_nurbsPoints.empty()) {
            const Position& last = m_nurbsPoints.back().point;
            m_machineState.workPosition = last;
            m_machineState.machinePosition =
                m_coordinates.toMachineCoords(last);
        }
        return Error{};
    }

    // Bare axis block while collecting: add a control point (P = weight).
    if (m_nurbsActive &&
        (block.hasWord(WordLetter::X) || block.hasWord(WordLetter::Y))) {
        NurbsControlPoint cp;
        cp.point = target;
        cp.weight = block.getWord(WordLetter::P, 1.0);
        m_nurbsPoints.push_back(cp);
    }
    return Error{};
}

} // namespace GCode
