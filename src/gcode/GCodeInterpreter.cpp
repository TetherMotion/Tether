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
    initializeDefaults();
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
    // Probe callback is stored in the probe handler
    (void)callback;
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
            switch (gnum) {
                case 17: m_machineState.plane = Plane::XY; break;
                case 18: m_machineState.plane = Plane::ZX; break;
                case 19: m_machineState.plane = Plane::YZ; break;
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
            if (gnum >= 54 && gnum <= 59) wcsNum = gnum - 53;
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

        case ModalGroup::TOOL_LENGTH: {
            switch (gnum) {
                case 43: {
                    // G43 H<tool>: apply tool length offset from tool table
                    int tool = static_cast<int>(block.getWord(WordLetter::H, 0));
                    m_machineState.toolLengthMode = ToolLengthMode::POSITIVE;
                    // Look up tool offset from tool table
                    // For now, use the Z offset from the tool entry
                    m_machineState.toolOffset.z() = 0.0; // Placeholder: tool table lookup
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                case 431: { // G43.1: dynamic tool length offset
                    m_machineState.toolLengthMode = ToolLengthMode::DYNAMIC;
                    if (block.hasWord(WordLetter::Z))
                        m_machineState.toolOffset.z() = block.getWord(WordLetter::Z);
                    m_coordinates.syncTransform(m_machineState);
                    return Error{};
                }
                case 49: { // G49: cancel tool length offset
                    m_machineState.toolLengthMode = ToolLengthMode::OFF;
                    m_machineState.toolOffset.z() = 0.0;
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
            if (gnum == 50) return dispatchG50();
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
                case 80: mode = MotionMode::CANNED_OFF; break;
                default: break;
            }
            m_machineState.motionMode = mode;
            break;
        }
    }

    // Update feed rate if F word is present
    if (block.hasWord(WordLetter::F))
        m_machineState.feedRate = block.getWord(WordLetter::F);

    // If no motion words are present, don't generate a segment.
    // The G-code only updates the modal motion mode and/or feed rate.
    if (!hasMotionWords(block) &&
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

    // When emit-arc-segments mode is active, emit a single arc MotionSegment
    // with full ArcParams instead of tessellating into line segments.
    if (m_emitArcSegments) {
        MotionSegment seg;
        seg.type = (mode == MotionMode::CW_ARC)
                       ? MotionSegment::Type::ARC_CW
                       : MotionSegment::Type::ARC_CCW;
        seg.endPosition = m_coordinates.toMachineCoords(end);
        seg.feedRate = m_machineState.feedRate;
        seg.lineNumber = block.sourceLineNumber;

        // centerOffset is relative to start (in program coordinates)
        Position centerPos = start;
        centerPos[a1] = center1;
        centerPos[a2] = center2;
        seg.centerOffset = centerPos - start;

        // Populate ArcParams
        seg.arc.center = centerPos;
        seg.arc.startPoint = start;
        seg.arc.endPoint = end;
        seg.arc.radius = radius;
        seg.arc.startAngle = startAngle;
        seg.arc.endAngle = endAngle;
        seg.arc.sweepAngle = sweep;
        seg.arc.clockwise = (mode == MotionMode::CW_ARC);
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

} // namespace GCode
