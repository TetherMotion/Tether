/**
 * @file GCodeInterpreter_gcodes.cpp
 * @brief Interpreter — G-code word processing.
 *
 * TU split out of GCodeInterpreter.cpp.
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
                case 400: { // G40: cancel cutter compensation
                    // Lead-out: return from the offset point to the
                    // programmed position.
                    if (m_compHasOffset) {
                        MotionSegment seg;
                        seg.type = MotionSegment::Type::LINEAR;
                        seg.endPosition = m_coordinates.toMachineCoords(
                            m_machineState.workPosition);
                        seg.feedRate = m_machineState.feedRate;
                        seg.lineNumber = block.sourceLineNumber;
                        segments.push_back(seg);
                        m_machineState.machinePosition = seg.endPosition;
                        ++m_stats.motionSegments;
                    }
                    m_machineState.cutterComp = CutterCompMode::OFF;
                    m_machineState.cutterRadius = 0.0;
                    m_compHasOffset = false;
                    return Error{};
                }
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
                    m_compHasOffset = false;
                    return Error{};
                }
                case 411:
                case 421: { // G41.1/G42.1 D<diameter>: dynamic comp
                    m_machineState.cutterComp = (gi == 411)
                        ? CutterCompMode::LEFT_DYNAMIC
                        : CutterCompMode::RIGHT_DYNAMIC;
                    m_machineState.cutterRadius =
                        block.getWord(WordLetter::D, 0.0) / 2.0;
                    m_compHasOffset = false;
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
                case 70: // G70 — Fanuc lathe finishing cycle, or inch units
                    if (featureEnabled(Feature::G70_G71_UNITS)) {
                        m_machineState.units = Units::INCH;
                        return Error{};
                    }
                    return dispatchG70(block, segments);
                case 71: // G71 — Fanuc lathe rough turning, or metric units
                    if (featureEnabled(Feature::G70_G71_UNITS)) {
                        m_machineState.units = Units::MM;
                        return Error{};
                    }
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
} // namespace GCode
