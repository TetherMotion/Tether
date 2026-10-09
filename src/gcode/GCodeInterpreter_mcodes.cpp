/**
 * @file GCodeInterpreter_mcodes.cpp
 * @brief Interpreter — M-code processing.
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
} // namespace GCode
