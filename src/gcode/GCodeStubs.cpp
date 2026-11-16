/**
 * @file GCodeStubs.cpp
 * @brief Link/compat implementations for motion handler scaffolding.
 *
 * @details
 * The motion handler classes (LinearMotionHandler, ArcMotionHandler, etc.)
 * have their APIs declared in headers, but the production motion pipeline
 * lives in GCodeInterpreter_*.cpp (handleMotion / executeCannedCycle /
 * executeProbe).  These handler objects are not dispatched to by the
 * interpreter; the process* entry points below therefore fail explicitly
 * rather than silently reporting success for a move that never happened.
 * ToolTable and the compensation helpers in this file are real and used.
 */

#include "tether/gcode/motion/GCodeG0G1.hpp"
#include "tether/gcode/motion/GCodeG2G3.hpp"
#include "tether/gcode/motion/GCodeSplines.hpp"
#include "tether/gcode/motion/GCodeCannedCycles.hpp"
#include "tether/gcode/motion/GCodeProbing.hpp"
#include "tether/gcode/motion/GCodeToolComp.hpp"
#include "tether/gcode/motion/GCodeAdvancedMotion.hpp"

#include <cstdio>

namespace GCode {

namespace {
Error handlerNotImplemented(const char* what) {
    Error err;
    err.code = ErrorCode::INVALID_MOTION;
    std::snprintf(err.message.data(), err.message.size(),
                  "%s: handler not implemented (use Interpreter pipeline)",
                  what);
    return err;
}
} // namespace

// --- LinearMotionHandler ---
LinearMotionHandler::LinearMotionHandler(const LinearMotionConfig& config)
    : m_config(config) {}
Error LinearMotionHandler::processG0(const Block&, MachineState&, VariableSystem&,
                                     std::vector<MotionSegment>&) {
    return handlerNotImplemented("G0");
}
Error LinearMotionHandler::processG1(const Block&, MachineState&, VariableSystem&,
                                     std::vector<MotionSegment>&) {
    return handlerNotImplemented("G1");
}

// --- ArcMotionHandler ---
ArcMotionHandler::ArcMotionHandler(const ArcMotionConfig& config)
    : m_config(config) {}
Error ArcMotionHandler::processG2(const Block&, MachineState&, VariableSystem&,
                                  std::vector<MotionSegment>&) {
    return handlerNotImplemented("G2");
}
Error ArcMotionHandler::processG3(const Block&, MachineState&, VariableSystem&,
                                  std::vector<MotionSegment>&) {
    return handlerNotImplemented("G3");
}

// --- SplineHandler ---
SplineHandler::SplineHandler(const SplineConfig& config)
    : m_config(config) {}

// --- CannedCycleHandler ---
CannedCycleHandler::CannedCycleHandler(const CannedCycleConfig& config)
    : m_config(config) {}

// --- ProbeHandler ---
ProbeHandler::ProbeHandler(const ProbeConfig& config)
    : m_config(config) {}

// --- ToolTable ---
ToolTable::ToolTable(size_t maxTools) {
    m_tools.reserve(maxTools);
}

const ToolEntry* ToolTable::getTool(int32_t toolNumber) const {
    for (const auto& t : m_tools) {
        if (t.toolNumber == toolNumber) return &t;
    }
    return nullptr;
}

ToolEntry* ToolTable::getTool(int32_t toolNumber) {
    for (auto& t : m_tools) {
        if (t.toolNumber == toolNumber) return &t;
    }
    return nullptr;
}

Error ToolTable::setTool(int32_t toolNumber, const ToolEntry& entry) {
    if (toolNumber < 0) {
        Error err;
        err.code = ErrorCode::TOOL_ERROR;
        std::snprintf(err.message.data(), err.message.size(), "%s",
                      "Invalid tool number");
        return err;
    }
    if (ToolEntry* t = getTool(toolNumber)) {
        *t = entry;
        t->toolNumber = toolNumber;
        return Error{};
    }
    if (m_tools.size() >= MAX_TOOLS) {
        Error err;
        err.code = ErrorCode::TOOL_ERROR;
        std::snprintf(err.message.data(), err.message.size(), "%s",
                      "Tool table full");
        return err;
    }
    ToolEntry e = entry;
    e.toolNumber = toolNumber;
    m_tools.push_back(e);
    return Error{};
}

const ToolEntry* ToolTable::getToolByPocket(int32_t pocket) const {
    for (const auto& t : m_tools) {
        if (t.pocketNumber == pocket) return &t;
    }
    return nullptr;
}

void ToolTable::setCurrentTool(int32_t toolNumber) {
    m_currentTool = toolNumber;
}

const ToolEntry* ToolTable::getCurrentToolEntry() const {
    return getTool(m_currentTool);
}

size_t ToolTable::getToolCount() const {
    return m_tools.size();
}

void ToolTable::clear() {
    m_tools.clear();
}

// --- ToolLengthComp ---
ToolLengthComp::ToolLengthComp(ToolTable& toolTable)
    : m_toolTable(toolTable) {}

// --- CutterRadiusComp ---
CutterRadiusComp::CutterRadiusComp(ToolTable& toolTable, const CutterCompConfig& config)
    : m_toolTable(toolTable), m_config(config) {}

// --- PathBlender ---
PathBlender::PathBlender(const PathBlendConfig& config)
    : m_config(config) {}

// --- TrochoidalHandler ---
TrochoidalHandler::TrochoidalHandler(const TrochoidalConfig& config)
    : m_config(config) {}

// --- VolumetricCompensation ---
VolumetricCompensation::VolumetricCompensation(const VolumetricConfig& config)
    : m_config(config) {}

// --- BacklashCompensation ---
BacklashCompensation::BacklashCompensation() = default;

} // namespace GCode
