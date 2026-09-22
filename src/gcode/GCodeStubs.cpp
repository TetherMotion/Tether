/**
 * @file GCodeStubs.cpp
 * @brief Stub implementations for motion handler constructors.
 *
 * @details
 * The motion handler classes (LinearMotionHandler, ArcMotionHandler, etc.)
 * have their APIs declared in headers but are not yet fully implemented.
 * This file provides the minimal constructor implementations needed to
 * link the GCodeInterpreter.
 */

#include "tether/gcode/motion/GCodeG0G1.hpp"
#include "tether/gcode/motion/GCodeG2G3.hpp"
#include "tether/gcode/motion/GCodeSplines.hpp"
#include "tether/gcode/motion/GCodeCannedCycles.hpp"
#include "tether/gcode/motion/GCodeProbing.hpp"
#include "tether/gcode/motion/GCodeToolComp.hpp"
#include "tether/gcode/motion/GCodeAdvancedMotion.hpp"

namespace GCode {

// --- LinearMotionHandler ---
LinearMotionHandler::LinearMotionHandler(const LinearMotionConfig& config)
    : m_config(config) {}
Error LinearMotionHandler::processG0(const Block&, MachineState&, VariableSystem&,
                                     std::vector<MotionSegment>&) { return Error{}; }
Error LinearMotionHandler::processG1(const Block&, MachineState&, VariableSystem&,
                                     std::vector<MotionSegment>&) { return Error{}; }

// --- ArcMotionHandler ---
ArcMotionHandler::ArcMotionHandler(const ArcMotionConfig& config)
    : m_config(config) {}
Error ArcMotionHandler::processG2(const Block&, MachineState&, VariableSystem&,
                                  std::vector<MotionSegment>&) { return Error{}; }
Error ArcMotionHandler::processG3(const Block&, MachineState&, VariableSystem&,
                                  std::vector<MotionSegment>&) { return Error{}; }

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
