/**
 * @file GCodeCoordinates.cpp
 * @brief Implementation of CoordinateSystemManager: WCS (G54-G59.3),
 *        G52, G92, G10, G68/G69, G51/G50, and the composed CoordinateTransform.
 */

#include "tether/gcode/motion/GCodeCoordinates.hpp"
#include "tether/gcode/GCodeVariables.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace GCode {

namespace {

/// Helper to construct an Error with a message string (copied into the
/// fixed-size char array).
Error makeError(ErrorCode code, const char* msg) {
    Error err;
    err.code = code;
    err.message.fill(0);
    err.context.fill(0);
    if (msg) std::snprintf(err.message.data(), err.message.size(), "%s", msg);
    return err;
}

} // namespace

// ============================================================================
// Construction / reset
// ============================================================================

CoordinateSystemManager::CoordinateSystemManager() {
    for (size_t i = 0; i < NUM_WORK_COORD_SYSTEMS; ++i) {
        m_wcs[i].number = static_cast<int32_t>(i + 1);
        m_wcs[i].name = wcsNumberToString(static_cast<int32_t>(i + 1));
    }
}

void CoordinateSystemManager::reset() {
    for (auto& wcs : m_wcs) {
        wcs.offset = Position{};
        wcs.modified = false;
        wcs.name = wcsNumberToString(wcs.number);
    }
    m_activeWCS = 1;
    m_g92Offset = Position{};
    m_g92Active = false;
    m_g92Saved = Position{};
    m_g53Active = false;
    m_g28Reference = Position{};
    m_g30References.fill(Position{});
    m_transform.reset();
}

// ============================================================================
// WCS selection (G54-G59.3)
// ============================================================================

Error CoordinateSystemManager::selectWCS(double gcode) {
    int32_t num = gcodeToWCSNumber(gcode);
    if (num < 1 || num > 9)
        return makeError(ErrorCode::INVALID_MOTION, "Invalid WCS G-code");
    m_activeWCS = num;
    return Error{};
}

Error CoordinateSystemManager::selectWCS(int32_t number) {
    if (number < 1 || number > 9)
        return makeError(ErrorCode::INVALID_MOTION, "WCS number out of range");
    m_activeWCS = number;
    return Error{};
}

const WorkCoordinateSystem& CoordinateSystemManager::getActiveWCS() const {
    return m_wcs[wcsIndex(m_activeWCS)];
}

const WorkCoordinateSystem& CoordinateSystemManager::getWCS(int32_t number) const {
    return m_wcs[wcsIndex(number)];
}

WorkCoordinateSystem& CoordinateSystemManager::getWCS(int32_t number) {
    return m_wcs[wcsIndex(number)];
}

// ============================================================================
// G92 offset
// ============================================================================

Error CoordinateSystemManager::processG92(
    const Block& block, const Position& machinePos,
    MachineState& state, VariableSystem& vars) {

    Position offset = m_g92Offset;
    // For each specified axis, set the G92 offset so that the current
    // machine position maps to the specified program coordinate.
    // program = machine - wcs - g92  =>  g92 = machine - wcs - program
    // But G92 sets the *program* coordinate of the *current* position.
    // The current machine position is machinePos. The WCS offset is
    // m_wcs[active].offset. So:
    //   g92_axis = machinePos_axis - wcs_axis - specified_program_axis
    const Position& wcsOff = m_wcs[wcsIndex(m_activeWCS)].offset;
    if (block.hasWord(WordLetter::X))
        m_g92Offset.x() = machinePos.x() - wcsOff.x() - block.getWord(WordLetter::X);
    if (block.hasWord(WordLetter::Y))
        m_g92Offset.y() = machinePos.y() - wcsOff.y() - block.getWord(WordLetter::Y);
    if (block.hasWord(WordLetter::Z))
        m_g92Offset.z() = machinePos.z() - wcsOff.z() - block.getWord(WordLetter::Z);
    if (block.hasWord(WordLetter::A))
        m_g92Offset[Axis::A] = machinePos[Axis::A] - wcsOff[Axis::A] - block.getWord(WordLetter::A);
    if (block.hasWord(WordLetter::B))
        m_g92Offset[Axis::B] = machinePos[Axis::B] - wcsOff[Axis::B] - block.getWord(WordLetter::B);
    if (block.hasWord(WordLetter::C))
        m_g92Offset[Axis::C] = machinePos[Axis::C] - wcsOff[Axis::C] - block.getWord(WordLetter::C);

    m_g92Active = true;
    state.g92Offset = m_g92Offset;
    syncTransform(state);
    (void)vars; // parameters synced separately via syncToVariables
    (void)offset;
    return Error{};
}

Error CoordinateSystemManager::processG92_1(MachineState& state, VariableSystem& vars) {
    m_g92Offset = Position{};
    m_g92Active = false;
    m_g92Saved = Position{};
    state.g92Offset = Position{};
    syncTransform(state);
    (void)vars;
    return Error{};
}

Error CoordinateSystemManager::processG92_2(MachineState& state) {
    m_g92Saved = m_g92Offset;
    m_g92Active = false;
    // Keep the offset value but mark inactive so toMachineCoords skips it.
    // For simplicity we clear the active flag and zero the applied offset;
    // G92.3 restores from m_g92Saved.
    m_g92Offset = Position{};
    state.g92Offset = Position{};
    syncTransform(state);
    return Error{};
}

Error CoordinateSystemManager::processG92_3(MachineState& state) {
    m_g92Offset = m_g92Saved;
    m_g92Active = true;
    state.g92Offset = m_g92Offset;
    syncTransform(state);
    return Error{};
}

// ============================================================================
// G53 (machine coordinates, non-modal)
// ============================================================================

bool CoordinateSystemManager::processG53() {
    m_g53Active = true;
    return true;
}

// ============================================================================
// G52 (local coordinate offset)
// ============================================================================

Error CoordinateSystemManager::processG52(const Block& block, MachineState& state) {
    // G52 sets a local offset in program space. With no axis words, reset.
    Position off = state.g52Offset;
    if (block.hasWord(WordLetter::X)) off.x() = block.getWord(WordLetter::X);
    if (block.hasWord(WordLetter::Y)) off.y() = block.getWord(WordLetter::Y);
    if (block.hasWord(WordLetter::Z)) off.z() = block.getWord(WordLetter::Z);
    if (block.hasWord(WordLetter::A)) off[Axis::A] = block.getWord(WordLetter::A);
    if (block.hasWord(WordLetter::B)) off[Axis::B] = block.getWord(WordLetter::B);
    if (block.hasWord(WordLetter::C)) off[Axis::C] = block.getWord(WordLetter::C);
    if (block.hasWord(WordLetter::U)) off[Axis::U] = block.getWord(WordLetter::U);
    if (block.hasWord(WordLetter::V)) off[Axis::V] = block.getWord(WordLetter::V);
    if (block.hasWord(WordLetter::W)) off[Axis::W] = block.getWord(WordLetter::W);
    // If no axis words at all, reset to zero.
    if (!block.hasWord(WordLetter::X) && !block.hasWord(WordLetter::Y) &&
        !block.hasWord(WordLetter::Z) && !block.hasWord(WordLetter::A) &&
        !block.hasWord(WordLetter::B) && !block.hasWord(WordLetter::C) &&
        !block.hasWord(WordLetter::U) && !block.hasWord(WordLetter::V) &&
        !block.hasWord(WordLetter::W)) {
        off = Position{};
    }
    state.g52Offset = off;
    syncTransform(state);
    return Error{};
}

Error CoordinateSystemManager::clearG52(MachineState& state) {
    state.g52Offset = Position{};
    syncTransform(state);
    return Error{};
}

// ============================================================================
// G68 / G69 (coordinate rotation)
// ============================================================================

Error CoordinateSystemManager::processG68(const Block& block, MachineState& state) {
    const bool hasA = block.hasWord(WordLetter::A);
    const bool hasB = block.hasWord(WordLetter::B);
    const bool hasC = block.hasWord(WordLetter::C);
    const bool hasI = block.hasWord(WordLetter::I);
    const bool hasJ = block.hasWord(WordLetter::J);
    const bool hasK = block.hasWord(WordLetter::K);
    const bool hasR = block.hasWord(WordLetter::R);

    // Pivot: X/Y/Z words (default 0).
    std::array<double, 3> pivot = {0, 0, 0};
    if (block.hasWord(WordLetter::X)) pivot[0] = block.getWord(WordLetter::X);
    if (block.hasWord(WordLetter::Y)) pivot[1] = block.getWord(WordLetter::Y);
    if (block.hasWord(WordLetter::Z)) pivot[2] = block.getWord(WordLetter::Z);

    if (hasI || hasJ || hasK) {
        // 3D axis-angle mode.
        std::array<double, 3> axis = {
            hasI ? block.getWord(WordLetter::I) : 0.0,
            hasJ ? block.getWord(WordLetter::J) : 0.0,
            hasK ? block.getWord(WordLetter::K) : 0.0,
        };
        double angle = hasR ? block.getWord(WordLetter::R) : 0.0;
        state.g68Mode = 2;
        state.g68Axis = axis;
        state.g68AxisAngle = angle;
        state.g68Pivot = Position{};
        state.g68Pivot.x() = pivot[0];
        state.g68Pivot.y() = pivot[1];
        state.g68Pivot.z() = pivot[2];
        state.g68Active = true;
    } else if (hasA || hasB || hasC) {
        // 3D Euler XYZ mode.
        std::array<double, 3> euler = {
            hasA ? block.getWord(WordLetter::A) : 0.0,
            hasB ? block.getWord(WordLetter::B) : 0.0,
            hasC ? block.getWord(WordLetter::C) : 0.0,
        };
        state.g68Mode = 1;
        state.g68Euler = euler;
        state.g68Pivot = Position{};
        state.g68Pivot.x() = pivot[0];
        state.g68Pivot.y() = pivot[1];
        state.g68Pivot.z() = pivot[2];
        state.g68Active = true;
    } else {
        // 2D plane rotation mode. R is the angle.
        double angle = hasR ? block.getWord(WordLetter::R) : 0.0;
        // For G17 (XY): pivot is (X, Y). For G18 (ZX): pivot is (Z, X) ->
        // mapped as (pivotA=Z, pivotB=X). For G19 (YZ): pivot is (Y, Z).
        // We store the raw in-plane pivots and let the transform map them.
        double pivotA = pivot[0]; // first in-plane axis
        double pivotB = pivot[1]; // second in-plane axis
        // If only one pivot word is given, the other defaults to 0.
        if (state.plane == Plane::ZX) {
            pivotA = block.hasWord(WordLetter::Z) ? block.getWord(WordLetter::Z) : 0.0;
            pivotB = block.hasWord(WordLetter::X) ? block.getWord(WordLetter::X) : 0.0;
        } else if (state.plane == Plane::YZ) {
            pivotA = block.hasWord(WordLetter::Y) ? block.getWord(WordLetter::Y) : 0.0;
            pivotB = block.hasWord(WordLetter::Z) ? block.getWord(WordLetter::Z) : 0.0;
        }
        state.g68Mode = 0;
        state.coordRotation = angle; // legacy field
        state.g68Pivot = Position{};
        state.g68Pivot.x() = pivotA;
        state.g68Pivot.y() = pivotB;
        state.g68Active = true;
    }
    syncTransform(state);
    return Error{};
}

Error CoordinateSystemManager::processG69(MachineState& state) {
    state.g68Active = false;
    state.g68Mode = 0;
    state.coordRotation = 0.0;
    state.g68Euler = {0, 0, 0};
    state.g68Axis = {0, 0, 0};
    state.g68AxisAngle = 0.0;
    state.g68Pivot = Position{};
    syncTransform(state);
    return Error{};
}

// ============================================================================
// G51 / G50 (scaling)
// ============================================================================

Error CoordinateSystemManager::processG51(const Block& block, MachineState& state) {
    Position scale = state.scaleFactors;
    // If P word is present, uniform scale for all axes.
    if (block.hasWord(WordLetter::P)) {
        double s = block.getWord(WordLetter::P);
        scale = Position::ones() * s;
    } else {
        // Per-axis scale factors.
        if (block.hasWord(WordLetter::X)) scale.x() = block.getWord(WordLetter::X);
        if (block.hasWord(WordLetter::Y)) scale.y() = block.getWord(WordLetter::Y);
        if (block.hasWord(WordLetter::Z)) scale.z() = block.getWord(WordLetter::Z);
        if (block.hasWord(WordLetter::A)) scale[Axis::A] = block.getWord(WordLetter::A);
        if (block.hasWord(WordLetter::B)) scale[Axis::B] = block.getWord(WordLetter::B);
        if (block.hasWord(WordLetter::C)) scale[Axis::C] = block.getWord(WordLetter::C);
        if (block.hasWord(WordLetter::U)) scale[Axis::U] = block.getWord(WordLetter::U);
        if (block.hasWord(WordLetter::V)) scale[Axis::V] = block.getWord(WordLetter::V);
        if (block.hasWord(WordLetter::W)) scale[Axis::W] = block.getWord(WordLetter::W);
    }
    state.scaleFactors = scale;
    state.g51Active = true;
    syncTransform(state);
    return Error{};
}

Error CoordinateSystemManager::processG50(MachineState& state) {
    state.scaleFactors = Position::ones();
    state.g51Active = false;
    syncTransform(state);
    return Error{};
}

// ============================================================================
// G10 (set coordinate data)
// ============================================================================

Error CoordinateSystemManager::processG10L2(
    int32_t pWord, const Block& block, VariableSystem& vars) {
    if (pWord < 1 || pWord > 9)
        return makeError(ErrorCode::INVALID_MOTION, "G10 P word out of range");
    WorkCoordinateSystem& wcs = m_wcs[wcsIndex(pWord)];
    if (block.hasWord(WordLetter::X)) wcs.offset.x() = block.getWord(WordLetter::X);
    if (block.hasWord(WordLetter::Y)) wcs.offset.y() = block.getWord(WordLetter::Y);
    if (block.hasWord(WordLetter::Z)) wcs.offset.z() = block.getWord(WordLetter::Z);
    if (block.hasWord(WordLetter::A)) wcs.offset[Axis::A] = block.getWord(WordLetter::A);
    if (block.hasWord(WordLetter::B)) wcs.offset[Axis::B] = block.getWord(WordLetter::B);
    if (block.hasWord(WordLetter::C)) wcs.offset[Axis::C] = block.getWord(WordLetter::C);
    wcs.modified = true;
    (void)vars;
    return Error{};
}

Error CoordinateSystemManager::processG10L20(
    int32_t pWord, const Block& block,
    const Position& machinePos, VariableSystem& vars) {
    if (pWord < 1 || pWord > 9)
        return makeError(ErrorCode::INVALID_MOTION, "G10 P word out of range");
    WorkCoordinateSystem& wcs = m_wcs[wcsIndex(pWord)];
    // L20: set WCS so that current machine position becomes the specified
    // program coordinate. wcs_offset = machine - program.
    if (block.hasWord(WordLetter::X)) wcs.offset.x() = machinePos.x() - block.getWord(WordLetter::X);
    if (block.hasWord(WordLetter::Y)) wcs.offset.y() = machinePos.y() - block.getWord(WordLetter::Y);
    if (block.hasWord(WordLetter::Z)) wcs.offset.z() = machinePos.z() - block.getWord(WordLetter::Z);
    if (block.hasWord(WordLetter::A)) wcs.offset[Axis::A] = machinePos[Axis::A] - block.getWord(WordLetter::A);
    if (block.hasWord(WordLetter::B)) wcs.offset[Axis::B] = machinePos[Axis::B] - block.getWord(WordLetter::B);
    if (block.hasWord(WordLetter::C)) wcs.offset[Axis::C] = machinePos[Axis::C] - block.getWord(WordLetter::C);
    wcs.modified = true;
    (void)vars;
    return Error{};
}

// ============================================================================
// G28 / G30 (reference points)
// ============================================================================

Error CoordinateSystemManager::processG28(
    const Block& block, MachineState& state,
    std::vector<MotionSegment>& segments) {
    // TODO: emit motion segments to the reference point via an intermediate.
    // For now, store the reference and update state.
    (void)block;
    (void)state;
    (void)segments;
    return Error{};
}

Error CoordinateSystemManager::processG28_1(
    const Position& machinePos, VariableSystem& vars) {
    m_g28Reference = machinePos;
    (void)vars;
    return Error{};
}

Error CoordinateSystemManager::processG30(
    const Block& block, int32_t pWord,
    MachineState& state, std::vector<MotionSegment>& segments) {
    (void)block; (void)pWord; (void)state; (void)segments;
    return Error{};
}

Error CoordinateSystemManager::processG30_1(
    const Position& machinePos, int32_t pWord, VariableSystem& vars) {
    int32_t idx = std::clamp(pWord, 1, 4) - 1;
    m_g30References[idx] = machinePos;
    (void)vars;
    return Error{};
}

const Position& CoordinateSystemManager::getG30Reference(int32_t point) const {
    int32_t idx = std::clamp(point, 1, 4) - 1;
    return m_g30References[idx];
}

// ============================================================================
// Coordinate transformation
// ============================================================================

void CoordinateSystemManager::syncTransform(const MachineState& state) {
    // WCS offset
    const Position& wcsOff = m_wcs[wcsIndex(m_activeWCS)].offset;
    m_transform.setWCSOffset({wcsOff.x(), wcsOff.y(), wcsOff.z()});

    // G92 offset (only if active)
    if (m_g92Active)
        m_transform.setG92Offset({m_g92Offset.x(), m_g92Offset.y(), m_g92Offset.z()});
    else
        m_transform.setG92Offset({0, 0, 0});

    // G52 offset
    m_transform.setG52Offset({state.g52Offset.x(), state.g52Offset.y(), state.g52Offset.z()});

    // Scaling (G51)
    if (state.g51Active) {
        m_transform.setScale(state.scaleFactors.x(), state.scaleFactors.y(), state.scaleFactors.z());
        std::array<double, 6> ext = {
            state.scaleFactors[Axis::A], state.scaleFactors[Axis::B],
            state.scaleFactors[Axis::C], state.scaleFactors[Axis::U],
            state.scaleFactors[Axis::V], state.scaleFactors[Axis::W],
        };
        m_transform.setExtendedScale(ext);
    } else {
        m_transform.clearScale();
    }

    // Rotation (G68)
    if (state.g68Active) {
        switch (state.g68Mode) {
            case 0: {
                // 2D plane rotation. The pivot is stored in g68Pivot.x()/y()
                // as the in-plane pivots (pivotA, pivotB).
                double pivotA = state.g68Pivot.x();
                double pivotB = state.g68Pivot.y();
                m_transform.setRotation2D(
                    state.coordRotation, state.plane, pivotA, pivotB);
                break;
            }
            case 1: {
                m_transform.setRotation3DEuler(
                    state.g68Euler[0], state.g68Euler[1], state.g68Euler[2],
                    {state.g68Pivot.x(), state.g68Pivot.y(), state.g68Pivot.z()});
                break;
            }
            case 2: {
                m_transform.setRotation3DAxisAngle(
                    state.g68Axis, state.g68AxisAngle,
                    {state.g68Pivot.x(), state.g68Pivot.y(), state.g68Pivot.z()});
                break;
            }
            default:
                m_transform.clearRotation();
                break;
        }
    } else {
        m_transform.clearRotation();
    }
}

Position CoordinateSystemManager::toMachineCoords(const Position& programPos) const {
    return m_transform.toMachine(programPos);
}

Position CoordinateSystemManager::toProgramCoords(const Position& machinePos) const {
    return m_transform.toProgram(machinePos);
}

Position CoordinateSystemManager::getTotalOffset() const {
    const Position& wcsOff = m_wcs[wcsIndex(m_activeWCS)].offset;
    Position total;
    for (size_t i = 0; i < MAX_AXES; ++i)
        total[i] = wcsOff[i] + (m_g92Active ? m_g92Offset[i] : 0.0) + 0.0;
    return total;
}

// ============================================================================
// Variable synchronization
// ============================================================================

void CoordinateSystemManager::syncToVariables(VariableSystem& vars) const {
    // WCS offsets: #5221-#5386
    for (size_t i = 0; i < NUM_WORK_COORD_SYSTEMS; ++i) {
        int32_t base = WCS_PARAM_BASE[i];
        vars.set(base + 0, m_wcs[i].offset.x());
        vars.set(base + 1, m_wcs[i].offset.y());
        vars.set(base + 2, m_wcs[i].offset.z());
        vars.set(base + 3, m_wcs[i].offset[Axis::A]);
        vars.set(base + 4, m_wcs[i].offset[Axis::B]);
        vars.set(base + 5, m_wcs[i].offset[Axis::C]);
    }
    // G92 offsets: #5211-#5219
    for (size_t i = 0; i < 9 && i < MAX_AXES; ++i)
        vars.set(G92_PARAM_BASE + static_cast<int32_t>(i), m_g92Offset[i]);
    // Active WCS number: #5220
    vars.set(5220, static_cast<double>(m_activeWCS - 1));
    // G28 reference: #5161-#5169
    for (size_t i = 0; i < 9 && i < MAX_AXES; ++i)
        vars.set(G28_PARAM_BASE + static_cast<int32_t>(i), m_g28Reference[i]);
}

void CoordinateSystemManager::loadFromVariables(const VariableSystem& vars) {
    for (size_t i = 0; i < NUM_WORK_COORD_SYSTEMS; ++i) {
        int32_t base = WCS_PARAM_BASE[i];
        m_wcs[i].offset.x() = vars.get(base + 0);
        m_wcs[i].offset.y() = vars.get(base + 1);
        m_wcs[i].offset.z() = vars.get(base + 2);
        m_wcs[i].offset[Axis::A] = vars.get(base + 3);
        m_wcs[i].offset[Axis::B] = vars.get(base + 4);
        m_wcs[i].offset[Axis::C] = vars.get(base + 5);
    }
    for (size_t i = 0; i < 9 && i < MAX_AXES; ++i)
        m_g92Offset[i] = vars.get(G92_PARAM_BASE + static_cast<int32_t>(i));
    m_activeWCS = static_cast<int32_t>(vars.get(5220)) + 1;
    for (size_t i = 0; i < 9 && i < MAX_AXES; ++i)
        m_g28Reference[i] = vars.get(G28_PARAM_BASE + static_cast<int32_t>(i));
}

void CoordinateSystemManager::updatePositionVariables(
    const Position& machinePos, VariableSystem& vars) const {
    for (size_t i = 0; i < 9 && i < MAX_AXES; ++i)
        vars.set(CURRENT_POS_PARAM_BASE + static_cast<int32_t>(i), machinePos[i]);
}

// ============================================================================
// Persistence
// ============================================================================

Error CoordinateSystemManager::saveToFile(const std::string& filename) const {
    std::ofstream ofs(filename);
    if (!ofs)
        return makeError(ErrorCode::FILE_NOT_FOUND, "Cannot open WCS file for writing");
    for (const auto& wcs : m_wcs) {
        ofs << wcs.number << ' ' << wcs.name << ' '
            << wcs.offset.x() << ' ' << wcs.offset.y() << ' ' << wcs.offset.z()
            << ' ' << wcs.offset[Axis::A] << ' ' << wcs.offset[Axis::B]
            << ' ' << wcs.offset[Axis::C] << '\n';
    }
    return Error{};
}

Error CoordinateSystemManager::loadFromFile(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs)
        return makeError(ErrorCode::FILE_NOT_FOUND, "Cannot open WCS file for reading");
    for (auto& wcs : m_wcs) {
        ifs >> wcs.number >> wcs.name
            >> wcs.offset.x() >> wcs.offset.y() >> wcs.offset.z()
            >> wcs.offset[Axis::A] >> wcs.offset[Axis::B] >> wcs.offset[Axis::C];
    }
    return Error{};
}

// ============================================================================
// Utility functions
// ============================================================================

int32_t gcodeToWCSNumber(double gcode) {
    if (gcode >= 54 && gcode <= 59) return static_cast<int32_t>(gcode - 53);
    if (gcode == 59.1) return 7;
    if (gcode == 59.2) return 8;
    if (gcode == 59.3) return 9;
    return 0;
}

double wcsNumberToGCode(int32_t number) {
    switch (number) {
        case 1: return 54;
        case 2: return 55;
        case 3: return 56;
        case 4: return 57;
        case 5: return 58;
        case 6: return 59;
        case 7: return 59.1;
        case 8: return 59.2;
        case 9: return 59.3;
        default: return 54;
    }
}

const char* wcsNumberToString(int32_t number) {
    switch (number) {
        case 1: return "G54";
        case 2: return "G55";
        case 3: return "G56";
        case 4: return "G57";
        case 5: return "G58";
        case 6: return "G59";
        case 7: return "G59.1";
        case 8: return "G59.2";
        case 9: return "G59.3";
        default: return "G54";
    }
}

Error processG54(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G54;
    return csm.selectWCS(1);
}
Error processG55(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G55;
    return csm.selectWCS(2);
}
Error processG56(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G56;
    return csm.selectWCS(3);
}
Error processG57(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G57;
    return csm.selectWCS(4);
}
Error processG58(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G58;
    return csm.selectWCS(5);
}
Error processG59(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G59;
    return csm.selectWCS(6);
}
Error processG59_1(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G59_1;
    return csm.selectWCS(7);
}
Error processG59_2(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G59_2;
    return csm.selectWCS(8);
}
Error processG59_3(MachineState& state, CoordinateSystemManager& csm) {
    state.coordSystem = CoordSystem::G59_3;
    return csm.selectWCS(9);
}

} // namespace GCode
