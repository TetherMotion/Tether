#pragma once

/// @file PrinterMotionState.hpp
/// @brief Printer motion state tracked by the G-code executor
///
/// @details
/// Modal state fields (units, distance mode, plane) now use the shared
/// strongly-typed enums from tether::gcode, eliminating duplication of
/// G20/G21/G90/G91/G17/G18/G19 semantics between the Klipper and main
/// Tether G-code interpreters. Backward-compatible bool accessors are
/// provided so existing consumers don't need to change.

#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/motion/CoordinateTransform.hpp"

#include <array>
#include <map>
#include <string>

namespace tether::klipper::klippy {

// ============================================================================
// Printer state (shared across G-code execution)
// ============================================================================

/// @brief Printer state tracked by the G-code executor.
struct PrinterMotionState {
    std::array<double, 4> position = {0, 0, 0, 0}; ///< X, Y, Z, E (mm)

    /// @brief Distance mode (G90/G91) — shared with GCode.
    GCode::DistanceMode distanceMode = GCode::DistanceMode::ABSOLUTE;

    /// @brief Units (G20/G21) — shared with GCode.
    GCode::Units units = GCode::Units::MM;

    /// @brief Arc plane (G17/G18/G19) — shared with GCode.
    GCode::Plane plane = GCode::Plane::XY;

    bool absoluteExtrude = false;    ///< M82/M83 extruder distance mode
    double feedrate = 1500.0;       ///< mm/min
    double extrudeFactor = 1.0;     ///< E scaling
    double speedFactor = 1.0;       ///< F scaling
    std::string homedAxes;          ///< e.g. "xyz"
    std::string activeExtruder = "extruder";
    std::map<std::string, bool> motorEnabled = {
        {"x", true}, {"y", true}, {"z", true}, {"e", true}
    };

    // --- Backward-compatible bool accessors ---

    /// @brief True if absolute coordinates (G90). Backward compat.
    bool absoluteCoordinates() const {
        return distanceMode == GCode::DistanceMode::ABSOLUTE;
    }
    /// @brief Set absolute/relative mode. Backward compat.
    void setAbsoluteCoordinates(bool abs) {
        distanceMode = abs ? GCode::DistanceMode::ABSOLUTE
                           : GCode::DistanceMode::INCREMENTAL;
    }

    /// @brief True if units are inches (G20). Backward compat.
    bool unitsInches() const {
        return units == GCode::Units::INCH;
    }
    /// @brief Set units to inches/mm. Backward compat.
    void setUnitsInches(bool inches) {
        units = inches ? GCode::Units::INCH
                       : GCode::Units::MM;
    }

    /// @brief Arc plane as int (0=XY, 1=XZ, 2=YZ). Backward compat.
    int arcPlane() const {
        switch (plane) {
            case GCode::Plane::XY: return 0;
            case GCode::Plane::ZX: return 1;
            case GCode::Plane::YZ: return 2;
            default: return 0;
        }
    }
    /// @brief Set arc plane from int. Backward compat.
    void setArcPlane(int p) {
        switch (p) {
            case 1:  plane = GCode::Plane::ZX; break;
            case 2:  plane = GCode::Plane::YZ; break;
            default: plane = GCode::Plane::XY; break;
        }
    }

    // Coordinate systems (G54-G59.3)
    int activeCoordSystem = 0;      ///< 0=G54, ..., 8=G59.3
    std::array<std::array<double, 3>, 9> coordSystemOffsets = {};  ///< Per-system X/Y/Z offsets

    // G52 local coordinate offset
    std::array<double, 3> g52Offset = {0, 0, 0};

    // G68 coordinate rotation
    bool g68Active = false;                 ///< G68 rotation active
    int  g68Mode = 0;                       ///< 0=2D plane, 1=Euler XYZ, 2=axis-angle
    double coordRotation = 0.0;             ///< 2D rotation angle (degrees)
    std::array<double, 3> g68Pivot = {0, 0, 0};     ///< Rotation pivot
    std::array<double, 3> g68Euler = {0, 0, 0};     ///< A/B/C Euler angles (degrees)
    std::array<double, 3> g68Axis = {0, 0, 0};      ///< I/J/K rotation axis
    double g68AxisAngle = 0.0;              ///< Axis-angle R (degrees)

    // G51 scaling
    bool g51Active = false;                 ///< G51 scaling active
    std::array<double, 3> scaleFactors = {1, 1, 1};  ///< Per-axis XYZ scale

    /// Composed coordinate transform (scale + rotation + WCS/G52/G92 offset).
    /// Rebuilt by rebuildCoordTransform() whenever any of the above change.
    GCode::CoordinateTransform coordTransform;

    /// @brief Rebuild coordTransform from the current state fields.
    void rebuildCoordTransform() {
        coordTransform.setWCSOffset(coordSystemOffsets[activeCoordSystem]);
        coordTransform.setG52Offset(g52Offset);
        // G92 is handled via setPosition in the Klipper path; keep zero here.
        coordTransform.setG92Offset({0, 0, 0});
        if (g51Active)
            coordTransform.setScale(scaleFactors[0], scaleFactors[1], scaleFactors[2]);
        else
            coordTransform.clearScale();
        if (g68Active) {
            switch (g68Mode) {
                case 0:
                    coordTransform.setRotation2D(
                        coordRotation, plane, g68Pivot[0], g68Pivot[1]);
                    break;
                case 1:
                    coordTransform.setRotation3DEuler(
                        g68Euler[0], g68Euler[1], g68Euler[2], g68Pivot);
                    break;
                case 2:
                    coordTransform.setRotation3DAxisAngle(
                        g68Axis, g68AxisAngle, g68Pivot);
                    break;
            }
        } else {
            coordTransform.clearRotation();
        }
    }

    // Path control
    int pathControlMode = 0;        ///< 0=exact stop, 1=exact path, 2=blending
    double pathBlendingTolerance = 0.0;

    // Spindle
    double spindleRpm = 0.0;        ///< Current spindle RPM (M3/M4/M5)

    // Coolant
    bool coolantFlood = false;
    bool coolantMist = false;

    // Software endstops
    bool softwareEndstopsEnabled = true;
    std::map<std::string, std::array<double, 2>> softwareEndstopLimits; ///< axis -> {min, max}

    // Canned cycle state
    bool cannedCycleActive = false;
    double cannedCycleRetractHeight = 0.0;
    double cannedCycleFeedRate = 0.0;

    // Filament width sensor
    bool filamentWidthSensorEnabled = false;
    double filamentWidthMeasured = 1.75;  ///< mm
};

} // namespace tether::klipper::klippy
