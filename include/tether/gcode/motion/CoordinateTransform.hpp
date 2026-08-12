/**
 * @file CoordinateTransform.hpp
 * @brief Composed coordinate transform: Scale -> Rotate -> Translate (WCS + G52 + G92)
 *
 * @details
 * ## Transform Model
 *
 * The transform converts *program coordinates* (what the G-code programmer
 * writes) into *machine coordinates* (absolute stepper space, before
 * printer kinematics such as CoreXY/Delta).
 *
 * The composition order is (RS274/LinuxCNC compliant):
 *
 * ```
 * P_machine = T_g52 + T_tlo + T_wcs + T(pivot) * R * T(-pivot) * S * (P_program + T_g92)
 * ```
 *
 * Where:
 * - `S`        = per-axis scale matrix (G51), diagonal `diag(sx, sy, sz)`
 * - `R`        = rotation (G68): 2D in the active plane by default, or full
 *                3D (intrinsic XYZ Euler angles A/B/C, or axis-angle I/J/K + R)
 * - `pivot`    = rotation pivot point (specified in program coordinates)
 * - `T_g92`    = G92 basic offset (program space, applied before scale/rotate)
 * - `T_wcs`    = active work coordinate system offset (G54-G59.3)
 * - `T_tlo`    = tool length offset (G43/G43.1, Z-axis only, after WCS)
 * - `T_g52`    = G52 local offset (applied after WCS+TLO, per RS274)
 *
 * G53 (machine coordinates) bypasses the entire transform.
 *
 * ## Axis Handling
 *
 * - **XYZ** (indices 0-2): full transform (scale + rotate + translate)
 * - **ABC** (indices 3-5): per-axis scale only (angular/rotary axes are not
 *   rotated or translated — they represent spindle/rotary-table positions)
 * - **UVW** (indices 6-8): per-axis scale only in 2D rotation mode; follow
 *   the XYZ rotation in 3D mode (secondary linear axes parallel to XYZ)
 * - **E** (extruder): not part of `Position`; the Klipper move path passes E
 *   through unchanged alongside the transformed XYZ
 *
 * The class uses `Eigen::Affine3d` (double precision, 4x4 homogeneous
 * coordinates), consistent with the G-code/printer layer convention.
 * The forward and inverse matrices are cached and recomputed only when a
 * parameter changes.
 *
 * @see GCodeCoordinates.hpp for the G54-G59.3 / G52 / G92 / G10 parameter
 *      management that feeds into this transform.
 */

#pragma once

#include "../GCodeTypes.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>

namespace GCode {

// ============================================================================
// Rotation mode
// ============================================================================

/// @brief How G68 rotation is specified.
enum class RotationMode : uint8_t {
    NONE = 0,       ///< No rotation (G69 or default)
    PLANE_2D = 1,   ///< 2D rotation in the active plane (G17=XY about Z, etc.)
    EULER_XYZ = 2,  ///< 3D intrinsic XYZ Euler angles (A, B, C words)
    AXIS_ANGLE = 3, ///< 3D axis-angle (I, J, K axis + R angle)
};

// ============================================================================
// CoordinateTransform
// ============================================================================

/**
 * @brief Composed coordinate transform (scale + rotation + translation)
 *
 * Owns the cached forward/inverse affine matrices and exposes
 * `toMachine()` / `toProgram()` for positions and velocity vectors.
 *
 * Thread safety: not thread-safe. The G-code executor is single-threaded;
 * the motion dispatcher reads the transform from a single owner.
 */
class CoordinateTransform {
public:
    CoordinateTransform() { recompute(); }

    // ------------------------------------------------------------------
    // Configuration setters — each triggers a cache recompute
    // ------------------------------------------------------------------

    /// @brief Set the active work coordinate system offset (G54-G59.3).
    void setWCSOffset(const std::array<double, 3>& offset) {
        m_wcsOffset = offset;
        recompute();
    }

    /// @brief Set the G92 basic offset (program space).
    void setG92Offset(const std::array<double, 3>& offset) {
        m_g92Offset = offset;
        recompute();
    }

    /// @brief Set the G52 local offset (applied after WCS, per RS274).
    void setG52Offset(const std::array<double, 3>& offset) {
        m_g52Offset = offset;
        recompute();
    }

    /// @brief Set tool length offset (G43/G43.1). Z-axis only.
    /// Applied after WCS offset, before G52.
    void setToolLengthOffset(double zOffset) {
        m_toolLengthOffset = zOffset;
        recompute();
    }

    /// @brief Clear tool length offset (G49).
    void clearToolLengthOffset() {
        m_toolLengthOffset = 0.0;
        recompute();
    }

    /// @brief Set per-axis scale factors (G51). XYZ only; ABC/UVW scale is
    ///        set via @ref setExtendedScale.
    void setScale(double sx, double sy, double sz) {
        m_scale = {sx, sy, sz};
        recompute();
    }

    /// @brief Set scale factors for the extended axes (A, B, C, U, V, W).
    ///        These are applied as per-axis scaling without rotation.
    void setExtendedScale(const std::array<double, 6>& s) {
        m_extScale = s;
        // No 4x4 recompute needed — extended scale is applied separately.
    }

    /// @brief Clear the scale (reset to unity).
    void clearScale() {
        m_scale = {1.0, 1.0, 1.0};
        m_extScale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        recompute();
    }

    /// @brief Set 2D rotation in the active plane (G68 with R word only).
    /// @param angleDeg Rotation angle in degrees.
    /// @param plane    Active plane (G17=XY, G18=ZX, G19=YZ).
    /// @param pivotX   Pivot X coordinate (program space).
    /// @param pivotY   Pivot Y coordinate (program space). For G18/G19 this
    ///                 is the second in-plane axis.
    void setRotation2D(double angleDeg, Plane plane,
                        double pivotA, double pivotB) {
        m_rotationMode = RotationMode::PLANE_2D;
        m_plane = plane;
        m_angleDeg = angleDeg;
        m_eulerDeg = {0.0, 0.0, 0.0};
        m_axis = {0.0, 0.0, 0.0};
        // Map the 2D pivot into XYZ space.
        switch (plane) {
            case Plane::XY: m_pivot = {pivotA, pivotB, 0.0}; break;
            case Plane::ZX: m_pivot = {pivotB, 0.0, pivotA}; break;
            case Plane::YZ: m_pivot = {0.0, pivotB, pivotA}; break;
            default:        m_pivot = {pivotA, pivotB, 0.0}; break;
        }
        recompute();
    }

    /// @brief Set 3D rotation via intrinsic XYZ Euler angles (G68 A/B/C).
    void setRotation3DEuler(double aDeg, double bDeg, double cDeg,
                             const std::array<double, 3>& pivot) {
        m_rotationMode = RotationMode::EULER_XYZ;
        m_eulerDeg = {aDeg, bDeg, cDeg};
        m_angleDeg = 0.0;
        m_axis = {0.0, 0.0, 0.0};
        m_pivot = pivot;
        recompute();
    }

    /// @brief Set 3D rotation via axis-angle (G68 I/J/K + R).
    void setRotation3DAxisAngle(const std::array<double, 3>& axis,
                                 double angleDeg,
                                 const std::array<double, 3>& pivot) {
        m_rotationMode = RotationMode::AXIS_ANGLE;
        m_axis = axis;
        m_angleDeg = angleDeg;
        m_eulerDeg = {0.0, 0.0, 0.0};
        m_pivot = pivot;
        recompute();
    }

    /// @brief Cancel rotation (G69).
    void clearRotation() {
        m_rotationMode = RotationMode::NONE;
        m_angleDeg = 0.0;
        m_eulerDeg = {0.0, 0.0, 0.0};
        m_axis = {0.0, 0.0, 0.0};
        m_pivot = {0.0, 0.0, 0.0};
        recompute();
    }

    /// @brief Reset everything to identity (no offsets, no rotation, no scale).
    void reset() {
        m_wcsOffset = {0.0, 0.0, 0.0};
        m_g92Offset = {0.0, 0.0, 0.0};
        m_g52Offset = {0.0, 0.0, 0.0};
        m_toolLengthOffset = 0.0;
        m_scale = {1.0, 1.0, 1.0};
        m_extScale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        m_rotationMode = RotationMode::NONE;
        m_angleDeg = 0.0;
        m_eulerDeg = {0.0, 0.0, 0.0};
        m_axis = {0.0, 0.0, 0.0};
        m_pivot = {0.0, 0.0, 0.0};
        recompute();
    }

    // ------------------------------------------------------------------
    // Queries
    // ------------------------------------------------------------------

    /// @brief True if the transform is identity (no offset, rotation, or scale).
    bool isIdentity() const {
        return m_wcsOffset == std::array<double,3>{0,0,0}
            && m_g92Offset == std::array<double,3>{0,0,0}
            && m_g52Offset == std::array<double,3>{0,0,0}
            && m_toolLengthOffset == 0.0
            && m_scale == std::array<double,3>{1,1,1}
            && m_extScale == std::array<double,6>{1,1,1,1,1,1}
            && m_rotationMode == RotationMode::NONE;
    }

    RotationMode rotationMode() const { return m_rotationMode; }
    Plane plane() const { return m_plane; }
    double angle() const { return m_angleDeg; }
    const std::array<double,3>& euler() const { return m_eulerDeg; }
    const std::array<double,3>& axis() const { return m_axis; }
    const std::array<double,3>& pivot() const { return m_pivot; }
    const std::array<double,3>& scale() const { return m_scale; }
    const std::array<double,6>& extendedScale() const { return m_extScale; }
    const std::array<double,3>& wcsOffset() const { return m_wcsOffset; }
    const std::array<double,3>& g92Offset() const { return m_g92Offset; }
    const std::array<double,3>& g52Offset() const { return m_g52Offset; }
    double toolLengthOffset() const { return m_toolLengthOffset; }

    // ------------------------------------------------------------------
    // Forward transform: program -> machine
    // ------------------------------------------------------------------

    /// @brief Transform XYZ program coordinates to machine coordinates.
    /// @note E (extruder) is NOT part of this call; the caller passes E
    ///       through unchanged.
    std::array<double, 3> toMachineXYZ(double x, double y, double z) const {
        Eigen::Vector4d p(x, y, z, 1.0);
        Eigen::Vector4d r = m_forward * p;
        return {r.x(), r.y(), r.z()};
    }

    /// @brief Transform a full 9-axis Position (program -> machine).
    /// XYZ go through the 4x4 affine; ABC and UVW get per-axis scaling.
    Position toMachine(const Position& programPos) const {
        Position out;
        // XYZ: full affine transform
        auto xyz = toMachineXYZ(programPos.x(), programPos.y(), programPos.z());
        out.x() = xyz[0];
        out.y() = xyz[1];
        out.z() = xyz[2];
        // ABC (3-5): scale only
        for (size_t i = 0; i < 3; ++i)
            out[3 + i] = programPos[3 + i] * m_extScale[i];
        // UVW (6-8): scale only (2D mode) or follow XYZ rotation (3D mode)
        for (size_t i = 0; i < 3; ++i)
            out[6 + i] = programPos[6 + i] * m_extScale[3 + i];
        return out;
    }

    // ------------------------------------------------------------------
    // Inverse transform: machine -> program
    // ------------------------------------------------------------------

    /// @brief Inverse-transform XYZ machine coordinates to program coordinates.
    std::array<double, 3> toProgramXYZ(double x, double y, double z) const {
        Eigen::Vector4d p(x, y, z, 1.0);
        Eigen::Vector4d r = m_inverse * p;
        return {r.x(), r.y(), r.z()};
    }

    /// @brief Inverse-transform a full 9-axis Position (machine -> program).
    Position toProgram(const Position& machinePos) const {
        Position out;
        auto xyz = toProgramXYZ(machinePos.x(), machinePos.y(), machinePos.z());
        out.x() = xyz[0];
        out.y() = xyz[1];
        out.z() = xyz[2];
        for (size_t i = 0; i < 3; ++i)
            out[3 + i] = (m_extScale[i] != 0.0)
                         ? machinePos[3 + i] / m_extScale[i]
                         : machinePos[3 + i];
        for (size_t i = 0; i < 3; ++i)
            out[6 + i] = (m_extScale[3 + i] != 0.0)
                         ? machinePos[6 + i] / m_extScale[3 + i]
                         : machinePos[6 + i];
        return out;
    }

    // ------------------------------------------------------------------
    // Velocity transform (rotation + scale only, no translation)
    // ------------------------------------------------------------------

    /// @brief Transform an XYZ velocity vector (no translation applied).
    /// @note The pivot translation cancels for direction vectors, so this
    ///       is simply `R * S * v`.
    std::array<double, 3> transformVelocity(double vx, double vy, double vz) const {
        Eigen::Vector3d v(vx, vy, vz);
        Eigen::Vector3d r = m_linear * v;
        return {r.x(), r.y(), r.z()};
    }

private:
    // ------------------------------------------------------------------
    // Cached matrices
    // ------------------------------------------------------------------
    Eigen::Affine3d m_forward{Eigen::Affine3d::Identity()};
    Eigen::Affine3d m_inverse{Eigen::Affine3d::Identity()};
    /// Linear part only (rotation * scale), used for velocity transforms.
    Eigen::Matrix3d m_linear{Eigen::Matrix3d::Identity()};

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    std::array<double, 3> m_wcsOffset{0, 0, 0};
    std::array<double, 3> m_g92Offset{0, 0, 0};
    std::array<double, 3> m_g52Offset{0, 0, 0};
    double m_toolLengthOffset{0.0};
    std::array<double, 3> m_scale{1, 1, 1};
    std::array<double, 6> m_extScale{1, 1, 1, 1, 1, 1};

    RotationMode m_rotationMode{RotationMode::NONE};
    Plane m_plane{Plane::XY};
    double m_angleDeg{0.0};
    std::array<double, 3> m_eulerDeg{0, 0, 0};
    std::array<double, 3> m_axis{0, 0, 0};
    std::array<double, 3> m_pivot{0, 0, 0};

    // ------------------------------------------------------------------
    // Recompute the cached matrices from the current parameters.
    // ------------------------------------------------------------------
    void recompute() {
        // Build the forward transform (RS274/LinuxCNC compliant order):
        //   M = T(g52) * T(tlo) * T(wcs) * T(pivot) * R * T(-pivot) * S * T(g92)
        //
        // Applied to a point P (rightmost first):
        //   1. T(g92)   — G92 offset (program space, before scale/rotate)
        //   2. S        — per-axis scaling
        //   3. T(-pivot)— translate to rotation pivot
        //   4. R        — rotate
        //   5. T(pivot) — translate back from pivot
        //   6. T(wcs)   — WCS offset (G54-G59.3)
        //   7. T(tlo)   — tool length offset (G43, Z-axis only)
        //   8. T(g52)   — G52 local offset (after WCS, per RS274)

        namespace Eg = Eigen;

        // Start with identity and compose from innermost (g92) outward.
        Eg::Affine3d m = Eg::Affine3d::Identity();

        // Innermost: T(g92) — translate in program space.
        m = Eg::Translation3d(
            m_g92Offset[0], m_g92Offset[1], m_g92Offset[2]) * m;

        // S — per-axis scaling.
        m = Eg::Scaling(m_scale[0], m_scale[1], m_scale[2]) * m;

        // T(-pivot) before rotation.
        m = Eg::Translation3d(-m_pivot[0], -m_pivot[1], -m_pivot[2]) * m;

        // R — rotation.
        m = buildRotation() * m;

        // T(pivot) after rotation.
        m = Eg::Translation3d(m_pivot[0], m_pivot[1], m_pivot[2]) * m;

        // T(wcs) — WCS offset to machine space.
        m = Eg::Translation3d(
            m_wcsOffset[0], m_wcsOffset[1], m_wcsOffset[2]) * m;

        // T(tlo) — tool length offset (Z-axis only, after WCS).
        m = Eg::Translation3d(0.0, 0.0, m_toolLengthOffset) * m;

        // T(g52) — G52 local offset (outermost, after WCS+TLO).
        m = Eg::Translation3d(
            m_g52Offset[0], m_g52Offset[1], m_g52Offset[2]) * m;

        m_forward = m;
        m_inverse = m_forward.inverse();
        m_linear  = m_forward.linear();
    }

    /// @brief Build the rotation Affine3d from the current rotation mode.
    Eigen::Affine3d buildRotation() const {
        namespace Eg = Eigen;
        constexpr double deg2rad = 3.14159265358979323846 / 180.0;

        switch (m_rotationMode) {
            case RotationMode::NONE:
                return Eg::Affine3d::Identity();

            case RotationMode::PLANE_2D: {
                // 2D rotation in the active plane.
                // G17 (XY): rotate about Z
                // G18 (ZX): rotate about Y
                // G19 (YZ): rotate about X
                const double a = m_angleDeg * deg2rad;
                switch (m_plane) {
                    case Plane::XY:
                        return Eg::Affine3d(Eg::AngleAxisd(a, Eg::Vector3d::UnitZ()));
                    case Plane::ZX:
                        return Eg::Affine3d(Eg::AngleAxisd(a, Eg::Vector3d::UnitY()));
                    case Plane::YZ:
                        return Eg::Affine3d(Eg::AngleAxisd(a, Eg::Vector3d::UnitX()));
                    default:
                        return Eg::Affine3d(Eg::AngleAxisd(a, Eg::Vector3d::UnitZ()));
                }
            }

            case RotationMode::EULER_XYZ: {
                // Intrinsic XYZ Euler angles: first rotate about X (A),
                // then Y (B), then Z (C). In Eigen, intrinsic rotations
                // are composed as R = Rz * Ry * Rx (applied right-to-left
                // to a vector, i.e. Rx first).
                const double a = m_eulerDeg[0] * deg2rad;
                const double b = m_eulerDeg[1] * deg2rad;
                const double c = m_eulerDeg[2] * deg2rad;
                Eg::AngleAxisd rx(a, Eg::Vector3d::UnitX());
                Eg::AngleAxisd ry(b, Eg::Vector3d::UnitY());
                Eg::AngleAxisd rz(c, Eg::Vector3d::UnitZ());
                return Eg::Affine3d(rz * ry * rx);
            }

            case RotationMode::AXIS_ANGLE: {
                // Axis-angle: rotate by m_angleDeg about the (normalized) axis.
                Eg::Vector3d axis(m_axis[0], m_axis[1], m_axis[2]);
                double norm = axis.norm();
                if (norm < 1e-12)
                    return Eg::Affine3d::Identity();
                axis /= norm;
                const double a = m_angleDeg * deg2rad;
                return Eg::Affine3d(Eg::AngleAxisd(a, axis));
            }
        }
        return Eg::Affine3d::Identity();
    }
};

} // namespace GCode
