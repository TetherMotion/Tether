#pragma once

/// @file KinematicsTransform.hpp
/// @brief Kinematics transform: converts Cartesian (X,Y,Z) to stepper positions
///        and back, for all supported printer kinematics types.
///
/// @details
/// This class was previously embedded in
/// `tether/klipper/motion/MotionTranslator.hpp`. It has been extracted into
/// the `tether_kinematics` module so that the kinematics conversion layer
/// is independent of the Klipper step-generation machinery.
///
/// For Cartesian: stepper = cartesian (identity).
/// For CoreXY: A = X+Y, B = X-Y.
/// For CoreXZ: A = X+Z, B = X-Z.
/// For Delta: uses DeltaPrinter::forwardActuatorKinematics.
/// For RotaryDelta: uses RotaryDeltaPrinter::forwardActuatorKinematics.

#include "tether/kinematics/PrinterKinematics.hpp"
#include "tether/kinematics/DeltaPrinter.hpp"
#include "tether/kinematics/RotaryDeltaPrinter.hpp"

#include <array>
#include <cmath>

namespace tether::kinematics {

/// @brief Kinematics transform: converts Cartesian (X,Y,Z) to stepper positions.
///
/// For Cartesian: stepper = cartesian (identity).
/// For CoreXY: A = X+Y, B = X-Y.
/// For CoreXZ: A = X+Z, B = X-Z.
/// For Delta: uses DeltaPrinter::forwardActuatorKinematics.
class KinematicsTransform {
public:
    /// @brief Set the kinematics type.
    void setKinematics(PrinterKinematics k) { kinematics_ = k; }

    /// @brief Get the kinematics type.
    PrinterKinematics kinematics() const { return kinematics_; }

    /// @brief Set the delta printer (for delta kinematics).
    void setDeltaPrinter(const DeltaPrinter* dp) { deltaPrinter_ = dp; }

    /// @brief Set the rotary delta printer (for rotary delta kinematics).
    void setRotaryDeltaPrinter(const RotaryDeltaPrinter* rdp) {
        rotaryDeltaPrinter_ = rdp;
    }

    /// @brief Set winch anchor parameters (for winch kinematics).
    void setWinchParams(double anchorRadius, double anchorHeight) {
        winchAnchorRadius_ = anchorRadius;
        winchAnchorHeight_ = anchorHeight;
    }

    /// @brief Transform a Cartesian position to stepper-space positions.
    ///
    /// @param x, y, z Cartesian position in mm.
    /// @return Array of 3 stepper positions (A, B, C) in mm.
    std::array<double, 3> forwardActuatorKinematics(double x, double y, double z) const {
        switch (kinematics_) {
            case PrinterKinematics::CoreXY:
                // A = X + Y, B = X - Y, C = Z
                return {x + y, x - y, z};
            case PrinterKinematics::CoreXZ:
                // A = X + Z, B = X - Z, C = Y
                return {x + z, x - z, y};
            case PrinterKinematics::CoreYZ:
                // A = Y + Z, B = Y - Z, C = X
                return {y + z, y - z, x};
            case PrinterKinematics::HybridCoreXY:
                // HybridCoreXY: X/Y use CoreXY belts, Z is independent (leadscrew)
                // A = X + Y, B = X - Y, C = Z
                return {x + y, x - y, z};
            case PrinterKinematics::HybridCoreXZ:
                // HybridCoreXZ: X/Z use CoreXZ belts, Y is independent
                // A = X + Z, B = X - Z, C = Y
                return {x + z, x - z, y};
            case PrinterKinematics::Delta:
                if (deltaPrinter_) {
                    return deltaPrinter_->forwardActuatorKinematics(x, y, z);
                }
                return {x, y, z};
            case PrinterKinematics::RotaryDelta: {
                // Rotary delta: three upper arms at 120° spacing.
                // Stepper positions are shoulder angles (radians).
                if (rotaryDeltaPrinter_) {
                    return rotaryDeltaPrinter_->forwardActuatorKinematics(x, y, z);
                }
                // Fallback: no geometry configured, return identity.
                return {x, y, z};
            }
            case PrinterKinematics::Polar: {
                // Polar: A = radius, B = angle (degrees), C = Z
                double radius = std::sqrt(x*x + y*y);
                double angle = std::atan2(y, x) * 180.0 / M_PI;
                return {radius, angle, z};
            }
            case PrinterKinematics::Winch: {
                // Winch/cable: A/B/C are cable lengths from three anchors.
                // Anchors at fixed positions (simplified: equilateral triangle).
                double anchorR = winchAnchorRadius_;
                double az1 = 0.0, az2 = 2.0*M_PI/3.0, az3 = 4.0*M_PI/3.0;
                double a1x = anchorR * std::cos(az1), a1y = anchorR * std::sin(az1);
                double a2x = anchorR * std::cos(az2), a2y = anchorR * std::sin(az2);
                double a3x = anchorR * std::cos(az3), a3y = anchorR * std::sin(az3);
                double h = winchAnchorHeight_;
                double la = std::sqrt((x-a1x)*(x-a1x) + (y-a1y)*(y-a1y) + (z-h)*(z-h));
                double lb = std::sqrt((x-a2x)*(x-a2x) + (y-a2y)*(y-a2y) + (z-h)*(z-h));
                double lc = std::sqrt((x-a3x)*(x-a3x) + (y-a3y)*(y-a3y) + (z-h)*(z-h));
                return {la, lb, lc};
            }
            case PrinterKinematics::Cartesian:
            case PrinterKinematics::None:
            default:
                return {x, y, z};
        }
    }

    /// @brief Transform stepper-space positions back to Cartesian.
    ///
    /// @param a, b, c Stepper positions in mm.
    /// @return Array of 3 Cartesian positions (X, Y, Z) in mm.
    std::array<double, 3> inverseActuatorKinematics(double a, double b, double c) const {
        switch (kinematics_) {
            case PrinterKinematics::CoreXY:
                // X = (A + B) / 2, Y = (A - B) / 2, Z = C
                return {(a + b) / 2.0, (a - b) / 2.0, c};
            case PrinterKinematics::CoreXZ:
                // X = (A + B) / 2, Y = C, Z = (A - B) / 2
                return {(a + b) / 2.0, c, (a - b) / 2.0};
            case PrinterKinematics::CoreYZ:
                // X = C, Y = (A + B) / 2, Z = (A - B) / 2
                return {c, (a + b) / 2.0, (a - b) / 2.0};
            case PrinterKinematics::Delta:
                if (deltaPrinter_) {
                    return deltaPrinter_->inverseActuatorKinematics(a, b, c);
                }
                return {a, b, c};
            case PrinterKinematics::HybridCoreXY:
                // X = (A + B) / 2, Y = (A - B) / 2, Z = C
                return {(a + b) / 2.0, (a - b) / 2.0, c};
            case PrinterKinematics::HybridCoreXZ:
                // X = (A + B) / 2, Y = C, Z = (A - B) / 2
                return {(a + b) / 2.0, c, (a - b) / 2.0};
            case PrinterKinematics::RotaryDelta: {
                // Inverse rotary delta: from shoulder angles to Cartesian
                if (rotaryDeltaPrinter_) {
                    return rotaryDeltaPrinter_->inverseActuatorKinematics(a, b, c);
                }
                return {a, b, c};
            }
            case PrinterKinematics::Polar: {
                // Inverse polar: A = radius, B = angle (degrees), C = Z
                double rad = a * M_PI / 180.0;
                return {b * std::cos(rad), b * std::sin(rad), c};
            }
            case PrinterKinematics::Winch: {
                // Inverse winch: trilateration from three cable lengths
                // Simplified: anchors at equilateral triangle, height h
                double anchorR = winchAnchorRadius_;
                double h = winchAnchorHeight_;
                double a1x = anchorR, a1y = 0.0;
                double a2x = anchorR * std::cos(2.0*M_PI/3.0);
                double a2y = anchorR * std::sin(2.0*M_PI/3.0);
                // Using first two anchors to estimate XY
                double da = a*a - b*b;
                double dx = da / (2.0 * (a1x - a2x));
                double dy = (a*a - dx*dx - (dx - a1x)*(dx - a1x) +
                            a2x*a2x + a2y*a2y - 2.0*dx*a2x) / (2.0 * a2y);
                double dz = std::sqrt(std::max(0.0, a*a - (dx-a1x)*(dx-a1x) - (dy-a1y)*(dy-a1y)));
                return {dx, dy, dz - h};
            }
            case PrinterKinematics::Cartesian:
            case PrinterKinematics::None:
            default:
                return {a, b, c};
        }
    }

private:
    PrinterKinematics kinematics_ = PrinterKinematics::Cartesian;
    const DeltaPrinter* deltaPrinter_ = nullptr;
    const RotaryDeltaPrinter* rotaryDeltaPrinter_ = nullptr;
    double winchAnchorRadius_ = 500.0;   ///< Winch anchor radius (mm)
    double winchAnchorHeight_ = 300.0;   ///< Winch anchor height (mm)
};

} // namespace tether::kinematics
