#pragma once

/// @file PrinterKinematics.hpp
/// @brief Printer kinematics type enumeration and per-kinematics configuration.
///
/// @details
/// Defines the `PrinterKinematics` enum (Cartesian, CoreXY, Delta, Polar,
/// Winch, etc.) and the configuration structs for kinematics types that
/// need extra parameters beyond the standard geometry classes
/// (`DeltaGeometry`/`RotaryDeltaGeometry` live in their respective headers).
///
/// This was previously embedded in
/// `tether/klipper/klippy/KlippyInstanceConfig.hpp` as the `Kinematics` enum
/// and inner structs of `KlippySettings`. It has been extracted into the
/// `tether_kinematics` module so that the kinematics transform layer
/// (`KinematicsTransform`) does not depend on the Klipper config layer.

#include <string>

namespace tether::kinematics {

/// @brief Printer kinematics type.
enum class PrinterKinematics {
    Cartesian,     ///< Standard Cartesian (X, Y, Z independent)
    CoreXY,        ///< CoreXY (X=A+B, Y=A-B)
    CoreXZ,        ///< CoreXZ (X=A+B, Z=A-B)
    CoreYZ,        ///< CoreYZ (Y=A+B, Z=A-B)
    HybridCoreXY,  ///< Hybrid CoreXY (X is independent, Y uses CoreXY)
    HybridCoreXZ,  ///< Hybrid CoreXZ (X is independent, Z uses CoreXZ)
    Delta,         ///< Linear Delta (3 towers)
    RotaryDelta,   ///< Rotary Delta
    Polar,         ///< Polar (R, Theta, Z)
    Winch,         ///< Cable/winch kinematics
    None,          ///< No kinematics (manual)
};

/// @brief Convert kinematics string to enum.
inline PrinterKinematics printerKinematicsFromString(const std::string& s) {
    if (s == "cartesian") return PrinterKinematics::Cartesian;
    if (s == "corexy") return PrinterKinematics::CoreXY;
    if (s == "corexz") return PrinterKinematics::CoreXZ;
    if (s == "coreyz") return PrinterKinematics::CoreYZ;
    if (s == "hybrid_corexy") return PrinterKinematics::HybridCoreXY;
    if (s == "hybrid_corexz") return PrinterKinematics::HybridCoreXZ;
    if (s == "delta") return PrinterKinematics::Delta;
    if (s == "rotary_delta") return PrinterKinematics::RotaryDelta;
    if (s == "polar") return PrinterKinematics::Polar;
    if (s == "winch") return PrinterKinematics::Winch;
    if (s == "none") return PrinterKinematics::None;
    return PrinterKinematics::Cartesian;
}

/// @brief Convert kinematics enum to string.
inline std::string printerKinematicsToString(PrinterKinematics k) {
    switch (k) {
        case PrinterKinematics::Cartesian:    return "cartesian";
        case PrinterKinematics::CoreXY:       return "corexy";
        case PrinterKinematics::CoreXZ:       return "corexz";
        case PrinterKinematics::CoreYZ:       return "coreyz";
        case PrinterKinematics::HybridCoreXY: return "hybrid_corexy";
        case PrinterKinematics::HybridCoreXZ: return "hybrid_corexz";
        case PrinterKinematics::Delta:        return "delta";
        case PrinterKinematics::RotaryDelta:  return "rotary_delta";
        case PrinterKinematics::Polar:        return "polar";
        case PrinterKinematics::Winch:        return "winch";
        case PrinterKinematics::None:         return "none";
    }
    return "cartesian";
}

/// @brief Polar printer configuration.
struct PolarConfig {
    double maxRadius = 200.0;        ///< Maximum radius (mm)
    double maxAngle = 360.0;         ///< Maximum angle (degrees, 0=continuous)
    bool continuousRotation = false; ///< If true, angle is continuous
};

/// @brief Winch/cable printer configuration.
struct WinchConfig {
    double anchorRadius = 500.0; ///< Distance from center to each anchor (mm)
    double anchorHeight = 300.0; ///< Height of anchors above bed (mm)
    int anchorCount = 3;         ///< Number of anchors (typically 3)
};

} // namespace tether::kinematics
