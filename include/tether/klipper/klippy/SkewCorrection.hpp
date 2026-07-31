#pragma once

/// @file SkewCorrection.hpp
/// @brief Skew correction manager

#include <cmath>
#include <array>

namespace tether::klipper::klippy {

/// @brief Skew correction parameters.
struct SkewParams {
    double xy = 0.0;  ///< XY skew angle (degrees)
    double xz = 0.0;  ///< XZ skew angle (degrees)
    double yz = 0.0;  ///< YZ skew angle (degrees)
};

/// @brief Skew correction manager.
class SkewCorrection {
public:
    /// @brief Set skew parameters (M852).
    void setParams(const SkewParams& params) { params_ = params; }

    /// @brief Get current skew parameters.
    const SkewParams& params() const { return params_; }

    /// @brief Apply skew correction to a position.
    /// @param x, y, z Raw position.
    /// @return Corrected position.
    std::array<double, 3> correct(double x, double y, double z) const {
        // Apply skew correction matrix
        // For small angles, the correction is approximately:
        // x' = x - y * tan(xy) - z * tan(xz)
        // y' = y - z * tan(yz)
        // z' = z
        double xyRad = params_.xy * M_PI / 180.0;
        double xzRad = params_.xz * M_PI / 180.0;
        double yzRad = params_.yz * M_PI / 180.0;

        double correctedY = y - z * std::tan(yzRad);
        double correctedX = x - correctedY * std::tan(xyRad) - z * std::tan(xzRad);
        double correctedZ = z;

        return {correctedX, correctedY, correctedZ};
    }

    /// @brief Check if skew correction is active.
    bool isActive() const {
        return params_.xy != 0.0 || params_.xz != 0.0 || params_.yz != 0.0;
    }

private:
    SkewParams params_;
};

} // namespace tether::klipper::klippy
