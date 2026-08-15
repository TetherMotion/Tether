/**
 * @file PressureFlowLut.hpp
 * @brief 2-D {Q, T} → P lookup table with bilinear interpolation.
 *
 * @details
 * CrossWlfRheology::pressureFromFlow is expensive (quadrature + bisection),
 * so for real-time use we precompute a 2-D table of pressure as a function
 * of (flow Q, temperature T) and interpolate bilinearly at runtime.
 *
 * The table is built offline (PressureFlowLut::build) from CrossWlfRheology
 * and is serializable to/from a flat (axes + values) representation so users
 * can ship or regenerate tables without re-running the rheology solver.
 *
 * Storage:
 *   - flowAxis_:   ascending Q grid points [mm³/s]
 *   - tempAxis_:   ascending T grid points [°C]
 *   - values_:     row-major (T-major) pressure values [Pa],
 *                  size = tempAxis_.size() * flowAxis_.size(),
 *                  index = tIdx * flowAxis_.size() + qIdx
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md §3
 */

#pragma once

#include "tether/control/extrusion/CrossWlfRheology.hpp"

#include <vector>

namespace tether::control::extrusion {

/// @brief 2-D {Q, T} → P lookup table with bilinear interpolation.
class PressureFlowLut {
public:
    PressureFlowLut() = default;

    /// @brief Build the LUT from a Cross-WLF model over the given axes.
    /// @param params Cross-WLF parameters.
    /// @param geom Nozzle geometry.
    /// @param flowAxis Ascending flow grid [mm³/s] (must be non-empty).
    /// @param tempAxis Ascending temperature grid [°C] (must be non-empty).
    void build(const CrossWlfParams& params,
               const NozzleGeometry& geom,
               std::vector<double> flowAxis,
               std::vector<double> tempAxis);

    /// @brief Bilinear interpolation of pressure at (Q, T).
    /// Out-of-range arguments are clamped to the nearest axis endpoint.
    /// @returns Pressure [Pa], or 0 if the table is empty.
    double pressure(double flowMm3PerS, double tempC) const;

    /// @brief Whether the table contains any data.
    bool empty() const { return values_.empty(); }

    /// @brief Number of flow grid points.
    size_t numFlow() const { return flowAxis_.size(); }
    /// @brief Number of temperature grid points.
    size_t numTemp() const { return tempAxis_.size(); }

    const std::vector<double>& flowAxis() const { return flowAxis_; }
    const std::vector<double>& tempAxis() const { return tempAxis_; }
    const std::vector<double>& values() const { return values_; }

    /// @brief Replace the table contents directly (e.g. from a serialized
    /// table). Axes must be ascending; values must be temp-major with
    /// size = tempAxis.size() * flowAxis.size().
    void assign(std::vector<double> flowAxis,
                std::vector<double> tempAxis,
                std::vector<double> values);

private:
    std::vector<double> flowAxis_;
    std::vector<double> tempAxis_;
    std::vector<double> values_; // temp-major
};

} // namespace tether::control::extrusion
