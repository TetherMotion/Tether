/**
 * @file PowerLawRheology.hpp
 * @brief Power-law non-Newtonian pipe-flow rheology (η(γ̇) = K_p · γ̇^(n-1)).
 *
 * @details
 * Models the steady-state, fully-developed pressure/flow relation for a
 * power-law fluid in a circular nozzle of radius R and length L:
 *
 *   Q = π·n/(3n+1) · (ΔP / (2·L·K_p))^(1/n) · R^((3n+1)/n)        (forward)
 *
 * Inverting for pressure as a function of flow gives the corrected resistance
 * constant (see docs/extrusion/NonNewtonianPressureAdvance.md, §1):
 *
 *   ΔP(Q) = C_n · Q^n
 *   C_n   = 2·L·K_p · ((3n+1)/(π·n))^n · R^-(3n+1)
 *
 * Newtonian sanity check (n = 1, K_p = η):
 *   C_1 = 2·L·η · (4/π) · R^-4 = 8·η·L/(π·R^4)
 *   ΔP  = 8·η·L·Q/(π·R^4) = Hagen-Poiseuille ✓
 *
 * All quantities are in SI-derived consistent units:
 *   Q      [mm³/s]   volumetric flow
 *   ΔP     [Pa]      pressure drop
 *   R, L   [mm]      nozzle geometry
 *   K_p    [Pa·s^n]  power-law consistency index
 *   n      [-]       flow index (n<1 shear-thinning, n=1 Newtonian, n>1 shear-thickening)
 *
 * The class is a pure, stateless collection of static helpers and is safe to
 * use from real-time contexts (no allocation, no globals).
 *
 * @note The earlier draft of the design used an incorrect inversion that
 *       produced R^(-3n) instead of R^-(3n+1); this implementation uses the
 *       corrected form. See jade-x-23-simon-baz.md §0.
 */

#pragma once

#include <cmath>
#include <limits>

namespace tether::control::extrusion {

/// @brief Nozzle/melt-channel geometry used by the rheology helpers.
struct NozzleGeometry {
    double radiusMm = 0.2;   ///< Nozzle radius [mm] (default 0.4 mm diameter)
    double lengthMm = 10.0;  ///< Nozzle / melt-zone length [mm]
};

/// @brief Power-law rheology parameters.
struct PowerLawParams {
    double consistencyIndex = 1000.0; ///< K_p [Pa·s^n]
    double flowIndex = 1.0;           ///< n [-] (1 = Newtonian)
};

/// @brief Stateless power-law pipe-flow rheology.
class PowerLawRheology {
public:
    /// @brief Compute the resistance constant C_n [Pa / (mm³/s)^n].
    ///   C_n = 2·L·K_p · ((3n+1)/(π·n))^n · R^-(3n+1)
    static double resistanceConstant(const PowerLawParams& p,
                                     const NozzleGeometry& g) {
        if (p.flowIndex <= 0.0 || g.radiusMm <= 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        const double n = p.flowIndex;
        const double factor = (3.0 * n + 1.0) / (M_PI * n);
        // R^-(3n+1) = exp(-(3n+1)·ln(R))
        const double rPow = std::exp(-(3.0 * n + 1.0) * std::log(g.radiusMm));
        return 2.0 * g.lengthMm * p.consistencyIndex *
               std::pow(factor, n) * rPow;
    }

    /// @brief Pressure drop for a given volumetric flow.
    ///   ΔP = C_n · Q^n   (Q in mm³/s, returns Pa)
    static double pressureFromFlow(double flowMm3PerS,
                                   const PowerLawParams& p,
                                   const NozzleGeometry& g) {
        if (flowMm3PerS <= 0.0) return 0.0;
        const double Cn = resistanceConstant(p, g);
        return Cn * std::pow(flowMm3PerS, p.flowIndex);
    }

    /// @brief Volumetric flow for a given pressure drop (forward solution).
    ///   Q = (πn/(3n+1)) · (ΔP/(2LK_p))^(1/n) · R^((3n+1)/n)
    static double flowFromPressure(double pressurePa,
                                   const PowerLawParams& p,
                                   const NozzleGeometry& g) {
        if (pressurePa <= 0.0 || p.consistencyIndex <= 0.0 ||
            p.flowIndex <= 0.0) {
            return 0.0;
        }
        const double n = p.flowIndex;
        const double coef = (M_PI * n) / (3.0 * n + 1.0);
        const double drive = std::pow(pressurePa / (2.0 * g.lengthMm *
                                                    p.consistencyIndex),
                                      1.0 / n);
        const double rPow = std::exp(((3.0 * n + 1.0) / n) *
                                     std::log(g.radiusMm));
        return coef * drive * rPow;
    }

    /// @brief Apparent viscosity η_app = K_p · γ̇^(n-1) at shear rate γ̇.
    /// For a circular pipe, the wall shear rate is γ̇_w = 4Q/(πR³) (Newtonian
    /// approximation; the Rabinowitsch correction is omitted for simplicity).
    static double apparentViscosity(double flowMm3PerS,
                                    const PowerLawParams& p,
                                    const NozzleGeometry& g) {
        if (g.radiusMm <= 0.0) return 0.0;
        const double shearRate = 4.0 * flowMm3PerS / (M_PI *
                                  std::pow(g.radiusMm, 3.0));
        if (shearRate <= 0.0) return std::numeric_limits<double>::infinity();
        return p.consistencyIndex * std::pow(shearRate, p.flowIndex - 1.0);
    }
};

} // namespace tether::control::extrusion
