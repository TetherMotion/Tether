/**
 * @file CrossWlfRheology.hpp
 * @brief Cross-WLF temperature-dependent rheology for polymer melts.
 *
 * @details
 * The Cross model with a WLF (Williams-Landel-Ferry) temperature shift gives
 * the zero-shear viscosity η₀(T) and the shear-thinning transition:
 *
 *   η(γ̇, T) = η₀(T) / (1 + (γ̇·η₀(T)/τ*)^(1 - n))
 *
 * with the WLF shift for the zero-shear viscosity:
 *
 *   log10(η₀(T)/η_ref) = -C1·(T - T_ref) / (C2 + (T - T_ref))
 *
 * (Standard WLF form; η_ref is the zero-shear viscosity at T_ref.)
 *
 * The pressure-flow relation for a Cross-WLF fluid in a circular pipe has no
 * closed-form inversion: the volumetric flow is the integral of the velocity
 * profile, which depends on η(γ̇, T) implicitly. We therefore solve it
 * numerically (bisection on the pressure drop, with the pipe-flow integral
 * evaluated by quadrature) in the .cpp. The header exposes the pure viscosity
 * model plus the inversion entry point.
 *
 * Units:
 *   γ̇      [1/s]   shear rate
 *   T      [°C]    melt temperature
 *   η      [Pa·s]  viscosity
 *   τ*     [Pa]    Cross critical stress
 *   n      [-]     Cross flow index (typically 0.3-0.6 for polymer melts)
 *   C1, C2 [K]     WLF constants (C2 is a temperature offset, e.g. 51.6 K)
 *   T_ref  [°C]    reference temperature for WLF
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md §3
 */

#pragma once

#include "tether/control/extrusion/PowerLawRheology.hpp"

#include <vector>

namespace tether::control::extrusion {

/// @brief Cross-WLF rheology parameters.
struct CrossWlfParams {
    double zeroShearViscosityRef = 1000.0; ///< η_ref [Pa·s] at T_ref
    double tauStar = 1.0e5;                ///< τ* [Pa] Cross critical stress
    double flowIndex = 0.4;                ///< n [-] (Cross index, 0<n<1)
    double wlfC1 = 17.44;                  ///< C1 [-] WLF constant
    double wlfC2 = 51.6;                   ///< C2 [K] WLF constant
    double refTempC = 200.0;               ///< T_ref [°C]
};

/// @brief Cross-WLF viscosity model + numerical pressure-flow inversion.
class CrossWlfRheology {
public:
    /// @brief Zero-shear viscosity at temperature T (WLF shift).
    ///   η₀(T) = η_ref · 10^(-C1·(T-T_ref)/(C2+(T-T_ref)))
    static double zeroShearViscosity(double tempC, const CrossWlfParams& p);

    /// @brief Full Cross-WLF viscosity at (shear rate, temperature).
    ///   η(γ̇,T) = η₀(T) / (1 + (γ̇·η₀(T)/τ*)^(1-n))
    static double viscosity(double shearRate, double tempC,
                            const CrossWlfParams& p);

    /// @brief Pressure drop for a given flow and temperature.
    /// Numerically integrates the pipe-flow velocity profile (quadrature on
    /// the radial shear-rate distribution) and inverts by bisection.
    /// @param flowMm3PerS Volumetric flow [mm³/s].
    /// @param tempC Melt temperature [°C].
    /// @return Pressure drop [Pa].
    static double pressureFromFlow(double flowMm3PerS, double tempC,
                                   const CrossWlfParams& p,
                                   const NozzleGeometry& g);

    /// @brief Volumetric flow for a given pressure drop and temperature.
    /// Inverts pressureFromFlow by bisection.
    static double flowFromPressure(double pressurePa, double tempC,
                                   const CrossWlfParams& p,
                                   const NozzleGeometry& g);

    /// @brief Number of radial quadrature points used by the integrator.
    static constexpr int kQuadraturePoints = 64;
};

} // namespace tether::control::extrusion
