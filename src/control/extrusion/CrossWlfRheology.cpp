/**
 * @file CrossWlfRheology.cpp
 * @brief Cross-WLF viscosity model and numerical pressure-flow inversion.
 */

#include "tether/control/extrusion/CrossWlfRheology.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::control::extrusion {

double CrossWlfRheology::zeroShearViscosity(double tempC,
                                            const CrossWlfParams& p) {
    const double dT = tempC - p.refTempC;
    // WLF: log10(η0/ηref) = -C1·dT/(C2+dT)
    const double denom = p.wlfC2 + dT;
    if (denom <= 0.0) {
        // Below the WLF valid range — clamp to a large viscosity.
        return p.zeroShearViscosityRef * 1.0e6;
    }
    const double shift = -p.wlfC1 * dT / denom;
    return p.zeroShearViscosityRef * std::pow(10.0, shift);
}

double CrossWlfRheology::viscosity(double shearRate, double tempC,
                                   const CrossWlfParams& p) {
    const double eta0 = zeroShearViscosity(tempC, p);
    if (shearRate <= 0.0) return eta0;
    const double ratio = shearRate * eta0 / p.tauStar;
    const double exponent = 1.0 - p.flowIndex; // (1-n), > 0 for n<1
    const double denom = 1.0 + std::pow(ratio, exponent);
    return eta0 / denom;
}

namespace {

/// @brief Volumetric flow for a Newtonian pipe at viscosity eta (used as a
/// scale for the bisection bracket).
double newtonianFlow(double pressurePa, double viscosityPas,
                     const NozzleGeometry& g) {
    if (viscosityPas <= 0.0) return 0.0;
    // Q = π R^4 ΔP / (8 η L)
    return M_PI * std::pow(g.radiusMm, 4.0) * pressurePa /
           (8.0 * viscosityPas * g.lengthMm);
}

/// @brief Integrate the pipe-flow velocity profile for a Cross-WLF fluid at
/// a given pressure gradient and temperature, returning Q [mm³/s].
///
/// For a circular pipe with pressure gradient dp/dz = -P/L, the wall shear
/// stress is τ_w = P·R/(2·L). The shear stress varies linearly with radius:
/// τ(r) = τ_w · (r/R). The shear rate is γ̇(r) = τ(r)/η(γ̇,T), which is
/// implicit for a shear-thinning fluid. We solve it by fixed-point iteration
/// at each radial node, then integrate Q = ∫ 2π r v(r) dr with v(R)=0 and
/// dv/dr = -γ̇.
double integrateFlow(double pressurePa, double tempC,
                     const CrossWlfParams& p, const NozzleGeometry& g) {
    if (pressurePa <= 0.0) return 0.0;
    const double R = g.radiusMm;
    const double L = g.lengthMm;
    const double tauWall = pressurePa * R / (2.0 * L);
    if (tauWall <= 0.0) return 0.0;

    const int N = CrossWlfRheology::kQuadraturePoints;
    double Q = 0.0;
    for (int i = 0; i < N; ++i) {
        // Midpoint of each radial shell, r in (0, R].
        const double r = R * (static_cast<double>(i) + 0.5) / N;
        const double dr = R / N;
        const double tau = tauWall * (r / R);
        // Solve γ̇ = τ/η(γ̇,T) by fixed-point iteration.
        double gammaDot = tau / CrossWlfRheology::zeroShearViscosity(tempC, p);
        for (int iter = 0; iter < 20; ++iter) {
            const double eta = CrossWlfRheology::viscosity(gammaDot, tempC, p);
            const double next = tau / eta;
            if (std::abs(next - gammaDot) <=
                1e-6 * (1.0 + std::abs(gammaDot))) {
                gammaDot = next;
                break;
            }
            gammaDot = next;
        }
        // Velocity gradient dv/dr = -γ̇ (axisymmetric). Integrate v(r) from
        // the wall inward: v(r) = ∫_r^R γ̇(s) ds. Approximate with the
        // trapezoidal rule over the shells from r to R.
        double v = 0.0;
        for (int j = i; j < N; ++j) {
            const double rj = R * (static_cast<double>(j) + 0.5) / N;
            const double rjNext = R * (static_cast<double>(j) + 1.5) / N;
            const double tauj = tauWall * (rj / R);
            const double taujNext = tauWall * (std::min(rjNext, R) / R);
            double gj = tauj / CrossWlfRheology::viscosity(
                tauj / CrossWlfRheology::zeroShearViscosity(tempC, p),
                tempC, p);
            double gjNext = taujNext / CrossWlfRheology::viscosity(
                taujNext / CrossWlfRheology::zeroShearViscosity(tempC, p),
                tempC, p);
            v += 0.5 * (gj + gjNext) * dr;
        }
        Q += 2.0 * M_PI * r * v * dr;
    }
    return Q;
}

} // namespace

double CrossWlfRheology::pressureFromFlow(double flowMm3PerS, double tempC,
                                          const CrossWlfParams& p,
                                          const NozzleGeometry& g) {
    if (flowMm3PerS <= 0.0) return 0.0;
    // Bisection on pressure. Bracket: [0, P_hi] where P_hi is large enough
    // that integrateFlow(P_hi) >= Q.
    const double eta0 = zeroShearViscosity(tempC, p);
    double Plo = 0.0;
    // Initial high guess from Newtonian estimate inverted: P = 8ηLQ/(πR⁴).
    double Phi = std::max(1.0, 8.0 * eta0 * g.lengthMm * flowMm3PerS /
                                  (M_PI * std::pow(g.radiusMm, 4.0)));
    // Expand bracket until flow(P_hi) >= target.
    for (int expand = 0; expand < 40; ++expand) {
        const double Qhi = integrateFlow(Phi, tempC, p, g);
        if (Qhi >= flowMm3PerS) break;
        Phi *= 2.0;
    }
    // Bisection.
    for (int iter = 0; iter < 60; ++iter) {
        const double Pmid = 0.5 * (Plo + Phi);
        const double Qmid = integrateFlow(Pmid, tempC, p, g);
        if (Qmid < flowMm3PerS) Plo = Pmid;
        else Phi = Pmid;
        if (Phi - Plo < 1e-3 * (1.0 + Plo)) break;
    }
    return 0.5 * (Plo + Phi);
}

double CrossWlfRheology::flowFromPressure(double pressurePa, double tempC,
                                          const CrossWlfParams& p,
                                          const NozzleGeometry& g) {
    if (pressurePa <= 0.0) return 0.0;
    return integrateFlow(pressurePa, tempC, p, g);
}

} // namespace tether::control::extrusion
