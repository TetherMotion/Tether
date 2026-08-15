/**
 * @file MeltZoneThermalObserver.cpp
 * @brief Forward-Euler integration of the two-state melt-zone thermal model.
 */

#include "tether/control/extrusion/MeltZoneThermalObserver.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

void MeltZoneThermalObserver::update(double heaterPWM, double flowMm3PerS,
                                     double dt) {
    if (dt <= 0.0) return;
    heaterPWM = std::clamp(heaterPWM, 0.0, 1.0);
    if (flowMm3PerS < 0.0) flowMm3PerS = 0.0;

    const double P_heater = heaterPWM * params_.heaterPowerScale; // [W]
    const double dT_hm = heaterBlockTemp_ - meltTemp_;            // [K]
    const double enthalpy = params_.filamentHeatCapacity * flowMm3PerS *
                            (meltTemp_ - params_.inletTempC);     // [W]

    // C_h dT_h/dt = P_heater - G_hm (T_h - T_m)
    const double dTh = (P_heater - params_.heaterMeltConductance * dT_hm) /
                       params_.heaterBlockCapacitance;
    // C_m dT_m/dt = G_hm (T_h - T_m) - ρ c_p Q (T_m - T_inlet)
    const double dTm = (params_.heaterMeltConductance * dT_hm - enthalpy) /
                       params_.meltZoneCapacitance;

    heaterBlockTemp_ += dTh * dt;
    meltTemp_ += dTm * dt;
}

} // namespace tether::control::extrusion
