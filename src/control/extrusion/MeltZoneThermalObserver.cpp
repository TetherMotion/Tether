/**
 * @file MeltZoneThermalObserver.cpp
 * @brief Forward-Euler integration of the three-state melt-zone thermal model
 *        with Luenberger sensor correction.
 */

#include "tether/control/extrusion/MeltZoneThermalObserver.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

void MeltZoneThermalObserver::predictStep(double heaterPWM,
                                          double flowMm3PerS, double dt) {
    if (dt <= 0.0) return;
    heaterPWM = std::clamp(heaterPWM, 0.0, 1.0);
    if (flowMm3PerS < 0.0) flowMm3PerS = 0.0;

    const double P_heater = heaterPWM * params_.heaterPowerScale;   // [W]
    const double dT_hs = heaterBlockTemp_ - sensorTemp_;            // [K]
    const double dT_sm = sensorTemp_ - meltTemp_;                   // [K]
    const double enthalpy = params_.filamentHeatCapacity * flowMm3PerS *
                            (meltTemp_ - params_.inletTempC);       // [W]

    // C_h dT_h/dt = P_heater - G_hs (T_h - T_s)
    const double dTh = (P_heater -
                        params_.heaterSensorConductance * dT_hs) /
                       params_.heaterBlockCapacitance;
    // C_s dT_s/dt = G_hs (T_h - T_s) - G_sm (T_s - T_m)
    const double dTs = (params_.heaterSensorConductance * dT_hs -
                        params_.sensorMeltConductance * dT_sm) /
                       params_.sensorCapacitance;
    // C_m dT_m/dt = G_sm (T_s - T_m) - ρ c_p Q (T_m - T_inlet)
    const double dTm = (params_.sensorMeltConductance * dT_sm - enthalpy) /
                       params_.meltZoneCapacitance;

    heaterBlockTemp_ += dTh * dt;
    sensorTemp_ += dTs * dt;
    meltTemp_ += dTm * dt;
}

void MeltZoneThermalObserver::correctStep(double measuredSensorTempC,
                                          double dt) {
    if (dt <= 0.0) return;
    // Innovation: difference between measured and predicted sensor temperature.
    const double innovation = measuredSensorTempC - sensorTemp_;
    // Luenberger correction: each state is nudged toward consistency with
    // the measurement, scaled by its gain and the time step.
    heaterBlockTemp_ += params_.luenbergerGainHeater * innovation * dt;
    sensorTemp_ += params_.luenbergerGainSensor * innovation * dt;
    meltTemp_ += params_.luenbergerGainMelt * innovation * dt;
}

void MeltZoneThermalObserver::update(double heaterPWM, double flowMm3PerS,
                                     double dt) {
    // Open-loop prediction only (no sensor measurement available).
    predictStep(heaterPWM, flowMm3PerS, dt);
}

void MeltZoneThermalObserver::updateWithMeasurement(
    double heaterPWM, double flowMm3PerS,
    double measuredSensorTempC, double dt) {
    // 1. Predict (open-loop model step).
    predictStep(heaterPWM, flowMm3PerS, dt);
    // 2. Correct (close the loop with the real sensor reading).
    correctStep(measuredSensorTempC, dt);
}

} // namespace tether::control::extrusion
