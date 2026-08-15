/**
 * @file MeltZoneThermalObserver.hpp
 * @brief Two-state lumped thermal observer for the hotend melt zone.
 *
 * @details
 * Models the hotend as two coupled thermal masses:
 *
 *   T_heater_block  — driven by heater power (input from PID output)
 *   T_melt          — the actual melt temperature driving rheology
 *
 * Coupling:
 *   C_h · dT_h/dt = P_heater - G_hm·(T_h - T_m)
 *   C_m · dT_m/dt = G_hm·(T_h - T_m) - ρ·c_p·Q·(T_m - T_inlet)
 *
 * where the last term is the enthalpy removed by the flowing filament
 * (volumetric flow Q [mm³/s], inlet temperature T_inlet, volumetric heat
 * capacity ρ·c_p [J/(mm³·K)]).
 *
 * The observer is integrated at the heater control interval with
 * forward-Euler (good enough for the slow thermal dynamics; the time constants
 * are seconds). It outputs T_melt_est, which feeds the Cross-WLF PA model.
 *
 * Inputs:
 *   - heaterPower: PWM fraction [0,1] applied to the heater block
 *   - heaterPowerScale: heater electrical power [W] at PWM=1
 *   - flowMm3PerS: current volumetric flow [mm³/s]
 *   - dt: integration step [s]
 *
 * @see docs/extrusion/FlowAdaptiveTemperatureControl.md §1
 */

#pragma once

namespace tether::control::extrusion {

/// @brief Parameters for the two-state melt-zone thermal observer.
struct MeltZoneThermalParams {
    double heaterBlockCapacitance = 8.0;   ///< C_h [J/K]
    double meltZoneCapacitance    = 2.0;   ///< C_m [J/K]
    double heaterMeltConductance  = 0.8;   ///< G_hm [W/K]
    double filamentHeatCapacity   = 2.1;   ///< ρ·c_p [J/(mm³·K)]
    double inletTempC             = 25.0;  ///< T_inlet [°C]
    double heaterPowerScale       = 40.0;  ///< [W] at PWM=1
};

/// @brief Two-state lumped melt-zone thermal observer.
class MeltZoneThermalObserver {
public:
    explicit MeltZoneThermalObserver(MeltZoneThermalParams params = MeltZoneThermalParams{})
        : params_(params) {}

    /// @brief Set the parameters.
    void setParams(MeltZoneThermalParams p) { params_ = p; }
    const MeltZoneThermalParams& params() const { return params_; }

    /// @brief Initialize both states to a known temperature (e.g. measured
    /// heater-block temperature at startup).
    void initialize(double heaterBlockTempC, double meltTempC) {
        heaterBlockTemp_ = heaterBlockTempC;
        meltTemp_ = meltTempC;
    }

    /// @brief Seed the heater-block state from a measured sensor reading
    /// (the heater-block thermistor). The melt state is left as-is.
    void setHeaterBlockTemp(double tempC) { heaterBlockTemp_ = tempC; }

    /// @brief Advance the observer by one control interval.
    /// @param heaterPWM Heater PWM fraction [0,1].
    /// @param flowMm3PerS Volumetric flow [mm³/s].
    /// @param dt Time step [s].
    void update(double heaterPWM, double flowMm3PerS, double dt);

    /// @brief Estimated melt-zone temperature [°C].
    double meltTempEst() const { return meltTemp_; }

    /// @brief Heater-block state [°C].
    double heaterBlockTemp() const { return heaterBlockTemp_; }

    /// @brief Reset state to inlet temperature.
    void reset() {
        heaterBlockTemp_ = params_.inletTempC;
        meltTemp_ = params_.inletTempC;
    }

private:
    MeltZoneThermalParams params_;
    double heaterBlockTemp_ = params_.inletTempC;
    double meltTemp_ = params_.inletTempC;
};

} // namespace tether::control::extrusion
