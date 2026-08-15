/**
 * @file MeltZoneThermalObserver.hpp
 * @brief Three-state lumped thermal observer for the hotend melt zone.
 *
 * @details
 * Models the hotend as three coupled thermal masses, reflecting the physical
 * layout of a typical 3D printer hotend:
 *
 *   Heater cartridge
 *        ↓ (G_hs)
 *   Heater block (T_h)  — largest thermal mass, directly heated
 *        ↓ (G_hs)
 *   Sensor point (T_s)  — thermistor location, thermally between block and melt
 *        ↓ (G_sm)
 *   Melt zone (T_m)     — where filament actually melts; cold plastic enters here
 *
 * The three coupled ODEs (forward-Euler integration):
 *
 *   C_h · dT_h/dt = P_heater - G_hs·(T_h - T_s)
 *   C_s · dT_s/dt = G_hs·(T_h - T_s) - G_sm·(T_s - T_m)
 *   C_m · dT_m/dt = G_sm·(T_s - T_m) - ρ·c_p·Q·(T_m - T_inlet)
 *
 * where:
 *   P_heater = heaterPWM × heaterPowerScale  [W]
 *   Q = volumetric flow [mm³/s]
 *   ρ·c_p = filament volumetric heat capacity [J/(mm³·K)]
 *   T_inlet = ambient/cold-filament temperature [°C]
 *
 * The last term in the melt-zone equation is the enthalpy removed by the
 * flowing filament — this is the dominant disturbance when cold plastic is
 * pushed through the extruder.
 *
 * ## Luenberger correction
 *
 * The thermistor measures T_s (the sensor point). The observer uses this
 * measurement to correct all three state estimates via a Luenberger gain
 * vector L = [L_h, L_s, L_m]:
 *
 *   innovation = T_s_measured - T_s_estimated
 *   T_h += L_h · innovation · dt
 *   T_s += L_s · innovation · dt
 *   T_m += L_m · innovation · dt
 *
 * This is critical because:
 *   - Without correction, the open-loop model drifts due to unmodelled losses
 *     (radiation, convection to ambient, fan cooling).
 *   - The sensor sees the melt-zone cooling *before* the heater block does
 *     (through the G_sm coupling), so the measurement carries information
 *     about the melt-zone state that the heater-block temperature alone
 *     cannot provide.
 *   - The gains determine how aggressively the observer trusts the measurement
 *     vs. the model. Typical values: L_s is largest (direct measurement),
 *     L_h is moderate (indirect through G_hs), L_m is moderate (indirect
 *     through G_sm).
 *
 * ## Why three states instead of two?
 *
 * The previous two-state model (heater block + melt zone) conflated the
 * sensor with the heater block. In reality, the sensor is physically between
 * the heater block and the melt zone, with its own thermal mass and coupling
 * resistances. This means:
 *
 *   1. The sensor drops *before* the heater block when cold plastic flows
 *      (the PID sees the disturbance earlier, but partially).
 *   2. The sensor lags *behind* the melt zone (it doesn't see the full
 *      magnitude of the melt-zone temperature drop).
 *   3. After flow stops, the sensor overshoots because the heater block
 *      is still hot while the melt zone has recovered — the PID sees a
 *      falsely high temperature and reduces power prematurely.
 *
 * The three-state model captures all three effects, enabling the
 * FlowAdaptiveHeaterController to compensate for the sensor's intermediate
 * position rather than fighting it.
 *
 * @see docs/extrusion/FlowAdaptiveTemperatureControl.md §1
 */

#pragma once

namespace tether::control::extrusion {

/// @brief Parameters for the three-state melt-zone thermal observer.
struct MeltZoneThermalParams {
    // --- Thermal capacitances [J/K] ---
    double heaterBlockCapacitance = 8.0;   ///< C_h [J/K] — heater block mass
    double sensorCapacitance      = 1.0;   ///< C_s [J/K] — thermistor + surrounding metal
    double meltZoneCapacitance    = 2.0;   ///< C_m [J/K] — melt zone thermal mass

    // --- Thermal conductances [W/K] ---
    double heaterSensorConductance = 2.0;  ///< G_hs [W/K] — heater block → sensor
    double sensorMeltConductance   = 1.5;  ///< G_sm [W/K] — sensor → melt zone

    // --- Filament properties ---
    double filamentHeatCapacity   = 2.1;   ///< ρ·c_p [J/(mm³·K)]
    double inletTempC             = 25.0;  ///< T_inlet [°C]

    // --- Heater ---
    double heaterPowerScale       = 40.0;  ///< [W] at PWM=1

    // --- Luenberger observer gains [1/s] ---
    /// Correction applied to T_h from sensor innovation.
    double luenbergerGainHeater   = 0.5;
    /// Correction applied to T_s from sensor innovation (direct measurement).
    double luenbergerGainSensor   = 2.0;
    /// Correction applied to T_m from sensor innovation.
    double luenbergerGainMelt     = 0.3;
};

/// @brief Three-state lumped melt-zone thermal observer with Luenberger correction.
class MeltZoneThermalObserver {
public:
    explicit MeltZoneThermalObserver(MeltZoneThermalParams params = MeltZoneThermalParams{})
        : params_(params) {}

    /// @brief Set the parameters.
    void setParams(MeltZoneThermalParams p) { params_ = p; }
    const MeltZoneThermalParams& params() const { return params_; }

    /// @brief Initialize all three states to known temperatures.
    void initialize(double heaterBlockTempC, double sensorTempC,
                    double meltTempC) {
        heaterBlockTemp_ = heaterBlockTempC;
        sensorTemp_ = sensorTempC;
        meltTemp_ = meltTempC;
    }

    /// @brief Convenience: initialize all states to the same temperature
    /// (e.g. from a single sensor reading at startup).
    void initialize(double tempC) {
        initialize(tempC, tempC, tempC);
    }

    /// @brief Seed the sensor state from a measured thermistor reading.
    /// The heater-block and melt states are left as-is.
    void setSensorTemp(double tempC) { sensorTemp_ = tempC; }

    /// @brief Advance the observer by one control interval.
    /// @param heaterPWM Heater PWM fraction [0,1].
    /// @param flowMm3PerS Volumetric flow [mm³/s].
    /// @param dt Time step [s].
    void update(double heaterPWM, double flowMm3PerS, double dt);

    /// @brief Advance the observer with Luenberger correction from a real
    /// sensor measurement.
    /// @param heaterPWM Heater PWM fraction [0,1].
    /// @param flowMm3PerS Volumetric flow [mm³/s].
    /// @param measuredSensorTempC The actual thermistor reading [°C].
    /// @param dt Time step [s].
    void updateWithMeasurement(double heaterPWM, double flowMm3PerS,
                               double measuredSensorTempC, double dt);

    /// @brief Estimated melt-zone temperature [°C].
    double meltTempEst() const { return meltTemp_; }

    /// @brief Heater-block state [°C].
    double heaterBlockTemp() const { return heaterBlockTemp_; }

    /// @brief Sensor-point state [°C].
    double sensorTemp() const { return sensorTemp_; }

    /// @brief Reset state to inlet temperature.
    void reset() {
        heaterBlockTemp_ = params_.inletTempC;
        sensorTemp_ = params_.inletTempC;
        meltTemp_ = params_.inletTempC;
    }

private:
    void predictStep(double heaterPWM, double flowMm3PerS, double dt);
    void correctStep(double measuredSensorTempC, double dt);

    MeltZoneThermalParams params_;
    double heaterBlockTemp_ = params_.inletTempC;
    double sensorTemp_ = params_.inletTempC;
    double meltTemp_ = params_.inletTempC;
};

} // namespace tether::control::extrusion
