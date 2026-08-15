/**
 * @file FlowAdaptiveHeaterController.hpp
 * @brief Model-based flow-adaptive heater controller with three-state thermal
 *        observer, pre-emphasis, and post-emphasis feed-forward.
 *
 * @details
 * Wraps a Control::PIDController (the same style as objects::Heater) and adds
 * two physically-motivated feed-forward terms that compensate for the enthalpy
 * carried away by the flowing filament:
 *
 *   P_ff = ρ·c_p·Q·(T_target - T_inlet)
 *
 * ## Three-state thermal model
 *
 * The controller uses a MeltZoneThermalObserver with three coupled states
 * (heater block T_h, sensor T_s, melt zone T_m) and a Luenberger correction
 * that uses the actual thermistor reading to correct the state estimate.
 *
 * This is critical because the thermistor is physically *between* the heater
 * block and the melt zone. When cold plastic flows:
 *
 *   1. T_m drops first (enthalpy drain at the melt zone).
 *   2. T_s drops second (sensor coupled to melt through G_sm, with delay).
 *   3. T_h drops last (heater block has the largest thermal mass).
 *
 * The PID reacts to T_s, so it *partially* compensates on its own — but with
 * a lag and a gain error, because the sensor doesn't see the full magnitude
 * of the melt-zone drop. The feed-forward terms cover the *uncompensated*
 * portion.
 *
 * ## Pre-emphasis
 *
 * At flow onset, the steady-state enthalpy power P_ff is applied *before*
 * the melt zone has cooled. The amount is reduced by the sensor coupling
 * factor α = G_sm / (G_sm + G_hs), which represents the fraction of the
 * melt-zone disturbance that the PID will see and react to on its own.
 *
 * The pre-emphasis also includes a heater-block boost to establish the
 * gradient ΔT = P_ff / G_total (where G_total is the series combination of
 * G_hs and G_sm) ahead of the thermal lag. Both terms are bounded by
 * maxPreEmphasisPower and maxHeaterOvershoot.
 *
 * Pre-emphasis is suppressed when the measured temperature is far from
 * target (during warmup) to avoid fighting the PID.
 *
 * ## Post-emphasis
 *
 * After high flow stops, the melt zone is thermally depleted but the sensor
 * is still lagging — it will read artificially high because the heater block
 * is still hot. This causes the PID to reduce power prematurely, leading to
 * the classic post-print temperature dip.
 *
 * The post-emphasis term models this as a "thermal debt" D that relaxes
 * toward zero with time constant τ_debt. When flow stops, D_target → 0 but
 * D is still positive, so the deficit (D_target - D) goes negative and we
 * add a decaying positive power to compensate. The magnitude is scaled by
 * (1 - α) because the PID's own reaction (through the sensor lag) will
 * partially cover the recovery.
 *
 * The controller derives from Control::ControllerBase (like all controllers)
 * so it composes with the existing framework. The PID backend is owned
 * internally and tuned via setGains(); the flow input is provided per-cycle
 * via the ControllerInput::feedforward field (reused as Q in mm³/s) or via
 * setFlow().
 *
 * @see docs/extrusion/FlowAdaptiveTemperatureControl.md §2
 */

#pragma once

#include "tether/control/ControllerBase.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/control/extrusion/MeltZoneThermalObserver.hpp"

namespace tether::control::extrusion {

/// @brief Parameters for the flow-adaptive heater controller.
struct FlowAdaptiveHeaterParams {
    // --- Filament and heater ---
    double filamentHeatCapacity  = 2.1;   ///< ρ·c_p [J/(mm³·K)]
    double inletTempC            = 25.0;  ///< T_inlet [°C]
    double heaterPowerScale      = 40.0;  ///< [W] at PWM=1

    // --- Three-state thermal model ---
    double heaterBlockCapacitance  = 8.0; ///< C_h [J/K]
    double sensorCapacitance       = 1.0; ///< C_s [J/K]
    double meltZoneCapacitance     = 2.0; ///< C_m [J/K]
    double heaterSensorConductance = 2.0; ///< G_hs [W/K]
    double sensorMeltConductance   = 1.5; ///< G_sm [W/K]

    // --- Luenberger observer gains [1/s] ---
    double luenbergerGainHeater = 0.5;
    double luenbergerGainSensor = 2.0;
    double luenbergerGainMelt   = 0.3;

    // --- Feed-forward limits ---
    double debtTimeConstant      = 2.0;   ///< τ [s] — post-emphasis decay
    double maxPreEmphasisPower   = 0.4;   ///< [0-1 PWM]
    double maxPostEmphasisPower  = 0.2;   ///< [0-1 PWM]
    double maxHeaterOvershoot    = 10.0;  ///< [°C]
};

/// @brief Flow-adaptive heater controller (PID + model-based pre/post-emphasis).
class FlowAdaptiveHeaterController : public ::Control::ControllerBase {
public:
    explicit FlowAdaptiveHeaterController(FlowAdaptiveHeaterParams params = FlowAdaptiveHeaterParams{})
        : params_(params),
          observer_(toObserverParams(params)) {}

    // --- ControllerBase interface ---
    ::Control::ControllerType getType() const override {
        return ::Control::ControllerType::Custom;
    }
    const char* getName() const override {
        return "FlowAdaptiveHeaterController";
    }
    const char* getDescription() const override {
        return "PID + three-state model-based pre/post-emphasis feed-forward.";
    }

    /// @brief Set the PID gains of the internal backend.
    void setGains(double kp, double ki, double kd) {
        pid_.setGains(kp, ki, kd);
    }

    /// @brief Set the flow rate for the next control cycle [mm³/s].
    void setFlow(double flowMm3PerS) { flowMm3PerS_ = flowMm3PerS; }
    double flow() const { return flowMm3PerS_; }

    /// @brief Set the parameters (also resets the observer params).
    void setParams(FlowAdaptiveHeaterParams p) {
        params_ = p;
        observer_.setParams(toObserverParams(p));
    }
    const FlowAdaptiveHeaterParams& params() const { return params_; }

    /// @brief Estimated melt-zone temperature from the internal observer.
    double meltTempEst() const { return observer_.meltTempEst(); }

    /// @brief Heater-block state from the internal observer.
    double heaterBlockTemp() const { return observer_.heaterBlockTemp(); }

    /// @brief Sensor-point state from the internal observer.
    double sensorTemp() const { return observer_.sensorTemp(); }

    /// @brief Last computed feed-forward contributions (diagnostics).
    struct EmphasisState {
        double preEmphasisPWM = 0.0;
        double postEmphasisPWM = 0.0;
        double thermalDebt = 0.0;
        double sensorCouplingAlpha = 0.0; ///< Fraction of disturbance PID sees
    };
    const EmphasisState& emphasis() const { return emphasis_; }

    /// @brief Reset internal state (PID, observer, debt).
    void resetImpl() override {
        pid_.reset();
        observer_.reset();
        thermalDebt_ = 0.0;
        flowMm3PerS_ = 0.0;
        emphasis_ = {};
    }

protected:
    ::Control::ControllerOutput computeImpl(
        const ::Control::ControllerInput& input) override;

private:
    static MeltZoneThermalParams toObserverParams(
        const FlowAdaptiveHeaterParams& p) {
        MeltZoneThermalParams m;
        m.heaterBlockCapacitance = p.heaterBlockCapacitance;
        m.sensorCapacitance = p.sensorCapacitance;
        m.meltZoneCapacitance = p.meltZoneCapacitance;
        m.heaterSensorConductance = p.heaterSensorConductance;
        m.sensorMeltConductance = p.sensorMeltConductance;
        m.filamentHeatCapacity = p.filamentHeatCapacity;
        m.inletTempC = p.inletTempC;
        m.heaterPowerScale = p.heaterPowerScale;
        m.luenbergerGainHeater = p.luenbergerGainHeater;
        m.luenbergerGainSensor = p.luenbergerGainSensor;
        m.luenbergerGainMelt = p.luenbergerGainMelt;
        return m;
    }

    /// @brief Compute the sensor coupling factor α = G_sm / (G_hs + G_sm).
    /// This is the fraction of the melt-zone thermal disturbance that the
    /// PID (which reacts to T_s) will compensate on its own. The feed-forward
    /// only needs to cover the remaining (1 - α) fraction.
    double sensorCouplingAlpha() const {
        const double Gsum = params_.heaterSensorConductance +
                            params_.sensorMeltConductance;
        return (Gsum > 0.0) ? params_.sensorMeltConductance / Gsum : 1.0;
    }

    /// @brief Effective series conductance from heater block to melt zone.
    double effectiveConductance() const {
        const double Ghs = params_.heaterSensorConductance;
        const double Gsm = params_.sensorMeltConductance;
        return (Ghs + Gsm > 0.0) ? (Ghs * Gsm) / (Ghs + Gsm) : 0.0;
    }

    FlowAdaptiveHeaterParams params_;
    ::Control::PIDController pid_;
    MeltZoneThermalObserver observer_;
    double flowMm3PerS_ = 0.0;
    double thermalDebt_ = 0.0;   ///< current thermal-debt state [W]
    EmphasisState emphasis_;
};

} // namespace tether::control::extrusion
