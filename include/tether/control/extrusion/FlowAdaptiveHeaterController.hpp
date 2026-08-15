/**
 * @file FlowAdaptiveHeaterController.hpp
 * @brief Flow-adaptive heater controller with pre/post-emphasis feed-forward.
 *
 * @details
 * Wraps a Control::PIDController (the same style as objects::Heater) and adds
 * two physically-motivated feed-forward terms that compensate for the
 * enthalpy carried away by the flowing filament:
 *
 *   P_ff = ρ·c_p·Q·(T_target - T_inlet)
 *
 * **Pre-emphasis:** at flow onset, the steady-state enthalpy power is applied
 * *before* the melt zone has cooled, plus a heater-block boost so the
 * heater→melt gradient ΔT_hm = P_ff/G_hm is established in advance. Limited
 * by `maxPreEmphasisPower` (PWM fraction) and `maxHeaterOvershoot` (°C above
 * target).
 *
 * **Post-emphasis:** after high flow stops, the melt zone is thermally
 * depleted and the heater→melt gradient must relax. A first-order "thermal
 * debt" state Ḋ = (D_target(Q_recent) - D)/τ_debt adds a decaying power
 * offset, limited by `maxPostEmphasisPower`. This prevents the classic
 * temperature dip after a fast print section and the overshoot when flow
 * stops.
 *
 * The controller derives from Control::ControllerBase (like all controllers)
 * so it composes with the existing framework. The PID backend is owned
 * internally and tuned via setGains(); the flow input is provided per-cycle
 * via the ControllerInput::feedforward field (reused as Q in mm³/s) or via
 * setFlow().
 *
 * Minimal physically-meaningful parameter set (7, all with SI units):
 *   filamentHeatCapacity    ρ·c_p   [J/(mm³·K)]
 *   meltZoneCapacitance     C_m     [J/K]
 *   heaterMeltConductance   G_hm    [W/K]
 *   debtTimeConstant        τ       [s]
 *   maxPreEmphasisPower     [0-1 PWM]
 *   maxPostEmphasisPower    [0-1 PWM]
 *   maxHeaterOvershoot      [°C]
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
    double filamentHeatCapacity  = 2.1;   ///< ρ·c_p [J/(mm³·K)]
    double meltZoneCapacitance   = 2.0;   ///< C_m [J/K]
    double heaterMeltConductance = 0.8;   ///< G_hm [W/K]
    double debtTimeConstant      = 2.0;   ///< τ [s]
    double maxPreEmphasisPower   = 0.4;   ///< [0-1 PWM]
    double maxPostEmphasisPower  = 0.2;   ///< [0-1 PWM]
    double maxHeaterOvershoot    = 10.0;  ///< [°C]
    double inletTempC            = 25.0;  ///< T_inlet [°C]
    double heaterPowerScale      = 40.0;  ///< [W] at PWM=1
};

/// @brief Flow-adaptive heater controller (PID + pre/post-emphasis).
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
        return "PID + flow pre/post-emphasis feed-forward for hotend heaters.";
    }

    /// @brief Set the PID gains of the internal backend.
    void setGains(double kp, double ki, double kd) {
        pid_.setGains(kp, ki, kd);
    }

    /// @brief Set the flow rate for the next control cycle [mm³/s].
    /// This is the planner's look-ahead Q (or the per-move E-velocity·A_f).
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

    /// @brief Last computed feed-forward contributions (diagnostics).
    struct EmphasisState {
        double preEmphasisPWM = 0.0;
        double postEmphasisPWM = 0.0;
        double thermalDebt = 0.0;
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
        m.meltZoneCapacitance = p.meltZoneCapacitance;
        m.heaterMeltConductance = p.heaterMeltConductance;
        m.filamentHeatCapacity = p.filamentHeatCapacity;
        m.inletTempC = p.inletTempC;
        m.heaterPowerScale = p.heaterPowerScale;
        return m;
    }

    FlowAdaptiveHeaterParams params_;
    ::Control::PIDController pid_;
    MeltZoneThermalObserver observer_;
    double flowMm3PerS_ = 0.0;
    double thermalDebt_ = 0.0;   ///< current thermal-debt state [W]
    EmphasisState emphasis_;
};

} // namespace tether::control::extrusion
