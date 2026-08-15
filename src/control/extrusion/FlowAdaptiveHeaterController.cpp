/**
 * @file FlowAdaptiveHeaterController.cpp
 * @brief PID + three-state model-based pre/post-emphasis feed-forward.
 */

#include "tether/control/extrusion/FlowAdaptiveHeaterController.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

::Control::ControllerOutput FlowAdaptiveHeaterController::computeImpl(
    const ::Control::ControllerInput& input) {
    const double dt = (input.dt > 0.0) ? input.dt : 0.1;
    const double target = input.reference;
    const double measured = input.measured;
    const double Q = flowMm3PerS_;

    // --- Sensor coupling factor: fraction of disturbance the PID sees ---
    // α = G_sm / (G_hs + G_sm)
    // The PID reacts to T_s, which is coupled to T_m through G_sm. The
    // fraction of the melt-zone disturbance that reaches the sensor (and
    // thus triggers PID compensation) is α. The feed-forward only needs
    // to cover the remaining (1 - α) fraction.
    const double alpha = sensorCouplingAlpha();
    const double oneMinusAlpha = 1.0 - alpha;
    const double G_eff = effectiveConductance();

    // --- Steady-state enthalpy power required to maintain target at flow Q ---
    // P_ff = ρ c_p Q (T_target - T_inlet)  [W]
    const double P_ff = (target - params_.inletTempC) *
                        params_.filamentHeatCapacity * Q;
    const double P_ff_pwm = (params_.heaterPowerScale > 0.0)
        ? P_ff / params_.heaterPowerScale : 0.0;

    // --- Pre-emphasis ---
    // The PID will compensate α·P_ff on its own (through the sensor coupling).
    // We only need to feed-forward the uncompensated (1-α)·P_ff, plus a
    // heater-block boost to establish the thermal gradient ahead of the lag.
    //
    // The boost establishes ΔT = P_ff / G_eff across the full heater→melt
    // path. Converting to PWM: boost = P_ff / G_eff × G_eff / P_scale = P_ff / P_scale.
    // But we only need the uncompensated fraction, so:
    //   boost_pwm = (1-α) × P_ff / P_scale
    // Bounded by maxHeaterOvershoot (converted to PWM via G_eff and P_scale).
    const double uncompensatedPff = oneMinusAlpha * P_ff;
    const double uncompensatedPffPWM = (params_.heaterPowerScale > 0.0)
        ? uncompensatedPff / params_.heaterPowerScale : 0.0;

    // Gradient boost: establish ΔT_hm = P_ff / G_eff in the heater block.
    // The PID won't create this gradient because it only sees T_s, not T_m.
    // Bounded by maxHeaterOvershoot [°C] → PWM via G_eff.
    const double boostPWM = (P_ff > 0.0 && G_eff > 0.0 &&
                             params_.heaterPowerScale > 0.0)
        ? std::min(oneMinusAlpha * params_.maxHeaterOvershoot *
                       G_eff / params_.heaterPowerScale,
                   params_.maxPreEmphasisPower)
        : 0.0;

    double prePWM = std::clamp(uncompensatedPffPWM + boostPWM, 0.0,
                               params_.maxPreEmphasisPower);
    // Suppress pre-emphasis during warmup (far from target).
    if (std::abs(measured - target) > params_.maxHeaterOvershoot) {
        prePWM = 0.0;
    }

    // --- Post-emphasis: thermal-debt relaxation ---
    // D_target(Q) = P_ff (steady-state enthalpy power). When flow stops,
    // D relaxes back to 0 with τ_debt, supplying a decaying power offset.
    //
    // The PID will partially compensate the recovery through the sensor
    // coupling (fraction α), so we only feed-forward (1-α) of the deficit.
    const double D_target = P_ff;
    if (params_.debtTimeConstant > 0.0) {
        thermalDebt_ += (D_target - thermalDebt_) / params_.debtTimeConstant * dt;
    } else {
        thermalDebt_ = D_target;
    }
    // The deficit is (D_target - D): when flow stops, D_target→0 but D is
    // still positive, so the deficit goes negative. We add a decaying
    // positive power proportional to -(deficit), scaled by (1-α).
    const double deficit = D_target - thermalDebt_; // [W], negative after stop
    double postPWM = (params_.heaterPowerScale > 0.0)
        ? std::clamp(-oneMinusAlpha * deficit / params_.heaterPowerScale,
                     0.0, params_.maxPostEmphasisPower)
        : 0.0;

    // --- PID backend ---
    auto pidOut = pid_.compute(input);
    double pwm = pidOut.control + prePWM + postPWM;
    pwm = std::clamp(pwm, 0.0, 1.0);

    // --- Update the thermal observer with the applied PWM and sensor reading ---
    // The measured temperature IS the sensor reading, so we use
    // updateWithMeasurement to close the observer loop.
    observer_.updateWithMeasurement(pwm, Q, measured, dt);

    // --- Diagnostics ---
    emphasis_.preEmphasisPWM = prePWM;
    emphasis_.postEmphasisPWM = postPWM;
    emphasis_.thermalDebt = thermalDebt_;
    emphasis_.sensorCouplingAlpha = alpha;

    ::Control::ControllerOutput out = pidOut;
    out.control = pwm;
    out.feedforward = prePWM + postPWM;
    return out;
}

} // namespace tether::control::extrusion
