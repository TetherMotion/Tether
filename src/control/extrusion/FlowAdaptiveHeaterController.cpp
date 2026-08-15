/**
 * @file FlowAdaptiveHeaterController.cpp
 * @brief PID + flow pre/post-emphasis feed-forward implementation.
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

    // --- Steady-state enthalpy power required to maintain target at flow Q ---
    // P_ff = ρ c_p Q (T_target - T_inlet)  [W]
    const double P_ff = (target - params_.inletTempC) *
                        params_.filamentHeatCapacity * Q;
    const double P_ff_pwm = (params_.heaterPowerScale > 0.0)
        ? P_ff / params_.heaterPowerScale : 0.0;

    // --- Pre-emphasis: apply P_ff ahead of the thermal lag, plus a
    // heater-block boost to establish the gradient ΔT_hm = P_ff / G_hm.
    // The boost is bounded by maxHeaterOvershoot (in °C, converted to PWM
    // via G_hm and heaterPowerScale). The boost is only added when there is
    // actual flow (P_ff > 0); with no flow there is no gradient to establish.
    const double boostPWM = (P_ff > 0.0 && params_.heaterMeltConductance > 0.0 &&
                             params_.heaterPowerScale > 0.0)
        ? std::min(params_.maxHeaterOvershoot *
                       params_.heaterMeltConductance /
                       params_.heaterPowerScale,
                   params_.maxPreEmphasisPower)
        : 0.0;
    double prePWM = std::clamp(P_ff_pwm + boostPWM, 0.0,
                               params_.maxPreEmphasisPower);
    // Only apply pre-emphasis when we are at/near target (avoid fighting
    // the PID during initial warmup).
    if (std::abs(measured - target) > params_.maxHeaterOvershoot) {
        prePWM = 0.0;
    }

    // --- Post-emphasis: thermal-debt relaxation.
    // D_target(Q) = P_ff (the steady-state enthalpy power). When flow stops,
    // D relaxes back to 0 with τ_debt, supplying a decaying power offset.
    const double D_target = P_ff;
    if (params_.debtTimeConstant > 0.0) {
        thermalDebt_ += (D_target - thermalDebt_) / params_.debtTimeConstant * dt;
    } else {
        thermalDebt_ = D_target;
    }
    // The post-emphasis term is the *deficit* (D_target - D): when flow stops,
    // D_target→0 but D is still positive, so the deficit goes negative and we
    // add a decaying positive power to compensate the dip. We expose the
    // signed debt and clamp the applied PWM.
    const double deficit = D_target - thermalDebt_; // [W], negative after stop
    double postPWM = (params_.heaterPowerScale > 0.0)
        ? std::clamp(-deficit / params_.heaterPowerScale, 0.0,
                     params_.maxPostEmphasisPower)
        : 0.0;

    // --- PID backend ---
    auto pidOut = pid_.compute(input);
    double pwm = pidOut.control + prePWM + postPWM;
    pwm = std::clamp(pwm, 0.0, 1.0);

    // --- Update the thermal observer with the applied PWM ---
    observer_.update(pwm, Q, dt);

    // --- Diagnostics ---
    emphasis_.preEmphasisPWM = prePWM;
    emphasis_.postEmphasisPWM = postPWM;
    emphasis_.thermalDebt = thermalDebt_;

    ::Control::ControllerOutput out = pidOut;
    out.control = pwm;
    out.feedforward = prePWM + postPWM;
    return out;
}

} // namespace tether::control::extrusion
