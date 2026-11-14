#include "tether/control/thermal/ActuationGovernor.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

void ActuationGovernor::update(double t, const TemperatureProfile& profile, double dt) {
    if (dt <= 0.0 || profile.empty()) {
        hold(dt > 0.0 ? dt : 0.0);
        return;
    }

    // Evaluate the profile at the preview horizon so the governed reference
    // starts moving before a segment boundary when physics requires it.
    const double tp = t + m_cfg.previewTime;
    const double tDes = profile.value(tp);
    const double dDes = profile.derivative(tp);

    // Desired rate: profile slope plus a proportional pull toward the level.
    const double err = tDes - m_r;
    const double dCmd = dDes + m_cfg.trackingGain * err;

    // Achievable rate band at the current governed temperature.
    const double dT = m_r - m_cfg.ambient;
    const double coolRate = m_cfg.alpha * m_cfg.uMin - m_cfg.beta * dT + m_cfg.disturbance;
    const double heatRate = m_cfg.alpha * m_cfg.uMax - m_cfg.beta * dT + m_cfg.disturbance;
    const double lo = std::min(coolRate, heatRate);
    const double hi = std::max(coolRate, heatRate);

    m_rate = std::clamp(dCmd, lo, hi);
    m_r += m_rate * dt;
}

void ActuationGovernor::hold(double dt) {
    // No profile: decay the rate toward zero and hold position.
    (void)dt;
    m_rate = 0.0;
}

} // namespace tether::control::thermal
