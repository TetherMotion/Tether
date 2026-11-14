#include "tether/control/thermal/OvenThermalModel.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

double OvenThermalModel::step(double u, double dt) {
    if (dt <= 0.0) return m_T;

    // Record the command at the start of this interval; its thermal effect
    // arrives `delay` seconds later.
    m_delayLine.push(m_time, u);
    const double uDelayed = m_delayLine.value(m_time - m_params.delay);
    m_time += dt;

    const double alpha = m_params.alpha;
    const double beta = m_params.beta;
    const double heat = alpha * uDelayed + m_params.disturbance;

    if (beta > 1e-9) {
        // Exact ZOH update of dT/dt = alpha*u - beta*(T-tAmb) + d.
        const double a = std::exp(-beta * dt);
        const double tEq = m_params.ambient + heat / beta;
        m_T = tEq + (m_T - tEq) * a;
    } else {
        // Integrating limit (beta ~ 0): pure accumulation.
        m_T += (heat - beta * (m_T - m_params.ambient)) * dt;
    }
    return m_T;
}

std::vector<double> OvenThermalModel::simulate(double uConst, double dt, int steps,
                                             double tAmb, double d) const {
    std::vector<double> out;
    out.reserve(static_cast<size_t>(steps) + 1);
    out.push_back(m_T);

    const double alpha = m_params.alpha;
    const double beta = m_params.beta;
    const int nDelay = m_params.delay > 0.0 && dt > 0.0
        ? static_cast<int>(std::lround(m_params.delay / dt)) : 0;

    // Input delay line over the simulation horizon. Pre-step history is
    // u=0 (heater off before t=0) so dead time is visible in step tests.
    std::deque<double> uHist(static_cast<size_t>(nDelay) + 1, 0.0);

    double T = m_T;
    for (int k = 0; k < steps; ++k) {
        // u applied now affects output nDelay steps later.
        double uEff = uHist.front();
        uHist.pop_front();
        uHist.push_back(uConst);

        const double loss = beta * (T - tAmb);
        const double heat = alpha * uEff + d;
        if (beta > 1e-9) {
            // Exact ZOH update of dT/dt = alpha*u - beta*(T-tAmb) + d.
            const double a = std::exp(-beta * dt);
            const double tEq = tAmb + heat / beta;
            T = tEq + (T - tEq) * a;
        } else {
            // Integrating limit (beta ~ 0): pure accumulation.
            T += (heat - loss) * dt;
        }
        out.push_back(T);
    }
    return out;
}

double OvenThermalModel::requiredInput(double tRef, double tRefDot,
                                       double ambient, double disturb) const {
    // Invert dT/dt = alpha*u - beta*(T - tAmb) + d  at (T=tRef, dT/dt=tRefDot):
    //   alpha*u = tRefDot + beta*(tRef - tAmb) - d
    const double alpha = m_params.alpha;
    if (alpha <= 1e-12) {
        // No heating authority: command minimum (can't do better).
        return m_params.uMin;
    }
    const double loss = m_params.beta * (tRef - ambient);
    return (tRefDot + loss - disturb) / alpha;
}

double OvenThermalModel::requiredInputClamped(double tRef, double tRefDot,
                                              double ambient, double disturb) const {
    return std::clamp(requiredInput(tRef, tRefDot, ambient, disturb),
                      m_params.uMin, m_params.uMax);
}

} // namespace tether::control::thermal
