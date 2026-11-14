/**
 * @file ActuationGovernor.hpp
 * @brief Shapes a temperature profile into the physically achievable
 *        reference trajectory for a one-sided (heat-only) oven actuator.
 *
 * @details
 * The governor rate-limits the reference so that the achievable
 * dT/dt band of the thermal model is never exceeded:
 *
 *   coolRate = α·uMin - β·(r - T_amb) + d     (passive cooling at uMin)
 *   heatRate = α·uMax - β·(r - T_amb) + d     (max heating at uMax)
 *
 * The governed reference r̂ moves toward the desired profile value with a
 * proportional pull plus the profile slope, clamped into [coolRate,
 * heatRate]. When the profile demands faster cooling than passive loss
 * allows, r̂ simply descends at the passive rate — feedback can never fix
 * an infeasible trajectory, so this is where feasibility is enforced.
 *
 * A preview horizon evaluates the profile `previewTime` ahead so cooling
 * can start before a descent segment begins.
 */

#pragma once

#include "tether/control/thermal/TemperatureProfile.hpp"

namespace tether::control::thermal {

struct GovernorConfig {
    double alpha{0.02};        /**< Model heating coefficient [°C/s/u] */
    double beta{0.005};        /**< Model loss coefficient [1/s] */
    double uMin{0.0};
    double uMax{1.0};
    double ambient{25.0};      /**< [°C] */
    double disturbance{0.0};   /**< [°C/s] */
    double trackingGain{0.05}; /**< Pull toward profile level [1/s] */
    double previewTime{0.0};   /**< Profile lookahead for anticipation [s] */
};

class ActuationGovernor {
public:
    explicit ActuationGovernor(const GovernorConfig& cfg = {}) : m_cfg(cfg) {}

    void setConfig(const GovernorConfig& cfg) { m_cfg = cfg; }
    const GovernorConfig& config() const { return m_cfg; }

    /**@brief Advance the governor one step toward the profile.
     * @param t       Current time [s] (same clock as the profile).
     * @param profile Desired temperature profile.
     * @param dt      Step size [s].
     */
    void update(double t, const TemperatureProfile& profile, double dt);

    /**@brief Hold the current value (no profile / idle). */
    void hold(double dt);

    /**@brief Governed (achievable) reference temperature [°C]. */
    double value() const { return m_r; }

    /**@brief Current governed reference rate [°C/s]. */
    double rate() const { return m_rate; }

    /**@brief Linear-extrapolated governed reference `ahead` seconds out. */
    double previewValue(double ahead) const { return m_r + m_rate * ahead; }

    void reset(double initialRef) { m_r = initialRef; m_rate = 0.0; }

private:
    GovernorConfig m_cfg;
    double m_r{25.0};
    double m_rate{0.0};
};

} // namespace tether::control::thermal
