/**
 * @file TemperatureProfile.hpp
 * @brief Ramp/soak temperature profile for oven control.
 *
 * @details
 * Piecewise-linear temperature reference r(t) built either from explicit
 * ramp/soak segments or from an arbitrary (t, T) point list — the latter
 * covers reflow-style curves directly.
 *
 * value(t) and derivative(t) are analytic, so the model-inverse
 * feedforward can use the exact profile slope rather than a finite
 * difference of a sampled setpoint.
 *
 * The profile starts at `initialTempC` at t=0; the first segment begins
 * there. Before the first segment point and after the last, the profile
 * holds its boundary value.
 */

#pragma once

#include <utility>
#include <vector>

namespace tether::control::thermal {

class TemperatureProfile {
public:
    /**@param initialTempC Temperature held at t=0 before the first segment. */
    explicit TemperatureProfile(double initialTempC = 25.0)
        : m_initialTemp(initialTempC) {}

    /**@brief Hold the current end temperature for `durationS` seconds. */
    void addSoak(double durationS);

    /**@brief Ramp from the current end temperature to `tempC`.
     *        The magnitude of `rateCPerS` sets the ramp rate; direction
     *        follows the target. */
    void addRampTo(double tempC, double rateCPerS);

    /**@brief Ramp to `tempC` at `rateCPerS`, then hold it for `durationS`. */
    void addRampSoak(double tempC, double rateCPerS, double durationS);

    /**@brief Build a piecewise-linear profile from (time, temp) points.
     *        Points must have strictly increasing times. Before the first
     *        point the profile holds points[0].second; after the last it
     *        holds points[N-1].second.
     *        @return empty profile if fewer than 2 points or times not
     *                increasing. */
    static TemperatureProfile fromPoints(
        const std::vector<std::pair<double, double>>& points);

    /**@brief Reference temperature at time t [°C]. */
    double value(double t) const;

    /**@brief Reference temperature rate at time t [°C/s].
     *        At segment knots the slope of the segment starting at t is
     *        returned; outside the profile the rate is 0. */
    double derivative(double t) const;

    /**@brief Total profile duration [s] (end of last segment). */
    double duration() const { return m_tEnd; }

    /**@brief Temperature at t=0 / before the first segment. */
    double initialTemp() const { return m_initialTemp; }

    bool empty() const { return m_segs.empty(); }
    void clear() {
        m_segs.clear();
        m_tEnd = 0.0;
        // m_initialTemp retained: clearing segments does not move t=0.
    }

private:
    struct Seg {
        double t0, t1;   // [s]
        double T0, T1;   // [°C]
    };

    double endTemp() const {
        return m_segs.empty() ? m_initialTemp : m_segs.back().T1;
    }

    double m_initialTemp;
    std::vector<Seg> m_segs;
    double m_tEnd{0.0};
};

} // namespace tether::control::thermal
