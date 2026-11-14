#include "tether/control/thermal/TemperatureProfile.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

namespace {
constexpr double kEps = 1e-12;
}

void TemperatureProfile::addSoak(double durationS) {
    if (durationS <= kEps) return;
    const double T = endTemp();
    m_segs.push_back({m_tEnd, m_tEnd + durationS, T, T});
    m_tEnd += durationS;
}

void TemperatureProfile::addRampTo(double tempC, double rateCPerS) {
    const double T0 = endTemp();
    if (std::abs(tempC - T0) <= kEps) return;
    const double rate = std::abs(rateCPerS) > kEps ? std::abs(rateCPerS) : 1.0;
    const double duration = std::abs(tempC - T0) / rate;
    m_segs.push_back({m_tEnd, m_tEnd + duration, T0, tempC});
    m_tEnd += duration;
}

void TemperatureProfile::addRampSoak(double tempC, double rateCPerS, double durationS) {
    addRampTo(tempC, rateCPerS);
    if (durationS > kEps) addSoak(durationS);
}

TemperatureProfile TemperatureProfile::fromPoints(
    const std::vector<std::pair<double, double>>& points) {
    TemperatureProfile p(points.empty() ? 25.0 : points.front().second);
    if (points.size() < 2) return TemperatureProfile{};
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].first <= points[i - 1].first) return TemperatureProfile{};
    }
    p.m_segs.reserve(points.size() - 1);
    p.m_tEnd = points.back().first;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        p.m_segs.push_back({points[i].first, points[i + 1].first,
                            points[i].second, points[i + 1].second});
    }
    return p;
}

TemperatureProfile TemperatureProfile::transformed(double tempOffsetC,
                                                 double timeShiftS) const {
    TemperatureProfile p(m_initialTemp + tempOffsetC);
    p.m_segs.reserve(m_segs.size());
    for (const auto& s : m_segs) {
        p.m_segs.push_back({s.t0 + timeShiftS, s.t1 + timeShiftS,
                            s.T0 + tempOffsetC, s.T1 + tempOffsetC});
    }
    p.m_tEnd = m_tEnd + timeShiftS;
    return p;
}

double TemperatureProfile::value(double t) const {
    if (m_segs.empty()) return m_initialTemp;
    if (t <= m_segs.front().t0) return m_segs.front().T0;
    if (t >= m_tEnd) return m_segs.back().T1;
    // Linear scan is fine: oven profiles have tens of segments at most.
    for (const auto& s : m_segs) {
        if (t <= s.t1) {
            const double f = (s.t1 > s.t0) ? (t - s.t0) / (s.t1 - s.t0) : 0.0;
            return s.T0 + f * (s.T1 - s.T0);
        }
    }
    return m_segs.back().T1;
}

double TemperatureProfile::derivative(double t) const {
    if (m_segs.empty() || t < m_segs.front().t0 || t >= m_tEnd) return 0.0;
    for (const auto& s : m_segs) {
        if (t < s.t1 || (t == s.t1 && &s == &m_segs.back())) {
            return (s.t1 > s.t0) ? (s.T1 - s.T0) / (s.t1 - s.t0) : 0.0;
        }
    }
    return 0.0;
}

} // namespace tether::control::thermal
