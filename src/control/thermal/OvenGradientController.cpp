#include "tether/control/thermal/OvenGradientController.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

void OvenGradientController::addZone(const GradientZoneConfig& cfg) {
    auto z = std::make_unique<Zone>();
    z->cfg = cfg;
    z->ctl = std::make_unique<OvenController>(cfg.controller);
    z->setpoint = cfg.controller.model.ambient;
    m_zones.push_back(std::move(z));
    m_status.emplace_back();
    m_lastU.push_back(0.0);
    m_lastMeasured.push_back(cfg.controller.model.ambient);
    m_coupleD.push_back(0.0);
}

void OvenGradientController::setZoneProfile(size_t zone,
                                            const TemperatureProfile& p) {
    Zone& z = *m_zones.at(zone);
    z.profile = p;
    z.hasProfile = !p.empty();
    z.ctl->setProfile(p);
}

void OvenGradientController::setZoneSetpoint(size_t zone, double tempC) {
    Zone& z = *m_zones.at(zone);
    z.setpoint = tempC;
    z.hasProfile = false;
    z.profile.clear();
    z.ctl->clearProfile();
}

void OvenGradientController::setZoneSetpoints(
    const std::vector<double>& tempsC, double maxDeltaC) {
    const size_t n = std::min(tempsC.size(), m_zones.size());
    // Forward pass: clamp each setpoint into the previous zone's
    // allowed band so |sp_i - sp_{i-1}| <= maxDeltaC.
    std::vector<double> sp = tempsC;
    if (maxDeltaC > 0.0) {
        for (size_t i = 1; i < n; ++i) {
            sp[i] = std::clamp(sp[i], sp[i - 1] - maxDeltaC,
                               sp[i - 1] + maxDeltaC);
        }
    }
    for (size_t i = 0; i < n; ++i) setZoneSetpoint(i, sp[i]);
}

bool OvenGradientController::setGradientProfile(const GradientSpec& spec) {
    if (m_zones.empty() || spec.base.empty()) return false;

    const size_t n = m_zones.size();
    std::vector<double> off(n, 0.0), shift(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        if (i < spec.tempOffsetC.size()) off[i] = spec.tempOffsetC[i];
        if (i < spec.timeShiftS.size()) shift[i] = spec.timeShiftS[i];
    }

    // Clamp adjacent offset deltas: |off_i - off_{i-1}| <= maxZoneDeltaC.
    // Forward pass from zone 0 — correct for monotone gradients and
    // feasible for arbitrary ones (each zone pulled toward its upstream
    // neighbor's band).
    if (spec.maxZoneDeltaC > 0.0) {
        for (size_t i = 1; i < n; ++i) {
            off[i] = std::clamp(off[i], off[i - 1] - spec.maxZoneDeltaC,
                                off[i - 1] + spec.maxZoneDeltaC);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        setZoneProfile(i, spec.base.transformed(off[i], shift[i]));
    }
    return true;
}

std::vector<double> OvenGradientController::compute(
    std::span<const double> measured, double dt) {
    if (measured.size() != m_zones.size()) return {};
    std::vector<double> u(m_zones.size(), 0.0);
    computeInto(measured, dt, u);
    return u;
}

bool OvenGradientController::computeInto(
    std::span<const double> measured, double dt,
    std::span<double> out) {
    const size_t n = m_zones.size();
    if (measured.size() != n || out.size() != n) return false;

    // Pass 1: coupling disturbance per zone from measured neighbor temps.
    // d_i [°C/s] = γL·(T_{i-1} − T_i) + γR·(T_{i+1} − T_i); boundary zones
    // have only the inward neighbor.
    for (size_t i = 0; i < n; ++i) {
        const double Ti = measured[i];
        double di = 0.0;
        if (i > 0)
            di += m_zones[i]->cfg.couplingLeft * (measured[i - 1] - Ti);
        if (i + 1 < n)
            di += m_zones[i]->cfg.couplingRight * (measured[i + 1] - Ti);
        m_coupleD[i] = di;
    }

    // Pass 2: zone controllers.
    m_maxRefDelta = 0.0;
    for (size_t i = 0; i < n; ++i) {
        Zone& z = *m_zones[i];
        z.ctl->setCouplingDisturbance(m_coupleD[i]);

        ControllerInput in;
        in.measured = measured[i];
        in.dt = dt;
        if (!z.hasProfile) in.reference = z.setpoint;
        const ControllerOutput co = z.ctl->compute(in);
        out[i] = co.control;

        ZoneStatus& s = m_status[i];
        s.controller = z.ctl->status();
        s.couplingDisturb = m_coupleD[i];
        s.measDeltaPrev = (i > 0) ? measured[i] - measured[i - 1] : 0.0;
        const double ref = s.controller.reference;
        s.refDeltaPrev = (i > 0)
            ? ref - m_status[i - 1].controller.reference : 0.0;
        if (i > 0)
            m_maxRefDelta = std::max(m_maxRefDelta, std::abs(s.refDeltaPrev));
    }

    // Alarm on measured adjacent-zone delta.
    double maxD = 0.0;
    for (size_t i = 1; i < n; ++i)
        maxD = std::max(maxD, std::abs(measured[i] - measured[i - 1]));
    m_alarm = (m_alarmLimitC > 0.0) && (maxD > m_alarmLimitC);

    for (size_t i = 0; i < n; ++i) {
        m_lastU[i] = out[i];
        m_lastMeasured[i] = measured[i];
    }
    return true;
}

double OvenGradientController::maxMeasuredDelta() const {
    double maxD = 0.0;
    for (size_t i = 1; i < m_lastMeasured.size(); ++i)
        maxD = std::max(maxD,
                        std::abs(m_lastMeasured[i] - m_lastMeasured[i - 1]));
    return maxD;
}

double OvenGradientController::maxSetpointDelta() const {
    return m_maxRefDelta;
}

void OvenGradientController::reset() {
    for (auto& z : m_zones) {
        z->ctl->reset();
        z->ctl->setCouplingDisturbance(0.0);
    }
    for (auto& s : m_status) s = ZoneStatus{};
    std::fill(m_lastU.begin(), m_lastU.end(), 0.0);
    m_maxRefDelta = 0.0;
    m_alarm = false;
}

} // namespace tether::control::thermal
