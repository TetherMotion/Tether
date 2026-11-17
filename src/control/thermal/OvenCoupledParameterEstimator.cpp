#include "tether/control/thermal/OvenCoupledParameterEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

void OvenCoupledParameterEstimator::setConfig(
    const CoupledEstimatorConfig& cfg) {
    m_cfg = cfg;
    while (m_win.size() > m_cfg.maxSamples) m_win.pop_front();
}

void OvenCoupledParameterEstimator::setZones(
    const std::vector<CoupledZoneParams>& priors) {
    m_n = static_cast<int>(priors.size());
    m_zones.assign(priors.size(), ZoneEst{});
    for (size_t i = 0; i < priors.size(); ++i) {
        ZoneEst& z = m_zones[i];
        const bool hasL = (i > 0);
        const bool hasR = (i + 1 < priors.size());
        z.hasLeft = hasL;
        z.hasRight = hasR;
        // Edge couplings are structurally absent -> pin to zero.
        const double cl = hasL ? priors[i].couplingLeft : 0.0;
        const double cr = hasR ? priors[i].couplingRight : 0.0;
        z.theta << priors[i].alpha, priors[i].beta, cl, cr,
            priors[i].disturbance;
        z.theta(0) = std::clamp(z.theta(0), m_cfg.alphaMin, m_cfg.alphaMax);
        z.theta(1) = std::clamp(z.theta(1), m_cfg.betaMin, m_cfg.betaMax);
        z.theta(2) = std::clamp(z.theta(2), m_cfg.couplingMin, m_cfg.couplingMax);
        z.theta(3) = std::clamp(z.theta(3), m_cfg.couplingMin, m_cfg.couplingMax);
        z.theta(4) = std::clamp(z.theta(4), m_cfg.dMin, m_cfg.dMax);
        z.S.setZero();
        z.s.setZero();
        z.updatesUsed = 0;
        z.lastExcited = false;
        z.lastResidual = 0.0;
    }
    m_win.clear();
    m_time = 0.0;
    m_cycles = 0;
}

void OvenCoupledParameterEstimator::reset() {
    for (auto& z : m_zones) {
        z.S.setZero();
        z.s.setZero();
        z.updatesUsed = 0;
        z.lastExcited = false;
        z.lastResidual = 0.0;
    }
    m_win.clear();
    m_time = 0.0;
    m_cycles = 0;
}

void OvenCoupledParameterEstimator::update(
    std::span<const double> uDelayed, std::span<const double> measured,
    std::span<const double> ambient, double dt) {
    const int n = m_n;
    if (dt <= 0.0 || n <= 0) return;
    if (uDelayed.size() < static_cast<size_t>(n) ||
        measured.size() < static_cast<size_t>(n) ||
        ambient.size() < static_cast<size_t>(n))
        return;
    ++m_cycles;

    Sample smp{};
    smp.t = m_time;
    for (int i = 0; i < n; ++i) {
        smp.u[static_cast<size_t>(i)] = uDelayed[static_cast<size_t>(i)];
        smp.T[static_cast<size_t>(i)] = measured[static_cast<size_t>(i)];
    }
    m_win.push_back(smp);
    m_time += dt;

    const double cutoff = m_time - m_cfg.windowTime;
    while (!m_win.empty() && m_win.front().t < cutoff) m_win.pop_front();
    while (m_win.size() > m_cfg.maxSamples) m_win.pop_front();

    // Time-based forgetting applies every cycle so memory is uniform
    // whether or not the current window is excited.
    const double lambda = std::exp(-dt / std::max(m_cfg.memTime, 1e-6));
    for (auto& z : m_zones) {
        z.S *= lambda;
        z.s *= lambda;
    }

    if (m_win.size() < 2) return;
    const double span = m_win.back().t - m_win.front().t;
    if (span < 0.5 * m_cfg.windowTime) return;   // window still filling

    // Single pass: column sums + per-zone extremes for the integral-form
    // equations and the excitation test (incl. neighbor-delta ranges).
    std::array<double, MAX_CONTROL_DIM> sumU{}, sumT{};
    std::array<double, MAX_CONTROL_DIM> uLo, uHi, tLo, tHi;
    std::array<double, MAX_CONTROL_DIM> dLLo, dLHi, dRLo, dRHi;
    for (int i = 0; i < n; ++i) {
        uLo[i] = 1e30; uHi[i] = -1e30;
        tLo[i] = 1e30; tHi[i] = -1e30;
        dLLo[i] = 1e30; dLHi[i] = -1e30;
        dRLo[i] = 1e30; dRHi[i] = -1e30;
    }

    for (const auto& smp : m_win) {
        for (int i = 0; i < n; ++i) {
            const double u = smp.u[static_cast<size_t>(i)];
            const double T = smp.T[static_cast<size_t>(i)];
            sumU[i] += u;
            sumT[i] += T;
            uLo[i] = std::min(uLo[i], u); uHi[i] = std::max(uHi[i], u);
            tLo[i] = std::min(tLo[i], T); tHi[i] = std::max(tHi[i], T);
        }
        for (int i = 0; i < n; ++i) {
            if (i > 0) {
                const double d =
                    smp.T[static_cast<size_t>(i - 1)] -
                    smp.T[static_cast<size_t>(i)];
                dLLo[i] = std::min(dLLo[i], d); dLHi[i] = std::max(dLHi[i], d);
            }
            if (i + 1 < n) {
                const double d =
                    smp.T[static_cast<size_t>(i + 1)] -
                    smp.T[static_cast<size_t>(i)];
                dRLo[i] = std::min(dRLo[i], d); dRHi[i] = std::max(dRHi[i], d);
            }
        }
    }

    const double cnt = static_cast<double>(m_win.size());
    const double g = std::clamp(m_cfg.smoothing, 0.0, 1.0);

    for (int i = 0; i < n; ++i) {
        ZoneEst& z = m_zones[static_cast<size_t>(i)];

        const double meanU = sumU[i] / cnt;
        const double meanTi = sumT[i] / cnt;
        const double amb = ambient[static_cast<size_t>(i)];

        const double uRange = uHi[i] - uLo[i];
        const double tRange = tHi[i] - tLo[i];
        double coupRange = 0.0;
        if (z.hasLeft && dLHi[i] > -1e29)
            coupRange = std::max(coupRange, dLHi[i] - dLLo[i]);
        if (z.hasRight && dRHi[i] > -1e29)
            coupRange = std::max(coupRange, dRHi[i] - dRLo[i]);

        const bool excited =
            (tRange > m_cfg.noiseFloor) &&
            (uRange > m_cfg.uExcursionMin ||
             tRange > m_cfg.tempSpanMin ||
             coupRange > m_cfg.tempSpanMin);
        z.lastExcited = excited;
        if (!excited) {
            z.lastResidual = 0.0;
            continue;
        }

        // Integral-form equation for zone i:
        //   dT_i/dt = a*u_i,d - b*(T_i-Tamb) + gL*(T_{i-1}-T_i)
        //             + gR*(T_{i+1}-T_i) + d
        // phi = [meanU, -(meanT_i-Tamb), mean(T_{i-1}-T_i),
        //        mean(T_{i+1}-T_i), 1]
        Vec phi = Vec::Zero();
        phi(0) = meanU;
        phi(1) = -(meanTi - amb);
        if (z.hasLeft)
            phi(2) = sumT[i - 1] / cnt - meanTi;
        if (z.hasRight)
            phi(3) = sumT[i + 1] / cnt - meanTi;
        phi(4) = 1.0;

        const double yRate =
            (m_win.back().T[static_cast<size_t>(i)] -
             m_win.front().T[static_cast<size_t>(i)]) /
            span;

        ++z.updatesUsed;
        z.S.noalias() += phi * phi.transpose();
        z.s.noalias() += phi * yRate;

        // Regularized solve toward the current estimate.
        Mat A = z.S;
        A.diagonal().array() += m_cfg.ridge;
        Vec b = z.s + m_cfg.ridge * z.theta;
        Vec thetaNew = A.ldlt().solve(b);

        // Projection onto the physical box.
        thetaNew(0) = std::clamp(thetaNew(0), m_cfg.alphaMin, m_cfg.alphaMax);
        thetaNew(1) = std::clamp(thetaNew(1), m_cfg.betaMin, m_cfg.betaMax);
        thetaNew(2) = std::clamp(thetaNew(2), m_cfg.couplingMin,
                                 m_cfg.couplingMax);
        thetaNew(3) = std::clamp(thetaNew(3), m_cfg.couplingMin,
                                 m_cfg.couplingMax);
        thetaNew(4) = std::clamp(thetaNew(4), m_cfg.dMin, m_cfg.dMax);

        z.theta = (1.0 - g) * z.theta + g * thetaNew;
        z.lastResidual = yRate - phi.dot(z.theta);
    }
}

CoupledZoneEstimate OvenCoupledParameterEstimator::estimate(
    size_t zone) const {
    CoupledZoneEstimate e;
    if (zone >= m_zones.size()) return e;
    const Vec& t = m_zones[zone].theta;
    e.alpha = t(0);
    e.beta = t(1);
    e.couplingLeft = t(2);
    e.couplingRight = t(3);
    e.disturbance = t(4);
    return e;
}

int OvenCoupledParameterEstimator::updatesUsed(size_t zone) const {
    return zone < m_zones.size() ? m_zones[zone].updatesUsed : 0;
}

bool OvenCoupledParameterEstimator::lastExcited(size_t zone) const {
    return zone < m_zones.size() ? m_zones[zone].lastExcited : false;
}

double OvenCoupledParameterEstimator::lastResidual(size_t zone) const {
    return zone < m_zones.size() ? m_zones[zone].lastResidual : 0.0;
}

} // namespace tether::control::thermal
