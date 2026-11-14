#include "tether/control/thermal/ThermalParameterEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

void ThermalParameterEstimator::setConfig(const EstimatorConfig& cfg) {
    m_cfg = cfg;
    while (m_win.size() > m_cfg.maxSamples) m_win.pop_front();
}

void ThermalParameterEstimator::setPrior(double alpha, double beta,
                                         double disturbance) {
    m_theta = Eigen::Vector3d(alpha, beta, disturbance);
    m_theta.x() = std::clamp(m_theta.x(), m_cfg.alphaMin, m_cfg.alphaMax);
    m_theta.y() = std::clamp(m_theta.y(), m_cfg.betaMin, m_cfg.betaMax);
    m_theta.z() = std::clamp(m_theta.z(), m_cfg.dMin, m_cfg.dMax);
    m_S.setZero();
    m_s.setZero();
}

void ThermalParameterEstimator::reset() {
    m_S.setZero();
    m_s.setZero();
    m_win.clear();
    m_time = 0.0;
    m_cycles = 0;
    m_updatesUsed = 0;
    m_lastExcited = false;
    m_lastResidual = 0.0;
}

void ThermalParameterEstimator::update(double uDelayed, double tempFilt,
                                        double ambient, double dt) {
    if (dt <= 0.0) return;
    ++m_cycles;

    m_win.push_back({m_time, uDelayed, tempFilt});
    m_time += dt;

    const double cutoff = m_time - m_cfg.windowTime;
    while (!m_win.empty() && m_win.front().t < cutoff) m_win.pop_front();
    while (m_win.size() > m_cfg.maxSamples) m_win.pop_front();

    // Time-based forgetting applies every cycle so memory is uniform
    // whether or not the current window is excited.
    const double lambda = std::exp(-dt / std::max(m_cfg.memTime, 1e-6));
    m_S *= lambda;
    m_s *= lambda;

    if (m_win.size() < 2) return;
    const double span = m_win.back().t - m_win.front().t;
    if (span < 0.5 * m_cfg.windowTime) return;   // window still filling

    // Single pass: sums + extremes for the integral-form equation and
    // the excitation test.
    double sumU = 0.0, sumT = 0.0;
    double uLo = 1e30, uHi = -1e30, tLo = 1e30, tHi = -1e30;
    for (const auto& smp : m_win) {
        sumU += smp.u;
        sumT += smp.T;
        uLo = std::min(uLo, smp.u);
        uHi = std::max(uHi, smp.u);
        tLo = std::min(tLo, smp.T);
        tHi = std::max(tHi, smp.T);
    }
    const double n = static_cast<double>(m_win.size());
    const double meanU = sumU / n;
    const double meanX = sumT / n - ambient;
    const double yRate = (m_win.back().T - m_win.front().T) / span;

    const double uRange = uHi - uLo;
    const double tRange = tHi - tLo;
    const bool excited = (tRange > m_cfg.noiseFloor) &&
                         (uRange > m_cfg.uExcursionMin ||
                          tRange > m_cfg.tempSpanMin);
    m_lastExcited = excited;
    if (!excited) {
        m_lastResidual = 0.0;
        return;
    }

    // Regressor for dT/dt = alpha*u_d - beta*(T-Tamb) + d:
    //   phi = [mean(u_d), -mean(T-Tamb), 1]
    const Eigen::Vector3d phi(meanU, -meanX, 1.0);

    ++m_updatesUsed;
    m_S.noalias() += phi * phi.transpose();
    m_s.noalias() += phi * yRate;

    // Regularized solve toward the current estimate (prior).
    Eigen::Matrix3d A = m_S;
    A.diagonal().array() += m_cfg.ridge;
    Eigen::Vector3d b = m_s + m_cfg.ridge * m_theta;
    Eigen::Vector3d thetaNew = A.ldlt().solve(b);

    // Projection onto the physical box.
    thetaNew.x() = std::clamp(thetaNew.x(), m_cfg.alphaMin, m_cfg.alphaMax);
    thetaNew.y() = std::clamp(thetaNew.y(), m_cfg.betaMin, m_cfg.betaMax);
    thetaNew.z() = std::clamp(thetaNew.z(), m_cfg.dMin, m_cfg.dMax);

    const double g = std::clamp(m_cfg.smoothing, 0.0, 1.0);
    m_theta = (1.0 - g) * m_theta + g * thetaNew;
    m_lastResidual = yRate - phi.dot(m_theta);
}

} // namespace tether::control::thermal
