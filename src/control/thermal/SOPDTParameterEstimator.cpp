#include "tether/control/thermal/SOPDTParameterEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace tether::control::thermal {

void SOPDTParameterEstimator::setConfig(const SOPDTEstimatorConfig& cfg) {
    m_cfg = cfg;
    while (m_hist.size() > m_cfg.maxSamples) m_hist.pop_front();
}

void SOPDTParameterEstimator::setPrior(double alpha, double beta,
                                       double tau2, double disturbance) {
    m_alpha = std::clamp(alpha, m_cfg.alphaMin, m_cfg.alphaMax);
    m_beta = std::clamp(beta, m_cfg.betaMin, m_cfg.betaMax);
    m_tau2 = std::clamp(tau2, m_cfg.tau2Min, m_cfg.tau2Max);
    m_d = std::clamp(disturbance, m_cfg.dMin, m_cfg.dMax);
    m_hist.clear();
    m_lastFitTime = -1e18;
    m_lastValid = false;
}

void SOPDTParameterEstimator::reset() {
    m_hist.clear();
    m_time = 0.0;
    m_lastFitTime = -1e18;
    m_strideAcc = 0.0;
    m_cycles = 0;
    m_updatesUsed = 0;
    m_lastExcited = false;
    m_lastValid = false;
    m_lastResidual = 0.0;
}

void SOPDTParameterEstimator::update(double uDelayed, double tempMeas,
                                     double ambient, double dt) {
    if (dt <= 0.0) return;
    ++m_cycles;
    m_dt = dt;
    m_ambient = ambient;
    m_time += dt;

    // Decimate to the fit grid: keep every ~simStride-th sample so the
    // OE sim runs at dtFit = dt·simStride. Fractional accumulator keeps
    // the average rate exact for non-integer strides.
    m_strideAcc += 1.0;
    const double stride = std::max(1.0, m_cfg.simStride);
    if (m_strideAcc < stride) return;
    m_strideAcc -= stride;

    m_hist.push_back({m_time - dt, uDelayed, tempMeas});
    while (m_hist.size() > m_cfg.maxSamples) m_hist.pop_front();

    if (m_time - m_lastFitTime >= m_cfg.fitInterval) {
        m_lastFitTime = m_time;
        runFit();
    }
}

double SOPDTParameterEstimator::evalCandidate(
        double ta, double tb, double dtFit,
        const std::vector<Sample>& hist, size_t iscore, double ambient,
        double& alphaOut, double& dOut, double& mseOut) const {
    const double beta = 1.0 / ta;
    detail::CascadeZoh zoh;
    zoh.init(beta, tb, dtFit);

    const size_t n = hist.size();
    const double z0 = hist.front().T - ambient;   // assumed initial deviation

    // Three superposed trajectories (system is LTI in (init, v)):
    //   Ru: v = u_d        -> coefficient of alpha
    //   R1: v = 1          -> coefficient of d
    //   H : v = 0, init (z0,z0) -> known initial-state response
    double y1a = 0.0, y2a = 0.0;   // Ru
    double y1b = 0.0, y2b = 0.0;   // R1
    double y1c = z0,   y2c = z0;   // H

    double Suu = 0.0, Sur = 0.0, Srr = 0.0, Suh = 0.0, Srh = 0.0, Syy = 0.0;
    size_t m = 0;

    for (size_t k = 0; k + 1 < n; ++k) {
        const double u = hist[k].u;
        auto [na1, na2] = zoh.step(y1a, y2a, u);
        auto [nb1, nb2] = zoh.step(y1b, y2b, 1.0);
        auto [nc1, nc2] = zoh.step(y1c, y2c, 0.0);
        y1a = na1; y2a = na2;
        y1b = nb1; y2b = nb2;
        y1c = nc1; y2c = nc2;

        const size_t idx = k + 1;             // post-step state ↔ T[idx]
        if (idx < iscore) continue;
        const double rhs = (hist[idx].T - ambient) - y2c;
        const double ru = y2a, r1 = y2b;
        Suu += ru * ru;  Sur += ru * r1;  Srr += r1 * r1;
        Suh += ru * rhs; Srh += r1 * rhs; Syy += rhs * rhs;
        ++m;
    }
    if (m < 8) {
        mseOut = 1e30;
        return 1e30;
    }

    // Ridge-regularized 2x2 solve for {alpha, d} toward the prior.
    const double a0 = m_alpha, d0 = m_d;
    const double asc = std::max(std::abs(a0), 1e-6);
    const double dsc = std::max(std::abs(d0), 0.05);
    const double w = m_cfg.wAD * static_cast<double>(m);
    const double M11 = Suu + w / (asc * asc);
    const double M22 = Srr + w / (dsc * dsc);
    const double M12 = Sur;
    const double r1 = Suh + w * a0 / (asc * asc);
    const double r2 = Srh + w * d0 / (dsc * dsc);
    const double det = M11 * M22 - M12 * M12;
    if (std::abs(det) < 1e-18) {
        mseOut = 1e30;
        return 1e30;
    }
    const double alpha = (r1 * M22 - r2 * M12) / det;
    const double d = (M11 * r2 - M12 * r1) / det;

    const double sse = Syy - 2.0 * (alpha * Suh + d * Srh)
                     + alpha * alpha * Suu + 2.0 * alpha * d * Sur
                     + d * d * Srr;
    const double mse = std::max(sse, 0.0) / static_cast<double>(m);

    alphaOut = alpha;
    dOut = d;
    mseOut = mse;

    // Lag penalty keeps the fit inside the trust region around the prior.
    const double ta0 = m_beta > 1e-9 ? 1.0 / m_beta : 1e9;
    const double tb0 = m_tau2;
    const double l1 = std::log(ta / std::max(ta0, 1e-9));
    const double l2 = std::log(tb / std::max(tb0, 1e-9));
    const double pen = m_cfg.wLag * (l1 * l1 + l2 * l2);
    return mse * (1.0 + pen);
}

void SOPDTParameterEstimator::runFit() {
    m_lastValid = false;

    const double tauTot = tau1() + std::max(m_tau2, 1e-6);
    const double dtFit = m_dt * std::max(1.0, m_cfg.simStride);
    // The scored window must span several slowest-lag time constants or
    // the two-lag split is under-determined — floor it at
    // windowLagFactor·(τ1+τ2) on top of the absolute windowTime.
    const double winTime = std::max(m_cfg.windowTime,
                                    m_cfg.windowLagFactor * tauTot);
    const double histTime = winTime
                          + m_cfg.spinupFactor * std::max(tauTot, 10.0);
    const size_t needHist = static_cast<size_t>(histTime / dtFit) + 8;
    const size_t nscore = static_cast<size_t>(winTime / dtFit) + 1;
    if (nscore < 8 || m_hist.size() < needHist) return;

    // Copy the needed tail into a contiguous buffer.
    const size_t n = std::min(m_hist.size(), needHist);
    std::vector<Sample> hist(n);
    auto it = m_hist.end();
    std::advance(it, -static_cast<std::ptrdiff_t>(n));
    std::copy(it, m_hist.end(), hist.begin());
    if (hist.size() <= nscore) return;
    const size_t iscore = hist.size() - nscore;

    // Excitation gate over the scored window.
    double uLo = 1e30, uHi = -1e30, tLo = 1e30, tHi = -1e30;
    for (size_t k = iscore; k < hist.size(); ++k) {
        uLo = std::min(uLo, hist[k].u); uHi = std::max(uHi, hist[k].u);
        tLo = std::min(tLo, hist[k].T); tHi = std::max(tHi, hist[k].T);
    }
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

    // Trust region around the prior lags.
    const double ta0 = m_beta > 1e-9 ? 1.0 / m_beta : 200.0;
    const double tb0 = std::max(m_tau2, 1e-6);
    const double taLo = std::max(0.5, ta0 * m_cfg.lagSpanLo);
    const double taHi = std::min(1e5, ta0 * m_cfg.lagSpanHi);
    const double tbLo = std::max(0.2, tb0 * m_cfg.lagSpanLo);
    const double tbHi = std::min(1e5, tb0 * m_cfg.lagSpanHi);
    if (!(taLo < taHi && tbLo < tbHi)) return;

    const int G = std::max(3, m_cfg.gridSize);
    const double lTa0 = std::log(taLo), lTa1 = std::log(taHi);
    const double lTb0 = std::log(tbLo), lTb1 = std::log(tbHi);

    double bestJ = 1e300, bAlpha = 0.0, bD = 0.0, bMse = 0.0;
    double bTa = ta0, bTb = tb0;

    auto tryPoint = [&](double ta, double tb) {
        double a, d, mse;
        const double J = evalCandidate(ta, tb, dtFit, hist, iscore,
                                       m_ambient, a, d, mse);
        if (J < bestJ) {
            bestJ = J; bAlpha = a; bD = d; bMse = mse; bTa = ta; bTb = tb;
        }
    };

    for (int i = 0; i < G; ++i) {
        const double ta = std::exp(lTa0 + (lTa1 - lTa0) * i / (G - 1));
        for (int j = 0; j < G; ++j) {
            const double tb = std::exp(lTb0 + (lTb1 - lTb0) * j / (G - 1));
            tryPoint(ta, tb);
        }
    }

    // Pattern-search refinement in log-lag space inside the trust region.
    double curTa = bTa, curTb = bTb, curJ = bestJ;
    double step = std::log(1.4);
    for (int it = 0; it < 40 && step > 0.02; ++it) {
        bool improved = false;
        for (int dim = 0; dim < 2; ++dim) {
            for (int s = -1; s <= 1; s += 2) {
                double ta = curTa, tb = curTb;
                if (dim == 0) ta *= std::exp(s * step);
                else          tb *= std::exp(s * step);
                if (ta < taLo || ta > taHi || tb < tbLo || tb > tbHi) continue;
                double a, d, mse;
                const double J = evalCandidate(ta, tb, dtFit, hist, iscore,
                                               m_ambient, a, d, mse);
                if (J < curJ) {
                    curJ = J; curTa = ta; curTb = tb; improved = true;
                    if (J < bestJ) {
                        bestJ = J; bAlpha = a; bD = d; bMse = mse;
                        bTa = ta; bTb = tb;
                    }
                }
            }
        }
        if (!improved) step *= 0.5;
    }

    // Map lags -> cascade params and project onto the physical box.
    const double alpha = bAlpha;
    const double beta = 1.0 / bTa;
    const double tau2 = bTb;
    const double K = beta > 1e-12 ? alpha / beta : 1e30;

    const bool ok =
        std::isfinite(alpha) && std::isfinite(beta) &&
        std::isfinite(tau2) && std::isfinite(bD) &&
        alpha >= m_cfg.alphaMin && alpha <= m_cfg.alphaMax &&
        beta >= m_cfg.betaMin && beta <= m_cfg.betaMax &&
        tau2 >= m_cfg.tau2Min && tau2 <= m_cfg.tau2Max &&
        K >= m_cfg.KMin && K <= m_cfg.KMax &&
        bD >= m_cfg.dMin && bD <= m_cfg.dMax;
    if (!ok) return;

    const double g = std::clamp(m_cfg.smoothing, 0.0, 1.0);
    m_alpha = (1.0 - g) * m_alpha + g * alpha;
    m_beta = (1.0 - g) * m_beta + g * beta;
    m_tau2 = (1.0 - g) * m_tau2 + g * tau2;
    m_d = (1.0 - g) * m_d + g * bD;

    ++m_updatesUsed;
    m_lastValid = true;
    m_lastResidual = std::sqrt(std::max(bMse, 0.0));
}

bool SOPDTParameterEstimator::debugEval(double ta, double tb, double& mse,
                                        double& alpha, double& d) const {
    const double tauTot = tau1() + std::max(m_tau2, 1e-6);
    const double dtFit = m_dt * std::max(1.0, m_cfg.simStride);
    const double winTime = std::max(m_cfg.windowTime,
                                    m_cfg.windowLagFactor * tauTot);
    const double histTime = winTime
                          + m_cfg.spinupFactor * std::max(tauTot, 10.0);
    const size_t needHist = static_cast<size_t>(histTime / dtFit) + 8;
    const size_t nscore = static_cast<size_t>(winTime / dtFit) + 1;
    if (nscore < 8 || m_hist.size() < needHist) return false;
    const size_t n = std::min(m_hist.size(), needHist);
    std::vector<Sample> hist(n);
    auto it = m_hist.end();
    std::advance(it, -static_cast<std::ptrdiff_t>(n));
    std::copy(it, m_hist.end(), hist.begin());
    if (hist.size() <= nscore) return false;
    const size_t iscore = hist.size() - nscore;
    double a, dd, mm;
    evalCandidate(ta, tb, dtFit, hist, iscore, m_ambient, a, dd, mm);
    mse = mm; alpha = a; d = dd;
    return true;
}

} // namespace tether::control::thermal
