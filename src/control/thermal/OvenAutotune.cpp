#include "tether/control/thermal/OvenAutotune.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::thermal {

namespace {

/**@brief First crossing time of level `level` in (t,y), linear interp.
 *        Returns -1 if never crossed. */
double crossingTime(const std::vector<double>& t, const std::vector<double>& y,
                    double level, double y0) {
    const double dir = (level >= y0) ? 1.0 : -1.0;
    for (size_t i = 1; i < t.size(); ++i) {
        const double d0 = (y[i - 1] - y0) * dir;
        const double d1 = (y[i] - y0) * dir;
        if (d0 < level - y0 && d1 >= level - y0) {
            // interpolate fraction between samples
            const double frac = (d1 != d0) ? (level - y0 - d0) / (d1 - d0) : 0.0;
            return t[i - 1] + frac * (t[i] - t[i - 1]);
        }
    }
    return -1.0;
}

} // namespace

FopdtEstimate identifyFopdt(const StepResponseData& data, double minDeltaT) {
    FopdtEstimate est;
    const size_t n = data.time.size();
    if (n < 8 || data.temp.size() != n) return est;

    // Baseline: mean of first 3 samples (pre/early response).
    const size_t nBase = std::min<size_t>(3, n / 4);
    double y0 = 0.0;
    for (size_t i = 0; i < nBase; ++i) y0 += data.temp[i];
    y0 /= static_cast<double>(nBase);

    // Steady state: mean of last 10% of samples.
    const size_t nTail = std::max<size_t>(3, n / 10);
    double yss = 0.0;
    for (size_t i = n - nTail; i < n; ++i) yss += data.temp[i];
    yss /= static_cast<double>(nTail);

    const double dT = yss - y0;
    if (std::abs(dT) < minDeltaT || std::abs(data.uStep) < 1e-9) return est;

    const double level10 = y0 + 0.1 * dT;
    const double level90 = y0 + 0.9 * dT;
    const double t10 = crossingTime(data.time, data.temp, level10, y0);
    const double t90 = crossingTime(data.time, data.temp, level90, y0);
    if (t10 < 0.0 || t90 <= t10) return est;

    // Two-point formulas for FOPDT:
    //   tau   = (t90 - t10) / ln(9)
    //   theta = t10 - tau * ln(10/9)
    const double tau = (t90 - t10) / std::log(9.0);
    double theta = t10 - tau * std::log(10.0 / 9.0);
    theta = std::max(0.0, theta);
    if (tau <= 0.0) return est;

    est.K = dT / data.uStep;
    est.tau = tau;
    est.theta = theta;
    est.valid = true;
    return est;
}

std::pair<double, double> simcPiGains(double K, double tau, double theta,
                                      double tauC) {
    if (std::abs(K) < 1e-12 || tau <= 0.0) return {0.0, 0.0};
    tau = std::clamp(tau, 1e-3, 1e6);
    theta = std::max(0.0, theta);

    // SIMC default: closed-loop time constant = dead time.
    double tc = tauC;
    if (tc <= 0.0) tc = (theta > 1e-6) ? theta : 0.1 * tau;

    const double kp = tau / (std::abs(K) * (tc + theta));
    const double Ti = std::min(tau, 4.0 * (tc + theta));
    const double ki = (Ti > 1e-9) ? kp / Ti : 0.0;
    return {kp, ki};
}

OvenAutotuneResult autoTuneOven(const StepResponseData& data,
                                double tauC, double minDeltaT) {
    OvenAutotuneResult r;
    r.model = identifyFopdt(data, minDeltaT);
    if (!r.model.valid) return r;

    const double tau = std::clamp(r.model.tau, 1e-3, 1e6);
    r.params.beta = 1.0 / tau;
    r.params.alpha = r.model.K / tau;   // K = α/β, β = 1/τ → α = K/τ
    r.params.delay = r.model.theta;
    r.params.disturbance = 0.0;
    r.params.ambient = data.tAmb;
    r.params.uMin = data.uMin;
    r.params.uMax = data.uMax;

    const auto [kp, ki] = simcPiGains(r.model.K, tau, r.model.theta, tauC);
    r.kp = kp;
    r.ki = ki;
    r.lambda = (tauC > 0.0) ? tauC
                            : ((r.model.theta > 1e-6) ? r.model.theta : 0.1 * tau);
    r.success = true;
    return r;
}

// ---------------------------------------------------------------------------
// SOPDT commissioning
// ---------------------------------------------------------------------------

namespace {

/**@brief Normalized unit-step response of K/((τa s+1)(τb s+1)) at t'>0.
 *        Symmetric in (τa, τb); handles the equal-pole limit. */
double sopdtStepNorm(double tp, double tauA, double tauB) {
    if (tp <= 0.0) return 0.0;
    if (tauA <= 1e-9 || tauB <= 1e-9) {
        // One pole vanished → FOPDT with the other.
        const double tau = std::max(tauA, tauB);
        return 1.0 - std::exp(-tp / tau);
    }
    const double d = tauA - tauB;
    if (std::abs(d) < 1e-6 * std::max(tauA, tauB)) {
        const double tau = 0.5 * (tauA + tauB);
        return 1.0 - std::exp(-tp / tau) * (1.0 + tp / tau);
    }
    return 1.0 - (tauA * std::exp(-tp / tauA) -
                  tauB * std::exp(-tp / tauB)) / d;
}

/**@brief SSE of the SOPDT fit over post-delay samples. */
double sopdtSse(const StepResponseData& data, double y0, double dT,
                double theta, double tauA, double tauB) {
    double sse = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < data.time.size(); ++i) {
        const double tp = data.time[i] - theta;
        if (tp <= 0.0) continue;
        const double yn = (data.temp[i] - y0) / dT;
        const double e = yn - sopdtStepNorm(tp, tauA, tauB);
        sse += e * e;
        ++cnt;
    }
    return cnt > 0 ? sse / static_cast<double>(cnt) : 1e30;
}

} // namespace

SopdtEstimate identifySopdt(const StepResponseData& data,
                            double minDeltaT) {
    SopdtEstimate est;
    const size_t n = data.time.size();
    if (n < 16 || data.temp.size() != n) return est;

    // Baseline / steady state / gain — identical to identifyFopdt.
    const size_t nBase = std::min<size_t>(3, n / 4);
    double y0 = 0.0;
    for (size_t i = 0; i < nBase; ++i) y0 += data.temp[i];
    y0 /= static_cast<double>(nBase);

    const size_t nTail = std::max<size_t>(3, n / 10);
    double yss = 0.0;
    for (size_t i = n - nTail; i < n; ++i) yss += data.temp[i];
    yss /= static_cast<double>(nTail);

    const double dT = yss - y0;
    if (std::abs(dT) < minDeltaT || std::abs(data.uStep) < 1e-9) return est;
    const double dir = (dT >= 0.0) ? 1.0 : -1.0;

    // Dead time: first sample departing baseline by >2% of the total
    // rise (or 0.05 °C, whichever is larger). θ is the last flat sample.
    const double depart = std::max(0.02 * std::abs(dT), 0.05);
    double theta = 0.0;
    for (size_t i = nBase; i < n; ++i) {
        if (std::abs(data.temp[i] - y0) > depart) {
            theta = (i > 0) ? data.time[i - 1] : data.time[0];
            break;
        }
    }
    theta = std::max(0.0, theta - data.time[0]);

    // Center the search on the FOPDT two-point tau (rough scale only).
    const FopdtEstimate f = identifyFopdt(data, minDeltaT);
    const double tauEff = (f.valid && f.tau > 0.0) ? f.tau
                                                 : (data.time.back() - theta) / 3.0;
    const double lo = std::max(0.2, 0.01 * tauEff);
    const double hi = std::max(lo * 4.0, 8.0 * tauEff);

    // Coarse log-grid over the upper triangle (response symmetric in
    // tauA/tauB), then pattern refinement on (tauA, tauB, theta).
    constexpr int kGrid = 22;
    double bestA = tauEff * 0.5, bestB = tauEff * 0.5;
    double bestSse = sopdtSse(data, y0, dT, theta, bestA, bestB);
    const double llo = std::log(lo), lhi = std::log(hi);
    for (int i = 0; i < kGrid; ++i) {
        const double ta = std::exp(llo + (lhi - llo) * i / (kGrid - 1));
        for (int j = i; j < kGrid; ++j) {
            const double tb = std::exp(llo + (lhi - llo) * j / (kGrid - 1));
            const double sse = sopdtSse(data, y0, dT, theta, ta, tb);
            if (sse < bestSse) { bestSse = sse; bestA = ta; bestB = tb; }
        }
    }

    // Pattern refinement: multiplicative tau moves + theta shifts.
    const double dtMed = (n > 1)
        ? (data.time[1] - data.time[0]) : 1.0;
    for (int it = 0; it < 24; ++it) {
        bool improved = false;
        for (double f : {0.75, 0.9, 1.11, 1.33}) {
            for (int which = 0; which < 2; ++which) {
                double ta = bestA, tb = bestB;
                if (which == 0) ta *= f; else tb *= f;
                ta = std::clamp(ta, lo, hi);
                tb = std::clamp(tb, lo, hi);
                const double sse = sopdtSse(data, y0, dT, theta, ta, tb);
                if (sse < bestSse * (1.0 - 1e-9)) {
                    bestSse = sse; bestA = ta; bestB = tb; improved = true;
                }
            }
        }
        for (double sh : {-dtMed, dtMed}) {
            const double th2 = std::max(0.0, theta + sh);
            const double sse = sopdtSse(data, y0, dT, th2, bestA, bestB);
            if (sse < bestSse * (1.0 - 1e-9)) {
                bestSse = sse; theta = th2; improved = true;
            }
        }
        if (!improved) break;
    }

    est.K = dT / data.uStep;
    est.tau1 = std::max(bestA, bestB);   // dominant (slow)
    est.tau2 = std::min(bestA, bestB);   // secondary (fast)
    est.theta = theta;
    est.fitRmse = std::sqrt(bestSse);
    est.valid = (est.tau1 > 0.0 && est.K != 0.0);
    (void)dir;
    return est;
}

SopdtGains simcGainsSopdt(double K, double tauDom, double tauSec,
                          double theta, double tauC) {
    SopdtGains g;
    if (std::abs(K) < 1e-12 || tauDom <= 0.0) return g;
    theta = std::max(0.0, theta);
    double tc = tauC;
    if (tc <= 0.0) tc = (theta > 1e-6) ? theta : 0.1 * (tauDom + tauSec);

    // SIMC SOPDT rule (Skogestad): dominant lag in the PI part,
    // secondary lag as derivative time.
    g.kp = tauDom / (std::abs(K) * (tc + theta));
    const double Ti = std::min(tauDom, 4.0 * (tc + theta));
    g.ki = (Ti > 1e-9) ? g.kp / Ti : 0.0;
    const double Td = std::max(0.0, tauSec);
    g.kd = g.kp * Td;
    return g;
}

namespace {

double imcLambda(double lambda, double theta, double tauSum) {
    if (lambda > 0.0) return lambda;
    return (theta > 1e-6) ? theta : 0.1 * tauSum;
}

} // namespace

SopdtGains imcPiGainsSopdt(double K, double tauDom, double tauSec,
                           double theta, double lambda) {
    SopdtGains g;
    const double tauSum = tauDom + tauSec;
    if (std::abs(K) < 1e-12 || tauSum <= 0.0) return g;
    const double lam = imcLambda(lambda, theta, tauSum);
    g.kp = tauSum / (std::abs(K) * (lam + theta));
    g.ki = (tauSum > 1e-9) ? g.kp / tauSum : 0.0;
    return g;
}

SopdtGains imcPidGainsSopdt(double K, double tauDom, double tauSec,
                            double theta, double lambda) {
    SopdtGains g = imcPiGainsSopdt(K, tauDom, tauSec, theta, lambda);
    const double tauSum = tauDom + tauSec;
    if (tauSum > 1e-9) {
        const double Td = (tauDom * tauSec) / tauSum;
        g.kd = g.kp * Td;
    }
    return g;
}

OvenAutotuneResultSOPDT autoTuneOvenSOPDT(const StepResponseData& data,
                                          OvenTuneMethod method,
                                          double tauC, double minDeltaT) {
    OvenAutotuneResultSOPDT r;
    r.model = identifySopdt(data, minDeltaT);
    if (!r.model.valid) return r;

    const double tauFast = r.model.tau2;    // secondary → stage 1
    const double tauSlow = r.model.tau1;    // dominant   → tau2 param
    const double K = r.model.K;

    r.params.beta = 1.0 / std::max(tauFast, 1e-3);
    r.params.alpha = K * r.params.beta;    // K = α/β
    r.params.tau2 = tauSlow;
    r.params.delay = r.model.theta;
    r.params.disturbance = 0.0;
    r.params.ambient = data.tAmb;
    r.params.uMin = data.uMin;
    r.params.uMax = data.uMax;

    const double lam = (method == OvenTuneMethod::ImcPi ||
                        method == OvenTuneMethod::ImcPid)
        ? imcLambda(tauC, r.model.theta, tauSlow + tauFast)
        : ((tauC > 0.0) ? tauC
                        : ((r.model.theta > 1e-6) ? r.model.theta
                                                  : 0.1 * (tauSlow + tauFast)));
    r.lambda = lam;

    switch (method) {
    case OvenTuneMethod::Simc:
        {
            const auto g = simcGainsSopdt(K, tauSlow, tauFast,
                                          r.model.theta, tauC);
            r.kp = g.kp; r.ki = g.ki; r.kd = g.kd;
        }
        break;
    case OvenTuneMethod::ImcPi: {
        const auto g = imcPiGainsSopdt(K, tauSlow, tauFast,
                                       r.model.theta, tauC);
        r.kp = g.kp; r.ki = g.ki; r.kd = g.kd;
        break;
    }
    case OvenTuneMethod::ImcPid: {
        const auto g = imcPidGainsSopdt(K, tauSlow, tauFast,
                                        r.model.theta, tauC);
        r.kp = g.kp; r.ki = g.ki; r.kd = g.kd;
        break;
    }
    case OvenTuneMethod::Manual:
        break;   // gains stay zero — user sets them explicitly
    }
    r.success = true;
    return r;
}

} // namespace tether::control::thermal
