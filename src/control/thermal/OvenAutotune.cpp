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

} // namespace tether::control::thermal
