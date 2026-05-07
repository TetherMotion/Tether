#include "tether/control/autotuning/classical/Common.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace Control {
namespace Autotuning {

void PIDGains::toStandardForm() {
    if (Kp > 0 && Ki > 0) Ti = Kp / Ki;
    if (Kp > 0 && Kd > 0) Td = Kd / Kp;
}

void PIDGains::toParallelForm() {
    if (Ti > 0) Ki = Kp / Ti;
    if (Td > 0) Kd = Kp * Td;
}

bool PIDGains::isValid() const { return Kp >= 0 && Ki >= 0 && Kd >= 0; }

FOPDTModel ProcessIdentification::tangentMethod(const std::vector<double>& time,
                                                 const std::vector<double>& response,
                                                 double stepSize) {
    FOPDTModel model;
    if (time.size() < 3 || response.size() < 3) return model;
    double initialValue = response[0];
    double finalValue = response.back();
    model.K = (finalValue - initialValue) / stepSize;

    // Find inflection point (maximum slope)
    double maxSlope = 0;
    size_t inflectionIdx = 1;
    for (size_t i = 1; i < response.size() - 1; ++i) {
        double slope = (response[i + 1] - response[i - 1]) / (time[i + 1] - time[i - 1]);
        if (slope > maxSlope) {
            maxSlope = slope;
            inflectionIdx = i;
        }
    }

    double tInfl = time[inflectionIdx];
    double yInfl = response[inflectionIdx];

    // L is where tangent intersects initial value
    // initialValue = maxSlope * (L - tInfl) + yInfl
    if (maxSlope == 0) {
        model.L = 0.0;
        model.tau = 1.0;
        return model;
    }

    model.L = tInfl - (yInfl - initialValue) / maxSlope;
    if (model.L < 0) model.L = 0;

    double tauPlusL = tInfl + (finalValue - yInfl) / maxSlope;
    model.tau = tauPlusL - model.L;
    if (model.tau <= 0) model.tau = 1.0;

    return model;
}

FOPDTModel ProcessIdentification::areaMethod(const std::vector<double>& time,
                                              const std::vector<double>& response,
                                              double stepSize) {
    FOPDTModel model;
    if (time.size() < 3) return model;

    double y0 = response[0];
    double yFinal = response.back();
    model.K = (yFinal - y0) / stepSize;

    double target632 = y0 + 0.632 * (yFinal - y0);
    double t632 = time.back();
    for (size_t i = 1; i < response.size(); ++i) {
        if (response[i] >= target632) {
            double frac = (target632 - response[i-1]) / (response[i] - response[i-1]);
            t632 = time[i-1] + frac * (time[i] - time[i-1]);
            break;
        }
    }

    double sum = t632;
    model.L = 0.1 * sum;
    model.tau = sum - model.L;
    return model;
}

FOPDTModel ProcessIdentification::twoPointMethod(const std::vector<double>& time,
                                                  const std::vector<double>& response,
                                                  double stepSize,
                                                  double t1Fraction,
                                                  double t2Fraction) {
    FOPDTModel model;
    if (time.size() < 3) return model;
    double y0 = response[0];
    double yFinal = response.back();
    model.K = (yFinal - y0) / stepSize;

    double target1 = y0 + t1Fraction * (yFinal - y0);
    double target2 = y0 + t2Fraction * (yFinal - y0);
    double t1 = 0, t2 = 0;
    for (size_t i = 1; i < response.size(); ++i) {
        if (t1 == 0 && response[i] >= target1) {
            double frac = (target1 - response[i-1]) / (response[i] - response[i-1]);
            t1 = time[i-1] + frac * (time[i] - time[i-1]);
        }
        if (t2 == 0 && response[i] >= target2) {
            double frac = (target2 - response[i-1]) / (response[i] - response[i-1]);
            t2 = time[i-1] + frac * (time[i] - time[i-1]);
            break;
        }
    }
    model.tau = 1.5 * (t2 - t1);
    model.L = t2 - model.tau;
    if (model.L < 0) model.L = 0;
    return model;
}

FOPDTModel ProcessIdentification::leastSquaresFit(const std::vector<double>& time,
                                                   const std::vector<double>& response,
                                                   double stepSize) {
    FOPDTModel best;
    double y0 = response[0];
    double yFinal = response.back();
    best.K = (yFinal - y0) / stepSize;
    best = areaMethod(time, response, stepSize);
    double bestError = std::numeric_limits<double>::max();
    for (double tauMult = 0.5; tauMult <= 2.0; tauMult += 0.1) {
        for (double LMult = 0.5; LMult <= 2.0; LMult += 0.1) {
            FOPDTModel trial;
            trial.K = best.K;
            trial.tau = best.tau * tauMult;
            trial.L = best.L * LMult;
            double mse = 0;
            for (size_t i = 0; i < time.size(); ++i) {
                double t = time[i];
                double yModel = y0;
                if (t > trial.L) {
                    yModel = y0 + stepSize * trial.K * (1.0 - std::exp(-(t - trial.L) / trial.tau));
                }
                double err = response[i] - yModel;
                mse += err * err;
            }
            mse /= time.size();
            if (mse < bestError) {
                bestError = mse;
                best.tau = trial.tau;
                best.L = trial.L;
            }
        }
    }
    return best;
}

SOPDTModel ProcessIdentification::identifySOPDT(const std::vector<double>& time,
                                                 const std::vector<double>& response,
                                                 double stepSize) {
    SOPDTModel model; FOPDTModel fopdt = leastSquaresFit(time, response, stepSize);
    model.K = fopdt.K; model.L = fopdt.L; model.tau1 = fopdt.tau * 0.7; model.tau2 = fopdt.tau * 0.3;
    return model;
}

std::pair<double, double> ProcessIdentification::estimateUltimate(const FOPDTModel& model) {
    double omega = M_PI / (2.0 * model.L);
    for (int i = 0; i < 10; ++i) {
        double phase = -std::atan(omega * model.tau) - omega * model.L;
        double target = -M_PI;
        double dphase = -model.tau / (1.0 + omega * omega * model.tau * model.tau) - model.L;
        omega = omega - (phase - target) / dphase;
        if (omega < 0) omega = M_PI / (2.0 * model.L);
    }
    double Tu = 2.0 * M_PI / omega;
    double mag = std::abs(model.K) / std::sqrt(1.0 + omega * omega * model.tau * model.tau);
    double Ku = 1.0 / mag;
    return {Ku, Tu};
}

} // namespace Autotuning
} // namespace Control
