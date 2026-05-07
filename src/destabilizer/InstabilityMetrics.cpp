/// @file InstabilityMetrics.cpp
/// @brief All 20 instability metric implementations.

#include "tether/destabilizer/InstabilityMetrics.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <cassert>
#include <complex>

namespace Destabilizer {

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------

double vectorNorm(const std::vector<double>& v) {
    double sum = 0.0;
    for (double x : v) sum += x * x;
    return std::sqrt(sum);
}

double vectorNormInf(const std::vector<double>& v) {
    double m = 0.0;
    for (double x : v) m = std::max(m, std::abs(x));
    return m;
}

std::vector<double> vectorDiff(const std::vector<double>& a,
                                const std::vector<double>& b) {
    size_t n = std::min(a.size(), b.size());
    std::vector<double> result(n);
    for (size_t i = 0; i < n; ++i) result[i] = a[i] - b[i];
    return result;
}

std::pair<double, double> linearFit(const std::vector<double>& x,
                                     const std::vector<double>& y) {
    size_t n = std::min(x.size(), y.size());
    if (n < 2) return {0.0, 0.0};

    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < n; ++i) {
        sx += x[i]; sy += y[i];
        sxx += x[i] * x[i]; sxy += x[i] * y[i];
    }
    double dn = static_cast<double>(n);
    double denom = dn * sxx - sx * sx;
    if (std::abs(denom) < 1e-30) return {0.0, 0.0};
    double b = (dn * sxy - sx * sy) / denom;
    double a = (sy - b * sx) / dn;
    return {a, b};
}

std::vector<double> computeFFTMagnitude(const std::vector<double>& signal, double dt) {
    // Zero-pad to next power of 2
    size_t n = 1;
    while (n < signal.size()) n <<= 1;

    std::vector<std::complex<double>> data(n, {0.0, 0.0});
    for (size_t i = 0; i < signal.size(); ++i) {
        data[i] = {signal[i], 0.0};
    }

    // Iterative Cooley-Tukey FFT
    // Bit-reversal permutation
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    // Butterfly operations
    for (size_t len = 2; len <= n; len <<= 1) {
        double angle = -2.0 * M_PI / static_cast<double>(len);
        std::complex<double> wlen(std::cos(angle), std::sin(angle));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t j = 0; j < len / 2; ++j) {
                auto u = data[i + j];
                auto v = data[i + j + len / 2] * w;
                data[i + j] = u + v;
                data[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    // Magnitude of positive frequencies
    size_t numFreqs = n / 2 + 1;
    std::vector<double> magnitudes(numFreqs);
    for (size_t i = 0; i < numFreqs; ++i) {
        magnitudes[i] = std::abs(data[i]) / static_cast<double>(n);
    }
    return magnitudes;
}

std::vector<std::pair<double, double>> computePSD(const std::vector<double>& signal,
                                                    double dt) {
    auto mags = computeFFTMagnitude(signal, dt);
    double df = 1.0 / (dt * signal.size());
    std::vector<std::pair<double, double>> psd;
    psd.reserve(mags.size());
    for (size_t i = 0; i < mags.size(); ++i) {
        double freq = i * df;
        double power = mags[i] * mags[i];
        psd.push_back({freq, power});
    }
    return psd;
}

double estimateSpectralRadius(const std::vector<std::vector<double>>& states, double dt) {
    if (states.size() < 2) return 0.0;
    int dim = static_cast<int>(states[0].size());
    if (dim == 0) return 0.0;

    // Simple estimate: max ratio of consecutive state norms
    double maxRatio = 0.0;
    for (size_t i = 1; i < states.size(); ++i) {
        double normPrev = vectorNorm(states[i-1]);
        double normCurr = vectorNorm(states[i]);
        if (normPrev > 1e-15) {
            maxRatio = std::max(maxRatio, normCurr / normPrev);
        }
    }
    return maxRatio;
}

// ---------------------------------------------------------------------------
// Metric Factory
// ---------------------------------------------------------------------------

std::unique_ptr<InstabilityMetric> createMetric(const WeightedMetric& wm) {
    switch (wm.type) {
    case MetricType::PeakStateDeviation:
        return std::make_unique<PeakStateDeviationMetric>();
    case MetricType::TerminalStateDeviation:
        return std::make_unique<TerminalStateDeviationMetric>();
    case MetricType::IntegratedSquaredError:
        return std::make_unique<ISEMetric>();
    case MetricType::IntegratedAbsoluteError:
        return std::make_unique<IAEMetric>();
    case MetricType::TimeWeightedISE:
        return std::make_unique<ITSEMetric>();
    case MetricType::ExponentialDivergenceRate:
        return std::make_unique<ExponentialDivergenceMetric>();
    case MetricType::RegionOfAttractionEscape:
        return std::make_unique<ROAEscapeMetric>(std::vector<SafeSetBound>{});
    case MetricType::ControlSaturationTime:
        return std::make_unique<ControlSaturationMetric>(-1e6, 1e6);
    case MetricType::OscillationAmplitudeGrowth:
        return std::make_unique<OscillationGrowthMetric>();
    case MetricType::LimitCycleEscapeCount:
        return std::make_unique<LimitCycleEscapeMetric>(wm.threshold);
    case MetricType::PhaseSpaceVolumeExpansion:
        return std::make_unique<PhaseSpaceExpansionMetric>();
    case MetricType::SpectralRadiusSensitivity:
        return std::make_unique<SpectralRadiusMetric>();
    case MetricType::SettlingTimeViolation:
        return std::make_unique<SettlingTimeViolationMetric>(wm.settlingTimeSpec);
    case MetricType::OvershootMagnitude:
        return std::make_unique<OvershootMetric>();
    case MetricType::EnergyInjected:
        return std::make_unique<EnergyInjectedMetric>();
    case MetricType::ConstraintViolationIntegral:
        return std::make_unique<ConstraintViolationMetric>(std::vector<SafeSetBound>{});
    case MetricType::TimeToInstability:
        return std::make_unique<TimeToInstabilityMetric>(wm.threshold);
    case MetricType::ControllerBandwidthExceedance:
        return std::make_unique<BandwidthExceedanceMetric>(wm.controllerBandwidth);
    case MetricType::CovarianceGrowth:
        return std::make_unique<CovarianceGrowthMetric>();
    case MetricType::NonlinearDistortion:
        return std::make_unique<NonlinearDistortionMetric>();
    }
    return std::make_unique<PeakStateDeviationMetric>();
}

// ---------------------------------------------------------------------------
// Helper: state deviation norm at a timestep
// ---------------------------------------------------------------------------

static double stateDeviationNorm(const std::vector<double>& state,
                                   const std::vector<double>& ref) {
    if (ref.empty()) return vectorNorm(state);
    return vectorNorm(vectorDiff(state, ref));
}

// ---------------------------------------------------------------------------
// 1. Peak State Deviation
// ---------------------------------------------------------------------------

double PeakStateDeviationMetric::compute(const RolloutTrajectory& traj) const {
    double peak = 0.0;
    for (const auto& s : traj.states) {
        peak = std::max(peak, stateDeviationNorm(s, traj.referenceState));
    }
    return peak;
}

// ---------------------------------------------------------------------------
// 2. Terminal State Deviation
// ---------------------------------------------------------------------------

double TerminalStateDeviationMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.empty()) return 0.0;
    return stateDeviationNorm(traj.states.back(), traj.referenceState);
}

// ---------------------------------------------------------------------------
// 3. ISE
// ---------------------------------------------------------------------------

double ISEMetric::compute(const RolloutTrajectory& traj) const {
    double ise = 0.0;
    for (const auto& s : traj.states) {
        double dev = stateDeviationNorm(s, traj.referenceState);
        ise += dev * dev * traj.dt;
    }
    return ise;
}

// ---------------------------------------------------------------------------
// 4. IAE
// ---------------------------------------------------------------------------

double IAEMetric::compute(const RolloutTrajectory& traj) const {
    double iae = 0.0;
    for (const auto& s : traj.states) {
        iae += stateDeviationNorm(s, traj.referenceState) * traj.dt;
    }
    return iae;
}

// ---------------------------------------------------------------------------
// 5. ITSE (Time-Weighted ISE)
// ---------------------------------------------------------------------------

double ITSEMetric::compute(const RolloutTrajectory& traj) const {
    double itse = 0.0;
    for (size_t i = 0; i < traj.states.size(); ++i) {
        double t = (i < traj.times.size()) ? traj.times[i] : i * traj.dt;
        double dev = stateDeviationNorm(traj.states[i], traj.referenceState);
        itse += t * dev * dev * traj.dt;
    }
    return itse;
}

// ---------------------------------------------------------------------------
// 6. Exponential Divergence Rate
// ---------------------------------------------------------------------------

double ExponentialDivergenceMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.size() < 2) return 0.0;

    // Fit log||x(t) - x_ref|| vs t
    std::vector<double> times, logDevs;
    for (size_t i = 0; i < traj.states.size(); ++i) {
        double dev = stateDeviationNorm(traj.states[i], traj.referenceState);
        if (dev > 1e-15) {
            double t = (i < traj.times.size()) ? traj.times[i] : i * traj.dt;
            times.push_back(t);
            logDevs.push_back(std::log(dev));
        }
    }

    if (times.size() < 2) return 0.0;
    auto [intercept, slope] = linearFit(times, logDevs);
    return slope;  // Positive = diverging
}

// ---------------------------------------------------------------------------
// 7. ROA Escape
// ---------------------------------------------------------------------------

ROAEscapeMetric::ROAEscapeMetric(const std::vector<SafeSetBound>& bounds)
    : bounds_(bounds) {}

double ROAEscapeMetric::compute(const RolloutTrajectory& traj) const {
    double totalEscape = 0.0;
    for (const auto& s : traj.states) {
        for (const auto& bound : bounds_) {
            if (bound.stateIndex < static_cast<int>(s.size())) {
                double val = s[bound.stateIndex];
                double violation = 0.0;
                if (val > bound.upperBound) violation = val - bound.upperBound;
                else if (val < bound.lowerBound) violation = bound.lowerBound - val;
                totalEscape += violation * traj.dt;
            }
        }
    }
    return totalEscape;
}

// ---------------------------------------------------------------------------
// 8. Control Saturation Time
// ---------------------------------------------------------------------------

ControlSaturationMetric::ControlSaturationMetric(double satMin, double satMax)
    : satMin_(satMin), satMax_(satMax) {}

double ControlSaturationMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.controlInputs.empty()) return 0.0;

    int saturatedSteps = 0;
    int totalSteps = static_cast<int>(traj.controlInputs.size());

    for (const auto& u : traj.controlInputs) {
        for (double val : u) {
            if (val >= satMax_ - 1e-10 || val <= satMin_ + 1e-10) {
                saturatedSteps++;
                break;
            }
        }
    }

    return static_cast<double>(saturatedSteps) / std::max(1, totalSteps);
}

// ---------------------------------------------------------------------------
// 9. Oscillation Amplitude Growth
// ---------------------------------------------------------------------------

double OscillationGrowthMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.size() < 4) return 0.0;

    // Extract first state component deviation
    std::vector<double> signal;
    for (const auto& s : traj.states) {
        double dev = stateDeviationNorm(s, traj.referenceState);
        signal.push_back(dev);
    }

    // Find local maxima (envelope)
    std::vector<double> peakTimes, peakVals;
    for (size_t i = 1; i + 1 < signal.size(); ++i) {
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            double t = (i < traj.times.size()) ? traj.times[i] : i * traj.dt;
            peakTimes.push_back(t);
            peakVals.push_back(signal[i]);
        }
    }

    if (peakTimes.size() < 2) return 0.0;

    // Fit envelope growth: log(peak) vs time
    std::vector<double> logPeaks;
    for (double p : peakVals) {
        if (p > 1e-15) logPeaks.push_back(std::log(p));
        else logPeaks.push_back(-30.0);
    }
    std::vector<double> fitTimes(peakTimes.begin(),
        peakTimes.begin() + std::min(peakTimes.size(), logPeaks.size()));

    auto [intercept, slope] = linearFit(fitTimes, logPeaks);
    return slope;  // Positive = growing oscillations
}

// ---------------------------------------------------------------------------
// 10. Limit-Cycle Escape Count
// ---------------------------------------------------------------------------

LimitCycleEscapeMetric::LimitCycleEscapeMetric(double threshold)
    : threshold_(threshold) {}

double LimitCycleEscapeMetric::compute(const RolloutTrajectory& traj) const {
    int escapes = 0;
    bool wasInside = true;
    for (const auto& s : traj.states) {
        double dev = stateDeviationNorm(s, traj.referenceState);
        bool isInside = (dev <= threshold_);
        if (wasInside && !isInside) escapes++;
        wasInside = isInside;
    }
    return static_cast<double>(escapes);
}

// ---------------------------------------------------------------------------
// 11. Phase-Space Volume Expansion (FTLE estimate)
// ---------------------------------------------------------------------------

double PhaseSpaceExpansionMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.size() < 2) return 0.0;

    // Estimate finite-time Lyapunov exponent from state growth
    double normFirst = vectorNorm(traj.states.front());
    double normLast = vectorNorm(traj.states.back());

    if (normFirst < 1e-15) normFirst = 1e-15;
    double T = traj.times.empty() ? traj.states.size() * traj.dt : traj.times.back();
    if (T < 1e-15) return 0.0;

    return std::log(std::max(1e-15, normLast / normFirst)) / T;
}

// ---------------------------------------------------------------------------
// 12. Spectral Radius Sensitivity
// ---------------------------------------------------------------------------

double SpectralRadiusMetric::compute(const RolloutTrajectory& traj) const {
    return estimateSpectralRadius(traj.states, traj.dt);
}

// ---------------------------------------------------------------------------
// 13. Settling-Time Violation
// ---------------------------------------------------------------------------

SettlingTimeViolationMetric::SettlingTimeViolationMetric(double settlingSpec)
    : settlingSpec_(settlingSpec) {}

double SettlingTimeViolationMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.empty()) return 0.0;

    // Find the last time the state deviation exceeds 2% of initial
    double initialDev = stateDeviationNorm(traj.states.front(), traj.referenceState);
    double threshold = 0.02 * initialDev;
    if (threshold < 1e-10) threshold = 1e-10;

    double settlingTime = 0.0;
    for (int i = static_cast<int>(traj.states.size()) - 1; i >= 0; --i) {
        double dev = stateDeviationNorm(traj.states[i], traj.referenceState);
        if (dev > threshold) {
            settlingTime = (i < static_cast<int>(traj.times.size())) ?
                traj.times[i] : i * traj.dt;
            break;
        }
    }

    if (settlingSpec_ > 0.0) {
        return settlingTime / settlingSpec_;  // >1 means violation
    }
    return settlingTime;
}

// ---------------------------------------------------------------------------
// 14. Overshoot Magnitude
// ---------------------------------------------------------------------------

double OvershootMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.empty() || traj.referenceState.empty()) return 0.0;

    // For first output/state: find max excursion past reference
    double maxOvershoot = 0.0;
    for (const auto& s : traj.states) {
        for (size_t i = 0; i < std::min(s.size(), traj.referenceState.size()); ++i) {
            double overshoot = s[i] - traj.referenceState[i];
            maxOvershoot = std::max(maxOvershoot, std::abs(overshoot));
        }
    }
    return maxOvershoot;
}

// ---------------------------------------------------------------------------
// 15. Energy Injected Into Plant
// ---------------------------------------------------------------------------

double EnergyInjectedMetric::compute(const RolloutTrajectory& traj) const {
    double energy = 0.0;
    for (const auto& s : traj.states) {
        double norm = vectorNorm(s);
        energy += norm * norm * traj.dt;
    }
    return energy;
}

// ---------------------------------------------------------------------------
// 16. Constraint Violation Integral
// ---------------------------------------------------------------------------

ConstraintViolationMetric::ConstraintViolationMetric(const std::vector<SafeSetBound>& bounds)
    : bounds_(bounds) {}

double ConstraintViolationMetric::compute(const RolloutTrajectory& traj) const {
    double totalViolation = 0.0;
    for (const auto& s : traj.states) {
        for (const auto& bound : bounds_) {
            if (bound.stateIndex < static_cast<int>(s.size())) {
                double val = s[bound.stateIndex];
                double violation = 0.0;
                if (val > bound.upperBound) violation = val - bound.upperBound;
                else if (val < bound.lowerBound) violation = bound.lowerBound - val;
                totalViolation += violation * violation * traj.dt;
            }
        }
    }
    return totalViolation;
}

// ---------------------------------------------------------------------------
// 17. Time-to-Instability
// ---------------------------------------------------------------------------

TimeToInstabilityMetric::TimeToInstabilityMetric(double threshold)
    : threshold_(threshold) {}

double TimeToInstabilityMetric::compute(const RolloutTrajectory& traj) const {
    double T = traj.times.empty() ?
        traj.states.size() * traj.dt : traj.times.back();

    for (size_t i = 0; i < traj.states.size(); ++i) {
        double dev = stateDeviationNorm(traj.states[i], traj.referenceState);
        if (dev > threshold_) {
            double tStar = (i < traj.times.size()) ? traj.times[i] : i * traj.dt;
            return T - tStar;  // Shorter time-to-instability = worse = higher metric
        }
    }
    return 0.0;  // Never crossed threshold
}

// ---------------------------------------------------------------------------
// 18. Controller Bandwidth Exceedance
// ---------------------------------------------------------------------------

BandwidthExceedanceMetric::BandwidthExceedanceMetric(double bandwidth)
    : bandwidth_(bandwidth) {}

double BandwidthExceedanceMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.controlInputs.empty()) return 0.0;

    // Extract first control channel as a signal
    std::vector<double> signal;
    for (const auto& u : traj.controlInputs) {
        signal.push_back(u.empty() ? 0.0 : u[0]);
    }

    auto psd = computePSD(signal, traj.dt);
    double totalPower = 0.0, aboveBwPower = 0.0;
    for (const auto& [freq, power] : psd) {
        totalPower += power;
        if (freq > bandwidth_) aboveBwPower += power;
    }

    if (totalPower < 1e-30) return 0.0;
    return aboveBwPower / totalPower;
}

// ---------------------------------------------------------------------------
// 19. Covariance Growth
// ---------------------------------------------------------------------------

double CovarianceGrowthMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.states.size() < 10) return 0.0;

    int dim = static_cast<int>(traj.states[0].size());
    if (dim == 0) return 0.0;

    // Compute running variance in windows
    size_t windowSize = std::max(size_t(5), traj.states.size() / 10);

    auto computeVarianceTrace = [&](size_t start, size_t end) -> double {
        double trace = 0.0;
        for (int d = 0; d < dim; ++d) {
            double mean = 0.0, var = 0.0;
            int count = 0;
            for (size_t i = start; i < end && i < traj.states.size(); ++i) {
                if (d < static_cast<int>(traj.states[i].size())) {
                    mean += traj.states[i][d];
                    count++;
                }
            }
            if (count > 0) mean /= count;
            for (size_t i = start; i < end && i < traj.states.size(); ++i) {
                if (d < static_cast<int>(traj.states[i].size())) {
                    double diff = traj.states[i][d] - mean;
                    var += diff * diff;
                }
            }
            if (count > 1) var /= (count - 1);
            trace += var;
        }
        return trace;
    };

    double earlyVar = computeVarianceTrace(0, windowSize);
    double lateVar = computeVarianceTrace(traj.states.size() - windowSize, traj.states.size());

    if (earlyVar < 1e-30) return lateVar > 1e-10 ? 1e10 : 0.0;
    return lateVar / earlyVar;
}

// ---------------------------------------------------------------------------
// 20. Nonlinear Distortion
// ---------------------------------------------------------------------------

double NonlinearDistortionMetric::compute(const RolloutTrajectory& traj) const {
    if (traj.outputs.empty() || traj.outputs[0].empty()) return 0.0;

    // Extract first output channel
    std::vector<double> outputSignal;
    for (const auto& o : traj.outputs) {
        outputSignal.push_back(o.empty() ? 0.0 : o[0]);
    }

    auto mags = computeFFTMagnitude(outputSignal, traj.dt);
    if (mags.empty()) return 0.0;

    // Find fundamental frequency (largest non-DC component)
    size_t fundamentalIdx = 1;
    double maxMag = 0.0;
    for (size_t i = 1; i < mags.size(); ++i) {
        if (mags[i] > maxMag) {
            maxMag = mags[i];
            fundamentalIdx = i;
        }
    }

    if (maxMag < 1e-30) return 0.0;

    // Compute THD: ratio of harmonic power to fundamental
    double harmonicPower = 0.0;
    for (size_t i = 1; i < mags.size(); ++i) {
        if (i != fundamentalIdx) {
            harmonicPower += mags[i] * mags[i];
        }
    }

    return std::sqrt(harmonicPower) / maxMag;
}

// ---------------------------------------------------------------------------
// Combined Metric Evaluator
// ---------------------------------------------------------------------------

void CombinedMetricEvaluator::configure(const std::vector<WeightedMetric>& metrics,
                                          const std::vector<SafeSetBound>& safeSet,
                                          double controlSatMin, double controlSatMax) {
    metrics_.clear();
    for (const auto& wm : metrics) {
        auto metric = createMetric(wm);

        // For metrics that need safe-set bounds, re-create with bounds
        if (wm.type == MetricType::RegionOfAttractionEscape) {
            metric = std::make_unique<ROAEscapeMetric>(safeSet);
        } else if (wm.type == MetricType::ConstraintViolationIntegral) {
            metric = std::make_unique<ConstraintViolationMetric>(safeSet);
        } else if (wm.type == MetricType::ControlSaturationTime) {
            metric = std::make_unique<ControlSaturationMetric>(controlSatMin, controlSatMax);
        }

        metrics_.emplace_back(wm.weight, std::move(metric));
    }
}

double CombinedMetricEvaluator::evaluate(const RolloutTrajectory& traj) const {
    double J = 0.0;
    for (const auto& [weight, metric] : metrics_) {
        J += weight * metric->compute(traj);
    }
    return J;
}

std::vector<double> CombinedMetricEvaluator::evaluateBreakdown(
    const RolloutTrajectory& traj) const
{
    std::vector<double> breakdown;
    breakdown.reserve(metrics_.size());
    for (const auto& [weight, metric] : metrics_) {
        breakdown.push_back(weight * metric->compute(traj));
    }
    return breakdown;
}

} // namespace Destabilizer
