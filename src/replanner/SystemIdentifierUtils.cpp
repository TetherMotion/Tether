/**
 * @file SystemIdentifierUtils.cpp
 * @brief StribeckCalculator, OnlineDelayEstimator, RelayAutoTuner implementations
 * 
 * Split from SystemIdentifier.cpp for maintainability.
 */

#include "tether/motion_replanner/SystemIdentifier.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// StribeckCalculator Implementation
//=============================================================================

double StribeckCalculator::calculate(double velocity,
                                     double coulomb, double staticFriction,
                                     double stribeckVelocity, double stribeckExponent,
                                     double viscousCoeff) {
    double absV = std::abs(velocity);
    double sign = (velocity >= 0) ? 1.0 : -1.0;
    
    double stribeck = (staticFriction - coulomb) *
        std::exp(-std::pow(absV / stribeckVelocity, stribeckExponent));
    
    return sign * (coulomb + stribeck) + viscousCoeff * velocity;
}

FrictionModelParams StribeckCalculator::fit(const std::vector<double>& velocities,
                                            const std::vector<double>& forces) {
    FrictionModelParams params;
    params.type = FrictionModelType::Stribeck;
    
    // Initial estimates
    // Coulomb: mean of forces at high velocity
    // Static: max force at low velocity
    // Stribeck velocity: velocity at half drop from static to Coulomb
    
    std::vector<std::pair<double, double>> sorted;
    for (size_t i = 0; i < velocities.size(); ++i) {
        sorted.push_back({std::abs(velocities[i]), std::abs(forces[i])});
    }
    std::sort(sorted.begin(), sorted.end());
    
    // Low velocity region (first 10%)
    size_t lowEnd = sorted.size() / 10;
    double maxLow = 0;
    for (size_t i = 0; i < lowEnd; ++i) {
        maxLow = std::max(maxLow, sorted[i].second);
    }
    
    // High velocity region (last 20%)
    size_t highStart = sorted.size() * 8 / 10;
    double sumHigh = 0;
    for (size_t i = highStart; i < sorted.size(); ++i) {
        sumHigh += sorted[i].second;
    }
    double meanHigh = sumHigh / (sorted.size() - highStart);
    
    params.staticFriction = maxLow;
    params.coulombForce = meanHigh;
    
    // Find Stribeck velocity (where force is midway)
    double midForce = (maxLow + meanHigh) / 2;
    params.stribeckVelocity = sorted[0].first;  // Default
    for (const auto& [v, f] : sorted) {
        if (f <= midForce) {
            params.stribeckVelocity = v;
            break;
        }
    }
    params.stribeckVelocity = std::max(0.1, params.stribeckVelocity);
    params.stribeckExponent = 2.0;  // Default
    
    // Estimate viscous from high velocity slope
    if (sorted.size() > 2) {
        double v1 = sorted[highStart].first;
        double v2 = sorted.back().first;
        double f1 = sorted[highStart].second;
        double f2 = sorted.back().second;
        
        if (v2 > v1) {
            params.viscousCoeff = (f2 - f1) / (v2 - v1);
        }
    }
    
    // Compute R²
    double ssRes = 0, ssTot = 0;
    double meanF = 0;
    for (auto f : forces) meanF += std::abs(f);
    meanF /= forces.size();
    
    for (size_t i = 0; i < velocities.size(); ++i) {
        double fitted = calculate(velocities[i], params.coulombForce, params.staticFriction,
                                  params.stribeckVelocity, params.stribeckExponent,
                                  params.viscousCoeff);
        ssRes += (forces[i] - fitted) * (forces[i] - fitted);
        ssTot += (std::abs(forces[i]) - meanF) * (std::abs(forces[i]) - meanF);
    }
    
    params.rSquared = 1.0 - ssRes / (ssTot + 1e-9);
    params.residualRMS = std::sqrt(ssRes / velocities.size());
    
    return params;
}

//=============================================================================
// OnlineDelayEstimator Implementation
//=============================================================================

OnlineDelayEstimator::OnlineDelayEstimator(double maxDelay, double samplePeriod)
    : maxDelay_(maxDelay), samplePeriod_(samplePeriod),
      currentDelay_(0), confidence_(0), forgettingFactor_(0.995) {
    historySize_ = static_cast<size_t>(maxDelay / samplePeriod * 2);
}

void OnlineDelayEstimator::update(double commanded, double actual, double timestamp) {
    commandHistory_.push_back(commanded);
    actualHistory_.push_back(actual);
    timeHistory_.push_back(timestamp);
    
    // Keep history bounded
    while (commandHistory_.size() > historySize_) {
        commandHistory_.erase(commandHistory_.begin());
        actualHistory_.erase(actualHistory_.begin());
        timeHistory_.erase(timeHistory_.begin());
    }
    
    if (commandHistory_.size() < historySize_ / 2) {
        return;
    }
    
    // Periodic correlation update
    int maxLag = static_cast<int>(maxDelay_ / samplePeriod_);
    auto corr = SystemIdentifier::crossCorrelation(commandHistory_, actualHistory_, maxLag);
    
    auto maxIt = std::max_element(corr.begin(), corr.end());
    int peakLag = std::distance(corr.begin(), maxIt) - maxLag;
    
    double newDelay = peakLag * samplePeriod_;
    
    // Exponential smoothing
    currentDelay_ = forgettingFactor_ * currentDelay_ + (1 - forgettingFactor_) * newDelay;
    confidence_ = *maxIt;
}

void OnlineDelayEstimator::reset() {
    commandHistory_.clear();
    actualHistory_.clear();
    timeHistory_.clear();
    currentDelay_ = 0;
    confidence_ = 0;
}

//=============================================================================
// RelayAutoTuner Implementation
//=============================================================================

RelayAutoTuner::RelayAutoTuner(double relayAmplitude, double hysteresis)
    : relayAmplitude_(relayAmplitude), hysteresis_(hysteresis),
      currentOutput_(relayAmplitude), oscillationCount_(0),
      lastCrossing_(0), lastAbove_(true) {}

double RelayAutoTuner::process(double setpoint, double measurement, double timestamp) {
    double error = setpoint - measurement;
    
    bool above = (error > hysteresis_);
    bool below = (error < -hysteresis_);
    
    if (above && !lastAbove_) {
        // Crossed from below to above
        currentOutput_ = relayAmplitude_;
        
        if (lastCrossing_ > 0) {
            peaks_.push_back(measurement);
            peakTimes_.push_back(timestamp);
            oscillationCount_++;
        }
        lastCrossing_ = timestamp;
        lastAbove_ = true;
    } else if (below && lastAbove_) {
        // Crossed from above to below
        currentOutput_ = -relayAmplitude_;
        
        if (lastCrossing_ > 0) {
            peaks_.push_back(measurement);
            peakTimes_.push_back(timestamp);
            oscillationCount_++;
        }
        lastCrossing_ = timestamp;
        lastAbove_ = false;
    }
    
    return currentOutput_;
}

RelayAutoTuner::TuningResult RelayAutoTuner::computeTuning() {
    TuningResult result = {};
    result.oscillationCount = oscillationCount_;
    
    if (peaks_.size() < 6) {
        result.isValid = false;
        return result;
    }
    
    // Compute average amplitude
    double sumAmp = 0;
    for (size_t i = 2; i < peaks_.size(); ++i) {
        sumAmp += std::abs(peaks_[i] - peaks_[i-1]);
    }
    double avgAmp = sumAmp / (peaks_.size() - 2) / 2;  // peak-to-peak / 2
    
    // Compute average period
    double sumPeriod = 0;
    for (size_t i = 4; i < peakTimes_.size(); i += 2) {
        sumPeriod += peakTimes_[i] - peakTimes_[i-2];
    }
    double avgPeriod = sumPeriod / ((peakTimes_.size() - 4) / 2 + 1);
    
    // Ultimate gain: Ku = 4*d / (pi*a) where d=relay amplitude, a=oscillation amplitude
    result.ultimateGain = 4.0 * relayAmplitude_ / (M_PI * avgAmp);
    result.ultimatePeriod = avgPeriod;
    
    // Ziegler-Nichols PID
    result.zieglerNichols.Kp = 0.6 * result.ultimateGain;
    result.zieglerNichols.Ki = 2.0 * result.zieglerNichols.Kp / result.ultimatePeriod;
    result.zieglerNichols.Kd = result.zieglerNichols.Kp * result.ultimatePeriod / 8.0;
    
    // Tyreus-Luyben (less aggressive)
    result.tyreus.Kp = 0.45 * result.ultimateGain;
    result.tyreus.Ki = result.tyreus.Kp / (2.2 * result.ultimatePeriod);
    result.tyreus.Kd = result.tyreus.Kp * result.ultimatePeriod / 6.3;
    
    // Some overshoot rule
    result.someOvershoot.Kp = 0.33 * result.ultimateGain;
    result.someOvershoot.Ki = result.someOvershoot.Kp / (0.5 * result.ultimatePeriod);
    result.someOvershoot.Kd = result.someOvershoot.Kp * result.ultimatePeriod / 3.0;
    
    // No overshoot rule
    result.noOvershoot.Kp = 0.2 * result.ultimateGain;
    result.noOvershoot.Ki = result.noOvershoot.Kp / (0.5 * result.ultimatePeriod);
    result.noOvershoot.Kd = result.noOvershoot.Kp * result.ultimatePeriod / 3.0;
    
    result.isValid = true;
    return result;
}

void RelayAutoTuner::reset() {
    peaks_.clear();
    peakTimes_.clear();
    oscillationCount_ = 0;
    lastCrossing_ = 0;
    currentOutput_ = relayAmplitude_;
    lastAbove_ = true;
}

//=============================================================================
// SystemIdentifier::computeFFT Implementation
//=============================================================================

void SystemIdentifier::computeFFT(
    const std::vector<double>& signal,
    std::vector<double>& frequencies,
    std::vector<double>& magnitudes,
    std::vector<double>& phases,
    double sampleRate) {
    
    const size_t N = signal.size();
    if (N == 0 || sampleRate <= 0.0) return;
    
    // DFT (real-input) — only positive frequencies up to Nyquist
    const size_t numBins = N / 2 + 1;
    frequencies.resize(numBins);
    magnitudes.resize(numBins);
    phases.resize(numBins);
    
    for (size_t k = 0; k < numBins; ++k) {
        double real = 0.0;
        double imag = 0.0;
        for (size_t n = 0; n < N; ++n) {
            double angle = 2.0 * M_PI * k * n / static_cast<double>(N);
            real += signal[n] * std::cos(angle);
            imag -= signal[n] * std::sin(angle);
        }
        frequencies[k] = static_cast<double>(k) * sampleRate / static_cast<double>(N);
        magnitudes[k] = std::sqrt(real * real + imag * imag) * 2.0 / static_cast<double>(N);
        phases[k] = std::atan2(imag, real);
    }
    // DC component doesn't need doubling
    if (!magnitudes.empty()) {
        magnitudes[0] /= 2.0;
    }
}

} // namespace MotionReplanner
