/**
 * @file SystemIdentifierCore.cpp
 * @brief SystemIdentifier core implementation with delay and friction identification
 * 
 * Split from SystemIdentifier.cpp for maintainability.
 */

#include "SystemIdentifier.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <stdexcept>

namespace MotionReplanner {

//=============================================================================
// FrictionModelParams Implementation
//=============================================================================

double FrictionModelParams::calculate(double velocity) const {
    double absV = std::abs(velocity);
    double sign = (velocity >= 0) ? 1.0 : -1.0;
    
    switch (type) {
        case FrictionModelType::Coulomb:
            return sign * coulombForce;
            
        case FrictionModelType::Viscous:
            return viscousCoeff * velocity;
            
        case FrictionModelType::CoulombViscous:
            return sign * coulombForce + viscousCoeff * velocity;
            
        case FrictionModelType::Stribeck: {
            double stribeck = (staticFriction - coulombForce) *
                std::exp(-std::pow(absV / stribeckVelocity, stribeckExponent));
            return sign * (coulombForce + stribeck) + viscousCoeff * velocity;
        }
        
        case FrictionModelType::LuGre:
            // Simplified steady-state LuGre
            return sign * coulombForce + viscousCoeff * velocity;
    }
    
    return 0.0;
}

std::string FrictionModelParams::modelName() const {
    switch (type) {
        case FrictionModelType::Coulomb: return "Coulomb";
        case FrictionModelType::Viscous: return "Viscous";
        case FrictionModelType::CoulombViscous: return "Coulomb+Viscous";
        case FrictionModelType::Stribeck: return "Stribeck";
        case FrictionModelType::LuGre: return "LuGre";
    }
    return "Unknown";
}

//=============================================================================
// SystemIdentifier Implementation - Core
//=============================================================================

SystemIdentifier::SystemIdentifier(const IdentificationConfig& config)
    : config_(config) {}

void SystemIdentifier::addSample(const IdentificationSample& sample) {
    samples_.push_back(sample);
}

void SystemIdentifier::addSamples(const std::vector<IdentificationSample>& samples) {
    samples_.insert(samples_.end(), samples.begin(), samples.end());
}

void SystemIdentifier::clearSamples() {
    samples_.clear();
}

double SystemIdentifier::computeSamplingRate() const {
    if (samples_.size() < 2) return 1000.0;
    
    double totalTime = samples_.back().timestamp - samples_.front().timestamp;
    return (samples_.size() - 1) / totalTime;
}

//-----------------------------------------------------------------------------
// Delay Identification
//-----------------------------------------------------------------------------

DelayIdentificationResult SystemIdentifier::identifyDelay() {
    DelayIdentificationResult result = {};
    
    if (samples_.size() < 100) {
        result.delayConfidence = 0;
        return result;
    }
    
    // Extract commanded and actual signals
    std::vector<double> commanded, actual;
    for (const auto& s : samples_) {
        commanded.push_back(s.commanded);
        actual.push_back(s.actual);
    }
    
    double sampleRate = computeSamplingRate();
    result.samplingPeriod = 1.0 / sampleRate;
    
    // Compute cross-correlation
    int maxLag = static_cast<int>(config_.maxDelay * sampleRate);
    maxLag = std::min(maxLag, static_cast<int>(samples_.size() / 4));
    
    auto corr = crossCorrelation(commanded, actual, maxLag);
    
    // Find peak
    auto maxIt = std::max_element(corr.begin(), corr.end());
    int peakLag = std::distance(corr.begin(), maxIt) - maxLag;
    
    result.crossCorrelation = *maxIt;
    result.crossCorrelationLag = peakLag;
    result.transportDelay = peakLag / sampleRate;
    
    // Compute confidence based on peak sharpness
    double peakVal = *maxIt;
    double secondPeak = 0;
    for (size_t i = 0; i < corr.size(); ++i) {
        if (std::abs(static_cast<int>(i) - maxLag - peakLag) > 5) {
            secondPeak = std::max(secondPeak, corr[i]);
        }
    }
    
    result.delayConfidence = (peakVal - secondPeak) / (peakVal + 1e-9);
    result.delayConfidence = std::min(1.0, std::max(0.0, result.delayConfidence));
    
    return result;
}

DelayIdentificationResult SystemIdentifier::identifyDelayFromStep(
    double stepTime, double stepMagnitude) {
    
    DelayIdentificationResult result = {};
    
    // Find step index
    size_t stepIndex = 0;
    for (size_t i = 0; i < samples_.size(); ++i) {
        if (samples_[i].timestamp >= stepTime) {
            stepIndex = i;
            break;
        }
    }
    
    if (stepIndex == 0 || stepIndex >= samples_.size() - 10) {
        result.delayConfidence = 0;
        return result;
    }
    
    // Find when actual signal starts responding
    double baseline = samples_[stepIndex].actual;
    double threshold = baseline + 0.1 * stepMagnitude;
    
    size_t responseIndex = stepIndex;
    for (size_t i = stepIndex; i < samples_.size(); ++i) {
        if (std::abs(samples_[i].actual - baseline) > std::abs(threshold - baseline)) {
            responseIndex = i;
            break;
        }
    }
    
    double sampleRate = computeSamplingRate();
    result.transportDelay = (responseIndex - stepIndex) / sampleRate;
    result.samplingPeriod = 1.0 / sampleRate;
    
    // Analyze step response characteristics
    double finalValue = baseline + stepMagnitude;
    
    // Rise time (10% to 90%)
    double val10 = baseline + 0.1 * stepMagnitude;
    double val90 = baseline + 0.9 * stepMagnitude;
    size_t idx10 = responseIndex, idx90 = responseIndex;
    
    for (size_t i = responseIndex; i < samples_.size(); ++i) {
        if (idx10 == responseIndex && 
            std::abs(samples_[i].actual - baseline) >= std::abs(val10 - baseline)) {
            idx10 = i;
        }
        if (std::abs(samples_[i].actual - baseline) >= std::abs(val90 - baseline)) {
            idx90 = i;
            break;
        }
    }
    
    result.riseTime = (idx90 - idx10) / sampleRate;
    
    // Settling time (2%)
    double settleThreshold = 0.02 * std::abs(stepMagnitude);
    size_t settleIndex = samples_.size() - 1;
    
    for (size_t i = samples_.size() - 1; i > responseIndex; --i) {
        if (std::abs(samples_[i].actual - finalValue) > settleThreshold) {
            settleIndex = i + 1;
            break;
        }
    }
    
    result.settlingTime = (settleIndex - stepIndex) / sampleRate;
    
    // Overshoot
    double maxVal = baseline;
    for (size_t i = responseIndex; i < samples_.size(); ++i) {
        if ((stepMagnitude > 0 && samples_[i].actual > maxVal) ||
            (stepMagnitude < 0 && samples_[i].actual < maxVal)) {
            maxVal = samples_[i].actual;
        }
    }
    
    result.overshoot = 100.0 * std::abs(maxVal - finalValue) / std::abs(stepMagnitude);
    
    result.delayConfidence = 0.9;  // Good confidence from step response
    
    return result;
}

//-----------------------------------------------------------------------------
// Friction Identification
//-----------------------------------------------------------------------------

FrictionIdentificationResult SystemIdentifier::identifyFriction() {
    FrictionIdentificationResult result = {};
    
    // Build velocity-force pairs from steady-state regions
    std::vector<double> velocities;
    std::vector<double> forces;
    
    for (const auto& sample : samples_) {
        if (std::abs(sample.velocity) > config_.velocityRangeMin &&
            std::abs(sample.velocity) < config_.velocityRangeMax) {
            velocities.push_back(sample.velocity);
            forces.push_back(sample.torque);  // or current
        }
    }
    
    if (velocities.empty()) {
        return result;
    }
    
    result.velocities = velocities;
    result.forces = forces;
    
    // Try different friction models
    result.allModels.push_back(fitFrictionModel(FrictionModelType::Coulomb, velocities, forces));
    result.allModels.push_back(fitFrictionModel(FrictionModelType::Viscous, velocities, forces));
    result.allModels.push_back(fitFrictionModel(FrictionModelType::CoulombViscous, velocities, forces));
    result.allModels.push_back(fitFrictionModel(FrictionModelType::Stribeck, velocities, forces));
    
    // Find best model by R²
    auto bestIt = std::max_element(result.allModels.begin(), result.allModels.end(),
        [](const FrictionModelParams& a, const FrictionModelParams& b) {
            return a.rSquared < b.rSquared;
        });
    
    result.bestModel = *bestIt;
    
    // Analyze direction asymmetry
    std::vector<double> posVel, posForce, negVel, negForce;
    for (size_t i = 0; i < velocities.size(); ++i) {
        if (velocities[i] > 0) {
            posVel.push_back(velocities[i]);
            posForce.push_back(forces[i]);
        } else {
            negVel.push_back(-velocities[i]);
            negForce.push_back(-forces[i]);
        }
    }
    
    if (!posVel.empty()) {
        result.positiveDirection = fitFrictionModel(
            FrictionModelType::CoulombViscous, posVel, posForce);
    }
    if (!negVel.empty()) {
        result.negativeDirection = fitFrictionModel(
            FrictionModelType::CoulombViscous, negVel, negForce);
    }
    
    result.asymmetryRatio = (result.negativeDirection.coulombForce + 1e-9) /
                           (result.positiveDirection.coulombForce + 1e-9);
    result.isSymmetric = std::abs(result.asymmetryRatio - 1.0) < 0.2;
    
    // Compute fitted forces
    result.fittedForces.resize(velocities.size());
    for (size_t i = 0; i < velocities.size(); ++i) {
        result.fittedForces[i] = result.bestModel.calculate(velocities[i]);
    }
    
    return result;
}

FrictionModelParams SystemIdentifier::fitFrictionModel(
    FrictionModelType type,
    const std::vector<double>& velocities,
    const std::vector<double>& forces) {
    
    FrictionModelParams params;
    params.type = type;
    
    if (velocities.empty()) {
        return params;
    }
    
    size_t n = velocities.size();
    
    switch (type) {
        case FrictionModelType::Coulomb: {
            // Mean of absolute forces
            double sum = 0;
            for (size_t i = 0; i < n; ++i) {
                sum += std::abs(forces[i]);
            }
            params.coulombForce = sum / n;
            break;
        }
        
        case FrictionModelType::Viscous: {
            // Linear regression through origin
            double sumVF = 0, sumVV = 0;
            for (size_t i = 0; i < n; ++i) {
                sumVF += velocities[i] * forces[i];
                sumVV += velocities[i] * velocities[i];
            }
            params.viscousCoeff = sumVF / (sumVV + 1e-9);
            break;
        }
        
        case FrictionModelType::CoulombViscous: {
            // Linear regression: F = Fc * sign(v) + b * v
            // Use absolute velocity for Coulomb term
            double sumAbsV = 0, sumF = 0, sumV = 0, sumVF = 0, sumVV = 0;
            for (size_t i = 0; i < n; ++i) {
                double absV = std::abs(velocities[i]);
                double signV = (velocities[i] >= 0) ? 1.0 : -1.0;
                sumAbsV += absV;
                sumF += forces[i] * signV;
                sumV += velocities[i];
                sumVF += velocities[i] * forces[i];
                sumVV += velocities[i] * velocities[i];
            }
            
            // Simple approximation
            params.coulombForce = sumF / n;
            params.viscousCoeff = (sumVF - params.coulombForce * sumAbsV) / (sumVV + 1e-9);
            break;
        }
        
        case FrictionModelType::Stribeck: {
            // Use iterative fitting
            params = StribeckCalculator::fit(velocities, forces);
            break;
        }
        
        default:
            break;
    }
    
    // Compute R²
    double ssRes = 0, ssTot = 0;
    double meanF = std::accumulate(forces.begin(), forces.end(), 0.0) / n;
    
    for (size_t i = 0; i < n; ++i) {
        double fitted = params.calculate(velocities[i]);
        ssRes += (forces[i] - fitted) * (forces[i] - fitted);
        ssTot += (forces[i] - meanF) * (forces[i] - meanF);
    }
    
    params.rSquared = 1.0 - ssRes / (ssTot + 1e-9);
    params.residualRMS = std::sqrt(ssRes / n);
    
    return params;
}

//-----------------------------------------------------------------------------
// Utility Functions
//-----------------------------------------------------------------------------

std::vector<double> SystemIdentifier::crossCorrelation(
    const std::vector<double>& signal1,
    const std::vector<double>& signal2,
    int maxLag) {
    
    std::vector<double> result(2 * maxLag + 1, 0.0);
    
    size_t n = std::min(signal1.size(), signal2.size());
    
    // Compute means
    double mean1 = std::accumulate(signal1.begin(), signal1.end(), 0.0) / n;
    double mean2 = std::accumulate(signal2.begin(), signal2.end(), 0.0) / n;
    
    // Compute standard deviations
    double std1 = 0, std2 = 0;
    for (size_t i = 0; i < n; ++i) {
        std1 += (signal1[i] - mean1) * (signal1[i] - mean1);
        std2 += (signal2[i] - mean2) * (signal2[i] - mean2);
    }
    std1 = std::sqrt(std1);
    std2 = std::sqrt(std2);
    
    double norm = std1 * std2 + 1e-9;
    
    // Compute cross-correlation for each lag
    for (int lag = -maxLag; lag <= maxLag; ++lag) {
        double sum = 0;
        int count = 0;
        
        for (size_t i = 0; i < n; ++i) {
            int j = static_cast<int>(i) + lag;
            if (j >= 0 && j < static_cast<int>(n)) {
                sum += (signal1[i] - mean1) * (signal2[j] - mean2);
                count++;
            }
        }
        
        result[lag + maxLag] = sum / norm;
    }
    
    return result;
}

std::vector<std::pair<size_t, double>> SystemIdentifier::findSteps(double threshold) {
    std::vector<std::pair<size_t, double>> steps;
    
    for (size_t i = 1; i < samples_.size(); ++i) {
        double delta = samples_[i].commanded - samples_[i-1].commanded;
        if (std::abs(delta) > threshold) {
            steps.push_back({i-1, delta});
        }
    }
    
    return steps;
}

void SystemIdentifier::extractStepResponse(size_t stepIndex, double stepMagnitude,
                                           std::vector<double>& time,
                                           std::vector<double>& response) {
    time.clear();
    response.clear();
    
    double t0 = samples_[stepIndex].timestamp;
    
    for (size_t i = stepIndex; i < samples_.size(); ++i) {
        time.push_back(samples_[i].timestamp - t0);
        response.push_back(samples_[i].actual);
        
        // Stop after settling
        if (time.back() > 10.0) break;  // Max 10 seconds
    }
}

std::pair<double, double> SystemIdentifier::fitFirstOrder(
    const std::vector<double>& time,
    const std::vector<double>& response,
    double finalValue) {
    
    // First-order: y(t) = K * (1 - exp(-t/T))
    // At t=T, y = K * (1 - 1/e) ≈ 0.632 * K
    
    double K = finalValue;
    double target63 = 0.632 * K;
    
    double T = 1.0;  // Default
    for (size_t i = 1; i < response.size(); ++i) {
        if (response[i] >= target63) {
            T = time[i];
            break;
        }
    }
    
    return {K, T};
}

std::tuple<double, double, double> SystemIdentifier::fitSecondOrder(
    const std::vector<double>& time,
    const std::vector<double>& response,
    double finalValue) {
    
    // Find overshoot
    double maxVal = 0;
    double peakTime = 0;
    for (size_t i = 0; i < response.size(); ++i) {
        if (response[i] > maxVal) {
            maxVal = response[i];
            peakTime = time[i];
        }
    }
    
    double overshoot = (maxVal - finalValue) / finalValue;
    
    // Estimate damping ratio from overshoot
    double zeta;
    if (overshoot > 0) {
        double logOs = std::log(overshoot);
        zeta = -logOs / std::sqrt(M_PI * M_PI + logOs * logOs);
    } else {
        zeta = 1.0;
    }
    
    // Estimate natural frequency from peak time
    double wn;
    if (zeta < 1.0 && peakTime > 0) {
        wn = M_PI / (peakTime * std::sqrt(1 - zeta * zeta));
    } else {
        // Use time constant approximation
        double T = 0;
        for (size_t i = 1; i < response.size(); ++i) {
            if (response[i] >= 0.632 * finalValue) {
                T = time[i];
                break;
            }
        }
        wn = 1.0 / (T + 0.001);
    }
    
    return {finalValue, wn, zeta};
}

} // namespace MotionReplanner
