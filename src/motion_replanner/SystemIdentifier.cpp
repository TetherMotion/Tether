/**
 * @file SystemIdentifier.cpp
 * @brief Implementation of system identification algorithms
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
// SystemIdentifier Implementation
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
// PID Analysis
//-----------------------------------------------------------------------------

PIDTuningAssessment SystemIdentifier::analyzePIDTuning() {
    PIDTuningAssessment result = {};
    
    if (samples_.size() < 50) {
        return result;
    }
    
    // Find step changes to analyze
    auto steps = findSteps(config_.stepThreshold);
    
    if (steps.empty()) {
        // Analyze continuous tracking instead
        return analyzeTracking();
    }
    
    // Analyze first step response
    auto [stepIdx, stepMag] = steps[0];
    
    std::vector<double> time, response;
    extractStepResponse(stepIdx, stepMag, time, response);
    
    if (response.empty()) {
        return result;
    }
    
    double finalValue = samples_[stepIdx].commanded + stepMag;
    double initialValue = samples_[stepIdx].commanded;
    
    // Steady state error
    double steadyValue = response.back();
    result.steadyStateError = (finalValue - steadyValue) / std::abs(stepMag) * 100;
    
    // Rise time
    double val10 = initialValue + 0.1 * stepMag;
    double val90 = initialValue + 0.9 * stepMag;
    double t10 = 0, t90 = 0;
    
    for (size_t i = 0; i < response.size(); ++i) {
        if (t10 == 0 && ((stepMag > 0 && response[i] >= val10) || 
                         (stepMag < 0 && response[i] <= val10))) {
            t10 = time[i];
        }
        if (t90 == 0 && ((stepMag > 0 && response[i] >= val90) || 
                         (stepMag < 0 && response[i] <= val90))) {
            t90 = time[i];
            break;
        }
    }
    
    result.riseTime = t90 - t10;
    
    // Overshoot
    double peakVal = initialValue;
    for (const auto& r : response) {
        if ((stepMag > 0 && r > peakVal) || (stepMag < 0 && r < peakVal)) {
            peakVal = r;
        }
    }
    
    result.overshoot = std::abs(peakVal - finalValue) / std::abs(stepMag) * 100;
    
    // Settling time
    double settleThreshold = std::abs(stepMag) * config_.settlingThreshold;
    for (size_t i = response.size() - 1; i > 0; --i) {
        if (std::abs(response[i] - finalValue) > settleThreshold) {
            result.settlingTime = time[i];
            break;
        }
    }
    
    // Estimate damping ratio from overshoot
    if (result.overshoot > 0) {
        double logOs = std::log(result.overshoot / 100.0);
        result.dampingRatio = -logOs / std::sqrt(M_PI * M_PI + logOs * logOs);
    } else {
        result.dampingRatio = 1.0;  // Overdamped or critically damped
    }
    
    // Estimate natural frequency from rise time
    if (result.riseTime > 0 && result.dampingRatio > 0) {
        result.naturalFrequency = 1.8 / result.riseTime;  // Approximate
    }
    
    // Compute quality scores
    result.stabilityScore = std::max(0.0, 100.0 - result.overshoot);
    result.responseScore = std::max(0.0, 100.0 * (1.0 - result.riseTime / 1.0));  // <1s is good
    result.accuracyScore = std::max(0.0, 100.0 - std::abs(result.steadyStateError));
    result.overallScore = (result.stabilityScore + result.responseScore + result.accuracyScore) / 3.0;
    
    // Generate recommendations
    if (result.overshoot > 20) {
        result.issues.push_back("High overshoot (" + std::to_string(result.overshoot) + "%)");
        result.recommendations.push_back("Reduce proportional gain or increase derivative gain");
    }
    
    if (result.steadyStateError > 2) {
        result.issues.push_back("Significant steady-state error");
        result.recommendations.push_back("Increase integral gain");
    }
    
    if (result.riseTime > 0.5) {
        result.issues.push_back("Slow response");
        result.recommendations.push_back("Increase proportional gain");
    }
    
    if (result.settlingTime > 2.0) {
        result.issues.push_back("Slow settling");
        result.recommendations.push_back("Adjust damping - may need D gain tuning");
    }
    
    // Suggest gains using heuristics
    suggestZieglerNichols(result);
    
    return result;
}

PIDTuningAssessment SystemIdentifier::analyzeTracking() {
    PIDTuningAssessment result = {};
    
    // Analyze continuous tracking error
    double sumError = 0, sumErrorSq = 0;
    double maxError = 0;
    
    for (const auto& s : samples_) {
        double error = s.commanded - s.actual;
        sumError += error;
        sumErrorSq += error * error;
        maxError = std::max(maxError, std::abs(error));
    }
    
    double meanError = sumError / samples_.size();
    double rmsError = std::sqrt(sumErrorSq / samples_.size());
    
    result.steadyStateError = meanError;
    result.accuracyScore = std::max(0.0, 100.0 * (1.0 - rmsError / 10.0));
    result.overallScore = result.accuracyScore;
    
    if (rmsError > 1.0) {
        result.issues.push_back("High RMS tracking error: " + std::to_string(rmsError));
        result.recommendations.push_back("Consider increasing all gains proportionally");
    }
    
    return result;
}

void SystemIdentifier::suggestZieglerNichols(PIDTuningAssessment& assessment) {
    // Use step response to estimate parameters
    // Assuming we have rise time and damping ratio
    
    if (assessment.riseTime <= 0 || assessment.naturalFrequency <= 0) {
        return;
    }
    
    // Approximate ultimate gain and period
    double Ku = 4.0 / (M_PI * assessment.dampingRatio);  // Approximation
    double Pu = 2.0 * M_PI / assessment.naturalFrequency;
    
    // Classic Ziegler-Nichols
    assessment.suggestedKp = 0.6 * Ku;
    assessment.suggestedKi = assessment.suggestedKp / (0.5 * Pu);
    assessment.suggestedKd = assessment.suggestedKp * 0.125 * Pu;
    
    assessment.tuningAdvice = "Ziegler-Nichols suggests: Kp=" + 
        std::to_string(assessment.suggestedKp) + ", Ki=" +
        std::to_string(assessment.suggestedKi) + ", Kd=" +
        std::to_string(assessment.suggestedKd);
}

//-----------------------------------------------------------------------------
// Dynamics Identification
//-----------------------------------------------------------------------------

DynamicsIdentificationResult SystemIdentifier::identifyDynamics() {
    DynamicsIdentificationResult result = {};
    
    auto steps = findSteps(config_.stepThreshold);
    if (steps.empty()) {
        return result;
    }
    
    auto [stepIdx, stepMag] = steps[0];
    
    std::vector<double> time, response;
    extractStepResponse(stepIdx, stepMag, time, response);
    
    if (response.size() < 10) {
        return result;
    }
    
    double finalValue = samples_[stepIdx].commanded + stepMag;
    double initialValue = samples_[stepIdx].commanded;
    
    // Normalize response
    std::vector<double> normResponse(response.size());
    for (size_t i = 0; i < response.size(); ++i) {
        normResponse[i] = (response[i] - initialValue) / stepMag;
    }
    
    // Try first-order fit
    auto [K1, T1] = fitFirstOrder(time, normResponse, 1.0);
    result.gain = K1;
    result.timeConstant = T1;
    
    // Try second-order fit
    auto [K2, wn, zeta] = fitSecondOrder(time, normResponse, 1.0);
    
    // Choose better model based on fit
    result.naturalFrequency = wn;
    result.dampingRatio = zeta;
    
    // Bandwidth estimate
    result.bandwidthHz = result.naturalFrequency / (2.0 * M_PI);
    
    // System order estimate
    if (zeta >= 1.0) {
        result.systemOrder = 1;
    } else {
        result.systemOrder = 2;
    }
    
    return result;
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

} // namespace MotionReplanner
