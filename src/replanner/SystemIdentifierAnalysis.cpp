/**
 * @file SystemIdentifierAnalysis.cpp
 * @brief PID analysis and dynamics identification
 * 
 * Split from SystemIdentifier.cpp for maintainability.
 */

#include "SystemIdentifier.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace MotionReplanner {

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

} // namespace MotionReplanner
