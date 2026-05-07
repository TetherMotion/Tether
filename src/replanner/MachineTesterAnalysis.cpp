/**
 * @file MachineTesterAnalysis.cpp
 * @brief Analysis helper functions for machine testing
 * 
 * Contains analysis methods for MachineTester:
 * - Step response analysis
 * - Circularity analysis
 * - Friction analysis
 * - Delay detection
 * - FrictionModel implementation
 * 
 * Split from MachineTester.cpp
 */

#include "MachineTester.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace MotionReplanner {

//-----------------------------------------------------------------------------
// Analysis Helpers
//-----------------------------------------------------------------------------

void MachineTester::analyzeStepResponse(TestResult& result,
                                        const std::vector<PositionSample>& desired,
                                        const std::vector<PositionSample>& actual) {
    if (actual.size() < 10) return;
    
    // Find step transitions in desired
    std::vector<size_t> stepIndices;
    for (size_t i = 1; i < desired.size(); ++i) {
        if (std::abs(desired[i].position[0] - desired[i-1].position[0]) > 0.1) {
            stepIndices.push_back(i);
        }
    }
    
    if (stepIndices.empty()) return;
    
    // Analyze first step response
    size_t stepIdx = stepIndices[0];
    double targetPos = desired[stepIdx].position[0];
    double startPos = desired[stepIdx - 1].position[0];
    double stepSize = targetPos - startPos;
    
    // Find rise time (10% to 90%)
    double pos10 = startPos + 0.1 * stepSize;
    double pos90 = startPos + 0.9 * stepSize;
    
    double time10 = 0, time90 = 0;
    double maxPos = startPos;
    double timeMax = 0;
    
    for (size_t i = stepIdx; i < actual.size(); ++i) {
        double pos = actual[i].position[0];
        double t = actual[i].timestamp - actual[stepIdx].timestamp;
        
        if (time10 == 0 && pos >= pos10) time10 = t;
        if (time90 == 0 && pos >= pos90) time90 = t;
        
        if (pos > maxPos) {
            maxPos = pos;
            // timeMax = t; // Not used
        }
        
        // Check for settling (within 2% of target)
        if (std::abs(pos - targetPos) < 0.02 * std::abs(stepSize)) {
            result.settlingTime = t;
            break;
        }
    }
    
    result.riseTime = time90 - time10;
    result.overshoot = (maxPos - targetPos) / std::abs(stepSize) * 100.0;
    result.steadyStateError = std::abs(actual.back().position[0] - targetPos);
}

void MachineTester::analyzeCircularity(TestResult& result,
                                       const std::vector<PositionSample>& actual,
                                       const MultiAxisTestConfig& config) {
    if (actual.size() < 10) return;
    
    double sumR = 0, sumR2 = 0;
    double minR = std::numeric_limits<double>::max();
    double maxR = std::numeric_limits<double>::lowest();
    
    for (const auto& sample : actual) {
        double du = sample.position[config.uAxis] - config.center[0];
        double dv = sample.position[config.vAxis] - config.center[1];
        double r = std::sqrt(du*du + dv*dv);
        
        sumR += r;
        sumR2 += r * r;
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
    }
    
    double meanR = sumR / actual.size();
    double variance = sumR2 / actual.size() - meanR * meanR;
    
    result.radiusError = meanR - config.radiusU;
    result.circularityError = maxR - minR;  // Total indicator reading
}

void MachineTester::analyzeFriction(TestResult& result,
                                    const std::vector<std::pair<double, std::vector<PositionSample>>>& data) {
    // Collect velocity vs required force (estimated from tracking error)
    std::vector<double> velocities;
    std::vector<double> forces;
    
    for (const auto& [vel, samples] : data) {
        velocities.push_back(std::abs(vel));
        
        // Estimate force from mean tracking error (simplified)
        double sumError = 0;
        for (const auto& s : samples) {
            sumError += std::abs(s.position[0]); // Simplified
        }
        forces.push_back(sumError / samples.size());
    }
    
    // Fit friction model
    auto model = FrictionModel::fitFromData(velocities, forces);
    
    result.staticFriction = model.staticFriction;
    result.viscousFriction = model.viscousFriction;
    result.coulombFriction = model.coulombFriction;
}

double MachineTester::findDelay(const std::vector<PositionSample>& desired,
                                const std::vector<PositionSample>& actual,
                                double maxDelay, double resolution) {
    double bestDelay = 0;
    double bestCorrelation = -1e9;
    
    for (double testDelay = 0; testDelay <= maxDelay; testDelay += resolution) {
        double correlation = 0;
        size_t count = 0;
        
        for (size_t i = 0; i < actual.size(); ++i) {
            double shiftedTime = actual[i].timestamp - testDelay;
            if (shiftedTime < 0 || shiftedTime > desired.back().timestamp) continue;
            
            // Find nearest desired sample
            size_t desiredIdx = static_cast<size_t>(shiftedTime / 0.001);
            if (desiredIdx >= desired.size()) continue;
            
            correlation += actual[i].velocity[0] * desired[desiredIdx].velocity[0];
            count++;
        }
        
        if (count > 0) {
            correlation /= count;
            if (correlation > bestCorrelation) {
                bestCorrelation = correlation;
                bestDelay = testDelay;
            }
        }
    }
    
    return bestDelay;
}

//=============================================================================
// FrictionModel Implementation
//=============================================================================

FrictionModel FrictionModel::fitFromData(const std::vector<double>& velocities,
                                          const std::vector<double>& forces) {
    FrictionModel model;
    
    if (velocities.empty() || forces.empty()) return model;
    
    // Find static friction (force at near-zero velocity)
    double minVel = *std::min_element(velocities.begin(), velocities.end());
    for (size_t i = 0; i < velocities.size(); ++i) {
        if (velocities[i] == minVel) {
            model.staticFriction = forces[i];
            break;
        }
    }
    
    // Fit viscous friction using linear regression
    // F = Fc + Fv * v
    double sumV = 0, sumF = 0, sumVV = 0, sumVF = 0;
    size_t n = velocities.size();
    
    for (size_t i = 0; i < n; ++i) {
        sumV += velocities[i];
        sumF += forces[i];
        sumVV += velocities[i] * velocities[i];
        sumVF += velocities[i] * forces[i];
    }
    
    double denom = n * sumVV - sumV * sumV;
    if (std::abs(denom) > 1e-9) {
        model.viscousFriction = (n * sumVF - sumV * sumF) / denom;
        model.coulombFriction = (sumF - model.viscousFriction * sumV) / n;
    }
    
    // Stribeck parameters (simplified)
    model.stribeckVelocity = 50.0;  // mm/min, typical value
    
    return model;
}

} // namespace MotionReplanner
