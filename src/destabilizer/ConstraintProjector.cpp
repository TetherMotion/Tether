/// @file ConstraintProjector.cpp
/// @brief Constraint projection implementations.

#include "tether/destabilizer/ConstraintProjector.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace Destabilizer {

void ConstraintProjector::configure(const ChannelConstraints& constraints,
                                     const PerturbationConfig& pertConfig,
                                     double horizon, double dt)
{
    constraints_ = constraints;
    pertConfig_ = pertConfig;
    horizon_ = horizon;
    dt_ = dt;
}

std::vector<bool> ConstraintProjector::project(std::vector<double>& theta) const {
    // Constraint activity flags: [amplitude, rate, energy, frequency, dutyCycle]
    std::vector<bool> active(5, false);

    if (theta.empty()) return active;

    // 1. Amplitude projection
    bool ampBefore = isAmplitudeActive(theta, constraints_.amplitudeMax);
    projectAmplitude(theta, constraints_.amplitudeMax);
    active[0] = isAmplitudeActive(theta, constraints_.amplitudeMax);

    // 2. Rate projection (for piecewise parameterizations)
    if (pertConfig_.type == PerturbationType::PiecewiseConstant ||
        pertConfig_.type == PerturbationType::PiecewiseLinear) {
        double segDur = horizon_ / std::max(1, static_cast<int>(theta.size()));
        if (constraints_.rateMax < 1e5) {
            double prevSize = 0.0;
            for (size_t i = 1; i < theta.size(); ++i) {
                prevSize += std::abs(theta[i] - theta[i-1]);
            }
            projectRate(theta, constraints_.rateMax, segDur);
            double afterSize = 0.0;
            for (size_t i = 1; i < theta.size(); ++i) {
                afterSize += std::abs(theta[i] - theta[i-1]);
            }
            active[1] = (afterSize < prevSize - 1e-10);
        }
    }

    // 3. Energy projection
    if (constraints_.energyMax < 1e5) {
        double segDur = horizon_ / std::max(1, static_cast<int>(theta.size()));
        double energyBefore = 0.0;
        for (double v : theta) energyBefore += v * v * segDur;
        projectEnergy(theta, constraints_.energyMax, segDur);
        double energyAfter = 0.0;
        for (double v : theta) energyAfter += v * v * segDur;
        active[2] = (energyAfter < energyBefore - 1e-10);
    }

    // 4. Frequency projection (for Fourier parameterization)
    if (pertConfig_.type == PerturbationType::FourierSpectral) {
        projectFrequency(theta, constraints_.freqMin, constraints_.freqMax);
        // Check if any frequency was clamped
        for (int k = 0; k < pertConfig_.numHarmonics && (3*k+1) < static_cast<int>(theta.size()); ++k) {
            double f = theta[3*k + 1];
            if (std::abs(f - constraints_.freqMin) < 1e-6 ||
                std::abs(f - constraints_.freqMax) < 1e-6) {
                active[3] = true;
            }
        }
    }

    // 5. Duty cycle projection
    if (constraints_.dutyCycleMax < 1.0 - 1e-6) {
        projectDutyCycle(theta, constraints_.dutyCycleMax, horizon_);
        active[4] = true;  // Simplified: always mark active when constrained
    }

    // Re-apply amplitude after other projections may have changed values
    projectAmplitude(theta, constraints_.amplitudeMax);

    return active;
}

void ConstraintProjector::projectAmplitude(std::vector<double>& theta, double aMax) {
    for (auto& v : theta) {
        v = std::clamp(v, -aMax, aMax);
    }
}

void ConstraintProjector::projectRate(std::vector<double>& theta, double rateMax,
                                        double segmentDuration) {
    if (theta.size() < 2 || segmentDuration <= 0.0) return;

    double maxDelta = rateMax * segmentDuration;

    // Forward pass: enforce rate from left to right
    for (size_t i = 1; i < theta.size(); ++i) {
        double diff = theta[i] - theta[i-1];
        if (std::abs(diff) > maxDelta) {
            theta[i] = theta[i-1] + std::copysign(maxDelta, diff);
        }
    }
}

void ConstraintProjector::projectEnergy(std::vector<double>& theta, double eMax,
                                          double segmentDuration) {
    if (theta.empty() || segmentDuration <= 0.0) return;

    // Compute current energy: ∫ u_p²dt ≈ Σ θ_i² · segDur
    double energy = 0.0;
    for (double v : theta) energy += v * v * segmentDuration;

    if (energy > eMax && energy > 0.0) {
        double scale = std::sqrt(eMax / energy);
        for (auto& v : theta) v *= scale;
    }
}

void ConstraintProjector::projectFrequency(std::vector<double>& theta,
                                             double fMin, double fMax) {
    // For Fourier parameterization: θ = [a1, f1, φ1, a2, f2, φ2, ...]
    // Clamp frequency components to [fMin, fMax]
    for (size_t i = 1; i < theta.size(); i += 3) {
        theta[i] = std::clamp(theta[i], fMin, fMax);
    }
}

void ConstraintProjector::projectDutyCycle(std::vector<double>& theta, double dMax,
                                             double horizon) {
    if (theta.empty() || dMax >= 1.0) return;

    // Count non-zero segments
    int totalSegments = static_cast<int>(theta.size());
    int maxActive = std::max(1, static_cast<int>(dMax * totalSegments));

    // Sort by absolute value (ascending) and zero out the smallest
    // to enforce the duty cycle
    std::vector<std::pair<double, int>> absVals;
    for (int i = 0; i < totalSegments; ++i) {
        absVals.push_back({std::abs(theta[i]), i});
    }
    std::sort(absVals.begin(), absVals.end());

    int numToZero = totalSegments - maxActive;
    for (int i = 0; i < numToZero; ++i) {
        theta[absVals[i].second] = 0.0;
    }
}

bool ConstraintProjector::isAmplitudeActive(const std::vector<double>& theta,
                                              double aMax, double tol) {
    for (double v : theta) {
        if (std::abs(std::abs(v) - aMax) < tol) return true;
    }
    return false;
}

bool ConstraintProjector::verifyIdempotence(std::vector<double> theta, double tol) const {
    auto thetaCopy = theta;
    project(thetaCopy);
    auto thetaCopy2 = thetaCopy;
    project(thetaCopy2);

    for (size_t i = 0; i < thetaCopy.size(); ++i) {
        if (std::abs(thetaCopy[i] - thetaCopy2[i]) > tol) return false;
    }
    return true;
}

} // namespace Destabilizer
