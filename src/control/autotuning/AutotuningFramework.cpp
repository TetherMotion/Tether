/**
 * @file AutotuningFramework.cpp
 * @brief Implementation of core autotuning framework components
 */

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// StandardCostFunctions Implementation
// ============================================================================

double StandardCostFunctions::ISE(const std::vector<double>& error, double dt) {
    double ise = 0.0;
    for (double e : error) {
        ise += e * e * dt;
    }
    return ise;
}

double StandardCostFunctions::IAE(const std::vector<double>& error, double dt) {
    double iae = 0.0;
    for (double e : error) {
        iae += std::abs(e) * dt;
    }
    return iae;
}

double StandardCostFunctions::ITAE(const std::vector<double>& error, double dt) {
    double itae = 0.0;
    double t = 0.0;
    for (double e : error) {
        itae += t * std::abs(e) * dt;
        t += dt;
    }
    return itae;
}

double StandardCostFunctions::ITSE(const std::vector<double>& error, double dt) {
    double itse = 0.0;
    double t = 0.0;
    for (double e : error) {
        itse += t * e * e * dt;
        t += dt;
    }
    return itse;
}

double StandardCostFunctions::combinedCost(const std::vector<double>& response,
                                           const std::vector<double>& reference,
                                           double overshootWeight,
                                           double settlingWeight,
                                           double controlEffortWeight,
                                           double dt) {
    if (response.empty() || reference.empty()) return std::numeric_limits<double>::max();
    
    size_t n = std::min(response.size(), reference.size());
    
    // Compute error
    std::vector<double> error(n);
    for (size_t i = 0; i < n; ++i) {
        error[i] = reference[i] - response[i];
    }
    
    // ISE component
    double ise = ISE(error, dt);
    
    // Overshoot component
    double finalValue = reference.back();
    double maxValue = *std::max_element(response.begin(), response.end());
    double overshoot = 0.0;
    if (finalValue > 0) {
        overshoot = std::max(0.0, (maxValue - finalValue) / finalValue);
    }
    
    // Settling time component (simplified - find time to stay within 2%)
    double settlingTime = 0.0;
    double threshold = 0.02 * finalValue;
    for (size_t i = n; i > 0; --i) {
        if (std::abs(response[i-1] - finalValue) > threshold) {
            settlingTime = i * dt;
            break;
        }
    }
    
    // Control effort (using error derivative as proxy)
    double controlEffort = 0.0;
    for (size_t i = 1; i < error.size(); ++i) {
        double de = (error[i] - error[i-1]) / dt;
        controlEffort += de * de * dt;
    }
    
    return ise + overshootWeight * overshoot + settlingWeight * settlingTime + 
           controlEffortWeight * controlEffort;
}

// ============================================================================
// SimulationCostFunction Implementation
// ============================================================================

SimulationCostFunction::SimulationCostFunction(
    std::shared_ptr<TunableController> controller,
    std::shared_ptr<ProcessModel> model,
    std::vector<double> referenceSignal,
    double dt)
    : m_controller(std::move(controller))
    , m_model(std::move(model))
    , m_reference(std::move(referenceSignal))
    , m_dt(dt) {}

double SimulationCostFunction::evaluate(const ParameterVector& params) {
    if (!m_controller || !m_model) return std::numeric_limits<double>::max();
    if (m_reference.empty() || m_dt <= 0.0) return std::numeric_limits<double>::max();
    
    // Set parameters
    if (!m_controller->setParameters(params)) {
        return std::numeric_limits<double>::max();
    }
    
    // Reset controller
    m_controller->reset();
    
    // Closed-loop simulation. The TunableController::evaluate API is batch
    // (whole trajectory in, whole trajectory out), so the loop is solved as
    // a fixed point: guess the process output y, compute the control
    // trajectory u = C(y, r), simulate the plant P(u) -> y', and iterate
    // until y' stops changing. For a stable, contractive closed loop this
    // converges in a few iterations; diverging parameter sets produce a
    // non-finite or exploding response which maps to max cost.
    const size_t n = m_reference.size();
    const FOPDTModel plant = m_model->toFOPDT();
    
    // Discrete-time FOPDT simulator: transport delay line + first-order lag.
    const int delaySteps = plant.L > 0.0
        ? static_cast<int>(std::lround(plant.L / m_dt)) : 0;
    std::vector<double> delayLine(delaySteps + 1, 0.0);
    size_t delayIdx = 0;
    const double a = plant.tau > 0.0
        ? 1.0 - std::exp(-m_dt / plant.tau) : 1.0;
    const double K = plant.K;
    
    auto simulatePlant = [&](const std::vector<double>& u,
                             std::vector<double>& y) {
        std::fill(delayLine.begin(), delayLine.end(), 0.0);
        delayIdx = 0;
        double state = 0.0;
        y.assign(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            const double ui = i < u.size() ? u[i] : 0.0;
            double ud = ui;
            if (delaySteps > 0) {
                ud = delayLine[delayIdx];
                delayLine[delayIdx] = ui;
                delayIdx = (delayIdx + 1) % delayLine.size();
            }
            state += a * (K * ud - state);
            y[i] = state;
        }
    };
    
    std::vector<double> yPrev(n, 0.0), yNew;
    constexpr int kMaxFixedPointIters = 12;
    for (int iter = 0; iter < kMaxFixedPointIters; ++iter) {
        std::vector<double> u = m_controller->evaluate(yPrev, m_reference, m_dt);
        simulatePlant(u, yNew);
        
        // Divergence guard: an exploding or non-finite response means the
        // parameter set destabilizes the loop.
        double maxAbs = 0.0;
        for (double v : yNew) {
            if (!std::isfinite(v)) return std::numeric_limits<double>::max();
            maxAbs = std::max(maxAbs, std::abs(v));
        }
        if (maxAbs > 1e8) return std::numeric_limits<double>::max();
        
        double maxDiff = 0.0;
        for (size_t i = 0; i < n; ++i) {
            maxDiff = std::max(maxDiff, std::abs(yNew[i] - yPrev[i]));
        }
        // Damped update improves convergence for mildly contractive loops.
        const double relax = (iter < 4) ? 0.5 : 1.0;
        for (size_t i = 0; i < n; ++i) {
            yPrev[i] += relax * (yNew[i] - yPrev[i]);
        }
        if (maxDiff < 1e-9 * std::max(1.0, maxAbs)) break;
    }
    
    // Compute cost on the converged process output
    return StandardCostFunctions::combinedCost(
        yNew, m_reference, m_weightOvershoot, m_weightSettling, m_weightControl, m_dt);
}

void SimulationCostFunction::setWeights(double ise, double overshoot, 
                                        double settling, double control) {
    m_weightISE = ise;
    m_weightOvershoot = overshoot;
    m_weightSettling = settling;
    m_weightControl = control;
}

// ============================================================================
// Utility Functions Implementation
// ============================================================================

StepResponseMetrics analyzeStepResponse(
    const std::vector<double>& response,
    double finalValue,
    double dt,
    double settlingThreshold) {
    
    StepResponseMetrics metrics;
    
    if (response.empty()) return metrics;
    
    // Rise time (10% to 90%)
    double target10 = 0.1 * finalValue;
    double target90 = 0.9 * finalValue;
    double time10 = 0.0, time90 = 0.0;
    bool found10 = false, found90 = false;
    
    for (size_t i = 0; i < response.size(); ++i) {
        double t = i * dt;
        if (!found10 && response[i] >= target10) {
            time10 = t;
            found10 = true;
        }
        if (!found90 && response[i] >= target90) {
            time90 = t;
            break;
        }
    }
    metrics.riseTime = time90 - time10;
    
    // Peak time and overshoot
    double maxValue = response[0];
    size_t peakIdx = 0;
    for (size_t i = 1; i < response.size(); ++i) {
        if (response[i] > maxValue) {
            maxValue = response[i];
            peakIdx = i;
        }
    }
    metrics.peakTime = peakIdx * dt;
    
    if (finalValue > 0) {
        metrics.overshoot = std::max(0.0, (maxValue - finalValue) / finalValue) * 100.0;
    }
    
    // Settling time (to within threshold)
    double threshold = settlingThreshold * std::abs(finalValue);
    metrics.settlingTime = response.size() * dt;  // Default to full duration
    
    for (size_t i = response.size(); i > 0; --i) {
        if (std::abs(response[i-1] - finalValue) > threshold) {
            metrics.settlingTime = i * dt;
            break;
        }
    }
    
    // Steady state
    metrics.steadyStateValue = response.back();
    metrics.steadyStateError = finalValue - response.back();
    
    return metrics;
}

FrequencyResponseMetrics analyzeFrequencyResponse(
    const ProcessModel& model,
    const TunableController& /*controller*/) {
    
    FrequencyResponseMetrics metrics;
    
    // Find gain crossover frequency (where |L(jw)| = 1)
    // And phase crossover frequency (where phase = -180°)
    
    double wcGain = 0.0;
    double wcPhase = 0.0;
    double minMagDiff = std::numeric_limits<double>::max();
    double minPhaseDiff = std::numeric_limits<double>::max();
    
    // Scan frequencies
    for (double omega = 0.001; omega < 1000.0; omega *= 1.1) {
        std::complex<double> G = model.evaluate(omega);
        double mag = std::abs(G);
        double phase = std::arg(G) * 180.0 / M_PI;  // Convert to degrees
        
        // Gain crossover (|G| = 1)
        double magDiff = std::abs(mag - 1.0);
        if (magDiff < minMagDiff) {
            minMagDiff = magDiff;
            wcGain = omega;
        }
        
        // Phase crossover (phase = -180)
        double phaseDiff = std::abs(phase + 180.0);
        if (phaseDiff < minPhaseDiff && mag > 0.001) {
            minPhaseDiff = phaseDiff;
            wcPhase = omega;
        }
    }
    
    metrics.crossoverFrequency = wcGain;
    
    // Gain margin: 1 / |G(jw_pc)| at phase crossover
    if (wcPhase > 0) {
        std::complex<double> G_pc = model.evaluate(wcPhase);
        double magAtPc = std::abs(G_pc);
        if (magAtPc > 0) {
            metrics.gainMargin = 20.0 * std::log10(1.0 / magAtPc);
        }
    }
    
    // Phase margin: 180 + phase(G(jw_gc)) at gain crossover
    if (wcGain > 0) {
        std::complex<double> G_gc = model.evaluate(wcGain);
        double phaseAtGc = std::arg(G_gc) * 180.0 / M_PI;
        metrics.phaseMargin = 180.0 + phaseAtGc;
    }
    
    // Bandwidth (where |G| drops to -3dB)
    for (double omega = 0.001; omega < 1000.0; omega *= 1.1) {
        std::complex<double> G = model.evaluate(omega);
        double magDb = 20.0 * std::log10(std::abs(G));
        if (magDb < -3.0) {
            metrics.bandwidth = omega;
            break;
        }
    }
    
    // Peak magnitude
    double peakMag = 0.0;
    for (double omega = 0.001; omega < 1000.0; omega *= 1.1) {
        std::complex<double> G = model.evaluate(omega);
        double mag = std::abs(G);
        if (mag > peakMag) {
            peakMag = mag;
        }
    }
    metrics.peakMagnitude = 20.0 * std::log10(peakMag);
    
    return metrics;
}

} // namespace Autotuning
} // namespace tether::control
