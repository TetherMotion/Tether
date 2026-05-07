/// @file Perturbation.cpp
/// @brief Perturbation signal parameterization implementations.

#include "tether/destabilizer/Perturbation.hpp"
#include <stdexcept>
#include <cstring>

namespace Destabilizer {

// ---------------------------------------------------------------------------
// PerturbationSignal (base)
// ---------------------------------------------------------------------------

std::vector<double> PerturbationSignal::generateSignal(
    const std::vector<double>& times,
    const std::vector<double>& theta,
    const std::vector<std::vector<double>>& states) const
{
    std::vector<double> signal(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
        const std::vector<double>& st = (i < states.size()) ? states[i] : std::vector<double>{};
        signal[i] = evaluate(times[i], theta, st);
    }
    return signal;
}

std::unique_ptr<PerturbationSignal> PerturbationSignal::create(
    PerturbationType type, const PerturbationConfig& config, double horizon)
{
    switch (type) {
    case PerturbationType::PiecewiseConstant:
        return std::make_unique<PiecewiseConstantPerturbation>(config.numSegments, horizon);
    case PerturbationType::PiecewiseLinear:
        return std::make_unique<PiecewiseLinearPerturbation>(config.numSegments, horizon);
    case PerturbationType::FourierSpectral:
        return std::make_unique<FourierPerturbation>(config.numHarmonics, horizon);
    case PerturbationType::NeuralMLP:
        // stateDim must be set by the caller; default to a reasonable size
        return std::make_unique<MLPPerturbation>(4, config.mlpHiddenSize, config.mlpNumLayers);
    case PerturbationType::ImpulseTrain:
        return std::make_unique<ImpulseTrainPerturbation>(config.numImpulses, horizon);
    case PerturbationType::BangBang:
        return std::make_unique<BangBangPerturbation>(config.numSegments, horizon);
    }
    return std::make_unique<PiecewiseConstantPerturbation>(config.numSegments, horizon);
}

// ---------------------------------------------------------------------------
// PiecewiseConstant
// ---------------------------------------------------------------------------

PiecewiseConstantPerturbation::PiecewiseConstantPerturbation(int numSegments, double horizon)
    : numSegments_(std::max(1, numSegments))
    , horizon_(horizon)
    , segmentDuration_(horizon / numSegments_)
{}

int PiecewiseConstantPerturbation::parameterCount() const {
    return numSegments_;
}

double PiecewiseConstantPerturbation::evaluate(
    double t, const std::vector<double>& theta, const std::vector<double>&) const
{
    if (theta.empty() || t < 0.0 || t > horizon_) return 0.0;
    int idx = static_cast<int>(t / segmentDuration_);
    idx = std::min(idx, numSegments_ - 1);
    if (idx < 0) idx = 0;
    if (idx >= static_cast<int>(theta.size())) return 0.0;
    return theta[idx];
}

// ---------------------------------------------------------------------------
// PiecewiseLinear
// ---------------------------------------------------------------------------

PiecewiseLinearPerturbation::PiecewiseLinearPerturbation(int numBreakpoints, double horizon)
    : numBreakpoints_(std::max(2, numBreakpoints))
    , horizon_(horizon)
{}

int PiecewiseLinearPerturbation::parameterCount() const {
    return numBreakpoints_;
}

double PiecewiseLinearPerturbation::evaluate(
    double t, const std::vector<double>& theta, const std::vector<double>&) const
{
    if (theta.empty() || t < 0.0) return 0.0;
    if (t >= horizon_) return theta.back();

    double spacing = horizon_ / (numBreakpoints_ - 1);
    double idx_f = t / spacing;
    int idx_lo = static_cast<int>(idx_f);
    if (idx_lo >= numBreakpoints_ - 1) return theta[numBreakpoints_ - 1];
    if (idx_lo < 0) idx_lo = 0;

    double frac = idx_f - idx_lo;
    int idx_hi = idx_lo + 1;
    if (idx_hi >= static_cast<int>(theta.size())) return theta[idx_lo];
    return theta[idx_lo] * (1.0 - frac) + theta[idx_hi] * frac;
}

// ---------------------------------------------------------------------------
// Fourier/Spectral
// ---------------------------------------------------------------------------

FourierPerturbation::FourierPerturbation(int numHarmonics, double horizon)
    : numHarmonics_(std::max(1, numHarmonics))
    , horizon_(horizon)
{}

int FourierPerturbation::parameterCount() const {
    return 3 * numHarmonics_;  // (amplitude, frequency, phase) per harmonic
}

double FourierPerturbation::evaluate(
    double t, const std::vector<double>& theta, const std::vector<double>&) const
{
    if (static_cast<int>(theta.size()) < 3 * numHarmonics_) return 0.0;
    double val = 0.0;
    for (int k = 0; k < numHarmonics_; ++k) {
        double amp = theta[3 * k];
        double freq = theta[3 * k + 1];
        double phase = theta[3 * k + 2];
        val += amp * std::sin(2.0 * M_PI * freq * t + phase);
    }
    return val;
}

// ---------------------------------------------------------------------------
// MLP (Neural Network Policy)
// ---------------------------------------------------------------------------

MLPPerturbation::MLPPerturbation(int stateDim, int hiddenSize, int numLayers)
    : stateDim_(std::max(1, stateDim))
    , hiddenSize_(std::max(1, hiddenSize))
    , numLayers_(std::max(1, numLayers))
{
    // Count parameters: input->hidden + (numLayers-1) * hidden->hidden + hidden->output
    // Each layer has weight matrix + bias vector
    paramCount_ = 0;
    // Input layer: stateDim -> hiddenSize
    paramCount_ += (stateDim_ + 1) * hiddenSize_;  // +1 for bias
    // Hidden layers: hiddenSize -> hiddenSize
    for (int i = 1; i < numLayers_; ++i) {
        paramCount_ += (hiddenSize_ + 1) * hiddenSize_;
    }
    // Output layer: hiddenSize -> 1
    paramCount_ += hiddenSize_ + 1;
}

int MLPPerturbation::parameterCount() const {
    return paramCount_;
}

double MLPPerturbation::evaluate(
    double /*t*/, const std::vector<double>& theta,
    const std::vector<double>& state) const
{
    if (static_cast<int>(theta.size()) < paramCount_) return 0.0;
    if (static_cast<int>(state.size()) < stateDim_) return 0.0;

    // Forward pass through MLP
    std::vector<double> activation(state.begin(), state.begin() + stateDim_);
    int offset = 0;

    auto applyLayer = [&](int inputSize, int outputSize) {
        std::vector<double> next(outputSize, 0.0);
        for (int j = 0; j < outputSize; ++j) {
            double sum = 0.0;
            for (int i = 0; i < inputSize; ++i) {
                sum += theta[offset + j * inputSize + i] * activation[i];
            }
            offset += inputSize;
            sum += theta[offset + j]; // bias
            next[j] = std::tanh(sum);  // tanh activation
        }
        offset += outputSize;
        activation = std::move(next);
    };

    // Input layer
    applyLayer(stateDim_, hiddenSize_);

    // Hidden layers
    for (int l = 1; l < numLayers_; ++l) {
        applyLayer(hiddenSize_, hiddenSize_);
    }

    // Output layer (linear, no activation)
    double output = 0.0;
    for (int i = 0; i < hiddenSize_; ++i) {
        output += theta[offset + i] * activation[i];
    }
    offset += hiddenSize_;
    output += theta[offset]; // bias

    return output;
}

// ---------------------------------------------------------------------------
// ImpulseTrain
// ---------------------------------------------------------------------------

ImpulseTrainPerturbation::ImpulseTrainPerturbation(int numImpulses, double horizon)
    : numImpulses_(std::max(1, numImpulses))
    , horizon_(horizon)
{}

int ImpulseTrainPerturbation::parameterCount() const {
    return 3 * numImpulses_;  // (time, amplitude, duration) per impulse
}

double ImpulseTrainPerturbation::evaluate(
    double t, const std::vector<double>& theta, const std::vector<double>&) const
{
    if (static_cast<int>(theta.size()) < 3 * numImpulses_) return 0.0;
    double val = 0.0;
    for (int m = 0; m < numImpulses_; ++m) {
        double tCenter = theta[3 * m] * horizon_;     // normalized to [0, T]
        double amp = theta[3 * m + 1];
        double dur = std::abs(theta[3 * m + 2]) * horizon_ * 0.1; // normalized duration
        if (dur < 1e-10) dur = 1e-10;
        if (t >= tCenter - dur / 2.0 && t <= tCenter + dur / 2.0) {
            val += amp;
        }
    }
    return val;
}

// ---------------------------------------------------------------------------
// BangBang
// ---------------------------------------------------------------------------

BangBangPerturbation::BangBangPerturbation(int numSwitches, double horizon)
    : numSwitches_(std::max(1, numSwitches))
    , horizon_(horizon)
{}

int BangBangPerturbation::parameterCount() const {
    return numSwitches_;  // switching times
}

double BangBangPerturbation::evaluate(
    double t, const std::vector<double>& theta, const std::vector<double>&) const
{
    if (theta.empty() || t < 0.0) return 1.0;  // Start at +1

    // Sort switching times (they should be pre-sorted after projection,
    // but handle unsorted gracefully)
    std::vector<double> switchTimes(theta.begin(),
        theta.begin() + std::min(static_cast<int>(theta.size()), numSwitches_));
    std::sort(switchTimes.begin(), switchTimes.end());

    // Count how many switches have occurred before time t
    int switches = 0;
    for (double st : switchTimes) {
        double actualTime = st * horizon_;  // normalized to [0, T]
        if (t >= actualTime) {
            switches++;
        } else {
            break;
        }
    }

    // Alternate sign: even=+1, odd=-1
    return (switches % 2 == 0) ? 1.0 : -1.0;
}

} // namespace Destabilizer
