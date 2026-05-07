#pragma once
/// @file Perturbation.hpp
/// @brief Perturbation signal parameterizations for the Destabilizer.

#include "DestabilizerTypes.hpp"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <random>

namespace Destabilizer {

/// Abstract base for perturbation signal generators.
/// Maps a parameter vector θ to a time-domain signal u_p(t).
class PerturbationSignal {
public:
    virtual ~PerturbationSignal() = default;

    /// Number of free parameters in θ.
    virtual int parameterCount() const = 0;

    /// Evaluate the perturbation signal at time t given parameters θ.
    /// For state-dependent parameterizations (e.g., MLP), state is provided.
    virtual double evaluate(double t, const std::vector<double>& theta,
                            const std::vector<double>& state = {}) const = 0;

    /// Generate the full signal on a time grid.
    std::vector<double> generateSignal(const std::vector<double>& times,
                                        const std::vector<double>& theta,
                                        const std::vector<std::vector<double>>& states = {}) const;

    /// Create a perturbation signal of the given type.
    static std::unique_ptr<PerturbationSignal> create(PerturbationType type,
                                                       const PerturbationConfig& config,
                                                       double horizon);
};

/// Piecewise-constant: N segments of constant amplitude over [0, T].
class PiecewiseConstantPerturbation : public PerturbationSignal {
public:
    PiecewiseConstantPerturbation(int numSegments, double horizon);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int numSegments_;
    double horizon_;
    double segmentDuration_;
};

/// Piecewise-linear: N breakpoints with linear interpolation.
class PiecewiseLinearPerturbation : public PerturbationSignal {
public:
    PiecewiseLinearPerturbation(int numBreakpoints, double horizon);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int numBreakpoints_;
    double horizon_;
};

/// Fourier/spectral: sum of K sinusoids with amplitude, frequency, phase.
/// θ = [a1, f1, φ1, a2, f2, φ2, ...]
class FourierPerturbation : public PerturbationSignal {
public:
    FourierPerturbation(int numHarmonics, double horizon);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int numHarmonics_;
    double horizon_;
};

/// MLP policy: small neural network mapping state → perturbation.
/// θ = flattened network weights and biases.
class MLPPerturbation : public PerturbationSignal {
public:
    MLPPerturbation(int stateDim, int hiddenSize, int numLayers);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int stateDim_;
    int hiddenSize_;
    int numLayers_;
    int paramCount_;
};

/// Impulse train: M impulses with (time, amplitude, duration).
/// θ = [t1, a1, d1, t2, a2, d2, ...]
class ImpulseTrainPerturbation : public PerturbationSignal {
public:
    ImpulseTrainPerturbation(int numImpulses, double horizon);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int numImpulses_;
    double horizon_;
};

/// Bang-bang: switching times between +A_max and -A_max.
/// θ = N switching times (sorted); amplitude alternates between +1 and -1.
/// Actual amplitude is scaled by ChannelConstraints.amplitudeMax at projection time.
class BangBangPerturbation : public PerturbationSignal {
public:
    BangBangPerturbation(int numSwitches, double horizon);
    int parameterCount() const override;
    double evaluate(double t, const std::vector<double>& theta,
                    const std::vector<double>& state = {}) const override;
private:
    int numSwitches_;
    double horizon_;
};

} // namespace Destabilizer
