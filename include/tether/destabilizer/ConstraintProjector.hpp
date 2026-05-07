#pragma once
/// @file ConstraintProjector.hpp
/// @brief Hard constraint projection for the Destabilizer.
///
/// After each optimizer step, θ is projected back into the feasible set.

#include "DestabilizerTypes.hpp"
#include "Perturbation.hpp"
#include <vector>

namespace Destabilizer {

class ConstraintProjector {
public:
    ConstraintProjector() = default;

    /// Configure projector for a given channel and pert
/// Projects a perturbation parameter vector θ into the feasible set defined
/// by the channel constraints. Returns a boolean vector indicating which
/// constraints were active (binding) during projection.urbation type.
    void configure(const ChannelConstraints& constraints,
                   const PerturbationConfig& pertConfig,
                   double horizon, double dt);

    /// Project θ in-place. Returns per-constraint activity flags.
    /// Constraint order: [amplitude, rate, energy, frequency, dutyCycle]
    std::vector<bool> project(std::vector<double>& theta) const;

    /// Project amplitude constraint: clamp each parameter to [-A_max, A_max].
    static void projectAmplitude(std::vector<double>& theta, double aMax);

    /// Project rate constraint on a piecewise signal.
    static void projectRate(std::vector<double>& theta, double rateMax,
                            double segmentDuration);

    /// Project energy constraint: scale θ so ∫ u_p²dt ≤ E_max.
    static void projectEnergy(std::vector<double>& theta, double eMax,
                              double segmentDuration);

    /// Project frequency constraint on Fourier parameterization.
    static void projectFrequency(std::vector<double>& theta, double fMin,
                                  double fMax);

    /// Project duty cycle constraint.
    static void projectDutyCycle(std::vector<double>& theta, double dMax,
                                  double horizon);

    /// Check if amplitude constraint is active (any param at boundary).
    static bool isAmplitudeActive(const std::vector<double>& theta, double aMax,
                                   double tol = 1e-6);

    /// Idempotence: project(project(θ)) == project(θ).
    bool verifyIdempotence(std::vector<double> theta, double tol = 1e-10) const;

private:
    ChannelConstraints constraints_;
    PerturbationConfig pertConfig_;
    double horizon_ = 5.0;
    double dt_ = 0.001;
};

} // namespace Destabilizer
