#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Rössler-attractor control benchmark.
 *
 * Models the three-state Rössler chaotic oscillator with a single perturbation
 * input. The current implementation preserves the spiral attractor and nonlinear
 * reinjection mechanism that make long-horizon trajectory shaping difficult.
 *
 * Control is difficult because the plant combines smooth local motion with a
 * global nonlinear fold, so behavior that looks benign over short windows can
 * diverge qualitatively over longer horizons.
 *
 * The current implementation models the standard three-state Rössler system. It
 * does not model measurement noise, stochastic forcing, or any higher-order
 * physical realization beyond the benchmark ODE.
 */
class RosslerSystem : public ParametricSystem {
public:
    RosslerSystem();
    const char* name() const override { return "Rössler System"; }
    const char* description() const override { return "Chaotic system with spiral attractor — control challenge"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 58; }
    int stateDim() const override { return 3; }
    int inputDim() const override { return 1; }
    int outputDim() const override { return 3; }
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    StateVector defaultInitialState() const override;
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
