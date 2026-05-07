#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Quadruple-tank MIMO benchmark.
 *
 * Models the Johansson four-tank process with four interacting water levels and
 * two pump inputs. The current implementation keeps the configurable routing
 * that yields either minimum-phase or non-minimum-phase multivariable behavior.
 *
 * Control is difficult because the input-output pairing can change character
 * with operating configuration, and upper-tank routing introduces slow inverse
 * response and strong interaction.
 *
 * The current implementation models four lumped tank levels and two pumps. It
 * does not model pump saturation curves, pipe-network delay, overflow, or more
 * detailed hydraulic losses beyond the benchmark formulation.
 */
class FourTankSystem : public ParametricSystem {
public:
    FourTankSystem();
    const char* name() const override { return "Four-Tank System"; }
    const char* description() const override { return "Johansson's quadruple-tank — MIMO, configurable min/non-min phase"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 40; }
    int stateDim() const override { return 4; }
    int inputDim() const override { return 2; }
    int outputDim() const override { return 2; }
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
