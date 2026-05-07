#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Fed-batch bioreactor benchmark.
 *
 * Models biomass, substrate, and reactor volume in a fed-batch fermentation
 * process with Monod-style growth and dilution from feed addition. The current
 * implementation uses feed rate as the single manipulated input so productivity,
 * substrate limitation, and dilution remain tightly coupled.
 *
 * Control is difficult because the process is slow, nonlinear, and strongly
 * state dependent. The same feed action that stimulates growth can also dilute
 * the reactor and alter future dynamic response.
 *
 * The current implementation models one dominant microbial population, one
 * limiting substrate, and simple volume growth. It does not model oxygen
 * transfer, temperature control, pH control, product inhibition, contamination,
 * or multi-phase broth rheology.
 */
class Bioreactor : public ParametricSystem {
public:
    Bioreactor();
    const char* name() const override { return "Bioreactor"; }
    const char* description() const override { return "Fed-batch fermentation — nonlinear Monod kinetics, slow dynamics"; }
    SystemCategory category() const override { return SystemCategory::ChemicalProcess; }
    int systemId() const override { return 53; }
    int stateDim() const override { return 3; } // [X, S, V] (biomass, substrate, volume)
    int inputDim() const override { return 1; } // [F] (feed rate)
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
