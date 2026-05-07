#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Simplified two-product distillation column benchmark.
 *
 * Models interacting composition dynamics for a reduced-order distillation
 * column driven by reflux and boilup. The current implementation preserves the
 * strong MIMO interaction between top and bottom quality variables without
 * resolving every tray explicitly.
 *
 * Control is difficult because the manipulated variables affect both outputs in
 * competing directions and with different delays. High interaction and poor
 * input pairing make naive decentralized control fragile.
 *
 * The current implementation models a compact set of dominant composition states
 * and two manipulated flows. It does not model full tray-by-tray hydraulics,
 * pressure dynamics, composition measurement delay, condenser/reboiler utility
 * limits, or startup and flooding behavior.
 */
class DistillationColumn : public ParametricSystem {
public:
    DistillationColumn();
    const char* name() const override { return "Distillation Column"; }
    const char* description() const override { return "Simplified two-product — MIMO, high interaction, ill-conditioned"; }
    SystemCategory category() const override { return SystemCategory::ChemicalProcess; }
    int systemId() const override { return 52; }
    int stateDim() const override { return 4; } // simplified tray compositions
    int inputDim() const override { return 2; } // [L, V] (reflux, boilup)
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
