#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Double-pendulum gantry-crane benchmark.
 *
 * Models a trolley carrying a two-link suspended load, for example hook plus
 * payload articulation. The current implementation preserves the two dominant
 * swing modes and their underactuated coupling.
 *
 * Control is difficult because suppressing the dominant sway can leave residual
 * motion in the secondary link, so transport and damping objectives interact.
 *
 * The current implementation models one trolley axis and two rigid pendulum
 * links. It does not model cable stretch, hoist motion, 3D sway, or structural
 * flexibility of the load.
 */
class DoublePendulumGantryCrane : public ParametricSystem {
public:
    DoublePendulumGantryCrane();
    const char* name() const override { return "Double-Pendulum Gantry Crane"; }
    const char* description() const override { return "Trolley with two-segment cable/load"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 14; }
    int stateDim() const override { return 6; }
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
