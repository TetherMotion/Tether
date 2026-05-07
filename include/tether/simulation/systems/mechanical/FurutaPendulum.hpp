#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Furuta-pendulum benchmark.
 *
 * Models a rotary arm driven in the horizontal plane with a pendulum free to
 * swing in the vertical plane. The current implementation preserves the main
 * inertial coupling between the actuated arm and passive pendulum.
 *
 * Control is difficult because the pendulum cannot be actuated directly and the
 * controller must coordinate arm motion to swing up and stabilize it.
 *
 * The current implementation models one rigid arm and one rigid pendulum. It
 * does not model motor current dynamics, bearing friction, arm flex, or encoder
 * quantization.
 */
class FurutaPendulum : public ParametricSystem {
public:
    FurutaPendulum();
    const char* name() const override { return "Furuta Pendulum"; }
    const char* description() const override { return "Rotary inverted pendulum — motor drives horizontal arm, vertical pendulum"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 8; }
    int stateDim() const override { return 4; }
    int inputDim() const override { return 1; }
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
