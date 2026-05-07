#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Acrobot underactuated-manipulator benchmark.
 *
 * Models a two-link planar robot actuated only at the elbow with a passive base
 * joint. The current implementation preserves the rigid-link geometry and elbow-
 * driven energy transfer that distinguishes the Acrobot from the Pendubot.
 *
 * Control is difficult because the actuator cannot directly control the global
 * orientation of the chain, so swing-up relies on nonlinear internal coupling.
 *
 * The current implementation models two rigid links and one elbow actuator. It
 * does not model motor limits, friction, flexible links, or collisions.
 */
class Acrobot : public ParametricSystem {
public:
    Acrobot();
    const char* name() const override { return "Acrobot"; }
    const char* description() const override { return "Two-link pendulum actuated only at elbow joint"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 7; }
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
