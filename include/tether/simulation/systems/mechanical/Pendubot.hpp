#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Pendubot underactuated-manipulator benchmark.
 *
 * Models a two-link planar robot actuated only at the shoulder joint. The
 * current implementation preserves the underactuated rigid-link coupling needed
 * for energy-shaping and swing-up studies.
 *
 * Control is difficult because one joint is passive, so missing actuation must
 * be recovered through momentum transfer and dynamic coupling.
 *
 * The current implementation models two rigid links and one actuator. It does
 * not model motor dynamics, joint backlash, elastic links, or contact with any
 * environment.
 */
class Pendubot : public ParametricSystem {
public:
    Pendubot();
    const char* name() const override { return "Pendubot"; }
    const char* description() const override { return "Two-link pendulum actuated only at shoulder joint"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 6; }
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
