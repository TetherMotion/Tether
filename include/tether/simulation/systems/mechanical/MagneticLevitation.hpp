#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single-axis magnetic-levitation benchmark.
 *
 * Models one electromagnet suspending one mass with position, velocity, and coil
 * current as states. The current implementation preserves nonlinear gap-dependent
 * magnetic force and fast electrical dynamics.
 *
 * Control is difficult because the equilibrium is open-loop unstable and the
 * magnetic force changes sharply as the air gap closes.
 *
 * The current implementation models one levitated body and one coil. It does not
 * model eddy currents, sensor quantization, lateral motion, magnetic saturation,
 * or detailed power electronics.
 */
class MagneticLevitation : public ParametricSystem {
public:
    MagneticLevitation();
    const char* name() const override { return "Magnetic Levitation"; }
    const char* description() const override { return "Single electromagnet suspending a steel ball — nonlinear"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 15; }
    int stateDim() const override { return 3; } // [y, dy, i]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
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
