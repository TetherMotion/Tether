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
/**
     * @param t Current simulation time [s].
     * @param state Current state vector, size stateDim().
     * @param input Current input vector, size inputDim().
     * @return State derivative vector, size stateDim().
     */
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
/**
     * @param t Current simulation time [s].
     * @param state Current state vector, size stateDim().
     * @param input Current input vector, size inputDim().
     * @return Output vector, size outputDim().
     */
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /**Start from a physically meaningful rest state.
     * @return state vector, size stateDim().
    */
    StateVector defaultInitialState() const override;
    /**Describe the tunable physical parameters exposed to experiment sweeps.
     * @return descriptor list.
    */
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /**Provide preset parameter sets for common study cases.
     * @return named preset list.
    */
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
