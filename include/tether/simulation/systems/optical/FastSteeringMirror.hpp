#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Two-axis fast steering mirror model for free-space optics.
     *
     * Models a tip-tilt mirror with two lightly damped structural axes, actuator
     * hysteresis-like internal states, cross-axis flexure coupling, and broadband
     * line-of-sight jitter entering as platform disturbance.
     *
     * Control is difficult because the mirror must reject high-frequency jitter
     * without exciting resonant flexure modes whose frequency and damping margins
     * are narrow. Any controller that ignores hysteresis or axis coupling tends to
     * trade pointing accuracy for excessive resonance amplification.
     *
     * The current implementation models dominant tip and tilt modes plus a simple
     * internal hysteresis surrogate and deterministic jitter spectrum. It does not
     * model detailed piezo Preisach hysteresis, full mirror surface deformation,
     * atmospheric scintillation, or moving-platform line-of-sight geometry.
     */
class FastSteeringMirror : public ParametricSystem {
public:
    FastSteeringMirror();
    const char* name() const override { return "Fast Steering Mirror"; }
    const char* description() const override { return "Two-axis jitter-rejection mirror with flexure resonance and actuator hysteresis"; }
    SystemCategory category() const override { return SystemCategory::OpticalPhotonic; }
    int systemId() const override { return 67; }
    int stateDim() const override { return 6; }
    int inputDim() const override { return 2; }
    int outputDim() const override { return 2; }
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