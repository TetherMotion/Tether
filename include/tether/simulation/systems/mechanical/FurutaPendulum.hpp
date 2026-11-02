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
