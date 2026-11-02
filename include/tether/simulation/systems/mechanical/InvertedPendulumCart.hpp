#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Classic cart-pole benchmark.
     *
     * Models a translating cart and one inverted pendulum link driven by a single
     * horizontal force input. The current implementation keeps the dominant rigid-
     * body cart-pole geometry and open-loop instability.
     *
     * Control is difficult because the upright equilibrium is unstable and the same
     * cart force must both reposition the base and catch the falling pendulum.
     *
     * The current implementation models one rigid pendulum and one rigid cart. It
     * does not model wheel slip, actuator saturation, rail compliance, or frictional
     * contact effects.
     */
class InvertedPendulumCart : public ParametricSystem {
public:
    InvertedPendulumCart();
    const char* name() const override { return "Inverted Pendulum on Cart"; }
    const char* description() const override { return "Classic cart-pole system with linear cart and inverted pendulum"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 3; }
    int stateDim() const override { return 4; } // [x, dx, theta, dtheta]
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
    std::vector<std::string> equationStrings() const override;
};

} // namespace Simulation
