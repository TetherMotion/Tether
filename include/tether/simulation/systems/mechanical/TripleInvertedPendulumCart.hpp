#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Triple cart-pole benchmark.
     *
     * Models a cart with three serial inverted links under one translational input.
     * The current implementation preserves the dominant rigid-link coupling and very
     * small stability margin of a long unstable chain.
     *
     * Control is difficult because tiny cart motions redistribute energy across
     * three unstable links, making the plant extremely sensitive to delay and model
     * mismatch.
     *
     * The current implementation models three rigid links and one cart actuator. It
     * does not model joint backlash, structural flex, rail limits, or unmodeled
     * damping mechanisms.
     */
class TripleInvertedPendulumCart : public ParametricSystem {
public:
    TripleInvertedPendulumCart();
    const char* name() const override { return "Triple Inverted Pendulum"; }
    const char* description() const override { return "Cart with three inverted pendulum links — near edge of controllability"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 5; }
    int stateDim() const override { return 8; } // [x, dx, theta1..3, dtheta1..3]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 4; }
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
