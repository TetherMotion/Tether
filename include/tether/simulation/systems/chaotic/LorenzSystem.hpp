#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Lorenz-attractor control benchmark.
     *
     * Models the classic three-state Lorenz attractor with one external control
     * input acting as a perturbation channel. The current implementation preserves
     * the convective-style nonlinear coupling and sensitive dependence on initial
     * conditions that define the benchmark.
     *
     * Control is difficult because nearby trajectories separate rapidly and the
     * plant naturally revisits unstable regions, so long-horizon behavior is very
     * sensitive to small errors.
     *
     * The current implementation models the standard three-state Lorenz equations.
     * It does not model sensor noise, actuator saturation, or any spatial fluid
     * dynamics beyond the reduced chaotic ODE benchmark.
     */
class LorenzSystem : public ParametricSystem {
public:
    LorenzSystem();
    const char* name() const override { return "Lorenz System"; }
    const char* description() const override { return "Chaotic attractor — stabilize to unstable fixed point"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 57; }
    int stateDim() const override { return 3; } // [x, y, z]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 3; }
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
