#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Counter-flow heat-exchanger benchmark.
     *
     * Models hot-side and cold-side outlet temperatures together with wall thermal
     * storage in a reduced counter-flow exchanger. The current implementation keeps
     * the dominant transport lag and wall inertia that shape outlet response.
     *
     * Control is difficult because input changes propagate through both transport
     * delay and thermal inertia. Temperature response can be slow and interacting,
     * so disturbance rejection easily trades against long settling tails.
     *
     * The current implementation models a small set of lumped thermal states. It
     * does not model full distributed PDE transport, fouling, phase change, valve
     * stiction, or separate dynamics for both manipulated flow streams.
     */
class HeatExchanger : public ParametricSystem {
public:
    HeatExchanger();
    const char* name() const override { return "Heat Exchanger"; }
    const char* description() const override { return "Two-fluid counter-flow heat exchanger with transport delay"; }
    SystemCategory category() const override { return SystemCategory::Thermal; }
    int systemId() const override { return 35; }
    int stateDim() const override { return 4; } // [T_h_out, T_c_out, T_wall1, T_wall2]
    int inputDim() const override { return 1; } // [flow_rate_hot]
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
