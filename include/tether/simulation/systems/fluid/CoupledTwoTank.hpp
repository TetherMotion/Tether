#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Coupled two-tank fluid benchmark.
     *
     * Models two hydraulically coupled tanks with two inflows and level interaction
     * through gravity-driven exchange. The current implementation preserves the main
     * nonlinear coupling while reducing the process to two lumped liquid levels.
     *
     * Control is difficult because actuation into one tank affects the other after
     * internal redistribution, and the cross coupling varies with operating point.
     *
     * The current implementation models two lumped chambers and gravity exchange. It
     * does not model pipe transport delay, valve nonlinearities, overflow logic, or
     * distributed pressure waves.
     */
class CoupledTwoTank : public ParametricSystem {
public:
    CoupledTwoTank();
    const char* name() const override { return "Coupled Two-Tank"; }
    const char* description() const override { return "Gravity-fed coupled tanks — nonlinear sqrt flow — MIMO"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 39; }
    int stateDim() const override { return 2; }
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
