#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Chua-circuit chaos benchmark.
     *
     * Models Chua's circuit using three coupled states and a piecewise-linear
     * nonlinear resistor characteristic. The current implementation preserves the
     * double-scroll style regime switching that makes the benchmark difficult.
     *
     * Control is difficult because the nonlinearity changes slope across operating
     * regions, so local linear behavior is only valid in small neighborhoods.
     *
     * The current implementation models the canonical reduced-order Chua circuit. It
     * does not model parasitic capacitance, real op-amp bandwidth limits, switching
     * noise, or detailed circuit implementation nonidealities.
     */
class ChuaCircuit : public ParametricSystem {
public:
    ChuaCircuit();
    const char* name() const override { return "Chua's Circuit"; }
    const char* description() const override { return "Electronic chaos — piecewise-linear nonlinearity"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 59; }
    int stateDim() const override { return 3; }
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
