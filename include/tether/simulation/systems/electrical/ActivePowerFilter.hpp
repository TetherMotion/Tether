#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Active power-filter benchmark.
     *
     * Models an active filter that injects compensating current while regulating a
     * DC-link state. The current implementation keeps one dominant filter-current
     * channel and DC-link energy state so harmonic-compensation control can be
     * studied with a reduced-order plant.
     *
     * Control is difficult because the target signals are periodic, high bandwidth,
     * and often contaminated by switching ripple and measurement noise. The plant
     * must reject harmonics without destabilizing the DC link.
     *
     * The current implementation models one equivalent compensation channel and a
     * lumped DC bus. It does not model individual phases, PWM switching details,
     * grid unbalance, current sensor saturation, or semiconductor constraints.
     */
class ActivePowerFilter : public ParametricSystem {
public:
    ActivePowerFilter();
    const char* name() const override { return "Active Power Filter"; }
    const char* description() const override { return "Tracking sinusoidal references for harmonic compensation"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 48; }
    int stateDim() const override { return 2; } // [i_f, v_dc]
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
