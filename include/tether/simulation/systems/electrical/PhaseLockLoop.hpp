#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Nonlinear phase-locked loop benchmark.
     *
     * Models a phase detector, loop integrator, and VCO frequency state driven by a
     * reference phase or frequency input. The current implementation keeps the core
     * nonlinear phase-wrap behavior and loop-filter dynamics in a compact form.
     *
     * Control is difficult because the plant is nonlinear in phase and operates on
     * wrapped angular quantities. Large phase errors and noisy inputs make tuning a
     * tradeoff between fast lock acquisition and low steady-state jitter.
     *
     * The current implementation models one loop filter and one VCO path. It does
     * not model charge-pump nonidealities, divider quantization, reference spurs,
     * RF front-end distortion, or digital clock-domain effects.
     */
class PhaseLockLoop : public ParametricSystem {
public:
    PhaseLockLoop();
    const char* name() const override { return "Phase-Locked Loop"; }
    const char* description() const override { return "Frequency/phase tracking with nonlinear phase detector"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 49; }
    int stateDim() const override { return 3; } // [phase_error, integrator, vco_freq]
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
