#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Reduced-order buck-boost converter benchmark.
     *
     * Models a bidirectional buck-boost stage with coupled current and voltage
     * dynamics and two control channels. The current implementation captures the
     * dominant multivariable interaction across step-up and step-down regimes using
     * an averaged state-space model.
     *
     * Control is difficult because the plant changes character with operating mode,
     * and current and voltage loops cannot be tuned independently. Cross coupling
     * and duty-ratio limits make robust multivariable control substantially harder.
     *
     * The current implementation models lumped current and voltage states with two
     * control inputs. It does not model switching harmonics, hard mode boundaries,
     * device protection logic, or parasitic network detail of real hardware.
     */
class BuckBoostConverter : public ParametricSystem {
public:
    BuckBoostConverter();
    const char* name() const override { return "Buck-Boost Converter"; }
    const char* description() const override { return "Bidirectional, nonlinear — MIMO with current and voltage loops"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 46; }
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
    StateVector defaultInput() const override;
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
