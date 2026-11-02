#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Half-car suspension benchmark.
     *
     * Models front and rear suspension dynamics including body heave and pitch under
     * two actuation channels. The current implementation preserves the dominant front
     * and rear coupling needed for MIMO ride-control studies.
     *
     * Control is difficult because front and rear actuators interact through pitch,
     * so improving one axle can worsen the other or excite body rotation.
     *
     * The current implementation models vertical and pitch motion only. It does not
     * model full roll dynamics, nonlinear dampers, steering effects, or detailed
     * tire-force variation beyond the reduced suspension states.
     */
class HalfCarSuspension : public ParametricSystem {
public:
    HalfCarSuspension();
    const char* name() const override { return "Half-Car Suspension"; }
    const char* description() const override { return "Front and rear suspension — MIMO: pitch + heave control"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 18; }
    int stateDim() const override { return 8; }
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
