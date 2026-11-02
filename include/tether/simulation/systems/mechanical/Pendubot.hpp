#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Pendubot underactuated-manipulator benchmark.
     *
     * Models a two-link planar robot actuated only at the shoulder joint. The
     * current implementation preserves the underactuated rigid-link coupling needed
     * for energy-shaping and swing-up studies.
     *
     * Control is difficult because one joint is passive, so missing actuation must
     * be recovered through momentum transfer and dynamic coupling.
     *
     * The current implementation models two rigid links and one actuator. It does
     * not model motor dynamics, joint backlash, elastic links, or contact with any
     * environment.
     */
class Pendubot : public ParametricSystem {
public:
    Pendubot();
    const char* name() const override { return "Pendubot"; }
    const char* description() const override { return "Two-link pendulum actuated only at shoulder joint"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 6; }
    int stateDim() const override { return 4; }
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
};

} // namespace Simulation
