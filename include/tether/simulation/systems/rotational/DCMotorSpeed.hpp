#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief DC-motor speed-control benchmark.
     *
     * Models rotor speed and armature current for a voltage-driven DC motor. The
     * current implementation keeps the coupled electrical and mechanical energy
     * storage that dominates speed-loop design.
     *
     * Control is difficult because current evolves much faster than speed, so the
     * plant contains separated time scales and easily saturates under fast commands.
     *
     * The current implementation models one rotor inertia, viscous friction, and a
     * simple armature circuit. It does not model commutation ripple, backlash,
     * current limits, temperature dependence, or load-side elasticity.
     */
class DCMotorSpeed : public ParametricSystem {
public:
    DCMotorSpeed();
    const char* name() const override { return "DC Motor Speed Control"; }
    const char* description() const override { return "DC motor with electrical dynamics and back-EMF — speed regulation"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 20; }
    int stateDim() const override { return 2; } // [omega, i]
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
