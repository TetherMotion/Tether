#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Two-mass coupled resonance benchmark.
     *
     * Models two translational masses connected by springs and dampers with the
     * actuator applied on one side. The current implementation preserves two lumped
     * masses and their dominant coupled resonant modes.
     *
     * Control is difficult because suppressing one mode can excite another, and the
     * local actuator does not move both masses identically.
     *
     * The current implementation models two rigid masses with linear elastic and
     * viscous couplings. It does not model backlash, friction, actuator compliance,
     * or higher-order distributed structural modes.
     */
class CoupledMassSpringDamper : public ParametricSystem {
public:
    CoupledMassSpringDamper();
    const char* name() const override { return "Coupled Mass-Spring-Damper"; }
    const char* description() const override { return "Two masses coupled by springs and dampers in series"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 2; }
    int stateDim() const override { return 4; } // [x1, dx1, x2, dx2]
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
    std::vector<std::string> equationStrings() const override;
};

} // namespace Simulation
