#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Kapitza-pendulum benchmark.
     *
     * Models an inverted pendulum with high-frequency vertical pivot excitation. The
     * current implementation preserves the parametric-excitation mechanism whose
     * averaged effect can stabilize the otherwise unstable upright equilibrium.
     *
     * Control is difficult because stability is created indirectly rather than by a
     * direct restoring torque, so fast excitation and slow pendulum motion must be
     * balanced simultaneously.
     *
     * The current implementation models one pendulum angle and one vibration input.
     * It does not model actuator bandwidth limits, multi-axis pivot motion, rod
     * flexibility, or impact and friction nonlinearities.
     */
class KapitzaPendulum : public ParametricSystem {
public:
    KapitzaPendulum();
    const char* name() const override { return "Kapitza Pendulum"; }
    const char* description() const override { return "Vibrationally stabilized inverted pendulum — parametric excitation"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 61; }
    int stateDim() const override { return 2; } // [theta, dtheta]
    int inputDim() const override { return 1; } // [vibration amplitude]
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
