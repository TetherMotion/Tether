#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Overdamped holographic optical tweezers benchmark.
     *
     * Models a single trapped microparticle in three dimensions using overdamped
     * viscous motion, anisotropic trap stiffness, cross-axis coupling, scattering
     * bias, and deterministic Brownian-like disturbance injection.
     *
     * Control is difficult because the particle lives in a strongly stochastic,
     * highly dissipative environment where the trap force varies with position. The
     * controller must regulate micron-scale motion while thermal agitation and trap
     * nonlinearity continuously push the particle away from the commanded location.
     *
     * The current implementation models one effective particle and one reduced trap
     * center command in x, y, and z. It does not model full electromagnetic field
     * propagation, multiple holographic traps, fluid heating, near-wall effects, or
     * true stochastic differential-equation noise with random sample paths.
     */
class OpticalTweezersTrap : public ParametricSystem {
public:
    OpticalTweezersTrap();
    const char* name() const override { return "Optical Tweezers Trap"; }
    const char* description() const override { return "Three-axis overdamped particle trapping with nonlinear optical restoring force"; }
    SystemCategory category() const override { return SystemCategory::OpticalPhotonic; }
    int systemId() const override { return 70; }
    int stateDim() const override { return 3; }
    int inputDim() const override { return 3; }
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