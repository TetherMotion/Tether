#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Triple-link underactuated gymnast benchmark.
     *
     * Models a three-link acrobatic chain driven by a single input, representing bar
     * gymnastics or aggressive multi-link swing-up. The current implementation keeps
     * all three link angles and rates so internal energy transfer dominates motion.
     *
     * Control is difficult because there are more generalized coordinates than
     * actuators, with strong nonlinear coupling and large energy swings.
     *
     * The current implementation models three rigid links and one actuator. It does
     * not model bar compliance, grip changes, contact events, muscle-like actuation,
     * or full humanoid whole-body dynamics.
     */
class TripleLinkGymnast : public ParametricSystem {
public:
    TripleLinkGymnast();
    const char* name() const override { return "Triple-Link Gymnast"; }
    const char* description() const override { return "Three rigid links, single actuator — massively underactuated swing-up"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 62; }
    int stateDim() const override { return 6; } // [theta1..3, dtheta1..3]
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
