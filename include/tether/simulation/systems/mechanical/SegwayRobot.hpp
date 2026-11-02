#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Self-balancing two-wheel robot benchmark.
     *
     * Models a Segway-style body with translational motion and body pitch under one
     * effective drive input. The current implementation preserves the inverted-body
     * coupling between motion and balance.
     *
     * Control is difficult because it is an inverted-balance problem embedded in a
     * mobile platform, so travel commands and stabilization compete for authority.
     *
     * The current implementation models planar body motion and one effective drive.
     * It does not model motor electrical dynamics, wheel slip, steering yaw motion,
     * rider compliance, or terrain irregularities.
     */
class SegwayRobot : public ParametricSystem {
public:
    SegwayRobot();
    const char* name() const override { return "Segway Robot"; }
    const char* description() const override { return "Self-balancing two-wheel robot — 2D side view"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 12; }
    int stateDim() const override { return 4; } // [x, dx, theta, dtheta]
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
