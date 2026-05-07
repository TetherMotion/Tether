#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Planar quadrotor benchmark.
 *
 * Models a rigid-body quadrotor constrained to a vertical plane with horizontal
 * and vertical translation plus vehicle pitch. The current implementation uses
 * two independent rotor thrust inputs so the dominant underactuated coupling
 * between attitude and translation is exposed directly.
 *
 * Control is difficult because lateral motion cannot be commanded directly; the
 * vehicle must tilt to generate horizontal force, which immediately trades off
 * altitude, pitch stability, and actuator headroom.
 *
 * The current implementation models planar rigid-body motion, gravity, and two
 * thrust channels. It does not model rotor aerodynamics, motor lag, wind gusts,
 * blade flapping, yaw dynamics, or full three-dimensional quadrotor behavior.
 */
class PlanarQuadrotor : public ParametricSystem {
public:
    PlanarQuadrotor();
    const char* name() const override { return "2D Quadrotor"; }
    const char* description() const override { return "Planar quadrotor — altitude + pitch, two rotors — MIMO"; }
    SystemCategory category() const override { return SystemCategory::AerospaceVehicle; }
    int systemId() const override { return 28; }
    int stateDim() const override { return 6; } // [x, dx, y, dy, theta, dtheta]
    int inputDim() const override { return 2; }
    int outputDim() const override { return 3; }
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    StateVector defaultInitialState() const override;
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
