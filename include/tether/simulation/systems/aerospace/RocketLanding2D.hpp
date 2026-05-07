#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Planar thrust-vector rocket landing benchmark.
 *
 * Models a landing rocket in a vertical plane with horizontal and vertical
 * motion, pitch rotation, and decreasing fuel mass. The current implementation
 * uses thrust magnitude and gimbal angle as inputs, which captures the core
 * nonlinear coupling of powered descent and terminal landing.
 *
 * Control is difficult because the plant is strongly nonlinear, time varying,
 * and actuator limited. Fuel burn changes inertia and control authority while
 * the controller must reduce descent rate, crossrange error, and attitude error
 * simultaneously near touchdown.
 *
 * The current implementation models planar rigid-body landing with thrust-vector
 * control and fuel depletion. It does not model slosh, engine ignition limits,
 * aero drag variation, leg contact dynamics, ground effect, or full six-DOF
 * booster motion.
 */
class RocketLanding2D : public ParametricSystem {
public:
    RocketLanding2D();
    const char* name() const override { return "2D Rocket Landing"; }
    const char* description() const override { return "Thrust vector controlled rocket — like SpaceX booster landing"; }
    SystemCategory category() const override { return SystemCategory::AerospaceVehicle; }
    int systemId() const override { return 29; }
    int stateDim() const override { return 7; } // [x, dx, y, dy, theta, dtheta, fuel_mass]
    int inputDim() const override { return 2; } // [thrust, gimbal_angle]
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
