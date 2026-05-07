#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Planar hovercraft benchmark.
 *
 * Models a hovercraft in the horizontal plane with x and y translation, heading,
 * and their corresponding rates. The current implementation uses net thrust and
 * yaw torque as the two control inputs so the underactuated body-frame coupling
 * remains explicit.
 *
 * Control is difficult because the craft cannot command independent forces in
 * all planar directions. Heading must be changed before thrust aligns with the
 * desired path, and the low-friction environment makes drift and overshoot easy.
 *
 * The current implementation models planar rigid-body motion with minimal drag.
 * It does not model skirt dynamics, wave interaction, actuator saturation maps,
 * ground-effect airflow, or three-dimensional hovercraft motion.
 */
class Hovercraft2D : public ParametricSystem {
public:
    Hovercraft2D();
    const char* name() const override { return "Hovercraft"; }
    const char* description() const override { return "2D top-down — MIMO, underactuated, friction-free hovering"; }
    SystemCategory category() const override { return SystemCategory::AerospaceVehicle; }
    int systemId() const override { return 32; }
    int stateDim() const override { return 6; } // [x, dx, y, dy, theta, dtheta]
    int inputDim() const override { return 2; } // [thrust, torque]
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
