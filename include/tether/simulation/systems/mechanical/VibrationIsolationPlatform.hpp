#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Precision vibration-isolation platform benchmark.
 *
 * Models a platform with translational and rotational rigid-body motion supported
 * by multiple actuators. The current implementation preserves the main geometric
 * coupling between translation and rotation in a reduced 2D isolation stage.
 *
 * Control is difficult because disturbance rejection in one direction can inject
 * moment into another mode, and high isolation performance operates near lightly
 * damped resonance.
 *
 * The current implementation models one reduced planar platform with three input
 * channels. It does not model full six-DOF structural compliance, sensor fusion,
 * floor anisotropy, or high-frequency flexible modes of the stage.
 */
class VibrationIsolationPlatform : public ParametricSystem {
public:
    VibrationIsolationPlatform();
    const char* name() const override { return "Vibration Isolation Platform"; }
    const char* description() const override { return "2D platform with three actuators — MIMO vibration isolation"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 19; }
    int stateDim() const override { return 6; } // [x, dx, y, dy, theta, dtheta]
    int inputDim() const override { return 3; }
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
