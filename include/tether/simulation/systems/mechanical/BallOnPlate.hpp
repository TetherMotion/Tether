#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Ball-on-plate benchmark.
 *
 * Models a ball rolling on a plate tilted about two orthogonal axes. The current
 * implementation preserves two-axis rolling motion and the coupled tilt channels
 * that make the plant non-collocated and multivariable.
 *
 * Control is difficult because the plant is both non-collocated and multivariable,
 * so tilt in one axis perturbs motion in the other.
 *
 * The current implementation models one rolling ball and two rigid plate tilt
 * axes. It does not model contact slip, plate flex, actuator saturation, or
 * camera-based sensing delay.
 */
class BallOnPlate : public ParametricSystem {
public:
    BallOnPlate();
    const char* name() const override { return "Ball on Plate"; }
    const char* description() const override { return "Ball rolling on a tilting plate — 2D MIMO balance"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 10; }
    int stateDim() const override { return 8; } // [rx, drx, ry, dry, thetax, dthetax, thetay, dthetay]
    int inputDim() const override { return 2; }
    int outputDim() const override { return 2; }
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
