#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single-gimbal control-moment-gyroscope benchmark.
 *
 * Models a spinning rotor that generates body torque through gimbal motion. The
 * current implementation preserves the nonlinear gyroscopic coupling between
 * gimbal angle, stored momentum, and resulting attitude torque.
 *
 * Control is difficult because usable torque depends on state-dependent geometry,
 * and singular configurations sharply reduce controllability.
 *
 * The current implementation models one gimbal and one equivalent body axis. It
 * does not model multi-CMG steering laws, gimbal rate limits, bearing losses, or
 * full spacecraft attitude kinematics.
 */
class ControlMomentGyroscope : public ParametricSystem {
public:
    ControlMomentGyroscope();
    const char* name() const override { return "Control Moment Gyroscope"; }
    const char* description() const override { return "Single-gimbal CMG — nonlinear gyroscopic coupling"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 26; }
    int stateDim() const override { return 4; }
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
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
