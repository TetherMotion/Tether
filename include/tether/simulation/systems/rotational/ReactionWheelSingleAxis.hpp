#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single-axis reaction-wheel benchmark.
 *
 * Models body angle, body rate, wheel speed, and drive current for a single-axis
 * reaction-wheel attitude-control system. The current implementation preserves
 * the internal momentum-exchange nature of the actuator.
 *
 * Control is difficult because total angular momentum is internally redistributed
 * rather than removed, so wheel saturation can accumulate under persistent bias.
 *
 * The current implementation models one rigid body axis and one wheel. It does
 * not model three-axis spacecraft coupling, desaturation thrusters, frictional
 * wheel imbalance, or sensor fusion in the attitude estimator.
 */
class ReactionWheelSingleAxis : public ParametricSystem {
public:
    ReactionWheelSingleAxis();
    const char* name() const override { return "Reaction Wheel (1-Axis)"; }
    const char* description() const override { return "Single-axis attitude control via reaction wheel"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 24; }
    int stateDim() const override { return 4; } // [theta, omega, omega_w, i]
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
