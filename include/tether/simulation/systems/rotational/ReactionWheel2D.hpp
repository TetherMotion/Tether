#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Two-axis reaction-wheel benchmark.
 *
 * Models two-axis attitude regulation using two reaction wheels on a shared rigid
 * body. The current implementation keeps the coupled body-angle and wheel-speed
 * states so small-MIMO momentum-exchange control remains visible.
 *
 * Control is difficult because the actuation axes interact through the shared
 * body and finite wheel authority, so axis decoupling is only approximate.
 *
 * The current implementation models two dominant rigid-body axes and two wheels.
 * It does not model full three-axis coupling, saturation management logic,
 * flexible appendages, or realistic star-tracker and gyro measurements.
 */
class ReactionWheel2D : public ParametricSystem {
public:
    ReactionWheel2D();
    const char* name() const override { return "Reaction Wheel (2-Axis)"; }
    const char* description() const override { return "Two-axis attitude control via two reaction wheels — MIMO"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 25; }
    int stateDim() const override { return 6; }
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
