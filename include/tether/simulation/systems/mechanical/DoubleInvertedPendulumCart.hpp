#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Double cart-pole benchmark.
 *
 * Models a cart carrying two serial inverted links driven by one cart force. The
 * current implementation preserves the two-link unstable chain and its dominant
 * coupled modes.
 *
 * Control is difficult because the controller must stabilize two coupled links
 * with different time scales, and suppressing one link can inject energy into
 * the other.
 *
 * The current implementation models rigid links and a rigid cart. It does not
 * model joint friction, structural flex, actuator limits, or impact with travel
 * stops.
 */
class DoubleInvertedPendulumCart : public ParametricSystem {
public:
    DoubleInvertedPendulumCart();
    const char* name() const override { return "Double Inverted Pendulum"; }
    const char* description() const override { return "Cart with two inverted pendulum links"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 4; }
    int stateDim() const override { return 6; } // [x, dx, theta1, dtheta1, theta2, dtheta2]
    int inputDim() const override { return 1; }
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
