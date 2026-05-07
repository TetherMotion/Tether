#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Triple cart-pole benchmark.
 *
 * Models a cart with three serial inverted links under one translational input.
 * The current implementation preserves the dominant rigid-link coupling and very
 * small stability margin of a long unstable chain.
 *
 * Control is difficult because tiny cart motions redistribute energy across
 * three unstable links, making the plant extremely sensitive to delay and model
 * mismatch.
 *
 * The current implementation models three rigid links and one cart actuator. It
 * does not model joint backlash, structural flex, rail limits, or unmodeled
 * damping mechanisms.
 */
class TripleInvertedPendulumCart : public ParametricSystem {
public:
    TripleInvertedPendulumCart();
    const char* name() const override { return "Triple Inverted Pendulum"; }
    const char* description() const override { return "Cart with three inverted pendulum links — near edge of controllability"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 5; }
    int stateDim() const override { return 8; } // [x, dx, theta1..3, dtheta1..3]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 4; }
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
