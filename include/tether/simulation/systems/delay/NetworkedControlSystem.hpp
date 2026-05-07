#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Networked-control delay benchmark.
 *
 * Models a low-order plant whose sensing and actuation are mediated by a
 * communication network with effective round-trip latency. The current
 * implementation includes plant dynamics together with a reduced timing state
 * that emulates variable network delay and timing uncertainty.
 *
 * Control is difficult because latency is not fixed and packets do not arrive
 * with perfectly uniform timing. That uncertainty erodes phase margin and can
 * destabilize controllers that are otherwise well behaved under deterministic
 * delay.
 *
 * The current implementation models lumped timing variability and one dominant
 * plant channel. It does not model packet drops, retransmission logic, sampled
 * quantization, protocol scheduling, or multi-node shared-network contention.
 */
class NetworkedControlSystem : public ParametricSystem {
public:
    NetworkedControlSystem();
    const char* name() const override { return "Networked Control System"; }
    const char* description() const override { return "Variable, stochastic round-trip delay emulation"; }
    SystemCategory category() const override { return SystemCategory::DelayDominated; }
    int systemId() const override { return 64; }
    int stateDim() const override { return 2; }
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
