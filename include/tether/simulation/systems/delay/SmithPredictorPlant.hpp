#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Delay-dominated Smith-predictor benchmark plant.
 *
 * Models a stable low-order process with a large transport delay between the
 * applied input and measured output. The current implementation keeps the core
 * dead-time difficulty visible while reducing the delay-free dynamics to a small
 * state-space plant suitable for compensation experiments.
 *
 * Control is difficult because the output remains unchanged for a long time
 * after the input is applied. That dead time hides whether the most recent
 * action was correct, so naive feedback tends to overreact and lose phase
 * margin.
 *
 * The current implementation models one dominant process state and an effective
 * pure-delay channel. It does not model distributed transport, actuator slew
 * limits, disturbance-model mismatch, or network-style packet jitter.
 */
class SmithPredictorPlant : public ParametricSystem {
public:
    SmithPredictorPlant();
    const char* name() const override { return "Smith Predictor Plant"; }
    const char* description() const override { return "Stable first-order + long pure delay — Smith predictor demo"; }
    SystemCategory category() const override { return SystemCategory::DelayDominated; }
    int systemId() const override { return 63; }
    int stateDim() const override { return 1; }
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
