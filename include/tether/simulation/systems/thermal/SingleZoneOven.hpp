#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single-zone oven benchmark.
 *
 * Models one lumped thermal chamber with heater input and heat loss to ambient.
 * The current implementation uses a single dominant temperature state so basic
 * thermal setpoint regulation and disturbance rejection can be studied directly.
 *
 * Control is difficult because thermal plants are slow and integrate energy over
 * long time horizons. Large dead-time-like transients make aggressive tuning
 * prone to overshoot and conservative tuning overly sluggish.
 *
 * The current implementation models one equivalent chamber temperature. It does
 * not model spatial gradients, heater saturation dynamics, fan recirculation,
 * radiation exchange detail, or door-opening disturbances.
 */
class SingleZoneOven : public ParametricSystem {
public:
    SingleZoneOven();
    const char* name() const override { return "Single-Zone Oven"; }
    const char* description() const override { return "Heating control with variable thermal resistance to outside"; }
    SystemCategory category() const override { return SystemCategory::Thermal; }
    int systemId() const override { return 33; }
    int stateDim() const override { return 1; } // [T]
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
