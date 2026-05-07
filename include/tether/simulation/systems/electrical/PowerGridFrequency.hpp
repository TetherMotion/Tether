#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single-area power-grid frequency benchmark.
 *
 * Models a generator-load area with frequency deviation, turbine power, and
 * valve/governor dynamics. The current implementation uses one equivalent area
 * so secondary regulation action can be studied without a full network model.
 *
 * Control is difficult because frequency deviation integrates system imbalance
 * and actuation is filtered by governor and turbine dynamics. Disturbances can
 * be persistent, so disturbance rejection and oscillatory behavior compete.
 *
 * The current implementation models one equivalent machine and one governor
 * chain. It does not model multiple areas, transmission constraints, renewable
 * intermittency, protection actions, or market-dispatch coordination.
 */
class PowerGridFrequency : public ParametricSystem {
public:
    PowerGridFrequency();
    const char* name() const override { return "Power Grid Frequency"; }
    const char* description() const override { return "Single generator + load — governor droop characteristic"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 47; }
    int stateDim() const override { return 3; } // [delta_f, P_m, P_valve]
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
