#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Multi-zone oven benchmark.
 *
 * Models several thermally coupled heater zones, each with its own temperature
 * state and heater command. The current implementation keeps three dominant zone
 * temperatures and nearest-neighbor coupling to preserve the main MIMO thermal
 * interaction seen in reflow and curing ovens.
 *
 * Control is difficult because each heater influences more than its local zone.
 * Thermal coupling is slow but persistent, so decentralized tuning can distort
 * the temperature profile when aggressive tracking is required.
 *
 * The current implementation models a small set of lumped zones. It does not
 * model product transport through the oven, radiative view factors, airflow
 * distribution, heater element aging, or detailed PCB thermal loading.
 */
class MultiZoneOven : public ParametricSystem {
public:
    MultiZoneOven();
    const char* name() const override { return "Multi-Zone Reflow Oven"; }
    const char* description() const override { return "3+ zones — MIMO, thermal coupling between zones"; }
    SystemCategory category() const override { return SystemCategory::Thermal; }
    int systemId() const override { return 34; }
    int stateDim() const override { return 3; } // [T1, T2, T3]
    int inputDim() const override { return 3; }
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
