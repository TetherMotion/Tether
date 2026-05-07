#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Pressure-vessel regulation benchmark.
 *
 * Models pressure buildup in one vessel with controlled inflow and demand-driven
 * outflow. The current implementation keeps a single stored-energy state so
 * pressure-loop behavior and downstream load sensitivity stay explicit.
 *
 * Control is difficult because the plant integrates flow imbalance and can be
 * strongly affected by downstream demand changes, especially near saturation.
 *
 * The current implementation models one lumped vessel volume and one pressure
 * state. It does not model temperature variation, compressibility changes with
 * pressure, safety relief logic, or pipe-network transients.
 */
class PressureVessel : public ParametricSystem {
public:
    PressureVessel();
    const char* name() const override { return "Pressure Vessel"; }
    const char* description() const override { return "Pressure regulation with variable outflow"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 43; }
    int stateDim() const override { return 1; } // [P]
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
