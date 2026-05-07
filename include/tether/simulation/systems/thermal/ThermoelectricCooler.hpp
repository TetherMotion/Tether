#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Thermoelectric cooler benchmark.
 *
 * Models hot-side and cold-side temperatures of a Peltier device with current as
 * the manipulated input. The current implementation preserves bidirectional heat
 * pumping, Joule heating, and the thermal coupling between the two sides.
 *
 * Control is difficult because the same current that pumps heat also generates
 * resistive heating. Performance is operating-point dependent, so a controller
 * must manage conflicting hot-side and cold-side objectives.
 *
 * The current implementation models two lumped thermal masses and one current
 * input. It does not model detailed semiconductor leg dynamics, current-driver
 * switching, contact resistance variation, condensation, or fan-cooling effects.
 */
class ThermoelectricCooler : public ParametricSystem {
public:
    ThermoelectricCooler();
    const char* name() const override { return "Thermoelectric Cooler"; }
    const char* description() const override { return "Peltier device — nonlinear, bidirectional heat pumping"; }
    SystemCategory category() const override { return SystemCategory::Thermal; }
    int systemId() const override { return 36; }
    int stateDim() const override { return 2; } // [T_cold, T_hot]
    int inputDim() const override { return 1; } // [current]
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
