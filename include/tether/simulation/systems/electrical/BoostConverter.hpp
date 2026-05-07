#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Averaged boost-converter benchmark.
 *
 * Models a step-up converter with inductor current and output voltage states.
 * The current implementation uses an averaged duty-cycle model that preserves
 * the characteristic inverse-response behavior of boost voltage regulation.
 *
 * Control is difficult because the output-voltage path is non-minimum phase.
 * An increase in duty cycle can initially move the output in the wrong
 * direction before the stored-energy benefit appears.
 *
 * The current implementation models the dominant LC dynamics and load coupling.
 * It does not model switching ripple, diode recovery, current-limit logic,
 * discrete PWM effects, or converter mode changes at very light load.
 */
class BoostConverter : public ParametricSystem {
public:
    BoostConverter();
    const char* name() const override { return "Boost Converter"; }
    const char* description() const override { return "Non-minimum phase step-up voltage regulation"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 45; }
    int stateDim() const override { return 2; }
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    StateVector defaultInitialState() const override;
    StateVector defaultInput() const override;
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
