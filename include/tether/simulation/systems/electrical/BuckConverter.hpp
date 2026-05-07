#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Averaged buck-converter benchmark.
 *
 * Models a step-down converter using inductor current and capacitor voltage as
 * the dominant energy-storage states with duty cycle as the input. The current
 * implementation uses an averaged switching model so output-voltage regulation
 * and current shaping can be studied without PWM-scale time steps.
 *
 * Control is difficult because the electrical dynamics are fast and strongly
 * constrained by switching and saturation limits. Small timing errors or overly
 * aggressive bandwidth can amplify ripple or erode robustness.
 *
 * The current implementation models the dominant LC energy storage and load
 * effect. It does not model discrete switching ripple, dead time, synchronous
 * rectifier commutation, EMI, or detailed semiconductor thermal behavior.
 */
class BuckConverter : public ParametricSystem {
public:
    BuckConverter();
    const char* name() const override { return "Buck Converter"; }
    const char* description() const override { return "Switching averaged model — step-down voltage regulation"; }
    SystemCategory category() const override { return SystemCategory::ElectricalElectronic; }
    int systemId() const override { return 44; }
    int stateDim() const override { return 2; } // [i_L, v_C]
    int inputDim() const override { return 1; } // [duty_cycle]
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
