#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Pharmacokinetic and pharmacodynamic anesthesia benchmark.
 *
 * Models drug distribution through central and peripheral compartments together
 * with an effect-site state that drives a BIS-like depth-of-anesthesia output.
 * The current implementation treats infusion rate as the single control input
 * and captures the delayed conversion from delivered drug to observed effect.
 *
 * Control is difficult because the input-output delay is long and state
 * dependent. Drug can accumulate in unmeasured compartments, so aggressive
 * regulation based only on the current BIS error can overdose the patient or
 * produce very long recovery transients.
 *
 * The current implementation models three PK compartments, one PD effect site,
 * and a static Hill nonlinearity for the BIS output. It does not model blood
 * pressure interactions, multiple simultaneous drugs, airway events, surgical
 * stimulation, or patient-specific adaptation beyond parameter tuning.
 */
class AnesthesiaControl : public ParametricSystem {
public:
    AnesthesiaControl();
    const char* name() const override { return "Anesthesia Depth Control"; }
    const char* description() const override { return "Pharmacokinetic/pharmacodynamic model — long and variable delay"; }
    SystemCategory category() const override { return SystemCategory::BiologicalBiomedical; }
    int systemId() const override { return 55; }
    int stateDim() const override { return 4; } // [c1, c2, c3, ce] (compartments + effect site)
    int inputDim() const override { return 1; } // [infusion_rate]
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
