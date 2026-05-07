#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Compact blood-glucose regulation benchmark.
 *
 * Models glucose concentration, remote insulin action, and plasma insulin using
 * a Bergman-style minimal patient model. The current implementation represents
 * insulin infusion as the single manipulated input and blood glucose as the
 * regulated output for closed-loop artificial-pancreas style experiments.
 *
 * Control is difficult because the plant is slow, delayed, and patient
 * dependent. Insulin acts long after delivery, so a controller that only reacts
 * to the current glucose error can easily over-correct and create delayed
 * hypoglycemia.
 *
 * The current implementation models basal insulin dynamics, delayed insulin
 * action, and simple endogenous pancreatic response. It does not model meals,
 * exercise, sensor calibration drift, subcutaneous absorption compartments, or
 * patient-to-patient variability beyond manual parameter changes.
 */
class BloodGlucose : public ParametricSystem {
public:
    BloodGlucose();
    const char* name() const override { return "Blood Glucose Regulation"; }
    const char* description() const override { return "Insulin pump — long delay, nonlinear patient model (Bergman minimal)"; }
    SystemCategory category() const override { return SystemCategory::BiologicalBiomedical; }
    int systemId() const override { return 54; }
    int stateDim() const override { return 3; } // [G, X, I] (glucose, remote insulin, plasma insulin)
    int inputDim() const override { return 1; } // [insulin_rate]
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
