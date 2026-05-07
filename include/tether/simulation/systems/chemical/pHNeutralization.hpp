#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief pH neutralization benchmark.
 *
 * Models a mixing tank with acid and base invariants plus liquid level, where
 * the measured pH arises from a strongly nonlinear titration relationship. The
 * current implementation keeps the chemistry reduced to the dominant invariants
 * so the extreme gain change around neutrality is preserved.
 *
 * Control is difficult because gain varies enormously across the operating
 * range. Near neutrality, tiny flow changes can produce large pH swings, while
 * the same actuation has little effect far from that region.
 *
 * The current implementation models one perfectly mixed vessel, invariant-based
 * chemistry, and a single manipulated base flow. It does not model transport
 * delay, imperfect mixing, electrode dynamics, buffer chemistry, or additional
 * feed disturbances beyond the nominal inlet assumptions.
 */
class pHNeutralization : public ParametricSystem {
public:
    pHNeutralization();
    const char* name() const override { return "pH Neutralization"; }
    const char* description() const override { return "Extremely nonlinear titration curve — process control challenge"; }
    SystemCategory category() const override { return SystemCategory::ChemicalProcess; }
    int systemId() const override { return 51; }
    int stateDim() const override { return 3; } // [W_a, W_b, h] (acid/base invariants + level)
    int inputDim() const override { return 1; } // [base flow rate]
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
