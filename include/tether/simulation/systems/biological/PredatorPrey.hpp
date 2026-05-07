#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Harvested predator-prey ecosystem benchmark.
 *
 * Models coupled prey and predator populations with nonlinear interaction terms
 * and an external harvesting or intervention input. The current implementation
 * uses a Lotka-Volterra style structure with optional prey carrying capacity so
 * that population balance and ecological oscillation remain explicit states.
 *
 * Control is difficult because the plant is inherently oscillatory and highly
 * state coupled. Suppressing one species too strongly can destabilize the other
 * population, and interventions that look beneficial in the short term may
 * amplify long-period cycles or drive the system toward ecological collapse.
 *
 * The current implementation models prey growth, predator reproduction, prey
 * carrying capacity, and direct harvesting. It does not model age structure,
 * seasonal migration, spatial habitat effects, multi-species food webs, or
 * stochastic environmental shocks beyond what an external user injects.
 */
class PredatorPrey : public ParametricSystem {
public:
    PredatorPrey();
    const char* name() const override { return "Predator-Prey Control"; }
    const char* description() const override { return "Lotka-Volterra with harvesting — nonlinear, oscillatory"; }
    SystemCategory category() const override { return SystemCategory::BiologicalBiomedical; }
    int systemId() const override { return 56; }
    int stateDim() const override { return 2; } // [prey, predator]
    int inputDim() const override { return 1; } // [harvesting_rate]
    int outputDim() const override { return 2; }
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
