#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Single gravity-drained tank benchmark.
 *
 * Models the liquid level in one gravity-drained tank with inlet flow as the
 * manipulated variable. The current implementation keeps one lumped level state
 * and the dominant square-root outflow nonlinearity.
 *
 * Control is difficult because the effective gain changes with operating level,
 * so a controller tuned near one fill level can become sluggish or aggressive
 * at another.
 *
 * The current implementation models one perfectly mixed tank and one inlet. It
 * does not model sensor delay, valve stiction, foaming, pipe dynamics, or any
 * distributed hydraulics inside the vessel.
 */
class SingleTankLevel : public ParametricSystem {
public:
    SingleTankLevel();
    const char* name() const override { return "Single Tank Level"; }
    const char* description() const override { return "Water tank level control — linear near operating point"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 38; }
    int stateDim() const override { return 1; } // [h]
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
