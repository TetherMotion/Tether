#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Quarter-car suspension benchmark.
 *
 * Models one sprung mass and one unsprung mass connected by suspension and tire
 * elements. The current implementation preserves the two dominant ride and wheel-
 * hop modes under one active suspension force input.
 *
 * Control is difficult because ride comfort and road holding conflict, and both
 * resonant modes must be managed under broadband road disturbance.
 *
 * The current implementation models one vertical corner of a vehicle. It does
 * not model full chassis pitch and roll, nonlinear dampers, suspension travel
 * stops, or tire-road contact loss.
 */
class QuarterCarSuspension : public ParametricSystem {
public:
    QuarterCarSuspension();
    const char* name() const override { return "Quarter-Car Suspension"; }
    const char* description() const override { return "Active suspension: sprung mass, unsprung mass, road input"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 17; }
    int stateDim() const override { return 4; } // [x_s, dx_s, x_u, dx_u]
    int inputDim() const override { return 1; }
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
