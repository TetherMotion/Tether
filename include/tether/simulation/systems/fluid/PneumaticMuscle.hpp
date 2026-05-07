#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Pneumatic artificial-muscle benchmark.
 *
 * Models a McKibben-style pneumatic muscle with contraction motion, velocity,
 * and internal pressure as dominant states. The current implementation preserves
 * the main pressure-length-force coupling that makes soft pneumatic actuation
 * difficult to regulate.
 *
 * Control is difficult because the force-length-pressure relationship is highly
 * nonlinear and often hysteretic, while air compressibility slows the response.
 *
 * The current implementation models one lumped pressure state and one axial
 * motion coordinate. It does not model braided-shell geometry in detail, valve
 * dead zones, temperature effects, or full hysteresis memory loops.
 */
class PneumaticMuscle : public ParametricSystem {
public:
    PneumaticMuscle();
    const char* name() const override { return "Pneumatic Muscle"; }
    const char* description() const override { return "McKibben actuator — highly nonlinear, hysteretic"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 42; }
    int stateDim() const override { return 3; } // [x, dx, P]
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
