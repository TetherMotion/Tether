#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Dual-axis magnetic-levitation benchmark.
 *
 * Models two levitated axes with coupled electrical and mechanical behavior. The
 * current implementation preserves the main multivariable gap-coupling effects
 * of a two-channel unstable levitation plant.
 *
 * Control is difficult because each actuator affects an unstable gap while also
 * perturbing the other channel through coupling.
 *
 * The current implementation models two levitated masses and two coil channels.
 * It does not model full magnetic field maps, lateral instability, sensor cross-
 * talk beyond the state equations, or power-stage protection logic.
 */
class DualMagneticLevitation : public ParametricSystem {
public:
    DualMagneticLevitation();
    const char* name() const override { return "Dual Magnetic Levitation"; }
    const char* description() const override { return "Two balls, two electromagnets — MIMO with magnetic coupling"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 16; }
    int stateDim() const override { return 6; } // [y1, dy1, i1, y2, dy2, i2]
    int inputDim() const override { return 2; }
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
