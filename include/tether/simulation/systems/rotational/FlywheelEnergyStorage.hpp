#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Flywheel energy-storage benchmark.
 *
 * Models flywheel rotor speed and electrical current for a motor-generator based
 * storage system. The current implementation keeps the dominant electromechanical
 * energy-storage path needed for charge-discharge regulation studies.
 *
 * Control is difficult because the plant must regulate speed over a wide energy
 * range while respecting electrical and mechanical stress limits.
 *
 * The current implementation models one rigid rotor and one drive circuit. It
 * does not model magnetic bearings, vacuum losses, thermal growth, containment
 * limits, or grid-side converter coordination.
 */
class FlywheelEnergyStorage : public ParametricSystem {
public:
    FlywheelEnergyStorage();
    const char* name() const override { return "Flywheel Energy Storage"; }
    const char* description() const override { return "Flywheel speed regulation for energy storage"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 27; }
    int stateDim() const override { return 2; } // [omega, i]
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
