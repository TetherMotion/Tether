#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Two-inertia flexible-shaft benchmark.
 *
 * Models a motor inertia and load inertia connected by a compliant, damped
 * shaft. The current implementation keeps motor-side and load-side positions and
 * speeds so torsional windup and resonance remain explicit.
 *
 * Control is difficult because torque applied at the motor is not transmitted
 * rigidly to the load, so fast control action can excite torsional resonance.
 *
 * The current implementation models one dominant shaft mode and lumped viscous
 * losses. It does not model gear backlash, multiple flexible shafts, bearing
 * nonlinearities, or high-order structural torsion modes.
 */
class FlexibleShaft : public ParametricSystem {
public:
    FlexibleShaft();
    const char* name() const override { return "Flexible Shaft"; }
    const char* description() const override { return "Motor driving load through compliant coupling — two-inertia system"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 22; }
    int stateDim() const override { return 4; } // [theta_m, omega_m, theta_l, omega_l]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
    /// Evaluate the coupled motor-load dynamics through the compliant shaft.
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    /// Report load-side position, which is the most common measured signal in drive systems.
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /// Start with aligned shafts and zero angular velocity.
    StateVector defaultInitialState() const override;
    /// Return the inertial, damping, and shaft-coupling coefficients.
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /// Provide presets spanning moderately flexible to strongly resonant shafts.
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
