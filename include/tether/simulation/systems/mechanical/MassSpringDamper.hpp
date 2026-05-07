#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Canonical single-DOF mass-spring-damper benchmark.
 *
 * Models one translational mass attached to a spring and viscous damper with a
 * force input. The current implementation keeps the dominant second-order energy
 * exchange between inertia and elasticity that defines the standard benchmark.
 *
 * Control is difficult mainly when damping is low, because the plant stores and
 * exchanges energy between inertia and elasticity and easily resonates.
 *
 * The current implementation models one linear spring, one viscous damper, and
 * one rigid mass. It does not model friction nonlinearities, backlash, actuator
 * saturation, or higher structural modes.
 */
class MassSpringDamper : public ParametricSystem {
public:
    MassSpringDamper();
    const char* name() const override { return "Mass-Spring-Damper"; }
    const char* description() const override { return "Single DOF mass-spring-damper: canonical linear second-order system"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 1; }
    int stateDim() const override { return 2; } // [x, dx]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
    /// Evaluate the second-order translational dynamics for the current force input.
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    /// Report the measured displacement state used by the identification tests.
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /// Start from rest so transients are driven entirely by the applied excitation.
    StateVector defaultInitialState() const override;
    /// Return the tunable physical coefficients used to shape resonance and damping.
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /// Provide underdamped, critically damped, and high-frequency benchmark presets.
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
    std::vector<std::string> equationStrings() const override;
};

} // namespace Simulation
