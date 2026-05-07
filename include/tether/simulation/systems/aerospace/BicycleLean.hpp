#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Bicycle and motorcycle lean-stabilization benchmark.
 *
 * Models lean and steer dynamics for a two-wheeled vehicle using lean angle,
 * lean rate, steer angle, and steer rate as the dominant states. The current
 * implementation focuses on the balance-relevant lateral dynamics rather than
 * full path-tracking vehicle motion.
 *
 * Control is difficult because the plant is speed dependent and can become only
 * lightly stable or unstable depending on operating point. Steering produces a
 * counterintuitive short-term response before it generates the restoring lean
 * dynamics needed for balance.
 *
 * The current implementation models reduced-order lean-steer coupling at a fixed
 * effective speed. It does not model tire slip, rider motion, full bicycle
 * kinematics, steering saturation, or detailed road-contact nonlinearities.
 */
class BicycleLean : public ParametricSystem {
public:
    BicycleLean();
    const char* name() const override { return "Bicycle Lean Stabilization"; }
    const char* description() const override { return "2D side view — speed-dependent stability, lean angle control"; }
    SystemCategory category() const override { return SystemCategory::AerospaceVehicle; }
    int systemId() const override { return 31; }
    int stateDim() const override { return 4; } // [phi, dphi, delta, ddelta]
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
