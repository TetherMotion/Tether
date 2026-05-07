#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Servo-hydraulic actuator benchmark.
 *
 * Models piston position, piston velocity, and chamber pressure for a hydraulic
 * actuator driven by a valve input. The current implementation keeps fluid
 * compressibility and valve-to-force coupling as the dominant nonlinear effects.
 *
 * Control is difficult because actuator force depends on pressure build-up that
 * does not change instantaneously, so aggressive bandwidth can excite pressure
 * oscillation and saturation.
 *
 * The current implementation models one lumped pressure state and one moving
 * mass. It does not model spool dynamics, hose compliance, leakage, cavitation,
 * or asymmetric cylinder chambers in full detail.
 */
class HydraulicActuator : public ParametricSystem {
public:
    HydraulicActuator();
    const char* name() const override { return "Hydraulic Actuator"; }
    const char* description() const override { return "Position control with compressible fluid and nonlinear valve"; }
    SystemCategory category() const override { return SystemCategory::FluidHydraulic; }
    int systemId() const override { return 41; }
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
