#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief DC-motor position-servo benchmark.
 *
 * Models shaft position, shaft speed, and armature current for a voltage-driven
 * DC motor. The current implementation adds the integrated position state on top
 * of the basic electromechanical motor dynamics used in servo positioning.
 *
 * Control is difficult because position regulation adds another level of phase
 * lag on top of the speed dynamics, so fast tracking and robustness conflict.
 *
 * The current implementation models one rigid shaft and one armature circuit. It
 * does not model encoder quantization, backlash, shaft compliance, load torque
 * disturbances beyond the plant equations, or current-controller hierarchy.
 */
class DCMotorPosition : public ParametricSystem {
public:
    DCMotorPosition();
    const char* name() const override { return "DC Motor Position Control"; }
    const char* description() const override { return "DC motor with position output — second-order with back-EMF"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 21; }
    int stateDim() const override { return 3; } // [theta, omega, i]
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
