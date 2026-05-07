#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Ball-on-beam benchmark.
 *
 * Models a ball rolling along a beam whose angle is indirectly actuated. The
 * current implementation preserves the non-collocated coupling between beam tilt
 * and ball translation.
 *
 * Control is difficult because the actuator moves the beam angle, not the ball
 * position directly, so tilt commands first affect acceleration through gravity.
 *
 * The current implementation models one rolling ball and one beam angle. It does
 * not model contact slip, actuator saturation, end stops, or beam flexibility.
 */
class BallOnBeam : public ParametricSystem {
public:
    BallOnBeam();
    const char* name() const override { return "Ball on Beam"; }
    const char* description() const override { return "Ball rolling on a tilting beam — 1D balance problem"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 9; }
    int stateDim() const override { return 4; } // [r, dr, theta, dtheta]
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
