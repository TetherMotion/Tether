#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Bouncing-ball hybrid benchmark.
 *
 * Models a ball interacting with a moving plate through free flight and impact
 * events. The current implementation preserves smooth gravitational motion plus
 * discontinuous impact behavior at contact.
 *
 * Control is difficult because the system is hybrid rather than purely smooth,
 * so small timing errors near impact can produce large rebound changes.
 *
 * The current implementation models one ball, one plate, and restitution-style
 * impacts. It does not model contact deformation, frictional contact, plate
 * flexibility, or random surface roughness.
 */
class BouncingBall : public ParametricSystem {
public:
    BouncingBall();
    const char* name() const override { return "Bouncing Ball"; }
    const char* description() const override { return "Ball bouncing on a vibrating plate — hybrid/impact dynamics"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 11; }
    int stateDim() const override { return 4; } // [y_ball, dy_ball, y_plate, dy_plate]
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
