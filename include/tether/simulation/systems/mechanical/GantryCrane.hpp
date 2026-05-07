#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Gantry-crane sway benchmark.
 *
 * Models a translating trolley with a suspended pendulum-like payload. The
 * current implementation preserves the main underactuated sway mode excited by
 * trolley motion.
 *
 * Control is difficult because rapid trolley motion naturally excites payload
 * oscillation, so travel time and swing suppression conflict.
 *
 * The current implementation models one trolley axis and one pendulum payload.
 * It does not model cable elasticity, hoist length variation, hook dynamics, or
 * three-dimensional swing.
 */
class GantryCrane : public ParametricSystem {
public:
    GantryCrane();
    const char* name() const override { return "Gantry Crane"; }
    const char* description() const override { return "Trolley on overhead rail with swinging pendulum load"; }
    SystemCategory category() const override { return SystemCategory::MechanicalTranslational; }
    int systemId() const override { return 13; }
    int stateDim() const override { return 4; } // [x, dx, theta, dtheta]
    int inputDim() const override { return 1; }
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
