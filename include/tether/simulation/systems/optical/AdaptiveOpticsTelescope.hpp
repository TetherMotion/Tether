#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Reduced-order adaptive optics telescope model.
 *
 * Models a two-mode adaptive-optics correction loop for a ground-based
 * telescope. The current implementation includes coupled atmospheric phase
 * modes, a second-order deformable-mirror actuator pair with influence-matrix
 * cross-coupling, and finite-bandwidth wavefront-sensor readout of the residual
 * phase error.
 *
 * Control is difficult because even this reduced model is MIMO, spatially
 * coupled, and delay limited. The controller must reject stochastic-like
 * turbulence while the mirror and sensor dynamics consume phase margin at the
 * kilohertz update rates where adaptive optics normally operates.
 *
 * The current implementation models only two dominant optical modes and a lumped
 * wavefront-sensor lag. It does not model a full actuator grid, explicit
 * Kolmogorov phase-screen propagation, pupil geometry, or estimator/computation
 * pipelines at the fidelity used in observatory-scale AO simulators.
 */
class AdaptiveOpticsTelescope : public ParametricSystem {
public:
    AdaptiveOpticsTelescope();
    const char* name() const override { return "Adaptive Optics Telescope"; }
    const char* description() const override { return "Reduced-order adaptive optics loop with deformable-mirror coupling and sensor lag"; }
    SystemCategory category() const override { return SystemCategory::OpticalPhotonic; }
    int systemId() const override { return 66; }
    int stateDim() const override { return 8; }
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