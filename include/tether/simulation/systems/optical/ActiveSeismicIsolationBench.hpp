#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Reduced-order multi-stage active seismic isolation plant.
 *
 * Models three cascaded isolation stages between ground and an optical bench,
 * each with lightly damped spring-mass dynamics and its own actuator channel.
 * The ground input appears as a broadband seismic disturbance with low- and
 * high-frequency components, and the bench motion is the controlled output.
 *
 * Control is difficult because the stages are strongly coupled and lightly
 * damped, so trying to suppress motion at one stage can easily excite another.
 * The plant is representative of robust sensor-blending and resonance-management
 * problems where stability margins must survive uncertain high-Q modes.
 *
 * The current implementation models one dominant translational axis and three
 * active stages. It does not model the full six-degree-of-freedom mechanics,
 * real geophone/interferometer noise floors, suspension geometry, or the large
 * MIMO sensor-fusion networks used in observatory-scale platforms such as LIGO.
 */
class ActiveSeismicIsolationBench : public ParametricSystem {
public:
    ActiveSeismicIsolationBench();
    const char* name() const override { return "Active Seismic Isolation Bench"; }
    const char* description() const override { return "Three-stage optical bench isolation with lightly damped coupled resonances"; }
    SystemCategory category() const override { return SystemCategory::OpticalPhotonic; }
    int systemId() const override { return 69; }
    int stateDim() const override { return 6; }
    int inputDim() const override { return 3; }
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