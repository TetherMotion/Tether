#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Disk-drive head-positioning benchmark.
 *
 * Models voice-coil positioning of a read/write head with one dominant flexible
 * structural mode. The current implementation keeps the rigid positioning motion
 * and one resonance state pair that limits servo bandwidth.
 *
 * Control is difficult because the plant is extremely fast yet resonance limited,
 * so tight settling time requires operating close to a flexible mode.
 *
 * The current implementation models one dominant resonance and one actuator
 * channel. It does not model full suspension mode families, track-following
 * disturbances, spindle eccentricity, or discrete-time sampling effects.
 */
class DiskDriveHead : public ParametricSystem {
public:
    DiskDriveHead();
    const char* name() const override { return "Disk Drive Head"; }
    const char* description() const override { return "HDD read/write head positioning — very fast, resonance-limited"; }
    SystemCategory category() const override { return SystemCategory::RotationalAngular; }
    int systemId() const override { return 23; }
    int stateDim() const override { return 4; }
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
