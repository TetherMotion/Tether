#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Conveyor transport-delay benchmark.
 *
 * Models material transport on a conveyor where an upstream control action is
 * only observed after the part has moved through a processing zone. The current
 * implementation uses a reduced internal transport state plus downstream
 * measurement, preserving the core industrial dead-time behavior.
 *
 * Control is difficult because the controller acts on material that has not yet
 * reached the sensor. Disturbances introduced upstream may only become visible
 * much later, so feedback alone is slow and predictive action is needed.
 *
 * The current implementation models one dominant transport path and one delayed
 * downstream measurement. It does not model conveyor slip, part-to-part
 * variation, multiple processing stations, queueing logic, or vision latency.
 */
class ConveyorBeltTracking : public ParametricSystem {
public:
    ConveyorBeltTracking();
    const char* name() const override { return "Conveyor Belt Tracking"; }
    const char* description() const override { return "Material moving through processing zone with transport delay"; }
    SystemCategory category() const override { return SystemCategory::DelayDominated; }
    int systemId() const override { return 65; }
    int stateDim() const override { return 2; }
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
