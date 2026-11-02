#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
     * @brief Room HVAC benchmark with hidden thermal mass.
     *
     * Models room air temperature together with slower wall and furniture thermal
     * masses under a single HVAC heat-flow input. The current implementation keeps
     * the dominant latent energy-storage behavior that causes building rebound and
     * time-of-day dependent transients.
     *
     * Control is difficult because the measured air temperature responds faster than
     * the stored heat in walls and contents. A controller can appear to reach the
     * setpoint while hidden thermal mass is still drifting.
     *
     * The current implementation models one room, one HVAC actuator, and two slow
     * internal thermal masses. It does not model humidity, multi-room airflow,
     * occupancy schedules, solar gains, or thermostat logic beyond the plant input.
     */
class RoomHVAC : public ParametricSystem {
public:
    RoomHVAC();
    const char* name() const override { return "Room HVAC"; }
    const char* description() const override { return "Room temperature control with walls, furniture thermal mass and occupancy disturbance"; }
    SystemCategory category() const override { return SystemCategory::Thermal; }
    int systemId() const override { return 37; }
    int stateDim() const override { return 3; } // [T_air, T_walls, T_furniture]
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
/**
     * @param t Current simulation time [s].
     * @param state Current state vector, size stateDim().
     * @param input Current input vector, size inputDim().
     * @return State derivative vector, size stateDim().
     */
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
/**
     * @param t Current simulation time [s].
     * @param state Current state vector, size stateDim().
     * @param input Current input vector, size inputDim().
     * @return Output vector, size outputDim().
     */
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /**Start from a physically meaningful rest state.
     * @return state vector, size stateDim().
    */
    StateVector defaultInitialState() const override;
    /**Describe the tunable physical parameters exposed to experiment sweeps.
     * @return descriptor list.
    */
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /**Provide preset parameter sets for common study cases.
     * @return named preset list.
    */
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
