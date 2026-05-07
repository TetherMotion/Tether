#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Longitudinal fixed-wing aircraft benchmark.
 *
 * Models forward speed, flight-path angle, angle of attack, pitch rate, and
 * altitude for a fixed-wing aircraft in the longitudinal plane. The current
 * implementation uses elevator deflection and thrust as inputs to capture the
 * main energy-management and pitch-coupling dynamics.
 *
 * Control is difficult because the aerodynamic states interact strongly across
 * time scales. Pitch stabilization, speed regulation, and altitude tracking are
 * not independent, and aggressive elevator action can improve one objective
 * while degrading another through stall margin and energy coupling.
 *
 * The current implementation models a reduced longitudinal airframe with lumped
 * aerodynamic coefficients. It does not model lateral-directional motion, stall
 * hysteresis, actuator rate limits, gust fields, propulsion spool dynamics, or
 * full envelope scheduling.
 */
class FixedWingAircraft2D : public ParametricSystem {
public:
    FixedWingAircraft2D();
    const char* name() const override { return "2D Fixed-Wing Aircraft"; }
    const char* description() const override { return "Longitudinal pitch + altitude control — MIMO"; }
    SystemCategory category() const override { return SystemCategory::AerospaceVehicle; }
    int systemId() const override { return 30; }
    int stateDim() const override { return 6; } // [x, V, gamma, alpha, q, h]
    int inputDim() const override { return 2; } // [elevator, thrust]
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
