#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Exothermic continuous stirred-tank reactor benchmark.
 *
 * Models concentration and temperature in a continuously stirred reactor with
 * feed-flow and cooling inputs. The current implementation couples material and
 * energy balances through Arrhenius kinetics so thermal runaway sensitivity and
 * conversion tradeoffs remain explicit.
 *
 * Control is difficult because the reactor can be highly nonlinear, stiff, and
 * multi-steady-state. Small temperature changes can cause large rate changes,
 * and one manipulated input often affects conversion and thermal safety at the
 * same time.
 *
 * The current implementation models one well-mixed reactor volume, one dominant
 * reaction, and lumped cooling action. It does not model spatial gradients,
 * jacket dynamics, catalyst deactivation, feed composition noise, or detailed
 * safety interlocks beyond what a user injects externally.
 */
class CSTR : public ParametricSystem {
public:
    CSTR();
    const char* name() const override { return "CSTR"; }
    const char* description() const override { return "Continuous stirred-tank reactor — MIMO, exothermic, multiple steady states"; }
    SystemCategory category() const override { return SystemCategory::ChemicalProcess; }
    int systemId() const override { return 50; }
    int stateDim() const override { return 2; } // [C_A, T]
    int inputDim() const override { return 2; } // [F, Q_cool]
    int outputDim() const override { return 2; }
    /// Evaluate coupled species and energy balances for an exothermic reactor.
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    /// Report concentration and temperature, which are the usual process measurements.
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /// Start from a moderate conversion operating point near the nominal feed temperature.
    StateVector defaultInitialState() const override;
    /// Return kinetic, thermal, and transport coefficients for operating-point studies.
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /// Provide standard and high-gain operating points for nonlinear identification experiments.
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
