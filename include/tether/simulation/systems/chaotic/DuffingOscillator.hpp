#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Forced Duffing-oscillator benchmark.
 *
 * Models a forced nonlinear oscillator with linear damping, linear stiffness,
 * and a cubic spring term. The current implementation preserves amplitude-
 * dependent resonance shift, multi-well behavior, and forcing-driven branch
 * transitions in a compact second-order model.
 *
 * Control is difficult because the plant is strongly amplitude dependent, so a
 * controller tuned around one regime can perform poorly after larger motion.
 *
 * The current implementation models one dominant nonlinear mode and one forcing
 * channel. It does not model higher structural modes, frictional hysteresis,
 * distributed beam dynamics, or true stochastic excitation.
 */
class DuffingOscillator : public ParametricSystem {
public:
    DuffingOscillator();
    const char* name() const override { return "Duffing Oscillator"; }
    const char* description() const override { return "Nonlinear resonance with chaotic forcing — fractal basin boundaries"; }
    SystemCategory category() const override { return SystemCategory::ChaoticExtreme; }
    int systemId() const override { return 60; }
    int stateDim() const override { return 2; }
    int inputDim() const override { return 1; }
    int outputDim() const override { return 1; }
    /// Evaluate the nonlinear forced oscillator with linear damping and cubic spring stiffness.
    StateVector dynamics(double t, const StateVector& state, const StateVector& input) const override;
    /// Report oscillator displacement for direct comparison with ETFE estimates.
    StateVector output(double t, const StateVector& state, const StateVector& input) const override;
    /// Start near the origin with a small displacement to avoid trivial zero motion.
    StateVector defaultInitialState() const override;
    /// Return the forcing and stiffness coefficients that shape the nonlinear response.
    std::vector<ParamDescriptor> parameterDescriptors() const override;
    /// Provide both the classic chaotic regime and a milder hardening-spring regime.
    std::vector<Preset> presets() const override;
    void applyPreset(int index) override;
    std::vector<std::string> stateNames() const override;
    std::vector<std::string> outputNames() const override;
    std::vector<std::string> inputNames() const override;
};

} // namespace Simulation
