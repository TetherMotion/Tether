#pragma once
#include "../../DynamicalSystem.hpp"

namespace Simulation {

/**
 * @brief Reduced-order optical cavity frequency-locking benchmark.
 *
 * Models a Pound-Drever-Hall style frequency-locking loop using cavity detuning,
 * piezo actuator dynamics, thermal drift, acoustic disturbance, and the sharp
 * nonlinear error signal generated near cavity resonance.
 *
 * Control is difficult because the useful linear region around resonance is very
 * small, while environmental disturbances and laser noise push the system across
 * steep gain variation. High bandwidth helps disturbance rejection but can also
 * amplify sensor noise and destabilize the piezo-mechanical path.
 *
 * The current implementation models one dominant cavity mode, one piezo axis,
 * and lumped acoustic and thermal disturbances. It does not model modulation
 * sidebands explicitly, photodetector shot noise, cavity higher-order modes, or
 * the full RF demodulation chain used in laboratory PDH hardware.
 */
class PoundDreverHallLock : public ParametricSystem {
public:
    PoundDreverHallLock();
    const char* name() const override { return "Pound-Drever-Hall Lock"; }
    const char* description() const override { return "Optical cavity frequency lock with nonlinear resonance error signal"; }
    SystemCategory category() const override { return SystemCategory::OpticalPhotonic; }
    int systemId() const override { return 68; }
    int stateDim() const override { return 5; }
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