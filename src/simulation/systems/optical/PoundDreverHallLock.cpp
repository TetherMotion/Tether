#include "tether/simulation/systems/optical/PoundDreverHallLock.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

namespace {

double sinusoid(double amplitude, double frequency_hz, double t, double phase = 0.0) {
    return amplitude * std::sin(2.0 * M_PI * frequency_hz * t + phase);
}

} // namespace

PoundDreverHallLock::PoundDreverHallLock() {
    initParam("linewidth", 1.0);
    initParam("detune_tau", 0.0015);
    initParam("piezo_wn_hz", 3200.0);
    initParam("piezo_zeta", 0.2);
    initParam("piezo_gain", 12.0);
    initParam("thermal_tau", 0.2);
    initParam("thermal_gain", 0.8);
    initParam("acoustic_amp", 1.4);
    initParam("acoustic_hz", 180.0);
    initParam("laser_noise_amp", 0.45);
    initParam("laser_noise_hz", 900.0);
    initParam("sense_tau", 0.0002);
}

StateVector PoundDreverHallLock::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double linewidth = params_.at("linewidth");
    const double detune_tau = params_.at("detune_tau");
    const double piezo_wn = 2.0 * M_PI * params_.at("piezo_wn_hz");
    const double piezo_zeta = params_.at("piezo_zeta");
    const double piezo_gain = params_.at("piezo_gain");
    const double thermal_tau = params_.at("thermal_tau");
    const double thermal_gain = params_.at("thermal_gain");
    const double acoustic_amp = params_.at("acoustic_amp");
    const double acoustic_hz = params_.at("acoustic_hz");
    const double laser_noise_amp = params_.at("laser_noise_amp");
    const double laser_noise_hz = params_.at("laser_noise_hz");
    const double sense_tau = params_.at("sense_tau");

    const double voltage = u.empty() ? 0.0 : u[0];

    const double detuning = s[0];
    const double piezo = s[1];
    const double piezo_rate = s[2];
    const double thermal = s[3];
    const double sensed = s[4];

    const double acoustic = sinusoid(acoustic_amp, acoustic_hz, t) + sinusoid(0.35 * acoustic_amp, 0.37 * acoustic_hz, t, 0.4);
    const double laser = sinusoid(laser_noise_amp, laser_noise_hz, t, 0.9);

    const double ddetuning = (-detuning + thermal + acoustic + laser - piezo_gain * piezo) / detune_tau;
    const double dpiezo = piezo_rate;
    const double dpiezo_rate = piezo_wn * piezo_wn * (voltage - piezo) - 2.0 * piezo_zeta * piezo_wn * piezo_rate;
    const double dthermal = (-thermal + thermal_gain * sinusoid(1.0, 0.8, t)) / thermal_tau;

    const double normalized = detuning / std::max(linewidth, 1e-9);
    const double error_signal = normalized * std::exp(-normalized * normalized);
    const double dsensed = (error_signal - sensed) / sense_tau;

    return {ddetuning, dpiezo, dpiezo_rate, dthermal, dsensed};
}

StateVector PoundDreverHallLock::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[4], s[0]};
}

StateVector PoundDreverHallLock::defaultInitialState() const {
    return {0.5, 0.0, 0.0, 0.15, 0.0};
}

std::vector<ParamDescriptor> PoundDreverHallLock::parameterDescriptors() const {
    return {{"linewidth","arb","Normalized cavity linewidth",1.0,0.05,20.0,0.05},
            {"detune_tau","s","Detuning time constant",0.0015,0.0001,0.05,0.0001},
            {"piezo_wn_hz","Hz","Piezo resonance",3200.0,50.0,30000.0,10.0},
            {"piezo_zeta","","Piezo damping ratio",0.2,0.01,1.5,0.01},
            {"piezo_gain","arb/V","Piezo detuning gain",12.0,0.1,100.0,0.1},
            {"thermal_tau","s","Thermal drift time constant",0.2,0.01,20.0,0.01},
            {"thermal_gain","arb","Thermal drift gain",0.8,0.0,10.0,0.05},
            {"acoustic_amp","arb","Acoustic disturbance amplitude",1.4,0.0,20.0,0.05},
            {"acoustic_hz","Hz","Acoustic disturbance frequency",180.0,1.0,5000.0,1.0},
            {"laser_noise_amp","arb","Laser frequency noise amplitude",0.45,0.0,10.0,0.01},
            {"laser_noise_hz","Hz","Laser noise frequency",900.0,1.0,50000.0,1.0},
            {"sense_tau","s","Photodetector-demod lag",0.0002,0.00001,0.01,0.00001}};
}

std::vector<Preset> PoundDreverHallLock::presets() const {
    return {{"Rigid cavity","Fast piezo and moderate acoustic noise",{{"linewidth",1.0},{"detune_tau",0.0015},{"piezo_wn_hz",3200.0},{"piezo_zeta",0.2},{"piezo_gain",12.0},{"thermal_tau",0.2},{"thermal_gain",0.8},{"acoustic_amp",1.4},{"acoustic_hz",180.0},{"laser_noise_amp",0.45},{"laser_noise_hz",900.0},{"sense_tau",0.0002}}},
            {"Soft mount","Stronger acoustic loading and slower sensing",{{"linewidth",0.7},{"detune_tau",0.0025},{"piezo_wn_hz",1800.0},{"piezo_zeta",0.12},{"piezo_gain",10.0},{"thermal_tau",0.35},{"thermal_gain",1.2},{"acoustic_amp",2.1},{"acoustic_hz",95.0},{"laser_noise_amp",0.65},{"laser_noise_hz",600.0},{"sense_tau",0.0004}}}};
}

void PoundDreverHallLock::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> PoundDreverHallLock::stateNames() const {
    return {"detuning","piezo","piezo_rate","thermal","error_filtered"};
}

std::vector<std::string> PoundDreverHallLock::outputNames() const {
    return {"PDH error","Detuning"};
}

std::vector<std::string> PoundDreverHallLock::inputNames() const {
    return {"Piezo voltage"};
}

}  // namespace Simulation
