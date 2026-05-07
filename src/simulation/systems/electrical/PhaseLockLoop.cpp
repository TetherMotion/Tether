#include "tether/simulation/systems/electrical/PhaseLockLoop.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PhaseLockLoop::PhaseLockLoop() {
    initParam("Kp_pll", 10.0);  // proportional gain
    initParam("Ki_pll", 100.0); // integral gain
    initParam("Kvco", 2.0*M_PI*1000.0); // VCO gain [rad/s/V]
    initParam("f_ref", 1000.0); // reference frequency [Hz]
}

StateVector PhaseLockLoop::dynamics(double t, const StateVector& s, const StateVector& u) const {
    double Kp = params_.at("Kp_pll"), Ki = params_.at("Ki_pll");
    double Kvco = params_.at("Kvco"), f_ref = params_.at("f_ref");
    double f_in = u.empty() ? f_ref : u[0]; // input frequency [Hz]

    double phase_error = s[0]; // phase detector output
    double integrator = s[1];  // loop filter integrator
    double vco_phase = s[2];   // VCO phase [rad]

    // Phase detector: difference between input and VCO phase
    double input_phase = 2.0*M_PI*f_in*t;
    double pe = std::sin(input_phase - vco_phase); // XOR-type PD approx

    double dpe = pe - phase_error; // filter (smoothing)
    double dint = phase_error;
    double vco_input = Kp*phase_error + Ki*integrator;
    double dvco = Kvco * vco_input + 2.0*M_PI*f_ref; // free-running + correction

    return {dpe, dint, dvco};
}

StateVector PhaseLockLoop::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[2]}; }
StateVector PhaseLockLoop::defaultInitialState() const { return {0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> PhaseLockLoop::parameterDescriptors() const {
    return {{"Kp_pll","","Proportional gain",10.0,0.1,100.0,0.1},
            {"Ki_pll","","Integral gain",100.0,1.0,10000.0,1.0},
            {"Kvco","rad/s/V","VCO gain",2.0*M_PI*1000.0,100.0,100000.0,100.0},
            {"f_ref","Hz","Reference freq",1000.0,1.0,100000.0,1.0}};
}

std::vector<Preset> PhaseLockLoop::presets() const {
    return {{"1 kHz","Standard PLL",{{"Kp_pll",10.0},{"Ki_pll",100.0},{"Kvco",2.0*M_PI*1000.0},{"f_ref",1000.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"10 kHz","Fast PLL",{{"Kp_pll",50.0},{"Ki_pll",500.0},{"Kvco",2.0*M_PI*10000.0},{"f_ref",10000.0}}}};
}
void PhaseLockLoop::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PhaseLockLoop::stateNames() const { return {"phase_error","integrator","vco_phase"}; }
std::vector<std::string> PhaseLockLoop::outputNames() const { return {"VCO phase"}; }
std::vector<std::string> PhaseLockLoop::inputNames() const { return {"Input frequency"}; }

}  // namespace Simulation
