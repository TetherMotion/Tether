#include "tether/simulation/systems/chaotic/DuffingOscillator.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DuffingOscillator::DuffingOscillator() {
    initParam("alpha_d", -1.0);  // linear stiffness
    initParam("beta_d", 1.0);   // cubic stiffness
    initParam("delta_d", 0.3);  // damping
    initParam("gamma_d", 0.37); // forcing amplitude
    initParam("omega_d", 1.2);  // forcing frequency
}

StateVector DuffingOscillator::dynamics(double t, const StateVector& s, const StateVector& u) const {
    double alpha = params_.at("alpha_d"), beta = params_.at("beta_d");
    double delta = params_.at("delta_d"), gamma_f = params_.at("gamma_d"), omega = params_.at("omega_d");
    double ux = u.empty() ? 0.0 : u[0];
    double x = s[0], dx = s[1];
    // The cubic stiffness term bends the resonance curve and introduces the
    // amplitude-dependent peak that makes Duffing useful for nonlinear ETFE tests.
    double ddx = -delta*dx - alpha*x - beta*x*x*x + gamma_f*std::cos(omega*t) + ux;
    return {dx, ddx};
}

StateVector DuffingOscillator::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector DuffingOscillator::defaultInitialState() const { return {0.1, 0.0}; }

std::vector<ParamDescriptor> DuffingOscillator::parameterDescriptors() const {
    return {{"alpha_d","","Linear stiffness",-1.0,-5.0,5.0,0.1},
            {"beta_d","","Cubic stiffness",1.0,-5.0,5.0,0.1},
            {"delta_d","","Damping",0.3,0.0,5.0,0.01},
            {"gamma_d","","Forcing amplitude",0.37,0.0,10.0,0.01},
            {"omega_d","rad/s","Forcing frequency",1.2,0.01,10.0,0.01}};
}

std::vector<Preset> DuffingOscillator::presets() const {
    return {{"Chaotic","Classic chaotic regime",{{"alpha_d",-1.0},{"beta_d",1.0},{"delta_d",0.3},{"gamma_d",0.37},{"omega_d",1.2}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Hardening spring","Simple nonlinear",{{"alpha_d",1.0},{"beta_d",0.5},{"delta_d",0.1},{"gamma_d",1.0},{"omega_d",1.0}}}};
}
void DuffingOscillator::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DuffingOscillator::stateNames() const { return {"x","dx/dt"}; }
std::vector<std::string> DuffingOscillator::outputNames() const { return {"Position"}; }
std::vector<std::string> DuffingOscillator::inputNames() const { return {"External force"}; }

}  // namespace Simulation
