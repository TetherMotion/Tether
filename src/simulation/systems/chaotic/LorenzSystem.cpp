#include "tether/simulation/systems/chaotic/LorenzSystem.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

LorenzSystem::LorenzSystem() {
    initParam("sigma", 10.0); initParam("rho", 28.0); initParam("beta_l", 8.0/3.0);
}

StateVector LorenzSystem::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double sigma = params_.at("sigma"), rho = params_.at("rho"), beta_l = params_.at("beta_l");
    double ux = u.empty() ? 0.0 : u[0];
    double x = s[0], y = s[1], z = s[2];
    return {sigma*(y - x) + ux, x*(rho - z) - y, x*y - beta_l*z};
}

StateVector LorenzSystem::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1], s[2]}; }
StateVector LorenzSystem::defaultInitialState() const { return {1.0, 1.0, 1.0}; }

std::vector<ParamDescriptor> LorenzSystem::parameterDescriptors() const {
    return {{"sigma","","Prandtl number",10.0,0.1,50.0,0.1},
            {"rho","","Rayleigh number",28.0,0.1,100.0,0.1},
            {"beta_l","","Geometric factor",8.0/3.0,0.1,10.0,0.01}};
}

std::vector<Preset> LorenzSystem::presets() const {
    return {{"Classic","σ=10,ρ=28,β=8/3",{{"sigma",10.0},{"rho",28.0},{"beta_l",8.0/3.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Periodic","ρ=15",{{"sigma",10.0},{"rho",15.0},{"beta_l",8.0/3.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Stable","ρ=0.5",{{"sigma",10.0},{"rho",0.5},{"beta_l",8.0/3.0}}}};
}
void LorenzSystem::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> LorenzSystem::stateNames() const { return {"x","y","z"}; }
std::vector<std::string> LorenzSystem::outputNames() const { return {"x","y","z"}; }
std::vector<std::string> LorenzSystem::inputNames() const { return {"Perturbation"}; }

}  // namespace Simulation
