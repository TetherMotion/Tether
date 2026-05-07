#include "tether/simulation/systems/rotational/FlexibleShaft.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

FlexibleShaft::FlexibleShaft() {
    initParam("Jm", 0.01);  // motor inertia [kg·m²]
    initParam("Jl", 0.05);  // load inertia [kg·m²]
    initParam("Ks", 10.0);  // shaft stiffness [N·m/rad]
    initParam("Ds", 0.01);  // shaft damping [N·m·s/rad]
    initParam("bm", 0.001); // motor friction
    initParam("bl", 0.001); // load friction
}

StateVector FlexibleShaft::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Jm = params_.at("Jm"), Jl = params_.at("Jl");
    double Ks = params_.at("Ks"), Ds = params_.at("Ds");
    double bm = params_.at("bm"), bl = params_.at("bl");
    double tau = u.empty() ? 0.0 : u[0];

    double theta_m = s[0], omega_m = s[1], theta_l = s[2], omega_l = s[3];
    double delta = theta_m - theta_l;

    double dtheta_m = omega_m;
    // The shaft stores elastic energy in the twist angle `delta`, and the same
    // coupling torque appears with opposite sign on the motor and load sides.
    double domega_m = (tau - Ks*delta - Ds*(omega_m-omega_l) - bm*omega_m) / Jm;
    double dtheta_l = omega_l;
    double domega_l = (Ks*delta + Ds*(omega_m-omega_l) - bl*omega_l) / Jl;

    return {dtheta_m, domega_m, dtheta_l, domega_l};
}

StateVector FlexibleShaft::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[2]}; }
StateVector FlexibleShaft::defaultInitialState() const { return {0.0, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> FlexibleShaft::parameterDescriptors() const {
    return {{"Jm","kg·m²","Motor inertia",0.01,1e-5,1.0,1e-4},
            {"Jl","kg·m²","Load inertia",0.05,1e-5,10.0,1e-4},
            {"Ks","N·m/rad","Shaft stiffness",10.0,0.1,1000.0,0.1},
            {"Ds","N·m·s/rad","Shaft damping",0.01,0.0,1.0,0.001},
            {"bm","","Motor friction",0.001,0.0,0.1,1e-4},
            {"bl","","Load friction",0.001,0.0,0.1,1e-4}};
}

std::vector<Preset> FlexibleShaft::presets() const {
    return {{"Standard","Moderate flex",{{"Jm",0.01},{"Jl",0.05},{"Ks",10.0},{"Ds",0.01},{"bm",0.001},{"bl",0.001}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Very flexible","Low stiffness",{{"Jm",0.01},{"Jl",0.05},{"Ks",1.0},{"Ds",0.005},{"bm",0.001},{"bl",0.001}}}};
}
void FlexibleShaft::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> FlexibleShaft::stateNames() const { return {"θ_m","ω_m","θ_l","ω_l"}; }
std::vector<std::string> FlexibleShaft::outputNames() const { return {"Load position"}; }
std::vector<std::string> FlexibleShaft::inputNames() const { return {"Motor torque"}; }

}  // namespace Simulation
