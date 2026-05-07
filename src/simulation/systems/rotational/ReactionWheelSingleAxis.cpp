#include "tether/simulation/systems/rotational/ReactionWheelSingleAxis.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ReactionWheelSingleAxis::ReactionWheelSingleAxis() {
    initParam("Js", 10.0);    // spacecraft inertia [kg·m²]
    initParam("Jw", 0.01);    // wheel inertia [kg·m²]
    initParam("Kt_rw", 0.01); // motor torque constant
    initParam("R_rw", 1.0);   // motor resistance [Ω]
    initParam("Ke_rw", 0.01); // back-EMF constant
    initParam("bw", 1e-4);    // wheel friction
}

StateVector ReactionWheelSingleAxis::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Js = params_.at("Js"), Jw = params_.at("Jw");
    double Kt = params_.at("Kt_rw"), R = params_.at("R_rw"), Ke = params_.at("Ke_rw");
    double bw = params_.at("bw");
    double V = u.empty() ? 0.0 : u[0];

    double theta = s[0], omega = s[1], omega_w = s[2], i = s[3];

    double tau_motor = Kt * i;
    double dtheta = omega;
    double domega = -tau_motor / Js; // reaction torque on spacecraft
    double domega_w = (tau_motor - bw * omega_w) / Jw;
    double di = (V - R*i - Ke*omega_w) / 0.001; // fast electrical dynamics

    return {dtheta, domega, domega_w, di};
}

StateVector ReactionWheelSingleAxis::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector ReactionWheelSingleAxis::defaultInitialState() const { return {0.1, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> ReactionWheelSingleAxis::parameterDescriptors() const {
    return {{"Js","kg·m²","Spacecraft inertia",10.0,0.1,1000.0,0.1},
            {"Jw","kg·m²","Wheel inertia",0.01,1e-5,1.0,1e-4},
            {"Kt_rw","N·m/A","Motor torque const",0.01,1e-4,1.0,1e-4},
            {"R_rw","Ω","Motor resistance",1.0,0.01,100.0,0.01},
            {"Ke_rw","V·s/rad","Back-EMF const",0.01,1e-4,1.0,1e-4},
            {"bw","","Wheel friction",1e-4,0.0,0.01,1e-5}};
}

std::vector<Preset> ReactionWheelSingleAxis::presets() const {
    return {{"CubeSat","Small satellite",{{"Js",0.5},{"Jw",0.001},{"Kt_rw",0.005},{"R_rw",2.0},{"Ke_rw",0.005},{"bw",1e-5}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Large sat","GEO satellite",{{"Js",100.0},{"Jw",0.05},{"Kt_rw",0.05},{"R_rw",0.5},{"Ke_rw",0.05},{"bw",1e-3}}}};
}
void ReactionWheelSingleAxis::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ReactionWheelSingleAxis::stateNames() const { return {"θ","ω","ω_w","i"}; }
std::vector<std::string> ReactionWheelSingleAxis::outputNames() const { return {"Attitude angle"}; }
std::vector<std::string> ReactionWheelSingleAxis::inputNames() const { return {"Motor voltage"}; }

}  // namespace Simulation
