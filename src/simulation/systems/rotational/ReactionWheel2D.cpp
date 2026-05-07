#include "tether/simulation/systems/rotational/ReactionWheel2D.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ReactionWheel2D::ReactionWheel2D() {
    initParam("Js_x", 10.0); initParam("Js_y", 10.0);
    initParam("Jw", 0.01);
    initParam("Kt_2d", 0.01); initParam("bw_2d", 1e-4);
}

StateVector ReactionWheel2D::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Jx = params_.at("Js_x"), Jy = params_.at("Js_y"), Jw = params_.at("Jw");
    double Kt = params_.at("Kt_2d"), bw = params_.at("bw_2d");
    double tau_x = u.size()>0 ? u[0] : 0.0;
    double tau_y = u.size()>1 ? u[1] : 0.0;

    double theta_x = s[0], omega_x = s[1], omega_wx = s[2];
    double theta_y = s[3], omega_y = s[4], omega_wy = s[5];

    return {omega_x,
            -Kt*tau_x/Jx,
            (Kt*tau_x - bw*omega_wx)/Jw,
            omega_y,
            -Kt*tau_y/Jy,
            (Kt*tau_y - bw*omega_wy)/Jw};
}

StateVector ReactionWheel2D::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[3]}; }
StateVector ReactionWheel2D::defaultInitialState() const { return {0.05, 0.0, 0.0, -0.03, 0.0, 0.0}; }

std::vector<ParamDescriptor> ReactionWheel2D::parameterDescriptors() const {
    return {{"Js_x","kg·m²","S/C inertia X",10.0,0.1,1000.0,0.1},
            {"Js_y","kg·m²","S/C inertia Y",10.0,0.1,1000.0,0.1},
            {"Jw","kg·m²","Wheel inertia",0.01,1e-5,1.0,1e-4},
            {"Kt_2d","N·m/A","Torque const",0.01,1e-4,1.0,1e-4},
            {"bw_2d","","Wheel friction",1e-4,0.0,0.01,1e-5}};
}

std::vector<Preset> ReactionWheel2D::presets() const {
    return {{"Symmetric","Equal inertia",{{"Js_x",10.0},{"Js_y",10.0},{"Jw",0.01},{"Kt_2d",0.01},{"bw_2d",1e-4}}}};
}
void ReactionWheel2D::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ReactionWheel2D::stateNames() const { return {"θ_x","ω_x","ω_wx","θ_y","ω_y","ω_wy"}; }
std::vector<std::string> ReactionWheel2D::outputNames() const { return {"Pitch","Yaw"}; }
std::vector<std::string> ReactionWheel2D::inputNames() const { return {"Torque X","Torque Y"}; }

}  // namespace Simulation
