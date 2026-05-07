#include "tether/simulation/systems/rotational/ControlMomentGyroscope.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ControlMomentGyroscope::ControlMomentGyroscope() {
    initParam("Js_cmg", 50.0);   // spacecraft inertia
    initParam("hw", 10.0);       // wheel angular momentum [N·m·s]
    initParam("Jg", 0.1);        // gimbal inertia [kg·m²]
    initParam("bg", 0.01);       // gimbal friction
}

StateVector ControlMomentGyroscope::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Js = params_.at("Js_cmg"), hw = params_.at("hw");
    double Jg = params_.at("Jg"), bg = params_.at("bg");
    double tau_g = u.empty() ? 0.0 : u[0]; // gimbal torque

    double theta = s[0], omega = s[1]; // spacecraft
    double delta_g = s[2], ddelta = s[3]; // gimbal angle, rate

    double output_torque = hw * ddelta * std::cos(delta_g); // CMG output torque

    double dtheta = omega;
    double domega = -output_torque / Js;
    double ddelta_g = ddelta;
    double dddelta = (tau_g - bg*ddelta) / Jg;

    return {dtheta, domega, ddelta_g, dddelta};
}

StateVector ControlMomentGyroscope::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector ControlMomentGyroscope::defaultInitialState() const { return {0.1, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> ControlMomentGyroscope::parameterDescriptors() const {
    return {{"Js_cmg","kg·m²","S/C inertia",50.0,1.0,1000.0,1.0},
            {"hw","N·m·s","Wheel momentum",10.0,0.1,100.0,0.1},
            {"Jg","kg·m²","Gimbal inertia",0.1,0.001,10.0,0.001},
            {"bg","","Gimbal friction",0.01,0.0,1.0,0.001}};
}

std::vector<Preset> ControlMomentGyroscope::presets() const {
    return {{"Standard","Typical CMG",{{"Js_cmg",50.0},{"hw",10.0},{"Jg",0.1},{"bg",0.01}}}};
}
void ControlMomentGyroscope::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ControlMomentGyroscope::stateNames() const { return {"θ","ω","δ_g","dδ_g/dt"}; }
std::vector<std::string> ControlMomentGyroscope::outputNames() const { return {"Attitude"}; }
std::vector<std::string> ControlMomentGyroscope::inputNames() const { return {"Gimbal torque"}; }

}  // namespace Simulation
