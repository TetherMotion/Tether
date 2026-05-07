#include "tether/simulation/systems/aerospace/BicycleLean.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BicycleLean::BicycleLean() {
    initParam("v", 5.0);     // forward speed [m/s]
    initParam("h_cg", 1.0);  // CG height [m]
    initParam("a", 0.5);     // front to CG [m]
    initParam("b_bike", 1.0);// wheelbase [m]
    initParam("g", 9.81);
    initParam("m_bike", 80.0);
}

StateVector BicycleLean::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double v = params_.at("v"), h = params_.at("h_cg"), wb = params_.at("b_bike"), g = params_.at("g");
    double steer = u.empty() ? 0.0 : u[0]; // steer torque

    double phi = s[0], dphi = s[1]; // lean angle
    double delta = s[2], ddelta_st = s[3]; // steer angle
    (void)ddelta_st;

    // Simplified lean dynamics: lean acceleration depends on gravity, speed, steer
    double ddphi = g/h * std::sin(phi) - v*v/(wb*h) * delta;
    double ddelta = steer; // simplified: direct steer control

    return {dphi, ddphi, ddelta_st, ddelta};
}

StateVector BicycleLean::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector BicycleLean::defaultInitialState() const { return {0.05, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> BicycleLean::parameterDescriptors() const {
    return {{"v","m/s","Speed",5.0,0.5,30.0,0.1},{"h_cg","m","CG height",1.0,0.3,2.0,0.01},
            {"a","m","Front to CG",0.5,0.1,1.5,0.01},{"b_bike","m","Wheelbase",1.0,0.5,2.5,0.01},
            {"g","m/s²","Gravity",9.81,0.0,20.0,0.01},{"m_bike","kg","Mass",80.0,10.0,200.0,1.0}};
}

std::vector<Preset> BicycleLean::presets() const {
    return {{"Low speed","Unstable",{{"v",2.0},{"h_cg",1.0},{"a",0.5},{"b_bike",1.0},{"g",9.81},{"m_bike",80.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"High speed","Stable",{{"v",10.0},{"h_cg",1.0},{"a",0.5},{"b_bike",1.0},{"g",9.81},{"m_bike",80.0}}}};
}
void BicycleLean::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BicycleLean::stateNames() const { return {"φ","dφ","δ","dδ"}; }
std::vector<std::string> BicycleLean::outputNames() const { return {"Lean angle"}; }
std::vector<std::string> BicycleLean::inputNames() const { return {"Steer torque"}; }

}  // namespace Simulation
