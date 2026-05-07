#include "tether/simulation/systems/mechanical/SegwayRobot.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

SegwayRobot::SegwayRobot() {
    initParam("M", 10.0); initParam("m", 70.0); initParam("l", 0.5);
    initParam("R", 0.2); initParam("Jw", 0.01); initParam("Jp", 2.0);
    initParam("g", 9.81); initParam("b", 0.5);
}

StateVector SegwayRobot::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M_w = params_.at("M"), m_p = params_.at("m"), l = params_.at("l");
    double R = params_.at("R"), g = params_.at("g"), b = params_.at("b");
    double tau = u.empty() ? 0.0 : u[0];

    double dx = s[1], th = s[2], dth = s[3];
    double sinT = std::sin(th), cosT = std::cos(th);
    double Mt = M_w + m_p;

    double ddx = (tau/R - b*dx + m_p*l*dth*dth*sinT - m_p*g*sinT*cosT) / (Mt - m_p*cosT*cosT);
    double ddth = (Mt*g*sinT - cosT*(tau/R - b*dx + m_p*l*dth*dth*sinT)) / (l * (Mt - m_p*cosT*cosT));

    return {dx, ddx, dth, ddth};
}

StateVector SegwayRobot::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector SegwayRobot::defaultInitialState() const { return {0, 0, 0.05, 0}; }

std::vector<ParamDescriptor> SegwayRobot::parameterDescriptors() const {
    return {{"M","kg","Wheel+frame mass",10.0,1.0,50.0,1.0},{"m","kg","Rider mass",70.0,10.0,150.0,1.0},
            {"l","m","CoM height",0.5,0.1,2.0,0.01},{"R","m","Wheel radius",0.2,0.05,0.5,0.01},
            {"g","m/s²","Gravity",9.81,0.1,20.0,0.01},{"b","N·s/m","Friction",0.5,0.0,10.0,0.1}};
}

std::vector<Preset> SegwayRobot::presets() const {
    return {{"Standard rider","70kg person",{{"M",10.0},{"m",70.0},{"l",0.5},{"R",0.2},{"g",9.81},{"b",0.5}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Light rider","50kg person",{{"M",10.0},{"m",50.0},{"l",0.45},{"R",0.2},{"g",9.81},{"b",0.5}}}};
}
void SegwayRobot::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> SegwayRobot::stateNames() const { return {"x","dx/dt","θ","dθ/dt"}; }
std::vector<std::string> SegwayRobot::outputNames() const { return {"Position","Lean angle"}; }
std::vector<std::string> SegwayRobot::inputNames() const { return {"Wheel torque"}; }

}  // namespace Simulation
