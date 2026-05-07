#include "tether/simulation/systems/mechanical/DoublePendulumGantryCrane.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DoublePendulumGantryCrane::DoublePendulumGantryCrane() {
    initParam("M", 10.0); initParam("m1", 3.0); initParam("m2", 2.0);
    initParam("l1", 2.0); initParam("l2", 1.5);
    initParam("g", 9.81); initParam("b", 1.0);
}

StateVector DoublePendulumGantryCrane::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M = params_.at("M"), m1 = params_.at("m1"), m2 = params_.at("m2");
    double l1 = params_.at("l1"), l2 = params_.at("l2");
    double g = params_.at("g"), b = params_.at("b");
    double F = u.empty() ? 0.0 : u[0];

    double dx = s[1], th1 = s[2], dth1 = s[3], th2 = s[4], dth2 = s[5];
    double s1 = std::sin(th1), c1 = std::cos(th1);
    double s2 = std::sin(th2), c2 = std::cos(th2);
    double s12 = std::sin(th1 - th2), c12 = std::cos(th1 - th2);

    double Mt = M + m1 + m2;
    // Simplified dynamics
    double ddx = (F - b*dx + (m1+m2)*l1*dth1*dth1*s1 + m2*l2*dth2*dth2*s2) / Mt;
    double ddth1 = (-g*s1 - ddx*c1 + l2*dth2*dth2*s12*m2/(m1+m2)) / l1;
    double ddth2 = (-g*s2 - ddx*c2 - l1*dth1*dth1*s12) / l2;

    return {dx, ddx, dth1, ddth1, dth2, ddth2};
}

StateVector DoublePendulumGantryCrane::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2], s[4]}; }
StateVector DoublePendulumGantryCrane::defaultInitialState() const { return {0, 0, 0.1, 0, 0.1, 0}; }

std::vector<ParamDescriptor> DoublePendulumGantryCrane::parameterDescriptors() const {
    return {{"M","kg","Trolley mass",10.0,1.0,100.0,1.0},{"m1","kg","Segment 1 mass",3.0,0.1,50.0,0.1},
            {"m2","kg","Segment 2 mass",2.0,0.1,50.0,0.1},{"l1","m","Segment 1 length",2.0,0.1,20.0,0.1},
            {"l2","m","Segment 2 length",1.5,0.1,20.0,0.1},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"b","N·s/m","Friction",1.0,0.0,20.0,0.1}};
}

std::vector<Preset> DoublePendulumGantryCrane::presets() const {
    return {{"Standard","Default",{{"M",10.0},{"m1",3.0},{"m2",2.0},{"l1",2.0},{"l2",1.5},{"g",9.81},{"b",1.0}}}};
}
void DoublePendulumGantryCrane::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DoublePendulumGantryCrane::stateNames() const { return {"x","dx/dt","θ₁","dθ₁/dt","θ₂","dθ₂/dt"}; }
std::vector<std::string> DoublePendulumGantryCrane::outputNames() const { return {"Trolley pos","θ₁","θ₂"}; }
std::vector<std::string> DoublePendulumGantryCrane::inputNames() const { return {"Trolley force"}; }

}  // namespace Simulation
