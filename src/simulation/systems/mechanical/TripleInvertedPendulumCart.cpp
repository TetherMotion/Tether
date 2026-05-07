#include "tether/simulation/systems/mechanical/TripleInvertedPendulumCart.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

TripleInvertedPendulumCart::TripleInvertedPendulumCart() {
    initParam("M", 1.0); initParam("m1", 0.1); initParam("m2", 0.08); initParam("m3", 0.05);
    initParam("l1", 0.5); initParam("l2", 0.3); initParam("l3", 0.2);
    initParam("g", 9.81); initParam("b", 0.1);
}

StateVector TripleInvertedPendulumCart::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M = params_.at("M"), m1 = params_.at("m1"), m2 = params_.at("m2"), m3 = params_.at("m3");
    double l1 = params_.at("l1"), l2 = params_.at("l2"), l3 = params_.at("l3");
    double g = params_.at("g"), b = params_.at("b");
    double F = u.empty() ? 0.0 : u[0];

    double dx = s[1];
    double th1 = s[2], dth1 = s[3];
    double th2 = s[4], dth2 = s[5];
    double th3 = s[6], dth3 = s[7];

    // Simplified dynamics using small angle linearization for tractability
    double Mt = M + m1 + m2 + m3;
    double ddx = (F - b*dx + (0.5*m1+m2+m3)*l1*g*std::sin(th1) + (0.5*m2+m3)*l2*g*std::sin(th2) + 0.5*m3*l3*g*std::sin(th3)) / Mt;
    double ddth1 = ((Mt)*g*std::sin(th1) - std::cos(th1)*F) / ((m1/3.0+m2+m3)*l1);
    double ddth2 = ((0.5*m2+m3)*g*std::sin(th2)) / (m2*l2/3.0 + m3*l2);
    double ddth3 = (0.5*m3*g*std::sin(th3)) / (m3*l3/3.0);

    return {dx, ddx, dth1, ddth1, dth2, ddth2, dth3, ddth3};
}

StateVector TripleInvertedPendulumCart::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[2], s[4], s[6]};
}

StateVector TripleInvertedPendulumCart::defaultInitialState() const { return {0,0,0.02,0,0.02,0,0.02,0}; }

std::vector<ParamDescriptor> TripleInvertedPendulumCart::parameterDescriptors() const {
    return {{"M","kg","Cart mass",1.0,0.01,100.0,0.1},{"m1","kg","Link 1 mass",0.1,0.001,10.0,0.01},
            {"m2","kg","Link 2 mass",0.08,0.001,10.0,0.01},{"m3","kg","Link 3 mass",0.05,0.001,10.0,0.01},
            {"l1","m","Link 1 len",0.5,0.01,5.0,0.01},{"l2","m","Link 2 len",0.3,0.01,5.0,0.01},
            {"l3","m","Link 3 len",0.2,0.01,5.0,0.01},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"b","N·s/m","Friction",0.1,0.0,10.0,0.01}};
}

std::vector<Preset> TripleInvertedPendulumCart::presets() const {
    return {{"Standard","Default params",{{"M",1.0},{"m1",0.1},{"m2",0.08},{"m3",0.05},{"l1",0.5},{"l2",0.3},{"l3",0.2},{"g",9.81},{"b",0.1}}}};
}

void TripleInvertedPendulumCart::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> TripleInvertedPendulumCart::stateNames() const { return {"x","dx/dt","θ₁","dθ₁/dt","θ₂","dθ₂/dt","θ₃","dθ₃/dt"}; }
std::vector<std::string> TripleInvertedPendulumCart::outputNames() const { return {"Cart pos","θ₁","θ₂","θ₃"}; }
std::vector<std::string> TripleInvertedPendulumCart::inputNames() const { return {"Force"}; }

}  // namespace Simulation
