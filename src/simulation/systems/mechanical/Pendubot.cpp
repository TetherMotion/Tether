#include "tether/simulation/systems/mechanical/Pendubot.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

Pendubot::Pendubot() {
    initParam("m1", 1.0); initParam("m2", 1.0);
    initParam("l1", 1.0); initParam("l2", 1.0);
    initParam("lc1", 0.5); initParam("lc2", 0.5);
    initParam("I1", 0.083); initParam("I2", 0.083);
    initParam("g", 9.81); initParam("b1", 0.1); initParam("b2", 0.1);
}

StateVector Pendubot::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m1 = params_.at("m1"), m2 = params_.at("m2");
    double l1 = params_.at("l1"), lc1 = params_.at("lc1"), lc2 = params_.at("lc2");
    double I1 = params_.at("I1"), I2 = params_.at("I2");
    double g = params_.at("g"), b1 = params_.at("b1"), b2 = params_.at("b2");
    double tau = u.empty() ? 0.0 : u[0]; // torque at shoulder only

    double th1 = s[0], dth1 = s[1], th2 = s[2], dth2 = s[3];
    double c2 = std::cos(th2), s2 = std::sin(th2);
    double s1 = std::sin(th1), s12 = std::sin(th1 + th2);

    double d11 = I1 + I2 + m2*l1*l1 + 2.0*m2*l1*lc2*c2;
    double d12 = I2 + m2*l1*lc2*c2;
    double d22 = I2;
    double h1 = -m2*l1*lc2*s2*dth2*(2.0*dth1 + dth2);
    double h2 = m2*l1*lc2*s2*dth1*dth1;
    double phi1 = (m1*lc1 + m2*l1)*g*s1 + m2*lc2*g*s12;
    double phi2 = m2*lc2*g*s12;

    double det = d11*d22 - d12*d12;
    if (std::abs(det) < 1e-12) det = 1e-12;

    double ddth1 = (d22*(tau - b1*dth1 - h1 - phi1) - d12*(-b2*dth2 - h2 - phi2)) / det;
    double ddth2 = (-d12*(tau - b1*dth1 - h1 - phi1) + d11*(-b2*dth2 - h2 - phi2)) / det;

    return {dth1, ddth1, dth2, ddth2};
}

StateVector Pendubot::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector Pendubot::defaultInitialState() const { return {M_PI, 0, 0, 0}; } // hanging down

std::vector<ParamDescriptor> Pendubot::parameterDescriptors() const {
    return {{"m1","kg","Link 1 mass",1.0,0.01,10.0,0.1},{"m2","kg","Link 2 mass",1.0,0.01,10.0,0.1},
            {"l1","m","Link 1 length",1.0,0.1,5.0,0.1},{"l2","m","Link 2 length",1.0,0.1,5.0,0.1},
            {"lc1","m","CoM link 1",0.5,0.01,5.0,0.01},{"lc2","m","CoM link 2",0.5,0.01,5.0,0.01},
            {"I1","kg·m²","Inertia 1",0.083,0.001,10.0,0.001},{"I2","kg·m²","Inertia 2",0.083,0.001,10.0,0.001},
            {"g","m/s²","Gravity",9.81,0.1,20.0,0.01},{"b1","N·m·s","Damping 1",0.1,0.0,5.0,0.01},{"b2","N·m·s","Damping 2",0.1,0.0,5.0,0.01}};
}

std::vector<Preset> Pendubot::presets() const {
    return {{"Standard","Default",{{"m1",1.0},{"m2",1.0},{"l1",1.0},{"l2",1.0},{"lc1",0.5},{"lc2",0.5},{"I1",0.083},{"I2",0.083},{"g",9.81},{"b1",0.1},{"b2",0.1}}}};
}
void Pendubot::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> Pendubot::stateNames() const { return {"θ₁","dθ₁/dt","θ₂","dθ₂/dt"}; }
std::vector<std::string> Pendubot::outputNames() const { return {"θ₁","θ₂"}; }
std::vector<std::string> Pendubot::inputNames() const { return {"Shoulder torque"}; }

}  // namespace Simulation
