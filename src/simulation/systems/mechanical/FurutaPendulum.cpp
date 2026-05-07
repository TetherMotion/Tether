#include "tether/simulation/systems/mechanical/FurutaPendulum.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

FurutaPendulum::FurutaPendulum() {
    initParam("J0", 0.01);  // arm inertia
    initParam("Jp", 0.005); // pendulum inertia
    initParam("m", 0.1);    // pendulum mass
    initParam("l", 0.2);    // pendulum CoM distance
    initParam("L", 0.15);   // arm length
    initParam("g", 9.81);
    initParam("b0", 0.01); initParam("bp", 0.005);
}

StateVector FurutaPendulum::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double J0 = params_.at("J0"), Jp = params_.at("Jp"), m = params_.at("m");
    double l = params_.at("l"), L = params_.at("L"), g = params_.at("g");
    double b0 = params_.at("b0"), bp = params_.at("bp");
    double tau = u.empty() ? 0.0 : u[0];

    double th0 = s[0], dth0 = s[1], alpha = s[2], dalpha = s[3];
    double sa = std::sin(alpha), ca = std::cos(alpha);

    double d11 = J0 + m*L*L + m*l*l*sa*sa;
    double d12 = m*L*l*ca;
    double d22 = Jp + m*l*l;

    double h1 = 2.0*m*l*l*sa*ca*dth0*dalpha + m*L*l*sa*dalpha*dalpha;
    double h2 = -m*l*l*sa*ca*dth0*dth0;

    double det = d11*d22 - d12*d12;
    if (std::abs(det) < 1e-12) det = 1e-12;

    double ddth0 = (d22*(tau - b0*dth0 - h1) - d12*(m*g*l*sa - bp*dalpha - h2)) / det;
    double ddalpha = (-d12*(tau - b0*dth0 - h1) + d11*(m*g*l*sa - bp*dalpha - h2)) / det;

    return {dth0, ddth0, dalpha, ddalpha};
}

StateVector FurutaPendulum::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector FurutaPendulum::defaultInitialState() const { return {0, 0, 0.1, 0}; }

std::vector<ParamDescriptor> FurutaPendulum::parameterDescriptors() const {
    return {{"J0","kg·m²","Arm inertia",0.01,0.001,1.0,0.001},{"Jp","kg·m²","Pend inertia",0.005,0.001,1.0,0.001},
            {"m","kg","Pend mass",0.1,0.01,5.0,0.01},{"l","m","Pend CoM dist",0.2,0.01,2.0,0.01},
            {"L","m","Arm length",0.15,0.01,1.0,0.01},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"b0","N·m·s","Arm damp",0.01,0.0,1.0,0.001},{"bp","N·m·s","Pend damp",0.005,0.0,1.0,0.001}};
}

std::vector<Preset> FurutaPendulum::presets() const {
    return {{"Quanser QUBE","Lab system",{{"J0",0.01},{"Jp",0.005},{"m",0.1},{"l",0.2},{"L",0.15},{"g",9.81},{"b0",0.01},{"bp",0.005}}}};
}
void FurutaPendulum::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> FurutaPendulum::stateNames() const { return {"θ_arm","dθ_arm/dt","α","dα/dt"}; }
std::vector<std::string> FurutaPendulum::outputNames() const { return {"Arm angle","Pendulum angle"}; }
std::vector<std::string> FurutaPendulum::inputNames() const { return {"Motor torque"}; }

}  // namespace Simulation
