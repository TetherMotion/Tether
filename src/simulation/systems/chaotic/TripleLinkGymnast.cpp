#include "tether/simulation/systems/chaotic/TripleLinkGymnast.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

TripleLinkGymnast::TripleLinkGymnast() {
    initParam("m1", 1.0); initParam("m2", 1.0); initParam("m3", 0.5);
    initParam("l1", 0.5); initParam("l2", 0.5); initParam("l3", 0.3);
    initParam("g", 9.81);
    initParam("b1", 0.01); initParam("b2", 0.01); initParam("b3", 0.01);
}

StateVector TripleLinkGymnast::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m1 = params_.at("m1"), m2 = params_.at("m2"), m3 = params_.at("m3");
    double l1 = params_.at("l1"), l2 = params_.at("l2"), l3 = params_.at("l3");
    double g = params_.at("g");
    double b1 = params_.at("b1"), b2 = params_.at("b2"), b3 = params_.at("b3");
    double tau = u.empty() ? 0.0 : u[0]; // torque on joint 1

    double th1 = s[0], th2 = s[1], th3 = s[2];
    double dth1 = s[3], dth2 = s[4], dth3 = s[5];

    // Simplified decoupled model (full Lagrangian would require 3x3 mass matrix inversion)
    double lc1 = l1/2.0, lc2 = l2/2.0, lc3 = l3/2.0;
    double I1 = m1*l1*l1/3.0, I2 = m2*l2*l2/3.0, I3 = m3*l3*l3/3.0;

    // Gravity torques
    double tau_g1 = -(m1*lc1 + m2*l1 + m3*l1) * g * std::sin(th1);
    double tau_g2 = -(m2*lc2 + m3*l2) * g * std::sin(th2);
    double tau_g3 = -m3*lc3 * g * std::sin(th3);

    // Coupling terms (simplified)
    double c12 = m2*l1*lc2*std::sin(th1-th2) * dth2*dth2;
    double c21 = -m2*l1*lc2*std::sin(th1-th2) * dth1*dth1;
    double c23 = m3*l2*lc3*std::sin(th2-th3) * dth3*dth3;
    double c32 = -m3*l2*lc3*std::sin(th2-th3) * dth2*dth2;

    double M1_eff = I1 + (m2+m3)*l1*l1;
    double M2_eff = I2 + m3*l2*l2;
    double M3_eff = I3;

    double ddth1 = (tau + tau_g1 + c12 - b1*dth1) / M1_eff;
    double ddth2 = (tau_g2 + c21 + c23 - b2*dth2) / M2_eff;
    double ddth3 = (tau_g3 + c32 - b3*dth3) / M3_eff;

    return {dth1, dth2, dth3, ddth1, ddth2, ddth3};
}

StateVector TripleLinkGymnast::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[1], s[2]};
}
StateVector TripleLinkGymnast::defaultInitialState() const { return {3.0, 3.0, 3.0, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> TripleLinkGymnast::parameterDescriptors() const {
    return {{"m1","kg","Mass 1",1.0,0.1,10.0,0.1},{"m2","kg","Mass 2",1.0,0.1,10.0,0.1},
            {"m3","kg","Mass 3",0.5,0.1,10.0,0.1},{"l1","m","Length 1",0.5,0.1,2.0,0.01},
            {"l2","m","Length 2",0.5,0.1,2.0,0.01},{"l3","m","Length 3",0.3,0.1,2.0,0.01},
            {"g","m/s²","Gravity",9.81,0.0,20.0,0.01},
            {"b1","","Damping 1",0.01,0.0,1.0,0.001},{"b2","","Damping 2",0.01,0.0,1.0,0.001},
            {"b3","","Damping 3",0.01,0.0,1.0,0.001}};
}

std::vector<Preset> TripleLinkGymnast::presets() const {
    return {{"Standard","Equal masses",
             {{"m1",1.0},{"m2",1.0},{"m3",0.5},{"l1",0.5},{"l2",0.5},{"l3",0.3}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"g",9.81},{"b1",0.01},{"b2",0.01},{"b3",0.01}}}};
}
void TripleLinkGymnast::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> TripleLinkGymnast::stateNames() const { return {"θ₁","θ₂","θ₃","dθ₁/dt","dθ₂/dt","dθ₃/dt"}; }
std::vector<std::string> TripleLinkGymnast::outputNames() const { return {"θ₁","θ₂","θ₃"}; }
std::vector<std::string> TripleLinkGymnast::inputNames() const { return {"Joint torque"}; }

}  // namespace Simulation
