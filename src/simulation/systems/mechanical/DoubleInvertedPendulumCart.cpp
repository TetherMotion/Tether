#include "tether/simulation/systems/mechanical/DoubleInvertedPendulumCart.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DoubleInvertedPendulumCart::DoubleInvertedPendulumCart() {
    initParam("M", 1.0); initParam("m1", 0.1); initParam("m2", 0.1);
    initParam("l1", 0.5); initParam("l2", 0.3);
    initParam("g", 9.81); initParam("b", 0.1);
}

StateVector DoubleInvertedPendulumCart::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M = params_.at("M"), m1 = params_.at("m1"), m2 = params_.at("m2");
    double l1 = params_.at("l1"), l2 = params_.at("l2");
    double g = params_.at("g"), b = params_.at("b");
    double F = u.empty() ? 0.0 : u[0];

    double dx = s[1], th1 = s[2], dth1 = s[3], th2 = s[4], dth2 = s[5];
    double s1 = std::sin(th1), c1 = std::cos(th1);
    double s2 = std::sin(th2), c2 = std::cos(th2);
    double s12 = std::sin(th1 - th2), c12 = std::cos(th1 - th2);

    // Simplified Lagrangian dynamics
    double Mt = M + m1 + m2;
    double d11 = Mt;
    double d12 = (0.5*m1 + m2)*l1*c1;
    double d13 = 0.5*m2*l2*c2;
    double d22 = (m1/3.0 + m2)*l1*l1;
    double d23 = 0.5*m2*l1*l2*c12;
    double d33 = m2*l2*l2/3.0;

    double h1 = -(0.5*m1 + m2)*l1*dth1*dth1*s1 - 0.5*m2*l2*dth2*dth2*s2;
    double h2 = 0.5*m2*l1*l2*dth2*dth2*s12 + (0.5*m1 + m2)*g*l1*s1;
    double h3 = -0.5*m2*l1*l2*dth1*dth1*s12 + 0.5*m2*g*l2*s2;

    // Solve 3x3 system: [d11 d12 d13; d12 d22 d23; d13 d23 d33] * [ddx; ddth1; ddth2] = [F-b*dx+h1; -h2; -h3]
    double rhs1 = F - b*dx + h1, rhs2 = -h2, rhs3 = -h3;

    // Cramer's rule (simplified for 3x3)
    double det = d11*(d22*d33 - d23*d23) - d12*(d12*d33 - d23*d13) + d13*(d12*d23 - d22*d13);
    if (std::abs(det) < 1e-12) det = 1e-12;

    double ddx = (rhs1*(d22*d33-d23*d23) - d12*(rhs2*d33-d23*rhs3) + d13*(rhs2*d23-d22*rhs3)) / det;
    double ddth1 = (d11*(rhs2*d33-d23*rhs3) - rhs1*(d12*d33-d23*d13) + d13*(d12*rhs3-rhs2*d13)) / det;
    double ddth2 = (d11*(d22*rhs3-rhs2*d23) - d12*(d12*rhs3-rhs2*d13) + rhs1*(d12*d23-d22*d13)) / det;

    return {dx, ddx, dth1, ddth1, dth2, ddth2};
}

StateVector DoubleInvertedPendulumCart::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[2], s[4]};
}

StateVector DoubleInvertedPendulumCart::defaultInitialState() const { return {0,0,0.05,0,0.05,0}; }

std::vector<ParamDescriptor> DoubleInvertedPendulumCart::parameterDescriptors() const {
    return {{"M","kg","Cart mass",1.0,0.01,100.0,0.1},{"m1","kg","Link 1 mass",0.1,0.001,10.0,0.01},
            {"m2","kg","Link 2 mass",0.1,0.001,10.0,0.01},{"l1","m","Link 1 length",0.5,0.01,5.0,0.01},
            {"l2","m","Link 2 length",0.3,0.01,5.0,0.01},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"b","N·s/m","Cart friction",0.1,0.0,10.0,0.01}};
}

std::vector<Preset> DoubleInvertedPendulumCart::presets() const {
    return {{"Standard", "Classic params", {{"M",1.0},{"m1",0.1},{"m2",0.1},{"l1",0.5},{"l2",0.3},{"g",9.81},{"b",0.1}}}};
}

void DoubleInvertedPendulumCart::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DoubleInvertedPendulumCart::stateNames() const { return {"x","dx/dt","θ₁","dθ₁/dt","θ₂","dθ₂/dt"}; }
std::vector<std::string> DoubleInvertedPendulumCart::outputNames() const { return {"Cart pos","θ₁","θ₂"}; }
std::vector<std::string> DoubleInvertedPendulumCart::inputNames() const { return {"Force"}; }

}  // namespace Simulation
