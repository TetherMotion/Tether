#include "tether/simulation/systems/mechanical/InvertedPendulumCart.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

InvertedPendulumCart::InvertedPendulumCart() {
    initParam("M", 1.0);    // cart mass [kg]
    initParam("m", 0.1);    // pendulum mass [kg]
    initParam("l", 0.5);    // pendulum length [m]
    initParam("g", 9.81);   // gravity [m/s²]
    initParam("b", 0.1);    // cart friction [N·s/m]
}

StateVector InvertedPendulumCart::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M = params_.at("M"), m = params_.at("m"), l = params_.at("l");
    double g = params_.at("g"), b = params_.at("b");
    double F = u.empty() ? 0.0 : u[0];
    // double x = s[0], dx = s[1], th = s[2], dth = s[3]; // x not used
    double dx = s[1], th = s[2], dth = s[3];
    double sinT = std::sin(th), cosT = std::cos(th);
    double denom = M + m - m * cosT * cosT;
    double ddx = (F - b*dx + m*l*dth*dth*sinT - m*g*sinT*cosT) / denom;
    double ddth = ((M+m)*g*sinT - cosT*(F - b*dx + m*l*dth*dth*sinT)) / (l * denom);
    return {dx, ddx, dth, ddth};
}

StateVector InvertedPendulumCart::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[2]};
}

StateVector InvertedPendulumCart::defaultInitialState() const { return {0.0, 0.0, 0.1, 0.0}; }

std::vector<ParamDescriptor> InvertedPendulumCart::parameterDescriptors() const {
    return {{"M", "kg", "Cart mass", 1.0, 0.01, 100.0, 0.1},
            {"m", "kg", "Pendulum mass", 0.1, 0.001, 10.0, 0.01},
            {"l", "m", "Pendulum length", 0.5, 0.01, 5.0, 0.01},
            {"g", "m/s²", "Gravity", 9.81, 0.1, 20.0, 0.01},
            {"b", "N·s/m", "Cart friction", 0.1, 0.0, 10.0, 0.01}};
}

std::vector<Preset> InvertedPendulumCart::presets() const {
    return {{"Standard", "Classic parameters", {{"M",1.0},{"m",0.1},{"l",0.5},{"g",9.81},{"b",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Heavy pendulum", "m ≈ M", {{"M",1.0},{"m",0.8},{"l",0.5},{"g",9.81},{"b",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Long pendulum", "2m length", {{"M",1.0},{"m",0.1},{"l",2.0},{"g",9.81},{"b",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Low gravity", "Moon gravity", {{"M",1.0},{"m",0.1},{"l",0.5},{"g",1.62},{"b",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Air table", "Nearly frictionless", {{"M",1.0},{"m",0.1},{"l",0.5},{"g",9.81},{"b",0.001}}}};
}

void InvertedPendulumCart::applyPreset(int index) {
    auto p = presets(); if (index >= 0 && index < (int)p.size()) setParameters(p[index].params);
}

std::vector<std::string> InvertedPendulumCart::stateNames() const { return {"x", "dx/dt", "θ", "dθ/dt"}; }
std::vector<std::string> InvertedPendulumCart::outputNames() const { return {"Cart position", "Angle"}; }
std::vector<std::string> InvertedPendulumCart::inputNames() const { return {"Force"}; }
std::vector<std::string> InvertedPendulumCart::equationStrings() const {
    return {"(M+m)\\ddot{x} + ml\\ddot{\\theta}\\cos\\theta - ml\\dot{\\theta}^2\\sin\\theta + b\\dot{x} = F",
            "ml^2\\ddot{\\theta} + ml\\ddot{x}\\cos\\theta - mgl\\sin\\theta = 0"};
}

}  // namespace Simulation
