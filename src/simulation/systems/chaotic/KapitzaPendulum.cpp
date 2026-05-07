#include "tether/simulation/systems/chaotic/KapitzaPendulum.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

KapitzaPendulum::KapitzaPendulum() {
    initParam("g", 9.81);      // gravity [m/s²]
    initParam("l", 0.3);       // pendulum length [m]
    initParam("A", 0.02);      // vibration amplitude [m]
    initParam("omega_k", 300.0); // vibration frequency [rad/s]
    initParam("b", 0.01);      // damping [N·m·s/rad]
    initParam("m", 0.1);       // mass [kg]
}

StateVector KapitzaPendulum::dynamics(double t, const StateVector& s, const StateVector& u) const {
    double g = params_.at("g"), l = params_.at("l");
    double A = params_.at("A"), omega = params_.at("omega_k");
    double b_damp = params_.at("b"), m = params_.at("m");
    double amp_input = u.empty() ? A : u[0]; // controllable vibration amplitude

    double theta = s[0], dtheta = s[1];

    // Pivot vibration: y_pivot = amp * cos(omega * t)
    // Effective gravity: g_eff = g + A*ω²*cos(ωt)
    double accel_pivot = -amp_input * omega * omega * std::cos(omega * t);
    double ddtheta = -(g + accel_pivot) * std::sin(theta) / l
                     - b_damp * dtheta / (m * l * l);

    return {dtheta, ddtheta};
}

StateVector KapitzaPendulum::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector KapitzaPendulum::defaultInitialState() const { return {3.1, 0.0}; } // near inverted

std::vector<ParamDescriptor> KapitzaPendulum::parameterDescriptors() const {
    return {{"g","m/s²","Gravity",9.81,0.0,20.0,0.01},
            {"l","m","Length",0.3,0.01,2.0,0.01},
            {"A","m","Vibration amplitude",0.02,0.001,0.1,0.001},
            {"omega_k","rad/s","Vibration freq",300.0,10.0,1000.0,10.0},
            {"b","N·m·s/rad","Damping",0.01,0.0,1.0,0.001},
            {"m","kg","Mass",0.1,0.01,10.0,0.01}};
}

std::vector<Preset> KapitzaPendulum::presets() const {
    return {{"Stabilized","High freq vibration",
             {{"g",9.81},{"l",0.3},{"A",0.02},{"omega_k",300.0},{"b",0.01},{"m",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Unstable","Low freq",
             {{"g",9.81},{"l",0.3},{"A",0.02},{"omega_k",50.0},{"b",0.01},{"m",0.1}}}};
}
void KapitzaPendulum::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> KapitzaPendulum::stateNames() const { return {"θ","dθ/dt"}; }
std::vector<std::string> KapitzaPendulum::outputNames() const { return {"Angle"}; }
std::vector<std::string> KapitzaPendulum::inputNames() const { return {"Vibration amplitude"}; }

}  // namespace Simulation
