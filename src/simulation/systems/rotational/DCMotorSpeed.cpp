#include "tether/simulation/systems/rotational/DCMotorSpeed.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DCMotorSpeed::DCMotorSpeed() {
    initParam("R", 1.0);    // resistance [Ω]
    initParam("L", 0.5e-3); // inductance [H]
    initParam("Kt", 0.01);  // torque constant [N·m/A]
    initParam("Ke", 0.01);  // back-EMF constant [V·s/rad]
    initParam("J", 0.01);   // rotor inertia [kg·m²]
    initParam("b", 0.1e-3); // viscous friction [N·m·s/rad]
}

StateVector DCMotorSpeed::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double R = params_.at("R"), L = params_.at("L"), Kt = params_.at("Kt");
    double Ke = params_.at("Ke"), J = params_.at("J"), b = params_.at("b");
    double V = u.empty() ? 0.0 : u[0];
    double omega = s[0], i = s[1];
    double domega = (Kt * i - b * omega) / J;
    double di = (V - R * i - Ke * omega) / L;
    return {domega, di};
}

StateVector DCMotorSpeed::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector DCMotorSpeed::defaultInitialState() const { return {0.0, 0.0}; }

std::vector<ParamDescriptor> DCMotorSpeed::parameterDescriptors() const {
    return {{"R","Ω","Resistance",1.0,0.01,100.0,0.01},
            {"L","H","Inductance",0.5e-3,1e-6,1.0,1e-4},
            {"Kt","N·m/A","Torque const",0.01,1e-4,1.0,1e-4},
            {"Ke","V·s/rad","Back-EMF const",0.01,1e-4,1.0,1e-4},
            {"J","kg·m²","Rotor inertia",0.01,1e-6,1.0,1e-4},
            {"b","N·m·s/rad","Viscous friction",0.1e-3,0.0,0.1,1e-5}};
}

std::vector<Preset> DCMotorSpeed::presets() const {
    return {{"Small motor","Low inertia",{{"R",1.0},{"L",0.5e-3},{"Kt",0.01},{"Ke",0.01},{"J",0.01},{"b",0.1e-3}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Large motor","High inertia",{{"R",0.5},{"L",2e-3},{"Kt",0.05},{"Ke",0.05},{"J",0.1},{"b",1e-3}}}};
}
void DCMotorSpeed::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DCMotorSpeed::stateNames() const { return {"ω","i"}; }
std::vector<std::string> DCMotorSpeed::outputNames() const { return {"Speed"}; }
std::vector<std::string> DCMotorSpeed::inputNames() const { return {"Voltage"}; }

}  // namespace Simulation
