#include "tether/simulation/systems/rotational/DCMotorPosition.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DCMotorPosition::DCMotorPosition() {
    initParam("R", 1.0); initParam("L", 0.5e-3); initParam("Kt", 0.01);
    initParam("Ke", 0.01); initParam("J", 0.01); initParam("b", 0.1e-3);
}

StateVector DCMotorPosition::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double R = params_.at("R"), L = params_.at("L"), Kt = params_.at("Kt");
    double Ke = params_.at("Ke"), J = params_.at("J"), b = params_.at("b");
    double V = u.empty() ? 0.0 : u[0];
    double theta = s[0], omega = s[1], i = s[2];
    (void)theta;
    double dtheta = omega;
    double domega = (Kt * i - b * omega) / J;
    double di = (V - R * i - Ke * omega) / L;
    return {dtheta, domega, di};
}

StateVector DCMotorPosition::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector DCMotorPosition::defaultInitialState() const { return {0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> DCMotorPosition::parameterDescriptors() const {
    return {{"R","Ω","Resistance",1.0,0.01,100.0,0.01},
            {"L","H","Inductance",0.5e-3,1e-6,1.0,1e-4},
            {"Kt","N·m/A","Torque const",0.01,1e-4,1.0,1e-4},
            {"Ke","V·s/rad","Back-EMF const",0.01,1e-4,1.0,1e-4},
            {"J","kg·m²","Rotor inertia",0.01,1e-6,1.0,1e-4},
            {"b","N·m·s/rad","Friction",0.1e-3,0.0,0.1,1e-5}};
}

std::vector<Preset> DCMotorPosition::presets() const {
    return {{"Standard","Typical servo",{{"R",1.0},{"L",0.5e-3},{"Kt",0.01},{"Ke",0.01},{"J",0.01},{"b",0.1e-3}}}};
}
void DCMotorPosition::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DCMotorPosition::stateNames() const { return {"θ","ω","i"}; }
std::vector<std::string> DCMotorPosition::outputNames() const { return {"Position"}; }
std::vector<std::string> DCMotorPosition::inputNames() const { return {"Voltage"}; }

}  // namespace Simulation
