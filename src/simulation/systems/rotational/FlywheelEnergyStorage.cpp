#include "tether/simulation/systems/rotational/FlywheelEnergyStorage.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

FlywheelEnergyStorage::FlywheelEnergyStorage() {
    initParam("J_fw", 1.0);    // flywheel inertia [kg·m²]
    initParam("R_fw", 0.5);    // motor resistance [Ω]
    initParam("L_fw", 1e-3);   // motor inductance [H]
    initParam("Kt_fw", 0.1);   // torque constant
    initParam("Ke_fw", 0.1);   // back-EMF constant
    initParam("b_fw", 0.001);  // bearing friction
}

StateVector FlywheelEnergyStorage::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double J = params_.at("J_fw"), R = params_.at("R_fw"), L = params_.at("L_fw");
    double Kt = params_.at("Kt_fw"), Ke = params_.at("Ke_fw"), b = params_.at("b_fw");
    double V = u.empty() ? 0.0 : u[0];
    double omega = s[0], i = s[1];
    double domega = (Kt*i - b*omega) / J;
    double di = (V - R*i - Ke*omega) / L;
    return {domega, di};
}

StateVector FlywheelEnergyStorage::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector FlywheelEnergyStorage::defaultInitialState() const { return {100.0, 0.0}; }

std::vector<ParamDescriptor> FlywheelEnergyStorage::parameterDescriptors() const {
    return {{"J_fw","kg·m²","Flywheel inertia",1.0,0.01,100.0,0.01},
            {"R_fw","Ω","Motor resistance",0.5,0.01,10.0,0.01},
            {"L_fw","H","Motor inductance",1e-3,1e-6,0.1,1e-5},
            {"Kt_fw","N·m/A","Torque const",0.1,0.001,1.0,0.001},
            {"Ke_fw","V·s/rad","Back-EMF const",0.1,0.001,1.0,0.001},
            {"b_fw","","Bearing friction",0.001,0.0,0.1,1e-4}};
}

std::vector<Preset> FlywheelEnergyStorage::presets() const {
    return {{"Standard","1 kW·s",{{"J_fw",1.0},{"R_fw",0.5},{"L_fw",1e-3},{"Kt_fw",0.1},{"Ke_fw",0.1},{"b_fw",0.001}}}};
}
void FlywheelEnergyStorage::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> FlywheelEnergyStorage::stateNames() const { return {"ω","i"}; }
std::vector<std::string> FlywheelEnergyStorage::outputNames() const { return {"Speed"}; }
std::vector<std::string> FlywheelEnergyStorage::inputNames() const { return {"Voltage"}; }

}  // namespace Simulation
