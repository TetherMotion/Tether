#include "tether/simulation/systems/fluid/PressureVessel.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PressureVessel::PressureVessel() {
    initParam("V_vessel", 0.1);   // vessel volume [m³]
    initParam("Cv", 1e-4);        // valve flow coefficient
    initParam("P_atm", 101325.0); // atmospheric pressure [Pa]
    initParam("T_gas", 300.0);    // gas temperature [K]
    initParam("R_gas", 287.0);    // specific gas constant (air)
    initParam("gamma_gas", 1.4);  // heat capacity ratio
}

StateVector PressureVessel::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double V = params_.at("V_vessel"), Cv = params_.at("Cv"), Pa = params_.at("P_atm");
    double T = params_.at("T_gas"), R = params_.at("R_gas"), gamma = params_.at("gamma_gas");
    double valve = u.empty() ? 0.0 : u[0]; // valve opening [0,1]

    double P = s[0];
    double dP_diff = P - Pa;
    double mdot = Cv * valve * std::copysign(std::sqrt(std::abs(dP_diff)), dP_diff); // simplified
    double dP = -gamma * R * T * mdot / V;
    return {dP};
}

StateVector PressureVessel::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector PressureVessel::defaultInitialState() const { return {500000.0}; }

std::vector<ParamDescriptor> PressureVessel::parameterDescriptors() const {
    return {{"V_vessel","m³","Volume",0.1,0.001,10.0,0.001},
            {"Cv","","Valve coeff",1e-4,1e-7,0.01,1e-6},
            {"P_atm","Pa","Atm pressure",101325.0,90000.0,110000.0,100.0},
            {"T_gas","K","Gas temperature",300.0,200.0,600.0,1.0},
            {"R_gas","J/(kg·K)","Gas constant",287.0,100.0,500.0,1.0},
            {"gamma_gas","","Heat cap ratio",1.4,1.0,1.67,0.01}};
}

std::vector<Preset> PressureVessel::presets() const {
    return {{"Air tank","Compressed air",{{"V_vessel",0.1},{"Cv",1e-4},{"P_atm",101325.0},{"T_gas",300.0},{"R_gas",287.0},{"gamma_gas",1.4}}}};
}
void PressureVessel::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PressureVessel::stateNames() const { return {"P"}; }
std::vector<std::string> PressureVessel::outputNames() const { return {"Pressure"}; }
std::vector<std::string> PressureVessel::inputNames() const { return {"Valve opening"}; }

}  // namespace Simulation
