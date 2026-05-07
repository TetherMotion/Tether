#include "tether/simulation/systems/thermal/ThermoelectricCooler.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ThermoelectricCooler::ThermoelectricCooler() {
    initParam("C_cold", 100.0); initParam("C_hot", 200.0);
    initParam("R_hs", 0.5);    // heatsink thermal resistance
    initParam("Se", 0.05);     // Seebeck coefficient [V/K]
    initParam("Re", 1.0);      // electrical resistance [Ω]
    initParam("K_th", 0.5);    // thermal conductance [W/K]
    initParam("T_amb", 25.0);
}

StateVector ThermoelectricCooler::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Cc = params_.at("C_cold"), Ch = params_.at("C_hot");
    double Rhs = params_.at("R_hs"), Se = params_.at("Se"), Re = params_.at("Re");
    double K = params_.at("K_th"), T_amb = params_.at("T_amb");
    double I = u.empty() ? 0.0 : u[0]; // current [A]

    double Tc = s[0], Th = s[1];

    double Qc = Se*I*Tc - 0.5*Re*I*I - K*(Th-Tc); // cold side cooling
    double Qh = Se*I*Th + 0.5*Re*I*I - K*(Th-Tc); // hot side heating

    double dTc = (-Qc) / Cc; // heat removed from cold side
    double dTh = (Qh - (Th-T_amb)/Rhs) / Ch;

    return {dTc, dTh};
}

StateVector ThermoelectricCooler::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector ThermoelectricCooler::defaultInitialState() const { return {25.0, 25.0}; }

std::vector<ParamDescriptor> ThermoelectricCooler::parameterDescriptors() const {
    return {{"C_cold","J/K","Cold-side cap",100.0,10.0,5000.0,10.0},
            {"C_hot","J/K","Hot-side cap",200.0,10.0,5000.0,10.0},
            {"R_hs","K/W","Heatsink R_th",0.5,0.01,5.0,0.01},
            {"Se","V/K","Seebeck coeff",0.05,0.001,0.2,0.001},
            {"Re","Ω","Elec. resistance",1.0,0.01,10.0,0.01},
            {"K_th","W/K","Thermal conductance",0.5,0.01,5.0,0.01},
            {"T_amb","°C","Ambient",25.0,-20.0,50.0,0.5}};
}

std::vector<Preset> ThermoelectricCooler::presets() const {
    return {{"Standard","Single TEC module",{{"C_cold",100.0},{"C_hot",200.0},{"R_hs",0.5}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"Se",0.05},{"Re",1.0},{"K_th",0.5},{"T_amb",25.0}}}};
}
void ThermoelectricCooler::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ThermoelectricCooler::stateNames() const { return {"T_cold","T_hot"}; }
std::vector<std::string> ThermoelectricCooler::outputNames() const { return {"Cold side T"}; }
std::vector<std::string> ThermoelectricCooler::inputNames() const { return {"Current"}; }

}  // namespace Simulation
