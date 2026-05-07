#include "tether/simulation/systems/thermal/SingleZoneOven.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

SingleZoneOven::SingleZoneOven() {
    initParam("C_th", 5000.0);  // thermal capacitance [J/K]
    initParam("R_th", 0.5);     // thermal resistance [K/W]
    initParam("T_amb", 25.0);   // ambient temperature [°C]
    initParam("eta", 0.9);      // heater efficiency
}

StateVector SingleZoneOven::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double C = params_.at("C_th"), R = params_.at("R_th");
    double T_amb = params_.at("T_amb"), eta = params_.at("eta");
    double Q = u.empty() ? 0.0 : u[0]; // heater power [W]
    double T = s[0];
    double dT = (eta*Q - (T - T_amb)/R) / C;
    return {dT};
}

StateVector SingleZoneOven::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector SingleZoneOven::defaultInitialState() const { return {25.0}; }

std::vector<ParamDescriptor> SingleZoneOven::parameterDescriptors() const {
    return {{"C_th","J/K","Thermal capacitance",5000.0,100.0,100000.0,100.0},
            {"R_th","K/W","Thermal resistance",0.5,0.01,10.0,0.01},
            {"T_amb","°C","Ambient temperature",25.0,-20.0,50.0,0.5},
            {"eta","","Heater efficiency",0.9,0.1,1.0,0.01}};
}

std::vector<Preset> SingleZoneOven::presets() const {
    return {{"Lab oven","Small lab oven",{{"C_th",5000.0},{"R_th",0.5},{"T_amb",25.0},{"eta",0.9}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Reflow","Solder reflow",{{"C_th",2000.0},{"R_th",0.3},{"T_amb",25.0},{"eta",0.85}}}};
}
void SingleZoneOven::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> SingleZoneOven::stateNames() const { return {"T"}; }
std::vector<std::string> SingleZoneOven::outputNames() const { return {"Temperature"}; }
std::vector<std::string> SingleZoneOven::inputNames() const { return {"Heater power"}; }

}  // namespace Simulation
