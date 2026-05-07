#include "tether/simulation/systems/fluid/SingleTankLevel.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

SingleTankLevel::SingleTankLevel() {
    initParam("A", 1.0);     // cross-section area [m²]
    initParam("a", 0.01);    // outlet orifice area [m²]
    initParam("Cd", 0.6);    // discharge coefficient
    initParam("g", 9.81);
}

StateVector SingleTankLevel::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double A = params_.at("A"), a = params_.at("a"), Cd = params_.at("Cd"), g = params_.at("g");
    double Qin = u.empty() ? 0.0 : u[0]; // inflow [m³/s]
    double h = std::max(s[0], 0.0);
    double Qout = Cd * a * std::sqrt(2.0*g*h);
    double dh = (Qin - Qout) / A;
    return {dh};
}

StateVector SingleTankLevel::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector SingleTankLevel::defaultInitialState() const { return {0.5}; }

std::vector<ParamDescriptor> SingleTankLevel::parameterDescriptors() const {
    return {{"A","m²","Tank area",1.0,0.01,10.0,0.01},
            {"a","m²","Outlet area",0.01,1e-5,0.1,1e-4},
            {"Cd","","Discharge coeff",0.6,0.1,1.0,0.01},
            {"g","m/s²","Gravity",9.81,0.0,20.0,0.01}};
}

std::vector<Preset> SingleTankLevel::presets() const {
    return {{"Standard","Medium tank",{{"A",1.0},{"a",0.01},{"Cd",0.6},{"g",9.81}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Fast drain","Large outlet",{{"A",1.0},{"a",0.05},{"Cd",0.6},{"g",9.81}}}};
}
void SingleTankLevel::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> SingleTankLevel::stateNames() const { return {"h"}; }
std::vector<std::string> SingleTankLevel::outputNames() const { return {"Level"}; }
std::vector<std::string> SingleTankLevel::inputNames() const { return {"Inflow"}; }

}  // namespace Simulation
