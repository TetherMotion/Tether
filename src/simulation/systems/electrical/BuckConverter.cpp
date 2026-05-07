#include "tether/simulation/systems/electrical/BuckConverter.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BuckConverter::BuckConverter() {
    initParam("L_buck", 100e-6);  // inductance [H]
    initParam("C_buck", 470e-6);  // capacitance [F]
    initParam("R_load", 10.0);    // load resistance [Ω]
    initParam("Vin", 12.0);       // input voltage [V]
    initParam("R_L", 0.1);        // inductor ESR [Ω]
}

StateVector BuckConverter::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double L = params_.at("L_buck"), C = params_.at("C_buck");
    double R = params_.at("R_load"), Vin = params_.at("Vin"), RL = params_.at("R_L");
    double d = u.empty() ? 0.5 : std::clamp(u[0], 0.0, 1.0); // duty cycle

    double iL = s[0], vC = s[1];
    double diL = (d*Vin - vC - RL*iL) / L;
    double dvC = (iL - vC/R) / C;
    return {diL, dvC};
}

StateVector BuckConverter::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[1]}; }
StateVector BuckConverter::defaultInitialState() const { return {0.0, 0.0}; }
StateVector BuckConverter::defaultInput() const { return {0.5}; }

std::vector<ParamDescriptor> BuckConverter::parameterDescriptors() const {
    return {{"L_buck","H","Inductance",100e-6,1e-6,10e-3,1e-6,true},
            {"C_buck","F","Capacitance",470e-6,1e-6,10e-3,1e-6,true},
            {"R_load","Ω","Load resistance",10.0,0.1,1000.0,0.1},
            {"Vin","V","Input voltage",12.0,1.0,100.0,0.1},
            {"R_L","Ω","Inductor ESR",0.1,0.0,5.0,0.01}};
}

std::vector<Preset> BuckConverter::presets() const {
    return {{"5V@1A","12→5V",{{"L_buck",100e-6},{"C_buck",470e-6},{"R_load",5.0},{"Vin",12.0},{"R_L",0.1}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"3.3V@2A","12→3.3V",{{"L_buck",47e-6},{"C_buck",100e-6},{"R_load",1.65},{"Vin",12.0},{"R_L",0.05}}}};
}
void BuckConverter::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BuckConverter::stateNames() const { return {"i_L","v_C"}; }
std::vector<std::string> BuckConverter::outputNames() const { return {"Output voltage"}; }
std::vector<std::string> BuckConverter::inputNames() const { return {"Duty cycle"}; }

}  // namespace Simulation
