#include "tether/simulation/systems/electrical/BoostConverter.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BoostConverter::BoostConverter() {
    initParam("L_boost", 220e-6); initParam("C_boost", 100e-6);
    initParam("R_boost", 50.0); initParam("Vin_boost", 5.0);
    initParam("R_Lb", 0.2);
}

StateVector BoostConverter::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double L = params_.at("L_boost"), C = params_.at("C_boost");
    double R = params_.at("R_boost"), Vin = params_.at("Vin_boost"), RL = params_.at("R_Lb");
    double d = u.empty() ? 0.5 : std::clamp(u[0], 0.0, 0.99); // duty cycle

    double iL = s[0], vC = s[1];
    double diL = (Vin - RL*iL - (1.0-d)*vC) / L;
    double dvC = ((1.0-d)*iL - vC/R) / C;
    return {diL, dvC};
}

StateVector BoostConverter::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[1]}; }
StateVector BoostConverter::defaultInitialState() const { return {0.0, 5.0}; }
StateVector BoostConverter::defaultInput() const { return {0.5}; }

std::vector<ParamDescriptor> BoostConverter::parameterDescriptors() const {
    return {{"L_boost","H","Inductance",220e-6,1e-6,10e-3,1e-6,true},
            {"C_boost","F","Capacitance",100e-6,1e-6,10e-3,1e-6,true},
            {"R_boost","Ω","Load resistance",50.0,1.0,1000.0,1.0},
            {"Vin_boost","V","Input voltage",5.0,1.0,50.0,0.1},
            {"R_Lb","Ω","Inductor ESR",0.2,0.0,5.0,0.01}};
}

std::vector<Preset> BoostConverter::presets() const {
    return {{"5→12V","Standard boost",{{"L_boost",220e-6},{"C_boost",100e-6},{"R_boost",50.0},{"Vin_boost",5.0},{"R_Lb",0.2}}}};
}
void BoostConverter::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BoostConverter::stateNames() const { return {"i_L","v_C"}; }
std::vector<std::string> BoostConverter::outputNames() const { return {"Output voltage"}; }
std::vector<std::string> BoostConverter::inputNames() const { return {"Duty cycle"}; }

}  // namespace Simulation
