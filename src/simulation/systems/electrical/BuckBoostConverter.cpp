#include "tether/simulation/systems/electrical/BuckBoostConverter.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BuckBoostConverter::BuckBoostConverter() {
    initParam("L_bb", 150e-6); initParam("C_bb", 220e-6);
    initParam("R_bb", 20.0); initParam("Vin_bb", 12.0);
    initParam("R_Lbb", 0.15);
}

StateVector BuckBoostConverter::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double L = params_.at("L_bb"), C = params_.at("C_bb");
    double R = params_.at("R_bb"), Vin = params_.at("Vin_bb"), RL = params_.at("R_Lbb");
    double d = u.size()>0 ? std::clamp(u[0], 0.0, 0.95) : 0.5; // duty cycle
    double Vin_mod = u.size()>1 ? u[1] : Vin; // variable input voltage

    double iL = s[0], vC = s[1];
    double diL = (d*Vin_mod - RL*iL - (1.0-d)*vC) / L;
    double dvC = ((1.0-d)*iL - vC/R) / C;
    return {diL, dvC};
}

StateVector BuckBoostConverter::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[1], s[0]}; }
StateVector BuckBoostConverter::defaultInitialState() const { return {0.0, 0.0}; }
StateVector BuckBoostConverter::defaultInput() const { return {0.5, params_.at("Vin_bb")}; }

std::vector<ParamDescriptor> BuckBoostConverter::parameterDescriptors() const {
    return {{"L_bb","H","Inductance",150e-6,1e-6,10e-3,1e-6,true},
            {"C_bb","F","Capacitance",220e-6,1e-6,10e-3,1e-6,true},
            {"R_bb","Ω","Load",20.0,0.5,500.0,0.5},
            {"Vin_bb","V","Nominal Vin",12.0,1.0,100.0,0.1},
            {"R_Lbb","Ω","ESR",0.15,0.0,5.0,0.01}};
}

std::vector<Preset> BuckBoostConverter::presets() const {
    return {{"Standard","12V nominal",{{"L_bb",150e-6},{"C_bb",220e-6},{"R_bb",20.0},{"Vin_bb",12.0},{"R_Lbb",0.15}}}};
}
void BuckBoostConverter::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BuckBoostConverter::stateNames() const { return {"i_L","v_C"}; }
std::vector<std::string> BuckBoostConverter::outputNames() const { return {"Output voltage","Inductor current"}; }
std::vector<std::string> BuckBoostConverter::inputNames() const { return {"Duty cycle","Input voltage"}; }

}  // namespace Simulation
