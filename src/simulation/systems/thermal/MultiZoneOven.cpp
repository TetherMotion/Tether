#include "tether/simulation/systems/thermal/MultiZoneOven.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

MultiZoneOven::MultiZoneOven() {
    initParam("C1", 3000.0); initParam("C2", 3000.0); initParam("C3", 3000.0);
    initParam("R12", 0.5); initParam("R23", 0.5);
    initParam("R1a", 1.0); initParam("R2a", 1.5); initParam("R3a", 1.0);
    initParam("T_amb", 25.0); initParam("eta", 0.9);
}

StateVector MultiZoneOven::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double C1 = params_.at("C1"), C2 = params_.at("C2"), C3 = params_.at("C3");
    double R12 = params_.at("R12"), R23 = params_.at("R23");
    double R1a = params_.at("R1a"), R2a = params_.at("R2a"), R3a = params_.at("R3a");
    double T_amb = params_.at("T_amb"), eta = params_.at("eta");
    double Q1 = u.size()>0 ? u[0] : 0.0;
    double Q2 = u.size()>1 ? u[1] : 0.0;
    double Q3 = u.size()>2 ? u[2] : 0.0;
    double T1 = s[0], T2 = s[1], T3 = s[2];

    double dT1 = (eta*Q1 - (T1-T2)/R12 - (T1-T_amb)/R1a) / C1;
    double dT2 = (eta*Q2 + (T1-T2)/R12 - (T2-T3)/R23 - (T2-T_amb)/R2a) / C2;
    double dT3 = (eta*Q3 + (T2-T3)/R23 - (T3-T_amb)/R3a) / C3;
    return {dT1, dT2, dT3};
}

StateVector MultiZoneOven::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1], s[2]}; }
StateVector MultiZoneOven::defaultInitialState() const { return {25.0, 25.0, 25.0}; }

std::vector<ParamDescriptor> MultiZoneOven::parameterDescriptors() const {
    return {{"C1","J/K","Zone 1 cap",3000.0,100.0,50000.0,100.0},
            {"C2","J/K","Zone 2 cap",3000.0,100.0,50000.0,100.0},
            {"C3","J/K","Zone 3 cap",3000.0,100.0,50000.0,100.0},
            {"R12","K/W","R zone 1-2",0.5,0.01,10.0,0.01},
            {"R23","K/W","R zone 2-3",0.5,0.01,10.0,0.01},
            {"R1a","K/W","R zone 1-amb",1.0,0.01,10.0,0.01},
            {"R2a","K/W","R zone 2-amb",1.5,0.01,10.0,0.01},
            {"R3a","K/W","R zone 3-amb",1.0,0.01,10.0,0.01},
            {"T_amb","°C","Ambient",25.0,-20.0,50.0,0.5},
            {"eta","","Efficiency",0.9,0.1,1.0,0.01}};
}

std::vector<Preset> MultiZoneOven::presets() const {
    return {{"Symmetric","Equal zones",{{"C1",3000.0},{"C2",3000.0},{"C3",3000.0}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"R12",0.5},{"R23",0.5},{"R1a",1.0},{"R2a",1.5},{"R3a",1.0},{"T_amb",25.0},{"eta",0.9}}}};
}
void MultiZoneOven::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> MultiZoneOven::stateNames() const { return {"T₁","T₂","T₃"}; }
std::vector<std::string> MultiZoneOven::outputNames() const { return {"Zone 1","Zone 2","Zone 3"}; }
std::vector<std::string> MultiZoneOven::inputNames() const { return {"Q₁","Q₂","Q₃"}; }

}  // namespace Simulation
