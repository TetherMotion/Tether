#include "tether/simulation/systems/thermal/HeatExchanger.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

HeatExchanger::HeatExchanger() {
    initParam("Ch", 2000.0); // hot-side thermal capacitance
    initParam("Cc", 2000.0); // cold-side thermal capacitance
    initParam("UA", 500.0);  // overall heat transfer coefficient × area
    initParam("mh_cp", 100.0); // hot mass flow × cp
    initParam("mc_cp", 80.0);  // cold mass flow × cp
    initParam("Th_in", 90.0);  // hot inlet [°C]
    initParam("Tc_in", 20.0);  // cold inlet [°C]
}

StateVector HeatExchanger::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Ch = params_.at("Ch"), Cc = params_.at("Cc"), UA = params_.at("UA");
    double mh = params_.at("mh_cp"), mc = params_.at("mc_cp");
    double Th_in = params_.at("Th_in"), Tc_in = params_.at("Tc_in");
    double flow_mod = u.empty() ? 1.0 : u[0]; // flow rate modifier

    double Th1 = s[0], Th2 = s[1]; // hot side: inlet, outlet lumped
    double Tc1 = s[2], Tc2 = s[3]; // cold side: inlet, outlet lumped

    double Q12 = UA * (Th1 - Tc2); // counter-flow heat exchange
    double Q22 = UA * (Th2 - Tc1);

    double dTh1 = (mh*flow_mod*(Th_in - Th1) - Q12) / Ch;
    double dTh2 = (mh*flow_mod*(Th1 - Th2) - Q22) / Ch;
    double dTc1 = (mc*(Tc_in - Tc1) + Q22) / Cc;
    double dTc2 = (mc*(Tc1 - Tc2) + Q12) / Cc;

    return {dTh1, dTh2, dTc1, dTc2};
}

StateVector HeatExchanger::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[1], s[3]}; }
StateVector HeatExchanger::defaultInitialState() const { return {90.0, 70.0, 20.0, 40.0}; }

std::vector<ParamDescriptor> HeatExchanger::parameterDescriptors() const {
    return {{"Ch","J/K","Hot-side cap",2000.0,100.0,50000.0,100.0},
            {"Cc","J/K","Cold-side cap",2000.0,100.0,50000.0,100.0},
            {"UA","W/K","UA product",500.0,10.0,5000.0,10.0},
            {"mh_cp","W/K","Hot flow×cp",100.0,1.0,1000.0,1.0},
            {"mc_cp","W/K","Cold flow×cp",80.0,1.0,1000.0,1.0},
            {"Th_in","°C","Hot inlet T",90.0,20.0,200.0,1.0},
            {"Tc_in","°C","Cold inlet T",20.0,0.0,50.0,0.5}};
}

std::vector<Preset> HeatExchanger::presets() const {
    return {{"Standard","Counter-flow",{{"Ch",2000.0},{"Cc",2000.0},{"UA",500.0},{"mh_cp",100.0},{"mc_cp",80.0},{"Th_in",90.0},{"Tc_in",20.0}}}};
}
void HeatExchanger::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> HeatExchanger::stateNames() const { return {"Th_in","Th_out","Tc_in","Tc_out"}; }
std::vector<std::string> HeatExchanger::outputNames() const { return {"Hot outlet","Cold outlet"}; }
std::vector<std::string> HeatExchanger::inputNames() const { return {"Flow modifier"}; }

}  // namespace Simulation
