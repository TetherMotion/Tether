#include "tether/simulation/systems/thermal/RoomHVAC.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

RoomHVAC::RoomHVAC() {
    initParam("C_air", 10000.0);    // air thermal capacitance [J/K]
    initParam("C_walls", 100000.0); // wall thermal capacitance
    initParam("C_furn", 30000.0);   // furniture thermal capacitance
    initParam("R_aw", 0.01);        // air-wall thermal resistance [K/W]
    initParam("R_af", 0.05);        // air-furniture resistance
    initParam("R_wo", 0.1);         // wall-outside resistance
    initParam("T_out", 35.0);       // outdoor temperature [°C]
    initParam("eta_hvac", 3.0);     // COP (coefficient of performance)
}

StateVector RoomHVAC::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Ca = params_.at("C_air"), Cw = params_.at("C_walls"), Cf = params_.at("C_furn");
    double Raw = params_.at("R_aw"), Raf = params_.at("R_af"), Rwo = params_.at("R_wo");
    double T_out = params_.at("T_out"), COP = params_.at("eta_hvac");
    double Qhvac = u.empty() ? 0.0 : u[0]; // HVAC power [W], negative=cooling

    double Ta = s[0], Tw = s[1], Tf = s[2];

    double dTa = (COP*Qhvac - (Ta-Tw)/Raw - (Ta-Tf)/Raf) / Ca;
    double dTw = ((Ta-Tw)/Raw - (Tw-T_out)/Rwo) / Cw;
    double dTf = ((Ta-Tf)/Raf) / Cf;

    return {dTa, dTw, dTf};
}

StateVector RoomHVAC::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector RoomHVAC::defaultInitialState() const { return {30.0, 28.0, 28.0}; }

std::vector<ParamDescriptor> RoomHVAC::parameterDescriptors() const {
    return {{"C_air","J/K","Air cap",10000.0,1000.0,100000.0,1000.0},
            {"C_walls","J/K","Wall cap",100000.0,10000.0,1000000.0,10000.0},
            {"C_furn","J/K","Furniture cap",30000.0,1000.0,100000.0,1000.0},
            {"R_aw","K/W","Air-wall R",0.01,0.001,1.0,0.001},
            {"R_af","K/W","Air-furn R",0.05,0.001,1.0,0.005},
            {"R_wo","K/W","Wall-out R",0.1,0.01,5.0,0.01},
            {"T_out","°C","Outdoor T",35.0,-10.0,50.0,0.5},
            {"eta_hvac","","COP",3.0,1.0,6.0,0.1}};
}

std::vector<Preset> RoomHVAC::presets() const {
    return {{"Small room","Well insulated",{{"C_air",10000.0},{"C_walls",100000.0},{"C_furn",30000.0}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"R_aw",0.01},{"R_af",0.05},{"R_wo",0.1},{"T_out",35.0},{"eta_hvac",3.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Large room","Poor insulation",{{"C_air",50000.0},{"C_walls",300000.0},{"C_furn",100000.0}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"R_aw",0.005},{"R_af",0.02},{"R_wo",0.05},{"T_out",38.0},{"eta_hvac",2.5}}}};
}
void RoomHVAC::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> RoomHVAC::stateNames() const { return {"T_air","T_walls","T_furniture"}; }
std::vector<std::string> RoomHVAC::outputNames() const { return {"Room temperature"}; }
std::vector<std::string> RoomHVAC::inputNames() const { return {"HVAC power"}; }

}  // namespace Simulation
