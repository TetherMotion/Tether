#include "tether/simulation/systems/electrical/PowerGridFrequency.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PowerGridFrequency::PowerGridFrequency() {
    initParam("H_grid", 5.0);    // inertia constant [s]
    initParam("D_grid", 1.0);    // damping coefficient [pu]
    initParam("Tg", 0.2);        // governor time constant [s]
    initParam("Tt", 0.3);        // turbine time constant [s]
    initParam("R_droop", 0.05);  // droop [pu]
    initParam("f0", 50.0);       // nominal frequency [Hz]
}

StateVector PowerGridFrequency::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double H = params_.at("H_grid"), D = params_.at("D_grid");
    double Tg = params_.at("Tg"), Tt = params_.at("Tt"), R = params_.at("R_droop");
    double dPL = u.empty() ? 0.0 : u[0]; // load disturbance [pu]

    double df = s[0];        // frequency deviation [pu]
    double Pm = s[1];        // mechanical power [pu]
    double Pv = s[2];        // valve position [pu]

    double ddf = (Pm - dPL - D*df) / (2.0*H);
    double dPm = (Pv - Pm) / Tt;
    double dPv = (-df/R - Pv) / Tg;

    return {ddf, dPm, dPv};
}

StateVector PowerGridFrequency::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector PowerGridFrequency::defaultInitialState() const { return {0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> PowerGridFrequency::parameterDescriptors() const {
    return {{"H_grid","s","Inertia constant",5.0,1.0,15.0,0.1},
            {"D_grid","pu","Damping",1.0,0.1,5.0,0.1},
            {"Tg","s","Governor τ",0.2,0.01,2.0,0.01},
            {"Tt","s","Turbine τ",0.3,0.01,5.0,0.01},
            {"R_droop","pu","Droop",0.05,0.01,0.1,0.001},
            {"f0","Hz","Nom freq",50.0,50.0,60.0,10.0}};
}

std::vector<Preset> PowerGridFrequency::presets() const {
    return {{"Standard","50Hz grid",{{"H_grid",5.0},{"D_grid",1.0},{"Tg",0.2},{"Tt",0.3},{"R_droop",0.05},{"f0",50.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Low inertia","Renewable-heavy",{{"H_grid",2.0},{"D_grid",0.5},{"Tg",0.2},{"Tt",0.3},{"R_droop",0.04},{"f0",50.0}}}};
}
void PowerGridFrequency::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PowerGridFrequency::stateNames() const { return {"Δf","P_m","P_valve"}; }
std::vector<std::string> PowerGridFrequency::outputNames() const { return {"Freq deviation"}; }
std::vector<std::string> PowerGridFrequency::inputNames() const { return {"Load disturbance"}; }

}  // namespace Simulation
