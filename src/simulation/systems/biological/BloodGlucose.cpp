#include "tether/simulation/systems/biological/BloodGlucose.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BloodGlucose::BloodGlucose() {
    initParam("p1", 0.028);    // glucose effectiveness [1/min]
    initParam("p2", 0.025);    // insulin action rate [1/min]
    initParam("p3", 5e-6);     // insulin sensitivity [mU/(L²·min)]
    initParam("n", 0.23);      // insulin clearance [1/min]
    initParam("Gb", 90.0);     // basal glucose [mg/dL]
    initParam("Ib", 15.0);     // basal insulin [mU/L]
    initParam("gamma_bg", 0.004); // pancreatic responsivity
    initParam("h", 80.0);      // glucose threshold [mg/dL]
}

StateVector BloodGlucose::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double p1 = params_.at("p1"), p2 = params_.at("p2"), p3 = params_.at("p3");
    double n = params_.at("n"), Gb = params_.at("Gb"), Ib = params_.at("Ib");
    double gamma_bg = params_.at("gamma_bg"), h = params_.at("h");
    double insulin_rate = u.empty() ? 0.0 : u[0]; // external insulin [mU/L/min]

    double G = s[0]; // glucose [mg/dL]
    double X = s[1]; // remote insulin action [1/min]
    double I = s[2]; // plasma insulin [mU/L]

    double dG = -p1 * (G - Gb) - X * G;
    double dX = -p2 * X + p3 * (I - Ib);
    double dI = gamma_bg * std::max(G - h, 0.0) - n * I + insulin_rate;

    return {dG, dX, dI};
}

StateVector BloodGlucose::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0]}; // glucose is the measured output
}
StateVector BloodGlucose::defaultInitialState() const { return {90.0, 0.0, 15.0}; }

std::vector<ParamDescriptor> BloodGlucose::parameterDescriptors() const {
    return {{"p1","1/min","Glucose effectiveness",0.028,0.001,0.1,0.001},
            {"p2","1/min","Insulin action rate",0.025,0.001,0.1,0.001},
            {"p3","","Insulin sensitivity",5e-6,1e-7,1e-4,1e-7,true},
            {"n","1/min","Insulin clearance",0.23,0.01,1.0,0.01},
            {"Gb","mg/dL","Basal glucose",90.0,60.0,120.0,1.0},
            {"Ib","mU/L","Basal insulin",15.0,5.0,50.0,1.0},
            {"gamma_bg","","Pancreatic resp",0.004,0.001,0.01,0.001},
            {"h","mg/dL","Glucose threshold",80.0,50.0,120.0,1.0}};
}

std::vector<Preset> BloodGlucose::presets() const {
    return {{"Healthy","Normal glucose tolerance",
             {{"p1",0.028},{"p2",0.025},{"p3",5e-6},{"n",0.23},{"Gb",90.0},{"Ib",15.0},{"gamma_bg",0.004},{"h",80.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Type 2 DM","Impaired insulin sensitivity",
             {{"p1",0.015},{"p2",0.015},{"p3",2e-6},{"n",0.15},{"Gb",120.0},{"Ib",25.0},{"gamma_bg",0.002},{"h",100.0}}}};
}
void BloodGlucose::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BloodGlucose::stateNames() const { return {"G","X","I"}; }
std::vector<std::string> BloodGlucose::outputNames() const { return {"Glucose"}; }
std::vector<std::string> BloodGlucose::inputNames() const { return {"Insulin rate"}; }

}  // namespace Simulation
