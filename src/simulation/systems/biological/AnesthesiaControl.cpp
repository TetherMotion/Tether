#include "tether/simulation/systems/biological/AnesthesiaControl.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

AnesthesiaControl::AnesthesiaControl() {
    initParam("k10", 0.0443);  // elimination rate [1/min]
    initParam("k12", 0.0523);  // redistribution 1→2 [1/min]
    initParam("k13", 0.0126);  // redistribution 1→3 [1/min]
    initParam("k21", 0.0291);  // redistribution 2→1 [1/min]
    initParam("k31", 0.0055);  // redistribution 3→1 [1/min]
    initParam("ke0", 0.456);   // effect site equilibration [1/min]
    initParam("V1", 4.27);     // central compartment volume [L]
    initParam("EC50", 3.4);    // half-maximal effect conc [μg/mL]
    initParam("gamma_an", 3.0); // Hill coefficient
}

StateVector AnesthesiaControl::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double k10 = params_.at("k10"), k12 = params_.at("k12"), k13 = params_.at("k13");
    double k21 = params_.at("k21"), k31 = params_.at("k31"), ke0 = params_.at("ke0");
    double V1 = params_.at("V1");
    double infusion = u.empty() ? 0.0 : u[0]; // infusion rate [mg/min]

    double c1 = s[0]; // central compartment [μg/mL]
    double c2 = s[1]; // rapid peripheral
    double c3 = s[2]; // slow peripheral
    double ce = s[3]; // effect site

    double dc1 = -(k10 + k12 + k13) * c1 + k21 * c2 + k31 * c3 + infusion / V1;
    double dc2 = k12 * c1 - k21 * c2;
    double dc3 = k13 * c1 - k31 * c3;
    double dce = ke0 * (c1 - ce);

    return {dc1, dc2, dc3, dce};
}

StateVector AnesthesiaControl::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    double EC50 = params_.at("EC50"), gamma = params_.at("gamma_an");
    double ce = std::max(s[3], 0.0);
    // BIS (Bispectral Index) as Hill function: BIS = 100 * (1 - ce^γ/(EC50^γ + ce^γ))
    double ceg = std::pow(ce, gamma);
    double ecg = std::pow(EC50, gamma);
    double BIS = 100.0 * (1.0 - ceg / (ecg + ceg));
    return {std::clamp(BIS, 0.0, 100.0)};
}
StateVector AnesthesiaControl::defaultInitialState() const { return {0.0, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> AnesthesiaControl::parameterDescriptors() const {
    return {{"k10","1/min","Elimination rate",0.0443,0.001,0.2,0.001},
            {"k12","1/min","Redistrib 1→2",0.0523,0.001,0.2,0.001},
            {"k13","1/min","Redistrib 1→3",0.0126,0.001,0.1,0.001},
            {"k21","1/min","Redistrib 2→1",0.0291,0.001,0.1,0.001},
            {"k31","1/min","Redistrib 3→1",0.0055,0.001,0.1,0.001},
            {"ke0","1/min","Effect site eq",0.456,0.01,2.0,0.01},
            {"V1","L","Central volume",4.27,1.0,20.0,0.1},
            {"EC50","μg/mL","Half-max conc",3.4,0.5,10.0,0.1},
            {"gamma_an","","Hill coefficient",3.0,1.0,5.0,0.1}};
}

std::vector<Preset> AnesthesiaControl::presets() const {
    return {{"Propofol","Standard patient",
             {{"k10",0.0443},{"k12",0.0523},{"k13",0.0126},{"k21",0.0291},{"k31",0.0055}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"ke0",0.456},{"V1",4.27},{"EC50",3.4},{"gamma_an",3.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Elderly","Reduced clearance",
             {{"k10",0.03},{"k12",0.04},{"k13",0.01},{"k21",0.02},{"k31",0.004}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"ke0",0.3},{"V1",3.5},{"EC50",2.5},{"gamma_an",3.0}}}};
}
void AnesthesiaControl::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> AnesthesiaControl::stateNames() const { return {"c1","c2","c3","ce"}; }
std::vector<std::string> AnesthesiaControl::outputNames() const { return {"BIS"}; }
std::vector<std::string> AnesthesiaControl::inputNames() const { return {"Infusion rate"}; }

}  // namespace Simulation
