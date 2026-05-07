#include "tether/simulation/systems/chemical/pHNeutralization.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

pHNeutralization::pHNeutralization() {
    initParam("V", 2.0);        // tank volume [L]
    initParam("F_acid", 0.01);  // acid stream flow [L/s]
    initParam("C_acid", 0.1);   // acid concentration [mol/L]
    initParam("C_base", 0.1);   // base concentration [mol/L]
    initParam("pKa", 4.7);      // dissociation constant
    initParam("A_tank", 0.04);  // tank cross-section [m²]
}

StateVector pHNeutralization::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double F_acid = params_.at("F_acid");
    double C_acid = params_.at("C_acid"), C_base = params_.at("C_base");
    double A_tank = params_.at("A_tank");
    double F_base = u.empty() ? 0.01 : u[0];

    double Wa = s[0]; // acid invariant
    double Wb = s[1]; // base invariant
    double h = s[2];  // liquid level [m]

    double F_out = F_acid + F_base;
    double V_cur = A_tank * std::max(h, 0.01);

    double dWa = (F_acid * C_acid - F_out * Wa) / V_cur;
    double dWb = (F_base * C_base - F_out * Wb) / V_cur;
    double dh = (F_acid + F_base - F_out) / (A_tank * 1000.0);

    return {dWa, dWb, dh};
}

StateVector pHNeutralization::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    double pKa = params_.at("pKa");
    double Wa = std::max(std::abs(s[0]), 1e-10);
    double Wb = std::max(std::abs(s[1]), 1e-10);
    double pH = pKa + std::log10(Wb / Wa);
    return {std::clamp(pH, 0.0, 14.0)};
}
StateVector pHNeutralization::defaultInitialState() const { return {0.05, 0.02, 0.5}; }

std::vector<ParamDescriptor> pHNeutralization::parameterDescriptors() const {
    return {{"V","L","Nominal volume",2.0,0.1,100.0,0.1},
            {"F_acid","L/s","Acid flow",0.01,0.001,0.1,0.001},
            {"C_acid","mol/L","Acid conc",0.1,0.001,1.0,0.001},
            {"C_base","mol/L","Base conc",0.1,0.001,1.0,0.001},
            {"pKa","","pKa",4.7,1.0,10.0,0.1},
            {"A_tank","m²","Tank area",0.04,0.001,1.0,0.001}};
}

std::vector<Preset> pHNeutralization::presets() const {
    return {{"Weak acid","Acetic acid + NaOH",
             {{"V",2.0},{"F_acid",0.01},{"C_acid",0.1},{"C_base",0.1},{"pKa",4.7},{"A_tank",0.04}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Strong acid","HCl + NaOH",
             {{"V",2.0},{"F_acid",0.01},{"C_acid",0.1},{"C_base",0.1},{"pKa",0.0},{"A_tank",0.04}}}};
}
void pHNeutralization::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> pHNeutralization::stateNames() const { return {"Wa","Wb","h"}; }
std::vector<std::string> pHNeutralization::outputNames() const { return {"pH"}; }
std::vector<std::string> pHNeutralization::inputNames() const { return {"Base flow rate"}; }

}  // namespace Simulation
