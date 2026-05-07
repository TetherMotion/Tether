#include "tether/simulation/systems/chemical/Bioreactor.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

Bioreactor::Bioreactor() {
    initParam("mu_max", 0.4);  // max specific growth rate [1/h]
    initParam("Ks", 0.1);     // half-saturation constant [g/L]
    initParam("Yxs", 0.5);    // yield coefficient [g/g]
    initParam("Sf", 10.0);    // feed substrate concentration [g/L]
    initParam("kd", 0.01);    // death rate [1/h]
}

StateVector Bioreactor::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double mu_max = params_.at("mu_max"), Ks = params_.at("Ks");
    double Yxs = params_.at("Yxs"), Sf = params_.at("Sf"), kd = params_.at("kd");
    double F = u.empty() ? 0.01 : u[0]; // feed rate [L/h]

    double X = std::max(s[0], 0.0);  // biomass [g/L]
    double S = std::max(s[1], 0.0);  // substrate [g/L]
    double V = std::max(s[2], 0.01); // volume [L]

    double D = F / V;
    double mu = mu_max * S / (Ks + S);

    double dX = (mu - kd - D) * X;
    double dS = -mu * X / Yxs + D * (Sf - S);
    double dV = F;

    return {dX, dS, dV};
}

StateVector Bioreactor::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {std::max(s[0], 0.0), std::max(s[1], 0.0)};
}
StateVector Bioreactor::defaultInitialState() const { return {1.0, 5.0, 1.0}; }

std::vector<ParamDescriptor> Bioreactor::parameterDescriptors() const {
    return {{"mu_max","1/h","Max growth rate",0.4,0.01,2.0,0.01},
            {"Ks","g/L","Half-saturation",0.1,0.001,5.0,0.01},
            {"Yxs","g/g","Yield coefficient",0.5,0.01,1.0,0.01},
            {"Sf","g/L","Feed substrate",10.0,0.1,100.0,0.1},
            {"kd","1/h","Death rate",0.01,0.0,0.2,0.001}};
}

std::vector<Preset> Bioreactor::presets() const {
    return {{"E. coli","Fast growth",{{"mu_max",0.8},{"Ks",0.05},{"Yxs",0.5},{"Sf",10.0},{"kd",0.01}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Yeast","Moderate growth",{{"mu_max",0.3},{"Ks",0.2},{"Yxs",0.4},{"Sf",20.0},{"kd",0.005}}}};
}
void Bioreactor::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> Bioreactor::stateNames() const { return {"X","S","V"}; }
std::vector<std::string> Bioreactor::outputNames() const { return {"Biomass","Substrate"}; }
std::vector<std::string> Bioreactor::inputNames() const { return {"Feed rate"}; }

}  // namespace Simulation
