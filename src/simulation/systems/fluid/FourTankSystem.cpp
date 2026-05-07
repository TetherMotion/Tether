#include "tether/simulation/systems/fluid/FourTankSystem.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

FourTankSystem::FourTankSystem() {
    initParam("A1", 28e-4); initParam("A2", 32e-4);
    initParam("A3", 28e-4); initParam("A4", 32e-4);
    initParam("a1", 0.071e-4); initParam("a2", 0.057e-4);
    initParam("a3", 0.071e-4); initParam("a4", 0.057e-4);
    initParam("g", 981.0); // cm/s² (Johansson uses cm)
    initParam("gamma1", 0.7); initParam("gamma2", 0.6); // flow split ratios
    initParam("k1", 3.33e-4); initParam("k2", 3.35e-4); // pump gains [cm³/s/V]
}

StateVector FourTankSystem::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double A1 = params_.at("A1"), A2 = params_.at("A2"), A3 = params_.at("A3"), A4 = params_.at("A4");
    double a1 = params_.at("a1"), a2 = params_.at("a2"), a3 = params_.at("a3"), a4 = params_.at("a4");
    double g = params_.at("g");
    double g1 = params_.at("gamma1"), g2 = params_.at("gamma2");
    double k1 = params_.at("k1"), k2 = params_.at("k2");

    double v1 = u.size()>0 ? u[0] : 3.0;
    double v2 = u.size()>1 ? u[1] : 3.0;

    double h1 = std::max(s[0], 0.0), h2 = std::max(s[1], 0.0);
    double h3 = std::max(s[2], 0.0), h4 = std::max(s[3], 0.0);

    double sq2g = std::sqrt(2.0*g);

    double dh1 = (-a1*sq2g*std::sqrt(h1) + a3*sq2g*std::sqrt(h3) + g1*k1*v1) / A1;
    double dh2 = (-a2*sq2g*std::sqrt(h2) + a4*sq2g*std::sqrt(h4) + g2*k2*v2) / A2;
    double dh3 = (-a3*sq2g*std::sqrt(h3) + (1.0-g2)*k2*v2) / A3;
    double dh4 = (-a4*sq2g*std::sqrt(h4) + (1.0-g1)*k1*v1) / A4;

    return {dh1, dh2, dh3, dh4};
}

StateVector FourTankSystem::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1]}; }
StateVector FourTankSystem::defaultInitialState() const { return {12.4, 12.7, 1.8, 1.4}; }

std::vector<ParamDescriptor> FourTankSystem::parameterDescriptors() const {
    return {{"A1","cm²","Tank 1 area",28e-4,1e-4,0.01,1e-5},
            {"A2","cm²","Tank 2 area",32e-4,1e-4,0.01,1e-5},
            {"gamma1","","Split 1",0.7,0.0,1.0,0.01},
            {"gamma2","","Split 2",0.6,0.0,1.0,0.01},
            {"k1","cm³/s/V","Pump 1 gain",3.33e-4,1e-5,0.01,1e-5},
            {"k2","cm³/s/V","Pump 2 gain",3.35e-4,1e-5,0.01,1e-5}};
}

std::vector<Preset> FourTankSystem::presets() const {
    return {{"Min phase","γ>0.5",{{"gamma1",0.7},{"gamma2",0.6}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Non-min phase","γ<0.5",{{"gamma1",0.3},{"gamma2",0.4}}}};
}
void FourTankSystem::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> FourTankSystem::stateNames() const { return {"h₁","h₂","h₃","h₄"}; }
std::vector<std::string> FourTankSystem::outputNames() const { return {"Level 1","Level 2"}; }
std::vector<std::string> FourTankSystem::inputNames() const { return {"Pump 1 V","Pump 2 V"}; }

}  // namespace Simulation
