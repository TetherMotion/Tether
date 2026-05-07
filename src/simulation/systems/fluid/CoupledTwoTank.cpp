#include "tether/simulation/systems/fluid/CoupledTwoTank.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

CoupledTwoTank::CoupledTwoTank() {
    initParam("A1", 1.0); initParam("A2", 1.0);
    initParam("a1", 0.01); initParam("a2", 0.01); initParam("a12", 0.005);
    initParam("Cd", 0.6); initParam("g", 9.81);
}

StateVector CoupledTwoTank::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double A1 = params_.at("A1"), A2 = params_.at("A2");
    double a1 = params_.at("a1"), a2 = params_.at("a2"), a12 = params_.at("a12");
    double Cd = params_.at("Cd"), g = params_.at("g");
    double Q1 = u.size()>0 ? u[0] : 0.0;
    double Q2 = u.size()>1 ? u[1] : 0.0;

    double h1 = std::max(s[0], 0.0), h2 = std::max(s[1], 0.0);
    double sq = std::sqrt(2.0*g);
    double Q12 = Cd*a12*sq * (h1>h2 ? std::sqrt(h1-h2) : -std::sqrt(h2-h1));
    double Qout1 = Cd*a1*sq*std::sqrt(h1);
    double Qout2 = Cd*a2*sq*std::sqrt(h2);

    return {(Q1 - Qout1 - Q12)/A1,
            (Q2 - Qout2 + Q12)/A2};
}

StateVector CoupledTwoTank::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1]}; }
StateVector CoupledTwoTank::defaultInitialState() const { return {0.5, 0.3}; }

std::vector<ParamDescriptor> CoupledTwoTank::parameterDescriptors() const {
    return {{"A1","m²","Tank 1 area",1.0,0.01,10.0,0.01},
            {"A2","m²","Tank 2 area",1.0,0.01,10.0,0.01},
            {"a1","m²","Outlet 1",0.01,1e-5,0.1,1e-4},
            {"a2","m²","Outlet 2",0.01,1e-5,0.1,1e-4},
            {"a12","m²","Coupling",0.005,1e-5,0.1,1e-4},
            {"Cd","","Discharge coeff",0.6,0.1,1.0,0.01},
            {"g","m/s²","Gravity",9.81,0.0,20.0,0.01}};
}

std::vector<Preset> CoupledTwoTank::presets() const {
    return {{"Standard","Equal tanks",{{"A1",1.0},{"A2",1.0},{"a1",0.01},{"a2",0.01},{"a12",0.005},{"Cd",0.6},{"g",9.81}}}};
}
void CoupledTwoTank::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> CoupledTwoTank::stateNames() const { return {"h₁","h₂"}; }
std::vector<std::string> CoupledTwoTank::outputNames() const { return {"Level 1","Level 2"}; }
std::vector<std::string> CoupledTwoTank::inputNames() const { return {"Inflow 1","Inflow 2"}; }

}  // namespace Simulation
