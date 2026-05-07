#include "tether/simulation/systems/fluid/HydraulicActuator.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

HydraulicActuator::HydraulicActuator() {
    initParam("m_hyd", 10.0);  // piston mass [kg]
    initParam("A_pist", 0.01); // piston area [m²]
    initParam("b_hyd", 100.0); // damping [N·s/m]
    initParam("beta", 1e9);    // bulk modulus [Pa]
    initParam("V0", 1e-3);     // chamber volume [m³]
    initParam("Cd_hyd", 0.6);  // valve discharge coeff
    initParam("Ps", 20e6);     // supply pressure [Pa]
}

StateVector HydraulicActuator::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m_hyd"), Ap = params_.at("A_pist"), b = params_.at("b_hyd");
    double beta = params_.at("beta"), V0 = params_.at("V0");
    double Cd = params_.at("Cd_hyd"), Ps = params_.at("Ps");
    double xv = u.empty() ? 0.0 : u[0]; // valve spool position [-1,1]

    // double x = s[0], dx = s[1], P = s[2]; // position, velocity, chamber pressure
    double dx = s[1], P = s[2];  // x not used

    // Valve flow (simplified)
    double dP_valve = xv > 0 ? (Ps - P) : P; // pressure drop across valve
    double Q = Cd * xv * std::sqrt(std::max(std::abs(dP_valve), 0.0));
    if (dP_valve < 0) Q = -Q;

    double ddx = (Ap * P - b * dx) / m;
    double dP = beta * (Q - Ap * dx) / V0;

    return {dx, ddx, dP};
}

StateVector HydraulicActuator::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector HydraulicActuator::defaultInitialState() const { return {0.0, 0.0, 10e6}; }

std::vector<ParamDescriptor> HydraulicActuator::parameterDescriptors() const {
    return {{"m_hyd","kg","Piston mass",10.0,0.1,100.0,0.1},
            {"A_pist","m²","Piston area",0.01,1e-4,0.1,1e-4},
            {"b_hyd","N·s/m","Damping",100.0,1.0,10000.0,1.0},
            {"beta","Pa","Bulk modulus",1e9,1e7,2e9,1e7},
            {"V0","m³","Chamber volume",1e-3,1e-5,0.01,1e-5},
            {"Ps","Pa","Supply pressure",20e6,1e6,35e6,1e6}};
}

std::vector<Preset> HydraulicActuator::presets() const {
    return {{"Standard","Industrial actuator",{{"m_hyd",10.0},{"A_pist",0.01},{"b_hyd",100.0},{"beta",1e9},{"V0",1e-3},{"Cd_hyd",0.6},{"Ps",20e6}}}};
}
void HydraulicActuator::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> HydraulicActuator::stateNames() const { return {"x","dx","P"}; }
std::vector<std::string> HydraulicActuator::outputNames() const { return {"Position"}; }
std::vector<std::string> HydraulicActuator::inputNames() const { return {"Valve spool"}; }

}  // namespace Simulation
