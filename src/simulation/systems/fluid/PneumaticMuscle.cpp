#include "tether/simulation/systems/fluid/PneumaticMuscle.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PneumaticMuscle::PneumaticMuscle() {
    initParam("m_pm", 1.0);    // load mass [kg]
    initParam("b_pm", 10.0);   // damping [N·s/m]
    initParam("k_pm", 100.0);  // passive stiffness [N/m]
    initParam("L0", 0.3);      // resting length [m]
    initParam("D0", 0.02);     // resting diameter [m]
    initParam("n_pm", 3.0);    // braid angle parameter
    initParam("V_pm", 1e-4);   // internal volume [m³]
    initParam("Patm", 101325.0); // atmospheric pressure
}

StateVector PneumaticMuscle::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m_pm"), b = params_.at("b_pm"), k = params_.at("k_pm");
    double L0 = params_.at("L0"), Patm = params_.at("Patm");
    double Qin = u.empty() ? 0.0 : u[0]; // air flow [kg/s]

    double x = s[0], dx = s[1], P = s[2]; // contraction, velocity, gauge pressure

    // McKibben force model (simplified)
    double epsilon = std::min(x / L0, 0.3); // contraction ratio
    double F_muscle = (P - Patm) * L0 * L0 * (3.0 * (1.0-epsilon)*(1.0-epsilon) - 1.0) * 0.01;
    F_muscle = std::max(F_muscle, 0.0); // can only pull

    double ddx = (F_muscle - k*x - b*dx) / m;
    double dP = 1e6 * Qin; // simplified pressure dynamics

    return {dx, ddx, dP};
}

StateVector PneumaticMuscle::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector PneumaticMuscle::defaultInitialState() const { return {0.0, 0.0, 101325.0}; }

std::vector<ParamDescriptor> PneumaticMuscle::parameterDescriptors() const {
    return {{"m_pm","kg","Load mass",1.0,0.01,50.0,0.01},
            {"b_pm","N·s/m","Damping",10.0,0.1,1000.0,0.1},
            {"k_pm","N/m","Stiffness",100.0,1.0,10000.0,1.0},
            {"L0","m","Rest length",0.3,0.05,1.0,0.01},
            {"Patm","Pa","Atm pressure",101325.0,90000.0,110000.0,100.0}};
}

std::vector<Preset> PneumaticMuscle::presets() const {
    return {{"Standard","20mm McKibben",{{"m_pm",1.0},{"b_pm",10.0},{"k_pm",100.0},{"L0",0.3},{"D0",0.02},{"n_pm",3.0},{"V_pm",1e-4},{"Patm",101325.0}}}};
}
void PneumaticMuscle::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PneumaticMuscle::stateNames() const { return {"x","dx","P"}; }
std::vector<std::string> PneumaticMuscle::outputNames() const { return {"Contraction"}; }
std::vector<std::string> PneumaticMuscle::inputNames() const { return {"Air flow"}; }

}  // namespace Simulation
