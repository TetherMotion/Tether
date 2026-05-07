#include "tether/simulation/systems/mechanical/MagneticLevitation.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

MagneticLevitation::MagneticLevitation() {
    initParam("m", 0.05); initParam("g", 9.81);
    initParam("R", 10.0); initParam("L0", 0.1); initParam("C", 1e-4);
    initParam("y0", 0.01);
}

StateVector MagneticLevitation::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), g = params_.at("g");
    double R = params_.at("R"), C = params_.at("C"), y0 = params_.at("y0");
    double V = u.empty() ? 0.0 : u[0];

    double y = std::max(s[0], 1e-6), dy = s[1], i = s[2];
    // Electromagnetic force: F = C * i^2 / y^2
    double Fmag = C * i * i / (y * y);
    double ddy = g - Fmag / m;
    // Inductance varies: L(y) = L0 * y0 / y
    double L0 = params_.at("L0");
    double L = L0 * y0 / y;
    double dLdy = -L0 * y0 / (y * y);
    double di = (V - R * i + dLdy * dy * i) / L;

    return {dy, ddy, di};
}

StateVector MagneticLevitation::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector MagneticLevitation::defaultInitialState() const { return {0.01, 0, 0.5}; }

std::vector<ParamDescriptor> MagneticLevitation::parameterDescriptors() const {
    return {{"m","kg","Ball mass",0.05,0.001,1.0,0.001},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"R","Ω","Coil resistance",10.0,0.1,100.0,0.1},{"L0","H","Nominal inductance",0.1,0.001,1.0,0.001},
            {"C","N·m²/A²","Force constant",1e-4,1e-6,1e-2,1e-6,true},{"y0","m","Reference gap",0.01,0.001,0.1,0.001}};
}

std::vector<Preset> MagneticLevitation::presets() const {
    return {{"Lab maglev","Standard lab",{{"m",0.05},{"g",9.81},{"R",10.0},{"L0",0.1},{"C",1e-4},{"y0",0.01}}}};
}
void MagneticLevitation::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> MagneticLevitation::stateNames() const { return {"y","dy/dt","i"}; }
std::vector<std::string> MagneticLevitation::outputNames() const { return {"Ball position"}; }
std::vector<std::string> MagneticLevitation::inputNames() const { return {"Voltage"}; }

}  // namespace Simulation
