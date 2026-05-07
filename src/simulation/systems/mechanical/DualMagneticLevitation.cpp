#include "tether/simulation/systems/mechanical/DualMagneticLevitation.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DualMagneticLevitation::DualMagneticLevitation() {
    initParam("m1", 0.05); initParam("m2", 0.05); initParam("g", 9.81);
    initParam("R1", 10.0); initParam("R2", 10.0);
    initParam("L0", 0.1); initParam("C", 1e-4);
    initParam("k_coupling", 0.01);
}

StateVector DualMagneticLevitation::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m1 = params_.at("m1"), m2 = params_.at("m2"), g = params_.at("g");
    double R1 = params_.at("R1"), R2 = params_.at("R2");
    double C = params_.at("C"), L0 = params_.at("L0");
    double kc = params_.at("k_coupling");
    double V1 = u.size() > 0 ? u[0] : 0.0, V2 = u.size() > 1 ? u[1] : 0.0;

    double y1 = std::max(s[0], 1e-6), dy1 = s[1], i1 = s[2];
    double y2 = std::max(s[3], 1e-6), dy2 = s[4], i2 = s[5];

    double F1 = C * i1 * i1 / (y1 * y1);
    double F2 = C * i2 * i2 / (y2 * y2);
    double Fc = kc * (y2 - y1); // magnetic coupling

    double ddy1 = g - F1/m1 - Fc/m1;
    double ddy2 = g - F2/m2 + Fc/m2;
    double di1 = (V1 - R1*i1) / L0;
    double di2 = (V2 - R2*i2) / L0;

    return {dy1, ddy1, di1, dy2, ddy2, di2};
}

StateVector DualMagneticLevitation::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[3]}; }
StateVector DualMagneticLevitation::defaultInitialState() const { return {0.01, 0, 0.5, 0.01, 0, 0.5}; }

std::vector<ParamDescriptor> DualMagneticLevitation::parameterDescriptors() const {
    return {{"m1","kg","Ball 1 mass",0.05,0.001,1.0,0.001},{"m2","kg","Ball 2 mass",0.05,0.001,1.0,0.001},
            {"R1","Ω","Coil 1 R",10.0,0.1,100.0,0.1},{"R2","Ω","Coil 2 R",10.0,0.1,100.0,0.1},
            {"k_coupling","N/m","Coupling",0.01,0.0,1.0,0.001}};
}

std::vector<Preset> DualMagneticLevitation::presets() const {
    return {{"Standard","Default",{{"m1",0.05},{"m2",0.05},{"g",9.81},{"R1",10.0},{"R2",10.0},{"L0",0.1},{"C",1e-4},{"k_coupling",0.01}}}};
}
void DualMagneticLevitation::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DualMagneticLevitation::stateNames() const { return {"y1","dy1/dt","i1","y2","dy2/dt","i2"}; }
std::vector<std::string> DualMagneticLevitation::outputNames() const { return {"Ball 1 pos","Ball 2 pos"}; }
std::vector<std::string> DualMagneticLevitation::inputNames() const { return {"Voltage 1","Voltage 2"}; }

}  // namespace Simulation
