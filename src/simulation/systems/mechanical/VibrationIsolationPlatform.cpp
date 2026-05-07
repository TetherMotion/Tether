#include "tether/simulation/systems/mechanical/VibrationIsolationPlatform.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

VibrationIsolationPlatform::VibrationIsolationPlatform() {
    initParam("m", 50.0); initParam("I", 5.0);
    initParam("k1", 10000.0); initParam("k2", 10000.0); initParam("k3", 10000.0);
    initParam("c1", 100.0); initParam("c2", 100.0); initParam("c3", 100.0);
    initParam("r1x", -0.3); initParam("r1y", -0.2);
    initParam("r2x", 0.3); initParam("r2y", -0.2);
    initParam("r3x", 0.0); initParam("r3y", 0.3);
}

StateVector VibrationIsolationPlatform::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), I = params_.at("I");
    double k1 = params_.at("k1"), k2 = params_.at("k2"), k3 = params_.at("k3");
    double c1 = params_.at("c1"), c2 = params_.at("c2"), c3 = params_.at("c3");
    double F1 = u.size()>0 ? u[0] : 0, F2 = u.size()>1 ? u[1] : 0, F3 = u.size()>2 ? u[2] : 0;

    double x = s[0], dx = s[1], y = s[2], dy = s[3], th = s[4], dth = s[5];

    double ddx = (-k1*x - k2*x - k3*x - c1*dx - c2*dx - c3*dx + F1 + F2 + F3) / m;
    double ddy = (-k1*y - k2*y - k3*y - c1*dy - c2*dy - c3*dy) / m;
    double ddth = (-0.1*(k1+k2+k3)*th - 0.1*(c1+c2+c3)*dth) / I;

    return {dx, ddx, dy, ddy, dth, ddth};
}

StateVector VibrationIsolationPlatform::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2], s[4]}; }
StateVector VibrationIsolationPlatform::defaultInitialState() const { return {0,0,0,0,0,0}; }

std::vector<ParamDescriptor> VibrationIsolationPlatform::parameterDescriptors() const {
    return {{"m","kg","Platform mass",50.0,1.0,500.0,1.0},{"I","kg·m²","Moment of inertia",5.0,0.1,100.0,0.1},
            {"k1","N/m","Spring 1",10000.0,100.0,100000.0,100.0},{"k2","N/m","Spring 2",10000.0,100.0,100000.0,100.0},
            {"k3","N/m","Spring 3",10000.0,100.0,100000.0,100.0},
            {"c1","N·s/m","Damper 1",100.0,0.0,10000.0,10.0},{"c2","N·s/m","Damper 2",100.0,0.0,10000.0,10.0},
            {"c3","N·s/m","Damper 3",100.0,0.0,10000.0,10.0}};
}

std::vector<Preset> VibrationIsolationPlatform::presets() const {
    return {{"Standard","Default",{{"m",50.0},{"I",5.0},{"k1",10000.0},{"k2",10000.0},{"k3",10000.0},{"c1",100.0},{"c2",100.0},{"c3",100.0}}}};
}
void VibrationIsolationPlatform::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> VibrationIsolationPlatform::stateNames() const { return {"x","dx/dt","y","dy/dt","θ","dθ/dt"}; }
std::vector<std::string> VibrationIsolationPlatform::outputNames() const { return {"X","Y","Rotation"}; }
std::vector<std::string> VibrationIsolationPlatform::inputNames() const { return {"Force 1","Force 2","Force 3"}; }

}  // namespace Simulation

