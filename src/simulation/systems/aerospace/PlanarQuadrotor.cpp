#include "tether/simulation/systems/aerospace/PlanarQuadrotor.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PlanarQuadrotor::PlanarQuadrotor() {
    initParam("m", 1.0); initParam("g", 9.81); initParam("l_arm", 0.25);
    initParam("Izz", 0.02); initParam("kd", 0.1);
}

StateVector PlanarQuadrotor::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), g = params_.at("g"), l = params_.at("l_arm"), Iz = params_.at("Izz"), kd = params_.at("kd");
    double f1 = u.size()>0 ? u[0] : m*g/2.0;
    double f2 = u.size()>1 ? u[1] : m*g/2.0;

    double x = s[0], dx = s[1], y = s[2], dy = s[3], th = s[4], dth = s[5];
    (void)x; (void)y;
    double F = f1 + f2;
    double ddx = -F*std::sin(th)/m - kd*dx/m;
    double ddy = F*std::cos(th)/m - g - kd*dy/m;
    double ddth = l*(f1 - f2)/Iz;
    return {dx, ddx, dy, ddy, dth, ddth};
}

StateVector PlanarQuadrotor::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2], s[4]}; }
StateVector PlanarQuadrotor::defaultInitialState() const { return {0,0,1.0,0,0,0}; }

std::vector<ParamDescriptor> PlanarQuadrotor::parameterDescriptors() const {
    return {{"m","kg","Mass",1.0,0.1,10.0,0.1},{"g","m/s²","Gravity",9.81,0.0,20.0,0.01},
            {"l_arm","m","Arm length",0.25,0.05,1.0,0.01},{"Izz","kg·m²","Inertia",0.02,0.001,1.0,0.001},
            {"kd","","Drag coeff",0.1,0.0,5.0,0.01}};
}

std::vector<Preset> PlanarQuadrotor::presets() const {
    return {{"Small","250g drone",{{"m",0.25},{"g",9.81},{"l_arm",0.1},{"Izz",0.003},{"kd",0.05}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Standard","1kg drone",{{"m",1.0},{"g",9.81},{"l_arm",0.25},{"Izz",0.02},{"kd",0.1}}}};
}
void PlanarQuadrotor::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PlanarQuadrotor::stateNames() const { return {"x","dx","y","dy","θ","dθ"}; }
std::vector<std::string> PlanarQuadrotor::outputNames() const { return {"x","y","θ"}; }
std::vector<std::string> PlanarQuadrotor::inputNames() const { return {"Thrust L","Thrust R"}; }

}  // namespace Simulation
