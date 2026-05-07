#include "tether/simulation/systems/aerospace/Hovercraft2D.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

Hovercraft2D::Hovercraft2D() {
    initParam("m_hc", 5.0); initParam("Izz_hc", 0.5);
    initParam("kd_hc", 0.1); initParam("kr_hc", 0.05);
}

StateVector Hovercraft2D::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m_hc"), Iz = params_.at("Izz_hc");
    double kd = params_.at("kd_hc"), kr = params_.at("kr_hc");
    double F = u.size()>0 ? u[0] : 0.0; // forward thrust
    double tau = u.size()>1 ? u[1] : 0.0; // yaw torque

    double x = s[0], dx = s[1], y = s[2], dy = s[3], th = s[4], dth = s[5];
    (void)x; (void)y;

    double ddx = (F*std::cos(th) - kd*dx) / m;
    double ddy = (F*std::sin(th) - kd*dy) / m;
    double ddth = (tau - kr*dth) / Iz;

    return {dx, ddx, dy, ddy, dth, ddth};
}

StateVector Hovercraft2D::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2], s[4]}; }
StateVector Hovercraft2D::defaultInitialState() const { return {0,0,0,0,0,0}; }

std::vector<ParamDescriptor> Hovercraft2D::parameterDescriptors() const {
    return {{"m_hc","kg","Mass",5.0,0.5,50.0,0.1},{"Izz_hc","kg·m²","Yaw inertia",0.5,0.01,10.0,0.01},
            {"kd_hc","","Linear drag",0.1,0.0,5.0,0.01},{"kr_hc","","Rotational drag",0.05,0.0,5.0,0.01}};
}

std::vector<Preset> Hovercraft2D::presets() const {
    return {{"Standard","Low friction",{{"m_hc",5.0},{"Izz_hc",0.5},{"kd_hc",0.1},{"kr_hc",0.05}}}};
}
void Hovercraft2D::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> Hovercraft2D::stateNames() const { return {"x","dx","y","dy","θ","dθ"}; }
std::vector<std::string> Hovercraft2D::outputNames() const { return {"x","y","θ"}; }
std::vector<std::string> Hovercraft2D::inputNames() const { return {"Thrust","Torque"}; }

}  // namespace Simulation
