#include "tether/simulation/systems/aerospace/RocketLanding2D.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

RocketLanding2D::RocketLanding2D() {
    initParam("m_dry", 20.0); initParam("g", 9.81); initParam("T_max", 600.0);
    initParam("l_r", 2.0); initParam("Izz_r", 10.0); initParam("Isp", 200.0);
    initParam("kd_r", 0.05);
}

StateVector RocketLanding2D::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m_dry = params_.at("m_dry"), g = params_.at("g"), T_max = params_.at("T_max");
    double l = params_.at("l_r"), Iz = params_.at("Izz_r"), Isp = params_.at("Isp"), kd = params_.at("kd_r");
    double throttle = u.size()>0 ? std::clamp(u[0], 0.0, 1.0) : 0.0;
    double gimbal = u.size()>1 ? std::clamp(u[1], -0.3, 0.3) : 0.0; // [rad]

    double x = s[0], dx = s[1], y = s[2], dy = s[3], th = s[4], dth = s[5], mf = s[6];
    (void)x;
    double m_total = m_dry + std::max(mf, 0.0);
    double T = T_max * throttle;

    double ddx = T*std::sin(th + gimbal)/m_total - kd*dx/m_total;
    double ddy = T*std::cos(th + gimbal)/m_total - g - kd*dy/m_total;
    double ddth = -T*l*std::sin(gimbal)/Iz;
    double dm = -T / (Isp * g); // fuel consumption

    // Ground constraint
    if (y <= 0.0 && dy < 0.0) {
        return {0, 0, 0, 0, 0, 0, dm};
    }

    return {dx, ddx, dy, ddy, dth, ddth, dm};
}

StateVector RocketLanding2D::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2], s[4]}; }
StateVector RocketLanding2D::defaultInitialState() const { return {0, 5.0, 100.0, -10.0, 0.1, 0, 5.0}; }

std::vector<ParamDescriptor> RocketLanding2D::parameterDescriptors() const {
    return {{"m_dry","kg","Dry mass",20.0,1.0,1000.0,1.0},{"g","m/s²","Gravity",9.81,0.0,20.0,0.01},
            {"T_max","N","Max thrust",600.0,10.0,10000.0,10.0},{"l_r","m","Engine offset",2.0,0.1,10.0,0.1},
            {"Izz_r","kg·m²","Inertia",10.0,0.1,1000.0,0.1},{"Isp","s","Specific impulse",200.0,50.0,500.0,1.0},
            {"kd_r","","Drag",0.05,0.0,1.0,0.01}};
}

std::vector<Preset> RocketLanding2D::presets() const {
    return {{"Hopper","Small test vehicle",{{"m_dry",20.0},{"g",9.81},{"T_max",600.0},{"l_r",2.0},{"Izz_r",10.0},{"Isp",200.0},{"kd_r",0.05}}}};
}
void RocketLanding2D::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> RocketLanding2D::stateNames() const { return {"x","dx","y","dy","θ","dθ","m_fuel"}; }
std::vector<std::string> RocketLanding2D::outputNames() const { return {"x","y","θ"}; }
std::vector<std::string> RocketLanding2D::inputNames() const { return {"Throttle","Gimbal angle"}; }

}  // namespace Simulation
