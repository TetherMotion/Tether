#include "tether/simulation/systems/mechanical/GantryCrane.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

GantryCrane::GantryCrane() {
    initParam("M", 10.0); initParam("m", 5.0); initParam("l", 2.0);
    initParam("g", 9.81); initParam("b", 1.0);
}

StateVector GantryCrane::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double M = params_.at("M"), m = params_.at("m"), l = params_.at("l");
    double g = params_.at("g"), b = params_.at("b");
    double F = u.empty() ? 0.0 : u[0];
    double dx = s[1], th = s[2], dth = s[3];
    double sinT = std::sin(th), cosT = std::cos(th);
    double denom = M + m*sinT*sinT;
    double ddx = (F - b*dx + m*sinT*(l*dth*dth + g*cosT)) / denom;
    double ddth = (-cosT*(F - b*dx) - m*l*dth*dth*sinT*cosT - (M+m)*g*sinT) / (l*denom);
    return {dx, ddx, dth, ddth};
}

StateVector GantryCrane::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector GantryCrane::defaultInitialState() const { return {0, 0, 0.2, 0}; }

std::vector<ParamDescriptor> GantryCrane::parameterDescriptors() const {
    return {{"M","kg","Trolley mass",10.0,1.0,100.0,1.0},{"m","kg","Payload mass",5.0,0.1,50.0,0.1},
            {"l","m","Cable length",2.0,0.1,20.0,0.1},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"b","N·s/m","Friction",1.0,0.0,20.0,0.1}};
}

std::vector<Preset> GantryCrane::presets() const {
    return {{"Industrial","Large crane",{{"M",50.0},{"m",20.0},{"l",10.0},{"g",9.81},{"b",5.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Lab","Small crane",{{"M",2.0},{"m",0.5},{"l",1.0},{"g",9.81},{"b",0.2}}}};
}
void GantryCrane::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> GantryCrane::stateNames() const { return {"x","dx/dt","θ","dθ/dt"}; }
std::vector<std::string> GantryCrane::outputNames() const { return {"Trolley pos","Swing angle"}; }
std::vector<std::string> GantryCrane::inputNames() const { return {"Trolley force"}; }

}  // namespace Simulation
