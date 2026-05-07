#include "tether/simulation/systems/mechanical/BouncingBall.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BouncingBall::BouncingBall() {
    initParam("m", 0.05); initParam("g", 9.81); initParam("e", 0.8);
    initParam("k_contact", 1e5); initParam("c_contact", 100.0);
}

StateVector BouncingBall::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), g = params_.at("g");
    double k_c = params_.at("k_contact"), c_c = params_.at("c_contact");
    double F = u.empty() ? 0.0 : u[0];

    double yb = s[0], dyb = s[1], yp = s[2], dyp = s[3];

    // Contact force (compliant contact model)
    double penetration = yp - yb;
    double Fc = 0.0;
    if (penetration > 0.0) {
        Fc = k_c * penetration + c_c * (dyp - dyb);
        if (Fc < 0.0) Fc = 0.0;
    }

    double ddyb = -g + Fc / m;
    double ddyp = F;  // plate acceleration directly controlled

    return {dyb, ddyb, dyp, ddyp};
}

StateVector BouncingBall::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector BouncingBall::defaultInitialState() const { return {0.5, 0, 0, 0}; }

std::vector<ParamDescriptor> BouncingBall::parameterDescriptors() const {
    return {{"m","kg","Ball mass",0.05,0.001,1.0,0.001},{"g","m/s²","Gravity",9.81,0.1,20.0,0.01},
            {"e","","Restitution coeff",0.8,0.0,1.0,0.01},
            {"k_contact","N/m","Contact stiffness",1e5,100.0,1e7,1000.0},
            {"c_contact","N·s/m","Contact damping",100.0,0.0,1e4,1.0}};
}

std::vector<Preset> BouncingBall::presets() const {
    return {{"Steel ball","High restitution",{{"m",0.05},{"g",9.81},{"e",0.9},{"k_contact",1e6},{"c_contact",50.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Rubber ball","Medium restitution",{{"m",0.05},{"g",9.81},{"e",0.7},{"k_contact",1e4},{"c_contact",200.0}}}};
}
void BouncingBall::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BouncingBall::stateNames() const { return {"y_ball","dy_ball/dt","y_plate","dy_plate/dt"}; }
std::vector<std::string> BouncingBall::outputNames() const { return {"Ball height"}; }
std::vector<std::string> BouncingBall::inputNames() const { return {"Plate accel"}; }

}  // namespace Simulation
