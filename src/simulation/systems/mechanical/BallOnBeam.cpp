#include "tether/simulation/systems/mechanical/BallOnBeam.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BallOnBeam::BallOnBeam() {
    initParam("m", 0.05); initParam("R", 0.01); initParam("g", 9.81);
    initParam("L", 1.0); initParam("Jb", 0.02);
}

StateVector BallOnBeam::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), R = params_.at("R"), g = params_.at("g");
    double Jb = params_.at("Jb");
    double tau = u.empty() ? 0.0 : u[0];

    double r = s[0], dr = s[1], th = s[2], dth = s[3];
    double J_ball = 2.0/5.0 * m * R * R;
    double denom = m + J_ball / (R * R);
    double ddr = (m*g*std::sin(th) + m*r*dth*dth) / denom;
    double ddth = (tau - m*g*r*std::cos(th) - 2.0*m*r*dr*dth) / (Jb + m*r*r);

    return {dr, ddr, dth, ddth};
}

StateVector BallOnBeam::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector BallOnBeam::defaultInitialState() const { return {0.2, 0, 0, 0}; }

std::vector<ParamDescriptor> BallOnBeam::parameterDescriptors() const {
    return {{"m","kg","Ball mass",0.05,0.001,1.0,0.001},{"R","m","Ball radius",0.01,0.001,0.1,0.001},
            {"g","m/s²","Gravity",9.81,0.1,20.0,0.01},{"L","m","Beam length",1.0,0.1,5.0,0.1},
            {"Jb","kg·m²","Beam inertia",0.02,0.001,1.0,0.001}};
}

std::vector<Preset> BallOnBeam::presets() const {
    return {{"Standard","Default",{{"m",0.05},{"R",0.01},{"g",9.81},{"L",1.0},{"Jb",0.02}}}};
}
void BallOnBeam::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BallOnBeam::stateNames() const { return {"r","dr/dt","θ","dθ/dt"}; }
std::vector<std::string> BallOnBeam::outputNames() const { return {"Ball position"}; }
std::vector<std::string> BallOnBeam::inputNames() const { return {"Beam torque"}; }

}  // namespace Simulation
