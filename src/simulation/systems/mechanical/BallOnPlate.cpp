#include "tether/simulation/systems/mechanical/BallOnPlate.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

BallOnPlate::BallOnPlate() {
    initParam("m", 0.05); initParam("R", 0.01); initParam("g", 9.81);
    initParam("Lx", 0.5); initParam("Ly", 0.5);
    initParam("Jpx", 0.02); initParam("Jpy", 0.02);
}

StateVector BallOnPlate::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m"), R = params_.at("R"), g = params_.at("g");
    double Jpx = params_.at("Jpx"), Jpy = params_.at("Jpy");
    double taux = u.size() > 0 ? u[0] : 0.0;
    double tauy = u.size() > 1 ? u[1] : 0.0;
    double J_ball = 2.0/5.0 * m * R * R;
    double denom = m + J_ball / (R * R);

    double rx = s[0], drx = s[1], ry = s[2], dry = s[3];
    double thx = s[4], dthx = s[5], thy = s[6], dthy = s[7];

    double ddrx = (m*g*std::sin(thx) + m*rx*dthx*dthx) / denom;
    double ddry = (m*g*std::sin(thy) + m*ry*dthy*dthy) / denom;
    double ddthx = (taux - m*g*rx*std::cos(thx)) / (Jpx + m*rx*rx);
    double ddthy = (tauy - m*g*ry*std::cos(thy)) / (Jpy + m*ry*ry);

    return {drx, ddrx, dry, ddry, dthx, ddthx, dthy, ddthy};
}

StateVector BallOnPlate::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector BallOnPlate::defaultInitialState() const { return {0.1, 0, 0.1, 0, 0, 0, 0, 0}; }

std::vector<ParamDescriptor> BallOnPlate::parameterDescriptors() const {
    return {{"m","kg","Ball mass",0.05,0.001,1.0,0.001},{"R","m","Ball radius",0.01,0.001,0.1,0.001},
            {"g","m/s²","Gravity",9.81,0.1,20.0,0.01},{"Lx","m","Plate X",0.5,0.1,2.0,0.1},
            {"Ly","m","Plate Y",0.5,0.1,2.0,0.1},{"Jpx","kg·m²","Inertia X",0.02,0.001,1.0,0.001},
            {"Jpy","kg·m²","Inertia Y",0.02,0.001,1.0,0.001}};
}

std::vector<Preset> BallOnPlate::presets() const {
    return {{"Standard","Default",{{"m",0.05},{"R",0.01},{"g",9.81},{"Lx",0.5},{"Ly",0.5},{"Jpx",0.02},{"Jpy",0.02}}}};
}
void BallOnPlate::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> BallOnPlate::stateNames() const { return {"rx","drx/dt","ry","dry/dt","θx","dθx/dt","θy","dθy/dt"}; }
std::vector<std::string> BallOnPlate::outputNames() const { return {"Ball X","Ball Y"}; }
std::vector<std::string> BallOnPlate::inputNames() const { return {"Torque X","Torque Y"}; }

}  // namespace Simulation
