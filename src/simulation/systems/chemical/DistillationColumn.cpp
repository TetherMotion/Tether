#include "tether/simulation/systems/chemical/DistillationColumn.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DistillationColumn::DistillationColumn() {
    initParam("alpha_rel", 2.0); // relative volatility
    initParam("M", 0.5);        // tray holdup [mol]
    initParam("zF", 0.5);       // feed composition
    initParam("F_feed", 1.0);   // feed flow [mol/s]
    initParam("q", 1.0);        // feed thermal condition
}

StateVector DistillationColumn::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double alpha = params_.at("alpha_rel"), M = params_.at("M");
    double zF = params_.at("zF"), F_f = params_.at("F_feed"), q = params_.at("q");

    double L = u.size() > 0 ? u[0] : 1.5;  // reflux [mol/s]
    double V = u.size() > 1 ? u[1] : 2.0;  // boilup [mol/s]

    double x1 = std::clamp(s[0], 0.0, 1.0); // top
    double x2 = std::clamp(s[1], 0.0, 1.0); // rectifying
    double x3 = std::clamp(s[2], 0.0, 1.0); // stripping
    double x4 = std::clamp(s[3], 0.0, 1.0); // bottom

    auto vle = [alpha](double x) { return alpha*x / (1.0 + (alpha-1.0)*x); };
    double y1 = vle(x1), y2 = vle(x2), y3 = vle(x3), y4 = vle(x4);

    double dx1 = (V*y2 - L*x1 - (V-L)*y1) / std::max(M, 0.01);
    double dx2 = (L*(x1-x2) + V*(y3-y2)) / std::max(M, 0.01);
    double dx3 = (L*(x2-x3) + V*(y4-y3) + F_f*zF*q) / std::max(M, 0.01);
    double dx4 = (L*(x3-x4) - V*y4 + V*vle(0.01)) / std::max(M, 0.01);

    return {dx1, dx2, dx3, dx4};
}

StateVector DistillationColumn::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {std::clamp(s[0], 0.0, 1.0), std::clamp(s[3], 0.0, 1.0)};
}
StateVector DistillationColumn::defaultInitialState() const { return {0.9, 0.7, 0.3, 0.1}; }

std::vector<ParamDescriptor> DistillationColumn::parameterDescriptors() const {
    return {{"alpha_rel","","Relative volatility",2.0,1.1,10.0,0.1},
            {"M","mol","Tray holdup",0.5,0.01,10.0,0.01},
            {"zF","","Feed composition",0.5,0.0,1.0,0.01},
            {"F_feed","mol/s","Feed flow",1.0,0.01,10.0,0.01},
            {"q","","Feed condition",1.0,0.0,2.0,0.1}};
}

std::vector<Preset> DistillationColumn::presets() const {
    return {{"Standard","α=2",{{"alpha_rel",2.0},{"M",0.5},{"zF",0.5},{"F_feed",1.0},{"q",1.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"High purity","α=3",{{"alpha_rel",3.0},{"M",0.5},{"zF",0.5},{"F_feed",1.0},{"q",1.0}}}};
}
void DistillationColumn::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DistillationColumn::stateNames() const { return {"x_top","x_rect","x_strip","x_bot"}; }
std::vector<std::string> DistillationColumn::outputNames() const { return {"Distillate comp","Bottoms comp"}; }
std::vector<std::string> DistillationColumn::inputNames() const { return {"Reflux L","Boilup V"}; }

}  // namespace Simulation
