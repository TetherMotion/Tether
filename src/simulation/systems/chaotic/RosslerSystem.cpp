#include "tether/simulation/systems/chaotic/RosslerSystem.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

RosslerSystem::RosslerSystem() {
    initParam("a", 0.2); initParam("b", 0.2); initParam("c_r", 5.7);
}

StateVector RosslerSystem::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double a = params_.at("a"), b = params_.at("b"), c = params_.at("c_r");
    double ux = u.empty() ? 0.0 : u[0];
    double x = s[0], y = s[1], z = s[2];
    return {-y - z + ux, x + a*y, b + z*(x - c)};
}

StateVector RosslerSystem::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1], s[2]}; }
StateVector RosslerSystem::defaultInitialState() const { return {1.0, 1.0, 0.0}; }

std::vector<ParamDescriptor> RosslerSystem::parameterDescriptors() const {
    return {{"a","","a",0.2,0.01,1.0,0.01},{"b","","b",0.2,0.01,1.0,0.01},{"c_r","","c",5.7,0.1,20.0,0.1}};
}

std::vector<Preset> RosslerSystem::presets() const {
    return {{"Classic","a=0.2,b=0.2,c=5.7",{{"a",0.2},{"b",0.2},{"c_r",5.7}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Period-2","c=4",{{"a",0.2},{"b",0.2},{"c_r",4.0}}}};
}
void RosslerSystem::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> RosslerSystem::stateNames() const { return {"x","y","z"}; }
std::vector<std::string> RosslerSystem::outputNames() const { return {"x","y","z"}; }
std::vector<std::string> RosslerSystem::inputNames() const { return {"Perturbation"}; }

}  // namespace Simulation
