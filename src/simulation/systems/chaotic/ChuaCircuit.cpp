#include "tether/simulation/systems/chaotic/ChuaCircuit.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ChuaCircuit::ChuaCircuit() {
    initParam("alpha_ch", 9.0); initParam("beta_ch", 14.286);
    initParam("m0", -1.0/7.0); initParam("m1", 2.0/7.0); initParam("Bp", 1.0);
}

StateVector ChuaCircuit::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double alpha = params_.at("alpha_ch"), beta = params_.at("beta_ch");
    double m0 = params_.at("m0"), m1 = params_.at("m1"), Bp = params_.at("Bp");
    double ux = u.empty() ? 0.0 : u[0];
    double x = s[0], y = s[1], z = s[2];

    double gx = m1*x + 0.5*(m0-m1)*(std::abs(x+Bp) - std::abs(x-Bp));
    return {alpha*(y - x - gx) + ux, x - y + z, -beta*y};
}

StateVector ChuaCircuit::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[1], s[2]}; }
StateVector ChuaCircuit::defaultInitialState() const { return {0.1, 0.0, 0.0}; }

std::vector<ParamDescriptor> ChuaCircuit::parameterDescriptors() const {
    return {{"alpha_ch","","α",9.0,1.0,20.0,0.1},{"beta_ch","","β",14.286,1.0,30.0,0.1},
            {"m0","","m₀",-1.0/7.0,-2.0,0.0,0.01},{"m1","","m₁",2.0/7.0,0.0,2.0,0.01},
            {"Bp","","Breakpoint",1.0,0.1,5.0,0.1}};
}

std::vector<Preset> ChuaCircuit::presets() const {
    return {{"Classic","Standard Chua",{{"alpha_ch",9.0},{"beta_ch",14.286},{"m0",-1.0/7.0},{"m1",2.0/7.0},{"Bp",1.0}}}};
}
void ChuaCircuit::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ChuaCircuit::stateNames() const { return {"x","y","z"}; }
std::vector<std::string> ChuaCircuit::outputNames() const { return {"x","y","z"}; }
std::vector<std::string> ChuaCircuit::inputNames() const { return {"Perturbation"}; }

}  // namespace Simulation
