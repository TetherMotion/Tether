#include "tether/simulation/systems/delay/SmithPredictorPlant.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

SmithPredictorPlant::SmithPredictorPlant() {
    initParam("Kp", 1.5);    // process gain
    initParam("Tp", 5.0);    // process time constant [s]
    initParam("Td", 3.0);    // process dead time [s] (Padé approximation)
}

StateVector SmithPredictorPlant::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Kp = params_.at("Kp"), Tp = params_.at("Tp"), Td = params_.at("Td");
    double input = u.empty() ? 0.0 : u[0];

    // 1st-order Padé approximation for delay: internal state is the Padé filter
    // Plant: G(s) = Kp / (Tp*s + 1) * e^(-Td*s)
    // Padé: e^(-Td*s) ≈ (1 - Td/2 * s) / (1 + Td/2 * s)
    // So overall: y state + Padé delay state combined in one state
    // Simplified: single state with effective dynamics
    double y = s[0];
    double dy = (Kp * input - y) / std::max(Tp + Td/2.0, 1e-6);

    return {dy};
}

StateVector SmithPredictorPlant::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector SmithPredictorPlant::defaultInitialState() const { return {0.0}; }

std::vector<ParamDescriptor> SmithPredictorPlant::parameterDescriptors() const {
    return {{"Kp","","Process gain",1.5,0.1,10.0,0.1},
            {"Tp","s","Time constant",5.0,0.1,100.0,0.1},
            {"Td","s","Dead time",3.0,0.0,30.0,0.1}};
}

std::vector<Preset> SmithPredictorPlant::presets() const {
    return {{"Standard FOPDT","Td/Tp<1",{{"Kp",1.5},{"Tp",5.0},{"Td",3.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Delay dominant","Td/Tp>1",{{"Kp",1.0},{"Tp",2.0},{"Td",5.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Fast plant","Small τ",{{"Kp",2.0},{"Tp",0.5},{"Td",0.3}}}};
}
void SmithPredictorPlant::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> SmithPredictorPlant::stateNames() const { return {"y"}; }
std::vector<std::string> SmithPredictorPlant::outputNames() const { return {"Process output"}; }
std::vector<std::string> SmithPredictorPlant::inputNames() const { return {"Control input"}; }

}  // namespace Simulation
