#include "tether/simulation/systems/delay/ConveyorBeltTracking.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ConveyorBeltTracking::ConveyorBeltTracking() {
    initParam("v_belt", 0.5);    // belt speed [m/s]
    initParam("L", 2.0);         // processing zone length [m]
    initParam("tau_act", 0.1);   // actuator time constant [s]
    initParam("K_proc", 1.0);    // process gain
}

StateVector ConveyorBeltTracking::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double v = params_.at("v_belt"), L = params_.at("L");
    double tau_act = params_.at("tau_act"), K_proc = params_.at("K_proc");
    double input = u.empty() ? 0.0 : u[0]; // desired process parameter

    double y = s[0];     // current output at end of belt
    double a = s[1];     // actuator state

    // Transport delay Td = L/v
    double Td = L / std::max(v, 0.01);

    // Actuator dynamics
    double da = (input - a) / std::max(tau_act, 1e-6);

    // Plant with transport delay (Padé): output tracks actuator with delay
    double dy = (K_proc * a - y) / std::max(Td, 1e-6);

    return {dy, da};
}

StateVector ConveyorBeltTracking::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector ConveyorBeltTracking::defaultInitialState() const { return {0.0, 0.0}; }

std::vector<ParamDescriptor> ConveyorBeltTracking::parameterDescriptors() const {
    return {{"v_belt","m/s","Belt speed",0.5,0.01,5.0,0.01},
            {"L","m","Zone length",2.0,0.1,20.0,0.1},
            {"tau_act","s","Actuator τ",0.1,0.01,5.0,0.01},
            {"K_proc","","Process gain",1.0,0.1,10.0,0.1}};
}

std::vector<Preset> ConveyorBeltTracking::presets() const {
    return {{"Standard","v=0.5 m/s, L=2m",{{"v_belt",0.5},{"L",2.0},{"tau_act",0.1},{"K_proc",1.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Fast belt","v=2 m/s",{{"v_belt",2.0},{"L",2.0},{"tau_act",0.1},{"K_proc",1.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Long zone","L=10m",{{"v_belt",0.5},{"L",10.0},{"tau_act",0.1},{"K_proc",1.0}}}};
}
void ConveyorBeltTracking::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ConveyorBeltTracking::stateNames() const { return {"y","actuator"}; }
std::vector<std::string> ConveyorBeltTracking::outputNames() const { return {"Belt output"}; }
std::vector<std::string> ConveyorBeltTracking::inputNames() const { return {"Process setpoint"}; }

}  // namespace Simulation
