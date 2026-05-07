#include "tether/simulation/systems/delay/NetworkedControlSystem.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

NetworkedControlSystem::NetworkedControlSystem() {
    initParam("Kp_ncs", 1.0);     // plant gain
    initParam("Tp_ncs", 1.0);     // plant time constant [s]
    initParam("Td_mean", 0.05);   // mean network delay [s]
    initParam("Td_var", 0.01);    // delay variance [s²]
    initParam("packet_loss", 0.0); // packet loss rate [0-1]
}

StateVector NetworkedControlSystem::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Kp = params_.at("Kp_ncs"), Tp = params_.at("Tp_ncs");
    double Td_mean = params_.at("Td_mean");
    double input = u.empty() ? 0.0 : u[0];

    // s[0] = plant output, s[1] = delayed input buffer (Padé)
    double y = s[0];
    double d = s[1];

    // Padé approximation with mean delay
    double delayed = 2.0 * input - d;
    double dd = (2.0 * input - 2.0 * d) / std::max(Td_mean, 1e-6);
    double dy = (Kp * delayed - y) / std::max(Tp, 1e-6);

    return {dy, dd};
}

StateVector NetworkedControlSystem::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector NetworkedControlSystem::defaultInitialState() const { return {0.0, 0.0}; }

std::vector<ParamDescriptor> NetworkedControlSystem::parameterDescriptors() const {
    return {{"Kp_ncs","","Plant gain",1.0,0.1,10.0,0.1},
            {"Tp_ncs","s","Plant time constant",1.0,0.01,100.0,0.01},
            {"Td_mean","s","Mean delay",0.05,0.001,1.0,0.001},
            {"Td_var","s²","Delay variance",0.01,0.0,0.1,0.001},
            {"packet_loss","","Packet loss rate",0.0,0.0,0.5,0.01}};
}

std::vector<Preset> NetworkedControlSystem::presets() const {
    return {{"LAN","Low delay",{{"Kp_ncs",1.0},{"Tp_ncs",1.0},{"Td_mean",0.01},{"Td_var",0.001},{"packet_loss",0.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"WiFi","Moderate delay",{{"Kp_ncs",1.0},{"Tp_ncs",1.0},{"Td_mean",0.05},{"Td_var",0.01},{"packet_loss",0.02}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Internet","High delay+loss",{{"Kp_ncs",1.0},{"Tp_ncs",1.0},{"Td_mean",0.2},{"Td_var",0.05},{"packet_loss",0.1}}}};
}
void NetworkedControlSystem::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> NetworkedControlSystem::stateNames() const { return {"y","d_buf"}; }
std::vector<std::string> NetworkedControlSystem::outputNames() const { return {"Plant output"}; }
std::vector<std::string> NetworkedControlSystem::inputNames() const { return {"Control input"}; }

}  // namespace Simulation
