#include "tether/simulation/systems/optical/FastSteeringMirror.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

namespace {

double sinusoid(double amplitude, double frequency_hz, double t, double phase = 0.0) {
    return amplitude * std::sin(2.0 * M_PI * frequency_hz * t + phase);
}

} // namespace

FastSteeringMirror::FastSteeringMirror() {
    initParam("fsm_wn_hz", 280.0);
    initParam("fsm_zeta", 0.08);
    initParam("act_gain", 1.0);
    initParam("axis_coupling", 0.12);
    initParam("hyst_tau", 0.003);
    initParam("hyst_gain", 0.22);
    initParam("jitter_amp", 0.0025);
    initParam("jitter_hz", 85.0);
}

StateVector FastSteeringMirror::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double wn = 2.0 * M_PI * params_.at("fsm_wn_hz");
    const double zeta = params_.at("fsm_zeta");
    const double act_gain = params_.at("act_gain");
    const double axis_coupling = params_.at("axis_coupling");
    const double hyst_tau = params_.at("hyst_tau");
    const double hyst_gain = params_.at("hyst_gain");
    const double jitter_amp = params_.at("jitter_amp");
    const double jitter_hz = params_.at("jitter_hz");

    const double ux = u.empty() ? 0.0 : u[0];
    const double uy = u.size() < 2 ? 0.0 : u[1];

    const double theta_x = s[0];
    const double omega_x = s[1];
    const double hyst_x = s[2];
    const double theta_y = s[3];
    const double omega_y = s[4];
    const double hyst_y = s[5];

    const double jitter_x = sinusoid(jitter_amp, jitter_hz, t) + sinusoid(0.4 * jitter_amp, 2.3 * jitter_hz, t, 0.5);
    const double jitter_y = sinusoid(jitter_amp, 1.4 * jitter_hz, t, 1.0) + sinusoid(0.35 * jitter_amp, 2.0 * jitter_hz, t, 0.9);
    const double effort_x = act_gain * (ux - hyst_gain * hyst_x);
    const double effort_y = act_gain * (uy - hyst_gain * hyst_y);

    const double dtheta_x = omega_x;
    const double domega_x = wn * wn * (effort_x - theta_x) - 2.0 * zeta * wn * omega_x + axis_coupling * (theta_y - theta_x) + jitter_x;
    const double dhyst_x = (theta_x - hyst_x) / hyst_tau;
    const double dtheta_y = omega_y;
    const double domega_y = wn * wn * (effort_y - theta_y) - 2.0 * zeta * wn * omega_y + axis_coupling * (theta_x - theta_y) + jitter_y;
    const double dhyst_y = (theta_y - hyst_y) / hyst_tau;

    return {dtheta_x, domega_x, dhyst_x, dtheta_y, domega_y, dhyst_y};
}

StateVector FastSteeringMirror::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[3]};
}

StateVector FastSteeringMirror::defaultInitialState() const {
    return {0.002, 0.0, 0.0, -0.0015, 0.0, 0.0};
}

std::vector<ParamDescriptor> FastSteeringMirror::parameterDescriptors() const {
    return {{"fsm_wn_hz","Hz","Mirror resonance",280.0,10.0,3000.0,1.0},
            {"fsm_zeta","","Mirror damping ratio",0.08,0.005,1.0,0.005},
            {"act_gain","rad/V","Actuator gain",1.0,0.1,10.0,0.05},
            {"axis_coupling","","Cross-axis coupling",0.12,0.0,1.0,0.01},
            {"hyst_tau","s","Hysteresis internal lag",0.003,0.0001,0.05,0.0001},
            {"hyst_gain","","Hysteresis gain",0.22,0.0,1.0,0.01},
            {"jitter_amp","rad/s²","Jitter forcing amplitude",0.0025,0.0,0.1,0.0005},
            {"jitter_hz","Hz","Dominant platform jitter",85.0,1.0,2000.0,1.0}};
}

std::vector<Preset> FastSteeringMirror::presets() const {
    return {{"Voice coil","Moderately damped mirror",{{"fsm_wn_hz",280.0},{"fsm_zeta",0.08},{"act_gain",1.0},{"axis_coupling",0.12},{"hyst_tau",0.003},{"hyst_gain",0.22},{"jitter_amp",0.0025},{"jitter_hz",85.0}}},
            {"Piezo mirror","Higher resonance with stronger hysteresis",{{"fsm_wn_hz",550.0},{"fsm_zeta",0.05},{"act_gain",0.8},{"axis_coupling",0.08},{"hyst_tau",0.006},{"hyst_gain",0.35},{"jitter_amp",0.0018},{"jitter_hz",120.0}}}};
}

void FastSteeringMirror::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> FastSteeringMirror::stateNames() const {
    return {"theta_x","omega_x","h_x","theta_y","omega_y","h_y"};
}

std::vector<std::string> FastSteeringMirror::outputNames() const {
    return {"Tip angle","Tilt angle"};
}

std::vector<std::string> FastSteeringMirror::inputNames() const {
    return {"Tip command","Tilt command"};
}

}  // namespace Simulation
