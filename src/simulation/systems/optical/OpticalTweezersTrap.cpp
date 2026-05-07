#include "tether/simulation/systems/optical/OpticalTweezersTrap.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

namespace {

double sinusoid(double amplitude, double frequency_hz, double t, double phase = 0.0) {
    return amplitude * std::sin(2.0 * M_PI * frequency_hz * t + phase);
}

} // namespace

OpticalTweezersTrap::OpticalTweezersTrap() {
    initParam("gamma", 7.5);
    initParam("kxy", 18.0);
    initParam("kz", 10.0);
    initParam("cross_coupling", 0.08);
    initParam("cubic_softening", 0.15);
    initParam("scattering_bias", 0.6);
    initParam("brownian_amp", 1.4);
    initParam("brownian_hz", 65.0);
}

StateVector OpticalTweezersTrap::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double gamma = params_.at("gamma");
    const double kxy = params_.at("kxy");
    const double kz = params_.at("kz");
    const double cross_coupling = params_.at("cross_coupling");
    const double cubic_softening = params_.at("cubic_softening");
    const double scattering_bias = params_.at("scattering_bias");
    const double brownian_amp = params_.at("brownian_amp");
    const double brownian_hz = params_.at("brownian_hz");

    const double ux = u.empty() ? 0.0 : u[0];
    const double uy = u.size() < 2 ? 0.0 : u[1];
    const double uz = u.size() < 3 ? 0.0 : u[2];

    const double x = s[0];
    const double y = s[1];
    const double z = s[2];

    const double ex = x - ux;
    const double ey = y - uy;
    const double ez = z - uz;
    const double noise_x = sinusoid(brownian_amp, brownian_hz, t) + sinusoid(0.45 * brownian_amp, 1.7 * brownian_hz, t, 0.3);
    const double noise_y = sinusoid(brownian_amp, 1.13 * brownian_hz, t, 0.7) + sinusoid(0.35 * brownian_amp, 0.61 * brownian_hz, t, 0.9);
    const double noise_z = sinusoid(0.8 * brownian_amp, 0.82 * brownian_hz, t, 1.1);

    const double dx = (-kxy * ex * (1.0 - cubic_softening * ex * ex) - cross_coupling * ey + noise_x) / gamma;
    const double dy = (-kxy * ey * (1.0 - cubic_softening * ey * ey) - cross_coupling * ex + noise_y) / gamma;
    const double dz = (-kz * ez * (1.0 - 0.5 * cubic_softening * ez * ez) + scattering_bias + noise_z) / gamma;

    return {dx, dy, dz};
}

StateVector OpticalTweezersTrap::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[1], s[2]};
}

StateVector OpticalTweezersTrap::defaultInitialState() const {
    return {0.35, -0.2, 0.15};
}

std::vector<ParamDescriptor> OpticalTweezersTrap::parameterDescriptors() const {
    return {{"gamma","pN·ms/um","Effective viscous drag",7.5,0.1,200.0,0.1},
            {"kxy","pN/um","Lateral trap stiffness",18.0,0.1,500.0,0.1},
            {"kz","pN/um","Axial trap stiffness",10.0,0.1,500.0,0.1},
            {"cross_coupling","","Cross-axis optical coupling",0.08,0.0,1.0,0.01},
            {"cubic_softening","1/um²","Nonlinear softening",0.15,0.0,2.0,0.01},
            {"scattering_bias","pN","Axial scattering bias",0.6,-10.0,10.0,0.05},
            {"brownian_amp","arb","Brownian-like disturbance amplitude",1.4,0.0,50.0,0.05},
            {"brownian_hz","Hz","Dominant disturbance bandwidth",65.0,0.1,5000.0,0.5}};
}

std::vector<Preset> OpticalTweezersTrap::presets() const {
    return {{"Water bead","Moderate viscous drag",{{"gamma",7.5},{"kxy",18.0},{"kz",10.0},{"cross_coupling",0.08},{"cubic_softening",0.15},{"scattering_bias",0.6},{"brownian_amp",1.4},{"brownian_hz",65.0}}},
            {"Strong trap","Higher stiffness and lower variance",{{"gamma",9.0},{"kxy",40.0},{"kz",24.0},{"cross_coupling",0.05},{"cubic_softening",0.08},{"scattering_bias",0.3},{"brownian_amp",0.8},{"brownian_hz",90.0}}}};
}

void OpticalTweezersTrap::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> OpticalTweezersTrap::stateNames() const {
    return {"x","y","z"};
}

std::vector<std::string> OpticalTweezersTrap::outputNames() const {
    return {"x","y","z"};
}

std::vector<std::string> OpticalTweezersTrap::inputNames() const {
    return {"Trap center x","Trap center y","Trap center z"};
}

}  // namespace Simulation
