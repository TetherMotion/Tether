#include "tether/simulation/systems/optical/AdaptiveOpticsTelescope.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

namespace {

double sinusoid(double amplitude, double frequency_hz, double t, double phase = 0.0) {
    return amplitude * std::sin(2.0 * M_PI * frequency_hz * t + phase);
}

} // namespace

AdaptiveOpticsTelescope::AdaptiveOpticsTelescope() {
    initParam("tau_atm", 0.0025);
    initParam("atm_coupling", 120.0);
    initParam("atm_gain", 0.45);
    initParam("turbulence_hz", 180.0);
    initParam("dm_wn_hz", 850.0);
    initParam("dm_zeta", 0.16);
    initParam("dm_cross", 0.18);
    initParam("influence_self", 0.92);
    initParam("influence_cross", 0.17);
    initParam("tau_wfs", 0.0008);
}

StateVector AdaptiveOpticsTelescope::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double tau_atm = params_.at("tau_atm");
    const double atm_coupling = params_.at("atm_coupling");
    const double atm_gain = params_.at("atm_gain");
    const double turbulence_hz = params_.at("turbulence_hz");
    const double dm_wn = 2.0 * M_PI * params_.at("dm_wn_hz");
    const double dm_zeta = params_.at("dm_zeta");
    const double dm_cross = params_.at("dm_cross");
    const double influence_self = params_.at("influence_self");
    const double influence_cross = params_.at("influence_cross");
    const double tau_wfs = params_.at("tau_wfs");

    const double u1 = u.empty() ? 0.0 : u[0];
    const double u2 = u.size() < 2 ? 0.0 : u[1];

    const double phi1 = s[0];
    const double phi2 = s[1];
    const double dm1 = s[2];
    const double dm1d = s[3];
    const double dm2 = s[4];
    const double dm2d = s[5];
    const double wfs1 = s[6];
    const double wfs2 = s[7];

    const double turb1 = sinusoid(atm_gain, turbulence_hz, t) + sinusoid(0.35 * atm_gain, 0.57 * turbulence_hz, t, 0.7);
    const double turb2 = sinusoid(atm_gain, 1.13 * turbulence_hz, t, M_PI / 3.0) + sinusoid(0.28 * atm_gain, 0.63 * turbulence_hz, t, 1.1);

    const double dphi1 = -phi1 / tau_atm + atm_coupling * phi2 + turb1;
    const double dphi2 = -phi2 / tau_atm - atm_coupling * phi1 + turb2;

    const double dm_cmd1 = (u1 - dm1) + dm_cross * (u2 - dm2);
    const double dm_cmd2 = (u2 - dm2) + dm_cross * (u1 - dm1);
    const double ddm1 = dm_wn * dm_wn * dm_cmd1 - 2.0 * dm_zeta * dm_wn * dm1d;
    const double ddm2 = dm_wn * dm_wn * dm_cmd2 - 2.0 * dm_zeta * dm_wn * dm2d;

    const double residual1 = phi1 - influence_self * dm1 - influence_cross * dm2;
    const double residual2 = phi2 - influence_cross * dm1 - influence_self * dm2;
    const double dwfs1 = (residual1 - wfs1) / tau_wfs;
    const double dwfs2 = (residual2 - wfs2) / tau_wfs;

    return {dphi1, dphi2, dm1d, ddm1, dm2d, ddm2, dwfs1, dwfs2};
}

StateVector AdaptiveOpticsTelescope::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[6], s[7]};
}

StateVector AdaptiveOpticsTelescope::defaultInitialState() const {
    return {0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 0.12, -0.08};
}

std::vector<ParamDescriptor> AdaptiveOpticsTelescope::parameterDescriptors() const {
    return {{"tau_atm","s","Atmospheric modal time constant",0.0025,0.0005,0.02,0.0001},
            {"atm_coupling","1/s","Atmospheric cross coupling",120.0,0.0,500.0,1.0},
            {"atm_gain","rad","Turbulence modal amplitude",0.45,0.01,2.0,0.01},
            {"turbulence_hz","Hz","Dominant turbulence frequency",180.0,10.0,1000.0,1.0},
            {"dm_wn_hz","Hz","Mirror modal natural frequency",850.0,50.0,5000.0,10.0},
            {"dm_zeta","","Mirror damping ratio",0.16,0.01,1.5,0.01},
            {"dm_cross","","Actuator cross coupling",0.18,0.0,0.8,0.01},
            {"influence_self","","Self influence coefficient",0.92,0.2,1.2,0.01},
            {"influence_cross","","Cross influence coefficient",0.17,0.0,0.8,0.01},
            {"tau_wfs","s","Wavefront sensor lag",0.0008,0.0001,0.01,0.0001}};
}

std::vector<Preset> AdaptiveOpticsTelescope::presets() const {
    return {{"8m telescope","Moderate turbulence with fast mirror",{{"tau_atm",0.0025},{"atm_coupling",120.0},{"atm_gain",0.45},{"turbulence_hz",180.0},{"dm_wn_hz",850.0},{"dm_zeta",0.16},{"dm_cross",0.18},{"influence_self",0.92},{"influence_cross",0.17},{"tau_wfs",0.0008}}},
            {"Poor seeing","Stronger turbulence and slower sensing",{{"tau_atm",0.004},{"atm_coupling",150.0},{"atm_gain",0.8},{"turbulence_hz",120.0},{"dm_wn_hz",650.0},{"dm_zeta",0.12},{"dm_cross",0.22},{"influence_self",0.88},{"influence_cross",0.2},{"tau_wfs",0.0015}}}};
}

void AdaptiveOpticsTelescope::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> AdaptiveOpticsTelescope::stateNames() const {
    return {"phi_1","phi_2","dm_1","dm_dot_1","dm_2","dm_dot_2","wfs_1","wfs_2"};
}

std::vector<std::string> AdaptiveOpticsTelescope::outputNames() const {
    return {"Residual mode 1","Residual mode 2"};
}

std::vector<std::string> AdaptiveOpticsTelescope::inputNames() const {
    return {"DM command 1","DM command 2"};
}

}  // namespace Simulation
