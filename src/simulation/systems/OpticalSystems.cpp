#include "tether/simulation/systems/OpticalSystems.hpp"

#include <algorithm>
#include <cmath>

namespace Simulation {

namespace {

double sinusoid(double amplitude, double frequency_hz, double t, double phase = 0.0) {
    return amplitude * std::sin(2.0 * M_PI * frequency_hz * t + phase);
}

} // namespace

// ============================================================================
// (66) Adaptive Optics Telescope
// ============================================================================
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

// ============================================================================
// (67) Fast Steering Mirror
// ============================================================================
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

// ============================================================================
// (68) Pound-Drever-Hall Lock
// ============================================================================
PoundDreverHallLock::PoundDreverHallLock() {
    initParam("linewidth", 1.0);
    initParam("detune_tau", 0.0015);
    initParam("piezo_wn_hz", 3200.0);
    initParam("piezo_zeta", 0.2);
    initParam("piezo_gain", 12.0);
    initParam("thermal_tau", 0.2);
    initParam("thermal_gain", 0.8);
    initParam("acoustic_amp", 1.4);
    initParam("acoustic_hz", 180.0);
    initParam("laser_noise_amp", 0.45);
    initParam("laser_noise_hz", 900.0);
    initParam("sense_tau", 0.0002);
}

StateVector PoundDreverHallLock::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double linewidth = params_.at("linewidth");
    const double detune_tau = params_.at("detune_tau");
    const double piezo_wn = 2.0 * M_PI * params_.at("piezo_wn_hz");
    const double piezo_zeta = params_.at("piezo_zeta");
    const double piezo_gain = params_.at("piezo_gain");
    const double thermal_tau = params_.at("thermal_tau");
    const double thermal_gain = params_.at("thermal_gain");
    const double acoustic_amp = params_.at("acoustic_amp");
    const double acoustic_hz = params_.at("acoustic_hz");
    const double laser_noise_amp = params_.at("laser_noise_amp");
    const double laser_noise_hz = params_.at("laser_noise_hz");
    const double sense_tau = params_.at("sense_tau");

    const double voltage = u.empty() ? 0.0 : u[0];

    const double detuning = s[0];
    const double piezo = s[1];
    const double piezo_rate = s[2];
    const double thermal = s[3];
    const double sensed = s[4];

    const double acoustic = sinusoid(acoustic_amp, acoustic_hz, t) + sinusoid(0.35 * acoustic_amp, 0.37 * acoustic_hz, t, 0.4);
    const double laser = sinusoid(laser_noise_amp, laser_noise_hz, t, 0.9);

    const double ddetuning = (-detuning + thermal + acoustic + laser - piezo_gain * piezo) / detune_tau;
    const double dpiezo = piezo_rate;
    const double dpiezo_rate = piezo_wn * piezo_wn * (voltage - piezo) - 2.0 * piezo_zeta * piezo_wn * piezo_rate;
    const double dthermal = (-thermal + thermal_gain * sinusoid(1.0, 0.8, t)) / thermal_tau;

    const double normalized = detuning / std::max(linewidth, 1e-9);
    const double error_signal = normalized * std::exp(-normalized * normalized);
    const double dsensed = (error_signal - sensed) / sense_tau;

    return {ddetuning, dpiezo, dpiezo_rate, dthermal, dsensed};
}

StateVector PoundDreverHallLock::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[4], s[0]};
}

StateVector PoundDreverHallLock::defaultInitialState() const {
    return {0.5, 0.0, 0.0, 0.15, 0.0};
}

std::vector<ParamDescriptor> PoundDreverHallLock::parameterDescriptors() const {
    return {{"linewidth","arb","Normalized cavity linewidth",1.0,0.05,20.0,0.05},
            {"detune_tau","s","Detuning time constant",0.0015,0.0001,0.05,0.0001},
            {"piezo_wn_hz","Hz","Piezo resonance",3200.0,50.0,30000.0,10.0},
            {"piezo_zeta","","Piezo damping ratio",0.2,0.01,1.5,0.01},
            {"piezo_gain","arb/V","Piezo detuning gain",12.0,0.1,100.0,0.1},
            {"thermal_tau","s","Thermal drift time constant",0.2,0.01,20.0,0.01},
            {"thermal_gain","arb","Thermal drift gain",0.8,0.0,10.0,0.05},
            {"acoustic_amp","arb","Acoustic disturbance amplitude",1.4,0.0,20.0,0.05},
            {"acoustic_hz","Hz","Acoustic disturbance frequency",180.0,1.0,5000.0,1.0},
            {"laser_noise_amp","arb","Laser frequency noise amplitude",0.45,0.0,10.0,0.01},
            {"laser_noise_hz","Hz","Laser noise frequency",900.0,1.0,50000.0,1.0},
            {"sense_tau","s","Photodetector-demod lag",0.0002,0.00001,0.01,0.00001}};
}

std::vector<Preset> PoundDreverHallLock::presets() const {
    return {{"Rigid cavity","Fast piezo and moderate acoustic noise",{{"linewidth",1.0},{"detune_tau",0.0015},{"piezo_wn_hz",3200.0},{"piezo_zeta",0.2},{"piezo_gain",12.0},{"thermal_tau",0.2},{"thermal_gain",0.8},{"acoustic_amp",1.4},{"acoustic_hz",180.0},{"laser_noise_amp",0.45},{"laser_noise_hz",900.0},{"sense_tau",0.0002}}},
            {"Soft mount","Stronger acoustic loading and slower sensing",{{"linewidth",0.7},{"detune_tau",0.0025},{"piezo_wn_hz",1800.0},{"piezo_zeta",0.12},{"piezo_gain",10.0},{"thermal_tau",0.35},{"thermal_gain",1.2},{"acoustic_amp",2.1},{"acoustic_hz",95.0},{"laser_noise_amp",0.65},{"laser_noise_hz",600.0},{"sense_tau",0.0004}}}};
}

void PoundDreverHallLock::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> PoundDreverHallLock::stateNames() const {
    return {"detuning","piezo","piezo_rate","thermal","error_filtered"};
}

std::vector<std::string> PoundDreverHallLock::outputNames() const {
    return {"PDH error","Detuning"};
}

std::vector<std::string> PoundDreverHallLock::inputNames() const {
    return {"Piezo voltage"};
}

// ============================================================================
// (69) Active Seismic Isolation Bench
// ============================================================================
ActiveSeismicIsolationBench::ActiveSeismicIsolationBench() {
    initParam("m1", 120.0);
    initParam("m2", 80.0);
    initParam("m3", 40.0);
    initParam("k1", 4.5e4);
    initParam("k12", 2.8e4);
    initParam("k23", 1.8e4);
    initParam("c1", 220.0);
    initParam("c12", 150.0);
    initParam("c23", 90.0);
    initParam("ground_low_amp", 0.0005);
    initParam("ground_low_hz", 0.8);
    initParam("ground_high_amp", 2.0e-5);
    initParam("ground_high_hz", 18.0);
}

StateVector ActiveSeismicIsolationBench::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double m1 = params_.at("m1");
    const double m2 = params_.at("m2");
    const double m3 = params_.at("m3");
    const double k1 = params_.at("k1");
    const double k12 = params_.at("k12");
    const double k23 = params_.at("k23");
    const double c1 = params_.at("c1");
    const double c12 = params_.at("c12");
    const double c23 = params_.at("c23");
    const double ground_low_amp = params_.at("ground_low_amp");
    const double ground_low_hz = params_.at("ground_low_hz");
    const double ground_high_amp = params_.at("ground_high_amp");
    const double ground_high_hz = params_.at("ground_high_hz");

    const double f1 = u.empty() ? 0.0 : u[0];
    const double f2 = u.size() < 2 ? 0.0 : u[1];
    const double f3 = u.size() < 3 ? 0.0 : u[2];

    const double x1 = s[0];
    const double v1 = s[1];
    const double x2 = s[2];
    const double v2 = s[3];
    const double x3 = s[4];
    const double v3 = s[5];

    const double wg1 = 2.0 * M_PI * ground_low_hz;
    const double wg2 = 2.0 * M_PI * ground_high_hz;
    const double ground = ground_low_amp * std::sin(wg1 * t) + ground_high_amp * std::sin(wg2 * t + 0.6);
    const double ground_v = ground_low_amp * wg1 * std::cos(wg1 * t) + ground_high_amp * wg2 * std::cos(wg2 * t + 0.6);

    const double dx1 = v1;
    const double dv1 = (-k1 * (x1 - ground) - c1 * (v1 - ground_v) - k12 * (x1 - x2) - c12 * (v1 - v2) + f1) / m1;
    const double dx2 = v2;
    const double dv2 = (k12 * (x1 - x2) + c12 * (v1 - v2) - k23 * (x2 - x3) - c23 * (v2 - v3) + f2) / m2;
    const double dx3 = v3;
    const double dv3 = (k23 * (x2 - x3) + c23 * (v2 - v3) + f3) / m3;

    return {dx1, dv1, dx2, dv2, dx3, dv3};
}

StateVector ActiveSeismicIsolationBench::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[4]};
}

StateVector ActiveSeismicIsolationBench::defaultInitialState() const {
    return {0.0003, 0.0, 0.0002, 0.0, 0.0001, 0.0};
}

std::vector<ParamDescriptor> ActiveSeismicIsolationBench::parameterDescriptors() const {
    return {{"m1","kg","Stage 1 mass",120.0,1.0,1000.0,1.0},
            {"m2","kg","Stage 2 mass",80.0,1.0,1000.0,1.0},
            {"m3","kg","Bench mass",40.0,1.0,1000.0,1.0},
            {"k1","N/m","Ground-stage stiffness",4.5e4,1e3,1e6,100.0},
            {"k12","N/m","Stage 1-2 stiffness",2.8e4,1e3,1e6,100.0},
            {"k23","N/m","Stage 2-3 stiffness",1.8e4,1e3,1e6,100.0},
            {"c1","N·s/m","Ground-stage damping",220.0,0.0,1e4,1.0},
            {"c12","N·s/m","Stage 1-2 damping",150.0,0.0,1e4,1.0},
            {"c23","N·s/m","Stage 2-3 damping",90.0,0.0,1e4,1.0},
            {"ground_low_amp","m","Low-frequency ground amplitude",0.0005,0.0,0.01,1e-5},
            {"ground_low_hz","Hz","Low-frequency ground content",0.8,0.01,20.0,0.01},
            {"ground_high_amp","m","High-frequency ground amplitude",2.0e-5,0.0,0.01,1e-6},
            {"ground_high_hz","Hz","High-frequency ground content",18.0,0.1,200.0,0.1}};
}

std::vector<Preset> ActiveSeismicIsolationBench::presets() const {
    return {{"Optical table","Moderate stage coupling",{{"m1",120.0},{"m2",80.0},{"m3",40.0},{"k1",4.5e4},{"k12",2.8e4},{"k23",1.8e4},{"c1",220.0},{"c12",150.0},{"c23",90.0},{"ground_low_amp",0.0005},{"ground_low_hz",0.8},{"ground_high_amp",2.0e-5},{"ground_high_hz",18.0}}},
            {"Ultra quiet","Softer support with lower seismic floor",{{"m1",150.0},{"m2",100.0},{"m3",50.0},{"k1",2.5e4},{"k12",1.8e4},{"k23",1.2e4},{"c1",180.0},{"c12",110.0},{"c23",70.0},{"ground_low_amp",0.0002},{"ground_low_hz",0.5},{"ground_high_amp",8.0e-6},{"ground_high_hz",12.0}}}};
}

void ActiveSeismicIsolationBench::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> ActiveSeismicIsolationBench::stateNames() const {
    return {"x1","v1","x2","v2","x3","v3"};
}

std::vector<std::string> ActiveSeismicIsolationBench::outputNames() const {
    return {"Bench displacement"};
}

std::vector<std::string> ActiveSeismicIsolationBench::inputNames() const {
    return {"Actuator 1","Actuator 2","Actuator 3"};
}

// ============================================================================
// (70) Optical Tweezers Trap
// ============================================================================
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

} // namespace Simulation