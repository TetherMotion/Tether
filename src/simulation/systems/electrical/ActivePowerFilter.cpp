#include "tether/simulation/systems/electrical/ActivePowerFilter.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ActivePowerFilter::ActivePowerFilter() {
    initParam("L_apf", 2e-3);   // filter inductance [H]
    initParam("C_dc", 2200e-6); // DC-link capacitance [F]
    initParam("R_apf", 0.1);    // filter resistance [Ω]
    initParam("Vdc_ref", 400.0);// DC link reference [V]
    initParam("Vgrid", 230.0);  // grid voltage [V RMS]
}

StateVector ActivePowerFilter::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double L = params_.at("L_apf"), C = params_.at("C_dc"), R = params_.at("R_apf");
    double Vg = params_.at("Vgrid");
    double d = u.empty() ? 0.0 : u[0]; // modulation index [-1,1]

    double i_f = s[0];   // filter current
    double v_dc = s[1];  // DC-link voltage

    double v_inv = d * v_dc; // inverter output
    double di_f = (Vg*std::sqrt(2.0) - v_inv - R*i_f) / L;
    double dv_dc = (d*i_f) / C; // power balance

    return {di_f, dv_dc};
}

StateVector ActivePowerFilter::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector ActivePowerFilter::defaultInitialState() const { return {0.0, 400.0}; }

std::vector<ParamDescriptor> ActivePowerFilter::parameterDescriptors() const {
    return {{"L_apf","H","Filter L",2e-3,0.1e-3,50e-3,0.1e-3},
            {"C_dc","F","DC-link C",2200e-6,100e-6,10e-3,100e-6},
            {"R_apf","Ω","Filter R",0.1,0.01,5.0,0.01},
            {"Vdc_ref","V","DC-link ref",400.0,100.0,800.0,10.0},
            {"Vgrid","V","Grid V_rms",230.0,100.0,480.0,1.0}};
}

std::vector<Preset> ActivePowerFilter::presets() const {
    return {{"Standard","230V grid",{{"L_apf",2e-3},{"C_dc",2200e-6},{"R_apf",0.1},{"Vdc_ref",400.0},{"Vgrid",230.0}}}};
}
void ActivePowerFilter::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> ActivePowerFilter::stateNames() const { return {"i_f","v_dc"}; }
std::vector<std::string> ActivePowerFilter::outputNames() const { return {"Filter current"}; }
std::vector<std::string> ActivePowerFilter::inputNames() const { return {"Modulation index"}; }

}  // namespace Simulation
