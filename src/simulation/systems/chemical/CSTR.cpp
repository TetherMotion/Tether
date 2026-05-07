#include "tether/simulation/systems/chemical/CSTR.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

CSTR::CSTR() {
    initParam("V", 0.1);         // volume [m³]
    initParam("rho", 1000.0);    // density [kg/m³]
    initParam("Cp", 4180.0);     // heat capacity [J/(kg·K)]
    initParam("dH", -5e4);       // heat of reaction [J/mol]
    initParam("E_over_R", 8750.0); // activation energy / R [K]
    initParam("k0", 7.2e10);     // pre-exponential factor [1/s]
    initParam("UA", 5e4);        // heat transfer coeff×area [W/K]
    initParam("CA_in", 1.0);     // feed concentration [mol/m³]
    initParam("T_in", 350.0);    // feed temperature [K]
    initParam("Tc_in", 300.0);   // coolant inlet temperature [K]
}

StateVector CSTR::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double V = params_.at("V"), rho = params_.at("rho"), Cp = params_.at("Cp");
    double dH = params_.at("dH"), E_R = params_.at("E_over_R"), k0 = params_.at("k0");
    double UA = params_.at("UA"), CA_in = params_.at("CA_in");
    double T_in = params_.at("T_in"), Tc_in = params_.at("Tc_in");

    double F = u.size() > 0 ? u[0] : 0.001;      // flow rate [m³/s]
    double Q_cool = u.size() > 1 ? u[1] : 0.0;    // additional cooling power [W]

    double CA = std::max(s[0], 0.0);
    double T = s[1];

    // Arrhenius kinetics amplify temperature changes into reaction-rate changes,
    // which is what gives the benchmark its operating-point sensitivity.
    double k = k0 * std::exp(-E_R / std::max(T, 1.0));
    double rA = k * CA;

    // Material and energy balances are separated explicitly here so tests can
    // reason about concentration disturbances and thermal disturbances independently.
    double dCA = F/V * (CA_in - CA) - rA;
    double dT = F/V * (T_in - T)
              + (-dH) * rA / (rho * Cp)
              - UA * (T - Tc_in) / (rho * Cp * V)
              - Q_cool / (rho * Cp * V);

    return {dCA, dT};
}

StateVector CSTR::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {std::max(s[0], 0.0), s[1]};
}
StateVector CSTR::defaultInitialState() const { return {0.5, 350.0}; }

std::vector<ParamDescriptor> CSTR::parameterDescriptors() const {
    return {{"V","m³","Reactor volume",0.1,0.001,10.0,0.01},
            {"rho","kg/m³","Density",1000.0,500.0,2000.0,10.0},
            {"Cp","J/(kg·K)","Heat capacity",4180.0,1000.0,10000.0,10.0},
            {"dH","J/mol","Heat of reaction",-5e4,-1e6,0.0,100.0},
            {"E_over_R","K","E/R activation",8750.0,1000.0,20000.0,100.0},
            {"k0","1/s","Pre-exponential",7.2e10,1e5,1e15,1e5,true},
            {"UA","W/K","UA heat transfer",5e4,100.0,1e6,100.0},
            {"CA_in","mol/m³","Feed conc",1.0,0.01,10.0,0.01},
            {"T_in","K","Feed temp",350.0,200.0,500.0,1.0},
            {"Tc_in","K","Coolant temp",300.0,200.0,400.0,1.0}};
}

std::vector<Preset> CSTR::presets() const {
    return {{"Exothermic","Standard exothermic CSTR",
             {{"V",0.1},{"rho",1000.0},{"Cp",4180.0},{"dH",-5e4},{"E_over_R",8750.0}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"k0",7.2e10},{"UA",5e4},{"CA_in",1.0},{"T_in",350.0},{"Tc_in",300.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"High gain","Sensitive operating point",
             {{"V",0.1},{"rho",1000.0},{"Cp",4180.0},{"dH",-1e5},{"E_over_R",8750.0}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"k0",7.2e10},{"UA",3e4},{"CA_in",2.0},{"T_in",350.0},{"Tc_in",300.0}}}};
}
void CSTR::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> CSTR::stateNames() const { return {"CA","T"}; }
std::vector<std::string> CSTR::outputNames() const { return {"Concentration A","Temperature"}; }
std::vector<std::string> CSTR::inputNames() const { return {"Flow rate","Cooling power"}; }

}  // namespace Simulation
