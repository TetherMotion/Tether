#include "tether/simulation/systems/rotational/DiskDriveHead.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

DiskDriveHead::DiskDriveHead() {
    initParam("Jv", 1e-6);    // VCM inertia [kg·m²]
    initParam("Kt_dd", 0.05); // torque constant
    initParam("bv", 1e-4);    // damping
    initParam("Kr", 100.0);   // resonance stiffness
    initParam("Jr", 1e-7);    // resonance mass
    initParam("br", 1e-5);    // resonance damping
}

StateVector DiskDriveHead::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double Jv = params_.at("Jv"), Kt = params_.at("Kt_dd"), bv = params_.at("bv");
    double Kr = params_.at("Kr"), Jr = params_.at("Jr"), br = params_.at("br");
    double V = u.empty() ? 0.0 : u[0];

    double x1 = s[0], v1 = s[1]; // main position, velocity
    double x2 = s[2], v2 = s[3]; // resonance mode

    double dx1 = v1;
    double dv1 = (Kt*V - bv*v1 - Kr*(x1-x2) - br*(v1-v2)) / Jv;
    double dx2 = v2;
    double dv2 = (Kr*(x1-x2) + br*(v1-v2)) / Jr;

    return {dx1, dv1, dx2, dv2};
}

StateVector DiskDriveHead::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0]}; }
StateVector DiskDriveHead::defaultInitialState() const { return {0.0, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> DiskDriveHead::parameterDescriptors() const {
    return {{"Jv","kg·m²","VCM inertia",1e-6,1e-9,1e-3,1e-8,true},
            {"Kt_dd","N/A","Torque const",0.05,0.001,1.0,0.001},
            {"bv","","VCM damping",1e-4,0.0,0.01,1e-5},
            {"Kr","N/m","Resonance stiffness",100.0,1.0,10000.0,1.0},
            {"Jr","kg·m²","Resonance inertia",1e-7,1e-10,1e-4,1e-9,true},
            {"br","","Resonance damping",1e-5,0.0,1e-3,1e-6}};
}

std::vector<Preset> DiskDriveHead::presets() const {
    return {{"Standard HDD","Typical 3.5in",{{"Jv",1e-6},{"Kt_dd",0.05},{"bv",1e-4},{"Kr",100.0},{"Jr",1e-7},{"br",1e-5}}}};
}
void DiskDriveHead::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> DiskDriveHead::stateNames() const { return {"x","v","x_r","v_r"}; }
std::vector<std::string> DiskDriveHead::outputNames() const { return {"Head position"}; }
std::vector<std::string> DiskDriveHead::inputNames() const { return {"VCM voltage"}; }

}  // namespace Simulation
