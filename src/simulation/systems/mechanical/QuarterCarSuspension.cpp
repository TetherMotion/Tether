#include "tether/simulation/systems/mechanical/QuarterCarSuspension.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

QuarterCarSuspension::QuarterCarSuspension() {
    initParam("ms", 300.0); initParam("mu", 50.0);
    initParam("ks", 15000.0); initParam("ku", 150000.0);
    initParam("cs", 1000.0); initParam("cu", 0.0);
    initParam("road_amp", 0.05); initParam("road_freq", 1.0);
}

StateVector QuarterCarSuspension::dynamics(double t, const StateVector& s, const StateVector& u) const {
    double ms = params_.at("ms"), mu = params_.at("mu");
    double ks = params_.at("ks"), ku = params_.at("ku");
    double cs = params_.at("cs"), cu = params_.at("cu");
    double Fa = u.empty() ? 0.0 : u[0];

    double road_amp = params_.at("road_amp"), road_freq = params_.at("road_freq");
    double zr = road_amp * std::sin(2.0 * M_PI * road_freq * t);

    double xs = s[0], dxs = s[1], xu = s[2], dxu = s[3];

    double ddxs = (-ks*(xs-xu) - cs*(dxs-dxu) + Fa) / ms;
    double ddxu = (ks*(xs-xu) + cs*(dxs-dxu) - ku*(xu-zr) - cu*dxu - Fa) / mu;

    return {dxs, ddxs, dxu, ddxu};
}

StateVector QuarterCarSuspension::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[0]-s[2]}; }
StateVector QuarterCarSuspension::defaultInitialState() const { return {0, 0, 0, 0}; }

std::vector<ParamDescriptor> QuarterCarSuspension::parameterDescriptors() const {
    return {{"ms","kg","Sprung mass",300.0,50.0,1000.0,10.0},{"mu","kg","Unsprung mass",50.0,10.0,200.0,5.0},
            {"ks","N/m","Suspension spring",15000.0,1000.0,100000.0,1000.0},
            {"ku","N/m","Tire spring",150000.0,10000.0,500000.0,10000.0},
            {"cs","N·s/m","Susp damping",1000.0,100.0,10000.0,100.0},
            {"road_amp","m","Road amplitude",0.05,0.0,0.5,0.01},
            {"road_freq","Hz","Road frequency",1.0,0.1,20.0,0.1}};
}

std::vector<Preset> QuarterCarSuspension::presets() const {
    return {{"Sedan","Typical sedan",{{"ms",300.0},{"mu",50.0},{"ks",15000.0},{"ku",150000.0},{"cs",1000.0},{"road_amp",0.05},{"road_freq",1.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Sports car","Stiff suspension",{{"ms",250.0},{"mu",45.0},{"ks",30000.0},{"ku",200000.0},{"cs",2000.0},{"road_amp",0.03},{"road_freq",2.0}}}};
}
void QuarterCarSuspension::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> QuarterCarSuspension::stateNames() const { return {"x_sprung","dx_s/dt","x_unsprung","dx_u/dt"}; }
std::vector<std::string> QuarterCarSuspension::outputNames() const { return {"Body pos","Suspension travel"}; }
std::vector<std::string> QuarterCarSuspension::inputNames() const { return {"Actuator force"}; }

}  // namespace Simulation
