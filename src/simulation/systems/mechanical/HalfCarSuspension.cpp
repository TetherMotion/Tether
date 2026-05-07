#include "tether/simulation/systems/mechanical/HalfCarSuspension.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

HalfCarSuspension::HalfCarSuspension() {
    initParam("ms", 600.0); initParam("Iy", 400.0);
    initParam("muf", 50.0); initParam("mur", 50.0);
    initParam("ksf", 15000.0); initParam("ksr", 15000.0);
    initParam("kuf", 150000.0); initParam("kur", 150000.0);
    initParam("csf", 1000.0); initParam("csr", 1000.0);
    initParam("lf", 1.2); initParam("lr", 1.4);
}

StateVector HalfCarSuspension::dynamics(double t, const StateVector& s, const StateVector& u) const {
    double ms = params_.at("ms"), Iy = params_.at("Iy");
    double muf = params_.at("muf"), mur = params_.at("mur");
    double ksf = params_.at("ksf"), ksr = params_.at("ksr");
    double kuf = params_.at("kuf"), kur = params_.at("kur");
    double csf = params_.at("csf"), csr = params_.at("csr");
    double lf = params_.at("lf"), lr = params_.at("lr");
    double Ff = u.size() > 0 ? u[0] : 0.0, Fr = u.size() > 1 ? u[1] : 0.0;

    double zs = s[0], dzs = s[1], theta = s[2], dtheta = s[3];
    double zuf = s[4], dzuf = s[5], zur = s[6], dzur = s[7];

    double zsf = zs - lf*theta, zsr = zs + lr*theta;
    double dzsf = dzs - lf*dtheta, dzsr = dzs + lr*dtheta;

    double Fsf = -ksf*(zsf-zuf) - csf*(dzsf-dzuf) + Ff;
    double Fsr = -ksr*(zsr-zur) - csr*(dzsr-dzur) + Fr;

    double ddzs = (Fsf + Fsr) / ms;
    double ddtheta = (-lf*Fsf + lr*Fsr) / Iy;

    double zr = 0.02 * std::sin(2.0*M_PI*t);
    double ddzuf = (-Fsf - kuf*(zuf-zr)) / muf;
    double ddzur = (-Fsr - kur*(zur-zr)) / mur;

    return {dzs, ddzs, dtheta, ddtheta, dzuf, ddzuf, dzur, ddzur};
}

StateVector HalfCarSuspension::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[0], s[2]}; }
StateVector HalfCarSuspension::defaultInitialState() const { return {0,0,0,0,0,0,0,0}; }

std::vector<ParamDescriptor> HalfCarSuspension::parameterDescriptors() const {
    return {{"ms","kg","Sprung mass",600.0,100.0,2000.0,10.0},{"Iy","kg·m²","Pitch inertia",400.0,50.0,2000.0,10.0},
            {"ksf","N/m","Front spring",15000.0,1000.0,100000.0,1000.0},{"ksr","N/m","Rear spring",15000.0,1000.0,100000.0,1000.0},
            {"lf","m","Front CG dist",1.2,0.5,3.0,0.1},{"lr","m","Rear CG dist",1.4,0.5,3.0,0.1}};
}

std::vector<Preset> HalfCarSuspension::presets() const {
    return {{"Sedan","Default",{{"ms",600.0},{"Iy",400.0},{"muf",50.0},{"mur",50.0},{"ksf",15000.0},{"ksr",15000.0},{"kuf",150000.0},{"kur",150000.0},{"csf",1000.0},{"csr",1000.0},{"lf",1.2},{"lr",1.4}}}};
}
void HalfCarSuspension::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> HalfCarSuspension::stateNames() const { return {"z_s","dz_s/dt","θ","dθ/dt","z_uf","dz_uf/dt","z_ur","dz_ur/dt"}; }
std::vector<std::string> HalfCarSuspension::outputNames() const { return {"Heave","Pitch"}; }
std::vector<std::string> HalfCarSuspension::inputNames() const { return {"Front force","Rear force"}; }

}  // namespace Simulation
