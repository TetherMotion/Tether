#include "tether/simulation/systems/mechanical/CoupledMassSpringDamper.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

CoupledMassSpringDamper::CoupledMassSpringDamper() {
    initParam("m1", 1.0); initParam("m2", 1.0);
    initParam("k1", 10.0); initParam("k2", 10.0); initParam("k12", 5.0);
    initParam("c1", 0.5); initParam("c2", 0.5); initParam("c12", 0.2);
}

StateVector CoupledMassSpringDamper::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m1 = params_.at("m1"), m2 = params_.at("m2");
    double k1 = params_.at("k1"), k2 = params_.at("k2"), k12 = params_.at("k12");
    double c1 = params_.at("c1"), c2 = params_.at("c2"), c12 = params_.at("c12");
    double F = u.empty() ? 0.0 : u[0];
    double x1 = s[0], dx1 = s[1], x2 = s[2], dx2 = s[3];
    double ddx1 = (F - k1*x1 - c1*dx1 - k12*(x1-x2) - c12*(dx1-dx2)) / m1;
    double ddx2 = (-k2*x2 - c2*dx2 + k12*(x1-x2) + c12*(dx1-dx2)) / m2;
    return {dx1, ddx1, dx2, ddx2};
}

StateVector CoupledMassSpringDamper::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[0], s[2]};
}

StateVector CoupledMassSpringDamper::defaultInitialState() const { return {0.0, 0.0, 0.0, 0.0}; }

std::vector<ParamDescriptor> CoupledMassSpringDamper::parameterDescriptors() const {
    return {{"m1", "kg", "Mass 1", 1.0, 0.01, 100.0, 0.1},
            {"m2", "kg", "Mass 2", 1.0, 0.01, 100.0, 0.1},
            {"k1", "N/m", "Spring 1", 10.0, 0.01, 10000.0, 1.0},
            {"k2", "N/m", "Spring 2", 10.0, 0.01, 10000.0, 1.0},
            {"k12", "N/m", "Coupling spring", 5.0, 0.0, 10000.0, 1.0},
            {"c1", "N·s/m", "Damper 1", 0.5, 0.0, 100.0, 0.1},
            {"c2", "N·s/m", "Damper 2", 0.5, 0.0, 100.0, 0.1},
            {"c12", "N·s/m", "Coupling damper", 0.2, 0.0, 100.0, 0.1}};
}

std::vector<Preset> CoupledMassSpringDamper::presets() const {
    return {{"Symmetric", "Identical masses and springs", {{"m1",1.0},{"m2",1.0},{"k1",10.0},{"k2",10.0},{"k12",5.0},{"c1",0.5},{"c2",0.5},{"c12",0.2}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Asymmetric", "Heavy second mass", {{"m1",1.0},{"m2",5.0},{"k1",10.0},{"k2",20.0},{"k12",5.0},{"c1",0.5},{"c2",1.0},{"c12",0.2}}}};
}

void CoupledMassSpringDamper::applyPreset(int index) {
    auto p = presets(); if (index >= 0 && index < (int)p.size()) setParameters(p[index].params);
}

std::vector<std::string> CoupledMassSpringDamper::stateNames() const { return {"x1", "dx1/dt", "x2", "dx2/dt"}; }
std::vector<std::string> CoupledMassSpringDamper::outputNames() const { return {"Position 1", "Position 2"}; }
std::vector<std::string> CoupledMassSpringDamper::inputNames() const { return {"Force"}; }
std::vector<std::string> CoupledMassSpringDamper::equationStrings() const {
    return {"m_1\\ddot{x}_1 + (c_1+c_{12})\\dot{x}_1 - c_{12}\\dot{x}_2 + (k_1+k_{12})x_1 - k_{12}x_2 = F",
            "m_2\\ddot{x}_2 + (c_2+c_{12})\\dot{x}_2 - c_{12}\\dot{x}_1 + (k_2+k_{12})x_2 - k_{12}x_1 = 0"};
}

}  // namespace Simulation
