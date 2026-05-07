#include "tether/simulation/systems/mechanical/MassSpringDamper.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

MassSpringDamper::MassSpringDamper() {
    initParam("m", 1.0);   // mass [kg]
    initParam("k", 10.0);  // spring constant [N/m]
    initParam("c", 0.5);   // damping coefficient [N·s/m]
}

StateVector MassSpringDamper::dynamics(double /*t*/, const StateVector& state, const StateVector& input) const {
    double m = params_.at("m"), k = params_.at("k"), c = params_.at("c");
    double x = state[0], dx = state[1];
    double F = input.empty() ? 0.0 : input[0];
    // Canonical force balance: applied force is opposed by viscous damping and
    // spring restoring force, which makes the resonance analytically predictable.
    return {dx, (F - c * dx - k * x) / m};
}

StateVector MassSpringDamper::output(double /*t*/, const StateVector& state, const StateVector& /*input*/) const {
    return {state[0]};
}

StateVector MassSpringDamper::defaultInitialState() const { return {0.0, 0.0}; }

std::vector<ParamDescriptor> MassSpringDamper::parameterDescriptors() const {
    return {{"m", "kg", "Mass", 1.0, 0.01, 100.0, 0.1},
            {"k", "N/m", "Spring constant", 10.0, 0.01, 10000.0, 1.0},
            {"c", "N·s/m", "Damping coefficient", 0.5, 0.0, 100.0, 0.1}};
}

std::vector<Preset> MassSpringDamper::presets() const {
    return {{"Underdamped", "Low damping, oscillatory", {{"m", 1.0}, {"k", 10.0}, {"c", 0.5}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Critically damped", "Fastest non-oscillatory", {{"m", 1.0}, {"k", 10.0}, {"c", 6.32}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Overdamped", "Slow return", {{"m", 1.0}, {"k", 10.0}, {"c", 15.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Heavy mass", "Large inertia", {{"m", 50.0}, {"k", 100.0}, {"c", 5.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Stiff spring", "High frequency", {{"m", 0.1}, {"k", 1000.0}, {"c", 1.0}}}};
}

void MassSpringDamper::applyPreset(int index) {
    auto p = presets();
    if (index >= 0 && index < static_cast<int>(p.size())) setParameters(p[index].params);
}

std::vector<std::string> MassSpringDamper::stateNames() const { return {"x", "dx/dt"}; }
std::vector<std::string> MassSpringDamper::outputNames() const { return {"Position"}; }
std::vector<std::string> MassSpringDamper::inputNames() const { return {"Force"}; }
std::vector<std::string> MassSpringDamper::equationStrings() const {
    return {"m\\ddot{x} + c\\dot{x} + kx = F"};
}

}  // namespace Simulation
