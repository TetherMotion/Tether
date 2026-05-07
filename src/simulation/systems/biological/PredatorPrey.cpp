#include "tether/simulation/systems/biological/PredatorPrey.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

PredatorPrey::PredatorPrey() {
    initParam("alpha", 1.0);    // prey birth rate
    initParam("beta_pp", 0.5);  // predation rate
    initParam("delta_pp", 0.5); // predator reproduction
    initParam("gamma_pp", 1.0); // predator death rate
    initParam("K", 100.0);      // carrying capacity (0=unlimited)
}

StateVector PredatorPrey::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double alpha = params_.at("alpha"), beta = params_.at("beta_pp");
    double delta = params_.at("delta_pp"), gam = params_.at("gamma_pp"), K = params_.at("K");
    double harvest = u.empty() ? 0.0 : u[0];

    double x = std::max(s[0], 0.0); // prey
    double y = std::max(s[1], 0.0); // predator

    double prey_growth = alpha * x;
    if (K > 0) prey_growth *= (1.0 - x / K);

    double dx = prey_growth - beta * x * y - harvest;
    double dy = delta * x * y - gam * y;

    return {dx, dy};
}

StateVector PredatorPrey::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {std::max(s[0], 0.0), std::max(s[1], 0.0)};
}
StateVector PredatorPrey::defaultInitialState() const { return {10.0, 5.0}; }

std::vector<ParamDescriptor> PredatorPrey::parameterDescriptors() const {
    return {{"alpha","1/t","Prey birth rate",1.0,0.01,5.0,0.01},
            {"beta_pp","1/(t·pred)","Predation rate",0.5,0.01,2.0,0.01},
            {"delta_pp","1/(t·prey)","Predator reprod",0.5,0.01,2.0,0.01},
            {"gamma_pp","1/t","Predator death",1.0,0.01,5.0,0.01},
            {"K","","Carrying capacity",100.0,0.0,10000.0,1.0}};
}

std::vector<Preset> PredatorPrey::presets() const {
    return {{"Classic LV","No carrying capacity",
             {{"alpha",1.0},{"beta_pp",0.5},{"delta_pp",0.5},{"gamma_pp",1.0},{"K",0.0}}}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
            {"Logistic prey","K=100",
             {{"alpha",1.0},{"beta_pp",0.5},{"delta_pp",0.5},{"gamma_pp",1.0},{"K",100.0}}}};
}
void PredatorPrey::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> PredatorPrey::stateNames() const { return {"Prey","Predator"}; }
std::vector<std::string> PredatorPrey::outputNames() const { return {"Prey pop","Predator pop"}; }
std::vector<std::string> PredatorPrey::inputNames() const { return {"Harvest rate"}; }

}  // namespace Simulation
