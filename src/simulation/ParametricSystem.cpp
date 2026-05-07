#include "tether/simulation/DynamicalSystem.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

// ============================================================================
// ParametricSystem base class implementations
// ============================================================================
void ParametricSystem::setParameters(const ParamMap& params) {
    for (const auto& [k, v] : params) {
        if (params_.count(k)) params_[k] = v;
    }
}

void ParametricSystem::setParameter(const std::string& name, double value) {
    params_[name] = value;
}

double ParametricSystem::getParameter(const std::string& name) const {
    auto it = params_.find(name);
    if (it != params_.end()) return it->second;
    return 0.0;
}

}  // namespace Simulation
