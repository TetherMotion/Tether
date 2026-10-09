/**
 * @file OptimizationAlgorithms.cpp
 * @brief Implementation of optimization algorithms for controller autotuning
 */

#include "tether/control/autotuning/OptimizationAlgorithms.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>

#include <Eigen/Dense>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// OptimizationAlgorithm Base Class
// ============================================================================

bool OptimizationAlgorithm::shouldTerminate(int iterations, int funcEvals,
                                            double costChange, double paramChange,
                                            double gradNorm) const {
    if (iterations >= m_criteria.maxIterations) return true;
    if (funcEvals >= m_criteria.maxFunctionEvaluations) return true;
    if (std::abs(costChange) < m_criteria.functionTolerance) return true;
    if (paramChange < m_criteria.parameterTolerance) return true;
    if (gradNorm < m_criteria.gradientTolerance) return true;
    return false;
}

double OptimizationAlgorithm::randomUniform(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(m_rng);
}

int OptimizationAlgorithm::randomInt(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(m_rng);
}

double OptimizationAlgorithm::randomGaussian(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(m_rng);
}

ParameterVector OptimizationAlgorithm::projectToBounds(const ParameterVector& params,
                                                        const std::vector<ParameterBounds>& bounds) {
    ParameterVector result = params;
    for (size_t i = 0; i < params.size() && i < bounds.size(); ++i) {
        result[i] = bounds[i].clamp(params[i]);
    }
    return result;
}

ParameterVector OptimizationAlgorithm::randomInBounds(const std::vector<ParameterBounds>& bounds) {
    ParameterVector result(bounds.size());
    for (size_t i = 0; i < bounds.size(); ++i) {
        result[i] = randomUniform(bounds[i].min, bounds[i].max);
    }
    return result;
}

// ============================================================================
} // namespace Autotuning
} // namespace tether::control
