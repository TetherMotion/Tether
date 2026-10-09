/**
 * @file OptimizationAlgorithms_local.cpp
 * @brief Optimization — local search.
 *
 * TU split out of OptimizationAlgorithms.cpp.
 */

#include "tether/control/autotuning/OptimizationAlgorithms.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>

#include <Eigen/Dense>

namespace tether::control {
namespace Autotuning {

// Gradient Descent
// ============================================================================

GradientDescent::GradientDescent(Variant variant) : m_variant(variant) {}

std::string GradientDescent::getName() const {
    switch (m_variant) {
        case Variant::Standard: return "Gradient Descent";
        case Variant::Momentum: return "Gradient Descent with Momentum";
        case Variant::Nesterov: return "Nesterov Accelerated Gradient";
        case Variant::Adam: return "Adam";
        case Variant::AdaGrad: return "AdaGrad";
        case Variant::RMSprop: return "RMSprop";
        default: return "Gradient Descent";
    }
}

std::string GradientDescent::getDescription() const {
    return "Gradient-based optimization with various enhancements for improved convergence.";
}

void GradientDescent::setAdamParams(double beta1, double beta2, double epsilon) {
    m_beta1 = beta1;
    m_beta2 = beta2;
    m_epsilon = epsilon;
}

ParameterVector GradientDescent::computeNumericalGradient(CostFunction& cost,
                                                          const ParameterVector& params,
                                                          const std::vector<ParameterBounds>& bounds) {
    ParameterVector grad(params.size());
    double h = m_gradientStepSize;
    
    for (size_t i = 0; i < params.size(); ++i) {
        ParameterVector plus = params;
        ParameterVector minus = params;
        plus[i] += h;
        minus[i] -= h;
        plus = projectToBounds(plus, bounds);
        minus = projectToBounds(minus, bounds);
        grad[i] = (cost.evaluate(plus) - cost.evaluate(minus)) / (2.0 * h);
    }
    return grad;
}

OptimizationResult GradientDescent::optimize(CostFunction& costFunction,
                                              const ParameterVector& initialParams,
                                              const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    result.bestParameters = initialParams;
    result.bestCost = costFunction.evaluate(initialParams);
    result.functionEvaluations = 1;
    
    ParameterVector x = initialParams;
    ParameterVector velocity(x.size(), 0.0);
    ParameterVector m(x.size(), 0.0);  // First moment (Adam)
    ParameterVector v(x.size(), 0.0);  // Second moment (Adam)
    
    double prevCost = result.bestCost;
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Compute gradient
        ParameterVector grad;
        if (m_useNumericalGradient || !costFunction.hasGradient()) {
            grad = computeNumericalGradient(costFunction, x, bounds);
            result.functionEvaluations += 2 * x.size();
        } else {
            auto optGrad = costFunction.gradient(x);
            if (optGrad) {
                grad = *optGrad;
            } else {
                grad = computeNumericalGradient(costFunction, x, bounds);
                result.functionEvaluations += 2 * x.size();
            }
        }
        
        // Apply variant-specific update
        switch (m_variant) {
            case Variant::Standard:
                for (size_t i = 0; i < x.size(); ++i) {
                    x[i] -= m_learningRate * grad[i];
                }
                break;
                
            case Variant::Momentum:
                for (size_t i = 0; i < x.size(); ++i) {
                    velocity[i] = m_momentum * velocity[i] - m_learningRate * grad[i];
                    x[i] += velocity[i];
                }
                break;
                
            case Variant::Nesterov:
                for (size_t i = 0; i < x.size(); ++i) {
                    double vPrev = velocity[i];
                    velocity[i] = m_momentum * velocity[i] - m_learningRate * grad[i];
                    x[i] += -m_momentum * vPrev + (1 + m_momentum) * velocity[i];
                }
                break;
                
            case Variant::Adam: {
                int t = iter + 1;
                for (size_t i = 0; i < x.size(); ++i) {
                    m[i] = m_beta1 * m[i] + (1 - m_beta1) * grad[i];
                    v[i] = m_beta2 * v[i] + (1 - m_beta2) * grad[i] * grad[i];
                    double mHat = m[i] / (1 - std::pow(m_beta1, t));
                    double vHat = v[i] / (1 - std::pow(m_beta2, t));
                    x[i] -= m_learningRate * mHat / (std::sqrt(vHat) + m_epsilon);
                }
                break;
            }
                
            case Variant::AdaGrad:
                for (size_t i = 0; i < x.size(); ++i) {
                    v[i] += grad[i] * grad[i];
                    x[i] -= m_learningRate * grad[i] / (std::sqrt(v[i]) + m_epsilon);
                }
                break;
                
            case Variant::RMSprop:
                for (size_t i = 0; i < x.size(); ++i) {
                    v[i] = 0.9 * v[i] + 0.1 * grad[i] * grad[i];
                    x[i] -= m_learningRate * grad[i] / (std::sqrt(v[i]) + m_epsilon);
                }
                break;
        }
        
        x = projectToBounds(x, bounds);
        double cost = costFunction.evaluate(x);
        result.functionEvaluations++;
        
        if (cost < result.bestCost) {
            result.bestCost = cost;
            result.bestParameters = x;
        }
        
        if (m_trackHistory) {
            result.costHistory.push_back(cost);
            result.parameterHistory.push_back(x);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, cost, x);
        }
        
        // Check convergence
        double gradNorm = 0;
        for (double g : grad) gradNorm += g * g;
        gradNorm = std::sqrt(gradNorm);
        
        if (shouldTerminate(iter, result.functionEvaluations, 
                           std::abs(cost - prevCost), 0, gradNorm)) {
            result.converged = true;
            result.terminationReason = "Convergence criteria met";
            break;
        }
        
        prevCost = cost;
        result.iterations = iter + 1;
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    
    return result;
}

// ============================================================================
// Nelder-Mead
// ============================================================================

void NelderMead::setCoefficients(double alpha, double gamma, double rho, double sigma) {
    m_alpha = alpha;
    m_gamma = gamma;
    m_rho = rho;
    m_sigma = sigma;
}

std::vector<ParameterVector> NelderMead::initializeSimplex(const ParameterVector& x0,
                                                           const std::vector<ParameterBounds>& bounds) {
    size_t n = x0.size();
    std::vector<ParameterVector> simplex(n + 1);
    simplex[0] = x0;
    
    for (size_t i = 0; i < n; ++i) {
        simplex[i + 1] = x0;
        double step = m_initialSize * bounds[i].range();
        if (x0[i] + step <= bounds[i].max) {
            simplex[i + 1][i] += step;
        } else {
            simplex[i + 1][i] -= step;
        }
    }
    return simplex;
}

ParameterVector NelderMead::centroid(const std::vector<ParameterVector>& simplex,
                                      size_t excludeIndex) {
    size_t n = simplex[0].size();
    ParameterVector c(n, 0.0);
    int count = 0;
    
    for (size_t i = 0; i < simplex.size(); ++i) {
        if (i != excludeIndex) {
            for (size_t j = 0; j < n; ++j) {
                c[j] += simplex[i][j];
            }
            count++;
        }
    }
    
    for (size_t j = 0; j < n; ++j) {
        c[j] /= count;
    }
    return c;
}

OptimizationResult NelderMead::optimize(CostFunction& costFunction,
                                         const ParameterVector& initialParams,
                                         const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    size_t n = initialParams.size();
    
    // Initialize simplex
    auto simplex = initializeSimplex(initialParams, bounds);
    std::vector<double> costs(n + 1);
    
    for (size_t i = 0; i <= n; ++i) {
        costs[i] = costFunction.evaluate(simplex[i]);
        result.functionEvaluations++;
    }

    // Handle degenerate case: zero-dimensional optimization
    if (n == 0) {
        result.bestCost = costs[0];
        result.bestParameters = simplex[0];
        result.elapsedTime = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - startTime).count();
        return result;
    }
    
    result.bestCost = *std::min_element(costs.begin(), costs.end());
    size_t bestIdx = std::distance(costs.begin(), 
                                   std::min_element(costs.begin(), costs.end()));
    result.bestParameters = simplex[bestIdx];
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Sort vertices by cost
        std::vector<size_t> order(n + 1);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), 
                  [&costs](size_t a, size_t b) { return costs[a] < costs[b]; });
        
        size_t best = order[0];
        size_t worst = order[n];
        size_t secondWorst = order[n - 1];
        
        // Centroid of all points except worst
        auto c = centroid(simplex, worst);
        
        // Reflection
        ParameterVector xr(n);
        for (size_t j = 0; j < n; ++j) {
            xr[j] = c[j] + m_alpha * (c[j] - simplex[worst][j]);
        }
        xr = projectToBounds(xr, bounds);
        double fr = costFunction.evaluate(xr);
        result.functionEvaluations++;
        
        if (fr >= costs[best] && fr < costs[secondWorst]) {
            // Accept reflection
            simplex[worst] = xr;
            costs[worst] = fr;
        } else if (fr < costs[best]) {
            // Try expansion
            ParameterVector xe(n);
            for (size_t j = 0; j < n; ++j) {
                xe[j] = c[j] + m_gamma * (xr[j] - c[j]);
            }
            xe = projectToBounds(xe, bounds);
            double fe = costFunction.evaluate(xe);
            result.functionEvaluations++;
            
            if (fe < fr) {
                simplex[worst] = xe;
                costs[worst] = fe;
            } else {
                simplex[worst] = xr;
                costs[worst] = fr;
            }
        } else {
            // Contraction
            ParameterVector xc(n);
            if (fr < costs[worst]) {
                // Outside contraction
                for (size_t j = 0; j < n; ++j) {
                    xc[j] = c[j] + m_rho * (xr[j] - c[j]);
                }
            } else {
                // Inside contraction
                for (size_t j = 0; j < n; ++j) {
                    xc[j] = c[j] + m_rho * (simplex[worst][j] - c[j]);
                }
            }
            xc = projectToBounds(xc, bounds);
            double fc = costFunction.evaluate(xc);
            result.functionEvaluations++;
            
            if (fc < std::min(fr, costs[worst])) {
                simplex[worst] = xc;
                costs[worst] = fc;
            } else {
                // Shrink
                for (size_t i = 0; i <= n; ++i) {
                    if (i != best) {
                        for (size_t j = 0; j < n; ++j) {
                            simplex[i][j] = simplex[best][j] + 
                                           m_sigma * (simplex[i][j] - simplex[best][j]);
                        }
                        simplex[i] = projectToBounds(simplex[i], bounds);
                        costs[i] = costFunction.evaluate(simplex[i]);
                        result.functionEvaluations++;
                    }
                }
            }
        }
        
        // Update best
        for (size_t i = 0; i <= n; ++i) {
            if (costs[i] < result.bestCost) {
                result.bestCost = costs[i];
                result.bestParameters = simplex[i];
            }
        }
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
        
        // Check convergence
        double maxDiff = 0;
        for (size_t i = 1; i <= n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                maxDiff = std::max(maxDiff, std::abs(simplex[i][j] - simplex[0][j]));
            }
        }
        
        if (maxDiff < m_criteria.parameterTolerance) {
            result.converged = true;
            result.terminationReason = "Simplex converged";
            break;
        }
        
        result.iterations = iter + 1;
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    
    return result;
}

// ============================================================================
} // namespace Autotuning
} // namespace tether::control
