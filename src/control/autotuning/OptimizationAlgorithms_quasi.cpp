/**
 * @file OptimizationAlgorithms_quasi.cpp
 * @brief Optimization — quasi-Newton / multi-start / hybrid.
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

// Powell's Method
// ============================================================================

double PowellMethod::lineSearch(CostFunction& cost, const ParameterVector& x,
                                const ParameterVector& direction,
                                const std::vector<ParameterBounds>& bounds) {
    // Golden section search
    const double phi = (1 + std::sqrt(5)) / 2;
    double a = -1.0, b = 1.0;
    
    auto eval = [&](double alpha) {
        ParameterVector point(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            point[i] = x[i] + alpha * direction[i];
        }
        point = projectToBounds(point, bounds);
        return cost.evaluate(point);
    };
    
    double c = b - (b - a) / phi;
    double d = a + (b - a) / phi;
    
    for (int i = 0; i < 20; ++i) {
        if (eval(c) < eval(d)) {
            b = d;
        } else {
            a = c;
        }
        c = b - (b - a) / phi;
        d = a + (b - a) / phi;
    }
    
    return (a + b) / 2;
}

OptimizationResult PowellMethod::optimize(CostFunction& costFunction,
                                           const ParameterVector& initialParams,
                                           const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    size_t n = initialParams.size();
    
    ParameterVector x = initialParams;
    double fx = costFunction.evaluate(x);
    result.functionEvaluations++;
    
    result.bestParameters = x;
    result.bestCost = fx;
    
    // Initialize directions to coordinate vectors
    std::vector<ParameterVector> directions(n);
    for (size_t i = 0; i < n; ++i) {
        directions[i].resize(n, 0.0);
        directions[i][i] = 1.0;
    }
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        ParameterVector x0 = x;
        double fx0 = fx;
        
        size_t maxDecreaseDir = 0;
        double maxDecrease = 0;
        
        // Line search along each direction
        for (size_t i = 0; i < n; ++i) {
            double prevFx = fx;
            double alpha = lineSearch(costFunction, x, directions[i], bounds);
            result.functionEvaluations += 40;  // Approximate for golden section
            
            for (size_t j = 0; j < n; ++j) {
                x[j] += alpha * directions[i][j];
            }
            x = projectToBounds(x, bounds);
            fx = costFunction.evaluate(x);
            result.functionEvaluations++;
            
            double decrease = prevFx - fx;
            if (decrease > maxDecrease) {
                maxDecrease = decrease;
                maxDecreaseDir = i;
            }
        }
        
        // Compute new direction
        ParameterVector newDir(n);
        for (size_t j = 0; j < n; ++j) {
            newDir[j] = x[j] - x0[j];
        }
        
        // Replace direction with maximum decrease
        directions[maxDecreaseDir] = newDir;
        
        // Line search along new direction
        double alpha = lineSearch(costFunction, x, newDir, bounds);
        result.functionEvaluations += 40;
        
        for (size_t j = 0; j < n; ++j) {
            x[j] += alpha * newDir[j];
        }
        x = projectToBounds(x, bounds);
        fx = costFunction.evaluate(x);
        result.functionEvaluations++;
        
        if (fx < result.bestCost) {
            result.bestCost = fx;
            result.bestParameters = x;
        }
        
        result.iterations = iter + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
        
        // Check convergence
        if (std::abs(fx0 - fx) < m_criteria.functionTolerance) {
            result.converged = true;
            result.terminationReason = "Function value converged";
            break;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    
    return result;
}

// ============================================================================
// BFGS Optimizer
// ============================================================================

ParameterVector BFGSOptimizer::computeGradient(CostFunction& cost, const ParameterVector& x) {
    ParameterVector grad(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        ParameterVector xp = x, xm = x;
        xp[i] += m_gradStep;
        xm[i] -= m_gradStep;
        grad[i] = (cost.evaluate(xp) - cost.evaluate(xm)) / (2 * m_gradStep);
    }
    return grad;
}

double BFGSOptimizer::lineSearch(CostFunction& cost, const ParameterVector& x,
                                  const ParameterVector& direction,
                                  const std::vector<ParameterBounds>& bounds) {
    // Backtracking line search with Armijo condition
    double alpha = 1.0;
    double c = 1e-4;
    double rho = 0.5;
    
    double fx = cost.evaluate(x);
    ParameterVector grad = computeGradient(cost, x);
    
    double dirDeriv = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        dirDeriv += grad[i] * direction[i];
    }
    
    for (int i = 0; i < 20; ++i) {
        ParameterVector xNew(x.size());
        for (size_t j = 0; j < x.size(); ++j) {
            xNew[j] = x[j] + alpha * direction[j];
        }
        xNew = projectToBounds(xNew, bounds);
        
        if (cost.evaluate(xNew) <= fx + c * alpha * dirDeriv) {
            return alpha;
        }
        alpha *= rho;
    }
    
    return alpha;
}

OptimizationResult BFGSOptimizer::optimize(CostFunction& costFunction,
                                            const ParameterVector& initialParams,
                                            const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    size_t n = initialParams.size();
    
    ParameterVector x = initialParams;
    double fx = costFunction.evaluate(x);
    result.functionEvaluations++;
    
    ParameterVector grad = computeGradient(costFunction, x);
    result.functionEvaluations += 2 * n;
    
    result.bestParameters = x;
    result.bestCost = fx;
    
    // Initialize Hessian approximation to identity
    std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) H[i][i] = 1.0;
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Compute search direction: p = -H * grad
        ParameterVector p(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                p[i] -= H[i][j] * grad[j];
            }
        }
        
        // Line search
        double alpha = lineSearch(costFunction, x, p, bounds);
        result.functionEvaluations += 40;
        
        // Update x
        ParameterVector xNew(n);
        for (size_t i = 0; i < n; ++i) {
            xNew[i] = x[i] + alpha * p[i];
        }
        xNew = projectToBounds(xNew, bounds);
        
        double fxNew = costFunction.evaluate(xNew);
        result.functionEvaluations++;
        
        ParameterVector gradNew = computeGradient(costFunction, xNew);
        result.functionEvaluations += 2 * n;
        
        // Compute s = x_new - x, y = grad_new - grad
        ParameterVector s(n), y(n);
        for (size_t i = 0; i < n; ++i) {
            s[i] = xNew[i] - x[i];
            y[i] = gradNew[i] - grad[i];
        }
        
        // BFGS update
        double ys = 0;
        for (size_t i = 0; i < n; ++i) ys += y[i] * s[i];
        
        if (ys > 1e-10) {
            ParameterVector Hy(n, 0.0);
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    Hy[i] += H[i][j] * y[j];
                }
            }
            
            double yHy = 0;
            for (size_t i = 0; i < n; ++i) yHy += y[i] * Hy[i];
            
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    H[i][j] += (ys + yHy) / (ys * ys) * s[i] * s[j];
                    H[i][j] -= (Hy[i] * s[j] + s[i] * Hy[j]) / ys;
                }
            }
        }
        
        x = xNew;
        fx = fxNew;
        grad = gradNew;
        
        if (fx < result.bestCost) {
            result.bestCost = fx;
            result.bestParameters = x;
        }
        
        result.iterations = iter + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
        
        // Check convergence
        double gradNorm = 0;
        for (double g : grad) gradNorm += g * g;
        gradNorm = std::sqrt(gradNorm);
        
        if (gradNorm < m_criteria.gradientTolerance) {
            result.converged = true;
            result.terminationReason = "Gradient converged";
            break;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    
    return result;
}

// ============================================================================
// Multi-Start Optimizer
// ============================================================================

MultiStartOptimizer::MultiStartOptimizer(std::shared_ptr<OptimizationAlgorithm> localOptimizer,
                                         int numStarts)
    : m_localOptimizer(std::move(localOptimizer)), m_numStarts(numStarts) {}

OptimizationResult MultiStartOptimizer::optimize(CostFunction& costFunction,
                                                  const ParameterVector& initialParams,
                                                  const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult bestResult;
    bestResult.bestCost = std::numeric_limits<double>::max();
    
    for (int i = 0; i < m_numStarts; ++i) {
        ParameterVector start = (i == 0) ? initialParams : randomInBounds(bounds);
        auto result = m_localOptimizer->optimize(costFunction, start, bounds);
        
        bestResult.functionEvaluations += result.functionEvaluations;
        bestResult.iterations += result.iterations;
        
        if (result.bestCost < bestResult.bestCost) {
            bestResult.bestCost = result.bestCost;
            bestResult.bestParameters = result.bestParameters;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    bestResult.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    bestResult.converged = true;
    bestResult.terminationReason = "All starts complete";
    
    return bestResult;
}

// ============================================================================
// Hybrid Optimizer
// ============================================================================

HybridOptimizer::HybridOptimizer(std::shared_ptr<OptimizationAlgorithm> globalOptimizer,
                                 std::shared_ptr<OptimizationAlgorithm> localOptimizer,
                                 double switchThreshold)
    : m_globalOptimizer(std::move(globalOptimizer)),
      m_localOptimizer(std::move(localOptimizer)),
      m_switchThreshold(switchThreshold) {}

OptimizationResult HybridOptimizer::optimize(CostFunction& costFunction,
                                              const ParameterVector& initialParams,
                                              const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Global search phase
    auto globalResult = m_globalOptimizer->optimize(costFunction, initialParams, bounds);
    
    // Local refinement
    auto localResult = m_localOptimizer->optimize(costFunction, globalResult.bestParameters, bounds);
    
    OptimizationResult result;
    result.bestParameters = localResult.bestParameters;
    result.bestCost = localResult.bestCost;
    result.iterations = globalResult.iterations + localResult.iterations;
    result.functionEvaluations = globalResult.functionEvaluations + localResult.functionEvaluations;
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Hybrid optimization complete";
    
    return result;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::shared_ptr<OptimizationAlgorithm> createOptimizer(const std::string& name) {
    if (name == "gradient" || name == "adam") {
        return std::make_shared<GradientDescent>(GradientDescent::Variant::Adam);
    } else if (name == "nelder-mead" || name == "simplex") {
        return std::make_shared<NelderMead>();
    } else if (name == "genetic" || name == "ga") {
        return std::make_shared<GeneticAlgorithm>();
    } else if (name == "pso" || name == "particle-swarm") {
        return std::make_shared<ParticleSwarmOptimization>();
    } else if (name == "sa" || name == "simulated-annealing") {
        return std::make_shared<SimulatedAnnealing>();
    } else if (name == "de" || name == "differential-evolution") {
        return std::make_shared<DifferentialEvolution>();
    } else if (name == "bayesian" || name == "bo") {
        return std::make_shared<BayesianOptimization>();
    } else if (name == "aco" || name == "ant-colony") {
        return std::make_shared<AntColonyOptimization>();
    } else if (name == "gwo" || name == "grey-wolf") {
        return std::make_shared<GreyWolfOptimizer>();
    } else if (name == "powell") {
        return std::make_shared<PowellMethod>();
    } else if (name == "bfgs") {
        return std::make_shared<BFGSOptimizer>();
    }
    return nullptr;
}

std::vector<std::string> getAvailableOptimizers() {
    return {
        "gradient", "adam", "nelder-mead", "genetic", "pso",
        "sa", "de", "bayesian", "aco", "gwo", "powell", "bfgs"
    };
}
} // namespace Autotuning
} // namespace tether::control
