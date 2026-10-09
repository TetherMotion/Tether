/**
 * @file OptimizationAlgorithms_bayes.cpp
 * @brief Optimization — Bayesian optimization.
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

// Bayesian Optimization
// ============================================================================

double BayesianOptimization::kernel(const ParameterVector& x1, const ParameterVector& x2) {
    double dist = 0;
    for (size_t i = 0; i < x1.size(); ++i) {
        double d = (x1[i] - x2[i]) / m_lengthScale;
        dist += d * d;
    }
    
    switch (m_kernel) {
        case KernelType::SquaredExponential:
            return std::exp(-0.5 * dist);
        case KernelType::Matern32: {
            double r = std::sqrt(dist);
            return (1 + std::sqrt(3.0) * r) * std::exp(-std::sqrt(3.0) * r);
        }
        case KernelType::Matern52: {
            double r = std::sqrt(dist);
            return (1 + std::sqrt(5.0) * r + 5.0/3.0 * dist) * std::exp(-std::sqrt(5.0) * r);
        }
        default:
            return std::exp(-0.5 * dist);
    }
}

std::pair<double, double> BayesianOptimization::predict(const ParameterVector& x) {
    if (m_X.empty()) {
        return {0.0, 1.0};
    }

    const size_t n = m_X.size();

    // Build kernel matrix K (n×n) with noise on diagonal
    Eigen::MatrixXd K(n, n);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            K(i, j) = kernel(m_X[i], m_X[j]);
            if (i == j) K(i, j) += m_noise;
        }
    }

    // Build kernel vector k(x, X) and observation vector y
    Eigen::VectorXd kx(n);
    Eigen::VectorXd y(n);
    for (size_t i = 0; i < n; ++i) {
        kx(i) = kernel(x, m_X[i]);
        y(i) = m_y[i];
    }

    // Cholesky decomposition of K (positive-definite due to noise)
    Eigen::LLT<Eigen::MatrixXd> chol(K);
    if (chol.info() != Eigen::Success) {
        // Fallback to simplified prediction if K is not PD
        double sumWeights = 0;
        double mean = 0;
        for (size_t i = 0; i < n; ++i) {
            mean += kx(i) * y(i);
            sumWeights += kx(i);
        }
        if (sumWeights > 0) mean /= sumWeights;
        double variance = std::max(0.0, kernel(x, x));
        return {mean, std::sqrt(variance)};
    }

    // Solve K * alpha = y  =>  alpha = K^-1 * y via Cholesky
    Eigen::VectorXd alpha = chol.solve(y);

    // GP posterior mean: mu = k^T * alpha
    double mean = kx.dot(alpha);

    // GP posterior variance: sigma^2 = k(x,x) - k^T * K^-1 * k
    // Solve K * v = kx  =>  v = K^-1 * kx
    Eigen::VectorXd v = chol.solve(kx);
    double variance = kernel(x, x) - kx.dot(v);
    variance = std::max(0.0, variance);

    return {mean, std::sqrt(variance)};
}

double BayesianOptimization::acquisitionValue(const ParameterVector& x, double bestY) {
    auto [mu, sigma] = predict(x);
    if (sigma < 1e-10) return -std::numeric_limits<double>::max();
    
    double z = (bestY - mu) / sigma;
    double pdf = std::exp(-0.5 * z * z) / std::sqrt(2 * M_PI);
    double cdf = 0.5 * (1 + std::erf(z / std::sqrt(2)));
    
    switch (m_acquisition) {
        case AcquisitionFunction::ExpectedImprovement:
            return (bestY - mu) * cdf + sigma * pdf;
        case AcquisitionFunction::ProbabilityOfImprovement:
            return cdf;
        case AcquisitionFunction::LowerConfidenceBound:
            return -(mu - m_kappa * sigma);  // Negative because we minimize
        case AcquisitionFunction::ThompsonSampling:
            return randomGaussian(mu, sigma);
        default:
            return (bestY - mu) * cdf + sigma * pdf;
    }
}

ParameterVector BayesianOptimization::optimizeAcquisition(
    const std::vector<ParameterBounds>& bounds, double bestY) {
    
    // Simple random search for acquisition maximization
    ParameterVector bestX = randomInBounds(bounds);
    double bestAcq = acquisitionValue(bestX, bestY);
    
    for (int i = 0; i < 1000; ++i) {
        ParameterVector x = randomInBounds(bounds);
        double acq = acquisitionValue(x, bestY);
        if (acq > bestAcq) {
            bestAcq = acq;
            bestX = x;
        }
    }
    return bestX;
}

OptimizationResult BayesianOptimization::optimize(CostFunction& costFunction,
                                                   const ParameterVector& initialParams,
                                                   const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    m_X.clear();
    m_y.clear();
    
    // Initial samples
    m_X.push_back(initialParams);
    m_y.push_back(costFunction.evaluate(initialParams));
    result.functionEvaluations++;
    
    for (int i = 1; i < m_initialSamples; ++i) {
        ParameterVector x = randomInBounds(bounds);
        m_X.push_back(x);
        m_y.push_back(costFunction.evaluate(x));
        result.functionEvaluations++;
    }
    
    // Find initial best
    double bestY = *std::min_element(m_y.begin(), m_y.end());
    size_t bestIdx = std::distance(m_y.begin(), std::min_element(m_y.begin(), m_y.end()));
    result.bestCost = bestY;
    result.bestParameters = m_X[bestIdx];
    
    for (int iter = 0; iter < m_criteria.maxIterations - m_initialSamples; ++iter) {
        // Find next point by optimizing acquisition function
        ParameterVector nextX = optimizeAcquisition(bounds, bestY);
        
        // Evaluate
        double y = costFunction.evaluate(nextX);
        result.functionEvaluations++;
        
        m_X.push_back(nextX);
        m_y.push_back(y);
        
        if (y < bestY) {
            bestY = y;
            result.bestCost = y;
            result.bestParameters = nextX;
        }
        
        result.iterations = iter + m_initialSamples + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Maximum iterations reached";
    
    return result;
}

// ============================================================================
// Ant Colony Optimization
// ============================================================================

ParameterVector AntColonyOptimization::constructSolution(
    const std::vector<Solution>& archive,
    const std::vector<ParameterBounds>& bounds) {
    
    size_t n = bounds.size();
    size_t k = archive.size();
    ParameterVector solution(n);
    
    // Compute weights based on rank
    std::vector<double> weights(k);
    double sumWeights = 0;
    for (size_t i = 0; i < k; ++i) {
        weights[i] = std::exp(-std::pow(static_cast<double>(i), 2) / (2 * m_q * m_q * k * k));
        sumWeights += weights[i];
    }
    for (double& w : weights) w /= sumWeights;
    
    // Select solution from archive based on weights
    double r = randomUniform();
    double cumSum = 0;
    size_t selected = 0;
    for (size_t i = 0; i < k; ++i) {
        cumSum += weights[i];
        if (r <= cumSum) {
            selected = i;
            break;
        }
    }
    
    // Generate new solution around selected with Gaussian sampling
    double sigma = m_xi / (k - 1);
    for (size_t i = 0; i < k; ++i) {
        sigma += std::abs(archive[i].params[0] - archive[selected].params[0]);
    }
    sigma /= k;
    
    for (size_t j = 0; j < n; ++j) {
        double mean = archive[selected].params[j];
        solution[j] = randomGaussian(mean, sigma * bounds[j].range());
    }
    
    return projectToBounds(solution, bounds);
}

OptimizationResult AntColonyOptimization::optimize(CostFunction& costFunction,
                                                    const ParameterVector& initialParams,
                                                    const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    
    // Initialize archive
    std::vector<Solution> archive(m_archiveSize);
    archive[0] = {initialParams, costFunction.evaluate(initialParams)};
    result.functionEvaluations++;
    
    for (int i = 1; i < m_archiveSize; ++i) {
        archive[i].params = randomInBounds(bounds);
        archive[i].cost = costFunction.evaluate(archive[i].params);
        result.functionEvaluations++;
    }
    
    // Sort archive by cost
    std::sort(archive.begin(), archive.end(),
              [](const Solution& a, const Solution& b) { return a.cost < b.cost; });
    
    result.bestCost = archive[0].cost;
    result.bestParameters = archive[0].params;
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Generate new solutions
        std::vector<Solution> newSolutions(m_numAnts);
        for (int i = 0; i < m_numAnts; ++i) {
            newSolutions[i].params = constructSolution(archive, bounds);
            newSolutions[i].cost = costFunction.evaluate(newSolutions[i].params);
            result.functionEvaluations++;
        }
        
        // Add to archive and sort
        archive.insert(archive.end(), newSolutions.begin(), newSolutions.end());
        std::sort(archive.begin(), archive.end(),
                  [](const Solution& a, const Solution& b) { return a.cost < b.cost; });
        
        // Keep only best k solutions
        archive.resize(m_archiveSize);
        
        result.bestCost = archive[0].cost;
        result.bestParameters = archive[0].params;
        result.iterations = iter + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Maximum iterations reached";
    
    return result;
}

// ============================================================================
// Grey Wolf Optimizer
// ============================================================================

OptimizationResult GreyWolfOptimizer::optimize(CostFunction& costFunction,
                                                const ParameterVector& initialParams,
                                                const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    size_t n = initialParams.size();
    
    // Initialize pack
    std::vector<Wolf> pack(m_packSize);
    pack[0] = {initialParams, costFunction.evaluate(initialParams)};
    result.functionEvaluations++;
    
    for (int i = 1; i < m_packSize; ++i) {
        pack[i].position = randomInBounds(bounds);
        pack[i].fitness = costFunction.evaluate(pack[i].position);
        result.functionEvaluations++;
    }
    
    // Sort to find alpha, beta, delta
    std::sort(pack.begin(), pack.end(),
              [](const Wolf& a, const Wolf& b) { return a.fitness < b.fitness; });
    
    Wolf& alpha = pack[0];
    Wolf& beta = pack[1];
    Wolf& delta = pack[2];
    
    result.bestCost = alpha.fitness;
    result.bestParameters = alpha.position;
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Linearly decrease a from 2 to 0
        double a = 2.0 - 2.0 * iter / m_criteria.maxIterations;
        
        for (int i = 3; i < m_packSize; ++i) {
            ParameterVector newPos(n);
            
            for (size_t j = 0; j < n; ++j) {
                // Calculate A and C coefficients for each leader
                double A1 = 2 * a * randomUniform() - a;
                double C1 = 2 * randomUniform();
                double D_alpha = std::abs(C1 * alpha.position[j] - pack[i].position[j]);
                double X1 = alpha.position[j] - A1 * D_alpha;
                
                double A2 = 2 * a * randomUniform() - a;
                double C2 = 2 * randomUniform();
                double D_beta = std::abs(C2 * beta.position[j] - pack[i].position[j]);
                double X2 = beta.position[j] - A2 * D_beta;
                
                double A3 = 2 * a * randomUniform() - a;
                double C3 = 2 * randomUniform();
                double D_delta = std::abs(C3 * delta.position[j] - pack[i].position[j]);
                double X3 = delta.position[j] - A3 * D_delta;
                
                newPos[j] = (X1 + X2 + X3) / 3.0;
            }
            
            pack[i].position = projectToBounds(newPos, bounds);
            pack[i].fitness = costFunction.evaluate(pack[i].position);
            result.functionEvaluations++;
        }
        
        // Update hierarchy
        std::sort(pack.begin(), pack.end(),
                  [](const Wolf& a, const Wolf& b) { return a.fitness < b.fitness; });
        
        result.bestCost = pack[0].fitness;
        result.bestParameters = pack[0].position;
        result.iterations = iter + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(iter, result.bestCost, result.bestParameters);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Maximum iterations reached";
    
    return result;
}

// ============================================================================
} // namespace Autotuning
} // namespace tether::control
