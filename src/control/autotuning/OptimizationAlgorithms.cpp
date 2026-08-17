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
// Genetic Algorithm
// ============================================================================

std::vector<GeneticAlgorithm::Individual> GeneticAlgorithm::initializePopulation(
    const ParameterVector& seed,
    const std::vector<ParameterBounds>& bounds) {
    
    std::vector<Individual> population(m_populationSize);
    population[0].genes = seed;
    
    for (int i = 1; i < m_populationSize; ++i) {
        population[i].genes = randomInBounds(bounds);
    }
    return population;
}

std::pair<int, int> GeneticAlgorithm::selectParents(const std::vector<Individual>& population) {
    auto tournamentSelect = [this, &population]() {
        int best = randomInt(0, static_cast<int>(population.size()) - 1);
        for (int i = 1; i < m_tournamentSize; ++i) {
            int candidate = randomInt(0, static_cast<int>(population.size()) - 1);
            if (population[candidate].fitness < population[best].fitness) {
                best = candidate;
            }
        }
        return best;
    };
    
    return {tournamentSelect(), tournamentSelect()};
}

GeneticAlgorithm::Individual GeneticAlgorithm::crossover(
    const Individual& parent1, const Individual& parent2,
    const std::vector<ParameterBounds>& bounds) {
    
    Individual child;
    size_t n = parent1.genes.size();
    child.genes.resize(n);
    
    switch (m_crossover) {
        case CrossoverMethod::SinglePoint: {
            size_t point = randomInt(0, static_cast<int>(n) - 1);
            for (size_t i = 0; i < n; ++i) {
                child.genes[i] = (i < point) ? parent1.genes[i] : parent2.genes[i];
            }
            break;
        }
        case CrossoverMethod::TwoPoint: {
            size_t p1 = randomInt(0, static_cast<int>(n) - 1);
            size_t p2 = randomInt(0, static_cast<int>(n) - 1);
            if (p1 > p2) std::swap(p1, p2);
            for (size_t i = 0; i < n; ++i) {
                child.genes[i] = (i >= p1 && i < p2) ? parent2.genes[i] : parent1.genes[i];
            }
            break;
        }
        case CrossoverMethod::Uniform:
            for (size_t i = 0; i < n; ++i) {
                child.genes[i] = (randomUniform() < 0.5) ? parent1.genes[i] : parent2.genes[i];
            }
            break;
        case CrossoverMethod::Arithmetic:
            for (size_t i = 0; i < n; ++i) {
                double alpha = randomUniform();
                child.genes[i] = alpha * parent1.genes[i] + (1 - alpha) * parent2.genes[i];
            }
            break;
        case CrossoverMethod::SBX: {
            double eta = 2.0;
            for (size_t i = 0; i < n; ++i) {
                double u = randomUniform();
                double beta;
                if (u <= 0.5) {
                    beta = std::pow(2 * u, 1.0 / (eta + 1));
                } else {
                    beta = std::pow(1.0 / (2 * (1 - u)), 1.0 / (eta + 1));
                }
                child.genes[i] = 0.5 * ((1 + beta) * parent1.genes[i] + 
                                        (1 - beta) * parent2.genes[i]);
            }
            break;
        }
    }
    
    child.genes = projectToBounds(child.genes, bounds);
    return child;
}

void GeneticAlgorithm::mutate(Individual& individual, 
                              const std::vector<ParameterBounds>& bounds) {
    for (size_t i = 0; i < individual.genes.size(); ++i) {
        if (randomUniform() < m_mutationRate) {
            switch (m_mutation) {
                case MutationMethod::Gaussian:
                    individual.genes[i] += randomGaussian(0, m_mutationStrength * bounds[i].range());
                    break;
                case MutationMethod::Uniform:
                    individual.genes[i] = randomUniform(bounds[i].min, bounds[i].max);
                    break;
                case MutationMethod::Polynomial: {
                    double eta = 20.0;
                    double u = randomUniform();
                    double delta;
                    if (u < 0.5) {
                        delta = std::pow(2 * u, 1.0 / (eta + 1)) - 1;
                    } else {
                        delta = 1 - std::pow(2 * (1 - u), 1.0 / (eta + 1));
                    }
                    individual.genes[i] += delta * bounds[i].range();
                    break;
                }
            }
            individual.genes[i] = bounds[i].clamp(individual.genes[i]);
        }
    }
}

OptimizationResult GeneticAlgorithm::optimize(CostFunction& costFunction,
                                               const ParameterVector& initialParams,
                                               const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    auto population = initializePopulation(initialParams, bounds);
    
    // Evaluate initial population
    for (auto& ind : population) {
        ind.fitness = costFunction.evaluate(ind.genes);
        result.functionEvaluations++;
        if (ind.fitness < result.bestCost) {
            result.bestCost = ind.fitness;
            result.bestParameters = ind.genes;
        }
    }
    
    for (int gen = 0; gen < m_criteria.maxIterations; ++gen) {
        // Sort by fitness
        std::sort(population.begin(), population.end(),
                  [](const Individual& a, const Individual& b) {
                      return a.fitness < b.fitness;
                  });
        
        std::vector<Individual> newPopulation;
        
        // Elitism
        for (int i = 0; i < m_eliteCount && i < m_populationSize; ++i) {
            newPopulation.push_back(population[i]);
        }
        
        // Generate offspring
        while (static_cast<int>(newPopulation.size()) < m_populationSize) {
            auto [p1, p2] = selectParents(population);
            
            Individual child;
            if (randomUniform() < m_crossoverRate) {
                child = crossover(population[p1], population[p2], bounds);
            } else {
                child = population[p1];
            }
            
            mutate(child, bounds);
            child.fitness = costFunction.evaluate(child.genes);
            result.functionEvaluations++;
            
            newPopulation.push_back(child);
            
            if (child.fitness < result.bestCost) {
                result.bestCost = child.fitness;
                result.bestParameters = child.genes;
            }
        }
        
        population = std::move(newPopulation);
        result.iterations = gen + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(gen, result.bestCost, result.bestParameters);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Maximum generations reached";
    
    return result;
}

// ============================================================================
// Particle Swarm Optimization
// ============================================================================

std::vector<int> ParticleSwarmOptimization::getNeighbors(int particleIndex, int swarmSize) {
    std::vector<int> neighbors;
    
    switch (m_topology) {
        case Topology::Global:
            for (int i = 0; i < swarmSize; ++i) {
                neighbors.push_back(i);
            }
            break;
        case Topology::Ring:
            neighbors.push_back((particleIndex - 1 + swarmSize) % swarmSize);
            neighbors.push_back(particleIndex);
            neighbors.push_back((particleIndex + 1) % swarmSize);
            break;
        case Topology::VonNeumann: {
            int side = static_cast<int>(std::sqrt(swarmSize));
            int row = particleIndex / side;
            int col = particleIndex % side;
            neighbors.push_back(particleIndex);
            neighbors.push_back(((row - 1 + side) % side) * side + col);
            neighbors.push_back(((row + 1) % side) * side + col);
            neighbors.push_back(row * side + (col - 1 + side) % side);
            neighbors.push_back(row * side + (col + 1) % side);
            break;
        }
    }
    return neighbors;
}

OptimizationResult ParticleSwarmOptimization::optimize(CostFunction& costFunction,
                                                        const ParameterVector& initialParams,
                                                        const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    size_t n = initialParams.size();
    
    // Initialize swarm
    std::vector<Particle> swarm(m_swarmSize);
    ParameterVector globalBest = initialParams;
    double globalBestCost = std::numeric_limits<double>::max();
    
    for (int i = 0; i < m_swarmSize; ++i) {
        if (i == 0) {
            swarm[i].position = initialParams;
        } else {
            swarm[i].position = randomInBounds(bounds);
        }
        
        swarm[i].velocity.resize(n, 0.0);
        swarm[i].bestPosition = swarm[i].position;
        swarm[i].bestCost = costFunction.evaluate(swarm[i].position);
        result.functionEvaluations++;
        
        if (swarm[i].bestCost < globalBestCost) {
            globalBestCost = swarm[i].bestCost;
            globalBest = swarm[i].position;
        }
    }
    
    result.bestCost = globalBestCost;
    result.bestParameters = globalBest;
    
    for (int iter = 0; iter < m_criteria.maxIterations; ++iter) {
        // Update inertia weight if adaptive
        double w = m_inertia;
        if (m_adaptiveInertia) {
            w = m_inertiaMax - (m_inertiaMax - m_inertiaMin) * iter / m_criteria.maxIterations;
        }
        
        for (int i = 0; i < m_swarmSize; ++i) {
            // Find neighborhood best
            auto neighbors = getNeighbors(i, m_swarmSize);
            ParameterVector neighborBest = swarm[neighbors[0]].bestPosition;
            double neighborBestCost = swarm[neighbors[0]].bestCost;
            for (int j : neighbors) {
                if (swarm[j].bestCost < neighborBestCost) {
                    neighborBestCost = swarm[j].bestCost;
                    neighborBest = swarm[j].bestPosition;
                }
            }
            
            // Update velocity and position
            for (size_t d = 0; d < n; ++d) {
                double r1 = randomUniform();
                double r2 = randomUniform();
                
                swarm[i].velocity[d] = w * swarm[i].velocity[d] +
                    m_cognitive * r1 * (swarm[i].bestPosition[d] - swarm[i].position[d]) +
                    m_social * r2 * (neighborBest[d] - swarm[i].position[d]);
                
                // Clamp velocity
                double vmax = m_velocityClamp * bounds[d].range();
                swarm[i].velocity[d] = std::max(-vmax, std::min(vmax, swarm[i].velocity[d]));
                
                swarm[i].position[d] += swarm[i].velocity[d];
            }
            
            swarm[i].position = projectToBounds(swarm[i].position, bounds);
            
            // Evaluate
            double cost = costFunction.evaluate(swarm[i].position);
            result.functionEvaluations++;
            
            // Update personal best
            if (cost < swarm[i].bestCost) {
                swarm[i].bestCost = cost;
                swarm[i].bestPosition = swarm[i].position;
                
                // Update global best
                if (cost < globalBestCost) {
                    globalBestCost = cost;
                    globalBest = swarm[i].position;
                }
            }
        }
        
        result.bestCost = globalBestCost;
        result.bestParameters = globalBest;
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
// Simulated Annealing
// ============================================================================

double SimulatedAnnealing::cool(double temp, int iteration) {
    switch (m_schedule) {
        case CoolingSchedule::Exponential:
            return temp * m_coolingRate;
        case CoolingSchedule::Linear:
            return temp - (m_initialTemp - m_finalTemp) / m_criteria.maxIterations;
        case CoolingSchedule::Logarithmic:
            return m_initialTemp / std::log(2.0 + iteration);
        case CoolingSchedule::Adaptive:
        default:
            return temp * m_coolingRate;
    }
}

ParameterVector SimulatedAnnealing::generateNeighbor(const ParameterVector& current,
                                                      const std::vector<ParameterBounds>& bounds) {
    ParameterVector neighbor = current;
    for (size_t i = 0; i < current.size(); ++i) {
        neighbor[i] += randomGaussian(0, m_stepSize * bounds[i].range());
    }
    return projectToBounds(neighbor, bounds);
}

OptimizationResult SimulatedAnnealing::optimize(CostFunction& costFunction,
                                                 const ParameterVector& initialParams,
                                                 const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    ParameterVector current = initialParams;
    double currentCost = costFunction.evaluate(current);
    result.functionEvaluations++;
    
    result.bestParameters = current;
    result.bestCost = currentCost;
    
    double temp = m_initialTemp;
    int totalIter = 0;
    
    while (temp > m_finalTemp && totalIter < m_criteria.maxIterations) {
        for (int i = 0; i < m_iterationsPerTemp; ++i) {
            ParameterVector neighbor = generateNeighbor(current, bounds);
            double neighborCost = costFunction.evaluate(neighbor);
            result.functionEvaluations++;
            
            double delta = neighborCost - currentCost;
            
            // Accept if better or with probability exp(-delta/T)
            if (delta < 0 || randomUniform() < std::exp(-delta / temp)) {
                current = neighbor;
                currentCost = neighborCost;
                
                if (currentCost < result.bestCost) {
                    result.bestCost = currentCost;
                    result.bestParameters = current;
                }
            }
            
            totalIter++;
        }
        
        temp = cool(temp, totalIter);
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(totalIter, result.bestCost, result.bestParameters);
        }
    }
    
    result.iterations = totalIter;
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Cooling complete";
    
    return result;
}

// ============================================================================
// Differential Evolution
// ============================================================================

ParameterVector DifferentialEvolution::mutate(const std::vector<ParameterVector>& population,
                                               int targetIndex, const ParameterVector& best,
                                               const std::vector<ParameterBounds>& bounds) {
    int popSize = static_cast<int>(population.size());
    int n = static_cast<int>(population[0].size());
    
    // Select random individuals (different from target)
    std::vector<int> indices;
    for (int i = 0; i < popSize; ++i) {
        if (i != targetIndex) indices.push_back(i);
    }
    std::shuffle(indices.begin(), indices.end(), m_rng);
    
    ParameterVector mutant(n);
    
    switch (m_strategy) {
        case Strategy::Best1Bin:
            for (int j = 0; j < n; ++j) {
                mutant[j] = best[j] + m_F * (population[indices[0]][j] - population[indices[1]][j]);
            }
            break;
        case Strategy::Rand1Bin:
            for (int j = 0; j < n; ++j) {
                mutant[j] = population[indices[0]][j] + 
                           m_F * (population[indices[1]][j] - population[indices[2]][j]);
            }
            break;
        case Strategy::RandToBest1Bin:
            for (int j = 0; j < n; ++j) {
                mutant[j] = population[targetIndex][j] + 
                           m_F * (best[j] - population[targetIndex][j]) +
                           m_F * (population[indices[0]][j] - population[indices[1]][j]);
            }
            break;
        case Strategy::Best2Bin:
            for (int j = 0; j < n; ++j) {
                mutant[j] = best[j] + 
                           m_F * (population[indices[0]][j] - population[indices[1]][j]) +
                           m_F * (population[indices[2]][j] - population[indices[3]][j]);
            }
            break;
        case Strategy::Rand2Bin:
            for (int j = 0; j < n; ++j) {
                mutant[j] = population[indices[0]][j] + 
                           m_F * (population[indices[1]][j] - population[indices[2]][j]) +
                           m_F * (population[indices[3]][j] - population[indices[4]][j]);
            }
            break;
        case Strategy::CurrentToPBest:
        default:
            for (int j = 0; j < n; ++j) {
                mutant[j] = population[targetIndex][j] + 
                           m_F * (best[j] - population[targetIndex][j]) +
                           m_F * (population[indices[0]][j] - population[indices[1]][j]);
            }
            break;
    }
    
    return projectToBounds(mutant, bounds);
}

ParameterVector DifferentialEvolution::crossover(const ParameterVector& target,
                                                  const ParameterVector& mutant) {
    size_t n = target.size();
    ParameterVector trial(n);
    size_t jrand = randomInt(0, static_cast<int>(n) - 1);
    
    for (size_t j = 0; j < n; ++j) {
        if (randomUniform() < m_CR || j == jrand) {
            trial[j] = mutant[j];
        } else {
            trial[j] = target[j];
        }
    }
    return trial;
}

OptimizationResult DifferentialEvolution::optimize(CostFunction& costFunction,
                                                    const ParameterVector& initialParams,
                                                    const std::vector<ParameterBounds>& bounds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    OptimizationResult result;
    
    // Initialize population
    std::vector<ParameterVector> population(m_populationSize);
    std::vector<double> costs(m_populationSize);
    
    population[0] = initialParams;
    for (int i = 1; i < m_populationSize; ++i) {
        population[i] = randomInBounds(bounds);
    }
    
    ParameterVector best = initialParams;
    double bestCost = std::numeric_limits<double>::max();
    
    for (int i = 0; i < m_populationSize; ++i) {
        costs[i] = costFunction.evaluate(population[i]);
        result.functionEvaluations++;
        if (costs[i] < bestCost) {
            bestCost = costs[i];
            best = population[i];
        }
    }
    
    result.bestCost = bestCost;
    result.bestParameters = best;
    
    for (int gen = 0; gen < m_criteria.maxIterations; ++gen) {
        for (int i = 0; i < m_populationSize; ++i) {
            ParameterVector mutant = mutate(population, i, best, bounds);
            ParameterVector trial = crossover(population[i], mutant);
            
            double trialCost = costFunction.evaluate(trial);
            result.functionEvaluations++;
            
            if (trialCost <= costs[i]) {
                population[i] = trial;
                costs[i] = trialCost;
                
                if (trialCost < bestCost) {
                    bestCost = trialCost;
                    best = trial;
                }
            }
        }
        
        result.bestCost = bestCost;
        result.bestParameters = best;
        result.iterations = gen + 1;
        
        if (m_trackHistory) {
            result.costHistory.push_back(result.bestCost);
        }
        
        if (m_progressCallback) {
            m_progressCallback(gen, result.bestCost, result.bestParameters);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedTime = std::chrono::duration<double>(endTime - startTime).count();
    result.converged = true;
    result.terminationReason = "Maximum generations reached";
    
    return result;
}

// ============================================================================
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
