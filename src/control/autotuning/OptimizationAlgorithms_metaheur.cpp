/**
 * @file OptimizationAlgorithms_metaheur.cpp
 * @brief Optimization — metaheuristics.
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
} // namespace Autotuning
} // namespace tether::control
