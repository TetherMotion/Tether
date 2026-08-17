/**
 * @file OptimizationAlgorithms.hpp
 * @brief Optimization Algorithms for Controller Autotuning
 * 
 * @details
 * This file implements various optimization algorithms used for controller tuning.
 * All algorithms follow a common interface and can be used interchangeably.
 * 
 * ## Available Algorithms
 * 
 * ### Gradient-Based Methods
 * - Gradient Descent (with momentum, Adam variants)
 * - Newton's Method
 * - BFGS / L-BFGS
 * 
 * ### Direct Search Methods
 * - Nelder-Mead (Simplex)
 * - Pattern Search
 * - Powell's Method
 * 
 * ### Evolutionary Algorithms
 * - Genetic Algorithm (GA)
 * - Differential Evolution (DE)
 * - Evolution Strategy (ES)
 * 
 * ### Swarm Intelligence
 * - Particle Swarm Optimization (PSO)
 * - Ant Colony Optimization (ACO)
 * - Grey Wolf Optimization (GWO)
 * 
 * ### Probabilistic Methods
 * - Simulated Annealing (SA)
 * - Bayesian Optimization
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include <random>
#include <functional>
#include <queue>
#include <map>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// Optimization Algorithm Base Class
// ============================================================================

/**
 * @brief Termination criteria for optimization
 */
struct TerminationCriteria {
    int maxIterations{1000};
    int maxFunctionEvaluations{10000};
    double functionTolerance{1e-8};
    double parameterTolerance{1e-8};
    double gradientTolerance{1e-6};
    double maxTime{std::numeric_limits<double>::max()};  // seconds
};

/**
 * @brief Optimization result
 */
struct OptimizationResult {
    ParameterVector bestParameters;
    double bestCost{std::numeric_limits<double>::max()};
    int iterations{0};
    int functionEvaluations{0};
    double elapsedTime{0.0};
    bool converged{false};
    std::string terminationReason;
    
    // Convergence history (optional)
    std::vector<double> costHistory;
    std::vector<ParameterVector> parameterHistory;
};

/**
 * @brief Abstract base class for optimization algorithms
 */
class OptimizationAlgorithm {
public:
    virtual ~OptimizationAlgorithm() = default;
    
    /**
     * @brief Get algorithm name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Get algorithm description
     */
    virtual std::string getDescription() const = 0;
    
    /**
     * @brief Check if algorithm requires gradient
     */
    virtual bool requiresGradient() const = 0;
    
    /**
     * @brief Optimize the given cost function
     * @param costFunction Function to minimize
     * @param initialParams Starting point
     * @param bounds Parameter bounds
     * @return Optimization result
     */
    virtual OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) = 0;
    
    /**
     * @brief Set termination criteria
     */
    void setTerminationCriteria(const TerminationCriteria& criteria) {
        m_criteria = criteria;
    }
    
    /**
     * @brief Set random seed for reproducibility
     */
    void setRandomSeed(unsigned int seed) {
        m_rng.seed(seed);
        m_seedSet = true;
    }
    
    /**
     * @brief Enable/disable convergence history tracking
     */
    void setTrackHistory(bool track) { m_trackHistory = track; }
    
    /**
     * @brief Set callback for progress updates
     */
    void setProgressCallback(
        std::function<void(int iteration, double cost, const ParameterVector&)> callback) {
        m_progressCallback = callback;
    }
    
protected:
    TerminationCriteria m_criteria;
    std::mt19937 m_rng{std::random_device{}()};
    bool m_seedSet{false};
    bool m_trackHistory{false};
    std::function<void(int, double, const ParameterVector&)> m_progressCallback;
    
    /**
     * @brief Check if termination criteria met
     */
    bool shouldTerminate(int iterations, int funcEvals, 
                        double costChange, double paramChange,
                        double gradNorm = std::numeric_limits<double>::max()) const;
    
    /**
     * @brief Generate random number in range
     */
    double randomUniform(double min = 0.0, double max = 1.0);
    
    /**
     * @brief Generate random integer in range
     */
    int randomInt(int min, int max);
    
    /**
     * @brief Generate Gaussian random number
     */
    double randomGaussian(double mean = 0.0, double stddev = 1.0);
    
    /**
     * @brief Project parameters onto bounds
     */
    static ParameterVector projectToBounds(const ParameterVector& params,
                                           const std::vector<ParameterBounds>& bounds);
    
    /**
     * @brief Generate random point within bounds
     */
    ParameterVector randomInBounds(const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Gradient Descent
// ============================================================================

/**
 * @brief Gradient Descent optimizer with various enhancements
 */
class GradientDescent : public OptimizationAlgorithm {
public:
    enum class Variant {
        Standard,       ///< Basic gradient descent
        Momentum,       ///< With momentum term
        Nesterov,       ///< Nesterov accelerated gradient
        Adam,           ///< Adaptive moment estimation
        AdaGrad,        ///< Adaptive gradient
        RMSprop         ///< Root mean square propagation
    };
    
    explicit GradientDescent(Variant variant = Variant::Adam);
    
    std::string getName() const override;
    std::string getDescription() const override;
    bool requiresGradient() const override { return true; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    /**
     * @brief Set learning rate
     */
    void setLearningRate(double lr) { m_learningRate = lr; }
    
    /**
     * @brief Set momentum coefficient (for momentum variants)
     */
    void setMomentum(double momentum) { m_momentum = momentum; }
    
    /**
     * @brief Set Adam parameters
     */
    void setAdamParams(double beta1 = 0.9, double beta2 = 0.999, double epsilon = 1e-8);
    
    /**
     * @brief Enable numerical gradient computation
     */
    void setNumericalGradient(bool enable, double stepSize = 1e-6) {
        m_useNumericalGradient = enable;
        m_gradientStepSize = stepSize;
    }
    
private:
    Variant m_variant;
    double m_learningRate{0.01};
    double m_momentum{0.9};
    double m_beta1{0.9};
    double m_beta2{0.999};
    double m_epsilon{1e-8};
    bool m_useNumericalGradient{true};
    double m_gradientStepSize{1e-6};
    
    ParameterVector computeNumericalGradient(CostFunction& cost,
                                             const ParameterVector& params,
                                             const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Nelder-Mead (Simplex) Method
// ============================================================================

/**
 * @brief Nelder-Mead Simplex optimization algorithm
 * 
 * Derivative-free method that maintains a simplex of n+1 points
 * and iteratively improves the worst point.
 */
class NelderMead : public OptimizationAlgorithm {
public:
    std::string getName() const override { return "Nelder-Mead"; }
    std::string getDescription() const override {
        return "Derivative-free simplex method. Good for low-dimensional "
               "problems (n < 10). Uses reflection, expansion, contraction, "
               "and shrinkage operations.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    /**
     * @brief Set simplex coefficients
     * @param alpha Reflection coefficient (default 1.0)
     * @param gamma Expansion coefficient (default 2.0)
     * @param rho Contraction coefficient (default 0.5)
     * @param sigma Shrinkage coefficient (default 0.5)
     */
    void setCoefficients(double alpha = 1.0, double gamma = 2.0,
                        double rho = 0.5, double sigma = 0.5);
    
    /**
     * @brief Set initial simplex size
     */
    void setInitialSimplexSize(double size) { m_initialSize = size; }
    
private:
    double m_alpha{1.0};   // Reflection
    double m_gamma{2.0};   // Expansion
    double m_rho{0.5};     // Contraction
    double m_sigma{0.5};   // Shrinkage
    double m_initialSize{0.1};
    
    std::vector<ParameterVector> initializeSimplex(
        const ParameterVector& x0,
        const std::vector<ParameterBounds>& bounds);
    
    ParameterVector centroid(const std::vector<ParameterVector>& simplex,
                            size_t excludeIndex);
};

// ============================================================================
// Genetic Algorithm
// ============================================================================

/**
 * @brief Genetic Algorithm for global optimization
 */
class GeneticAlgorithm : public OptimizationAlgorithm {
public:
    enum class SelectionMethod {
        Tournament,
        Roulette,
        Rank,
        Elitist
    };
    
    enum class CrossoverMethod {
        SinglePoint,
        TwoPoint,
        Uniform,
        Arithmetic,
        SBX  // Simulated Binary Crossover
    };
    
    enum class MutationMethod {
        Gaussian,
        Uniform,
        Polynomial
    };
    
    std::string getName() const override { return "Genetic Algorithm"; }
    std::string getDescription() const override {
        return "Population-based evolutionary algorithm using selection, "
               "crossover, and mutation. Good for global optimization with "
               "multiple local minima.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setPopulationSize(int size) { m_populationSize = size; }
    void setCrossoverRate(double rate) { m_crossoverRate = rate; }
    void setMutationRate(double rate) { m_mutationRate = rate; }
    void setEliteCount(int count) { m_eliteCount = count; }
    void setSelectionMethod(SelectionMethod method) { m_selection = method; }
    void setCrossoverMethod(CrossoverMethod method) { m_crossover = method; }
    void setMutationMethod(MutationMethod method) { m_mutation = method; }
    void setTournamentSize(int size) { m_tournamentSize = size; }
    void setMutationStrength(double strength) { m_mutationStrength = strength; }
    
private:
    int m_populationSize{50};
    double m_crossoverRate{0.8};
    double m_mutationRate{0.1};
    int m_eliteCount{2};
    int m_tournamentSize{3};
    double m_mutationStrength{0.1};
    
    SelectionMethod m_selection{SelectionMethod::Tournament};
    CrossoverMethod m_crossover{CrossoverMethod::SBX};
    MutationMethod m_mutation{MutationMethod::Polynomial};
    
    struct Individual {
        ParameterVector genes;
        double fitness{std::numeric_limits<double>::max()};
    };
    
    std::vector<Individual> initializePopulation(
        const ParameterVector& seed,
        const std::vector<ParameterBounds>& bounds);
    
    std::pair<int, int> selectParents(const std::vector<Individual>& population);
    Individual crossover(const Individual& parent1, const Individual& parent2,
                        const std::vector<ParameterBounds>& bounds);
    void mutate(Individual& individual, const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Particle Swarm Optimization
// ============================================================================

/**
 * @brief Particle Swarm Optimization (PSO)
 */
class ParticleSwarmOptimization : public OptimizationAlgorithm {
public:
    enum class Topology {
        Global,     ///< All particles are neighbors (gbest)
        Ring,       ///< Each particle has 2 neighbors (lbest)
        VonNeumann  ///< 2D grid topology
    };
    
    std::string getName() const override { return "Particle Swarm Optimization"; }
    std::string getDescription() const override {
        return "Swarm intelligence algorithm simulating social behavior. "
               "Each particle adjusts its trajectory based on personal and "
               "global best positions.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setSwarmSize(int size) { m_swarmSize = size; }
    void setInertiaWeight(double w) { m_inertia = w; }
    void setCognitiveCoeff(double c1) { m_cognitive = c1; }
    void setSocialCoeff(double c2) { m_social = c2; }
    void setVelocityClamp(double vmax) { m_velocityClamp = vmax; }
    void setTopology(Topology topology) { m_topology = topology; }
    void setAdaptiveInertia(bool adaptive, double wMin = 0.4, double wMax = 0.9) {
        m_adaptiveInertia = adaptive;
        m_inertiaMin = wMin;
        m_inertiaMax = wMax;
    }
    
private:
    int m_swarmSize{30};
    double m_inertia{0.729};      // Inertia weight
    double m_cognitive{1.49445};   // Personal best coefficient
    double m_social{1.49445};      // Global best coefficient
    double m_velocityClamp{0.5};
    Topology m_topology{Topology::Global};
    bool m_adaptiveInertia{true};
    double m_inertiaMin{0.4};
    double m_inertiaMax{0.9};
    
    struct Particle {
        ParameterVector position;
        ParameterVector velocity;
        ParameterVector bestPosition;
        double bestCost{std::numeric_limits<double>::max()};
    };
    
    std::vector<int> getNeighbors(int particleIndex, int swarmSize);
};

// ============================================================================
// Simulated Annealing
// ============================================================================

/**
 * @brief Simulated Annealing optimization
 */
class SimulatedAnnealing : public OptimizationAlgorithm {
public:
    enum class CoolingSchedule {
        Exponential,    ///< T(k) = T0 * alpha^k
        Linear,         ///< T(k) = T0 - k * delta
        Logarithmic,    ///< T(k) = T0 / log(1 + k)
        Adaptive        ///< Adjust based on acceptance rate
    };
    
    std::string getName() const override { return "Simulated Annealing"; }
    std::string getDescription() const override {
        return "Probabilistic optimization inspired by metallurgical annealing. "
               "Accepts worse solutions with decreasing probability, enabling "
               "escape from local minima.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setInitialTemperature(double T0) { m_initialTemp = T0; }
    void setFinalTemperature(double Tf) { m_finalTemp = Tf; }
    void setCoolingSchedule(CoolingSchedule schedule) { m_schedule = schedule; }
    void setCoolingRate(double alpha) { m_coolingRate = alpha; }
    void setIterationsPerTemp(int iters) { m_iterationsPerTemp = iters; }
    void setStepSize(double step) { m_stepSize = step; }
    void setAdaptiveStep(bool adaptive) { m_adaptiveStep = adaptive; }
    
private:
    double m_initialTemp{100.0};
    double m_finalTemp{1e-6};
    double m_coolingRate{0.95};
    int m_iterationsPerTemp{100};
    double m_stepSize{0.1};
    CoolingSchedule m_schedule{CoolingSchedule::Exponential};
    bool m_adaptiveStep{true};
    
    double cool(double temp, int iteration);
    ParameterVector generateNeighbor(const ParameterVector& current,
                                     const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Differential Evolution
// ============================================================================

/**
 * @brief Differential Evolution algorithm
 */
class DifferentialEvolution : public OptimizationAlgorithm {
public:
    enum class Strategy {
        Best1Bin,       ///< DE/best/1/bin
        Rand1Bin,       ///< DE/rand/1/bin
        RandToBest1Bin, ///< DE/rand-to-best/1/bin
        Best2Bin,       ///< DE/best/2/bin
        Rand2Bin,       ///< DE/rand/2/bin
        CurrentToPBest  ///< DE/current-to-pbest
    };
    
    std::string getName() const override { return "Differential Evolution"; }
    std::string getDescription() const override {
        return "Population-based stochastic optimizer using vector differences "
               "for mutation. Effective for continuous optimization with "
               "relatively few parameters.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setPopulationSize(int size) { m_populationSize = size; }
    void setMutationFactor(double F) { m_F = F; }
    void setCrossoverRate(double CR) { m_CR = CR; }
    void setStrategy(Strategy strategy) { m_strategy = strategy; }
    void setAdaptiveParameters(bool adaptive) { m_adaptive = adaptive; }
    
private:
    int m_populationSize{50};
    double m_F{0.8};      // Mutation factor
    double m_CR{0.9};     // Crossover rate
    Strategy m_strategy{Strategy::Best1Bin};
    bool m_adaptive{false};
    
    ParameterVector mutate(const std::vector<ParameterVector>& population,
                          int targetIndex, const ParameterVector& best,
                          const std::vector<ParameterBounds>& bounds);
    ParameterVector crossover(const ParameterVector& target,
                             const ParameterVector& mutant);
};

// ============================================================================
// Bayesian Optimization
// ============================================================================

/**
 * @brief Bayesian Optimization using Gaussian Process surrogate
 */
class BayesianOptimization : public OptimizationAlgorithm {
public:
    enum class AcquisitionFunction {
        ExpectedImprovement,    ///< EI
        ProbabilityOfImprovement, ///< PI
        LowerConfidenceBound,   ///< LCB/UCB
        ThompsonSampling        ///< TS
    };
    
    enum class KernelType {
        SquaredExponential,     ///< RBF kernel
        Matern32,               ///< Matern 3/2
        Matern52                ///< Matern 5/2
    };
    
    std::string getName() const override { return "Bayesian Optimization"; }
    std::string getDescription() const override {
        return "Model-based optimization using Gaussian Process surrogate. "
               "Efficient for expensive black-box functions. Balances "
               "exploration and exploitation via acquisition function.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setAcquisitionFunction(AcquisitionFunction acq) { m_acquisition = acq; }
    void setKernel(KernelType kernel) { m_kernel = kernel; }
    void setExplorationWeight(double kappa) { m_kappa = kappa; }
    void setInitialSamples(int n) { m_initialSamples = n; }
    void setLengthScale(double ls) { m_lengthScale = ls; }
    void setNoise(double noise) { m_noise = noise; }
    
private:
    AcquisitionFunction m_acquisition{AcquisitionFunction::ExpectedImprovement};
    KernelType m_kernel{KernelType::Matern52};
    double m_kappa{2.576};  // For LCB
    int m_initialSamples{5};
    double m_lengthScale{1.0};
    double m_noise{1e-6};
    
    // Observed data
    std::vector<ParameterVector> m_X;
    std::vector<double> m_y;
    
    // Gaussian Process methods
    double kernel(const ParameterVector& x1, const ParameterVector& x2);
    std::pair<double, double> predict(const ParameterVector& x);
    double acquisitionValue(const ParameterVector& x, double bestY);
    ParameterVector optimizeAcquisition(const std::vector<ParameterBounds>& bounds,
                                        double bestY);
};

// ============================================================================
// Ant Colony Optimization (for discrete optimization)
// ============================================================================

/**
 * @brief Ant Colony Optimization for Continuous Domains (ACO-R)
 */
class AntColonyOptimization : public OptimizationAlgorithm {
public:
    std::string getName() const override { return "Ant Colony Optimization"; }
    std::string getDescription() const override {
        return "Continuous domain ACO using Gaussian kernel probability "
               "density functions. Good for problems with multiple optima.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setArchiveSize(int k) { m_archiveSize = k; }
    void setAntsPerIteration(int m) { m_numAnts = m; }
    void setConvergenceSpeed(double q) { m_q = q; }
    void setLocalSearchIntensity(double xi) { m_xi = xi; }
    
private:
    int m_archiveSize{50};
    int m_numAnts{10};
    double m_q{0.5};   // Convergence speed
    double m_xi{0.85}; // Local search intensity
    
    struct Solution {
        ParameterVector params;
        double cost;
    };
    
    ParameterVector constructSolution(const std::vector<Solution>& archive,
                                      const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Grey Wolf Optimization
// ============================================================================

/**
 * @brief Grey Wolf Optimizer (GWO)
 */
class GreyWolfOptimizer : public OptimizationAlgorithm {
public:
    std::string getName() const override { return "Grey Wolf Optimizer"; }
    std::string getDescription() const override {
        return "Swarm intelligence algorithm mimicking grey wolf hunting. "
               "Uses alpha, beta, delta wolves to guide the search.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    // Configuration
    void setPackSize(int n) { m_packSize = n; }
    
private:
    int m_packSize{30};
    
    struct Wolf {
        ParameterVector position;
        double fitness{std::numeric_limits<double>::max()};
        
        Wolf& operator=(const Wolf&) = default;
    };
};

// ============================================================================
// Powell's Conjugate Direction Method
// ============================================================================

/**
 * @brief Powell's method (conjugate direction method)
 */
class PowellMethod : public OptimizationAlgorithm {
public:
    std::string getName() const override { return "Powell's Method"; }
    std::string getDescription() const override {
        return "Derivative-free method using conjugate directions. "
               "Performs line searches along each direction and updates "
               "search directions.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
private:
    double lineSearch(CostFunction& cost, const ParameterVector& x,
                     const ParameterVector& direction,
                     const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// BFGS Quasi-Newton Method
// ============================================================================

/**
 * @brief BFGS Quasi-Newton optimization
 */
class BFGSOptimizer : public OptimizationAlgorithm {
public:
    std::string getName() const override { return "BFGS"; }
    std::string getDescription() const override {
        return "Quasi-Newton method that approximates the Hessian matrix. "
               "Superlinear convergence near minimum. Uses numerical gradient.";
    }
    bool requiresGradient() const override { return true; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
    void setGradientStepSize(double h) { m_gradStep = h; }
    
private:
    double m_gradStep{1e-6};
    
    ParameterVector computeGradient(CostFunction& cost, const ParameterVector& x);
    double lineSearch(CostFunction& cost, const ParameterVector& x,
                     const ParameterVector& direction,
                     const std::vector<ParameterBounds>& bounds);
};

// ============================================================================
// Multi-Start Optimizer
// ============================================================================

/**
 * @brief Multi-start wrapper for local optimizers
 */
class MultiStartOptimizer : public OptimizationAlgorithm {
public:
    explicit MultiStartOptimizer(std::shared_ptr<OptimizationAlgorithm> localOptimizer,
                                 int numStarts = 10);
    
    std::string getName() const override { 
        return "Multi-Start " + m_localOptimizer->getName(); 
    }
    std::string getDescription() const override {
        return "Runs local optimizer from multiple random starting points "
               "to increase chance of finding global minimum.";
    }
    bool requiresGradient() const override { 
        return m_localOptimizer->requiresGradient(); 
    }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
private:
    std::shared_ptr<OptimizationAlgorithm> m_localOptimizer;
    int m_numStarts;
};

// ============================================================================
// Hybrid Optimizer
// ============================================================================

/**
 * @brief Hybrid optimizer combining global and local search
 */
class HybridOptimizer : public OptimizationAlgorithm {
public:
    /**
     * @brief Construct hybrid optimizer
     * @param globalOptimizer Global optimizer (e.g., GA, PSO)
     * @param localOptimizer Local optimizer (e.g., Nelder-Mead, BFGS)
     * @param switchThreshold When to switch to local (cost improvement threshold)
     */
    HybridOptimizer(std::shared_ptr<OptimizationAlgorithm> globalOptimizer,
                    std::shared_ptr<OptimizationAlgorithm> localOptimizer,
                    double switchThreshold = 0.01);
    
    std::string getName() const override { return "Hybrid Optimizer"; }
    std::string getDescription() const override {
        return "Combines global search with local refinement for effective "
               "optimization of multi-modal landscapes.";
    }
    bool requiresGradient() const override { return false; }
    
    OptimizationResult optimize(
        CostFunction& costFunction,
        const ParameterVector& initialParams,
        const std::vector<ParameterBounds>& bounds) override;
    
private:
    std::shared_ptr<OptimizationAlgorithm> m_globalOptimizer;
    std::shared_ptr<OptimizationAlgorithm> m_localOptimizer;
    double m_switchThreshold;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create optimizer by name
 */
std::shared_ptr<OptimizationAlgorithm> createOptimizer(const std::string& name);

/**
 * @brief Get list of available optimizer names
 */
std::vector<std::string> getAvailableOptimizers();

} // namespace Autotuning
} // namespace tether::control
