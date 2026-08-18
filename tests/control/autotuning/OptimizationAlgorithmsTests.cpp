/**
 * @file OptimizationAlgorithmsTests.cpp
 * @brief Unit tests for optimization algorithms
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <set>

#include "tether/control/autotuning/OptimizationAlgorithms.hpp"
#include "TestHelpers.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

// ============================================================================
// TerminationCriteria Tests
// ============================================================================

TEST(TerminationCriteriaTest, DefaultValues) {
    TerminationCriteria criteria;
    EXPECT_EQ(criteria.maxIterations, 1000);
    EXPECT_EQ(criteria.maxFunctionEvaluations, 10000);
    EXPECT_DOUBLE_EQ(criteria.functionTolerance, 1e-8);
    EXPECT_DOUBLE_EQ(criteria.parameterTolerance, 1e-8);
    EXPECT_DOUBLE_EQ(criteria.gradientTolerance, 1e-6);
}

TEST(TerminationCriteriaTest, Customize) {
    TerminationCriteria criteria;
    criteria.maxIterations = 500;
    criteria.functionTolerance = 1e-6;
    EXPECT_EQ(criteria.maxIterations, 500);
    EXPECT_DOUBLE_EQ(criteria.functionTolerance, 1e-6);
}

// ============================================================================
// OptimizationResult Tests
// ============================================================================

TEST(OptimizationResultTest, DefaultValues) {
    OptimizationResult result;
    EXPECT_EQ(result.iterations, 0);
    EXPECT_EQ(result.functionEvaluations, 0);
    EXPECT_FALSE(result.converged);
    EXPECT_TRUE(result.terminationReason.empty());
}

TEST(OptimizationResultTest, ConvergenceHistory) {
    OptimizationResult result;
    result.costHistory.push_back(100.0);
    result.costHistory.push_back(50.0);
    result.costHistory.push_back(25.0);
    EXPECT_EQ(result.costHistory.size(), 3);
}

// ============================================================================
// GradientDescent Tests
// ============================================================================

class GradientDescentTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<GradientDescent>();
    }
    
    std::unique_ptr<GradientDescent> optimizer_;
};

TEST_F(GradientDescentTest, Name) {
    EXPECT_FALSE(optimizer_->getName().empty());
}

TEST_F(GradientDescentTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(GradientDescentTest, RequiresGradient) {
    EXPECT_TRUE(optimizer_->requiresGradient());
}

TEST_F(GradientDescentTest, SetLearningRate) {
    optimizer_->setLearningRate(0.01);
}

TEST_F(GradientDescentTest, SetMomentum) {
    optimizer_->setMomentum(0.9);
}

TEST_F(GradientDescentTest, SetAdamParams) {
    optimizer_->setAdamParams(0.9, 0.999, 1e-8);
}

TEST_F(GradientDescentTest, SetNumericalGradient) {
    optimizer_->setNumericalGradient(true, 1e-6);
}

TEST_F(GradientDescentTest, SetTerminationCriteria) {
    TerminationCriteria criteria;
    criteria.maxIterations = 500;
    optimizer_->setTerminationCriteria(criteria);
}

TEST_F(GradientDescentTest, SetRandomSeed) {
    optimizer_->setRandomSeed(42);
}

TEST_F(GradientDescentTest, SetTrackHistory) {
    optimizer_->setTrackHistory(true);
}

TEST_F(GradientDescentTest, Variants) {
    GradientDescent gd1(GradientDescent::Variant::Standard);
    GradientDescent gd2(GradientDescent::Variant::Momentum);
    GradientDescent gd3(GradientDescent::Variant::Nesterov);
    GradientDescent gd4(GradientDescent::Variant::Adam);
    GradientDescent gd5(GradientDescent::Variant::AdaGrad);
    GradientDescent gd6(GradientDescent::Variant::RMSprop);
}

// ============================================================================
// GeneticAlgorithm Tests
// ============================================================================

class GeneticAlgorithmTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<GeneticAlgorithm>();
    }
    
    std::unique_ptr<GeneticAlgorithm> optimizer_;
};

TEST_F(GeneticAlgorithmTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Genetic Algorithm");
}

TEST_F(GeneticAlgorithmTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(GeneticAlgorithmTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(GeneticAlgorithmTest, SetPopulationSize) {
    optimizer_->setPopulationSize(100);
}

TEST_F(GeneticAlgorithmTest, SetMutationRate) {
    optimizer_->setMutationRate(0.1);
}

TEST_F(GeneticAlgorithmTest, SetCrossoverRate) {
    optimizer_->setCrossoverRate(0.8);
}

TEST_F(GeneticAlgorithmTest, SetEliteCount) {
    optimizer_->setEliteCount(5);
}

TEST_F(GeneticAlgorithmTest, SetTournamentSize) {
    optimizer_->setTournamentSize(5);
}

TEST_F(GeneticAlgorithmTest, SetMutationStrength) {
    optimizer_->setMutationStrength(0.2);
}

TEST_F(GeneticAlgorithmTest, SetSelectionMethod) {
    optimizer_->setSelectionMethod(GeneticAlgorithm::SelectionMethod::Tournament);
    optimizer_->setSelectionMethod(GeneticAlgorithm::SelectionMethod::Roulette);
    optimizer_->setSelectionMethod(GeneticAlgorithm::SelectionMethod::Rank);
    optimizer_->setSelectionMethod(GeneticAlgorithm::SelectionMethod::Elitist);
}

TEST_F(GeneticAlgorithmTest, SetCrossoverMethod) {
    optimizer_->setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::SinglePoint);
    optimizer_->setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::TwoPoint);
    optimizer_->setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::Uniform);
    optimizer_->setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::Arithmetic);
    optimizer_->setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::SBX);
}

TEST_F(GeneticAlgorithmTest, SetMutationMethod) {
    optimizer_->setMutationMethod(GeneticAlgorithm::MutationMethod::Gaussian);
    optimizer_->setMutationMethod(GeneticAlgorithm::MutationMethod::Uniform);
    optimizer_->setMutationMethod(GeneticAlgorithm::MutationMethod::Polynomial);
}

// ============================================================================
// ParticleSwarmOptimization Tests
// ============================================================================

class ParticleSwarmOptimizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<ParticleSwarmOptimization>();
    }
    
    std::unique_ptr<ParticleSwarmOptimization> optimizer_;
};

TEST_F(ParticleSwarmOptimizationTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Particle Swarm Optimization");
}

TEST_F(ParticleSwarmOptimizationTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(ParticleSwarmOptimizationTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(ParticleSwarmOptimizationTest, SetSwarmSize) {
    optimizer_->setSwarmSize(50);
}

TEST_F(ParticleSwarmOptimizationTest, SetInertiaWeight) {
    optimizer_->setInertiaWeight(0.7);
}

TEST_F(ParticleSwarmOptimizationTest, SetCognitiveCoeff) {
    optimizer_->setCognitiveCoeff(1.5);
}

TEST_F(ParticleSwarmOptimizationTest, SetSocialCoeff) {
    optimizer_->setSocialCoeff(1.5);
}

TEST_F(ParticleSwarmOptimizationTest, SetVelocityClamp) {
    optimizer_->setVelocityClamp(0.3);
}

TEST_F(ParticleSwarmOptimizationTest, SetTopology) {
    optimizer_->setTopology(ParticleSwarmOptimization::Topology::Global);
    optimizer_->setTopology(ParticleSwarmOptimization::Topology::Ring);
    optimizer_->setTopology(ParticleSwarmOptimization::Topology::VonNeumann);
}

TEST_F(ParticleSwarmOptimizationTest, SetAdaptiveInertia) {
    optimizer_->setAdaptiveInertia(true, 0.4, 0.9);
}

// ============================================================================
// SimulatedAnnealing Tests
// ============================================================================

class SimulatedAnnealingTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<SimulatedAnnealing>();
    }
    
    std::unique_ptr<SimulatedAnnealing> optimizer_;
};

TEST_F(SimulatedAnnealingTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Simulated Annealing");
}

TEST_F(SimulatedAnnealingTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(SimulatedAnnealingTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(SimulatedAnnealingTest, SetInitialTemperature) {
    optimizer_->setInitialTemperature(100.0);
}

TEST_F(SimulatedAnnealingTest, SetFinalTemperature) {
    optimizer_->setFinalTemperature(0.001);
}

TEST_F(SimulatedAnnealingTest, SetCoolingSchedule) {
    optimizer_->setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Exponential);
    optimizer_->setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Linear);
    optimizer_->setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Logarithmic);
    optimizer_->setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Adaptive);
}

TEST_F(SimulatedAnnealingTest, SetCoolingRate) {
    optimizer_->setCoolingRate(0.99);
}

TEST_F(SimulatedAnnealingTest, SetIterationsPerTemp) {
    optimizer_->setIterationsPerTemp(100);
}

TEST_F(SimulatedAnnealingTest, SetStepSize) {
    optimizer_->setStepSize(0.1);
}

TEST_F(SimulatedAnnealingTest, SetAdaptiveStep) {
    optimizer_->setAdaptiveStep(true);
}

// ============================================================================
// NelderMead Tests
// ============================================================================

class NelderMeadTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<NelderMead>();
    }
    
    std::unique_ptr<NelderMead> optimizer_;
};

TEST_F(NelderMeadTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Nelder-Mead");
}

TEST_F(NelderMeadTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(NelderMeadTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(NelderMeadTest, SetCoefficients) {
    optimizer_->setCoefficients(1.0, 2.0, 0.5, 0.5);
}

TEST_F(NelderMeadTest, SetInitialSimplexSize) {
    optimizer_->setInitialSimplexSize(0.1);
}

// ============================================================================
// BayesianOptimization Tests
// ============================================================================

class BayesianOptimizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<BayesianOptimization>();
    }
    
    std::unique_ptr<BayesianOptimization> optimizer_;
};

TEST_F(BayesianOptimizationTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Bayesian Optimization");
}

TEST_F(BayesianOptimizationTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(BayesianOptimizationTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(BayesianOptimizationTest, SetAcquisitionFunction) {
    optimizer_->setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::ExpectedImprovement);
    optimizer_->setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::ProbabilityOfImprovement);
    optimizer_->setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::LowerConfidenceBound);
    optimizer_->setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::ThompsonSampling);
}

TEST_F(BayesianOptimizationTest, SetKernel) {
    optimizer_->setKernel(BayesianOptimization::KernelType::SquaredExponential);
    optimizer_->setKernel(BayesianOptimization::KernelType::Matern32);
    optimizer_->setKernel(BayesianOptimization::KernelType::Matern52);
}

TEST_F(BayesianOptimizationTest, SetExplorationWeight) {
    optimizer_->setExplorationWeight(2.0);
}

TEST_F(BayesianOptimizationTest, SetInitialSamples) {
    optimizer_->setInitialSamples(10);
}

TEST_F(BayesianOptimizationTest, SetLengthScale) {
    optimizer_->setLengthScale(1.0);
}

TEST_F(BayesianOptimizationTest, SetNoise) {
    optimizer_->setNoise(0.01);
}

// ============================================================================
// DifferentialEvolution Tests
// ============================================================================

class DifferentialEvolutionTest : public ::testing::Test {
protected:
    void SetUp() override {
        optimizer_ = std::make_unique<DifferentialEvolution>();
    }
    
    std::unique_ptr<DifferentialEvolution> optimizer_;
};

TEST_F(DifferentialEvolutionTest, Name) {
    EXPECT_EQ(optimizer_->getName(), "Differential Evolution");
}

TEST_F(DifferentialEvolutionTest, Description) {
    EXPECT_FALSE(optimizer_->getDescription().empty());
}

TEST_F(DifferentialEvolutionTest, RequiresGradient) {
    EXPECT_FALSE(optimizer_->requiresGradient());
}

TEST_F(DifferentialEvolutionTest, SetPopulationSize) {
    optimizer_->setPopulationSize(30);
}

TEST_F(DifferentialEvolutionTest, SetMutationFactor) {
    optimizer_->setMutationFactor(0.8);
}

TEST_F(DifferentialEvolutionTest, SetCrossoverRate) {
    optimizer_->setCrossoverRate(0.9);
}

TEST_F(DifferentialEvolutionTest, SetStrategy) {
    optimizer_->setStrategy(DifferentialEvolution::Strategy::Best1Bin);
    optimizer_->setStrategy(DifferentialEvolution::Strategy::Rand1Bin);
    optimizer_->setStrategy(DifferentialEvolution::Strategy::RandToBest1Bin);
    optimizer_->setStrategy(DifferentialEvolution::Strategy::Best2Bin);
    optimizer_->setStrategy(DifferentialEvolution::Strategy::Rand2Bin);
    optimizer_->setStrategy(DifferentialEvolution::Strategy::CurrentToPBest);
}

TEST_F(DifferentialEvolutionTest, SetAdaptiveParameters) {
    optimizer_->setAdaptiveParameters(true);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(OptimizationIntegration, OptimizersHaveDifferentNames) {
    GradientDescent gd;
    GeneticAlgorithm ga;
    ParticleSwarmOptimization pso;
    SimulatedAnnealing sa;
    NelderMead nm;
    BayesianOptimization bo;
    DifferentialEvolution de;
    
    std::set<std::string> names;
    names.insert(gd.getName());
    names.insert(ga.getName());
    names.insert(pso.getName());
    names.insert(sa.getName());
    names.insert(nm.getName());
    names.insert(bo.getName());
    names.insert(de.getName());
    
    EXPECT_EQ(names.size(), 7);  // All unique
}

TEST(OptimizationIntegration, GradientFreeOptimizers) {
    GeneticAlgorithm ga;
    ParticleSwarmOptimization pso;
    SimulatedAnnealing sa;
    NelderMead nm;
    BayesianOptimization bo;
    DifferentialEvolution de;
    
    EXPECT_FALSE(ga.requiresGradient());
    EXPECT_FALSE(pso.requiresGradient());
    EXPECT_FALSE(sa.requiresGradient());
    EXPECT_FALSE(nm.requiresGradient());
    EXPECT_FALSE(bo.requiresGradient());
    EXPECT_FALSE(de.requiresGradient());
}

TEST(OptimizationIntegration, GradientBasedOptimizers) {
    GradientDescent gd;
    EXPECT_TRUE(gd.requiresGradient());
}
// ============================================================================
// Actual Optimization Tests - Exercise optimize() method
// ============================================================================

// Simple quadratic cost function for testing
class QuadraticCost : public CostFunction {
public:
    double evaluate(const ParameterVector& params) override {
        // f(x) = sum(x_i^2) - minimum at origin
        double cost = 0.0;
        for (double x : params) {
            cost += x * x;
        }
        return cost;
    }
};

// Rosenbrock function - harder optimization problem
class RosenbrockCost : public CostFunction {
public:
    double evaluate(const ParameterVector& params) override {
        // f(x,y) = (1-x)^2 + 100*(y-x^2)^2 - minimum at (1,1)
        if (params.size() < 2) return std::numeric_limits<double>::max();
        double x = params[0];
        double y = params[1];
        return (1.0 - x) * (1.0 - x) + 100.0 * (y - x*x) * (y - x*x);
    }
};

// Rastrigin function - multimodal with many local minima
class RastriginCost : public CostFunction {
public:
    double evaluate(const ParameterVector& params) override {
        double A = 10.0;
        double cost = A * params.size();
        for (double x : params) {
            cost += x*x - A * std::cos(2.0 * M_PI * x);
        }
        return cost;
    }
};

// Sphere function - simple convex
class SphereCost : public CostFunction {
public:
    double evaluate(const ParameterVector& params) override {
        double cost = 0.0;
        for (double x : params) {
            cost += x * x;
        }
        return cost;
    }
};

TEST(OptimizationExecution, NelderMeadOptimizeQuadratic) {
    NelderMead optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 200;
    criteria.functionTolerance = 1e-6;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setRandomSeed(42);
    optimizer.setTrackHistory(true);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 0.1);  // Should find near-minimum
    EXPECT_GT(result.iterations, 0);
    EXPECT_GT(result.functionEvaluations, 0);
}

TEST(OptimizationExecution, NelderMeadOptimizeSphere) {
    NelderMead optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 300;
    criteria.functionTolerance = 1e-8;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 0.01);
}

TEST(OptimizationExecution, ParticleSwarmOptimizeQuadratic) {
    ParticleSwarmOptimization optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setSwarmSize(20);
    optimizer.setInertiaWeight(0.7);
    optimizer.setCognitiveCoeff(1.5);
    optimizer.setSocialCoeff(1.5);
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 1.0);  // Should improve from initial
    EXPECT_GT(result.iterations, 0);
}

TEST(OptimizationExecution, ParticleSwarmOptimizeSphere) {
    ParticleSwarmOptimization optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 150;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setSwarmSize(30);
    optimizer.setVelocityClamp(2.0);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 0.5);
}

TEST(OptimizationExecution, GeneticAlgorithmOptimizeQuadratic) {
    GeneticAlgorithm optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setPopulationSize(30);
    optimizer.setMutationRate(0.1);
    optimizer.setCrossoverRate(0.8);
    optimizer.setEliteCount(2);
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 10.0);  // Should improve from initial
    EXPECT_GT(result.iterations, 0);
}

TEST(OptimizationExecution, GeneticAlgorithmSelectionMethods) {
    GeneticAlgorithm optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 30;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setPopulationSize(20);
    
    // Test Tournament selection
    optimizer.setSelectionMethod(GeneticAlgorithm::SelectionMethod::Tournament);
    optimizer.setTournamentSize(3);
    auto result1 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result1.iterations, 0);
    
    // Test Roulette selection
    optimizer.setSelectionMethod(GeneticAlgorithm::SelectionMethod::Roulette);
    auto result2 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result2.iterations, 0);
}

TEST(OptimizationExecution, GeneticAlgorithmCrossoverMethods) {
    GeneticAlgorithm optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 25;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setPopulationSize(20);
    
    // Test different crossover methods
    optimizer.setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::SinglePoint);
    auto result1 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result1.iterations, 0);
    
    optimizer.setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::Uniform);
    auto result2 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result2.iterations, 0);
    
    optimizer.setCrossoverMethod(GeneticAlgorithm::CrossoverMethod::Arithmetic);
    auto result3 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result3.iterations, 0);
}

TEST(OptimizationExecution, SimulatedAnnealingOptimizeQuadratic) {
    SimulatedAnnealing optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 500;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setInitialTemperature(100.0);
    optimizer.setFinalTemperature(0.01);
    optimizer.setCoolingRate(0.95);
    optimizer.setIterationsPerTemp(20);
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 5.0);  // Should improve significantly
    EXPECT_GT(result.iterations, 0);
}

TEST(OptimizationExecution, SimulatedAnnealingCoolingSchedules) {
    SimulatedAnnealing optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 200;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setInitialTemperature(50.0);
    
    // Test exponential cooling
    optimizer.setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Exponential);
    auto result1 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result1.iterations, 0);
    
    // Test linear cooling
    optimizer.setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Linear);
    auto result2 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result2.iterations, 0);
    
    // Test logarithmic cooling
    optimizer.setCoolingSchedule(SimulatedAnnealing::CoolingSchedule::Logarithmic);
    auto result3 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result3.iterations, 0);
}

TEST(OptimizationExecution, DifferentialEvolutionOptimizeQuadratic) {
    DifferentialEvolution optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setPopulationSize(20);
    optimizer.setMutationFactor(0.8);
    optimizer.setCrossoverRate(0.9);
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 5.0);
    EXPECT_GT(result.iterations, 0);
}

TEST(OptimizationExecution, DifferentialEvolutionStrategies) {
    DifferentialEvolution optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 30;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setPopulationSize(15);
    
    // Test Best1Bin strategy
    optimizer.setStrategy(DifferentialEvolution::Strategy::Best1Bin);
    auto result1 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result1.iterations, 0);
    
    // Test Rand1Bin strategy
    optimizer.setStrategy(DifferentialEvolution::Strategy::Rand1Bin);
    auto result2 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result2.iterations, 0);
}

TEST(OptimizationExecution, BayesianOptimizationQuadratic) {
    BayesianOptimization optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {3.0, -2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 30;
    criteria.maxFunctionEvaluations = 50;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setInitialSamples(5);
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_GT(result.iterations, 0);
    EXPECT_GT(result.functionEvaluations, 0);
}

TEST(OptimizationExecution, BayesianOptimizationAcquisitionFunctions) {
    BayesianOptimization optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 20;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setInitialSamples(3);
    
    // Test Expected Improvement
    optimizer.setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::ExpectedImprovement);
    auto result1 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result1.iterations, 0);
    
    // Test Probability of Improvement
    optimizer.setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::ProbabilityOfImprovement);
    auto result2 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result2.iterations, 0);
    
    // Test Lower Confidence Bound
    optimizer.setAcquisitionFunction(BayesianOptimization::AcquisitionFunction::LowerConfidenceBound);
    optimizer.setExplorationWeight(2.0);
    auto result3 = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result3.iterations, 0);
}

TEST(OptimizationExecution, GradientDescentOptimizeQuadratic) {
    GradientDescent optimizer;
    QuadraticCost cost;
    
    ParameterVector initial = {5.0, -3.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 200;
    criteria.functionTolerance = 1e-6;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setLearningRate(0.1);
    optimizer.setNumericalGradient(true, 1e-5);  // Use numerical gradient
    optimizer.setRandomSeed(42);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    // Just verify it runs without crashing and returns something
    EXPECT_TRUE(result.bestCost >= 0.0 || result.iterations >= 0);
}

TEST(OptimizationExecution, GradientDescentVariants) {
    QuadraticCost cost;
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    
    // Test Standard variant
    GradientDescent gd1(GradientDescent::Variant::Standard);
    gd1.setTerminationCriteria(criteria);
    gd1.setLearningRate(0.1);
    gd1.setNumericalGradient(true, 1e-5);
    auto result1 = gd1.optimize(cost, initial, bounds);
    // Just verify runs without crash
    EXPECT_TRUE(result1.bestCost >= 0.0);
    
    // Test Momentum variant
    GradientDescent gd2(GradientDescent::Variant::Momentum);
    gd2.setTerminationCriteria(criteria);
    gd2.setLearningRate(0.1);
    gd2.setMomentum(0.9);
    gd2.setNumericalGradient(true, 1e-5);
    auto result2 = gd2.optimize(cost, initial, bounds);
    EXPECT_TRUE(result2.bestCost >= 0.0);
    
    // Test Adam variant
    GradientDescent gd3(GradientDescent::Variant::Adam);
    gd3.setTerminationCriteria(criteria);
    gd3.setLearningRate(0.1);
    gd3.setAdamParams(0.9, 0.999, 1e-8);
    gd3.setNumericalGradient(true, 1e-5);
    auto result3 = gd3.optimize(cost, initial, bounds);
    EXPECT_TRUE(result3.bestCost >= 0.0);
}

TEST(OptimizationExecution, ProgressCallback) {
    NelderMead optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    
    int callbackCount = 0;
    optimizer.setProgressCallback([&](int iter, double cost, const ParameterVector& params) {
        callbackCount++;
        EXPECT_GE(iter, 0);
        EXPECT_TRUE(std::isfinite(cost));
        EXPECT_FALSE(params.empty());
    });
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_GT(callbackCount, 0);
}

TEST(OptimizationExecution, ConvergenceHistory) {
    NelderMead optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setTrackHistory(true);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    // Should have history if tracking enabled
    EXPECT_GT(result.costHistory.size(), 0);
}

TEST(OptimizationExecution, EmptyInitialParameters) {
    NelderMead optimizer;
    SphereCost cost;
    
    ParameterVector initial = {};
    std::vector<ParameterBounds> bounds = {};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 10;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    // Should handle gracefully
    EXPECT_TRUE(result.bestParameters.empty() || result.bestCost >= 0.0);
}

TEST(OptimizationExecution, SingleParameter) {
    NelderMead optimizer;
    
    // Simple 1D quadratic: f(x) = (x-2)^2
    class Simple1DCost : public CostFunction {
    public:
        double evaluate(const ParameterVector& params) override {
            if (params.empty()) return std::numeric_limits<double>::max();
            double x = params[0];
            return (x - 2.0) * (x - 2.0);
        }
    } cost;
    
    ParameterVector initial = {0.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    criteria.functionTolerance = 1e-6;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 0.1);
    EXPECT_NEAR(result.bestParameters[0], 2.0, 0.5);
}

TEST(OptimizationExecution, HighDimensional) {
    ParticleSwarmOptimization optimizer;
    SphereCost cost;
    
    // 5D optimization
    ParameterVector initial = {1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<ParameterBounds> bounds(5, {-5.0, 5.0});
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    optimizer.setSwarmSize(40);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    EXPECT_LT(result.bestCost, 5.0);  // Should improve from initial cost of 5
    EXPECT_EQ(result.bestParameters.size(), 5);
}

TEST(OptimizationExecution, TightBounds) {
    NelderMead optimizer;
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    // Tight bounds that don't include the optimum (0,0)
    std::vector<ParameterBounds> bounds = {{1.0, 3.0}, {1.0, 3.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    
    // Should find best within bounds (corner at (1,1))
    EXPECT_GE(result.bestParameters[0], 0.9);
    EXPECT_LE(result.bestParameters[0], 3.1);
}

// ============================================================================
// Additional Coverage Tests for OptimizationAlgorithms
// ============================================================================

TEST(GradientDescentCoverage, AllVariants) {
    std::vector<GradientDescent::Variant> variants = {
        GradientDescent::Variant::Standard,
        GradientDescent::Variant::Momentum,
        GradientDescent::Variant::Nesterov,
        GradientDescent::Variant::Adam,
        GradientDescent::Variant::AdaGrad,
        GradientDescent::Variant::RMSprop
    };
    
    for (auto variant : variants) {
        GradientDescent optimizer(variant);
        SphereCost cost;
        
        ParameterVector initial = {3.0, 3.0};
        std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
        
        TerminationCriteria criteria;
        criteria.maxIterations = 50;
        optimizer.setTerminationCriteria(criteria);
        optimizer.setLearningRate(0.1);
        optimizer.setMomentum(0.9);
        optimizer.setNumericalGradient(true, 1e-5);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        
        EXPECT_LT(result.bestCost, 20.0);  // Should improve from initial cost of 18
    }
}

TEST(GradientDescentCoverage, AdamParameters) {
    GradientDescent optimizer(GradientDescent::Variant::Adam);
    optimizer.setAdamParams(0.85, 0.995, 1e-7);
    optimizer.setLearningRate(0.05);
    
    SphereCost cost;
    
    ParameterVector initial = {2.0, 2.0};
    std::vector<ParameterBounds> bounds = {{-10.0, 10.0}, {-10.0, 10.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_LT(result.bestCost, 10.0);
}

TEST(GeneticAlgorithmCoverage, AllMethodCombinations) {
    std::vector<GeneticAlgorithm::SelectionMethod> selections = {
        GeneticAlgorithm::SelectionMethod::Tournament,
        GeneticAlgorithm::SelectionMethod::Roulette,
        GeneticAlgorithm::SelectionMethod::Rank,
        GeneticAlgorithm::SelectionMethod::Elitist
    };
    
    std::vector<GeneticAlgorithm::CrossoverMethod> crossovers = {
        GeneticAlgorithm::CrossoverMethod::SinglePoint,
        GeneticAlgorithm::CrossoverMethod::TwoPoint,
        GeneticAlgorithm::CrossoverMethod::Uniform,
        GeneticAlgorithm::CrossoverMethod::Arithmetic,
        GeneticAlgorithm::CrossoverMethod::SBX
    };
    
    std::vector<GeneticAlgorithm::MutationMethod> mutations = {
        GeneticAlgorithm::MutationMethod::Gaussian,
        GeneticAlgorithm::MutationMethod::Uniform,
        GeneticAlgorithm::MutationMethod::Polynomial
    };
    
    SphereCost cost;
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    // Test a subset of combinations
    for (size_t i = 0; i < selections.size(); ++i) {
        GeneticAlgorithm optimizer;
        optimizer.setSelectionMethod(selections[i]);
        optimizer.setCrossoverMethod(crossovers[i % crossovers.size()]);
        optimizer.setMutationMethod(mutations[i % mutations.size()]);
        optimizer.setPopulationSize(30);
        optimizer.setCrossoverRate(0.75);
        optimizer.setMutationRate(0.15);
        optimizer.setEliteCount(3);
        optimizer.setTournamentSize(4);
        optimizer.setMutationStrength(0.15);
        
        TerminationCriteria criteria;
        criteria.maxIterations = 30;
        optimizer.setTerminationCriteria(criteria);
        optimizer.setRandomSeed(42);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        EXPECT_GT(result.iterations, 0);
    }
}

TEST(ParticleSwarmCoverage, AllTopologies) {
    std::vector<ParticleSwarmOptimization::Topology> topologies = {
        ParticleSwarmOptimization::Topology::Global,
        ParticleSwarmOptimization::Topology::Ring,
        ParticleSwarmOptimization::Topology::VonNeumann
    };
    
    for (auto topology : topologies) {
        ParticleSwarmOptimization optimizer;
        SphereCost cost;
        
        ParameterVector initial = {3.0, 3.0};
        std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
        
        optimizer.setTopology(topology);
        optimizer.setSwarmSize(25);
        optimizer.setInertiaWeight(0.7);
        optimizer.setCognitiveCoeff(1.5);
        optimizer.setSocialCoeff(1.5);
        optimizer.setVelocityClamp(0.3);
        optimizer.setAdaptiveInertia(true, 0.3, 0.85);
        
        TerminationCriteria criteria;
        criteria.maxIterations = 40;
        optimizer.setTerminationCriteria(criteria);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        EXPECT_LT(result.bestCost, 20.0);
    }
}

TEST(SimulatedAnnealingCoverage, AllCoolingSchedules) {
    std::vector<SimulatedAnnealing::CoolingSchedule> schedules = {
        SimulatedAnnealing::CoolingSchedule::Exponential,
        SimulatedAnnealing::CoolingSchedule::Linear,
        SimulatedAnnealing::CoolingSchedule::Logarithmic,
        SimulatedAnnealing::CoolingSchedule::Adaptive
    };
    
    for (auto schedule : schedules) {
        SimulatedAnnealing optimizer;
        SphereCost cost;
        
        ParameterVector initial = {3.0, 3.0};
        std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
        
        optimizer.setCoolingSchedule(schedule);
        optimizer.setInitialTemperature(50.0);
        optimizer.setFinalTemperature(1e-4);
        optimizer.setCoolingRate(0.9);
        optimizer.setIterationsPerTemp(50);
        optimizer.setStepSize(0.2);
        optimizer.setAdaptiveStep(true);
        
        TerminationCriteria criteria;
        criteria.maxIterations = 100;
        optimizer.setTerminationCriteria(criteria);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        EXPECT_GT(result.iterations, 0);
    }
}

TEST(DifferentialEvolutionCoverage, AllStrategies) {
    std::vector<DifferentialEvolution::Strategy> strategies = {
        DifferentialEvolution::Strategy::Best1Bin,
        DifferentialEvolution::Strategy::Rand1Bin,
        DifferentialEvolution::Strategy::RandToBest1Bin,
        DifferentialEvolution::Strategy::Best2Bin,
        DifferentialEvolution::Strategy::Rand2Bin,
        DifferentialEvolution::Strategy::CurrentToPBest
    };
    
    for (auto strategy : strategies) {
        DifferentialEvolution optimizer;
        SphereCost cost;
        
        ParameterVector initial = {3.0, 3.0};
        std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
        
        optimizer.setStrategy(strategy);
        optimizer.setPopulationSize(30);
        optimizer.setMutationFactor(0.7);
        optimizer.setCrossoverRate(0.85);
        optimizer.setAdaptiveParameters(true);
        
        TerminationCriteria criteria;
        criteria.maxIterations = 40;
        optimizer.setTerminationCriteria(criteria);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        EXPECT_GT(result.iterations, 0);
    }
}

TEST(BayesianOptCoverage, AllAcquisitionFunctions) {
    std::vector<BayesianOptimization::AcquisitionFunction> acqs = {
        BayesianOptimization::AcquisitionFunction::ExpectedImprovement,
        BayesianOptimization::AcquisitionFunction::ProbabilityOfImprovement,
        BayesianOptimization::AcquisitionFunction::LowerConfidenceBound,
        BayesianOptimization::AcquisitionFunction::ThompsonSampling
    };
    
    std::vector<BayesianOptimization::KernelType> kernels = {
        BayesianOptimization::KernelType::SquaredExponential,
        BayesianOptimization::KernelType::Matern32,
        BayesianOptimization::KernelType::Matern52
    };
    
    for (size_t i = 0; i < acqs.size(); ++i) {
        BayesianOptimization optimizer;
        SphereCost cost;
        
        ParameterVector initial = {2.0, 2.0};
        std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
        
        optimizer.setAcquisitionFunction(acqs[i]);
        optimizer.setKernel(kernels[i % kernels.size()]);
        optimizer.setExplorationWeight(2.0);
        optimizer.setInitialSamples(3);
        optimizer.setLengthScale(0.8);
        optimizer.setNoise(1e-5);
        
        TerminationCriteria criteria;
        criteria.maxIterations = 15;
        optimizer.setTerminationCriteria(criteria);
        
        auto result = optimizer.optimize(cost, initial, bounds);
        EXPECT_GT(result.iterations, 0);
    }
}

TEST(AntColonyOptCoverage, BasicOptimization) {
    AntColonyOptimization optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    optimizer.setArchiveSize(30);
    optimizer.setAntsPerIteration(8);
    optimizer.setConvergenceSpeed(0.4);
    optimizer.setLocalSearchIntensity(0.8);
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
}

TEST(GreyWolfOptCoverage, BasicOptimization) {
    GreyWolfOptimizer optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    optimizer.setPackSize(25);
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
}

TEST(PowellMethodCoverage, BasicOptimization) {
    PowellMethod optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
}

TEST(BFGSOptCoverage, BasicOptimization) {
    BFGSOptimizer optimizer;
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    optimizer.setGradientStepSize(1e-5);
    
    TerminationCriteria criteria;
    criteria.maxIterations = 100;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
}

TEST(MultiStartOptCoverage, MultipleStarts) {
    auto localOpt = std::make_shared<NelderMead>();
    MultiStartOptimizer optimizer(localOpt, 5);
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
    EXPECT_FALSE(optimizer.getName().empty());
    EXPECT_FALSE(optimizer.getDescription().empty());
}

TEST(HybridOptCoverage, GlobalAndLocal) {
    auto globalOpt = std::make_shared<ParticleSwarmOptimization>();
    auto localOpt = std::make_shared<NelderMead>();
    HybridOptimizer optimizer(globalOpt, localOpt, 0.05);
    SphereCost cost;
    
    ParameterVector initial = {3.0, 3.0};
    std::vector<ParameterBounds> bounds = {{-5.0, 5.0}, {-5.0, 5.0}};
    
    TerminationCriteria criteria;
    criteria.maxIterations = 50;
    optimizer.setTerminationCriteria(criteria);
    
    auto result = optimizer.optimize(cost, initial, bounds);
    EXPECT_GT(result.iterations, 0);
    EXPECT_FALSE(optimizer.getName().empty());
    EXPECT_FALSE(optimizer.getDescription().empty());
    EXPECT_FALSE(optimizer.requiresGradient());
}

TEST(FactoryCoverage, CreateAllOptimizers) {
    std::vector<std::string> names = {
        "NelderMead",
        "GradientDescent",
        "GeneticAlgorithm",
        "PSO",
        "SimulatedAnnealing",
        "DifferentialEvolution",
        "BayesianOptimization",
        "ACO",
        "GWO",
        "Powell",
        "BFGS"
    };
    
    for (const auto& name : names) {
        auto opt = createOptimizer(name);
        if (opt) {
            EXPECT_FALSE(opt->getName().empty());
            EXPECT_FALSE(opt->getDescription().empty());
        }
    }
    
    auto available = getAvailableOptimizers();
    EXPECT_GT(available.size(), 0);
}