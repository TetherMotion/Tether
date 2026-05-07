/// @file test_optimizers.cpp
/// @brief Unit tests for all 8 optimizer implementations.

#include <gtest/gtest.h>
#include "tether/destabilizer/Optimizers.hpp"
#include <cmath>
#include <numeric>

using namespace Destabilizer;

// Simple quadratic: f(x) = -||x||^2 (negative, since optimizers maximize)
static double negativeQuadratic(const std::vector<double>& theta) {
    double sum = 0.0;
    for (double v : theta) sum += v * v;
    return -sum;
}

// Rosenbrock (minimize → negate for maximize)
static double negativeRosenbrock2D(const std::vector<double>& theta) {
    if (theta.size() < 2) return 0.0;
    double a = 1.0, b = 100.0;
    double x = theta[0], y = theta[1];
    return -((a - x) * (a - x) + b * (y - x * x) * (y - x * x));
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

TEST(OptimizerFactory, CreatesAllTypes) {
    auto sgd = Optimizer::create(OptimizerType::VanillaSGD);
    ASSERT_NE(sgd, nullptr);
    auto spsa = Optimizer::create(OptimizerType::SPSA);
    ASSERT_NE(spsa, nullptr);
    auto adam = Optimizer::create(OptimizerType::Adam);
    ASSERT_NE(adam, nullptr);
    auto cmaes = Optimizer::create(OptimizerType::CMAES);
    ASSERT_NE(cmaes, nullptr);
    auto cem = Optimizer::create(OptimizerType::CrossEntropy);
    ASSERT_NE(cem, nullptr);
    auto rs = Optimizer::create(OptimizerType::RandomSearch);
    ASSERT_NE(rs, nullptr);
    auto cd = Optimizer::create(OptimizerType::CoordinateDescent);
    ASSERT_NE(cd, nullptr);
    auto es = Optimizer::create(OptimizerType::EvolutionaryStrategy);
    ASSERT_NE(es, nullptr);
}

// ---------------------------------------------------------------------------
// VanillaSGD
// ---------------------------------------------------------------------------

TEST(VanillaSGD, DescendsOnQuadratic) {
    OptimizerConfig config;
    config.type = OptimizerType::VanillaSGD;
    config.learningRate = 0.1;
    config.fdEpsilon = 1e-4;
    config.centralDifferences = true;

    auto opt = Optimizer::create(OptimizerType::VanillaSGD);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    // The optimizer does gradient ASCENT on the objective
    // For -||x||^2, gradient = -2x, so ascent goes toward 0
    for (int i = 0; i < 50; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    // Should move toward origin
    EXPECT_LT(std::abs(theta[0]), 1.0);
    EXPECT_LT(std::abs(theta[1]), 1.0);
}

TEST(VanillaSGD, GradientNorm) {
    OptimizerConfig config;
    config.learningRate = 0.01;
    auto opt = Optimizer::create(OptimizerType::VanillaSGD);
    opt->initialize(2, config, 42);
    std::vector<double> theta = {1.0, 1.0};
    opt->step(theta, negativeQuadratic);
    EXPECT_GT(opt->gradientNorm(), 0.0);
}

TEST(VanillaSGD, Reset) {
    OptimizerConfig config;
    auto opt = Optimizer::create(OptimizerType::VanillaSGD);
    opt->initialize(2, config, 42);
    std::vector<double> theta = {1.0, 1.0};
    opt->step(theta, negativeQuadratic);
    opt->reset();
    EXPECT_EQ(opt->iteration(), 0);
}

// ---------------------------------------------------------------------------
// SPSA
// ---------------------------------------------------------------------------

TEST(SPSA, ConvergesToward0) {
    OptimizerConfig config;
    config.spsa_a = 0.1;
    config.spsa_c = 0.1;
    config.spsa_alpha = 0.602;
    config.spsa_gamma = 0.101;

    auto opt = Optimizer::create(OptimizerType::SPSA);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {3.0, 3.0};
    for (int i = 0; i < 100; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double norm = std::sqrt(theta[0]*theta[0] + theta[1]*theta[1]);
    EXPECT_LT(norm, 5.0); // Should improve from initial
}

// ---------------------------------------------------------------------------
// Adam
// ---------------------------------------------------------------------------

TEST(Adam, ConvergesOnQuadratic) {
    OptimizerConfig config;
    config.learningRate = 0.1;
    config.adam_beta1 = 0.9;
    config.adam_beta2 = 0.999;
    config.adam_epsilon = 1e-8;
    config.spsa_c = 0.1;
    config.spsa_gamma = 0.101;

    auto opt = Optimizer::create(OptimizerType::Adam);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    for (int i = 0; i < 100; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    EXPECT_LT(std::abs(theta[0]), 2.0);
    EXPECT_LT(std::abs(theta[1]), 2.0);
}

TEST(Adam, GradientNormPositive) {
    OptimizerConfig config;
    config.learningRate = 0.01;
    auto opt = Optimizer::create(OptimizerType::Adam);
    opt->initialize(2, config, 42);
    std::vector<double> theta = {1.0, 1.0};
    opt->step(theta, negativeQuadratic);
    EXPECT_GT(opt->gradientNorm(), 0.0);
}

// ---------------------------------------------------------------------------
// CMA-ES
// ---------------------------------------------------------------------------

TEST(CMAES, ImprovesFitness) {
    OptimizerConfig config;
    config.cmaes_sigma0 = 1.0;
    config.cmaes_populationSize = 10;

    auto opt = Optimizer::create(OptimizerType::CMAES);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 20; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GT(final_val, initial); // Should improve (maximize)
}

// ---------------------------------------------------------------------------
// CEM
// ---------------------------------------------------------------------------

TEST(CEM, ImprovesFitness) {
    OptimizerConfig config;
    config.cem_populationSize = 50;
    config.cem_eliteCount = 5;

    auto opt = Optimizer::create(OptimizerType::CrossEntropy);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 30; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GT(final_val, initial);
}

// ---------------------------------------------------------------------------
// RandomSearch
// ---------------------------------------------------------------------------

TEST(RandomSearch, FindsBetterSolution) {
    OptimizerConfig config;
    config.batchSize = 20;

    auto opt = Optimizer::create(OptimizerType::RandomSearch);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {10.0, 10.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 50; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GE(final_val, initial);
}

// ---------------------------------------------------------------------------
// CoordinateDescent
// ---------------------------------------------------------------------------

TEST(CoordinateDescent, ImprovesOnQuadratic) {
    OptimizerConfig config;
    config.learningRate = 0.5;
    config.cd_sweepsPerIteration = 1;

    auto opt = Optimizer::create(OptimizerType::CoordinateDescent);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {3.0, 3.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 10; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GE(final_val, initial);
}

// ---------------------------------------------------------------------------
// EvolutionaryStrategy
// ---------------------------------------------------------------------------

TEST(EvolutionaryStrategy, ImprovesFitness) {
    OptimizerConfig config;
    config.es_populationSize = 30;
    config.es_parentCount = 5;
    config.es_mutationRate = 0.5;

    auto opt = Optimizer::create(OptimizerType::EvolutionaryStrategy);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 30; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GT(final_val, initial);
}

// ---------------------------------------------------------------------------
// Common interface tests
// ---------------------------------------------------------------------------

TEST(Optimizer, AllOptimizersHavePositiveStepSize) {
    std::vector<OptimizerType> types = {
        OptimizerType::VanillaSGD,
        OptimizerType::SPSA,
        OptimizerType::Adam,
        OptimizerType::CMAES,
        OptimizerType::CrossEntropy,
        OptimizerType::RandomSearch,
        OptimizerType::CoordinateDescent,
        OptimizerType::EvolutionaryStrategy,
    };

    OptimizerConfig config;
    config.learningRate = 0.01;
    config.cmaes_sigma0 = 0.3;
    config.cmaes_populationSize = 10;
    config.cem_populationSize = 20;
    config.cem_eliteCount = 5;
    config.batchSize = 5;
    config.es_populationSize = 20;
    config.es_parentCount = 5;
    config.es_mutationRate = 0.1;

    for (auto type : types) {
        auto opt = Optimizer::create(type);
        opt->initialize(2, config, 42);
        std::vector<double> theta = {1.0, 1.0};
        opt->step(theta, negativeQuadratic);
        EXPECT_GE(opt->currentStepSize(), 0.0)
            << "Optimizer type " << static_cast<int>(type);
    }
}

// ---------------------------------------------------------------------------
// Reset tests
// ---------------------------------------------------------------------------

TEST(Optimizer, AllOptimizersResetCorrectly) {
    std::vector<OptimizerType> types = {
        OptimizerType::VanillaSGD,
        OptimizerType::SPSA,
        OptimizerType::Adam,
        OptimizerType::CMAES,
        OptimizerType::CrossEntropy,
        OptimizerType::RandomSearch,
        OptimizerType::CoordinateDescent,
        OptimizerType::EvolutionaryStrategy,
    };

    OptimizerConfig config;
    config.learningRate = 0.01;
    config.cmaes_sigma0 = 0.3;
    config.cmaes_populationSize = 10;
    config.cem_populationSize = 20;
    config.cem_eliteCount = 5;
    config.batchSize = 5;
    config.es_populationSize = 20;
    config.es_parentCount = 5;
    config.es_mutationRate = 0.1;

    for (auto type : types) {
        auto opt = Optimizer::create(type);
        opt->initialize(2, config, 42);
        std::vector<double> theta = {1.0, 1.0};
        opt->step(theta, negativeQuadratic);
        EXPECT_GT(opt->iteration(), 0)
            << "Optimizer type " << static_cast<int>(type);
        opt->reset();
        EXPECT_EQ(opt->iteration(), 0)
            << "Optimizer type " << static_cast<int>(type) << " failed to reset";
    }
}

// ---------------------------------------------------------------------------
// Forward-difference gradient (VanillaSGD non-central)
// ---------------------------------------------------------------------------

TEST(VanillaSGD, ForwardDifferences) {
    OptimizerConfig config;
    config.type = OptimizerType::VanillaSGD;
    config.learningRate = 0.1;
    config.fdEpsilon = 1e-4;
    config.centralDifferences = false; // Forward differences

    auto opt = Optimizer::create(OptimizerType::VanillaSGD);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {5.0, 5.0};
    for (int i = 0; i < 50; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    EXPECT_LT(std::abs(theta[0]), 2.0);
    EXPECT_LT(std::abs(theta[1]), 2.0);
}

// ---------------------------------------------------------------------------
// Adam factory creation
// ---------------------------------------------------------------------------

TEST(Adam, FactoryCreation) {
    auto opt = Optimizer::create(OptimizerType::Adam);
    ASSERT_NE(opt, nullptr);
    OptimizerConfig config;
    config.learningRate = 0.01;
    opt->initialize(2, config, 42);
    EXPECT_EQ(opt->iteration(), 0);
}

// ---------------------------------------------------------------------------
// CoordinateDescent improvement path
// ---------------------------------------------------------------------------

TEST(CoordinateDescent, FindsImprovement) {
    OptimizerConfig config;
    config.learningRate = 1.0;
    config.cd_sweepsPerIteration = 2;

    auto opt = Optimizer::create(OptimizerType::CoordinateDescent);
    opt->initialize(2, config, 42);

    std::vector<double> theta = {3.0, 3.0};
    double initial = negativeQuadratic(theta);
    for (int i = 0; i < 20; ++i) {
        opt->step(theta, negativeQuadratic);
    }
    double final_val = negativeQuadratic(theta);
    EXPECT_GT(final_val, initial);
}
