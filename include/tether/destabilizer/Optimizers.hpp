#pragma once
/// @file Optimizers.hpp
/// @brief Stochastic optimization algorithms for the Destabilizer.
///
/// Each optimizer searches over perturbation parameters θ to MAXIMIZE
/// an instability metric J(θ).

#include "DestabilizerTypes.hpp"
#include <vector>
#include <functional>
#include <random>
#include <memory>
#include <cmath>

namespace Destabilizer {

/// Objective function: given θ, returns J(θ).
/// The Destabilizer engine provides this by running a simulation rollout.
using ObjectiveFunction = std::function<double(const std::vector<double>&)>;

/// Abstract optimizer base class.
class Optimizer {
public:
    virtual ~Optimizer() = default;

    /// Initialize the optimizer with the problem dimension and config.
    virtual void initialize(int dim, const OptimizerConfig& config, uint64_t seed) = 0;

    /// Run one optimization step. Updates theta in-place. Returns current J.
    virtual double step(std::vector<double>& theta, const ObjectiveFunction& objective) = 0;

    /// Get current iteration count.
    virtual int iteration() const = 0;

    /// Get current gradient norm (0 for gradient-free methods).
    virtual double gradientNorm() const = 0;

    /// Get current step size / learning rate.
    virtual double currentStepSize() const = 0;

    /// Get number of function evaluations used in the last step.
    virtual int lastStepEvaluations() const = 0;

    /// Reset optimizer state.
    virtual void reset() = 0;

    /// Create an optimizer of the given type.
    static std::unique_ptr<Optimizer> create(OptimizerType type);
};

/// Vanilla SGD with finite-difference gradient estimation.
class VanillaSGDOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return gradNorm_; }
    double currentStepSize() const override { return lr_; }
    int lastStepEvaluations() const override { return lastEvals_; }
    void reset() override;
private:
    int dim_ = 0;
    double lr_ = 0.01;
    double fdEps_ = 1e-3;
    bool central_ = true;
    int iter_ = 0;
    double gradNorm_ = 0.0;
    int lastEvals_ = 0;
    std::mt19937_64 rng_;
};

/// SPSA (Simultaneous Perturbation Stochastic Approximation).
class SPSAOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return gradNorm_; }
    double currentStepSize() const override { return ak_; }
    int lastStepEvaluations() const override { return 2; }
    void reset() override;
private:
    int dim_ = 0;
    double a_, c_, alpha_, gamma_;
    double A_ = 10.0;
    int iter_ = 0;
    double ak_ = 0.0, ck_ = 0.0;
    double gradNorm_ = 0.0;
    std::mt19937_64 rng_;
};

/// Adam optimizer with finite-difference or SPSA gradient estimation.
class AdamOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return gradNorm_; }
    double currentStepSize() const override { return lr_; }
    int lastStepEvaluations() const override { return lastEvals_; }
    void reset() override;
private:
    int dim_ = 0;
    double lr_, beta1_, beta2_, eps_;
    double spsa_c_, spsa_gamma_;
    bool useSPSA_ = true;
    std::vector<double> m_, v_;
    int iter_ = 0;
    double gradNorm_ = 0.0;
    int lastEvals_ = 0;
    std::mt19937_64 rng_;

    std::vector<double> estimateGradientSPSA(const std::vector<double>& theta,
                                              const ObjectiveFunction& objective);
    std::vector<double> estimateGradientFD(const std::vector<double>& theta,
                                            const ObjectiveFunction& objective,
                                            double eps, bool central);
};

/// CMA-ES (Covariance Matrix Adaptation Evolution Strategy).
class CMAESOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return 0.0; }
    double currentStepSize() const override { return sigma_; }
    int lastStepEvaluations() const override { return lambda_; }
    void reset() override;
private:
    int dim_ = 0;
    int lambda_ = 0;  // population size
    int mu_ = 0;      // parent count
    double sigma_ = 0.3;
    std::vector<double> mean_;
    std::vector<double> pc_, ps_;
    std::vector<std::vector<double>> C_;  // covariance matrix
    std::vector<double> weights_;
    double mueff_ = 0.0, cs_ = 0.0, ds_ = 0.0, cc_ = 0.0, c1_ = 0.0, cmu_ = 0.0;
    double chiN_ = 0.0;
    int iter_ = 0;
    std::mt19937_64 rng_;

    std::vector<double> sampleNormal();
    void decomposeCovariance();
    std::vector<std::vector<double>> eigenVectors_;
    std::vector<double> eigenValues_;
    bool needsDecomposition_ = true;
};

/// Cross-Entropy Method (CEM).
class CEMOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return 0.0; }
    double currentStepSize() const override { return avgStd_; }
    int lastStepEvaluations() const override { return populationSize_; }
    void reset() override;
private:
    int dim_ = 0;
    int populationSize_ = 100;
    int eliteCount_ = 10;
    std::vector<double> mean_, stddev_;
    int iter_ = 0;
    double avgStd_ = 1.0;
    std::mt19937_64 rng_;
};

/// Random search baseline.
class RandomSearchOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return 0.0; }
    double currentStepSize() const override { return scale_; }
    int lastStepEvaluations() const override { return batchSize_; }
    void reset() override;
private:
    int dim_ = 0;
    int batchSize_ = 1;
    double scale_ = 1.0;
    double bestJ_ = -1e30;
    std::vector<double> bestTheta_;
    int iter_ = 0;
    std::mt19937_64 rng_;
};

/// Coordinate descent over θ components.
class CoordinateDescentOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return gradNorm_; }
    double currentStepSize() const override { return stepSize_; }
    int lastStepEvaluations() const override { return lastEvals_; }
    void reset() override;
private:
    int dim_ = 0;
    double stepSize_ = 0.01;
    int sweepsPerIter_ = 1;
    int iter_ = 0;
    double gradNorm_ = 0.0;
    int lastEvals_ = 0;
    int currentCoord_ = 0;
    std::mt19937_64 rng_;
};

/// (μ, λ) Evolutionary Strategy with mutation and selection.
class EvolutionaryStrategyOptimizer : public Optimizer {
public:
    void initialize(int dim, const OptimizerConfig& config, uint64_t seed) override;
    double step(std::vector<double>& theta, const ObjectiveFunction& objective) override;
    int iteration() const override { return iter_; }
    double gradientNorm() const override { return 0.0; }
    double currentStepSize() const override { return mutationRate_; }
    int lastStepEvaluations() const override { return populationSize_; }
    void reset() override;
private:
    int dim_ = 0;
    int populationSize_ = 50;
    int parentCount_ = 10;
    double mutationRate_ = 0.1;
    int iter_ = 0;
    std::mt19937_64 rng_;
};

} // namespace Destabilizer
