/// @file Optimizers.cpp
/// @brief All 8 optimizer implementations for the Destabilizer.

#include "tether/destabilizer/Optimizers.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <cassert>

namespace Destabilizer {

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::unique_ptr<Optimizer> Optimizer::create(OptimizerType type) {
    switch (type) {
    case OptimizerType::VanillaSGD:         return std::make_unique<VanillaSGDOptimizer>();
    case OptimizerType::SPSA:               return std::make_unique<SPSAOptimizer>();
    case OptimizerType::Adam:               return std::make_unique<AdamOptimizer>();
    case OptimizerType::CMAES:              return std::make_unique<CMAESOptimizer>();
    case OptimizerType::CrossEntropy:       return std::make_unique<CEMOptimizer>();
    case OptimizerType::RandomSearch:       return std::make_unique<RandomSearchOptimizer>();
    case OptimizerType::CoordinateDescent:  return std::make_unique<CoordinateDescentOptimizer>();
    case OptimizerType::EvolutionaryStrategy: return std::make_unique<EvolutionaryStrategyOptimizer>();
    }
    return std::make_unique<AdamOptimizer>();
}

// ---------------------------------------------------------------------------
// Vanilla SGD
// ---------------------------------------------------------------------------

void VanillaSGDOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    lr_ = config.learningRate;
    fdEps_ = config.fdEpsilon;
    central_ = config.centralDifferences;
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
    rng_.seed(seed);
}

double VanillaSGDOptimizer::step(std::vector<double>& theta,
                                  const ObjectiveFunction& objective) {
    double J0 = objective(theta);
    std::vector<double> grad(dim_, 0.0);

    if (central_) {
        // Central differences: ∂J/∂θ_i ≈ (J(θ+εe_i) - J(θ-εe_i)) / (2ε)
        for (int i = 0; i < dim_; ++i) {
            auto thetaPlus = theta;
            auto thetaMinus = theta;
            thetaPlus[i] += fdEps_;
            thetaMinus[i] -= fdEps_;
            grad[i] = (objective(thetaPlus) - objective(thetaMinus)) / (2.0 * fdEps_);
        }
        lastEvals_ = 2 * dim_ + 1;
    } else {
        // Forward differences: ∂J/∂θ_i ≈ (J(θ+εe_i) - J(θ)) / ε
        for (int i = 0; i < dim_; ++i) {
            auto thetaPlus = theta;
            thetaPlus[i] += fdEps_;
            grad[i] = (objective(thetaPlus) - J0) / fdEps_;
        }
        lastEvals_ = dim_ + 1;
    }

    // Gradient ascent (maximize J)
    gradNorm_ = 0.0;
    for (int i = 0; i < dim_; ++i) {
        gradNorm_ += grad[i] * grad[i];
        theta[i] += lr_ * grad[i];
    }
    gradNorm_ = std::sqrt(gradNorm_);

    iter_++;
    return J0;
}

void VanillaSGDOptimizer::reset() {
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
}

// ---------------------------------------------------------------------------
// SPSA
// ---------------------------------------------------------------------------

void SPSAOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    a_ = config.spsa_a;
    c_ = config.spsa_c;
    alpha_ = config.spsa_alpha;
    gamma_ = config.spsa_gamma;
    A_ = std::max(1.0, 0.1 * config.learningRate);
    iter_ = 0;
    gradNorm_ = 0.0;
    rng_.seed(seed);
}

double SPSAOptimizer::step(std::vector<double>& theta,
                            const ObjectiveFunction& objective) {
    iter_++;
    ak_ = a_ / std::pow(iter_ + A_, alpha_);
    ck_ = c_ / std::pow(static_cast<double>(iter_), gamma_);

    // Generate Bernoulli perturbation vector
    std::bernoulli_distribution dist(0.5);
    std::vector<double> delta(dim_);
    for (int i = 0; i < dim_; ++i) {
        delta[i] = dist(rng_) ? 1.0 : -1.0;
    }

    // Evaluate at θ ± ck*Δ
    std::vector<double> thetaPlus(dim_), thetaMinus(dim_);
    for (int i = 0; i < dim_; ++i) {
        thetaPlus[i] = theta[i] + ck_ * delta[i];
        thetaMinus[i] = theta[i] - ck_ * delta[i];
    }

    double Jplus = objective(thetaPlus);
    double Jminus = objective(thetaMinus);

    // SPSA gradient estimate
    std::vector<double> grad(dim_);
    gradNorm_ = 0.0;
    for (int i = 0; i < dim_; ++i) {
        grad[i] = (Jplus - Jminus) / (2.0 * ck_ * delta[i]);
        gradNorm_ += grad[i] * grad[i];
        theta[i] += ak_ * grad[i];  // Gradient ascent
    }
    gradNorm_ = std::sqrt(gradNorm_);

    return (Jplus + Jminus) / 2.0;
}

void SPSAOptimizer::reset() {
    iter_ = 0;
    gradNorm_ = 0.0;
    ak_ = 0.0;
    ck_ = 0.0;
}

// ---------------------------------------------------------------------------
// Adam
// ---------------------------------------------------------------------------

void AdamOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    lr_ = config.learningRate;
    beta1_ = config.adam_beta1;
    beta2_ = config.adam_beta2;
    eps_ = config.adam_epsilon;
    spsa_c_ = config.spsa_c;
    spsa_gamma_ = config.spsa_gamma;
    useSPSA_ = true;  // Default to SPSA gradient estimation
    m_.assign(dim, 0.0);
    v_.assign(dim, 0.0);
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
    rng_.seed(seed);
}

std::vector<double> AdamOptimizer::estimateGradientSPSA(
    const std::vector<double>& theta, const ObjectiveFunction& objective)
{
    double ck = spsa_c_ / std::pow(std::max(1, iter_), spsa_gamma_);
    std::bernoulli_distribution dist(0.5);
    std::vector<double> delta(dim_);
    for (int i = 0; i < dim_; ++i) {
        delta[i] = dist(rng_) ? 1.0 : -1.0;
    }

    std::vector<double> thetaPlus(dim_), thetaMinus(dim_);
    for (int i = 0; i < dim_; ++i) {
        thetaPlus[i] = theta[i] + ck * delta[i];
        thetaMinus[i] = theta[i] - ck * delta[i];
    }

    double Jp = objective(thetaPlus);
    double Jm = objective(thetaMinus);
    lastEvals_ = 2;

    std::vector<double> grad(dim_);
    for (int i = 0; i < dim_; ++i) {
        grad[i] = (Jp - Jm) / (2.0 * ck * delta[i]);
    }
    return grad;
}

std::vector<double> AdamOptimizer::estimateGradientFD(
    const std::vector<double>& theta, const ObjectiveFunction& objective,
    double eps, bool central)
{
    std::vector<double> grad(dim_);
    if (central) {
        for (int i = 0; i < dim_; ++i) {
            auto tp = theta; tp[i] += eps;
            auto tm = theta; tm[i] -= eps;
            grad[i] = (objective(tp) - objective(tm)) / (2.0 * eps);
        }
        lastEvals_ = 2 * dim_;
    } else {
        double J0 = objective(theta);
        for (int i = 0; i < dim_; ++i) {
            auto tp = theta; tp[i] += eps;
            grad[i] = (objective(tp) - J0) / eps;
        }
        lastEvals_ = dim_ + 1;
    }
    return grad;
}

double AdamOptimizer::step(std::vector<double>& theta,
                            const ObjectiveFunction& objective) {
    iter_++;

    // Estimate gradient
    auto grad = useSPSA_ ?
        estimateGradientSPSA(theta, objective) :
        estimateGradientFD(theta, objective, 1e-3, true);

    // Adam update (gradient ASCENT: we add, not subtract)
    gradNorm_ = 0.0;
    for (int i = 0; i < dim_; ++i) {
        m_[i] = beta1_ * m_[i] + (1.0 - beta1_) * grad[i];
        v_[i] = beta2_ * v_[i] + (1.0 - beta2_) * grad[i] * grad[i];

        double mHat = m_[i] / (1.0 - std::pow(beta1_, iter_));
        double vHat = v_[i] / (1.0 - std::pow(beta2_, iter_));

        theta[i] += lr_ * mHat / (std::sqrt(vHat) + eps_);
        gradNorm_ += grad[i] * grad[i];
    }
    gradNorm_ = std::sqrt(gradNorm_);

    return objective(theta);
}

void AdamOptimizer::reset() {
    m_.assign(dim_, 0.0);
    v_.assign(dim_, 0.0);
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
}

// ---------------------------------------------------------------------------
// CMA-ES
// ---------------------------------------------------------------------------

void CMAESOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    sigma_ = config.cmaes_sigma0;
    lambda_ = config.cmaes_populationSize;
    if (lambda_ <= 0) lambda_ = 4 + static_cast<int>(3 * std::log(dim));
    mu_ = lambda_ / 2;

    mean_.assign(dim, 0.0);
    pc_.assign(dim, 0.0);
    ps_.assign(dim, 0.0);

    // Initialize covariance as identity
    C_.assign(dim, std::vector<double>(dim, 0.0));
    for (int i = 0; i < dim; ++i) C_[i][i] = 1.0;

    // Compute weights
    weights_.resize(mu_);
    double sumW = 0.0;
    for (int i = 0; i < mu_; ++i) {
        weights_[i] = std::log(mu_ + 0.5) - std::log(i + 1.0);
        sumW += weights_[i];
    }
    for (auto& w : weights_) w /= sumW;

    mueff_ = 1.0 / std::inner_product(weights_.begin(), weights_.end(),
                                        weights_.begin(), 0.0);

    // Strategy parameters
    cs_ = (mueff_ + 2.0) / (dim + mueff_ + 5.0);
    ds_ = 1.0 + 2.0 * std::max(0.0, std::sqrt((mueff_ - 1.0) / (dim + 1.0)) - 1.0) + cs_;
    cc_ = (4.0 + mueff_ / dim) / (dim + 4.0 + 2.0 * mueff_ / dim);
    c1_ = 2.0 / ((dim + 1.3) * (dim + 1.3) + mueff_);
    cmu_ = std::min(1.0 - c1_,
                     2.0 * (mueff_ - 2.0 + 1.0 / mueff_) /
                     ((dim + 2.0) * (dim + 2.0) + mueff_));
    chiN_ = std::sqrt(static_cast<double>(dim)) *
            (1.0 - 1.0 / (4.0 * dim) + 1.0 / (21.0 * dim * dim));

    iter_ = 0;
    rng_.seed(seed);
    needsDecomposition_ = true;
}

std::vector<double> CMAESOptimizer::sampleNormal() {
    std::normal_distribution<double> dist(0.0, 1.0);
    std::vector<double> z(dim_);
    for (int i = 0; i < dim_; ++i) z[i] = dist(rng_);

    if (needsDecomposition_) {
        decomposeCovariance();
    }

    // Transform by sqrt(C): y = B * D * z
    std::vector<double> y(dim_, 0.0);
    for (int i = 0; i < dim_; ++i) {
        double dz = std::sqrt(std::max(0.0, eigenValues_[i])) * z[i];
        for (int j = 0; j < dim_; ++j) {
            y[j] += eigenVectors_[j][i] * dz;
        }
    }
    return y;
}

void CMAESOptimizer::decomposeCovariance() {
    // Simple eigendecomposition for symmetric matrix via Jacobi iteration
    // For production, use a proper linear algebra library
    eigenVectors_.assign(dim_, std::vector<double>(dim_, 0.0));
    eigenValues_.assign(dim_, 0.0);

    // Copy C
    auto A = C_;

    // Initialize eigenvectors to identity
    for (int i = 0; i < dim_; ++i) eigenVectors_[i][i] = 1.0;

    // Jacobi eigenvalue algorithm (simplified)
    for (int sweep = 0; sweep < 100; ++sweep) {
        double offDiag = 0.0;
        for (int p = 0; p < dim_; ++p)
            for (int q = p + 1; q < dim_; ++q)
                offDiag += std::abs(A[p][q]);

        if (offDiag < 1e-15 * dim_) break;

        for (int p = 0; p < dim_; ++p) {
            for (int q = p + 1; q < dim_; ++q) {
                if (std::abs(A[p][q]) < 1e-30) continue;

                double tau = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
                double t = (tau >= 0) ?
                    1.0 / (tau + std::sqrt(1.0 + tau * tau)) :
                    -1.0 / (-tau + std::sqrt(1.0 + tau * tau));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;

                // Rotate
                double app = A[p][p], aqq = A[q][q], apq = A[p][q];
                A[p][p] = c*c*app - 2*s*c*apq + s*s*aqq;
                A[q][q] = s*s*app + 2*s*c*apq + c*c*aqq;
                A[p][q] = A[q][p] = 0.0;

                for (int r = 0; r < dim_; ++r) {
                    if (r == p || r == q) continue;
                    double arp = A[r][p], arq = A[r][q];
                    A[r][p] = A[p][r] = c*arp - s*arq;
                    A[r][q] = A[q][r] = s*arp + c*arq;
                }

                // Update eigenvectors
                for (int r = 0; r < dim_; ++r) {
                    double vp = eigenVectors_[r][p], vq = eigenVectors_[r][q];
                    eigenVectors_[r][p] = c*vp - s*vq;
                    eigenVectors_[r][q] = s*vp + c*vq;
                }
            }
        }
    }

    for (int i = 0; i < dim_; ++i) {
        eigenValues_[i] = std::max(1e-20, A[i][i]);
    }
    needsDecomposition_ = false;
}

double CMAESOptimizer::step(std::vector<double>& theta,
                             const ObjectiveFunction& objective) {
    // Copy current mean
    mean_ = theta;

    // Sample population
    struct Individual {
        std::vector<double> x;
        std::vector<double> z;  // Normal sample (for covariance update)
        double fitness;
    };
    std::vector<Individual> population(lambda_);

    for (int k = 0; k < lambda_; ++k) {
        population[k].z = sampleNormal();
        population[k].x.resize(dim_);
        for (int i = 0; i < dim_; ++i) {
            population[k].x[i] = mean_[i] + sigma_ * population[k].z[i];
        }
        population[k].fitness = objective(population[k].x);
    }

    // Sort by fitness (descending — we maximize)
    std::sort(population.begin(), population.end(),
              [](const Individual& a, const Individual& b) {
                  return a.fitness > b.fitness;
              });

    // Update mean
    std::vector<double> oldMean = mean_;
    for (int i = 0; i < dim_; ++i) {
        mean_[i] = 0.0;
        for (int k = 0; k < mu_; ++k) {
            mean_[i] += weights_[k] * population[k].x[i];
        }
    }

    // Update evolution paths
    std::vector<double> meanDiff(dim_);
    for (int i = 0; i < dim_; ++i) {
        meanDiff[i] = (mean_[i] - oldMean[i]) / sigma_;
    }

    for (int i = 0; i < dim_; ++i) {
        ps_[i] = (1.0 - cs_) * ps_[i] + std::sqrt(cs_ * (2.0 - cs_) * mueff_) * meanDiff[i];
    }

    double psNorm = 0.0;
    for (double p : ps_) psNorm += p * p;
    psNorm = std::sqrt(psNorm);

    bool hsig = psNorm / std::sqrt(1.0 - std::pow(1.0 - cs_, 2.0 * (iter_ + 1))) < (1.4 + 2.0 / (dim_ + 1.0)) * chiN_;

    for (int i = 0; i < dim_; ++i) {
        pc_[i] = (1.0 - cc_) * pc_[i] +
                 (hsig ? std::sqrt(cc_ * (2.0 - cc_) * mueff_) * meanDiff[i] : 0.0);
    }

    // Update covariance matrix
    for (int i = 0; i < dim_; ++i) {
        for (int j = 0; j <= i; ++j) {
            double rankOne = pc_[i] * pc_[j];
            double rankMu = 0.0;
            for (int k = 0; k < mu_; ++k) {
                rankMu += weights_[k] * population[k].z[i] * population[k].z[j];
            }
            C_[i][j] = (1.0 - c1_ - cmu_) * C_[i][j] + c1_ * rankOne + cmu_ * rankMu;
            C_[j][i] = C_[i][j];
        }
    }
    needsDecomposition_ = true;

    // Update step size
    sigma_ *= std::exp((cs_ / ds_) * (psNorm / chiN_ - 1.0));
    sigma_ = std::max(1e-20, std::min(sigma_, 1e10));

    // Output best
    theta = mean_;
    iter_++;

    return population[0].fitness;
}

void CMAESOptimizer::reset() {
    iter_ = 0;
    if (dim_ > 0) {
        mean_.assign(dim_, 0.0);
        pc_.assign(dim_, 0.0);
        ps_.assign(dim_, 0.0);
        C_.assign(dim_, std::vector<double>(dim_, 0.0));
        for (int i = 0; i < dim_; ++i) C_[i][i] = 1.0;
        needsDecomposition_ = true;
    }
}

// ---------------------------------------------------------------------------
// CEM (Cross-Entropy Method)
// ---------------------------------------------------------------------------

void CEMOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    populationSize_ = config.cem_populationSize;
    eliteCount_ = config.cem_eliteCount;
    mean_.assign(dim, 0.0);
    stddev_.assign(dim, 1.0);
    iter_ = 0;
    rng_.seed(seed);
}

double CEMOptimizer::step(std::vector<double>& theta,
                           const ObjectiveFunction& objective) {
    std::normal_distribution<double> dist(0.0, 1.0);

    // Sample population
    struct Sample { std::vector<double> x; double fitness; };
    std::vector<Sample> population(populationSize_);

    for (int k = 0; k < populationSize_; ++k) {
        population[k].x.resize(dim_);
        for (int i = 0; i < dim_; ++i) {
            population[k].x[i] = mean_[i] + stddev_[i] * dist(rng_);
        }
        population[k].fitness = objective(population[k].x);
    }

    // Sort by fitness (descending — maximize)
    std::sort(population.begin(), population.end(),
              [](const Sample& a, const Sample& b) {
                  return a.fitness > b.fitness;
              });

    // Update mean and stddev from elite
    for (int i = 0; i < dim_; ++i) {
        double sum = 0.0, sumSq = 0.0;
        for (int k = 0; k < eliteCount_; ++k) {
            sum += population[k].x[i];
            sumSq += population[k].x[i] * population[k].x[i];
        }
        mean_[i] = sum / eliteCount_;
        double var = sumSq / eliteCount_ - mean_[i] * mean_[i];
        stddev_[i] = std::sqrt(std::max(1e-10, var));
    }

    avgStd_ = 0.0;
    for (double s : stddev_) avgStd_ += s;
    avgStd_ /= dim_;

    theta = mean_;
    iter_++;

    return population[0].fitness;
}

void CEMOptimizer::reset() {
    iter_ = 0;
    mean_.assign(dim_, 0.0);
    stddev_.assign(dim_, 1.0);
    avgStd_ = 1.0;
}

// ---------------------------------------------------------------------------
// Random Search
// ---------------------------------------------------------------------------

void RandomSearchOptimizer::initialize(int dim, const OptimizerConfig& config, uint64_t seed) {
    dim_ = dim;
    batchSize_ = config.batchSize;
    scale_ = 1.0;
    bestJ_ = -1e30;
    bestTheta_.assign(dim, 0.0);
    iter_ = 0;
    rng_.seed(seed);
}

double RandomSearchOptimizer::step(std::vector<double>& theta,
                                    const ObjectiveFunction& objective) {
    std::normal_distribution<double> dist(0.0, scale_);

    for (int b = 0; b < batchSize_; ++b) {
        std::vector<double> candidate(dim_);
        for (int i = 0; i < dim_; ++i) {
            candidate[i] = theta[i] + dist(rng_);
        }
        double J = objective(candidate);
        if (J > bestJ_) {
            bestJ_ = J;
            bestTheta_ = candidate;
        }
    }

    theta = bestTheta_;
    iter_++;
    return bestJ_;
}

void RandomSearchOptimizer::reset() {
    iter_ = 0;
    bestJ_ = -1e30;
    bestTheta_.assign(dim_, 0.0);
}

// ---------------------------------------------------------------------------
// Coordinate Descent
// ---------------------------------------------------------------------------

void CoordinateDescentOptimizer::initialize(int dim, const OptimizerConfig& config,
                                              uint64_t seed) {
    dim_ = dim;
    stepSize_ = config.learningRate;
    sweepsPerIter_ = config.cd_sweepsPerIteration;
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
    currentCoord_ = 0;
    rng_.seed(seed);
}

double CoordinateDescentOptimizer::step(std::vector<double>& theta,
                                          const ObjectiveFunction& objective) {
    double bestJ = objective(theta);
    lastEvals_ = 1;

    for (int sweep = 0; sweep < sweepsPerIter_; ++sweep) {
        for (int i = 0; i < dim_; ++i) {
            // Try +step
            theta[i] += stepSize_;
            double Jplus = objective(theta);
            theta[i] -= stepSize_;

            // Try -step
            theta[i] -= stepSize_;
            double Jminus = objective(theta);
            theta[i] += stepSize_;

            lastEvals_ += 2;

            // Pick the best direction (maximize J)
            if (Jplus > bestJ && Jplus >= Jminus) {
                theta[i] += stepSize_;
                bestJ = Jplus;
            } else if (Jminus > bestJ) {
                theta[i] -= stepSize_;
                bestJ = Jminus;
            }

            gradNorm_ = std::abs(Jplus - Jminus) / (2.0 * stepSize_);
        }
    }

    iter_++;
    return bestJ;
}

void CoordinateDescentOptimizer::reset() {
    iter_ = 0;
    gradNorm_ = 0.0;
    lastEvals_ = 0;
    currentCoord_ = 0;
}

// ---------------------------------------------------------------------------
// Evolutionary Strategy
// ---------------------------------------------------------------------------

void EvolutionaryStrategyOptimizer::initialize(int dim, const OptimizerConfig& config,
                                                 uint64_t seed) {
    dim_ = dim;
    populationSize_ = config.es_populationSize;
    parentCount_ = config.es_parentCount;
    mutationRate_ = config.es_mutationRate;
    iter_ = 0;
    rng_.seed(seed);
}

double EvolutionaryStrategyOptimizer::step(std::vector<double>& theta,
                                            const ObjectiveFunction& objective) {
    std::normal_distribution<double> mutation(0.0, mutationRate_);

    struct Individual { std::vector<double> x; double fitness; 
        Individual& operator=(const Individual&) = default;
    };
    std::vector<Individual> population(populationSize_);

    // Generate offspring by mutation
    for (int k = 0; k < populationSize_; ++k) {
        population[k].x.resize(dim_);
        for (int i = 0; i < dim_; ++i) {
            population[k].x[i] = theta[i] + mutation(rng_);
        }
        population[k].fitness = objective(population[k].x);
    }

    // Sort by fitness (descending — maximize)
    std::sort(population.begin(), population.end(),
              [](const Individual& a, const Individual& b) {
                  return a.fitness > b.fitness;
              });

    // Recombine: mean of top parents
    for (int i = 0; i < dim_; ++i) {
        double sum = 0.0;
        for (int k = 0; k < parentCount_; ++k) {
            sum += population[k].x[i];
        }
        theta[i] = sum / parentCount_;
    }

    iter_++;
    return population[0].fitness;
}

void EvolutionaryStrategyOptimizer::reset() {
    iter_ = 0;
}

} // namespace Destabilizer
