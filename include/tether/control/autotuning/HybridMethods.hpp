/**
 * @file HybridMethods.hpp
 * @brief Hybrid Controller Tuning Methods
 * 
 * @details
 * Methods that combine multiple approaches for improved performance.
 * 
 * ## Implemented Methods
 * 
 * ### Z-N + Optimization
 * Use Z-N as starting point for numerical optimization
 * 
 * ### IMC + Relay
 * Combine IMC structure with relay identification
 * 
 * ### Fuzzy + PID
 * Fuzzy-tuned PID parameters
 * 
 * ### GA + PID
 * Genetic algorithm optimization of PID
 * 
 * ### Neural PID
 * Neural network augmented PID
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include "ClassicalTuningMethods.hpp"
#include "ModelBasedMethods.hpp"
#include "OptimizationAlgorithms.hpp"
#include "AdaptiveMethods.hpp"
#include <memory>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// Ziegler-Nichols + Optimization
// ============================================================================

/**
 * @brief Z-N with Subsequent Optimization
 * 
 * Uses Z-N tuning rules for initial gains, then refines
 * using numerical optimization.
 * 
 * ## Benefits
 * - Fast initial tuning (no optimization needed to start)
 * - Optimization converges faster from good starting point
 * - Can handle additional constraints
 */
class ZNWithOptimization : public OfflineAutotuner {
public:
    std::string getName() const override { return "Z-N + Optimization"; }
    std::string getDescription() const override {
        return "Ziegler-Nichols for initial guess, optimization for refinement. "
               "Fast convergence with good starting point.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set initial tuning method
     */
    enum class InitialMethod {
        ZNStepResponse,
        ZNUltimateCycle,
        CohenCoon,
        IMC
    };
    void setInitialMethod(InitialMethod method) { m_initialMethod = method; }
    
    /**
     * @brief Set optimization algorithm
     */
    template<typename Opt>
    void setOptimizer() {
        m_optimizer = std::make_unique<Opt>();
    }
    
    /**
     * @brief Set cost function for optimization
     * @param cost Cost function to minimise (takes ownership).
     */
    void setCostFunction(std::unique_ptr<CostFunction> cost);
    
    /**
     * @brief Set optimization bounds (relative to initial)
     * @param lower Lower bound as a fraction of the initial gain.
     * @param upper Upper bound as a fraction of the initial gain.
     */
    void setRelativeBounds(double lower, double upper);
    
    /**
     * @brief Set maximum optimization iterations
     * @param n Iteration budget.
     */
    void setMaxIterations(int n) { m_maxIterations = n; }
    
    /**
     * @brief Get initial (Z-N) gains
     * @return Gains produced by the Ziegler-Nichols seed step.
     */
    PIDGains getInitialGains() const { return m_initialGains; }
    
    /**
     * @brief Get improvement ratio
     * @return Final cost divided by the initial (Z-N) cost; < 1 is better.
     */
    double getImprovementRatio() const;
    
private:
    InitialMethod m_initialMethod{InitialMethod::ZNStepResponse};
    std::unique_ptr<OptimizationAlgorithm> m_optimizer;
    std::unique_ptr<CostFunction> m_costFunction;
    
    double m_lowerBound{0.5}, m_upperBound{2.0};
    int m_maxIterations{100};
    
    PIDGains m_initialGains;
    PIDGains m_optimizedGains;
    double m_initialCost{0.0}, m_finalCost{0.0};
};

// ============================================================================
// IMC + Relay Identification
// ============================================================================

/**
 * @brief IMC with Relay-Based Identification
 * 
 * Uses relay feedback experiment to identify process model,
 * then designs IMC-based controller.
 * 
 * ## Procedure
 * 1. Run relay experiment
 * 2. Extract FOPDT model from oscillation
 * 3. Design IMC controller
 * 4. Convert to PID form
 */
class IMCWithRelay : public OnlineAutotuner {
public:
    std::string getName() const override { return "IMC + Relay"; }
    std::string getDescription() const override {
        return "Relay identification + IMC design. Automatic model "
               "extraction and controller synthesis.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set relay amplitude
     * @param d Relay output amplitude (must stay within the safe range).
     */
    void setRelayAmplitude(double d) { m_relayAmplitude = d; }
    
    /**
     * @brief Set relay hysteresis
     * @param h Switching hysteresis; must exceed the noise amplitude.
     */
    void setRelayHysteresis(double h) { m_hysteresis = h; }
    
    /**
     * @brief Set IMC time constant factor
     * @param lambda Closed-loop time constant multiplier relative to dead time.
     */
    void setIMCFactor(double lambda) { m_lambdaFactor = lambda; }
    
    /**
     * @brief Get identified model
     * @return FOPDT model identified from the relay experiment.
     */
    FOPDTModel getIdentifiedModel() const { return m_identifiedModel; }
    
private:
    // Relay parameters
    double m_relayAmplitude{1.0};
    double m_hysteresis{0.0};
    
    // IMC parameter
    double m_lambdaFactor{1.0};
    
    // State
    enum class Phase {
        RelayTest,
        Complete
    };
    Phase m_phase{Phase::RelayTest};
    
    AstromHagglundRelay m_relayTuner;
    IMCDesign m_imcDesigner;
    
    FOPDTModel m_identifiedModel;
    bool m_running{false};
};

// ============================================================================
// Fuzzy PID
// ============================================================================

/**
 * @brief Fuzzy-Augmented PID Controller
 * 
 * Base PID with fuzzy logic gain scheduling.
 * Adjusts gains online based on error and error rate.
 * 
 * ## Structure
 * Kp = Kp0 * (1 + ΔKp_fuzzy)
 * Ki = Ki0 * (1 + ΔKi_fuzzy)
 * Kd = Kd0 * (1 + ΔKd_fuzzy)
 */
class FuzzyPID : public OnlineAutotuner {
public:
    std::string getName() const override { return "Fuzzy PID"; }
    std::string getDescription() const override {
        return "PID with fuzzy gain scheduling. Online adaptation "
               "without system identification.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override { return false; }
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set base PID gains
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     */
    void setBaseGains(double Kp, double Ki, double Kd);
    
    /**
     * @brief Set gain adjustment factors
     * @param alphaKp Maximum relative adjustment applied to Kp.
     * @param alphaKi Maximum relative adjustment applied to Ki.
     * @param alphaKd Maximum relative adjustment applied to Kd.
     */
    void setAdjustmentFactors(double alphaKp, double alphaKi, double alphaKd);
    
    /**
     * @brief Set error scaling
     * @param scale Normalisation applied to the error before fuzzification.
     */
    void setErrorScale(double scale) { m_errorScale = scale; }
    
    /**
     * @brief Get current effective gains
     * @return Gains after the latest fuzzy adjustment.
     */
    PIDGains getCurrentGains() const;
    
    /**
     * @brief Set rule complexity
     */
    enum class RuleComplexity {
        Simple,     ///< 3x3 rule base
        Medium,     ///< 5x5 rule base
        Complex     ///< 7x7 rule base
    };
    void setRuleComplexity(RuleComplexity c) { m_complexity = c; }
    
private:
    double m_Kp0{1}, m_Ki0{0.1}, m_Kd0{0.05};
    double m_alphaKp{0.5}, m_alphaKi{0.5}, m_alphaKd{0.5};
    double m_errorScale{1.0};
    
    RuleComplexity m_complexity{RuleComplexity::Medium};
    
    FuzzyTuning m_fuzzy;
    
    double m_lastError{0};
    bool m_running{false};
};

// ============================================================================
// GA-Tuned PID
// ============================================================================

/**
 * @brief Genetic Algorithm PID Tuning
 * 
 * Uses genetic algorithm to optimize PID parameters.
 * 
 * ## Chromosome
 * [Kp, Ki, Kd] or [Kp, Ti, Td] depending on form
 * 
 * ## Fitness
 * Simulation-based (ITAE, ISE, etc.)
 */
class GAPIDTuning : public OfflineAutotuner {
public:
    std::string getName() const override { return "GA-Tuned PID"; }
    std::string getDescription() const override {
        return "Genetic algorithm optimization of PID. Global search "
               "handles multimodal cost landscapes.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set GA parameters
     * @param n Number of individuals per generation.
     */
    void setPopulationSize(int n) { m_popSize = n; }
    /// @param n Number of generations to evolve.
    void setGenerations(int n) { m_generations = n; }
    /// @param r Per-gene mutation probability.
    void setMutationRate(double r) { m_mutationRate = r; }
    /// @param r Per-pair crossover probability.
    void setCrossoverRate(double r) { m_crossoverRate = r; }
    
    /**
     * @brief Set parameter bounds
     * @param KpMin Lower bound for Kp.
     * @param KpMax Upper bound for Kp.
     * @param KiMin Lower bound for Ki.
     * @param KiMax Upper bound for Ki.
     * @param KdMin Lower bound for Kd.
     * @param KdMax Upper bound for Kd.
     */
    void setBounds(double KpMin, double KpMax,
                  double KiMin, double KiMax,
                  double KdMin, double KdMax);
    
    /**
     * @brief Set cost function
     * @param cost Cost function evaluating each candidate (takes ownership).
     */
    void setCostFunction(std::unique_ptr<CostFunction> cost);
    
    /**
     * @brief Set elitism (% of best to keep)
     * @param fraction Fraction of the best individuals carried over unchanged.
     */
    void setElitism(double fraction) { m_elitism = fraction; }
    
    /**
     * @brief Get evolution history
     * @return Best fitness per generation.
     */
    std::vector<double> getBestFitnessHistory() const { return m_fitnessHistory; }
    
private:
    int m_popSize{50};
    int m_generations{100};
    double m_mutationRate{0.1};
    double m_crossoverRate{0.8};
    double m_elitism{0.1};
    
    double m_KpMin{0}, m_KpMax{100};
    double m_KiMin{0}, m_KiMax{100};
    double m_KdMin{0}, m_KdMax{100};
    
    std::unique_ptr<CostFunction> m_costFunction;
    std::vector<double> m_fitnessHistory;
    
    GeneticAlgorithm m_ga;
};

// ============================================================================
// PSO-Tuned PID
// ============================================================================

/**
 * @brief Particle Swarm Optimization PID Tuning
 * 
 * Uses PSO for global optimization of PID parameters.
 */
class PSOPIDTuning : public OfflineAutotuner {
public:
    std::string getName() const override { return "PSO-Tuned PID"; }
    std::string getDescription() const override {
        return "Particle swarm optimization of PID. Efficient global search "
               "with fast convergence.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set PSO parameters
     * @param n Number of particles in the swarm.
     */
    void setSwarmSize(int n) { m_swarmSize = n; }
    /// @param n Number of swarm iterations.
    void setIterations(int n) { m_iterations = n; }
    /// @param w Inertia weight (velocity retention).
    void setInertia(double w) { m_w = w; }
    /// @param c1 Cognitive (personal-best) acceleration coefficient.
    void setCognitiveCoeff(double c1) { m_c1 = c1; }
    /// @param c2 Social (global-best) acceleration coefficient.
    void setSocialCoeff(double c2) { m_c2 = c2; }
    
    /**
     * @brief Set parameter bounds
     * @param KpMin Lower bound for Kp.
     * @param KpMax Upper bound for Kp.
     * @param KiMin Lower bound for Ki.
     * @param KiMax Upper bound for Ki.
     * @param KdMin Lower bound for Kd.
     * @param KdMax Upper bound for Kd.
     */
    void setBounds(double KpMin, double KpMax,
                  double KiMin, double KiMax,
                  double KdMin, double KdMax);
    
    /**
     * @brief Set cost function
     * @param cost Cost function evaluating each particle (takes ownership).
     */
    void setCostFunction(std::unique_ptr<CostFunction> cost);
    
private:
    int m_swarmSize{30};
    int m_iterations{100};
    double m_w{0.729};
    double m_c1{1.494};
    double m_c2{1.494};
    
    double m_KpMin{0}, m_KpMax{100};
    double m_KiMin{0}, m_KiMax{100};
    double m_KdMin{0}, m_KdMax{100};
    
    std::unique_ptr<CostFunction> m_costFunction;
    
    ParticleSwarmOptimization m_pso;
};

// ============================================================================
// Neural PID
// ============================================================================

/**
 * @brief Neural Network Augmented PID
 * 
 * PID controller with neural network correction term.
 * 
 * ## Structure
 * u = u_pid + u_nn
 * 
 * where:
 * - u_pid: conventional PID output
 * - u_nn: neural network correction
 */
class NeuralPID : public OnlineAutotuner {
public:
    std::string getName() const override { return "Neural PID"; }
    std::string getDescription() const override {
        return "PID with neural network compensation. Learns to "
               "correct for unmodeled dynamics.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    double update(double measured, double reference, 
                 double control, double dt) override;
    bool isComplete() const override;
    void start() override;
    void stop() override;
    TuningResult getIntermediateResult() const override;
    
    /**
     * @brief Set base PID gains
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     */
    void setBaseGains(double Kp, double Ki, double Kd);
    
    /**
     * @brief Set neural network architecture
     * @param hiddenLayers Neuron count per hidden layer.
     */
    void setNetworkArchitecture(const std::vector<int>& hiddenLayers);
    
    /**
     * @brief Set learning rate
     * @param lr Gradient step size for online learning.
     */
    void setLearningRate(double lr) { m_learningRate = lr; }
    
    /**
     * @brief Set NN contribution limit
     * @param limit Maximum magnitude of the NN correction term.
     */
    void setNNLimit(double limit) { m_nnLimit = limit; }
    
    /**
     * @brief Enable/disable online learning
     * @param enable true to keep adapting the network while running.
     */
    void enableOnlineLearning(bool enable) { m_onlineLearning = enable; }
    
    /**
     * @brief Pre-train network on data
     * @param inputs Training input samples.
     * @param targets Target outputs (same length as @p inputs).
     */
    void pretrain(const std::vector<std::vector<double>>& inputs,
                 const std::vector<double>& targets);
    
    /**
     * @brief Get neural network contribution
     * @return Current NN correction term added to the PID output.
     */
    double getNNOutput() const { return m_nnOutput; }
    
private:
    double m_Kp{1}, m_Ki{0.1}, m_Kd{0.05};
    std::vector<int> m_hiddenLayers{10};
    double m_learningRate{0.01};
    double m_nnLimit{0.5};
    bool m_onlineLearning{true};
    
    NeuralNetworkTuning m_nn;
    
    // PID state
    double m_integral{0};
    double m_lastError{0};
    
    double m_nnOutput{0};
    bool m_running{false};
};

// ============================================================================
// Cascade Autotuner
// ============================================================================

/**
 * @brief Cascade Loop Autotuner
 * 
 * Automatic tuning of cascade control loops.
 * Tunes inner loop first, then outer loop.
 */
class CascadeAutotuner : public OfflineAutotuner {
public:
    std::string getName() const override { return "Cascade Autotuner"; }
    std::string getDescription() const override {
        return "Sequential tuning of cascade loops. Inner loop first, "
               "then outer with inner closed.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set inner loop tuning method
     * @param tuner Autotuner applied to the inner (fast) loop.
     */
    void setInnerTuner(std::unique_ptr<OfflineAutotuner> tuner);
    
    /**
     * @brief Set outer loop tuning method
     * @param tuner Autotuner applied to the outer (slow) loop.
     */
    void setOuterTuner(std::unique_ptr<OfflineAutotuner> tuner);
    
    /**
     * @brief Set bandwidth ratio (outer/inner)
     * Typically 0.1-0.2 for good separation
     * @param ratio Outer-loop bandwidth divided by inner-loop bandwidth.
     */
    void setBandwidthRatio(double ratio) { m_bandwidthRatio = ratio; }
    
    /**
     * @brief Get inner loop gains
     * @return Gains produced for the inner loop.
     */
    PIDGains getInnerGains() const { return m_innerGains; }
    
    /**
     * @brief Get outer loop gains
     * @return Gains produced for the outer loop.
     */
    PIDGains getOuterGains() const { return m_outerGains; }
    
private:
    std::unique_ptr<OfflineAutotuner> m_innerTuner;
    std::unique_ptr<OfflineAutotuner> m_outerTuner;
    double m_bandwidthRatio{0.15};
    
    PIDGains m_innerGains;
    PIDGains m_outerGains;
};

// ============================================================================
// Multivariable Tuning
// ============================================================================

/**
 * @brief Decentralized Multivariable Tuning
 * 
 * Tune MIMO system as decoupled SISO loops.
 * Uses RGA for pairing, then individual tuning.
 */
class DecentralizedTuning : public OfflineAutotuner {
public:
    std::string getName() const override { return "Decentralized MIMO Tuning"; }
    std::string getDescription() const override {
        return "MIMO as decoupled SISO loops. Uses RGA analysis "
               "for loop pairing and interaction handling.";
    }
    bool isCompatible(const TunableController& controller) const override { return true; }
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set process gain matrix
     * @param K Steady-state gain matrix (outputs × inputs).
     */
    void setGainMatrix(const std::vector<std::vector<double>>& K);
    
    /**
     * @brief Set time constant matrix
     * @param tau Time constant matrix (outputs × inputs) [s].
     */
    void setTimeConstantMatrix(const std::vector<std::vector<double>>& tau);
    
    /**
     * @brief Set delay matrix
     * @param theta Dead-time matrix (outputs × inputs) [s].
     */
    void setDelayMatrix(const std::vector<std::vector<double>>& theta);
    
    /**
     * @brief Set SISO tuning method
     * @param tuner Autotuner applied to each decoupled loop.
     */
    void setSISOTuner(std::unique_ptr<OfflineAutotuner> tuner);
    
    /**
     * @brief Set detuning factor for interaction
     * @param f Gain reduction applied to counter loop interaction (0-1).
     */
    void setDetuningFactor(double f) { m_detuningFactor = f; }
    
    /**
     * @brief Get Relative Gain Array
     * @return RGA matrix computed from the gain matrix.
     */
    std::vector<std::vector<double>> getRGA() const { return m_RGA; }
    
    /**
     * @brief Get loop pairing
     * @return (output, input) index pairs chosen from the RGA.
     */
    std::vector<std::pair<int, int>> getLoopPairing() const { return m_pairing; }
    
private:
    std::vector<std::vector<double>> m_K, m_tau, m_theta;
    std::unique_ptr<OfflineAutotuner> m_sisoTuner;
    double m_detuningFactor{0.5};
    
    std::vector<std::vector<double>> m_RGA;
    std::vector<std::pair<int, int>> m_pairing;
    
    std::vector<std::vector<double>> computeRGA() const;
    std::vector<std::pair<int, int>> determinesPairing() const;
};

} // namespace Autotuning
} // namespace tether::control
