/**
 * @file HybridMethods.cpp
 * @brief Implementation of hybrid controller tuning methods
 */

#include "tether/control/autotuning/HybridMethods.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace Control {
namespace Autotuning {

// ============================================================================
// ZN with Optimization
// ============================================================================

bool ZNWithOptimization::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

void ZNWithOptimization::setCostFunction(std::unique_ptr<CostFunction> cost) {
    m_costFunction = std::move(cost);
}

void ZNWithOptimization::setRelativeBounds(double lower, double upper) {
    m_lowerBound = lower;
    m_upperBound = upper;
}

double ZNWithOptimization::getImprovementRatio() const {
    if (m_initialCost > 0) {
        return (m_initialCost - m_finalCost) / m_initialCost;
    }
    return 0.0;
}

TuningResult ZNWithOptimization::tune(TunableController& controller,
                                       const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    if (!model) {
        result.message = "Process model required";
        return result;
    }
    
    // Step 1: Get initial gains from classical method
    FOPDTModel fopdt = model->toFOPDT();
    
    switch (m_initialMethod) {
        case InitialMethod::ZNStepResponse:
            m_initialGains = ZieglerNicholsStepResponse::calculateGains(fopdt, PIDForm::Parallel);
            break;
        case InitialMethod::ZNUltimateCycle: {
            auto [Ku, Tu] = ProcessIdentification::estimateUltimate(fopdt);
            m_initialGains = ZieglerNicholsUltimateCycle::calculateGains(Ku, Tu, PIDForm::Parallel);
            break;
        }
        case InitialMethod::CohenCoon:
            m_initialGains = CohenCoon::calculateGains(fopdt, PIDForm::Parallel);
            break;
        case InitialMethod::IMC:
            m_initialGains = LambdaTuning::calculateGains(fopdt, fopdt.tau, false);
            break;
    }
    
    if (!m_initialGains.isValid()) {
        result.message = "Initial tuning failed";
        return result;
    }
    
    // Step 2: Set up optimization
    ParameterVector initialParams = {m_initialGains.Kp, m_initialGains.Ki, m_initialGains.Kd};
    
    std::vector<ParameterBounds> bounds(3);
    bounds[0] = {m_lowerBound * initialParams[0], m_upperBound * initialParams[0]};
    bounds[1] = {m_lowerBound * initialParams[1], m_upperBound * initialParams[1]};
    bounds[2] = {0.0, m_upperBound * initialParams[2]};  // Kd can be 0
    
    // Create default optimizer if not set
    if (!m_optimizer) {
        m_optimizer = std::make_unique<NelderMead>();
    }
    
    // Create default cost function if not set
    if (!m_costFunction) {
        result.message = "Cost function required for optimization";
        return result;
    }
    
    // Compute initial cost
    m_initialCost = m_costFunction->evaluate(initialParams);
    
    // Step 3: Optimize
    TerminationCriteria criteria;
    criteria.maxIterations = m_maxIterations;
    m_optimizer->setTerminationCriteria(criteria);
    
    auto optResult = m_optimizer->optimize(*m_costFunction, initialParams, bounds);
    
    // Store results
    m_optimizedGains.Kp = optResult.bestParameters[0];
    m_optimizedGains.Ki = optResult.bestParameters[1];
    m_optimizedGains.Kd = optResult.bestParameters[2];
    m_finalCost = optResult.bestCost;
    
    result.parameters = optResult.bestParameters;
    result.cost = optResult.bestCost;
    result.iterations = optResult.iterations;
    result.functionEvaluations = optResult.functionEvaluations;
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Z-N + Optimization successful" : "Failed to set parameters";
    
    return result;
}

// ============================================================================
// IMC with Relay
// ============================================================================

bool IMCWithRelay::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult IMCWithRelay::tune(TunableController& controller,
                                 const ProcessModel* model) {
    TuningResult result;
    
    if (!isComplete()) {
        result.success = false;
        result.message = "Relay experiment not complete";
        return result;
    }
    
    // Design IMC controller from identified model
    double lambda = m_lambdaFactor * m_identifiedModel.tau;
    PIDGains gains = LambdaTuning::calculateGains(m_identifiedModel, lambda, false);
    
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "IMC+Relay tuning successful" : "Failed to set parameters";
    
    return result;
}

double IMCWithRelay::update(double measured, double reference, double control, double dt) {
    if (!m_running) {
        return 0.0;
    }
    
    double output = m_relayTuner.update(measured, reference, control, dt);
    
    if (m_relayTuner.isComplete()) {
        // Extract model from relay results
        double Ku = m_relayTuner.getUltimateGain();
        double Tu = m_relayTuner.getUltimatePeriod();
        
        // Convert to FOPDT (approximate)
        m_identifiedModel.K = 1.0;  // Normalize
        m_identifiedModel.tau = Tu / 4.0;
        m_identifiedModel.L = Tu / 8.0;
        
        m_phase = Phase::Complete;
    }
    
    return output;
}

bool IMCWithRelay::isComplete() const {
    return m_phase == Phase::Complete;
}

void IMCWithRelay::start() {
    m_running = true;
    m_phase = Phase::RelayTest;
    
    AstromHagglundRelay::Config config;
    config.relayAmplitude = m_relayAmplitude;
    config.hysteresis = m_hysteresis;
    m_relayTuner.setConfig(config);
    m_relayTuner.start();
}

void IMCWithRelay::stop() {
    m_running = false;
    m_relayTuner.stop();
}

TuningResult IMCWithRelay::getIntermediateResult() const {
    TuningResult result;
    result.success = isComplete();
    
    if (result.success) {
        double lambda = m_lambdaFactor * m_identifiedModel.tau;
        PIDGains gains = LambdaTuning::calculateGains(m_identifiedModel, lambda, false);
        result.parameters = {gains.Kp, gains.Ki, gains.Kd};
        result.message = "Model identified, IMC gains computed";
    } else {
        result.message = "Relay identification in progress";
    }
    
    return result;
}

// ============================================================================
// Fuzzy PID
// ============================================================================

bool FuzzyPID::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult FuzzyPID::tune(TunableController& controller,
                            const ProcessModel* model) {
    TuningResult result;
    
    // Fuzzy PID is online - just return current gains
    auto gains = getCurrentGains();
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = "Fuzzy PID gains applied";
    
    return result;
}

double FuzzyPID::update(double measured, double reference, double control, double dt) {
    if (!m_running) {
        return 0.0;
    }
    
    double error = reference - measured;
    double errorRate = (error - m_lastError) / dt;
    m_lastError = error;
    
    // Use the fuzzy tuner's update method to compute adjustments
    double fuzzyOutput = m_fuzzy.update(measured, reference, control, dt);
    
    // Apply fuzzy adjustments (simplified - using fuzzy output as a scaling factor)
    double scale = 1.0 + m_alphaKp * (error * m_errorScale);
    
    double Kp = m_Kp0 * scale;
    double Ki = m_Ki0 * (1.0 + m_alphaKi * std::tanh(error * m_errorScale));
    double Kd = m_Kd0 * (1.0 + m_alphaKd * std::tanh(errorRate * m_errorScale));
    
    // PID output (not actually used - just for interface compliance)
    return Kp * error + Ki * error * dt + Kd * errorRate + fuzzyOutput;
}

void FuzzyPID::start() {
    m_running = true;
    m_lastError = 0.0;
}

void FuzzyPID::stop() {
    m_running = false;
}

TuningResult FuzzyPID::getIntermediateResult() const {
    TuningResult result;
    auto gains = getCurrentGains();
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = true;
    result.message = "Current fuzzy PID gains";
    return result;
}

void FuzzyPID::setBaseGains(double Kp, double Ki, double Kd) {
    m_Kp0 = Kp;
    m_Ki0 = Ki;
    m_Kd0 = Kd;
}

void FuzzyPID::setAdjustmentFactors(double alphaKp, double alphaKi, double alphaKd) {
    m_alphaKp = alphaKp;
    m_alphaKi = alphaKi;
    m_alphaKd = alphaKd;
}

PIDGains FuzzyPID::getCurrentGains() const {
    PIDGains gains;
    gains.Kp = m_Kp0;
    gains.Ki = m_Ki0;
    gains.Kd = m_Kd0;
    return gains;
}

// ============================================================================
// GA-Tuned PID
// ============================================================================

bool GAPIDTuning::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

void GAPIDTuning::setBounds(double KpMin, double KpMax,
                            double KiMin, double KiMax,
                            double KdMin, double KdMax) {
    m_KpMin = KpMin; m_KpMax = KpMax;
    m_KiMin = KiMin; m_KiMax = KiMax;
    m_KdMin = KdMin; m_KdMax = KdMax;
}

void GAPIDTuning::setCostFunction(std::unique_ptr<CostFunction> cost) {
    m_costFunction = std::move(cost);
}

TuningResult GAPIDTuning::tune(TunableController& controller,
                               const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    if (!m_costFunction) {
        result.message = "Cost function required";
        return result;
    }
    
    // Set up bounds
    std::vector<ParameterBounds> bounds = {
        {m_KpMin, m_KpMax},
        {m_KiMin, m_KiMax},
        {m_KdMin, m_KdMax}
    };
    
    // Configure GA
    m_ga.setPopulationSize(m_popSize);
    m_ga.setCrossoverRate(m_crossoverRate);
    m_ga.setMutationRate(m_mutationRate);
    m_ga.setEliteCount(static_cast<int>(m_elitism * m_popSize));
    
    TerminationCriteria criteria;
    criteria.maxIterations = m_generations;
    m_ga.setTerminationCriteria(criteria);
    
    // Track history
    m_fitnessHistory.clear();
    m_ga.setTrackHistory(true);
    
    // Initial parameters
    ParameterVector initial = {
        (m_KpMin + m_KpMax) / 2,
        (m_KiMin + m_KiMax) / 2,
        (m_KdMin + m_KdMax) / 2
    };
    
    // Run GA
    auto optResult = m_ga.optimize(*m_costFunction, initial, bounds);
    m_fitnessHistory = optResult.costHistory;
    
    result.parameters = optResult.bestParameters;
    result.cost = optResult.bestCost;
    result.iterations = optResult.iterations;
    result.functionEvaluations = optResult.functionEvaluations;
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "GA optimization successful" : "Failed to set parameters";
    
    return result;
}

// ============================================================================
// PSO-Tuned PID
// ============================================================================

bool PSOPIDTuning::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

void PSOPIDTuning::setBounds(double KpMin, double KpMax,
                             double KiMin, double KiMax,
                             double KdMin, double KdMax) {
    m_KpMin = KpMin; m_KpMax = KpMax;
    m_KiMin = KiMin; m_KiMax = KiMax;
    m_KdMin = KdMin; m_KdMax = KdMax;
}

void PSOPIDTuning::setCostFunction(std::unique_ptr<CostFunction> cost) {
    m_costFunction = std::move(cost);
}

TuningResult PSOPIDTuning::tune(TunableController& controller,
                                const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    if (!m_costFunction) {
        result.message = "Cost function required";
        return result;
    }
    
    // Set up bounds
    std::vector<ParameterBounds> bounds = {
        {m_KpMin, m_KpMax},
        {m_KiMin, m_KiMax},
        {m_KdMin, m_KdMax}
    };
    
    // Configure PSO
    m_pso.setSwarmSize(m_swarmSize);
    m_pso.setInertiaWeight(m_w);
    m_pso.setCognitiveCoeff(m_c1);
    m_pso.setSocialCoeff(m_c2);
    
    TerminationCriteria criteria;
    criteria.maxIterations = m_iterations;
    m_pso.setTerminationCriteria(criteria);
    
    // Initial parameters
    ParameterVector initial = {
        (m_KpMin + m_KpMax) / 2,
        (m_KiMin + m_KiMax) / 2,
        (m_KdMin + m_KdMax) / 2
    };
    
    // Run PSO
    auto optResult = m_pso.optimize(*m_costFunction, initial, bounds);
    
    result.parameters = optResult.bestParameters;
    result.cost = optResult.bestCost;
    result.iterations = optResult.iterations;
    result.functionEvaluations = optResult.functionEvaluations;
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "PSO optimization successful" : "Failed to set parameters";
    
    return result;
}

// ============================================================================
// Neural PID
// ============================================================================

bool NeuralPID::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult NeuralPID::tune(TunableController& controller,
                             const ProcessModel* model) {
    TuningResult result;
    
    // Return current base gains
    result.parameters = {m_Kp, m_Ki, m_Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = "Neural PID base gains applied";
    
    return result;
}

double NeuralPID::update(double measured, double reference, double control, double dt) {
    if (!m_running) {
        return 0.0;
    }
    
    double error = reference - measured;
    double errorRate = (error - m_lastError) / dt;
    m_integral += error * dt;
    m_lastError = error;
    
    // PID output
    double uPID = m_Kp * error + m_Ki * m_integral + m_Kd * errorRate;
    
    // Neural network correction - use the NN tuner's update method
    double nnCorrection = m_nn.update(measured, reference, control, dt);
    
    // Limit NN contribution
    m_nnOutput = std::max(-m_nnLimit, std::min(m_nnLimit, nnCorrection));
    
    return uPID + m_nnOutput;
}

bool NeuralPID::isComplete() const {
    return false;  // Online learning never completes
}

void NeuralPID::start() {
    m_running = true;
    m_integral = 0.0;
    m_lastError = 0.0;
    m_nnOutput = 0.0;
}

void NeuralPID::stop() {
    m_running = false;
}

TuningResult NeuralPID::getIntermediateResult() const {
    TuningResult result;
    result.parameters = {m_Kp, m_Ki, m_Kd};
    result.success = true;
    result.message = "Current Neural PID state";
    return result;
}

void NeuralPID::setBaseGains(double Kp, double Ki, double Kd) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

void NeuralPID::setNetworkArchitecture(const std::vector<int>& hiddenLayers) {
    m_hiddenLayers = hiddenLayers;
}

void NeuralPID::pretrain(const std::vector<std::vector<double>>& inputs,
                         const std::vector<double>& targets) {
    // Pre-training would go here
}

// ============================================================================
// Cascade Autotuner
// ============================================================================

bool CascadeAutotuner::isCompatible(const TunableController& controller) const {
    return true;  // Cascade works with any controller
}

void CascadeAutotuner::setInnerTuner(std::unique_ptr<OfflineAutotuner> tuner) {
    m_innerTuner = std::move(tuner);
}

void CascadeAutotuner::setOuterTuner(std::unique_ptr<OfflineAutotuner> tuner) {
    m_outerTuner = std::move(tuner);
}

TuningResult CascadeAutotuner::tune(TunableController& controller,
                                     const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    if (!m_innerTuner || !m_outerTuner) {
        result.message = "Both inner and outer tuners must be set";
        return result;
    }
    
    // This is a placeholder - actual cascade tuning requires
    // access to both inner and outer loops
    result.message = "Cascade tuning requires separate inner/outer controllers";
    return result;
}

// ============================================================================
// Decentralized Tuning
// ============================================================================

void DecentralizedTuning::setGainMatrix(const std::vector<std::vector<double>>& K) {
    m_K = K;
}

void DecentralizedTuning::setTimeConstantMatrix(const std::vector<std::vector<double>>& tau) {
    m_tau = tau;
}

void DecentralizedTuning::setDelayMatrix(const std::vector<std::vector<double>>& theta) {
    m_theta = theta;
}

void DecentralizedTuning::setSISOTuner(std::unique_ptr<OfflineAutotuner> tuner) {
    m_sisoTuner = std::move(tuner);
}

std::vector<std::vector<double>> DecentralizedTuning::computeRGA() const {
    size_t n = m_K.size();
    if (n == 0) return {};
    
    std::vector<std::vector<double>> RGA(n, std::vector<double>(n, 0.0));
    
    // RGA = K .* (K^-1)^T (element-wise product with transpose of inverse)
    // Simplified: for 2x2, lambda_ij = K_ij / K_ii * K_jj / det(K)
    
    if (n == 2) {
        double det = m_K[0][0] * m_K[1][1] - m_K[0][1] * m_K[1][0];
        if (std::abs(det) > 1e-10) {
            RGA[0][0] = m_K[0][0] * m_K[1][1] / det;
            RGA[0][1] = -m_K[0][1] * m_K[1][0] / det;
            RGA[1][0] = -m_K[1][0] * m_K[0][1] / det;
            RGA[1][1] = m_K[1][1] * m_K[0][0] / det;
        }
    }
    
    return RGA;
}

std::vector<std::pair<int, int>> DecentralizedTuning::determinesPairing() const {
    std::vector<std::pair<int, int>> pairing;
    size_t n = m_RGA.size();
    
    // Simple pairing: choose diagonal if RGA close to 1
    for (size_t i = 0; i < n; ++i) {
        double maxRGA = 0;
        int bestJ = 0;
        for (size_t j = 0; j < n; ++j) {
            if (m_RGA[i][j] > maxRGA) {
                maxRGA = m_RGA[i][j];
                bestJ = j;
            }
        }
        pairing.push_back({static_cast<int>(i), bestJ});
    }
    
    return pairing;
}

TuningResult DecentralizedTuning::tune(TunableController& controller,
                                        const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    if (m_K.empty()) {
        result.message = "Gain matrix must be set";
        return result;
    }
    
    // Compute RGA
    m_RGA = computeRGA();
    m_pairing = determinesPairing();
    
    // This is a placeholder - full implementation would tune each loop
    result.message = "Decentralized tuning computed RGA and pairing";
    result.success = true;
    
    return result;
}

} // namespace Autotuning
} // namespace Control
