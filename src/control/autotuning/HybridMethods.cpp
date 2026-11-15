/**
 * @file HybridMethods.cpp
 * @brief Implementation of hybrid controller tuning methods
 */

#include "tether/control/autotuning/HybridMethods.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cctype>

#include <Eigen/Dense>

namespace tether::control {
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

namespace {

/**
 * @brief Concrete FOPDT process model for per-loop decentralized tuning.
 *
 * The DecentralizedTuning gain/time-constant/delay matrices describe one
 * FOPDT loop per (output, input) pair; this adapter exposes a single such
 * loop through the ProcessModel interface.
 */
class FOPDTLoopModel : public ProcessModel {
public:
    explicit FOPDTLoopModel(const FOPDTModel& model) : m_model(model) {}

    ProcessModelType getType() const override { return ProcessModelType::FOPDT; }

    std::complex<double> evaluate(double omega) const override {
        return m_model.evaluate(omega);
    }

    std::vector<std::pair<double, double>> stepResponse(
        double stepMagnitude, double dt, double duration) const override {
        std::vector<std::pair<double, double>> result;
        const int steps = static_cast<int>(duration / dt);
        result.reserve(steps);
        double y = 0.0;
        const int delaySteps = m_model.L > 0.0
            ? static_cast<int>(std::lround(m_model.L / dt)) : 0;
        std::vector<double> delay(delaySteps + 1, 0.0);
        size_t idx = 0;
        const double a = m_model.tau > 0.0
            ? 1.0 - std::exp(-dt / m_model.tau) : 1.0;
        for (int i = 0; i < steps; ++i) {
            double ud = stepMagnitude;
            if (delaySteps > 0) {
                ud = delay[idx];
                delay[idx] = stepMagnitude;
                idx = (idx + 1) % delay.size();
            }
            y += a * (m_model.K * ud - y);
            result.emplace_back(i * dt, y);
        }
        return result;
    }

    FOPDTModel toFOPDT() const override { return m_model; }

    std::pair<double, double> getUltimateParams() const override {
        if (m_model.L <= 0.0 || m_model.K == 0.0) {
            return {1e6, 4.0 * m_model.tau};
        }
        return {m_model.tau / (m_model.K * m_model.L), 4.0 * m_model.L};
    }

private:
    FOPDTModel m_model;
};

/**
 * @brief Minimal SISO PID TunableController ([Kp, Ki, Kd] parameters).
 *
 * Used as the per-loop tuning target inside DecentralizedTuning where no
 * user-supplied controller object exists for each decoupled loop.
 */
class SISO_PID_TunableController : public TunableController {
public:
    ParameterDescriptors getParameterDescriptors() const override {
        return {
            {"Kp", m_Kp, {1e-4, 1e4}, 1.0, false},
            {"Ki", m_Ki, {0.0, 1e4}, 1.0, false},
            {"Kd", m_Kd, {0.0, 1e4}, 1.0, false}
        };
    }

    ParameterVector getParameters() const override {
        return {m_Kp, m_Ki, m_Kd};
    }

    bool setParameters(const ParameterVector& params) override {
        if (params.size() < 3) return false;
        m_Kp = params[0];
        m_Ki = params[1];
        m_Kd = params[2];
        return true;
    }

    std::string getControllerTypeName() const override { return "PID"; }

    std::vector<double> evaluate(
        const std::vector<double>& input,
        const std::vector<double>& reference,
        double dt) override {
        m_integral = 0.0;
        m_prevError = 0.0;
        m_prevDeriv = 0.0;
        std::vector<double> output;
        output.reserve(input.size());
        for (size_t i = 0; i < input.size(); ++i) {
            const double r = i < reference.size() ? reference[i] : 0.0;
            const double e = r - (i < input.size() ? input[i] : 0.0);
            m_integral += e * dt;
            const double deriv = (e - m_prevError) / (dt + 1e-10);
            const double alpha = 0.1;
            const double d = alpha * deriv + (1.0 - alpha) * m_prevDeriv;
            m_prevDeriv = d;
            m_prevError = e;
            output.push_back(m_Kp * e + m_Ki * m_integral + m_Kd * d);
        }
        return output;
    }

    void reset() override {
        m_integral = 0.0;
        m_prevError = 0.0;
        m_prevDeriv = 0.0;
    }

    std::shared_ptr<TunableController> clone() const override {
        auto copy = std::make_shared<SISO_PID_TunableController>();
        copy->setParameters({m_Kp, m_Ki, m_Kd});
        return copy;
    }

private:
    double m_Kp{1.0}, m_Ki{0.1}, m_Kd{0.0};
    double m_integral{0.0}, m_prevError{0.0}, m_prevDeriv{0.0};
};

/** Map the leading entries of a parameter vector onto PIDGains. */
PIDGains gainsFromParams(const ParameterVector& params) {
    PIDGains g;
    if (params.size() > 0) g.Kp = params[0];
    if (params.size() > 1) g.Ki = params[1];
    if (params.size() > 2) g.Kd = params[2];
    if (g.Ti == 0.0 && g.Ki != 0.0 && g.Kp != 0.0) g.Ti = g.Kp / g.Ki;
    if (g.Td == 0.0) g.Td = (g.Kd != 0.0 && g.Kp != 0.0) ? g.Kd / g.Kp : 0.0;
    return g;
}

/** Find the index of the integral-gain parameter by descriptor name. */
int findIntegralParamIndex(const TunableController& controller) {
    const auto descs = controller.getParameterDescriptors();
    for (size_t i = 0; i < descs.size(); ++i) {
        std::string name = descs[i].name;
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (name.find("ki") != std::string::npos ||
            name.find("integral") != std::string::npos ||
            name == "i") {
            return static_cast<int>(i);
        }
    }
    return -1;
}

} // namespace

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
    
    // Inner (fast) loop first. The caller supplies one controller — treated
    // as the outer loop — so the inner loop is tuned on an independent
    // clone with the same parameter structure.
    auto innerController = controller.clone();
    if (!innerController) {
        result.message = "Controller does not support clone(); cannot tune inner loop";
        return result;
    }
    
    TuningResult innerResult = m_innerTuner->tune(*innerController, model);
    if (!innerResult.success) {
        result.message = "Inner loop tuning failed: " + innerResult.message;
        return result;
    }
    m_innerGains = gainsFromParams(innerController->getParameters());
    
    // Outer (slow) loop: tune the caller's controller, then detune its
    // integral gain so the outer bandwidth stays near bandwidthRatio of
    // the inner loop — the standard cascade separation rule.
    TuningResult outerResult = m_outerTuner->tune(controller, model);
    if (!outerResult.success) {
        result.message = "Outer loop tuning failed: " + outerResult.message;
        return result;
    }
    
    ParameterVector outerParams = controller.getParameters();
    const int kiIdx = findIntegralParamIndex(controller);
    if (kiIdx >= 0 && kiIdx < static_cast<int>(outerParams.size()) &&
        m_bandwidthRatio > 0.0 && m_bandwidthRatio < 1.0) {
        outerParams[kiIdx] *= m_bandwidthRatio;
        if (!controller.setParameters(outerParams)) {
            result.message = "Outer tuning succeeded but bandwidth detune rejected";
            return result;
        }
    }
    
    m_outerGains = gainsFromParams(controller.getParameters());
    
    result.parameters = controller.getParameters();
    result.cost = outerResult.cost;
    result.iterations = innerResult.iterations + outerResult.iterations;
    result.functionEvaluations = innerResult.functionEvaluations +
                                 outerResult.functionEvaluations;
    result.success = true;
    result.message = "Cascade tuning complete (inner then bandwidth-detuned outer)";
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

    // RGA = K .* (K^-1)^T  (element-wise product with transpose of inverse)
    Eigen::MatrixXd K(n, n);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            K(i, j) = m_K[i][j];

    // Compute inverse via LU decomposition
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(K);
    if (std::abs(lu.determinant()) < 1e-10) {
        return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
    }

    Eigen::MatrixXd Kinv = lu.inverse();
    Eigen::MatrixXd KinvT = Kinv.transpose();

    std::vector<std::vector<double>> RGA(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            RGA[i][j] = K(i, j) * KinvT(i, j);

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
    (void)model;  // Per-loop models come from the K/tau/theta matrices.
    TuningResult result;
    result.success = false;
    
    if (m_K.empty()) {
        result.message = "Gain matrix must be set";
        return result;
    }
    if (!m_sisoTuner) {
        result.message = "SISO tuner must be set";
        return result;
    }
    
    const size_t n = m_K.size();
    auto square = [&](const std::vector<std::vector<double>>& M) {
        return M.size() == n &&
               std::all_of(M.begin(), M.end(),
                           [n](const auto& row) { return row.size() == n; });
    };
    if (!square(m_tau) || !square(m_theta)) {
        result.message = "Gain, time-constant and delay matrices must be square and equal-sized";
        return result;
    }
    
    // Compute RGA and loop pairing
    m_RGA = computeRGA();
    m_pairing = determinesPairing();
    
    if (m_pairing.empty()) {
        result.message = "RGA pairing produced no loops";
        return result;
    }
    
    // Tune each paired loop as a SISO FOPDT problem. The detuning factor
    // scales the proportional gain to account for residual interaction.
    ParameterVector allParams;
    int tunedLoops = 0;
    for (const auto& [outIdx, inIdx] : m_pairing) {
        if (outIdx < 0 || inIdx < 0 ||
            static_cast<size_t>(outIdx) >= n || static_cast<size_t>(inIdx) >= n) {
            continue;
        }
        
        FOPDTModel loop;
        loop.K = m_K[outIdx][inIdx];
        loop.tau = m_tau[outIdx][inIdx];
        loop.L = m_theta[outIdx][inIdx];
        if (!loop.isValid() || loop.K == 0.0) {
            continue;
        }
        
        FOPDTLoopModel loopModel(loop);
        SISO_PID_TunableController loopController;
        TuningResult loopResult = m_sisoTuner->tune(loopController, &loopModel);
        if (!loopResult.success || loopResult.parameters.size() < 3) {
            continue;
        }
        
        ParameterVector lp = loopResult.parameters;
        lp[0] *= m_detuningFactor;  // detune Kp for loop interaction
        allParams.insert(allParams.end(), lp.begin(), lp.end());
        ++tunedLoops;
        
        result.iterations += loopResult.iterations;
        result.functionEvaluations += loopResult.functionEvaluations;
        result.cost = std::min(result.cost, loopResult.cost);
    }
    
    if (tunedLoops == 0) {
        result.message = "No paired loop could be tuned";
        return result;
    }
    
    result.parameters = allParams;
    result.success = true;
    result.message = "Tuned " + std::to_string(tunedLoops) +
                     " decoupled loops; parameters are concatenated per-loop "
                     "[Kp,Ki,Kd,...] in pairing order";
    
    // Apply only when the controller's parameter vector matches the
    // concatenated layout exactly; otherwise leave it untouched so the
    // caller can distribute the per-loop gains itself.
    if (controller.getParameters().size() == allParams.size() &&
        controller.setParameters(allParams)) {
        result.message += "; applied to controller";
    } else {
        result.message += "; not applied (parameter layout mismatch)";
    }
    
    return result;
}

} // namespace Autotuning
} // namespace tether::control
