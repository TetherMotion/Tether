// ClassicalTuningMethods.cpp
// This file previously contained monolithic implementations of the classical
// tuning methods. The implementations have been moved into separate files
// under src/control/autotuning/classical/. This file is intentionally left
// as a small compatibility stub to avoid duplicate symbol definitions.

#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

namespace Control {
namespace Autotuning {

// Compatibility stub: concrete implementations live in
// src/control/autotuning/classical/*. No definitions here.

} // namespace Autotuning
} // namespace Control


#if 0

TuningResult AstromHagglundRelay::tune(TunableController& controller,
                                       const ProcessModel* model) {
    TuningResult result;
    
    if (m_state != State::Complete) {
        result.success = false;
        result.message = "Relay experiment not complete. Run online update first.";
        return result;
    }
    
    result.parameters = {m_gains.Kp, m_gains.Ki, m_gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Relay feedback tuning successful"
                                    : "Failed to set parameters";
    return result;
}

double AstromHagglundRelay::update(double measured, double reference,
                                    double control, double dt) {
    if (m_state != State::Running) {
        return 0.0;
    }
    
    m_elapsed += dt;
    double error = reference - measured;
    
    // Relay logic with hysteresis
    if (error > m_config.hysteresis) {
        m_relayOutput = m_config.relayAmplitude;
    } else if (error < -m_config.hysteresis) {
        m_relayOutput = -m_config.relayAmplitude;
    }
    
    // Detect zero crossings (sign changes of error)
    if (m_lastError * error < 0) {
        // Zero crossing detected
        if (error > 0 && !m_peakTimes.empty()) {
            // Positive crossing - record valley
            m_valleyValues.push_back(measured);
        } else if (error < 0 && !m_peakTimes.empty()) {
            // Negative crossing - record peak
            m_peakValues.push_back(measured);
            m_peakTimes.push_back(m_elapsed);
            m_cycles++;
        } else if (m_peakTimes.empty()) {
            // First crossing
            m_peakTimes.push_back(m_elapsed);
        }
    }
    
    m_lastError = error;
    
    // Check completion
    if (m_cycles >= m_config.minCycles) {
        // Calculate amplitude and period
        if (m_peakValues.size() >= 2 && m_valleyValues.size() >= 1) {
            double avgPeak = std::accumulate(m_peakValues.begin(), m_peakValues.end(), 0.0) 
                           / m_peakValues.size();
            double avgValley = std::accumulate(m_valleyValues.begin(), m_valleyValues.end(), 0.0)
                             / m_valleyValues.size();
            double amplitude = (avgPeak - avgValley) / 2.0;
            
            // Period from peak times
            double totalPeriod = 0;
            for (size_t i = 1; i < m_peakTimes.size(); ++i) {
                totalPeriod += m_peakTimes[i] - m_peakTimes[i - 1];
            }
            m_Tu = totalPeriod / (m_peakTimes.size() - 1);
            
            // Ku from describing function
            double d = m_config.relayAmplitude;
            m_Ku = 4.0 * d / (M_PI * amplitude);
            
            // Apply tuning rule
            switch (m_rule) {
                case TuningRule::ZieglerNichols:
                    m_gains = ZieglerNicholsUltimateCycle::calculateGains(m_Ku, m_Tu, PIDForm::Parallel);
                    break;
                case TuningRule::TyreusLuyben:
                    m_gains = TyreusLuyben::calculateGains(m_Ku, m_Tu, false);
                    break;
                case TuningRule::AMIGO:
                    // AMIGO based on Ku, Tu
                    m_gains.Kp = 0.35 * m_Ku;
                    m_gains.Ti = 0.85 * m_Tu;
                    m_gains.Td = 0.15 * m_Tu;
                    m_gains.Ki = m_gains.Kp / m_gains.Ti;
                    m_gains.Kd = m_gains.Kp * m_gains.Td;
                    break;
            }
            
            m_state = State::Complete;
        }
    }
    
    if (m_cycles >= m_config.maxCycles) {
        m_state = State::Failed;
    }
    
    return m_relayOutput;
}

bool AstromHagglundRelay::isComplete() const {
    return m_state == State::Complete || m_state == State::Failed;
}

void AstromHagglundRelay::start() {
    m_state = State::Running;
    m_setpoint = 0.0;
    m_relayOutput = m_config.relayAmplitude;
    m_lastError = 0.0;
    m_peakTimes.clear();
    m_peakValues.clear();
    m_valleyValues.clear();
    m_elapsed = 0.0;
    m_cycles = 0;
    m_Ku = 0.0;
    m_Tu = 0.0;
}

void AstromHagglundRelay::stop() {
    m_state = State::Idle;
}

TuningResult AstromHagglundRelay::getIntermediateResult() const {
    TuningResult result;
    result.success = (m_state == State::Complete);
    result.parameters = {m_gains.Kp, m_gains.Ki, m_gains.Kd};
    result.iterations = m_cycles;
    result.message = result.success ? "Relay identification complete" 
                                    : "Relay identification in progress";
    return result;
}

// ============================================================================
// Lopez Method (ITAE/IAE/ISE)
// ============================================================================

bool LopezMethod::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult LopezMethod::tune(TunableController& controller,
                               const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) {
        processModel = model->toFOPDT();
    }
    
    if (!processModel.isValid()) {
        result.message = "Process model required for Lopez method";
        return result;
    }
    
    PIDGains gains = calculateGains(processModel, m_form, m_criterion, m_responseType);
    
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Lopez tuning successful"
                                    : "Failed to set parameters";
    return result;
}

PIDGains LopezMethod::calculateGains(const FOPDTModel& model, PIDForm form,
                                     Criterion criterion, ResponseType response) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    double ratio = L / tau;
    
    if (K == 0 || tau == 0) {
        return gains;
    }
    
    // Lopez correlations: Kp = (1/K) * A * (tau/L)^B
    // Ti = tau / (C * (L/tau)^D)
    // Td = E * tau * (L/tau)^F
    
    double A, B, C, D, E, F;
    
    if (response == ResponseType::Setpoint) {
        switch (criterion) {
            case Criterion::IAE:
                A = 0.758; B = 0.861; C = 1.02; D = -0.323; E = 0.0; F = 0.0;
                break;
            case Criterion::ITAE:
                A = 0.586; B = 0.916; C = 1.03; D = -0.165; E = 0.0; F = 0.0;
                break;
            case Criterion::ISE:
            default:
                A = 1.048; B = 0.897; C = 1.195; D = -0.368; E = 0.0; F = 0.0;
                break;
        }
    } else {  // Disturbance
        switch (criterion) {
            case Criterion::IAE:
                A = 0.984; B = 0.986; C = 0.608; D = 0.707; E = 0.0; F = 0.0;
                break;
            case Criterion::ITAE:
                A = 1.357; B = 0.947; C = 0.842; D = 0.738; E = 0.0; F = 0.0;
                break;
            case Criterion::ISE:
            default:
                A = 1.495; B = 0.945; C = 1.101; D = 0.771; E = 0.0; F = 0.0;
                break;
        }
    }
    
    gains.Kp = (1.0 / K) * A * std::pow(tau / L, B);
    gains.Ti = tau / (C * std::pow(ratio, D));
    
    if (gains.Ti > 0) {
        gains.Ki = gains.Kp / gains.Ti;
    }
    
    return gains;
}

// ============================================================================
// Lambda/IMC Tuning
// ============================================================================

bool LambdaTuning::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult LambdaTuning::tune(TunableController& controller,
                                const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) {
        processModel = model->toFOPDT();
    }
    
    if (!processModel.isValid()) {
        result.message = "Process model required for Lambda tuning";
        return result;
    }
    
    double lambda = m_lambda;
    if (m_useAutoLambda) {
        lambda = m_lambdaFactor * processModel.tau;
    }
    
    PIDGains gains = calculateGains(processModel, lambda, false);
    
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Lambda tuning successful"
                                    : "Failed to set parameters";
    return result;
}

PIDGains LambdaTuning::calculateGains(const FOPDTModel& model, double lambda,
                                      bool includeDerivative) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    if (K == 0) {
        return gains;
    }
    
    // Lambda/IMC rules:
    // Kp = tau / (K * (lambda + L))
    // Ti = tau
    // Td = L/2 (optional)
    
    gains.Kp = tau / (K * (lambda + L));
    gains.Ti = tau;
    
    if (includeDerivative) {
        gains.Td = L / 2.0;
    }
    
    if (gains.Ti > 0) {
        gains.Ki = gains.Kp / gains.Ti;
    }
    gains.Kd = gains.Kp * gains.Td;
    
    return gains;
}

void LambdaTuning::setRobustness(double robustness) {
    // robustness 0.0 = aggressive (lambda = L), 1.0 = conservative (lambda = 3*tau)
    m_useAutoLambda = true;
    m_lambdaFactor = 0.5 + 2.5 * robustness;  // Maps to [0.5, 3.0] * tau
}

// ============================================================================
// SIMC Method
// ============================================================================

bool SIMCMethod::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult SIMCMethod::tune(TunableController& controller,
                              const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) {
        processModel = model->toFOPDT();
    }
    
    if (!processModel.isValid()) {
        result.message = "Process model required for SIMC";
        return result;
    }
    
    PIDGains gains = calculateGains(processModel, m_tauC);
    
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "SIMC tuning successful"
                                    : "Failed to set parameters";
    return result;
}

PIDGains SIMCMethod::calculateGains(const FOPDTModel& model, double tauC) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    if (K == 0) {
        return gains;
    }
    
    // SIMC rules:
    // Kp = tau / (K * (tauC + L))
    // Ti = min(tau, 4*(tauC + L))
    
    gains.Kp = tau / (K * (tauC + L));
    gains.Ti = std::min(tau, 4.0 * (tauC + L));
    
    if (gains.Ti > 0) {
        gains.Ki = gains.Kp / gains.Ti;
    }
    
    return gains;
}

PIDGains SIMCMethod::calculateGains(const SOPDTModel& model, double tauC) {
    PIDGains gains;
    
    double K = model.K;
    double tau1 = model.tau1;
    double tau2 = model.tau2;
    double L = model.L;
    
    if (K == 0) {
        return gains;
    }
    
    // SIMC rules for SOPDT:
    // Kp = tau1 / (K * (tauC + L))
    // Ti = min(tau1, 4*(tauC + L))
    // Td = tau2
    
    gains.Kp = tau1 / (K * (tauC + L));
    gains.Ti = std::min(tau1, 4.0 * (tauC + L));
    gains.Td = tau2;
    
    if (gains.Ti > 0) {
        gains.Ki = gains.Kp / gains.Ti;
    }
    gains.Kd = gains.Kp * gains.Td;
    
    return gains;
}

// ============================================================================
// AMIGO Method
// ============================================================================

bool AMIGOMethod::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult AMIGOMethod::tune(TunableController& controller,
                               const ProcessModel* model) {
    TuningResult result;
    result.success = false;
    
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) {
        processModel = model->toFOPDT();
    }
    
    if (!processModel.isValid()) {
        result.message = "Process model required for AMIGO";
        return result;
    }
    
    PIDGains gains = calculateGains(processModel, PIDForm::Parallel);
    
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "AMIGO tuning successful"
                                    : "Failed to set parameters";
    return result;
}

PIDGains AMIGOMethod::calculateGains(const FOPDTModel& model, PIDForm form) {
    PIDGains gains;
    
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    
    if (K == 0 || tau == 0) {
        return gains;
    }
    
    // AMIGO PI rules (for Ms < 1.4):
    // Kp = (1/K) * (0.15 + (0.35 - L*tau/(L+tau)^2) * (tau/L))
    // Ti = 0.35*L + (13*L*tau^2)/(tau^2 + 12*L*tau + 7*L^2)
    
    // AMIGO PID rules:
    double ratio = tau / L;
    gains.Kp = (1.0/K) * (0.2 + 0.45 * ratio);
    gains.Ti = (0.4 * L + 0.8 * tau) / (L + 0.1 * tau) * L;
    gains.Td = 0.5 * L * tau / (0.3 * L + tau);
    
    if (gains.Ti > 0) {
        gains.Ki = gains.Kp / gains.Ti;
    }
    gains.Kd = gains.Kp * gains.Td;
    
    return gains;
}

// ============================================================================
// Process Identification Utilities
// ============================================================================

FOPDTModel ProcessIdentification::tangentMethod(const std::vector<double>& time,
                                                 const std::vector<double>& response,
                                                 double stepSize) {
    return ZieglerNicholsStepResponse::identifyModel(time, response, stepSize, response[0]);
}

FOPDTModel ProcessIdentification::areaMethod(const std::vector<double>& time,
                                              const std::vector<double>& response,
                                              double stepSize) {
    FOPDTModel model;
    
    if (time.size() < 3) {
        return model;
    }
    
    double y0 = response[0];
    double yFinal = response.back();
    model.K = (yFinal - y0) / stepSize;
    
    // Find time to reach 63.2% of final value
    double target632 = y0 + 0.632 * (yFinal - y0);
    double t632 = time.back();
    
    for (size_t i = 1; i < response.size(); ++i) {
        if (response[i] >= target632) {
            // Linear interpolation
            double frac = (target632 - response[i-1]) / (response[i] - response[i-1]);
            t632 = time[i-1] + frac * (time[i] - time[i-1]);
            break;
        }
    }
    
    // For a pure FOPDT: y reaches 63.2% at t = L + tau
    // Need another point to separate L and tau
    // Using simplified assumption: L ≈ 0.1 * (L + tau)
    double sum = t632;
    model.L = 0.1 * sum;
    model.tau = sum - model.L;
    
    return model;
}

FOPDTModel ProcessIdentification::twoPointMethod(const std::vector<double>& time,
                                                  const std::vector<double>& response,
                                                  double stepSize,
                                                  double t1Fraction,
                                                  double t2Fraction) {
    FOPDTModel model;
    
    if (time.size() < 3) {
        return model;
    }
    
    double y0 = response[0];
    double yFinal = response.back();
    model.K = (yFinal - y0) / stepSize;
    
    double target1 = y0 + t1Fraction * (yFinal - y0);
    double target2 = y0 + t2Fraction * (yFinal - y0);
    
    double t1 = 0, t2 = 0;
    
    for (size_t i = 1; i < response.size(); ++i) {
        if (t1 == 0 && response[i] >= target1) {
            double frac = (target1 - response[i-1]) / (response[i] - response[i-1]);
            t1 = time[i-1] + frac * (time[i] - time[i-1]);
        }
        if (t2 == 0 && response[i] >= target2) {
            double frac = (target2 - response[i-1]) / (response[i] - response[i-1]);
            t2 = time[i-1] + frac * (time[i] - time[i-1]);
            break;
        }
    }
    
    // Two-point method formulas (for 28.3% and 63.2%)
    // tau = 1.5 * (t2 - t1)
    // L = t2 - tau
    model.tau = 1.5 * (t2 - t1);
    model.L = t2 - model.tau;
    if (model.L < 0) model.L = 0;
    
    return model;
}

FOPDTModel ProcessIdentification::leastSquaresFit(const std::vector<double>& time,
                                                   const std::vector<double>& response,
                                                   double stepSize) {
    // Simple iterative fit - minimize MSE between model and data
    FOPDTModel best;
    double y0 = response[0];
    double yFinal = response.back();
    best.K = (yFinal - y0) / stepSize;
    
    // Initial estimate using area method
    best = areaMethod(time, response, stepSize);
    
    double bestError = std::numeric_limits<double>::max();
    
    // Grid search refinement
    for (double tauMult = 0.5; tauMult <= 2.0; tauMult += 0.1) {
        for (double LMult = 0.5; LMult <= 2.0; LMult += 0.1) {
            FOPDTModel trial;
            trial.K = best.K;
            trial.tau = best.tau * tauMult;
            trial.L = best.L * LMult;
            
            // Compute MSE
            double mse = 0;
            for (size_t i = 0; i < time.size(); ++i) {
                double t = time[i];
                double yModel = y0;
                if (t > trial.L) {
                    yModel = y0 + stepSize * trial.K * (1.0 - std::exp(-(t - trial.L) / trial.tau));
                }
                double err = response[i] - yModel;
                mse += err * err;
            }
            mse /= time.size();
            
            if (mse < bestError) {
                bestError = mse;
                best.tau = trial.tau;
                best.L = trial.L;
            }
        }
    }
    
    return best;
}

SOPDTModel ProcessIdentification::identifySOPDT(const std::vector<double>& time,
                                                 const std::vector<double>& response,
                                                 double stepSize) {
    SOPDTModel model;
    
    // First get FOPDT as starting point
    FOPDTModel fopdt = leastSquaresFit(time, response, stepSize);
    
    model.K = fopdt.K;
    model.L = fopdt.L;
    model.tau1 = fopdt.tau * 0.7;
    model.tau2 = fopdt.tau * 0.3;
    
    return model;
}

std::pair<double, double> ProcessIdentification::estimateUltimate(const FOPDTModel& model) {
    // Estimate Ku and Tu from FOPDT model using Nyquist criterion
    // At critical frequency, phase = -180 degrees
    // For FOPDT: phase = -arctan(omega*tau) - omega*L = -pi
    
    // Approximate: omega_c ≈ pi / (2*L) for L/tau < 1
    // Better estimate using iteration
    
    double omega = M_PI / (2.0 * model.L);  // Initial guess
    
    // Newton iteration for better omega_c
    for (int i = 0; i < 10; ++i) {
        double phase = -std::atan(omega * model.tau) - omega * model.L;
        double target = -M_PI;
        double dphase = -model.tau / (1.0 + omega * omega * model.tau * model.tau) - model.L;
        omega = omega - (phase - target) / dphase;
        if (omega < 0) omega = M_PI / (2.0 * model.L);
    }
    
    double Tu = 2.0 * M_PI / omega;
    
    // |G(j*omega_c)| = K / sqrt(1 + omega_c^2 * tau^2)
    double mag = std::abs(model.K) / std::sqrt(1.0 + omega * omega * model.tau * model.tau);
    double Ku = 1.0 / mag;
    
    return {Ku, Tu};
}

// ============================================================================
// Classical Tuning Factory
// ============================================================================

std::unique_ptr<AutotunerBase> ClassicalTuningFactory::create(Method method) {
    switch (method) {
        case Method::ZieglerNicholsStep:
            return std::make_unique<ZieglerNicholsStepResponse>();
        case Method::ZieglerNicholsUltimate:
            return std::make_unique<ZieglerNicholsUltimateCycle>();
        case Method::TyreusLuyben:
            return std::make_unique<TyreusLuyben>();
        case Method::CohenCoon:
            return std::make_unique<CohenCoon>();
        case Method::CHR_SetpointNoOS: {
            auto tuner = std::make_unique<ChienHronesReswick>();
            tuner->setTuningMode(ChienHronesReswick::Mode::SetpointNoOvershoot);
            return tuner;
        }
        case Method::CHR_Setpoint20OS: {
            auto tuner = std::make_unique<ChienHronesReswick>();
            tuner->setTuningMode(ChienHronesReswick::Mode::Setpoint20Overshoot);
            return tuner;
        }
        case Method::CHR_RegulatorNoOS: {
            auto tuner = std::make_unique<ChienHronesReswick>();
            tuner->setTuningMode(ChienHronesReswick::Mode::RegulatorNoOvershoot);
            return tuner;
        }
        case Method::CHR_Regulator20OS: {
            auto tuner = std::make_unique<ChienHronesReswick>();
            tuner->setTuningMode(ChienHronesReswick::Mode::Regulator20Overshoot);
            return tuner;
        }
        case Method::LopezITAE: {
            auto tuner = std::make_unique<LopezMethod>();
            tuner->setCriterion(LopezMethod::Criterion::ITAE);
            return tuner;
        }
        case Method::LopezIAE: {
            auto tuner = std::make_unique<LopezMethod>();
            tuner->setCriterion(LopezMethod::Criterion::IAE);
            return tuner;
        }
        case Method::LopezISE: {
            auto tuner = std::make_unique<LopezMethod>();
            tuner->setCriterion(LopezMethod::Criterion::ISE);
            return tuner;
        }
        case Method::Lambda:
            return std::make_unique<LambdaTuning>();
        case Method::SIMC:
            return std::make_unique<SIMCMethod>();
        case Method::AMIGO:
            return std::make_unique<AMIGOMethod>();
        case Method::RelayFeedback:
            return std::make_unique<AstromHagglundRelay>();
        default:
            return nullptr;
    }
}

std::vector<ClassicalTuningFactory::Method> ClassicalTuningFactory::getAvailableMethods() {
    return {
        Method::ZieglerNicholsStep,
        Method::ZieglerNicholsUltimate,
        Method::TyreusLuyben,
        Method::CohenCoon,
        Method::CHR_SetpointNoOS,
        Method::CHR_Setpoint20OS,
        Method::CHR_RegulatorNoOS,
        Method::CHR_Regulator20OS,
        Method::LopezITAE,
        Method::LopezIAE,
        Method::LopezISE,
        Method::Lambda,
        Method::SIMC,
        Method::AMIGO,
        Method::RelayFeedback
    };
}

std::string ClassicalTuningFactory::getMethodName(Method method) {
    switch (method) {
        case Method::ZieglerNicholsStep: return "Ziegler-Nichols Step Response";
        case Method::ZieglerNicholsUltimate: return "Ziegler-Nichols Ultimate Cycle";
        case Method::TyreusLuyben: return "Tyreus-Luyben";
        case Method::CohenCoon: return "Cohen-Coon";
        case Method::CHR_SetpointNoOS: return "CHR Setpoint 0% OS";
        case Method::CHR_Setpoint20OS: return "CHR Setpoint 20% OS";
        case Method::CHR_RegulatorNoOS: return "CHR Regulator 0% OS";
        case Method::CHR_Regulator20OS: return "CHR Regulator 20% OS";
        case Method::LopezITAE: return "Lopez ITAE";
        case Method::LopezIAE: return "Lopez IAE";
        case Method::LopezISE: return "Lopez ISE";
        case Method::Lambda: return "Lambda/IMC";
        case Method::SIMC: return "SIMC (Skogestad)";
        case Method::AMIGO: return "AMIGO";
        case Method::RelayFeedback: return "Relay Feedback";
        default: return "Unknown";
    }
}

} // namespace Autotuning

#endif
} // namespace Control
