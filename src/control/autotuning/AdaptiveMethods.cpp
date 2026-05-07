/**
 * @file AdaptiveMethods.cpp
 * @brief Implementation of adaptive and self-tuning control methods
 */

#include "tether/control/autotuning/AdaptiveMethods.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace Control {
namespace Autotuning {

// ============================================================================
// GainScheduler Implementation
// ============================================================================

TuningResult GainScheduler::tune(TunableController& controller,
                                 const ProcessModel* /*model*/) {
    TuningResult result;
    result.success = true;
    result.parameters = m_currentGains;
    result.message = "Gain scheduler initialized";
    return result;
}

double GainScheduler::update(double measured, double reference,
                             double /*control*/, double dt) {
    // Compute scheduling variable
    double schedVar = 0.0;
    if (m_scheduleFunc) {
        schedVar = m_scheduleFunc(measured, reference);
    } else {
        schedVar = measured;  // Default: use measured value
    }
    
    // Get target gains
    ParameterVector targetGains;
    if (m_useFunctionMap && m_gainFunction) {
        targetGains = m_gainFunction(schedVar);
    } else if (!m_gainTable.empty()) {
        // Table lookup with optional interpolation
        auto upper = m_gainTable.lower_bound(schedVar);
        
        if (upper == m_gainTable.end()) {
            targetGains = m_gainTable.rbegin()->second;
        } else if (upper == m_gainTable.begin()) {
            targetGains = upper->second;
        } else if (m_interpolate) {
            auto lower = std::prev(upper);
            double t = (schedVar - lower->first) / (upper->first - lower->first + 1e-10);
            t = std::clamp(t, 0.0, 1.0);
            
            // Linear interpolation
            targetGains.resize(std::min(lower->second.size(), upper->second.size()));
            for (size_t i = 0; i < targetGains.size(); ++i) {
                targetGains[i] = lower->second[i] + t * (upper->second[i] - lower->second[i]);
            }
        } else {
            // Nearest neighbor
            auto lower = std::prev(upper);
            if (schedVar - lower->first < upper->first - schedVar) {
                targetGains = lower->second;
            } else {
                targetGains = upper->second;
            }
        }
    }
    
    // Apply rate limiting
    if (!m_currentGains.empty() && m_rateLimit < std::numeric_limits<double>::max()) {
        double maxChange = m_rateLimit * dt;
        for (size_t i = 0; i < std::min(m_currentGains.size(), targetGains.size()); ++i) {
            double delta = targetGains[i] - m_currentGains[i];
            if (std::abs(delta) > maxChange) {
                targetGains[i] = m_currentGains[i] + std::copysign(maxChange, delta);
            }
        }
    }
    
    m_currentGains = targetGains;
    m_lastScheduleVar = schedVar;
    
    return schedVar;
}

TuningResult GainScheduler::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = m_currentGains;
    return result;
}

void GainScheduler::addGainSet(double operatingPoint, const ParameterVector& gains) {
    m_gainTable[operatingPoint] = gains;
    m_useFunctionMap = false;
}

void GainScheduler::setGainFunction(std::function<ParameterVector(double)> func) {
    m_gainFunction = std::move(func);
    m_useFunctionMap = true;
}

void GainScheduler::setSchedulingVariable(std::function<double(double, double)> func) {
    m_scheduleFunc = std::move(func);
}

// ============================================================================
// MRAC Implementation
// ============================================================================

bool MRAC::isCompatible(const TunableController& controller) const {
    return controller.getParameters().size() >= 2;
}

TuningResult MRAC::tune(TunableController& controller,
                        const ProcessModel* /*model*/) {
    // Initialize parameters from controller
    m_theta = controller.getParameters();
    
    // Initialize adaptation gains if not set
    if (m_gamma.empty()) {
        m_gamma.resize(m_theta.size(), 0.1);
    }
    
    TuningResult result;
    result.success = true;
    result.parameters = m_theta;
    result.message = "MRAC initialized - requires online adaptation";
    return result;
}

double MRAC::update(double measured, double reference,
                    double /*control*/, double dt) {
    if (!m_running) return 0.0;
    
    // Update reference model
    if (m_secondOrderRef) {
        // Second-order reference model
        double ym2Dot = m_Km * m_wn * m_wn * reference 
                       - 2.0 * m_zeta * m_wn * m_ymDot 
                       - m_wn * m_wn * m_ym;
        m_ymDot += ym2Dot * dt;
        m_ym += m_ymDot * dt;
    } else {
        // First-order reference model
        double ymDot = (m_Km * reference - m_ym) / m_taum;
        m_ym += ymDot * dt;
    }
    
    // Tracking error
    m_em = measured - m_ym;
    
    // Build regressor
    m_phi = {m_em, reference - measured, measured};
    
    // Parameter adaptation based on selected law
    for (size_t i = 0; i < m_theta.size() && i < m_phi.size() && i < m_gamma.size(); ++i) {
        double gradTerm = 0.0;
        
        switch (m_law) {
            case AdaptationLaw::MIT:
                gradTerm = m_gamma[i] * m_em * m_phi[i];
                break;
                
            case AdaptationLaw::NormalizedMIT: {
                double normSq = 0.0;
                for (double p : m_phi) normSq += p * p;
                gradTerm = m_gamma[i] * m_em * m_phi[i] / (1.0 + normSq);
                break;
            }
            
            case AdaptationLaw::Lyapunov:
                gradTerm = m_gamma[i] * m_em * m_phi[i];
                break;
        }
        
        // Sigma modification for robustness
        double sigmaTerm = m_sigma * m_gamma[i] * m_theta[i];
        
        // Update parameter
        m_theta[i] -= (gradTerm + sigmaTerm) * dt;
        
        // Parameter projection (bounds)
        if (i < m_paramBounds.size()) {
            m_theta[i] = std::clamp(m_theta[i], 
                                    m_paramBounds[i].min, 
                                    m_paramBounds[i].max);
        }
    }
    
    return m_em;
}

bool MRAC::isComplete() const {
    return std::abs(m_em) < 0.01;
}

void MRAC::start() {
    m_running = true;
    m_ym = 0.0;
    m_ymDot = 0.0;
    m_em = 0.0;
}

void MRAC::stop() {
    m_running = false;
}

TuningResult MRAC::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = m_theta;
    return result;
}

void MRAC::setReferenceModel(double Km, double taum) {
    m_Km = Km;
    m_taum = taum;
    m_secondOrderRef = false;
}

void MRAC::setReferenceModel(double Km, double wn, double zeta) {
    m_Km = Km;
    m_wn = wn;
    m_zeta = zeta;
    m_secondOrderRef = true;
}

void MRAC::setAdaptationGain(const std::vector<double>& gamma) {
    m_gamma = gamma;
}

void MRAC::setParameterBounds(const std::vector<ParameterBounds>& bounds) {
    m_paramBounds = bounds;
}

// ============================================================================
// SelfTuningRegulator Implementation
// ============================================================================

bool SelfTuningRegulator::isCompatible(const TunableController& controller) const {
    return controller.getParameters().size() >= 2;
}

TuningResult SelfTuningRegulator::tune(TunableController& /*controller*/,
                                       const ProcessModel* /*model*/) {
    // Initialize estimation
    m_aHat.resize(m_na, 0.0);
    m_bHat.resize(m_nb, 0.0);
    if (m_estimator == EstimationMethod::ELS) {
        m_cHat.resize(m_na, 0.0);
    }
    
    // Initialize covariance matrix
    int nParams = m_na + m_nb;
    if (m_estimator == EstimationMethod::ELS) {
        nParams += m_na;
    }
    m_P.resize(nParams, std::vector<double>(nParams, 0.0));
    for (int i = 0; i < nParams; ++i) {
        m_P[i][i] = 1000.0;
    }
    
    // Initialize data buffers
    m_yBuffer.clear();
    m_uBuffer.clear();
    m_eBuffer.clear();
    m_yBuffer.resize(m_na + 1, 0.0);
    m_uBuffer.resize(m_nb + m_nk, 0.0);
    m_eBuffer.resize(m_na + 1, 0.0);
    
    TuningResult result;
    result.success = true;
    result.message = "STR initialized - collecting data for identification";
    return result;
}

double SelfTuningRegulator::update(double measured, double /*reference*/,
                                   double control, double /*dt*/) {
    if (!m_running) return 0.0;
    
    // Update buffers
    m_yBuffer.push_front(measured);
    if (m_yBuffer.size() > static_cast<size_t>(m_na + 1)) {
        m_yBuffer.pop_back();
    }
    
    m_uBuffer.push_front(control);
    if (m_uBuffer.size() > static_cast<size_t>(m_nb + m_nk)) {
        m_uBuffer.pop_back();
    }
    
    // Perform estimation update
    updateEstimate(measured, control);
    
    // Periodically update controller
    m_sampleCount++;
    if (m_sampleCount >= m_updateInterval) {
        updateController();
        m_sampleCount = 0;
    }
    
    // Return prediction error
    std::vector<double> phi = buildRegressor();
    double yPred = 0.0;
    for (size_t i = 0; i < m_aHat.size() && i < phi.size(); ++i) {
        yPred -= m_aHat[i] * phi[i];
    }
    for (size_t i = 0; i < m_bHat.size(); ++i) {
        if (m_aHat.size() + i < phi.size()) {
            yPred += m_bHat[i] * phi[m_aHat.size() + i];
        }
    }
    
    double error = measured - yPred;
    m_eBuffer.push_front(error);
    if (m_eBuffer.size() > static_cast<size_t>(m_na + 1)) {
        m_eBuffer.pop_back();
    }
    
    return error;
}

bool SelfTuningRegulator::isComplete() const {
    return false;
}

void SelfTuningRegulator::start() {
    m_running = true;
    m_sampleCount = 0;
}

void SelfTuningRegulator::stop() {
    m_running = false;
}

TuningResult SelfTuningRegulator::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = m_currentGains;
    return result;
}

void SelfTuningRegulator::setModelStructure(int na, int nb, int nk) {
    m_na = na;
    m_nb = nb;
    m_nk = nk;
}

double SelfTuningRegulator::getEstimationCovariance() const {
    if (m_P.empty()) return 0.0;
    
    double trace = 0.0;
    for (size_t i = 0; i < m_P.size(); ++i) {
        trace += m_P[i][i];
    }
    return trace / m_P.size();
}

void SelfTuningRegulator::updateEstimate(double y, double /*u*/) {
    std::vector<double> phi = buildRegressor();
    if (phi.empty()) return;
    
    // Prediction
    double yPred = 0.0;
    for (size_t i = 0; i < m_aHat.size() && i < phi.size(); ++i) {
        yPred -= m_aHat[i] * phi[i];
    }
    for (size_t i = 0; i < m_bHat.size(); ++i) {
        if (m_aHat.size() + i < phi.size()) {
            yPred += m_bHat[i] * phi[m_aHat.size() + i];
        }
    }
    
    double error = y - yPred;
    
    // RLS update
    std::vector<double> Pphi(m_P.size(), 0.0);
    for (size_t i = 0; i < m_P.size() && i < phi.size(); ++i) {
        for (size_t j = 0; j < m_P[i].size() && j < phi.size(); ++j) {
            Pphi[i] += m_P[i][j] * phi[j];
        }
    }
    
    double phiPphi = 0.0;
    for (size_t i = 0; i < phi.size() && i < Pphi.size(); ++i) {
        phiPphi += phi[i] * Pphi[i];
    }
    
    double denom = m_lambda + phiPphi;
    if (std::abs(denom) < 1e-10) return;
    
    std::vector<double> K(Pphi.size());
    for (size_t i = 0; i < Pphi.size(); ++i) {
        K[i] = Pphi[i] / denom;
    }
    
    // Update parameters
    for (size_t i = 0; i < m_aHat.size() && i < K.size(); ++i) {
        m_aHat[i] += K[i] * error;
    }
    for (size_t i = 0; i < m_bHat.size(); ++i) {
        if (m_aHat.size() + i < K.size()) {
            m_bHat[i] += K[m_aHat.size() + i] * error;
        }
    }
    
    // Update covariance
    std::vector<std::vector<double>> KphiP(m_P.size(), std::vector<double>(m_P.size(), 0.0));
    for (size_t i = 0; i < m_P.size() && i < K.size(); ++i) {
        for (size_t j = 0; j < m_P.size() && j < phi.size(); ++j) {
            for (size_t k = 0; k < m_P.size(); ++k) {
                if (j < m_P[k].size()) {
                    KphiP[i][k] += K[i] * phi[j] * m_P[j][k];
                }
            }
        }
    }
    
    for (size_t i = 0; i < m_P.size(); ++i) {
        for (size_t j = 0; j < m_P[i].size(); ++j) {
            m_P[i][j] = (m_P[i][j] - KphiP[i][j]) / m_lambda;
        }
    }
}

void SelfTuningRegulator::updateController() {
    if (m_bHat.empty() || std::abs(m_bHat[0]) < 1e-10) return;
    
    double aSum = 1.0;
    for (double a : m_aHat) aSum += a;
    double bSum = 0.0;
    for (double b : m_bHat) bSum += b;
    double K = bSum / (aSum + 1e-10);
    
    double tau = 1.0;
    if (m_aHat.size() >= 1 && std::abs(m_aHat[0]) < 1.0) {
        tau = 1.0 / (1.0 - m_aHat[0] + 1e-10);
    }
    
    double L = m_nk * 0.1;
    double lambda = std::max(0.5 * tau, 2.0 * L);
    double Kp = tau / (K * (lambda + L));
    double Ti = tau;
    double Td = 0.0;
    
    m_currentGains = {Kp, Kp / Ti, Kp * Td};
}

std::vector<double> SelfTuningRegulator::buildRegressor() const {
    std::vector<double> phi;
    
    for (int i = 1; i <= m_na && i < static_cast<int>(m_yBuffer.size()); ++i) {
        phi.push_back(m_yBuffer[i]);
    }
    
    for (int i = m_nk; i < m_nk + m_nb && i < static_cast<int>(m_uBuffer.size()); ++i) {
        phi.push_back(m_uBuffer[i]);
    }
    
    if (m_estimator == EstimationMethod::ELS) {
        for (int i = 1; i <= m_na && i < static_cast<int>(m_eBuffer.size()); ++i) {
            phi.push_back(m_eBuffer[i]);
        }
    }
    
    return phi;
}

// ============================================================================
// ExtremumSeekingControl Implementation
// ============================================================================

TuningResult ExtremumSeekingControl::tune(TunableController& controller,
                                          const ProcessModel* /*model*/) {
    m_theta = controller.getParameters();
    
    m_xi.resize(m_theta.size(), 0.0);
    m_hpfState.resize(m_theta.size(), 0.0);
    
    m_phases.resize(m_theta.size());
    for (size_t i = 0; i < m_phases.size(); ++i) {
        m_phases[i] = 2.0 * M_PI * i / m_phases.size();
    }
    
    TuningResult result;
    result.success = true;
    result.parameters = m_theta;
    return result;
}

double ExtremumSeekingControl::update(double measured, double reference,
                                      double /*control*/, double dt) {
    if (!m_running) return 0.0;
    
    m_time += dt;
    
    double cost = 0.0;
    if (m_costFunc) {
        cost = m_costFunc(measured, reference);
    } else {
        cost = (measured - reference) * (measured - reference);
    }
    
    double alpha_hp = dt * m_filterCutoff / (1.0 + dt * m_filterCutoff);
    
    if (m_multiParam) {
        for (size_t i = 0; i < m_theta.size(); ++i) {
            double hpf = alpha_hp * (cost - m_hpfState[i]) + (1.0 - alpha_hp) * m_xi[i];
            m_hpfState[i] = cost;
            
            double demod = hpf * std::sin(m_omega * m_time + m_phases[i]);
            m_xi[i] = (1.0 - alpha_hp) * m_xi[i] + alpha_hp * demod;
            m_theta[i] -= m_k * m_xi[i] * dt;
        }
    } else {
        double hpf = alpha_hp * (cost - m_hpfState[0]) + (1.0 - alpha_hp) * m_xi[0];
        m_hpfState[0] = cost;
        
        double demod = hpf * std::sin(m_omega * m_time);
        m_xi[0] = (1.0 - alpha_hp) * m_xi[0] + alpha_hp * demod;
        
        if (!m_theta.empty()) {
            m_theta[0] -= m_k * m_xi[0] * dt;
        }
    }
    
    return cost;
}

void ExtremumSeekingControl::start() {
    m_running = true;
    m_time = 0.0;
    
    // Ensure state vectors are initialized
    if (m_theta.empty()) {
        m_theta.resize(1, 0.0);
    }
    if (m_xi.empty() || m_xi.size() != m_theta.size()) {
        m_xi.resize(m_theta.size(), 0.0);
    }
    if (m_hpfState.empty() || m_hpfState.size() != m_theta.size()) {
        m_hpfState.resize(m_theta.size(), 0.0);
    }
    if (m_phases.empty() || m_phases.size() != m_theta.size()) {
        m_phases.resize(m_theta.size());
        for (size_t i = 0; i < m_phases.size(); ++i) {
            m_phases[i] = 2.0 * M_PI * i / m_phases.size();
        }
    }
}

void ExtremumSeekingControl::stop() {
    m_running = false;
}

TuningResult ExtremumSeekingControl::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = m_theta;
    return result;
}

void ExtremumSeekingControl::setPerturbation(double amplitude, double frequency) {
    m_amplitude = amplitude;
    m_omega = frequency;
}

void ExtremumSeekingControl::setCostFunction(std::function<double(double, double)> cost) {
    m_costFunc = std::move(cost);
}

void ExtremumSeekingControl::setNumParameters(int n) {
    m_theta.resize(n, 0.0);
    m_xi.resize(n, 0.0);
    m_hpfState.resize(n, 0.0);
    m_phases.resize(n);
    for (int i = 0; i < n; ++i) {
        m_phases[i] = 2.0 * M_PI * i / n;
    }
}

// ============================================================================
// FuzzyTuning Implementation
// ============================================================================

bool FuzzyTuning::isCompatible(const TunableController& controller) const {
    return controller.getParameters().size() >= 3;
}

TuningResult FuzzyTuning::tune(TunableController& /*controller*/,
                               const ProcessModel* /*model*/) {
    TuningResult result;
    result.success = true;
    result.parameters = {m_Kp0, m_Ki0, m_Kd0};
    return result;
}

double FuzzyTuning::update(double measured, double reference,
                           double /*control*/, double dt) {
    if (!m_running) return 0.0;
    
    double error = reference - measured;
    double errorRate = (error - m_lastError) / dt;
    m_lastError = error;
    
    (void)errorRate;  // Used in full fuzzy inference
    
    double dKp = 0.0, dKi = 0.0, dKd = 0.0;
    defuzzify(dKp, dKi, dKd);
    
    return error;
}

void FuzzyTuning::start() {
    m_running = true;
    m_lastError = 0.0;
}

void FuzzyTuning::stop() {
    m_running = false;
}

TuningResult FuzzyTuning::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = {m_Kp0, m_Ki0, m_Kd0};
    return result;
}

double FuzzyTuning::MembershipFunction::evaluate(double x) const {
    switch (type) {
        case Triangular:
            if (x <= a || x >= c) return 0.0;
            if (x <= b) return (x - a) / (b - a + 1e-10);
            return (c - x) / (c - b + 1e-10);
            
        case Gaussian:
            return std::exp(-0.5 * (x - a) * (x - a) / (b * b + 1e-10));
            
        case Trapezoidal:
            if (x <= a || x >= d) return 0.0;
            if (x >= b && x <= c) return 1.0;
            if (x < b) return (x - a) / (b - a + 1e-10);
            return (d - x) / (d - c + 1e-10);
            
        default:
            return 0.0;
    }
}

void FuzzyTuning::setErrorRange(double min, double max) {
    m_eMin = min;
    m_eMax = max;
}

void FuzzyTuning::setErrorRateRange(double min, double max) {
    m_deMin = min;
    m_deMax = max;
}

void FuzzyTuning::setGainRanges(double dKpMax, double dKiMax, double dKdMax) {
    m_dKpMax = dKpMax;
    m_dKiMax = dKiMax;
    m_dKdMax = dKdMax;
}

void FuzzyTuning::addRule(const std::string& errorSet, const std::string& errorRateSet,
                          double dKp, double dKi, double dKd) {
    Rule rule;
    rule.errorSet = errorSet;
    rule.rateSet = errorRateSet;
    rule.dKp = dKp;
    rule.dKi = dKi;
    rule.dKd = dKd;
    m_rules.push_back(rule);
}

void FuzzyTuning::useDefaultRules() {
    m_rules.clear();
    
    m_errorMFs["NB"] = {MembershipFunction::Triangular, -1.0, -1.0, -0.5, 0.0};
    m_errorMFs["NS"] = {MembershipFunction::Triangular, -1.0, -0.5, 0.0, 0.0};
    m_errorMFs["ZO"] = {MembershipFunction::Triangular, -0.5, 0.0, 0.5, 0.0};
    m_errorMFs["PS"] = {MembershipFunction::Triangular, 0.0, 0.5, 1.0, 0.0};
    m_errorMFs["PB"] = {MembershipFunction::Triangular, 0.5, 1.0, 1.0, 0.0};
    
    m_rateMFs["N"] = {MembershipFunction::Triangular, -1.0, -1.0, 0.0, 0.0};
    m_rateMFs["Z"] = {MembershipFunction::Triangular, -0.5, 0.0, 0.5, 0.0};
    m_rateMFs["P"] = {MembershipFunction::Triangular, 0.0, 1.0, 1.0, 0.0};
    
    addRule("NB", "N", 1.0, 0.0, 1.0);
    addRule("NB", "Z", 1.0, 0.0, 0.5);
    addRule("NB", "P", 0.5, 0.0, 0.0);
    addRule("NS", "N", 0.5, 0.0, 0.5);
    addRule("NS", "Z", 0.5, 0.0, 0.0);
    addRule("NS", "P", 0.0, 0.0, 0.0);
    addRule("ZO", "N", 0.0, 0.5, 0.0);
    addRule("ZO", "Z", 0.0, 0.5, 0.0);
    addRule("ZO", "P", 0.0, 0.5, 0.0);
    addRule("PS", "N", 0.0, 0.0, 0.0);
    addRule("PS", "Z", -0.5, 0.0, 0.0);
    addRule("PS", "P", -0.5, 0.0, -0.5);
    addRule("PB", "N", -0.5, 0.0, 0.0);
    addRule("PB", "Z", -1.0, 0.0, -0.5);
    addRule("PB", "P", -1.0, 0.0, -1.0);
}

void FuzzyTuning::setBaseGains(double Kp, double Ki, double Kd) {
    m_Kp0 = Kp;
    m_Ki0 = Ki;
    m_Kd0 = Kd;
}

double FuzzyTuning::fuzzify(double value, const MembershipFunction& mf) const {
    return mf.evaluate(value);
}

void FuzzyTuning::defuzzify(double& dKp, double& dKi, double& dKd) {
    double totalWeight = 0.0;
    dKp = 0.0;
    dKi = 0.0;
    dKd = 0.0;
    
    double normError = std::clamp(m_lastError / m_eMax, -1.0, 1.0);
    
    for (const auto& rule : m_rules) {
        double errorDegree = 0.0;
        auto errorIt = m_errorMFs.find(rule.errorSet);
        if (errorIt != m_errorMFs.end()) {
            errorDegree = fuzzify(normError, errorIt->second);
        }
        
        double rateDegree = 0.5;
        double weight = std::min(errorDegree, rateDegree);
        
        dKp += weight * rule.dKp;
        dKi += weight * rule.dKi;
        dKd += weight * rule.dKd;
        totalWeight += weight;
    }
    
    if (totalWeight > 1e-10) {
        dKp /= totalWeight;
        dKi /= totalWeight;
        dKd /= totalWeight;
    }
    
    dKp *= m_dKpMax;
    dKi *= m_dKiMax;
    dKd *= m_dKdMax;
}

// ============================================================================
// NeuralNetworkTuning Implementation
// ============================================================================

bool NeuralNetworkTuning::isCompatible(const TunableController& controller) const {
    return controller.getParameters().size() >= 1;
}

TuningResult NeuralNetworkTuning::tune(TunableController& /*controller*/,
                                       const ProcessModel* /*model*/) {
    if (m_W.empty()) {
        setArchitecture({8});
    }
    
    TuningResult result;
    result.success = true;
    result.parameters = m_lastOutput;
    return result;
}

double NeuralNetworkTuning::update(double measured, double reference,
                                   double /*control*/, double dt) {
    if (!m_running) return 0.0;
    
    static double integralError = 0.0;
    static double lastError = 0.0;
    
    double error = reference - measured;
    integralError += error * dt;
    double derivError = (error - lastError) / dt;
    lastError = error;
    
    std::vector<double> inputs = {error, integralError, derivError, measured};
    m_lastOutput = forward(inputs);
    
    return error;
}

bool NeuralNetworkTuning::isComplete() const {
    return false;
}

void NeuralNetworkTuning::start() {
    m_running = true;
}

void NeuralNetworkTuning::stop() {
    m_running = false;
}

TuningResult NeuralNetworkTuning::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    result.parameters = m_lastOutput;
    return result;
}

void NeuralNetworkTuning::setArchitecture(const std::vector<int>& layers) {
    m_layers = layers;
    
    m_W.clear();
    m_b.clear();
    
    int inputSize = 4;
    int prevSize = inputSize;
    
    for (int hiddenSize : layers) {
        std::vector<std::vector<double>> W(hiddenSize, std::vector<double>(prevSize));
        std::vector<double> b(hiddenSize, 0.0);
        
        double scale = std::sqrt(2.0 / (prevSize + hiddenSize));
        for (int i = 0; i < hiddenSize; ++i) {
            for (int j = 0; j < prevSize; ++j) {
                W[i][j] = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * scale;
            }
        }
        
        m_W.push_back(W);
        m_b.push_back(b);
        prevSize = hiddenSize;
    }
    
    int outputSize = 3;
    std::vector<std::vector<double>> Wout(outputSize, std::vector<double>(prevSize));
    std::vector<double> bout(outputSize, 0.0);
    
    double scale = std::sqrt(2.0 / (prevSize + outputSize));
    for (int i = 0; i < outputSize; ++i) {
        for (int j = 0; j < prevSize; ++j) {
            Wout[i][j] = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * scale;
        }
    }
    
    m_W.push_back(Wout);
    m_b.push_back(bout);
}

void NeuralNetworkTuning::loadWeights(const std::vector<std::vector<double>>& weights) {
    size_t idx = 0;
    for (auto& layer : m_W) {
        for (auto& neuron : layer) {
            for (auto& w : neuron) {
                if (idx < weights.size() && !weights[idx].empty()) {
                    w = weights[idx][0];
                }
                idx++;
            }
        }
    }
}

void NeuralNetworkTuning::trainBatch(const std::vector<std::vector<double>>& inputs,
                                     const std::vector<std::vector<double>>& targets,
                                     int epochs) {
    for (int epoch = 0; epoch < epochs; ++epoch) {
        for (size_t sample = 0; sample < inputs.size() && sample < targets.size(); ++sample) {
            std::vector<std::vector<double>> activations;
            activations.push_back(inputs[sample]);
            
            std::vector<double> current = inputs[sample];
            for (size_t layer = 0; layer < m_W.size(); ++layer) {
                std::vector<double> next(m_W[layer].size());
                for (size_t i = 0; i < m_W[layer].size(); ++i) {
                    double sum = m_b[layer][i];
                    for (size_t j = 0; j < current.size() && j < m_W[layer][i].size(); ++j) {
                        sum += m_W[layer][i][j] * current[j];
                    }
                    next[i] = activate(sum);
                }
                activations.push_back(next);
                current = next;
            }
            
            std::vector<double> delta(current.size());
            for (size_t i = 0; i < current.size() && i < targets[sample].size(); ++i) {
                double error = targets[sample][i] - current[i];
                delta[i] = error * activateDerivative(current[i]);
            }
            
            for (int layer = static_cast<int>(m_W.size()) - 1; layer >= 0; --layer) {
                const auto& prevAct = activations[layer];
                
                for (size_t i = 0; i < m_W[layer].size() && i < delta.size(); ++i) {
                    for (size_t j = 0; j < m_W[layer][i].size() && j < prevAct.size(); ++j) {
                        m_W[layer][i][j] += m_learningRate * delta[i] * prevAct[j];
                    }
                    m_b[layer][i] += m_learningRate * delta[i];
                }
                
                if (layer > 0) {
                    std::vector<double> newDelta(m_W[layer][0].size(), 0.0);
                    for (size_t j = 0; j < newDelta.size(); ++j) {
                        for (size_t i = 0; i < m_W[layer].size() && i < delta.size(); ++i) {
                            if (j < m_W[layer][i].size()) {
                                newDelta[j] += m_W[layer][i][j] * delta[i];
                            }
                        }
                        newDelta[j] *= activateDerivative(prevAct[j]);
                    }
                    delta = newDelta;
                }
            }
        }
    }
}

std::vector<double> NeuralNetworkTuning::forward(const std::vector<double>& input) {
    std::vector<double> current = input;
    
    for (size_t layer = 0; layer < m_W.size(); ++layer) {
        std::vector<double> next(m_W[layer].size());
        for (size_t i = 0; i < m_W[layer].size(); ++i) {
            double sum = m_b[layer][i];
            for (size_t j = 0; j < current.size() && j < m_W[layer][i].size(); ++j) {
                sum += m_W[layer][i][j] * current[j];
            }
            
            if (layer == m_W.size() - 1) {
                next[i] = 10.0 / (1.0 + std::exp(-sum));
            } else {
                next[i] = activate(sum);
            }
        }
        current = next;
    }
    
    return current;
}

double NeuralNetworkTuning::activate(double x) const {
    switch (m_activation) {
        case Activation::ReLU:
            return std::max(0.0, x);
        case Activation::Tanh:
            return std::tanh(x);
        case Activation::Sigmoid:
            return 1.0 / (1.0 + std::exp(-x));
        case Activation::Linear:
        default:
            return x;
    }
}

double NeuralNetworkTuning::activateDerivative(double x) const {
    switch (m_activation) {
        case Activation::ReLU:
            return x > 0 ? 1.0 : 0.0;
        case Activation::Tanh: {
            double t = std::tanh(x);
            return 1.0 - t * t;
        }
        case Activation::Sigmoid: {
            double s = 1.0 / (1.0 + std::exp(-x));
            return s * (1.0 - s);
        }
        case Activation::Linear:
        default:
            return 1.0;
    }
}

// ============================================================================
// MMAC Implementation
// ============================================================================

TuningResult MMAC::tune(TunableController& /*controller*/,
                        const ProcessModel* /*model*/) {
    if (!m_models.empty()) {
        m_probabilities.resize(m_models.size(), 1.0 / m_models.size());
    }
    
    TuningResult result;
    result.success = true;
    
    if (!m_models.empty()) {
        result.parameters = m_models[0].gains;
    }
    
    return result;
}

double MMAC::update(double measured, double /*reference*/,
                    double /*control*/, double /*dt*/) {
    if (!m_running || m_models.empty()) return 0.0;
    
    updateProbabilities(measured);
    computeMixedGains();
    
    m_lastY = measured;
    
    return measured;
}

void MMAC::start() {
    m_running = true;
}

void MMAC::stop() {
    m_running = false;
}

TuningResult MMAC::getIntermediateResult() const {
    TuningResult result;
    result.success = true;
    
    if (m_activeModel >= 0 && m_activeModel < static_cast<int>(m_models.size())) {
        result.parameters = m_models[m_activeModel].gains;
    }
    
    return result;
}

void MMAC::addModel(const FOPDTModel& model, const ParameterVector& gains) {
    ModelEntry entry;
    entry.model = model;
    entry.gains = gains;
    entry.yPred = 0.0;
    entry.residual = 0.0;
    entry.likelihood = 1.0;
    m_models.push_back(entry);
    
    m_probabilities.resize(m_models.size(), 1.0 / m_models.size());
}

void MMAC::updateProbabilities(double y) {
    if (m_models.empty()) return;
    
    double totalLikelihood = 0.0;
    double variance = 0.1;
    
    for (size_t i = 0; i < m_models.size(); ++i) {
        double error = y - m_models[i].yPred;
        m_models[i].residual = error;
        
        double likelihood = std::exp(-0.5 * error * error / variance);
        m_models[i].likelihood = likelihood;
        
        m_probabilities[i] *= likelihood;
        totalLikelihood += m_probabilities[i];
    }
    
    if (totalLikelihood > 1e-10) {
        for (double& p : m_probabilities) {
            p /= totalLikelihood;
        }
    } else {
        for (double& p : m_probabilities) {
            p = 1.0 / m_probabilities.size();
        }
    }
    
    double maxProb = 0.0;
    int newActive = 0;
    for (size_t i = 0; i < m_probabilities.size(); ++i) {
        if (m_probabilities[i] > maxProb) {
            maxProb = m_probabilities[i];
            newActive = static_cast<int>(i);
        }
    }
    
    if (m_activeModel >= 0 && newActive != m_activeModel) {
        if (maxProb - m_probabilities[m_activeModel] < m_hysteresis) {
            newActive = m_activeModel;
        }
    }
    
    m_activeModel = newActive;
}

ParameterVector MMAC::computeMixedGains() const {
    if (m_models.empty()) return {};
    
    switch (m_mixingMode) {
        case MixingMode::Switching:
            if (m_activeModel >= 0 && m_activeModel < static_cast<int>(m_models.size())) {
                return m_models[m_activeModel].gains;
            }
            return m_models[0].gains;
            
        case MixingMode::BayesianMixing:
        case MixingMode::SoftmaxMixing: {
            size_t numParams = m_models[0].gains.size();
            ParameterVector mixed(numParams, 0.0);
            
            for (size_t i = 0; i < m_models.size(); ++i) {
                double weight = m_probabilities[i];
                
                if (m_mixingMode == MixingMode::SoftmaxMixing) {
                    weight = std::exp(10.0 * weight);
                }
                
                for (size_t j = 0; j < numParams && j < m_models[i].gains.size(); ++j) {
                    mixed[j] += weight * m_models[i].gains[j];
                }
            }
            
            if (m_mixingMode == MixingMode::SoftmaxMixing) {
                double totalWeight = 0.0;
                for (double p : m_probabilities) {
                    totalWeight += std::exp(10.0 * p);
                }
                for (double& g : mixed) {
                    g /= totalWeight;
                }
            }
            
            return mixed;
        }
    }
    
    return m_models[0].gains;
}

} // namespace Autotuning
} // namespace Control
