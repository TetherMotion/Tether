/**
 * @file LearningControllers.cpp
 * @brief Implementation of Learning-Based Controllers
 */

#include "control/LearningControllers.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace tether::control {

// ============================================================================
// ILCBase Implementation
// ============================================================================

void ILCBase::setTrajectoryLength(size_t length) {
    m_trajLength = length;
    m_feedforward.resize(length, 0.0);
    m_prevFeedforward.resize(length, 0.0);
    m_errorHistory.resize(length, 0.0);
    m_controlHistory.resize(length, 0.0);
    computeQFilterCoeffs();
}

void ILCBase::startTrial() {
    m_currentIndex = 0;
    m_rmsError = 0.0;
    m_maxError = 0.0;
    
    // Clear current trial history
    std::fill(m_errorHistory.begin(), m_errorHistory.end(), 0.0);
    std::fill(m_controlHistory.begin(), m_controlHistory.end(), 0.0);
    
    // Save previous feedforward
    m_prevFeedforward = m_feedforward;
}

void ILCBase::endTrial() {
    // Calculate error metrics
    if (m_trajLength > 0) {
        double sumSq = 0.0;
        for (size_t i = 0; i < m_trajLength; ++i) {
            double absErr = std::fabs(m_errorHistory[i]);
            sumSq += m_errorHistory[i] * m_errorHistory[i];
            if (absErr > m_maxError) {
                m_maxError = absErr;
            }
        }
        m_rmsError = std::sqrt(sumSq / m_trajLength);
    }
    
    // Update feedforward signal via derived class
    updateLearning();
    
    // Apply Q-filter if enabled
    if (m_useQFilter) {
        applyQFilter(m_feedforward);
    }
    
    // Apply forgetting factor
    if (m_forgettingFactor < 1.0) {
        for (size_t i = 0; i < m_trajLength; ++i) {
            m_feedforward[i] = m_forgettingFactor * m_feedforward[i] + 
                               (1.0 - m_forgettingFactor) * m_prevFeedforward[i];
        }
    }
    
    m_trialNum++;
}

void ILCBase::recordSample(size_t timeIndex) {
    if (timeIndex < m_trajLength) {
        m_errorHistory[timeIndex] = m_currentError;
        m_controlHistory[timeIndex] = m_currentControl;
    }
}

void ILCBase::setQFilter(double cutoff) {
    m_qFilterCutoff = std::clamp(cutoff, 0.0, 1.0);
    computeQFilterCoeffs();
}

double ILCBase::getFeedforward(size_t timeIndex) const {
    if (timeIndex < m_feedforward.size()) {
        return m_feedforward[timeIndex];
    }
    return 0.0;
}

void ILCBase::resetLearning() {
    std::fill(m_feedforward.begin(), m_feedforward.end(), 0.0);
    std::fill(m_prevFeedforward.begin(), m_prevFeedforward.end(), 0.0);
    m_trialNum = 0;
    m_rmsError = 0.0;
    m_maxError = 0.0;
}

void ILCBase::applyQFilter(std::vector<double>& signal) {
    if (signal.empty()) return;
    
    // Simple first-order low-pass filter (forward-backward for zero phase)
    double alpha = m_qFilterCutoff;
    
    // Forward pass
    std::vector<double> temp(signal.size());
    temp[0] = signal[0];
    for (size_t i = 1; i < signal.size(); ++i) {
        temp[i] = alpha * signal[i] + (1.0 - alpha) * temp[i-1];
    }
    
    // Backward pass for zero-phase
    signal[signal.size()-1] = temp[signal.size()-1];
    for (int i = static_cast<int>(signal.size()) - 2; i >= 0; --i) {
        signal[i] = alpha * temp[i] + (1.0 - alpha) * signal[i+1];
    }
}

void ILCBase::computeQFilterCoeffs() {
    // For more advanced filters, compute coefficients here
    // Currently using simple first-order filter in applyQFilter
    m_qFilterCoeffs.clear();
    m_qFilterCoeffs.push_back(m_qFilterCutoff);
}

// ============================================================================
// P-Type ILC Implementation
// ============================================================================

ControllerOutput PTypeILC::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    // Compute error
    m_currentError = input.reference - input.measured;
    
    // Get feedforward from learned signal
    double ff = 0.0;
    if (m_currentIndex < m_feedforward.size()) {
        ff = m_feedforward[m_currentIndex];
    }
    
    // Output is feedforward (ILC is open-loop per trial)
    output.control = ff;
    output.feedforward = ff;
    output.error = m_currentError;
    
    // Store for learning
    m_currentControl = output.control;
    
    return output;
}

void PTypeILC::resetImpl() {
    resetLearning();
}

void PTypeILC::updateLearning() {
    // P-type update: u_{k+1} = u_k + γ·e_k
    for (size_t i = 0; i < m_trajLength; ++i) {
        m_feedforward[i] = m_prevFeedforward[i] + m_gamma * m_errorHistory[i];
    }
}

// ============================================================================
// PD-Type ILC Implementation
// ============================================================================

void PDTypeILC::setLearningGains(double gammaP, double gammaD) {
    m_gammaP = gammaP;
    m_gammaD = gammaD;
}

ControllerOutput PDTypeILC::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    // Compute error
    m_currentError = input.reference - input.measured;
    
    // Get feedforward
    double ff = 0.0;
    if (m_currentIndex < m_feedforward.size()) {
        ff = m_feedforward[m_currentIndex];
    }
    
    output.control = ff;
    output.feedforward = ff;
    output.error = m_currentError;
    
    m_currentControl = output.control;
    
    return output;
}

void PDTypeILC::resetImpl() {
    resetLearning();
    m_errorDerivHistory.clear();
}

void PDTypeILC::updateLearning() {
    // Compute error derivative (finite difference)
    m_errorDerivHistory.resize(m_trajLength, 0.0);
    
    if (m_trajLength > 1) {
        m_errorDerivHistory[0] = m_errorHistory[1] - m_errorHistory[0];
        for (size_t i = 1; i < m_trajLength - 1; ++i) {
            // Central difference
            m_errorDerivHistory[i] = (m_errorHistory[i+1] - m_errorHistory[i-1]) / 2.0;
        }
        m_errorDerivHistory[m_trajLength-1] = 
            m_errorHistory[m_trajLength-1] - m_errorHistory[m_trajLength-2];
    }
    
    // PD-type update: u_{k+1} = u_k + γp·e_k + γd·ė_k
    for (size_t i = 0; i < m_trajLength; ++i) {
        m_feedforward[i] = m_prevFeedforward[i] + 
                          m_gammaP * m_errorHistory[i] +
                          m_gammaD * m_errorDerivHistory[i];
    }
}

// ============================================================================
// Phase-Lead ILC Implementation
// ============================================================================

void PhaseLeadILC::setParameters(double gamma, int phaseLead) {
    m_gamma = gamma;
    m_phaseLead = phaseLead;
}

void PhaseLeadILC::setPhaseFromFrequency(double freq, double phaseDelay, double sampleRate) {
    // Convert phase delay at given frequency to samples
    // phaseLead = (phaseDelay / 2π) × (sampleRate / freq)
    double period = sampleRate / freq;
    m_phaseLead = static_cast<int>(phaseDelay * period / (2.0 * M_PI));
}

ControllerOutput PhaseLeadILC::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    m_currentError = input.reference - input.measured;
    
    double ff = 0.0;
    if (m_currentIndex < m_feedforward.size()) {
        ff = m_feedforward[m_currentIndex];
    }
    
    output.control = ff;
    output.feedforward = ff;
    output.error = m_currentError;
    
    m_currentControl = output.control;
    
    return output;
}

void PhaseLeadILC::resetImpl() {
    resetLearning();
}

void PhaseLeadILC::updateLearning() {
    // Phase-lead update: u_{k+1}(t) = u_k(t) + γ·e_k(t+δ)
    for (size_t i = 0; i < m_trajLength; ++i) {
        // Get time-advanced error
        int advancedIdx = static_cast<int>(i) + m_phaseLead;
        double advancedError = 0.0;
        
        if (advancedIdx >= 0 && advancedIdx < static_cast<int>(m_trajLength)) {
            advancedError = m_errorHistory[advancedIdx];
        } else if (advancedIdx >= static_cast<int>(m_trajLength)) {
            // Extrapolate using last value
            advancedError = m_errorHistory[m_trajLength - 1];
        }
        
        m_feedforward[i] = m_prevFeedforward[i] + m_gamma * advancedError;
    }
}

// ============================================================================
// Norm-Optimal ILC Implementation
// ============================================================================

void NormOptimalILC::setPlantModel(const double* G, int N) {
    m_N = N;
    m_G.resize(N * N);
    std::copy(G, G + N * N, m_G.begin());
    m_designed = false;
}

void NormOptimalILC::setPlantFromImpulseResponse(const double* impulseResponse, int length) {
    m_N = length;
    m_G.resize(m_N * m_N, 0.0);
    
    // Build Toeplitz (lower triangular) matrix
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j <= i && j < length; ++j) {
            m_G[i * m_N + (i - j)] = impulseResponse[j];
        }
    }
    
    m_designed = false;
}

void NormOptimalILC::setWeights(double Qe, double R, double S) {
    m_Qe = Qe;
    m_R = R;
    m_S = S;
    m_designed = false;
}

bool NormOptimalILC::design() {
    if (m_N <= 0 || m_G.empty()) {
        return false;
    }
    
    // For scalar weights, the optimal solution simplifies to:
    // Q = (G'G·Qe + R + S)^(-1) × (G'G·Qe + R)
    // L = (G'G·Qe + R + S)^(-1) × G'·Qe
    
    // Compute G'G
    std::vector<double> GtG(m_N * m_N, 0.0);
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            double sum = 0.0;
            for (int k = 0; k < m_N; ++k) {
                sum += m_G[k * m_N + i] * m_G[k * m_N + j];
            }
            GtG[i * m_N + j] = sum;
        }
    }
    
    // Compute G'
    std::vector<double> Gt(m_N * m_N, 0.0);
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            Gt[i * m_N + j] = m_G[j * m_N + i];
        }
    }
    
    // Build matrix A = G'G·Qe + (R + S)·I
    std::vector<double> A(m_N * m_N, 0.0);
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            A[i * m_N + j] = GtG[i * m_N + j] * m_Qe;
        }
        A[i * m_N + i] += m_R + m_S;  // Add to diagonal
    }
    
    // Invert A using simple Gauss-Jordan (for embedded systems)
    // In production, use more robust methods
    std::vector<double> Ainv(m_N * m_N, 0.0);
    std::vector<double> Acopy = A;
    
    // Initialize Ainv as identity
    for (int i = 0; i < m_N; ++i) {
        Ainv[i * m_N + i] = 1.0;
    }
    
    // Gauss-Jordan elimination
    for (int col = 0; col < m_N; ++col) {
        // Find pivot
        int pivot = col;
        for (int row = col + 1; row < m_N; ++row) {
            if (std::fabs(Acopy[row * m_N + col]) > std::fabs(Acopy[pivot * m_N + col])) {
                pivot = row;
            }
        }
        
        // Swap rows
        if (pivot != col) {
            for (int j = 0; j < m_N; ++j) {
                std::swap(Acopy[col * m_N + j], Acopy[pivot * m_N + j]);
                std::swap(Ainv[col * m_N + j], Ainv[pivot * m_N + j]);
            }
        }
        
        // Check for singularity
        double pivotVal = Acopy[col * m_N + col];
        if (std::fabs(pivotVal) < 1e-12) {
            return false;
        }
        
        // Scale pivot row
        for (int j = 0; j < m_N; ++j) {
            Acopy[col * m_N + j] /= pivotVal;
            Ainv[col * m_N + j] /= pivotVal;
        }
        
        // Eliminate column
        for (int row = 0; row < m_N; ++row) {
            if (row != col) {
                double factor = Acopy[row * m_N + col];
                for (int j = 0; j < m_N; ++j) {
                    Acopy[row * m_N + j] -= factor * Acopy[col * m_N + j];
                    Ainv[row * m_N + j] -= factor * Ainv[col * m_N + j];
                }
            }
        }
    }
    
    // Compute Q = Ainv × (G'G·Qe + R·I)
    m_Q.resize(m_N * m_N, 0.0);
    std::vector<double> B(m_N * m_N, 0.0);
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            B[i * m_N + j] = GtG[i * m_N + j] * m_Qe;
        }
        B[i * m_N + i] += m_R;
    }
    
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            double sum = 0.0;
            for (int k = 0; k < m_N; ++k) {
                sum += Ainv[i * m_N + k] * B[k * m_N + j];
            }
            m_Q[i * m_N + j] = sum;
        }
    }
    
    // Compute L = Ainv × G' × Qe
    m_L.resize(m_N * m_N, 0.0);
    std::vector<double> temp(m_N * m_N, 0.0);
    
    // temp = G' × Qe
    for (int i = 0; i < m_N * m_N; ++i) {
        temp[i] = Gt[i] * m_Qe;
    }
    
    // L = Ainv × temp
    for (int i = 0; i < m_N; ++i) {
        for (int j = 0; j < m_N; ++j) {
            double sum = 0.0;
            for (int k = 0; k < m_N; ++k) {
                sum += Ainv[i * m_N + k] * temp[k * m_N + j];
            }
            m_L[i * m_N + j] = sum;
        }
    }
    
    m_designed = true;
    return true;
}

ControllerOutput NormOptimalILC::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    m_currentError = input.reference - input.measured;
    
    double ff = 0.0;
    if (m_currentIndex < m_feedforward.size()) {
        ff = m_feedforward[m_currentIndex];
    }
    
    output.control = ff;
    output.feedforward = ff;
    output.error = m_currentError;
    
    m_currentControl = output.control;
    
    return output;
}

void NormOptimalILC::resetImpl() {
    resetLearning();
}

void NormOptimalILC::updateLearning() {
    if (!m_designed || m_Q.empty() || m_L.empty()) {
        // Fall back to simple P-type
        double gamma = 0.3;
        for (size_t i = 0; i < m_trajLength; ++i) {
            m_feedforward[i] = m_prevFeedforward[i] + gamma * m_errorHistory[i];
        }
        return;
    }
    
    // Optimal update: u_{k+1} = Q × u_k + L × e_k
    std::vector<double> newFF(m_trajLength, 0.0);
    
    for (size_t i = 0; i < m_trajLength; ++i) {
        double qu = 0.0;
        double le = 0.0;
        
        for (size_t j = 0; j < m_trajLength; ++j) {
            qu += m_Q[i * m_N + j] * m_prevFeedforward[j];
            le += m_L[i * m_N + j] * m_errorHistory[j];
        }
        
        newFF[i] = qu + le;
    }
    
    m_feedforward = newFF;
}

// ============================================================================
// Current Iteration Learning Implementation
// ============================================================================

void CurrentIterationLearning::startTrial() {
    if (m_ilc) {
        m_ilc->startTrial();
    }
    m_currentIndex = 0;
}

void CurrentIterationLearning::endTrial() {
    if (m_ilc) {
        m_ilc->endTrial();
    }
}

ControllerOutput CurrentIterationLearning::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    output.error = input.reference - input.measured;
    
    double ffControl = 0.0;
    double fbControl = 0.0;
    
    // Get feedforward from ILC
    if (m_ilc) {
        auto ilcOutput = m_ilc->compute(input);
        ffControl = ilcOutput.feedforward;
        m_ilc->recordSample(m_currentIndex);
    }
    
    // Get feedback from feedback controller
    if (m_feedback) {
        auto fbOutput = m_feedback->compute(input);
        fbControl = fbOutput.control;
    }
    
    // Combine feedforward and feedback
    output.control = m_ffWeight * ffControl + (1.0 - m_ffWeight) * fbControl;
    output.feedforward = ffControl;
    
    m_currentIndex++;
    
    return output;
}

void CurrentIterationLearning::resetImpl() {
    if (m_ilc) {
        m_ilc->reset();
    }
    if (m_feedback) {
        m_feedback->reset();
    }
    m_currentIndex = 0;
}

// ============================================================================
// Repetitive Controller Implementation
// ============================================================================

void RepetitiveController::setPeriod(double period) {
    m_period = period;
    m_delayLength = static_cast<size_t>(m_period * m_sampleRate);
    m_delayLine.clear();
    m_delayLine.resize(m_delayLength, 0.0);
}

void RepetitiveController::setQFilter(double cutoffHz) {
    m_qCutoff = cutoffHz;
    // Compute filter coefficient
    double omega = 2.0 * M_PI * cutoffHz / m_sampleRate;
    m_filterAlpha = omega / (omega + 1.0);
}

void RepetitiveController::setSampleRate(double rate) {
    m_sampleRate = rate;
    // Update delay length
    m_delayLength = static_cast<size_t>(m_period * m_sampleRate);
    m_delayLine.clear();
    m_delayLine.resize(m_delayLength, 0.0);
    // Update Q-filter
    setQFilter(m_qCutoff);
}

ControllerOutput RepetitiveController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    // Compute error
    double error = input.reference - input.measured;
    output.error = error;
    
    // Get delayed signal from one period ago
    double delayedSignal = 0.0;
    if (!m_delayLine.empty()) {
        delayedSignal = m_delayLine.back();
    }
    
    // Q-filter the learning signal (low-pass for robustness)
    double learningSignal = error + delayedSignal;
    m_filterState = m_filterAlpha * learningSignal + (1.0 - m_filterAlpha) * m_filterState;
    
    // Output with gain
    output.control = m_gain * m_filterState;
    output.feedforward = delayedSignal;
    
    // Update delay line
    m_delayLine.push_front(m_filterState);
    if (m_delayLine.size() > m_delayLength) {
        m_delayLine.pop_back();
    }
    
    return output;
}

void RepetitiveController::resetImpl() {
    m_delayLine.clear();
    m_delayLine.resize(m_delayLength, 0.0);
    m_filterState = 0.0;
}

} // namespace tether::control
