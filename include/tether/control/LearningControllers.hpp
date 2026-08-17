/**
 * @file LearningControllers.hpp
 * @brief Learning-Based Controllers: Iterative Learning Control (ILC)
 * 
 * @details
 * This file implements learning-based control methods that improve
 * performance over repeated operations by learning from past trials.
 * 
 * ## Iterative Learning Control (ILC)
 * 
 * ### Motivation
 * Many industrial processes are repetitive:
 * - Robot picking same objects repeatedly
 * - CNC machines cutting same patterns
 * - Batch chemical processes
 * - Printing/coating operations
 * 
 * ILC exploits this repetition to achieve perfect tracking.
 * 
 * ### Basic Principle
 * ```
 * u_{k+1}(t) = u_k(t) + L · e_k(t)
 * 
 * Where:
 *   k = trial/iteration number
 *   u_k(t) = control input for trial k at time t
 *   e_k(t) = tracking error for trial k
 *   L = learning operator
 * ```
 * 
 * ### Convergence
 * ILC converges if: ||I - G·L|| < 1
 * Where G is the plant transfer function.
 * 
 * ### Learning Operators
 * 
 * **P-type ILC** (simplest)
 * ```
 * u_{k+1}(t) = u_k(t) + γ · e_k(t)
 * ```
 * 
 * **D-type ILC**
 * ```
 * u_{k+1}(t) = u_k(t) + γ · ė_k(t)
 * ```
 * 
 * **PD-type ILC**
 * ```
 * u_{k+1}(t) = u_k(t) + γ_p · e_k(t) + γ_d · ė_k(t)
 * ```
 * 
 * **Phase-Lead ILC** (robustness to phase lag)
 * ```
 * u_{k+1}(t) = u_k(t) + γ · e_k(t + δ)
 * ```
 * 
 * ### Current Iteration Learning (CILC)
 * Combines ILC with feedback:
 * ```
 * u(t) = u_ff(t) + u_fb(t)
 * u_ff ← ILC
 * u_fb ← PID/other
 * ```
 * 
 * ### Q-filter
 * Low-pass filter for robustness:
 * ```
 * u_{k+1} = Q(u_k + L·e_k)
 * ```
 * Q removes high-frequency learning signal that can cause instability.
 * 
 * ## Usage Examples
 * 
 * ### Basic P-type ILC
 * ```cpp
 * PTypeILC ilc;
 * ilc.setLearningGain(0.5);          // γ = 0.5
 * ilc.setTrajectoryLength(1000);      // 1000 samples per trial
 * ilc.setQFilter(0.9);               // Low-pass filter cutoff
 * 
 * // Trial loop
 * for (int trial = 0; trial < 100; trial++) {
 *     ilc.startTrial();
 *     
 *     for (int t = 0; t < 1000; t++) {
 *         ControllerInput input;
 *         input.reference = trajectory[t];
 *         input.measured = sensor.read();
 *         input.dt = 0.001;
 *         
 *         auto output = ilc.compute(input);
 *         motor.set(output.control);
 *         
 *         ilc.recordSample(t);  // Store for learning
 *     }
 *     
 *     ilc.endTrial();  // Update feedforward signal
 *     
 *     // Check convergence
 *     if (ilc.getRMSError() < threshold) break;
 * }
 * ```
 * 
 * ### ILC + Feedback
 * ```cpp
 * PTypeILC ilc;
 * PIDController pid;
 * 
 * ilc.setLearningGain(0.3);
 * pid.setGains(1.0, 0.1, 0.05);
 * 
 * // During trial
 * auto ilcOutput = ilc.compute(input);       // Feedforward from learning
 * auto pidOutput = pid.compute(input);        // Feedback for disturbances
 * double totalControl = ilcOutput.feedforward + pidOutput.control;
 * ```
 * 
 * ### Norm-Optimal ILC
 * ```cpp
 * NormOptimalILC noilc;
 * 
 * // Set plant model (lifted system matrix)
 * noilc.setPlantModel(G, n);
 * 
 * // Set weights
 * noilc.setWeights(Qe, Re, Se);  // Error, input change, input weights
 * 
 * // Design offline
 * noilc.design();
 * 
 * // Use online
 * noilc.updateFeedforward(errorVector);
 * ```
 * 
 * ## Implementation Notes
 * 
 * ### Memory Requirements
 * ILC requires storing:
 * - Previous trial control signal: N samples
 * - Current trial error signal: N samples
 * - Feedforward signal: N samples
 * 
 * For N=1000 and double precision: ~24 KB
 * 
 * ### Computational Cost
 * - P-type ILC: O(N) per trial update
 * - Norm-optimal ILC: O(N²) or O(N³) depending on implementation
 * 
 * ### Practical Considerations
 * 1. **Initial trial**: Use feedback-only or nominal feedforward
 * 2. **Synchronization**: Ensure trials are well-aligned in time
 * 3. **Disturbances**: ILC assumes repeatable disturbances
 * 4. **Model changes**: May need to reset learning
 * 
 * @see ControllerBase
 * @see PIDController
 */

#pragma once

#include "ControllerBase.hpp"
#include <vector>
#include <deque>
#include <functional>

namespace tether::control {

// ============================================================================
// ILC Base Class
// ============================================================================

/**
 * @brief Base class for Iterative Learning Controllers
 * 
 * Provides common infrastructure for ILC implementations:
 * - Trial management
 * - Signal storage
 * - Q-filtering
 */
class ILCBase : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::ILC; }
    
    /**
     * @brief Set trajectory length (samples per trial)
     * @param length Number of samples
     */
    void setTrajectoryLength(size_t length);
    
    /**
     * @brief Get trajectory length
     */
    size_t getTrajectoryLength() const { return m_trajLength; }
    
    /**
     * @brief Start a new trial
     * 
     * Resets sample counter and prepares for new trial.
     */
    virtual void startTrial();
    
    /**
     * @brief End current trial and update learning
     * 
     * Processes recorded data and updates feedforward signal.
     */
    virtual void endTrial();
    
    /**
     * @brief Record sample at current time step
     * 
     * Stores error and control for learning update.
     * 
     * @param timeIndex Current sample index (0 to trajLength-1)
     */
    void recordSample(size_t timeIndex);
    
    /**
     * @brief Get current trial number
     */
    int getTrialNumber() const { return m_trialNum; }
    
    /**
     * @brief Get RMS error of last completed trial
     */
    double getRMSError() const { return m_rmsError; }
    
    /**
     * @brief Get max absolute error of last trial
     */
    double getMaxError() const { return m_maxError; }
    
    /**
     * @brief Set Q-filter cutoff (normalized 0-1)
     * 
     * Q-filter smooths the learned signal for robustness.
     * 
     * @param cutoff Normalized cutoff frequency (0=no learning, 1=no filter)
     */
    void setQFilter(double cutoff);
    
    /**
     * @brief Enable/disable Q-filtering
     */
    void setQFilterEnabled(bool enabled) { m_useQFilter = enabled; }
    
    /**
     * @brief Set forgetting factor
     * 
     * For non-repetitive disturbances, gradually forget old learning:
     * u_ff = λ * u_ff_prev + (1-λ) * learning_update
     * 
     * @param lambda Forgetting factor (0=no memory, 1=full memory)
     */
    void setForgettingFactor(double lambda) { m_forgettingFactor = lambda; }
    
    /**
     * @brief Get feedforward signal at time index
     */
    double getFeedforward(size_t timeIndex) const;
    
    /**
     * @brief Reset all learning
     */
    void resetLearning();
    
protected:
    size_t m_trajLength{0};
    size_t m_currentIndex{0};
    int m_trialNum{0};
    
    // Signal storage
    std::vector<double> m_feedforward;   // Learned feedforward signal
    std::vector<double> m_prevFeedforward;
    std::vector<double> m_errorHistory;  // Error from current/last trial
    std::vector<double> m_controlHistory;
    
    // Performance metrics
    double m_rmsError{0.0};
    double m_maxError{0.0};
    
    // Q-filter
    bool m_useQFilter{true};
    double m_qFilterCutoff{0.9};
    std::vector<double> m_qFilterCoeffs;
    
    // Forgetting
    double m_forgettingFactor{1.0};
    
    // Current sample storage
    double m_currentError{0.0};
    double m_currentControl{0.0};
    
    // Internal methods
    void applyQFilter(std::vector<double>& signal);
    void computeQFilterCoeffs();
    virtual void updateLearning() = 0;  // Implemented by derived classes
};

// ============================================================================
// P-Type ILC
// ============================================================================

/**
 * @brief P-Type Iterative Learning Control
 * 
 * Simplest ILC: updates feedforward proportional to error.
 * 
 * ## Update Law
 * ```
 * u_{k+1}(t) = Q[u_k(t) + γ·e_k(t)]
 * ```
 * 
 * ## Convergence
 * Converges if: |1 - G(jω)·γ| < 1 for all ω
 * 
 * ## Characteristics
 * - Simple and intuitive
 * - May not converge for systems with phase lag > 90°
 * - Good for systems with small delay
 */
class PTypeILC : public ILCBase {
public:
    const char* getName() const override { return "P-Type ILC"; }
    const char* getDescription() const override {
        return "Proportional ILC: u_{k+1} = u_k + γ·e_k. Simple, converges for "
               "systems with small phase lag. Use Q-filter for robustness.";
    }
    
    /**
     * @brief Set learning gain
     * @param gamma Learning gain (typically 0.1 to 0.9)
     */
    void setLearningGain(double gamma) { m_gamma = gamma; }
    
    /**
     * @brief Get learning gain
     */
    double getLearningGain() const { return m_gamma; }
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    void updateLearning() override;
    
private:
    double m_gamma{0.5};
};

// ============================================================================
// PD-Type ILC
// ============================================================================

/**
 * @brief PD-Type Iterative Learning Control
 * 
 * Uses both error and error derivative for phase lead.
 * 
 * ## Update Law
 * ```
 * u_{k+1}(t) = Q[u_k(t) + γ_p·e_k(t) + γ_d·ė_k(t)]
 * ```
 * 
 * ## Advantages
 * - Better convergence for systems with phase lag
 * - Derivative term provides phase lead compensation
 * 
 * ## Tuning
 * - γ_d > 0 provides phase lead
 * - Larger γ_d for larger plant phase lag
 */
class PDTypeILC : public ILCBase {
public:
    const char* getName() const override { return "PD-Type ILC"; }
    const char* getDescription() const override {
        return "PD ILC: u_{k+1} = u_k + γp·e + γd·ė. Derivative term adds phase "
               "lead for better convergence with phase-lagging systems.";
    }
    
    /**
     * @brief Set learning gains
     * @param gammaP Proportional learning gain
     * @param gammaD Derivative learning gain
     */
    void setLearningGains(double gammaP, double gammaD);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    void updateLearning() override;
    
private:
    double m_gammaP{0.3};
    double m_gammaD{0.2};
    std::vector<double> m_errorDerivHistory;
};

// ============================================================================
// Phase-Lead ILC
// ============================================================================

/**
 * @brief Phase-Lead Iterative Learning Control
 * 
 * Uses time-advanced error for phase compensation.
 * 
 * ## Update Law
 * ```
 * u_{k+1}(t) = Q[u_k(t) + γ·e_k(t + δ)]
 * 
 * Where δ = phase lead (in samples)
 * ```
 * 
 * ## Advantages
 * - Simple to implement
 * - Direct phase compensation
 * - Works well when phase lag is known
 * 
 * ## Phase Lead Selection
 * δ = T·φ / (2π)
 * Where T = period, φ = phase lag [rad]
 */
class PhaseLeadILC : public ILCBase {
public:
    const char* getName() const override { return "Phase-Lead ILC"; }
    const char* getDescription() const override {
        return "Phase-lead ILC: u_{k+1} = u_k + γ·e(t+δ). Time-shifts error "
               "for phase compensation. Set δ based on plant phase lag.";
    }
    
    /**
     * @brief Set learning gain and phase lead
     * @param gamma Learning gain
     * @param phaseLead Phase lead in samples (positive = look ahead)
     */
    void setParameters(double gamma, int phaseLead);
    
    /**
     * @brief Set phase lead from frequency and phase
     * @param freq Frequency [Hz]
     * @param phaseDelay Phase delay [rad]
     * @param sampleRate Sample rate [Hz]
     */
    void setPhaseFromFrequency(double freq, double phaseDelay, double sampleRate);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    void updateLearning() override;
    
private:
    double m_gamma{0.5};
    int m_phaseLead{0};  // In samples
};

// ============================================================================
// Norm-Optimal ILC
// ============================================================================

/**
 * @brief Norm-Optimal Iterative Learning Control
 * 
 * Optimal ILC using quadratic cost minimization.
 * 
 * ## Cost Function
 * ```
 * J = ||e_{k+1}||²_Qe + ||Δu_{k+1}||²_R + ||u_{k+1}||²_S
 * 
 * Where:
 *   e_{k+1} = reference - G·u_{k+1}  (predicted error)
 *   Δu_{k+1} = u_{k+1} - u_k         (input change)
 *   Qe, R, S = weighting matrices
 * ```
 * 
 * ## Update Law
 * ```
 * u_{k+1} = Q·u_k + L·e_k
 * 
 * Where Q, L are designed by minimizing J
 * ```
 * 
 * ## Advantages
 * - Optimal in 2-norm sense
 * - Systematic design via weights
 * - Can incorporate input constraints
 * - Monotonic convergence guaranteed
 * 
 * ## Requirements
 * - Plant model G (lifted system form)
 * - More computation than simple ILC
 */
class NormOptimalILC : public ILCBase {
public:
    const char* getName() const override { return "Norm-Optimal ILC"; }
    const char* getDescription() const override {
        return "Optimal ILC minimizing ||e||² + ||Δu||² + ||u||². "
               "Requires plant model. Guaranteed monotonic convergence. "
               "More computation but better performance than simple ILC.";
    }
    
    /**
     * @brief Set plant model (lifted/Toeplitz form)
     * 
     * The lifted system matrix G relates input to output:
     * y = G·u
     * 
     * For discrete system with impulse response g[0], g[1], ..., g[N-1]:
     * ```
     *     [g[0]    0      0    ...]
     * G = [g[1]  g[0]     0    ...]
     *     [g[2]  g[1]   g[0]   ...]
     *     [...   ...    ...    ...]
     * ```
     * 
     * @param G Lifted system matrix (N×N)
     * @param N Trajectory length
     */
    void setPlantModel(const double* G, int N);
    
    /**
     * @brief Set plant from impulse response
     * @param impulseResponse System impulse response
     * @param length Length of impulse response
     */
    void setPlantFromImpulseResponse(const double* impulseResponse, int length);
    
    /**
     * @brief Set weighting matrices
     * @param Qe Error weight (scalar or N×N)
     * @param R Input change weight (scalar or N×N)
     * @param S Input weight (scalar or N×N)
     */
    void setWeights(double Qe, double R, double S);
    
    /**
     * @brief Design the optimal learning matrices Q and L
     * 
     * Computes:
     * Q = (G'Qe·G + R + S)⁻¹(G'Qe·G + R)
     * L = (G'Qe·G + R + S)⁻¹G'Qe
     * 
     * @return true if design succeeded
     */
    bool design();
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    void updateLearning() override;
    
private:
    int m_N{0};  // System/trajectory dimension
    
    // Plant model (Toeplitz matrix)
    std::vector<double> m_G;
    
    // Weights
    double m_Qe{1.0};
    double m_R{0.01};
    double m_S{0.0};
    
    // Learning matrices
    std::vector<double> m_Q;  // N×N
    std::vector<double> m_L;  // N×N
    
    bool m_designed{false};
};

// ============================================================================
// Current Iteration Learning Control (CILC)
// ============================================================================

/**
 * @brief Current-Iteration Learning Controller
 * 
 * Combines ILC feedforward with real-time feedback for:
 * - Better tracking of learned trajectory
 * - Rejection of non-repeating disturbances
 * - Improved transient response
 * 
 * ## Structure
 * ```
 *                    ┌─────────┐
 *   u_ff(t) ────────►│         │
 *   (from ILC)       │    +    │───► u(t)
 *   u_fb(t) ────────►│         │
 *   (from PID)       └─────────┘
 * ```
 * 
 * ## Usage
 * ```cpp
 * CurrentIterationLearning cilc;
 * cilc.setILC(myILC);        // Set ILC for feedforward
 * cilc.setFeedback(myPID);   // Set PID for feedback
 * cilc.setFeedforwardWeight(0.8);  // ILC dominates
 * ```
 */
class CurrentIterationLearning : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::ILC; }
    const char* getName() const override { return "Current-Iteration Learning Control"; }
    const char* getDescription() const override {
        return "Combines ILC feedforward with real-time feedback (e.g., PID). "
               "ILC handles repeating trajectory, feedback handles disturbances.";
    }
    
    /**
     * @brief Set the ILC component
     * @param ilc Pointer to ILC controller (ownership not transferred)
     */
    void setILC(ILCBase* ilc) { m_ilc = ilc; }
    
    /**
     * @brief Set the feedback controller
     * @param feedback Pointer to feedback controller (ownership not transferred)
     */
    void setFeedback(ControllerBase* feedback) { m_feedback = feedback; }
    
    /**
     * @brief Set feedforward/feedback mixing weight
     * @param weight Weight for feedforward (0=pure FB, 1=pure FF)
     */
    void setFeedforwardWeight(double weight) { m_ffWeight = weight; }
    
    /**
     * @brief Start new trial (delegates to ILC)
     */
    void startTrial();
    
    /**
     * @brief End trial (delegates to ILC)
     */
    void endTrial();
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    ILCBase* m_ilc{nullptr};
    ControllerBase* m_feedback{nullptr};
    double m_ffWeight{0.5};
    size_t m_currentIndex{0};
};

// ============================================================================
// Repetitive Control
// ============================================================================

/**
 * @brief Repetitive Controller
 * 
 * Similar to ILC but operates in continuous time using
 * internal model principle with delay.
 * 
 * ## Principle
 * For periodic disturbances with period T:
 * ```
 * C(s) = e^{-sT} / (1 - e^{-sT}) × Q(s) × K(s)
 * ```
 * 
 * This provides infinite gain at harmonics of 1/T.
 * 
 * ## vs ILC
 * - RC: Continuous, good for periodic disturbances
 * - ILC: Trial-based, good for repetitive tasks
 * 
 * ## Usage
 * ```cpp
 * RepetitiveController rc;
 * rc.setPeriod(0.1);           // 100ms period = 10Hz fundamental
 * rc.setQFilter(100);          // Cutoff at 100Hz
 * rc.setStabilizingGain(0.5);  // K for stability
 * ```
 */
class RepetitiveController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::ILC; }
    const char* getName() const override { return "Repetitive Controller"; }
    const char* getDescription() const override {
        return "Continuous-time learning for periodic disturbances using "
               "internal model principle. Infinite gain at harmonics. "
               "Good for vibration rejection, periodic noise.";
    }
    
    /**
     * @brief Set repetition period
     * @param period Period [s]
     */
    void setPeriod(double period);
    
    /**
     * @brief Set Q-filter cutoff frequency
     * @param cutoffHz Cutoff frequency [Hz]
     */
    void setQFilter(double cutoffHz);
    
    /**
     * @brief Set stabilizing gain
     * @param gain Gain K (typically 0.3-0.7)
     */
    void setStabilizingGain(double gain) { m_gain = gain; }
    
    /**
     * @brief Set sample rate
     */
    void setSampleRate(double rate);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    double m_period{0.1};     // Repetition period [s]
    double m_sampleRate{1000}; // Sample rate [Hz]
    double m_gain{0.5};
    double m_qCutoff{100};    // Q-filter cutoff [Hz]
    
    size_t m_delayLength{0};  // Samples in one period
    std::deque<double> m_delayLine;  // Circular buffer for delay
    
    // Q-filter state
    double m_filterState{0.0};
    double m_filterAlpha{0.9};
};

} // namespace tether::control
