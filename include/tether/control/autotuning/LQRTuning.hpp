/**
 * @file LQRTuning.hpp
 * @brief LQR/LQG-Based Tuning Methods
 * 
 * @details
 * Methods for optimal and near-optimal controller tuning based on
 * linear quadratic framework.
 * 
 * ## Implemented Methods
 * 
 * ### Q/R Weight Optimization
 * Find optimal Q, R matrices for given specifications
 * 
 * ### LQR/LTR (Loop Transfer Recovery)
 * Recover desired loop shape with Kalman filter
 * 
 * ### Regional Pole Placement
 * Constrain poles to specific regions
 * 
 * ### Iterative Feedback Tuning (IFT)
 * Data-driven optimization without model
 * 
 * ### Virtual Reference Feedback Tuning (VRFT)
 * One-shot data-driven tuning
 * 
 * ### Fictitious Reference Iterative Tuning (FRIT)
 * Single experiment optimization
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include "OptimizationAlgorithms.hpp"
#include <functional>

namespace tether::control {
namespace Autotuning {

// ============================================================================
// Q/R Weight Optimization
// ============================================================================

/**
 * @brief LQR Weight Matrix Optimizer
 * 
 * Optimizes Q and R matrices to achieve desired closed-loop
 * specifications (settling time, overshoot, input usage).
 * 
 * ## Cost function
 * J = w1*J_settling + w2*J_overshoot + w3*J_effort
 * 
 * ## Optimization over
 * - Diagonal Q elements (state weights)
 * - Diagonal R elements (control weights)
 */
class QROptimizer : public OfflineAutotuner {
public:
    std::string getName() const override { return "Q/R Weight Optimization"; }
    std::string getDescription() const override {
        return "Optimize LQR weight matrices for desired specifications. "
               "Uses simulation-based cost optimization.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Target specifications
     */
    struct Specifications {
        double settlingTime{1.0};      ///< Desired settling time [s]
        double overshoot{0.1};         ///< Max overshoot (fraction)
        double riseTime{0.3};          ///< Desired rise time [s]
        double steadyStateError{0.01}; ///< Acceptable SS error
        double controlEffort{1.0};     ///< Max control input
    };
    void setSpecifications(const Specifications& specs) { m_specs = specs; }
    
    /**
     * @brief Set specification weights
     */
    void setWeights(double wSettling, double wOvershoot, double wEffort);
    
    /**
     * @brief Set state dimension
     */
    void setStateDimension(int n) { m_stateDim = n; }
    
    /**
     * @brief Set control dimension
     */
    void setControlDimension(int m) { m_controlDim = m; }
    
    /**
     * @brief Fix some Q elements (not optimized)
     */
    void fixQElements(const std::vector<int>& indices, 
                     const std::vector<double>& values);
    
    /**
        * @brief Get optimized Q matrix (flattened row-major)
     */
    std::vector<double> getOptimalQ() const { return m_Qopt; }
    
    /**
        * @brief Get optimized R matrix (flattened row-major)
     */
    std::vector<double> getOptimalR() const { return m_Ropt; }
    
    /**
     * @brief Set optimization algorithm
     */
    template<typename Opt>
    void setOptimizer() { 
        m_optimizer = std::make_unique<Opt>(); 
    }
    
private:
    Specifications m_specs;
    double m_wSettling{1.0}, m_wOvershoot{1.0}, m_wEffort{0.5};
    
    int m_stateDim{2};
    int m_controlDim{1};
    
    std::vector<int> m_fixedQIndices;
    std::vector<double> m_fixedQValues;
    
    std::vector<double> m_Qopt, m_Ropt;
    
    std::unique_ptr<OptimizationAlgorithm> m_optimizer;
    
    double evaluateCost(const std::vector<double>& params,
                       const ProcessModel* model);
};

// ============================================================================
// LQR/LTR (Loop Transfer Recovery)
// ============================================================================

/**
 * @brief Loop Transfer Recovery
 * 
 * Design Kalman filter gains to recover desired loop transfer
 * function at plant input.
 * 
 * ## Procedure
 * 1. Design target loop shape L(s) = K(sI-A)^-1 B
 * 2. Design Kalman filter: Kf = lim(q→∞) of ARE solution
 * 3. As q increases, loop at input → G*K
 * 
 * Useful for robustness at plant input.
 */
class LoopTransferRecovery : public OfflineAutotuner {
public:
    std::string getName() const override { return "Loop Transfer Recovery"; }
    std::string getDescription() const override {
        return "Recover LQR loop shape at plant input using Kalman filter. "
               "Improves robustness to input uncertainty.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set state-space model
     */
    void setStateSpaceModel(const std::vector<std::vector<double>>& A,
                           const std::vector<std::vector<double>>& B,
                           const std::vector<std::vector<double>>& C);
    
    /**
     * @brief Set target LQ gains (computed separately)
     */
    void setTargetGains(const std::vector<double>& K);
    
    /**
     * @brief Set recovery parameter
     * Higher q = better recovery but more noise sensitivity
     */
    void setRecoveryParameter(double q) { m_q = q; }
    
    /**
     * @brief Set process noise matrix (V1)
     */
    void setProcessNoise(const std::vector<std::vector<double>>& V1);
    
    /**
     * @brief Set measurement noise matrix (V2)
     */
    void setMeasurementNoise(const std::vector<std::vector<double>>& V2);
    
    /**
     * @brief Get computed Kalman filter gain
     */
    std::vector<std::vector<double>> getKalmanGain() const { return m_Kf; }
    
    /**
     * @brief Get loop recovery error
     */
    double getRecoveryError() const { return m_recoveryError; }
    
private:
    std::vector<std::vector<double>> m_A, m_B, m_C;
    std::vector<double> m_K;  // Target gains
    double m_q{1e6};
    
    std::vector<std::vector<double>> m_V1, m_V2;
    std::vector<std::vector<double>> m_Kf;
    double m_recoveryError{0.0};
};

// ============================================================================
// Regional Pole Placement
// ============================================================================

/**
 * @brief Regional Pole Placement via LMI
 * 
 * Find state feedback gain K such that poles are in
 * specified region (disk, sector, vertical strip).
 * 
 * ## Regions
 * - Disk: |λ - c| < r (stability margin)
 * - Sector: |arg(λ)| < θ (damping)
 * - Vertical strip: α < Re(λ) < β (time constant)
 */
class RegionalPolePlacement : public OfflineAutotuner {
public:
    std::string getName() const override { return "Regional Pole Placement"; }
    std::string getDescription() const override {
        return "Constrain closed-loop poles to specific LMI region. "
               "Guarantees minimum stability margin and damping.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief LMI Region specification
     */
    struct LMIRegion {
        enum Type { Disk, Sector, Strip, Intersection };
        Type type{Disk};
        
        // Disk: center (c, 0), radius r
        double diskCenterReal{0.0};
        double diskRadius{1.0};
        
        // Sector: half-angle theta (damping ζ = cos(θ))
        double sectorAngle{0.7};  // ~45 deg
        
        // Vertical strip: α < Re(λ) < β
        double stripLeft{-10.0};
        double stripRight{-0.1};
    };
    
    /**
     * @brief Add region constraint
     */
    void addRegion(const LMIRegion& region);
    
    /**
     * @brief Clear all regions
     */
    void clearRegions() { m_regions.clear(); }
    
    /**
     * @brief Set state-space model
     */
    void setStateSpaceModel(const std::vector<std::vector<double>>& A,
                           const std::vector<std::vector<double>>& B);
    
    /**
     * @brief Set gain structure (for structured synthesis)
     */
    void setGainStructure(const std::vector<bool>& structure);
    
    /**
     * @brief Get pole locations of designed system
     */
    std::vector<std::pair<double, double>> getPoles() const { return m_poles; }
    
private:
    std::vector<std::vector<double>> m_A, m_B;
    std::vector<LMIRegion> m_regions;
    std::vector<bool> m_gainStructure;
    
    std::vector<std::pair<double, double>> m_poles;
    
    bool checkRegionConstraint(const std::vector<std::pair<double, double>>& poles,
                              const LMIRegion& region);
};

// ============================================================================
// Iterative Feedback Tuning (IFT)
// ============================================================================

/**
 * @brief Iterative Feedback Tuning
 * 
 * Data-driven optimization that computes gradient of criterion
 * using experiments on the real system.
 * 
 * ## Algorithm
 * 1. Run experiment with current controller
 * 2. Run experiment with special test signal
 * 3. Compute gradient estimate from data
 * 4. Update parameters: θ_{k+1} = θ_k - γ * ∇J
 * 
 * No model required!
 */
class IterativeFeedbackTuning : public OnlineAutotuner {
public:
    std::string getName() const override { return "Iterative Feedback Tuning"; }
    std::string getDescription() const override {
        return "Data-driven optimization using gradient from experiments. "
               "No model required, works on real system.";
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
     * @brief Set reference signal for experiments
     */
    void setReferenceSignal(const std::vector<double>& r);
    
    /**
     * @brief Set cost function weights
     * J = λ_y * ||y - y_d||² + λ_u * ||u||²
     */
    void setWeights(double lambdaY, double lambdaU);
    
    /**
     * @brief Set step size
     */
    void setStepSize(double gamma) { m_gamma = gamma; }
    
    /**
     * @brief Set number of iterations
     */
    void setIterations(int n) { m_maxIterations = n; }
    
    /**
     * @brief Set finite difference step
     */
    void setFiniteDifferenceStep(double delta) { m_delta = delta; }
    
    /**
     * @brief Current iteration phase
     */
    enum class Phase {
        Normal,         ///< Normal experiment
        Gradient,       ///< Gradient experiment
        Idle           ///< Between experiments
    };
    Phase getCurrentPhase() const { return m_phase; }
    
    /**
     * @brief Get current gradient estimate
     */
    std::vector<double> getGradient() const { return m_gradient; }
    
private:
    std::vector<double> m_refSignal;
    double m_lambdaY{1.0}, m_lambdaU{0.1};
    double m_gamma{0.1};
    double m_delta{0.01};
    int m_maxIterations{10};
    
    Phase m_phase{Phase::Idle};
    int m_iteration{0};
    int m_sampleIndex{0};
    
    std::vector<double> m_gradient;
    std::vector<double> m_yData, m_uData;
    std::vector<double> m_yGradData, m_uGradData;
    
    ParameterVector m_currentParams;
    bool m_running{false};
    
    void computeGradient();
    void updateParameters();
};

// ============================================================================
// Virtual Reference Feedback Tuning (VRFT)
// ============================================================================

/**
 * @brief Virtual Reference Feedback Tuning
 * 
 * One-shot data-driven tuning that converts control design
 * to identification problem.
 * 
 * ## Key idea
 * Compute "virtual reference" r̃ such that:
 * If r̃ were the actual reference, output would be y
 * 
 * Then solve: min ||C(θ)*e - u||²
 * where e = r̃ - y
 */
class VRFTuning : public OfflineAutotuner {
public:
    std::string getName() const override { return "VRFT"; }
    std::string getDescription() const override {
        return "One-shot data-driven tuning. Requires single experiment, "
               "converts to identification problem.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set experimental data
     */
    void setData(const std::vector<double>& u, 
                const std::vector<double>& y,
                double Ts);
    
    /**
     * @brief Set reference model
     * Desired closed-loop transfer function
     */
    void setReferenceModel(double K, double tau);  // First order
    void setReferenceModel(double K, double wn, double zeta);  // Second order
    
    /**
     * @brief Set controller structure
     * 
     * PID: C(s) = Kp + Ki/s + Kd*s / (1 + s/N)
     */
    void setControllerStructure(const std::vector<std::string>& terms);
    
    /**
     * @brief Set prefilter for noise reduction
     */
    void setPrefilter(const std::vector<double>& numL,
                     const std::vector<double>& denL);
    
    /**
     * @brief Get virtual reference signal
     */
    std::vector<double> getVirtualReference() const { return m_rVirtual; }
    
    /**
     * @brief Get fit quality (R²)
     */
    double getFitQuality() const { return m_R2; }
    
private:
    std::vector<double> m_u, m_y;
    double m_Ts{0.01};
    
    // Reference model
    double m_Km{1.0}, m_taum{1.0};
    bool m_secondOrder{false};
    double m_wn{1.0}, m_zeta{0.707};
    
    // Controller structure
    std::vector<std::string> m_structure{"Kp", "Ki", "Kd"};
    
    // Prefilter
    std::vector<double> m_numL{1.0}, m_denL{1.0};
    
    // Results
    std::vector<double> m_rVirtual;
    double m_R2{0.0};
    
    std::vector<double> computeVirtualReference();
    std::vector<std::vector<double>> buildRegressor(const std::vector<double>& e);
};

// ============================================================================
// Fictitious Reference Iterative Tuning (FRIT)
// ============================================================================

/**
 * @brief Fictitious Reference Iterative Tuning
 * 
 * Simplified IFT using single closed-loop experiment.
 * Computes parameters that would give desired response.
 * 
 * ## Method
 * 1. Run one experiment
 * 2. Compute fictitious reference: r̃ = M⁻¹ * y
 * 3. Optimize: min ||M*r̃ - y||²
 */
class FRITuning : public OfflineAutotuner {
public:
    std::string getName() const override { return "FRIT"; }
    std::string getDescription() const override {
        return "Single-experiment optimization. Finds parameters that "
               "would have given desired response.";
    }
    bool isCompatible(const TunableController& controller) const override;
    
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set experimental data
     */
    void setData(const std::vector<double>& r,
                const std::vector<double>& u,
                const std::vector<double>& y,
                double Ts);
    
    /**
     * @brief Set reference model
     */
    void setReferenceModel(double K, double tau);
    void setReferenceModel(double K, double wn, double zeta);
    
    /**
     * @brief Set optimization algorithm
     */
    void setOptimizer(std::unique_ptr<OptimizationAlgorithm> opt);
    
    /**
     * @brief Get model matching error
     */
    double getMatchingError() const { return m_matchingError; }
    
private:
    std::vector<double> m_r, m_u, m_y;
    double m_Ts{0.01};
    
    double m_Km{1.0}, m_taum{1.0};
    bool m_secondOrder{false};
    double m_wn{1.0}, m_zeta{0.707};
    
    std::unique_ptr<OptimizationAlgorithm> m_optimizer;
    double m_matchingError{0.0};
    
    std::vector<double> computeFictitiousReference();
    double evaluateCost(const ParameterVector& params);
};

// ============================================================================
// Data-Driven Utilities
// ============================================================================

/**
 * @brief Utilities for data-driven tuning
 */
namespace DataDrivenUtils {
    /**
     * @brief Filter signal (IIR)
     */
    std::vector<double> filter(const std::vector<double>& signal,
                               const std::vector<double>& num,
                               const std::vector<double>& den);
    
    /**
     * @brief Compute numerical derivative
     */
    std::vector<double> derivative(const std::vector<double>& signal, double Ts);
    
    /**
     * @brief Compute integral
     */
    std::vector<double> integrate(const std::vector<double>& signal, double Ts);
    
    /**
     * @brief Least squares fit
     */
    std::vector<double> leastSquares(const std::vector<std::vector<double>>& A,
                                     const std::vector<double>& b);
    
    /**
     * @brief Compute cross-correlation
     */
    std::vector<double> crossCorrelation(const std::vector<double>& x,
                                         const std::vector<double>& y);
}

} // namespace Autotuning
} // namespace tether::control
