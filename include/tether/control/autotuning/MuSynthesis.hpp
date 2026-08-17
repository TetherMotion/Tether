/**
 * @file MuSynthesis.hpp
 * @brief μ-Synthesis (Mu Synthesis) Robust Controller Design
 * 
 * @details
 * This file implements the μ-synthesis (mu-synthesis) framework for robust
 * controller design with structured uncertainty. μ-synthesis provides
 * less conservative controllers than H∞ when uncertainty structure is known.
 * 
 * ## Theory Overview
 * 
 * ### Structured Singular Value (μ)
 * The structured singular value μ_Δ(M) is defined as:
 * ```
 * μ_Δ(M) = 1 / min{σ̄(Δ) : det(I - MΔ) = 0, Δ ∈ Δ}
 * ```
 * where Δ is the structured uncertainty set.
 * 
 * ### Robust Performance Condition
 * The closed-loop system achieves robust performance if:
 * ```
 * sup μ_Δ̃(F_l(P, K)(jω)) < 1
 *  ω
 * ```
 * where Δ̃ includes both uncertainty and performance blocks.
 * 
 * ### D-K Iteration
 * Since μ cannot be directly optimized, D-K iteration alternates:
 * 1. **D-step**: Fix K, find D(ω) to minimize σ̄(DMD⁻¹)
 * 2. **K-step**: Fix D, synthesize K via H∞ for D̃PD̃⁻¹
 * 
 * ## Uncertainty Structures
 * 
 * ### Common Block Structures
 * - **Full blocks**: Δᵢ ∈ C^(nᵢ×nᵢ)
 * - **Repeated scalars**: Δᵢ = δᵢI (nᵢ copies)
 * - **Real uncertainties**: δᵢ ∈ R
 * - **Complex uncertainties**: δᵢ ∈ C
 * 
 * ### Example: Parametric Uncertainty
 * Plant with uncertain gain and time constant:
 * ```
 *           K(1 + δ_K)
 * G(s) = ────────────────
 *        (τ(1 + δ_τ)s + 1)
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * MuSynthesisController mu;
 * 
 * // Set nominal plant
 * mu.setNominalPlant(A, B, C, D, n, m, p);
 * 
 * // Define uncertainty structure
 * mu.addUncertaintyBlock("gain", 1, MuBlockType::Real, 0.2);
 * mu.addUncertaintyBlock("dynamics", 2, MuBlockType::Full, 0.1);
 * 
 * // Set performance weights
 * mu.setSensitivityWeight(1.5, 10.0, 0.001);
 * mu.setControlWeight(100.0);
 * 
 * // Synthesize controller
 * mu.synthesize();  // D-K iteration
 * 
 * // Check robustness
 * double muPeak = mu.getPeakMu();
 * if (muPeak < 1.0) {
 *     // Robust performance achieved
 * }
 * ```
 * 
 * @see HInfinityController
 * @see RobustControllers.hpp
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "../ControllerBase.hpp"
#include "../RobustControllers.hpp"
#include <vector>
#include <complex>
#include <string>
#include <memory>

namespace tether::control {

// ============================================================================
// Uncertainty Block Types
// ============================================================================

/**
 * @brief Type of uncertainty block in Δ structure
 */
enum class MuBlockType {
    Full,           ///< Full complex block Δᵢ ∈ C^(n×n)
    Scalar,         ///< Scalar uncertainty δᵢ ∈ C
    RepeatedScalar, ///< Repeated scalar δᵢ*I
    Real,           ///< Real parametric uncertainty δᵢ ∈ R
    RepeatedReal    ///< Repeated real scalar
};

/**
 * @brief Description of one uncertainty block
 */
struct UncertaintyBlock {
    std::string name;           ///< Block name for identification
    MuBlockType type{MuBlockType::Full};
    int rows{1};                ///< Block row dimension
    int cols{1};                ///< Block column dimension
    int repetitions{1};         ///< Number of repetitions (for repeated scalars)
    double bound{1.0};          ///< Uncertainty bound (|Δ| ≤ bound)
    
    int totalRows() const { 
        return (type == MuBlockType::RepeatedScalar || type == MuBlockType::RepeatedReal) 
               ? rows * repetitions : rows;
    }
    int totalCols() const {
        return (type == MuBlockType::RepeatedScalar || type == MuBlockType::RepeatedReal)
               ? cols * repetitions : cols;
    }
};

// ============================================================================
// D-Scale Structure
// ============================================================================

/**
 * @brief D-scale fitting result
 */
struct DScaleFit {
    std::vector<double> frequencies;    ///< Frequency points [rad/s]
    std::vector<std::vector<std::complex<double>>> D;  ///< D(jω) values
    int order{2};                       ///< Order of rational fit
    
    // Rational fit coefficients (for state-space realization)
    std::vector<double> numeratorCoeffs;
    std::vector<double> denominatorCoeffs;
};

// ============================================================================
// μ Analysis Results
// ============================================================================

/**
 * @brief Results from μ analysis
 */
struct MuAnalysisResult {
    std::vector<double> frequencies;
    std::vector<double> muUpper;        ///< Upper bounds
    std::vector<double> muLower;        ///< Lower bounds
    double peakMuUpper{0.0};           ///< Peak upper bound
    double peakMuLower{0.0};           ///< Peak lower bound
    double peakFrequency{0.0};         ///< Frequency of peak
    
    bool robustStability() const { return peakMuUpper < 1.0; }
    bool robustPerformance() const { return peakMuUpper < 1.0; }
};

// ============================================================================
// μ-Synthesis Controller
// ============================================================================

/**
 * @brief μ-Synthesis Robust Controller
 * 
 * Implements D-K iteration for structured uncertainty.
 * Provides less conservative design than H∞ when uncertainty
 * structure is known.
 * 
 * ## D-K Iteration Algorithm
 * 
 * 1. Initialize D = I
 * 2. K-step: Synthesize H∞ controller for scaled plant D*P*D⁻¹
 * 3. D-step: For each frequency, find D to minimize σ̄(DMD⁻¹)
 * 4. Fit rational D(s) to frequency data
 * 5. Check convergence, goto 2 if not converged
 */
class MuSynthesisController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "μ-Synthesis Controller"; }
    const char* getDescription() const override {
        return "Robust controller using μ-synthesis (D-K iteration). "
               "Handles structured uncertainty more efficiently than H∞. "
               "Define uncertainty structure, then run synthesize().";
    }
    
    // ========================================================================
    // Plant and Uncertainty Setup
    // ========================================================================
    
    /**
     * @brief Set nominal plant model
     * @param A, B, C, D State-space matrices
     * @param n, m, p Dimensions
     */
    void setNominalPlant(const double* A, const double* B, 
                        const double* C, const double* D,
                        int n, int m, int p);
    
    /**
     * @brief Add uncertainty block to Δ structure
     * @param name Block name
     * @param size Block dimension
     * @param type Block type
     * @param bound Uncertainty bound
     */
    void addUncertaintyBlock(const std::string& name, int size,
                            MuBlockType type, double bound = 1.0);
    
    /**
     * @brief Add repeated scalar uncertainty
     * @param name Block name
     * @param repetitions Number of repetitions
     * @param isReal True for real uncertainty
     * @param bound Uncertainty bound
     */
    void addRepeatedScalar(const std::string& name, int repetitions,
                          bool isReal = false, double bound = 1.0);
    
    /**
     * @brief Clear all uncertainty blocks
     */
    void clearUncertaintyBlocks();
    
    /**
     * @brief Set interconnection matrices (M = F_u(P, Δ))
     * 
     * P is partitioned as:
     * [z]   [P11 P12] [w]
     * [e] = [P21 P22] [u]
     */
    void setInterconnection(const double* P11, const double* P12,
                           const double* P21, const double* P22,
                           int nz, int ne, int nw, int nu);
    
    // ========================================================================
    // Performance Weights
    // ========================================================================
    
    /**
     * @brief Set sensitivity weight for tracking/disturbance rejection
     * W₁(s) = (s/M + ωB) / (s + ωB·ε)
     */
    void setSensitivityWeight(double M, double omegaB, double epsilon);
    
    /**
     * @brief Set control effort weight
     */
    void setControlWeight(double maxControl);
    
    /**
     * @brief Set complementary sensitivity weight for robustness
     * W₃(s) = (s + ωT/M) / (ε·s + ωT)
     */
    void setComplementarySensitivityWeight(double M, double omegaT, double epsilon);
    
    // ========================================================================
    // Synthesis
    // ========================================================================
    
    /**
     * @brief Configuration for D-K iteration
     */
    struct SynthesisConfig {
        int maxIterations = 10;          ///< Max D-K iterations
        double convergenceTol = 0.01;    ///< Convergence tolerance
        int dScaleOrder = 2;             ///< Order of D-scale fits
        int numFrequencies = 100;        ///< Frequency points for analysis
        double freqMin = 1e-3;           ///< Minimum frequency [rad/s]
        double freqMax = 1e3;            ///< Maximum frequency [rad/s]
        bool verbose = false;            ///< Print iteration info
    };
    
    /**
     * @brief Run μ-synthesis (D-K iteration)
     * @param config Synthesis configuration
     * @return true if synthesis succeeded
     */
    bool synthesize(const SynthesisConfig& config);
    
    /**
     * @brief Run μ-synthesis with default configuration
     */
    bool synthesize() { return synthesize(SynthesisConfig{}); }
    
    /**
     * @brief Get current μ analysis result
     */
    const MuAnalysisResult& getMuAnalysis() const { return m_muResult; }
    
    /**
     * @brief Get peak μ value (upper bound)
     */
    double getPeakMu() const { return m_muResult.peakMuUpper; }
    
    /**
     * @brief Check if robust performance achieved
     */
    bool hasRobustPerformance() const { return m_muResult.robustPerformance(); }
    
    /**
     * @brief Get number of D-K iterations performed
     */
    int getIterationCount() const { return m_iterations; }
    
    /**
     * @brief Get controller state-space matrices
     */
    void getControllerMatrices(double* Ak, double* Bk, 
                              double* Ck, double* Dk) const;
    
    // ========================================================================
    // μ Analysis
    // ========================================================================
    
    /**
     * @brief Compute μ upper bound at single frequency
     * @param M Transfer function matrix at frequency
     * @param size Matrix dimension
     * @return Upper bound on μ
     */
    double computeMuUpperBound(const std::complex<double>* M, int size) const;
    
    /**
     * @brief Compute μ lower bound at single frequency
     * @param M Transfer function matrix at frequency
     * @param size Matrix dimension
     * @return Lower bound on μ
     */
    double computeMuLowerBound(const std::complex<double>* M, int size) const;
    
    /**
     * @brief Perform full μ analysis over frequency range
     */
    MuAnalysisResult analyzeMu(double freqMin, double freqMax, int numPoints);
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    // Plant dimensions
    int m_n{0}, m_m{0}, m_p{0};
    
    // Nominal plant matrices
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
    std::array<double, MAX_OUTPUT_DIM * MAX_CONTROL_DIM> m_D{};
    
    // Uncertainty structure
    std::vector<UncertaintyBlock> m_uncertaintyBlocks;
    int m_totalDeltaRows{0};
    int m_totalDeltaCols{0};
    
    // Performance weights
    WeightingFunction m_W1, m_W2, m_W3;
    bool m_hasW1{false}, m_hasW2{false}, m_hasW3{false};
    
    // Synthesized controller
    static constexpr int MAX_CONTROLLER_DIM = 2 * MAX_STATE_DIM;
    int m_nk{0};
    std::array<double, MAX_CONTROLLER_DIM * MAX_CONTROLLER_DIM> m_Ak{};
    std::array<double, MAX_CONTROLLER_DIM * MAX_OUTPUT_DIM> m_Bk{};
    std::array<double, MAX_CONTROL_DIM * MAX_CONTROLLER_DIM> m_Ck{};
    std::array<double, MAX_CONTROL_DIM * MAX_OUTPUT_DIM> m_Dk{};
    
    // Controller state
    std::array<double, MAX_CONTROLLER_DIM> m_xk{};
    
    // D-scales (for each frequency point)
    DScaleFit m_dScales;
    
    // Analysis results
    MuAnalysisResult m_muResult;
    int m_iterations{0};
    bool m_synthesized{false};
    
    // Internal methods
    void buildAugmentedPlant();
    bool dStep(const std::vector<double>& frequencies);
    bool kStep(double gamma);
    void fitDScales(const std::vector<double>& frequencies, int order);
    std::complex<double> evaluateClosedLoop(double omega) const;
};

// ============================================================================
// μ Analysis Utilities
// ============================================================================

/**
 * @brief Utilities for μ computation
 */
class MuAnalysis {
public:
    /**
     * @brief Compute μ upper bound using D-scaling
     * 
     * μ(M) ≤ inf σ̄(DMD⁻¹)
     *        D∈D
     */
    static double upperBound(const std::complex<double>* M, int n,
                            const std::vector<UncertaintyBlock>& blocks);
    
    /**
     * @brief Compute μ lower bound using power iteration
     * 
     * Uses structured perturbation to find destabilizing Δ
     */
    static double lowerBound(const std::complex<double>* M, int n,
                            const std::vector<UncertaintyBlock>& blocks);
    
    /**
     * @brief Find optimal D-scale at one frequency
     * 
     * Solves: min σ̄(DMD⁻¹) subject to D ∈ D
     *          D
     * 
     * @param M Transfer function matrix
     * @param n Dimension
     * @param blocks Uncertainty structure
     * @return Optimal D matrix (diagonal)
     */
    static std::vector<std::complex<double>> findOptimalDScale(
        const std::complex<double>* M, int n,
        const std::vector<UncertaintyBlock>& blocks);
    
    /**
     * @brief Fit rational transfer function to frequency data
     * @param frequencies Frequency points [rad/s]
     * @param values Complex values at frequencies
     * @param order Order of rational approximation
     * @return Coefficients (numerator, denominator)
     */
    static std::pair<std::vector<double>, std::vector<double>> fitRational(
        const std::vector<double>& frequencies,
        const std::vector<std::complex<double>>& values,
        int order);
};

// ============================================================================
// Structured Uncertainty Model
// ============================================================================

/**
 * @brief Model with structured uncertainty
 * 
 * Represents uncertain system as LFT (Linear Fractional Transformation):
 * G = F_u(M, Δ)
 */
class StructuredUncertainModel {
public:
    /**
     * @brief Set nominal system matrices
     */
    void setNominal(const double* A, const double* B, 
                   const double* C, const double* D,
                   int n, int m, int p);
    
    /**
     * @brief Add multiplicative input uncertainty
     * G_true = G * (I + W_I * Δ_I)
     */
    void addInputMultiplicative(const WeightingFunction& W, double bound = 1.0);
    
    /**
     * @brief Add multiplicative output uncertainty
     * G_true = (I + W_O * Δ_O) * G
     */
    void addOutputMultiplicative(const WeightingFunction& W, double bound = 1.0);
    
    /**
     * @brief Add additive uncertainty
     * G_true = G + W_A * Δ_A
     */
    void addAdditive(const WeightingFunction& W, double bound = 1.0);
    
    /**
     * @brief Add parametric uncertainty
     * @param paramName Parameter name
     * @param A_delta Sensitivity of A to parameter
     * @param B_delta Sensitivity of B to parameter
     * @param bound Relative uncertainty bound
     */
    void addParametric(const std::string& paramName,
                      const double* A_delta, const double* B_delta,
                      double bound);
    
    /**
     * @brief Build LFT representation
     * @return (M, Δ-structure) for μ-synthesis
     */
    void buildLFT(double* M11, double* M12, double* M21, double* M22,
                 std::vector<UncertaintyBlock>& blocks) const;
    
    /**
     * @brief Sample uncertain system
     * @return State-space matrices for one sample of uncertainty
     */
    void sampleUncertainty(double* A, double* B, double* C, double* D) const;
    
private:
    int m_n{0}, m_m{0}, m_p{0};
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
    std::array<double, MAX_OUTPUT_DIM * MAX_CONTROL_DIM> m_D{};
    
    struct UncertaintyDesc {
        enum Type { InputMult, OutputMult, Additive, Parametric };
        Type type;
        WeightingFunction weight;
        std::string name;
        double bound;
        std::vector<double> sensitivity_A;
        std::vector<double> sensitivity_B;
    };
    std::vector<UncertaintyDesc> m_uncertainties;
};

} // namespace tether::control
