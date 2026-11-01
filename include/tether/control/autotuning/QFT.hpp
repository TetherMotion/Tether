/**
 * @file QFT.hpp
 * @brief Quantitative Feedback Theory (QFT) Controller Design
 * 
 * @details
 * This file implements Quantitative Feedback Theory (QFT), a frequency-domain
 * robust control design method developed by Isaac Horowitz. QFT provides a
 * systematic approach to designing controllers that meet specifications despite
 * plant uncertainty.
 * 
 * ## Theory Overview
 * 
 * ### The QFT Paradigm
 * QFT addresses the problem of designing a controller C(s) and prefilter F(s)
 * for an uncertain plant P(s) to achieve:
 * 
 * 1. **Robust Stability**: Closed-loop stable for all P ∈ P (plant family)
 * 2. **Robust Tracking**: Output tracks reference within specified bounds
 * 3. **Robust Disturbance Rejection**: Disturbances attenuated appropriately
 * 4. **Control Effort Limits**: Reasonable actuator signals
 * 
 * ### Key Concepts
 * 
 * #### Plant Templates
 * For each frequency ω, the plant template T(ω) is the set of all possible
 * plant values: T(ω) = {P(jω) : P ∈ P}
 * 
 * #### Nichols Chart
 * QFT design is performed on the Nichols chart (gain vs. phase) where:
 * - Loop transmission L = PC is plotted
 * - Closed-loop M-circles and N-circles help visualize performance
 * 
 * #### Bounds
 * Specifications are translated into bounds on L(jω) in the Nichols chart:
 * - Tracking bounds from |T_L| ≤ |F*L/(1+L)| ≤ |T_U|
 * - Stability bounds from |L/(1+L)| ≤ M_max
 * - Disturbance bounds from |1/(1+L)| ≤ specified
 * 
 * ### Design Procedure
 * 
 * 1. **Model Uncertainty**: Define plant family P
 * 2. **Specifications**: Define tracking, stability, disturbance requirements
 * 3. **Templates**: Compute plant templates at each frequency
 * 4. **Bounds**: Translate specs to bounds on Nichols chart
 * 5. **Loop Shaping**: Design L(s) = P_0(s)C(s) to satisfy all bounds
 * 6. **Prefilter**: Design F(s) for tracking specifications
 * 
 * ## Usage Example
 * 
 * ```cpp
 * QFTController qft;
 * 
 * // Define uncertain plant
 * qft.setNominalPlant(K_nom, tau_nom, delay_nom);  // FOPDT
 * qft.setUncertaintyRanges(K_min, K_max, tau_min, tau_max, delay_max);
 * 
 * // Set specifications
 * qft.setTrackingSpec(T_lower, T_upper);  // Tracking bounds
 * qft.setDisturbanceSpec(max_sensitivity); // |S| < spec
 * qft.setStabilityMargin(6.0);  // Gain margin in dB
 * 
 * // Design
 * qft.computeTemplates();
 * qft.computeBounds();
 * qft.shapeLoop();  // Automatic or interactive
 * qft.designPrefilter();
 * 
 * // Use
 * double u = qft.compute(r, y, dt);
 * ```
 * 
 * @see HInfinityController
 * @see MuSynthesisController
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "../ControllerBase.hpp"
#include "AutotuningFramework.hpp"
#include <vector>
#include <complex>
#include <utility>
#include <functional>

namespace tether::control {

// ============================================================================
// QFT Data Structures
// ============================================================================

/**
 * @brief Nichols chart point (gain in dB, phase in degrees)
 */
struct NicholsPoint {
    double gain{0.0};   ///< Gain [dB]
    double phase{0.0};  ///< Phase [degrees]
    
    NicholsPoint() = default;
    NicholsPoint(double g, double p) : gain(g), phase(p) {}
    
    static NicholsPoint fromComplex(std::complex<double> z);
    std::complex<double> toComplex() const;
};

/**
 * @brief Plant template at a frequency
 */
struct PlantTemplate {
    double frequency{0.0};  ///< Frequency [rad/s]
    std::vector<NicholsPoint> points;  ///< Template points
    NicholsPoint nominal;   ///< Nominal plant point
    
    /**
     * @brief Get template boundary (convex hull)
     * @return Convex hull of the template points in Nichols space.
     */
    std::vector<NicholsPoint> boundary() const;
    
    /**
     * @brief Check if point is inside template
     * @param point Nichols point to test.
     * @return true when the point lies inside the convex hull.
     */
    bool contains(const NicholsPoint& point) const;
};

/**
 * @brief QFT specification bound at a frequency
 */
struct QFTBound {
    double frequency{0.0};
    std::vector<NicholsPoint> boundary;  ///< Forbidden region boundary
    enum Type {
        Tracking,
        Stability,
        Disturbance,
        ControlEffort,
        Combined
    } type{Combined};
    
    /**
     * @brief Check if loop point satisfies bound
     * @param loopPoint Loop transmission point at this frequency.
     * @return true when the point lies outside the forbidden region.
     */
    bool isSatisfied(const NicholsPoint& loopPoint) const;
};

/**
 * @brief Tracking specification
 */
struct TrackingSpec {
    std::vector<double> frequencies;
    std::vector<double> lowerBound;  ///< |T(jω)| ≥ lower [dB]
    std::vector<double> upperBound;  ///< |T(jω)| ≤ upper [dB]
};

/**
 * @brief Disturbance rejection specification
 */
struct DisturbanceSpec {
    std::vector<double> frequencies;
    std::vector<double> maxSensitivity;  ///< |S(jω)| ≤ spec [dB]
};

/**
 * @brief Transfer function representation
 */
struct TransferFunction {
    std::vector<double> num;  ///< Numerator coefficients (highest order first)
    std::vector<double> den;  ///< Denominator coefficients
    
    std::complex<double> evaluate(double omega) const;
    int order() const { return static_cast<int>(den.size()) - 1; }
    
    static TransferFunction fromZPK(const std::vector<std::complex<double>>& zeros,
                                    const std::vector<std::complex<double>>& poles,
                                    double gain);
};

// ============================================================================
// QFT Controller
// ============================================================================

/**
 * @brief Quantitative Feedback Theory (QFT) Controller
 * 
 * Implements QFT design methodology for robust control with plant uncertainty.
 * Design is performed in the frequency domain on the Nichols chart.
 */
class QFTController : public ControllerBase {
public:
    ControllerType getType() const override { return ControllerType::Custom; }
    const char* getName() const override { return "QFT Controller"; }
    const char* getDescription() const override {
        return "Quantitative Feedback Theory robust controller. "
               "Designs loop gain to satisfy bounds on Nichols chart. "
               "Handles plant uncertainty explicitly via templates.";
    }
    
    // ========================================================================
    // Plant Uncertainty Definition
    // ========================================================================
    
    /**
     * @brief Set nominal FOPDT plant
     * G(s) = K * exp(-L*s) / (τs + 1)
     *
     * @param K Process gain.
     * @param tau Time constant τ [s].
     * @param L Dead time L [s].
     */
    void setNominalPlant(double K, double tau, double L);
    
    /**
     * @brief Set nominal SOPDT plant
     * G(s) = K * exp(-L*s) / ((τ₁s + 1)(τ₂s + 1))
     *
     * @param K Process gain.
     * @param tau1 First time constant τ₁ [s].
     * @param tau2 Second time constant τ₂ [s].
     * @param L Dead time L [s].
     */
    void setNominalPlant(double K, double tau1, double tau2, double L);
    
    /**
     * @brief Set nominal plant as transfer function
     * @param tf Nominal plant P_nom(s).
     */
    void setNominalPlant(const TransferFunction& tf);
    
    /**
     * @brief Set nominal plant as state-space
     * @param A State matrix (n×n, row-major).
     * @param B Input matrix (n×m, row-major).
     * @param C Output matrix (p×n, row-major).
     * @param D Feedthrough matrix (p×m, row-major).
     * @param n State dimension.
     * @param m Input dimension.
     * @param p Output dimension.
     */
    void setNominalPlant(const double* A, const double* B,
                        const double* C, const double* D,
                        int n, int m, int p);
    
    /**
     * @brief Define FOPDT uncertainty ranges
     * @param Kmin Minimum process gain.
     * @param Kmax Maximum process gain.
     * @param tauMin Minimum time constant [s].
     * @param tauMax Maximum time constant [s].
     * @param Lmin Minimum dead time [s].
     * @param Lmax Maximum dead time [s].
     */
    void setFOPDTUncertainty(double Kmin, double Kmax,
                             double tauMin, double tauMax,
                             double Lmin, double Lmax);
    
    /**
     * @brief Define parametric uncertainty
     * @param paramIndex Index of uncertain parameter
     * @param minValue Minimum value
     * @param maxValue Maximum value
     * @param numSamples Samples for template generation
     */
    void addParametricUncertainty(int paramIndex, double minValue, 
                                  double maxValue, int numSamples = 5);
    
    /**
     * @brief Set multiplicative uncertainty model
     * P_true = P_nom * (1 + W*Δ), |Δ| ≤ 1
     * @param W Uncertainty weight shaping the bound.
     */
    void setMultiplicativeUncertainty(const TransferFunction& W);
    
    /**
     * @brief Directly provide plant family samples
     * @param plants Plant variants used to build the templates.
     */
    void setPlantSamples(const std::vector<TransferFunction>& plants);
    
    // ========================================================================
    // Specifications
    // ========================================================================
    
    /**
     * @brief Set tracking specification
     * @param freqs Frequency points
     * @param lower Lower bound |T(jω)| [dB]
     * @param upper Upper bound |T(jω)| [dB]
     */
    void setTrackingSpec(const std::vector<double>& freqs,
                        const std::vector<double>& lower,
                        const std::vector<double>& upper);
    
    /**
     * @brief Set tracking with standard second-order bounds
     * @param bandwidth Bandwidth [rad/s]
     * @param dampingRange (min, max) damping ratios
     */
    void setTrackingSpec(double bandwidth, 
                        std::pair<double, double> dampingRange = {0.4, 0.8});
    
    /**
     * @brief Set disturbance rejection specification
     * |S(jω)| ≤ spec(ω)
     * @param freqs Frequency points [rad/s].
     * @param maxSensitivity Maximum |S| at each frequency.
     */
    void setDisturbanceSpec(const std::vector<double>& freqs,
                           const std::vector<double>& maxSensitivity);
    
    /**
     * @brief Set simple disturbance spec
     * |S| ≤ Smax below crossover, roll off above
     * @param Smax Maximum sensitivity below the crossover frequency.
     * @param crossover Frequency below which the bound applies [rad/s].
     */
    void setDisturbanceSpec(double Smax, double crossover);
    
    /**
     * @brief Set stability margins
     * @param gainMargin Minimum gain margin [dB]
     * @param phaseMargin Minimum phase margin [degrees]
     */
    void setStabilityMargins(double gainMargin, double phaseMargin);
    
    /**
     * @brief Set control effort bound
     * |C*S| ≤ bound
     * @param bound Transfer function bounding controller output magnitude
     *        per frequency
     */
    void setControlEffortSpec(const TransferFunction& bound);
    
    // ========================================================================
    // Template and Bound Computation
    // ========================================================================
    
    /**
     * @brief Design frequency points
     * @param freqs Frequency grid the bounds and templates are
     *        evaluated on [rad/s]
     */
    void setDesignFrequencies(const std::vector<double>& freqs);

    /**
     * @brief Auto-select design frequencies based on plant
     * @param numPoints Number of logarithmically spaced points
     */
    void autoSelectFrequencies(int numPoints = 20);
    
    /**
     * @brief Compute plant templates at design frequencies
     */
    void computeTemplates();
    
    /**
     * @brief Compute QFT bounds at design frequencies
     */
    void computeBounds();
    
    /**
     * @brief Get computed templates
     * @return Plant templates at the design frequencies.
     */
    const std::vector<PlantTemplate>& getTemplates() const { return m_templates; }
    
    /**
     * @brief Get computed bounds
     * @return QFT bounds at the design frequencies.
     */
    const std::vector<QFTBound>& getBounds() const { return m_bounds; }
    
    // ========================================================================
    // Loop Shaping
    // ========================================================================
    
    /**
     * @brief Automatic loop shaping using optimization
     * @return true if successful design found
     */
    bool autoShapeLoop();
    
    /**
     * @brief Configuration for automatic loop shaping
     */
    struct LoopShapingConfig {
        int maxPoles{6};            ///< Maximum controller poles
        int maxZeros{5};            ///< Maximum controller zeros
        bool allowUnstablePoles{false};  ///< Allow RHP poles
        double integralLowFreq{0.1}; ///< Frequency for integral action
        double filterHighFreq{100.0}; ///< High-frequency rolloff
        int optimizationIterations{1000};
    };
    
    /**
     * @brief Auto shape with configuration
     * @param config Search space and iteration budget for the shaper.
     * @return true when a bound-satisfying controller was found.
     */
    bool autoShapeLoop(const LoopShapingConfig& config);
    
    /**
     * @brief Manually set controller
     * @param C Loop controller C(s) to evaluate against the bounds.
     */
    void setController(const TransferFunction& C);
    
    /**
     * @brief Add controller element
     * @param pole Real pole location to append.
     */
    void addControllerPole(double pole);
    /// @param zero Real zero location to append.
    void addControllerZero(double zero);
    void addControllerPole(std::complex<double> pole);  // Complex pair
    void addControllerZero(std::complex<double> zero);  // Complex pair
    void setControllerGain(double gain);
    
    /**
     * @brief Get designed controller
     * @return Controller C(s) currently being evaluated.
     */
    const TransferFunction& getController() const { return m_controller; }
    
    /**
     * @brief Check if all bounds satisfied
     * @return true when the controller meets every bound at every frequency.
     */
    bool checkBounds() const;
    
    /**
     * @brief Get bound satisfaction at each frequency
     * @return Per-frequency satisfaction flags (design frequency order).
     */
    std::vector<bool> getBoundSatisfaction() const;
    
    // ========================================================================
    // Prefilter Design
    // ========================================================================
    
    /**
     * @brief Automatic prefilter design for tracking
     */
    void designPrefilter();
    
    /**
     * @brief Design prefilter with specified bandwidth
     * @param bandwidth Desired closed-loop bandwidth [rad/s].
     * @param order Filter order (1-3).
     */
    void designPrefilter(double bandwidth, int order = 2);
    
    /**
     * @brief Manually set prefilter
     * @param F Prefilter F(s) applied to the reference.
     */
    void setPrefilter(const TransferFunction& F);
    
    /**
     * @brief Get designed prefilter
     * @return Reference prefilter F(s).
     */
    const TransferFunction& getPrefilter() const { return m_prefilter; }
    
    // ========================================================================
    // Analysis
    // ========================================================================
    
    /**
     * @brief Get loop transmission L(jω) = P_nom(jω) * C(jω)
     * @param omega Frequency [rad/s].
     * @return Complex loop transmission at @p omega.
     */
    std::complex<double> evaluateLoop(double omega) const;
    
    /**
     * @brief Get closed-loop response T(jω) = L/(1+L)
     * @param omega Frequency [rad/s].
     * @return Complex closed-loop response at @p omega.
     */
    std::complex<double> evaluateClosedLoop(double omega) const;
    
    /**
     * @brief Get sensitivity S(jω) = 1/(1+L)
     * @param omega Frequency [rad/s].
     * @return Complex sensitivity at @p omega.
     */
    std::complex<double> evaluateSensitivity(double omega) const;
    
    /**
     * @brief Analyze stability margins
     */
    struct StabilityMargins {
        double gainMargin{0.0};      ///< [dB]
        double phaseMargin{0.0};     ///< [degrees]
        double gainCrossover{0.0};   ///< [rad/s]
        double phaseCrossover{0.0};  ///< [rad/s]
    };
    StabilityMargins analyzeMargins() const;
    
    /**
     * @brief Get worst-case closed-loop response over uncertainty
     * @param omega Frequency [rad/s].
     * @return (magnitude, phase) pairs of the worst-case plant response.
     */
    std::pair<std::vector<double>, std::vector<double>> 
        worstCaseResponse(double omega) const;
    
protected:
    ControllerOutput computeImpl(const ControllerInput& input) override;
    void resetImpl() override;
    
private:
    // Nominal plant
    TransferFunction m_nominalPlant;
    int m_n{0}, m_m{1}, m_p{1};
    
    // State-space for complex plants
    std::array<double, MAX_STATE_DIM * MAX_STATE_DIM> m_A{};
    std::array<double, MAX_STATE_DIM * MAX_CONTROL_DIM> m_B{};
    std::array<double, MAX_OUTPUT_DIM * MAX_STATE_DIM> m_C{};
    std::array<double, MAX_OUTPUT_DIM * MAX_CONTROL_DIM> m_D{};
    
    // Plant family
    std::vector<TransferFunction> m_plantSamples;
    
    // FOPDT uncertainty parameters
    struct FOPDTBounds {
        double Kmin{0.5}, Kmax{2.0};
        double tauMin{0.5}, tauMax{2.0};
        double Lmin{0.0}, Lmax{0.2};
    } m_fopdtBounds;
    bool m_useFOPDT{false};
    
    // Specifications
    TrackingSpec m_trackingSpec;
    DisturbanceSpec m_disturbanceSpec;
    double m_gainMarginSpec{6.0};    // dB
    double m_phaseMarginSpec{45.0};  // degrees
    
    // Design frequencies
    std::vector<double> m_designFreqs;
    
    // Computed data
    std::vector<PlantTemplate> m_templates;
    std::vector<QFTBound> m_bounds;
    
    // Controller and prefilter
    TransferFunction m_controller;
    TransferFunction m_prefilter;
    
    // Controller state (discrete-time implementation)
    std::vector<double> m_controllerState;
    std::vector<double> m_prefilterState;
    double m_dt{0.001};
    
    // Internal methods
    void generateFOPDTSamples(int numSamples);
    QFTBound computeTrackingBound(double omega, const PlantTemplate& templ) const;
    QFTBound computeStabilityBound(double omega, const PlantTemplate& templ) const;
    QFTBound computeDisturbanceBound(double omega, const PlantTemplate& templ) const;
    QFTBound combineBounds(const std::vector<QFTBound>& bounds) const;
    double boundViolation(const TransferFunction& C) const;
    void discretizeController(double dt);
};

// ============================================================================
// QFT Utilities
// ============================================================================

/**
 * @brief Nichols chart utilities
 */
class NicholsChart {
public:
    /**
     * @brief Generate M-circle (constant closed-loop magnitude)
     * @param M Magnitude [dB]
     * @return Points on Nichols chart
     */
    static std::vector<NicholsPoint> mCircle(double M);
    
    /**
     * @brief Generate N-circle (constant closed-loop phase)
     * @param N Phase [degrees]
     * @return Points on Nichols chart
     */
    static std::vector<NicholsPoint> nCircle(double N);
    
    /**
     * @brief Compute closed-loop magnitude from Nichols point
     * @param point Open-loop point (gain dB, phase deg)
     * @return |T| = |L/(1+L)| closed-loop magnitude [dB]
     */
    static double closedLoopMagnitude(const NicholsPoint& point);

    /**
     * @brief Compute closed-loop phase from Nichols point
     * @param point Open-loop point (gain dB, phase deg)
     * @return Closed-loop phase [degrees]
     */
    static double closedLoopPhase(const NicholsPoint& point);

    /**
     * @brief Check if point is in right-half Nichols plane (unstable)
     * @param point Open-loop point to test
     * @return `true` when the point lies in the Nichols-plane region
     *         indicating instability
     */
    static bool isUnstableRegion(const NicholsPoint& point);
};

/**
 * @brief Template manipulation utilities
 */
class TemplateUtils {
public:
    /**
     * @brief Compute convex hull of template
     * @param points Template boundary points
     * @return Convex-hull vertices enclosing the uncertainty region
     */
    static std::vector<NicholsPoint> convexHull(
        const std::vector<NicholsPoint>& points);

    /**
     * @brief Shift template by loop gain
     * @param templ Original template
     * @param loopGain Loop gain to add [dB, degrees]
     * @return Template displaced by `loopGain`
     */
    static PlantTemplate shiftTemplate(const PlantTemplate& templ,
                                       const NicholsPoint& loopGain);

    /**
     * @brief Interpolate template at intermediate frequency
     * @param t1 Lower-frequency template
     * @param t2 Higher-frequency template
     * @param omega Frequency to interpolate at [rad/s]
     * @return Template interpolated between t1 and t2 at `omega`
     */
    static PlantTemplate interpolateTemplate(const PlantTemplate& t1,
                                             const PlantTemplate& t2,
                                             double omega);
};

/**
 * @brief QFT loop shaping optimizer
 */
class QFTLoopShaper {
public:
    /**
     * @brief Configuration
     */
    struct Config {
        int maxPoles = 6;
        int maxZeros = 5;
        double gainMin = 0.001;
        double gainMax = 1000.0;
        double poleMin = 0.001;
        double poleMax = 1000.0;
        int populationSize = 50;
        int generations = 200;
        
        static Config defaultConfig() { return Config{}; }
    };
    
    /**
     * @brief Optimize controller to satisfy QFT bounds
     * @param nominalPlant Nominal plant model (center of the template)
     * @param bounds Horowitz bounds to satisfy
     * @param frequencies Design frequency grid [rad/s]
     * @param config Optimizer structure/search limits
     * @return Shaped controller transfer function
     */
    static TransferFunction optimize(
        const TransferFunction& nominalPlant,
        const std::vector<QFTBound>& bounds,
        const std::vector<double>& frequencies,
        const Config& config);

    /**
     * @brief Optimize with default configuration
     * @param nominalPlant Nominal plant model
     * @param bounds Horowitz bounds to satisfy
     * @param frequencies Design frequency grid [rad/s]
     * @return Shaped controller transfer function
     */
    static TransferFunction optimize(
        const TransferFunction& nominalPlant,
        const std::vector<QFTBound>& bounds,
        const std::vector<double>& frequencies) {
        return optimize(nominalPlant, bounds, frequencies, Config{});
    }
};

} // namespace tether::control
