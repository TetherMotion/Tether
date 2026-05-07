#pragma once

#include <complex>

#include <tether/identification/DenseLinearAlgebra.hpp>
#include <tether/identification/PolynomialModels.hpp>

namespace Identification {

/**
 * @brief Represents the types of static memoryless nonlinearities that can be applied.
 */
enum class StaticNonlinearityType {
    Linear,     ///< Standard linear mapping: y = gain * x + offset
    Saturation, ///< Limits the output between a lower and upper bound.
    DeadZone,   ///< Output remains zero if input is within the deadzone symmetric around origin.
    Cubic       ///< Nonlinear cubic mapping: y = gain * x + cubic * x^3 + offset
};

/**
 * @brief A structure that defines and applies a static, memoryless nonlinearity.
 *
 * This function block maps an input scalar directly to an output scalar 
 * using mathematical rules (like saturation, dead-zone, and polynomial logic) 
 * independent of past values (no state memory).
 */
struct StaticNonlinearity {
    StaticNonlinearityType type{StaticNonlinearityType::Linear}; ///< The form of the nonlinearity.
    double gain{1.0};                                            ///< Linear gain scaling the input.
    double offset{0.0};                                          ///< Fixed DC offset applied to the output.
    double lower_limit{-1.0};                                    ///< The minimum output bound (Saturation only).
    double upper_limit{1.0};                                     ///< The maximum output bound (Saturation only).
    double deadzone{0.0};                                        ///< The symmetric width of the deadzone (+/- value).
    double cubic{0.0};                                           ///< The coefficient of the cubic term (Cubic only).

    /**
     * @brief Evaluates the nonlinear function on a scalar value.
     * @param value The scalar input to process.
     * @return The scalar output evaluating the nonlinearity.
     */
    double apply(double value) const;
};

/**
 * @brief Represents a Hammerstein-Wiener block structured system model.
 * 
 * A Hammerstein-Wiener model describes a nonlinear system as a sequence of three blocks:
 * 1. A memoryless static input nonlinearity (Hammerstein element)
 * 2. A linear time-invariant dynamic block (typically modeled as a Discrete Polynomial Model)
 * 3. A memoryless static output nonlinearity (Wiener element)
 *
 * It is commonly used for modeling systems with actuators that saturate, sensors that 
 * have deadzones, or elements displaying odd harmonics.
 */
struct HammersteinWienerModel {
    StaticNonlinearity input_non_linearity;  ///< Non-linearity applied to input signals before the linear block.
    DiscretePolynomialModel linear_block;    ///< Dynamic LTI core of the model.
    StaticNonlinearity output_non_linearity; ///< Non-linearity applied to the linear block's output.

    /**
     * @brief Generates simulated output for the model given an input sequence.
     * @param input A vector of time-series input values.
     * @return The vector of simulated outputs shaped by the static nonlinearities and LTI dynamics.
     */
    Vector simulate(const Vector& input) const;
};

/**
 * @brief Encapsulates the results of an Empirical Transfer Function Estimate (ETFE).
 */
struct ETFEResult {
    Vector frequencies; /// Frequencies at which the magnitude, phase, and coherence were evaluated [rad/s].
    Vector magnitude;   /// Amplitude ratio between the input and output spectra.
    Vector phase;       /// Phase difference between the input and output spectra [rad].
    Vector coherence;   /// Squared magnitude squared coherence, measuring causality between input and output.
};

/**
 * @brief A class to identify and parametrize block-oriented Hammerstein-Wiener models.
 */
class HammersteinWienerIdentifier {
public:
    /**
     * @brief Identifies a Hammerstein-Wiener model from input-output data.
     * 
     * Uses optimization/iterative methods to untangle the cross-coupling of 
     * static non-linear boundaries and dynamic parameters. 
     * 
     * @param input The measured input data vector.
     * @param output The measured output data vector.
     * @param orders The required structure of the internal discrete polynomial LTI block.
     * @param input_type The shape template for the input non-linearity block.
     * @param output_type The shape template for the output non-linearity block.
     * @return The identified model struct containing parameterized elements.
     */
    static HammersteinWienerModel identify(const Vector& input,
                                           const Vector& output,
                                           const PolynomialModelOrders& orders,
                                           StaticNonlinearityType input_type,
                                           StaticNonlinearityType output_type);
};

/**
 * @brief Evaluates nonparametric black-box spectral models via the Empirical Transfer Function Estimate (ETFE).
 *
 * The ETFE applies Fast Fourier Transforms (FFT or DFT) directly to time-domain input
 * and output vectors to estimate the frequency response, $ G(e^{j\omega}) = \frac{Y(\omega)}{U(\omega)} $.
 */
class ETFEEstimator {
public:
    /**
     * @brief Calculates the Empirical Transfer Function.
     * 
     * The ETFE is susceptible to noise (high variance) but serves as a swift, unfiltered
     * check of system resonances and frequency domain logic.
     * 
     * @param input Timed input data sequence.
     * @param output Timed output data sequence.
     * @param sample_time The physical sampling period (e.g., dt) to convert index step to true frequencies.
     * @return Struct storing magnitudes, phases, and coherence over valid frequencies.
     */
    static ETFEResult estimate(const Vector& input,
                               const Vector& output,
                               double sample_time);
};

} // namespace Identification