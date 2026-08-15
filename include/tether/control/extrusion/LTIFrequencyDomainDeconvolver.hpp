/**
 * @file LTIFrequencyDomainDeconvolver.hpp
 * @brief Regularized LTI frequency-domain deconvolution (Wiener/Tikhonov).
 *
 * @details
 * Given a measured LTI impulse response h[n] and a target output trajectory
 * y_tgt[n], computes the required input x_req[n] such that the convolution
 * (x_req * h)[n] approximates y_tgt[n].
 *
 * In the frequency domain:  Y[k] = H[k]·X[k]  →  X_req[k] = Y_tgt[k] / H[k]
 *
 * Direct division is ill-posed because |H[k]| → 0 at high frequencies
 * (the system is low-pass).  Tikhonov regularization stabilises the
 * inversion:
 *
 *   X_req[k] = Y_tgt[k] · H*[k] / (|H[k]|² + λ)
 *
 * where λ > 0 is the regularization parameter.  This is equivalent to the
 * Wiener filter when λ represents the noise-to-signal ratio.
 *
 * Algorithm:
 *   1. Zero-pad h and y_tgt to N ≥ L_y + L_h − 1 (next power of 2 for FFT).
 *   2. FFT both.
 *   3. Regularized spectral division.
 *   4. IFFT.
 *   5. Truncate to L_y and shift left by the group delay (peak index of h).
 *
 * The regularized inverse filter h_inv[n] = IFFT(H*[k] / (|H[k]|² + λ)) can
 * be precomputed once and reused for multiple target trajectories via a
 * simple time-domain convolution.
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md
 */

#pragma once

#include <complex>
#include <vector>

namespace tether::control::extrusion {

/// @brief Parameters for the LTI frequency-domain deconvolver.
struct LTIDeconvolutionParams {
    /// @brief Tikhonov regularization parameter λ > 0.
    /// Larger values produce smoother (less aggressive) input commands
    /// at the cost of tracking accuracy.  Typical range: 1e-8 … 1e-2.
    double lambda = 1e-6;

    /// @brief If true, zero-pad to the next power of two for FFT efficiency.
    /// If false, pad to exactly L_y + L_h − 1.
    bool padToPowerOfTwo = true;
};

/// @brief Regularized LTI frequency-domain deconvolver.
///
/// Usage:
///   1. setImpulseResponse(h)  — set the measured system impulse response.
///   2. precomputeInverseFilter() — build the regularized inverse.
///   3. deconvolve(y_tgt) — compute the required input for a target trajectory.
///
/// Alternatively, deconvolve(y_tgt, h) performs all three steps in one call.
class LTIFrequencyDomainDeconvolver {
public:
    explicit LTIFrequencyDomainDeconvolver(LTIDeconvolutionParams params = {});

    /// @brief Set the measured LTI impulse response h[n].
    void setImpulseResponse(const std::vector<double>& h);

    /// @brief Precompute the regularized inverse filter h_inv[n].
    /// Must be called after setImpulseResponse() and before deconvolve().
    void precomputeInverseFilter();

    /// @brief Deconvolve a target trajectory using the precomputed inverse.
    /// @param y_tgt Target output trajectory of length L_y.
    /// @return Required input x_req of length L_y (causally aligned).
    std::vector<double> deconvolve(const std::vector<double>& y_tgt) const;

    /// @brief One-shot deconvolution: set h, precompute, and deconvolve.
    /// @param y_tgt Target output trajectory.
    /// @param h Measured impulse response.
    /// @return Required input x_req.
    std::vector<double> deconvolve(const std::vector<double>& y_tgt,
                                   const std::vector<double>& h);

    /// @return The group delay (index of the peak of h[n]).
    int groupDelay() const { return groupDelay_; }

    /// @return The precomputed regularized inverse filter.
    const std::vector<double>& inverseFilter() const { return hInv_; }

    /// @return The current parameters.
    const LTIDeconvolutionParams& params() const { return params_; }

    /// @brief Update the regularization parameter and recompute the inverse.
    void setLambda(double lambda);

    /// @brief Reset all state.
    void reset();

private:
    LTIDeconvolutionParams params_;
    std::vector<double> h_;       ///< impulse response
    std::vector<double> hInv_;    ///< regularized inverse filter
    int groupDelay_ = 0;          ///< peak index of h
    int fftSize_ = 0;             ///< last FFT size used

    /// @brief Compute the next valid FFT size ≥ minSize.
    int computeFFTSize(int minSize) const;

    /// @brief Find the index of the peak of h (group delay estimate).
    int findPeakIndex(const std::vector<double>& h) const;
};

} // namespace tether::control::extrusion
