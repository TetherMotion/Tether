// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file FftProcessor.hpp
 * @brief FFT and spectral analysis helpers extracted from PathRelativeFFT
 *
 * @details
 * Encapsulates the signal-processing sub-responsibility of PathRelativeFFT:
 *  - Cooley-Tukey radix-2 FFT (in-place, complex)
 *  - Window functions (Rectangular, Hann, Hamming, Blackman)
 *  - Detrending (DC removal, linear trend removal)
 *  - Spectral peak detection with prominence filtering
 *
 * All methods are static (stateless) and operate on raw data vectors.
 */

#include "tether/motion_replanner/PathRelativeFFT.hpp" // FFTConfig, SpectralPeak

#include <complex>
#include <vector>

namespace tether::motion::replanner {

class FftProcessor {
public:
    /// Cooley-Tukey radix-2 FFT (in-place, complex).
    /// @param data Must be a power-of-2 sized complex vector.
    static void fft(std::vector<std::complex<double>>& data);

    /// Apply a window function to a real-valued signal.
    /// @param signal The signal (modified in-place).
    /// @param window Window type.
    static void applyWindow(std::vector<double>& signal,
                            FFTConfig::Window window);

    /// Detrend: remove DC and/or linear trend.
    /// @param signal The signal (modified in-place).
    /// @param removeDC If true, subtract the mean.
    /// @param removeLinear If true, remove a least-squares linear fit.
    static void detrend(std::vector<double>& signal,
                        bool removeDC, bool removeLinear);

    /// Find the top N spectral peaks with prominence filtering.
    /// @param freqs Frequency bins (Hz).
    /// @param mags Magnitude spectrum.
    /// @param phases Phase spectrum (radians).
    /// @param psd Power spectral density.
    /// @param maxPeaks Maximum number of peaks to return.
    /// @param prominenceThreshold Minimum prominence (fraction of max).
    /// @return Vector of spectral peaks, sorted by prominence (descending).
    static std::vector<SpectralPeak> findPeaks(
        const std::vector<double>& freqs,
        const std::vector<double>& mags,
        const std::vector<double>& phases,
        const std::vector<double>& psd,
        std::size_t maxPeaks,
        double prominenceThreshold);

private:
    /// Compute the arithmetic mean of a vector.
    static double mean(const std::vector<double>& v);
};

} // namespace tether::motion::replanner
