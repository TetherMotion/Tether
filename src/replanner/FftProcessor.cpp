// SPDX-License-Identifier: MIT

/**
 * @file FftProcessor.cpp
 * @brief Implementation of FFT and spectral analysis helpers
 */

#include "tether/motion_replanner/FftProcessor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tether::motion::replanner {

//=============================================================================
// Private helpers
//=============================================================================

double FftProcessor::mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

//=============================================================================
// FFT
//=============================================================================

void FftProcessor::fft(std::vector<std::complex<double>>& data) {
    auto n = data.size();
    if (n <= 1) return;

    // Bit-reversal permutation
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    // Butterfly operations
    for (std::size_t len = 2; len <= n; len <<= 1) {
        double angle = -2.0 * M_PI / static_cast<double>(len);
        std::complex<double> wlen(std::cos(angle), std::sin(angle));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (std::size_t j = 0; j < len / 2; ++j) {
                auto u = data[i + j];
                auto v = data[i + j + len / 2] * w;
                data[i + j] = u + v;
                data[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

//=============================================================================
// Window functions
//=============================================================================

void FftProcessor::applyWindow(std::vector<double>& signal,
                                FFTConfig::Window window) {
    auto n = signal.size();
    if (n < 2) return;

    for (std::size_t i = 0; i < n; ++i) {
        double w = 1.0;
        double x = static_cast<double>(i) / static_cast<double>(n - 1);

        switch (window) {
            case FFTConfig::Window::Rectangular:
                w = 1.0;
                break;
            case FFTConfig::Window::Hann:
                w = 0.5 * (1.0 - std::cos(2.0 * M_PI * x));
                break;
            case FFTConfig::Window::Hamming:
                w = 0.54 - 0.46 * std::cos(2.0 * M_PI * x);
                break;
            case FFTConfig::Window::Blackman:
                w = 0.42 - 0.5 * std::cos(2.0 * M_PI * x)
                    + 0.08 * std::cos(4.0 * M_PI * x);
                break;
        }
        signal[i] *= w;
    }
}

//=============================================================================
// Detrending
//=============================================================================

void FftProcessor::detrend(std::vector<double>& signal,
                            bool removeDC, bool removeLinear) {
    if (signal.empty()) return;

    if (removeLinear) {
        // Least-squares linear fit: y = a + b*x
        double n = static_cast<double>(signal.size());
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (std::size_t i = 0; i < signal.size(); ++i) {
            double x = static_cast<double>(i);
            sx += x; sy += signal[i];
            sxx += x * x; sxy += x * signal[i];
        }
        double denom = n * sxx - sx * sx;
        if (std::abs(denom) > 1e-15) {
            double b = (n * sxy - sx * sy) / denom;
            double a = (sy - b * sx) / n;
            for (std::size_t i = 0; i < signal.size(); ++i) {
                signal[i] -= (a + b * static_cast<double>(i));
            }
            return;
        }
    }

    if (removeDC) {
        double m = mean(signal);
        for (double& v : signal) v -= m;
    }
}

//=============================================================================
// Peak detection
//=============================================================================

std::vector<SpectralPeak> FftProcessor::findPeaks(
    const std::vector<double>& freqs,
    const std::vector<double>& mags,
    const std::vector<double>& phases,
    const std::vector<double>& psd,
    std::size_t maxPeaks,
    double prominenceThreshold) {

    if (mags.size() < 3) return {};

    // Find local maxima (skip DC bin 0 and Nyquist)
    std::vector<std::pair<std::size_t, double>> candidates;
    for (std::size_t i = 1; i < mags.size() - 1; ++i) {
        if (mags[i] > mags[i - 1] && mags[i] > mags[i + 1]) {
            // Compute prominence: height above the higher of the two
            // surrounding valleys (simplified: just compare to neighbors)
            double leftValley = mags[i - 1];
            double rightValley = mags[i + 1];
            double valley = std::max(leftValley, rightValley);
            double prominence = mags[i] - valley;
            candidates.emplace_back(i, prominence);
        }
    }

    // Sort by prominence descending
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Filter by prominence threshold (relative to max magnitude)
    double maxMag = 0.0;
    for (double m : mags) maxMag = std::max(maxMag, m);
    double promThresh = prominenceThreshold * maxMag;

    std::vector<SpectralPeak> peaks;
    for (const auto& [idx, prom] : candidates) {
        if (prom < promThresh) break;
        if (peaks.size() >= maxPeaks) break;

        SpectralPeak p;
        p.frequency = freqs[idx];
        p.magnitude = mags[idx];
        p.power = psd[idx];
        p.phase = phases[idx];
        p.prominence = prom;
        peaks.push_back(p);
    }

    return peaks;
}

} // namespace tether::motion::replanner
