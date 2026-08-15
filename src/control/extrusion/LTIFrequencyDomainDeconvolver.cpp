/**
 * @file LTIFrequencyDomainDeconvolver.cpp
 * @brief Regularized LTI frequency-domain deconvolution implementation.
 */

#include "tether/control/extrusion/LTIFrequencyDomainDeconvolver.hpp"

#include <unsupported/Eigen/FFT>

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

LTIFrequencyDomainDeconvolver::LTIFrequencyDomainDeconvolver(
    LTIDeconvolutionParams params)
    : params_(std::move(params)) {}

void LTIFrequencyDomainDeconvolver::setImpulseResponse(
    const std::vector<double>& h) {
    h_ = h;
    groupDelay_ = findPeakIndex(h);
    hInv_.clear();
    fftSize_ = 0;
}

void LTIFrequencyDomainDeconvolver::precomputeInverseFilter() {
    if (h_.empty()) return;

    // FFT size: at least 1 (we only need the inverse filter, not a full
    // convolution), but we use a reasonable size for spectral resolution.
    // Use next power of 2 ≥ max(h_.size(), 16) for decent frequency resolution.
    const int minSize = static_cast<int>(h_.size());
    const int N = computeFFTSize(minSize);
    fftSize_ = N;

    // Zero-pad h to length N.
    std::vector<double> hPadded(N, 0.0);
    std::copy(h_.begin(), h_.end(), hPadded.begin());

    // FFT of h.
    Eigen::FFT<double> fft;
    std::vector<std::complex<double>> H(N);
    fft.fwd(H, hPadded);

    // Regularized inverse spectrum: H_inv[k] = H*[k] / (|H[k]|^2 + lambda).
    std::vector<std::complex<double>> HInv(N);
    for (int k = 0; k < N; ++k) {
        const double magSq = std::norm(H[k]);  // |H[k]|^2
        const double denom = magSq + params_.lambda;
        // H*[k] / (|H[k]|^2 + lambda) = conj(H[k]) / denom
        HInv[k] = std::conj(H[k]) / denom;
    }

    // IFFT to get the time-domain inverse filter.
    std::vector<double> hInvPadded(N, 0.0);
    fft.inv(hInvPadded, HInv);

    // Store the inverse filter (full length N; truncation happens in
    // deconvolve() where we know the target length).
    hInv_ = hInvPadded;
}

std::vector<double> LTIFrequencyDomainDeconvolver::deconvolve(
    const std::vector<double>& y_tgt) const {
    if (hInv_.empty() || y_tgt.empty()) return {};

    const int Ly = static_cast<int>(y_tgt.size());
    const int Lh = static_cast<int>(h_.size());
    // Convolution length: Ly + Lh - 1, but we use the precomputed inverse
    // filter which has length fftSize_.  We need N >= Ly + fftSize_ - 1
    // for linear convolution.  Recompute if necessary.
    const int minN = Ly + static_cast<int>(hInv_.size()) - 1;
    const int N = computeFFTSize(minN);

    // Zero-pad y_tgt and h_inv to N.
    std::vector<double> yPadded(N, 0.0);
    std::copy(y_tgt.begin(), y_tgt.end(), yPadded.begin());

    std::vector<double> hInvPadded(N, 0.0);
    const int copyLen = std::min(static_cast<int>(hInv_.size()), N);
    std::copy_n(hInv_.begin(), copyLen, hInvPadded.begin());

    // FFT both.
    Eigen::FFT<double> fft;
    std::vector<std::complex<double>> Y(N), HInv(N);
    fft.fwd(Y, yPadded);
    fft.fwd(HInv, hInvPadded);

    // Multiply: X_req[k] = Y_tgt[k] * H_inv[k].
    std::vector<std::complex<double>> XReq(N);
    for (int k = 0; k < N; ++k) {
        XReq[k] = Y[k] * HInv[k];
    }

    // IFFT.
    std::vector<double> xPadded(N, 0.0);
    fft.inv(xPadded, XReq);

    // Truncate to Ly and shift left by group delay.
    std::vector<double> xReq(Ly, 0.0);
    for (int i = 0; i < Ly; ++i) {
        const int srcIdx = i + groupDelay_;
        if (srcIdx >= 0 && srcIdx < N) {
            xReq[i] = xPadded[srcIdx];
        }
    }
    return xReq;
}

std::vector<double> LTIFrequencyDomainDeconvolver::deconvolve(
    const std::vector<double>& y_tgt, const std::vector<double>& h) {
    setImpulseResponse(h);
    precomputeInverseFilter();
    return deconvolve(y_tgt);
}

void LTIFrequencyDomainDeconvolver::setLambda(double lambda) {
    params_.lambda = lambda;
    if (!h_.empty()) precomputeInverseFilter();
}

void LTIFrequencyDomainDeconvolver::reset() {
    h_.clear();
    hInv_.clear();
    groupDelay_ = 0;
    fftSize_ = 0;
}

int LTIFrequencyDomainDeconvolver::computeFFTSize(int minSize) const {
    if (!params_.padToPowerOfTwo) return std::max(minSize, 1);
    int n = 1;
    while (n < minSize) n <<= 1;
    return n;
}

int LTIFrequencyDomainDeconvolver::findPeakIndex(
    const std::vector<double>& h) const {
    if (h.empty()) return 0;
    int peakIdx = 0;
    double peakVal = h[0];
    for (int i = 1; i < static_cast<int>(h.size()); ++i) {
        if (std::abs(h[i]) > std::abs(peakVal)) {
            peakVal = h[i];
            peakIdx = i;
        }
    }
    return peakIdx;
}

} // namespace tether::control::extrusion
