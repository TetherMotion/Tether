/**
 * @file OverlapAddLpvDeconvolver.cpp
 * @brief Gain-scheduled overlap-add LPV deconvolution implementation.
 */

#include "tether/control/extrusion/OverlapAddLpvDeconvolver.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

OverlapAddLpvDeconvolver::OverlapAddLpvDeconvolver(OverlapAddLpvParams params)
    : params_(std::move(params)) {}

void OverlapAddLpvDeconvolver::addOperatingPoint(
    double p, const std::vector<double>& h) {
    LtiFrequencyDomainDeconvolver lti;
    LtiDeconvolutionParams lp;
    lp.lambda = params_.lambda;
    lti = LtiFrequencyDomainDeconvolver(lp);
    lti.setImpulseResponse(h);
    lti.precomputeInverseFilter();
    inverseFilterLut_[p] = lti.inverseFilter();
}

void OverlapAddLpvDeconvolver::addInverseFilter(
    double p, const std::vector<double>& hInv) {
    inverseFilterLut_[p] = hInv;
}

std::vector<double> OverlapAddLpvDeconvolver::deconvolve(
    const std::vector<double>& y_tgt, const std::vector<double>& p) const {
    if (y_tgt.empty() || inverseFilterLut_.empty()) return {};
    if (y_tgt.size() != p.size()) return {};

    const int N = static_cast<int>(y_tgt.size());
    const int B = params_.blockSize;
    const int H = hopSize();
    if (B <= 0 || H <= 0) return {};

    const auto window = hannWindow(B);
    std::vector<double> result(N, 0.0);

    int blockStart = 0;
    while (blockStart < N) {
        const int blockEnd = std::min(blockStart + B, N);
        const int blockLen = blockEnd - blockStart;

        // Average scheduling parameter for this block.
        double pAvg = 0.0;
        for (int i = blockStart; i < blockEnd; ++i) pAvg += p[i];
        pAvg /= blockLen;

        // Interpolate inverse filter at pAvg.
        auto hInv = interpolateInverseFilter(pAvg);

        // Extract and window the block.
        std::vector<double> block(blockLen, 0.0);
        for (int i = 0; i < blockLen; ++i) {
            block[i] = y_tgt[blockStart + i] * window[i];
        }

        // Convolve windowed block with interpolated inverse filter.
        auto blockOut = convolve(block, hInv);

        // Overlap-add into result.
        for (int i = 0; i < static_cast<int>(blockOut.size()); ++i) {
            const int idx = blockStart + i;
            if (idx >= 0 && idx < N) {
                result[idx] += blockOut[i];
            }
        }

        blockStart += H;
        if (H == 0) break;  // safety guard
    }

    return result;
}

void OverlapAddLpvDeconvolver::reset() {
    inverseFilterLut_.clear();
}

std::vector<double> OverlapAddLpvDeconvolver::interpolateInverseFilter(
    double p) const {
    if (inverseFilterLut_.empty()) return {};

    // Exact match.
    auto it = inverseFilterLut_.find(p);
    if (it != inverseFilterLut_.end()) return it->second;

    // Find the two closest operating points.
    auto upper = inverseFilterLut_.lower_bound(p);
    if (upper == inverseFilterLut_.begin()) return upper->second;
    if (upper == inverseFilterLut_.end()) {
        auto last = std::prev(upper);
        return last->second;
    }

    auto lower = std::prev(upper);
    const double p0 = lower->first;
    const double p1 = upper->first;
    const double t = (p - p0) / (p1 - p0);

    const auto& h0 = lower->second;
    const auto& h1 = upper->second;
    const int len = static_cast<int>(std::max(h0.size(), h1.size()));

    std::vector<double> result(len, 0.0);
    for (int i = 0; i < len; ++i) {
        const double v0 = (i < static_cast<int>(h0.size())) ? h0[i] : 0.0;
        const double v1 = (i < static_cast<int>(h1.size())) ? h1[i] : 0.0;
        result[i] = v0 + t * (v1 - v0);
    }
    return result;
}

std::vector<double> OverlapAddLpvDeconvolver::hannWindow(int N) {
    std::vector<double> w(N, 0.0);
    for (int i = 0; i < N; ++i) {
        // Hann window: 0.5 * (1 - cos(2π n / (N-1)))
        // For periodic Hann (better for overlap-add with 50% overlap):
        // 0.5 * (1 - cos(2π n / N))
        w[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / N));
    }
    return w;
}

std::vector<double> OverlapAddLpvDeconvolver::convolve(
    const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return {};
    const int la = static_cast<int>(a.size());
    const int lb = static_cast<int>(b.size());
    std::vector<double> result(la + lb - 1, 0.0);
    for (int i = 0; i < la; ++i) {
        for (int j = 0; j < lb; ++j) {
            result[i + j] += a[i] * b[j];
        }
    }
    return result;
}

} // namespace tether::control::extrusion
