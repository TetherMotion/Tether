/**
 * @file ArxLpvInverseFilter.cpp
 * @brief Time-domain LPV inverse IIR filter implementation.
 */

#include "tether/control/extrusion/ArxLpvInverseFilter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::control::extrusion {

ArxLpvInverseFilter::ArxLpvInverseFilter(int na, int nb)
    : na_(na), nb_(nb) {
    ensureBufferSize();
}

void ArxLpvInverseFilter::addModelPoint(const ArxLpvModelPoint& point) {
    modelLut_[point.parameter] = point;
}

void ArxLpvInverseFilter::addModelPoint(
    double p, std::vector<double> a, std::vector<double> b, int delay) {
    addModelPoint(ArxLpvModelPoint(p, std::move(a), std::move(b), delay));
}

double ArxLpvInverseFilter::process(double yTargetCurrent,
                                     double yTargetAhead,
                                     double pCurrent) {
    if (modelLut_.empty()) return 0.0;

    // Interpolate coefficients if the parameter changed.
    if (std::isnan(lastP_) || std::abs(pCurrent - lastP_) > 1e-12) {
        interpolateCoefficients(pCurrent);
        lastP_ = pCurrent;
        ensureBufferSize();
    }

    // The inverse difference equation (with delay d factored out):
    //   x_req[n] = (1 / b_0) * ( y_tgt[n+d]
    //             + Σ a_i * y_tgt[n+d-i]
    //             − Σ b_j * x_req[n-j] )
    //
    // yTargetAhead = y_tgt[n+d] is provided by the caller.
    // We need y_tgt[n+d-i] for i = 1..Na.  These are:
    //   y_tgt[n+d-1] = yTargetCurrent (if d=1) or from the buffer.
    // More generally, y_tgt[n+d-i] = yTargetAhead shifted back by i.
    //
    // For d=0: y_tgt[n-i] comes from yTargetHistory_.
    // For d=1: y_tgt[n+1-i]: i=1 → y_tgt[n] = yTargetCurrent,
    //          i=2 → y_tgt[n-1] from history, etc.
    //
    // We build a combined view: [yTargetAhead, yTargetCurrent, *history]

    // Build the y_tgt sequence needed: y_tgt[n+d], y_tgt[n+d-1], ...
    // We need values at offsets n+d, n+d-1, n+d-2, ..., n+d-Na
    // = yTargetAhead, then yTargetCurrent, then history[0], history[1], ...

    double numerator = yTargetAhead;

    // Add AR terms: Σ a_i * y_tgt[n+d-i]
    for (int i = 1; i <= na_; ++i) {
        double yVal;
        if (i == currentDelay_) {
            yVal = yTargetCurrent;
        } else if (i < currentDelay_) {
            // y_tgt[n+d-i] with i < d → y_tgt[n + (d-i)] → future value
            // not available without more lookahead.  Approximate with
            // yTargetCurrent (zero-order hold) for i < d.
            yVal = yTargetCurrent;
        } else {
            // i > d: y_tgt[n + d - i] = y_tgt[n - (i - d)]
            const int histIdx = i - currentDelay_ - 1;
            if (histIdx < static_cast<int>(yTargetHistory_.size())) {
                yVal = yTargetHistory_[histIdx];
            } else {
                yVal = 0.0;
            }
        }
        numerator += currentA_[i - 1] * yVal;
    }

    // Subtract input terms: Σ b_j * x_req[n-j]
    for (int j = 1; j <= nb_; ++j) {
        double xVal;
        if (j - 1 < static_cast<int>(xReqHistory_.size())) {
            xVal = xReqHistory_[j - 1];
        } else {
            xVal = 0.0;
        }
        numerator -= currentB_[j] * xVal;
    }

    // Divide by b_0 (with safety check).
    const double b0 = currentB_[0];
    if (std::abs(b0) < 1e-15) {
        return 0.0;  // avoid division by zero
    }
    double xReq = numerator / b0;

    // Update ring buffers.
    // Push current values to the front, pop from the back.
    yTargetHistory_.push_front(yTargetCurrent);
    if (static_cast<int>(yTargetHistory_.size()) > na_ + currentDelay_) {
        yTargetHistory_.pop_back();
    }
    xReqHistory_.push_front(xReq);
    if (static_cast<int>(xReqHistory_.size()) > nb_) {
        xReqHistory_.pop_back();
    }

    return xReq;
}

std::vector<double> ArxLpvInverseFilter::process(
    const std::vector<double>& yTarget, const std::vector<double>& p) {
    if (yTarget.empty() || yTarget.size() != p.size()) return {};
    reset();

    const int N = static_cast<int>(yTarget.size());
    std::vector<double> xReq(N, 0.0);

    for (int n = 0; n < N; ++n) {
        // Determine the lookahead value y_tgt[n+d].
        // First, interpolate coefficients to find the current delay.
        if (std::isnan(lastP_) || std::abs(p[n] - lastP_) > 1e-12) {
            interpolateCoefficients(p[n]);
            lastP_ = p[n];
            ensureBufferSize();
        }

        const int d = currentDelay_;
        const double yAhead = (n + d < N) ? yTarget[n + d] : yTarget[n];

        xReq[n] = process(yTarget[n], yAhead, p[n]);
    }
    return xReq;
}

void ArxLpvInverseFilter::reset() {
    yTargetHistory_.clear();
    xReqHistory_.clear();
    lastP_ = std::numeric_limits<double>::quiet_NaN();
    currentA_.clear();
    currentB_.clear();
    currentDelay_ = 0;
}

void ArxLpvInverseFilter::interpolateCoefficients(double p) {
    if (modelLut_.empty()) return;

    auto it = modelLut_.find(p);
    if (it != modelLut_.end()) {
        currentA_ = it->second.aCoeffs;
        currentB_ = it->second.bCoeffs;
        currentDelay_ = it->second.delay;
        return;
    }

    auto upper = modelLut_.lower_bound(p);
    if (upper == modelLut_.begin()) {
        currentA_ = upper->second.aCoeffs;
        currentB_ = upper->second.bCoeffs;
        currentDelay_ = upper->second.delay;
        return;
    }
    if (upper == modelLut_.end()) {
        auto last = std::prev(upper);
        currentA_ = last->second.aCoeffs;
        currentB_ = last->second.bCoeffs;
        currentDelay_ = last->second.delay;
        return;
    }

    auto lower = std::prev(upper);
    const double p0 = lower->first;
    const double p1 = upper->first;
    const double t = (p - p0) / (p1 - p0);

    const auto& a0 = lower->second.aCoeffs;
    const auto& a1 = upper->second.aCoeffs;
    const auto& b0 = lower->second.bCoeffs;
    const auto& b1 = upper->second.bCoeffs;

    currentA_.resize(std::max(a0.size(), a1.size()), 0.0);
    for (size_t i = 0; i < currentA_.size(); ++i) {
        const double v0 = (i < a0.size()) ? a0[i] : 0.0;
        const double v1 = (i < a1.size()) ? a1[i] : 0.0;
        currentA_[i] = v0 + t * (v1 - v0);
    }

    currentB_.resize(std::max(b0.size(), b1.size()), 0.0);
    for (size_t i = 0; i < currentB_.size(); ++i) {
        const double v0 = (i < b0.size()) ? b0[i] : 0.0;
        const double v1 = (i < b1.size()) ? b1[i] : 0.0;
        currentB_[i] = v0 + t * (v1 - v0);
    }

    // Interpolate delay (round to nearest integer).
    currentDelay_ = static_cast<int>(std::round(
        lower->second.delay + t * (upper->second.delay - lower->second.delay)));
}

void ArxLpvInverseFilter::ensureBufferSize() {
    const int yHistSize = na_ + currentDelay_;
    while (static_cast<int>(yTargetHistory_.size()) < yHistSize) {
        yTargetHistory_.push_back(0.0);
    }
    while (static_cast<int>(yTargetHistory_.size()) > yHistSize) {
        yTargetHistory_.pop_back();
    }

    while (static_cast<int>(xReqHistory_.size()) < nb_) {
        xReqHistory_.push_back(0.0);
    }
    while (static_cast<int>(xReqHistory_.size()) > nb_) {
        xReqHistory_.pop_back();
    }
}

} // namespace tether::control::extrusion
