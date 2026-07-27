/**
 * @file PiecewiseNurbsPath.cpp
 * @brief Implementation of tether::motion::PiecewiseNurbsPath.
 */

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <algorithm>
#include <stdexcept>

namespace tether::motion {

PiecewiseNurbsPath::PiecewiseNurbsPath(std::vector<NurbsCurve> pieces)
    : pieces_(std::move(pieces)), dim_(0) {
    if (pieces_.empty()) {
        throw std::invalid_argument("PiecewiseNurbsPath: no pieces");
    }
    dim_ = pieces_[0].dim();
    for (const NurbsCurve& p : pieces_) {
        if (p.dim() != dim_) {
            throw std::invalid_argument(
                "PiecewiseNurbsPath: piece dimension mismatch");
        }
    }
    prefix_.assign(pieces_.size() + 1, 0.0);
}

double PiecewiseNurbsPath::pieceLength(std::size_t i) const {
    // NurbsCurve::length() is itself memoized per curve.
    return pieces_[i].length();
}

void PiecewiseNurbsPath::ensureComputedThrough(std::size_t i) const {
    while (computed_ <= i) {
        prefix_[computed_ + 1] = prefix_[computed_] + pieceLength(computed_);
        ++computed_;
    }
}

double PiecewiseNurbsPath::totalLength() const {
    if (cachedTotal_ >= 0.0) return cachedTotal_;
    // Finish computing all prefix lengths lazily.
    while (computed_ < pieces_.size()) {
        prefix_[computed_ + 1] = prefix_[computed_] + pieceLength(computed_);
        ++computed_;
    }
    cachedTotal_ = prefix_[pieces_.size()];
    return cachedTotal_;
}

PiecewiseNurbsPath::Located PiecewiseNurbsPath::locate(double s) const {
    if (s <= 0.0) return {0, 0.0};

    // If the total is already known, binary search over the full prefix array.
    if (cachedTotal_ >= 0.0) {
        s = std::min(s, cachedTotal_);
        std::size_t lo = 0;
        std::size_t hi = pieces_.size(); // prefix_[hi] = total >= s
        while (hi - lo > 1) {
            const std::size_t mid = lo + (hi - lo) / 2;
            if (prefix_[mid] <= s) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        double localS = s - prefix_[lo];
        const double pl = prefix_[lo + 1] - prefix_[lo];
        localS = std::max(0.0, std::min(localS, pl));
        return {lo, localS};
    }

    // Lazy forward scan: compute prefix lengths one piece at a time until
    // we pass s. Only touches pieces [0..k-1] where k is the piece containing
    // s — queries near the start are O(k) and never touch later pieces.
    while (computed_ < pieces_.size()) {
        const double pieceLen = pieceLength(computed_);
        prefix_[computed_ + 1] = prefix_[computed_] + pieceLen;
        ++computed_;
        if (prefix_[computed_] > s) {
            const std::size_t idx = computed_ - 1;
            double localS = s - prefix_[idx];
            localS = std::max(0.0, std::min(localS, pieceLen));
            return {idx, localS};
        }
    }

    // s >= total: all pieces are now computed; cache and clamp to the last.
    cachedTotal_ = prefix_[pieces_.size()];
    const std::size_t idx = pieces_.size() - 1;
    const double pl = prefix_[idx + 1] - prefix_[idx];
    return {idx, pl};
}

ArcDerivatives PiecewiseNurbsPath::evaluate(double s, int order) const {
    if (order < 0 || order > 3) {
        throw std::invalid_argument(
            "PiecewiseNurbsPath::evaluate: order must be 0..3");
    }
    const Located loc = locate(s);
    const double u = pieces_[loc.piece].invertLength(loc.localS);
    return pieces_[loc.piece].arcDerivatives(u, order);
}

PiecewiseNurbsPath PiecewiseNurbsPath::trim(double s0, double s1) const {
    const double total = totalLength();
    s0 = std::max(0.0, std::min(s0, total));
    s1 = std::max(0.0, std::min(s1, total));
    if (s1 < s0) std::swap(s0, s1);

    Located a = locate(s0);
    Located b = locate(s1);

    // Normalize exact junction hits: s0 at a piece end means "start of the
    // next piece", s1 at a piece start means "end of the previous piece".
    while (a.piece < b.piece &&
           a.localS >= pieces_[a.piece].length()) {
        ++a.piece;
        a.localS = 0.0;
    }
    while (b.piece > a.piece && b.localS <= 0.0) {
        --b.piece;
        b.localS = pieces_[b.piece].length();
    }
    if (a.piece == b.piece && b.localS - a.localS <= 0.0) {
        throw std::invalid_argument(
            "PiecewiseNurbsPath::trim: zero-length trim is not representable");
    }

    std::vector<NurbsCurve> out;
    if (a.piece == b.piece) {
        out.push_back(pieces_[a.piece].trim(a.localS, b.localS));
    } else {
        out.push_back(pieces_[a.piece].trim(
            a.localS, pieces_[a.piece].length()));
        for (std::size_t i = a.piece + 1; i < b.piece; ++i) {
            out.push_back(pieces_[i]);
        }
        out.push_back(pieces_[b.piece].trim(0.0, b.localS));
    }
    return PiecewiseNurbsPath(std::move(out));
}

bool PiecewiseNurbsPath::isG0Connected(double tol) const {
    for (std::size_t i = 1; i < pieces_.size(); ++i) {
        if (pieces_[i - 1].endPoint().distanceTo(pieces_[i].startPoint()) >
            tol) {
            return false;
        }
    }
    return true;
}

std::size_t PiecewiseNurbsPath::totalArcLengthComputations() const noexcept {
    std::size_t sum = 0;
    for (const NurbsCurve& p : pieces_) {
        sum += p.arcLengthComputationCount();
    }
    return sum;
}

std::size_t PiecewiseNurbsPath::estimatedMemoryBytes() const noexcept {
    std::size_t sum = sizeof(PiecewiseNurbsPath) +
                      prefix_.capacity() * sizeof(double);
    for (const NurbsCurve& p : pieces_) {
        sum += p.estimatedMemoryBytes();
    }
    return sum;
}

} // namespace tether::motion
