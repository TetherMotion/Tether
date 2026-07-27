/**
 * @file PiecewiseNurbsPath.hpp
 * @brief Piecewise NURBS path with lazy arc-length mapping (plan §4.8)
 *
 * @details
 * Holds an ordered sequence of NurbsCurve pieces plus a prefix-length array
 * used to map global arc length s → (piece, local parameter). Memory is
 * O(pieces); nothing is ever sampled globally — a 10⁶-piece path stores only
 * its control points.
 *
 * Laziness: the prefix array structure is allocated once (O(pieces)), but
 * individual piece lengths are computed on first need and memoized
 * (watermark scheme — see PiecewiseNurbsPath.cpp). Consequently per-piece
 * arc-length caches are created only for pieces that were actually queried
 * (or that lie before a queried piece in a binary search); the
 * `totalArcLengthComputations()` counter exposes this to tests. Line pieces
 * (degree 1) have exact closed-form length and never trigger quadrature.
 *
 * The path does NOT require continuity of the inputs; `isG0Connected` is a
 * diagnostic checker for tests.
 *
 * Math reference: (M5) arc-length derivatives up to order 3,
 * (M7) splitting by knot insertion. See docs/motion/BlendingAlgorithm.md.
 *
 * Thread-safety: NOT thread-safe (lazy mutable caches) — one path per thread.
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cstddef>
#include <vector>

namespace tether::motion {

class PiecewiseNurbsPath {
public:
    /// @throws std::invalid_argument if pieces is empty or dimensions differ.
    explicit PiecewiseNurbsPath(std::vector<NurbsCurve> pieces);

    std::size_t numPieces() const noexcept { return pieces_.size(); }
    std::size_t dim() const noexcept { return dim_; }
    const std::vector<NurbsCurve>& pieces() const noexcept { return pieces_; }
    const NurbsCurve& piece(std::size_t i) const { return pieces_.at(i); }

    /// Total arc length (computes any not-yet-computed piece lengths once).
    double totalLength() const;

    /// Result of locating a global arc length: piece index + local length.
    struct Located {
        std::size_t piece;
        double localS;
    };

    /// Map global arc length s (clamped to [0, totalLength()]) to a piece and
    /// the local arc length within it. Binary search, O(log n) + amortized
    /// O(1) lazy length computation.
    Located locate(double s) const;

    /// Arc-length evaluation: position (order 0) plus T, κ⃗, j⃗ for
    /// order 1..3, computed exactly on the located piece.
    /// @throws std::domain_error at degenerate junctions (|C′| ~ 0).
    ArcDerivatives evaluate(double s, int order) const;

    /// Convenience: position only (never throws on degenerate tangents).
    RVec evaluatePosition(double s) const { return evaluate(s, 0).position; }

    /// Arc-length trim: sub-path covering global lengths [s0, s1] (clamped,
    /// swapped if needed). Pieces fully inside are kept by value; boundary
    /// pieces are trimmed exactly via NurbsCurve::trim.
    PiecewiseNurbsPath trim(double s0, double s1) const;

    /// Diagnostic: consecutive piece endpoints coincide within tol
    /// (G0 connectivity). The path itself works without it.
    bool isG0Connected(double tol) const;

    /// Sum of NurbsCurve::arcLengthComputationCount() over all pieces —
    /// laziness counter for tests (0 for an all-line path).
    std::size_t totalArcLengthComputations() const noexcept;

    /// Approximate total memory footprint in bytes (for the huge-path test:
    /// proves no per-point sample tables are allocated).
    std::size_t estimatedMemoryBytes() const noexcept;

private:
    /// Ensure prefix lengths through piece index i (inclusive) are computed.
    void ensureComputedThrough(std::size_t i) const;
    /// Length of piece i, memoized.
    double pieceLength(std::size_t i) const;

    std::vector<NurbsCurve> pieces_;
    std::size_t dim_;

    // Lazy prefix sums: prefix_[i] = Σ_{j<i} length(piece j), valid for
    // i ≤ computed_ (watermark). Mutable: lazy cache, not thread-safe.
    mutable std::vector<double> prefix_;
    mutable std::size_t computed_ = 0;
    mutable double cachedTotal_ = -1.0; // < 0: not computed
};

} // namespace tether::motion
