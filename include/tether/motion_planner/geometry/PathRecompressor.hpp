/**
 * @file PathRecompressor.hpp
 * @brief Error-bounded geometric recompression of a PiecewiseNurbsPath
 *
 * @details
 * Converts pathological inputs (thousands of tiny G1 segments emitted by
 * slicers for curved geometry) back into a compact piecewise NURBS
 * representation with a *certified* geometric error bound. This is the
 * geometric analogue of the profile-space ReNURBS machinery in
 * `profile_renurbs/` — it compresses the path itself, not a velocity
 * profile sampled over it.
 *
 * ## Pipeline
 *
 * 1. **Run discovery.** The piece list is partitioned into maximal runs
 *    of degree-1 (polyline) pieces. A run ends at:
 *    - any piece with degree > 1 (exact arcs and explicit splines are
 *      already compressed and are emitted verbatim),
 *    - a caller-supplied hard break (piece index that must start a new
 *      run — e.g. a dwell boundary whose junction must be preserved),
 *    - a sharp corner: the angle between consecutive piece directions
 *      exceeds `cornerAngleDeg` (tessellation vertices turn by a degree
 *      or two; intentional geometry turns sharply).
 *
 * 2. **Fitting.** Each run's polyline vertices are approximated by one
 *    clamped B-spline of degree `degree` (weights 1) solved by least
 *    squares with both endpoints interpolated exactly (G0 continuity of
 *    the output chain is thus bitwise at run boundaries). Near-collinear
 *    runs collapse to a single degree-1 line.
 *
 * 3. **Certification.** Each fitted piece is checked in *both* Hausdorff
 *    directions against the source polyline:
 *    - every source vertex → certified distance to the fitted curve via
 *      `pointCurveDistance` (Bernstein root isolation — no missed roots),
 *    - dense samples on the fitted curve → distance to the polyline,
 *      bounded for the unsampled gaps by the convex-hull speed bound
 *      |C'(u)| ≤ max_i |Q_i| of the derivative control points.
 *    The bound `maxCertifiedDeviation` therefore applies to every point
 *    of both the original and the fitted geometry, not only at samples.
 *
 * 4. **Adaptive failure handling.** If a run cannot be certified within
 *    `tolerance`, the fit either gains control points (up to
 *    `maxControlPoints`) or the run is split at the worst-fitting vertex
 *    and retried. At `maxSplitDepth` the original pieces are emitted
 *    verbatim — the output is *always* within tolerance by construction.
 *
 * ## Attribute mapping
 *
 * `sourceMap[j] = {srcBegin, srcEnd}` gives, for every output piece, the
 * half-open range of source pieces it replaces. Callers re-derive
 * per-piece attributes (feed rates, deviations, junction semantics)
 * conservatively from that range; junction stamps/corner velocities are
 * recomputed downstream on the output geometry itself, so the fitted
 * tangents — which differ from the polyline by less than the geometric
 * tolerance — produce consistent junction limits.
 */
#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <cstddef>
#include <vector>

namespace tether::motion {

/** Tunables for recompressPiecewiseNurbs(). */
struct RecompressConfig {
    /**
     * Certified Hausdorff bound between the output path and the source
     * path, in the path's coordinate units (mm for G-code toolpaths).
     * Every fitted piece and every source point it replaces deviate by
     * at most this amount.
     */
    double tolerance = 0.02;

    /** Degree of fitted B-spline pieces (≥ 2 for genuine compression). */
    int degree = 3;

    /**
     * Junction tangent discontinuity (degrees) that ends a fit run.
     * Corners sharper than this are preserved exactly — the fitted
     * pieces on both sides interpolate the corner vertex.
     */
    double cornerAngleDeg = 10.0;

    /** Maximum control points for one fitted piece before the run is split. */
    std::size_t maxControlPoints = 48;

    /**
     * Runs shorter than this many source pieces are copied verbatim
     * (a cubic fit needs ≥ degree+2 vertices to be underconstrained-free).
     */
    std::size_t minRunPieces = 3;

    /**
     * Recursion cap for split-on-fail: at the cap the source pieces of
     * the failing sub-run are emitted verbatim (zero deviation).
     */
    int maxSplitDepth = 10;
};

/** Output-piece → covered source-piece range, half-open [srcBegin, srcEnd). */
struct RecompressPieceMap {
    std::size_t srcBegin = 0;
    std::size_t srcEnd = 0;    ///< one past the last covered source piece
};

struct RecompressResult {
    explicit RecompressResult(PiecewiseNurbsPath p)
        : path(std::move(p)) {}

    PiecewiseNurbsPath path;
    std::vector<RecompressPieceMap> sourceMap;

    /**
     * Largest certified Hausdorff deviation over all fitted pieces
     * (0 for verbatim pieces). ≤ RecompressConfig::tolerance always.
     */
    double maxCertifiedDeviation = 0.0;

    std::size_t runsFitted = 0;        ///< runs replaced by a fitted piece
    std::size_t runsSplit = 0;         ///< runs recursively split
    std::size_t piecesKeptVerbatim = 0;///< source pieces copied unchanged
};

/**
 * @brief Recompress a PiecewiseNurbsPath into fewer pieces under a
 *        certified geometric error bound.
 *
 * @param src            Source path. Piece order and traversal direction
 *                       are preserved; the output stays G0-connected
 *                       wherever the input was.
 * @param cfg            Fitting/certification tunables.
 * @param hardBreakPieces Piece indices (into `src`) that must begin a new
 *                       output boundary — no fitted piece spans across
 *                       them. Use for dwell points, semantic boundaries,
 *                       or any junction the caller needs preserved.
 * @return Recompressed path plus per-output-piece source mapping and the
 *         achieved certified deviation.
 *
 * @throws std::invalid_argument if `src` is empty or `cfg` inconsistent
 *         (tolerance ≤ 0, degree < 1).
 */
RecompressResult recompressPiecewiseNurbs(
    const PiecewiseNurbsPath& src,
    const RecompressConfig& cfg = {},
    const std::vector<std::size_t>& hardBreakPieces = {});

} // namespace tether::motion
