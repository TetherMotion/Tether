/**
 * @file BlendSolver.cpp
 * @brief Implementation of BlendSolver — the (M15) bisection loop.
 *
 * Equation numbers (M.x) refer to docs/motion/BlendingAlgorithm.md.
 */

#include "tether/motion_planner/blend/BlendSolver.hpp"

#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"
#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/blend/DeviationCertifier.hpp"
#include "tether/motion_planner/blend/PHQuinticBlendBuilder.hpp"
#include "tether/motion_planner/geometry/PointCurveDistance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace tether::motion {

namespace {

/// Auto-select a certificate width: 1e-3·|tol| with a floor of 1e-9.
double autoEpsilon(double tol) {
    return std::max(1e-9, 1e-3 * std::abs(tol));
}

/// Check if a piece is long enough to trim by `trim` on the junction end.
bool canTrim(const NurbsCurve& piece, double trim, double minLength) {
    return piece.length() >= std::max(2.0 * minLength, trim);
}

/// Coarse deviation lower bound: sample the blend and the removed corner
/// pieces at a small number of points and compute the min point-to-curve
/// distance in both directions (blend→Ω and Ω→blend). The result is a
/// lower bound on the true Hausdorff deviation — if it already exceeds
/// the tolerance, the full (M10) certification would reject the blend, so
/// the caller can skip that expensive step.
///
/// This is the key performance optimization for tight tolerances: the
/// (M10) certifier's grid size scales as L/(2ε), and ε = 1e-3·|tol|. For
/// tight tolerances, ε is tiny, making the grid enormous (up to 100000
/// samples, each calling pointCurveDistance). Most bisection iterations
/// test high speeds where the deviation is far above the tolerance —
/// this pre-check skips the expensive certification for those iterations.
double coarseDeviationLowerBound(const NurbsCurve& blend,
                                 const NurbsCurve& removedIn,
                                 const NurbsCurve& removedOut,
                                 std::size_t N = 16) {
    double maxDist = 0.0;
    // Blend → Ω direction: for each blend sample, min dist to removedIn/Out.
    for (std::size_t i = 0; i <= N; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(N);
        const RVec p = blend.evaluate(t);
        const double dIn = pointCurveDistance(removedIn, p).distance;
        const double dOut = pointCurveDistance(removedOut, p).distance;
        maxDist = std::max(maxDist, std::min(dIn, dOut));
    }
    // Ω → blend direction: for each removed-piece sample, dist to blend.
    for (std::size_t i = 0; i <= N; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(N);
        const RVec qIn = removedIn.evaluate(t);
        maxDist = std::max(maxDist, pointCurveDistance(blend, qIn).distance);
        const RVec qOut = removedOut.evaluate(t);
        maxDist = std::max(maxDist, pointCurveDistance(blend, qOut).distance);
    }
    return maxDist;
}

} // namespace

BlendSolver::BlendSolver(const NurbsCurve& in, const NurbsCurve& out,
                         const CornerAnalysis& corner)
    : in_(in), out_(out), corner_(corner) {}

BlendGeometry BlendSolver::solve(const BlendSpec& spec) const {
    spec.validate();

    BlendGeometry result;

    // Step 1: exact-stop / exact-path modes.
    if (spec.mode == PathMode::ExactStop) {
        result.outcome = BlendOutcome::ExactStop;
        result.reason = "ExactStop mode requested";
        return result;
    }
    if (spec.mode == PathMode::ExactPath) {
        result.outcome = BlendOutcome::ExactStop;
        result.reason = "ExactPath mode: no blend";
        return result;
    }

    // Step 2: straight corner — nothing to do.
    if (corner_.kind == CornerKind::Straight) {
        result.outcome = BlendOutcome::NoBlendNeeded;
        result.reason = "Corner is straight (θ < minAngle)";
        return result;
    }

    // Step 3: cusp — unsafe to blend.
    if (corner_.kind == CornerKind::Cusp) {
        result.outcome = BlendOutcome::ExactStop;
        result.reason = "Corner is a cusp (θ > maxAngle); blending unsafe";
        return result;
    }

    // Step 4: check neighbor lengths.
    // The max trim is maxBlendFraction × free length. Free length = total
    // length (no prior blends yet at this layer).
    const double maxTrimIn = spec.maxBlendFraction * in_.length();
    const double maxTrimOut = spec.maxBlendFraction * out_.length();
    if (!canTrim(in_, maxTrimIn, spec.minSegmentLength) ||
        !canTrim(out_, maxTrimOut, spec.minSegmentLength)) {
        result.outcome = BlendOutcome::ExactStop;
        result.reason = "Neighbor piece too short to trim";
        return result;
    }

    // Step 5-7: dispatch to the curve-type-specific solver.
    if (spec.curveType == BlendCurveType::PHQuintic) {
        return solvePH(spec);
    }
    return solveBezier(spec);
}

BlendGeometry BlendSolver::solveBezier(const BlendSpec& spec) const {
    BlendGeometry result;
    const double tol = spec.tolerance;
    const double absTol = std::abs(tol);
    const double eps = (spec.certEpsilon > 0.0)
        ? spec.certEpsilon : autoEpsilon(tol);
    DeviationCertifier cert(eps);

    // (M15) bisection on the speed α₁ = β₁ (symmetric search).
    // Range: [speedLo, speedHi]. Start wide; the blend's deviation
    // increases monotonically with speed (faster = wider cut).
    const double maxTrimIn = spec.maxBlendFraction * in_.length();
    const double maxTrimOut = spec.maxBlendFraction * out_.length();
    // The speed is bounded by the trim distance: α₁ ≤ trimIn (the blend
    // can't be longer than the trim). Use a conservative upper bound.
    const double speedHi = std::min(maxTrimIn, maxTrimOut);
    double speedLo = 1e-6 * speedHi;
    double speedHi2 = speedHi;

    // For negative tolerance (M20), augment curvature.
    const bool isEar = (tol < 0.0);
    RVec cutDir;
    if (isEar) {
        cutDir = (corner_.tangentOut - corner_.tangentIn).normalized();
    }

    const int maxIters = 40;
    for (int iter = 0; iter < maxIters; ++iter) {
        const double speed = 0.5 * (speedLo + speedHi2);
        const double trimIn = speed;  // α₁ ≈ trim distance (approximate)
        const double trimOut = speed;

        // Extract boundary conditions at the trim points.
        BoundaryConditions entry, exitBc;
        try {
            entry = boundaryAt(in_, in_.length() - trimIn, true);
            exitBc = boundaryAt(out_, trimOut, false);
        } catch (...) {
            // Degenerate parameterization — shrink the speed.
            speedHi2 = speed;
            continue;
        }

        // Augment curvature for ear blends (M20).
        if (isEar) {
            const double lambda = 0.5 * speed; // heuristic
            entry.curvature = entry.curvature + cutDir * lambda;
            exitBc.curvature = exitBc.curvature + cutDir * lambda;
        }

        // Build the blend.
        std::optional<NurbsCurve> blend;
        try {
            if (spec.continuity == Continuity::G3) {
                blend = BlendCurveBuilder::buildSeptic(
                    entry, exitBc, speed, speed);
            } else {
                blend = BlendCurveBuilder::buildQuintic(
                    entry, exitBc, speed, speed);
            }
        } catch (...) {
            speedHi2 = speed;
            continue;
        }

        // The "removed" parts of the original path: the corner pieces
        // from the trim points to the vertex. The deviation is the max
        // distance from the blend to these removed pieces (how far the
        // blend departs from the original corner path).
        //   removedIn  = in_[length-trimIn .. length]  (entry trim → vertex)
        //   removedOut = out_[0 .. trimOut]            (vertex → exit trim)
        std::optional<NurbsCurve> removedIn, removedOut;
        try {
            removedIn = in_.trim(in_.length() - trimIn, in_.length());
            removedOut = out_.trim(0.0, trimOut);
        } catch (...) {
            speedHi2 = speed;
            continue;
        }

        // Coarse pre-check: if the deviation lower bound already exceeds
        // the tolerance, the full (M10) certification would reject this
        // blend. Skip the expensive certification (which can use grids of
        // up to 100000 samples for tight tolerances) and continue the
        // bisection with a smaller speed.
        if (coarseDeviationLowerBound(*blend, *removedIn, *removedOut) > absTol) {
            speedHi2 = speed;
            continue;
        }

        // Certify.
        DeviationCertificate dev;
        try {
            dev = isEar
                ? cert.certify(*blend, *removedIn, *removedOut, &cutDir)
                : cert.certify(*blend, *removedIn, *removedOut);
        } catch (...) {
            speedHi2 = speed;
            continue;
        }

        // Acceptance check.
        bool accepted = false;
        if (isEar) {
            // Inside cut must be ~0, outside ear ≤ |tol|.
            accepted = (dev.insideHi <= eps * 10.0) &&
                       (dev.outsideHi <= absTol);
        } else {
            accepted = (dev.upper <= absTol);
        }

        if (accepted) {
            result.outcome = BlendOutcome::Blended;
            result.blendCurve = std::move(blend);
            result.trimIn = trimIn;
            result.trimOut = trimOut;
            result.deviation = dev;
            result.solverIterations = iter + 1;
            return result;
        }

        // Deviation too large → reduce speed (smaller cut).
        // Deviation too small → increase speed (larger cut, closer to tol).
        // For ear blends, use the outside component.
        const double measured = isEar ? dev.outsideHi : dev.upper;
        if (measured > absTol) {
            speedHi2 = speed; // too much: shrink
        } else {
            speedLo = speed;  // too little: grow
        }

        if (speedHi2 - speedLo < 1e-9 * speedHi) {
            // Converged but not accepted — give up.
            break;
        }
    }

    result.outcome = BlendOutcome::ExactStop;
    result.reason = "Bisection did not find an acceptable blend";
    return result;
}

BlendGeometry BlendSolver::solvePH(const BlendSpec& spec) const {
    BlendGeometry result;
    const double tol = spec.tolerance;
    const double absTol = std::abs(tol);
    const double eps = (spec.certEpsilon > 0.0)
        ? spec.certEpsilon : autoEpsilon(tol);
    DeviationCertifier cert(eps);

    // For PH, we don't bisect on speed — the Hermite construction has
    // no free speed (it's determined by the endpoint tangent magnitudes).
    // Instead, we vary the trim distance (which sets the tangent
    // magnitudes via c = 2θ/π) and certify all 8 candidates at each
    // trim level.
    const double maxTrimIn = spec.maxBlendFraction * in_.length();
    const double maxTrimOut = spec.maxBlendFraction * out_.length();
    const double maxTrim = std::min(maxTrimIn, maxTrimOut);

    const bool isEar = (tol < 0.0);
    RVec cutDir;
    if (isEar) {
        cutDir = (corner_.tangentOut - corner_.tangentIn).normalized();
    }

    double trimLo = spec.minSegmentLength;
    double trimHi = maxTrim;

    const int maxIters = 30;
    for (int iter = 0; iter < maxIters; ++iter) {
        const double trim = 0.5 * (trimLo + trimHi);

        BoundaryConditions entry, exitBc;
        try {
            entry = boundaryAt(in_, in_.length() - trim, true);
            exitBc = boundaryAt(out_, trim, false);
        } catch (...) {
            trimHi = trim;
            continue;
        }

        // Build all 8 PH candidates.
        std::vector<PHQuinticBlendBuilder::Result> candidates;
        try {
            candidates = PHQuinticBlendBuilder::buildCandidates(
                entry, exitBc, corner_.planeE1, corner_.planeE2);
        } catch (...) {
            trimHi = trim;
            continue;
        }

        // The "removed" corner pieces (see solveBezier for explanation).
        std::optional<NurbsCurve> removedIn, removedOut;
        try {
            removedIn = in_.trim(in_.length() - trim, in_.length());
            removedOut = out_.trim(0.0, trim);
        } catch (...) {
            trimHi = trim;
            continue;
        }

        // Certify each non-degenerate candidate; keep the best.
        DeviationCertificate bestDev;
        std::optional<NurbsCurve> bestCurve;
        std::optional<PHData> bestPH;
        bool haveBest = false;
        double bestUpper = std::numeric_limits<double>::infinity();

        for (const auto& cand : candidates) {
            if (cand.degenerate) continue;
            // Coarse pre-check: skip candidates whose deviation is clearly
            // above tolerance (see solveBezier for rationale).
            if (coarseDeviationLowerBound(cand.curve, *removedIn, *removedOut)
                > absTol) {
                continue;
            }
            DeviationCertificate dev;
            try {
                dev = isEar
                    ? cert.certify(cand.curve, *removedIn, *removedOut, &cutDir)
                    : cert.certify(cand.curve, *removedIn, *removedOut);
            } catch (...) {
                continue;
            }
            const double upper = isEar ? dev.outsideHi : dev.upper;
            if (upper < bestUpper) {
                bestUpper = upper;
                bestDev = dev;
                bestCurve = cand.curve; // copy into optional
                bestPH = cand.ph;       // copy PHData sidecar
                haveBest = true;
            }
        }

        if (!haveBest) {
            trimHi = trim;
            continue;
        }

        // Acceptance check.
        bool accepted = isEar
            ? (bestDev.insideHi <= eps * 10.0 && bestDev.outsideHi <= absTol)
            : (bestDev.upper <= absTol);

        if (accepted) {
            result.outcome = BlendOutcome::Blended;
            result.blendCurve = std::move(bestCurve);
            result.trimIn = trim;
            result.trimOut = trim;
            result.deviation = bestDev;
            result.phData = std::move(bestPH);
            result.solverIterations = iter + 1;
            return result;
        }

        const double measured = isEar ? bestDev.outsideHi : bestDev.upper;
        if (measured > absTol) {
            trimHi = trim;  // too much deviation: shrink trim
        } else {
            trimLo = trim;  // too little: grow trim
        }

        if (trimHi - trimLo < 1e-9 * maxTrim) break;
    }

    result.outcome = BlendOutcome::ExactStop;
    result.reason = "PH bisection did not find an acceptable blend";
    return result;
}

} // namespace tether::motion
