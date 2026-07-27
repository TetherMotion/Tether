/**
 * @file DeviationCertifier.cpp
 * @brief Certified Hausdorff deviation via the (M10) Lipschitz certificate.
 *
 * Equation numbers (M.x) refer to docs/motion/BlendingAlgorithm.md.
 */

#include "tether/motion_planner/blend/DeviationCertifier.hpp"

#include "tether/motion_planner/geometry/PointCurveDistance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace tether::motion {

namespace {

/// Lipschitz constant of a Bézier curve B(t): ‖B'‖ ≤ n·max_i‖P_{i+1}−P_i‖
/// (G.4 derivative identity + triangle inequality). The distance-to-a-
/// fixed-set function f(t) = dist(B(t), Ω) is 1-Lipschitz in B(t), so
/// |f'| ≤ ‖B'‖ ≤ L. (M10.)
double blendLipschitz(const NurbsCurve& blend) {
    const auto& cps = blend.controlPoints();
    if (cps.size() < 2) return 0.0;
    double maxStep = 0.0;
    for (std::size_t i = 1; i < cps.size(); ++i) {
        maxStep = std::max(maxStep, cps[i].distanceTo(cps[i - 1]));
    }
    return static_cast<double>(blend.degree()) * maxStep;
}

/// Sample the blend B(t) on a uniform grid of N+1 points (t = i/N) and
/// return the max distance to Ω = trimmedIn ∪ trimmedOut, plus the
/// per-sample (t, point, side) data for signed splitting.
struct Sample {
    double t;
    RVec point;
    double dist;  // dist(B(t), Ω)
    double side;  // (B(t) − V)·c ; 0 if no cut direction
};

std::vector<Sample> sampleBlend(const NurbsCurve& blend,
                                const NurbsCurve& trimmedIn,
                                const NurbsCurve& trimmedOut,
                                std::size_t N,
                                const RVec* cutDir,
                                const RVec& vertex) {
    std::vector<Sample> samples;
    samples.reserve(N + 1);
    for (std::size_t i = 0; i <= N; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(N);
        const RVec p = blend.evaluate(t);
        // dist to Ω = min(dist to trimmedIn, dist to trimmedOut).
        const double dIn = pointCurveDistance(trimmedIn, p).distance;
        const double dOut = pointCurveDistance(trimmedOut, p).distance;
        const double d = std::min(dIn, dOut);
        double side = 0.0;
        if (cutDir) {
            side = (p - vertex).dot(*cutDir);
        }
        samples.push_back({t, p, d, side});
    }
    return samples;
}

/// Sample Ω (the trimmed original pieces) on arc-length grids and return
/// the max distance to the blend. Ω is arc-length parameterized so the
/// Lipschitz constant of dist(q, B) along Ω is 1.
struct OmegaSample {
    double dist;  // dist(q, B)
    RVec point;
    double side;
};

std::vector<OmegaSample> sampleOmega(const NurbsCurve& blend,
                                     const NurbsCurve& piece,
                                     std::size_t N,
                                     const RVec* cutDir,
                                     const RVec& vertex) {
    std::vector<OmegaSample> samples;
    samples.reserve(N + 1);
    const double L = piece.length();
    for (std::size_t i = 0; i <= N; ++i) {
        const double s = L * static_cast<double>(i) /
                         static_cast<double>(N);
        const double u = (s <= 0.0) ? piece.knotMin()
                         : (s >= L) ? piece.knotMax()
                                    : piece.invertLength(s);
        const RVec q = piece.evaluate(u);
        const double d = pointCurveDistance(blend, q).distance;
        double side = 0.0;
        if (cutDir) {
            side = (q - vertex).dot(*cutDir);
        }
        samples.push_back({d, q, side});
    }
    return samples;
}

} // namespace

DeviationCertifier::DeviationCertifier(double epsilon)
    : epsilon_(epsilon) {
    if (!(epsilon > 0.0)) {
        throw std::invalid_argument(
            "DeviationCertifier: epsilon must be > 0");
    }
}

DeviationCertificate DeviationCertifier::certify(
    const NurbsCurve& blend, const NurbsCurve& trimmedIn,
    const NurbsCurve& trimmedOut, const RVec* cutDirection) const {

    if (blend.dim() != trimmedIn.dim() || blend.dim() != trimmedOut.dim()) {
        throw std::invalid_argument(
            "DeviationCertifier: dimension mismatch");
    }

    // --- (M10) grid spacing for the blend term ---------------------------
    // |f'| ≤ L = blendLipschitz; choose h so L·h/2 ≤ ε ⇒ N = ceil(L/(2ε)).
    const double L = blendLipschitz(blend);
    std::size_t Nblend = 16; // minimum grid
    if (L > 0.0) {
        Nblend = std::max<std::size_t>(Nblend,
                      static_cast<std::size_t>(std::ceil(L / (2.0 * epsilon_))));
    }
    Nblend = std::min<std::size_t>(Nblend, 100000); // safety cap

    const RVec vertex = trimmedIn.endPoint(); // junction = end of trimmedIn

    // --- Term 1: max_t dist(B(t), Ω) -------------------------------------
    const auto bsamples = sampleBlend(blend, trimmedIn, trimmedOut, Nblend,
                                      cutDirection, vertex);
    double maxBlendDist = 0.0;
    for (const auto& s : bsamples) maxBlendDist = std::max(maxBlendDist, s.dist);

    // --- Term 2: max_{q∈Ω} dist(q, B) ------------------------------------
    // Ω is arc-length parameterized ⇒ Lipschitz = 1 ⇒ N = ceil(L_Ω/(2ε))
    // where L_Ω = total trimmed length. Use a per-piece grid.
    const double Lin = trimmedIn.length();
    const double Lout = trimmedOut.length();
    const double Lomega = Lin + Lout;
    std::size_t Nomega = 16;
    if (Lomega > 0.0) {
        Nomega = std::max<std::size_t>(Nomega,
                      static_cast<std::size_t>(std::ceil(Lomega / (2.0 * epsilon_))));
    }
    Nomega = std::min<std::size_t>(Nomega, 100000);

    const std::size_t Nin = (Lin > 0.0)
        ? std::max<std::size_t>(4, static_cast<std::size_t>(
              std::ceil(Nomega * Lin / Lomega))) : 1;
    const std::size_t Nout = (Lout > 0.0)
        ? std::max<std::size_t>(4, static_cast<std::size_t>(
              std::ceil(Nomega * Lout / Lomega))) : 1;

    const auto inSamples = sampleOmega(blend, trimmedIn, Nin,
                                       cutDirection, vertex);
    const auto outSamples = sampleOmega(blend, trimmedOut, Nout,
                                        cutDirection, vertex);
    double maxOmegaDist = 0.0;
    for (const auto& s : inSamples) maxOmegaDist = std::max(maxOmegaDist, s.dist);
    for (const auto& s : outSamples) maxOmegaDist = std::max(maxOmegaDist, s.dist);

    // --- Assemble the certificate (M14) ----------------------------------
    // δ = max(term1, term2). Each term has a Lipschitz slack:
    //   term1 ∈ [maxBlendDist, maxBlendDist + L·h/2]
    //   term2 ∈ [maxOmegaDist, maxOmegaDist + 1·h_Ω/2]
    const double hBlend = 1.0 / static_cast<double>(Nblend);
    const double slackBlend = L * hBlend * 0.5;
    const double hOmega = (Lomega > 0.0) ? Lomega / static_cast<double>(Nomega) : 0.0;
    const double slackOmega = hOmega * 0.5; // Lipschitz = 1

    DeviationCertificate cert;
    cert.lower = std::max(maxBlendDist, maxOmegaDist);
    cert.upper = std::max(maxBlendDist + slackBlend, maxOmegaDist + slackOmega);
    // Guarantee: upper - lower ≤ epsilon (by construction of N).
    // (Both slacks are ≤ ε/2 by grid choice; the max slack is ≤ ε.)

    // --- Signed split (M20) ----------------------------------------------
    if (cutDirection) {
        // Inside = side > 0 (interior cut — forbidden for ears).
        // Outside = side < 0 (the ear height).
        // Samples within ε_cert of side 0 are attributed to both sides
        // conservatively.
        const double sideBand = epsilon_;
        double maxIn = 0.0, maxOut = 0.0;
        auto account = [&](double side, double dist) {
            if (side > sideBand) maxIn = std::max(maxIn, dist);
            else if (side < -sideBand) maxOut = std::max(maxOut, dist);
            else { maxIn = std::max(maxIn, dist); maxOut = std::max(maxOut, dist); }
        };
        for (const auto& s : bsamples) account(s.side, s.dist);
        for (const auto& s : inSamples) account(s.side, s.dist);
        for (const auto& s : outSamples) account(s.side, s.dist);

        // Apply the same Lipschitz slack to the signed components.
        const double slack = std::max(slackBlend, slackOmega);
        cert.insideLo = maxIn;
        cert.insideHi = maxIn + slack;
        cert.outsideLo = maxOut;
        cert.outsideHi = maxOut + slack;
    }

    return cert;
}

} // namespace tether::motion
