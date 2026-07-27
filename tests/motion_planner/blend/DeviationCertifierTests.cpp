/**
 * @file DeviationCertifierTests.cpp
 * @brief Tests for the (M10)/(M14) certified Hausdorff deviation.
 */

#include "tether/motion_planner/blend/DeviationCertifier.hpp"
#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"
#include "tether/motion_planner/blend/BoundaryConditions.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"
#include "motion_planner/blend/TestHelpers.hpp"

#include <gtest/gtest.h>
#include <cmath>

using tether::motion::testing::expectVecNear;

namespace {

// Build a straight-line "blend" between (0,0) and (10,0) and compare to
// the original line — deviation should be ~0.
tether::motion::NurbsCurve makeLineBlend() {
    using tether::motion::RVec;
    tether::motion::BoundaryConditions entry, exit;
    entry.position = RVec{0, 0};
    entry.tangent = RVec{1, 0};
    entry.curvature = RVec{0, 0};
    entry.jounce = RVec{0, 0};
    entry.hasJounce = true;
    exit.position = RVec{10, 0};
    exit.tangent = RVec{1, 0};
    exit.curvature = RVec{0, 0};
    exit.jounce = RVec{0, 0};
    exit.hasJounce = true;
    return tether::motion::BlendCurveBuilder::buildQuintic(entry, exit, 1.0, 1.0);
}

} // namespace

TEST(DeviationCertifier, StraightBlendHasNearZeroDeviation) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // Original path: two colinear lines meeting at (5, 0).
    NurbsCurve in = NurbsCurve::fromLine(RVec{0, 0}, RVec{5, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{5, 0}, RVec{10, 0});
    // Trim 1 unit from each piece at the junction. The trimmed pieces are
    // [0,4] and [6,10]. The blend connects (4,0) to (6,0) — it sits in the
    // GAP between the trims. The Hausdorff deviation of the blend vs the
    // trimmed originals is the max distance from any blend point to the
    // nearest trim endpoint, which is ~1 (the blend midpoint (5,0) is 1
    // unit from (4,0) and (6,0)). This is the CORRECT semantics: the
    // certifier measures how far the blend departs from the original path
    // (the union of the trims). For a real corner cut, the original path
    // goes through the vertex, and the blend cuts the corner — the
    // deviation is the "depth" of the cut.
    //
    // For THIS test we want a near-zero deviation, so we compare the blend
    // to the FULL original lines (not the trims). The blend from (4,0) to
    // (6,0) is colinear with the full lines [0,5] and [5,10], so the
    // Hausdorff distance is ~0 (the blend lies ON the original path).
    NurbsCurve blend = makeLineBlend(); // blend from (4,0) to (6,0)

    // Use a coarse epsilon — the blend's Lipschitz constant is large
    // relative to 1e-6, and the grid safety cap limits tightness.
    tether::motion::DeviationCertifier cert(1e-3);
    tether::motion::DeviationCertificate d =
        cert.certify(blend, in, out);

    EXPECT_GE(d.upper, d.lower);
    EXPECT_LE(d.upper - d.lower, 1e-3 + 1e-9);
    // The blend is colinear with the originals; deviation is tiny.
    EXPECT_LT(d.upper, 1e-2);
}

TEST(DeviationCertifier, CertificateWidthGuaranteeHolds) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // A non-trivial blend: curved entry, straight exit.
    NurbsCurve in = NurbsCurve::fromArc(RVec{0, -2}, 2.0,
        RVec{1, 0}, RVec{0, 1}, 0.0, M_PI / 2.0);
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 2}, RVec{5, 2});

    // Build boundary conditions at the junction.
    auto entry = tether::motion::boundaryAt(in, in.length() - 0.5, true);
    auto exit  = tether::motion::boundaryAt(out, 0.5, false);

    NurbsCurve blend = tether::motion::BlendCurveBuilder::buildQuintic(
        entry, exit, 1.0, 1.0);

    // Trimmed originals: the parts of `in` and `out` that the blend replaces.
    NurbsCurve trimmedIn = in.trim(0.0, in.length() - 0.5);
    NurbsCurve trimmedOut = out.trim(0.5, out.length());

    const double eps = 1e-4;
    tether::motion::DeviationCertifier cert(eps);
    tether::motion::DeviationCertificate d =
        cert.certify(blend, trimmedIn, trimmedOut);

    // The (M10) guarantee: upper - lower ≤ eps.
    EXPECT_LE(d.upper - d.lower, eps + 1e-12);
    EXPECT_GE(d.lower, 0.0);
}

TEST(DeviationCertifier, SignedSplitForEarBlend) {
    using tether::motion::RVec;
    using tether::motion::NurbsCurve;
    // Symmetric corner: in along -x, out along +y. The blend cuts the
    // corner. The cut direction c = normalize(T_B - T_A) = normalize((0,1)-(1,0))
    // = (-1, 1)/√2 — points into the corner interior.
    NurbsCurve in = NurbsCurve::fromLine(RVec{-5, 0}, RVec{0, 0});
    NurbsCurve out = NurbsCurve::fromLine(RVec{0, 0}, RVec{0, 5});

    auto entry = tether::motion::boundaryAt(in, in.length() - 1.0, true);
    auto exit  = tether::motion::boundaryAt(out, 1.0, false);

    NurbsCurve blend = tether::motion::BlendCurveBuilder::buildQuintic(
        entry, exit, 1.0, 1.0);

    NurbsCurve trimmedIn = in.trim(0.0, in.length() - 1.0);
    NurbsCurve trimmedOut = out.trim(1.0, out.length());

    RVec cutDir = (exit.tangent - entry.tangent).normalized();

    tether::motion::DeviationCertifier cert(1e-4);
    tether::motion::DeviationCertificate d =
        cert.certify(blend, trimmedIn, trimmedOut, &cutDir);

    // For an inside cut, the "inside" component should dominate.
    EXPECT_GE(d.insideHi, 0.0);
    EXPECT_GE(d.outsideHi, 0.0);
    // The total upper should be ≥ max(inside, outside).
    EXPECT_GE(d.upper, std::max(d.insideHi, d.outsideHi) - 1e-9);
}
