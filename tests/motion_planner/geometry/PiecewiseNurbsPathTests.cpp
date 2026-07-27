/**
 * @file PiecewiseNurbsPathTests.cpp
 * @brief Tests for PiecewiseNurbsPath: prefix/eval correctness, trim,
 *        connectivity diagnostics, and laziness on a 10^6-piece path.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

using tether::motion::ArcDerivatives;
using tether::motion::NurbsCurve;
using tether::motion::PiecewiseNurbsPath;
using tether::motion::RVec;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectVecNear(const RVec& a, const RVec& b, double tol) {
    ASSERT_EQ(a.dim(), b.dim());
    for (std::size_t i = 0; i < a.dim(); ++i) {
        EXPECT_NEAR(a[i], b[i], tol) << "component " << i;
    }
}

// line (0,0)->(3,0), quarter arc R=3 to (6,3), line (6,3)->(6,7)
PiecewiseNurbsPath makeMixedPath() {
    NurbsCurve l0 = NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{3.0, 0.0});
    NurbsCurve arc = NurbsCurve::fromArc(RVec{3.0, 3.0}, 3.0, RVec{1.0, 0.0},
                                         RVec{0.0, 1.0}, -kPi / 2.0, kPi / 2.0);
    NurbsCurve l1 = NurbsCurve::fromLine(RVec{6.0, 3.0}, RVec{6.0, 7.0});
    return PiecewiseNurbsPath({l0, arc, l1});
}

} // namespace

TEST(PiecewiseNurbsPath, MixedPathLengthAndEval) {
    PiecewiseNurbsPath path = makeMixedPath();
    const double expectedTotal = 3.0 + 3.0 * kPi / 2.0 + 4.0;
    EXPECT_NEAR(path.totalLength(), expectedTotal, 1e-9);
    EXPECT_TRUE(path.isG0Connected(1e-12));

    expectVecNear(path.evaluate(0.0, 0).position, RVec{0.0, 0.0}, 1e-12);
    expectVecNear(path.evaluate(expectedTotal, 0).position, RVec{6.0, 7.0},
                  1e-12);

    // Junction at s=3: position (3,0), tangent +x (G1 through the corner).
    ArcDerivatives j = path.evaluate(3.0, 1);
    expectVecNear(j.position, RVec{3.0, 0.0}, 1e-9);
    expectVecNear(j.tangent, RVec{1.0, 0.0}, 1e-9);

    // Mid-arc: s = 3 + 3π/4 → angle −π/4 on the circle.
    ArcDerivatives mid = path.evaluate(3.0 + 3.0 * kPi / 4.0, 2);
    const double inv = 1.0 / std::sqrt(2.0);
    expectVecNear(mid.position, RVec{3.0 + 3.0 * inv, 3.0 - 3.0 * inv}, 1e-9);
    expectVecNear(mid.tangent, RVec{inv, inv}, 1e-9);
    EXPECT_NEAR(mid.curvature.norm(), 1.0 / 3.0, 1e-9);
}

TEST(PiecewiseNurbsPath, LocatePieceIndices) {
    PiecewiseNurbsPath path = makeMixedPath();
    const double total = path.totalLength();

    EXPECT_EQ(path.locate(0.0).piece, 0u);
    EXPECT_EQ(path.locate(1.5).piece, 0u);
    EXPECT_EQ(path.locate(4.0).piece, 1u);
    EXPECT_EQ(path.locate(total - 1.0).piece, 2u);
    EXPECT_EQ(path.locate(total).piece, 2u);
    // Out-of-range s clamps instead of crashing.
    EXPECT_EQ(path.locate(-5.0).piece, 0u);
    EXPECT_EQ(path.locate(total + 5.0).piece, 2u);
}

TEST(PiecewiseNurbsPath, TrimConsistency) {
    PiecewiseNurbsPath path = makeMixedPath();
    const double s0 = 1.5;
    const double s1 = 3.0 + 3.0 * kPi / 4.0;

    PiecewiseNurbsPath sub = path.trim(s0, s1);
    EXPECT_NEAR(sub.totalLength(), s1 - s0, 1e-8);
    EXPECT_TRUE(sub.isG0Connected(1e-9));
    expectVecNear(sub.evaluate(0.0, 0).position,
                  path.evaluate(s0, 0).position, 1e-8);
    expectVecNear(sub.evaluate(s1 - s0, 0).position,
                  path.evaluate(s1, 0).position, 1e-8);
}

TEST(PiecewiseNurbsPath, DisconnectedInputAllowedButDetected) {
    PiecewiseNurbsPath path(
        {NurbsCurve::fromLine(RVec{0.0, 0.0}, RVec{1.0, 0.0}),
         NurbsCurve::fromLine(RVec{5.0, 5.0}, RVec{6.0, 5.0})});
    EXPECT_FALSE(path.isG0Connected(1e-9));
    // Evaluation still works across the gap (by arc length bookkeeping).
    EXPECT_NEAR(path.totalLength(), 2.0, 1e-12);
    expectVecNear(path.evaluate(1.5, 0).position, RVec{5.5, 5.0}, 1e-12);
}

TEST(PiecewiseNurbsPath, LazyPerPieceArcLength) {
    // Five non-linear (quintic Bézier) pieces: arc length needs quadrature.
    std::mt19937_64 rng(41);
    std::uniform_real_distribution<double> coord(-3.0, 3.0);

    std::vector<NurbsCurve> pieces;
    RVec prev{0.0, 0.0};
    for (int i = 0; i < 5; ++i) {
        std::vector<RVec> cps(6);
        cps[0] = prev;
        for (int k = 1; k < 6; ++k) {
            cps[k] = RVec{coord(rng), coord(rng)};
        }
        prev = cps[5];
        std::vector<double> wts(6, 1.0);
        std::vector<double> knots(6, 0.0);
        knots.insert(knots.end(), 6, 1.0);
        pieces.emplace_back(cps, wts, knots, 5);
    }
    PiecewiseNurbsPath path(pieces);

    // Nothing computed at construction.
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        EXPECT_EQ(path.piece(i).arcLengthComputationCount(), 0u) << "piece " << i;
    }

    // Query only near the start: binary search touches prefix lengths of the
    // first pieces only; later pieces must stay completely untouched.
    path.evaluate(0.01, 1);
    EXPECT_EQ(path.piece(4).arcLengthComputationCount(), 0u);
    EXPECT_EQ(path.piece(3).arcLengthComputationCount(), 0u);
    EXPECT_EQ(path.piece(2).arcLengthComputationCount(), 0u);
    EXPECT_GT(path.piece(0).arcLengthComputationCount(), 0u);
}

TEST(PiecewiseNurbsPath, HugePathLazinessAndMemory) {
    // 1,000,000 line pieces in a zig-zag: (i, i%2) -> (i+1, (i+1)%2).
    constexpr std::size_t kNumPieces = 1000000;
    constexpr double kSegLen = 1.41421356237309504880; // sqrt(2)

    std::vector<NurbsCurve> pieces;
    pieces.reserve(kNumPieces);
    const auto t0 = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < kNumPieces; ++i) {
        pieces.push_back(NurbsCurve::fromLine(
            RVec{static_cast<double>(i), static_cast<double>(i % 2)},
            RVec{static_cast<double>(i + 1), static_cast<double>((i + 1) % 2)}));
    }
    PiecewiseNurbsPath path(std::move(pieces));
    const auto t1 = std::chrono::steady_clock::now();
    const double constructSeconds =
        std::chrono::duration<double>(t1 - t0).count();
    // Generous Debug-mode budget; construction must be linear, no sampling.
    EXPECT_LT(constructSeconds, 30.0);

    // Memory: O(pieces), no per-point sample tables. Even a tiny 100-entry
    // sample table per piece would add 1e6 * 100 * 16 bytes = 1.6 GB; the
    // path stores only control points/weights/knots (<< 400 MB total).
    EXPECT_LT(path.estimatedMemoryBytes(), 400u * 1024u * 1024u);

    // Total length is exact closed form for lines — zero quadrature calls.
    EXPECT_NEAR(path.totalLength(), kNumPieces * kSegLen, 1e-3);
    EXPECT_EQ(path.totalArcLengthComputations(), 0u);

    // Known positions: s = k·sqrt(2) is the k-th vertex.
    expectVecNear(path.evaluate(1000.0 * kSegLen, 0).position,
                  RVec{1000.0, 0.0}, 1e-6);
    expectVecNear(path.evaluate(1001.0 * kSegLen, 0).position,
                  RVec{1001.0, 1.0}, 1e-6);

    // 10,000 random evaluations within a generous wall-time budget (Debug).
    std::mt19937_64 rng(43);
    std::uniform_real_distribution<double> uni(0.0, path.totalLength());
    const auto t2 = std::chrono::steady_clock::now();
    double checksum = 0.0;
    for (int i = 0; i < 10000; ++i) {
        checksum += path.evaluate(uni(rng), 0).position[0];
    }
    const auto t3 = std::chrono::steady_clock::now();
    const double evalSeconds = std::chrono::duration<double>(t3 - t2).count();
    EXPECT_GT(checksum, 0.0);
    EXPECT_LT(evalSeconds, 5.0);

    // Still no quadrature anywhere (lines are exact).
    EXPECT_EQ(path.totalArcLengthComputations(), 0u);
}
