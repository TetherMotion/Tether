/// @file PathRecompressorTests.cpp
/// @brief Tests for error-bounded NURBS recompression of PiecewiseNurbsPath
///
/// Exercises run discovery, certified error bounds, corner/dwell
/// preservation, adaptive split/verbatim fallback, and the converter
/// integration (attribute merging + dwell remap).

#include "tether/motion_planner/geometry/PathRecompressor.hpp"
#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/motion_planner/geometry/PointCurveDistance.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion;
using GCode::PlanningSegment;
using GCode::SegmentMotionType;
using GCode::Position;

namespace {

/// Build a path of degree-1 pieces through the given vertices.
PiecewiseNurbsPath polylinePath(const std::vector<RVec>& vts) {
    std::vector<NurbsCurve> pieces;
    pieces.reserve(vts.size() - 1);
    for (std::size_t i = 0; i + 1 < vts.size(); ++i)
        pieces.push_back(NurbsCurve::fromLine(vts[i], vts[i + 1]));
    return PiecewiseNurbsPath(std::move(pieces));
}

/// Tessellate a circular arc into polyline vertices (like slicer output).
std::vector<RVec> tessellateArc(const RVec& c, double r, double a0,
                                double a1, int n) {
    std::vector<RVec> vts;
    vts.reserve(n + 1);
    for (int i = 0; i <= n; ++i) {
        const double a = a0 + (a1 - a0) * i / n;
        RVec p = c;
        p[0] += r * std::cos(a);
        p[1] += r * std::sin(a);
        vts.push_back(p);
    }
    return vts;
}

/// Independent verification: max point-on-output → source-polyline
/// distance over a dense u-grid, plus vertex→output-piece distances.
double verifyDeviation(const NurbsCurve& out,
                       const PiecewiseNurbsPath& src,
                       std::size_t srcBegin, std::size_t srcEnd) {
    double worst = 0.0;
    const double uMin = out.knotMin(), uMax = out.knotMax();
    for (int k = 0; k <= 200; ++k) {
        const RVec p = out.evaluate(uMin + (uMax - uMin) * k / 200.0);
        // distance from p to the source pieces' polyline
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = srcBegin; i < srcEnd; ++i) {
            const NurbsCurve& s = src.piece(i);
            const RVec a = s.evaluate(s.knotMin());
            const RVec b = s.evaluate(s.knotMax());
            const RVec ab = b - a;
            const double l2 = ab.normSq();
            double t = l2 > 1e-14 ? (p - a).dot(ab) / l2 : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            best = std::min(best, p.distanceTo(a + ab * t));
        }
        worst = std::max(worst, best);
    }
    // vertices → certified distance to the fitted curve
    for (std::size_t i = srcBegin; i <= srcEnd; ++i) {
        RVec v;
        if (i < srcEnd) {
            const NurbsCurve& s = src.piece(i);
            v = s.evaluate(s.knotMin());
        } else {
            const NurbsCurve& s = src.piece(srcEnd - 1);
            v = s.evaluate(s.knotMax());
        }
        worst = std::max(worst, pointCurveDistance(out, v).distance);
    }
    return worst;
}

} // anonymous namespace

// ── Basic compression ────────────────────────────────────────────────────────

TEST(PathRecompressor, TessellatedCircleCompresses) {
    // 200 tiny segments on a quarter circle r=25 — typical slicer output.
    auto path = polylinePath(
        tessellateArc(RVec{0.0, 0.0, 0.0}, 25.0, 0.0, M_PI / 2, 200));
    ASSERT_EQ(path.numPieces(), 200u);

    RecompressConfig cfg;
    cfg.tolerance = 0.02;
    auto rc = recompressPiecewiseNurbs(path, cfg);

    EXPECT_LT(rc.path.numPieces(), 10u);
    EXPECT_LE(rc.maxCertifiedDeviation, cfg.tolerance);
    EXPECT_GT(rc.runsFitted, 0u);
    ASSERT_EQ(rc.sourceMap.size(), rc.path.numPieces());

    // Whole source covered by the mapping.
    EXPECT_EQ(rc.sourceMap.front().srcBegin, 0u);
    EXPECT_EQ(rc.sourceMap.back().srcEnd, 200u);
}

TEST(PathRecompressor, CollinearRunMergesToLine) {
    // A straight edge chopped into 50 pieces (bad G-code).
    std::vector<RVec> vts;
    for (int i = 0; i <= 50; ++i)
        vts.push_back(RVec{2.0 * i, 0.0, 0.0});
    auto path = polylinePath(std::move(vts));

    RecompressConfig cfg;
    auto rc = recompressPiecewiseNurbs(path, cfg);

    ASSERT_EQ(rc.path.numPieces(), 1u);
    EXPECT_EQ(rc.path.piece(0).degree(), 1);
    EXPECT_NEAR(rc.path.totalLength(), 100.0, 1e-9);
    EXPECT_NEAR(rc.maxCertifiedDeviation, 0.0, 1e-9);
}

// ── Corner / boundary preservation ───────────────────────────────────────────

TEST(PathRecompressor, SharpCornersPreserved) {
    // Square outline tessellated into micro-segments per edge.
    std::vector<RVec> vts;
    const double corners[4][2] = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    for (int e = 0; e < 4; ++e) {
        const double x0 = corners[e][0], y0 = corners[e][1];
        const double x1 = corners[(e + 1) % 4][0];
        const double y1 = corners[(e + 1) % 4][1];
        for (int i = 0; i < 25; ++i) { // 25 pieces per edge; corner vertex shared
            vts.push_back(RVec{x0 + (x1 - x0) * i / 25.0,
                               y0 + (y1 - y0) * i / 25.0, 0.0});
        }
    }
    vts.push_back(RVec{0.0, 0.0, 0.0}); // close the loop
    auto path = polylinePath(vts);
    ASSERT_EQ(path.numPieces(), 100u);

    RecompressConfig cfg;
    cfg.cornerAngleDeg = 10.0;
    auto rc = recompressPiecewiseNurbs(path, cfg);

    // Each edge collapses to a line (or one fitted piece): ~4-5 pieces.
    EXPECT_LE(rc.path.numPieces(), 8u);

    // The four corner vertices must still be path vertices: walk the
    // output and check each corner lies at a piece boundary.
    for (const auto& c : corners) {
        bool found = false;
        for (std::size_t j = 0; j < rc.path.numPieces(); ++j) {
            const auto& p = rc.path.piece(j);
            if (p.evaluate(p.knotMin()).distanceTo(RVec{c[0], c[1], 0.0}) <
                1e-9) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "corner (" << c[0] << "," << c[1]
                           << ") not preserved as a piece boundary";
    }
}

TEST(PathRecompressor, NonLinePieceEmittedVerbatim) {
    // line run, exact arc, line run
    std::vector<NurbsCurve> pieces;
    for (int i = 0; i < 30; ++i)
        pieces.push_back(NurbsCurve::fromLine(
            RVec{i * 0.5, 0.0, 0.0}, RVec{(i + 1) * 0.5, 0.0, 0.0}));
    const RVec arcStart{15.0, 0.0, 0.0};
    auto arc = NurbsCurve::fromArc(RVec{15.0, 10.0, 0.0}, 10.0,
                                   RVec{0.0, -1.0, 0.0}, RVec{1.0, 0.0, 0.0},
                                   0.0, M_PI / 2);
    const RVec arcEnd = arc.endPoint();
    pieces.push_back(arc);
    for (int i = 0; i < 30; ++i)
        pieces.push_back(NurbsCurve::fromLine(
            RVec{arcEnd[0] + i * 0.5, arcEnd[1], arcEnd[2]},
            RVec{arcEnd[0] + (i + 1) * 0.5, arcEnd[1], arcEnd[2]}));
    PiecewiseNurbsPath src(std::move(pieces));
    ASSERT_EQ(src.numPieces(), 61u);

    RecompressConfig cfg;
    auto rc = recompressPiecewiseNurbs(src, cfg);

    // Expect ~3 output pieces: line-run, arc (verbatim), line-run.
    ASSERT_EQ(rc.path.numPieces(), 3u);
    // The arc piece is emitted verbatim (degree 2, rational).
    EXPECT_EQ(rc.path.piece(1).degree(), 2);
    EXPECT_GT(rc.path.piece(1).numControlPoints(), 2u);
    // sourceMap covers all 61 source pieces.
    EXPECT_EQ(rc.sourceMap[2].srcEnd, 61u);
}

TEST(PathRecompressor, HardBreakSplitsRun) {
    std::vector<RVec> vts;
    for (int i = 0; i <= 40; ++i)
        vts.push_back(RVec{1.0 * i, 0.0, 0.0});
    auto path = polylinePath(vts); // 40 collinear pieces

    RecompressConfig cfg;
    // Hard break at piece 20 → two runs → two output line pieces.
    auto rc = recompressPiecewiseNurbs(path, cfg, {20});

    ASSERT_EQ(rc.path.numPieces(), 2u);
    EXPECT_EQ(rc.sourceMap[0].srcEnd, 20u);
    EXPECT_EQ(rc.sourceMap[1].srcBegin, 20u);
    // The break vertex is preserved exactly.
    const auto& p0 = rc.path.piece(0);
    EXPECT_NEAR(p0.evaluate(p0.knotMax())[0], 20.0, 1e-9);
}

// ── Certification ────────────────────────────────────────────────────────────

TEST(PathRecompressor, CertifiedBoundHoldsOnWigglyPath) {
    // Sine wave — smooth enough to fit but wiggly enough to exercise
    // the certification path.
    std::vector<RVec> vts;
    for (int i = 0; i <= 300; ++i)
        vts.push_back(RVec{0.3 * i, 5.0 * std::sin(0.05 * i), 0.0});
    auto path = polylinePath(vts);

    RecompressConfig cfg;
    cfg.tolerance = 0.05;
    auto rc = recompressPiecewiseNurbs(path, cfg);

    EXPECT_LT(rc.path.numPieces(), path.numPieces());
    EXPECT_LE(rc.maxCertifiedDeviation, cfg.tolerance);

    // Independent check: every fitted output piece stays within tol of
    // its source sub-polyline.
    for (std::size_t j = 0; j < rc.path.numPieces(); ++j) {
        const auto [b, e] = rc.sourceMap[j];
        const double d = verifyDeviation(rc.path.piece(j), path, b, e);
        EXPECT_LE(d, cfg.tolerance + 1e-9)
            << "output piece " << j << " deviates " << d << " mm";
    }
}

TEST(PathRecompressor, JaggedPathStaysWithinBound) {
    // High-frequency noise with amplitude above the tolerance: no
    // smooth fit can satisfy it — the recompressor must split and/or
    // keep pieces verbatim, never exceed the bound.
    std::vector<RVec> vts;
    for (int i = 0; i <= 120; ++i)
        vts.push_back(RVec{0.5 * i,
                           (i % 2 ? 0.08 : -0.08), 0.0});
    auto path = polylinePath(vts);

    RecompressConfig cfg;
    cfg.tolerance = 0.02; // smaller than the noise amplitude
    auto rc = recompressPiecewiseNurbs(path, cfg);

    EXPECT_LE(rc.maxCertifiedDeviation, cfg.tolerance);
    for (std::size_t j = 0; j < rc.path.numPieces(); ++j) {
        const auto [b, e] = rc.sourceMap[j];
        const double d = verifyDeviation(rc.path.piece(j), path, b, e);
        EXPECT_LE(d, cfg.tolerance + 1e-9);
    }
}

TEST(PathRecompressor, OutputChainG0Connected) {
    auto path = polylinePath(
        tessellateArc(RVec{0.0, 0.0, 0.0}, 30.0, -M_PI / 4, M_PI / 4, 80));
    RecompressConfig cfg;
    auto rc = recompressPiecewiseNurbs(path, cfg);

    EXPECT_TRUE(rc.path.isG0Connected(1e-9));
    // Endpoints preserved exactly.
    const RVec s0 = path.piece(0).evaluate(path.piece(0).knotMin());
    const RVec sn = path.piece(path.numPieces() - 1)
                        .evaluate(path.piece(path.numPieces() - 1).knotMax());
    const auto& q0 = rc.path.piece(0);
    const auto& qn = rc.path.piece(rc.path.numPieces() - 1);
    EXPECT_NEAR(q0.evaluate(q0.knotMin()).distanceTo(s0), 0.0, 1e-9);
    EXPECT_NEAR(qn.evaluate(qn.knotMax()).distanceTo(sn), 0.0, 1e-9);
}

// ── Converter integration ────────────────────────────────────────────────────

namespace {

/// Tessellated quarter-circle as PlanningSegments (bad G-code input).
std::vector<PlanningSegment> tessellatedSegments(int n, double feed) {
    std::vector<PlanningSegment> segs;
    const auto vts = tessellateArc(RVec{0.0, 0.0, 0.0}, 25.0, 0.0,
                                   M_PI / 2, n);
    for (int i = 0; i < n; ++i) {
        PlanningSegment seg;
        for (int d = 0; d < 3; ++d) {
            seg.start[d] = vts[i][d];
            seg.end[d] = vts[i + 1][d];
        }
        seg.motionType = SegmentMotionType::Linear;
        seg.feedRate = feed;
        segs.push_back(seg);
    }
    return segs;
}

} // anonymous namespace

TEST(PathRecompressorConverter, BadGcodeCompressesThroughConverter) {
    auto segs = tessellatedSegments(200, 3000.0);

    auto plain = piecewiseNurbsFromSegments(segs);
    EXPECT_EQ(plain.path.numPieces(), 200u);

    SegmentRecompressOptions opts;
    opts.enabled = true;
    opts.tolerance = 0.02;
    auto rc = piecewiseNurbsFromSegments(segs, opts);

    EXPECT_LT(rc.path.numPieces(), 20u);
    // Attribute arrays stay parallel to the output path.
    EXPECT_EQ(rc.feedRates.size(), rc.path.numPieces());
    EXPECT_EQ(rc.deviations.size(), rc.path.numPieces());
    EXPECT_EQ(rc.extruderSpeeds.size(), rc.path.numPieces());
    // Feed rate merge: uniform 3000 mm/min → 50 mm/s on every piece.
    for (double f : rc.feedRates)
        EXPECT_NEAR(f, 50.0, 1e-9);
}

TEST(PathRecompressorConverter, FeedRateMergeTakesTightest) {
    auto segs = tessellatedSegments(100, 3000.0);
    // Slow down a middle stretch — the merged piece must inherit the
    // tighter cap, not the looser one.
    for (int i = 40; i < 60; ++i) segs[i].feedRate = 1200.0;

    SegmentRecompressOptions opts;
    opts.enabled = true;
    opts.tolerance = 0.05;
    auto rc = piecewiseNurbsFromSegments(segs, opts);

    // No output piece may carry a cap looser than 1200/60=20 mm/s if it
    // covers a slow segment; check globally: min over output equals
    // min over inputs.
    double mn = std::numeric_limits<double>::infinity();
    for (double f : rc.feedRates)
        if (f > 0.0) mn = std::min(mn, f);
    EXPECT_NEAR(mn, 20.0, 1e-9);
}

TEST(PathRecompressorConverter, DwellBoundaryRemapped) {
    // line-run, dwell, line-run: the dwell splits the run; its arc
    // length must sit at the boundary between the two output pieces.
    auto mk = [](double x0, double x1, int n) {
        std::vector<PlanningSegment> out;
        for (int i = 0; i < n; ++i) {
            PlanningSegment seg;
            seg.start[0] = x0 + (x1 - x0) * i / n;
            seg.end[0] = x0 + (x1 - x0) * (i + 1) / n;
            seg.motionType = SegmentMotionType::Linear;
            seg.feedRate = 3000.0;
            out.push_back(seg);
        }
        return out;
    };
    auto segs = mk(0.0, 20.0, 40);
    PlanningSegment dwell;
    dwell.motionType = SegmentMotionType::Dwell;
    dwell.segmentTime = 0.5;
    segs.push_back(dwell);
    auto tail = mk(20.0, 40.0, 40);
    segs.insert(segs.end(), tail.begin(), tail.end());

    SegmentRecompressOptions opts;
    opts.enabled = true;
    auto rc = piecewiseNurbsFromSegments(segs, opts);

    ASSERT_EQ(rc.dwellPoints.size(), 1u);
    EXPECT_NEAR(rc.dwellPoints[0].arcLength, 20.0, 1e-6);
    EXPECT_NEAR(rc.dwellPoints[0].duration, 0.5, 1e-9);
    // The dwell position must be a piece boundary on the output path.
    double acc = 0.0;
    bool onBoundary = false;
    for (std::size_t j = 0; j < rc.path.numPieces(); ++j) {
        acc += rc.path.piece(j).length();
        if (std::abs(acc - rc.dwellPoints[0].arcLength) < 1e-6)
            onBoundary = true;
    }
    EXPECT_TRUE(onBoundary);
}

TEST(PathRecompressorConverter, DisabledKeepsOneToOne) {
    auto segs = tessellatedSegments(64, 3000.0);
    SegmentRecompressOptions opts; // enabled = false
    auto rc = piecewiseNurbsFromSegments(segs, opts);
    EXPECT_EQ(rc.path.numPieces(), 64u);
}
