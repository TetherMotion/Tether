/**
 * @file test_gcode_analysis_coverage.cpp
 * @brief Coverage tests for the tether::gcode::analysis analyzers
 *        (PatternAnalyzer, CncToolpathAnalyzer, FirstLayerAnalyzer,
 *        OverhangAnalyzer, SelfIntersectionAnalyzer, StockCollisionAnalyzer,
 *        ThermalCoolingAnalyzer, ThermalSimulationAnalyzer,
 *        VolumetricFlowAnalyzer, ZSeamAnalyzer, GcodeDiffAnalyzer,
 *        AnalysisUtil helpers).
 */

#include <gtest/gtest.h>

#include "tether/gcode/analysis/AnalysisTypes.hpp"
#include "tether/gcode/analysis/CncToolpathAnalyzer.hpp"
#include "tether/gcode/analysis/FirstLayerAnalyzer.hpp"
#include "tether/gcode/analysis/GcodeDiffAnalyzer.hpp"
#include "tether/gcode/analysis/OverhangAnalyzer.hpp"
#include "tether/gcode/analysis/PatternAnalyzer.hpp"
#include "tether/gcode/analysis/SelfIntersectionAnalyzer.hpp"
#include "tether/gcode/analysis/StockCollisionAnalyzer.hpp"
#include "tether/gcode/analysis/ThermalCoolingAnalyzer.hpp"
#include "tether/gcode/analysis/ThermalSimulationAnalyzer.hpp"
#include "tether/gcode/analysis/VolumetricFlowAnalyzer.hpp"
#include "tether/gcode/analysis/ZSeamAnalyzer.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace tether::gcode::analysis;
using GCode::PlanningSegment;
using GCode::SegmentMotionType;
using tether::gcode::SegmentSpeed;

namespace {

GCode::Position pos(double x, double y, double z = 0.0) {
    GCode::Position p;
    p.coords[0] = x;
    p.coords[1] = y;
    p.coords[2] = z;
    return p;
}

PlanningSegment linear(double x0, double y0, double x1, double y1,
                       double z, int block, double feed = 1500.0) {
    PlanningSegment s;
    s.start = pos(x0, y0, z);
    s.end = pos(x1, y1, z);
    s.motionType = SegmentMotionType::Linear;
    s.feedRate = feed;
    s.segmentLength = std::hypot(x1 - x0, y1 - y0);
    s.segmentTime = s.segmentLength / (feed / 60.0);
    s.blockIndex = block;
    return s;
}

PlanningSegment rapid(double x, double y, double z, int block) {
    PlanningSegment s;
    s.start = pos(0, 0, z);
    s.end = pos(x, y, z);
    s.motionType = SegmentMotionType::Rapid;
    s.isRapid = true;
    s.segmentLength = std::hypot(x, y);
    s.segmentTime = s.segmentLength / 100.0;
    s.blockIndex = block;
    return s;
}

PlanningSegment arcCW(double x0, double y0, double x1, double y1,
                      double cx, double cy, double z, int block) {
    PlanningSegment s;
    s.start = pos(x0, y0, z);
    s.end = pos(x1, y1, z);
    s.center = pos(cx, cy, z);
    s.motionType = SegmentMotionType::ArcCW;
    s.arcRadius = std::hypot(x0 - cx, y0 - cy);
    s.arcSweep = M_PI / 2.0;
    s.segmentLength = s.arcRadius * s.arcSweep;
    s.segmentTime = s.segmentLength / 25.0;
    s.blockIndex = block;
    return s;
}

/// One section of a 3D-print-style job: two 40 mm square layers with
/// extrusion, a retraction, a travel move and a quarter-arc corner.
struct PrintJob {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
};

PrintJob makePrintJob() {
    PrintJob j;
    int block = 0;
    auto push = [&](PlanningSegment s, std::string g, double eDelta) {
        SegmentSpeed ss;
        ss.timeStart = 0;
        ss.duration = s.segmentTime;
        ss.blockIndex = s.blockIndex;
        ss.lineNumber = static_cast<int32_t>(j.lines.size()) + 1;
        ss.speedLinear = s.segmentLength / s.segmentTime;
        j.segs.push_back(std::move(s));
        j.speeds.push_back(ss);
        j.lines.push_back(std::move(g));
        ++block;
    };

    double e = 0.0;
    for (double z : {0.2, 0.4}) {
        const double c[][2] = {{0, 0}, {40, 0}, {40, 40}, {0, 40}, {0, 0}};
        for (int i = 0; i < 4; ++i) {
            e += 2.0;
            push(linear(c[i][0], c[i][1], c[i + 1][0], c[i + 1][1], z, block,
                        1800.0),
                 "G1 X" + std::to_string(c[i + 1][0]) + " Y" +
                     std::to_string(c[i + 1][1]) + " Z" + std::to_string(z) +
                     " E" + std::to_string(e) + " F1800",
                 2.0);
        }
        // Retraction + travel to next layer.
        push(rapid(5, 5, z, block), "G1 E-3.0 F2400", 0.0);
        push(rapid(0, 0, z + 0.2, block), "G0 X0 Y0 Z" + std::to_string(z + 0.2), 0.0);
    }
    // An arc corner for pattern classification.
    push(arcCW(0, 0, 10, 10, 0, 10, 0.4, block),
         "G2 X10 Y10 I0 J10 E" + std::to_string(e + 1.0), 1.0);
    return j;
}

void expectPopulated(const Section& s) {
    EXPECT_FALSE(s.metrics.empty());
}

} // namespace

// ============================================================================
// PatternAnalyzer
// ============================================================================

TEST(PatternAnalyzerCoverage, EmptyInput) {
    Section s = analyzePatterns({}, {}, {});
    EXPECT_TRUE(s.metrics.empty());
}

TEST(PatternAnalyzerCoverage, LinearAndArcJob) {
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
    EXPECT_FALSE(s.name.empty());
}

TEST(PatternAnalyzerCoverage, SummaryDetailLevel) {
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "summary";
    Section s = analyzePatterns(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
}

// ============================================================================
// CncToolpathAnalyzer
// ============================================================================

TEST(CncToolpathAnalyzerCoverage, EmptyAndMilling) {
    Section empty = analyzeCncToolpath({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());

    std::vector<PlanningSegment> segs = {
        rapid(10, 10, 5, 0),
        linear(10, 10, 50, 10, -1, 1),
        linear(50, 10, 50, 40, -1, 2),
        arcCW(50, 40, 40, 50, 40, 40, -1, 3),
    };
    std::vector<SegmentSpeed> speeds(segs.size());
    for (size_t i = 0; i < segs.size(); ++i) {
        speeds[i].lineNumber = static_cast<int32_t>(i) + 1;
        speeds[i].duration = std::max(segs[i].segmentTime, 1e-6);
        speeds[i].speedLinear = segs[i].segmentLength / speeds[i].duration;
    }
    std::vector<std::string> lines = {"G0 X10 Y10 Z5", "G1 X50 Y10 Z-1 F800",
                                      "G1 Y40", "G2 X40 Y50 I-10 J0"};
    Options opts; opts.detailLevel = "full";
    Section s = analyzeCncToolpath(segs, speeds, lines, opts);
    expectPopulated(s);
}

// ============================================================================
// FirstLayerAnalyzer
// ============================================================================

TEST(FirstLayerAnalyzerCoverage, PrintJob) {
    Section empty = analyzeFirstLayer({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzeFirstLayer(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
}

// ============================================================================
// OverhangAnalyzer
// ============================================================================

TEST(OverhangAnalyzerCoverage, OverhangingLayer) {
    Section empty = analyzeOverhangs({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());
    auto j = makePrintJob();
    // Third layer juts out beyond the supported area.
    j.segs.push_back(linear(-20, 0, 60, 0, 0.6, 100));
    SegmentSpeed ss; ss.lineNumber = 11; ss.duration = 1.0; ss.speedLinear = 80;
    j.speeds.push_back(ss);
    j.lines.push_back("G1 X60 E20 F1800");
    Options opts; opts.detailLevel = "full";
    Section s = analyzeOverhangs(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
}

// ============================================================================
// SelfIntersectionAnalyzer
// ============================================================================

TEST(SelfIntersectionAnalyzerCoverage, CrossingPaths) {
    Section empty = analyzeSelfIntersections({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());

    std::vector<PlanningSegment> segs = {
        linear(0, 0, 10, 10, 0.2, 0),   // diagonal \
        linear(10, 10, 10, 0, 0.2, 1),  // down
        linear(10, 0, 0, 0, 0.2, 2),    // back left
        linear(0, 0, 0, 10, 0.2, 3),    // up — closes the bow-tie
        linear(0, 10, 10, 10, 0.2, 4),
        linear(10, 10, 0, 0, 0.2, 5),   // diagonal crosses segment 0
    };
    std::vector<SegmentSpeed> speeds(segs.size());
    for (size_t i = 0; i < segs.size(); ++i) {
        speeds[i].lineNumber = static_cast<int32_t>(i) + 1;
        speeds[i].duration = segs[i].segmentTime;
    }
    std::vector<std::string> lines(segs.size(), "G1");
    Options opts; opts.detailLevel = "full";
    Section s = analyzeSelfIntersections(segs, speeds, lines, opts);
    expectPopulated(s);
    EXPECT_GT(s.totalEventCount + s.events.size(), 0u);
}

// ============================================================================
// StockCollisionAnalyzer
// ============================================================================

TEST(StockCollisionAnalyzerCoverage, OutOfBoundsMove) {
    Section empty = analyzeStockCollision({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzeStockCollision(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
}

// ============================================================================
// ThermalCoolingAnalyzer / ThermalSimulationAnalyzer
// ============================================================================

TEST(ThermalAnalyzersCoverage, PrintJobWithTempLines) {
    Section e1 = analyzeThermalCooling({}, {}, {});
    Section e2 = analyzeThermalSimulation({}, {}, {});
    EXPECT_TRUE(e1.metrics.empty());
    EXPECT_TRUE(e2.metrics.empty());

    auto j = makePrintJob();
    j.lines.insert(j.lines.begin(),
                   {"M104 S200", "M109 S200", "M106 S255", "M140 S60", "M190 S60"});
    // Re-map lineNumbers past the inserted header lines.
    for (size_t i = 0; i < j.speeds.size(); ++i)
        j.speeds[i].lineNumber = static_cast<int32_t>(i) + 6;

    Options opts; opts.detailLevel = "full";
    Section c = analyzeThermalCooling(j.segs, j.speeds, j.lines, opts);
    Section t = analyzeThermalSimulation(j.segs, j.speeds, j.lines, opts);
    expectPopulated(c);
    expectPopulated(t);
}

// ============================================================================
// VolumetricFlowAnalyzer
// ============================================================================

TEST(VolumetricFlowAnalyzerCoverage, ExtrusionMoves) {
    Section empty = analyzeVolumetricFlow({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzeVolumetricFlow(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
    EXPECT_FALSE(s.name.empty());
}

// ============================================================================
// ZSeamAnalyzer
// ============================================================================

TEST(ZSeamAnalyzerCoverage, LayerChangesAtCorner) {
    Section empty = analyzeZSeam({}, {}, {});
    EXPECT_TRUE(empty.metrics.empty());
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzeZSeam(j.segs, j.speeds, j.lines, opts);
    expectPopulated(s);
}

// ============================================================================
// GcodeDiffAnalyzer
// ============================================================================

TEST(GcodeDiffAnalyzerCoverage, IdenticalFiles) {
    std::vector<std::string> lines = {"G28", "G1 X10 Y10 E2 F1500", "M104 S200"};
    auto r = GcodeDiffAnalyzer::diff(lines, lines);
    EXPECT_EQ(r.summary.totalAdded, 0);
    EXPECT_EQ(r.summary.totalRemoved, 0);
    EXPECT_EQ(r.summary.totalModified, 0);
    EXPECT_GT(r.summary.totalUnchanged, 0);
    EXPECT_DOUBLE_EQ(r.summary.similarityScore, 1.0);
}

TEST(GcodeDiffAnalyzerCoverage, AddedRemovedModified) {
    std::vector<std::string> oldLines = {
        "G28", "G1 X10 Y10 E2 F1500", "G1 X20 Y20 E3", "M104 S200", "G1 Z5"};
    std::vector<std::string> newLines = {
        "G28", "G1 X10 Y10 E2.5 F1500", "G1 X25 Y20 E3", "G1 X0 Y0", "M104 S210"};
    auto r = GcodeDiffAnalyzer::diff(oldLines, newLines);
    EXPECT_GT(r.summary.totalModified + r.summary.totalAdded +
                  r.summary.totalRemoved,
              0);
    EXPECT_LT(r.summary.similarityScore, 1.0);
    EXPECT_FALSE(r.wordChanges.empty() ||
                 (r.added.empty() && r.removed.empty() && r.modified.empty()));
}

TEST(GcodeDiffAnalyzerCoverage, EmptyInputs) {
    auto r = GcodeDiffAnalyzer::diff({}, {});
    EXPECT_EQ(r.summary.totalUnchanged, 0);
    auto one = GcodeDiffAnalyzer::diff({"G28"}, {});
    EXPECT_GT(one.summary.totalRemoved, 0);
}

// ============================================================================
// Detail-level option coverage across a shared job
// ============================================================================

TEST(AnalysisOptionsCoverage, DetailLevelsVaryEventDepth) {
    auto j = makePrintJob();
    for (const char* lvl : {"summary", "standard", "full"}) {
        Options opts; opts.detailLevel = lvl;
        Section s = analyzePatterns(j.segs, j.speeds, j.lines, opts);
        EXPECT_FALSE(s.metrics.empty()) << "level " << lvl;
    }
}

// ============================================================================
// Coverage additions — pattern detection, stock bounds, CNC chip-load
// ============================================================================

namespace {

/// Push a segment + matching speed entry + gcode line (like makePrintJob).
void pushSeg(std::vector<PlanningSegment>& segs,
             std::vector<SegmentSpeed>& speeds,
             std::vector<std::string>& lines,
             PlanningSegment s, const std::string& g) {
    SegmentSpeed ss;
    ss.timeStart = 0;
    ss.duration = s.segmentTime;
    ss.blockIndex = s.blockIndex;
    ss.lineNumber = static_cast<int32_t>(lines.size()) + 1;
    ss.speedLinear = s.segmentTime > 0 ? s.segmentLength / s.segmentTime : 0;
    segs.push_back(std::move(s));
    speeds.push_back(ss);
    lines.push_back(g);
}

PlanningSegment arcR(double x0, double y0, double x1, double y1,
                     double cx, double cy, int block, bool ccw = false) {
    PlanningSegment s;
    s.start = pos(x0, y0, 0.0);
    s.end = pos(x1, y1, 0.0);
    s.center = pos(cx, cy, 0.0);
    s.motionType = ccw ? SegmentMotionType::ArcCCW : SegmentMotionType::ArcCW;
    s.arcRadius = std::hypot(x0 - cx, y0 - cy);
    s.arcSweep = M_PI / 2.0;
    s.segmentLength = s.arcRadius * s.arcSweep;
    s.segmentTime = s.segmentLength / 25.0;
    s.blockIndex = block;
    return s;
}

/// Closed square loop starting/ending at (cx,cy) ± half, as 4 linears.
void pushSquare(std::vector<PlanningSegment>& segs,
                std::vector<SegmentSpeed>& speeds,
                std::vector<std::string>& lines,
                double cx, double cy, double half, int& block) {
    const double c[][2] = {{cx - half, cy - half}, {cx + half, cy - half},
                           {cx + half, cy + half}, {cx - half, cy + half},
                           {cx - half, cy - half}};
    for (int i = 0; i < 4; ++i) {
        pushSeg(segs, speeds, lines,
                linear(c[i][0], c[i][1], c[i + 1][0], c[i + 1][1], 0.0, block),
                "G1");
        ++block;
    }
}

} // namespace

TEST(PatternAnalyzerCoverage, SpiralIncreasingRadii) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Three quarter arcs, same center, radii 5/10/15 — increasing spiral.
    pushSeg(segs, speeds, lines, arcR(5, 0, 0, 5, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(10, 0, 0, 10, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(15, 0, 0, 15, 0, 0, block++), "G2");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    bool hasSpiral = false;
    for (const auto& e : s.events) if (e.type == "spiral") hasSpiral = true;
    EXPECT_TRUE(hasSpiral);
}

TEST(PatternAnalyzerCoverage, SpiralDecreasingRadii) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    pushSeg(segs, speeds, lines, arcR(15, 0, 0, 15, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(10, 0, 0, 10, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(5, 0, 0, 5, 0, 0, block++), "G2");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    bool hasSpiral = false;
    for (const auto& e : s.events) if (e.type == "spiral") hasSpiral = true;
    EXPECT_TRUE(hasSpiral);
}

TEST(PatternAnalyzerCoverage, SpiralBrokenTrendKeepsPrefix) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Radii 5→15→8 — non-monotonic: the trend break emits the 5→15
    // prefix as a two-arc spiral.
    pushSeg(segs, speeds, lines, arcR(5, 0, 0, 5, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(15, 0, 0, 15, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(8, 0, 0, 8, 0, 0, block++), "G2");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    bool hasSpiral = false;
    for (const auto& e : s.events) if (e.type == "spiral") hasSpiral = true;
    EXPECT_TRUE(hasSpiral);
}

TEST(PatternAnalyzerCoverage, ConcentricArcLoops) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Two concentric full circles about (0,0), r=10 then r=15 — arc-loop
    // centroids are exact (no vertex bias) so detectConcentric pairs them.
    for (double r : {10.0, 15.0}) {
        pushSeg(segs, speeds, lines, arcR(r, 0, 0, r, 0, 0, block++), "G2");
        pushSeg(segs, speeds, lines, arcR(0, r, -r, 0, 0, 0, block++), "G2");
        pushSeg(segs, speeds, lines, arcR(-r, 0, 0, -r, 0, 0, block++), "G2");
        pushSeg(segs, speeds, lines, arcR(0, -r, r, 0, 0, 0, block++), "G2");
    }

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    bool hasConcentric = false;
    for (const auto& e : s.events) if (e.type == "concentric") hasConcentric = true;
    EXPECT_TRUE(hasConcentric);
}

TEST(PatternAnalyzerCoverage, ArcLoopDetected) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Full circle as 4 quarter arcs, radius 10 → arc-sweep loop ≥ 1.75π.
    pushSeg(segs, speeds, lines, arcR(10, 0, 0, 10, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(0, 10, -10, 0, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(-10, 0, 0, -10, 0, 0, block++), "G2");
    pushSeg(segs, speeds, lines, arcR(0, -10, 10, 0, 0, 0, block++), "G2");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    // An arc loop alone is a loop but needs a partner for "concentric" —
    // still populates arc metrics.
    expectPopulated(s);
}

TEST(PatternAnalyzerCoverage, LinearLoopDetected) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // A single closed square exercises tryCloseLoop's linear branch.
    pushSquare(segs, speeds, lines, 20, 20, 20, block);

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    expectPopulated(s);
}

TEST(PatternAnalyzerCoverage, ZigzagInfill) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Four alternating 10 mm strokes spaced 2 mm in Y.
    pushSeg(segs, speeds, lines, linear(0, 0, 10, 0, 0, block++), "G1");
    pushSeg(segs, speeds, lines, linear(10, 2, 0, 2, 0, block++), "G1");
    pushSeg(segs, speeds, lines, linear(0, 4, 10, 4, 0, block++), "G1");
    pushSeg(segs, speeds, lines, linear(10, 6, 0, 6, 0, block++), "G1");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    bool hasZigzag = false;
    for (const auto& e : s.events) if (e.type == "zigzag") hasZigzag = true;
    EXPECT_TRUE(hasZigzag);
}

TEST(PatternAnalyzerCoverage, ZigzagTooFewStrokes) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    pushSeg(segs, speeds, lines, linear(0, 0, 10, 0, 0, block++), "G1");
    pushSeg(segs, speeds, lines, linear(10, 2, 0, 2, 0, block++), "G1");

    Options opts; opts.detailLevel = "full";
    Section s = analyzePatterns(segs, speeds, lines, opts);
    for (const auto& e : s.events) EXPECT_NE(e.type, "zigzag");
}

TEST(StockCollisionAnalyzerCoverage, StockBoundsAndViolations) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {
        "; STOCK_X: 100", "; STOCK_Y: 100", "; STOCK_Z: 20"};
    int block = 0;

    // Rapid inside stock → rapid_collision.
    PlanningSegment r = rapid(10, 10, 5, block++);
    pushSeg(segs, speeds, lines, r, "G0 X10 Y10 Z5");
    // Cut below stock bottom → cut_below_stock.
    pushSeg(segs, speeds, lines, linear(0, 0, 5, 0, -2, block++), "G1");
    // Rapid below safeZ → clearance_violation.
    pushSeg(segs, speeds, lines, rapid(200, 200, 25, block++), "G0 X200 Y200 Z25");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeStockCollision(segs, speeds, lines, opts);
    EXPECT_EQ(s.name, "stock_collision");
    EXPECT_LT(s.score, 100.0);
    bool rapidColl = false, below = false, clearance = false;
    for (const auto& e : s.events) {
        if (e.type == "rapid_collision") rapidColl = true;
        if (e.type == "cut_below_stock") below = true;
        if (e.type == "clearance_violation") clearance = true;
    }
    EXPECT_TRUE(rapidColl);
    EXPECT_TRUE(below);
    EXPECT_TRUE(clearance);
}

TEST(StockCollisionAnalyzerCoverage, PrintAreaBoundsOutOfBounds) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {
        "; print_area_min_x:0", "; print_area_max_x:100",
        "; print_area_min_y:0", "; print_area_max_y:100"};
    int block = 0;
    pushSeg(segs, speeds, lines, linear(50, 50, 150, 50, 0.2, block++), "G1");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeStockCollision(segs, speeds, lines, opts);
    bool oob = false;
    for (const auto& e : s.events) if (e.type == "out_of_bounds") oob = true;
    EXPECT_TRUE(oob);
}

TEST(StockCollisionAnalyzerCoverage, CircularBed) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {
        "; BED_X: 200", "; BED_Y: 200", "; bed_shape=0x0,CIRCLE"};
    int block = 0;
    // Bed is centered ±100; point (95,95) is outside r=100 circle.
    pushSeg(segs, speeds, lines, linear(95, 95, 98, 95, 0.2, block++), "G1");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeStockCollision(segs, speeds, lines, opts);
    bool oob = false;
    for (const auto& e : s.events) if (e.type == "out_of_bounds") oob = true;
    EXPECT_TRUE(oob);
}

TEST(StockCollisionAnalyzerCoverage, SummaryAndTopLimit) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {
        "; STOCK_X: 100", "; STOCK_Y: 100", "; STOCK_Z: 20"};
    int block = 0;
    pushSeg(segs, speeds, lines, rapid(10, 10, 5, block++), "G0");
    pushSeg(segs, speeds, lines, linear(0, 0, 5, 0, -2, block++), "G1");

    Options summary; summary.detailLevel = "summary";
    Section ss = analyzeStockCollision(segs, speeds, lines, summary);
    EXPECT_TRUE(ss.events.empty());  // summary returns before events

    Options lim; lim.detailLevel = "full"; lim.topEventLimit = 1;
    Section sl = analyzeStockCollision(segs, speeds, lines, lim);
    EXPECT_EQ(sl.events.size(), 1u);
    EXPECT_TRUE(sl.hasMoreEvents);
}

TEST(StockCollisionAnalyzerCoverage, NoBoundsSkipsChecks) {
    auto j = makePrintJob();
    Options opts; opts.detailLevel = "full";
    Section s = analyzeStockCollision(j.segs, j.speeds, {}, opts);
    EXPECT_TRUE(s.events.empty());
    EXPECT_DOUBLE_EQ(s.score, 100.0);
}

TEST(CncToolpathAnalyzerCoverage, SpindleToolAndChipEvents) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;

    // Program: spindle on, tool select, CAM comments, cutting moves.
    lines.push_back("M3 S10000");
    lines.push_back("T2");
    lines.push_back("; MATERIAL: STEEL");
    lines.push_back("; TOOL_DIA: 8");
    lines.push_back("; FLUTES: 4");
    lines.push_back("; AXIAL_DEPTH: 3");
    lines.push_back("; RADIAL_DEPTH: 2");

    int block = 0;
    auto cut = [&](double x0, double y0, double x1, double y1,
                   const std::string& g) {
        pushSeg(segs, speeds, lines,
                linear(x0, y0, x1, y1, -1.0, block++, 1200.0), g);
    };
    cut(0, 0, 10, 0, "G1 X10 F1200");
    lines.push_back("T3");          // tool change mid-job
    cut(10, 0, 20, 0, "G1 X20 F1200");
    lines.push_back("S5000");       // spindle rpm change
    cut(20, 0, 30, 0, "G1 X30 F1200");
    lines.push_back("M5");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeCncToolpath(segs, speeds, lines, opts);
    EXPECT_EQ(s.name, "cnc_toolpath");
    bool toolChg = false, spinChg = false;
    for (const auto& e : s.events) {
        if (e.type == "tool_change") toolChg = true;
        if (e.type == "spindle_change") spinChg = true;
    }
    EXPECT_TRUE(toolChg);
    EXPECT_TRUE(spinChg);
}

TEST(CncToolpathAnalyzerCoverage, ChipLoadOutOfRange) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {"M3 S1000", "; FLUTES: 2"};
    int block = 0;
    // feed 5000 / (1000 * 2) = 2.5 mm/tooth — way over aluminum 0.15 max.
    pushSeg(segs, speeds, lines, linear(0, 0, 50, 0, -1, block++, 5000.0), "G1");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeCncToolpath(segs, speeds, lines, opts);
    bool chipHigh = false;
    for (const auto& e : s.events) {
        if (e.type == "chip_load" || e.message.find("chip") != std::string::npos)
            chipHigh = true;
    }
    EXPECT_TRUE(chipHigh);
}

TEST(CncToolpathAnalyzerCoverage, NoSpindleNoEvents) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines;
    int block = 0;
    // Cutting move with no spindle/tool — chipEvents empty path.
    pushSeg(segs, speeds, lines, linear(0, 0, 10, 0, -1, block++), "G1");
    pushSeg(segs, speeds, lines, rapid(100, 100, 10, block++), "G0");

    Options opts; opts.detailLevel = "full";
    Section s = analyzeCncToolpath(segs, speeds, lines, opts);
    // cuttingMoves > 0 → section emitted; no chip events.
    if (!s.name.empty()) {
        for (const auto& e : s.events) {
            EXPECT_NE(e.type, "chip_load");
        }
    }
}

TEST(CncToolpathAnalyzerCoverage, SummarySkipsEvents) {
    std::vector<PlanningSegment> segs;
    std::vector<SegmentSpeed> speeds;
    std::vector<std::string> lines = {"M3 S8000", "T1"};
    int block = 0;
    pushSeg(segs, speeds, lines, linear(0, 0, 10, 0, -1, block++), "G1");

    Options opts; opts.detailLevel = "summary";
    Section s = analyzeCncToolpath(segs, speeds, lines, opts);
    EXPECT_TRUE(s.events.empty());
    expectPopulated(s);
}
