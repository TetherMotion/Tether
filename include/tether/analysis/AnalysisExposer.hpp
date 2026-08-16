/**
 * @file AnalysisExposer.hpp
 * @brief Expose all G-code analysis results via the Tether IO protocol.
 *
 * This exposer registers signals for each of the 10 analysis components
 * so that external clients (e.g. WebGCodeViewer) can read the results
 * over the Tether IO protocol (SLIP-based binary, TCP port 4000).
 *
 * The exposer holds pointers to the 10 analyzer objects and exposes
 * their latest results as F64 / U32 / Bool / String signals.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/AccelerationProfileAnalyzer.hpp"
#include "tether/analysis/ArcAnalyzer.hpp"
#include "tether/analysis/CoordinateSystemAnalyzer.hpp"
#include "tether/analysis/CurvatureAnalyzer.hpp"
#include "tether/analysis/MachineLimitChecker.hpp"
#include "tether/analysis/ModalStateAnalyzer.hpp"
#include "tether/analysis/PathContinuityChecker.hpp"
#include "tether/analysis/PathTopologyDetector.hpp"
#include "tether/analysis/RetractionAnalyzer.hpp"
#include "tether/analysis/ToolpathEfficiencyAnalyzer.hpp"
#include "tether/io/ParameterExposer.hpp"
#include "tether/io/Registry.hpp"

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Module ID for the G-code analysis component.
inline constexpr uint64_t ModuleIdAnalysis = 0x0012;

/// @brief Holds the latest analysis results and exposes them via IO protocol.
///
/// Usage:
/// @code
///   AnalysisExposer exposer;
///   exposer.expose(registry, ModuleIdAnalysis);
///   exposer.analyze(gcodeLines);  // Run all 10 analyses
///   // External client reads signals via IO protocol
/// @endcode
class AnalysisExposer : public tether::io::IParameterExposer {
public:
    AnalysisExposer() {
        analyzers_.machineLimit = std::make_unique<MachineLimitChecker>();
        analyzers_.curvature = std::make_unique<CurvatureAnalyzer>();
        analyzers_.arc = std::make_unique<ArcAnalyzer>();
        analyzers_.modal = std::make_unique<ModalStateAnalyzer>();
        analyzers_.topology = std::make_unique<PathTopologyDetector>();
        analyzers_.efficiency = std::make_unique<ToolpathEfficiencyAnalyzer>();
        analyzers_.retraction = std::make_unique<RetractionAnalyzer>();
        analyzers_.accel = std::make_unique<AccelerationProfileAnalyzer>();
        analyzers_.coord = std::make_unique<CoordinateSystemAnalyzer>();
        analyzers_.continuity = std::make_unique<PathContinuityChecker>();
    }

    /// @brief Run all 10 analyses on the given G-code and store results.
    void analyze(const std::vector<std::string>& gcodeLines) {
        std::lock_guard lock(mutex_);
        results_.machineLimit = analyzers_.machineLimit->check(gcodeLines);
        results_.curvature = analyzers_.curvature->analyze(gcodeLines);
        results_.arc = analyzers_.arc->analyze(gcodeLines);
        results_.modal = analyzers_.modal->analyze(gcodeLines);
        results_.topology = analyzers_.topology->analyze(gcodeLines);
        results_.efficiency = analyzers_.efficiency->analyze(gcodeLines);
        results_.retraction = analyzers_.retraction->analyze(gcodeLines);
        results_.accel = analyzers_.accel->analyze(gcodeLines);
        results_.coord = analyzers_.coord->analyze(gcodeLines);
        results_.continuity = analyzers_.continuity->analyze(gcodeLines);
        hasResults_.store(true, std::memory_order_release);
    }

    const char* moduleName() const override { return "gcode_analysis"; }

    void expose(tether::io::Registry& registry, uint64_t idBase) override {
        const std::string group = "gcode_analysis";
        using VT = tether::io::ValueType;

        // Helper lambdas for F64 and U32 signals.
        auto addF64 = [&](uint32_t localId, const char* name,
                          const char* desc, auto getter) {
            registry.addSignal({
                tether::io::makeId(idBase, localId), name, desc, group, VT::F64,
                [this, getter](void* dest) {
                    double v = 0.0;
                    if (hasResults_.load(std::memory_order_acquire)) {
                        std::lock_guard lock(mutex_);
                        v = getter();
                    }
                    std::memcpy(dest, &v, sizeof(double));
                }
            });
        };
        auto addU32 = [&](uint32_t localId, const char* name,
                          const char* desc, auto getter) {
            registry.addSignal({
                tether::io::makeId(idBase, localId), name, desc, group, VT::U32,
                [this, getter](void* dest) {
                    uint32_t v = 0;
                    if (hasResults_.load(std::memory_order_acquire)) {
                        std::lock_guard lock(mutex_);
                        v = static_cast<uint32_t>(getter());
                    }
                    std::memcpy(dest, &v, sizeof(uint32_t));
                }
            });
        };
        auto addBool = [&](uint32_t localId, const char* name,
                           const char* desc, auto getter) {
            registry.addSignal({
                tether::io::makeId(idBase, localId), name, desc, group, VT::Bool,
                [this, getter](void* dest) {
                    uint8_t v = 0;
                    if (hasResults_.load(std::memory_order_acquire)) {
                        std::lock_guard lock(mutex_);
                        v = getter() ? 1 : 0;
                    }
                    std::memcpy(dest, &v, 1);
                }
            });
        };

        // === 1. Machine Limit Checker ===
        addU32(0x0101, "ml.violation_count", "Machine limit violation count",
               [&] { return results_.machineLimit.violationCount; });
        addU32(0x0102, "ml.error_count", "Machine limit error count",
               [&] { return results_.machineLimit.errorCount; });
        addF64(0x0103, "ml.safety_score", "Machine safety score (0-100)",
               [&] { return results_.machineLimit.safetyScore; });

        // === 2. Curvature Analyzer ===
        addF64(0x0201, "curv.max_curvature", "Maximum curvature (1/mm)",
               [&] { return results_.curvature.maxCurvature; });
        addF64(0x0202, "curv.min_radius", "Minimum radius of curvature (mm)",
               [&] { return results_.curvature.minRadius; });
        addF64(0x0203, "curv.avg_curvature", "Average curvature (1/mm)",
               [&] { return results_.curvature.avgCurvature; });
        addU32(0x0204, "curv.sharp_turns", "Sharp turn count",
               [&] { return results_.curvature.sharpTurnCount; });
        addF64(0x0205, "curv.smoothness", "Smoothness score (0-100)",
               [&] { return results_.curvature.smoothnessScore; });
        addU32(0x0206, "curv.corner_count", "Corner count",
               [&] { return results_.curvature.cornerCount; });
        addF64(0x0207, "curv.cornering_efficiency", "Cornering efficiency (0-100)",
               [&] { return results_.curvature.corneringEfficiencyScore; });

        // === 3. Arc Analyzer ===
        addU32(0x0301, "arc.candidate_count", "Arc fitting candidate count",
               [&] { return static_cast<uint32_t>(results_.arc.candidates.size()); });
        addU32(0x0302, "arc.arc_count", "Existing arc (G2/G3) count",
               [&] { return results_.arc.arcCount; });
        addF64(0x0303, "arc.avg_radius", "Average arc radius (mm)",
               [&] { return results_.arc.avgRadius; });
        addF64(0x0304, "arc.min_radius", "Minimum arc radius (mm)",
               [&] { return results_.arc.minRadius; });
        addF64(0x0305, "arc.total_length", "Total arc length (mm)",
               [&] { return results_.arc.totalArcLength; });
        addU32(0x0306, "arc.issue_count", "Arc quality issue count",
               [&] { return results_.arc.issueCount; });
        addF64(0x0307, "arc.quality_score", "Arc quality score (0-100)",
               [&] { return results_.arc.qualityScore; });
        addF64(0x0308, "arc.arc_percentage", "Arc percentage of total path",
               [&] { return results_.arc.arcPercentage; });

        // === 4. Modal State Analyzer ===
        addU32(0x0401, "modal.change_count", "Total modal state changes",
               [&] { return static_cast<uint32_t>(results_.modal.changes.size()); });
        addU32(0x0402, "modal.redundant_count", "Redundant modal commands",
               [&] { return results_.modal.redundantCommands; });
        addF64(0x0403, "modal.modal_score", "Modal efficiency score (0-100)",
               [&] { return results_.modal.modalScore; });
        addU32(0x0404, "modal.transition_count", "Modal transition count",
               [&] { return results_.modal.transitionCount; });
        addF64(0x0405, "modal.stability_score", "Modal stability score (0-100)",
               [&] { return results_.modal.stabilityScore; });
        addBool(0x0406, "modal.properly_reset", "Modal states properly reset at end",
                [&] { return results_.modal.isProperlyReset; });

        // === 5. Path Topology Detector ===
        addU32(0x0501, "topo.loop_count", "Toolpath loop count",
               [&] { return results_.topology.loopCount; });
        addU32(0x0502, "topo.intersection_count", "Self-intersection count",
               [&] { return results_.topology.intersectionCount; });
        addBool(0x0503, "topo.has_cutting_x", "Has cutting self-intersections",
                [&] { return results_.topology.hasCuttingIntersections; });
        addU32(0x0504, "topo.overlap_count", "Overlap count",
               [&] { return results_.topology.overlapCount; });
        addF64(0x0505, "topo.overlap_area", "Total overlap area (mm²)",
               [&] { return results_.topology.totalOverlapArea; });
        addBool(0x0506, "topo.is_symmetric", "Toolpath is symmetric",
                [&] { return results_.topology.isSymmetric; });
        addF64(0x0507, "topo.symmetry_score", "Symmetry score (0-100)",
               [&] { return results_.topology.symmetryScore; });

        // === 6. Toolpath Efficiency Analyzer ===
        addF64(0x0601, "eff.cutting_distance", "Cutting distance (mm)",
               [&] { return results_.efficiency.cuttingDistance; });
        addF64(0x0602, "eff.travel_distance", "Travel distance (mm)",
               [&] { return results_.efficiency.travelDistance; });
        addF64(0x0603, "eff.total_distance", "Total distance (mm)",
               [&] { return results_.efficiency.totalDistance; });
        addF64(0x0604, "eff.cutting_pct", "Cutting percentage",
               [&] { return results_.efficiency.cuttingPercentage; });
        addF64(0x0605, "eff.engagement_ratio", "Engagement ratio",
               [&] { return results_.efficiency.engagementRatio; });
        addF64(0x0606, "eff.efficiency_score", "Efficiency score (0-100)",
               [&] { return results_.efficiency.efficiencyScore; });
        addF64(0x0607, "eff.air_cutting_pct", "Air cutting percentage",
               [&] { return results_.efficiency.airCuttingPercentage; });
        addF64(0x0608, "eff.rapid_pct", "Rapid travel percentage",
               [&] { return results_.efficiency.rapidPercentage; });

        // === 7. Retraction Analyzer ===
        addU32(0x0701, "ret.count", "Retraction count",
               [&] { return results_.retraction.count; });
        addF64(0x0702, "ret.avg_distance", "Average retraction distance (mm)",
               [&] { return results_.retraction.avgDistance; });
        addF64(0x0703, "ret.max_distance", "Max retraction distance (mm)",
               [&] { return results_.retraction.maxDistance; });
        addF64(0x0704, "ret.total_time", "Total retraction time (s)",
               [&] { return results_.retraction.totalRetractionTime; });
        addU32(0x0705, "ret.zhop_count", "Z-hop count",
               [&] { return results_.retraction.zHopCount; });
        addF64(0x0706, "ret.optimization_score", "Optimization score (0-100)",
               [&] { return results_.retraction.optimizationScore; });
        addF64(0x0707, "ret.frequency_score", "Frequency score (0-100)",
               [&] { return results_.retraction.frequencyScore; });
        addF64(0x0708, "ret.recommended_distance", "Recommended retraction distance (mm)",
               [&] { return results_.retraction.recommendedDistance; });

        // === 8. Acceleration Profile Analyzer ===
        addF64(0x0801, "accel.avg", "Average acceleration (mm/s²)",
               [&] { return results_.accel.avgAcceleration; });
        addF64(0x0802, "accel.max", "Max acceleration (mm/s²)",
               [&] { return results_.accel.maxAcceleration; });
        addU32(0x0803, "accel.jerk_count", "Jerk event count",
               [&] { return results_.accel.jerkCount; });
        addF64(0x0804, "accel.smoothness", "Smoothness score (0-100)",
               [&] { return results_.accel.smoothnessScore; });
        addF64(0x0805, "accel.limited_time", "Acceleration-limited time (s)",
               [&] { return results_.accel.limitedTime; });
        addF64(0x0806, "accel.overhead_pct", "Acceleration overhead percentage",
               [&] { return results_.accel.overheadPercentage; });
        addU32(0x0807, "accel.direction_changes", "Direction change count",
               [&] { return results_.accel.directionChanges; });

        // === 9. Coordinate System Analyzer ===
        addU32(0x0901, "coord.offset_count", "Work offset count",
               [&] { return static_cast<uint32_t>(results_.coord.offsets.size()); });
        addU32(0x0902, "coord.rotation_count", "Rotation event count",
               [&] { return results_.coord.rotationEventCount; });
        addF64(0x0903, "coord.max_rotation", "Max rotation angle (degrees)",
               [&] { return results_.coord.maxRotationAngle; });
        addBool(0x0904, "coord.rotation_active_end", "Rotation active at end",
                [&] { return results_.coord.activeRotationAtEnd; });
        addU32(0x0905, "coord.scale_count", "Scale event count",
               [&] { return results_.coord.scaleEventCount; });
        addBool(0x0906, "coord.scale_active_end", "Scale active at end",
                [&] { return results_.coord.scaleActiveAtEnd; });
        addU32(0x0907, "coord.wcs_count", "WCS count",
               [&] { return results_.coord.wcsCount; });

        // === 10. Path Continuity Checker ===
        addU32(0x0A01, "cont.issue_count", "Continuity issue count",
               [&] { return results_.continuity.issueCount; });
        addU32(0x0A02, "cont.high_severity", "High severity issue count",
               [&] { return results_.continuity.highSeverityCount; });
        addF64(0x0A03, "cont.total_gap", "Total gap distance (mm)",
               [&] { return results_.continuity.totalGapDistance; });
        addF64(0x0A04, "cont.continuity_score", "Continuity score (0-100)",
               [&] { return results_.continuity.continuityScore; });
        addBool(0x0A05, "cont.is_continuous", "Toolpath is continuous",
                [&] { return results_.continuity.isContinuous; });
        addU32(0x0A06, "cont.layer_count", "Layer count",
               [&] { return results_.continuity.layerCount; });
        addU32(0x0A07, "cont.total_gaps", "Total gaps across all layers",
               [&] { return results_.continuity.totalGaps; });
    }

    // --- Access to analyzers for configuration ---
    MachineLimitChecker& machineLimitChecker() { return *analyzers_.machineLimit; }
    CurvatureAnalyzer& curvatureAnalyzer() { return *analyzers_.curvature; }
    ArcAnalyzer& arcAnalyzer() { return *analyzers_.arc; }
    PathTopologyDetector& topologyDetector() { return *analyzers_.topology; }
    ToolpathEfficiencyAnalyzer& efficiencyAnalyzer() { return *analyzers_.efficiency; }
    RetractionAnalyzer& retractionAnalyzer() { return *analyzers_.retraction; }
    AccelerationProfileAnalyzer& accelAnalyzer() { return *analyzers_.accel; }
    PathContinuityChecker& continuityChecker() { return *analyzers_.continuity; }

private:
    struct AnalyzerSet {
        std::unique_ptr<MachineLimitChecker> machineLimit;
        std::unique_ptr<CurvatureAnalyzer> curvature;
        std::unique_ptr<ArcAnalyzer> arc;
        std::unique_ptr<ModalStateAnalyzer> modal;
        std::unique_ptr<PathTopologyDetector> topology;
        std::unique_ptr<ToolpathEfficiencyAnalyzer> efficiency;
        std::unique_ptr<RetractionAnalyzer> retraction;
        std::unique_ptr<AccelerationProfileAnalyzer> accel;
        std::unique_ptr<CoordinateSystemAnalyzer> coord;
        std::unique_ptr<PathContinuityChecker> continuity;
    };

    struct ResultSet {
        MachineLimitResult machineLimit;
        CurvatureResult curvature;
        ArcAnalysisResult arc;
        ModalStateResult modal;
        PathTopologyResult topology;
        ToolpathEfficiencyResult efficiency;
        RetractionResult retraction;
        AccelerationProfileResult accel;
        CoordinateSystemResult coord;
        PathContinuityResult continuity;
    };

    AnalyzerSet analyzers_;
    ResultSet results_;
    std::atomic<bool> hasResults_{false};
    mutable std::mutex mutex_;
};

} // namespace tether::analysis
