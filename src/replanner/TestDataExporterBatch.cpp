/**
 * @file TestDataExporterBatch.cpp
 * @brief Batch exporter and report generator implementation
 */

#include "tether/motion_replanner/TestDataExporter.hpp"
#include <filesystem>
#include <algorithm>

namespace MotionReplanner {

//=============================================================================
// BatchExporter Implementation
//=============================================================================

BatchExporter::BatchExporter(const std::string& outputDir, const ExportConfig& config)
    : outputDir_(outputDir), config_(config) {
    std::filesystem::create_directories(outputDir);
}

std::string BatchExporter::makeFilename(const std::string& baseName, const std::string& extension) {
    std::string filename = outputDir_ + "/";
    if (!prefix_.empty()) {
        filename += prefix_ + "_";
    }
    filename += baseName + "." + extension;
    generatedFiles_.push_back(filename);
    return filename;
}

void BatchExporter::exportReplannerData(const MotionReplanner& replanner) {
    TrajectoryExporter exporter(config_);
    
    auto stats = replanner.getOverallStatistics();
    exporter.exportErrorStatistics(makeFilename("error_statistics", "json"), stats);
    
    auto errors = replanner.getAllErrors();
    exporter.exportTrackingErrors(makeFilename("tracking_errors", "csv"), errors);
    
    auto segments = replanner.getSegmentPerformance();
    exporter.exportSegmentPerformance(makeFilename("segment_performance", "csv"), segments);
    
    auto suggestions = replanner.getParameterSuggestions();
    exporter.exportSuggestions(makeFilename("suggestions", "csv"), suggestions);
}

void BatchExporter::exportHeatmapData(const HeatmapBuilder& builder) {
    HeatmapExporter exporter(config_);
    
    // Export 2D heatmaps for each plane
    const auto& xyHeatmap = builder.getXYHeatmap();
    exporter.exportHeatmap2D(makeFilename("heatmap_xy", "csv"), xyHeatmap);
    
    const auto& xzHeatmap = builder.getXZHeatmap();
    exporter.exportHeatmap2D(makeFilename("heatmap_xz", "csv"), xzHeatmap);
    
    const auto& yzHeatmap = builder.getYZHeatmap();
    exporter.exportHeatmap2D(makeFilename("heatmap_yz", "csv"), yzHeatmap);
    
    // Export 3D heatmap
    const auto& heatmap3D = builder.get3DHeatmap();
    exporter.exportHeatmap3D(makeFilename("heatmap_3d", "csv"), heatmap3D);
    
    // Export per-axis 1D heatmaps
    const auto& axisHeatmaps = builder.getAxisHeatmaps();
    for (int axis = 0; axis < 3; ++axis) {
        std::string name = "heatmap_axis_" + std::string(1, "XYZ"[axis]);
        exporter.exportHeatmap1D(makeFilename(name, "csv"), axisHeatmaps[axis]);
    }
}

void BatchExporter::exportTestResults(const std::vector<TestResult>& results) {
    TestResultExporter exporter(config_);
    
    for (size_t i = 0; i < results.size(); ++i) {
        std::string name = results[i].testName;
        // Sanitize filename
        std::replace(name.begin(), name.end(), ' ', '_');
        std::replace(name.begin(), name.end(), '/', '_');
        
        std::string ext = (config_.format == ExportFormat::JSON) ? "json" : "csv";
        exporter.exportTestResult(makeFilename("test_" + name, ext), results[i]);
    }
}

void BatchExporter::exportIdentificationData(const SystemIdentifier& identifier,
    const DelayIdentificationResult& delay,
    const FrictionIdentificationResult& friction,
    const PIDTuningAssessment& pid) {
    
    TestResultExporter exporter(config_);
    
    exporter.exportDelayIdentification(makeFilename("delay_identification", "json"), delay);
    exporter.exportFrictionModel(makeFilename("friction_model", "json"), friction);
    exporter.exportPIDAssessment(makeFilename("pid_assessment", "json"), pid);
}

bool BatchExporter::generateManifest(const std::string& filename) {
    std::ofstream file(outputDir_ + "/" + filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    json.keyValue("timestamp", DataExporter(config_).getCurrentTimestamp());
    json.keyValue("fileCount", static_cast<int>(generatedFiles_.size()));
    
    json.key("files");
    json.beginArray();
    for (const auto& f : generatedFiles_) {
        json.value(f);
    }
    json.endArray();
    
    json.endObject();
    
    file << json.str();
    return file.good();
}

//=============================================================================
// ReportGenerator Implementation
//=============================================================================

ReportGenerator::ReportGenerator() {}

void ReportGenerator::addSection(const ReportSection& section) {
    sections_.push_back(section);
}

void ReportGenerator::addSummary(const std::vector<TestResult>& results) {
    ReportSection section;
    section.title = "Test Summary";
    
    std::ostringstream ss;
    ss << "## Overview\n\n";
    ss << "Total tests: " << results.size() << "\n\n";
    
    int passed = 0, failed = 0;
    for (const auto& r : results) {
        if (r.passed) passed++;
        else failed++;
    }
    
    ss << "- Passed: " << passed << "\n";
    ss << "- Failed: " << failed << "\n\n";
    
    ss << "## Results\n\n";
    ss << "| Test Name | Type | Max Error | Mean Error | Status |\n";
    ss << "|-----------|------|-----------|------------|--------|\n";
    
    for (const auto& r : results) {
        ss << "| " << r.testName
           << " | " << r.testType
           << " | " << std::fixed << std::setprecision(4) << r.positionError.maxError
           << " | " << r.positionError.meanError
           << " | " << (r.passed ? "✓ Pass" : "✗ Fail") << " |\n";
    }
    
    section.content = ss.str();
    sections_.push_back(section);
}

void ReportGenerator::addErrorStatistics(const ErrorStatistics& stats) {
    ReportSection section;
    section.title = "Error Statistics";
    
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    
    ss << "## Position Error Analysis\n\n";
    ss << "| Metric | Value |\n";
    ss << "|--------|-------|\n";
    ss << "| Sample Count | " << stats.sampleCount << " |\n";
    ss << "| Min Error | " << stats.minError << " mm |\n";
    ss << "| Max Error | " << stats.maxError << " mm |\n";
    ss << "| Mean Error | " << stats.meanError << " mm |\n";
    ss << "| Geometric Mean | " << stats.geometricMean << " mm |\n";
    ss << "| Std Dev | " << stats.stdDev << " mm |\n";
    ss << "| RMS Error | " << stats.rmsError << " mm |\n\n";
    
    ss << "## Percentiles\n\n";
    ss << "| Percentile | Value |\n";
    ss << "|------------|-------|\n";
    ss << "| 95th | " << stats.p95Error << " mm |\n";
    ss << "| 99th | " << stats.p99Error << " mm |\n\n";
    
    if (stats.cornerCount > 0) {
        ss << "## Corner Analysis\n\n";
        ss << "| Metric | Value |\n";
        ss << "|--------|-------|\n";
        ss << "| Corner Count | " << stats.cornerCount << " |\n";
        ss << "| Max Corner Error | " << stats.maxCornerError << " mm |\n";
        ss << "| Mean Corner Error | " << stats.meanCornerError << " mm |\n";
    }
    
    section.content = ss.str();
    sections_.push_back(section);
}

void ReportGenerator::addSystemIdentification(const DelayIdentificationResult& delay,
    const FrictionIdentificationResult& friction,
    const PIDTuningAssessment& pid) {
    
    ReportSection section;
    section.title = "System Identification";
    
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    
    ss << "## Transport Delay\n\n";
    ss << "- Delay: " << delay.transportDelay * 1000 << " ms\n";
    ss << "- Confidence: " << delay.delayConfidence * 100 << "%\n";
    ss << "- Rise Time: " << delay.riseTime * 1000 << " ms\n";
    ss << "- Settling Time: " << delay.settlingTime * 1000 << " ms\n";
    ss << "- Overshoot: " << delay.overshoot << "%\n\n";
    
    ss << "## Friction Model\n\n";
    ss << "Best fit: **" << friction.bestModel.modelName() << "**\n\n";
    ss << "| Parameter | Value |\n";
    ss << "|-----------|-------|\n";
    ss << "| Coulomb Force | " << friction.bestModel.coulombForce << " N |\n";
    ss << "| Static Friction | " << friction.bestModel.staticFriction << " N |\n";
    ss << "| Viscous Coeff | " << friction.bestModel.viscousCoeff << " N/(m/s) |\n";
    ss << "| R² | " << friction.bestModel.rSquared << " |\n\n";
    
    ss << "Direction symmetry: " << (friction.isSymmetric ? "Symmetric" : "Asymmetric") << "\n\n";
    
    ss << "## PID Tuning Assessment\n\n";
    ss << "Overall Score: **" << pid.overallScore << "/100**\n\n";
    ss << "| Metric | Score |\n";
    ss << "|--------|-------|\n";
    ss << "| Stability | " << pid.stabilityScore << " |\n";
    ss << "| Response | " << pid.responseScore << " |\n";
    ss << "| Accuracy | " << pid.accuracyScore << " |\n\n";
    
    if (!pid.issues.empty()) {
        ss << "### Issues\n\n";
        for (const auto& issue : pid.issues) {
            ss << "- " << issue << "\n";
        }
        ss << "\n";
    }
    
    if (!pid.recommendations.empty()) {
        ss << "### Recommendations\n\n";
        for (const auto& rec : pid.recommendations) {
            ss << "- " << rec << "\n";
        }
        ss << "\n";
    }
    
    section.content = ss.str();
    sections_.push_back(section);
}

void ReportGenerator::addRecommendations(const std::vector<ParameterSuggestion>& suggestions) {
    ReportSection section;
    section.title = "Parameter Recommendations";
    
    std::ostringstream ss;
    ss << "## Suggested Changes\n\n";
    
    ss << "| Segment | Current Feed | Suggested Feed | Confidence | Reason |\n";
    ss << "|---------|--------------|----------------|------------|--------|\n";
    
    for (const auto& sug : suggestions) {
        ss << "| " << sug.segmentIndex
           << " | " << std::fixed << std::setprecision(2) << sug.currentFeedRate
           << " | " << sug.suggestedFeedRate
           << " | " << sug.confidenceScore * 100 << "%"
           << " | " << sug.reason << " |\n";
    }
    
    section.content = ss.str();
    sections_.push_back(section);
}

bool ReportGenerator::exportMarkdown(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "# Motion Test Report\n\n";
    file << "Generated: " << DataExporter().getCurrentTimestamp() << "\n\n";
    file << "---\n\n";
    
    for (const auto& section : sections_) {
        file << "# " << section.title << "\n\n";
        file << section.content << "\n\n";
        file << "---\n\n";
    }
    
    return file.good();
}

bool ReportGenerator::exportHTML(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "<!DOCTYPE html>\n<html>\n<head>\n";
    file << "<title>Motion Test Report</title>\n";
    file << "<style>\n";
    file << "body { font-family: sans-serif; max-width: 1200px; margin: 0 auto; padding: 20px; }\n";
    file << "table { border-collapse: collapse; width: 100%; margin: 20px 0; }\n";
    file << "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }\n";
    file << "th { background-color: #4CAF50; color: white; }\n";
    file << "tr:nth-child(even) { background-color: #f2f2f2; }\n";
    file << ".pass { color: green; }\n";
    file << ".fail { color: red; }\n";
    file << "</style>\n</head>\n<body>\n";
    
    file << "<h1>Motion Test Report</h1>\n";
    file << "<p>Generated: " << DataExporter().getCurrentTimestamp() << "</p>\n";
    
    // Note: Full HTML generation would convert Markdown to HTML
    // For now, wrap content in <pre> tags
    for (const auto& section : sections_) {
        file << "<h2>" << section.title << "</h2>\n";
        file << "<pre>" << section.content << "</pre>\n";
    }
    
    file << "</body>\n</html>\n";
    
    return file.good();
}

bool ReportGenerator::exportLaTeX(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "\\documentclass{article}\n";
    file << "\\usepackage{booktabs}\n";
    file << "\\usepackage{graphicx}\n";
    file << "\\begin{document}\n\n";
    
    file << "\\title{Motion Test Report}\n";
    file << "\\date{" << DataExporter().getCurrentTimestamp() << "}\n";
    file << "\\maketitle\n\n";
    
    for (const auto& section : sections_) {
        file << "\\section{" << section.title << "}\n";
        // Convert Markdown to LaTeX (simplified)
        file << "\\begin{verbatim}\n" << section.content << "\n\\end{verbatim}\n\n";
    }
    
    file << "\\end{document}\n";
    
    return file.good();
}

} // namespace MotionReplanner
