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

//=============================================================================
// BatchExporter::exportEvaluationData
//=============================================================================

void BatchExporter::exportEvaluationData(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual,
    const tether::motion::replanner::QuantitativeEvaluation& quant,
    const tether::motion::replanner::SpectralEvaluation& spectral,
    const tether::motion::replanner::QualitativeEvaluation& qual,
    const SvgConfig& svgConfig) {

    //--- Export quantitative metrics as CSV ---
    {
        std::string fn = makeFilename("evaluation_quantitative", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Metric,Value,Unit\n";
            f << "SampleCount," << quant.sampleCount << "\n";
            f << "PathLength," << quant.pathLength << ",mm\n";
            f << "Duration," << quant.duration << ",s\n";
            // Integrals
            f << "IAE_s," << quant.integrals.iae_s << ",mm2\n";
            f << "ISE_s," << quant.integrals.ise_s << ",mm3\n";
            f << "ITAE_t," << quant.integrals.itae_t << ",mm*s\n";
            f << "ITSE_t," << quant.integrals.itse_t << ",mm2*s\n";
            f << "IAE_lag," << quant.integrals.iae_lag << ",mm2\n";
            f << "ISE_lag," << quant.integrals.ise_lag << ",mm3\n";
            // Norms
            f << "L1_contour," << quant.norms.l1_contour << ",mm2\n";
            f << "L2_contour," << quant.norms.l2_contour << ",mm^1.5\n";
            f << "Linf_contour," << quant.norms.linf_contour << ",mm\n";
            f << "L1_lag," << quant.norms.l1_lag << ",mm2\n";
            f << "L2_lag," << quant.norms.l2_lag << ",mm^1.5\n";
            f << "Linf_lag," << quant.norms.linf_lag << ",mm\n";
            f << "L1_combined," << quant.norms.l1_combined << ",mm2\n";
            f << "L2_combined," << quant.norms.l2_combined << ",mm^1.5\n";
            f << "Linf_combined," << quant.norms.linf_combined << ",mm\n";
            // Shape
            f << "Hausdorff," << quant.shape.hausdorff << ",mm\n";
            f << "Frechet," << quant.shape.frechet << ",mm\n";
            f << "DTW," << quant.shape.dtw << ",mm\n";
            f << "PathLengthRatio," << quant.shape.pathLengthRatio << "\n";
            f << "CurvatureErrorMax," << quant.shape.curvatureErrorMax << ",1/mm\n";
            f << "CurvatureErrorRms," << quant.shape.curvatureErrorRms << ",1/mm\n";
            // Kinematic
            f << "VelocityTrackingRms," << quant.kinematic.velocityTrackingRms << ",mm/s\n";
            f << "VelocityTrackingMax," << quant.kinematic.velocityTrackingMax << ",mm/s\n";
            f << "AccelTrackingRms," << quant.kinematic.accelTrackingRms << ",mm/s2\n";
            f << "AccelTrackingMax," << quant.kinematic.accelTrackingMax << ",mm/s2\n";
            f << "JerkActualMax," << quant.kinematic.jerkActualMax << ",mm/s3\n";
            f << "JerkActualRms," << quant.kinematic.jerkActualRms << ",mm/s3\n";
            f << "SmoothnessIndex," << quant.kinematic.smoothnessIndex << ",mm2/s5\n";
            // Surface finish
            f << "Ra," << quant.surface.ra << ",um\n";
            f << "Rq," << quant.surface.rq << ",um\n";
            f << "Rz," << quant.surface.rz << ",um\n";
            f << "PeakCount," << quant.surface.peakCount << "\n";
            // Following error
            f << "MaxFollowingError," << quant.following.maxFollowingError << ",mm\n";
            f << "MeanFollowingError," << quant.following.meanFollowingError << ",mm\n";
            f << "SettlingDistance," << quant.following.settlingDistance << ",mm\n";
            f << "CrossCorrelationPeak," << quant.following.crossCorrelationPeak << "\n";
            f << "CrossCorrelationLag," << quant.following.crossCorrelationLag << ",s\n";
            // Stats
            f << "ContourMaxError," << quant.contourStats.maxError << ",mm\n";
            f << "ContourMeanError," << quant.contourStats.meanError << ",mm\n";
            f << "ContourRmsError," << quant.contourStats.rmsError << ",mm\n";
            f << "ContourP95," << quant.contourStats.p95Error << ",mm\n";
            f << "ContourP99," << quant.contourStats.p99Error << ",mm\n";
            f << "LagMaxError," << quant.lagStats.maxError << ",mm\n";
            f << "LagMeanError," << quant.lagStats.meanError << ",mm\n";
        }
    }

    //--- Export qualitative metrics as CSV ---
    {
        std::string fn = makeFilename("evaluation_qualitative", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Aspect,Grade,Score,Description\n";
            auto writeAssessment = [&](const std::string& name,
                                       const tether::motion::replanner::QualitativeAssessment& a) {
                f << name << ","
                  << tether::motion::replanner::gradeToString(a.grade) << ","
                  << a.score << ","
                  << "\"" << a.description << "\"\n";
            };
            writeAssessment("PathFidelity", qual.pathFidelity);
            writeAssessment("SurfaceFinish", qual.surfaceFinish);
            writeAssessment("TimingFidelity", qual.timingFidelity);
            writeAssessment("Smoothness", qual.smoothness);
            writeAssessment("OscillationSeverity", qual.oscillationSeverity);
            writeAssessment("CornerPreservation", qual.cornerPreservation);
            writeAssessment("Overall", qual.overall);

            f << "\nDiagnostic Messages:\n";
            for (const auto& msg : qual.diagnosticMessages) {
                f << "\"" << msg << "\"\n";
            }
        }
    }

    //--- Export spectral data as CSV ---
    {
        auto writeSpectrum = [&](const std::string& name,
                                 const tether::motion::replanner::ComponentSpectrum& s) {
            std::string fn = makeFilename(name, "csv");
            std::ofstream f(fn);
            if (!f.is_open()) return;
            f << "frequency,magnitude,phase,psd\n";
            for (std::size_t i = 0; i < s.frequencies.size(); ++i) {
                f << s.frequencies[i] << ","
                  << s.magnitudes[i] << ","
                  << s.phases[i] << ","
                  << s.powerSpectralDensity[i] << "\n";
            }
        };

        writeSpectrum("spectral_spatial_contour", spectral.spatialContour);
        writeSpectrum("spectral_spatial_lag", spectral.spatialLag);
        writeSpectrum("spectral_temporal_contour", spectral.temporalContour);
        writeSpectrum("spectral_temporal_lag", spectral.temporalLag);
    }

    //--- Export SVG plots ---
    {
        SvgExporter svgExporter(svgConfig);
        auto svgFiles = svgExporter.exportAllPlots(
            outputDir_,
            prefix_.empty() ? "eval" : prefix_,
            desired, actual, quant, spectral);
        for (const auto& f : svgFiles) {
            // Already in output dir, just track
            auto it = std::find(generatedFiles_.begin(), generatedFiles_.end(), f);
            if (it == generatedFiles_.end()) {
                generatedFiles_.push_back(f);
            }
        }

        // Also export a dashboard
        std::string dashPath = outputDir_ + "/" +
            (prefix_.empty() ? "eval" : prefix_) + "_dashboard.svg";
        if (svgExporter.exportDashboard(dashPath, desired, actual, quant, spectral)) {
            auto it = std::find(generatedFiles_.begin(), generatedFiles_.end(), dashPath);
            if (it == generatedFiles_.end()) {
                generatedFiles_.push_back(dashPath);
            }
        }
    }
}

//=============================================================================
// BatchExporter::exportKdeData
//=============================================================================

void BatchExporter::exportKdeData(
    const tether::motion::replanner::KdeEvaluation& kde,
    const SvgConfig& svgConfig) {

    using namespace tether::motion::replanner;

    //--- Export raw sample pairs as CSV ---
    {
        std::string fn = makeFilename("kde_samples", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "index,derivative,deviation,arcLength,time\n";
            for (std::size_t i = 0; i < kde.derivatives.size(); ++i) {
                f << i << ","
                  << kde.derivatives[i] << ","
                  << kde.deviations[i] << ","
                  << (i < kde.arcLengths.size() ? kde.arcLengths[i] : 0.0) << ","
                  << (i < kde.times.size() ? kde.times[i] : 0.0) << "\n";
            }
        }
    }

    //--- Export KDE density grid as CSV (matrix) ---
    {
        std::string fn = makeFilename("kde_density_grid", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            // Header row: Y bin values (descending so top of matrix = high Y)
            f << "xBin\\\\yBin";
            for (int iy = static_cast<int>(kde.grid.yBins.size()) - 1; iy >= 0; --iy) {
                f << "," << kde.grid.yBins[static_cast<std::size_t>(iy)];
            }
            f << "\n";
            // Each row: X bin value, then densities for each Y bin (descending)
            for (std::size_t ix = 0; ix < kde.grid.xBins.size(); ++ix) {
                f << kde.grid.xBins[ix];
                for (int iy = static_cast<int>(kde.grid.yBins.size()) - 1; iy >= 0; --iy) {
                    f << "," << kde.grid.at(ix, static_cast<std::size_t>(iy));
                }
                f << "\n";
            }
        }
    }

    //--- Export KDE density grid as JSON ---
    {
        std::string fn = makeFilename("kde_density_grid", "json");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "{\n";
            f << "  \"derivativeAxis\": \"" << toString(kde.derivativeAxis) << "\",\n";
            f << "  \"deviationAxis\": \"" << toString(kde.deviationAxis) << "\",\n";
            f << "  \"kernel\": \"" << toString(kde.kernel) << "\",\n";
            f << "  \"bandwidthMethod\": \"" << toString(kde.bandwidthMethod) << "\",\n";
            f << "  \"bandwidthX\": " << kde.grid.bandwidthX << ",\n";
            f << "  \"bandwidthY\": " << kde.grid.bandwidthY << ",\n";
            f << "  \"sampleCount\": " << kde.grid.sampleCount << ",\n";
            f << "  \"gridSizeX\": " << kde.grid.xBins.size() << ",\n";
            f << "  \"gridSizeY\": " << kde.grid.yBins.size() << ",\n";
            f << "  \"xBins\": [";
            for (std::size_t i = 0; i < kde.grid.xBins.size(); ++i) {
                if (i > 0) f << ",";
                f << kde.grid.xBins[i];
            }
            f << "],\n";
            f << "  \"yBins\": [";
            for (std::size_t i = 0; i < kde.grid.yBins.size(); ++i) {
                if (i > 0) f << ",";
                f << kde.grid.yBins[i];
            }
            f << "],\n";
            f << "  \"density\": [";
            for (std::size_t i = 0; i < kde.grid.density.size(); ++i) {
                if (i > 0) f << ",";
                f << kde.grid.density[i];
            }
            f << "]\n";
            f << "}\n";
        }
    }

    //--- Export conditional statistics as CSV ---
    {
        std::string fn = makeFilename("kde_conditional", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "xValue,mass,meanY,stdY,modeY,medianY,p05Y,p25Y,p75Y,p95Y,valid\n";
            for (const auto& cs : kde.conditional) {
                f << cs.xValue << ","
                  << cs.mass << ","
                  << cs.meanY << ","
                  << cs.stdY << ","
                  << cs.modeY << ","
                  << cs.medianY << ","
                  << cs.p05Y << ","
                  << cs.p25Y << ","
                  << cs.p75Y << ","
                  << cs.p95Y << ","
                  << (cs.valid ? 1 : 0) << "\n";
            }
        }
    }

    //--- Export marginal statistics as CSV ---
    {
        std::string fn = makeFilename("kde_marginals", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Axis,Mean,StdDev,Skewness,Kurtosis,Min,Max,Median,P05,P25,P75,P95,Mode\n";
            auto writeMarg = [&](const std::string& name, const MarginalStats& m) {
                f << name << ","
                  << m.mean << ","
                  << m.stdDev << ","
                  << m.skewness << ","
                  << m.kurtosis << ","
                  << m.min << ","
                  << m.max << ","
                  << m.median << ","
                  << m.p05 << ","
                  << m.p25 << ","
                  << m.p75 << ","
                  << m.p95 << ","
                  << m.mode << "\n";
            };
            writeMarg(toString(kde.derivativeAxis), kde.derivativeMarginal);
            writeMarg(toString(kde.deviationAxis), kde.deviationMarginal);
        }
    }

    //--- Export dependence metrics as CSV ---
    {
        std::string fn = makeFilename("kde_dependence", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Metric,Value\n";
            f << "PearsonCorrelation," << kde.pearsonCorrelation << "\n";
            f << "SpearmanCorrelation," << kde.spearmanCorrelation << "\n";
            f << "KendallTau," << kde.kendallTau << "\n";
            f << "MutualInformation_bits," << kde.mutualInformation << "\n";
            f << "CorrelationRatio," << kde.correlationRatio << "\n";
            f << "DistanceCorrelation," << kde.distanceCorrelation << "\n";
            f << "DependenceIndex," << kde.dependenceIndex << "\n";
            f << "JointEntropy_bits," << kde.jointEntropy << "\n";
            f << "ConditionalEntropy_bits," << kde.conditionalEntropy << "\n";
            f << "NormalizedMutualInformation," << kde.normalizedMutualInfo << "\n";
            f << "ModeDerivative," << kde.modeDerivative << "\n";
            f << "ModeDeviation," << kde.modeDeviation << "\n";
            f << "MaxDensity," << kde.maxDensity << "\n";
        }
    }

    //--- Export threshold analysis as CSV ---
    {
        std::string fn = makeFilename("kde_thresholds", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Tolerance,DerivativeValue,Probability,Found,Description\n";
            for (const auto& t : kde.thresholds) {
                f << t.tolerance << ","
                  << t.derivativeValue << ","
                  << t.probability << ","
                  << (t.found ? 1 : 0) << ","
                  << "\"" << t.description << "\"\n";
            }
        }
    }

    //--- Export tail risk metrics as CSV ---
    {
        std::string fn = makeFilename("kde_tail_risk", "csv");
        std::ofstream f(fn);
        if (f.is_open()) {
            f << "Metric,Value,Unit\n";
            f << "TailFraction," << kde.tailFraction << "\n";
            f << "VaR95," << kde.var95 << ",mm\n";
            f << "ExpectedTailDeviation," << kde.expectedTailDeviation << ",mm\n";
            f << "CVaR95," << kde.conditionalVar95 << ",mm\n";
        }
    }

    //--- Export SVG plots ---
    {
        SvgExporter svgExporter(svgConfig);
        auto svgFiles = svgExporter.exportAllKdePlots(
            outputDir_,
            prefix_.empty() ? "kde" : prefix_,
            kde);
        for (const auto& f : svgFiles) {
            auto it = std::find(generatedFiles_.begin(), generatedFiles_.end(), f);
            if (it == generatedFiles_.end()) {
                generatedFiles_.push_back(f);
            }
        }
    }
}

} // namespace MotionReplanner
