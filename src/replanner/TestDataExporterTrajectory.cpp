/**
 * @file TestDataExporterTrajectory.cpp
 * @brief Trajectory data exporter implementation
 */

#include "TestDataExporter.hpp"
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// TrajectoryExporter Implementation
//=============================================================================

bool TrajectoryExporter::exportTrajectory(const std::string& filename,
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    if (config_.format == ExportFormat::JSON || config_.format == ExportFormat::JSONPretty) {
        writeJSONTrajectory(file, desired, actual);
    } else {
        writeCSVTrajectory(file, desired, actual);
    }
    
    return file.good();
}

void TrajectoryExporter::writeCSVTrajectory(std::ostream& out,
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) {
    
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        out << "time" << d
            << "desired_x" << d << "desired_y" << d << "desired_z" << d
            << "desired_vx" << d << "desired_vy" << d << "desired_vz" << d
            << "actual_x" << d << "actual_y" << d << "actual_z" << d
            << "actual_vx" << d << "actual_vy" << d << "actual_vz" << d
            << "error_x" << d << "error_y" << d << "error_z" << d
            << "error_magnitude\n";
    }
    
    size_t n = std::min(desired.size(), actual.size());
    for (size_t i = 0; i < n; i += config_.downsampleFactor) {
        const auto& des = desired[i];
        const auto& act = actual[i];
        
        double ex = des.position[0] - act.position[0];
        double ey = des.position[1] - act.position[1];
        double ez = des.position[2] - act.position[2];
        double emag = std::sqrt(ex*ex + ey*ey + ez*ez);
        
        out << formatDouble(des.time) << d
            << formatDouble(des.position[0]) << d
            << formatDouble(des.position[1]) << d
            << formatDouble(des.position[2]) << d
            << formatDouble(des.velocity[0]) << d
            << formatDouble(des.velocity[1]) << d
            << formatDouble(des.velocity[2]) << d
            << formatDouble(act.position[0]) << d
            << formatDouble(act.position[1]) << d
            << formatDouble(act.position[2]) << d
            << formatDouble(act.velocity[0]) << d
            << formatDouble(act.velocity[1]) << d
            << formatDouble(act.velocity[2]) << d
            << formatDouble(ex) << d
            << formatDouble(ey) << d
            << formatDouble(ez) << d
            << formatDouble(emag) << "\n";
    }
}

void TrajectoryExporter::writeJSONTrajectory(std::ostream& out,
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) {
    
    bool pretty = (config_.format == ExportFormat::JSONPretty);
    JSONBuilder json(pretty);
    
    json.beginObject();
    
    if (config_.includeMetadata) {
        json.key("metadata");
        json.beginObject();
        json.keyValue("testName", metadata_.testName);
        json.keyValue("timestamp", getCurrentTimestamp());
        json.keyValue("sampleCount", static_cast<int>(desired.size()));
        json.endObject();
    }
    
    // Desired trajectory
    json.key("desired");
    json.beginObject();
    {
        std::vector<double> time, x, y, z, vx, vy, vz;
        for (const auto& s : desired) {
            time.push_back(s.time);
            x.push_back(s.position[0]);
            y.push_back(s.position[1]);
            z.push_back(s.position[2]);
            vx.push_back(s.velocity[0]);
            vy.push_back(s.velocity[1]);
            vz.push_back(s.velocity[2]);
        }
        json.doubleArray("time", time);
        json.doubleArray("x", x);
        json.doubleArray("y", y);
        json.doubleArray("z", z);
        json.doubleArray("vx", vx);
        json.doubleArray("vy", vy);
        json.doubleArray("vz", vz);
    }
    json.endObject();
    
    // Actual trajectory
    json.key("actual");
    json.beginObject();
    {
        std::vector<double> time, x, y, z, vx, vy, vz;
        for (const auto& s : actual) {
            time.push_back(s.time);
            x.push_back(s.position[0]);
            y.push_back(s.position[1]);
            z.push_back(s.position[2]);
            vx.push_back(s.velocity[0]);
            vy.push_back(s.velocity[1]);
            vz.push_back(s.velocity[2]);
        }
        json.doubleArray("time", time);
        json.doubleArray("x", x);
        json.doubleArray("y", y);
        json.doubleArray("z", z);
        json.doubleArray("vx", vx);
        json.doubleArray("vy", vy);
        json.doubleArray("vz", vz);
    }
    json.endObject();
    
    json.endObject();
    
    out << json.str();
}

bool TrajectoryExporter::exportTrackingErrors(const std::string& filename,
    const std::vector<TrackingError>& errors) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        file << "time" << d
             << "error_x" << d << "error_y" << d << "error_z" << d
             << "combined_error" << d
             << "velocity_error_x" << d
             << "lag_error" << d
             << "contour_error" << d
             << "is_critical_point\n";
    }
    
    for (const auto& e : errors) {
        file << formatDouble(e.timestamp) << d
             << formatDouble(e.positionError[0]) << d
             << formatDouble(e.positionError[1]) << d
             << formatDouble(e.positionError[2]) << d
             << formatDouble(e.combinedPositionError) << d
             << formatDouble(e.velocityError[0]) << d
             << formatDouble(e.lagError) << d
             << formatDouble(e.contourError) << d
             << (e.isCriticalPoint ? "1" : "0") << "\n";
    }
    
    return file.good();
}

bool TrajectoryExporter::exportSegmentPerformance(const std::string& filename,
    const std::vector<SegmentPerformance>& segments) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        file << "segment_index" << d
             << "commanded_feed_rate" << d << "achieved_mean_feed_rate" << d
             << "commanded_accel" << d << "achieved_max_accel" << d
             << "feed_rate_ratio" << d << "accuracy_score" << d
             << "limit_adjustment_needed" << d
             << "overall_score\n";
    }
    
    for (const auto& seg : segments) {
        file << seg.segmentIndex << d
             << formatDouble(seg.commandedFeedRate) << d
             << formatDouble(seg.achievedMeanFeedRate) << d
             << formatDouble(seg.commandedAccel) << d
             << formatDouble(seg.achievedMaxAccel) << d
             << formatDouble(seg.feedRateRatio) << d
             << formatDouble(seg.accuracyScore) << d
             << (seg.limitAdjustmentNeeded ? "1" : "0") << d
             << formatDouble(seg.overallScore) << "\n";
    }
    
    return file.good();
}

bool TrajectoryExporter::exportErrorStatistics(const std::string& filename,
    const ErrorStatistics& stats) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    if (config_.format == ExportFormat::JSON || config_.format == ExportFormat::JSONPretty) {
        JSONBuilder json(config_.format == ExportFormat::JSONPretty);
        
        json.beginObject();
        json.keyValue("sampleCount", static_cast<int>(stats.sampleCount));
        json.keyValue("minError", stats.minError);
        json.keyValue("maxError", stats.maxError);
        json.keyValue("meanError", stats.meanError);
        json.keyValue("geometricMean", stats.geometricMean);
        json.keyValue("stdDev", stats.stdDev);
        json.keyValue("rmsError", stats.rmsError);
        
        json.key("percentiles");
        json.beginObject();
        json.keyValue("p95", stats.p95Error);
        json.keyValue("p99", stats.p99Error);
        json.endObject();
        
        json.key("cornerAnalysis");
        json.beginObject();
        json.keyValue("cornerCount", static_cast<int>(stats.cornerCount));
        json.keyValue("maxCornerError", stats.maxCornerError);
        json.keyValue("meanCornerError", stats.meanCornerError);
        json.endObject();
        
        json.endObject();
        
        file << json.str();
    } else {
        file << "Metric,Value\n"
             << "SampleCount," << stats.sampleCount << "\n"
             << "MinError," << formatDouble(stats.minError) << "\n"
             << "MaxError," << formatDouble(stats.maxError) << "\n"
             << "MeanError," << formatDouble(stats.meanError) << "\n"
             << "GeometricMean," << formatDouble(stats.geometricMean) << "\n"
             << "StdDev," << formatDouble(stats.stdDev) << "\n"
             << "RMSError," << formatDouble(stats.rmsError) << "\n"
             << "P95," << formatDouble(stats.p95Error) << "\n"
             << "P99," << formatDouble(stats.p99Error) << "\n"
             << "CornerCount," << stats.cornerCount << "\n"
             << "MaxCornerError," << formatDouble(stats.maxCornerError) << "\n"
             << "MeanCornerError," << formatDouble(stats.meanCornerError) << "\n";
    }
    
    return file.good();
}

bool TrajectoryExporter::exportSuggestions(const std::string& filename,
    const std::vector<ParameterSuggestion>& suggestions) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    char d = config_.delimiter;
    
    if (config_.includeHeader) {
        file << "segment_id" << d
             << "parameter" << d
             << "current_value" << d << "suggested_value" << d
             << "expected_improvement" << d
             << "confidence" << d
             << "reason\n";
    }
    
    for (const auto& sug : suggestions) {
        file << sug.segmentIndex << d
             << escapeCSV("feedRate") << d
             << formatDouble(sug.currentFeedRate) << d
             << formatDouble(sug.suggestedFeedRate) << d
             << formatDouble(0.0) << d  // expectedImprovement not available
             << formatDouble(sug.confidenceScore) << d
             << escapeCSV(sug.reason) << "\n";
    }
    
    return file.good();
}

} // namespace MotionReplanner
