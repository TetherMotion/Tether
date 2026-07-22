/**
 * @file CSVExporter.cpp
 * @brief CSV export implementation
 */

#include "CSVExporter.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace GCodeExport {

/// Reject filenames containing path traversal or absolute path components.
/// This prevents an attacker who controls the filename from writing to
/// arbitrary locations (e.g. "../../etc/passwd" or "/etc/cron.d/evil").
static bool isSafeFilename(const std::string& filename) {
    if (filename.empty()) return false;
    if (filename[0] == '/' || filename[0] == '\\') return false;
    if (filename.find("..") != std::string::npos) return false;
    if (filename.find('\0') != std::string::npos) return false;
    return true;
}

CSVExporter::CSVExporter(const CSVConfig& config)
    : config_(config) {}

bool CSVExporter::exportToFile(const std::vector<TrajectorySample>& samples, const std::string& filename) {
    if (!isSafeFilename(filename)) return false;
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    exportToStream(samples, file);
    return file.good();
}

void CSVExporter::exportToStream(const std::vector<TrajectorySample>& samples, std::ostream& out) {
    if (config_.includeHeader) {
        writeHeader(out);
    }
    
    if (config_.includeUnits) {
        writeUnitsRow(out);
    }
    
    for (const auto& sample : samples) {
        writeSample(out, sample);
    }
}

void CSVExporter::writeHeader(std::ostream& out) {
    std::vector<std::string> columns;
    
    if (config_.exportTime) {
        columns.push_back("time");
    }
    
    if (config_.exportPathPosition) {
        columns.push_back("path_position");
    }
    
    // Position columns
    if (config_.exportPosition) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                columns.push_back("pos_" + config_.axisNames[i]);
            }
        }
    }
    
    // Velocity columns
    if (config_.exportVelocity) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                columns.push_back("vel_" + config_.axisNames[i]);
            }
        }
    }
    
    // Acceleration columns
    if (config_.exportAcceleration) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                columns.push_back("acc_" + config_.axisNames[i]);
            }
        }
    }
    
    // Jerk columns
    if (config_.exportJerk) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                columns.push_back("jerk_" + config_.axisNames[i]);
            }
        }
    }
    
    // Combined metrics
    if (config_.exportCombinedMetrics) {
        columns.push_back("linear_velocity");
        columns.push_back("linear_acceleration");
        columns.push_back("linear_jerk");
        columns.push_back("curvature");
        columns.push_back("centripetal_accel");
    }
    
    // Segment info
    if (config_.exportSegmentInfo) {
        columns.push_back("segment_index");
        columns.push_back("block_index");
        columns.push_back("motion_type");
    }
    
    // Write header row
    for (size_t i = 0; i < columns.size(); ++i) {
        if (i > 0) out << config_.delimiter;
        out << columns[i];
    }
    out << "\n";
}

void CSVExporter::writeUnitsRow(std::ostream& out) {
    std::vector<std::string> units;
    
    if (config_.exportTime) {
        units.push_back("s");
    }
    
    if (config_.exportPathPosition) {
        units.push_back("mm");
    }
    
    // Position units
    if (config_.exportPosition) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                units.push_back(i < 3 || i >= 6 ? "mm" : "deg");
            }
        }
    }
    
    // Velocity units
    if (config_.exportVelocity) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                units.push_back(i < 3 || i >= 6 ? "mm/s" : "deg/s");
            }
        }
    }
    
    // Acceleration units
    if (config_.exportAcceleration) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                units.push_back(i < 3 || i >= 6 ? "mm/s²" : "deg/s²");
            }
        }
    }
    
    // Jerk units
    if (config_.exportJerk) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                units.push_back(i < 3 || i >= 6 ? "mm/s³" : "deg/s³");
            }
        }
    }
    
    // Combined metrics units
    if (config_.exportCombinedMetrics) {
        units.push_back("mm/s");
        units.push_back("mm/s²");
        units.push_back("mm/s³");
        units.push_back("1/mm");
        units.push_back("mm/s²");
    }
    
    // Segment info units
    if (config_.exportSegmentInfo) {
        units.push_back("-");
        units.push_back("-");
        units.push_back("-");
    }
    
    // Write units row
    for (size_t i = 0; i < units.size(); ++i) {
        if (i > 0) out << config_.delimiter;
        out << units[i];
    }
    out << "\n";
}

void CSVExporter::writeSample(std::ostream& out, const TrajectorySample& sample) {
    bool first = true;
    
    auto writeField = [&](const std::string& value) {
        if (!first) out << config_.delimiter;
        first = false;
        out << value;
    };
    
    if (config_.exportTime) {
        writeField(formatDouble(sample.time, config_.timePrecision));
    }
    
    if (config_.exportPathPosition) {
        writeField(formatDouble(sample.pathPosition, config_.positionPrecision));
    }
    
    // Position
    if (config_.exportPosition) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                writeField(formatDouble(sample.position[i], config_.positionPrecision));
            }
        }
    }
    
    // Velocity
    if (config_.exportVelocity) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                writeField(formatDouble(sample.velocity[i], config_.velocityPrecision));
            }
        }
    }
    
    // Acceleration
    if (config_.exportAcceleration) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                writeField(formatDouble(sample.acceleration[i], config_.accelerationPrecision));
            }
        }
    }
    
    // Jerk
    if (config_.exportJerk) {
        for (size_t i = 0; i < 9; ++i) {
            if (config_.includeAxes[i]) {
                writeField(formatDouble(sample.jerk[i], config_.jerkPrecision));
            }
        }
    }
    
    // Combined metrics
    if (config_.exportCombinedMetrics) {
        writeField(formatDouble(sample.linearVelocity, config_.velocityPrecision));
        writeField(formatDouble(sample.linearAcceleration, config_.accelerationPrecision));
        writeField(formatDouble(sample.linearJerk, config_.jerkPrecision));
        writeField(formatDouble(sample.curvature, 9));  // High precision for curvature
        writeField(formatDouble(sample.centripetalAccel, config_.accelerationPrecision));
    }
    
    // Segment info
    if (config_.exportSegmentInfo) {
        writeField(std::to_string(sample.segmentIndex));
        writeField(std::to_string(sample.blockIndex));
        writeField(std::to_string(sample.motionType));
    }
    
    out << "\n";
}

std::string CSVExporter::formatDouble(double value, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

bool CSVExporter::exportStatistics(const TrajectoryStatistics& stats, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "Metric" << config_.delimiter << "Value" << config_.delimiter << "Unit\n";
    
    file << "Duration" << config_.delimiter << stats.duration << config_.delimiter << "s\n";
    file << "Path Length" << config_.delimiter << stats.pathLength << config_.delimiter << "mm\n";
    file << "Sample Count" << config_.delimiter << stats.sampleCount << config_.delimiter << "-\n";
    file << "Max Linear Velocity" << config_.delimiter << stats.maxLinearVelocity << config_.delimiter << "mm/s\n";
    file << "Max Linear Acceleration" << config_.delimiter << stats.maxLinearAcceleration << config_.delimiter << "mm/s²\n";
    file << "Max Linear Jerk" << config_.delimiter << stats.maxLinearJerk << config_.delimiter << "mm/s³\n";
    file << "Max Curvature" << config_.delimiter << stats.maxCurvature << config_.delimiter << "1/mm\n";
    file << "Max Centripetal Accel" << config_.delimiter << stats.maxCentripetalAccel << config_.delimiter << "mm/s²\n";
    file << "Meets Limits" << config_.delimiter << (stats.meetsLimits ? "true" : "false") << config_.delimiter << "-\n";
    file << "Violation Count" << config_.delimiter << stats.violations.size() << config_.delimiter << "-\n";
    
    file << "\nPer-Axis Statistics\n";
    file << "Axis" << config_.delimiter 
         << "Min Pos" << config_.delimiter << "Max Pos" << config_.delimiter
         << "Min Vel" << config_.delimiter << "Max Vel" << config_.delimiter
         << "Min Acc" << config_.delimiter << "Max Acc" << config_.delimiter
         << "Min Jerk" << config_.delimiter << "Max Jerk\n";
    
    const char* axisNames[] = {"X", "Y", "Z", "A", "B", "C", "U", "V", "W"};
    for (size_t i = 0; i < 9; ++i) {
        const auto& as = stats.axisStats[i];
        file << axisNames[i] << config_.delimiter
             << as.minPosition << config_.delimiter << as.maxPosition << config_.delimiter
             << as.minVelocity << config_.delimiter << as.maxVelocity << config_.delimiter
             << as.minAcceleration << config_.delimiter << as.maxAcceleration << config_.delimiter
             << as.minJerk << config_.delimiter << as.maxJerk << "\n";
    }
    
    return file.good();
}

bool CSVExporter::exportViolations(const std::vector<LimitViolation>& violations, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "Time" << config_.delimiter 
         << "Axis" << config_.delimiter 
         << "Type" << config_.delimiter
         << "Value" << config_.delimiter
         << "Limit" << config_.delimiter
         << "Overshoot %\n";
    
    const char* axisNames[] = {"X", "Y", "Z", "A", "B", "C", "U", "V", "W", "Combined"};
    
    for (const auto& v : violations) {
        file << v.time << config_.delimiter
             << (v.axis >= 0 ? axisNames[v.axis] : "Combined") << config_.delimiter
             << v.limitType << config_.delimiter
             << v.value << config_.delimiter
             << v.limit << config_.delimiter
             << v.overshoot << "\n";
    }
    
    return file.good();
}

// BatchExporter implementation

int BatchExporter::exportAll(
    const std::vector<TrajectorySample>& samples,
    const TrajectoryStatistics* stats,
    const ExportSpec& spec
) {
    createdFiles_.clear();
    int successCount = 0;
    
    // Compute stats if not provided
    TrajectoryStatistics computedStats;
    if (!stats) {
        TrajectoryAnalyzer analyzer;
        computedStats = analyzer.computeStatistics(samples);
        stats = &computedStats;
    }
    
    // Export SVG
    if (spec.exportSVG) {
        std::string svgFile = spec.basename + ".svg";
        SVGExporter svg(spec.svgConfig);
        if (svg.exportToFile(samples, svgFile)) {
            createdFiles_.push_back(svgFile);
            ++successCount;
        }
    }
    
    // Export CSV trajectory
    if (spec.exportCSV) {
        std::string csvFile = spec.basename + "_trajectory.csv";
        CSVExporter csv(spec.csvConfig);
        if (csv.exportToFile(samples, csvFile)) {
            createdFiles_.push_back(csvFile);
            ++successCount;
        }
    }
    
    // Export statistics
    if (spec.exportStatistics) {
        std::string statsFile = spec.basename + "_statistics.csv";
        CSVExporter csv(spec.csvConfig);
        if (csv.exportStatistics(*stats, statsFile)) {
            createdFiles_.push_back(statsFile);
            ++successCount;
        }
    }
    
    // Export violations
    if (spec.exportViolations && !stats->violations.empty()) {
        std::string violFile = spec.basename + "_violations.csv";
        CSVExporter csv(spec.csvConfig);
        if (csv.exportViolations(stats->violations, violFile)) {
            createdFiles_.push_back(violFile);
            ++successCount;
        }
    }
    
    return successCount;
}

} // namespace GCodeExport
