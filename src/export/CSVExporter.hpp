/**
 * @file CSVExporter.hpp
 * @brief CSV export for trajectory data with all kinematic derivatives
 */

#pragma once

#include "TrajectoryAnalyzer.hpp"
#include "SVGExporter.hpp"
#include <string>
#include <ostream>
#include <vector>
#include <bitset>

namespace GCodeExport {

/**
 * @brief Configuration for CSV export
 */
struct CSVConfig {
    // Delimiter
    char delimiter = ',';
    
    // Which axes to include (X=0, Y=1, Z=2, A=3, B=4, C=5, U=6, V=7, W=8)
    std::bitset<9> includeAxes{0b111111111};  // All axes by default
    
    // What to export
    bool exportTime = true;
    bool exportPathPosition = true;
    bool exportPosition = true;
    bool exportVelocity = true;
    bool exportAcceleration = true;
    bool exportJerk = true;
    bool exportCombinedMetrics = true;       ///< Linear vel/accel/jerk, curvature
    bool exportSegmentInfo = true;
    
    // Formatting
    int positionPrecision = 6;
    int velocityPrecision = 4;
    int accelerationPrecision = 3;
    int jerkPrecision = 2;
    int timePrecision = 6;
    
    // Headers
    bool includeHeader = true;
    bool includeUnits = true;                 ///< Add units row after header
    
    // Axis names
    std::array<std::string, 9> axisNames = {"X", "Y", "Z", "A", "B", "C", "U", "V", "W"};
};

/**
 * @brief Exports trajectory data to CSV format
 */
class CSVExporter {
public:
    explicit CSVExporter(const CSVConfig& config = {});
    
    /**
     * @brief Export trajectory samples to CSV file
     * @param samples Trajectory samples
     * @param filename Output filename
     * @return true on success
     */
    bool exportToFile(const std::vector<TrajectorySample>& samples, const std::string& filename);
    
    /**
     * @brief Export trajectory samples to stream
     */
    void exportToStream(const std::vector<TrajectorySample>& samples, std::ostream& out);
    
    /**
     * @brief Export statistics to CSV
     */
    bool exportStatistics(const TrajectoryStatistics& stats, const std::string& filename);
    
    /**
     * @brief Export limit violations to CSV
     */
    bool exportViolations(const std::vector<LimitViolation>& violations, const std::string& filename);
    
    void configure(const CSVConfig& config) { config_ = config; }
    
private:
    CSVConfig config_;
    
    void writeHeader(std::ostream& out);
    void writeUnitsRow(std::ostream& out);
    void writeSample(std::ostream& out, const TrajectorySample& sample);
    
    std::string formatDouble(double value, int precision);
};

/**
 * @brief Multi-format batch exporter
 * 
 * Convenience class for exporting to multiple formats at once
 */
class BatchExporter {
public:
    struct ExportSpec {
        std::string basename;                 ///< Base filename without extension
        bool exportSVG = true;
        bool exportCSV = true;
        bool exportStatistics = true;
        bool exportViolations = true;
        
        SVGConfig svgConfig;
        CSVConfig csvConfig;
    };
    
    /**
     * @brief Export to all requested formats
     * @param samples Trajectory samples
     * @param stats Statistics (optional, computed if nullptr)
     * @param spec Export specification
     * @return Number of successful exports
     */
    int exportAll(
        const std::vector<TrajectorySample>& samples,
        const TrajectoryStatistics* stats,
        const ExportSpec& spec
    );
    
    /**
     * @brief Get list of created files
     */
    const std::vector<std::string>& createdFiles() const { return createdFiles_; }
    
private:
    std::vector<std::string> createdFiles_;
};

} // namespace GCodeExport
