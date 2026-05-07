/**
 * @file TestDataExporter.hpp
 * @brief Export test data for documentation and visualization
 * 
 * Provides comprehensive data export in multiple formats:
 * - CSV for raw data analysis
 * - JSON for structured data interchange
 * - Binary for efficient storage
 */

#pragma once

#include "MotionReplanner.hpp"
#include "PerformanceHeatmap.hpp"
#include "MachineTester.hpp"
#include "SystemIdentifier.hpp"
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <functional>

namespace MotionReplanner {

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief Export format options
 */
enum class ExportFormat {
    CSV,            ///< Comma-separated values
    TSV,            ///< Tab-separated values
    JSON,           ///< JSON format
    JSONPretty,     ///< Pretty-printed JSON
    Binary,         ///< Binary format for efficiency
    Numpy           ///< NumPy-compatible binary
};

/**
 * @brief Export configuration
 */
struct ExportConfig {
    ExportFormat format = ExportFormat::CSV;
    char delimiter = ',';
    int precision = 6;
    bool includeHeader = true;
    bool includeTimestamp = true;
    bool includeMetadata = true;
    bool compressOutput = false;
    std::string timestampFormat = "%Y-%m-%d %H:%M:%S";
    
    // Column selection
    std::vector<std::string> selectedColumns;  ///< Empty = all columns
    
    // Filtering
    double timeStart = 0;
    double timeEnd = -1;  ///< -1 = no end limit
    int downsampleFactor = 1;
};

/**
 * @brief Metadata for exported data
 */
struct ExportMetadata {
    std::string testName;
    std::string testType;
    std::string machineId;
    std::string timestamp;
    std::string version = "1.0";
    std::map<std::string, std::string> customFields;
    
    // Test parameters
    double sampleRate = 0;
    size_t sampleCount = 0;
    double duration = 0;
    
    // Summary statistics (optional)
    std::map<std::string, double> statistics;
};

//=============================================================================
// Base Exporter
//=============================================================================

/**
 * @brief Base class for data exporters
 */
class DataExporter {
public:
    explicit DataExporter(const ExportConfig& config = {});
    virtual ~DataExporter() = default;
    
    void setConfig(const ExportConfig& config) { config_ = config; }
    const ExportConfig& config() const { return config_; }
    
    void setMetadata(const ExportMetadata& metadata) { metadata_ = metadata; }
    const ExportMetadata& metadata() const { return metadata_; }
    
protected:
    ExportConfig config_;
    ExportMetadata metadata_;
    
public:
    // Helper methods
    std::string formatDouble(double value) const;
    std::string escapeCSV(const std::string& str) const;
    std::string escapeJSON(const std::string& str) const;
    std::string getCurrentTimestamp() const;
};

//=============================================================================
// Trajectory Data Exporter
//=============================================================================

/**
 * @brief Export trajectory and tracking data
 */
class TrajectoryExporter : public DataExporter {
public:
    using DataExporter::DataExporter;
    
    /**
     * @brief Export desired vs actual trajectory
     */
    bool exportTrajectory(const std::string& filename,
                         const std::vector<GCodeExport::TrajectorySample>& desired,
                         const std::vector<GCodeExport::TrajectorySample>& actual);
    
    /**
     * @brief Export tracking error analysis
     */
    bool exportTrackingErrors(const std::string& filename,
                             const std::vector<TrackingError>& errors);
    
    /**
     * @brief Export segment performance data
     */
    bool exportSegmentPerformance(const std::string& filename,
                                  const std::vector<SegmentPerformance>& segments);
    
    /**
     * @brief Export error statistics
     */
    bool exportErrorStatistics(const std::string& filename,
                              const ErrorStatistics& stats);
    
    /**
     * @brief Export parameter suggestions
     */
    bool exportSuggestions(const std::string& filename,
                          const std::vector<ParameterSuggestion>& suggestions);
    
private:
    void writeCSVTrajectory(std::ostream& out,
                           const std::vector<GCodeExport::TrajectorySample>& desired,
                           const std::vector<GCodeExport::TrajectorySample>& actual);
    
    void writeJSONTrajectory(std::ostream& out,
                            const std::vector<GCodeExport::TrajectorySample>& desired,
                            const std::vector<GCodeExport::TrajectorySample>& actual);
};

//=============================================================================
// Heatmap Data Exporter
//=============================================================================

/**
 * @brief Export heatmap data for visualization
 */
class HeatmapExporter : public DataExporter {
public:
    using DataExporter::DataExporter;
    
    /**
     * @brief Export 1D heatmap data
     */
    bool exportHeatmap1D(const std::string& filename, const Heatmap1D& heatmap);
    
    /**
     * @brief Export 2D heatmap data
     */
    bool exportHeatmap2D(const std::string& filename, const Heatmap2D& heatmap);
    
    /**
     * @brief Export 3D heatmap data
     */
    bool exportHeatmap3D(const std::string& filename, const Heatmap3D& heatmap);
    
    /**
     * @brief Export differential heatmap
     */
    bool exportDifferentialHeatmap(const std::string& filename,
                                   const DifferentialHeatmap& heatmap);
    
    /**
     * @brief Export suggested limits
     */
    bool exportSuggestedLimits(const std::string& filename,
                              const SuggestedLimits& limits);
    
private:
    void writeCSVHeatmap2D(std::ostream& out, const Heatmap2D& heatmap);
    void writeJSONHeatmap2D(std::ostream& out, const Heatmap2D& heatmap);
    void writeNumpyHeatmap2D(std::ostream& out, const Heatmap2D& heatmap);
};

//=============================================================================
// Test Result Exporter
//=============================================================================

/**
 * @brief Export machine test results
 */
class TestResultExporter : public DataExporter {
public:
    using DataExporter::DataExporter;
    
    /**
     * @brief Export complete test result
     */
    bool exportTestResult(const std::string& filename, const TestResult& result);
    
    /**
     * @brief Export friction identification result
     */
    bool exportFrictionModel(const std::string& filename,
                            const FrictionIdentificationResult& result);
    
    /**
     * @brief Export delay identification result
     */
    bool exportDelayIdentification(const std::string& filename,
                                   const DelayIdentificationResult& result);
    
    /**
     * @brief Export PID tuning assessment
     */
    bool exportPIDAssessment(const std::string& filename,
                            const PIDTuningAssessment& assessment);
    
    /**
     * @brief Export dynamics identification
     */
    bool exportDynamicsModel(const std::string& filename,
                            const DynamicsIdentificationResult& result);
    
private:
    void writeCSVTestResult(std::ostream& out, const TestResult& result);
    void writeJSONTestResult(std::ostream& out, const TestResult& result);
};

//=============================================================================
// Batch Exporter
//=============================================================================

/**
 * @brief Export multiple files in batch
 */
class BatchExporter {
public:
    explicit BatchExporter(const std::string& outputDir, const ExportConfig& config = {});
    
    /**
     * @brief Set base filename prefix
     */
    void setPrefix(const std::string& prefix) { prefix_ = prefix; }
    
    /**
     * @brief Export all replanner data
     */
    void exportReplannerData(const MotionReplanner& replanner);
    
    /**
     * @brief Export all heatmap data
     */
    void exportHeatmapData(const HeatmapBuilder& builder);
    
    /**
     * @brief Export all test results
     */
    void exportTestResults(const std::vector<TestResult>& results);
    
    /**
     * @brief Export system identification data
     */
    void exportIdentificationData(const SystemIdentifier& identifier,
                                  const DelayIdentificationResult& delay,
                                  const FrictionIdentificationResult& friction,
                                  const PIDTuningAssessment& pid);
    
    /**
     * @brief Generate manifest file listing all exported files
     */
    bool generateManifest(const std::string& filename);
    
    /**
     * @brief Get list of generated files
     */
    const std::vector<std::string>& generatedFiles() const { return generatedFiles_; }
    
private:
    std::string outputDir_;
    std::string prefix_;
    ExportConfig config_;
    std::vector<std::string> generatedFiles_;
    
    std::string makeFilename(const std::string& baseName, const std::string& extension);
};

//=============================================================================
// Streaming Exporter (for real-time data)
//=============================================================================

/**
 * @brief Stream data to file in real-time
 */
class StreamingExporter {
public:
    StreamingExporter(const std::string& filename, const ExportConfig& config = {});
    ~StreamingExporter();
    
    /**
     * @brief Open file for writing
     */
    bool open();
    
    /**
     * @brief Close file
     */
    void close();
    
    /**
     * @brief Check if open
     */
    bool isOpen() const { return file_.is_open(); }
    
    /**
     * @brief Write header
     */
    void writeHeader(const std::vector<std::string>& columns);
    
    /**
     * @brief Write data row
     */
    void writeRow(const std::vector<double>& values);
    void writeRow(const std::vector<std::string>& values);
    
    /**
     * @brief Write trajectory sample
     */
    void writeSample(const GCodeExport::TrajectorySample& desired,
                    const GCodeExport::TrajectorySample& actual);
    
    /**
     * @brief Write tracking error
     */
    void writeError(const TrackingError& error);
    
    /**
     * @brief Flush to disk
     */
    void flush();
    
    /**
     * @brief Get bytes written
     */
    size_t bytesWritten() const { return bytesWritten_; }
    
private:
    std::string filename_;
    ExportConfig config_;
    std::ofstream file_;
    size_t bytesWritten_;
    bool headerWritten_;
};

//=============================================================================
// Report Generator
//=============================================================================

/**
 * @brief Generate comprehensive test reports
 */
class ReportGenerator {
public:
    struct ReportSection {
        std::string title;
        std::string content;
        std::vector<std::string> figures;  ///< Paths to associated figures
        std::vector<std::string> tables;   ///< Paths to associated data files
    };
    
    ReportGenerator();
    
    /**
     * @brief Add section to report
     */
    void addSection(const ReportSection& section);
    
    /**
     * @brief Generate summary section from test results
     */
    void addSummary(const std::vector<TestResult>& results);
    
    /**
     * @brief Generate error statistics section
     */
    void addErrorStatistics(const ErrorStatistics& stats);
    
    /**
     * @brief Generate system identification section
     */
    void addSystemIdentification(const DelayIdentificationResult& delay,
                                 const FrictionIdentificationResult& friction,
                                 const PIDTuningAssessment& pid);
    
    /**
     * @brief Generate recommendations section
     */
    void addRecommendations(const std::vector<ParameterSuggestion>& suggestions);
    
    /**
     * @brief Export as Markdown
     */
    bool exportMarkdown(const std::string& filename);
    
    /**
     * @brief Export as HTML
     */
    bool exportHTML(const std::string& filename);
    
    /**
     * @brief Export as LaTeX
     */
    bool exportLaTeX(const std::string& filename);
    
private:
    std::vector<ReportSection> sections_;
    ExportMetadata metadata_;
    
    std::string generateMarkdownTable(const std::vector<std::vector<std::string>>& data,
                                     const std::vector<std::string>& headers);
};

//=============================================================================
// JSON Helper
//=============================================================================

/**
 * @brief Simple JSON builder
 */
class JSONBuilder {
public:
    JSONBuilder(bool pretty = false);
    
    void beginObject();
    void endObject();
    void beginArray();
    void endArray();
    
    void key(const std::string& k);
    void value(const std::string& v);
    void value(double v);
    void value(int v);
    void value(bool v);
    void valueNull();
    
    void keyValue(const std::string& k, const std::string& v);
    void keyValue(const std::string& k, double v);
    void keyValue(const std::string& k, int v);
    void keyValue(const std::string& k, bool v);
    
    // Array of doubles (efficient)
    void doubleArray(const std::string& key, const std::vector<double>& values);
    void doubleArray(const std::vector<double>& values);
    
    std::string str() const { return ss_.str(); }
    
private:
    std::ostringstream ss_;
    bool pretty_;
    int indent_;
    bool needComma_;
    
    void newline();
    void comma();
};

} // namespace MotionReplanner
