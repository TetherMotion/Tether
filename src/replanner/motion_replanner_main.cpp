/**
 * @file motion_replanner_main.cpp
 * @brief Main integration program for motion replanning framework
 * 
 * This CLI tool provides:
 * - Trajectory analysis with closed-loop feedback
 * - Machine performance testing
 * - System identification (delay, friction, PID)
 * - Heatmap generation
 * - G-Code export
 * - Data export for visualization
 */

#include "MotionReplanner.hpp"
#include "PerformanceHeatmap.hpp"
#include "MachineTester.hpp"
#include "SystemIdentifier.hpp"
#include "GCodeGenerator.hpp"
#include "TestDataExporter.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;
using namespace MotionReplanner;

//=============================================================================
// Command-line parsing
//=============================================================================

struct Options {
    std::string command;
    std::string inputFile;
    std::string outputDir = "./output";
    std::string configFile;
    
    // Replanner options
    double systemDelay = 0.001;  // 1ms default
    double cornerThreshold = 30.0;  // degrees
    bool monitorMode = true;
    
    // Test options
    std::string testType = "circle";
    double testRadius = 50.0;
    double testFeedRate = 1000.0;
    int testRevolutions = 3;
    
    // Heatmap options
    double heatmapResolution = 10.0;
    std::array<double, 3> workspaceMin = {0, 0, 0};
    std::array<double, 3> workspaceMax = {300, 300, 100};
    
    // Export options
    std::string exportFormat = "csv";
    std::string gcodeDialect = "linuxcnc";
    
    // Flags
    bool verbose = false;
    bool help = false;
};

void printUsage(const char* progName) {
    std::cout << R"(
Motion Replanner - Comprehensive motion analysis and testing framework

Usage: )" << progName << R"( <command> [options]

Commands:
  analyze     Analyze trajectory from input file
  test        Generate and run machine tests
  identify    System identification (delay, friction, PID)
  heatmap     Generate performance heatmaps
  gcode       Generate G-Code test patterns
  export      Export data for visualization
  report      Generate comprehensive test report

Common Options:
  -i, --input <file>      Input data file
  -o, --output <dir>      Output directory (default: ./output)
  -c, --config <file>     Configuration file
  -v, --verbose           Verbose output
  -h, --help              Show this help

Analyze Options:
  --delay <seconds>       System delay compensation (default: 0.001)
  --corner <degrees>      Corner detection threshold (default: 30)
  --monitor               Enable monitoring mode (default)
  --replan                Enable replanning mode

Test Options:
  --test-type <type>      Test type: circle, ellipse, sinusoid, ramp, square
  --radius <mm>           Test radius (default: 50)
  --feed <mm/min>         Feed rate (default: 1000)
  --revolutions <n>       Number of revolutions (default: 3)

Heatmap Options:
  --resolution <mm>       Heatmap resolution (default: 10)
  --workspace <x,y,z,X,Y,Z>  Workspace bounds (default: 0,0,0,300,300,100)

Export Options:
  --format <fmt>          Export format: csv, json (default: csv)
  --dialect <name>        G-Code dialect: linuxcnc, fanuc, grbl (default: linuxcnc)

Examples:
  )" << progName << R"( analyze -i trajectory.csv -o results/ --delay 0.002
  )" << progName << R"( test --test-type circle --radius 100 -o tests/
  )" << progName << R"( gcode --test-type sinusoid --dialect grbl -o gcode/
  )" << progName << R"( heatmap -i performance_data.csv -o heatmaps/
  )" << progName << R"( report -i ./data/ -o report/

)" << std::endl;
}

bool parseOptions(int argc, char** argv, Options& opts) {
    if (argc < 2) {
        return false;
    }
    
    opts.command = argv[1];
    
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            opts.help = true;
        } else if (arg == "-v" || arg == "--verbose") {
            opts.verbose = true;
        } else if (arg == "-i" || arg == "--input") {
            if (++i < argc) opts.inputFile = argv[i];
        } else if (arg == "-o" || arg == "--output") {
            if (++i < argc) opts.outputDir = argv[i];
        } else if (arg == "-c" || arg == "--config") {
            if (++i < argc) opts.configFile = argv[i];
        } else if (arg == "--delay") {
            if (++i < argc) opts.systemDelay = std::atof(argv[i]);
        } else if (arg == "--corner") {
            if (++i < argc) opts.cornerThreshold = std::atof(argv[i]);
        } else if (arg == "--monitor") {
            opts.monitorMode = true;
        } else if (arg == "--replan") {
            opts.monitorMode = false;
        } else if (arg == "--test-type") {
            if (++i < argc) opts.testType = argv[i];
        } else if (arg == "--radius") {
            if (++i < argc) opts.testRadius = std::atof(argv[i]);
        } else if (arg == "--feed") {
            if (++i < argc) opts.testFeedRate = std::atof(argv[i]);
        } else if (arg == "--revolutions") {
            if (++i < argc) opts.testRevolutions = std::atoi(argv[i]);
        } else if (arg == "--resolution") {
            if (++i < argc) opts.heatmapResolution = std::atof(argv[i]);
        } else if (arg == "--workspace") {
            if (++i < argc) {
                // Parse x,y,z,X,Y,Z format
                char* str = argv[i];
                char* token = std::strtok(str, ",");
                for (int j = 0; j < 3 && token; ++j) {
                    opts.workspaceMin[j] = std::atof(token);
                    token = std::strtok(nullptr, ",");
                }
                for (int j = 0; j < 3 && token; ++j) {
                    opts.workspaceMax[j] = std::atof(token);
                    token = std::strtok(nullptr, ",");
                }
            }
        } else if (arg == "--format") {
            if (++i < argc) opts.exportFormat = argv[i];
        } else if (arg == "--dialect") {
            if (++i < argc) opts.gcodeDialect = argv[i];
        }
    }
    
    return true;
}

//=============================================================================
// Data Loading
//=============================================================================

bool loadTrajectoryCSV(const std::string& filename,
                       std::vector<GCodeExport::TrajectorySample>& desired,
                       std::vector<GCodeExport::TrajectorySample>& actual) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return false;
    }
    
    std::string line;
    
    // Skip header
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        double t, dx, dy, dz, dvx, dvy, dvz, ax, ay, az, avx, avy, avz;
        
        if (std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &t, &dx, &dy, &dz, &dvx, &dvy, &dvz, &ax, &ay, &az, &avx, &avy, &avz) >= 7) {
            GCodeExport::TrajectorySample des = {t, {dx, dy, dz}, {dvx, dvy, dvz}, {0, 0, 0}};
            GCodeExport::TrajectorySample act = {t, {ax, ay, az}, {avx, avy, avz}, {0, 0, 0}};
            desired.push_back(des);
            actual.push_back(act);
        }
    }
    
    return !desired.empty();
}

//=============================================================================
// Commands
//=============================================================================

int cmdAnalyze(const Options& opts) {
    if (opts.inputFile.empty()) {
        std::cerr << "Error: Input file required for analyze command" << std::endl;
        return 1;
    }
    
    std::cout << "Analyzing trajectory: " << opts.inputFile << std::endl;
    
    // Load data
    std::vector<GCodeExport::TrajectorySample> desired, actual;
    if (!loadTrajectoryCSV(opts.inputFile, desired, actual)) {
        return 1;
    }
    
    std::cout << "Loaded " << desired.size() << " samples" << std::endl;
    
    // Configure replanner
    ReplannerConfig config;
    config.systemDelay = opts.systemDelay;
    config.cornerAngleThreshold = opts.cornerThreshold;
    config.monitorMode = opts.monitorMode;
    
    MotionReplanner replanner(config);
    
    // Set desired trajectory
    replanner.setDesiredTrajectory(desired);
    
    // Process actual samples
    for (const auto& sample : actual) {
        replanner.addActualSample(sample);
    }
    
    replanner.processAccumulatedSamples();
    
    // Get results
    auto stats = replanner.getStatistics();
    
    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << "Sample count: " << stats.count << std::endl;
    std::cout << "Min error:    " << stats.minError << " mm" << std::endl;
    std::cout << "Max error:    " << stats.maxError << " mm" << std::endl;
    std::cout << "Mean error:   " << stats.meanError << " mm" << std::endl;
    std::cout << "Geo. mean:    " << stats.geometricMeanError << " mm" << std::endl;
    std::cout << "Std dev:      " << stats.stdDevError << " mm" << std::endl;
    std::cout << "RMS error:    " << stats.rmsError << " mm" << std::endl;
    std::cout << "\nPercentiles:" << std::endl;
    std::cout << "  50%: " << stats.percentile50 << " mm" << std::endl;
    std::cout << "  90%: " << stats.percentile90 << " mm" << std::endl;
    std::cout << "  95%: " << stats.percentile95 << " mm" << std::endl;
    std::cout << "  99%: " << stats.percentile99 << " mm" << std::endl;
    
    if (stats.cornerCount > 0) {
        std::cout << "\nCorner Analysis:" << std::endl;
        std::cout << "  Count:      " << stats.cornerCount << std::endl;
        std::cout << "  Max error:  " << stats.maxCornerError << " mm" << std::endl;
        std::cout << "  Mean error: " << stats.meanCornerError << " mm" << std::endl;
    }
    
    // Export results
    fs::create_directories(opts.outputDir);
    
    ExportConfig exportConfig;
    exportConfig.format = (opts.exportFormat == "json") ? ExportFormat::JSONPretty : ExportFormat::CSV;
    
    BatchExporter exporter(opts.outputDir, exportConfig);
    exporter.exportReplannerData(replanner);
    exporter.generateManifest("manifest.json");
    
    std::cout << "\nResults exported to: " << opts.outputDir << std::endl;
    
    // Show suggestions if any
    auto suggestions = replanner.getSuggestions();
    if (!suggestions.empty()) {
        std::cout << "\n=== Parameter Suggestions ===" << std::endl;
        for (const auto& sug : suggestions) {
            std::cout << "Segment " << sug.segmentId << ": " << sug.parameterName
                      << " " << sug.currentValue << " -> " << sug.suggestedValue
                      << " (" << sug.reason << ")" << std::endl;
        }
    }
    
    return 0;
}

int cmdTest(const Options& opts) {
    std::cout << "Generating machine tests" << std::endl;
    
    MachineTester tester;
    
    // Create test configuration
    TestSequence sequence;
    sequence.name = "Machine Calibration";
    sequence.description = "Comprehensive machine performance tests";
    
    // Add tests based on type
    if (opts.testType == "circle") {
        MultiAxisTestConfig config;
        config.type = MultiAxisTestType::Circle;
        config.center = {150, 150, 10};
        config.radiusU = opts.testRadius;
        config.feedRate = opts.testFeedRate;
        config.revolutions = opts.testRevolutions;
        sequence.multiAxisTests.push_back(config);
    } else if (opts.testType == "ellipse") {
        MultiAxisTestConfig config;
        config.type = MultiAxisTestType::Ellipse;
        config.center = {150, 150, 10};
        config.radiusU = opts.testRadius;
        config.radiusV = opts.testRadius * 0.6;
        config.feedRate = opts.testFeedRate;
        config.revolutions = opts.testRevolutions;
        sequence.multiAxisTests.push_back(config);
    } else if (opts.testType == "sinusoid") {
        SingleAxisTestConfig config;
        config.type = SingleAxisTestType::Sinusoid;
        config.axis = 0;  // X
        config.centerPosition = 150;
        config.amplitude = opts.testRadius;
        config.frequency = 0.5;
        config.cycles = opts.testRevolutions;
        config.velocity = opts.testFeedRate;
        sequence.singleAxisTests.push_back(config);
    } else if (opts.testType == "square") {
        MultiAxisTestConfig config;
        config.type = MultiAxisTestType::Square;
        config.center = {150, 150, 10};
        config.radiusU = opts.testRadius;
        config.feedRate = opts.testFeedRate;
        config.revolutions = opts.testRevolutions;
        sequence.multiAxisTests.push_back(config);
    }
    
    // Generate trajectories
    auto trajectories = tester.generateTestTrajectories(sequence);
    
    std::cout << "Generated " << trajectories.size() << " test trajectories" << std::endl;
    
    // Export test data
    fs::create_directories(opts.outputDir);
    
    ExportConfig exportConfig;
    TrajectoryExporter exporter(exportConfig);
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        std::string filename = opts.outputDir + "/test_" + std::to_string(i) + "_trajectory.csv";
        
        // Export desired trajectory (actual will be empty until test is run)
        std::vector<GCodeExport::TrajectorySample> empty;
        exporter.exportTrajectory(filename, trajectories[i], empty);
        
        std::cout << "Exported: " << filename << std::endl;
    }
    
    return 0;
}

int cmdIdentify(const Options& opts) {
    if (opts.inputFile.empty()) {
        std::cerr << "Error: Input file required for identify command" << std::endl;
        return 1;
    }
    
    std::cout << "System identification from: " << opts.inputFile << std::endl;
    
    // Load data
    std::vector<GCodeExport::TrajectorySample> desired, actual;
    if (!loadTrajectoryCSV(opts.inputFile, desired, actual)) {
        return 1;
    }
    
    // Convert to identification samples
    SystemIdentifier identifier;
    
    for (size_t i = 0; i < std::min(desired.size(), actual.size()); ++i) {
        IdentificationSample sample;
        sample.timestamp = desired[i].time;
        sample.commanded = desired[i].position[0];  // Use X axis
        sample.actual = actual[i].position[0];
        sample.velocity = actual[i].velocity[0];
        identifier.addSample(sample);
    }
    
    // Identify delay
    std::cout << "\n=== Delay Identification ===" << std::endl;
    auto delay = identifier.identifyDelay();
    std::cout << "Transport delay: " << delay.transportDelay * 1000 << " ms" << std::endl;
    std::cout << "Confidence:      " << delay.delayConfidence * 100 << "%" << std::endl;
    std::cout << "Rise time:       " << delay.riseTime * 1000 << " ms" << std::endl;
    std::cout << "Settling time:   " << delay.settlingTime * 1000 << " ms" << std::endl;
    std::cout << "Overshoot:       " << delay.overshoot << "%" << std::endl;
    
    // Identify friction (if force/torque data available)
    std::cout << "\n=== Friction Identification ===" << std::endl;
    auto friction = identifier.identifyFriction();
    if (friction.isValid()) {
        std::cout << "Best model:   " << friction.bestModel.modelName() << std::endl;
        std::cout << "Coulomb:      " << friction.bestModel.coulombForce << " N" << std::endl;
        std::cout << "Viscous:      " << friction.bestModel.viscousCoeff << " N/(m/s)" << std::endl;
        std::cout << "R²:           " << friction.bestModel.rSquared << std::endl;
    } else {
        std::cout << "Insufficient data for friction identification" << std::endl;
    }
    
    // Analyze PID tuning
    std::cout << "\n=== PID Tuning Analysis ===" << std::endl;
    auto pid = identifier.analyzePIDTuning();
    std::cout << "Overall score:   " << pid.overallScore << "/100" << std::endl;
    std::cout << "Stability:       " << pid.stabilityScore << "/100" << std::endl;
    std::cout << "Response:        " << pid.responseScore << "/100" << std::endl;
    std::cout << "Accuracy:        " << pid.accuracyScore << "/100" << std::endl;
    
    if (!pid.issues.empty()) {
        std::cout << "\nIssues:" << std::endl;
        for (const auto& issue : pid.issues) {
            std::cout << "  - " << issue << std::endl;
        }
    }
    
    if (!pid.recommendations.empty()) {
        std::cout << "\nRecommendations:" << std::endl;
        for (const auto& rec : pid.recommendations) {
            std::cout << "  - " << rec << std::endl;
        }
    }
    
    // Export results
    fs::create_directories(opts.outputDir);
    
    TestResultExporter exporter;
    exporter.exportDelayIdentification(opts.outputDir + "/delay_identification.json", delay);
    exporter.exportFrictionModel(opts.outputDir + "/friction_model.json", friction);
    exporter.exportPIDAssessment(opts.outputDir + "/pid_assessment.json", pid);
    
    std::cout << "\nResults exported to: " << opts.outputDir << std::endl;
    
    return 0;
}

int cmdHeatmap(const Options& opts) {
    if (opts.inputFile.empty()) {
        std::cerr << "Error: Input file required for heatmap command" << std::endl;
        return 1;
    }
    
    std::cout << "Generating heatmaps from: " << opts.inputFile << std::endl;
    
    // Load data
    std::vector<GCodeExport::TrajectorySample> desired, actual;
    if (!loadTrajectoryCSV(opts.inputFile, desired, actual)) {
        return 1;
    }
    
    // Configure heatmap
    HeatmapConfig config;
    config.resolution1D = 50;
    config.resolution2D = static_cast<int>((opts.workspaceMax[0] - opts.workspaceMin[0]) / opts.heatmapResolution);
    config.resolution3D = opts.heatmapResolution;
    config.minBounds = opts.workspaceMin;
    config.maxBounds = opts.workspaceMax;
    
    HeatmapBuilder builder(config);
    
    // Add samples to builder
    for (size_t i = 0; i < std::min(desired.size(), actual.size()); ++i) {
        builder.addSample(desired[i], actual[i]);
    }
    
    builder.finalize();
    
    // Export heatmaps
    fs::create_directories(opts.outputDir);
    
    ExportConfig exportConfig;
    exportConfig.format = (opts.exportFormat == "json") ? ExportFormat::JSONPretty : ExportFormat::CSV;
    
    BatchExporter exporter(opts.outputDir, exportConfig);
    exporter.exportHeatmapData(builder);
    exporter.generateManifest("manifest.json");
    
    // Print summary
    auto limits = builder.getSuggestedLimits();
    std::cout << "\n=== Suggested Limits ===" << std::endl;
    std::cout << "Max velocity:     " << limits.maxVelocity << " mm/s" << std::endl;
    std::cout << "Max acceleration: " << limits.maxAcceleration << " mm/s²" << std::endl;
    std::cout << "Confidence:       " << limits.confidence * 100 << "%" << std::endl;
    
    std::cout << "\nHeatmaps exported to: " << opts.outputDir << std::endl;
    
    return 0;
}

int cmdGcode(const Options& opts) {
    std::cout << "Generating G-Code test patterns" << std::endl;
    
    // Configure G-Code options
    GCodeOptions gcodeOpts;
    gcodeOpts.useMetric = true;
    gcodeOpts.absoluteMode = true;
    gcodeOpts.defaultFeedRate = opts.testFeedRate;
    gcodeOpts.safeZ = 20.0;
    
    // Set dialect
    if (opts.gcodeDialect == "fanuc") {
        gcodeOpts.dialect = GCodeDialect::Fanuc;
    } else if (opts.gcodeDialect == "grbl") {
        gcodeOpts.dialect = GCodeDialect::Grbl;
    } else if (opts.gcodeDialect == "marlin") {
        gcodeOpts.dialect = GCodeDialect::Marlin;
    } else {
        gcodeOpts.dialect = GCodeDialect::LinuxCNC;
    }
    
    TestPatternGenerator generator(gcodeOpts);
    
    // Create output directory
    fs::create_directories(opts.outputDir);
    
    // Generate test pattern based on type
    GCodeProgram prog(gcodeOpts);
    std::string filename;
    
    if (opts.testType == "circle") {
        prog = generator.generateCircleTest(150, 150, 10, opts.testRadius,
                                           opts.testFeedRate, opts.testRevolutions);
        filename = "circle_test.nc";
    } else if (opts.testType == "ellipse") {
        prog = generator.generateEllipseTest(150, 150, 10, opts.testRadius, opts.testRadius * 0.6,
                                            0, opts.testFeedRate, opts.testRevolutions);
        filename = "ellipse_test.nc";
    } else if (opts.testType == "sinusoid") {
        prog = generator.generateSinusoidTest(0, 150, opts.testRadius, 0.5,
                                             opts.testRevolutions, opts.testFeedRate);
        filename = "sinusoid_test.nc";
    } else if (opts.testType == "ramp") {
        prog = generator.generateRampTest(0, 50, 250, opts.testFeedRate, opts.testRevolutions);
        filename = "ramp_test.nc";
    } else if (opts.testType == "square") {
        prog = generator.generateSquareTest(150, 150, 10, opts.testRadius * 2,
                                           opts.testFeedRate, opts.testRevolutions);
        filename = "square_test.nc";
    } else if (opts.testType == "rounded_square") {
        prog = generator.generateRoundedSquareTest(150, 150, 10, opts.testRadius * 2,
                                                   10, opts.testFeedRate, opts.testRevolutions);
        filename = "rounded_square_test.nc";
    } else if (opts.testType == "friction") {
        std::vector<double> feedRates = {100, 200, 500, 1000, 2000, 5000};
        prog = generator.generateFrictionTest(0, 100, feedRates, 3);
        filename = "friction_test.nc";
    }
    
    // Save program
    std::string fullPath = opts.outputDir + "/" + filename;
    prog.saveToFile(fullPath);
    
    std::cout << "Generated: " << fullPath << std::endl;
    std::cout << "Estimated duration: " << prog.estimateDuration() << " seconds" << std::endl;
    
    auto bounds = prog.getBounds();
    std::cout << "Bounds: X[" << bounds.first[0] << ", " << bounds.second[0] << "] "
              << "Y[" << bounds.first[1] << ", " << bounds.second[1] << "] "
              << "Z[" << bounds.first[2] << ", " << bounds.second[2] << "]" << std::endl;
    
    return 0;
}

int cmdReport(const Options& opts) {
    std::cout << "Generating comprehensive report" << std::endl;
    
    fs::create_directories(opts.outputDir);
    
    ReportGenerator report;
    
    // If input directory specified, load data from there
    if (!opts.inputFile.empty() && fs::is_directory(opts.inputFile)) {
        // Load and process files
        for (const auto& entry : fs::directory_iterator(opts.inputFile)) {
            if (entry.path().extension() == ".json") {
                // Process JSON files for report
                std::cout << "Processing: " << entry.path() << std::endl;
            }
        }
    }
    
    // Generate default report sections
    ReportGenerator::ReportSection intro;
    intro.title = "Introduction";
    intro.content = "This report summarizes the motion performance analysis results.\n\n"
                   "The analysis includes:\n"
                   "- Trajectory tracking error statistics\n"
                   "- Performance heatmaps\n"
                   "- System identification results\n"
                   "- Parameter recommendations\n";
    report.addSection(intro);
    
    // Export reports in multiple formats
    report.exportMarkdown(opts.outputDir + "/report.md");
    report.exportHTML(opts.outputDir + "/report.html");
    
    std::cout << "Reports generated in: " << opts.outputDir << std::endl;
    
    return 0;
}

//=============================================================================
// Main
//=============================================================================

int main(int argc, char** argv) {
    Options opts;
    
    if (!parseOptions(argc, argv, opts) || opts.help) {
        printUsage(argv[0]);
        return opts.help ? 0 : 1;
    }
    
    try {
        if (opts.command == "analyze") {
            return cmdAnalyze(opts);
        } else if (opts.command == "test") {
            return cmdTest(opts);
        } else if (opts.command == "identify") {
            return cmdIdentify(opts);
        } else if (opts.command == "heatmap") {
            return cmdHeatmap(opts);
        } else if (opts.command == "gcode") {
            return cmdGcode(opts);
        } else if (opts.command == "report") {
            return cmdReport(opts);
        } else if (opts.command == "export") {
            // Alias for analyze with export
            return cmdAnalyze(opts);
        } else {
            std::cerr << "Unknown command: " << opts.command << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
