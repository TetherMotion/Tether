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

#include "tether/motion_replanner/MotionReplanner.hpp"
#include "tether/motion_replanner/PerformanceHeatmap.hpp"
#include "tether/motion_replanner/MachineTester.hpp"
#include "tether/motion_replanner/SystemIdentifier.hpp"
#include "tether/motion_replanner/GCodeGenerator.hpp"
#include "tether/motion_replanner/TestDataExporter.hpp"
#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/PathRelativeFFT.hpp"
#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/SvgExporter.hpp"

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
    bool enableFFT = true;  // For evaluate command
    bool enableKDE = true;  // For evaluate command
    std::string kdeDerivative = "velocity";   // velocity/acceleration/jerk/curvature/feedrate/arclength/time
    std::string kdeDeviation = "contour";     // contour/lag/combined/binormal/tracking/velocity/acceleration
    std::string kdeKernel = "gaussian";       // gaussian/epanechnikov/uniform/triangular/quartic/cosine
    std::string kdeBandwidth = "silverman";   // silverman/scott/isj/fixed/lscv/likelihoodcv
    std::string kdeColormap = "viridis";      // viridis/inferno/plasma/magma/jet/hot/cool/grayscale/bluered
};

void printUsage(const char* progName) {
    std::cout << R"(
Motion Replanner - Comprehensive motion analysis and testing framework

Usage: )" << progName << R"( <command> [options]

Commands:
  analyze     Analyze trajectory from input file
  evaluate    Evaluate desired vs actual path (quantitative, qualitative, FFT, SVG)
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
  --format <fmt>          Export format: csv, json, svg, all (default: csv)
  --dialect <name>        G-Code dialect: linuxcnc, fanuc, grbl (default: linuxcnc)

Evaluate Options:
  --no-fft                Skip FFT/spectral analysis
  --fft                   Enable FFT/spectral analysis (default: enabled)
  --no-kde                Skip KDE derivative-vs-deviation analysis
  --kde                   Enable KDE analysis (default: enabled)
  --kde-derivative <ax>   KDE derivative axis: velocity, acceleration, jerk,
                          curvature, feedrate, arclength, time (default: velocity)
  --kde-deviation <ax>    KDE deviation axis: contour, lag, combined, binormal,
                          tracking, velocity, acceleration (default: contour)
  --kde-kernel <k>        KDE kernel: gaussian, epanechnikov, uniform, triangular,
                          quartic, cosine (default: gaussian)
  --kde-bandwidth <m>     Bandwidth method: silverman, scott, isj, fixed, lscv,
                          likelihoodcv (default: silverman)
  --kde-colormap <c>      Colormap: viridis, inferno, plasma, magma, jet, hot,
                          cool, grayscale, bluered (default: viridis)

Examples:
  )" << progName << R"( analyze -i trajectory.csv -o results/ --delay 0.002
  )" << progName << R"( evaluate -i trajectory.csv -o eval_results/ --format all
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
        } else if (arg == "--no-fft") {
            opts.enableFFT = false;
        } else if (arg == "--fft") {
            opts.enableFFT = true;
        } else if (arg == "--no-kde") {
            opts.enableKDE = false;
        } else if (arg == "--kde") {
            opts.enableKDE = true;
        } else if (arg == "--kde-derivative") {
            if (++i < argc) opts.kdeDerivative = argv[i];
        } else if (arg == "--kde-deviation") {
            if (++i < argc) opts.kdeDeviation = argv[i];
        } else if (arg == "--kde-kernel") {
            if (++i < argc) opts.kdeKernel = argv[i];
        } else if (arg == "--kde-bandwidth") {
            if (++i < argc) opts.kdeBandwidth = argv[i];
        } else if (arg == "--kde-colormap") {
            if (++i < argc) opts.kdeColormap = argv[i];
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
            GCodeExport::TrajectorySample des;
            des.time = t;
            des.position = {dx, dy, dz, 0, 0, 0, 0, 0, 0};
            des.velocity = {dvx, dvy, dvz, 0, 0, 0, 0, 0, 0};
            des.acceleration = {};
            GCodeExport::TrajectorySample act;
            act.time = t;
            act.position = {ax, ay, az, 0, 0, 0, 0, 0, 0};
            act.velocity = {avx, avy, avz, 0, 0, 0, 0, 0, 0};
            act.acceleration = {};
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

    ::MotionReplanner::MotionReplanner replanner(config);

    // Set desired trajectory
    replanner.setDesiredTrajectory(desired);

    // Get results
    auto stats = replanner.getOverallStatistics();
    
    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << "Sample count: " << stats.sampleCount << std::endl;
    std::cout << "Min error:    " << stats.minError << " mm" << std::endl;
    std::cout << "Max error:    " << stats.maxError << " mm" << std::endl;
    std::cout << "Mean error:   " << stats.meanError << " mm" << std::endl;
    std::cout << "Geo. mean:    " << stats.geometricMean << " mm" << std::endl;
    std::cout << "Std dev:      " << stats.stdDev << " mm" << std::endl;
    std::cout << "RMS error:    " << stats.rmsError << " mm" << std::endl;
    std::cout << "\nPercentiles:" << std::endl;
    std::cout << "  95%: " << stats.p95Error << " mm" << std::endl;
    std::cout << "  99%: " << stats.p99Error << " mm" << std::endl;
    
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
    auto suggestions = replanner.getParameterSuggestions();
    if (!suggestions.empty()) {
        std::cout << "\n=== Parameter Suggestions ===" << std::endl;
        for (const auto& sug : suggestions) {
            std::cout << "Segment " << sug.segmentIndex << ": "
                      << "feed " << sug.currentFeedRate << " -> " << sug.suggestedFeedRate
                      << ", accel " << sug.currentAccel << " -> " << sug.suggestedAccel
                      << std::endl;
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
    
    // Run tests
    auto results = tester.runTestSequence(sequence);

    std::cout << "Generated " << results.size() << " test results" << std::endl;
    
    // Export test data
    fs::create_directories(opts.outputDir);
    
    ExportConfig exportConfig;
    BatchExporter batchExporter(opts.outputDir, exportConfig);
    batchExporter.exportTestResults(results);
    batchExporter.generateManifest("manifest.json");
    std::cout << "Exported test results to: " << opts.outputDir << std::endl;
    
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

    // Process samples
    for (size_t i = 0; i < std::min(desired.size(), actual.size()); ++i) {
        double trackingError = 0.0, contourError = 0.0;
        // Compute simple errors
        double dx = actual[i].position[0] - desired[i].position[0];
        double dy = actual[i].position[1] - desired[i].position[1];
        double dz = actual[i].position[2] - desired[i].position[2];
        trackingError = std::sqrt(dx*dx + dy*dy + dz*dz);
        contourError = trackingError; // simplified

        double cmdFeed = std::sqrt(desired[i].velocity[0]*desired[i].velocity[0] +
                                   desired[i].velocity[1]*desired[i].velocity[1] +
                                   desired[i].velocity[2]*desired[i].velocity[2]);
        double actFeed = std::sqrt(actual[i].velocity[0]*actual[i].velocity[0] +
                                   actual[i].velocity[1]*actual[i].velocity[1] +
                                   actual[i].velocity[2]*actual[i].velocity[2]);

        builder.processSample(desired[i].position, desired[i].velocity,
                              trackingError, contourError, cmdFeed, actFeed);
    }

    // Export heatmaps
    fs::create_directories(opts.outputDir);

    ExportConfig exportConfig;
    exportConfig.format = (opts.exportFormat == "json") ? ExportFormat::JSONPretty : ExportFormat::CSV;

    BatchExporter exporter(opts.outputDir, exportConfig);
    exporter.exportHeatmapData(builder);
    exporter.generateManifest("manifest.json");

    // Print summary from XY heatmap
    auto& xyHeatmap = builder.getXYHeatmap();
    auto limits = xyHeatmap.getSuggestedLimits(150.0, 150.0);
    std::cout << "\n=== Suggested Limits (at center) ===" << std::endl;
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

int cmdEvaluate(const Options& opts) {
    if (opts.inputFile.empty()) {
        std::cerr << "Error: Input file required for evaluate command" << std::endl;
        return 1;
    }

    std::cout << "Evaluating trajectory: " << opts.inputFile << std::endl;

    // Load data
    std::vector<GCodeExport::TrajectorySample> desired, actual;
    if (!loadTrajectoryCSV(opts.inputFile, desired, actual)) {
        return 1;
    }

    std::cout << "Loaded " << desired.size() << " samples" << std::endl;

    // Create output directory
    fs::create_directories(opts.outputDir);

    //--- Quantitative evaluation ---
    tether::motion::replanner::EvaluatorConfig evalConfig;
    evalConfig.useCertifiedContourError = true;
    tether::motion::replanner::PathEvaluator evaluator(evalConfig);

    std::cout << "Computing quantitative metrics..." << std::endl;
    auto quant = evaluator.evaluateQuantitative(desired, actual);

    //--- Spectral evaluation ---
    tether::motion::replanner::SpectralEvaluation spectral;
    if (opts.enableFFT) {
        std::cout << "Computing path-relative FFT analysis..." << std::endl;
        tether::motion::replanner::PathRelativeFFT fftEval;
        spectral = fftEval.evaluate(desired, actual);
    }

    //--- Qualitative evaluation ---
    std::cout << "Computing qualitative grades..." << std::endl;
    auto qual = evaluator.evaluateQualitative(quant,
        opts.enableFFT ? &spectral : nullptr);

    //--- Print summary to stdout ---
    std::cout << "\n=== Path Evaluation Summary ===" << std::endl;
    std::cout << "Samples: " << quant.sampleCount << std::endl;
    std::cout << "Path length: " << quant.pathLength << " mm" << std::endl;
    std::cout << "Duration: " << quant.duration << " s" << std::endl;

    std::cout << "\n--- Quantitative Metrics ---" << std::endl;
    std::cout << "Contour error (max): " << quant.norms.linf_contour << " mm" << std::endl;
    std::cout << "Contour error (RMS): " << quant.contourStats.rmsError << " mm" << std::endl;
    std::cout << "Lag error (max): " << quant.following.maxFollowingError << " mm" << std::endl;
    std::cout << "Hausdorff distance: " << quant.shape.hausdorff << " mm" << std::endl;
    std::cout << "Frechet distance: " << quant.shape.frechet << " mm" << std::endl;
    std::cout << "Path length ratio: " << quant.shape.pathLengthRatio << std::endl;
    std::cout << "Surface Ra: " << quant.surface.ra << " µm" << std::endl;
    std::cout << "Surface Rq: " << quant.surface.rq << " µm" << std::endl;
    std::cout << "Max jerk: " << quant.kinematic.jerkActualMax << " mm/s³" << std::endl;
    std::cout << "Smoothness index: " << quant.kinematic.smoothnessIndex << std::endl;

    if (opts.enableFFT) {
        std::cout << "\n--- Spectral Analysis ---" << std::endl;
        std::cout << "Oscillation detected: " << (spectral.oscillationDetected ? "YES" : "NO") << std::endl;
        std::cout << "Oscillation severity: " << spectral.oscillationSeverity << std::endl;
        std::cout << "Spatial contour dominant freq: " << spectral.spatialContour.dominantFrequency << " cyc/mm" << std::endl;
        std::cout << "Temporal contour dominant freq: " << spectral.temporalContour.dominantFrequency << " Hz" << std::endl;
        if (spectral.oscillationDetected) {
            std::cout << "Description: " << spectral.oscillationDescription << std::endl;
        }
    }

    std::cout << "\n--- Qualitative Grades ---" << std::endl;
    std::cout << "Path fidelity:      " << tether::motion::replanner::gradeToString(qual.pathFidelity.grade)
              << " (" << qual.pathFidelity.description << ")" << std::endl;
    std::cout << "Surface finish:     " << tether::motion::replanner::gradeToString(qual.surfaceFinish.grade)
              << " (" << qual.surfaceFinish.description << ")" << std::endl;
    std::cout << "Timing fidelity:    " << tether::motion::replanner::gradeToString(qual.timingFidelity.grade)
              << " (" << qual.timingFidelity.description << ")" << std::endl;
    std::cout << "Smoothness:         " << tether::motion::replanner::gradeToString(qual.smoothness.grade)
              << " (" << qual.smoothness.description << ")" << std::endl;
    if (opts.enableFFT) {
        std::cout << "Oscillation:        " << tether::motion::replanner::gradeToString(qual.oscillationSeverity.grade)
                  << " (" << qual.oscillationSeverity.description << ")" << std::endl;
    }
    std::cout << "Overall:            " << tether::motion::replanner::gradeToString(qual.overall.grade)
              << " (" << qual.overall.description << ")" << std::endl;

    if (!qual.diagnosticMessages.empty()) {
        std::cout << "\n--- Diagnostic Messages ---" << std::endl;
        for (const auto& msg : qual.diagnosticMessages) {
            std::cout << "  - " << msg << std::endl;
        }
    }

    //--- KDE derivative-vs-deviation analysis ---
    tether::motion::replanner::KdeEvaluation kde;
    if (opts.enableKDE) {
        std::cout << "\n--- KDE Derivative-vs-Deviation Analysis ---" << std::endl;

        // Parse derivative axis
        auto parseDerivative = [](const std::string& s) {
            if (s == "velocity") return tether::motion::replanner::DerivativeAxis::Velocity;
            if (s == "acceleration") return tether::motion::replanner::DerivativeAxis::Acceleration;
            if (s == "jerk") return tether::motion::replanner::DerivativeAxis::Jerk;
            if (s == "curvature") return tether::motion::replanner::DerivativeAxis::Curvature;
            if (s == "feedrate") return tether::motion::replanner::DerivativeAxis::FeedRate;
            if (s == "arclength") return tether::motion::replanner::DerivativeAxis::ArcLength;
            if (s == "time") return tether::motion::replanner::DerivativeAxis::Time;
            return tether::motion::replanner::DerivativeAxis::Velocity;
        };
        auto parseDeviation = [](const std::string& s) {
            if (s == "contour") return tether::motion::replanner::DeviationAxis::ContourError;
            if (s == "lag") return tether::motion::replanner::DeviationAxis::LagError;
            if (s == "combined") return tether::motion::replanner::DeviationAxis::CombinedError;
            if (s == "binormal") return tether::motion::replanner::DeviationAxis::BinormalError;
            if (s == "tracking") return tether::motion::replanner::DeviationAxis::TrackingError;
            if (s == "velocity") return tether::motion::replanner::DeviationAxis::VelocityError;
            if (s == "acceleration") return tether::motion::replanner::DeviationAxis::AccelerationError;
            return tether::motion::replanner::DeviationAxis::ContourError;
        };
        auto parseKernel = [](const std::string& s) {
            if (s == "gaussian") return tether::motion::replanner::KernelType::Gaussian;
            if (s == "epanechnikov") return tether::motion::replanner::KernelType::Epanechnikov;
            if (s == "uniform") return tether::motion::replanner::KernelType::Uniform;
            if (s == "triangular") return tether::motion::replanner::KernelType::Triangular;
            if (s == "quartic") return tether::motion::replanner::KernelType::Quartic;
            if (s == "cosine") return tether::motion::replanner::KernelType::Cosine;
            return tether::motion::replanner::KernelType::Gaussian;
        };
        auto parseBandwidth = [](const std::string& s) {
            if (s == "silverman") return tether::motion::replanner::BandwidthMethod::Silverman;
            if (s == "scott") return tether::motion::replanner::BandwidthMethod::Scott;
            if (s == "isj") return tether::motion::replanner::BandwidthMethod::ISJ;
            if (s == "fixed") return tether::motion::replanner::BandwidthMethod::Fixed;
            if (s == "lscv") return tether::motion::replanner::BandwidthMethod::LeastSquaresCV;
            if (s == "likelihoodcv") return tether::motion::replanner::BandwidthMethod::LikelihoodCV;
            return tether::motion::replanner::BandwidthMethod::Silverman;
        };
        auto parseColormap = [](const std::string& s) {
            if (s == "viridis") return tether::motion::replanner::KdeColormap::Viridis;
            if (s == "inferno") return tether::motion::replanner::KdeColormap::Inferno;
            if (s == "plasma") return tether::motion::replanner::KdeColormap::Plasma;
            if (s == "magma") return tether::motion::replanner::KdeColormap::Magma;
            if (s == "jet") return tether::motion::replanner::KdeColormap::Jet;
            if (s == "hot") return tether::motion::replanner::KdeColormap::Hot;
            if (s == "cool") return tether::motion::replanner::KdeColormap::Cool;
            if (s == "grayscale") return tether::motion::replanner::KdeColormap::Grayscale;
            if (s == "bluered") return tether::motion::replanner::KdeColormap::BlueRed;
            return tether::motion::replanner::KdeColormap::Viridis;
        };

        tether::motion::replanner::KdeConfig kdeConfig;
        kdeConfig.derivativeAxis = parseDerivative(opts.kdeDerivative);
        kdeConfig.deviationAxis = parseDeviation(opts.kdeDeviation);
        kdeConfig.kernel = parseKernel(opts.kdeKernel);
        kdeConfig.bandwidthMethod = parseBandwidth(opts.kdeBandwidth);
        kdeConfig.useCertifiedContourError = true;

        std::cout << "Computing KDE: " << opts.kdeDerivative << " vs " << opts.kdeDeviation
                  << " (" << opts.kdeKernel << " kernel, " << opts.kdeBandwidth << " bandwidth)..." << std::endl;

        tether::motion::replanner::KdeDerivativeAnalyzer kdeAnalyzer(kdeConfig);
        kde = kdeAnalyzer.evaluate(desired, actual);

        if (kde.hasSufficientData) {
            std::cout << "Samples: " << kde.derivatives.size() << std::endl;
            std::cout << "Bandwidth: h_x=" << kde.grid.bandwidthX
                      << ", h_y=" << kde.grid.bandwidthY << std::endl;
            std::cout << "Pearson r: " << kde.pearsonCorrelation << std::endl;
            std::cout << "Spearman rho: " << kde.spearmanCorrelation << std::endl;
            std::cout << "Kendall tau: " << kde.kendallTau << std::endl;
            std::cout << "Mutual information: " << kde.mutualInformation << " bits" << std::endl;
            std::cout << "Correlation ratio (eta^2): " << kde.correlationRatio << std::endl;
            std::cout << "Distance correlation: " << kde.distanceCorrelation << std::endl;
            std::cout << "Normalized MI: " << kde.normalizedMutualInfo << std::endl;
            std::cout << "Mode: (" << kde.modeDerivative << ", " << kde.modeDeviation << ")" << std::endl;
            std::cout << "VaR95: " << kde.var95 << " mm" << std::endl;
            std::cout << "CVaR95: " << kde.conditionalVar95 << " mm" << std::endl;

            if (!kde.thresholds.empty()) {
                std::cout << "\nDeviation thresholds:" << std::endl;
                for (const auto& t : kde.thresholds) {
                    if (t.found) {
                        std::cout << "  " << t.description << std::endl;
                    }
                }
            }

            std::cout << "\n" << kde.summary << std::endl;
        } else {
            std::cout << "Insufficient data for KDE analysis (need >= 30 samples, got "
                      << kde.derivatives.size() << ")" << std::endl;
        }
    }

    //--- Export all data ---
    std::cout << "\nExporting results to: " << opts.outputDir << std::endl;
    ExportConfig exportConfig;
    exportConfig.format = (opts.exportFormat == "json") ? ExportFormat::JSONPretty : ExportFormat::CSV;

    BatchExporter exporter(opts.outputDir, exportConfig);
    exporter.exportEvaluationData(desired, actual, quant, spectral, qual);

    // Export KDE data
    if (opts.enableKDE && kde.hasSufficientData) {
        SvgConfig svgConfig;
        // Parse colormap
        auto parseColormap = [](const std::string& s) {
            if (s == "viridis") return tether::motion::replanner::KdeColormap::Viridis;
            if (s == "inferno") return tether::motion::replanner::KdeColormap::Inferno;
            if (s == "plasma") return tether::motion::replanner::KdeColormap::Plasma;
            if (s == "magma") return tether::motion::replanner::KdeColormap::Magma;
            if (s == "jet") return tether::motion::replanner::KdeColormap::Jet;
            if (s == "hot") return tether::motion::replanner::KdeColormap::Hot;
            if (s == "cool") return tether::motion::replanner::KdeColormap::Cool;
            if (s == "grayscale") return tether::motion::replanner::KdeColormap::Grayscale;
            if (s == "bluered") return tether::motion::replanner::KdeColormap::BlueRed;
            return tether::motion::replanner::KdeColormap::Viridis;
        };
        svgConfig.kdeColormap = parseColormap(opts.kdeColormap);
        exporter.exportKdeData(kde, svgConfig);
    }

    exporter.generateManifest("manifest.json");

    std::cout << "Done. " << exporter.generatedFiles().size() << " files generated." << std::endl;

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
        } else if (opts.command == "evaluate") {
            return cmdEvaluate(opts);
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
