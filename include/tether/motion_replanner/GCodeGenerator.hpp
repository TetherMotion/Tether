/**
 * @file GCodeGenerator.hpp
 * @brief G-code generation for test patterns and machine calibration
 * 
 * Features:
 * - Generate G-code for single-axis and multi-axis test patterns
 * - Complete G-code export functionality
 * - Support for various machine dialects (LinuxCNC, Fanuc, Mach3, etc.)
 * - Test pattern library for calibration
 */

#pragma once

#include "MachineTester.hpp"
#include <string>
#include <vector>
#include <array>
#include <ostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <functional>

namespace MotionReplanner {

//=============================================================================
// G-Code Dialects
//=============================================================================

/**
 * @brief G-code machine dialect
 */
enum class GCodeDialect {
    LinuxCNC,           ///< LinuxCNC / EMC2
    Fanuc,              ///< Fanuc-style
    Mach3,              ///< Mach3 / Mach4
    Grbl,               ///< Grbl
    Marlin,             ///< Marlin 3D printer
    Haas,               ///< Haas controllers
    Generic             ///< Generic compatible
};

/**
 * @brief G-code generation options
 */
struct GCodeOptions {
    GCodeDialect dialect = GCodeDialect::LinuxCNC;
    
    // Formatting
    int positionPrecision = 4;            ///< Decimal places for position
    int feedPrecision = 1;                ///< Decimal places for feed rate
    bool useLineNumbers = false;
    int lineNumberIncrement = 10;
    bool useParenComments = true;         ///< (comment) vs ; comment
    
    // Units and modes
    bool useMetric = true;                ///< G21 vs G20
    bool absoluteMode = true;             ///< G90 vs G91
    bool useArcMode = true;               ///< Use G2/G3 for arcs
    double arcMinRadius = 0.1;            ///< Min radius to use arc vs linear
    
    // Motion parameters
    double rapidRate = 10000.0;           ///< mm/min
    double defaultFeedRate = 1000.0;      ///< mm/min
    
    // Safety
    double safeZ = 10.0;                  ///< Safe Z height
    bool retractBetweenMoves = true;
    
    // Path tolerance for arc fitting
    double arcTolerance = 0.001;          ///< mm
};

//=============================================================================
// G-Code Block
//=============================================================================

/**
 * @brief A single G-code block (line)
 */
struct GCodeBlock {
    int lineNumber = -1;
    std::string gCode;                    ///< G command (G0, G1, G2, etc.)
    std::string mCode;                    ///< M command
    
    std::array<double, 9> position;       ///< X, Y, Z, A, B, C, U, V, W
    std::array<bool, 9> hasPosition;      ///< Which axes are specified
    
    double feedRate = -1;                 ///< F value
    double spindleSpeed = -1;             ///< S value
    
    // Arc parameters
    double i = 0, j = 0, k = 0;           ///< Arc center offset
    double r = 0;                         ///< Arc radius (alternative)
    bool hasIJK = false;
    bool hasR = false;
    
    std::string comment;
    
    GCodeBlock() {
        position.fill(0);
        hasPosition.fill(false);
    }
    
    /**
     * @brief Convert to string
     */
    std::string toString(const GCodeOptions& opts) const;
};

//=============================================================================
// G-Code Program
//=============================================================================

/**
 * @brief Complete G-code program
 */
class GCodeProgram {
public:
    GCodeProgram(const GCodeOptions& opts = {});
    
    /**
     * @brief Add a block
     */
    void addBlock(const GCodeBlock& block);
    void addBlock(const std::string& line);
    
    /**
     * @brief Add common commands
     */
    void addComment(const std::string& comment);
    void addBlankLine();
    
    void addProgramStart();
    void addProgramEnd();
    
    void addUnitsMetric();
    void addUnitsInch();
    void addAbsoluteMode();
    void addIncrementalMode();
    
    void addRapid(double x, double y, double z);
    void addRapidXY(double x, double y);
    void addRapidZ(double z);
    
    void addLinear(double x, double y, double z, double feedRate = -1);
    void addLinearXY(double x, double y, double feedRate = -1);
    
    void addArcCW(double x, double y, double i, double j, double feedRate = -1);
    void addArcCCW(double x, double y, double i, double j, double feedRate = -1);
    void addArcCW_R(double x, double y, double r, double feedRate = -1);
    void addArcCCW_R(double x, double y, double r, double feedRate = -1);
    
    void addDwell(double seconds);
    void addToolChange(int toolNumber);
    void addSpindleOn(double rpm, bool cw = true);
    void addSpindleOff();
    void addCoolantOn();
    void addCoolantOff();
    
    void addFeedRate(double feedRate);
    
    /**
     * @brief Access blocks
     */
    const std::vector<GCodeBlock>& blocks() const { return blocks_; }
    std::vector<GCodeBlock>& blocks() { return blocks_; }
    
    /**
     * @brief Export to string
     */
    std::string toString() const;
    
    /**
     * @brief Export to file
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Export to stream
     */
    void write(std::ostream& out) const;
    
    /**
     * @brief Get estimated duration
     */
    double estimateDuration() const;
    
    /**
     * @brief Get bounds
     */
    std::pair<std::array<double, 3>, std::array<double, 3>> getBounds() const;
    
private:
    GCodeOptions opts_;
    std::vector<GCodeBlock> blocks_;
    int nextLineNumber_ = 10;
    
    double currentX_ = 0, currentY_ = 0, currentZ_ = 0;
    double currentFeed_ = 0;
};

//=============================================================================
// Test Pattern Generator
//=============================================================================

/**
 * @brief Generates G-code test patterns
 */
class TestPatternGenerator {
public:
    explicit TestPatternGenerator(const GCodeOptions& opts = {});
    
    /**
     * @brief Generate single-axis test pattern
     */
    GCodeProgram generateSingleAxisTest(const SingleAxisTestConfig& config);
    
    /**
     * @brief Generate multi-axis test pattern
     */
    GCodeProgram generateMultiAxisTest(const MultiAxisTestConfig& config);
    
    /**
     * @brief Generate circle test
     */
    GCodeProgram generateCircleTest(double centerX, double centerY, double centerZ,
                                     double radius, double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate ellipse test
     */
    GCodeProgram generateEllipseTest(double centerX, double centerY, double centerZ,
                                      double radiusX, double radiusY, double rotation,
                                      double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate helix test
     */
    GCodeProgram generateHelixTest(double centerX, double centerY, double startZ,
                                    double radius, double pitch, double feedRate,
                                    int revolutions);
    
    /**
     * @brief Generate sinusoid test
     */
    GCodeProgram generateSinusoidTest(int axis, double center, double amplitude,
                                       double frequency, int cycles, double feedRate);
    
    /**
     * @brief Generate ramp test
     */
    GCodeProgram generateRampTest(int axis, double start, double end,
                                   double feedRate, int repeats = 1);
    
    /**
     * @brief Generate square test with corners
     */
    GCodeProgram generateSquareTest(double centerX, double centerY, double centerZ,
                                     double size, double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate rounded square test
     */
    GCodeProgram generateRoundedSquareTest(double centerX, double centerY, double centerZ,
                                            double size, double cornerRadius,
                                            double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate friction test sequence
     */
    GCodeProgram generateFrictionTest(int axis, double distance,
                                       const std::vector<double>& feedRates,
                                       int repeatsPerRate = 3);
    
    /**
     * @brief Generate workspace sweep for heatmap
     */
    GCodeProgram generateWorkspaceSweep(const HeatmapConfig& heatmapConfig,
                                         double testRadius, double feedRate);
    
    /**
     * @brief Generate complete calibration sequence
     */
    GCodeProgram generateCalibrationSequence(const TestSequence& sequence);
    
    /**
     * @brief Configuration
     */
    void setOptions(const GCodeOptions& opts) { opts_ = opts; }
    const GCodeOptions& options() const { return opts_; }
    
private:
    GCodeOptions opts_;
    
    void addCirclePath(GCodeProgram& prog, double cx, double cy, double r,
                       double feed, bool clockwise = false);
    void addEllipsePath(GCodeProgram& prog, double cx, double cy,
                        double rx, double ry, double rotation,
                        double feed, int segments = 72);
    void addSinePath(GCodeProgram& prog, int axis, double center, double amp,
                     double length, double feed, int points);
};

//=============================================================================
// G-Code Exporter
//=============================================================================

/**
 * @brief Export trajectories to G-code
 */
class GCodeExporter {
public:
    explicit GCodeExporter(const GCodeOptions& opts = {});
    
    /**
     * @brief Export position samples to G-code
     */
    GCodeProgram exportTrajectory(const std::vector<PositionSample>& samples);
    
    /**
     * @brief Export with arc fitting
     */
    GCodeProgram exportTrajectoryWithArcs(const std::vector<PositionSample>& samples,
                                           double arcTolerance = 0.001);
    
    /**
     * @brief Export test results as G-code with annotations
     */
    GCodeProgram exportTestResult(const TestResult& result);
    
    void setOptions(const GCodeOptions& opts) { opts_ = opts; }
    
private:
    GCodeOptions opts_;
    
    bool fitArc(const std::vector<PositionSample>& samples, size_t start, size_t& end,
                double& cx, double& cy, double& r, bool& clockwise);
};

//=============================================================================
// Inline Implementations
//=============================================================================

inline std::string GCodeBlock::toString(const GCodeOptions& opts) const {
    std::ostringstream ss;
    ss << std::fixed;
    
    if (lineNumber >= 0 && opts.useLineNumbers) {
        ss << "N" << lineNumber << " ";
    }
    
    if (!gCode.empty()) {
        ss << gCode;
    }
    
    if (!mCode.empty()) {
        if (!gCode.empty()) ss << " ";
        ss << mCode;
    }
    
    const char* axisNames = "XYZABCUVW";
    for (int i = 0; i < 9; ++i) {
        if (hasPosition[i]) {
            ss << " " << axisNames[i] << std::setprecision(opts.positionPrecision) 
               << position[i];
        }
    }
    
    if (hasIJK) {
        ss << std::setprecision(opts.positionPrecision);
        if (std::abs(i) > 1e-9 || std::abs(j) > 1e-9) {
            ss << " I" << i << " J" << j;
        }
        if (std::abs(k) > 1e-9) {
            ss << " K" << k;
        }
    }
    
    if (hasR) {
        ss << " R" << std::setprecision(opts.positionPrecision) << r;
    }
    
    if (feedRate > 0) {
        ss << " F" << std::setprecision(opts.feedPrecision) << feedRate;
    }
    
    if (spindleSpeed > 0) {
        ss << " S" << static_cast<int>(spindleSpeed);
    }
    
    if (!comment.empty()) {
        if (opts.useParenComments) {
            ss << " (" << comment << ")";
        } else {
            ss << " ; " << comment;
        }
    }
    
    return ss.str();
}

} // namespace MotionReplanner
