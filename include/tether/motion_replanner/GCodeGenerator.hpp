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
     * @param opts Formatting options (precision, comments, ...).
     * @return Single G-code line.
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
     * @param block Block appended to the program.
     */
    void addBlock(const GCodeBlock& block);
    /// @param line Pre-formatted G-code line appended verbatim.
    void addBlock(const std::string& line);
    
    /**
     * @brief Add common commands
     * @param comment Comment text (emitted with the program's comment style).
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
     * @return Program blocks in order.
     */
    const std::vector<GCodeBlock>& blocks() const { return blocks_; }
    /// @return Mutable program blocks.
    std::vector<GCodeBlock>& blocks() { return blocks_; }
    
    /**
     * @brief Export to string
     * @return Complete program text.
     */
    std::string toString() const;
    
    /**
     * @brief Export to file
     * @param filename Destination path.
     * @return true when the file was written.
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Export to stream
     * @param out Stream receiving the program text.
     */
    void write(std::ostream& out) const;
    
    /**
     * @brief Get estimated duration
     * @return Estimated run time [s] from the programmed feeds.
     */
    double estimateDuration() const;
    
    /**
     * @brief Get bounds
     * @return (min XYZ, max XYZ) over all programmed positions.
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
     * @param config Axis, travel range, feed rates and repeats.
     * @return Program exercising one axis.
     */
    GCodeProgram generateSingleAxisTest(const SingleAxisTestConfig& config);
    
    /**
     * @brief Generate multi-axis test pattern
     * @param config Axes, ranges and sequencing options.
     * @return Program exercising several axes.
     */
    GCodeProgram generateMultiAxisTest(const MultiAxisTestConfig& config);
    
    /**
     * @brief Generate circle test
     * @param centerX Circle centre X [mm].
     * @param centerY Circle centre Y [mm].
     * @param centerZ Z height for the circle [mm].
     * @param radius Circle radius [mm].
     * @param feedRate Feed rate [mm/min].
     * @param revolutions Number of laps.
     * @return Circle program.
     */
    GCodeProgram generateCircleTest(double centerX, double centerY, double centerZ,
                                     double radius, double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate ellipse test
     * @param centerX Ellipse centre X [mm].
     * @param centerY Ellipse centre Y [mm].
     * @param centerZ Z height [mm].
     * @param radiusX Semi-axis along X [mm].
     * @param radiusY Semi-axis along Y [mm].
     * @param rotation In-plane rotation [deg].
     * @param feedRate Feed rate [mm/min].
     * @param revolutions Number of laps.
     * @return Ellipse program.
     */
    GCodeProgram generateEllipseTest(double centerX, double centerY, double centerZ,
                                      double radiusX, double radiusY, double rotation,
                                      double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate helix test
     * @param centerX Helix axis X [mm].
     * @param centerY Helix axis Y [mm].
     * @param startZ Starting Z height [mm].
     * @param radius Helix radius [mm].
     * @param pitch Z advance per revolution [mm].
     * @param feedRate Feed rate [mm/min].
     * @param revolutions Number of revolutions.
     * @return Helix program.
     */
    GCodeProgram generateHelixTest(double centerX, double centerY, double startZ,
                                    double radius, double pitch, double feedRate,
                                    int revolutions);
    
    /**
     * @brief Generate sinusoid test
     * @param axis Axis index (0 = X).
     * @param center Centre position [mm].
     * @param amplitude Peak amplitude [mm].
     * @param frequency Spatial frequency [1/mm].
     * @param cycles Number of periods.
     * @param feedRate Feed rate [mm/min].
     * @return Sinusoid program.
     */
    GCodeProgram generateSinusoidTest(int axis, double center, double amplitude,
                                       double frequency, int cycles, double feedRate);
    
    /**
     * @brief Generate ramp test
     * @param axis Axis index (0 = X).
     * @param start Start position [mm].
     * @param end End position [mm].
     * @param feedRate Feed rate [mm/min].
     * @param repeats Number of back-and-forth passes.
     * @return Ramp program.
     */
    GCodeProgram generateRampTest(int axis, double start, double end,
                                   double feedRate, int repeats = 1);
    
    /**
     * @brief Generate square test with corners
     * @param centerX Square centre X [mm].
     * @param centerY Square centre Y [mm].
     * @param centerZ Z height [mm].
     * @param size Edge length [mm].
     * @param feedRate Feed rate [mm/min].
     * @param revolutions Number of laps.
     * @return Square program (sharp corners).
     */
    GCodeProgram generateSquareTest(double centerX, double centerY, double centerZ,
                                     double size, double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate rounded square test
     * @param centerX Square centre X [mm].
     * @param centerY Square centre Y [mm].
     * @param centerZ Z height [mm].
     * @param size Edge length [mm].
     * @param cornerRadius Corner radius [mm].
     * @param feedRate Feed rate [mm/min].
     * @param revolutions Number of laps.
     * @return Rounded-square program.
     */
    GCodeProgram generateRoundedSquareTest(double centerX, double centerY, double centerZ,
                                            double size, double cornerRadius,
                                            double feedRate, int revolutions = 1);
    
    /**
     * @brief Generate friction test sequence
     * @param axis Axis index (0 = X).
     * @param distance Stroke length [mm].
     * @param feedRates Feed rates to sweep [mm/min].
     * @param repeatsPerRate Passes per feed rate.
     * @return Friction-characterisation program.
     */
    GCodeProgram generateFrictionTest(int axis, double distance,
                                       const std::vector<double>& feedRates,
                                       int repeatsPerRate = 3);
    
    /**
     * @brief Generate workspace sweep for heatmap
     * @param heatmapConfig Grid configuration of the workspace.
     * @param testRadius Radius of the pattern at each grid point [mm].
     * @param feedRate Feed rate [mm/min].
     * @return Sweep program covering the configured grid.
     */
    GCodeProgram generateWorkspaceSweep(const HeatmapConfig& heatmapConfig,
                                         double testRadius, double feedRate);
    
    /**
     * @brief Generate complete calibration sequence
     * @param sequence Ordered list of tests to emit.
     * @return Concatenated calibration program.
     */
    GCodeProgram generateCalibrationSequence(const TestSequence& sequence);
    
    /**
     * @brief Configuration
     * @param opts Formatting options for generated programs.
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
     * @param samples Trajectory samples to emit (linear G1 moves).
     * @return Generated program.
     */
    GCodeProgram exportTrajectory(const std::vector<PositionSample>& samples);
    
    /**
     * @brief Export with arc fitting
     * @param samples Trajectory samples to emit.
     * @param arcTolerance Maximum deviation when fitting G2/G3 arcs [mm].
     * @return Generated program (arcs where the fit succeeds).
     */
    GCodeProgram exportTrajectoryWithArcs(const std::vector<PositionSample>& samples,
                                           double arcTolerance = 0.001);
    
    /**
     * @brief Export test results as G-code with annotations
     * @param result Test result whose trajectory is emitted.
     * @return Generated program with per-segment comments.
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
