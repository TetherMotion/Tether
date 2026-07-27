/**
 * @file GCodeAdapter.hpp
 * @brief Integration with Existing G-Code Parser
 *
 * @details
 * This file provides the adapter layer between the existing G-code parser
 * and the new motion planner system. It converts parsed G-code commands
 * into MotionSegments with full traceability.
 *
 * ## Integration
 *
 * The adapter:
 * 1. Takes parsed G-code from the existing GCode::Parser
 * 2. Converts commands to MotionSegments
 * 3. Preserves line number references for traceability
 * 4. Handles modal state (G17/18/19 plane selection, G90/91 absolute/incremental)
 *
 * @see GCodeParser.hpp (existing)
 * @see MotionSegment.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "MotionSegment.hpp"
#include "SourceReference.hpp"
#include "MotionPlan.hpp"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <optional>
#include <sstream>
#include <iostream>

namespace MotionPlanner {

// ============================================================================
// G-Code Modal State
// ============================================================================

/**
 * @brief Active plane for arc interpretation
 */
enum class Plane : uint8_t {
    XY = 17,  // G17
    XZ = 18,  // G18
    YZ = 19   // G19
};

/**
 * @brief Distance mode
 */
enum class DistanceMode : uint8_t {
    Absolute = 90,     // G90
    Incremental = 91   // G91
};

/**
 * @brief Feed rate mode
 */
enum class FeedMode : uint8_t {
    UnitsPerMinute = 94,  // G94
    InverseTime = 93      // G93
};

/**
 * @brief Motion mode
 */
enum class MotionMode : uint8_t {
    Rapid = 0,          // G0
    Linear = 1,         // G1
    ArcCW = 2,          // G2
    ArcCCW = 3,         // G3
    Dwell = 4,          // G4
    SplineCubic = 5,    // G5 (if supported)
    SplineQuintic = 6   // G5.1 (if supported)
};

/**
 * @brief G-code modal state
 */
struct GCodeModalState {
    Plane activePlane = Plane::XY;
    DistanceMode distanceMode = DistanceMode::Absolute;
    FeedMode feedMode = FeedMode::UnitsPerMinute;
    MotionMode motionMode = MotionMode::Rapid;
    
    /// Current position (all axes)
    std::array<double, MAX_MOTION_AXES> currentPosition{};
    
    /// Current feed rate (units/min)
    double feedRate = 100.0;
    
    /// Current spindle speed
    double spindleSpeed = 0.0;
    
    /// Current tool number
    int currentTool = 0;
    
    /// Units: true = metric (mm), false = imperial (inches)
    bool isMetric = true;
};

// ============================================================================
// Parsed G-Code Command
// ============================================================================

/**
 * @brief Represents a parsed G-code command
 *
 * This is a generic representation that can be populated from
 * different G-code parsers.
 */
struct ParsedGCodeCommand {
    /// Line number in source file (1-based)
    size_t lineNumber = 0;
    
    /// Original line text
    std::string lineText;
    
    /// G command number (-1 if none)
    int gCode = -1;
    
    /// M command number (-1 if none)
    int mCode = -1;
    
    /// Axis coordinates (NaN if not specified)
    std::array<double, MAX_MOTION_AXES> coordinates;
    
    /// Arc center offsets (I, J, K) - relative to start point
    std::array<double, 3> arcOffsets{0, 0, 0};
    
    /// Arc radius (R word) - alternative to I, J, K
    std::optional<double> arcRadius;
    
    /// Feed rate (F word)
    std::optional<double> feedRate;
    
    /// Spindle speed (S word)
    std::optional<double> spindleSpeed;
    
    /// Tool number (T word)
    std::optional<int> toolNumber;
    
    /// Dwell time in seconds (P word)
    std::optional<double> dwellTime;
    
    /// N line number (from G-code)
    std::optional<int> nLineNumber;
    
    /// Is this a comment-only line?
    bool isComment = false;
    
    /// Comment text
    std::string comment;
    
    /// BSPLINE/NURBS parameters (Siemens/Heidenhain)
    std::vector<double> bsplineKnots;    ///< KNOT= values
    std::vector<double> bsplinePoles;    ///< POLES= values (flat: x0,y0,z0,x1,y1,z1,...)
    std::vector<double> bsplineWeights;  ///< WEIGHTS= values
    size_t bsplineDegree = 0;            ///< DEGREE= value
    bool isBSPLINE = false;              ///< Siemens BSPLINE command
    bool isNURBSCmd = false;             ///< Heidenhain NURBS command
    
    ParsedGCodeCommand() {
        coordinates.fill(std::nan(""));
    }
    
    /**
     * @brief Check if coordinate is specified
     */
    bool hasCoordinate(size_t axis) const {
        return axis < coordinates.size() && !std::isnan(coordinates[axis]);
    }
    
    /**
     * @brief Check if any coordinate is specified
     */
    bool hasAnyCoordinate() const {
        for (size_t i = 0; i < coordinates.size(); ++i) {
            if (hasCoordinate(i)) return true;
        }
        return false;
    }
};

// ============================================================================
// G-Code to Motion Segment Converter
// ============================================================================

/**
 * @brief Converts parsed G-code commands to motion segments
 */
class GCodeToMotionConverter {
public:
    /**
     * @brief Constructor
     *
     * @param sourceFile Source file info for traceability
     */
    explicit GCodeToMotionConverter(std::shared_ptr<SourceFile> sourceFile = nullptr)
        : sourceFile_(sourceFile ? std::move(sourceFile) : 
                      std::make_shared<SourceFile>("<unknown>")) {}

    /**
     * @brief Process a single G-code command
     *
     * @param cmd Parsed G-code command
     * @return Optional motion segment (empty for non-motion commands)
     */
    std::optional<MotionSegment> processCommand(const ParsedGCodeCommand& cmd) {
        // Create source reference
        SourceReference sourceRef = SourceReference::fromLine(
            cmd.lineNumber, sourceFile_);
        
        // Update modal state from command
        updateModalState(cmd);
        
        // Handle motion commands
        if (cmd.gCode >= 0) {
            switch (cmd.gCode) {
                case 0:  // Rapid
                    return processRapid(cmd, sourceRef);
                    
                case 1:  // Linear
                    return processLinear(cmd, sourceRef);
                    
                case 2:  // Arc CW
                    return processArc(cmd, true, sourceRef);
                    
                case 3:  // Arc CCW
                    return processArc(cmd, false, sourceRef);
                    
                case 4:  // Dwell
                    return processDwell(cmd, sourceRef);
                    
                case 17: case 18: case 19:  // Plane select
                case 90: case 91:            // Distance mode
                case 93: case 94:            // Feed mode
                case 20: case 21:            // Units
                    // Modal state updates - no motion segment
                    return std::nullopt;
                    
                default:
                    // Unknown G-code
                    return std::nullopt;
            }
        }
        
        // Handle BSPLINE/NURBS commands (Siemens/Heidenhain)
        if (cmd.isBSPLINE || cmd.isNURBSCmd) {
            return processBSPLINEorNURBS(cmd, sourceRef);
        }
        
        // Handle modal motion (coordinates without G-code)
        if (cmd.hasAnyCoordinate() && cmd.gCode < 0) {
            switch (state_.motionMode) {
                case MotionMode::Rapid:
                    return processRapid(cmd, sourceRef);
                case MotionMode::Linear:
                    return processLinear(cmd, sourceRef);
                case MotionMode::ArcCW:
                    return processArc(cmd, true, sourceRef);
                case MotionMode::ArcCCW:
                    return processArc(cmd, false, sourceRef);
                default:
                    return std::nullopt;
            }
        }
        
        return std::nullopt;
    }

    /**
     * @brief Process a sequence of commands
     *
     * @param commands Parsed G-code commands
     * @return Motion segment list
     */
    MotionSegmentList processCommands(const std::vector<ParsedGCodeCommand>& commands) {
        MotionSegmentList segments;
        
        for (const auto& cmd : commands) {
            auto segment = processCommand(cmd);
            if (segment) {
                // Debug: print segment positions
                std::cerr << "Segment: start=" << segment->startPosition[0] << "," << segment->startPosition[1]
                          << " end=" << segment->endPosition[0] << "," << segment->endPosition[1] << "\n";
                segments.append(std::move(*segment));
            }
        }
        
        return segments;
    }

    /**
     * @brief Reset modal state to defaults
     */
    void reset() {
        state_ = GCodeModalState{};
    }

    /**
     * @brief Access modal state
     */
    const GCodeModalState& state() const { return state_; }
    GCodeModalState& state() { return state_; }

    /**
     * @brief Set source file
     */
    void setSourceFile(std::shared_ptr<SourceFile> file) {
        sourceFile_ = std::move(file);
    }

private:
    /**
     * @brief Update modal state from command
     */
    void updateModalState(const ParsedGCodeCommand& cmd) {
        // Feed rate
        if (cmd.feedRate) {
            state_.feedRate = *cmd.feedRate;
        }
        
        // Spindle speed
        if (cmd.spindleSpeed) {
            state_.spindleSpeed = *cmd.spindleSpeed;
        }
        
        // Tool
        if (cmd.toolNumber) {
            state_.currentTool = *cmd.toolNumber;
        }
        
        // G-code modal groups
        if (cmd.gCode >= 0) {
            switch (cmd.gCode) {
                case 0: state_.motionMode = MotionMode::Rapid; break;
                case 1: state_.motionMode = MotionMode::Linear; break;
                case 2: state_.motionMode = MotionMode::ArcCW; break;
                case 3: state_.motionMode = MotionMode::ArcCCW; break;
                
                case 17: state_.activePlane = Plane::XY; break;
                case 18: state_.activePlane = Plane::XZ; break;
                case 19: state_.activePlane = Plane::YZ; break;
                
                case 90: state_.distanceMode = DistanceMode::Absolute; break;
                case 91: state_.distanceMode = DistanceMode::Incremental; break;
                
                case 93: state_.feedMode = FeedMode::InverseTime; break;
                case 94: state_.feedMode = FeedMode::UnitsPerMinute; break;
                
                case 20: state_.isMetric = false; break;
                case 21: state_.isMetric = true; break;
            }
        }
    }

    /**
     * @brief Compute target position from command
     */
    std::array<double, MAX_MOTION_AXES> computeTargetPosition(const ParsedGCodeCommand& cmd) {
        std::array<double, MAX_MOTION_AXES> target = state_.currentPosition;
        
        for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            if (cmd.hasCoordinate(i)) {
                if (state_.distanceMode == DistanceMode::Absolute) {
                    target[i] = cmd.coordinates[i];
                } else {
                    target[i] = state_.currentPosition[i] + cmd.coordinates[i];
                }
            }
        }
        
        return target;
    }

    /**
     * @brief Process G0 (rapid move)
     */
    std::optional<MotionSegment> processRapid(const ParsedGCodeCommand& cmd,
                                               const SourceReference& sourceRef) {
        auto target = computeTargetPosition(cmd);
        
        // Check for actual movement
        bool hasMovement = false;
        for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            if (std::abs(target[i] - state_.currentPosition[i]) > 1e-9) {
                hasMovement = true;
                break;
            }
        }
        
        if (!hasMovement) {
            return std::nullopt;
        }
        
        auto segment = MotionSegment::rapid(state_.currentPosition, target);
        segment.sourceRef = sourceRef;
        
        state_.currentPosition = target;
        return segment;
    }

    /**
     * @brief Process G1 (linear move)
     */
    std::optional<MotionSegment> processLinear(const ParsedGCodeCommand& cmd,
                                                const SourceReference& sourceRef) {
        auto target = computeTargetPosition(cmd);
        
        // Check for actual movement
        bool hasMovement = false;
        double distance = 0.0;
        for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            double d = target[i] - state_.currentPosition[i];
            if (std::abs(d) > 1e-9) {
                hasMovement = true;
            }
            distance += d * d;
        }
        distance = std::sqrt(distance);
        
        if (!hasMovement) {
            return std::nullopt;
        }
        
        // Convert feed rate from units/min to units/sec
        double feedRatePerSec = state_.feedRate / 60.0;
        
        auto segment = MotionSegment::linear(state_.currentPosition, target, feedRatePerSec);
        segment.sourceRef = sourceRef;
        
        state_.currentPosition = target;
        return segment;
    }

    /**
     * @brief Process G2/G3 (arc move)
     */
    std::optional<MotionSegment> processArc(const ParsedGCodeCommand& cmd,
                                             bool clockwise,
                                             const SourceReference& sourceRef) {
        auto target = computeTargetPosition(cmd);
        
        // Get arc plane indices
        size_t planeAxis1, planeAxis2, normalAxis;
        switch (state_.activePlane) {
            case Plane::XY: planeAxis1 = 0; planeAxis2 = 1; normalAxis = 2; break;
            case Plane::XZ: planeAxis1 = 0; planeAxis2 = 2; normalAxis = 1; break;
            case Plane::YZ: planeAxis1 = 1; planeAxis2 = 2; normalAxis = 0; break;
        }
        
        // Compute arc center
        std::array<double, MAX_MOTION_AXES> center = state_.currentPosition;
        double radius;
        
        if (cmd.arcRadius) {
            // R format - compute center from radius
            radius = *cmd.arcRadius;
            
            // Midpoint between start and end
            double mx = (state_.currentPosition[planeAxis1] + target[planeAxis1]) / 2.0;
            double my = (state_.currentPosition[planeAxis2] + target[planeAxis2]) / 2.0;
            
            // Distance from start to midpoint
            double dx = target[planeAxis1] - state_.currentPosition[planeAxis1];
            double dy = target[planeAxis2] - state_.currentPosition[planeAxis2];
            double halfChord = std::sqrt(dx*dx + dy*dy) / 2.0;
            
            if (halfChord > std::abs(radius)) {
                // Invalid arc - chord longer than diameter
                return std::nullopt;
            }
            
            // Height of arc from chord to center
            double h = std::sqrt(radius*radius - halfChord*halfChord);
            
            // Perpendicular direction
            double len = std::sqrt(dx*dx + dy*dy);
            double px = -dy / len;
            double py = dx / len;
            
            // Direction to center depends on CW/CCW and radius sign
            bool flip = (clockwise != (radius > 0));
            if (flip) {
                h = -h;
            }
            
            center[planeAxis1] = mx + px * h;
            center[planeAxis2] = my + py * h;
            radius = std::abs(radius);
        } else {
            // I, J, K format - center is offset from start
            center[planeAxis1] = state_.currentPosition[planeAxis1] + cmd.arcOffsets[0];
            center[planeAxis2] = state_.currentPosition[planeAxis2] + cmd.arcOffsets[1];
            
            // Compute radius from center to start
            double dx = state_.currentPosition[planeAxis1] - center[planeAxis1];
            double dy = state_.currentPosition[planeAxis2] - center[planeAxis2];
            radius = std::sqrt(dx*dx + dy*dy);
        }
        
        // Convert feed rate
        double feedRatePerSec = state_.feedRate / 60.0;
        
        // Create segment
        MotionSegment segment;
        if (clockwise) {
            segment = MotionSegment::arcCW(state_.currentPosition, target, center, 
                                           radius, feedRatePerSec);
        } else {
            segment = MotionSegment::arcCCW(state_.currentPosition, target, center,
                                            radius, feedRatePerSec);
        }
        segment.sourceRef = sourceRef;
        
        state_.currentPosition = target;
        return segment;
    }

    /**
     * @brief Process G4 (dwell)
     */
    std::optional<MotionSegment> processDwell(const ParsedGCodeCommand& cmd,
                                               const SourceReference& sourceRef) {
        double duration = cmd.dwellTime.value_or(0.0);
        
        if (duration <= 0.0) {
            return std::nullopt;
        }
        
        auto segment = MotionSegment::dwell(state_.currentPosition, duration);
        segment.sourceRef = sourceRef;
        
        return segment;
    }

    /**
     * @brief Process BSPLINE (Siemens) or NURBS (Heidenhain) commands
     *
     * Siemens format:  BSPLINE X10 Y20 Z5 KNOT=0,0,0,1,2,2,2 POLES=0,0,0,5,10,2,10,20,5 WEIGHTS=1,1,1 DEGREE=2 F1000
     * Heidenhain format: NURBS X10 Y20 Z5 KNOT=0,0,0,1,2,2,2 POLES=... WEIGHTS=... DEGREE=2 F1000
     */
    std::optional<MotionSegment> processBSPLINEorNURBS(const ParsedGCodeCommand& cmd,
                                                         const SourceReference& sourceRef) {
        // Validate required fields
        if (cmd.bsplinePoles.empty() || cmd.bsplineDegree == 0) {
            return std::nullopt;
        }

        size_t degree = cmd.bsplineDegree;

        // Parse poles - flat array of coordinates (x0,y0[,z0],x1,y1[,z1],...)
        // Detect dimensionality from end position or default to MAX_MOTION_AXES
        size_t dim = MAX_MOTION_AXES;
        size_t numPoles = cmd.bsplinePoles.size() / dim;
        if (cmd.bsplinePoles.size() % dim != 0) {
            // Try 2D
            dim = 2;
            numPoles = cmd.bsplinePoles.size() / dim;
            if (cmd.bsplinePoles.size() % dim != 0) {
                return std::nullopt; // Invalid pole data
            }
        }

        // Build pole position arrays
        std::vector<std::array<double, MAX_MOTION_AXES>> poles(numPoles);
        for (size_t i = 0; i < numPoles; ++i) {
            poles[i].fill(0.0);
            for (size_t d = 0; d < dim; ++d) {
                poles[i][d] = cmd.bsplinePoles[i * dim + d];
            }
        }

        // Knots
        std::vector<double> knots = cmd.bsplineKnots;
        if (knots.empty()) {
            // Generate uniform clamped knot vector
            size_t n = numPoles;
            size_t numKnots = n + degree + 1;
            knots.resize(numKnots);
            for (size_t i = 0; i <= degree; ++i) {
                knots[i] = 0.0;
                knots[numKnots - 1 - i] = 1.0;
            }
            size_t numInternal = numKnots - 2 * (degree + 1);
            for (size_t i = 0; i < numInternal; ++i) {
                knots[degree + 1 + i] = static_cast<double>(i + 1) / static_cast<double>(numInternal + 1);
            }
        }

        // Weights
        std::vector<double> weights = cmd.bsplineWeights;
        if (weights.empty()) {
            weights.assign(numPoles, 1.0);
        }

        // Validate: n + p + 1 == knots.size()
        if (knots.size() != numPoles + degree + 1) {
            return std::nullopt;
        }

        // Start and end positions
        std::array<double, MAX_MOTION_AXES> startPos = state_.currentPosition;
        std::array<double, MAX_MOTION_AXES> endPos;
        endPos.fill(0.0);
        for (size_t d = 0; d < dim && d < MAX_MOTION_AXES; ++d) {
            endPos[d] = poles.back()[d];
        }

        // Feed rate
        double feedRatePerSec = state_.feedRate / 60.0;
        if (cmd.feedRate) {
            feedRatePerSec = *cmd.feedRate / 60.0;
            state_.feedRate = *cmd.feedRate;
        }

        auto segment = MotionSegment::nurbs(
            startPos, endPos, poles, weights, knots, degree, feedRatePerSec);
        segment.sourceRef = sourceRef;

        state_.currentPosition = endPos;
        return segment;
    }

    /**
     * @brief Parse a comma-separated list of doubles from a parameter string
     */
    static std::vector<double> parseDoubleList(const std::string& str) {
        std::vector<double> result;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                result.push_back(std::stod(token));
            } catch (...) {
                // Skip invalid tokens
            }
        }
        return result;
    }

    /**
     * @brief Parse a BSPLINE or NURBS G-code line into a ParsedGCodeCommand
     *
     * Handles lines like:
     *   BSPLINE X10 Y20 Z5 KNOT=0,0,0,1,2,2,2 POLES=0,0,0,5,10,2,10,20,5 WEIGHTS=1,1,1 DEGREE=2 F1000
     *   NURBS X10 Y20 Z5 KNOT=0,0,0,1,2,2,2 POLES=0,0,0,5,10,2,10,20,5 WEIGHTS=1,1,1 DEGREE=2 F1000
     */
    static ParsedGCodeCommand parseBSPLINELine(const std::string& line, size_t lineNumber = 0) {
        ParsedGCodeCommand cmd;
        cmd.lineNumber = lineNumber;
        cmd.lineText = line;

        // Determine command type
        std::string upper = line;
        for (auto& c : upper) c = std::toupper(c);

        if (upper.find("BSPLINE") == 0 || upper.find(" BSPLINE") != std::string::npos) {
            cmd.isBSPLINE = true;
        } else if (upper.find("NURBS") == 0 || upper.find(" NURBS") != std::string::npos) {
            cmd.isNURBSCmd = true;
        } else {
            return cmd;
        }

        // Tokenize
        std::istringstream iss(line);
        std::string token;
        while (iss >> token) {
            std::string upperToken = token;
            for (auto& c : upperToken) c = std::toupper(c);

            if (upperToken == "BSPLINE" || upperToken == "NURBS") {
                continue;
            }

            // Check for parameter assignments (KNOT=..., POLES=..., etc.)
            auto eqPos = upperToken.find('=');
            if (eqPos != std::string::npos) {
                std::string key = upperToken.substr(0, eqPos);
                std::string value = token.substr(eqPos + 1);
                
                if (key == "KNOT") {
                    cmd.bsplineKnots = parseDoubleList(value);
                } else if (key == "POLES") {
                    cmd.bsplinePoles = parseDoubleList(value);
                } else if (key == "WEIGHTS") {
                    cmd.bsplineWeights = parseDoubleList(value);
                } else if (key == "DEGREE") {
                    try { cmd.bsplineDegree = std::stoul(value); } catch (...) {}
                }
                continue;
            }

            // Check for axis words (X, Y, Z, A, B, C)
            if (upperToken.size() > 1) {
                char axis = upperToken[0];
                std::string valStr = token.substr(1);
                try {
                    double val = std::stod(valStr);
                    switch (axis) {
                        case 'X': cmd.coordinates[0] = val; break;
                        case 'Y': cmd.coordinates[1] = val; break;
                        case 'Z': cmd.coordinates[2] = val; break;
                        case 'A': if (MAX_MOTION_AXES > 3) cmd.coordinates[3] = val; break;
                        case 'B': if (MAX_MOTION_AXES > 4) cmd.coordinates[4] = val; break;
                        case 'C': if (MAX_MOTION_AXES > 5) cmd.coordinates[5] = val; break;
                        case 'F': cmd.feedRate = val; break;
                        default: break;
                    }
                } catch (...) {}
            }
        }

        return cmd;
    }

    GCodeModalState state_;
    std::shared_ptr<SourceFile> sourceFile_;
};

// ============================================================================
// G-Code to Motion Plan
// ============================================================================

/**
 * @brief High-level interface to convert G-code to motion plan
 */
template<size_t Dim, typename T = double>
class GCodeToMotionPlan {
public:
    using Plan = MotionPlan<Dim, T>;
    using Builder = MotionPlanBuilder<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;
    using Config = MotionPlanConfig<T>;

    /**
     * @brief Constructor
     */
    GCodeToMotionPlan(Limits limits = {}, Config config = {})
        : builder_(limits, config) {}

    /**
     * @brief Build motion plan from G-code commands
     *
     * @param commands Parsed G-code commands
     * @param sourceFile Source file info for traceability
     * @return Complete motion plan
     */
    Plan build(const std::vector<ParsedGCodeCommand>& commands,
               std::shared_ptr<SourceFile> sourceFile = nullptr) {
        GCodeToMotionConverter converter(sourceFile);
        MotionSegmentList segments = converter.processCommands(commands);
        
        // Get default feed rate from modal state
        T feedRate = static_cast<T>(converter.state().feedRate / 60.0);
        
        return builder_.build(segments, feedRate);
    }

    /**
     * @brief Access builder for configuration
     */
    Builder& builder() { return builder_; }
    const Builder& builder() const { return builder_; }

private:
    Builder builder_;
};

// ============================================================================
// Type Aliases
// ============================================================================

using GCodeToMotionPlan2D = GCodeToMotionPlan<2, double>;
using GCodeToMotionPlan3D = GCodeToMotionPlan<3, double>;

}  // namespace MotionPlanner
