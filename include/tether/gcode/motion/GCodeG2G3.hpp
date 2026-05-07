/**
 * @file GCodeG2G3.hpp
 * @brief G2 (CW Arc) and G3 (CCW Arc) with Helical Interpolation
 * 
 * @details
 * ## G2 - Clockwise Arc (Modal Group 1)
 * ## G3 - Counter-Clockwise Arc (Modal Group 1)
 * 
 * These commands produce circular arcs in the selected plane (G17/G18/G19).
 * Adding a linear axis motion creates helical interpolation.
 * 
 * ### Arc Direction
 * 
 * Arc direction is defined when looking at the plane from the positive
 * perpendicular axis:
 * 
 * | Plane | View From | CW (G2) | CCW (G3) |
 * |-------|-----------|---------|----------|
 * | G17 (XY) | +Z | Clockwise | Counter-clockwise |
 * | G18 (XZ) | +Y | Clockwise | Counter-clockwise |
 * | G19 (YZ) | +X | Clockwise | Counter-clockwise |
 * 
 * ### Format 1: Center (I, J, K)
 * 
 * ```gcode
 * G2 X__ Y__ I__ J__ F__    ; XY plane
 * G2 X__ Z__ I__ K__ F__    ; XZ plane
 * G2 Y__ Z__ J__ K__ F__    ; YZ plane
 * ```
 * 
 * I, J, K are **incremental offsets** from the start point to the center:
 * ```
 *     End (X, Y)
 *       \
 *        \  Arc
 *         \
 *   Center (Start + I, Start + J)
 *          |
 *    Start (current)
 * ```
 * 
 * ⚠️ **Important**: I, J, K are ALWAYS incremental from start, regardless
 * of G90/G91 mode. This is LinuxCNC behavior. Some controls use absolute
 * center mode—configure via `absoluteIJK` option.
 * 
 * ### Format 2: Radius (R)
 * 
 * ```gcode
 * G2 X__ Y__ R__ F__
 * ```
 * 
 * - R > 0: Arc ≤ 180° (minor arc)
 * - R < 0: Arc > 180° (major arc)
 * 
 * ```
 * Minor arc (R > 0)        Major arc (R < 0)
 *                          
 *     End                      End
 *    /                        /
 *   (  ← Arc                 (    ← Arc (goes the long way)
 *    \                        \________/
 *   Start                     Start
 * ```
 * 
 * **Limitation**: R format cannot specify full circles (360°).
 * Use I, J, K format for full circles.
 * 
 * ### Full Circles
 * 
 * For a full circle, end point equals start point:
 * ```gcode
 * ; Full circle centered at (0, 0), starting at (10, 0)
 * G0 X10 Y0
 * G2 I-10 J0 F500          ; X, Y omitted = same as start
 * 
 * ; Or explicitly:
 * G2 X10 Y0 I-10 J0 F500   ; End = Start
 * ```
 * 
 * ### Helical Interpolation
 * 
 * Adding perpendicular axis motion creates a helix:
 * 
 * ```gcode
 * ; XY plane helix with Z descent
 * G17
 * G3 X10 Y0 Z-5 I-5 J0 F500
 * 
 * ; This creates a counterclockwise helix:
 * ;   - Circular motion in XY
 * ;   - Linear descent in Z
 * ;   - Combined = 3D helix
 * ```
 * 
 * Multi-turn helix (P word specifies extra turns):
 * ```gcode
 * G3 X10 Y0 Z-10 I-5 J0 P3 F500  ; 3 full turns + final arc
 * ```
 * 
 * ### Arc Parameters
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | X, Y, Z | End position |
 * | I | X offset to center |
 * | J | Y offset to center |
 * | K | Z offset to center |
 * | R | Radius (alternative to IJK) |
 * | P | Number of full turns (helix) |
 * | F | Feed rate |
 * 
 * ### Arc Feed Rate
 * 
 * Feed rate applies to the arc path length:
 * - For arc only: F = tangential speed along arc
 * - For helix: F = speed along the 3D helical path
 * 
 * Total helix length:
 * ```
 * L = sqrt(arc_length² + linear_length²)
 * ```
 * 
 * ### Plane Selection
 * 
 * | Active Plane | Arc Axes | Linear Axis | Center |
 * |--------------|----------|-------------|--------|
 * | G17 (XY) | X, Y | Z | I, J |
 * | G18 (XZ) | X, Z | Y | I, K |
 * | G19 (YZ) | Y, Z | X | J, K |
 * 
 * ### Arc Tolerance
 * 
 * LinuxCNC checks that the end point lies on the arc within tolerance.
 * If |radius_start - radius_end| > tolerance, error is raised.
 * 
 * Configure tolerance via `arcRadiusTolerance` (default 0.005mm).
 * 
 * ### Small Arcs and Linear Approximation
 * 
 * Very small arcs (< minArcRadius) may be converted to linear moves
 * to avoid numerical issues.
 * 
 * ### Multi-Turn Arcs (P Word)
 * 
 * ```gcode
 * ; 5 full circles plus arc to endpoint
 * G2 X10 Y0 I-5 J0 P5 F500
 * 
 * ; P must be ≥ 1 when specified
 * ; P0 is same as omitting P (single arc)
 * ```
 * 
 * For helical boring/milling:
 * ```gcode
 * ; Helical plunge into material
 * G0 X10 Y0 Z5           ; Position above center
 * G3 Z-10 I-5 J0 P10 F200 ; 10-turn helix, 15mm total descent
 * ```
 * 
 * ### Examples
 * 
 * ```gcode
 * ; Quarter circle CW
 * G0 X0 Y0
 * G2 X10 Y10 I10 J0 F500
 * 
 * ; Semicircle CCW
 * G0 X20 Y0
 * G3 X0 Y0 R10 F500
 * 
 * ; Full circle using IJK
 * G0 X10 Y0
 * G2 I-10 F500
 * 
 * ; 3D helix: 3 turns descending
 * G0 X10 Y0 Z0
 * G3 Z-15 I-10 J0 P3 F300
 * 
 * ; Thread milling (helical with continuous descent)
 * #<pitch> = 1.5
 * #<depth> = -20
 * #<turns> = [ABS[#<depth>] / #<pitch>]
 * G3 Z#<depth> I-5 J0 P#<turns> F100
 * ```
 * 
 * @see GCodeG0G1 for linear motion
 * @see GCodeG5 for spline interpolation
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <cmath>
#include <vector>
#include <optional>

namespace GCode {

// Forward declarations
class MachineState;
class MotionPlanner;
class VariableSystem;

// ============================================================================
// Arc Configuration
// ============================================================================

/**
 * @brief Configuration for arc motion
 */
struct ArcMotionConfig {
    // === Arc Geometry ===
    
    /// Minimum arc radius (below this, convert to line)
    double minArcRadius{0.001};  // mm
    
    /// Maximum arc radius
    double maxArcRadius{100000.0};  // mm
    
    /// Arc endpoint radius tolerance
    double arcRadiusTolerance{0.005};  // mm
    
    /// Use absolute I, J, K (Fanuc style) vs incremental (LinuxCNC)
    bool absoluteIJK{false};
    
    /// Allow arcs > 360° with P word
    bool allowMultiTurn{true};
    
    /// Maximum turns for multi-turn arc
    int32_t maxTurns{100};
    
    // === Arc Discretization ===
    
    /// Maximum arc segment length (for output)
    double maxArcSegmentLength{1.0};  // mm
    
    /// Maximum deviation for arc linearization
    double maxArcDeviation{0.01};  // mm
    
    /// Minimum points per arc
    int32_t minArcPoints{8};
    
    /// Maximum points per arc
    int32_t maxArcPoints{10000};
    
    // === Helix ===
    
    /// Enable helical interpolation
    bool enableHelix{true};
    
    /// Maximum helix angle (from plane)
    double maxHelixAngle{89.0};  // degrees
    
    // === Full Circle Detection ===
    
    /// Tolerance for full circle detection
    double fullCircleTolerance{0.0001};  // mm
    
    // === Feed Rate ===
    
    /// Feed rate applies to arc path (true) or chord (false)
    bool feedAlongArc{true};
    
    /// Maximum tangential acceleration
    double maxTangentialAccel{1000.0};  // mm/s²
    
    /// Maximum centripetal acceleration (limits speed on tight arcs)
    double maxCentripetalAccel{2000.0};  // mm/s²
};

// ============================================================================
// Arc Geometry Types
// ============================================================================

/**
 * @brief Arc direction
 */
enum class ArcDirection {
    CW,     ///< Clockwise (G2)
    CCW     ///< Counter-clockwise (G3)
};

/**
 * @brief Arc specification
 */
struct ArcSpec {
    /// Start position (full 9-axis)
    Position start;
    
    /// End position (full 9-axis)
    Position end;
    
    /// Center position (in arc plane)
    double centerX{0};
    double centerY{0};
    
    /// Radius
    double radius{0};
    
    /// Start angle (radians, from center)
    double startAngle{0};
    
    /// End angle (radians)
    double endAngle{0};
    
    /// Swept angle (may be > 2π for multi-turn)
    double sweepAngle{0};
    
    /// Direction
    ArcDirection direction{ArcDirection::CW};
    
    /// Active plane
    Plane plane{Plane::XY};
    
    /// Helix linear axis delta
    double helixDelta{0};
    
    /// Number of full turns (P word)
    int32_t turns{0};
    
    /// Arc length (2D)
    double arcLength{0};
    
    /// Total path length (including helix)
    double totalLength{0};
    
    /// Is full circle
    bool isFullCircle{false};
};

// ============================================================================
// Arc Generation Functions
// ============================================================================

/**
 * @brief Parse arc parameters from block
 * 
 * @param block Parsed G-code block
 * @param currentPos Current machine position
 * @param direction Arc direction (from G2/G3)
 * @param plane Active plane
 * @param config Arc configuration
 * @param[out] spec Computed arc specification
 * @return Error if parameters are invalid
 */
Error parseArcParameters(
    const Block& block,
    const Position& currentPos,
    ArcDirection direction,
    Plane plane,
    const ArcMotionConfig& config,
    ArcSpec& spec
);

/**
 * @brief Calculate arc center from radius format
 * 
 * Given start, end, and radius, compute center point.
 * 
 * @param startX Start X
 * @param startY Start Y
 * @param endX End X
 * @param endY End Y
 * @param radius Signed radius (negative = major arc)
 * @param direction CW or CCW
 * @param[out] centerX Computed center X
 * @param[out] centerY Computed center Y
 * @return Error if impossible geometry
 */
Error calculateArcCenter(
    double startX, double startY,
    double endX, double endY,
    double radius,
    ArcDirection direction,
    double& centerX, double& centerY
);

/**
 * @brief Validate arc geometry
 * 
 * Checks:
 * - Radius consistency
 * - Non-degenerate arc
 * - Within tolerance
 */
Error validateArcGeometry(
    const ArcSpec& spec,
    const ArcMotionConfig& config
);

/**
 * @brief Calculate arc angles
 * 
 * @param spec Arc specification (updated with angles)
 */
void calculateArcAngles(ArcSpec& spec);

/**
 * @brief Calculate point on arc
 * 
 * @param spec Arc specification
 * @param t Parameter (0 to 1)
 * @return Position on arc (only arc plane axes valid)
 */
Position interpolateArc(const ArcSpec& spec, double t);

// ============================================================================
// Arc Discretization
// ============================================================================

/**
 * @brief Discretize arc into line segments
 * 
 * @param spec Arc specification
 * @param config Configuration (for segment limits)
 * @param[out] points Vector of points on arc
 * @return Error if discretization fails
 */
Error discretizeArc(
    const ArcSpec& spec,
    const ArcMotionConfig& config,
    std::vector<Position>& points
);

/**
 * @brief Calculate optimal number of segments
 * 
 * Based on arc length and deviation tolerance.
 * 
 * @param radius Arc radius
 * @param sweepAngle Swept angle (radians)
 * @param maxDeviation Maximum chord deviation
 * @param config Configuration
 * @return Number of segments
 */
int32_t calculateArcSegments(
    double radius,
    double sweepAngle,
    double maxDeviation,
    const ArcMotionConfig& config
);

// ============================================================================
// Motion Segment Generation
// ============================================================================

/**
 * @brief Generate motion segment for G2 (CW arc)
 * 
 * @param block Parsed block
 * @param state Machine state
 * @param config Arc configuration
 * @param[out] segments Generated motion segments
 * @return Error if invalid
 */
Error generateG2Segments(
    const Block& block,
    const MachineState& state,
    const ArcMotionConfig& config,
    std::vector<MotionSegment>& segments
);

/**
 * @brief Generate motion segment for G3 (CCW arc)
 */
Error generateG3Segments(
    const Block& block,
    const MachineState& state,
    const ArcMotionConfig& config,
    std::vector<MotionSegment>& segments
);

/**
 * @brief Generate single arc/helix motion segment
 * 
 * This keeps the arc as a single segment for output to
 * hardware that supports native arc commands.
 */
Error generateArcSegment(
    const ArcSpec& spec,
    double feedRate,
    std::vector<MotionSegment>& segments
);

// ============================================================================
// Feed Rate Calculation
// ============================================================================

/**
 * @brief Calculate arc feed rate considering centripetal limit
 * 
 * Maximum speed on arc is limited by centripetal acceleration:
 * v_max = sqrt(a_centripetal * radius)
 * 
 * @param requestedFeed Requested feed rate
 * @param radius Arc radius
 * @param maxCentripetalAccel Maximum centripetal acceleration
 * @return Limited feed rate
 */
double limitArcFeedRate(
    double requestedFeed,
    double radius,
    double maxCentripetalAccel
);

/**
 * @brief Calculate total helix path length
 * 
 * L = sqrt(arc_length² + linear_delta²)
 * 
 * For multi-turn:
 * L = sqrt((turns * 2 * pi * radius + final_arc)² + linear_delta²)
 */
double calculateHelixLength(const ArcSpec& spec);

// ============================================================================
// Arc Handler Class
// ============================================================================

/**
 * @brief Handler for G2/G3 arc commands
 */
class ArcMotionHandler {
public:
    /**
     * @brief Constructor
     */
    explicit ArcMotionHandler(const ArcMotionConfig& config = {});
    
    /**
     * @brief Process G2 command
     * @param block Parsed block
     * @param state Machine state (updated)
     * @param vars Variable system
     * @param[out] segments Generated motion segments
     * @return Error if failed
     */
    Error processG2(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G3 command
     */
    Error processG3(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Update configuration
     */
    void setConfig(const ArcMotionConfig& config) { m_config = config; }
    
    /**
     * @brief Get current configuration
     */
    const ArcMotionConfig& getConfig() const { return m_config; }
    
    /**
     * @brief Get last computed arc specification
     */
    const ArcSpec& getLastArc() const { return m_lastArc; }
    
    /**
     * @brief Set output mode
     * @param nativeArcs Output native arc segments (true) or linearize (false)
     */
    void setNativeArcOutput(bool nativeArcs) { m_nativeArcs = nativeArcs; }
    
    /**
     * @brief Check if native arc output is enabled
     */
    bool getNativeArcOutput() const { return m_nativeArcs; }
    
private:
    ArcMotionConfig m_config;
    ArcSpec m_lastArc;
    bool m_nativeArcs{true};
    
    Error processArc(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        ArcDirection direction,
        std::vector<MotionSegment>& segments
    );
};

// ============================================================================
// G17/G18/G19 Plane Selection
// ============================================================================

/**
 * @brief Plane axis mapping
 */
struct PlaneAxes {
    int primaryAxis;    ///< First arc axis (index)
    int secondaryAxis;  ///< Second arc axis (index)
    int normalAxis;     ///< Perpendicular (helix) axis
    int primaryOffset;  ///< I, J, K index for primary
    int secondaryOffset; ///< I, J, K index for secondary
};

/**
 * @brief Get axis indices for plane
 */
PlaneAxes getPlaneAxes(Plane plane);

/**
 * @brief Process G17 (XY plane select)
 */
Error processG17(MachineState& state);

/**
 * @brief Process G18 (XZ plane select)
 */
Error processG18(MachineState& state);

/**
 * @brief Process G19 (YZ plane select)
 */
Error processG19(MachineState& state);

} // namespace GCode
