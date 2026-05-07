/**
 * @file GCodeG0G1.hpp
 * @brief G0 (Rapid Move) and G1 (Linear Feed) Implementation
 * 
 * @details
 * ## G0 - Rapid Move (Modal Group 1)
 * 
 * G0 moves all specified axes simultaneously to the target position at the 
 * machine's maximum traverse rate. The path is NOT guaranteed to be a 
 * straight line—axes may reach their destinations at different times.
 * 
 * ### Syntax
 * ```gcode
 * G0 X__ Y__ Z__ A__ B__ C__ U__ V__ W__
 * ```
 * 
 * ### Parameters
 * | Letter | Description | Units |
 * |--------|-------------|-------|
 * | X | Target X position | mm or inch |
 * | Y | Target Y position | mm or inch |
 * | Z | Target Z position | mm or inch |
 * | A | Target A rotation | degrees |
 * | B | Target B rotation | degrees |
 * | C | Target C rotation | degrees |
 * | U | Target U position | mm or inch |
 * | V | Target V position | mm or inch |
 * | W | Target W position | mm or inch |
 * 
 * ### Motion Model
 * 
 * For independent axis motion (typical):
 * ```
 *   |      /
 *   |    /
 * Y |  /    <- Diagonal only if same time to complete
 *   |/______
 *        X
 * ```
 * 
 * For coordinated rapids (COORDINATED_RAPIDS feature):
 * - All axes move together in straight line
 * - Overall speed limited by slowest axis ratio
 * 
 * ### Safety Considerations
 * 
 * ⚠️ **WARNING**: G0 moves at MAXIMUM speed. Use with caution near:
 * - Obstacles and fixtures
 * - Workpiece surface
 * - Tool change positions
 * 
 * **Best Practice**: Raise Z first, then move XY, then lower Z:
 * ```gcode
 * G0 Z[safe_height]     ; Retract Z first
 * G0 X10 Y20            ; Move over position
 * G0 Z[work_height]     ; Plunge to work
 * ```
 * 
 * ### Feed Override
 * 
 * G0 respects the rapid override (typically 25%, 50%, 100% on machines).
 * LinuxCNC stores override as percentage parameter #<_rapid_override>.
 * 
 * ### Example Usage
 * 
 * ```gcode
 * ; Move to start position
 * G0 X0 Y0 Z50
 * 
 * ; Retract and rapid to next hole
 * G0 Z5
 * G0 X100 Y100
 * G0 Z0.5               ; Approach cautiously
 * 
 * ; With incremental mode
 * G91 G0 X10 Y10        ; Move 10mm in X and Y
 * G90                   ; Back to absolute
 * ```
 * 
 * ---
 * 
 * ## G1 - Linear Feed (Modal Group 1)
 * 
 * G1 moves all specified axes in a coordinated straight line at the 
 * specified feed rate. Motion is fully interpolated.
 * 
 * ### Syntax
 * ```gcode
 * G1 X__ Y__ Z__ A__ B__ C__ U__ V__ W__ F__
 * ```
 * 
 * ### Parameters
 * | Letter | Description | Units |
 * |--------|-------------|-------|
 * | X-W | Target positions | mm/inch/degrees |
 * | F | Feed rate (modal) | mm/min or inch/min |
 * 
 * ### Feed Rate Interpretation
 * 
 * Feed rate interpretation depends on G93/G94/G95:
 * 
 * | Mode | F Meaning | Formula |
 * |------|-----------|---------|
 * | G94 | Units per minute | F = mm/min |
 * | G95 | Units per revolution | F = mm/rev (requires spindle) |
 * | G93 | Inverse time | F = 1/minutes for move |
 * 
 * ### Inverse Time Mode (G93)
 * 
 * When G93 is active, F specifies how many times per minute this move 
 * would be completed:
 * 
 * ```gcode
 * G93                   ; Inverse time mode
 * G1 X10 F2.0           ; Complete in 0.5 minutes (30 seconds)
 * G1 X20 F60.0          ; Complete in 1 second
 * ```
 * 
 * Required for:
 * - 5-axis machining where path length is complex
 * - Non-linear axes
 * - Constant surface speed calculations
 * 
 * ### Motion Model
 * 
 * Linear interpolation ensures straight line:
 * ```
 *   End (X1, Y1)
 *     /
 *    /  <- Straight line path
 *   /
 * Start (X0, Y0)
 * ```
 * 
 * Time to complete: T = Distance / FeedRate
 * 
 * ### Multi-Axis Considerations
 * 
 * With rotary axes (A, B, C):
 * - Linear distance includes XYZ only by default
 * - Rotary axes treated as tangential
 * - Set rotation_factor in config for rotary contribution
 * 
 * Feed rate calculation with rotary:
 * ```
 * linear_dist = sqrt(dx² + dy² + dz²)
 * rotary_dist = sqrt(da² + db² + dc²) * rotation_factor
 * total_dist = sqrt(linear_dist² + rotary_dist²)
 * time = total_dist / feed_rate
 * ```
 * 
 * ### Path Blending / Look-ahead
 * 
 * When enabled (G64), consecutive G1 moves may be blended:
 * 
 * ```gcode
 * G64 P0.05 Q0.1        ; Path blending, tolerance 0.05mm
 * G1 X10 F1000
 * G1 X20 Y10            ; May not reach exactly X10 before turning
 * G1 X30 Y0
 * ```
 * 
 * With G61 (exact stop), each point is reached before starting next.
 * 
 * ### Adaptive Feed Rate
 * 
 * When M52 P1 is active, feed rate can be dynamically scaled via:
 * - analog input
 * - motion.adaptive-feed HAL pin
 * - #<_adaptive_feed> parameter
 * 
 * ### Examples
 * 
 * ```gcode
 * ; Basic linear cut
 * G1 X100 Y50 F500      ; Move to (100, 50) at 500mm/min
 * 
 * ; Ramp into material
 * G1 Z-1 F100           ; Slow plunge
 * G1 X50 F500           ; Cutting feed
 * 
 * ; Helical interpolation (combined with Z)
 * G1 X0 Y10 Z-0.5 F200  ; Linear helix segment
 * 
 * ; With variables
 * #<feedrate> = 600
 * #<depth> = -2.5
 * G1 Z#<depth> F#<feedrate>
 * 
 * ; Incremental move
 * G91
 * G1 X10 Y5 Z-0.1 F300  ; Move relative 10, 5, -0.1
 * G90
 * ```
 * 
 * @see MotionPlanner
 * @see GCodeG2G3 for arc interpolation
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <cmath>
#include <vector>

namespace GCode {

// Forward declarations
class MachineState;
class MotionPlanner;
class VariableSystem;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for G0/G1 motion
 */
struct LinearMotionConfig {
    // === Rapid (G0) Settings ===
    
    /// Coordinate rapids (straight line) vs independent axis motion
    bool coordinatedRapids{false};
    
    /// Rapid override minimum (0.0 to 1.0)
    double rapidOverrideMin{0.0};
    
    /// Rapid override maximum (0.0 to 1.0)
    double rapidOverrideMax{1.0};
    
    /// Safe Z height for rapid retract
    double safeZHeight{50.0};  // mm
    
    // === Feed (G1) Settings ===
    
    /// Default feed rate (if none specified)
    double defaultFeedRate{100.0};  // mm/min
    
    /// Minimum allowed feed rate
    double minFeedRate{0.1};  // mm/min
    
    /// Maximum allowed feed rate
    double maxFeedRate{10000.0};  // mm/min
    
    /// Plunge feed limit (Z negative direction)
    double maxPlungeFeed{500.0};  // mm/min
    
    /// Auto-limit plunge feed when Z is descending
    bool limitPlungeFeed{true};
    
    // === Path Blending ===
    
    /// Enable path blending (G64 default)
    bool pathBlendingDefault{true};
    
    /// Default path blend tolerance
    double blendToleranceP{0.05};  // mm
    
    /// Default blend threshold Q (start blend at Q distance)
    double blendToleranceQ{0.1};  // mm
    
    /// Maximum acceleration for blending
    double maxBlendAccel{500.0};  // mm/s²
    
    // === Feed Mode ===
    
    /// Default feed mode (G93/G94/G95)
    FeedMode defaultFeedMode{FeedMode::UNITS_PER_MIN};
    
    /// Require F word after G93 (inverse time) on every line
    bool requireFInG93{true};
    
    // === Multi-axis ===
    
    /// Include rotary axes in feed calculation
    bool includeRotaryInFeed{true};
    
    /// Rotation factor for rotary contribution (radius equivalent)
    double rotationFactor{1.0};
    
    // === Safety ===
    
    /// Require F word before first G1
    bool requireInitialFeed{true};
    
    /// Warn if G0 plunges into work
    bool warnRapidPlunge{true};
    
    /// Maximum safe rapid plunge depth
    double maxSafeRapidPlunge{5.0};  // mm from current Z
};

// ============================================================================
// Motion Segment Generation
// ============================================================================

/**
 * @brief Generate motion segment for G0 rapid
 * 
 * @param[in] current Current machine position
 * @param[in] target Target position from block
 * @param[in] state Machine state (for units, distance mode)
 * @param[in] config Motion configuration
 * @param[out] segment Generated motion segment
 * @return Error if invalid
 * 
 * @note Target may be partial (only some axes specified).
 *       Unspecified axes retain current position.
 */
Error generateG0Segment(
    const Position& current,
    const Position& target,
    const MachineState& state,
    const LinearMotionConfig& config,
    MotionSegment& segment
);

/**
 * @brief Generate motion segment for G1 linear feed
 * 
 * @param[in] current Current machine position
 * @param[in] target Target position from block
 * @param[in] feedRate Feed rate (interpretation depends on state)
 * @param[in] state Machine state
 * @param[in] config Motion configuration
 * @param[out] segment Generated motion segment
 * @return Error if invalid
 */
Error generateG1Segment(
    const Position& current,
    const Position& target,
    double feedRate,
    const MachineState& state,
    const LinearMotionConfig& config,
    MotionSegment& segment
);

// ============================================================================
// Feed Rate Calculation
// ============================================================================

/**
 * @brief Calculate effective feed rate considering mode
 * 
 * @param fWord F word from block
 * @param distance Move distance (for inverse time)
 * @param feedMode Current feed mode (G93/G94/G95)
 * @param spindleSpeed Current spindle RPM (for G95)
 * @return Feed rate in mm/min (or units/min)
 */
double calculateEffectiveFeedRate(
    double fWord,
    double distance,
    FeedMode feedMode,
    double spindleSpeed = 0.0
);

/**
 * @brief Calculate path distance including rotary contribution
 * 
 * @param start Start position
 * @param end End position
 * @param rotationFactor Factor for rotary axes
 * @param includeRotary Include rotary in calculation
 * @return Total path distance
 */
double calculatePathDistance(
    const Position& start,
    const Position& end,
    double rotationFactor = 1.0,
    bool includeRotary = true
);

/**
 * @brief Calculate move time
 * 
 * @param distance Path distance
 * @param feedRate Feed rate in units/min
 * @return Time in seconds
 */
inline double calculateMoveTime(double distance, double feedRate) {
    if (feedRate <= 0) return 0;
    return (distance / feedRate) * 60.0;  // Convert to seconds
}

// ============================================================================
// Path Blending
// ============================================================================

/**
 * @brief Path blend parameters
 */
struct PathBlendParams {
    GCode::PathMode mode{GCode::PathMode::BLEND};
    double toleranceP{0.05};  ///< Path deviation tolerance
    double toleranceQ{0.1};   ///< Naive CAM tolerance
};

/**
 * @brief Calculate blend parameters between two segments
 * 
 * @param seg1 First segment
 * @param seg2 Second segment
 * @param blend Blending parameters
 * @param[out] blendAccel Blend acceleration
 * @param[out] blendVel Blend velocity at junction
 * @return true if blending is possible
 */
bool calculatePathBlend(
    const MotionSegment& seg1,
    const MotionSegment& seg2,
    const PathBlendParams& blend,
    double& blendAccel,
    double& blendVel
);

// ============================================================================
// Lookahead Integration
// ============================================================================

/**
 * @brief Analyze lookahead queue for velocity planning
 * 
 * For negative feed rates (reverse motion), this examines
 * lookbehind instead of lookahead.
 * 
 * @param segments Queue of upcoming segments
 * @param blendParams Blend parameters
 * @param maxAccel Maximum acceleration
 * @return Vector of planned velocities for each junction
 */
std::vector<double> planLookaheadVelocities(
    const std::vector<MotionSegment>& segments,
    const PathBlendParams& blendParams,
    double maxAccel
);

// ============================================================================
// Validation
// ============================================================================

/**
 * @brief Validate G0 parameters
 */
Error validateG0(const Block& block, const MachineState& state);

/**
 * @brief Validate G1 parameters
 */
Error validateG1(const Block& block, const MachineState& state,
                 const LinearMotionConfig& config);

/**
 * @brief Check for rapid plunge warning
 */
bool checkRapidPlungeWarning(
    const Position& current,
    const Position& target,
    const LinearMotionConfig& config
);

// ============================================================================
// G0/G1 Handler Class
// ============================================================================

/**
 * @brief Handler for G0 and G1 commands
 */
class LinearMotionHandler {
public:
    /**
     * @brief Constructor
     * @param config Configuration for linear motion
     */
    explicit LinearMotionHandler(const LinearMotionConfig& config = {});
    
    /**
     * @brief Process G0 command
     * @param block Parsed block
     * @param state Machine state (updated)
     * @param vars Variable system
     * @param[out] segments Generated motion segments
     * @return Error if failed
     */
    Error processG0(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G1 command
     * @param block Parsed block
     * @param state Machine state (updated)
     * @param vars Variable system
     * @param[out] segments Generated motion segments
     * @return Error if failed
     */
    Error processG1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Update configuration
     */
    void setConfig(const LinearMotionConfig& config) { m_config = config; }
    
    /**
     * @brief Get current configuration
     */
    const LinearMotionConfig& getConfig() const { return m_config; }
    
    /**
     * @brief Set path blend mode
     */
    void setPathMode(PathMode mode) { m_pathMode = mode; }
    void setPathMode(PathMode mode, double p, double q = 0);
    
    /**
     * @brief Get path blend parameters
     */
    const PathBlendParams& getPathBlend() const { return m_blendParams; }
    
    /**
     * @brief Set rapid override
     */
    void setRapidOverride(double override);
    double getRapidOverride() const { return m_rapidOverride; }
    
    /**
     * @brief Set feed override
     */
    void setFeedOverride(double override);
    double getFeedOverride() const { return m_feedOverride; }
    
    /**
     * @brief Enable/disable adaptive feed
     */
    void setAdaptiveFeed(bool enable, double factor = 1.0);
    bool getAdaptiveFeedEnabled() const { return m_adaptiveFeedEnabled; }
    double getAdaptiveFeedFactor() const { return m_adaptiveFeedFactor; }
    
private:
    LinearMotionConfig m_config;
    PathBlendParams m_blendParams;
    PathMode m_pathMode{PathMode::BLEND};
    
    double m_rapidOverride{1.0};
    double m_feedOverride{1.0};
    bool m_adaptiveFeedEnabled{false};
    double m_adaptiveFeedFactor{1.0};
    
    Position resolveTarget(const Block& block, const MachineState& state);
    double resolveFeedRate(const Block& block, const MachineState& state);
};

} // namespace GCode
