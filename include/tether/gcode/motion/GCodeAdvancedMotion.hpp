/**
 * @file GCodeAdvancedMotion.hpp
 * @brief Advanced Motion: Trochoidal Milling, Volumetric Compensation, Path Blending
 * 
 * @details
 * This file contains advanced motion features for high-performance machining:
 * 
 * 1. **Trochoidal Milling** - Circular toolpath for slot milling
 * 2. **Volumetric Compensation** - Machine geometry error correction
 * 3. **Path Blending (G64)** - Continuous motion through corners
 * 4. **Adaptive Feed Rate** - Speed adjustment based on load
 * 5. **Corner Rounding** - Smooth transitions at direction changes
 * 6. **Backlash Compensation** - Mechanical play correction
 * 
 * ---
 * 
 * ## Trochoidal Milling
 * 
 * ### Concept
 * 
 * Trochoidal milling uses a circular toolpath that advances along a slot,
 * keeping tool engagement (arc of contact) constant and low.
 * 
 * ```
 *        Direction of cut →
 * 
 *     ↗   ↗   ↗   ↗
 *    ○   ○   ○   ○    ← Tool spirals forward
 *     ↘   ↘   ↘   ↘
 *    ════════════════  ← Slot being cut
 * ```
 * 
 * ### Benefits
 * 
 * - Reduced tool wear (constant chip load)
 * - Higher feed rates possible
 * - Better chip evacuation
 * - Reduced heat buildup
 * - Can cut full-width slots without ramping
 * 
 * ### Parameters
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | Circle diameter | Tool circle size (< slot width) |
 * | Stepover | Advance per circle (mm) |
 * | Slot width | Final slot dimension |
 * | Engagement | Max arc of contact (degrees) |
 * 
 * ### G-Code (Custom, based on various implementations)
 * 
 * ```gcode
 * ; Trochoidal slot (example custom code)
 * G12.1 X100 Y0 P10 Q5 R5 F1000
 * ; X100 Y0 = endpoint
 * ; P10 = slot width
 * ; Q5 = stepover
 * ; R5 = circle radius
 * ; F1000 = feed rate
 * ```
 * 
 * ---
 * 
 * ## Volumetric Compensation
 * 
 * ### Concept
 * 
 * Real machines have geometric errors (squareness, straightness, positioning).
 * Volumetric compensation corrects these throughout the work envelope.
 * 
 * ### Error Sources
 * 
 * | Error Type | Description |
 * |------------|-------------|
 * | Linear | Axis positioning error (leadscrew pitch) |
 * | Straightness | Axis wanders side-to-side |
 * | Squareness | Axes not at 90° |
 * | Angular | Pitch, roll, yaw of moving axes |
 * 
 * ### Compensation Model
 * 
 * At any point (X, Y, Z), the machine has error:
 * ```
 * ErrorX = f(X, Y, Z)
 * ErrorY = g(X, Y, Z)
 * ErrorZ = h(X, Y, Z)
 * ```
 * 
 * Compensated position:
 * ```
 * X_actual = X_commanded - ErrorX(X, Y, Z)
 * Y_actual = Y_commanded - ErrorY(X, Y, Z)
 * Z_actual = Z_commanded - ErrorZ(X, Y, Z)
 * ```
 * 
 * ### Implementation
 * 
 * Typically uses:
 * - 3D lookup table (error grid)
 * - Trilinear interpolation between grid points
 * - Real-time application in motion controller
 * 
 * ---
 * 
 * ## Path Blending (G64, G61, G61.1)
 * 
 * ### G64 - Continuous Path (Blending)
 * 
 * ```gcode
 * G64           ; Enable path blending (default tolerance)
 * G64 P0.05     ; Blend tolerance 0.05mm
 * G64 P0.05 Q0.1 ; Blend tolerance P, naive CAM tolerance Q
 * ```
 * 
 * Path blending allows the machine to round corners instead of
 * decelerating to zero at each direction change.
 * 
 * ```
 * G61 (Exact stop):        G64 (Blending):
 * 
 *   │                        │
 *   │                        │
 *   └───────                 └╮──────
 *     ↑                       ↑
 *   Stops here             Rounds corner
 * ```
 * 
 * ### Tolerance Parameters
 * 
 * **P tolerance**: Maximum path deviation at corners
 * 
 * **Q tolerance (Naive CAM)**: For rough CAM output with many small 
 * segments. Allows internal segments to deviate by Q while still 
 * maintaining P at final corners.
 * 
 * ### G61 - Exact Stop Mode
 * 
 * ```gcode
 * G61           ; Full stop at each endpoint
 * ```
 * 
 * Machine decelerates to zero velocity at each programmed point.
 * Slowest but most accurate.
 * 
 * ### G61.1 - Exact Path Mode
 * 
 * ```gcode
 * G61.1         ; Exact path, but may not stop
 * ```
 * 
 * Machine passes through each point exactly, but doesn't require
 * zero velocity. Tangent direction preserved.
 * 
 * ---
 * 
 * ## Adaptive Feed Rate Control
 * 
 * ### Concept
 * 
 * Dynamically adjust feed rate based on:
 * - Spindle load
 * - Cutting force
 * - Tool deflection
 * - External signal
 * 
 * ### Enabling
 * 
 * ```gcode
 * M52 P1        ; Enable adaptive feed
 * M52 P0        ; Disable adaptive feed
 * ```
 * 
 * ### Signal Source
 * 
 * - Analog input (0-10V → 0-100%)
 * - HAL pin (motion.adaptive-feed)
 * - Parameter #<_adaptive_feed>
 * 
 * ### Feed Calculation
 * 
 * ```
 * Effective_Feed = Programmed_Feed × Adaptive_Factor × Feed_Override
 * ```
 * 
 * ---
 * 
 * ## Corner Rounding
 * 
 * ### Automatic Arc Insertion
 * 
 * At direction changes, insert arc tangent to both segments.
 * 
 * ```
 * Original:                 Rounded:
 *   │                         │
 *   │                         │
 *   └────                     ╰────
 * ```
 * 
 * ### Parameters
 * 
 * - Minimum angle (don't round shallow corners)
 * - Maximum radius
 * - Target deviation (derived from blend tolerance)
 * 
 * ---
 * 
 * ## Backlash Compensation
 * 
 * ### Concept
 * 
 * Mechanical backlash causes position error when axis reverses.
 * Compensation adds extra motion on reversal.
 * 
 * ```
 * Without compensation:    With compensation:
 *   
 * Command: +10, -5         Command: +10, -(5+backlash), +backlash
 * Actual:  +10, -4.9       Actual:  +10, -5 (correct!)
 * ```
 * 
 * ### Types
 * 
 * **Simple**: Fixed backlash value per axis
 * 
 * **Velocity-dependent**: Varies with speed (acceleration phase)
 * 
 * **Position-dependent**: Varies with axis position (lookup table)
 * 
 * ### Configuration
 * 
 * Set per axis:
 * - Backlash amount (mm)
 * - Takeup velocity (mm/s)
 * 
 * @see GCodeG0G1 for basic motion
 * @see GCodeConfig for configuration structures
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <vector>
#include <array>
#include <functional>
#include <optional>

namespace GCode {

// Forward declarations
class MachineState;
class VariableSystem;

// ============================================================================
// Trochoidal Milling
// ============================================================================

/**
 * @brief Trochoidal milling parameters
 */
struct TrochoidalParams {
    /// Slot start position
    Position start;
    
    /// Slot end position  
    Position end;
    
    /// Slot width
    double slotWidth{10.0};
    
    /// Tool diameter
    double toolDiameter{6.0};
    
    /// Circle radius (tool center path)
    double circleRadius{3.0};
    
    /// Stepover per circle
    double stepover{2.0};
    
    /// Depth of cut
    double depth{-5.0};
    
    /// Feed rate
    double feedRate{1000.0};
    
    /// Plunge feed rate
    double plungeFeed{200.0};
    
    /// Maximum tool engagement (degrees)
    double maxEngagement{90.0};
    
    /// Direction (1 = climb, -1 = conventional)
    int direction{1};
    
    /// Points per circle
    int pointsPerCircle{36};
};

/**
 * @brief Generate trochoidal milling toolpath
 * 
 * @param params Trochoidal parameters
 * @param state Machine state
 * @param[out] segments Generated motion segments
 * @return Error if parameters invalid
 */
Error generateTrochoidalPath(
    const TrochoidalParams& params,
    const MachineState& state,
    std::vector<MotionSegment>& segments
);

/**
 * @brief Calculate optimal trochoidal parameters
 * 
 * @param toolDiameter Tool diameter
 * @param slotWidth Desired slot width
 * @param maxEngagement Maximum tool engagement angle
 * @param[out] circleRadius Computed circle radius
 * @param[out] stepover Computed stepover
 */
void calculateTrochoidalParams(
    double toolDiameter,
    double slotWidth,
    double maxEngagement,
    double& circleRadius,
    double& stepover
);

/**
 * @brief Trochoidal milling handler
 */
class TrochoidalHandler {
public:
    explicit TrochoidalHandler(const TrochoidalConfig& config = {});
    
    /**
     * @brief Process G12.1 (trochoidal slot - custom code)
     */
    Error processG12_1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Set configuration
     */
    void setConfig(const TrochoidalConfig& config) { m_config = config; }
    const TrochoidalConfig& getConfig() const { return m_config; }
    
private:
    TrochoidalConfig m_config;
};

// ============================================================================
// Volumetric Compensation
// ============================================================================

/**
 * @brief Error vector at a grid point
 */
struct VolumetricError {
    double ex{0};  // X error
    double ey{0};  // Y error
    double ez{0};  // Z error
};

/**
 * @brief Volumetric compensation grid
 */
class VolumetricCompensation {
public:
    /**
     * @brief Constructor
     * @param config Volumetric compensation configuration
     */
    explicit VolumetricCompensation(const VolumetricConfig& config = {});
    
    /**
     * @brief Enable/disable compensation
     */
    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }
    
    /**
     * @brief Set error at grid point
     * 
     * @param ix X index
     * @param iy Y index
     * @param iz Z index
     * @param error Error vector
     */
    Error setError(int ix, int iy, int iz, const VolumetricError& error);
    
    /**
     * @brief Get error at grid point
     */
    const VolumetricError* getError(int ix, int iy, int iz) const;
    
    /**
     * @brief Interpolate error at arbitrary position
     * 
     * Uses trilinear interpolation.
     */
    VolumetricError interpolateError(const Position& pos) const;
    
    /**
     * @brief Apply compensation to position
     * 
     * @param pos Commanded position
     * @return Compensated position
     */
    Position applyCompensation(const Position& pos) const;
    
    /**
     * @brief Remove compensation from position
     */
    Position removeCompensation(const Position& pos) const;
    
    /**
     * @brief Load compensation table from file
     */
    Error loadFromFile(const std::string& filename);
    
    /**
     * @brief Save compensation table to file
     */
    Error saveToFile(const std::string& filename) const;
    
    /**
     * @brief Get grid size
     */
    void getGridSize(int& nx, int& ny, int& nz) const;
    
    /**
     * @brief Get config
     */
    const VolumetricConfig& getConfig() const { return m_config; }
    
private:
    VolumetricConfig m_config;
    bool m_enabled{false};
    
    // 3D grid of errors
    std::vector<VolumetricError> m_errors;
    int m_nx{0}, m_ny{0}, m_nz{0};
    
    // Convert position to grid indices
    void posToGridIndex(const Position& pos, 
                        int& ix, int& iy, int& iz,
                        double& fx, double& fy, double& fz) const;
    
    // Array index from grid indices
    size_t gridIndex(int ix, int iy, int iz) const;
};

// ============================================================================
// Path Blending
// ============================================================================

/**
 * @brief Path blending mode (G64/G61/G61.1)
 */
enum class PathBlendMode {
    EXACT_STOP,     ///< G61 - Full stop at corners
    EXACT_PATH,     ///< G61.1 - Pass through points exactly
    CONTINUOUS      ///< G64 - Blend corners
};

/**
 * @brief Path blending configuration
 */
struct PathBlendConfig {
    /// Blend mode
    PathBlendMode mode{PathBlendMode::CONTINUOUS};
    
    /// Path tolerance P (mm)
    double toleranceP{0.05};
    
    /// Naive CAM tolerance Q (mm)
    double toleranceQ{0.1};
    
    /// Maximum blend velocity
    double maxBlendVelocity{10000.0};  // mm/min
    
    /// Maximum blend acceleration
    double maxBlendAccel{500.0};  // mm/s²
    
    /// Minimum angle to blend (radians)
    double minBlendAngle{0.01};
    
    /// Maximum angle to blend (radians) - above this, exact stop
    double maxBlendAngle{M_PI * 0.9};
};

/**
 * @brief Path blending handler
 */
class PathBlender {
public:
    explicit PathBlender(const PathBlendConfig& config = {});
    
    /**
     * @brief Process G61 (exact stop)
     */
    Error processG61(MachineState& state);
    
    /**
     * @brief Process G61.1 (exact path)
     */
    Error processG61_1(MachineState& state);
    
    /**
     * @brief Process G64 (path blending)
     */
    Error processG64(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Calculate blend at corner
     * 
     * @param prev Previous segment end
     * @param corner Corner point
     * @param next Next segment start direction
     * @param feedRate Desired feed rate
     * @param[out] blendSegments Blended path segments
     * @return Error if blending fails
     */
    Error calculateCornerBlend(
        const Position& prev,
        const Position& corner,
        const Position& next,
        double feedRate,
        std::vector<MotionSegment>& blendSegments
    );
    
    /**
     * @brief Plan velocities for segment queue (lookahead)
     * 
     * @param segments Input segments
     * @param[out] velocities Planned entry/exit velocities
     */
    Error planVelocities(
        const std::vector<MotionSegment>& segments,
        std::vector<std::pair<double, double>>& velocities
    );
    
    /**
     * @brief Get current mode
     */
    PathBlendMode getMode() const { return m_config.mode; }
    
    /**
     * @brief Set configuration
     */
    void setConfig(const PathBlendConfig& config) { m_config = config; }
    const PathBlendConfig& getConfig() const { return m_config; }
    
private:
    PathBlendConfig m_config;
    
    // Calculate junction velocity that respects tolerances
    double calculateJunctionVelocity(
        const Position& dir1,
        const Position& dir2,
        double inVel,
        double outVel
    );
    
    // Generate arc blend at corner
    void generateArcBlend(
        const Position& p1,
        const Position& p2,
        const Position& p3,
        double radius,
        std::vector<MotionSegment>& output
    );
};

// ============================================================================
// Adaptive Feed Rate
// ============================================================================

/**
 * @brief Adaptive feed rate source
 */
enum class AdaptiveFeedSource {
    DISABLED,       ///< No adaptive feed
    PARAMETER,      ///< From #<_adaptive_feed> parameter
    ANALOG_INPUT,   ///< From analog input
    SPINDLE_LOAD,   ///< Based on spindle load
    CALLBACK        ///< User callback function
};

/**
 * @brief Adaptive feed rate callback
 * @return Feed factor (0.0 to 1.0, or higher for speed up)
 */
using AdaptiveFeedCallback = std::function<double(
    const Position& currentPos,
    double programmedFeed
)>;

/**
 * @brief Adaptive feed rate controller
 */
class AdaptiveFeedController {
public:
    /**
     * @brief Enable adaptive feed (M52 P1)
     */
    void enable();
    
    /**
     * @brief Disable adaptive feed (M52 P0)
     */
    void disable();
    
    /**
     * @brief Check if enabled
     */
    bool isEnabled() const { return m_enabled; }
    
    /**
     * @brief Set feed source
     */
    void setSource(AdaptiveFeedSource source);
    
    /**
     * @brief Set callback for CALLBACK source
     */
    void setCallback(AdaptiveFeedCallback callback);
    
    /**
     * @brief Get current adaptive factor
     */
    double getAdaptiveFactor(
        const Position& pos,
        const VariableSystem& vars
    ) const;
    
    /**
     * @brief Apply adaptive feed to rate
     */
    double applyAdaptiveFeed(
        double programmedFeed,
        const Position& pos,
        const VariableSystem& vars
    ) const;
    
    /**
     * @brief Set limits
     */
    void setMinFactor(double min) { m_minFactor = min; }
    void setMaxFactor(double max) { m_maxFactor = max; }
    
private:
    bool m_enabled{false};
    AdaptiveFeedSource m_source{AdaptiveFeedSource::DISABLED};
    AdaptiveFeedCallback m_callback;
    double m_minFactor{0.0};
    double m_maxFactor{2.0};
};

// ============================================================================
// Backlash Compensation
// ============================================================================

/**
 * @brief Backlash compensation per axis
 */
struct AxisBacklash {
    double amount{0};      ///< Backlash amount (mm)
    double takeupVel{10};  ///< Takeup velocity (mm/s)
    bool enabled{false};
    
    /// Last direction of axis motion (+1, -1, 0)
    int lastDirection{0};
    
    /// Accumulated backlash to apply
    double pending{0};
};

/**
 * @brief Backlash compensation handler
 */
class BacklashCompensation {
public:
    /**
     * @brief Constructor
     * @param config Per-axis backlash configuration
     */
    BacklashCompensation();
    
    /**
     * @brief Set backlash for axis
     */
    void setBacklash(int axis, double amount, double takeupVel = 10.0);
    
    /**
     * @brief Enable/disable backlash compensation for axis
     */
    void setEnabled(int axis, bool enabled);
    
    /**
     * @brief Global enable/disable
     */
    void setGlobalEnabled(bool enabled) { m_globalEnabled = enabled; }
    bool isGlobalEnabled() const { return m_globalEnabled; }
    
    /**
     * @brief Apply backlash compensation to motion
     * 
     * @param start Start position
     * @param end End position
     * @param[out] compensatedEnd Compensated end position
     * @param[out] extraMoves Any extra moves needed for takeup
     */
    void applyCompensation(
        const Position& start,
        const Position& end,
        Position& compensatedEnd,
        std::vector<MotionSegment>& extraMoves
    );
    
    /**
     * @brief Get backlash amount for axis
     */
    double getBacklash(int axis) const;
    
    /**
     * @brief Reset direction tracking (after homing)
     */
    void resetDirections();
    
private:
    std::array<AxisBacklash, 9> m_axes;  // XYZABCUVW
    bool m_globalEnabled{false};
    
    int getDirection(double delta) const;
};

// ============================================================================
// Feed/Rapid Override
// ============================================================================

/**
 * @brief Feed and rapid override controller
 */
class OverrideController {
public:
    /**
     * @brief Set feed override (0.0 to 2.0, typically)
     */
    void setFeedOverride(double override);
    double getFeedOverride() const { return m_feedOverride; }
    
    /**
     * @brief Set rapid override (0.0 to 1.0, typically)
     */
    void setRapidOverride(double override);
    double getRapidOverride() const { return m_rapidOverride; }
    
    /**
     * @brief Set spindle override
     */
    void setSpindleOverride(double override);
    double getSpindleOverride() const { return m_spindleOverride; }
    
    /**
     * @brief Enable/disable feed override (M48/M49)
     */
    void setFeedOverrideEnabled(bool enabled) { m_feedOverrideEnabled = enabled; }
    bool isFeedOverrideEnabled() const { return m_feedOverrideEnabled; }
    
    /**
     * @brief Apply feed override
     */
    double applyFeedOverride(double feed) const;
    
    /**
     * @brief Apply rapid override
     */
    double applyRapidOverride(double rapid) const;
    
    /**
     * @brief Apply spindle override
     */
    double applySpindleOverride(double rpm) const;
    
    /**
     * @brief Process M48 (enable overrides)
     */
    Error processM48();
    
    /**
     * @brief Process M49 (disable overrides)
     */
    Error processM49();
    
    /**
     * @brief Process M50 (feed override control)
     */
    Error processM50(const Block& block);
    
    /**
     * @brief Process M51 (spindle override control)
     */
    Error processM51(const Block& block);
    
private:
    double m_feedOverride{1.0};
    double m_rapidOverride{1.0};
    double m_spindleOverride{1.0};
    bool m_feedOverrideEnabled{true};
    bool m_spindleOverrideEnabled{true};
};

} // namespace GCode
