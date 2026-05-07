/**
 * @file GCodeProbing.hpp
 * @brief Probing Cycles: G38.2, G38.3, G38.4, G38.5, G38.x
 * 
 * @details
 * ## Overview
 * 
 * Probing cycles are used for:
 * - Tool length measurement
 * - Workpiece edge/corner finding
 * - Surface mapping
 * - Part verification
 * - Automatic work offset setting
 * 
 * ## Probe Commands
 * 
 * | G-Code | Motion | On Contact | On Miss |
 * |--------|--------|------------|---------|
 * | G38.2 | Toward | Stop + Store | Error |
 * | G38.3 | Toward | Stop + Store | No error |
 * | G38.4 | Away | Stop + Store | Error |
 * | G38.5 | Away | Stop + Store | No error |
 * 
 * ### Syntax
 * ```gcode
 * G38.2 X__ Y__ Z__ F__
 * ```
 * 
 * ### Parameters
 * | Parameter | Description |
 * |-----------|-------------|
 * | X, Y, Z | Target position (probe moves toward this) |
 * | F | Probe feed rate (slow for accuracy) |
 * 
 * ### Result Storage
 * 
 * After probing, results are stored in parameters:
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | #5061 | X position at trip |
 * | #5062 | Y position at trip |
 * | #5063 | Z position at trip |
 * | #5064 | A position at trip |
 * | #5065 | B position at trip |
 * | #5066 | C position at trip |
 * | #5067 | U position at trip |
 * | #5068 | V position at trip |
 * | #5069 | W position at trip |
 * | #5070 | 1 if probe tripped, 0 if not |
 * 
 * ### Motion Model
 * 
 * ```
 * G38.2 (probe toward, error on miss):
 * 
 *                    Target
 *                      ↓
 * Probe ─────────→ ● ─ ─ ─ ─ → X
 *                  ↑
 *              Trip here
 *              (motion stops)
 * 
 * G38.4 (probe away, error on miss):
 * 
 * Surface──────┐
 *              │
 * Probe ●──────┼─────────→ Target
 *              │ ↑
 *              │ Trip when leaving surface
 * ```
 * 
 * ## Typical Probing Routines
 * 
 * ### Edge Finding
 * ```gcode
 * ; Find X edge
 * G0 X-10 Y0 Z5           ; Position outside part
 * G0 Z-5                  ; Lower probe
 * G38.2 X10 F50           ; Probe toward part
 * #<x_edge> = #5061       ; Store X edge
 * G0 X[#5061 - 5]         ; Back off
 * ```
 * 
 * ### Corner Finding
 * ```gcode
 * ; Find corner (X and Y edges)
 * G38.2 X20 F50           ; Find X edge
 * #<x> = #5061
 * G0 X[#<x> - 5]
 * G0 Y-10
 * G38.2 Y20 F50           ; Find Y edge
 * #<y> = #5062
 * G0 Y[#<y> - 5]
 * ; Corner is at (#<x>, #<y>)
 * ```
 * 
 * ### Center Finding (Bore)
 * ```gcode
 * ; Find center of bore
 * G0 X0 Y0 Z5             ; Above approximate center
 * G0 Z-10                 ; Into bore
 * 
 * G38.2 X-50 F50          ; Probe X-
 * #<x_minus> = #5061
 * G0 X0
 * 
 * G38.2 X50 F50           ; Probe X+
 * #<x_plus> = #5061
 * G0 X0
 * 
 * G38.2 Y-50 F50          ; Probe Y-
 * #<y_minus> = #5062
 * G0 Y0
 * 
 * G38.2 Y50 F50           ; Probe Y+
 * #<y_plus> = #5062
 * G0 Y0 Z5
 * 
 * ; Calculate center
 * #<center_x> = [[#<x_minus> + #<x_plus>] / 2]
 * #<center_y> = [[#<y_minus> + #<y_plus>] / 2]
 * #<diameter> = [#<x_plus> - #<x_minus>]
 * ```
 * 
 * ### Tool Length Measurement
 * ```gcode
 * ; Touch off on tool setter
 * G0 X[#<setter_x>] Y[#<setter_y>]
 * G0 Z[#<setter_z> + 10]
 * G38.2 Z[#<setter_z> - 10] F20
 * G43.1 Z[#5063 - #<setter_z>]  ; Set tool offset
 * G0 Z10
 * ```
 * 
 * ### Surface Mapping
 * ```gcode
 * ; Simple surface map (store in file or array)
 * #<grid_x> = 0
 * O100 while [#<grid_x> LE 100]
 *     #<grid_y> = 0
 *     O101 while [#<grid_y> LE 100]
 *         G0 X#<grid_x> Y#<grid_y> Z5
 *         G38.2 Z-10 F50
 *         ; Store #5063 for this point
 *         #<grid_y> = [#<grid_y> + 10]
 *     O101 endwhile
 *     #<grid_x> = [#<grid_x> + 10]
 * O100 endwhile
 * ```
 * 
 * ## Error Handling
 * 
 * G38.2 and G38.4 generate errors if probe doesn't trip:
 * - Motion stops at target
 * - Program stops (unless M48/M49 skip enabled)
 * - #5070 = 0
 * 
 * G38.3 and G38.5 do NOT error:
 * - Useful for conditional probing
 * - Check #5070 after probe move
 * 
 * ```gcode
 * G38.3 X10 F50           ; Probe, no error on miss
 * O100 if [#5070 EQ 1]
 *     ; Probe hit something
 *     #<edge> = #5061
 * O100 else
 *     ; Nothing found
 *     (MSG, No surface found)
 * O100 endif
 * ```
 * 
 * ## Advanced: Protected Positioning (G43.1 + G38)
 * 
 * LinuxCNC supports protected positioning where probe overtravel
 * is limited by tool length compensation:
 * 
 * ```gcode
 * G43.1 Z-5               ; Expect surface at Z-5
 * G38.2 Z-10 F50          ; Probe down, but soft-limit at -5
 * ```
 * 
 * ## Configuration
 * 
 * Key parameters:
 * - Probe input signal
 * - Debounce time
 * - Maximum overtravel
 * - Signal polarity
 * 
 * @see GCodeToolComp for G43/G43.1
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <vector>
#include <optional>
#include <functional>
#include <array>

namespace GCode {

// Forward declarations
class MachineState;
class VariableSystem;

// ============================================================================
// Configuration
// ============================================================================

#ifndef GCODE_PROBE_CONFIG_DEFINED
#define GCODE_PROBE_CONFIG_DEFINED
/**
 * @brief Probe configuration
 */
struct ProbeConfig {
    // === Probe Input ===
    
    /// Probe input is normally open (true) or normally closed (false)
    bool normallyOpen{true};
    
    /// Debounce time (seconds)
    double debounceTime{0.010};  // 10ms
    
    /// Input number/pin
    int32_t inputPin{0};
    
    // === Motion ===
    
    /// Maximum probe feed rate
    double maxProbeFeed{500.0};  // mm/min
    
    /// Default probe feed if not specified
    double defaultProbeFeed{100.0};  // mm/min
    
    /// Minimum probe feed rate
    double minProbeFeed{1.0};  // mm/min
    
    /// Maximum travel before error (if no trip)
    double maxProbeTravel{200.0};  // mm
    
    /// Deceleration distance after trip
    double stopDistance{0.5};  // mm
    
    // === Accuracy ===
    
    /// Enable second touch (slow approach for accuracy)
    bool enableSecondTouch{true};
    
    /// Second touch feed rate
    double secondTouchFeed{10.0};  // mm/min
    
    /// Back-off distance before second touch
    double secondTouchBackoff{1.0};  // mm
    
    // === Safety ===
    
    /// Require probe signal valid before move
    bool checkProbeBeforeMove{true};
    
    /// Error if probe already tripped at start (G38.2/G38.3)
    bool errorOnPreTrip{true};
    
    /// Maximum overtravel beyond expected surface
    double maxOvertravel{5.0};  // mm
};
#endif // GCODE_PROBE_CONFIG_DEFINED

// ============================================================================
// Probe Types
// ============================================================================

/**
 * @brief Probe cycle type
 */
enum class ProbeType {
    TOWARD_WITH_ERROR,    ///< G38.2 - Probe toward, error on no contact
    TOWARD_NO_ERROR,      ///< G38.3 - Probe toward, no error on miss
    AWAY_WITH_ERROR,      ///< G38.4 - Probe away from, error on no break
    AWAY_NO_ERROR         ///< G38.5 - Probe away from, no error on miss
};

#ifndef GCODE_PROBE_RESULT_DEFINED
#define GCODE_PROBE_RESULT_DEFINED
/**
 * @brief Probe result
 */
struct ProbeResult {
    /// Did probe trip?
    bool tripped{false};
    
    /// Position at trip (machine coordinates)
    Position tripPosition;
    
    /// Position at trip (work coordinates)
    Position tripWorkPosition;
    
    /// Probe type used
    ProbeType type{ProbeType::TOWARD_WITH_ERROR};
    
    /// Error if any
    Error error;
    
    /// Distance traveled
    double travelDistance{0};
    
    /// Timestamp of trip
    double tripTime{0};
};
#endif // GCODE_PROBE_RESULT_DEFINED

// ============================================================================
// Probe Handler
// ============================================================================

/**
 * @brief Callback for probe input reading
 * @return true if probe is tripped
 */
using ProbeInputCallback = std::function<bool()>;

/**
 * @brief Callback for probe motion execution
 * 
 * This is called to actually perform the probe motion. The implementation
 * should move toward target while monitoring probe input.
 * 
 * @param target Target position
 * @param feedRate Feed rate
 * @param probeType Type of probe (toward/away)
 * @param[out] result Probe result filled in by callback
 * @return Error if motion failed
 */
using ProbeMotionCallback = std::function<Error(
    const Position& target,
    double feedRate,
    ProbeType probeType,
    ProbeResult& result
)>;

/**
 * @brief Handler for probing cycles
 */
class ProbeHandler {
public:
    /**
     * @brief Constructor
     */
    explicit ProbeHandler(const ProbeConfig& config = {});
    
    /**
     * @brief Set probe input callback
     */
    void setProbeInputCallback(ProbeInputCallback callback);
    
    /**
     * @brief Set probe motion callback
     */
    void setProbeMotionCallback(ProbeMotionCallback callback);
    
    // ========================================================================
    // Probe Commands
    // ========================================================================
    
    /**
     * @brief Process G38.2 (probe toward, error on miss)
     */
    Error processG38_2(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G38.3 (probe toward, no error)
     */
    Error processG38_3(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G38.4 (probe away, error on miss)
     */
    Error processG38_4(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G38.5 (probe away, no error)
     */
    Error processG38_5(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    // ========================================================================
    // Utility Probing Routines
    // ========================================================================
    
    /**
     * @brief Perform single-axis probe
     * 
     * @param axis Axis to probe (0=X, 1=Y, 2=Z, etc.)
     * @param direction Direction (+1 or -1)
     * @param distance Maximum distance
     * @param feedRate Feed rate
     * @param state Machine state
     * @param vars Variable system
     * @return Probe result
     */
    ProbeResult probeAxis(
        int axis,
        int direction,
        double distance,
        double feedRate,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Find edge on axis
     * 
     * Performs probe toward, backs off, probes again for accuracy.
     */
    ProbeResult findEdge(
        int axis,
        int direction,
        double searchDistance,
        double feedRate,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Find center of bore
     * 
     * Probes +X, -X, +Y, -Y and computes center.
     * 
     * @param diameter Approximate bore diameter (for positioning)
     * @param feedRate Probe feed rate
     * @param state Machine state
     * @param vars Variable system
     * @param[out] centerX Computed center X
     * @param[out] centerY Computed center Y
     * @param[out] measuredDia Measured diameter
     */
    Error findBoreCenter(
        double diameter,
        double feedRate,
        MachineState& state,
        VariableSystem& vars,
        double& centerX,
        double& centerY,
        double& measuredDia
    );
    
    /**
     * @brief Find center of boss (outside)
     */
    Error findBossCenter(
        double width,
        double feedRate,
        MachineState& state,
        VariableSystem& vars,
        double& centerX,
        double& centerY,
        double& measuredWidth
    );
    
    /**
     * @brief Find corner (inside or outside)
     */
    Error findCorner(
        bool inside,           // true = inside corner, false = outside
        int xDirection,        // +1 or -1
        int yDirection,        // +1 or -1
        double searchDistance,
        double feedRate,
        MachineState& state,
        VariableSystem& vars,
        double& cornerX,
        double& cornerY
    );
    
    /**
     * @brief Measure tool length
     * 
     * @param setterX Tool setter X position
     * @param setterY Tool setter Y position
     * @param setterZ Tool setter Z position (top surface)
     * @param feedRate Probe feed rate
     * @param state Machine state
     * @param vars Variable system
     * @param[out] toolLength Measured tool length
     */
    Error measureToolLength(
        double setterX,
        double setterY,
        double setterZ,
        double feedRate,
        MachineState& state,
        VariableSystem& vars,
        double& toolLength
    );
    
    // ========================================================================
    // Results
    // ========================================================================
    
    /**
     * @brief Get last probe result
     */
    const ProbeResult& getLastResult() const { return m_lastResult; }
    
    /**
     * @brief Check if probe is currently tripped
     */
    bool isProbeTripped() const;
    
    /**
     * @brief Update probe result variables (#5061-#5070)
     */
    void updateProbeVariables(VariableSystem& vars, const ProbeResult& result);
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setConfig(const ProbeConfig& config) { m_config = config; }
    const ProbeConfig& getConfig() const { return m_config; }
    
private:
    ProbeConfig m_config;
    ProbeResult m_lastResult;
    ProbeInputCallback m_inputCallback;
    ProbeMotionCallback m_motionCallback;
    
    Error executeProbe(
        const Block& block,
        ProbeType type,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    Error validateProbeParams(
        const Block& block,
        const MachineState& state
    );
};

// ============================================================================
// Protected Positioning
// ============================================================================

/**
 * @brief Protected positioning mode (G43.1 + G38)
 * 
 * When active, probe moves are limited by expected surface position.
 */
struct ProtectedPositioning {
    bool enabled{false};
    Position expectedSurface;
    double maxOvertravel{5.0};
};

// ============================================================================
// Surface Mapping
// ============================================================================

/**
 * @brief Surface map point
 */
struct SurfacePoint {
    double x{0};
    double y{0};
    double z{0};  // Measured Z
};

/**
 * @brief Surface mapping configuration
 */
struct SurfaceMapConfig {
    double startX{0};
    double startY{0};
    double endX{100};
    double endY{100};
    double stepX{10};
    double stepY{10};
    double safeZ{5};
    double probeZ{-10};
    double feedRate{100};
};

/**
 * @brief Generate surface map
 * 
 * @param config Map configuration
 * @param handler Probe handler
 * @param state Machine state
 * @param vars Variable system
 * @param[out] points Measured surface points
 * @return Error if probing failed
 */
Error generateSurfaceMap(
    const SurfaceMapConfig& config,
    ProbeHandler& handler,
    MachineState& state,
    VariableSystem& vars,
    std::vector<SurfacePoint>& points
);

/**
 * @brief Interpolate Z from surface map
 * 
 * @param points Surface map points
 * @param x X position
 * @param y Y position
 * @return Interpolated Z
 */
double interpolateSurface(
    const std::vector<SurfacePoint>& points,
    double x,
    double y
);

} // namespace GCode
