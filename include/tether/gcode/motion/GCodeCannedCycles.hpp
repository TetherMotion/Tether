/**
 * @file GCodeCannedCycles.hpp
 * @brief Canned Cycles: G73, G76, G80-G89 (Drilling, Tapping, Boring)
 * 
 * @details
 * ## Overview
 * 
 * Canned cycles automate repetitive operations like drilling and tapping.
 * They combine multiple motions into a single command.
 * 
 * ## Common Elements
 * 
 * All canned cycles use these parameters:
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | X, Y | Hole position (in selected plane) |
 * | Z | Hole bottom (depth) |
 * | R | Retract plane (rapid to this before/after) |
 * | L | Repeat count (default 1) |
 * | P | Dwell time at bottom (milliseconds) |
 * | Q | Peck increment |
 * | F | Feed rate |
 * 
 * ### Retract Mode (G98/G99)
 * 
 * | Mode | Description |
 * |------|-------------|
 * | G98 | Retract to initial Z (safe travel between holes) |
 * | G99 | Retract to R plane (faster, but may hit clamps) |
 * 
 * ```
 * Initial Z ─┬─G98 retracts here─────────────G98───┐
 *            │                                      │
 * R Plane ───┼─G99 retracts here───G99────────G99──┼──
 *            │                       │              │
 * Z (depth) ─┴───────Drill──────────┴──────────────┴──
 * ```
 * 
 * ### Cycle Motion
 * 
 * Each cycle executes:
 * 1. Rapid to XY position
 * 2. Rapid to R plane
 * 3. Perform cycle (feed to Z with variations)
 * 4. Retract per G98/G99
 * 5. Repeat if L > 1 or if incremental mode
 * 
 * ---
 * 
 * ## G73 - High-Speed Peck Drill
 * 
 * Chip-breaking cycle with small retracts. Does NOT fully retract
 * to clear chips—faster for ductile materials.
 * 
 * ### Motion
 * ```
 * 1. Rapid to XY
 * 2. Rapid to R
 * 3. Feed down Q
 * 4. Rapid retract 0.1mm (chip break)
 * 5. Feed to next Q
 * 6. Repeat until Z
 * 7. Retract to R (G99) or initial (G98)
 * ```
 * 
 * ### Example
 * ```gcode
 * G73 X10 Y10 Z-20 R2 Q5 F200     ; Peck every 5mm
 * ```
 * 
 * ---
 * 
 * ## G74 - Left-Hand Tapping (CCW)
 * 
 * For left-hand threads. Spindle runs CCW, reverses at bottom.
 * 
 * ### Motion
 * ```
 * 1. Rapid to XY
 * 2. Spindle CCW at specified RPM
 * 3. Rapid to R
 * 4. Synchronized feed to Z (F = pitch × RPM)
 * 5. Spindle stops, reverses to CW
 * 6. Synchronized retract to R
 * 7. Retract
 * ```
 * 
 * ### Feed Rate
 * For tapping, F = pitch × RPM (mm/min) or F = 1/pitch (TPI mode)
 * 
 * ### Example
 * ```gcode
 * S500 M4                          ; 500 RPM CCW
 * G74 X10 Y10 Z-15 R2 F1.5         ; M10x1.5 left-hand tap
 * ```
 * 
 * ---
 * 
 * ## G76 - Fine Boring (Orient Spindle)
 * 
 * For precise bores. At bottom, spindle orients then tool
 * shifts to clear wall on retract.
 * 
 * ### Parameters
 * | Parameter | Description |
 * |-----------|-------------|
 * | I | X shift amount |
 * | J | Y shift amount |
 * | Q | Spindle orient position (0-360°) |
 * 
 * ### Motion
 * ```
 * 1. Rapid to XY
 * 2. Rapid to R
 * 3. Feed to Z
 * 4. Orient spindle to Q degrees
 * 5. Shift X by I, Y by J
 * 6. Rapid retract
 * 7. Shift back
 * ```
 * 
 * ### Example
 * ```gcode
 * G76 X0 Y0 Z-25 R2 Q180 I0.5 J0 F100
 * ```
 * 
 * ---
 * 
 * ## G80 - Cancel Canned Cycle
 * 
 * Deactivates any active canned cycle mode.
 * 
 * ```gcode
 * G80    ; Cancel drilling cycle, return to normal motion
 * ```
 * 
 * ---
 * 
 * ## G81 - Simple Drill
 * 
 * Basic drilling: rapid to R, feed to Z, retract.
 * 
 * ### Motion
 * ```
 * 1. Rapid to XY
 * 2. Rapid to R
 * 3. Feed to Z
 * 4. Rapid retract
 * ```
 * 
 * ### Example
 * ```gcode
 * G81 X10 Y10 Z-10 R2 F200
 * X20                               ; Repeat at X20 (cycle is modal)
 * X30
 * G80                               ; Cancel
 * ```
 * 
 * ---
 * 
 * ## G82 - Drill with Dwell
 * 
 * Same as G81, but dwells at bottom. Good for clearing chips
 * and achieving precise depth.
 * 
 * ### Motion
 * ```
 * 1-3. Same as G81
 * 4. Dwell P milliseconds
 * 5. Rapid retract
 * ```
 * 
 * ### Example
 * ```gcode
 * G82 X10 Y10 Z-10 R2 P500 F200     ; Dwell 0.5 sec
 * ```
 * 
 * ---
 * 
 * ## G83 - Deep Hole Peck Drill
 * 
 * Full retract peck cycle. Clears chips completely between pecks.
 * Slower than G73 but essential for deep holes.
 * 
 * ### Motion
 * ```
 * 1. Rapid to XY
 * 2. Rapid to R
 * 3. Feed down Q
 * 4. Rapid to R (full retract)
 * 5. Rapid to previous depth - clearance
 * 6. Feed next Q
 * 7. Repeat until Z
 * 8. Retract
 * ```
 * 
 * ### Clearance
 * 
 * On re-entry, rapid stops `peckClearance` mm above previous
 * depth to avoid crashing into chips.
 * 
 * ### Example
 * ```gcode
 * G83 X10 Y10 Z-50 R2 Q5 F100       ; Deep hole, 5mm pecks
 * ```
 * 
 * ---
 * 
 * ## G84 - Right-Hand Tapping (CW)
 * 
 * For standard right-hand threads.
 * 
 * ### Feed Rate
 * Critical: F must match pitch × RPM exactly.
 * 
 * For metric: F = pitch (mm) × RPM
 * For imperial: F = (1/TPI) × RPM
 * 
 * ### Example
 * ```gcode
 * S300 M3                           ; 300 RPM CW
 * G84 X10 Y10 Z-15 R2 F0.5          ; M8×1.25 at 0.5mm/rev × 300 = 150mm/min
 * ```
 * 
 * ### With Rigid Tapping (G33.1 alternative)
 * 
 * If machine has encoder feedback:
 * ```gcode
 * G84.2 X10 Y10 Z-15 R2 P1.5 F1.5   ; Rigid tap M10×1.5
 * ```
 * 
 * ---
 * 
 * ## G85 - Boring (Feed Out)
 * 
 * Boring cycle that feeds out at same rate. Ensures smooth bore finish.
 * 
 * ### Motion
 * ```
 * 1-3. Same as drill
 * 4. FEED retract (not rapid)
 * ```
 * 
 * ### Example
 * ```gcode
 * G85 X0 Y0 Z-30 R2 F100
 * ```
 * 
 * ---
 * 
 * ## G86 - Boring (Spindle Stop, Rapid Out)
 * 
 * At bottom, spindle stops before rapid retract. Prevents
 * drag marks but may leave witness mark.
 * 
 * ### Motion
 * ```
 * 1-3. Same as drill
 * 4. Spindle stop
 * 5. Rapid retract
 * 6. Restart spindle
 * ```
 * 
 * ---
 * 
 * ## G87 - Back Boring
 * 
 * For counter-bore on back side of part. Requires oriented
 * spindle stop and tool shift.
 * 
 * ### Motion
 * ```
 * 1. Orient spindle, shift tool
 * 2. Rapid to Z (below work)
 * 3. Shift back (tool enters bore)
 * 4. Start spindle, feed up to R (back-bore)
 * 5. Spindle orient, shift
 * 6. Rapid retract
 * ```
 * 
 * ---
 * 
 * ## G88 - Boring (Manual Retract)
 * 
 * Stops at bottom for manual retract. Machine holds until
 * operator presses cycle start.
 * 
 * ---
 * 
 * ## G89 - Boring (Dwell, Feed Out)
 * 
 * Combination of G82 (dwell) and G85 (feed out).
 * 
 * ### Motion
 * ```
 * 1-3. Same as drill
 * 4. Dwell P milliseconds
 * 5. Feed retract
 * ```
 * 
 * ### Example
 * ```gcode
 * G89 X0 Y0 Z-30 R2 P1000 F100      ; Dwell 1 sec
 * ```
 * 
 * ---
 * 
 * ## Incremental Mode with Canned Cycles
 * 
 * With G91:
 * - X, Y are incremental between holes
 * - Z is absolute (depth below R)
 * - R can be incremental (for sloped surfaces)
 * - L specifies repeat count
 * 
 * ```gcode
 * G91                               ; Incremental
 * G81 X0 Y0 Z-10 R2 L5 F200         ; 5 holes, stationary
 * G81 X10 Y0 Z-10 R2 L10 F200       ; 10 holes, X+10 each
 * G90
 * G80
 * ```
 * 
 * @see GCodeG33 for threading
 * @see GCodeProbing for G38.x
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <vector>
#include <optional>
#include <functional>

namespace GCode {

// Forward declarations
class MachineState;
class VariableSystem;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for canned cycles
 */
struct CannedCycleConfig {
    // === Retract ===
    
    /// Default retract mode (G98/G99)
    bool defaultRetractToInitial{true};  // G98
    
    /// Rapid retract rate (if not infinite)
    double rapidRetractRate{10000.0};  // mm/min
    
    // === Peck Drilling ===
    
    /// G73 chip-break retract distance
    double chipBreakRetract{0.1};  // mm
    
    /// G83 clearance above previous depth
    double peckClearance{0.5};  // mm
    
    /// Minimum peck depth
    double minPeckDepth{0.1};  // mm
    
    /// Maximum peck depth
    double maxPeckDepth{100.0};  // mm
    
    // === Tapping ===
    
    /// Enable rigid tapping
    bool enableRigidTapping{true};
    
    /// Spindle synchronization tolerance (for tapping)
    double tapSyncTolerance{0.02};  // mm
    
    /// Tap retract speed multiplier
    double tapRetractMultiplier{1.0};
    
    // === Boring ===
    
    /// Enable spindle orient (for G76, G87)
    bool enableSpindleOrient{true};
    
    /// Shift distance for fine boring (if not specified)
    double defaultBoreShift{0.5};  // mm
    
    // === Dwell ===
    
    /// Minimum dwell time
    double minDwellTime{0.0};  // seconds
    
    /// Maximum dwell time
    double maxDwellTime{60.0};  // seconds
    
    // === Safety ===
    
    /// Require R to be above Z
    bool requireRAboveZ{true};
    
    /// Require spindle running for drilling
    bool requireSpindleForDrill{true};
    
    /// Check for tool breakage (if probing available)
    bool toolBreakCheck{false};
};

// ============================================================================
// Cycle Types
// ============================================================================

/**
 * @brief Canned cycle type
 */
enum class CannedCycleType {
    NONE,           ///< No cycle active (G80)
    DRILL_PECK_G73, ///< G73 High-speed peck drill
    TAP_LEFT_G74,   ///< G74 Left-hand tapping
    BORE_FINE_G76,  ///< G76 Fine boring (spindle orient)
    DRILL_SIMPLE,   ///< G81 Simple drill
    DRILL_DWELL,    ///< G82 Drill with dwell
    DRILL_PECK_G83, ///< G83 Deep hole peck drill
    TAP_RIGHT_G84,  ///< G84 Right-hand tapping
    BORE_FEED,      ///< G85 Boring (feed out)
    BORE_STOP,      ///< G86 Boring (spindle stop)
    BORE_BACK,      ///< G87 Back boring
    BORE_MANUAL,    ///< G88 Boring (manual retract)
    BORE_DWELL      ///< G89 Boring (dwell, feed out)
};

#ifndef GCODE_CANNED_CYCLE_PARAMS_DEFINED
#define GCODE_CANNED_CYCLE_PARAMS_DEFINED
/**
 * @brief Canned cycle parameters
 */
struct CannedCycleParams {
    /// Cycle type
    CannedCycleType type{CannedCycleType::NONE};
    
    /// Position (XY or appropriate for plane)
    double x{0}, y{0};
    
    /// Bottom (Z or appropriate)
    double z{0};
    
    /// Retract plane
    double r{0};
    
    /// Initial plane (for G98)
    double initialZ{0};
    
    /// Peck depth (Q)
    double q{0};
    
    /// Dwell time in seconds (converted from P milliseconds)
    double dwell{0};
    
    /// Feed rate
    double feedRate{0};
    
    /// Repeat count
    int32_t repeatCount{1};
    
    /// Bore shift (I, J for G76/G87)
    double shiftI{0}, shiftJ{0};
    
    /// Spindle orient angle (Q for G76)
    double orientAngle{0};
    
    /// Retract mode: true = initial (G98), false = R plane (G99)
    bool retractToInitial{true};
    
    /// Current spindle speed (for tapping calculations)
    double spindleRPM{0};
    
    /// Thread pitch (for tapping, derived from F and RPM)
    double threadPitch{0};
};
#endif // GCODE_CANNED_CYCLE_PARAMS_DEFINED

// ============================================================================
// Canned Cycle State
// ============================================================================

/**
 * @brief Persistent canned cycle state
 */
struct CannedCycleState {
    /// Active cycle type (NONE if cancelled)
    CannedCycleType activeType{CannedCycleType::NONE};
    
    /// Stored parameters (modal)
    CannedCycleParams params;
    
    /// Retract mode
    bool retractToInitial{true};  // G98 = true, G99 = false
    
    /// First hole done (affects R handling)
    bool firstHoleDone{false};
    
    /// Current repetition in L sequence
    int32_t currentRepeat{0};
};

// ============================================================================
// Canned Cycle Handler
// ============================================================================

/**
 * @brief Callback for cycle motions
 * 
 * Instead of generating MotionSegments directly, cycles call this
 * callback for each motion. This allows integration with various
 * motion planners.
 */
using CycleMotionCallback = std::function<Error(
    MotionSegment::Type type,  // RAPID or LINEAR
    const Position& target,
    double feedRate,           // 0 for rapid
    double dwell               // seconds, 0 if none
)>;

/**
 * @brief Callback for spindle control in cycles
 */
using CycleSpindleCallback = std::function<Error(
    bool running,              // true = run, false = stop
    bool clockwise,            // true = CW (M3), false = CCW (M4)
    double rpm,                // 0 = use current
    std::optional<double> orientAngle  // for M19 orient
)>;

/**
 * @brief Handler for canned cycles
 */
class CannedCycleHandler {
public:
    /**
     * @brief Constructor
     */
    explicit CannedCycleHandler(const CannedCycleConfig& config = {});
    
    /**
     * @brief Set motion callback
     */
    void setMotionCallback(CycleMotionCallback callback);
    
    /**
     * @brief Set spindle callback
     */
    void setSpindleCallback(CycleSpindleCallback callback);
    
    // ========================================================================
    // Cycle Commands
    // ========================================================================
    
    /**
     * @brief Process G73 (high-speed peck drill)
     */
    Error processG73(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G74 (left-hand tap)
     */
    Error processG74(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G76 (fine boring)
     */
    Error processG76(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G80 (cancel cycle)
     */
    Error processG80(MachineState& state);
    
    /**
     * @brief Process G81 (simple drill)
     */
    Error processG81(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G82 (drill with dwell)
     */
    Error processG82(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G83 (deep peck drill)
     */
    Error processG83(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G84 (right-hand tap)
     */
    Error processG84(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G85 (boring, feed out)
     */
    Error processG85(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G86 (boring, spindle stop)
     */
    Error processG86(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G87 (back boring)
     */
    Error processG87(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G88 (boring, manual retract)
     */
    Error processG88(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G89 (boring, dwell, feed out)
     */
    Error processG89(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G98 (retract to initial)
     */
    Error processG98(MachineState& state);
    
    /**
     * @brief Process G99 (retract to R plane)
     */
    Error processG99(MachineState& state);
    
    // ========================================================================
    // Cycle Execution
    // ========================================================================
    
    /**
     * @brief Execute hole at current XY (repeat cycle)
     * 
     * Called when coordinates appear without a new G-code while
     * a cycle is modal.
     */
    Error executeRepeat(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Check if canned cycle is active
     */
    bool isCycleActive() const;
    
    /**
     * @brief Get active cycle type
     */
    CannedCycleType getActiveCycle() const;
    
    /**
     * @brief Get cycle state
     */
    const CannedCycleState& getState() const { return m_state; }
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setConfig(const CannedCycleConfig& config) { m_config = config; }
    const CannedCycleConfig& getConfig() const { return m_config; }
    
private:
    CannedCycleConfig m_config;
    CannedCycleState m_state;
    CycleMotionCallback m_motionCallback;
    CycleSpindleCallback m_spindleCallback;
    
    // Parse common cycle parameters from block
    Error parseCycleParams(
        const Block& block,
        const MachineState& state,
        CannedCycleParams& params
    );
    
    // Generate cycle motions
    Error executeDrillCycle(
        const CannedCycleParams& params,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    Error executePeckCycleG73(
        const CannedCycleParams& params,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    Error executePeckCycleG83(
        const CannedCycleParams& params,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    Error executeTapCycle(
        const CannedCycleParams& params,
        bool leftHand,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    Error executeBoringCycle(
        const CannedCycleParams& params,
        CannedCycleType type,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    // Motion helpers
    Error addRapid(const Position& target, std::vector<MotionSegment>& segments);
    Error addFeed(const Position& target, double feedRate, 
                  std::vector<MotionSegment>& segments);
    Error addDwell(double seconds, std::vector<MotionSegment>& segments);
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Calculate thread pitch from feed and RPM
 */
inline double calculateThreadPitch(double feedRate, double rpm) {
    if (rpm <= 0) return 0;
    return feedRate / rpm;  // mm/rev
}

/**
 * @brief Calculate tapping feed from pitch and RPM
 */
inline double calculateTapFeed(double pitch, double rpm) {
    return pitch * rpm;  // mm/min
}

/**
 * @brief Get string name for cycle type
 */
const char* cycleTypeName(CannedCycleType type);

} // namespace GCode
