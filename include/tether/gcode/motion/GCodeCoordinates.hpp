/**
 * @file GCodeCoordinates.hpp
 * @brief Coordinate Systems: G53-G59.3, G10, G92 (WCS and Offsets)
 * 
 * @details
 * ## Coordinate System Overview
 * 
 * G-code uses multiple coordinate systems layered on top of each other:
 * 
 * ```
 * Machine Coordinate → Work Offset → G92 Offset → Tool Offset → Program Coordinate
 *                                                              (what you program)
 * ```
 * 
 * ### Coordinate Layers
 * 
 * | Layer | Set By | Stored In | Description |
 * |-------|--------|-----------|-------------|
 * | Machine | Home | Fixed | Absolute reference |
 * | Work (WCS) | G54-G59.3 | #5221-#5386 | Part origin |
 * | G92 | G92 | #5211-#5219 | Temporary shift |
 * | Tool | G43 | Tool table | Tool length |
 * 
 * ### Calculation
 * 
 * ```
 * Machine Position = Program Position 
 *                  + Work Offset (G54, etc.)
 *                  + G92 Offset
 *                  + Tool Length Offset
 * ```
 * 
 * ---
 * 
 * ## G53 - Machine Coordinate (Non-Modal)
 * 
 * Move in machine coordinates, ignoring all offsets.
 * Only affects the line it's on.
 * 
 * ```gcode
 * G53 G0 X0 Y0 Z0         ; Move to machine home
 * G0 X10 Y10              ; This uses active WCS again
 * ```
 * 
 * **Use Cases:**
 * - Tool change position
 * - Pallet change
 * - Safe home position
 * - Accessing fixed points (tool setter, probes)
 * 
 * ```gcode
 * ; Go to tool change position
 * G53 G0 Z0               ; Machine Z0 (usually top)
 * G53 G0 X-50 Y500        ; Tool change position
 * M6
 * ```
 * 
 * ---
 * 
 * ## G54-G59.3 - Work Coordinate Systems (WCS)
 * 
 * Nine available coordinate systems, each with independent origin.
 * 
 * | G-Code | Parameters | Description |
 * |--------|------------|-------------|
 * | G54 | #5221-#5226 | WCS 1 (default) |
 * | G55 | #5241-#5246 | WCS 2 |
 * | G56 | #5261-#5266 | WCS 3 |
 * | G57 | #5281-#5286 | WCS 4 |
 * | G58 | #5301-#5306 | WCS 5 |
 * | G59 | #5321-#5326 | WCS 6 |
 * | G59.1 | #5341-#5346 | WCS 7 |
 * | G59.2 | #5361-#5366 | WCS 8 |
 * | G59.3 | #5381-#5386 | WCS 9 |
 * 
 * ### Parameter Layout (per WCS)
 * | Offset | Parameter | Axis |
 * |--------|-----------|------|
 * | +0 | X offset | X |
 * | +1 | Y offset | Y |
 * | +2 | Z offset | Z |
 * | +3 | A offset | A |
 * | +4 | B offset | B |
 * | +5 | C offset | C |
 * 
 * ### Selecting WCS
 * ```gcode
 * G54                     ; Select WCS 1
 * G55                     ; Select WCS 2
 * G59.3                   ; Select WCS 9
 * ```
 * 
 * WCS selection is modal—stays active until another is selected.
 * 
 * ### Use Cases
 * 
 * **Multiple Parts:**
 * ```gcode
 * ; Part 1 at G54
 * G54
 * G0 X0 Y0               ; Origin of part 1
 * ; ... machine part 1 ...
 * 
 * ; Part 2 at G55
 * G55
 * G0 X0 Y0               ; Origin of part 2
 * ; ... same program, different part ...
 * ```
 * 
 * **Multi-sided Machining:**
 * ```gcode
 * G54                    ; Top of part
 * ; ... machine top ...
 * 
 * ; Flip part
 * G55                    ; Bottom (different Z datum)
 * ; ... machine bottom ...
 * ```
 * 
 * ---
 * 
 * ## G10 - Set Coordinate System Data
 * 
 * Modify WCS offsets without leaving the program.
 * 
 * ### Syntax
 * ```gcode
 * G10 L2 P__ X__ Y__ Z__ A__ B__ C__   ; Absolute values
 * G10 L20 P__ X__ Y__ Z__ A__ B__ C__  ; Current position becomes this
 * ```
 * 
 * ### L2 - Set Absolute Offset
 * ```gcode
 * G10 L2 P1 X10 Y20 Z5   ; G54 origin is at machine (10, 20, 5)
 * G10 L2 P2 X100 Y0 Z0   ; G55 origin is at machine (100, 0, 0)
 * ```
 * 
 * P values:
 * - P1 = G54
 * - P2 = G55
 * - P3 = G56
 * - P4 = G57
 * - P5 = G58
 * - P6 = G59
 * - P7 = G59.1
 * - P8 = G59.2
 * - P9 = G59.3
 * 
 * ### L20 - Set Based on Current Position
 * 
 * Sets WCS so that current position becomes the specified coordinate.
 * 
 * ```gcode
 * ; I'm at some location, make this X0 Y0
 * G10 L20 P1 X0 Y0       ; G54 now has current pos as X0 Y0
 * 
 * ; Equivalent to:
 * ; G54_X_offset = machine_X - 0
 * ; G54_Y_offset = machine_Y - 0
 * ```
 * 
 * **Common Pattern - Touch Off:**
 * ```gcode
 * ; Touch tool to part corner
 * G10 L20 P1 X0 Y0       ; This is now X0 Y0 in G54
 * 
 * ; Touch tool to top of part
 * G10 L20 P1 Z0          ; This is now Z0 in G54
 * ```
 * 
 * ### L10 - Set Tool Table Entry
 * ```gcode
 * G10 L10 P1 Z1.5        ; Tool 1 Z offset = 1.5
 * G10 L10 P1 R5          ; Tool 1 radius = 5
 * ```
 * 
 * ### L11 - Set Tool Based on Current Position
 * ```gcode
 * G10 L11 P1 Z0          ; Tool 1 offset = current Z - workpiece Z
 * ```
 * 
 * ---
 * 
 * ## G92 - Coordinate System Offset
 * 
 * Shifts ALL coordinate systems by a fixed amount.
 * 
 * ```gcode
 * G92 X__ Y__ Z__        ; Set current position to these values
 * G92.1                  ; Cancel G92 (reset to zero)
 * G92.2                  ; Disable G92 (save values, but don't apply)
 * G92.3                  ; Re-enable G92 (restore saved values)
 * ```
 * 
 * ### How G92 Works
 * ```gcode
 * ; Machine is at (100, 50, 25)
 * G92 X0 Y0 Z0
 * ; Now program sees (0, 0, 0) but machine is still at (100, 50, 25)
 * ; G92 offset = (100, 50, 25)
 * ```
 * 
 * ### G92 Persistence
 * 
 * ⚠️ **Warning**: G92 offsets are:
 * - Persistent across program runs
 * - Applied to ALL work coordinate systems
 * - A common source of confusion!
 * 
 * Always clear G92 at program start:
 * ```gcode
 * G92.1                  ; Clear any previous G92 offset
 * ```
 * 
 * ### When to Use G92
 * 
 * **Good uses:**
 * - Temporary offset for repeat operations
 * - Quick adjustment without modifying WCS
 * 
 * **Better alternatives:**
 * - Use G54-G59.3 for part origins
 * - Use G10 L20 for touch-off
 * 
 * ---
 * 
 * ## Position Parameters
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | #5420-#5428 | Current X-W position (work coordinates) |
 * | #5220 | Current WCS number (0=G54, 1=G55, etc.) |
 * | #5221-#5226 | G54 X-C offsets |
 * | #5241-#5246 | G55 X-C offsets |
 * | ... | ... |
 * | #5211-#5219 | G92 offsets |
 * 
 * ---
 * 
 * ## G28, G30 - Return to Reference Point
 * 
 * ### G28 - Return via Point
 * ```gcode
 * G28                    ; Go to stored position #5161-#5166 via current
 * G28 X0 Y0              ; Go to X0 Y0, then to reference point
 * G28.1                  ; Store current position as reference
 * ```
 * 
 * Motion: Current → Intermediate (X,Y,Z in command) → Reference
 * 
 * ### G30 - Return to Secondary Reference
 * ```gcode
 * G30                    ; Go to secondary reference #5181-#5186
 * G30 P2                 ; Go to point 2
 * G30.1                  ; Store current as secondary reference
 * ```
 * 
 * **Use Case - Tool Change:**
 * ```gcode
 * G28 G91 Z0             ; Rapid Z up to reference (incremental Z0 = current)
 * G28 G91 X0 Y0          ; Then rapid XY to reference
 * ; Change tool
 * G0 X[#5001] Y[#5002]   ; Return to saved position
 * ```
 * 
 * ---
 * 
 * ## Examples
 * 
 * ### Multi-Part Setup
 * ```gcode
 * ; Setup: Set origins for 4 parts on fixture
 * ; Part 1: G54 at machine (0, 0, 0)
 * ; Part 2: G55 at machine (200, 0, 0)
 * ; Part 3: G56 at machine (0, 200, 0)
 * ; Part 4: G57 at machine (200, 200, 0)
 * 
 * G10 L2 P1 X0 Y0 Z0
 * G10 L2 P2 X200 Y0 Z0
 * G10 L2 P3 X0 Y200 Z0
 * G10 L2 P4 X200 Y200 Z0
 * 
 * ; Machine all 4 parts
 * G54
 * M98 P1000             ; Call machining subroutine
 * G55
 * M98 P1000
 * G56
 * M98 P1000
 * G57
 * M98 P1000
 * 
 * M30
 * ```
 * 
 * ### Probed Origin
 * ```gcode
 * ; Find corner and set as G54 origin
 * G0 X-10 Y-10 Z5
 * G38.2 X20 F50          ; Find X edge
 * G0 X[#5061 - 5]
 * G10 L20 P1 X0          ; Set X origin
 * 
 * G0 X-10
 * G38.2 Y20 F50          ; Find Y edge
 * G0 Y[#5062 - 5]
 * G10 L20 P1 Y0          ; Set Y origin
 * 
 * G0 Z10
 * G38.2 Z-10 F20         ; Find Z surface
 * G10 L20 P1 Z0          ; Set Z origin
 * 
 * G54                    ; Now at probed origin
 * G0 X0 Y0               ; Origin of part
 * ```
 * 
 * @see GCodeToolComp for tool length offset
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include "CoordinateTransform.hpp"
#include <array>
#include <vector>

namespace GCode {

// Forward declarations
class MachineState;
class VariableSystem;

// ============================================================================
// Constants
// ============================================================================

/// Number of work coordinate systems (G54-G59.3)
constexpr size_t NUM_WORK_COORD_SYSTEMS = 9;

/// Parameter base addresses for each WCS
constexpr int32_t WCS_PARAM_BASE[NUM_WORK_COORD_SYSTEMS] = {
    5221,  // G54 (WCS 1)
    5241,  // G55 (WCS 2)
    5261,  // G56 (WCS 3)
    5281,  // G57 (WCS 4)
    5301,  // G58 (WCS 5)
    5321,  // G59 (WCS 6)
    5341,  // G59.1 (WCS 7)
    5361,  // G59.2 (WCS 8)
    5381   // G59.3 (WCS 9)
};

/// G92 offset parameter base
constexpr int32_t G92_PARAM_BASE = 5211;

/// G28 reference point parameter base
constexpr int32_t G28_PARAM_BASE = 5161;

/// G30 reference points parameter bases (4 points)
constexpr int32_t G30_PARAM_BASE[4] = {5181, 5186, 5191, 5196};

/// Current position parameter base
constexpr int32_t CURRENT_POS_PARAM_BASE = 5420;

/// G68 coordinate rotation parameter base (custom extension).
/// #5400 = rotation active (0/1)
/// #5401 = rotation mode (0=2D, 1=Euler, 2=axis-angle)
/// #5402 = 2D rotation angle (degrees)
/// #5403 = pivot X
/// #5404 = pivot Y
/// #5405 = pivot Z
/// #5406 = Euler A angle (degrees)
/// #5407 = Euler B angle (degrees)
/// #5408 = Euler C angle (degrees)
/// #5409 = axis I
/// #5410 = axis J
/// #5411 = axis K
/// #5412 = axis-angle R (degrees)
constexpr int32_t G68_PARAM_BASE = 5400;

/// G51 scaling parameter base (custom extension).
/// #5413 = scaling active (0/1)
/// #5414 = scale X
/// #5415 = scale Y
/// #5416 = scale Z
/// #5417 = scale A
/// #5418 = scale B
/// #5419 = scale C
constexpr int32_t G51_PARAM_BASE = 5413;

// ============================================================================
// Work Coordinate System
// ============================================================================

/**
 * @brief Single work coordinate system (WCS)
 */
struct WorkCoordinateSystem {
    /// Offset from machine coordinates
    Position offset;

    /// WCS rotation angle in degrees (G10 L2 R word).
    /// Applied as a 2D rotation in the active plane when this WCS is selected.
    double rotation{0.0};

    /// WCS number (1-9)
    int32_t number{1};

    /// Name (optional, e.g., "PART_1")
    std::string name;

    /// Is modified from default
    bool modified{false};
};

/**
 * @brief Coordinate system manager
 */
class CoordinateSystemManager {
public:
    /**
     * @brief Constructor
     */
    CoordinateSystemManager();
    
    // ========================================================================
    // WCS Selection (G54-G59.3)
    // ========================================================================
    
    /**
     * @brief Select work coordinate system by G-code
     * @param gcode G54, G55, ..., G59, G59.1, G59.2, G59.3
     */
    Error selectWCS(double gcode);
    
    /**
     * @brief Select work coordinate system by number (1-9)
     */
    Error selectWCS(int32_t number);
    
    /**
     * @brief Get currently selected WCS
     */
    const WorkCoordinateSystem& getActiveWCS() const;
    
    /**
     * @brief Get WCS by number (1-9)
     */
    const WorkCoordinateSystem& getWCS(int32_t number) const;
    WorkCoordinateSystem& getWCS(int32_t number);
    
    /**
     * @brief Get active WCS number
     */
    int32_t getActiveWCSNumber() const { return m_activeWCS; }
    
    // ========================================================================
    // G92 Offset
    // ========================================================================
    
    /**
     * @brief Process G92 (set position)
     * 
     * Sets the offset so current position becomes the specified coordinate.
     */
    Error processG92(
        const Block& block,
        const Position& machinePos,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G92.1 (reset G92)
     */
    Error processG92_1(MachineState& state, VariableSystem& vars);
    
    /**
     * @brief Process G92.2 (disable G92)
     */
    Error processG92_2(MachineState& state);
    
    /**
     * @brief Process G92.3 (re-enable G92)
     */
    Error processG92_3(MachineState& state);
    
    /**
     * @brief Get G92 offset
     */
    const Position& getG92Offset() const { return m_g92Offset; }
    
    /**
     * @brief Check if G92 is active
     */
    bool isG92Active() const { return m_g92Active; }
    
    // ========================================================================
    // G53 - Machine Coordinates
    // ========================================================================
    
    /**
     * @brief Process G53 (machine coordinate mode)
     * 
     * Returns true—caller should use machine coordinates for this block only.
     */
    bool processG53();
    
    /**
     * @brief Check if in G53 mode (non-modal, check after processing)
     */
    bool inG53Mode() const { return m_g53Active; }
    
    /**
     * @brief Clear G53 mode (call after block executed)
     */
    void clearG53() { m_g53Active = false; }

    // ========================================================================
    // G52 - Local Coordinate Offset
    // ========================================================================

    /**
     * @brief Process G52 (set local coordinate offset)
     *
     * Sets a temporary offset applied in program space, before scale/rotation.
     * G52 with no axis words resets the local offset to zero.
     *
     * @param block  G-code block with X/Y/Z/A/B/C/U/V/W words
     * @param state  Machine state (updated with g52Offset)
     */
    Error processG52(const Block& block, MachineState& state);

    /**
     * @brief Reset G52 offset to zero.
     */
    Error clearG52(MachineState& state);

    // ========================================================================
    // G68 / G69 - Coordinate System Rotation
    // ========================================================================

    /**
     * @brief Process G68 (coordinate system rotation)
     *
     * Syntax variants:
     * - `G68 X__ Y__ R__`        : 2D rotation in the active plane (G17=XY
     *   about Z, G18=ZX about Y, G19=YZ about X). X/Y are the in-plane pivot.
     * - `G68 X__ Y__ Z__ A__ B__ C__` : 3D intrinsic XYZ Euler rotation
     *   about pivot (X,Y,Z). A/B/C are angles in degrees.
     * - `G68 X__ Y__ Z__ I__ J__ K__ R__` : 3D axis-angle rotation about
     *   pivot (X,Y,Z). I/J/K is the rotation axis, R is the angle in degrees.
     *
     * The rotation mode is inferred from which words are present:
     * - Only R (plus optional X/Y pivot) -> 2D plane rotation
     * - A/B/C present -> 3D Euler XYZ
     * - I/J/K present -> 3D axis-angle (R is the angle)
     *
     * @param block  G-code block
     * @param state  Machine state (updated with g68* fields)
     */
    Error processG68(const Block& block, MachineState& state);

    /**
     * @brief Process G69 (cancel coordinate rotation)
     */
    Error processG69(MachineState& state);

    // ========================================================================
    // G51 / G50 - Scaling
    // ========================================================================

    /**
     * @brief Process G51 (scaling)
     *
     * Syntax variants:
     * - `G51 P<scale>`           : uniform scale factor for all axes
     * - `G51 X__ Y__ Z__ A__ B__ C__ U__ V__ W__` : per-axis scale factors
     *
     * @param block  G-code block
     * @param state  Machine state (updated with scaleFactors)
     */
    Error processG51(const Block& block, MachineState& state);

    /**
     * @brief Process G50 (cancel scaling)
     */
    Error processG50(MachineState& state);

    // ========================================================================
    // G10 - Set Coordinate Data
    // ========================================================================
    
    /**
     * @brief Process G10 L2 (set WCS offset absolute)
     * 
     * @param pWord WCS number (1-9)
     * @param offset New offset values (only specified axes are changed)
     */
    Error processG10L2(
        int32_t pWord,
        const Block& block,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G10 L20 (set WCS based on current position)
     * 
     * Sets WCS so current position becomes specified coordinate.
     */
    Error processG10L20(
        int32_t pWord,
        const Block& block,
        const Position& machinePos,
        VariableSystem& vars
    );
    
    // ========================================================================
    // G28, G30 - Reference Points
    // ========================================================================
    
    /**
     * @brief Process G28 (return to reference)
     */
    Error processG28(
        const Block& block,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G28.1 (store reference point)
     */
    Error processG28_1(
        const Position& machinePos,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G30 (return to secondary reference)
     */
    Error processG30(
        const Block& block,
        int32_t pWord,  // Which reference (1-4)
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G30.1 (store secondary reference)
     */
    Error processG30_1(
        const Position& machinePos,
        int32_t pWord,
        VariableSystem& vars
    );
    
    /**
     * @brief Get G28 reference point
     */
    const Position& getG28Reference() const { return m_g28Reference; }
    
    /**
     * @brief Get G30 reference point
     */
    const Position& getG30Reference(int32_t point = 1) const;
    
    // ========================================================================
    // Coordinate Transformation
    // ========================================================================

    /**
     * @brief Convert program coordinates to machine coordinates
     *
     * Applies the full composed transform: scale -> rotate -> translate
     * (G52 + G92 + WCS). See @ref CoordinateTransform for the composition.
     *
     * @param programPos Program coordinates
     * @return Machine coordinates
     */
    Position toMachineCoords(const Position& programPos) const;

    /**
     * @brief Convert machine coordinates to program coordinates (inverse).
     */
    Position toProgramCoords(const Position& machinePos) const;

    /**
     * @brief Get total additive offset (WCS + G92 + G52).
     * @note This ignores rotation and scaling; it is the legacy additive
     *       offset only. For the full transform use @ref toMachineCoords.
     */
    Position getTotalOffset() const;

    /**
     * @brief Access the composed CoordinateTransform (scale + rotation +
     *        translation). Rebuilt whenever WCS/G52/G92/G68/G51 parameters
     *        change via the process* methods.
     */
    const CoordinateTransform& transform() const { return m_transform; }

    /**
     * @brief Rebuild the transform from the current MachineState.
     * Called automatically by the process* methods; can also be called
     * manually after directly modifying state fields.
     */
    void syncTransform(const MachineState& state);
    
    // ========================================================================
    // Variable Synchronization
    // ========================================================================
    
    /**
     * @brief Sync all offsets to variable system
     */
    void syncToVariables(VariableSystem& vars) const;
    
    /**
     * @brief Load offsets from variable system
     */
    void loadFromVariables(const VariableSystem& vars);
    
    /**
     * @brief Update current position variables (#5420-#5428)
     */
    void updatePositionVariables(
        const Position& machinePos,
        VariableSystem& vars
    ) const;
    
    // ========================================================================
    // Persistence
    // ========================================================================
    
    /**
     * @brief Save all WCS to file
     */
    Error saveToFile(const std::string& filename) const;
    
    /**
     * @brief Load all WCS from file
     */
    Error loadFromFile(const std::string& filename);
    
    /**
     * @brief Reset all to defaults
     */
    void reset();
    
private:
    // Work coordinate systems (index 0-8 = WCS 1-9)
    std::array<WorkCoordinateSystem, NUM_WORK_COORD_SYSTEMS> m_wcs;
    
    // Active WCS number (1-9)
    int32_t m_activeWCS{1};
    
    // G92 offset
    Position m_g92Offset;
    bool m_g92Active{false};
    Position m_g92Saved;  // For G92.2/G92.3
    
    // G53 flag (non-modal)
    bool m_g53Active{false};

    // Reference points
    Position m_g28Reference;
    std::array<Position, 4> m_g30References;

    // Composed coordinate transform (scale + rotation + translation).
    // Rebuilt by syncTransform() whenever WCS/G52/G92/G68/G51 change.
    CoordinateTransform m_transform;

    // Convert WCS number (1-9) to index (0-8)
    int32_t wcsIndex(int32_t number) const {
        return std::clamp(number, 1, 9) - 1;
    }
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert G-code (G54-G59.3) to WCS number (1-9)
 */
int32_t gcodeToWCSNumber(double gcode);

/**
 * @brief Convert WCS number (1-9) to G-code (54-59.3)
 */
double wcsNumberToGCode(int32_t number);

/**
 * @brief Get WCS name string (e.g., "G54", "G59.1")
 */
const char* wcsNumberToString(int32_t number);

/**
 * @brief Process coordinate system G-codes
 */
Error processG54(MachineState& state, CoordinateSystemManager& csm);
Error processG55(MachineState& state, CoordinateSystemManager& csm);
Error processG56(MachineState& state, CoordinateSystemManager& csm);
Error processG57(MachineState& state, CoordinateSystemManager& csm);
Error processG58(MachineState& state, CoordinateSystemManager& csm);
Error processG59(MachineState& state, CoordinateSystemManager& csm);
Error processG59_1(MachineState& state, CoordinateSystemManager& csm);
Error processG59_2(MachineState& state, CoordinateSystemManager& csm);
Error processG59_3(MachineState& state, CoordinateSystemManager& csm);

} // namespace GCode
