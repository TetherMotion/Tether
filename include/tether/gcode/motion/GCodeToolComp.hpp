/**
 * @file GCodeToolComp.hpp
 * @brief Tool Compensation: G40-G42 (Cutter Comp), G43-G49 (Tool Length)
 * 
 * @details
 * ## Tool Length Compensation (G43, G44, G49)
 * 
 * Tool length offset (TLO) compensates for different tool lengths,
 * allowing programs to be written to the part surface.
 * 
 * ### G43 - Tool Length Offset + (Add)
 * 
 * ```gcode
 * G43 H__    ; Apply offset from tool table, slot H
 * G43 Z__    ; Dynamic offset (LinuxCNC extension)
 * G43.1 Z__  ; Dynamic offset (standard)
 * G43.2 H__  ; Additional offset (stacking)
 * ```
 * 
 * #### Usage
 * ```gcode
 * T1 M6                  ; Load tool 1
 * G43 H1                 ; Apply tool 1's length offset
 * G0 Z0                  ; Move to Z0 (part surface + offset)
 * 
 * ; Or with dynamic offset:
 * G43.1 Z50.5            ; Apply 50.5mm offset directly
 * ```
 * 
 * #### How It Works
 * ```
 * Program Z coordinate
 *        ↓
 * Z_machine = Z_program + TLO
 * 
 * Example: G0 Z0 with TLO=50.5
 *          Machine moves to Z50.5
 * ```
 * 
 * ### G43.1 - Dynamic Tool Length Offset
 * 
 * Apply offset directly without tool table lookup.
 * 
 * ```gcode
 * G43.1 Z-25.4           ; Apply -25.4mm offset
 * G43.1 Z[#5063]         ; Use probed Z as offset
 * ```
 * 
 * ### G43.2 - Additional Offset
 * 
 * Add to current offset (for wear adjustment, probing, etc.)
 * 
 * ```gcode
 * G43 H1                 ; Base offset
 * G43.2 H2               ; Add tool 2's offset to tool 1's
 * ```
 * 
 * ### G44 - Tool Length Offset - (Subtract)
 * 
 * Rarely used. Subtracts offset instead of adding.
 * 
 * ### G49 - Cancel Tool Length Offset
 * 
 * ```gcode
 * G49                    ; TLO = 0
 * ```
 * 
 * ---
 * 
 * ## Cutter Radius Compensation (G40-G42)
 * 
 * CRC (also called CDC - Cutter Diameter Compensation) offsets the
 * toolpath perpendicular to motion, compensating for cutter size.
 * 
 * ### G40 - Cancel Cutter Compensation
 * 
 * ```gcode
 * G40                    ; Disable CRC
 * ```
 * 
 * ### G41 - Cutter Compensation Left
 * 
 * Tool is LEFT of programmed path (looking in direction of motion).
 * 
 * ```gcode
 * G41 D__               ; D = diameter register (tool table)
 * G41 D0                ; D0 = current tool's diameter
 * G41.1 D__             ; D = actual diameter value
 * ```
 * 
 * ### G42 - Cutter Compensation Right
 * 
 * Tool is RIGHT of programmed path.
 * 
 * ```gcode
 * G42 D__               ; Same as G41 but offset right
 * G42.1 D__             ; Dynamic diameter
 * ```
 * 
 * ### CRC Model
 * 
 * ```
 * Direction of cut →
 * 
 * G41 (Left):
 *        ┌──────────────────┐
 *        │   MATERIAL       │
 *        │                  │
 * Path → ├─ ─ ─ ─ ─ ─ ─ ─ ─┤ ← Programmed path
 *        │    (Tool center  │
 *        │     follows      │
 *        │     offset path) │
 *        └──────────────────┘
 *           ↑
 *        Tool center path (offset by radius)
 * 
 * G42 (Right):
 *        ┌──────────────────┐
 *        │     Tool center  │
 *        │     follows      │
 *        │     offset path  │
 * Path → ├─ ─ ─ ─ ─ ─ ─ ─ ─┤ ← Programmed path  
 *        │   MATERIAL       │
 *        │                  │
 *        └──────────────────┘
 * ```
 * 
 * ### Entry and Exit Moves
 * 
 * CRC requires specific entry/exit handling:
 * 
 * 1. **Entry Move**: First move after G41/G42 must be long enough
 *    for the tool to ramp to the offset position.
 * 
 * 2. **Exit Move**: Must exit with G40 before ending or changing
 *    to non-cutter comp moves.
 * 
 * ```gcode
 * ; Correct entry
 * G0 X-10 Y-10           ; Position outside part
 * G41 D1                 ; Enable left compensation
 * G1 X0 Y0 F500          ; Entry move (line) - comp ramps on
 * G1 X100 Y0             ; Cutting
 * ...
 * G40                    ; Cancel before exit
 * G0 X-10 Y-10           ; Exit move
 * ```
 * 
 * ### Inside vs Outside Corners
 * 
 * ```
 * Outside corner:        Inside corner:
 * Tool arcs around       Tool goes straight
 * 
 *     ↗                     ↗
 *    /                     │
 *   ● → →                  └ ● → →
 *      (arc)                  (intersection)
 * ```
 * 
 * - **Outside corners**: Tool path includes arc to maintain contact
 * - **Inside corners**: Tool overshoots to fully clean corner
 * 
 * ### Gouging Detection
 * 
 * If tool radius is too large for inside corner, gouging would occur.
 * Controller should:
 * 1. Generate error (strict mode)
 * 2. Limit depth (lenient mode)
 * 3. Use smaller tool
 * 
 * ```
 * Gouge condition:
 *       ↓
 *   ┌───●───┐   Tool too big for slot
 *   │       │   Would remove material on opposite side
 *   └───────┘
 * ```
 * 
 * ### Two-Move Lookahead
 * 
 * CRC requires looking at the next two moves to properly compute
 * the offset path, especially at corners.
 * 
 * ---
 * 
 * ## Tool Table
 * 
 * Tools are stored in a table with:
 * - Tool number
 * - Pocket number
 * - Length (Z offset)
 * - Diameter
 * - Orientation (lathe)
 * - Wear offsets
 * 
 * ### Accessing Tool Data
 * 
 * LinuxCNC provides parameters:
 * 
 * | Parameter | Description |
 * |-----------|-------------|
 * | #5400 | Current tool number |
 * | #5401 | Current tool pocket |
 * | #5402 | Tool X offset |
 * | #5403 | Tool Y offset |
 * | #5404 | Tool Z offset |
 * | #5410 | Tool diameter |
 * | #5411 | Front angle |
 * | #5412 | Back angle |
 * | #5413 | Orientation |
 * 
 * ---
 * 
 * ## Examples
 * 
 * ### Pocket with CRC
 * ```gcode
 * ; Mill square pocket with cutter comp
 * #<depth> = -10
 * #<size> = 50
 * 
 * G0 X-10 Y[#<size>/2]          ; Position
 * G0 Z5
 * G1 Z#<depth> F100             ; Plunge
 * 
 * G41 D1                        ; Left comp, tool 1's diameter
 * G1 X0 F500                    ; Entry move
 * G1 Y0                         ; First side
 * G1 X#<size>                   ; Second side
 * G1 Y#<size>                   ; Third side
 * G1 X0                         ; Fourth side
 * G1 Y[#<size>/2]               ; Return
 * G40                           ; Cancel comp
 * G1 X-10                       ; Exit move
 * G0 Z10
 * ```
 * 
 * ### Tool Length Touch-Off
 * ```gcode
 * ; Probe tool and set offset
 * G0 X[#<setter_x>] Y[#<setter_y>]
 * G0 Z[#<setter_z> + 50]
 * G38.2 Z[#<setter_z> - 10] F50
 * #<measured_z> = #5063
 * G43.1 Z[#<measured_z> - #<setter_z>]
 * G0 Z10
 * ```
 * 
 * @see GCodeProbing for tool measurement
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <vector>
#include <optional>
#include <array>

namespace GCode {

// Forward declarations
struct MachineState;
class VariableSystem;

// ============================================================================
// Tool Table
// ============================================================================

/**
 * @brief Tool entry in tool table
 */
struct ToolEntry {
    int32_t toolNumber{0};
    int32_t pocketNumber{0};
    
    // Length offsets
    double xOffset{0};
    double yOffset{0};
    double zOffset{0};
    double aOffset{0};
    double bOffset{0};
    double cOffset{0};
    double uOffset{0};
    double vOffset{0};
    double wOffset{0};
    
    // Cutter geometry
    double diameter{0};
    double radius{0};  // = diameter / 2
    
    // Lathe orientation
    int32_t orientation{0};  // 0-9, lathe tool orientation
    double frontAngle{0};    // Lathe front angle
    double backAngle{0};     // Lathe back angle
    
    // Wear offsets (added to base)
    double xWear{0};
    double yWear{0};
    double zWear{0};
    double diameterWear{0};
    
    // Metadata
    std::string comment;
    
    /**
     * @brief Get effective radius (including wear)
     * @return (diameter + diameterWear) / 2.
     */
    double getEffectiveRadius() const {
        return (diameter + diameterWear) / 2.0;
    }
    
    /**
     * @brief Get effective Z offset (including wear)
     * @return zOffset + zWear.
     */
    double getEffectiveZOffset() const {
        return zOffset + zWear;
    }
};

/**
 * @brief Tool table manager
 */
class ToolTable {
public:
    /**
     * @brief Constructor
     * @param maxTools Maximum tool slots
     */
    explicit ToolTable(size_t maxTools = MAX_TOOLS);
    
    /**
     * @brief Get tool entry
     * @param toolNumber Tool number (T word value).
     * @return Entry, or nullptr when the tool is not defined.
     */
    const ToolEntry* getTool(int32_t toolNumber) const;
    /// @param toolNumber Tool number (T word value).
    /// @return Mutable entry, or nullptr when the tool is not defined.
    ToolEntry* getTool(int32_t toolNumber);
    
    /**
     * @brief Set tool entry
     * @param toolNumber Tool number to define or replace.
     * @param entry Tool geometry, offsets and wear values.
     * @return Error::OK, or an out-of-range error.
     */
    Error setTool(int32_t toolNumber, const ToolEntry& entry);
    
    /**
     * @brief Get tool by pocket
     * @param pocket Pocket (slot) number.
     * @return Entry loaded in that pocket, or nullptr when empty.
     */
    const ToolEntry* getToolByPocket(int32_t pocket) const;
    
    /**
     * @brief Get current tool number
     * @return Tool number selected by the last M6 (0 = none).
     */
    int32_t getCurrentTool() const { return m_currentTool; }
    
    /**
     * @brief Set current tool (after M6)
     * @param toolNumber Tool number that is now in the spindle.
     */
    void setCurrentTool(int32_t toolNumber);
    
    /**
     * @brief Get current tool entry
     * @return Entry of the tool in the spindle, or nullptr when none.
     */
    const ToolEntry* getCurrentToolEntry() const;
    
    /**
     * @brief Load tool table from file
     * @param filename Path of the tool table file.
     * @return Error::OK, or a file/parse error.
     */
    Error loadFromFile(const std::string& filename);
    
    /**
     * @brief Save tool table to file
     * @param filename Destination path.
     * @return Error::OK, or a file I/O error.
     */
    Error saveToFile(const std::string& filename) const;
    
    /**
     * @brief Clear all tools
     */
    void clear();
    
    /**
     * @brief Get number of defined tools
     * @return Count of non-empty tool slots.
     */
    size_t getToolCount() const;
    
private:
    std::vector<ToolEntry> m_tools;
    int32_t m_currentTool{0};
};

// ============================================================================
// Tool Length Compensation
// ============================================================================

/**
 * @brief Tool length compensation state
 */
struct ToolLengthCompState {
    bool active{false};
    
    // Active offset (from G43, G43.1, G44)
    double xOffset{0};
    double yOffset{0};
    double zOffset{0};
    double aOffset{0};
    double bOffset{0};
    double cOffset{0};
    
    // Additional offset (from G43.2)
    double additionalZ{0};
    
    // Source of offset
    int32_t hWord{0};  // Tool table slot (0 = dynamic)
    bool isDynamic{false};
    bool isNegative{false};  // G44 mode
    
    /**
     * @brief Get total Z offset
     * @return Signed tool-length offset plus the additional (G43.2) offset.
     */
    double getTotalZOffset() const {
        double sign = isNegative ? -1.0 : 1.0;
        return (zOffset * sign) + additionalZ;
    }
};

/**
 * @brief Tool length compensation handler
 */
class ToolLengthComp {
public:
    /**
     * @brief Constructor
     * @param toolTable Table supplying tool lengths; must outlive this object.
     */
    explicit ToolLengthComp(ToolTable& toolTable);
    
    /**
     * @brief Process G43 (TLO positive)
     * @param block G-code block (H word selects the tool slot).
     * @param state Machine state (TLO state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK, or an unknown-tool error.
     */
    Error processG43(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G43.1 (dynamic TLO)
     * @param block G-code block with axis words giving the offset directly.
     * @param state Machine state (TLO state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK on success.
     */
    Error processG43_1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G43.2 (additional offset)
     * @param block G-code block with the additional offset axis words.
     * @param state Machine state (TLO state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK on success.
     */
    Error processG43_2(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G44 (TLO negative)
     * @param block G-code block (H word selects the tool slot).
     * @param state Machine state (TLO state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK, or an unknown-tool error.
     */
    Error processG44(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G49 (cancel TLO)
     * @param state Machine state (TLO cleared).
     * @return Error::OK on success.
     */
    Error processG49(MachineState& state);
    
    /**
     * @brief Apply TLO to position
     * @param programPos Program coordinates.
     * @return Machine coordinates with the tool-length offset applied.
     */
    Position applyOffset(const Position& programPos) const;
    
    /**
     * @brief Remove TLO from position
     * @param machinePos Machine coordinates.
     * @return Program coordinates with the tool-length offset removed.
     */
    Position removeOffset(const Position& machinePos) const;
    
    /**
     * @brief Get current state
     * @return Active TLO state (offset, source, sign).
     */
    const ToolLengthCompState& getState() const { return m_state; }
    
    /**
     * @brief Check if TLO is active
     * @return true when a G43/G44 offset is currently applied.
     */
    bool isActive() const { return m_state.active; }
    
private:
    ToolTable& m_toolTable;
    ToolLengthCompState m_state;
};

// ============================================================================
// Cutter Radius Compensation
// ============================================================================

/**
 * @brief Cutter compensation side
 */
enum class CutterCompSide {
    OFF,    ///< G40
    LEFT,   ///< G41
    RIGHT   ///< G42
};

/**
 * @brief Cutter compensation state
 */
struct CutterCompState {
    CutterCompSide side{CutterCompSide::OFF};
    
    double radius{0};         // Active cutter radius
    int32_t dWord{0};         // D register (0 = current tool)
    bool isDynamic{false};    // G41.1/G42.1 mode
    
    // Entry/exit state
    bool isRamping{false};    // In entry move
    int movesSinceEnable{0};  // Moves since G41/G42
    
    // Previous path for corner calculations
    Position lastPos;
    Position lastDir;  // Unit direction of last move
    bool hasLastMove{false};
};

/**
 * @brief Configuration for cutter compensation
 */
struct CutterCompConfig {
    /// Entry move minimum length (multiple of radius)
    double minEntryLength{2.0};
    
    /// Exit move minimum length
    double minExitLength{2.0};
    
    /// Enable gouge checking
    bool checkGouging{true};
    
    /// Error on gouge vs. limit depth
    bool errorOnGouge{true};
    
    /// Arc step for outside corner arcs
    double cornerArcStep{0.1};  // mm
    
    /// Enable lookahead for corner calculation
    bool useLookahead{true};
    
    /// Lookahead moves for corner planning
    int lookaheadMoves{2};
};

/**
 * @brief Cutter radius compensation handler
 */
class CutterRadiusComp {
public:
    /**
     * @brief Constructor
     * @param toolTable Table supplying tool radii; must outlive this object.
     * @param config Corner handling and gouge-check settings.
     */
    explicit CutterRadiusComp(ToolTable& toolTable,
                              const CutterCompConfig& config = {});
    
    /**
     * @brief Process G40 (cancel)
     * @param block G-code block (unused axis words ignored).
     * @param state Machine state (compensation state cleared).
     * @param segments Motion segments appended for the lead-out move.
     * @return Error::OK on success.
     */
    Error processG40(
        const Block& block,
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G41 (left compensation)
     * @param block G-code block (D word selects the tool).
     * @param state Machine state (compensation state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK, or an unknown-tool error.
     */
    Error processG41(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G41.1 (dynamic left)
     * @param block G-code block with the D radius word.
     * @param state Machine state (compensation state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK on success.
     */
    Error processG41_1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G42 (right compensation)
     * @param block G-code block (D word selects the tool).
     * @param state Machine state (compensation state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK, or an unknown-tool error.
     */
    Error processG42(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G42.1 (dynamic right)
     * @param block G-code block with the D radius word.
     * @param state Machine state (compensation state updated).
     * @param vars Variable system (#5400-#5413 updated).
     * @return Error::OK on success.
     */
    Error processG42_1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Apply compensation to motion segment
     * 
     * @param start Programmed path start point.
     * @param end Programmed path end point.
     * @param nextEnd Next path's end point (corner calculation); nullptr when
     *        unknown.
     * @param[out] compensatedPath Output compensated segments
     * @return Error if compensation fails (gouge, etc.)
     */
    Error applyCompensation(
        const Position& start,
        const Position& end,
        const Position* nextEnd,  // nullptr if not known
        std::vector<MotionSegment>& compensatedPath
    );
    
    /**
     * @brief Get current state
     * @return Active compensation state (side, radius, corner mode).
     */
    const CutterCompState& getState() const { return m_state; }
    
    /**
     * @brief Check if compensation is active
     * @return true when G41/G42 is in effect.
     */
    bool isActive() const { return m_state.side != CutterCompSide::OFF; }
    
    /**
     * @brief Get active radius
     * @return Effective tool radius used for compensation.
     */
    double getRadius() const { return m_state.radius; }
    
    /**
     * @brief Check for gouging
     * 
     * @param p1_start First path segment start point.
     * @param p1_end First path segment end point.
     * @param p2_start Second path segment start point.
     * @param p2_end Second path segment end point.
     * @param radius Tool radius.
     * @return true if gouge would occur
     */
    bool checkGouge(
        const Position& p1_start,
        const Position& p1_end,
        const Position& p2_start,
        const Position& p2_end,
        double radius
    );
    
private:
    ToolTable& m_toolTable;
    CutterCompConfig m_config;
    CutterCompState m_state;
    
    Error enableComp(
        CutterCompSide side,
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        bool dynamic
    );
    
    // Calculate offset position
    Position calculateOffsetPosition(
        const Position& pos,
        const Position& direction,
        double radius,
        CutterCompSide side
    );
    
    // Handle corner transitions
    Error handleCorner(
        const Position& current,
        const Position& next,
        const Position& afterNext,
        std::vector<MotionSegment>& output
    );
    
    // Generate arc for outside corner
    void generateCornerArc(
        const Position& center,
        const Position& start,
        const Position& end,
        double radius,
        bool clockwise,
        std::vector<MotionSegment>& output
    );
};

// ============================================================================
// Tool Change
// ============================================================================

/**
 * @brief Tool change callback
 */
using ToolChangeCallback = std::function<Error(
    int32_t newTool,
    int32_t oldTool,
    MachineState& state
)>;

/**
 * @brief Process T word (tool select)
 * @param toolNumber Tool number from the T word.
 * @param state Machine state (selected tool updated).
 * @param toolTable Table used to validate the selection.
 * @return Error::OK, or an unknown-tool error.
 */
Error processTWord(
    int32_t toolNumber,
    MachineState& state,
    ToolTable& toolTable
);

/**
 * @brief Process M6 (tool change)
 * @param state Machine state (current tool updated on success).
 * @param toolTable Table providing the new tool's geometry.
 * @param callback Optional machine-specific change sequence (ATC moves).
 * @return Error::OK, or the callback's error.
 */
Error processM6(
    MachineState& state,
    ToolTable& toolTable,
    ToolChangeCallback callback = nullptr
);

// ============================================================================
// Tool Compensation Variables
// ============================================================================

/**
 * @brief Update tool-related variables (#5400-#5413)
 * @param vars Variable system receiving the values.
 * @param toolTable Table supplying the tool data.
 * @param tloState Active tool-length compensation state.
 * @param crcState Active cutter-radius compensation state.
 */
void updateToolVariables(
    VariableSystem& vars,
    const ToolTable& toolTable,
    const ToolLengthCompState& tloState,
    const CutterCompState& crcState
);

} // namespace GCode
