/**
 * @file GCodeInterpreter.hpp
 * @brief Main G-Code Interpreter - Integrates All Components
 * 
 * @details
 * ## Overview
 * 
 * The GCodeInterpreter class is the main entry point for G-code processing.
 * It integrates all subsystems:
 * 
 * - Lexer (tokenization)
 * - Parser (block parsing)
 * - Variable system
 * - O-code executor (control flow)
 * - Motion handlers (G0-G3, G5, etc.)
 * - Canned cycles
 * - Probing
 * - Tool compensation
 * - Coordinate systems
 * - Advanced features
 * 
 * ## Architecture
 * 
 * ```
 *                    ┌─────────────────────┐
 *                    │   GCodeInterpreter  │
 *                    └─────────┬───────────┘
 *                              │
 *         ┌────────────────────┼────────────────────┐
 *         │                    │                    │
 *    ┌────┴────┐         ┌─────┴─────┐        ┌─────┴─────┐
 *    │  Lexer  │         │  Parser   │        │ Variables │
 *    └────┬────┘         └─────┬─────┘        └───────────┘
 *         │                    │
 *         │              ┌─────┴─────┐
 *         │              │  O-Codes  │
 *         │              └───────────┘
 *         │
 *    ┌────┴─────────────────────────────────────────────┐
 *    │                   Motion Handlers                │
 *    │  ┌─────┐ ┌─────┐ ┌───────┐ ┌────────┐ ┌──────┐  │
 *    │  │G0/G1│ │G2/G3│ │Splines│ │Cycles  │ │Probe │  │
 *    │  └─────┘ └─────┘ └───────┘ └────────┘ └──────┘  │
 *    │  ┌─────────┐ ┌────────────┐ ┌─────────────────┐ │
 *    │  │Tool Comp│ │Coordinates │ │Advanced Motion  │ │
 *    │  └─────────┘ └────────────┘ └─────────────────┘ │
 *    └──────────────────────────────────────────────────┘
 *                              │
 *                    ┌─────────┴─────────┐
 *                    │  Motion Planner   │
 *                    │  (External/Hook)  │
 *                    └───────────────────┘
 * ```
 * 
 * ## Usage
 * 
 * ### Basic Execution
 * 
 * ```cpp
 * GCode::Interpreter interpreter;
 * 
 * // Configure
 * interpreter.setMotionCallback([](const MotionSegment& seg) {
 *     // Send to motion controller
 *     return Error{};
 * });
 * 
 * // Load and run program
 * interpreter.loadFile("program.ngc");
 * interpreter.run();
 * 
 * // Or step through
 * while (!interpreter.isFinished()) {
 *     interpreter.step();
 * }
 * ```
 * 
 * ### Line-by-Line Execution
 * 
 * ```cpp
 * interpreter.executeLine("G0 X10 Y20 Z5");
 * interpreter.executeLine("G1 X50 F500");
 * interpreter.executeLine("M30");
 * ```
 * 
 * ### MDI Mode (Manual Data Input)
 * 
 * ```cpp
 * interpreter.setMode(InterpreterMode::MDI);
 * interpreter.executeLine("G0 X0 Y0");  // Immediate execution
 * ```
 * 
 * ### Dry Run (Verification)
 * 
 * ```cpp
 * interpreter.setDryRun(true);
 * interpreter.loadFile("program.ngc");
 * if (interpreter.verify()) {
 *     // Program is valid
 * }
 * ```
 * 
 * ## Execution Model
 * 
 * ### Block Processing
 * 
 * 1. Lexer tokenizes line
 * 2. Parser creates Block
 * 3. Expression evaluation
 * 4. Modal state update
 * 5. Motion generation
 * 6. Output to planner
 * 
 * ### Lookahead
 * 
 * The interpreter maintains a block queue for:
 * - Velocity planning at corners
 * - Cutter compensation corner handling
 * - Reverse motion (negative feed rates)
 * 
 * ### Error Handling
 * 
 * | Error Level | Action |
 * |-------------|--------|
 * | Warning | Log, continue |
 * | Error | Stop block, recover |
 * | Fatal | Stop program |
 * 
 * ### Modal Groups
 * 
 * G-codes are organized into modal groups. Only one G-code per group
 * can be active at a time.
 * 
 * | Group | Description | Codes |
 * |-------|-------------|-------|
 * | 0 | Non-modal | G4, G10, G28, G30, G53, G92 |
 * | 1 | Motion | G0, G1, G2, G3, G33, G38.x, G73-G89 |
 * | 2 | Plane | G17, G18, G19 |
 * | 3 | Distance | G90, G91 |
 * | 4 | IJK Mode | G90.1, G91.1 |
 * | 5 | Feed Mode | G93, G94, G95 |
 * | 6 | Units | G20, G21 |
 * | 7 | Cutter Comp | G40, G41, G42 |
 * | 8 | Tool Length | G43, G44, G49 |
 * | 9 | Unused | |
 * | 10 | Return Mode | G98, G99 |
 * | 12 | WCS | G54-G59.3 |
 * | 13 | Path Mode | G61, G61.1, G64 |
 * | 14 | Spindle Mode | G96, G97 |
 * | 15 | Lathe Diameter | G7, G8 |
 * 
 * @see GCodeParser
 * @see GCodeTypes
 */

#pragma once

#include "GCodeTypes.hpp"
#include "GCodeConfig.hpp"
#include "GCodeVariables.hpp"
#include "GCodeLexer.hpp"
#include "GCodeParser.hpp"
#include "GCodeOCodes.hpp"
#include "motion/GCodeG0G1.hpp"
#include "motion/GCodeG2G3.hpp"
#include "motion/GCodeSplines.hpp"
#include "motion/GCodeCannedCycles.hpp"
#include "motion/GCodeProbing.hpp"
#include "motion/GCodeToolComp.hpp"
#include "motion/GCodeCoordinates.hpp"
#include "motion/GCodeAdvancedMotion.hpp"

#include <string>
#include <memory>
#include <functional>
#include <queue>
#include <optional>

namespace GCode {

// ============================================================================
// Interpreter Mode
// ============================================================================

/**
 * @brief Interpreter execution mode
 */
enum class InterpreterMode {
    AUTO,       ///< Running program from file
    MDI,        ///< Manual Data Input (single lines)
    STEP,       ///< Single-step through program
    VERIFY      ///< Verify only, no motion output
};

/**
 * @brief Interpreter state
 */
enum class InterpreterState {
    IDLE,           ///< No program loaded
    READY,          ///< Program loaded, ready to run
    RUNNING,        ///< Executing program
    PAUSED,         ///< Paused mid-execution
    FINISHED,       ///< Program completed (M30/M2)
    ERROR,          ///< Error occurred
    STOPPED         ///< Emergency stop
};

// ============================================================================
// Callbacks
// ============================================================================

/**
 * @brief Motion output callback
 * 
 * Called for each motion segment generated.
 */
using MotionCallback = std::function<Error(const MotionSegment& segment)>;

/**
 * @brief Message callback (for M100, MSG, etc.)
 */
using MessageCallback = std::function<void(const std::string& message)>;

/**
 * @brief M-code callback
 * 
 * Called for M-codes not handled internally.
 * 
 * @param mcode M-code number
 * @param pWord Optional P parameter
 * @param qWord Optional Q parameter
 * @return Error if M-code fails
 */
using MCodeCallback = std::function<Error(
    int32_t mcode,
    std::optional<double> pWord,
    std::optional<double> qWord
)>;

/**
 * @brief Spindle callback
 */
using SpindleCallback = std::function<Error(
    bool enable,
    bool clockwise,
    double rpm
)>;

/**
 * @brief Coolant callback
 */
using CoolantCallback = std::function<Error(
    bool mist,
    bool flood
)>;

/**
 * @brief Dwell callback
 */
using DwellCallback = std::function<Error(double seconds)>;

/**
 * @brief Program control callback (M0, M1, M2, M30)
 */
using ProgramControlCallback = std::function<void(
    int32_t mcode  // 0=stop, 1=optional stop, 2=end, 30=end/rewind
)>;

// ============================================================================
// Interpreter Configuration
// ============================================================================

/**
 * @brief Master interpreter configuration
 */
struct InterpreterConfig {
    /// Parser configuration
    ParserConfig parser;
    
    /// Motion configurations
    LinearMotionConfig linearMotion;
    ArcMotionConfig arcMotion;
    SplineConfig spline;
    CannedCycleConfig cannedCycles;
    ProbeConfig probe;
    CutterCompConfig cutterComp;
    PathBlendConfig pathBlend;
    TrochoidalConfig trochoidal;
    VolumetricConfig volumetric;
    
    /// Tool table
    size_t maxTools{MAX_TOOLS};
    std::string toolTableFile;
    
    /// Coordinate persistence
    std::string wcsFile;
    
    /// O-code search paths
    std::vector<std::string> subSearchPaths;
    
    /// Feature flags
    FeatureFlags features;
    
    /// Error handling
    bool stopOnError{true};
    bool skipOptionalBlocks{false};  // Block delete '/'
    bool m1OptionalStop{false};      // M1 behavior
    
    /// Execution limits
    size_t maxProgramSize{10000000};  // 10MB
    size_t maxLineLength{1024};
    uint32_t maxExecutionTime{0};     // seconds, 0 = unlimited
};

// ============================================================================
// Main Interpreter Class
// ============================================================================

/**
 * @brief Main G-code interpreter
 */
class Interpreter {
public:
    /**
     * @brief Constructor
     * @param config Interpreter configuration
     */
    explicit Interpreter(const InterpreterConfig& config = {});
    
    /**
     * @brief Destructor
     */
    ~Interpreter();
    
    // ========================================================================
    // Program Loading
    // ========================================================================
    
    /**
     * @brief Load program from file
     */
    Error loadFile(const std::string& filename);
    
    /**
     * @brief Load program from string
     */
    Error loadString(const std::string& program);
    
    /**
     * @brief Unload current program
     */
    void unload();
    
    /**
     * @brief Check if program is loaded
     */
    bool isProgramLoaded() const;
    
    /**
     * @brief Get loaded filename
     */
    const std::string& getFilename() const { return m_filename; }
    
    // ========================================================================
    // Execution Control
    // ========================================================================
    
    /**
     * @brief Run program to completion
     */
    Error run();
    
    /**
     * @brief Execute single step
     */
    Error step();
    
    /**
     * @brief Pause execution
     */
    void pause();
    
    /**
     * @brief Resume from pause
     */
    Error resume();
    
    /**
     * @brief Stop execution
     */
    void stop();
    
    /**
     * @brief Reset interpreter state
     */
    void reset();
    
    /**
     * @brief Execute single line (MDI mode)
     */
    Error executeLine(const std::string& line);
    
    /**
     * @brief Verify program without executing
     */
    Error verify();
    
    // ========================================================================
    // State
    // ========================================================================
    
    /**
     * @brief Get interpreter state
     */
    InterpreterState getState() const { return m_state; }
    
    /**
     * @brief Check if finished
     */
    bool isFinished() const;
    
    /**
     * @brief Get current line number
     */
    uint32_t getCurrentLine() const;
    
    /**
     * @brief Get total lines
     */
    uint32_t getTotalLines() const;
    
    /**
     * @brief Get machine state
     */
    const MachineState& getMachineState() const { return m_machineState; }
    MachineState& getMachineState() { return m_machineState; }
    
    /**
     * @brief Get last error
     */
    const Error& getLastError() const { return m_lastError; }
    
    /**
     * @brief Get error history
     */
    const std::vector<Error>& getErrors() const { return m_errors; }
    
    // ========================================================================
    // Mode Control
    // ========================================================================
    
    /**
     * @brief Set execution mode
     */
    void setMode(InterpreterMode mode) { m_mode = mode; }
    InterpreterMode getMode() const { return m_mode; }
    
    /**
     * @brief Set dry run (no motion output)
     */
    void setDryRun(bool dryRun) { m_dryRun = dryRun; }
    bool isDryRun() const { return m_dryRun; }
    
    /**
     * @brief Set block delete mode
     */
    void setBlockDelete(bool enabled);
    bool isBlockDeleteEnabled() const;
    
    /**
     * @brief Set optional stop mode
     */
    void setOptionalStop(bool enabled);
    bool isOptionalStopEnabled() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setMotionCallback(MotionCallback callback);
    void setMessageCallback(MessageCallback callback);
    void setMCodeCallback(MCodeCallback callback);
    void setSpindleCallback(SpindleCallback callback);
    void setCoolantCallback(CoolantCallback callback);
    void setDwellCallback(DwellCallback callback);
    void setProgramControlCallback(ProgramControlCallback callback);
    void setToolChangeCallback(ToolChangeCallback callback);
    void setProbeCallback(ProbeMotionCallback callback);
    
    // ========================================================================
    // Component Access
    // ========================================================================
    
    /**
     * @brief Get variable system
     */
    VariableSystem& getVariables() { return m_variables; }
    const VariableSystem& getVariables() const { return m_variables; }
    
    /**
     * @brief Get tool table
     */
    ToolTable& getToolTable() { return m_toolTable; }
    const ToolTable& getToolTable() const { return m_toolTable; }
    
    /**
     * @brief Get coordinate system manager
     */
    CoordinateSystemManager& getCoordinates() { return m_coordinates; }
    const CoordinateSystemManager& getCoordinates() const { return m_coordinates; }
    
    /**
     * @brief Get O-code executor
     */
    OCodeExecutor& getOCodeExecutor() { return *m_oCodeExecutor; }
    
    /**
     * @brief Get override controller
     */
    OverrideController& getOverrides() { return m_overrides; }
    const OverrideController& getOverrides() const { return m_overrides; }
    
    // ========================================================================
    // Position
    // ========================================================================
    
    /**
     * @brief Get current position (program coordinates)
     */
    Position getCurrentPosition() const;
    
    /**
     * @brief Get current position (machine coordinates)
     */
    Position getMachinePosition() const;
    
    /**
     * @brief Set position (for homing, etc.)
     */
    void setPosition(const Position& pos, bool machineCoords = false);
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Get configuration
     */
    const InterpreterConfig& getConfig() const { return m_config; }
    
    /**
     * @brief Update configuration
     */
    void setConfig(const InterpreterConfig& config);
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    struct Statistics {
        uint32_t linesProcessed{0};
        uint32_t blocksExecuted{0};
        uint32_t motionSegments{0};
        uint32_t errors{0};
        uint32_t warnings{0};
        double executionTime{0};  // seconds
        double totalPathLength{0};  // mm
    };
    
    const Statistics& getStatistics() const { return m_stats; }
    void resetStatistics();
    
private:
    // Configuration
    InterpreterConfig m_config;
    
    // Core components
    std::unique_ptr<Lexer> m_lexer;
    std::unique_ptr<Parser> m_parser;
    VariableSystem m_variables;
    std::unique_ptr<OCodeExecutor> m_oCodeExecutor;
    
    // Motion handlers
    LinearMotionHandler m_linearMotion;
    ArcMotionHandler m_arcMotion;
    SplineHandler m_splineHandler;
    CannedCycleHandler m_cannedCycles;
    ProbeHandler m_probeHandler;
    ToolLengthComp m_toolLengthComp;
    CutterRadiusComp m_cutterRadiusComp;
    CoordinateSystemManager m_coordinates;
    PathBlender m_pathBlender;
    TrochoidalHandler m_trochoidalHandler;
    VolumetricCompensation m_volumetricComp;
    BacklashCompensation m_backlashComp;
    AdaptiveFeedController m_adaptiveFeed;
    OverrideController m_overrides;
    
    // Tool management
    ToolTable m_toolTable;
    
    // State
    InterpreterMode m_mode{InterpreterMode::AUTO};
    InterpreterState m_state{InterpreterState::IDLE};
    MachineState m_machineState;
    bool m_dryRun{false};
    std::string m_filename;
    std::string m_programSource;
    
    // Error handling
    Error m_lastError;
    std::vector<Error> m_errors;
    
    // Callbacks
    MotionCallback m_motionCallback;
    MessageCallback m_messageCallback;
    MCodeCallback m_mcodeCallback;
    SpindleCallback m_spindleCallback;
    CoolantCallback m_coolantCallback;
    DwellCallback m_dwellCallback;
    ProgramControlCallback m_programCallback;
    ToolChangeCallback m_toolChangeCallback;
    
    // Statistics
    Statistics m_stats;
    
    // Execution
    Error executeBlock(const Block& block);
    Error processGCodes(const Block& block, std::vector<MotionSegment>& segments);
    Error processMCodes(const Block& block);
    Error handleMotion(const Block& block, std::vector<MotionSegment>& segments);
    Error outputSegments(const std::vector<MotionSegment>& segments);
    
    // G-code dispatch
    Error dispatchGCode(double gcode, const Block& block, 
                        std::vector<MotionSegment>& segments);
    
    // M-code dispatch
    Error dispatchMCode(int32_t mcode, const Block& block);
    
    // State management
    void updateModalState(const Block& block);
    void updatePositionVariables();

    // Initialization
    void initializeDefaults();

    // ------------------------------------------------------------------
    // Coordinate system dispatch helpers (G52/G68/G69/G51/G50)
    //
    // These wire the parsed G-code words into the CoordinateSystemManager,
    // which in turn rebuilds the composed CoordinateTransform. The
    // interpreter's motion handlers should call m_coordinates.toMachineCoords()
    // to transform program coordinates to machine coordinates before
    // emitting MotionSegments.
    // ------------------------------------------------------------------

    /// @brief Dispatch G52 (local offset) to the coordinate manager.
    Error dispatchG52(const Block& block) {
        return m_coordinates.processG52(block, m_machineState);
    }

    /// @brief Dispatch G68 (coordinate rotation) to the coordinate manager.
    Error dispatchG68(const Block& block) {
        return m_coordinates.processG68(block, m_machineState);
    }

    /// @brief Dispatch G69 (cancel rotation) to the coordinate manager.
    Error dispatchG69() {
        return m_coordinates.processG69(m_machineState);
    }

    /// @brief Dispatch G51 (scaling) to the coordinate manager.
    Error dispatchG51(const Block& block) {
        return m_coordinates.processG51(block, m_machineState);
    }

    /// @brief Dispatch G50 (cancel scaling) to the coordinate manager.
    Error dispatchG50() {
        return m_coordinates.processG50(m_machineState);
    }

    /// @brief Transform a program-space position to machine coordinates
    /// using the composed coordinate transform (WCS + G52 + G92 + G68 + G51).
    Position toMachine(const Position& programPos) const {
        return m_coordinates.toMachineCoords(programPos);
    }

    /// @brief Transform machine coordinates back to program space.
    Position toProgram(const Position& machinePos) const {
        return m_coordinates.toProgramCoords(machinePos);
    }
};

} // namespace GCode
