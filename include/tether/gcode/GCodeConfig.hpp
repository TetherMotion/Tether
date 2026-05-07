/**
 * @file GCodeConfig.hpp
 * @brief G-Code Interpreter Configuration
 * 
 * @details
 * This file provides the configuration interface for the G-Code interpreter.
 * All behavior can be customized through the configuration structure.
 * 
 * ## Configuration Categories
 * 
 * ### Machine Configuration
 * - Axis limits (soft limits)
 * - Maximum velocities and accelerations
 * - Home positions
 * - Axis mappings
 * 
 * ### Motion Configuration
 * - Default feed rates
 * - Rapid rate limits
 * - Acceleration profiles
 * - Jerk limits
 * - Path blending tolerances
 * - Lookahead depth
 * 
 * ### Compensation Configuration
 * - Tool table size
 * - Cutter compensation parameters
 * - Backlash compensation values
 * - Volumetric compensation grids
 * 
 * ### Parser Configuration
 * - LinuxCNC compatibility mode
 * - O-code style (numbered/named)
 * - Expression evaluation options
 * - Comment handling
 * 
 * ## Usage Example
 * 
 * ```cpp
 * GCode::Config config;
 * 
 * // Set machine limits
 * config.axes[Axis::X].minLimit = -500.0;
 * config.axes[Axis::X].maxLimit = 500.0;
 * config.axes[Axis::X].maxVelocity = 10000.0;  // mm/min
 * config.axes[Axis::X].maxAcceleration = 500.0; // mm/s²
 * 
 * // Enable features
 * config.features.cutterCompensation = true;
 * config.features.toolLengthOffset = true;
 * config.features.backlashCompensation = true;
 * 
 * // Set tolerances
 * config.motion.defaultBlendTolerance = 0.01;  // mm
 * config.motion.arcTolerance = 0.001;          // mm
 * 
 * GCode::Interpreter interp(config);
 * ```
 * 
 * ## Configuration Persistence
 * 
 * Configuration can be saved to and loaded from NVS or JSON format.
 * 
 * @see GCodeInterpreter
 * @see GCodeTypes.hpp
 */

#pragma once

#include "GCodeTypes.hpp"
#include <array>
#include <functional>

namespace GCode {

// ============================================================================
// Axis Configuration
// ============================================================================

/**
 * @brief Configuration for a single axis
 */
struct AxisConfig {
    /// Axis enabled
    bool enabled{true};
    
    /// Soft limits
    double minLimit{-1e9};
    double maxLimit{1e9};
    bool limitsEnabled{false};
    
    /// Velocity limits (units/min)
    double maxVelocity{10000.0};
    double defaultRapidVelocity{5000.0};
    
    /// Acceleration (units/s²)
    double maxAcceleration{500.0};
    
    /// Jerk limit (units/s³), 0 = unlimited
    double maxJerk{0.0};
    
    /// Home position
    double homePosition{0.0};
    int8_t homeSequence{0};  // 0 = no home, negative = simultaneous
    
    /// Backlash compensation
    double backlash{0.0};
    
    /// Steps per unit (for step-based systems)
    double stepsPerUnit{200.0};
    
    /// Rotary axis specific
    bool isRotary{false};
    bool isWrapped{false};      // Wraps 0-360
    double wrapAngle{360.0};
    
    /// Secondary axis mapping (for U/V/W to X/Y/Z)
    int8_t mappedAxis{-1};      // -1 = not mapped
};

// ============================================================================
// Motion Configuration
// ============================================================================

/**
 * @brief Motion planning configuration
 */
struct MotionConfig {
    /// Default feed rate (units/min)
    double defaultFeedRate{100.0};
    
    /// Maximum feed rate override (percentage/100)
    double maxFeedOverride{2.0};
    
    /// Rapid rate (percentage of max velocity)
    double rapidRate{1.0};
    
    /// Arc/spline linearization tolerance
    double arcTolerance{0.001};
    
    /// Path blending tolerance (G64 P default)
    double defaultBlendTolerance{0.01};
    
    /// Naive CAM tolerance (G64 Q default)
    double defaultNaiveCamTolerance{0.0};
    
    /// Corner rounding radius limit
    double maxCornerRadius{100.0};
    
    /// Lookahead buffer depth
    size_t lookaheadDepth{MAX_LOOKAHEAD};
    
    /// Lookbehind for reverse feed rates
    size_t lookbehindDepth{MAX_LOOKBEHIND};
    
    /// Enable adaptive feedrate control
    bool adaptiveFeedEnabled{true};
    
    /// Minimum feedrate (never go below this)
    double minimumFeedRate{1.0};
    
    /// Acceleration time constant (for S-curve)
    double accelerationTimeConstant{0.0};
    
    /// Trajectory cycle time (seconds)
    double cycleTime{0.001};
};

// ============================================================================
// Coordinate System Configuration
// ============================================================================

/**
 * @brief Coordinate system configuration
 */
struct CoordinateConfig {
    /// Default coordinate system
    CoordSystem defaultSystem{CoordSystem::G54};
    
    /// Persist G92 offsets
    bool persistG92{true};
    
    /// Default distance mode
    DistanceMode defaultDistanceMode{DistanceMode::ABSOLUTE};
    
    /// Default arc distance mode
    ArcDistanceMode defaultArcMode{ArcDistanceMode::INCREMENTAL};
    
    /// Default units
    Units defaultUnits{Units::MM};
    
    /// Units conversion factor (internal units/mm)
    double unitsPerMM{1.0};
};

// ============================================================================
// Tool Configuration
// ============================================================================

/**
 * @brief Tool and compensation configuration
 */
struct ToolConfig {
    /// Maximum tool number
    size_t maxTools{MAX_TOOLS};
    
    /// Random tool changer mode
    bool randomToolChanger{false};
    
    /// Tool change position (G30)
    Position toolChangePosition{};
    bool moveToToolChangePos{true};
    
    /// Default tool length offset axis
    Axis toolLengthAxis{Axis::Z};
    
    /// Cutter compensation entry distance
    double cutterCompEntryDist{10.0};
    
    /// Cutter compensation corner handling
    enum class CornerMode : uint8_t {
        SHARP,      ///< Keep sharp corners
        ROUNDED,    ///< Round outside corners
        LOOPED      ///< Loop around outside corners
    };
    CornerMode cornerMode{CornerMode::ROUNDED};
};

// ============================================================================
// Language Configuration
// ============================================================================

/**
 * @brief G-code language and compatibility configuration
 *
 * This config is about language/compatibility knobs (dialects, comment styles, etc.).
 * Streaming/lookahead behavior is configured via `GCode::ParserConfig` in
 * `GCodeParser.hpp`.
 */
struct LanguageConfig {
    /// LinuxCNC compatibility mode
    bool linuxCNCMode{true};
    
    /// Enable Fanuc-style (M98/M99) subroutines
    bool enableFanucSubs{true};
    
    /// Enable named subroutines
    bool enableNamedSubs{true};
    
    /// Case insensitive parsing
    bool caseInsensitive{true};
    
    /// Allow spaces in numbers (e.g., "1 000" = 1000)
    bool allowSpacesInNumbers{false};
    
    /// Comment styles
    bool enableParenComments{true};    // ( comment )
    bool enableSemicolonComments{true}; // ; comment
    
    /// Maximum expression nesting
    size_t maxExpressionDepth{10};
    
    /// Subroutine search paths (null-terminated)
    std::array<const char*, 8> subroutinePaths{};
    
    /// Program file path
    const char* programPath{nullptr};
};

// ============================================================================
// Probing Configuration
// ============================================================================

/**
 * @brief Probing configuration
 */
struct ProbeConfig {
    /// Default probe feed rate
    double defaultProbeFeedRate{100.0};
    
    /// Probe search distance
    double searchDistance{50.0};
    
    /// Second touch backoff
    double backoff{2.0};
    
    /// Second touch feed rate (slower)
    double fineFeedRate{10.0};
    
    /// Probe input signal
    int32_t probeInput{0};
    
    /// Probe signal active state
    bool probeActiveHigh{true};
    
    /// Probe protection (minimum height)
    double minProbeHeight{-100.0};
};

#define GCODE_PROBE_CONFIG_DEFINED

// ============================================================================
// Spindle Configuration
// ============================================================================

/**
 * @brief Spindle configuration
 */
struct SpindleConfig {
    /// Number of spindles
    size_t numSpindles{1};
    
    /// Maximum spindle speed
    double maxSpeed{24000.0};
    
    /// Minimum spindle speed
    double minSpeed{0.0};
    
    /// Speed at which at-speed is considered true
    double atSpeedTolerance{50.0};
    
    /// Spindle up time (seconds)
    double spinUpTime{2.0};
    
    /// CSS maximum RPM (G96 D)
    double cssMaxRPM{3000.0};
    
    /// Default orient angle (M19)
    double orientAngle{0.0};
};

// ============================================================================
// Feature Flags
// ============================================================================

/**
 * @brief Feature enable flags
 */
struct FeatureFlags {
    /// G-code features
    bool cutterCompensation{true};
    bool toolLengthOffset{true};
    bool coordinateRotation{true};
    bool polarCoordinates{false};
    
    /// Motion features
    bool pathBlending{true};
    bool naiveCamDetector{true};
    bool cornerRounding{true};
    bool jerkLimit{false};
    bool sCurveAccel{false};
    
    /// Compensation features
    bool backlashCompensation{false};
    bool volumetricCompensation{false};
    bool thermalCompensation{false};
    
    /// Advanced features
    bool adaptiveFeedrate{true};
    bool dynamicFeedOverride{true};
    bool negativeFeeds{false};      // Lookbehind for reverse
    bool splineInterpolation{true};
    bool nurbsInterpolation{true};
    
    /// Cycles
    bool cannedCycles{true};
    bool threadingCycles{true};
    bool probing{true};
    bool rigidTapping{true};
    bool trochoidalMilling{false};
    
    /// Lathe features
    bool latheMode{false};
    bool diameterMode{false};
    bool constantSurfaceSpeed{true};
    bool roughingCycles{true};
    
    /// Safety features
    bool softLimits{true};
    bool feedHold{true};
    bool blockDelete{true};
    bool optionalStop{true};
};

// ============================================================================
// Volumetric Compensation
// ============================================================================

/**
 * @brief Volumetric compensation grid point
 */
struct VolumetricPoint {
    Position nominal;       ///< Nominal position
    Position correction;    ///< Correction to apply
};

/**
 * @brief Volumetric compensation configuration
 */
struct VolumetricConfig {
    /// Enable volumetric compensation
    bool enabled{false};
    
    /// Grid dimensions
    std::array<size_t, 3> gridSize{10, 10, 10};
    
    /// Grid origin
    Position gridOrigin{};
    
    /// Grid spacing
    Position gridSpacing{};
    
    /// Interpolation method
    enum class InterpMethod : uint8_t {
        NEAREST,
        TRILINEAR,
        TRICUBIC
    };
    InterpMethod interpolation{InterpMethod::TRILINEAR};
    
    /// Compensation data pointer (external storage)
    const VolumetricPoint* data{nullptr};
    size_t dataSize{0};
};

// ============================================================================
// Trochoidal Milling Configuration
// ============================================================================

/**
 * @brief Trochoidal milling parameters
 */
struct TrochoidalConfig {
    /// Default stepover (percentage of tool diameter)
    double defaultStepover{0.1};
    
    /// Default engagement angle (degrees)
    double defaultEngagement{60.0};
    
    /// Minimum arc radius
    double minArcRadius{0.5};
    
    /// Entry helix pitch
    double helixPitch{1.0};
    
    /// Retract height for slot entry
    double entryRetract{2.0};
};

// ============================================================================
// Engraving Configuration
// ============================================================================

/**
 * @brief Engraving/text configuration
 */
struct EngravingConfig {
    /// Default character height
    double defaultHeight{10.0};
    
    /// Default character spacing (percentage of height)
    double charSpacing{0.8};
    
    /// Default line spacing (percentage of height)
    double lineSpacing{1.5};
    
    /// Default engraving depth
    double defaultDepth{0.5};
    
    /// Mirror text
    bool mirror{false};
    
    /// Font selection
    enum class Font : uint8_t {
        SIMPLE,
        GOTHIC,
        ROMAN,
        SCRIPT
    };
    Font font{Font::SIMPLE};
};

// ============================================================================
// Complete Configuration
// ============================================================================

/**
 * @brief Complete G-Code interpreter configuration
 */
struct Config {
    /// Axis configurations
    std::array<AxisConfig, MAX_AXES> axes{};
    
    /// Motion planning
    MotionConfig motion{};
    
    /// Coordinate systems
    CoordinateConfig coordinates{};
    
    /// Tool handling
    ToolConfig tools{};
    
    /// Language / compatibility settings
    LanguageConfig language{};
    
    /// Probing
    ProbeConfig probe{};
    
    /// Spindle
    SpindleConfig spindle{};
    
    /// Feature flags
    FeatureFlags features{};
    
    /// Volumetric compensation
    VolumetricConfig volumetric{};
    
    /// Trochoidal milling
    TrochoidalConfig trochoidal{};
    
    /// Engraving
    EngravingConfig engraving{};
    
    /**
     * @brief Apply default configuration
     */
    void setDefaults() {
        // Enable primary axes
        axes[static_cast<size_t>(Axis::X)].enabled = true;
        axes[static_cast<size_t>(Axis::Y)].enabled = true;
        axes[static_cast<size_t>(Axis::Z)].enabled = true;
        
        // Set rotary axis flags
        axes[static_cast<size_t>(Axis::A)].isRotary = true;
        axes[static_cast<size_t>(Axis::B)].isRotary = true;
        axes[static_cast<size_t>(Axis::C)].isRotary = true;
        
        // Default limits for typical 3-axis mill
        for (size_t i = 0; i < 3; ++i) {
            axes[i].minLimit = -500.0;
            axes[i].maxLimit = 500.0;
            axes[i].maxVelocity = 5000.0;
            axes[i].maxAcceleration = 300.0;
        }
    }
    
    /**
     * @brief Validate configuration
     * @return Error if invalid configuration
     */
    Error validate() const {
        Error err{};
        
        // Check at least one axis enabled
        bool hasAxis = false;
        for (const auto& axis : axes) {
            if (axis.enabled) {
                hasAxis = true;
                if (axis.maxVelocity <= 0) {
                    err.code = ErrorCode::INVALID_MOTION;
                    snprintf(err.message.data(), err.message.size(),
                            "Invalid max velocity");
                    return err;
                }
                if (axis.maxAcceleration <= 0) {
                    err.code = ErrorCode::INVALID_MOTION;
                    snprintf(err.message.data(), err.message.size(),
                            "Invalid max acceleration");
                    return err;
                }
            }
        }
        
        if (!hasAxis) {
            err.code = ErrorCode::INVALID_MOTION;
            snprintf(err.message.data(), err.message.size(),
                    "No axes enabled");
            return err;
        }
        
        // Check motion config
        if (motion.cycleTime <= 0) {
            err.code = ErrorCode::INVALID_MOTION;
            snprintf(err.message.data(), err.message.size(),
                    "Invalid cycle time");
            return err;
        }
        
        return err;
    }
};

// ============================================================================
// Configuration Callbacks
// ============================================================================

/**
 * @brief Configuration change notification callback
 */
using ConfigChangeCallback = std::function<void(const Config&)>;

/**
 * @brief Configuration loader callback
 */
using ConfigLoadCallback = std::function<bool(Config&)>;

/**
 * @brief Configuration saver callback
 */
using ConfigSaveCallback = std::function<bool(const Config&)>;

} // namespace GCode
