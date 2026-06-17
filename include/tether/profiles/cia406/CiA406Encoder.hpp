/**
 * @file CiA406Encoder.hpp
 * @brief Comprehensive CiA 406 Encoder Device Controller
 * 
 * @details
 * This module provides a complete implementation for interfacing with
 * CiA 406 compliant encoders and position sensors. It supports:
 * 
 * ## Encoder Classes
 * - **C1**: Absolute single-turn (rotary and linear)
 * - **C2**: Absolute multi-turn (rotary and linear)  
 * - **C3**: Incremental (with reference mark)
 * - **C4**: Incremental with limit switches
 * 
 * ## Features
 * - Automatic encoder discovery and classification
 * - Position reading with scaling and offset
 * - Velocity and acceleration calculation
 * - Multi-turn position tracking
 * - Working area monitoring
 * - Alarm and warning handling
 * - Support for various interfaces (SSI, BiSS, EnDat, SinCos)
 * - Reference/homing procedures
 * - Position preset functionality
 * 
 * ## PDO Configurations
 * 
 * Pre-configured PDO mappings for typical use cases:
 * 
 * | Configuration    | TxPDO Contents                                 |
 * |------------------|------------------------------------------------|
 * | Basic            | Position, Status                               |
 * | With Velocity    | Position, Velocity, Status                     |
 * | Full Diagnostic  | Position, Velocity, Status, Alarms, Temp       |
 * | Multi-turn       | Position, Multi-turn, Single-turn, Status      |
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "profiles/cia406/CiA406Encoder.hpp"
 * 
 * // Create encoder instance for slave at position 1
 * CiA406::Encoder encoder(1);
 * 
 * // Initialize and detect encoder type
 * if (!encoder.initialize()) {
 *     TETHER_LOGE("ENCODER", "Initialization failed");
 *     return;
 * }
 * 
 * // Configure PDO mapping
 * encoder.applyPDOMapping(CiA406::PDOMappingPreset::WithVelocity);
 * 
 * // Enable scaling
 * encoder.setScaling(4096, 360000);  // 4096 counts = 360.000 degrees
 * 
 * // Main loop
 * while (running) {
 *     encoder.update();  // Call in cyclic task
 *     
 *     int32_t position = encoder.getPosition();
 *     int32_t velocity = encoder.getVelocity();
 *     
 *     if (encoder.hasAlarm()) {
 *         TETHER_LOGW("ENCODER", "Alarm: 0x%04X", encoder.getAlarms());
 *     }
 * }
 * ```
 * 
 * @see CiA406Defs.hpp for object dictionary definitions
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <array>
#include <memory>
#include <vector>

namespace EtherCAT { namespace CoE { class CoEManager; } }

namespace CiA406 {

// Forward declarations
class Encoder;

// ============================================================================
// Encoder Types and Enumerations (Extended versions with Unknown state)
// Note: These extend the base definitions in CiA406Defs.hpp
// ============================================================================

/**
 * @brief Extended encoder class as defined in CiA 406 (with Unknown state)
 */
enum class EncoderClassEx : uint8_t {
    Unknown = 0,
    C1_AbsoluteSingleTurn = 1,
    C2_AbsoluteMultiTurn = 2,
    C3_Incremental = 3,
    C4_IncrementalWithLimits = 4,
};

/**
 * @brief Get human-readable name for encoder class
 */
const char* getEncoderClassName(EncoderClassEx cls);

/**
 * @brief Extended encoder measurement type (with Unknown state)
 */
enum class EncoderTypeEx : uint8_t {
    Rotary = 0,
    Linear = 1,
};

/**
 * @brief Encoder interface type
 */
enum class InterfaceType : uint8_t {
    Parallel = 0,
    SSI = 1,
    BiSS_C = 2,
    BiSS_B = 3,
    EnDat21 = 4,
    EnDat22 = 5,
    SinCos_1Vpp = 6,
    TTL_RS422 = 7,
    Hiperface = 8,
    DRIVE_CLiQ = 9,
    Tamagawa = 10,
};

/**
 * @brief Get human-readable name for interface type
 */
const char* getInterfaceTypeName(InterfaceType type);

/**
 * @brief Encoder operating status flags
 */
struct OperatingStatus {
    bool position_valid;          ///< Position value is valid
    bool scaling_active;          ///< Scaling function is enabled
    bool reference_done;          ///< Reference/homing complete
    bool preset_executed;         ///< Preset was applied
    bool overspeed_warning;       ///< Velocity exceeds limit
    bool counting_range_exceeded; ///< Position outside counting range
    bool supply_voltage_low;      ///< Supply voltage below threshold
    bool supply_voltage_high;     ///< Supply voltage above threshold
    
    /// Parse from raw status word
    static OperatingStatus fromRaw(uint16_t raw);
    
    /// Convert to raw status word
    uint16_t toRaw() const;
};

/**
 * @brief Encoder alarm flags
 */
struct AlarmFlags {
    bool hardware_error;          ///< General hardware fault
    bool temperature_exceeded;    ///< Over-temperature
    bool light_source_error;      ///< Optical encoder light source failure
    bool battery_error;           ///< Backup battery failure (multi-turn)
    bool position_error;          ///< Position reading error
    bool communication_error;     ///< Interface communication error
    bool multi_turn_error;        ///< Multi-turn position lost
    bool initialization_error;    ///< Startup/init failure
    bool speed_exceeded;          ///< Over-speed condition
    bool position_limit_exceeded; ///< Position outside valid range
    
    /// Parse from raw alarm word
    static AlarmFlags fromRaw(uint16_t raw);
    
    /// Check if any alarm is active
    bool hasAnyAlarm() const;
    
    /// Get description of active alarms
    std::string getDescription() const;
};

/**
 * @brief Working area monitoring state
 */
enum class WorkingAreaState : uint8_t {
    WithinArea1 = 0,
    BelowLowLimit1 = 1,
    AboveHighLimit1 = 2,
    WithinArea2 = 3,
    BelowLowLimit2 = 4,
    AboveHighLimit2 = 5,
    NotConfigured = 255,
};

// ============================================================================
// Encoder Capabilities
// ============================================================================

/**
 * @brief Describes the capabilities of a specific encoder
 */
struct EncoderCapabilities {
    EncoderClassEx encoder_class;
    EncoderTypeEx encoder_type;
    InterfaceType interface_type;
    
    uint32_t single_turn_resolution;   ///< Resolution bits (single turn)
    uint32_t multi_turn_revolutions;   ///< Number of distinguishable revolutions
    uint32_t total_measuring_range;    ///< Total counting range
    
    bool has_velocity_output;          ///< Device provides velocity
    bool has_acceleration_output;      ///< Device provides acceleration
    bool has_working_area_monitoring;  ///< Working area limits available
    bool has_scaling_function;         ///< Scaling can be configured
    bool has_preset_function;          ///< Position can be preset
    bool has_reference_input;          ///< Reference/home signal available
    bool has_limit_switches;           ///< Limit switch inputs (C4)
    bool has_temperature_sensor;       ///< Temperature monitoring
    bool has_signal_quality;           ///< Signal quality indicator
    
    uint16_t supported_alarms;         ///< Bitmap of supported alarms
    uint16_t supported_warnings;       ///< Bitmap of supported warnings
};

// ============================================================================
// PDO Mapping Presets
// ============================================================================

/**
 * @brief Pre-defined PDO mapping configurations
 */
enum class PDOMappingPreset : uint8_t {
    /// Basic: Position + Status
    Basic,
    
    /// With velocity: Position + Velocity + Status
    WithVelocity,
    
    /// Full: Position + Velocity + Status + Alarms
    Full,
    
    /// Multi-turn: Position + Multi-turn + Single-turn + Status
    MultiTurn,
    
    /// High-speed: Position only (minimal latency)
    HighSpeed,
    
    /// Diagnostic: All diagnostic info (Position, Velocity, Status, Alarms, Temp, Quality)
    Diagnostic,
    
    /// Custom (user-defined)
    Custom,
};

/**
 * @brief PDO mapping entry
 */
struct PDOMappingEntry {
    uint16_t index;
    uint8_t subindex;
    uint8_t bits;
    
    uint32_t toMappingValue() const {
        return (static_cast<uint32_t>(index) << 16) |
               (static_cast<uint32_t>(subindex) << 8) |
               static_cast<uint32_t>(bits);
    }
};

/**
 * @brief TxPDO (Input) data structure - Position basic
 */
struct TxPDOBasic {
    int32_t position;
    uint16_t status;
} __attribute__((packed));

/**
 * @brief TxPDO with velocity
 */
struct TxPDOWithVelocity {
    int32_t position;
    int32_t velocity;
    uint16_t status;
} __attribute__((packed));

/**
 * @brief TxPDO full diagnostic
 */
struct TxPDOFull {
    int32_t position;
    int32_t velocity;
    uint16_t status;
    uint16_t alarms;
    int16_t temperature;
    uint8_t signal_quality;
} __attribute__((packed));

/**
 * @brief TxPDO multi-turn
 */
struct TxPDOMultiTurn {
    int32_t position;
    int16_t multi_turn;
    uint16_t single_turn;
    uint16_t status;
} __attribute__((packed));

// ============================================================================
// Scaling Configuration
// ============================================================================

/**
 * @brief Position scaling configuration
 */
struct ScalingConfig {
    bool enabled;
    int32_t numerator;    ///< Scaling numerator
    int32_t denominator;  ///< Scaling denominator
    int32_t offset;       ///< Position offset
    
    /// Apply scaling to raw position
    int32_t apply(int32_t raw_position) const {
        if (!enabled || denominator == 0) return raw_position;
        return ((int64_t)raw_position * numerator) / denominator + offset;
    }
    
    /// Reverse scaling (user units to raw)
    int32_t reverse(int32_t scaled_position) const {
        if (!enabled || numerator == 0) return scaled_position;
        return ((int64_t)(scaled_position - offset) * denominator) / numerator;
    }
};

// ============================================================================
// Reference/Homing Configuration
// ============================================================================

/**
 * @brief Reference (homing) mode for incremental encoders
 */
enum class ReferenceMode : uint8_t {
    None = 0,
    OnZeroPulse = 1,       ///< Reference on next zero/index pulse
    OnLimitSwitch = 2,     ///< Reference on limit switch
    OnExternalInput = 3,   ///< Reference on external signal
    CurrentPosition = 4,   ///< Set current position as reference
};

/**
 * @brief Reference/homing configuration
 */
struct ReferenceConfig {
    ReferenceMode mode;
    int32_t reference_position;  ///< Position to set at reference
    bool direction_positive;     ///< Search direction
    uint32_t search_velocity;    ///< Velocity during search (if applicable)
};

// ============================================================================
// Encoder Event Callbacks
// ============================================================================

/**
 * @brief Event types for encoder callbacks
 */
enum class EncoderEvent : uint8_t {
    PositionUpdated,      ///< New position available
    AlarmTriggered,       ///< Alarm condition detected
    AlarmCleared,         ///< Alarm condition cleared
    WarningTriggered,     ///< Warning condition detected
    ReferenceDone,        ///< Reference procedure complete
    WorkingAreaEntered,   ///< Entered working area
    WorkingAreaExited,    ///< Exited working area
    CommunicationError,   ///< Communication lost/recovered
};

/**
 * @brief Callback function type for encoder events
 */
using EncoderEventCallback = std::function<void(EncoderEvent event, uint16_t slave_addr, uint32_t data)>;

// ============================================================================
// Main Encoder Class
// ============================================================================

/**
 * @brief CiA 406 Encoder Device Controller
 * 
 * Provides comprehensive control over CiA 406 compliant encoders.
 */
class Encoder {
public:
    /**
     * @brief Construct encoder controller
     * @param coe CoEManager instance for SDO access
     */
    explicit Encoder(EtherCAT::CoE::CoEManager& coe);
    
    ~Encoder() = default;
    
    // Prevent copying (device-bound resource)
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    
    // Allow moving
    Encoder(Encoder&&) = default;
    Encoder& operator=(Encoder&&) = default;
    
    // ========================================================================
    // Initialization and Configuration
    // ========================================================================
    
    /**
     * @brief Initialize encoder and detect capabilities
     * 
     * Reads device type, capabilities, and configures for operation.
     * Must be called before using other methods.
     * 
     * @return true if initialization succeeded
     */
    bool initialize();
    
    /**
     * @brief Check if encoder is initialized
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get encoder capabilities
     */
    const EncoderCapabilities& getCapabilities() const { return capabilities_; }
    
    /**
     * @brief Apply PDO mapping preset
     * @param preset Mapping preset to apply
     * @return true if mapping was applied successfully
     */
    bool applyPDOMapping(PDOMappingPreset preset);
    
    /**
     * @brief Apply custom PDO mapping
     * @param entries Array of mapping entries
     * @param count Number of entries
     * @return true if mapping was applied
     */
    bool applyCustomPDOMapping(const PDOMappingEntry* entries, size_t count);
    
    /**
     * @brief Get current PDO mapping preset
     */
    PDOMappingPreset getCurrentMapping() const { return current_mapping_; }
    
    // ========================================================================
    // Position Reading
    // ========================================================================
    
    /**
     * @brief Update encoder data (call cyclically)
     * 
     * Reads PDO data and updates internal state. Call this in your
     * cyclic task at the desired update rate.
     */
    void update();
    
    /**
     * @brief Get current position (with scaling if enabled)
     * @return Position in user units
     */
    int32_t getPosition() const { return scaled_position_; }
    
    /**
     * @brief Get raw position (before scaling)
     * @return Raw encoder counts
     */
    int32_t getRawPosition() const { return raw_position_; }
    
    /**
     * @brief Get multi-turn value (revolutions)
     * @return Revolution count (only for multi-turn encoders)
     */
    int16_t getMultiTurnValue() const { return multi_turn_value_; }
    
    /**
     * @brief Get single-turn value (position within revolution)
     * @return Position within current revolution
     */
    uint16_t getSingleTurnValue() const { return single_turn_value_; }
    
    /**
     * @brief Get current velocity
     * @return Velocity in user units per second
     */
    int32_t getVelocity() const { return velocity_; }
    
    /**
     * @brief Get current acceleration
     * @return Acceleration in user units per second²
     */
    int32_t getAcceleration() const { return acceleration_; }
    
    // ========================================================================
    // Scaling and Offset
    // ========================================================================
    
    /**
     * @brief Enable position scaling
     * @param numerator Scaling numerator
     * @param denominator Scaling denominator
     * @param offset Position offset (added after scaling)
     */
    void setScaling(int32_t numerator, int32_t denominator, int32_t offset = 0);
    
    /**
     * @brief Set scaling in terms of resolution and user range
     * @param encoder_counts Number of encoder counts (e.g., 4096)
     * @param user_units Equivalent user units (e.g., 360000 for 360.000 degrees)
     */
    void setScalingFromRange(int32_t encoder_counts, int32_t user_units);
    
    /**
     * @brief Get current scaling configuration
     */
    const ScalingConfig& getScaling() const { return scaling_; }
    
    /**
     * @brief Disable scaling (use raw counts)
     */
    void disableScaling();
    
    /**
     * @brief Set position offset
     * @param offset Offset value in user units
     */
    void setOffset(int32_t offset);
    
    // ========================================================================
    // Preset and Reference
    // ========================================================================
    
    /**
     * @brief Preset encoder position
     * 
     * Sets the current position to the specified value.
     * 
     * @param position New position value
     * @return true if preset was successful
     */
    bool presetPosition(int32_t position);
    
    /**
     * @brief Set current position as zero
     * @return true if zeroing was successful
     */
    bool setZero() { return presetPosition(0); }
    
    /**
     * @brief Start reference procedure (for incremental encoders)
     * @param config Reference configuration
     * @return true if reference started
     */
    bool startReference(const ReferenceConfig& config);
    
    /**
     * @brief Check if reference is complete
     */
    bool isReferenced() const { return status_.reference_done; }
    
    /**
     * @brief Abort ongoing reference procedure
     */
    void abortReference();
    
    // ========================================================================
    // Status and Alarms
    // ========================================================================
    
    /**
     * @brief Get current operating status
     */
    const OperatingStatus& getStatus() const { return status_; }
    
    /**
     * @brief Check if position is valid
     */
    bool isPositionValid() const { return status_.position_valid; }
    
    /**
     * @brief Get current alarm flags
     */
    const AlarmFlags& getAlarms() const { return alarms_; }
    
    /**
     * @brief Get raw alarm word
     */
    uint16_t getAlarmsRaw() const { return alarms_raw_; }
    
    /**
     * @brief Check if any alarm is active
     */
    bool hasAlarm() const { return alarms_.hasAnyAlarm(); }
    
    /**
     * @brief Clear alarms (if clearable)
     * @return true if alarms were cleared
     */
    bool clearAlarms();
    
    /**
     * @brief Get warning flags
     */
    uint16_t getWarnings() const { return warnings_raw_; }
    
    /**
     * @brief Check if any warning is active
     */
    bool hasWarning() const { return warnings_raw_ != 0; }
    
    // ========================================================================
    // Working Area Monitoring
    // ========================================================================
    
    /**
     * @brief Configure working area limits
     * @param low_limit Lower position limit
     * @param high_limit Upper position limit
     * @param area_index Working area index (1 or 2)
     * @return true if limits were set
     */
    bool setWorkingArea(int32_t low_limit, int32_t high_limit, uint8_t area_index = 1);
    
    /**
     * @brief Get current working area state
     */
    WorkingAreaState getWorkingAreaState() const { return working_area_state_; }
    
    /**
     * @brief Check if position is within working area
     */
    bool isWithinWorkingArea() const;
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get encoder temperature
     * @return Temperature in 0.1°C units (e.g., 250 = 25.0°C)
     */
    int16_t getTemperature() const { return temperature_; }
    
    /**
     * @brief Get supply voltage
     * @return Voltage in mV
     */
    uint16_t getSupplyVoltage() const { return supply_voltage_; }
    
    /**
     * @brief Get signal quality
     * @return Quality percentage (0-100)
     */
    uint8_t getSignalQuality() const { return signal_quality_; }
    
    /**
     * @brief Get operating time
     * @return Operating hours
     */
    uint32_t getOperatingTime() const { return operating_time_; }
    
    /**
     * @brief Read all diagnostic data from encoder
     * @return true if diagnostics were read
     */
    bool readDiagnostics();
    
    /**
     * @brief Get encoder serial number
     */
    const std::string& getSerialNumber() const { return serial_number_; }
    
    /**
     * @brief Get encoder firmware version
     */
    const std::string& getFirmwareVersion() const { return firmware_version_; }
    
    // ========================================================================
    // Interface-Specific Configuration
    // ========================================================================
    
    /**
     * @brief Configure SSI interface parameters
     * @param clock_khz Clock frequency in kHz
     * @param data_bits Number of data bits
     * @param gray_code Use Gray code encoding
     * @param msb_first MSB transmitted first
     */
    bool configureSSI(uint16_t clock_khz, uint8_t data_bits, 
                      bool gray_code = true, bool msb_first = true);
    
    /**
     * @brief Configure BiSS interface parameters
     */
    bool configureBiSS(uint32_t clock_hz, uint8_t data_bits);
    
    /**
     * @brief Configure EnDat interface parameters
     */
    bool configureEnDat(uint32_t clock_hz);
    
    // ========================================================================
    // Event Handling
    // ========================================================================
    
    /**
     * @brief Register event callback
     */
    void setEventCallback(EncoderEventCallback callback);
    
    /**
     * @brief Clear event callback
     */
    void clearEventCallback();
    
    // ========================================================================
    // SDO Access (Direct Object Access)
    // ========================================================================
    
    /**
     * @brief Read object via SDO
     */
    bool readObject(uint16_t index, uint8_t subindex, void* data, size_t size, size_t* out_size);
    
    /**
     * @brief Write object via SDO
     */
    bool writeObject(uint16_t index, uint8_t subindex, const void* data, size_t size);
    
    // ========================================================================
    // Slave Information
    // ========================================================================
    
private:
    // Slave identification
    EtherCAT::CoE::CoEManager& m_coe;
    bool initialized_{false};
    
    // Capabilities
    EncoderCapabilities capabilities_{};
    
    // Current PDO configuration
    PDOMappingPreset current_mapping_{PDOMappingPreset::Basic};
    size_t pdo_size_{0};
    
    // Position data
    int32_t raw_position_{0};
    int32_t scaled_position_{0};
    int16_t multi_turn_value_{0};
    uint16_t single_turn_value_{0};
    int32_t velocity_{0};
    int32_t acceleration_{0};
    int32_t previous_position_{0};
    int32_t previous_velocity_{0};
    uint32_t last_update_time_{0};
    
    // Scaling
    ScalingConfig scaling_{false, 1, 1, 0};
    
    // Status
    OperatingStatus status_{};
    uint16_t status_raw_{0};
    AlarmFlags alarms_{};
    uint16_t alarms_raw_{0};
    uint16_t warnings_raw_{0};
    WorkingAreaState working_area_state_{WorkingAreaState::NotConfigured};
    
    // Diagnostics
    int16_t temperature_{0};
    uint16_t supply_voltage_{0};
    uint8_t signal_quality_{0};
    uint32_t operating_time_{0};
    std::string serial_number_;
    std::string firmware_version_;
    
    // Event handling
    EncoderEventCallback event_callback_;
    uint16_t previous_alarms_{0};
    
    // Internal helpers
    bool detectEncoderClass();
    bool readCapabilities();
    bool configureSyncManagers();
    void processStatus(uint16_t status_word);
    void processAlarms(uint16_t alarm_word);
    void fireEvent(EncoderEvent event, uint32_t data = 0);
    void calculateVelocity(int32_t new_position, uint32_t dt_us);
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create encoder with automatic type detection
 * @param sdo SDOManager instance for SDO access
 * @param slave_position Slave position
 * @return Initialized encoder or nullptr if detection failed
 */
std::unique_ptr<Encoder> createEncoder(EtherCAT::CoE::CoEManager& coe);

/**
 * @brief Scan for all encoders on the network
 * @param sdo SDOManager instance for SDO access
 * @return Vector of slave positions with CiA 406 devices
 */
// scanForEncoders removed - no longer applicable with per-slave CoEManager

} // namespace CiA406
