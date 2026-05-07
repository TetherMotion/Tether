/**
 * @file CiA401IO.hpp
 * @brief CiA 401 I/O Module Controller Interface
 * 
 * @details
 * Comprehensive controller for CiA 401 compliant I/O modules including:
 * - Digital inputs with filtering and interrupt configuration
 * - Digital outputs with error handling and PWM
 * - Analog inputs with scaling and threshold monitoring
 * - Analog outputs with error values and scaling
 * - Counter inputs for pulse counting
 * - Frequency/PWM inputs for signal measurement
 * - PWM outputs for proportional control
 * 
 * @note All operations use SDO for configuration, PDO for real-time I/O
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "profiles/cia401/CiA401Defs.hpp"

namespace EtherCAT { namespace SDO { class SDOManager; } }

namespace CiA401 {

// Forward declarations
class IOModule;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief PDO mapping presets for different use cases
 */
enum class PDOMappingPreset : uint8_t {
    /** Minimal - only most essential I/O */
    Minimal,
    
    /** Basic digital I/O only (8-bit blocks) */
    DigitalOnly,
    
    /** Basic analog I/O only */
    AnalogOnly,
    
    /** Digital 16-bit blocks */
    Digital16,
    
    /** Digital 32-bit blocks */
    Digital32,
    
    /** High resolution analog (32-bit) */
    AnalogHighRes,
    
    /** Combined digital and analog */
    Combined,
    
    /** Full featured with all channels */
    Full,
    
    /** Custom mapping (user-defined) */
    Custom
};

/**
 * @brief Events that can trigger callbacks
 */
enum class IOEvent : uint8_t {
    /** Digital input changed */
    DigitalInputChanged,
    
    /** Digital output updated */
    DigitalOutputUpdated,
    
    /** Analog input crossed upper threshold */
    AnalogUpperLimit,
    
    /** Analog input crossed lower threshold */
    AnalogLowerLimit,
    
    /** Analog input delta threshold */
    AnalogDeltaTriggered,
    
    /** Counter overflow */
    CounterOverflow,
    
    /** Counter underflow */
    CounterUnderflow,
    
    /** Counter reached preset */
    CounterPresetReached,
    
    /** Communication error detected */
    CommunicationError,
    
    /** Module initialized successfully */
    Initialized,
    
    /** Module fault detected */
    ModuleFault
};

/**
 * @brief Digital signal edge type
 */
enum class EdgeType : uint8_t {
    /** Any edge (rising or falling) */
    Any = 0x01,
    
    /** Rising edge only (low to high) */
    Rising = 0x02,
    
    /** Falling edge only (high to low) */
    Falling = 0x04,
    
    /** Both rising and falling edges */
    Both = 0x06
};

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for I/O events
 * @param event The event type
 * @param slave_addr Slave address
 * @param channel Channel number (if applicable)
 * @param value Associated value
 */
using IOEventCallback = std::function<void(IOEvent event, uint16_t slave_addr, 
                                           uint8_t channel, uint32_t value)>;

/**
 * @brief Callback for digital input changes
 * @param slave_addr Slave address
 * @param channel Bit position (0-based)
 * @param state New state (true = high)
 */
using DigitalInputCallback = std::function<void(uint16_t slave_addr, 
                                                uint8_t channel, bool state)>;

/**
 * @brief Callback for analog threshold events
 * @param slave_addr Slave address
 * @param channel Channel number
 * @param value Current value
 * @param limit_type 1=upper, 2=lower, 3=delta
 */
using AnalogThresholdCallback = std::function<void(uint16_t slave_addr,
                                                   uint8_t channel, int32_t value,
                                                   uint8_t limit_type)>;

// ============================================================================
// Capability Structures
// ============================================================================

/**
 * @brief Module capabilities discovered during initialization
 */
struct ModuleCapabilities {
    ModuleType module_type;
    
    // Digital capabilities
    uint8_t digital_input_8bit_blocks;    ///< Number of 8-bit input blocks
    uint8_t digital_input_16bit_blocks;   ///< Number of 16-bit input blocks
    uint8_t digital_input_32bit_blocks;   ///< Number of 32-bit input blocks
    uint8_t digital_output_8bit_blocks;   ///< Number of 8-bit output blocks
    uint8_t digital_output_16bit_blocks;  ///< Number of 16-bit output blocks
    uint8_t digital_output_32bit_blocks;  ///< Number of 32-bit output blocks
    
    // Analog capabilities
    uint8_t analog_input_16bit_channels;  ///< Number of 16-bit AI channels
    uint8_t analog_input_32bit_channels;  ///< Number of 32-bit AI channels
    uint8_t analog_output_16bit_channels; ///< Number of 16-bit AO channels
    uint8_t analog_output_32bit_channels; ///< Number of 32-bit AO channels
    
    // Optional features
    bool has_counters;
    bool has_frequency_input;
    bool has_pwm_output;
    uint8_t counter_channels;
    uint8_t frequency_channels;
    uint8_t pwm_channels;
    
    // Feature flags
    bool supports_interrupt;
    bool supports_filtering;
    bool supports_polarity;
    
    ModuleCapabilities() 
        : module_type(ModuleType::DigitalInputOnly),
          digital_input_8bit_blocks(0), digital_input_16bit_blocks(0),
          digital_input_32bit_blocks(0), digital_output_8bit_blocks(0),
          digital_output_16bit_blocks(0), digital_output_32bit_blocks(0),
          analog_input_16bit_channels(0), analog_input_32bit_channels(0),
          analog_output_16bit_channels(0), analog_output_32bit_channels(0),
          has_counters(false), has_frequency_input(false), has_pwm_output(false),
          counter_channels(0), frequency_channels(0), pwm_channels(0),
          supports_interrupt(false), supports_filtering(false), supports_polarity(false) {}
    
    /** Get total digital input count */
    uint16_t getTotalDigitalInputs() const {
        return digital_input_8bit_blocks * 8 + 
               digital_input_16bit_blocks * 16 +
               digital_input_32bit_blocks * 32;
    }
    
    /** Get total digital output count */
    uint16_t getTotalDigitalOutputs() const {
        return digital_output_8bit_blocks * 8 +
               digital_output_16bit_blocks * 16 +
               digital_output_32bit_blocks * 32;
    }
    
    /** Get total analog input count */
    uint16_t getTotalAnalogInputs() const {
        return analog_input_16bit_channels + analog_input_32bit_channels;
    }
    
    /** Get total analog output count */
    uint16_t getTotalAnalogOutputs() const {
        return analog_output_16bit_channels + analog_output_32bit_channels;
    }
};

/**
 * @brief Digital channel state tracking
 */
struct DigitalChannelState {
    uint8_t block_index;        ///< Block index
    uint8_t bit_position;       ///< Bit position within block
    bool current_state;         ///< Current state
    bool previous_state;        ///< Previous state for edge detection
    EdgeType edge_mask;         ///< Configured edge detection
    
    DigitalChannelState()
        : block_index(0), bit_position(0), current_state(false),
          previous_state(false), edge_mask(EdgeType::Any) {}
};

/**
 * @brief Analog channel state tracking
 */
struct AnalogChannelState {
    uint8_t channel;            ///< Channel index
    int32_t raw_value;          ///< Raw ADC value
    int32_t scaled_value;       ///< Scaled engineering units
    int32_t previous_value;     ///< Previous value for delta
    bool upper_limit_active;    ///< Upper limit triggered
    bool lower_limit_active;    ///< Lower limit triggered
    
    AnalogChannelState()
        : channel(0), raw_value(0), scaled_value(0), previous_value(0),
          upper_limit_active(false), lower_limit_active(false) {}
};

// ============================================================================
// Main IOModule Class
// ============================================================================

/**
 * @brief CiA 401 I/O Module Controller
 * 
 * @details
 * Provides full control of CiA 401 compliant I/O modules including:
 * - Automatic capability detection
 * - Digital I/O with filtering, polarity, and interrupts
 * - Analog I/O with scaling and thresholds
 * - Counter and frequency input support
 * - PWM output control
 * - Event-driven callbacks for state changes
 * 
 * Usage:
 * @code
 * CiA401::IOModule io(1); // Slave address 1
 * io.initialize();
 * 
 * // Read digital input
 * bool state = io.readDigitalInput(0);
 * 
 * // Write digital output
 * io.writeDigitalOutput(0, true);
 * 
 * // Read analog input
 * int32_t voltage = io.readAnalogInput(0);
 * 
 * // Write analog output
 * io.writeAnalogOutput(0, 5000); // 5V = 5000mV
 * @endcode
 */
class IOModule {
public:
    // ========================================================================
    // Construction and Initialization
    // ========================================================================
    
    /**
     * @brief Construct I/O module controller
     * @param slave_addr Position address or configured address
     * @param use_configured_addr If true, slave_addr is configured address
     */
    explicit IOModule(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr = false);
    
    /**
     * @brief Destructor
     */
    ~IOModule();
    
    /**
     * @brief Initialize the I/O module
     * @details Reads device type, detects capabilities, applies default config
     * @return true if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Check if module is initialized
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get slave address
     */
    uint16_t getSlaveAddress() const { return slave_addr_; }
    
    /**
     * @brief Check if using configured address
     */
    bool isUsingConfiguredAddress() const { return use_configured_addr_; }
    
    /**
     * @brief Get module capabilities
     */
    const ModuleCapabilities& getCapabilities() const { return capabilities_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    /**
     * @brief Apply PDO mapping preset
     * @param preset The mapping preset to apply
     * @return true if successful
     */
    bool applyPDOMapping(PDOMappingPreset preset);
    
    /**
     * @brief Get current PDO mapping preset
     */
    PDOMappingPreset getCurrentMapping() const { return current_mapping_; }
    
    // ========================================================================
    // Update Cycle
    // ========================================================================
    
    /**
     * @brief Process received PDO data
     * @param data Pointer to PDO data
     * @param len Length of PDO data
     */
    void processTxPDO(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare PDO data for transmission
     * @param data Buffer to write PDO data
     * @param max_len Maximum buffer size
     * @return Number of bytes written
     */
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    
    /**
     * @brief Update module state (call periodically)
     * @details Reads inputs via SDO if PDO not configured
     */
    void update();
    
    // ========================================================================
    // Digital Input Operations
    // ========================================================================
    
    /**
     * @brief Read single digital input
     * @param channel Channel number (0-based)
     * @return true if input is high
     */
    bool readDigitalInput(uint8_t channel);
    
    /**
     * @brief Read 8-bit digital input block
     * @param block Block index (0-based)
     * @return 8-bit input state
     */
    uint8_t readDigitalInput8(uint8_t block);
    
    /**
     * @brief Read 16-bit digital input block
     * @param block Block index (0-based)
     * @return 16-bit input state
     */
    uint16_t readDigitalInput16(uint8_t block);
    
    /**
     * @brief Read 32-bit digital input block
     * @param block Block index (0-based)
     * @return 32-bit input state
     */
    uint32_t readDigitalInput32(uint8_t block);
    
    /**
     * @brief Configure digital input polarity
     * @param block Block index
     * @param mask Polarity mask (1 = inverted)
     * @return true if successful
     */
    bool setDigitalInputPolarity(uint8_t block, uint8_t mask);
    
    /**
     * @brief Configure digital input filter
     * @param block Block index
     * @param filter_const Filter constant (see FilterConstant namespace)
     * @return true if successful
     */
    bool setDigitalInputFilter(uint8_t block, uint8_t filter_const);
    
    /**
     * @brief Configure digital input interrupt
     * @param block Block index
     * @param mask Bit mask for interrupt-enabled channels
     * @param edge Edge type to trigger on
     * @return true if successful
     */
    bool setDigitalInputInterrupt(uint8_t block, uint8_t mask, EdgeType edge);
    
    /**
     * @brief Enable/disable global digital interrupts
     * @param enable true to enable interrupts
     * @return true if successful
     */
    bool setDigitalInterruptEnable(bool enable);
    
    // ========================================================================
    // Digital Output Operations
    // ========================================================================
    
    /**
     * @brief Write single digital output
     * @param channel Channel number (0-based)
     * @param state Output state (true = high)
     * @return true if successful
     */
    bool writeDigitalOutput(uint8_t channel, bool state);
    
    /**
     * @brief Write 8-bit digital output block
     * @param block Block index (0-based)
     * @param value 8-bit output value
     * @return true if successful
     */
    bool writeDigitalOutput8(uint8_t block, uint8_t value);
    
    /**
     * @brief Write 16-bit digital output block
     * @param block Block index (0-based)
     * @param value 16-bit output value
     * @return true if successful
     */
    bool writeDigitalOutput16(uint8_t block, uint16_t value);
    
    /**
     * @brief Write 32-bit digital output block
     * @param block Block index (0-based)
     * @param value 32-bit output value
     * @return true if successful
     */
    bool writeDigitalOutput32(uint8_t block, uint32_t value);
    
    /**
     * @brief Set single output bit without affecting others
     * @param channel Channel number
     * @param state New state
     * @return true if successful
     */
    bool setDigitalOutputBit(uint8_t channel, bool state);
    
    /**
     * @brief Toggle single output bit
     * @param channel Channel number
     * @return true if successful
     */
    bool toggleDigitalOutput(uint8_t channel);
    
    /**
     * @brief Configure digital output polarity
     * @param block Block index
     * @param mask Polarity mask (1 = inverted)
     * @return true if successful
     */
    bool setDigitalOutputPolarity(uint8_t block, uint8_t mask);
    
    /**
     * @brief Configure digital output error behavior
     * @param block Block index
     * @param mode Error mode (see ErrorMode namespace)
     * @param value Error value (if mode = UseErrorValue)
     * @return true if successful
     */
    bool setDigitalOutputErrorMode(uint8_t block, uint8_t mode, uint8_t value);
    
    // ========================================================================
    // Analog Input Operations
    // ========================================================================
    
    /**
     * @brief Read 16-bit analog input raw value
     * @param channel Channel number (1-based per CiA 401)
     * @return Raw ADC value
     */
    int16_t readAnalogInput16(uint8_t channel);
    
    /**
     * @brief Read 32-bit analog input raw value
     * @param channel Channel number (1-based)
     * @return Raw ADC value (high resolution)
     */
    int32_t readAnalogInput32(uint8_t channel);
    
    /**
     * @brief Read scaled analog input
     * @param channel Channel number (1-based)
     * @return Scaled value in engineering units
     */
    int32_t readAnalogInputScaled(uint8_t channel);
    
    /**
     * @brief Configure analog input scaling
     * @param channel Channel number (1-based)
     * @param offset Offset value
     * @param scaling Scaling factor (Q16 fixed point: 0x10000 = 1.0)
     * @return true if successful
     */
    bool setAnalogInputScaling(uint8_t channel, int32_t offset, int32_t scaling);
    
    /**
     * @brief Configure analog input from range
     * @param channel Channel number (1-based)
     * @param raw_min Minimum raw ADC value
     * @param raw_max Maximum raw ADC value
     * @param eng_min Minimum engineering value
     * @param eng_max Maximum engineering value
     * @return true if successful
     */
    bool setAnalogInputRange(uint8_t channel, int32_t raw_min, int32_t raw_max,
                             int32_t eng_min, int32_t eng_max);
    
    /**
     * @brief Configure analog input SI unit
     * @param channel Channel number (1-based)
     * @param unit_type Unit type (see SIUnitType namespace)
     * @param prefix Unit prefix (see SIUnitPrefix namespace)
     * @return true if successful
     */
    bool setAnalogInputSIUnit(uint8_t channel, uint8_t unit_type, uint8_t prefix);
    
    /**
     * @brief Configure analog input threshold interrupt
     * @param channel Channel number (1-based)
     * @param upper_limit Upper threshold
     * @param lower_limit Lower threshold
     * @param delta Delta threshold
     * @param trigger Trigger selection mask
     * @return true if successful
     */
    bool setAnalogInputThreshold(uint8_t channel, int16_t upper_limit,
                                 int16_t lower_limit, int16_t delta,
                                 uint8_t trigger);
    
    /**
     * @brief Enable/disable global analog input interrupts
     * @param enable true to enable
     * @return true if successful
     */
    bool setAnalogInputInterruptEnable(bool enable);
    
    // ========================================================================
    // Analog Output Operations
    // ========================================================================
    
    /**
     * @brief Write 16-bit analog output
     * @param channel Channel number (1-based)
     * @param value Output value
     * @return true if successful
     */
    bool writeAnalogOutput16(uint8_t channel, int16_t value);
    
    /**
     * @brief Write 32-bit analog output (high resolution)
     * @param channel Channel number (1-based)
     * @param value Output value
     * @return true if successful
     */
    bool writeAnalogOutput32(uint8_t channel, int32_t value);
    
    /**
     * @brief Write scaled analog output
     * @param channel Channel number (1-based)
     * @param value Engineering value (will be scaled to raw)
     * @return true if successful
     */
    bool writeAnalogOutputScaled(uint8_t channel, int32_t value);
    
    /**
     * @brief Configure analog output scaling
     * @param channel Channel number (1-based)
     * @param offset Offset value
     * @param scaling Scaling factor (Q16 fixed point)
     * @return true if successful
     */
    bool setAnalogOutputScaling(uint8_t channel, int32_t offset, int32_t scaling);
    
    /**
     * @brief Configure analog output error behavior
     * @param channel Channel number (1-based)
     * @param mode Error mode
     * @param value Error output value
     * @return true if successful
     */
    bool setAnalogOutputErrorMode(uint8_t channel, uint8_t mode, int16_t value);
    
    // ========================================================================
    // Counter Operations (Optional)
    // ========================================================================
    
    /**
     * @brief Read counter value
     * @param channel Counter channel (1-based)
     * @return Counter value
     */
    uint32_t readCounter(uint8_t channel);
    
    /**
     * @brief Set counter preset value
     * @param channel Counter channel (1-based)
     * @param preset Preset value
     * @return true if successful
     */
    bool setCounterPreset(uint8_t channel, uint32_t preset);
    
    /**
     * @brief Load preset into counter
     * @param channel Counter channel (1-based)
     * @return true if successful
     */
    bool loadCounterPreset(uint8_t channel);
    
    /**
     * @brief Configure counter
     * @param channel Counter channel (1-based)
     * @param enable Enable counting
     * @param count_up true for up, false for down
     * @return true if successful
     */
    bool configureCounter(uint8_t channel, bool enable, bool count_up);
    
    /**
     * @brief Get counter status
     * @param channel Counter channel (1-based)
     * @return Status byte (see CounterStatusBits)
     */
    uint8_t getCounterStatus(uint8_t channel);
    
    // ========================================================================
    // Frequency Input Operations (Optional)
    // ========================================================================
    
    /**
     * @brief Read frequency input
     * @param channel Channel number (1-based)
     * @return Frequency in Hz
     */
    uint32_t readFrequencyInput(uint8_t channel);
    
    /**
     * @brief Read period input
     * @param channel Channel number (1-based)
     * @return Period in microseconds
     */
    uint32_t readPeriodInput(uint8_t channel);
    
    /**
     * @brief Read duty cycle input
     * @param channel Channel number (1-based)
     * @return Duty cycle (0-10000 = 0-100.00%)
     */
    uint16_t readDutyCycleInput(uint8_t channel);
    
    // ========================================================================
    // PWM Output Operations (Optional)
    // ========================================================================
    
    /**
     * @brief Set PWM duty cycle
     * @param channel Channel number (1-based)
     * @param duty Duty cycle (0-10000 = 0-100.00%)
     * @return true if successful
     */
    bool setPWMDutyCycle(uint8_t channel, uint16_t duty);
    
    /**
     * @brief Set PWM frequency
     * @param channel Channel number (1-based)
     * @param frequency Frequency in Hz
     * @return true if successful
     */
    bool setPWMFrequency(uint8_t channel, uint32_t frequency);
    
    /**
     * @brief Configure PWM output
     * @param channel Channel number (1-based)
     * @param frequency Frequency in Hz
     * @param duty Initial duty cycle (0-10000)
     * @return true if successful
     */
    bool configurePWM(uint8_t channel, uint32_t frequency, uint16_t duty);
    
    // ========================================================================
    // Event Handling
    // ========================================================================
    
    /**
     * @brief Set general event callback
     * @param callback Callback function
     */
    void setEventCallback(IOEventCallback callback);
    
    /**
     * @brief Set digital input change callback
     * @param callback Callback function
     */
    void setDigitalInputCallback(DigitalInputCallback callback);
    
    /**
     * @brief Set analog threshold callback
     * @param callback Callback function
     */
    void setAnalogThresholdCallback(AnalogThresholdCallback callback);
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get module diagnostic information
     * @return Human-readable diagnostic string
     */
    std::string getDiagnostics() const;
    
    /**
     * @brief Check for module faults
     * @return true if fault present
     */
    bool hasFault() const;
    
    /**
     * @brief Reset module faults
     * @return true if successful
     */
    bool resetFault();

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    bool detectCapabilities();
    bool applyDefaultConfiguration();
    bool setupPDOMapping_Minimal();
    bool setupPDOMapping_DigitalOnly();
    bool setupPDOMapping_AnalogOnly();
    bool setupPDOMapping_Combined();
    bool setupPDOMapping_Full();
    
    void processDigitalInputs(const uint8_t* data, size_t offset);
    void processAnalogInputs(const uint8_t* data, size_t offset);
    void checkDigitalInputChanges();
    void checkAnalogThresholds();
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    EtherCAT::SDO::SDOManager& m_sdo;
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    ModuleCapabilities capabilities_;
    PDOMappingPreset current_mapping_;
    
    // State tracking
    std::vector<uint8_t> digital_input_state_;
    std::vector<uint8_t> digital_output_state_;
    std::vector<int16_t> analog_input_state_;
    std::vector<int16_t> analog_output_state_;
    
    // Previous state for change detection
    std::vector<uint8_t> prev_digital_input_state_;
    std::vector<int16_t> prev_analog_input_state_;
    
    // Scaling configuration
    std::vector<AnalogInputConfig> ai_configs_;
    std::vector<AnalogOutputConfig> ao_configs_;
    
    // Counter state
    std::vector<uint32_t> counter_values_;
    
    // Callbacks
    IOEventCallback event_callback_;
    DigitalInputCallback digital_callback_;
    AnalogThresholdCallback analog_callback_;
    
    // Error tracking
    bool has_fault_;
    uint32_t last_error_;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get name of PDO mapping preset
 */
inline const char* getPDOMappingName(PDOMappingPreset preset) {
    switch (preset) {
        case PDOMappingPreset::Minimal: return "Minimal";
        case PDOMappingPreset::DigitalOnly: return "Digital Only";
        case PDOMappingPreset::AnalogOnly: return "Analog Only";
        case PDOMappingPreset::Digital16: return "Digital 16-bit";
        case PDOMappingPreset::Digital32: return "Digital 32-bit";
        case PDOMappingPreset::AnalogHighRes: return "Analog High-Res";
        case PDOMappingPreset::Combined: return "Combined";
        case PDOMappingPreset::Full: return "Full";
        case PDOMappingPreset::Custom: return "Custom";
        default: return "Unknown";
    }
}

/**
 * @brief Get name of I/O event
 */
inline const char* getIOEventName(IOEvent event) {
    switch (event) {
        case IOEvent::DigitalInputChanged: return "Digital Input Changed";
        case IOEvent::DigitalOutputUpdated: return "Digital Output Updated";
        case IOEvent::AnalogUpperLimit: return "Analog Upper Limit";
        case IOEvent::AnalogLowerLimit: return "Analog Lower Limit";
        case IOEvent::AnalogDeltaTriggered: return "Analog Delta Triggered";
        case IOEvent::CounterOverflow: return "Counter Overflow";
        case IOEvent::CounterUnderflow: return "Counter Underflow";
        case IOEvent::CounterPresetReached: return "Counter Preset Reached";
        case IOEvent::CommunicationError: return "Communication Error";
        case IOEvent::Initialized: return "Initialized";
        case IOEvent::ModuleFault: return "Module Fault";
        default: return "Unknown";
    }
}

/**
 * @brief Convert engineering value with scaling
 * @param raw Raw value
 * @param offset Offset
 * @param scale Scale factor (Q16)
 * @return Scaled value
 */
inline int32_t applyScaling(int32_t raw, int32_t offset, int32_t scale) {
    int64_t scaled = (static_cast<int64_t>(raw) * scale) >> 16;
    return static_cast<int32_t>(scaled) + offset;
}

/**
 * @brief Reverse scaling to get raw value
 * @param scaled Scaled value
 * @param offset Offset
 * @param scale Scale factor (Q16)
 * @return Raw value
 */
inline int32_t reverseScaling(int32_t scaled, int32_t offset, int32_t scale) {
    if (scale == 0) return 0;
    int64_t raw = (static_cast<int64_t>(scaled - offset) << 16) / scale;
    return static_cast<int32_t>(raw);
}

} // namespace CiA401
