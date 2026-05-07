/**
 * @file CiA404Device.hpp
 * @brief CiA 404 Measuring Device and Closed Loop Controller Interface
 * 
 * @details
 * Comprehensive controller for CiA 404 compliant devices including:
 * - Process data inputs with scaling and engineering units
 * - Process data outputs for control signals
 * - PID closed loop controller with anti-windup
 * - Alarm and warning monitoring
 * - Two-point calibration support
 * - Sensor diagnostics
 * 
 * @note Supports temperature transmitters, pressure transmitters, 
 *       flow meters, level sensors, and general analog I/O
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "profiles/cia404/CiA404Defs.hpp"

namespace EtherCAT { namespace SDO { class SDOManager; } }

namespace CiA404 {

// Forward declarations
class MeasuringDevice;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief PDO mapping presets
 */
enum class PDOMappingPreset : uint8_t {
    /** Minimal - single process value */
    Minimal,
    
    /** Process input with status */
    InputWithStatus,
    
    /** Multiple inputs */
    MultiInput,
    
    /** Controller mode - setpoint and feedback */
    Controller,
    
    /** Full controller with all parameters */
    ControllerFull,
    
    /** Process I/O with alarms */
    WithAlarms,
    
    /** Custom mapping */
    Custom
};

/**
 * @brief Device events
 */
enum class DeviceEvent : uint8_t {
    /** Value updated */
    ValueUpdated,
    
    /** High-high alarm triggered */
    AlarmHighHigh,
    
    /** High alarm triggered */
    AlarmHigh,
    
    /** Low alarm triggered */
    AlarmLow,
    
    /** Low-low alarm triggered */
    AlarmLowLow,
    
    /** Rate of change alarm */
    AlarmRateOfChange,
    
    /** Warning high */
    WarningHigh,
    
    /** Warning low */
    WarningLow,
    
    /** Sensor fault */
    SensorFault,
    
    /** Controller active */
    ControllerActive,
    
    /** Controller output saturated */
    OutputSaturated,
    
    /** Calibration required */
    CalibrationRequired,
    
    /** Calibration complete */
    CalibrationComplete,
    
    /** Calibration failed */
    CalibrationFailed,
    
    /** Communication error */
    CommunicationError
};

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief General event callback
 */
using DeviceEventCallback = std::function<void(DeviceEvent event, uint16_t slave_addr,
                                               uint8_t channel, int32_t value)>;

/**
 * @brief Alarm callback
 */
using AlarmCallback = std::function<void(uint16_t slave_addr, uint8_t channel,
                                        uint16_t alarm_bits, int32_t value)>;

/**
 * @brief Value update callback
 */
using ValueCallback = std::function<void(uint16_t slave_addr, uint8_t channel,
                                        int32_t raw_value, int32_t eng_value)>;

// ============================================================================
// Capability Structures
// ============================================================================

/**
 * @brief Device capabilities discovered during initialization
 */
struct DeviceCapabilities {
    uint8_t num_process_inputs;    ///< Number of process inputs
    uint8_t num_process_outputs;   ///< Number of process outputs
    bool has_controller;           ///< Has closed loop controller
    bool has_calibration;          ///< Has calibration support
    bool has_linearization;        ///< Has linearization table
    bool has_alarms;               ///< Has alarm functionality
    bool has_diagnostics;          ///< Has diagnostic readings
    
    DeviceCapabilities()
        : num_process_inputs(0)
        , num_process_outputs(0)
        , has_controller(false)
        , has_calibration(false)
        , has_linearization(false)
        , has_alarms(false)
        , has_diagnostics(false)
    {}
};

/**
 * @brief Process input channel state
 */
struct ProcessInputState {
    int32_t raw_value;             ///< Raw ADC value
    int32_t scaled_value;          ///< Scaled engineering value
    int32_t filtered_value;        ///< Filtered value
    uint8_t status;                ///< Channel status
    uint16_t alarm_status;         ///< Active alarms
    uint16_t warning_status;       ///< Active warnings
    
    ProcessInputState()
        : raw_value(0)
        , scaled_value(0)
        , filtered_value(0)
        , status(0)
        , alarm_status(0)
        , warning_status(0)
    {}
};

/**
 * @brief Controller state
 */
struct ControllerState {
    int32_t setpoint;              ///< Current setpoint
    int32_t actual_value;          ///< Process variable
    int32_t deviation;             ///< Control deviation (error)
    int32_t output;                ///< Controller output
    int32_t integral_sum;          ///< Integral accumulator
    uint8_t mode;                  ///< Current mode
    uint16_t status;               ///< Controller status bits
    
    ControllerState()
        : setpoint(0)
        , actual_value(0)
        , deviation(0)
        , output(0)
        , integral_sum(0)
        , mode(ControllerModes::Manual)
        , status(0)
    {}
    
    bool isActive() const { return status & ControllerStatusBits::Active; }
    bool isOutputSaturated() const {
        return (status & ControllerStatusBits::OutputUpperLimit) ||
               (status & ControllerStatusBits::OutputLowerLimit);
    }
};

// ============================================================================
// Main MeasuringDevice Class
// ============================================================================

/**
 * @brief CiA 404 Measuring Device and Closed Loop Controller
 * 
 * @details
 * Provides full control of CiA 404 compliant devices including:
 * - Process value reading with automatic scaling
 * - PID control with anti-windup and bumpless transfer
 * - Alarm and warning management
 * - Two-point calibration
 * - Sensor diagnostics
 * 
 * Usage:
 * @code
 * CiA404::MeasuringDevice device(1);
 * device.initialize();
 * 
 * // Configure input
 * device.configureInput(1, CiA404::InputRange::Current_4_20mA,
 *                       CiA404::EngineeringUnit::Bar, 2);
 * device.setInputScaling(1, 0, 10000, 0, 100); // 4-20mA = 0-10 bar
 * 
 * // Read scaled value
 * int32_t pressure = device.getScaledValue(1);
 * 
 * // Enable PID control
 * CiA404::PIDParameters pid;
 * pid.kp = 0x10000; // Kp = 1.0
 * pid.ti = 5000;    // Ti = 5 seconds
 * device.setPIDParameters(pid);
 * device.setControllerMode(CiA404::ControllerModes::PID_Auto);
 * @endcode
 */
class MeasuringDevice {
public:
    // ========================================================================
    // Construction and Initialization
    // ========================================================================
    
    /**
     * @brief Construct measuring device controller
     * @param slave_addr Position address or configured address
     * @param use_configured_addr If true, slave_addr is configured address
     */
    explicit MeasuringDevice(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr = false);
    
    /**
     * @brief Destructor
     */
    ~MeasuringDevice();
    
    /**
     * @brief Initialize the device
     * @return true if successful
     */
    bool initialize();
    
    /**
     * @brief Check if initialized
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
     * @brief Get device capabilities
     */
    const DeviceCapabilities& getCapabilities() const { return capabilities_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    /**
     * @brief Apply PDO mapping preset
     * @param preset Mapping preset to apply
     * @return true if successful
     */
    bool applyPDOMapping(PDOMappingPreset preset);
    
    /**
     * @brief Get current PDO mapping
     */
    PDOMappingPreset getCurrentMapping() const { return current_mapping_; }
    
    // ========================================================================
    // Update Cycle
    // ========================================================================
    
    /**
     * @brief Process received PDO data
     * @param data PDO data
     * @param len Data length
     */
    void processTxPDO(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare PDO data for transmission
     * @param data Output buffer
     * @param max_len Buffer size
     * @return Bytes written
     */
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    
    /**
     * @brief Update device state (call periodically)
     */
    void update();
    
    // ========================================================================
    // Process Input Operations
    // ========================================================================
    
    /**
     * @brief Get raw process input value
     * @param channel Channel number (1-based)
     * @return Raw value
     */
    int32_t getRawValue(uint8_t channel);
    
    /**
     * @brief Get scaled process input value
     * @param channel Channel number (1-based)
     * @return Scaled engineering value
     */
    int32_t getScaledValue(uint8_t channel);
    
    /**
     * @brief Get filtered process input value
     * @param channel Channel number (1-based)
     * @return Filtered value
     */
    int32_t getFilteredValue(uint8_t channel);
    
    /**
     * @brief Get process input status
     * @param channel Channel number (1-based)
     * @return Status byte
     */
    uint8_t getInputStatus(uint8_t channel);
    
    /**
     * @brief Get process input state structure
     * @param channel Channel number (1-based)
     * @return Process input state
     */
    const ProcessInputState& getInputState(uint8_t channel);
    
    /**
     * @brief Configure process input channel
     * @param channel Channel number (1-based)
     * @param input_range Input range type
     * @param unit Engineering unit code
     * @param decimal_places Number of decimal places
     * @return true if successful
     */
    bool configureInput(uint8_t channel, uint8_t input_range,
                        uint16_t unit, uint8_t decimal_places);
    
    /**
     * @brief Set input scaling from raw to engineering units
     * @param channel Channel number (1-based)
     * @param raw_min Minimum raw value
     * @param raw_max Maximum raw value
     * @param eng_min Minimum engineering value
     * @param eng_max Maximum engineering value
     * @return true if successful
     */
    bool setInputScaling(uint8_t channel, int32_t raw_min, int32_t raw_max,
                        int32_t eng_min, int32_t eng_max);
    
    /**
     * @brief Set input filter time constant
     * @param channel Channel number (1-based)
     * @param filter_time_ms Filter time in milliseconds
     * @return true if successful
     */
    bool setInputFilter(uint8_t channel, uint16_t filter_time_ms);
    
    // ========================================================================
    // Process Output Operations
    // ========================================================================
    
    /**
     * @brief Set process output value
     * @param channel Channel number (1-based)
     * @param value Output value
     * @return true if successful
     */
    bool setOutputValue(uint8_t channel, int32_t value);
    
    /**
     * @brief Get current output value
     * @param channel Channel number (1-based)
     * @return Output value
     */
    int32_t getOutputValue(uint8_t channel);
    
    /**
     * @brief Configure output range
     * @param channel Channel number (1-based)
     * @param output_range Output range type
     * @return true if successful
     */
    bool configureOutput(uint8_t channel, uint8_t output_range);
    
    /**
     * @brief Set output error behavior
     * @param channel Channel number (1-based)
     * @param behavior Error behavior code
     * @param error_value Value to output on error
     * @return true if successful
     */
    bool setOutputErrorBehavior(uint8_t channel, uint8_t behavior, int32_t error_value);
    
    // ========================================================================
    // Closed Loop Controller Operations
    // ========================================================================
    
    /**
     * @brief Set controller setpoint
     * @param setpoint Target setpoint value
     * @return true if successful
     */
    bool setSetpoint(int32_t setpoint);
    
    /**
     * @brief Get current setpoint
     * @return Setpoint value
     */
    int32_t getSetpoint() const;
    
    /**
     * @brief Set controller mode
     * @param mode Controller mode (see ControllerModes)
     * @return true if successful
     */
    bool setControllerMode(uint8_t mode);
    
    /**
     * @brief Get current controller mode
     * @return Controller mode
     */
    uint8_t getControllerMode() const;
    
    /**
     * @brief Set PID parameters
     * @param params PID parameters structure
     * @return true if successful
     */
    bool setPIDParameters(const PIDParameters& params);
    
    /**
     * @brief Get current PID parameters
     * @return PID parameters
     */
    PIDParameters getPIDParameters();
    
    /**
     * @brief Set individual PID gain
     * @param kp Proportional gain (Q16 fixed point)
     * @param ti Integral time in ms
     * @param td Derivative time in ms
     * @return true if successful
     */
    bool setPIDGains(int32_t kp, int32_t ti, int32_t td);
    
    /**
     * @brief Set output limits
     * @param min Minimum output
     * @param max Maximum output
     * @return true if successful
     */
    bool setOutputLimits(int32_t min, int32_t max);
    
    /**
     * @brief Set setpoint ramp rate
     * @param rate Ramp rate per sample
     * @return true if successful
     */
    bool setSetpointRamp(int32_t rate);
    
    /**
     * @brief Set feedforward gain
     * @param gain Feedforward gain (Q16)
     * @return true if successful
     */
    bool setFeedforward(int32_t gain);
    
    /**
     * @brief Get controller state
     * @return Controller state
     */
    const ControllerState& getControllerState() const;
    
    /**
     * @brief Get control deviation (error)
     * @return Deviation value
     */
    int32_t getDeviation() const;
    
    /**
     * @brief Get controller output
     * @return Output value
     */
    int32_t getControllerOutput() const;
    
    /**
     * @brief Check if controller is active
     * @return true if active
     */
    bool isControllerActive() const;
    
    /**
     * @brief Reset controller integrator
     * @return true if successful
     */
    bool resetIntegrator();
    
    // ========================================================================
    // Alarm Operations
    // ========================================================================
    
    /**
     * @brief Configure alarm limits
     * @param channel Channel number (1-based)
     * @param config Alarm configuration
     * @return true if successful
     */
    bool configureAlarms(uint8_t channel, const AlarmConfig& config);
    
    /**
     * @brief Set high-high alarm limit
     * @param channel Channel number
     * @param limit Alarm limit
     * @return true if successful
     */
    bool setAlarmHighHigh(uint8_t channel, int32_t limit);
    
    /**
     * @brief Set high alarm limit
     */
    bool setAlarmHigh(uint8_t channel, int32_t limit);
    
    /**
     * @brief Set low alarm limit
     */
    bool setAlarmLow(uint8_t channel, int32_t limit);
    
    /**
     * @brief Set low-low alarm limit
     */
    bool setAlarmLowLow(uint8_t channel, int32_t limit);
    
    /**
     * @brief Set alarm hysteresis
     */
    bool setAlarmHysteresis(uint8_t channel, int32_t hysteresis);
    
    /**
     * @brief Get alarm status
     * @param channel Channel number (1-based)
     * @return Alarm status bits
     */
    uint16_t getAlarmStatus(uint8_t channel);
    
    /**
     * @brief Get warning status
     * @param channel Channel number (1-based)
     * @return Warning status bits
     */
    uint16_t getWarningStatus(uint8_t channel);
    
    /**
     * @brief Check if any alarm is active
     * @param channel Channel number (1-based)
     * @return true if alarm active
     */
    bool hasAlarm(uint8_t channel);
    
    /**
     * @brief Acknowledge alarms
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool acknowledgeAlarms(uint8_t channel);
    
    // ========================================================================
    // Calibration Operations
    // ========================================================================
    
    /**
     * @brief Start zero point calibration
     * @param channel Channel number (1-based)
     * @param zero_eng Engineering value at zero
     * @return true if successful
     */
    bool calibrateZero(uint8_t channel, int32_t zero_eng);
    
    /**
     * @brief Start span calibration
     * @param channel Channel number (1-based)
     * @param span_eng Engineering value at span
     * @return true if successful
     */
    bool calibrateSpan(uint8_t channel, int32_t span_eng);
    
    /**
     * @brief Perform two-point calibration
     * @param channel Channel number (1-based)
     * @param raw1 First raw value
     * @param eng1 First engineering value
     * @param raw2 Second raw value
     * @param eng2 Second engineering value
     * @return true if successful
     */
    bool calibrateTwoPoint(uint8_t channel, int32_t raw1, int32_t eng1,
                           int32_t raw2, int32_t eng2);
    
    /**
     * @brief Accept current calibration
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool acceptCalibration(uint8_t channel);
    
    /**
     * @brief Cancel/reject calibration
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool rejectCalibration(uint8_t channel);
    
    /**
     * @brief Restore factory calibration
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool restoreFactoryCalibration(uint8_t channel);
    
    /**
     * @brief Save calibration to NVS
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool saveCalibration(uint8_t channel);
    
    /**
     * @brief Get calibration status
     * @param channel Channel number (1-based)
     * @return Calibration status bits
     */
    uint8_t getCalibrationStatus(uint8_t channel);
    
    /**
     * @brief Set tare (zero) value
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool setTare(uint8_t channel);
    
    /**
     * @brief Clear tare value
     * @param channel Channel number (1-based)
     * @return true if successful
     */
    bool clearTare(uint8_t channel);
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get sensor status
     * @param channel Channel number (1-based)
     * @return Sensor status bits
     */
    uint8_t getSensorStatus(uint8_t channel);
    
    /**
     * @brief Get sensor supply voltage
     * @param channel Channel number (1-based)
     * @return Supply voltage in mV
     */
    uint16_t getSensorSupplyVoltage(uint8_t channel);
    
    /**
     * @brief Get sensor temperature
     * @param channel Channel number (1-based)
     * @return Temperature in 0.1°C
     */
    int16_t getSensorTemperature(uint8_t channel);
    
    /**
     * @brief Get signal quality
     * @param channel Channel number (1-based)
     * @return Quality 0-100%
     */
    uint8_t getSignalQuality(uint8_t channel);
    
    /**
     * @brief Get operating hours
     * @return Operating hours
     */
    uint32_t getOperatingHours();
    
    /**
     * @brief Get human-readable diagnostics
     * @return Diagnostic string
     */
    std::string getDiagnostics() const;
    
    // ========================================================================
    // Event Handling
    // ========================================================================
    
    /**
     * @brief Set general event callback
     */
    void setEventCallback(DeviceEventCallback callback);
    
    /**
     * @brief Set alarm callback
     */
    void setAlarmCallback(AlarmCallback callback);
    
    /**
     * @brief Set value update callback
     */
    void setValueCallback(ValueCallback callback);

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    bool detectCapabilities();
    bool applyDefaultConfiguration();
    
    void processInputPDO(const uint8_t* data, size_t len);
    void processControllerPDO(const uint8_t* data, size_t len);
    void checkAlarms();
    void fireEvent(DeviceEvent event, uint8_t channel, int32_t value);
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    EtherCAT::SDO::SDOManager& m_sdo;
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    DeviceCapabilities capabilities_;
    PDOMappingPreset current_mapping_;
    
    // Input state
    std::vector<ProcessInputState> input_states_;
    std::vector<AnalogInputChannelConfig> input_configs_;
    std::vector<AlarmConfig> alarm_configs_;
    std::vector<CalibrationData> calibration_data_;
    
    // Output state
    std::vector<int32_t> output_values_;
    
    // Controller state
    ControllerState controller_state_;
    PIDParameters pid_params_;
    
    // Previous values for change detection
    std::vector<uint16_t> prev_alarm_status_;
    
    // Callbacks
    DeviceEventCallback event_callback_;
    AlarmCallback alarm_callback_;
    ValueCallback value_callback_;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get name of device event
 */
inline const char* getDeviceEventName(DeviceEvent event) {
    switch (event) {
        case DeviceEvent::ValueUpdated: return "Value Updated";
        case DeviceEvent::AlarmHighHigh: return "Alarm High-High";
        case DeviceEvent::AlarmHigh: return "Alarm High";
        case DeviceEvent::AlarmLow: return "Alarm Low";
        case DeviceEvent::AlarmLowLow: return "Alarm Low-Low";
        case DeviceEvent::AlarmRateOfChange: return "Alarm Rate of Change";
        case DeviceEvent::WarningHigh: return "Warning High";
        case DeviceEvent::WarningLow: return "Warning Low";
        case DeviceEvent::SensorFault: return "Sensor Fault";
        case DeviceEvent::ControllerActive: return "Controller Active";
        case DeviceEvent::OutputSaturated: return "Output Saturated";
        case DeviceEvent::CalibrationRequired: return "Calibration Required";
        case DeviceEvent::CalibrationComplete: return "Calibration Complete";
        case DeviceEvent::CalibrationFailed: return "Calibration Failed";
        case DeviceEvent::CommunicationError: return "Communication Error";
        default: return "Unknown";
    }
}

/**
 * @brief Get name of input range type
 */
inline const char* getInputRangeName(uint8_t range) {
    switch (range) {
        case InputRange::Voltage_0_10V: return "0-10V";
        case InputRange::Voltage_PM10V: return "±10V";
        case InputRange::Voltage_0_5V: return "0-5V";
        case InputRange::Voltage_1_5V: return "1-5V";
        case InputRange::Current_4_20mA: return "4-20mA";
        case InputRange::Current_0_20mA: return "0-20mA";
        case InputRange::RTD_PT100: return "PT100";
        case InputRange::RTD_PT1000: return "PT1000";
        case InputRange::TC_Type_J: return "TC Type J";
        case InputRange::TC_Type_K: return "TC Type K";
        case InputRange::TC_Type_T: return "TC Type T";
        case InputRange::StrainGauge: return "Strain Gauge";
        case InputRange::LVDT: return "LVDT";
        default: return "Unknown";
    }
}

/**
 * @brief Get name of controller mode
 */
inline const char* getControllerModeName(uint8_t mode) {
    switch (mode) {
        case ControllerModes::Manual: return "Manual";
        case ControllerModes::PID_Auto: return "PID Auto";
        case ControllerModes::P_Only: return "P Only";
        case ControllerModes::PI_Control: return "PI Control";
        case ControllerModes::PD_Control: return "PD Control";
        case ControllerModes::CascadeInner: return "Cascade Inner";
        case ControllerModes::CascadeOuter: return "Cascade Outer";
        case ControllerModes::RatioControl: return "Ratio Control";
        case ControllerModes::FeedforwardOnly: return "Feedforward Only";
        default: return "Unknown";
    }
}

/**
 * @brief Convert Q16 fixed point to float
 */
inline float q16ToFloat(int32_t q16) {
    return static_cast<float>(q16) / 65536.0f;
}

/**
 * @brief Convert float to Q16 fixed point
 */
inline int32_t floatToQ16(float f) {
    return static_cast<int32_t>(f * 65536.0f);
}

} // namespace CiA404
