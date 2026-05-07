/**
 * @file CiA408Valve.hpp
 * @brief CiA 408 Fluid Power Valve Controller
 *
 * Provides comprehensive control over CiA 408 compliant proportional valves,
 * servo valves, and variable displacement pumps.
 *
 * Features:
 * - Open loop and closed loop control
 * - Position, velocity, pressure, force control modes
 * - Spool position feedback (LVDT)
 * - Dual pressure sensing (A/B ports)
 * - Dither control for friction reduction
 * - Comprehensive diagnostics
 * - Calibration support
 */

#pragma once

#include "profiles/cia408/CiA408Defs.hpp"
#include <cstdint>
#include <string>
#include <functional>

namespace CiA408 {

// ============================================================================
// Forward Declarations
// ============================================================================

class ValveController;

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Valve specifications
 */
struct ValveSpec {
    uint8_t  valve_type = 0;
    uint16_t nominal_flow = 0;      // L/min * 10
    uint16_t nominal_pressure = 0;  // bar * 10
    int16_t  nominal_stroke = 0;    // µm
    uint16_t response_time = 0;     // ms
    uint8_t  hysteresis = 0;        // 0.1%
    uint8_t  repeatability = 0;     // 0.1%
};

/**
 * @brief Controller parameters
 */
struct ControllerParams {
    // Position loop
    int32_t pos_kp = 0;
    int32_t pos_ki = 0;
    int32_t pos_kd = 0;
    int32_t pos_kv = 0;    // Velocity feedforward
    int32_t pos_ka = 0;    // Acceleration feedforward
    int32_t pos_limit = 0;
    
    // Velocity loop
    int32_t vel_kp = 0;
    int32_t vel_ki = 0;
    int32_t vel_limit = 0;
    
    // Pressure loop
    int32_t prs_kp = 0;
    int32_t prs_ki = 0;
    int32_t prs_limit = 0;
};

/**
 * @brief Dither configuration
 */
struct DitherConfig {
    uint16_t amplitude = 0;    // 0.01% of nominal
    uint16_t frequency = 0;    // Hz
    bool     enabled = false;
};

/**
 * @brief Current valve state
 */
struct ValveState {
    uint16_t statusword = 0;
    int16_t  setpoint = 0;         // ±10000 = ±100.00%
    int16_t  actual_value = 0;
    
    // Position feedback
    int32_t  position_actual = 0;  // µm
    int16_t  velocity_actual = 0;  // mm/s * 10
    
    // Pressure readings
    int16_t  pressure_a = 0;       // bar * 10
    int16_t  pressure_b = 0;
    int16_t  pressure_p = 0;       // Supply
    int16_t  pressure_t = 0;       // Tank
    
    // Electrical readings
    int16_t  current_a = 0;        // mA * 10
    int16_t  current_b = 0;
    int16_t  temperature = 0;      // °C * 10
    
    // Diagnostic
    uint16_t fault_code = 0;
    uint16_t warning_code = 0;
    int16_t  following_error = 0;
    int16_t  controller_output = 0;
    
    // Status checks
    bool isReady() const { return statusword & StatuswordBits::Ready; }
    bool isEnabled() const { return statusword & StatuswordBits::Enabled; }
    bool hasFault() const { return statusword & StatuswordBits::Fault; }
    bool hasWarning() const { return statusword & StatuswordBits::Warning; }
    bool isTargetReached() const { return statusword & StatuswordBits::TargetReached; }
    bool isInPosition() const { return statusword & StatuswordBits::InPosition; }
    bool isOverloaded() const { return statusword & StatuswordBits::Overload; }
    
    float getSetpointPercent() const { return rawToPercent(setpoint); }
    float getActualPercent() const { return rawToPercent(actual_value); }
    float getPressureABar() const { return rawToBar(pressure_a); }
    float getPressureBBar() const { return rawToBar(pressure_b); }
    float getTemperatureCelsius() const { return temperature / 10.0f; }
};

/**
 * @brief Device capabilities
 */
struct ValveCapabilities {
    uint8_t  valve_type = 0;
    bool     has_position_feedback = false;
    bool     has_pressure_sensors = false;
    bool     has_dual_coils = false;
    bool     supports_closed_loop = false;
    bool     supports_pressure_control = false;
    bool     supports_force_control = false;
    bool     supports_dither = false;
    uint8_t  num_channels = 1;
};

// ============================================================================
// Callback Types
// ============================================================================

using StateChangeCallback = std::function<void(uint16_t old_status, uint16_t new_status)>;
using FaultCallback = std::function<void(uint16_t fault_code)>;
using TargetReachedCallback = std::function<void()>;

// ============================================================================
// PDO Mapping Presets
// ============================================================================

enum class PDOMappingPreset {
    Basic,           // Controlword + setpoint + statusword + actual
    Extended,        // + pressure readings + mode
    Position,        // For servo valves with position control
    Full,            // All available data
    Custom
};

// ============================================================================
// Valve Controller Class
// ============================================================================

class ValveController {
public:
    // ========================================================================
    // Construction and Lifecycle
    // ========================================================================
    
    /**
     * @brief Construct valve controller
     * @param slave_addr Slave station address
     * @param use_configured_addr Use configured vs auto-increment addressing
     */
    explicit ValveController(uint16_t slave_addr, bool use_configured_addr = false);
    
    ~ValveController();
    
    /**
     * @brief Initialize the valve controller
     */
    bool initialize();
    
    /**
     * @brief Check initialization status
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get device capabilities
     */
    const ValveCapabilities& getCapabilities() const { return capabilities_; }
    
    /**
     * @brief Get valve specifications
     */
    const ValveSpec& getValveSpec() const { return valve_spec_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    /**
     * @brief Apply PDO mapping preset
     */
    bool applyPDOMapping(PDOMappingPreset preset);
    
    // ========================================================================
    // Cyclic Update
    // ========================================================================
    
    /**
     * @brief Process received TxPDO data
     */
    void processTxPDO(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare RxPDO data for transmission
     */
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    
    /**
     * @brief Update via SDO (non-cyclic)
     */
    void update();
    
    // ========================================================================
    // Basic Control
    // ========================================================================
    
    /**
     * @brief Enable valve output
     */
    bool enable();
    
    /**
     * @brief Disable valve output
     */
    bool disable();
    
    /**
     * @brief Reset faults
     */
    bool resetFault();
    
    /**
     * @brief Emergency stop (fast closing)
     */
    bool fastStop();
    
    /**
     * @brief Check if enabled
     */
    bool isEnabled() const;
    
    /**
     * @brief Check for faults
     */
    bool hasFault() const;
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    /**
     * @brief Set operating mode
     */
    bool setOperatingMode(uint8_t mode);
    
    /**
     * @brief Get current operating mode
     */
    uint8_t getOperatingMode() const { return current_mode_; }
    
    /**
     * @brief Enable closed loop control
     */
    bool enableClosedLoop(bool enable);
    
    /**
     * @brief Enable pressure compensation
     */
    bool enablePressureCompensation(bool enable);
    
    // ========================================================================
    // Setpoint Control
    // ========================================================================
    
    /**
     * @brief Set valve opening (percentage)
     * @param percent Setpoint from -100.0 to +100.0
     */
    bool setSetpoint(float percent);
    
    /**
     * @brief Set valve opening (raw value)
     * @param value Raw setpoint from -10000 to +10000
     */
    bool setSetpointRaw(int16_t value);
    
    /**
     * @brief Get current setpoint
     */
    float getSetpoint() const;
    int16_t getSetpointRaw() const;
    
    /**
     * @brief Get actual valve position
     */
    float getActualValue() const;
    int16_t getActualValueRaw() const;
    
    /**
     * @brief Set setpoint ramp (rate limiter)
     * @param rate_percent_per_sec Rate in %/s
     */
    bool setSetpointRamp(float rate_percent_per_sec);
    
    // ========================================================================
    // Position Control (Servo Valves)
    // ========================================================================
    
    /**
     * @brief Set position setpoint (micrometers)
     */
    bool setPosition(int32_t position_um);
    
    /**
     * @brief Get actual position (micrometers)
     */
    int32_t getPosition() const;
    
    /**
     * @brief Set position window for "in position" detection
     */
    bool setPositionWindow(int32_t window_um, uint16_t time_ms);
    
    /**
     * @brief Check if position target reached
     */
    bool isInPosition() const;
    
    // ========================================================================
    // Velocity Control
    // ========================================================================
    
    /**
     * @brief Set velocity setpoint (mm/s)
     */
    bool setVelocity(float velocity_mm_s);
    
    /**
     * @brief Get actual velocity (mm/s)
     */
    float getVelocity() const;
    
    // ========================================================================
    // Pressure Control
    // ========================================================================
    
    /**
     * @brief Set pressure setpoint (bar)
     */
    bool setPressureSetpoint(float pressure_bar);
    
    /**
     * @brief Get pressure at port A (bar)
     */
    float getPressureA() const;
    
    /**
     * @brief Get pressure at port B (bar)
     */
    float getPressureB() const;
    
    /**
     * @brief Get supply pressure (bar)
     */
    float getSupplyPressure() const;
    
    /**
     * @brief Get differential pressure A-B (bar)
     */
    float getDifferentialPressure() const;
    
    /**
     * @brief Set maximum pressure limit
     */
    bool setMaxPressure(float pressure_bar);
    
    // ========================================================================
    // Controller Tuning
    // ========================================================================
    
    /**
     * @brief Set position controller gains
     */
    bool setPositionGains(int32_t kp, int32_t ki, int32_t kd);
    
    /**
     * @brief Set velocity controller gains
     */
    bool setVelocityGains(int32_t kp, int32_t ki);
    
    /**
     * @brief Set pressure controller gains
     */
    bool setPressureGains(int32_t kp, int32_t ki);
    
    /**
     * @brief Get controller parameters
     */
    ControllerParams getControllerParams();
    
    /**
     * @brief Set controller parameters
     */
    bool setControllerParams(const ControllerParams& params);
    
    // ========================================================================
    // Dither Control
    // ========================================================================
    
    /**
     * @brief Configure dither
     */
    bool configureDither(uint16_t amplitude, uint16_t frequency_hz);
    
    /**
     * @brief Enable/disable dither
     */
    bool enableDither(bool enable);
    
    /**
     * @brief Get dither configuration
     */
    DitherConfig getDitherConfig() const { return dither_config_; }
    
    // ========================================================================
    // Limits
    // ========================================================================
    
    /**
     * @brief Set position limits
     */
    bool setPositionLimits(int32_t min_um, int32_t max_um);
    
    /**
     * @brief Set velocity limits
     */
    bool setVelocityLimits(float max_mm_s, float accel_mm_s2);
    
    /**
     * @brief Set current limits (mA)
     */
    bool setCurrentLimits(uint16_t max_current_a_ma, uint16_t max_current_b_ma);
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get complete valve state
     */
    const ValveState& getState() const { return state_; }
    
    /**
     * @brief Get coil current (mA)
     */
    float getCoilCurrentA() const;
    float getCoilCurrentB() const;
    
    /**
     * @brief Get coil temperature (°C)
     */
    float getCoilTemperature() const;
    
    /**
     * @brief Get supply voltage (V)
     */
    float getSupplyVoltage() const;
    
    /**
     * @brief Get following error
     */
    float getFollowingError() const;
    
    /**
     * @brief Get operating hours
     */
    uint32_t getOperatingHours() const;
    
    /**
     * @brief Get cycle count
     */
    uint32_t getCycleCount() const;
    
    /**
     * @brief Get fault code
     */
    uint16_t getFaultCode() const { return state_.fault_code; }
    
    /**
     * @brief Get warning code
     */
    uint16_t getWarningCode() const { return state_.warning_code; }
    
    /**
     * @brief Get diagnostic string
     */
    std::string getDiagnostics() const;
    
    // ========================================================================
    // Calibration
    // ========================================================================
    
    /**
     * @brief Start null point calibration
     */
    bool startNullCalibration();
    
    /**
     * @brief Start gain calibration
     */
    bool startGainCalibration();
    
    /**
     * @brief Start auto-tuning
     */
    bool startAutoTune();
    
    /**
     * @brief Store calibration to non-volatile memory
     */
    bool storeCalibration();
    
    /**
     * @brief Reset calibration to factory defaults
     */
    bool resetCalibration();
    
    /**
     * @brief Get calibration status
     */
    uint8_t getCalibrationStatus();
    
    /**
     * @brief Set null offset manually
     */
    bool setNullOffset(int16_t offset);
    
    /**
     * @brief Set deadband
     */
    bool setDeadband(uint16_t deadband);
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setStateChangeCallback(StateChangeCallback callback);
    void setFaultCallback(FaultCallback callback);
    void setTargetReachedCallback(TargetReachedCallback callback);

private:
    // ========================================================================
    // Private Methods
    // ========================================================================
    
    bool detectCapabilities();
    bool readValveSpec();
    void processBasicPDO(const uint8_t* data, size_t len);
    void processExtendedPDO(const uint8_t* data, size_t len);
    void processPositionPDO(const uint8_t* data, size_t len);
    void processFullPDO(const uint8_t* data, size_t len);
    void checkStateChanges();
    void fireFault(uint16_t fault);
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    // ========================================================================
    // Private Data
    // ========================================================================
    
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    ValveCapabilities capabilities_;
    ValveSpec valve_spec_;
    PDOMappingPreset current_mapping_;
    
    ValveState state_;
    uint16_t prev_statusword_;
    
    uint16_t controlword_;
    int16_t target_setpoint_;
    int32_t target_position_;
    int16_t target_velocity_;
    int16_t target_pressure_;
    
    uint8_t current_mode_;
    DitherConfig dither_config_;
    ControllerParams controller_params_;
    
    // Callbacks
    StateChangeCallback state_change_callback_;
    FaultCallback fault_callback_;
    TargetReachedCallback target_reached_callback_;
};

} // namespace CiA408
