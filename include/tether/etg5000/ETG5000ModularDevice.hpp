/**
 * @file ETG5000ModularDevice.hpp
 * @brief ETG.5000.1 Modular Device Controller
 *
 * Provides comprehensive support for modular EtherCAT devices with
 * hot-swappable I/O modules.
 *
 * Features:
 * - Module detection and enumeration
 * - Slot configuration management
 * - Hot-swap support
 * - Per-module diagnostics
 * - Flexible PDO mapping
 * - Auto-configuration support
 */

#pragma once

#include "etg5000/ETG5000Defs.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <vector>
#include <memory>

namespace EtherCAT { namespace SDO { class SDOManager; } }

namespace ETG5000 {

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Module descriptor
 */
struct ModuleDescriptor {
    uint8_t  slot = 0;
    uint16_t expected_type = ModuleType::Unknown;
    uint32_t expected_vendor = 0;
    uint32_t expected_product = 0;
    bool     optional = false;
    bool     allow_compatible = false;  // Allow compatible replacement
};

/**
 * @brief Module runtime state
 */
struct ModuleState {
    uint8_t  slot = 0;
    uint16_t module_type = ModuleType::Unknown;
    uint8_t  status = ModuleStatus::NotPresent;
    uint16_t diag_status = 0;
    uint16_t last_error = 0;
    int16_t  temperature = 0;     // 0.1°C
    uint16_t supply_voltage = 0;  // 0.1V
    uint32_t error_count = 0;
    
    bool isPresent() const { return status != ModuleStatus::NotPresent; }
    bool isOperational() const { return status == ModuleStatus::Operational; }
    bool hasError() const { return status == ModuleStatus::Error; }
    float getTemperature_C() const { return temperature / 10.0f; }
    float getSupplyVoltage_V() const { return supply_voltage / 10.0f; }
};

/**
 * @brief Device configuration state
 */
struct DeviceConfigState {
    uint8_t detected_modules = 0;
    uint8_t configured_slots = 0;
    uint8_t config_state = ConfigState::Unconfigured;
    bool all_modules_ok = false;
    bool config_mismatch = false;
    std::vector<uint8_t> missing_slots;
    std::vector<uint8_t> extra_slots;
    std::vector<uint8_t> mismatched_slots;
};

/**
 * @brief Overall device state
 */
struct DeviceState {
    uint16_t statusword = 0;
    DeviceConfigState config;
    std::vector<ModuleState> modules;
    
    bool isReady() const { return statusword & StatuswordBits::Ready; }
    bool isOperational() const { return statusword & StatuswordBits::OperationalMode; }
    bool hasConfigMismatch() const { return statusword & StatuswordBits::ConfigMismatch; }
    bool hasModuleError() const { return statusword & StatuswordBits::ModuleError; }
    bool hasDiagAvailable() const { return statusword & StatuswordBits::DiagAvailable; }
    bool hasHotSwapEvent() const { return statusword & StatuswordBits::HotSwapEvent; }
    bool hasWarning() const { return statusword & StatuswordBits::Warning; }
    bool hasError() const { return statusword & StatuswordBits::Error; }
};

// ============================================================================
// Callback Types
// ============================================================================

using ModuleEventCallback = std::function<void(uint8_t slot, uint8_t event_type)>;
using ConfigChangeCallback = std::function<void(const DeviceConfigState& config)>;
using DiagnosticCallback = std::function<void(uint8_t slot, uint16_t diag_status)>;
using ErrorCallback = std::function<void(uint8_t slot, uint16_t error_code)>;

// Module event types
namespace ModuleEvent {
    constexpr uint8_t Inserted = 0x01;
    constexpr uint8_t Removed = 0x02;
    constexpr uint8_t Operational = 0x03;
    constexpr uint8_t Error = 0x04;
    constexpr uint8_t ConfigChanged = 0x05;
    constexpr uint8_t DiagUpdated = 0x06;
}

// ============================================================================
// Modular Device Controller Class
// ============================================================================

class ModularDevice {
public:
    explicit ModularDevice(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr = false);
    ~ModularDevice();
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    bool initialize();
    bool isInitialized() const { return initialized_; }
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set expected slot configuration
     * @param descriptors Vector of expected module descriptors
     */
    bool setExpectedConfiguration(const std::vector<ModuleDescriptor>& descriptors);
    
    /**
     * @brief Enable auto-configuration mode
     */
    bool enableAutoConfiguration(bool enable);
    
    /**
     * @brief Validate current configuration against expected
     */
    bool validateConfiguration();
    
    /**
     * @brief Accept current configuration as valid
     */
    bool acceptConfiguration();
    
    /**
     * @brief Save configuration to EEPROM
     */
    bool saveConfiguration();
    
    /**
     * @brief Get configuration state
     */
    const DeviceConfigState& getConfigState() const { return state_.config; }
    
    // ========================================================================
    // Module Enumeration
    // ========================================================================
    
    /**
     * @brief Scan for connected modules
     */
    bool scanModules();
    
    /**
     * @brief Get number of detected modules
     */
    uint8_t getModuleCount() const { return state_.config.detected_modules; }
    
    /**
     * @brief Get module information by slot
     */
    bool getModuleInfo(uint8_t slot, SlotInfo& info);
    
    /**
     * @brief Get module state by slot
     */
    const ModuleState* getModuleState(uint8_t slot) const;
    
    /**
     * @brief Get all module states
     */
    const std::vector<ModuleState>& getAllModules() const { return state_.modules; }
    
    /**
     * @brief Check if module is present in slot
     */
    bool isModulePresent(uint8_t slot) const;
    
    /**
     * @brief Check if module is operational
     */
    bool isModuleOperational(uint8_t slot) const;
    
    // ========================================================================
    // Module Control
    // ========================================================================
    
    /**
     * @brief Enable a specific module
     */
    bool enableModule(uint8_t slot, bool enable);
    
    /**
     * @brief Enable all modules
     */
    bool enableAllModules(bool enable);
    
    /**
     * @brief Reset module errors
     */
    bool resetModuleErrors(uint8_t slot);
    
    /**
     * @brief Reset all module errors
     */
    bool resetAllErrors();
    
    /**
     * @brief Reset module to factory defaults
     */
    bool resetModuleToDefaults(uint8_t slot);
    
    // ========================================================================
    // Process Data Access
    // ========================================================================
    
    /**
     * @brief Get module input data offset in process image
     */
    uint16_t getModuleInputOffset(uint8_t slot) const;
    
    /**
     * @brief Get module output data offset in process image
     */
    uint16_t getModuleOutputOffset(uint8_t slot) const;
    
    /**
     * @brief Get module input data size (bits)
     */
    uint16_t getModuleInputSize(uint8_t slot) const;
    
    /**
     * @brief Get module output data size (bits)
     */
    uint16_t getModuleOutputSize(uint8_t slot) const;
    
    /**
     * @brief Read module input data
     */
    bool readModuleInput(uint8_t slot, void* data, size_t len);
    
    /**
     * @brief Write module output data
     */
    bool writeModuleOutput(uint8_t slot, const void* data, size_t len);
    
    // ========================================================================
    // Cyclic Update
    // ========================================================================
    
    void processTxPDO(const uint8_t* data, size_t len);
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    void update();
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    /**
     * @brief Get module diagnostics
     */
    uint16_t getModuleDiagStatus(uint8_t slot) const;
    
    /**
     * @brief Get module last error code
     */
    uint16_t getModuleLastError(uint8_t slot) const;
    
    /**
     * @brief Get module temperature
     */
    float getModuleTemperature(uint8_t slot) const;
    
    /**
     * @brief Get module supply voltage
     */
    float getModuleSupplyVoltage(uint8_t slot) const;
    
    /**
     * @brief Get module error count
     */
    uint32_t getModuleErrorCount(uint8_t slot) const;
    
    /**
     * @brief Get module operating hours
     */
    uint32_t getModuleOperatingHours(uint8_t slot) const;
    
    /**
     * @brief Get device diagnostics string
     */
    std::string getDiagnostics() const;
    
    // ========================================================================
    // State Access
    // ========================================================================
    
    const DeviceState& getState() const { return state_; }
    uint16_t getStatusword() const { return state_.statusword; }
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setModuleEventCallback(ModuleEventCallback callback);
    void setConfigChangeCallback(ConfigChangeCallback callback);
    void setDiagnosticCallback(DiagnosticCallback callback);
    void setErrorCallback(ErrorCallback callback);

private:
    bool readModuleList();
    void updateModuleStates();
    void checkConfigurationState();
    void checkHotSwapEvents();
    void notifyModuleEvent(uint8_t slot, uint8_t event_type);
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len) const;
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len) const;
    
    EtherCAT::SDO::SDOManager& m_sdo;
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    DeviceState state_;
    uint16_t prev_statusword_;
    std::vector<uint8_t> prev_module_status_;
    
    uint16_t controlword_;
    std::vector<ModuleDescriptor> expected_config_;
    bool auto_config_enabled_;
    
    // Process data offsets (calculated during scan)
    std::vector<uint16_t> input_offsets_;
    std::vector<uint16_t> output_offsets_;
    std::vector<uint16_t> input_sizes_;
    std::vector<uint16_t> output_sizes_;
    
    ModuleEventCallback module_event_callback_;
    ConfigChangeCallback config_change_callback_;
    DiagnosticCallback diagnostic_callback_;
    ErrorCallback error_callback_;
};

} // namespace ETG5000
