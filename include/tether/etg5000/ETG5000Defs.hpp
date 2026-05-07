/**
 * @file ETG5000Defs.hpp
 * @brief ETG.5000.1 Modular Device Profile Object Dictionary
 *
 * Defines the object dictionary for ETG.5000.1 compliant modular devices
 * as specified in the EtherCAT Technology Group specification.
 *
 * Object Index Ranges:
 * - 0xF000-0xF00F: Module Configuration
 * - 0xF010-0xF01F: Module Identity
 * - 0xF020-0xF02F: Module Diagnostics
 * - 0xF030-0xF03F: Module Process Data
 * - 0xF050-0xF05F: Module Parameters
 */

#pragma once

#include <cstdint>

namespace ETG5000 {

// ============================================================================
// Module Types
// ============================================================================

namespace ModuleType {
    constexpr uint16_t Unknown           = 0x0000;
    constexpr uint16_t DigitalInput      = 0x0001;
    constexpr uint16_t DigitalOutput     = 0x0002;
    constexpr uint16_t DigitalIO         = 0x0003;
    constexpr uint16_t AnalogInput       = 0x0010;
    constexpr uint16_t AnalogOutput      = 0x0011;
    constexpr uint16_t AnalogIO          = 0x0012;
    constexpr uint16_t Encoder           = 0x0020;
    constexpr uint16_t MotorController   = 0x0021;
    constexpr uint16_t ServoAmplifier    = 0x0022;
    constexpr uint16_t Stepper           = 0x0023;
    constexpr uint16_t Communication     = 0x0030;
    constexpr uint16_t Gateway           = 0x0031;
    constexpr uint16_t Counter           = 0x0040;
    constexpr uint16_t Timer             = 0x0041;
    constexpr uint16_t Temperature       = 0x0050;
    constexpr uint16_t Pressure          = 0x0051;
    constexpr uint16_t Safety            = 0x0060;
    constexpr uint16_t Power             = 0x0070;
    constexpr uint16_t Custom            = 0x00FF;
}

// ============================================================================
// Module Status
// ============================================================================

namespace ModuleStatus {
    constexpr uint8_t NotPresent         = 0x00;
    constexpr uint8_t Present            = 0x01;
    constexpr uint8_t Operational        = 0x02;
    constexpr uint8_t Error              = 0x03;
    constexpr uint8_t Disabled           = 0x04;
    constexpr uint8_t ConfigurationError = 0x05;
    constexpr uint8_t PowerError         = 0x06;
    constexpr uint8_t CommunicationError = 0x07;
}

// ============================================================================
// Module Configuration (0xF000-0xF00F)
// ============================================================================

namespace ModuleConfig {
    constexpr uint16_t DetectedModuleCount   = 0xF000;  // Number of detected modules
    constexpr uint16_t ConfiguredModuleCount = 0xF001;  // Number of configured slots
    constexpr uint16_t ModuleIdentList       = 0xF002;  // Array of module identifiers
    constexpr uint16_t ModuleTypeList        = 0xF003;  // Array of module types
    constexpr uint16_t ModuleStatusList      = 0xF004;  // Array of module status
    constexpr uint16_t SlotConfiguration     = 0xF005;  // Slot configuration data
    constexpr uint16_t AutoConfiguration     = 0xF006;  // Enable auto-configuration
    constexpr uint16_t ConfigurationState    = 0xF007;  // Overall config state
    constexpr uint16_t ModuleAddressList     = 0xF008;  // Module addresses
    constexpr uint16_t PluggedModuleTypes    = 0xF009;  // Types actually plugged
}

// Configuration state values
namespace ConfigState {
    constexpr uint8_t Unconfigured       = 0x00;
    constexpr uint8_t ConfigurationValid = 0x01;
    constexpr uint8_t ConfigMismatch     = 0x02;
    constexpr uint8_t ModuleMissing      = 0x03;
    constexpr uint8_t ExtraModules       = 0x04;
    constexpr uint8_t ConfigError        = 0xFF;
}

// ============================================================================
// Module Identity (0xF010-0xF01F)
// ============================================================================

namespace ModuleIdent {
    constexpr uint16_t VendorID          = 0xF010;  // Vendor ID per slot
    constexpr uint16_t ProductCode       = 0xF011;  // Product code per slot
    constexpr uint16_t RevisionNumber    = 0xF012;  // Revision number per slot
    constexpr uint16_t SerialNumber      = 0xF013;  // Serial number per slot
    constexpr uint16_t ModuleName        = 0xF014;  // Module name string per slot
    constexpr uint16_t HardwareVersion   = 0xF015;  // Hardware version per slot
    constexpr uint16_t FirmwareVersion   = 0xF016;  // Firmware version per slot
    constexpr uint16_t Capabilities      = 0xF017;  // Module capabilities bitfield
}

// Module capability bits
namespace ModuleCaps {
    constexpr uint32_t PDOInput          = 0x00000001;  // Has input PDO data
    constexpr uint32_t PDOOutput         = 0x00000002;  // Has output PDO data
    constexpr uint32_t SDOAccess         = 0x00000004;  // Supports SDO access
    constexpr uint32_t HotSwap           = 0x00000008;  // Supports hot swap
    constexpr uint32_t Diagnostics       = 0x00000010;  // Has diagnostic data
    constexpr uint32_t Parameters        = 0x00000020;  // Has configurable params
    constexpr uint32_t Timestamps        = 0x00000040;  // Provides timestamps
    constexpr uint32_t Calibration       = 0x00000080;  // Has calibration data
    constexpr uint32_t Safety            = 0x00000100;  // Safety-related module
    constexpr uint32_t DistributedClock  = 0x00000200;  // DC sync capable
}

// ============================================================================
// Module Diagnostics (0xF020-0xF02F)
// ============================================================================

namespace ModuleDiag {
    constexpr uint16_t DiagnosticStatus  = 0xF020;  // Overall diagnostic status
    constexpr uint16_t ErrorCount        = 0xF021;  // Error counter per slot
    constexpr uint16_t LastError         = 0xF022;  // Last error code per slot
    constexpr uint16_t WarningStatus     = 0xF023;  // Warning bits per slot
    constexpr uint16_t Temperature       = 0xF024;  // Module temperature per slot
    constexpr uint16_t SupplyVoltage     = 0xF025;  // Supply voltage per slot
    constexpr uint16_t CycleCount        = 0xF026;  // Cycle counter per slot
    constexpr uint16_t OperatingHours    = 0xF027;  // Operating hours per slot
    constexpr uint16_t QualityIndicator  = 0xF028;  // Signal quality indicator
    constexpr uint16_t DiagHistory       = 0xF029;  // Diagnostic history
    constexpr uint16_t LifetimeData      = 0xF02A;  // Lifetime statistics
}

// Diagnostic status bits
namespace DiagStatus {
    constexpr uint16_t ModuleOK          = 0x0001;
    constexpr uint16_t Warning           = 0x0002;
    constexpr uint16_t Error             = 0x0004;
    constexpr uint16_t InternalError     = 0x0008;
    constexpr uint16_t ExternalError     = 0x0010;
    constexpr uint16_t OverTemperature   = 0x0020;
    constexpr uint16_t UnderVoltage      = 0x0040;
    constexpr uint16_t OverVoltage       = 0x0080;
    constexpr uint16_t WireBreak         = 0x0100;
    constexpr uint16_t ShortCircuit      = 0x0200;
    constexpr uint16_t Overload          = 0x0400;
    constexpr uint16_t CalibrationError  = 0x0800;
    constexpr uint16_t ConfigError       = 0x1000;
    constexpr uint16_t CommError         = 0x2000;
}

// ============================================================================
// Module Process Data (0xF030-0xF03F)
// ============================================================================

namespace ModulePDO {
    constexpr uint16_t InputPDOAssign    = 0xF030;  // Input PDO assignment per slot
    constexpr uint16_t OutputPDOAssign   = 0xF031;  // Output PDO assignment per slot
    constexpr uint16_t InputPDOMapping   = 0xF032;  // Input PDO mapping per slot
    constexpr uint16_t OutputPDOMapping  = 0xF033;  // Output PDO mapping per slot
    constexpr uint16_t PDOSize           = 0xF034;  // PDO size info per slot
    constexpr uint16_t InputOffset       = 0xF035;  // Input data offset per slot
    constexpr uint16_t OutputOffset      = 0xF036;  // Output data offset per slot
    constexpr uint16_t SyncManager       = 0xF037;  // SM assignment per slot
    constexpr uint16_t FMMU              = 0xF038;  // FMMU assignment per slot
}

// ============================================================================
// Module Parameters (0xF050-0xF05F)
// ============================================================================

namespace ModuleParam {
    constexpr uint16_t ParameterData     = 0xF050;  // Module-specific parameters
    constexpr uint16_t ParameterLock     = 0xF051;  // Parameter lock per slot
    constexpr uint16_t FactoryDefaults   = 0xF052;  // Restore factory defaults
    constexpr uint16_t SaveParameters    = 0xF053;  // Save params to EEPROM
    constexpr uint16_t LoadParameters    = 0xF054;  // Load params from EEPROM
    constexpr uint16_t FilterSettings    = 0xF055;  // Input filter settings
    constexpr uint16_t ScalingFactor     = 0xF056;  // Scaling factors
    constexpr uint16_t OffsetValue       = 0xF057;  // Offset values
    constexpr uint16_t RangeSettings     = 0xF058;  // Range configuration
}

// ============================================================================
// Slot Information Structure
// ============================================================================

struct SlotInfo {
    uint8_t  slot_number;
    uint16_t module_type;
    uint8_t  status;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
    uint32_t capabilities;
    uint16_t input_size;      // Input PDO size in bits
    uint16_t output_size;     // Output PDO size in bits
    uint16_t input_offset;    // Byte offset in process image
    uint16_t output_offset;
    uint16_t diag_status;
    int16_t  temperature;     // 0.1°C
    uint16_t supply_voltage;  // 0.1V
    uint32_t error_count;
    uint32_t operating_hours;
};

// Maximum number of slots
constexpr uint8_t MAX_SLOTS = 64;

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

/**
 * @brief Basic modular device PDO (input)
 */
struct ModularInputPDO {
    uint16_t statusword;
    uint8_t  module_count;
    uint8_t  config_state;
    uint16_t diag_status_bitmap[4];  // Up to 64 modules, 1 bit each
    // Followed by module-specific data
};

/**
 * @brief Basic modular device PDO (output)
 */
struct ModularOutputPDO {
    uint16_t controlword;
    uint8_t  reserved[2];
    // Followed by module-specific data
};

/**
 * @brief Module status summary
 */
struct ModuleStatusSummary {
    uint8_t slot;
    uint8_t status;
    uint16_t diag_status;
    uint16_t last_error;
};

/**
 * @brief Module identification record
 */
struct ModuleIdentRecord {
    uint8_t  slot;
    uint16_t module_type;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
};

#pragma pack(pop)

// ============================================================================
// Controlword Bits
// ============================================================================

namespace ControlwordBits {
    constexpr uint16_t EnableAll         = 0x0001;  // Enable all modules
    constexpr uint16_t ResetErrors       = 0x0002;  // Reset error states
    constexpr uint16_t StartConfiguration= 0x0004;  // Start configuration
    constexpr uint16_t AcceptConfig      = 0x0008;  // Accept current config
    constexpr uint16_t SaveConfig        = 0x0010;  // Save configuration
    constexpr uint16_t ResetToDefaults   = 0x0020;  // Reset to defaults
    constexpr uint16_t EnableDiagnostics = 0x0040;  // Enable diagnostics
    constexpr uint16_t ForceUpdate       = 0x0080;  // Force PDO update
}

// ============================================================================
// Statusword Bits
// ============================================================================

namespace StatuswordBits {
    constexpr uint16_t Ready             = 0x0001;
    constexpr uint16_t AllModulesOK      = 0x0002;
    constexpr uint16_t ConfigValid       = 0x0004;
    constexpr uint16_t ConfigMismatch    = 0x0008;
    constexpr uint16_t ModuleError       = 0x0010;
    constexpr uint16_t DiagAvailable     = 0x0020;
    constexpr uint16_t HotSwapEvent      = 0x0040;
    constexpr uint16_t PowerOK           = 0x0080;
    constexpr uint16_t ConfigurationMode = 0x0100;
    constexpr uint16_t OperationalMode   = 0x0200;
    constexpr uint16_t SafeState         = 0x0400;
    constexpr uint16_t Warning           = 0x0800;
    constexpr uint16_t Error             = 0x1000;
    constexpr uint16_t Fault             = 0x2000;
}

// ============================================================================
// Error Codes
// ============================================================================

namespace ErrorCode {
    constexpr uint16_t NoError           = 0x0000;
    constexpr uint16_t ModuleNotFound    = 0x0001;
    constexpr uint16_t ModuleTypeMismatch= 0x0002;
    constexpr uint16_t ConfigurationError= 0x0003;
    constexpr uint16_t CommunicationError= 0x0004;
    constexpr uint16_t HardwareError     = 0x0005;
    constexpr uint16_t OverTemperature   = 0x0006;
    constexpr uint16_t UnderVoltage      = 0x0007;
    constexpr uint16_t OverVoltage       = 0x0008;
    constexpr uint16_t InternalError     = 0x0009;
    constexpr uint16_t CalibrationError  = 0x000A;
    constexpr uint16_t WireBreak         = 0x000B;
    constexpr uint16_t ShortCircuit      = 0x000C;
    constexpr uint16_t ModuleOverload    = 0x000D;
    constexpr uint16_t UnsupportedModule = 0x000E;
    constexpr uint16_t ParameterError    = 0x000F;
}

} // namespace ETG5000
