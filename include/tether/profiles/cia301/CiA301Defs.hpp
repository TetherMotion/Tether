/**
 * @file CiA301Defs.hpp
 * @brief CiA 301 CANopen Object Dictionary Definitions
 * 
 * @details
 * This file contains all the CiA 301 predefined object dictionary indexes and
 * subindexes according to the CANopen DS-301 specification. These definitions
 * are used for EtherCAT CoE (CANopen over EtherCAT) communication.
 * 
 * ## Object Dictionary Structure
 * 
 * The CANopen object dictionary is organized into several areas:
 * 
 * | Range         | Description                                    |
 * |---------------|------------------------------------------------|
 * | 0x0000        | Reserved (not used)                            |
 * | 0x0001-0x025F | Data type definitions                          |
 * | 0x0260-0x0FFF | Reserved                                       |
 * | 0x1000-0x1FFF | Communication profile area (CiA 301)           |
 * | 0x2000-0x5FFF | Manufacturer specific                          |
 * | 0x6000-0x9FFF | Standardized device profile (CiA 402, etc.)    |
 * | 0xA000-0xAFFF | Interface profile                              |
 * | 0xB000-0xFFFF | Reserved / System                              |
 * 
 * @see CiA 301 DS-301 CANopen application layer specification
 * @see ETG.1000.6 EtherCAT CoE protocol
 */

#pragma once

#include <cstdint>

#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/PDO.hpp"
#include "tether/ethercat/SDO.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"

namespace CiA301 {

// ============================================================================
// Object Dictionary Index Ranges
// ============================================================================

namespace IndexRange {
    constexpr uint16_t DataTypesStart       = 0x0001;
    constexpr uint16_t DataTypesEnd         = 0x025F;
    constexpr uint16_t CommProfileStart     = 0x1000;
    constexpr uint16_t CommProfileEnd       = 0x1FFF;
    constexpr uint16_t ManufacturerStart    = 0x2000;
    constexpr uint16_t ManufacturerEnd      = 0x5FFF;
    constexpr uint16_t DeviceProfileStart   = 0x6000;
    constexpr uint16_t DeviceProfileEnd     = 0x9FFF;
    constexpr uint16_t InterfaceProfileStart = 0xA000;
    constexpr uint16_t InterfaceProfileEnd  = 0xAFFF;
}

// ============================================================================
// Data Types (0x0001 - 0x025F)
//
// Canonical data types are defined once in
// `EtherCAT::ObjectDictionary::ObjectDictionaryDataType` — expose the enum
// here so callers can continue to refer to `CiA301::DataType` as the type.
// The numeric CiA301 constants were removed to avoid duplication.
// ============================================================================

using DataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

// ============================================================================
// Communication Profile Area (0x1000 - 0x1FFF)
// ============================================================================

/**
 * @brief Device Type Object (0x1000)
 * 
 * Contains the device profile number and additional info:
 * - Bits 0-15: Device profile number (e.g., 402 for drives)
 * - Bits 16-31: Additional information
 */
constexpr uint16_t DeviceType               = 0x1000;

/**
 * @brief Error Register (0x1001)
 * 
 * Indicates the error state of the device:
 * - Bit 0: Generic error
 * - Bit 1: Current error
 * - Bit 2: Voltage error
 * - Bit 3: Temperature error
 * - Bit 4: Communication error
 * - Bit 5: Device profile specific
 * - Bit 6: Reserved
 * - Bit 7: Manufacturer specific
 */
constexpr uint16_t ErrorRegister            = 0x1001;

/**
 * @brief Manufacturer Status Register (0x1002)
 * 
 * Manufacturer-specific status information.
 */
constexpr uint16_t ManufacturerStatusRegister = 0x1002;

/**
 * @brief Pre-defined Error Field (0x1003)
 * 
 * History of errors (array):
 * - Subindex 0: Number of errors
 * - Subindex 1-n: Error codes (newest first)
 */
constexpr uint16_t PreDefinedErrorField     = 0x1003;

namespace PreDefinedErrorFieldSub {
    constexpr uint8_t NumberOfErrors        = 0x00;
    constexpr uint8_t Error1                = 0x01;
    // ... up to implementation limit
}

/**
 * @brief COB-ID SYNC (0x1005)
 * 
 * COB-ID of the synchronization object.
 */
constexpr uint16_t CobIdSync                = 0x1005;

/**
 * @brief Communication Cycle Period (0x1006)
 * 
 * Cycle time for synchronous communication in microseconds.
 */
constexpr uint16_t CommunicationCyclePeriod = 0x1006;

/**
 * @brief Synchronous Window Length (0x1007)
 * 
 * Time window for synchronous PDO transmission in microseconds.
 */
constexpr uint16_t SynchronousWindowLength  = 0x1007;

/**
 * @brief Manufacturer Device Name (0x1008)
 * 
 * Human-readable device name string.
 */
constexpr uint16_t ManufacturerDeviceName   = 0x1008;

/**
 * @brief Manufacturer Hardware Version (0x1009)
 * 
 * Hardware version string.
 */
constexpr uint16_t ManufacturerHWVersion    = 0x1009;

/**
 * @brief Manufacturer Software Version (0x100A)
 * 
 * Software/firmware version string.
 */
constexpr uint16_t ManufacturerSWVersion    = 0x100A;

// 0x100B reserved

/**
 * @brief Guard Time (0x100C)
 * 
 * Guard time for NMT node guarding (not typically used in EtherCAT).
 */
constexpr uint16_t GuardTime                = 0x100C;

/**
 * @brief Life Time Factor (0x100D)
 * 
 * Life time factor for NMT node guarding.
 */
constexpr uint16_t LifeTimeFactor           = 0x100D;

// 0x100E-0x100F reserved

/**
 * @brief Store Parameters (0x1010)
 * 
 * Command to store parameters to non-volatile memory:
 * - Subindex 1: Save all parameters
 * - Subindex 2: Save communication parameters
 * - Subindex 3: Save application parameters
 * - Subindex 4-127: Manufacturer specific
 * 
 * Write 0x65766173 ("save") to execute.
 */
constexpr uint16_t StoreParameters          = 0x1010;

namespace StoreParametersSub {
    constexpr uint8_t SaveAllParameters     = 0x01;
    constexpr uint8_t SaveCommParameters    = 0x02;
    constexpr uint8_t SaveAppParameters     = 0x03;
}

/**
 * @brief Restore Default Parameters (0x1011)
 * 
 * Command to restore default parameters:
 * - Subindex 1: Restore all defaults
 * - Subindex 2: Restore communication defaults
 * - Subindex 3: Restore application defaults
 * 
 * Write 0x64616F6C ("load") to execute.
 */
constexpr uint16_t RestoreDefaultParameters = 0x1011;

namespace RestoreDefaultParametersSub {
    constexpr uint8_t RestoreAllDefaults    = 0x01;
    constexpr uint8_t RestoreCommDefaults   = 0x02;
    constexpr uint8_t RestoreAppDefaults    = 0x03;
}

/**
 * @brief COB-ID Time Stamp (0x1012)
 */
constexpr uint16_t CobIdTimeStamp           = 0x1012;

/**
 * @brief High Resolution Time Stamp (0x1013)
 * 
 * Current time in microseconds.
 */
constexpr uint16_t HighResolutionTimeStamp  = 0x1013;

/**
 * @brief COB-ID Emergency (0x1014)
 */
constexpr uint16_t CobIdEmergency           = 0x1014;

/**
 * @brief Inhibit Time Emergency (0x1015)
 * 
 * Minimum time between emergency messages (100 µs units).
 */
constexpr uint16_t InhibitTimeEmergency     = 0x1015;

/**
 * @brief Consumer Heartbeat Time (0x1016)
 * 
 * Array of consumer heartbeat entries:
 * - Subindex 0: Number of entries
 * - Subindex n: Node-ID (bits 16-23) + time ms (bits 0-15)
 */
constexpr uint16_t ConsumerHeartbeatTime    = 0x1016;

/**
 * @brief Producer Heartbeat Time (0x1017)
 * 
 * Heartbeat producer time in milliseconds.
 */
constexpr uint16_t ProducerHeartbeatTime    = 0x1017;

/**
 * @brief Identity Object (0x1018)
 * 
 * Device identification record:
 * - Subindex 0: Number of entries (4)
 * - Subindex 1: Vendor ID
 * - Subindex 2: Product Code
 * - Subindex 3: Revision Number
 * - Subindex 4: Serial Number
 */
constexpr uint16_t Identity                 = 0x1018;

namespace IdentitySub {
    constexpr uint8_t NumberOfEntries       = 0x00;
    constexpr uint8_t VendorID              = 0x01;
    constexpr uint8_t ProductCode           = 0x02;
    constexpr uint8_t RevisionNumber        = 0x03;
    constexpr uint8_t SerialNumber          = 0x04;
}

/**
 * @brief Verify Configuration (0x1020)
 * 
 * Subindex 1: Configuration date (days since 1984-01-01)
 * Subindex 2: Configuration time (ms since midnight)
 */
constexpr uint16_t VerifyConfiguration      = 0x1020;

namespace VerifyConfigurationSub {
    constexpr uint8_t ConfigurationDate     = 0x01;
    constexpr uint8_t ConfigurationTime     = 0x02;
}

/**
 * @brief Emergency Consumer Object (0x1028)
 * 
 * Array of COB-IDs for emergency consumption.
 */
constexpr uint16_t EmergencyConsumerObject  = 0x1028;

/**
 * @brief Error Behavior Object (0x1029)
 * 
 * Behavior on communication errors:
 * - Subindex 1: Communication error
 * - Subindex 2-254: Profile/manufacturer specific
 * 
 * Values: 0=Pre-Op, 1=No change, 2=Stop
 */
constexpr uint16_t ErrorBehavior            = 0x1029;

namespace ErrorBehaviorSub {
    constexpr uint8_t CommunicationError    = 0x01;
}

// SDO-related CiA301 definitions moved to `tether/ethercat/SDO.hpp`.
// See `tether/ethercat/SDO.hpp` for the EtherCAT::SDO constants and
// backward-compatible `CiA301` aliases.
// PDO-related CiA301 definitions moved to `tether/ethercat/PDO.hpp`.  
// See `tether/ethercat/PDO.hpp` for the EtherCAT::PDO constants and
// backward-compatible `CiA301` aliases.

// Sync Manager related CiA301 definitions moved to `tether/ethercat/SyncManager.hpp`.


// ============================================================================
// Emergency Error Codes
// ============================================================================

namespace EmergencyErrorCode {
    // Error reset / No error
    constexpr uint16_t NoError                  = 0x0000;
    
    // Generic errors
    constexpr uint16_t GenericError             = 0x1000;
    
    // Current errors
    constexpr uint16_t Current                  = 0x2000;
    constexpr uint16_t CurrentInputSide         = 0x2100;
    constexpr uint16_t CurrentInsideDevice      = 0x2200;
    constexpr uint16_t CurrentOutputSide        = 0x2300;
    
    // Voltage errors
    constexpr uint16_t Voltage                  = 0x3000;
    constexpr uint16_t MainsVoltage             = 0x3100;
    constexpr uint16_t VoltageInsideDevice      = 0x3200;
    constexpr uint16_t OutputVoltage            = 0x3300;
    
    // Temperature errors
    constexpr uint16_t Temperature              = 0x4000;
    constexpr uint16_t AmbientTemperature       = 0x4100;
    constexpr uint16_t DeviceTemperature        = 0x4200;
    
    // Device hardware errors
    constexpr uint16_t DeviceHardware           = 0x5000;
    
    // Device software errors
    constexpr uint16_t DeviceSoftware           = 0x6000;
    constexpr uint16_t InternalSoftware         = 0x6100;
    constexpr uint16_t UserSoftware             = 0x6200;
    constexpr uint16_t DataSet                  = 0x6300;
    
    // Additional modules
    constexpr uint16_t AdditionalModules        = 0x7000;
    
    // Monitoring errors
    constexpr uint16_t Monitoring               = 0x8000;
    constexpr uint16_t Communication            = 0x8100;
    constexpr uint16_t ProtocolError            = 0x8200;
    
    // External error
    constexpr uint16_t ExternalError            = 0x9000;
    
    // Additional functions
    constexpr uint16_t AdditionalFunctions      = 0xF000;
    
    // Device specific
    constexpr uint16_t DeviceSpecific           = 0xFF00;
}

// ============================================================================
// Save/Load Magic Values
// ============================================================================

namespace StorageCommand {
    constexpr uint32_t Save                     = 0x65766173; // "save" (ASCII, little-endian)
    constexpr uint32_t Load                     = 0x64616F6C; // "load" (ASCII, little-endian)
}

// ============================================================================
// Transmission Types
// ============================================================================

namespace TransmissionType {
    constexpr uint8_t Synchronous               = 0x00;  // Sync with SYNC object
    constexpr uint8_t Sync1                     = 0x01;  // After every SYNC
    constexpr uint8_t Sync240                   = 0xF0;  // After 240th SYNC
    constexpr uint8_t AsyncRTR                  = 0xFC;  // Async, on RTR
    constexpr uint8_t AsyncManufacturer         = 0xFD;  // Manufacturer specific
    constexpr uint8_t AsyncDeviceProfile        = 0xFE;  // Device profile event
    constexpr uint8_t AsyncDeviceEvent          = 0xFF;  // Device event (application)
}

} // namespace CiA301
