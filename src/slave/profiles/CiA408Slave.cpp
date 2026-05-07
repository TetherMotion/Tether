/**
 * @file CiA408Slave.cpp
 * @brief CiA 408 Fluid Power Technology (Hydraulic Valves) Slave Implementation
 *
 * @details
 * This implementation provides a complete CiA 408 compliant hydraulic valve
 * slave with:
 * - Valve command input and position feedback
 * - Pressure sensing (port A and port B)
 * - Flow calculation
 * - Deadband compensation
 * - Dither for improved valve response
 * - Fault handling and diagnostics
 * - PDO mappings for real-time control
 *
 * CiA 408 Object Index Ranges:
 * - 0x6000-0x60FF: Operating parameters
 * - 0x6100-0x61FF: Device parameters
 * - 0x6200-0x62FF: Command and feedback
 * - 0x6300-0x63FF: Pressure and flow
 * - 0x6400-0x64FF: Configuration
 * - 0x6500-0x65FF: Status and diagnostics
 */

#include "slave/profiles/CiA408Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 408 Object Dictionary Indices
// ============================================================================

namespace CiA408Index {
    // Operating Parameters (0x6000-0x60FF)
    constexpr uint16_t ValveType                = 0x6000;  // Valve type
    constexpr uint16_t NominalCurrent           = 0x6001;  // Nominal current (mA)
    constexpr uint16_t MaxCurrent               = 0x6002;  // Maximum current (mA)
    constexpr uint16_t ResponseTime             = 0x6003;  // Response time (ms)
    
    // Device Parameters (0x6100-0x61FF)
    constexpr uint16_t DeadbandCompensation     = 0x6100;  // Deadband compensation value
    constexpr uint16_t GainPositive             = 0x6101;  // Positive direction gain
    constexpr uint16_t GainNegative             = 0x6102;  // Negative direction gain
    constexpr uint16_t RampUp                   = 0x6103;  // Ramp-up rate
    constexpr uint16_t RampDown                 = 0x6104;  // Ramp-down rate
    constexpr uint16_t DitherFrequency          = 0x6110;  // Dither frequency (Hz)
    constexpr uint16_t DitherAmplitude          = 0x6111;  // Dither amplitude (0.01%)
    constexpr uint16_t DitherEnable             = 0x6112;  // Dither enable
    
    // Command and Feedback (0x6200-0x62FF)
    constexpr uint16_t CommandValue             = 0x6200;  // Command input (-10000 to +10000)
    constexpr uint16_t ActualPosition           = 0x6201;  // Actual spool position
    constexpr uint16_t TargetPosition           = 0x6202;  // Target position
    constexpr uint16_t PositionError            = 0x6203;  // Position error
    constexpr uint16_t CommandFiltered          = 0x6204;  // Filtered command
    constexpr uint16_t OutputCurrent            = 0x6210;  // Output current (mA)
    
    // Pressure and Flow (0x6300-0x63FF)
    constexpr uint16_t PressurePortA            = 0x6300;  // Pressure port A (mbar)
    constexpr uint16_t PressurePortB            = 0x6301;  // Pressure port B (mbar)
    constexpr uint16_t PressureSupply           = 0x6302;  // Supply pressure (mbar)
    constexpr uint16_t PressureTank             = 0x6303;  // Tank pressure (mbar)
    constexpr uint16_t DeltaPressure            = 0x6304;  // Differential pressure
    constexpr uint16_t FlowRate                 = 0x6310;  // Flow rate (0.01 L/min)
    constexpr uint16_t MaxFlowRate              = 0x6311;  // Maximum flow rate
    
    // Configuration (0x6400-0x64FF)
    constexpr uint16_t CommandMin               = 0x6400;  // Minimum command value
    constexpr uint16_t CommandMax               = 0x6401;  // Maximum command value
    constexpr uint16_t NeutralPosition          = 0x6402;  // Neutral position
    constexpr uint16_t PositionMin              = 0x6410;  // Minimum position
    constexpr uint16_t PositionMax              = 0x6411;  // Maximum position
    constexpr uint16_t PressureMin              = 0x6420;  // Minimum pressure
    constexpr uint16_t PressureMax              = 0x6421;  // Maximum pressure
    
    // Status and Diagnostics (0x6500-0x65FF)
    constexpr uint16_t StatusWord               = 0x6500;  // Status word
    constexpr uint16_t ControlWord              = 0x6501;  // Control word
    constexpr uint16_t FaultCode                = 0x6502;  // Fault code
    constexpr uint16_t WarningCode              = 0x6503;  // Warning code
    constexpr uint16_t OperatingHours           = 0x6510;  // Operating hours
    constexpr uint16_t CycleCount               = 0x6511;  // Cycle count
    constexpr uint16_t Temperature              = 0x6520;  // Internal temperature
}

// ============================================================================
// Status Bit Definitions
// ============================================================================

namespace CiA408Status {
    // Status word bits (0x6500)
    constexpr uint16_t Ready                    = 0x0001;  // Valve ready
    constexpr uint16_t Enabled                  = 0x0002;  // Valve enabled
    constexpr uint16_t Fault                    = 0x0008;  // Fault active
    constexpr uint16_t Warning                  = 0x0010;  // Warning active
    constexpr uint16_t TargetReached            = 0x0020;  // Target position reached
    constexpr uint16_t PositionLimited          = 0x0040;  // Position limited
    constexpr uint16_t CurrentLimited           = 0x0080;  // Current limited
    constexpr uint16_t InDeadband               = 0x0100;  // In deadband zone
    constexpr uint16_t DitherActive             = 0x0200;  // Dither is active
    
    // Control word bits (0x6501)
    constexpr uint16_t CtrlEnable               = 0x0001;  // Enable valve
    constexpr uint16_t CtrlQuickStop            = 0x0002;  // Quick stop (center valve)
    constexpr uint16_t CtrlFaultReset           = 0x0004;  // Reset faults
    constexpr uint16_t CtrlEnableDither         = 0x0008;  // Enable dither
    constexpr uint16_t CtrlForceNeutral         = 0x0010;  // Force neutral position
    
    // Fault codes (0x6502)
    constexpr uint16_t FaultOvercurrent         = 0x0001;  // Overcurrent
    constexpr uint16_t FaultUndervoltage        = 0x0002;  // Undervoltage
    constexpr uint16_t FaultOvervoltage         = 0x0004;  // Overvoltage
    constexpr uint16_t FaultOvertemperature     = 0x0008;  // Overtemperature
    constexpr uint16_t FaultWireBreak           = 0x0010;  // Wire break detected
    constexpr uint16_t FaultShortCircuit        = 0x0020;  // Short circuit detected
    constexpr uint16_t FaultPositionSensor      = 0x0040;  // Position sensor error
    constexpr uint16_t FaultPressureSensor      = 0x0080;  // Pressure sensor error
}

// ============================================================================
// PDO Layout for CiA 408
// ============================================================================

// PDO layout is now a per-instance member (CiA408Slave::PDOLayout pdoLayout_)

// ============================================================================
// Forward declarations for file-local helper functions
// ============================================================================

namespace {
    void processControlWord408(CiA408Slave* slave, uint16_t controlWord,
                               const CiA408SlaveConfig& config, uint16_t& faultCode);
    void updateStatusWord408(CiA408Slave* slave, uint16_t& statusWord,
                             const CiA408SlaveConfig& config, uint16_t faultCode,
                             int16_t targetPosition);
    void calculateFlow408(CiA408Slave* slave, const CiA408SlaveConfig& config, int32_t& flow);
}  // anonymous namespace

// ============================================================================
// CiA408Slave Implementation
// ============================================================================

CiA408Slave::CiA408Slave(const CiA408SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA408, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , valveConfig_(config)
{
    // Initialize status
    statusWord_ = CiA408Status::Ready;
}

CiA408Slave::~CiA408Slave() = default;

// ============================================================================
// Object Dictionary Registration
// ============================================================================

void CiA408Slave::initObjectDictionary() {
    // Register standard CiA 301 communication objects
    ProfileSlave::registerCiA301Objects();
    
    auto& od = getObjectDictionary();
    
    // ========================================================================
    // Operating Parameters (0x6000-0x60FF)
    // ========================================================================
    
    // 0x6000 - Valve Type
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::ValveType,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,  // Read-only
            .name = "Valve type",
            .defaultValue = static_cast<uint8_t>(valveConfig_.valveType)
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(valveConfig_.valveType);
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6003 - Response Time
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::ResponseTime,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Response time (ms)",
            .defaultValue = valveConfig_.responseTime
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.responseTime, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Device Parameters (0x6100-0x61FF)
    // ========================================================================
    
    // 0x6100 - Deadband Compensation (per-instance member)
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::DeadbandCompensation,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,  // Read-write
            .name = "Deadband compensation",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &deadbandCompensation_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&deadbandCompensation_, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6110 - Dither Frequency
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::DitherFrequency,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Dither frequency (Hz)",
            .defaultValue = valveConfig_.ditherFrequency
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.ditherFrequency, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&valveConfig_.ditherFrequency, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6111 - Dither Amplitude
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::DitherAmplitude,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Dither amplitude (0.01%)",
            .defaultValue = valveConfig_.ditherAmplitude
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.ditherAmplitude, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&valveConfig_.ditherAmplitude, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Command and Feedback (0x6200-0x62FF)
    // ========================================================================
    
    // 0x6200 - Command Value
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::CommandValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Command value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &command_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            int16_t newCmd;
            std::memcpy(&newCmd, data, 2);
            // Clamp to valid range
            command_ = std::clamp(newCmd, valveConfig_.commandMin, valveConfig_.commandMax);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6201 - Actual Position
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::ActualPosition,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x01,  // Read-only
            .name = "Actual position",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &actualPosition_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6204 - Filtered Command
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::CommandFiltered,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Filtered command",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &commandFiltered_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Pressure and Flow (0x6300-0x63FF)
    // ========================================================================
    
    // 0x6300 - Pressure Port A
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::PressurePortA,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Pressure port A (mbar)",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &pressureA_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6301 - Pressure Port B
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::PressurePortB,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Pressure port B (mbar)",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &pressureB_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6310 - Flow Rate
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::FlowRate,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Flow rate (0.01 L/min)",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &flow_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Configuration (0x6400-0x64FF)
    // ========================================================================
    
    // 0x6400 - Command Min
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::CommandMin,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Command minimum",
            .defaultValue = static_cast<uint32_t>(static_cast<uint16_t>(valveConfig_.commandMin))
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.commandMin, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6401 - Command Max
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::CommandMax,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Command maximum",
            .defaultValue = static_cast<uint32_t>(static_cast<uint16_t>(valveConfig_.commandMax))
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.commandMax, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6402 - Neutral Position
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::NeutralPosition,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Neutral position",
            .defaultValue = static_cast<uint32_t>(static_cast<uint16_t>(valveConfig_.neutralPosition))
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &valveConfig_.neutralPosition, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&valveConfig_.neutralPosition, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Status and Diagnostics (0x6500-0x65FF)
    // ========================================================================
    
    // 0x6500 - Status Word
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::StatusWord,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Status word",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &statusWord_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6501 - Control Word
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::ControlWord,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Control word",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &controlWord_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t newControl;
            std::memcpy(&newControl, data, 2);
            processControlWord408(this, newControl, valveConfig_, faultCode_);
            controlWord_ = newControl;
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6502 - Fault Code
    od.registerObject(
        ODEntryInfo{
            .index = CiA408Index::FaultCode,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Fault code",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &faultCode_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
}

// ============================================================================
// PDO Mappings
// ============================================================================

void CiA408Slave::initPDOMappings() {
    // ========================================================================
    // TxPDO Mapping (Slave -> Master, SM3)
    // ========================================================================
    // Standard CiA 408 TxPDO:
    // - Status word (0x6500:0) - 16 bits
    // - Actual position (0x6201:0) - 16 bits
    // - Pressure A (0x6300:0) - 32 bits (optional)
    // - Pressure B (0x6301:0) - 32 bits (optional)
    // - Flow rate (0x6310:0) - 32 bits (optional)
    // - Fault code (0x6502:0) - 16 bits
    
    std::vector<uint32_t> txPdoMapping;
    
    // Status word (2 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA408Index::StatusWord, 0, 16));
    pdoLayout_.statusOffset = 0;
    
    // Actual position (2 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA408Index::ActualPosition, 0, 16));
    pdoLayout_.actualPosOffset = 2;
    
    size_t offset = 4;
    
    // Pressure A (4 bytes) - if feedback available
    if (valveConfig_.hasPressureFeedback) {
        txPdoMapping.push_back(PDOMapEntry(CiA408Index::PressurePortA, 0, 32));
        pdoLayout_.pressureAOffset = offset;
        offset += 4;
        
        // Pressure B (4 bytes)
        txPdoMapping.push_back(PDOMapEntry(CiA408Index::PressurePortB, 0, 32));
        pdoLayout_.pressureBOffset = offset;
        offset += 4;
    }
    
    // Flow rate (4 bytes) - if feedback available
    if (valveConfig_.hasFlowFeedback) {
        txPdoMapping.push_back(PDOMapEntry(CiA408Index::FlowRate, 0, 32));
        pdoLayout_.flowOffset = offset;
        offset += 4;
    }
    
    // Fault code (2 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA408Index::FaultCode, 0, 16));
    pdoLayout_.faultOffset = offset;
    
    registerPDOMapping(0x1A00, txPdoMapping);
    
    // ========================================================================
    // RxPDO Mapping (Master -> Slave, SM2)
    // ========================================================================
    // Standard CiA 408 RxPDO:
    // - Control word (0x6501:0) - 16 bits
    // - Command value (0x6200:0) - 16 bits
    
    std::vector<uint32_t> rxPdoMapping;
    
    // Control word (2 bytes)
    rxPdoMapping.push_back(PDOMapEntry(CiA408Index::ControlWord, 0, 16));
    pdoLayout_.controlOffset = 0;
    
    // Command value (2 bytes)
    rxPdoMapping.push_back(PDOMapEntry(CiA408Index::CommandValue, 0, 16));
    pdoLayout_.commandOffset = 2;
    
    registerPDOMapping(0x1600, rxPdoMapping);
}

// ============================================================================
// TxPDO Update (Slave -> Master)
// ============================================================================

void CiA408Slave::updateTxPDO() {
    auto* txData = getTxPDOPtr<uint8_t>(0);
    if (!txData) return;
    
    // Build status word
    updateStatusWord408(this, statusWord_, valveConfig_, faultCode_, targetPosition_);
    
    // Write status word
    std::memcpy(txData + pdoLayout_.statusOffset, &statusWord_, 2);
    
    // Write actual position
    std::memcpy(txData + pdoLayout_.actualPosOffset, &actualPosition_, 2);
    
    // Write pressure if available
    if (valveConfig_.hasPressureFeedback) {
        std::memcpy(txData + pdoLayout_.pressureAOffset, &pressureA_, 4);
        std::memcpy(txData + pdoLayout_.pressureBOffset, &pressureB_, 4);
    }
    
    // Write flow if available
    if (valveConfig_.hasFlowFeedback) {
        std::memcpy(txData + pdoLayout_.flowOffset, &flow_, 4);
    }
    
    // Write fault code
    std::memcpy(txData + pdoLayout_.faultOffset, &faultCode_, 2);
}

// ============================================================================
// RxPDO Processing (Master -> Slave)
// ============================================================================

void CiA408Slave::processRxPDO() {
    const auto* rxData = getRxPDOPtr<uint8_t>(0);
    if (!rxData) return;
    
    // Read control word
    uint16_t newControlWord;
    std::memcpy(&newControlWord, rxData + pdoLayout_.controlOffset, 2);
    processControlWord408(this, newControlWord, valveConfig_, faultCode_);
    controlWord_ = newControlWord;
    
    // Read command value
    int16_t newCommand;
    std::memcpy(&newCommand, rxData + pdoLayout_.commandOffset, 2);
    
    // Clamp to valid range
    command_ = std::clamp(newCommand, valveConfig_.commandMin, valveConfig_.commandMax);
}

// ============================================================================
// Control Word Processing (static helper in anonymous namespace)
// ============================================================================

namespace {

void processControlWord408(CiA408Slave* slave, uint16_t controlWord,
                           const CiA408SlaveConfig& config, uint16_t& faultCode) {
    // Enable/disable
    bool enableRequest = (controlWord & CiA408Status::CtrlEnable) != 0;
    if (enableRequest && !slave->isEnabled() && slave->getFaultCode() == 0) {
        slave->setEnabled(true);
    } else if (!enableRequest) {
        slave->setEnabled(false);
    }
    
    // Quick stop (center valve) - handled via setActualPosition
    if (controlWord & CiA408Status::CtrlQuickStop) {
        slave->setActualPosition(config.neutralPosition);
    }
    
    // Fault reset
    if (controlWord & CiA408Status::CtrlFaultReset) {
        slave->clearFault();
    }
    
    // Dither enable (per-instance member)
    slave->setDitherEnabled(
        (controlWord & CiA408Status::CtrlEnableDither) != 0 &&
        config.supportsDither);
}

}  // anonymous namespace

// ============================================================================
// Status Word Update (static helper in anonymous namespace)
// ============================================================================

namespace {

void updateStatusWord408(CiA408Slave* slave, uint16_t& statusWord,
                         const CiA408SlaveConfig& config, uint16_t faultCode,
                         int16_t targetPosition) {
    statusWord = CiA408Status::Ready;
    
    if (slave->isEnabled()) {
        statusWord |= CiA408Status::Enabled;
    }
    
    if (faultCode != 0) {
        statusWord |= CiA408Status::Fault;
    }
    
    // Check if target reached (within 1% of target)
    int16_t error = targetPosition - slave->getActualPosition();
    if (error < 0) error = -error;
    if (error < 100) {  // 1% of 10000
        statusWord |= CiA408Status::TargetReached;
    }
    
    // Check deadband (per-instance member)
    int16_t cmd = slave->getCommand();
    int16_t deadband = slave->getDeadbandCompensation();
    if (cmd >= -deadband && cmd <= deadband) {
        statusWord |= CiA408Status::InDeadband;
    }
    
    // Dither status (per-instance member)
    if (slave->isDitherEnabled() && config.supportsDither) {
        statusWord |= CiA408Status::DitherActive;
    }
}

}  // anonymous namespace

// ============================================================================
// Simulation / Valve Update
// ============================================================================

void CiA408Slave::simulate(uint64_t deltaNs) {
    // Update position based on command
    updatePosition(deltaNs);
    
    // Apply dither if enabled (per-instance member)
    if (ditherEnabled_ && valveConfig_.supportsDither) {
        applyDither();
    }
    
    // Use simulation callback if set
    if (simCallback_) {
        simCallback_(command_, actualPosition_);
    }
    
    // Calculate flow from valve position and pressure differential
    if (valveConfig_.hasFlowFeedback) {
        calculateFlow408(this, valveConfig_, flow_);
    }
}

// ============================================================================
// Position Update
// ============================================================================

void CiA408Slave::updatePosition(uint64_t deltaNs) {
    if (!enabled_) {
        // When disabled, slowly return to neutral
        int16_t error = valveConfig_.neutralPosition - actualPosition_;
        if (error != 0) {
            int16_t step = (error > 0) ? 1 : -1;
            int16_t absError = (error > 0) ? error : -error;
            actualPosition_ += step * std::min(static_cast<int16_t>(10), absError);
        }
        return;
    }
    
    // Calculate target from command with deadband compensation (per-instance member)
    targetPosition_ = command_;
    
    // Apply deadband compensation
    if (targetPosition_ > deadbandCompensation_) {
        targetPosition_ += deadbandCompensation_;
    } else if (targetPosition_ < -deadbandCompensation_) {
        targetPosition_ -= deadbandCompensation_;
    } else {
        targetPosition_ = valveConfig_.neutralPosition;
    }
    
    // Clamp target to valid range
    targetPosition_ = std::clamp(targetPosition_, 
                                  valveConfig_.positionMin, 
                                  valveConfig_.positionMax);
    
    // First-order response model
    // Time constant = response time
    double dtSeconds = static_cast<double>(deltaNs) / 1.0e9;
    double responseTimeSeconds = static_cast<double>(valveConfig_.responseTime) / 1000.0;
    
    if (responseTimeSeconds > 0.0 && dtSeconds > 0.0) {
        double alpha = 1.0 - std::exp(-dtSeconds / responseTimeSeconds);
        double error = static_cast<double>(targetPosition_ - actualPosition_);
        actualPosition_ += static_cast<int16_t>(error * alpha);
    } else {
        actualPosition_ = targetPosition_;
    }
    
    // Update filtered command
    commandFiltered_ = actualPosition_;
}

// ============================================================================
// Dither Application
// ============================================================================

void CiA408Slave::applyDither() {
    // Simple sinusoidal dither
    ditherPhase_ += 1;
    if (ditherPhase_ >= 360) {
        ditherPhase_ = 0;
    }
    
    // Calculate dither offset
    double phaseRad = static_cast<double>(ditherPhase_) * 3.14159265 / 180.0;
    double ditherOffset = std::sin(phaseRad) * 
                          (static_cast<double>(valveConfig_.ditherAmplitude) / 100.0);
    
    // Apply dither to filtered command (not actual position directly)
    commandFiltered_ += static_cast<int16_t>(ditherOffset);
}

// ============================================================================
// Flow Calculation (static helper in anonymous namespace)
// ============================================================================

namespace {

void calculateFlow408(CiA408Slave* slave, const CiA408SlaveConfig& config, int32_t& flow) {
    // Simplified flow model: Q = Kv * sqrt(dP) * position
    // Where Kv is the flow coefficient and dP is pressure differential
    
    int32_t pressureA = slave->getPressureA();
    int32_t pressureB = slave->getPressureB();
    int32_t deltaP = pressureA - pressureB;
    if (deltaP < 0) deltaP = -deltaP;
    
    // Normalized position (-1 to +1)
    double normalizedPos = static_cast<double>(slave->getActualPosition()) / 10000.0;
    
    // Flow proportional to position and sqrt of pressure
    double sqrtDP = std::sqrt(static_cast<double>(deltaP));
    double flowRate = normalizedPos * sqrtDP * 
                      (static_cast<double>(config.maxFlow) / 10000.0);
    
    flow = static_cast<int32_t>(flowRate);
}

}  // anonymous namespace

// ============================================================================
// Public API Methods
// ============================================================================

void CiA408Slave::setActualPosition(int16_t position) {
    actualPosition_ = std::clamp(position, valveConfig_.positionMin, valveConfig_.positionMax);
}

void CiA408Slave::setPressure(int32_t a, int32_t b) {
    pressureA_ = std::clamp(a, valveConfig_.pressureMin, valveConfig_.pressureMax);
    pressureB_ = std::clamp(b, valveConfig_.pressureMin, valveConfig_.pressureMax);
}

void CiA408Slave::setFlow(int32_t flow) {
    flow_ = std::clamp(flow, -valveConfig_.maxFlow, valveConfig_.maxFlow);
}

void CiA408Slave::setFault(uint16_t faultCode) {
    faultCode_ = faultCode;
    if (faultCode != 0) {
        statusWord_ |= 0x0008;   // Set fault bit in status word
        enabled_ = false;         // Disable on fault
    }
}

void CiA408Slave::clearFault() {
    faultCode_ = 0;
    statusWord_ &= ~0x0008;  // Clear fault bit in status word
}

void CiA408Slave::setEnabled(bool enabled) {
    if (enabled && faultCode_ == 0) {
        enabled_ = true;
    } else if (!enabled) {
        enabled_ = false;
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<CiA408Slave> createCiA408Slave(const CiA408SlaveConfig& config) {
    return std::make_unique<CiA408Slave>(config);
}

std::unique_ptr<CiA408Slave> createProportionalValve() {
    CiA408SlaveConfig config;
    config.identity.deviceName = "Proportional Valve";
    config.valveType = ValveType::ProportionalValve;
    config.hasPositionFeedback = true;
    config.hasPressureFeedback = false;
    config.hasFlowFeedback = false;
    config.responseTime = 20;  // 20ms typical for proportional valves
    config.supportsDither = true;
    config.ditherFrequency = 200;
    config.ditherAmplitude = 50;
    return std::make_unique<CiA408Slave>(config);
}

std::unique_ptr<CiA408Slave> createServoValve() {
    CiA408SlaveConfig config;
    config.identity.deviceName = "Servo Valve";
    config.valveType = ValveType::ServoValve;
    config.hasPositionFeedback = true;
    config.hasPressureFeedback = true;
    config.hasFlowFeedback = true;
    config.responseTime = 5;  // 5ms typical for servo valves
    config.supportsDither = true;
    config.ditherFrequency = 400;
    config.ditherAmplitude = 25;
    return std::make_unique<CiA408Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
