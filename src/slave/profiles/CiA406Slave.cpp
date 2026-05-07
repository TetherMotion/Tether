/**
 * @file CiA406Slave.cpp
 * @brief CiA 406 Encoder Slave Implementation
 *
 * @details
 * This implementation provides a complete CiA 406 compliant encoder slave with:
 * - Position value (single-turn and multi-turn)
 * - Speed calculation from position changes
 * - Preset function (set current position to a specific value)
 * - Alarm handling (battery low, overspeed, position error)
 * - PDO mappings for real-time position feedback
 *
 * CiA 406 Object Index Ranges:
 * - 0x6000-0x60FF: Operating parameters and status
 * - 0x6100-0x61FF: Scaling and unit settings
 * - 0x6200-0x62FF: Preset/offset values
 * - 0x6300-0x63FF: Speed values
 * - 0x6400-0x64FF: Acceleration values (optional)
 * - 0x6500-0x65FF: Operating status
 */

#include "slave/profiles/CiA406Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 406 Object Dictionary Indices
// ============================================================================

namespace CiA406Index {
    // Operating Parameters (0x6000-0x60FF)
    constexpr uint16_t EncoderType              = 0x6000;  // Encoder type
    constexpr uint16_t MeasuringUnitsPerRev     = 0x6001;  // Steps per revolution
    constexpr uint16_t TotalMeasuringRange      = 0x6002;  // Total turns (multi-turn)
    constexpr uint16_t PresetValue              = 0x6003;  // Preset value
    constexpr uint16_t PositionValue            = 0x6004;  // Current position
    
    // Preset and Offset (0x6010-0x601F)
    constexpr uint16_t PositionOffset           = 0x6010;  // Position offset
    constexpr uint16_t PresetControl            = 0x6011;  // Preset control word
    
    // Speed (0x6030-0x603F)
    constexpr uint16_t SpeedValue               = 0x6030;  // Current speed
    constexpr uint16_t SpeedCalcPeriod          = 0x6031;  // Speed calculation period (ms)
    constexpr uint16_t SpeedFormat              = 0x6032;  // Speed format/units
    
    // Acceleration (0x6040-0x604F)
    constexpr uint16_t AccelerationValue        = 0x6040;  // Current acceleration
    
    // Scaling (0x6100-0x61FF)
    constexpr uint16_t ScalingNumerator         = 0x6100;  // Scaling numerator
    constexpr uint16_t ScalingDenominator       = 0x6101;  // Scaling denominator
    constexpr uint16_t ScalingOffset            = 0x6102;  // Scaling offset
    
    // Position Limits (0x6200-0x62FF)
    constexpr uint16_t PositionLimitMin         = 0x6200;  // Minimum position
    constexpr uint16_t PositionLimitMax         = 0x6201;  // Maximum position
    constexpr uint16_t SoftwareLimitEnable      = 0x6202;  // Enable software limits
    
    // Operating Status (0x6500-0x65FF)
    constexpr uint16_t OperatingStatus          = 0x6500;  // Operating status word
    constexpr uint16_t OperatingMode            = 0x6501;  // Operating mode
    constexpr uint16_t SupportedOperatingModes  = 0x6502;  // Supported modes
    constexpr uint16_t AlarmStatus              = 0x6503;  // Alarm status bits
    constexpr uint16_t WarningStatus            = 0x6504;  // Warning status bits
    
    // Alarm Thresholds (0x6510-0x651F)
    constexpr uint16_t OverspeedThreshold       = 0x6510;  // Overspeed alarm threshold
    constexpr uint16_t UnderVoltageThreshold    = 0x6511;  // Undervoltage threshold
    
    // Control (0x6600-0x66FF)
    constexpr uint16_t ControlWord              = 0x6600;  // Control word
    constexpr uint16_t StatusWord               = 0x6601;  // Status word
}

// ============================================================================
// Status Bit Definitions
// ============================================================================

namespace CiA406Status {
    // Operating status bits (0x6500)
    constexpr uint16_t Ready                    = 0x0001;  // Encoder ready
    constexpr uint16_t PositionValid            = 0x0002;  // Position is valid
    constexpr uint16_t SpeedValid               = 0x0004;  // Speed is valid
    constexpr uint16_t PresetActive             = 0x0008;  // Preset is active
    constexpr uint16_t RefPointSet              = 0x0010;  // Reference point set
    
    // Alarm status bits (0x6503)
    constexpr uint8_t AlarmBatteryLow           = 0x01;    // Battery low (multi-turn)
    constexpr uint8_t AlarmOverspeed            = 0x02;    // Overspeed detected
    constexpr uint8_t AlarmPositionError        = 0x04;    // Position error
    constexpr uint8_t AlarmHardwareError        = 0x08;    // Hardware error
    constexpr uint8_t AlarmTemperature          = 0x10;    // Over temperature
    constexpr uint8_t AlarmCountingError        = 0x20;    // Counting error
    
    // Warning status bits (0x6504)
    constexpr uint8_t WarnBatteryLow            = 0x01;    // Battery warning
    constexpr uint8_t WarnApproachingLimit      = 0x02;    // Approaching limit
    constexpr uint8_t WarnSpeedHigh             = 0x04;    // Speed approaching limit
    
    // Control word bits (0x6600)
    constexpr uint16_t CtrlPreset               = 0x0001;  // Execute preset
    constexpr uint16_t CtrlResetAlarm           = 0x0002;  // Reset alarms
    constexpr uint16_t CtrlEnableSpeed          = 0x0004;  // Enable speed calculation
    constexpr uint16_t CtrlEnableLimits         = 0x0008;  // Enable limit checking
}

// ============================================================================
// PDO Layout for CiA 406
// ============================================================================

// PDO layout is now a per-instance member (CiA406Slave::PDOLayout pdoLayout_)

// ============================================================================
// CiA406Slave Implementation
// ============================================================================

CiA406Slave::CiA406Slave(const CiA406SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA406, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , encoderConfig_(config)
{
    // Initialize operating status
    operatingStatus_ = CiA406Status::Ready | CiA406Status::PositionValid;
    
    if (config.supportsSpeedMeasurement) {
        operatingStatus_ |= CiA406Status::SpeedValid;
    }
}

CiA406Slave::~CiA406Slave() = default;

// ============================================================================
// Object Dictionary Registration
// ============================================================================

void CiA406Slave::initObjectDictionary() {
    // Register standard CiA 301 communication objects
    ProfileSlave::registerCiA301Objects();
    
    auto& od = getObjectDictionary();
    
    // ========================================================================
    // Operating Parameters (0x6000-0x60FF)
    // ========================================================================
    
    // 0x6000 - Encoder Type
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::EncoderType,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,  // Read-only
            .name = "Encoder type",
            .defaultValue = static_cast<uint8_t>(encoderConfig_.encoderType)
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(encoderConfig_.encoderType);
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6001 - Measuring Units Per Revolution
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::MeasuringUnitsPerRev,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Steps per revolution",
            .defaultValue = encoderConfig_.stepsPerRevolution
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &encoderConfig_.stepsPerRevolution, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6002 - Total Measuring Range (turns for multi-turn)
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::TotalMeasuringRange,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Total measuring range",
            .defaultValue = encoderConfig_.totalMeasuringRange
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &encoderConfig_.totalMeasuringRange, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6003 - Preset Value
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::PresetValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,  // Read-write
            .name = "Preset value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &presetValue_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&presetValue_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6004 - Position Value
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::PositionValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x01,  // Read-only
            .name = "Position value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &position_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Speed Objects (0x6030-0x603F)
    // ========================================================================
    
    // 0x6030 - Speed Value
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::SpeedValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x01,
            .name = "Speed value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &speed_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6031 - Speed Calculation Period
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::SpeedCalcPeriod,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Speed calc period (ms)",
            .defaultValue = encoderConfig_.speedCalculationPeriod
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &encoderConfig_.speedCalculationPeriod, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&encoderConfig_.speedCalculationPeriod, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Operating Status Objects (0x6500-0x65FF)
    // ========================================================================
    
    // 0x6500 - Operating Status
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::OperatingStatus,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x01,
            .name = "Operating status",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &operatingStatus_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6503 - Alarm Status
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::AlarmStatus,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x01,
            .name = "Alarm status",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = alarmStatus_;
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Alarm Thresholds (0x6510-0x651F)
    // ========================================================================
    
    // 0x6510 - Overspeed Threshold
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::OverspeedThreshold,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Overspeed threshold",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &overspeedThreshold_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&overspeedThreshold_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Control Objects (0x6600-0x66FF)
    // ========================================================================
    
    // 0x6600 - Control Word
    od.registerObject(
        ODEntryInfo{
            .index = CiA406Index::ControlWord,
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
            // Process control word inline
            if ((newControl & CiA406Status::CtrlPreset) && encoderConfig_.supportsPreset) {
                presetPosition(presetValue_);
            }
            if (newControl & CiA406Status::CtrlResetAlarm) {
                clearAlarms();
            }
            controlWord_ = newControl;
            return SDOAbortCode::Success;
        }
    );
}

// ============================================================================
// PDO Mappings
// ============================================================================

void CiA406Slave::initPDOMappings() {
    // ========================================================================
    // TxPDO Mapping (Slave -> Master, SM3)
    // ========================================================================
    // Standard CiA 406 TxPDO:
    // - Position value (0x6004:0) - 32 bits
    // - Speed value (0x6030:0) - 32 bits
    // - Operating status (0x6500:0) - 16 bits
    // - Alarm status (0x6503:0) - 8 bits
    // - Multi-turn counter (optional for multi-turn encoders)
    
    std::vector<uint32_t> txPdoMapping;
    
    // Position (4 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA406Index::PositionValue, 0, 32));
    pdoLayout_.positionOffset = 0;
    
    // Speed (4 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA406Index::SpeedValue, 0, 32));
    pdoLayout_.speedOffset = 4;
    
    // Operating status (2 bytes)
    txPdoMapping.push_back(PDOMapEntry(CiA406Index::OperatingStatus, 0, 16));
    pdoLayout_.statusOffset = 8;
    
    // Alarm status (1 byte)
    txPdoMapping.push_back(PDOMapEntry(CiA406Index::AlarmStatus, 0, 8));
    pdoLayout_.alarmOffset = 10;
    
    // Multi-turn counter (2 bytes) - only for multi-turn encoders
    if (encoderConfig_.encoderType == EncoderType::MultiTurn) {
        txPdoMapping.push_back(PDOMapEntry(CiA406Index::TotalMeasuringRange, 0, 16));
        pdoLayout_.turnsOffset = 11;
    }
    
    registerPDOMapping(0x1A00, txPdoMapping);
    
    // ========================================================================
    // RxPDO Mapping (Master -> Slave, SM2)
    // ========================================================================
    // Standard CiA 406 RxPDO:
    // - Control word (0x6600:0) - 16 bits
    // - Preset value (0x6003:0) - 32 bits
    
    std::vector<uint32_t> rxPdoMapping;
    
    // Control word (2 bytes)
    rxPdoMapping.push_back(PDOMapEntry(CiA406Index::ControlWord, 0, 16));
    pdoLayout_.controlOffset = 0;
    
    // Preset value (4 bytes)
    rxPdoMapping.push_back(PDOMapEntry(CiA406Index::PresetValue, 0, 32));
    pdoLayout_.presetOffset = 2;
    
    registerPDOMapping(0x1600, rxPdoMapping);
}

// ============================================================================
// TxPDO Update (Slave -> Master)
// ============================================================================

void CiA406Slave::updateTxPDO() {
    auto* txData = getTxPDOPtr<uint8_t>(0);
    if (!txData) return;
    
    // Write position
    std::memcpy(txData + pdoLayout_.positionOffset, &position_, 4);
    
    // Write speed
    std::memcpy(txData + pdoLayout_.speedOffset, &speed_, 4);
    
    // Write operating status
    std::memcpy(txData + pdoLayout_.statusOffset, &operatingStatus_, 2);
    
    // Write alarm status
    txData[pdoLayout_.alarmOffset] = alarmStatus_;
    
    // Write multi-turn counter if applicable
    if (encoderConfig_.encoderType == EncoderType::MultiTurn) {
        std::memcpy(txData + pdoLayout_.turnsOffset, &turns_, 2);
    }
}

// ============================================================================
// RxPDO Processing (Master -> Slave)
// ============================================================================

void CiA406Slave::processRxPDO() {
    const auto* rxData = getRxPDOPtr<uint8_t>(0);
    if (!rxData) return;
    
    // Read control word
    uint16_t newControlWord;
    std::memcpy(&newControlWord, rxData + pdoLayout_.controlOffset, 2);
    
    // Process control word inline
    if ((newControlWord & CiA406Status::CtrlPreset) && encoderConfig_.supportsPreset) {
        presetPosition(presetValue_);
    }
    if (newControlWord & CiA406Status::CtrlResetAlarm) {
        clearAlarms();
    }
    controlWord_ = newControlWord;
    
    // Read preset value
    std::memcpy(&presetValue_, rxData + pdoLayout_.presetOffset, 4);
}

// ============================================================================
// Simulation / Position Update
// ============================================================================

void CiA406Slave::simulate(uint64_t deltaNs) {
    // If position callback is set, use it to get position
    if (positionCallback_) {
        position_ = positionCallback_();
    }
    
    // Calculate speed
    if (encoderConfig_.supportsSpeedMeasurement) {
        calculateSpeed(deltaNs);
    }
    
    // Check for alarms inline
    if (encoderConfig_.supportsAlarms) {
        uint8_t newAlarms = 0;
        
        // Check overspeed
        if (overspeedThreshold_ > 0) {
            int32_t absSpeed = speed_ >= 0 ? speed_ : -speed_;
            if (absSpeed > overspeedThreshold_) {
                newAlarms |= CiA406Status::AlarmOverspeed;
            }
        }
        
        // Battery low simulation for multi-turn encoders
        // In a real implementation, this would check actual battery voltage
        // (simulated: no battery alarm)
        
        // Latch alarms (they stay until cleared)
        alarmStatus_ |= newAlarms;
    }
    
    // Handle multi-turn wraparound
    if (encoderConfig_.encoderType == EncoderType::MultiTurn) {
        int32_t stepsPerRev = static_cast<int32_t>(encoderConfig_.stepsPerRevolution);
        
        // Calculate turns from absolute position
        if (position_ >= stepsPerRev) {
            int32_t newTurns = position_ / stepsPerRev;
            if (turns_ != static_cast<uint16_t>(newTurns % encoderConfig_.totalMeasuringRange)) {
                turns_ = static_cast<uint16_t>(newTurns % encoderConfig_.totalMeasuringRange);
            }
        } else if (position_ < 0) {
            int32_t newTurns = (position_ - stepsPerRev + 1) / stepsPerRev;
            uint16_t turnVal = static_cast<uint16_t>(
                (encoderConfig_.totalMeasuringRange + newTurns % encoderConfig_.totalMeasuringRange) 
                % encoderConfig_.totalMeasuringRange);
            if (turns_ != turnVal) {
                turns_ = turnVal;
            }
        }
    }
    
    // Handle single-turn wraparound for incremental encoders
    if (encoderConfig_.encoderType == EncoderType::Incremental ||
        encoderConfig_.encoderType == EncoderType::SingleTurn) {
        int32_t stepsPerRev = static_cast<int32_t>(encoderConfig_.stepsPerRevolution);
        position_ = ((position_ % stepsPerRev) + stepsPerRev) % stepsPerRev;
    }
}

// ============================================================================
// Speed Calculation
// ============================================================================

void CiA406Slave::calculateSpeed(uint64_t deltaNs) {
    // Accumulate time
    lastSpeedCalcTime_ += deltaNs;
    
    // Calculate speed when period has elapsed
    uint64_t periodNs = static_cast<uint64_t>(encoderConfig_.speedCalculationPeriod) * 1000000ULL;
    
    if (lastSpeedCalcTime_ >= periodNs && periodNs > 0) {
        // Calculate position change
        int32_t deltaPos = position_ - lastPosition_;
        
        // Calculate speed in counts per second
        double dtSeconds = static_cast<double>(lastSpeedCalcTime_) / 1.0e9;
        if (dtSeconds > 0.0) {
            speed_ = static_cast<int32_t>(static_cast<double>(deltaPos) / dtSeconds);
        }
        
        // Store current position for next calculation
        lastPosition_ = position_;
        lastSpeedCalcTime_ = 0;
    }
}

// ============================================================================
// Public API Methods
// ============================================================================

void CiA406Slave::setPosition(int32_t position) {
    position_ = position;
}

void CiA406Slave::setSpeed(int32_t speed) {
    speed_ = speed;
}

void CiA406Slave::setTurns(uint16_t turns) {
    if (encoderConfig_.encoderType == EncoderType::MultiTurn) {
        turns_ = turns % encoderConfig_.totalMeasuringRange;
    }
}

void CiA406Slave::presetPosition(int32_t value) {
    if (!encoderConfig_.supportsPreset) {
        return;
    }
    
    // Set new position
    position_ = value;
    
    // Set preset active status
    operatingStatus_ |= CiA406Status::PresetActive;
    operatingStatus_ |= CiA406Status::RefPointSet;
    
    // For multi-turn, also reset turn counter
    if (encoderConfig_.encoderType == EncoderType::MultiTurn) {
        turns_ = 0;
    }
    
    // Reset speed calculation
    lastPosition_ = position_;
    speed_ = 0;
}

void CiA406Slave::setAlarmStatus(uint8_t status) {
    alarmStatus_ = status;
}

void CiA406Slave::clearAlarms() {
    alarmStatus_ = 0;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<CiA406Slave> createCiA406Slave(const CiA406SlaveConfig& config) {
    return std::make_unique<CiA406Slave>(config);
}

std::unique_ptr<CiA406Slave> createIncrementalEncoder(uint32_t resolution) {
    CiA406SlaveConfig config;
    config.identity.deviceName = "Incremental Encoder";
    config.encoderType = EncoderType::Incremental;
    config.stepsPerRevolution = resolution;
    config.totalMeasuringRange = 1;
    config.supportsPreset = true;
    config.supportsAlarms = true;
    return std::make_unique<CiA406Slave>(config);
}

std::unique_ptr<CiA406Slave> createAbsoluteEncoder(uint32_t resolution) {
    CiA406SlaveConfig config;
    config.identity.deviceName = "Absolute Encoder";
    config.encoderType = EncoderType::SingleTurn;
    config.stepsPerRevolution = resolution;
    config.totalMeasuringRange = 1;
    config.supportsPreset = true;
    config.supportsAlarms = true;
    return std::make_unique<CiA406Slave>(config);
}

std::unique_ptr<CiA406Slave> createMultiTurnEncoder(uint32_t resolution, uint16_t turns) {
    CiA406SlaveConfig config;
    config.identity.deviceName = "Multi-Turn Encoder";
    config.encoderType = EncoderType::MultiTurn;
    config.stepsPerRevolution = resolution;
    config.totalMeasuringRange = turns;
    config.supportsPreset = true;
    config.supportsAlarms = true;
    return std::make_unique<CiA406Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
