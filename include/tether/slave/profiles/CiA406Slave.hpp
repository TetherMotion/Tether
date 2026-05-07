/**
 * @file CiA406Slave.hpp
 * @brief CiA 406 Encoder Slave Implementation
 *
 * @details
 * Implements a CiA 406 compliant encoder slave with:
 * - Incremental encoder simulation
 * - Absolute encoder (single/multi-turn) simulation
 * - Position preset
 * - Speed calculation
 * - Alarm handling
 *
 * ## Encoder Types
 *
 * | Type | Value | Description |
 * |------|-------|-------------|
 * | Incremental | 0 | Incremental encoder (requires homing) |
 * | SingleTurn  | 1 | Absolute single-turn encoder |
 * | MultiTurn   | 2 | Absolute multi-turn encoder |
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Encoder Type
// ============================================================================

enum class EncoderType : uint8_t {
    Incremental = 0,
    SingleTurn  = 1,
    MultiTurn   = 2,
};

// ============================================================================
// CiA 406 Slave Configuration
// ============================================================================

struct CiA406SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000196,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 406 Encoder",
    };
    
    EncoderType encoderType = EncoderType::SingleTurn;
    
    // Resolution
    uint32_t stepsPerRevolution = 131072;  // 17-bit encoder
    uint16_t totalMeasuringRange = 1;      // Number of turns (multi-turn)
    
    // Speed measurement
    bool supportsSpeedMeasurement = true;
    uint16_t speedCalculationPeriod = 1;   // ms
    
    // Preset
    bool supportsPreset = true;
    
    // Alarms
    bool supportsAlarms = true;
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 406 Slave Class
// ============================================================================

class CiA406Slave : public ProfileSlave {
public:
    explicit CiA406Slave(const CiA406SlaveConfig& config);
    ~CiA406Slave() override;
    
    const char* getProfileName() const override { return "CiA 406"; }
    uint32_t getDeviceType() const override { return 0x00000196; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Position
    int32_t getPosition() const { return position_; }
    void setPosition(int32_t position);  // For simulation
    
    // Speed
    int32_t getSpeed() const { return speed_; }
    void setSpeed(int32_t speed);  // For simulation
    
    // Multi-turn (for multi-turn encoders)
    uint16_t getTurns() const { return turns_; }
    void setTurns(uint16_t turns);
    
    // Preset
    void presetPosition(int32_t value);
    
    // Alarms
    bool isAlarmActive() const { return alarmStatus_ != 0; }
    uint8_t getAlarmStatus() const { return alarmStatus_; }
    void setAlarmStatus(uint8_t status);  // For simulation
    void clearAlarms();
    
    // Operating status
    uint16_t getOperatingStatus() const { return operatingStatus_; }
    
    // Simulation callbacks
    using PositionCallback = std::function<int32_t()>;
    void setPositionCallback(PositionCallback callback) { positionCallback_ = callback; }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA406SlaveConfig encoderConfig_;
    
    // Position and speed
    int32_t position_ = 0;          // 0x6004
    int32_t speed_ = 0;             // 0x6030
    uint16_t turns_ = 0;            // Multi-turn counter
    
    // Operating status
    uint16_t operatingStatus_ = 0;  // 0x6500
    
    // Alarms
    uint8_t alarmStatus_ = 0;       // 0x6503
    
    // Preset
    int32_t presetValue_ = 0;       // 0x6003
    
    // Speed calculation
    int32_t lastPosition_ = 0;
    uint64_t lastSpeedCalcTime_ = 0;
    
    PositionCallback positionCallback_;
    
    // PDO layout (offsets within PDO data, per-instance)
    struct PDOLayout {
        size_t positionOffset = 0;      // TxPDO: 4 bytes position
        size_t speedOffset = 4;         // TxPDO: 4 bytes speed
        size_t statusOffset = 8;        // TxPDO: 2 bytes status
        size_t alarmOffset = 10;        // TxPDO: 1 byte alarm
        size_t turnsOffset = 11;        // TxPDO: 2 bytes multi-turn
        size_t controlOffset = 0;       // RxPDO: 2 bytes control
        size_t presetOffset = 2;        // RxPDO: 4 bytes preset
    };
    PDOLayout pdoLayout_;
    
    // Control state (per-instance, not shared)
    uint16_t controlWord_ = 0;         // 0x6600
    int32_t overspeedThreshold_ = 0;   // 0x6510
    
    void calculateSpeed(uint64_t deltaNs);
};

std::unique_ptr<CiA406Slave> createCiA406Slave(const CiA406SlaveConfig& config);

// Factory functions for common encoder types
std::unique_ptr<CiA406Slave> createIncrementalEncoder(uint32_t resolution = 4096);
std::unique_ptr<CiA406Slave> createAbsoluteEncoder(uint32_t resolution = 131072);
std::unique_ptr<CiA406Slave> createMultiTurnEncoder(uint32_t resolution = 131072, uint16_t turns = 4096);

}  // namespace slave
}  // namespace EtherCAT
