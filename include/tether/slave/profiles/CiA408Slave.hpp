/**
 * @file CiA408Slave.hpp
 * @brief CiA 408 Fluid Power Technology (Hydraulic Valves) Slave
 *
 * @details
 * Implements a CiA 408 compliant hydraulic valve slave with:
 * - Valve command input
 * - Position feedback
 * - Pressure feedback
 * - Flow control
 * - Fault handling
 *
 * ## Valve Types
 *
 * | Type | Description |
 * |------|-------------|
 * | DirectionalValve | On/off directional control |
 * | ProportionalValve | Proportional flow control |
 * | ServoValve | High-precision servo valve |
 * | PressureValve | Pressure control valve |
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Valve Type
// ============================================================================

enum class ValveType : uint8_t {
    DirectionalValve  = 0,
    ProportionalValve = 1,
    ServoValve        = 2,
    PressureValve     = 3,
};

// ============================================================================
// CiA 408 Slave Configuration
// ============================================================================

struct CiA408SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000198,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 408 Hydraulic Valve",
    };
    
    ValveType valveType = ValveType::ProportionalValve;
    
    // Valve configuration
    int16_t commandMin = -10000;      // Minimum command value (-100.00%)
    int16_t commandMax = 10000;       // Maximum command value (+100.00%)
    int16_t neutralPosition = 0;      // Neutral/center position
    
    // Position feedback
    bool hasPositionFeedback = true;
    int16_t positionMin = -10000;
    int16_t positionMax = 10000;
    
    // Pressure feedback
    bool hasPressureFeedback = false;
    int32_t pressureMin = 0;          // Minimum pressure (mbar)
    int32_t pressureMax = 35000;      // Maximum pressure (350 bar)
    
    // Flow
    bool hasFlowFeedback = false;
    int32_t maxFlow = 100000;         // Max flow (0.01 L/min units)
    
    // Response time
    uint16_t responseTime = 10;       // ms
    
    // Dither
    bool supportsDither = false;
    uint16_t ditherFrequency = 200;   // Hz
    uint16_t ditherAmplitude = 50;    // 0.5%
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 408 Slave Class
// ============================================================================

class CiA408Slave : public ProfileSlave {
public:
    explicit CiA408Slave(const CiA408SlaveConfig& config);
    ~CiA408Slave() override;
    
    const char* getProfileName() const override { return "CiA 408"; }
    uint32_t getDeviceType() const override { return 0x00000198; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Command
    int16_t getCommand() const { return command_; }
    
    // Position feedback
    int16_t getActualPosition() const { return actualPosition_; }
    void setActualPosition(int16_t position);  // For simulation
    
    // Pressure feedback
    int32_t getPressureA() const { return pressureA_; }
    int32_t getPressureB() const { return pressureB_; }
    void setPressure(int32_t a, int32_t b);  // For simulation
    
    // Flow
    int32_t getFlow() const { return flow_; }
    void setFlow(int32_t flow);  // For simulation
    
    // Status
    uint16_t getStatusWord() const { return statusWord_; }
    bool isFaulted() const { return (statusWord_ & 0x0008) != 0; }
    
    // Fault handling
    void setFault(uint16_t faultCode);
    void clearFault();
    uint16_t getFaultCode() const { return faultCode_; }
    
    // Enable/disable
    void setEnabled(bool enabled);
    bool isEnabled() const { return enabled_; }
    
    // Deadband compensation (per-instance)
    int16_t getDeadbandCompensation() const { return deadbandCompensation_; }
    void setDeadbandCompensation(int16_t val) { deadbandCompensation_ = val; }
    
    // Dither enable (per-instance)
    bool isDitherEnabled() const { return ditherEnabled_; }
    void setDitherEnabled(bool enabled) { ditherEnabled_ = enabled; }
    
    // Simulation callback
    using SimulationCallback = std::function<void(int16_t command, int16_t& position)>;
    void setSimulationCallback(SimulationCallback callback) { simCallback_ = callback; }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA408SlaveConfig valveConfig_;
    
    // Command
    int16_t command_ = 0;             // Input command
    int16_t commandFiltered_ = 0;     // Filtered command
    
    // Position
    int16_t actualPosition_ = 0;
    int16_t targetPosition_ = 0;
    
    // Pressure
    int32_t pressureA_ = 0;           // Port A pressure
    int32_t pressureB_ = 0;           // Port B pressure
    
    // Flow
    int32_t flow_ = 0;
    
    // Status
    uint16_t statusWord_ = 0;
    uint16_t controlWord_ = 0;
    uint16_t faultCode_ = 0;
    bool enabled_ = false;
    
    // Dither
    uint16_t ditherPhase_ = 0;
    
    // PDO layout (offsets within PDO data, per-instance)
    struct PDOLayout {
        size_t statusOffset = 0;        // TxPDO: 2 bytes status
        size_t actualPosOffset = 2;     // TxPDO: 2 bytes position
        size_t pressureAOffset = 4;     // TxPDO: 4 bytes pressure A
        size_t pressureBOffset = 8;     // TxPDO: 4 bytes pressure B
        size_t flowOffset = 12;         // TxPDO: 4 bytes flow
        size_t faultOffset = 16;        // TxPDO: 2 bytes fault
        size_t controlOffset = 0;       // RxPDO: 2 bytes control
        size_t commandOffset = 2;       // RxPDO: 2 bytes command
    };
    PDOLayout pdoLayout_;
    
    // Valve parameters (per-instance, not shared)
    int16_t deadbandCompensation_ = 0;
    bool ditherEnabled_ = false;
    
    SimulationCallback simCallback_;
    
    void updatePosition(uint64_t deltaNs);
    void applyDither();
};

std::unique_ptr<CiA408Slave> createCiA408Slave(const CiA408SlaveConfig& config);

// Factory functions
std::unique_ptr<CiA408Slave> createProportionalValve();
std::unique_ptr<CiA408Slave> createServoValve();

}  // namespace slave
}  // namespace EtherCAT
