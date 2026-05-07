/**
 * @file CiA402SlaveCore.cpp
 * @brief CiA 402 Slave - Core class, constructor, destructor
 */

#include "CiA402SlaveCommon.hpp"

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA402Slave Constructor / Destructor
// ============================================================================

CiA402Slave::CiA402Slave(const CiA402SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA402, SlaveConfig{
        .identity = config.identity,
        .rxPdoSize = 17,   // Control word(2) + Target pos(4) + Target vel(4) + Target torque(2) + Mode(1) + Digital out(4)
        .txPdoSize = 17,   // Status word(2) + Actual pos(4) + Actual vel(4) + Actual torque(2) + Mode disp(1) + Digital in(4)
        .supportsDC = config.supportsDC,
        .defaultCycleTime = config.defaultCycleTime,
        .supportsBootstrap = false,
        .logConfig = config.logConfig
      })
    , driveConfig_(config)
    , driveState_(CiA402State::NotReadyToSwitchOn)
    , controlWord_(0)
    , statusWord_(0)
    , operatingMode_(0)
    , operatingModeDisplay_(0)
    , errorCode_(0)
    , actualPosition_(0)
    , targetPosition_(0)
    , positionDemand_(0)
    , positionOffset_(0)
    , actualVelocity_(0)
    , targetVelocity_(0)
    , velocityDemand_(0)
    , velocityOffset_(0)
    , profileVelocity_(config.maxProfileVelocity / 2)
    , profileAcceleration_(config.maxAcceleration / 2)
    , profileDeceleration_(config.maxDeceleration / 2)
    , actualTorque_(0)
    , targetTorque_(0)
    , torqueDemand_(0)
    , torqueOffset_(0)
    , maxTorque_(config.maxTorque)
    , homingComplete_(false)
    , homingActive_(false)
    , homingMethod_(config.homingMethod)
    , homingSwitchSpeed_(config.homingSwitchSpeed)
    , homingZeroSpeed_(config.homingZeroSpeed)
    , homingAcceleration_(config.homingAcceleration)
    , homeOffset_(config.homingOffset)
    , touchProbeFunction_(0)
    , touchProbeStatus_(0)
    , touchProbe1Pos_(0)
    , touchProbe2Pos_(0)
    , digitalInputs_(0)
    , digitalOutputs_(0)
    , digitalOutputMask_(0xFFFFFFFF)
    , targetReached_(false)
    , setPointAcknowledge_(false)
    , internalPosition_(0)
    , internalVelocity_(0)
    , lastSimTime_(0)
{
    // Initialize to NotReadyToSwitchOn, will transition to SwitchOnDisabled
    // after initialization completes
}

CiA402Slave::~CiA402Slave() = default;

// ============================================================================
// Operating Mode Support
// ============================================================================

bool CiA402Slave::isModeSupported(int8_t mode) const {
    // Convert mode value to mode flag
    uint32_t modeFlag = 0;
    
    switch (mode) {
        case CiA402ModeValue::ProfilePosition:
            modeFlag = CiA402Mode::PP;
            break;
        case CiA402ModeValue::VelocityMode:
            modeFlag = CiA402Mode::VL;
            break;
        case CiA402ModeValue::ProfileVelocity:
            modeFlag = CiA402Mode::PV;
            break;
        case CiA402ModeValue::ProfileTorque:
            modeFlag = CiA402Mode::PT;
            break;
        case CiA402ModeValue::HomingMode:
            modeFlag = CiA402Mode::HM;
            break;
        case CiA402ModeValue::InterpolatedPosition:
            modeFlag = CiA402Mode::IP;
            break;
        case CiA402ModeValue::CyclicSyncPosition:
            modeFlag = CiA402Mode::CSP;
            break;
        case CiA402ModeValue::CyclicSyncVelocity:
            modeFlag = CiA402Mode::CSV;
            break;
        case CiA402ModeValue::CyclicSyncTorque:
            modeFlag = CiA402Mode::CST;
            break;
        default:
            return false;
    }
    
    return (driveConfig_.supportedModes & modeFlag) != 0;
}

// ============================================================================
// Position Control
// ============================================================================

void CiA402Slave::setActualPosition(int32_t position) {
    actualPosition_ = position;
    internalPosition_ = position;
}

int32_t CiA402Slave::getFollowingError() const {
    return positionDemand_ - actualPosition_;
}

// ============================================================================
// Velocity Control
// ============================================================================

void CiA402Slave::setActualVelocity(int32_t velocity) {
    actualVelocity_ = velocity;
    internalVelocity_ = velocity;
}

// ============================================================================
// Torque Control
// ============================================================================

void CiA402Slave::setActualTorque(int16_t torque) {
    actualTorque_ = torque;
}

// ============================================================================
// Homing
// ============================================================================

void CiA402Slave::setHomingComplete(bool complete) {
    homingComplete_ = complete;
    if (complete) {
        homingActive_ = false;
    }
}

void CiA402Slave::setHomePosition(int32_t position) {
    actualPosition_ = position;
    internalPosition_ = position;
    positionDemand_ = position;
    homingComplete_ = true;
    homingActive_ = false;
}

void CiA402Slave::setHomingCallback(HomingCallback callback) {
    homingCallback_ = std::move(callback);
}

// ============================================================================
// Touch Probe
// ============================================================================

void CiA402Slave::triggerTouchProbe(int probeNum, int32_t position) {
    if (probeNum == 1) {
        touchProbe1Pos_ = position;
        // Set status bit for probe 1 positive edge
        touchProbeStatus_ |= (1 << 0);  // Probe 1 positive stored
    } else if (probeNum == 2) {
        touchProbe2Pos_ = position;
        // Set status bit for probe 2 positive edge
        touchProbeStatus_ |= (1 << 4);  // Probe 2 positive stored
    }
}

int32_t CiA402Slave::getTouchProbePosition(int probeNum) const {
    if (probeNum == 1) return touchProbe1Pos_;
    if (probeNum == 2) return touchProbe2Pos_;
    return 0;
}

// ============================================================================
// Digital I/O
// ============================================================================

void CiA402Slave::setDigitalInputs(uint32_t inputs) {
    digitalInputs_ = inputs;
}

// ============================================================================
// Supported Drive Functions
// ============================================================================

uint32_t CiA402Slave::getSupportedDriveFunctions() const {
    return driveConfig_.supportedModes;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<CiA402Slave> createCiA402Slave(const CiA402SlaveConfig& config) {
    return std::make_unique<CiA402Slave>(config);
}

std::unique_ptr<CiA402Slave> createServoDrive(uint32_t encoderResolution) {
    CiA402SlaveConfig config;
    config.identity.deviceName = "Servo Drive";
    config.positionEncoderResolution = encoderResolution;
    config.supportedModes = CiA402Mode::CSP | CiA402Mode::CSV | CiA402Mode::CST | 
                            CiA402Mode::PP | CiA402Mode::HM;
    config.maxProfileVelocity = 3000000;    // 3M counts/s
    config.maxMotorVelocity = 4000000;      // 4M counts/s
    config.maxAcceleration = 100000000;     // 100M counts/s²
    config.maxDeceleration = 100000000;
    config.maxTorque = 1000;                // 100% rated
    config.simulatedInertia = 0.0001f;      // Small inertia for servo
    config.simulatedFriction = 0.01f;
    return std::make_unique<CiA402Slave>(config);
}

std::unique_ptr<CiA402Slave> createStepperDrive(uint32_t stepsPerRevolution, uint32_t microstepping) {
    CiA402SlaveConfig config;
    config.identity.deviceName = "Stepper Drive";
    config.positionEncoderResolution = stepsPerRevolution * microstepping;
    config.supportedModes = CiA402Mode::CSP | CiA402Mode::PP | CiA402Mode::HM;
    config.maxProfileVelocity = 200000;     // 200k steps/s
    config.maxMotorVelocity = 250000;
    config.maxAcceleration = 5000000;       // 5M steps/s²
    config.maxDeceleration = 5000000;
    config.maxTorque = 1000;
    config.simulatedInertia = 0.001f;
    config.simulatedFriction = 0.05f;
    return std::make_unique<CiA402Slave>(config);
}

std::unique_ptr<CiA402Slave> createFrequencyInverter() {
    CiA402SlaveConfig config;
    config.identity.deviceName = "Frequency Inverter";
    config.supportedModes = CiA402Mode::CSV | CiA402Mode::PV | CiA402Mode::VL;
    config.positionEncoderResolution = 1;   // No position feedback
    config.maxProfileVelocity = 60000;      // 60000 RPM (scaled)
    config.maxMotorVelocity = 72000;
    config.maxAcceleration = 10000;
    config.maxDeceleration = 10000;
    config.maxTorque = 1500;                // 150% for short time
    config.simulatedInertia = 0.1f;         // Large inertia (motor + load)
    config.simulatedFriction = 1.0f;
    return std::make_unique<CiA402Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
