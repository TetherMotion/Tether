/**
 * @file MotionControllerAxis.cpp
 * @brief CiA402Axis implementation - Individual axis control
 * 
 * This file contains the CiA402Axis class implementation including:
 * - Constructor and status management
 * - State machine interactions
 * - Motion execution (position, velocity, torque)
 * - Homing operations
 * - Configuration and callbacks
 */

#include "profiles/cia402/MotionController.hpp"
#include "tether/platform/EspCompat.hpp"
#include <algorithm>
#include <cmath>

static const char* TAG = "CiA402Axis";

namespace CiA402 {

// ============================================================================
// CiA402Axis Implementation - Construction and Status
// ============================================================================

CiA402Axis::CiA402Axis(AxisId id, DriveBackendUPtr backend)
    : m_id(id)
    , m_backend(std::move(backend))
    , m_homingHandler(std::make_unique<HomingHandler>())
{
    m_name = "Axis " + std::to_string(id);
    
    // Initialize with default limits
    m_limits.maxVelocity = static_cast<uint32_t>(CIA402_DEFAULT_MAX_VELOCITY);
    m_limits.maxAcceleration = static_cast<uint32_t>(CIA402_DEFAULT_MAX_ACCELERATION);
    m_limits.maxDeceleration = (CIA402_DEFAULT_MAX_DECELERATION > 0.0)
        ? static_cast<uint32_t>(CIA402_DEFAULT_MAX_DECELERATION)
        : m_limits.maxAcceleration;
    m_limits.maxJerk = static_cast<uint32_t>(CIA402_DEFAULT_MAX_JERK);
    
    // Set up callbacks from backend
    if (m_backend) {
        m_backend->setStateChangeCallback([this](State from, State to) {
            TETHER_LOGD(TAG, "Axis {}: State {} -> {}", 
                    (unsigned long)m_id, (int)from, (int)to);
        });
        
        m_backend->setErrorCallback([this](uint16_t code, const std::string& msg) {
            TETHER_LOGE(TAG, "Axis {} fault: 0x{:04X} - {}",
                    (unsigned long)m_id, code, msg.c_str());
            if (m_faultCallback) {
                m_faultCallback(code, msg);
            }
        });
    }
}

CiA402Axis::~CiA402Axis() = default;

std::string CiA402Axis::getName() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_name;
}

void CiA402Axis::setName(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_name = name;
}

CiA402Axis::Status CiA402Axis::getStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    Status status;
    
    if (m_backend) {
        DriveState ds = m_backend->getState();
        status.state = m_stateMachine.getCurrentState();
        status.mode = m_operatingMode;
        status.motionComplete = !m_motionActive;
        status.homingComplete = m_homingHandler ? 
            (m_homingHandler->getState() == HomingState::Attained) : false;
        status.targetReached = (ds.statusWord & 
            static_cast<uint16_t>(StatusWordBit::TargetReached)) != 0;
        status.followingError = (ds.statusWord & 
            static_cast<uint16_t>(StatusWordBit::FollowingError)) != 0;
        status.fault = (status.state == State::Fault);
        status.actualPosition = ds.actualPosition;
        status.actualVelocity = ds.actualVelocity;
        status.actualTorque = ds.actualTorque;
        status.errorCode = ds.errorCode;
    }
    
    return status;
}

State CiA402Axis::getState() const {
    return m_stateMachine.getCurrentState();
}

// ============================================================================
// State Control
// ============================================================================

bool CiA402Axis::requestState(State target, uint32_t timeoutMs) {
    if (!m_backend) return false;
    
    int64_t startTime = esp_timer_get_time();
    int64_t timeoutUs = static_cast<int64_t>(timeoutMs) * 1000;
    
    while ((esp_timer_get_time() - startTime) < timeoutUs) {
        // Update state machine
        m_backend->updateInputs();
        uint16_t statusWord = m_backend->readStatusWord();
        // Feed status word to state machine (internally decoded)
        m_stateMachine.update();
        
        if (m_stateMachine.getCurrentState() == target) {
            return true;
        }
        
        // Request state transition - this updates control word
        TransitionResult result = m_stateMachine.requestState(target, 0);
        if (result == TransitionResult::Timeout || result == TransitionResult::Pending) {
            Tether::Platform::Clock::instance().delayMilliseconds(1);
            continue;
        }
        
        uint16_t controlWord = m_stateMachine.getControlWord();
        m_backend->writeControlWord(controlWord);
        m_backend->updateOutputs();
        
        Tether::Platform::Clock::instance().delayMilliseconds(1);
    }
    
    TETHER_LOGW(TAG, "Axis {}: State transition to {} timed out",
            (unsigned long)m_id, (int)target);
    return false;
}

bool CiA402Axis::enable(uint32_t timeoutMs) {
    return requestState(State::OperationEnabled, timeoutMs);
}

bool CiA402Axis::disable(uint32_t timeoutMs) {
    return requestState(State::SwitchOnDisabled, timeoutMs);
}

bool CiA402Axis::quickStop() {
    if (!m_backend) return false;
    
    m_stateMachine.quickStop();
    uint16_t controlWord = m_stateMachine.getControlWord();
    m_backend->writeControlWord(controlWord);
    return m_backend->updateOutputs();
}

bool CiA402Axis::clearFault() {
    if (!m_backend) return false;
    
    // Fault reset sequence using resetFault()
    m_stateMachine.resetFault();
    uint16_t controlWord = m_stateMachine.getControlWord();
    m_backend->writeControlWord(controlWord);
    m_backend->updateOutputs();
    
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    // The resetFault() sets and clears the bit internally, 
    // just need to write the updated control word
    controlWord = m_stateMachine.getControlWord();
    m_backend->writeControlWord(controlWord);
    return m_backend->updateOutputs();
}

bool CiA402Axis::setOperatingMode(OperatingMode mode) {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_operatingMode = mode;
    return m_backend->setOperatingMode(mode);
}

OperatingMode CiA402Axis::getOperatingMode() const {
    return m_operatingMode;
}

// ============================================================================
// Position Motion
// ============================================================================

bool CiA402Axis::moveAbsolute(int32_t position, uint32_t velocity) {
    MotionCommand cmd;
    cmd.targetPosition = position;
    cmd.velocity = velocity;
    cmd.relative = false;
    return executeMotion(cmd);
}

bool CiA402Axis::moveRelative(int32_t distance, uint32_t velocity) {
    MotionCommand cmd;
    cmd.targetPosition = distance;
    cmd.velocity = velocity;
    cmd.relative = true;
    return executeMotion(cmd);
}

bool CiA402Axis::executeMotion(const MotionCommand& cmd) {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Set operating mode if needed
    if (m_operatingMode != OperatingMode::ProfilePosition &&
        m_operatingMode != OperatingMode::CyclicSyncPosition) {
        m_backend->setOperatingMode(OperatingMode::ProfilePosition);
        m_operatingMode = OperatingMode::ProfilePosition;
    }
    
    // Store command
    m_currentMotion = cmd;
    
    // Calculate target position
    int32_t targetPos = cmd.targetPosition;
    if (cmd.relative) {
        targetPos += m_backend->getActualPosition();
    }
    
    // Set profile parameters if specified
    if (cmd.velocity > 0) {
        m_backend->setProfileVelocity(cmd.velocity);
    }
    if (cmd.acceleration > 0) {
        m_backend->setProfileAcceleration(cmd.acceleration);
    }
    if (cmd.deceleration > 0) {
        m_backend->setProfileDeceleration(cmd.deceleration);
    }
    
    // Set motion profile type
    m_backend->setMotionProfileType(static_cast<int16_t>(cmd.profileType));
    
    // Set target position
    m_backend->setTargetPosition(targetPos);
    
    // Create profile for internal tracking
    m_activeProfile = createProfile(cmd.profileType, m_limits);
    if (m_activeProfile) {
        m_profileTime = 0.0;  // Reset profile time
        m_activeProfile->plan(
            static_cast<double>(m_backend->getActualPosition()),
            static_cast<double>(targetPos),
            static_cast<double>(m_backend->getActualVelocity())
        );
        m_profileState = m_activeProfile->evaluate(0.0);
    }
    
    // Signal new setpoint
    m_newSetpoint = true;
    m_motionActive = true;
    
    return true;
}

bool CiA402Axis::halt() {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Set halt bit in control word
    m_stateMachine.setHalt(true);
    uint16_t controlWord = buildControlWord();
    m_backend->writeControlWord(controlWord);
    
    m_motionActive = false;
    
    return m_backend->updateOutputs();
}

bool CiA402Axis::waitMotionComplete(uint32_t timeoutMs) {
    int64_t startTime = esp_timer_get_time();
    int64_t timeoutUs = static_cast<int64_t>(timeoutMs) * 1000;
    
    while ((esp_timer_get_time() - startTime) < timeoutUs) {
        if (!m_motionActive) {
            return true;
        }
        
        DriveState ds = m_backend->getState();
        if (ds.statusWord & static_cast<uint16_t>(StatusWordBit::TargetReached)) {
            m_motionActive = false;
            return true;
        }
        
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    
    return false;
}

// ============================================================================
// Velocity Motion
// ============================================================================

bool CiA402Axis::setVelocity(int32_t velocity) {
    VelocityCommand cmd;
    cmd.targetVelocity = velocity;
    return executeVelocity(cmd);
}

bool CiA402Axis::executeVelocity(const VelocityCommand& cmd) {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Set operating mode
    if (m_operatingMode != OperatingMode::ProfileVelocity &&
        m_operatingMode != OperatingMode::CyclicSyncVelocity) {
        m_backend->setOperatingMode(OperatingMode::ProfileVelocity);
        m_operatingMode = OperatingMode::ProfileVelocity;
    }
    
    // Set profile parameters
    if (cmd.acceleration > 0) {
        m_backend->setProfileAcceleration(cmd.acceleration);
    }
    if (cmd.deceleration > 0) {
        m_backend->setProfileDeceleration(cmd.deceleration);
    }
    
    // Set target velocity
    m_backend->setTargetVelocity(cmd.targetVelocity);
    
    return m_backend->updateOutputs();
}

// ============================================================================
// Torque Control
// ============================================================================

bool CiA402Axis::setTorque(int16_t torque) {
    TorqueCommand cmd;
    cmd.targetTorque = torque;
    return executeTorque(cmd);
}

bool CiA402Axis::executeTorque(const TorqueCommand& cmd) {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Set operating mode
    if (m_operatingMode != OperatingMode::ProfileTorque &&
        m_operatingMode != OperatingMode::CyclicSyncTorque) {
        m_backend->setOperatingMode(OperatingMode::ProfileTorque);
        m_operatingMode = OperatingMode::ProfileTorque;
    }
    
    // Set target torque
    m_backend->setTargetTorque(cmd.targetTorque);
    
    return m_backend->updateOutputs();
}

// ============================================================================
// Cyclic Synchronous Commands
// ============================================================================

void CiA402Axis::setCyclicPosition(int32_t position, int32_t velocityFF, int16_t torqueFF) {
    if (!m_backend) return;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_backend->setTargetPosition(position);
    if (velocityFF != 0) {
        m_backend->setVelocityOffset(velocityFF);
    }
    if (torqueFF != 0) {
        m_backend->setTorqueOffset(torqueFF);
    }
}

void CiA402Axis::setCyclicVelocity(int32_t velocity, int16_t torqueFF) {
    if (!m_backend) return;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_backend->setTargetVelocity(velocity);
    if (torqueFF != 0) {
        m_backend->setTorqueOffset(torqueFF);
    }
}

void CiA402Axis::setCyclicTorque(int16_t torque) {
    if (!m_backend) return;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_backend->setTargetTorque(torque);
}

// ============================================================================
// Homing
// ============================================================================

bool CiA402Axis::startHoming(const HomingCommand& cmd) {
    if (!m_backend) return false;
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_homingCommand = cmd;
    
    // Set operating mode to homing
    m_backend->setOperatingMode(OperatingMode::Homing);
    m_operatingMode = OperatingMode::Homing;
    
    // Configure homing parameters
    HomingParams params;
    params.method = static_cast<int8_t>(cmd.method);
    params.speedSwitch = cmd.speedSwitch;
    params.speedZero = cmd.speedZero;
    params.acceleration = cmd.acceleration;
    params.offset = cmd.offset;
    m_backend->configureHoming(params);
    
    // Initialize homing handler
    if (m_homingHandler) {
        // Configure using HomingParams 
        HomingParams hParams;
        hParams.method = static_cast<int8_t>(cmd.method);
        hParams.speedSwitch = cmd.speedSwitch;
        hParams.speedZero = cmd.speedZero;
        hParams.acceleration = cmd.acceleration;
        hParams.offset = cmd.offset;
        m_homingHandler->configure(hParams);
        m_homingHandler->start();
    }
    
    // Start homing with control word via state machine
    m_stateMachine.startHoming();
    uint16_t controlWord = buildControlWord();
    m_backend->writeControlWord(controlWord);
    
    m_homingActive = true;
    m_homed = false;
    
    return m_backend->updateOutputs();
}

bool CiA402Axis::startHoming(HomingMethod method) {
    HomingCommand cmd;
    cmd.method = static_cast<uint16_t>(static_cast<int8_t>(method));
    return startHoming(cmd);
}

bool CiA402Axis::waitHomingComplete(uint32_t timeoutMs) {
    int64_t startTime = esp_timer_get_time();
    int64_t timeoutUs = static_cast<int64_t>(timeoutMs) * 1000;
    
    while ((esp_timer_get_time() - startTime) < timeoutUs) {
        if (!m_homingActive || m_homed) {
            return m_homed;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    
    return false;
}

bool CiA402Axis::isHomingComplete() const {
    if (m_homingHandler) {
        return m_homingHandler->getState() == HomingState::Attained;
    }
    return false;
}

bool CiA402Axis::isHomed() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_homed;
}

bool CiA402Axis::hasFault() const {
    return getStatus().fault;
}

bool CiA402Axis::isEnabled() const {
    return getState() == State::OperationEnabled;
}

bool CiA402Axis::setTargetPosition(int32_t target, const MotionCommand& mode) {
    MotionCommand cmd = mode;
    cmd.targetPosition = target;
    return executeMotion(cmd);
}

bool CiA402Axis::setTargetVelocity(int32_t target, const VelocityCommand& mode) {
    VelocityCommand cmd = mode;
    cmd.targetVelocity = target;
    return executeVelocity(cmd);
}

bool CiA402Axis::setTargetTorque(int16_t target, const TorqueCommand& mode) {
    TorqueCommand cmd = mode;
    cmd.targetTorque = target;
    return executeTorque(cmd);
}

bool CiA402Axis::home(const HomingCommand& cmd) {
    return startHoming(cmd);
}

// ============================================================================
// Configuration
// ============================================================================

void CiA402Axis::setMotionLimits(const MotionLimits& limits) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_limits = limits;
}

MotionLimits CiA402Axis::getMotionLimits() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_limits;
}

void CiA402Axis::setProfileVelocity(uint32_t velocity) {
    if (m_backend) {
        m_backend->setProfileVelocity(velocity);
    }
    m_limits.maxVelocity = velocity;
}

void CiA402Axis::setProfileAcceleration(uint32_t acceleration) {
    if (m_backend) {
        m_backend->setProfileAcceleration(acceleration);
    }
    m_limits.maxAcceleration = acceleration;
}

void CiA402Axis::setProfileDeceleration(uint32_t deceleration) {
    if (m_backend) {
        m_backend->setProfileDeceleration(deceleration);
    }
    m_limits.maxDeceleration = deceleration;
}

void CiA402Axis::setDefaultProfileType(ProfileType type) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_defaultProfileType = type;
}

void CiA402Axis::setPositionPID(const PIDGains& gains, const PIDLimits& limits) {
    m_positionPID.setGains(gains);
    m_positionPID.setLimits(limits);
}

void CiA402Axis::setVelocityPID(const PIDGains& gains, const PIDLimits& limits) {
    m_velocityPID.setGains(gains);
    m_velocityPID.setLimits(limits);
}

PIDController& CiA402Axis::getPositionPID() {
    return m_positionPID;
}

PIDController& CiA402Axis::getVelocityPID() {
    return m_velocityPID;
}

// ============================================================================
// Gearing
// ============================================================================

void CiA402Axis::setGearingMaster(CiA402Axis* masterAxis) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gearingMaster = masterAxis;
}

void CiA402Axis::setGearRatio(int32_t numerator, int32_t denominator) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gearNumerator = numerator;
    m_gearDenominator = (denominator != 0) ? denominator : 1;
}

void CiA402Axis::enableGearing(bool enable) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gearingEnabled = enable;
}

bool CiA402Axis::isGearingSlave() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_gearingEnabled && m_gearingMaster != nullptr;
}

// ============================================================================
// Callbacks
// ============================================================================

void CiA402Axis::setMotionCompleteCallback(MotionCompleteCallback cb) {
    m_motionCompleteCallback = cb;
}

void CiA402Axis::setHomingCompleteCallback(HomingCompleteCallback cb) {
    m_homingCompleteCallback = cb;
}

void CiA402Axis::setFaultCallback(FaultCallback cb) {
    m_faultCallback = cb;
}

// ============================================================================
// Cyclic Update
// ============================================================================

void CiA402Axis::update(float dtSeconds) {
    if (!m_backend) return;
    
    // Update inputs from drive
    m_backend->updateInputs();
    
    // Update state machine
    updateStateMachine();
    
    // Update gearing if slave
    updateGearing();
    
    // Update motion generation
    updateMotionGeneration(dtSeconds);
    
    // Check motion complete
    checkMotionComplete();
    
    // Check homing complete
    checkHomingComplete();
    
    // Build and send control word
    uint16_t controlWord = buildControlWord();
    m_backend->writeControlWord(controlWord);
    
    // Update outputs to drive
    m_backend->updateOutputs();
}

void CiA402Axis::updateStateMachine() {
    // Update state machine - it uses its callback to read status word
    m_stateMachine.update();
}

void CiA402Axis::updateMotionGeneration(float dt) {
    if (!m_activeProfile || !m_motionActive) return;
    
    // Update profile - track time and evaluate
    m_profileTime += static_cast<double>(dt);
    m_profileState = m_activeProfile->evaluate(m_profileTime);
    
    // For cyclic sync modes, we generate the setpoints internally
    if (m_operatingMode == OperatingMode::CyclicSyncPosition ||
        m_operatingMode == OperatingMode::CyclicSyncVelocity) {
        
        if (m_operatingMode == OperatingMode::CyclicSyncPosition) {
            m_backend->setTargetPosition(static_cast<int32_t>(m_profileState.position));
            // Velocity feedforward
            m_backend->setVelocityOffset(static_cast<int32_t>(m_profileState.velocity));
        } else {
            m_backend->setTargetVelocity(static_cast<int32_t>(m_profileState.velocity));
        }
    }
    
    // Check if profile is complete
    if (m_activeProfile->isComplete(m_profileTime)) {
        m_motionActive = false;
    }
}

void CiA402Axis::updateGearing() {
    if (!m_gearingEnabled || !m_gearingMaster) return;
    
    // Get master position
    int32_t masterPos = m_gearingMaster->getStatus().actualPosition;
    
    // Calculate slave position with gear ratio
    int64_t slavePos = (static_cast<int64_t>(masterPos) * m_gearNumerator) / 
                       m_gearDenominator;
    
    // Apply to this axis (in CSP mode)
    if (m_operatingMode == OperatingMode::CyclicSyncPosition) {
        m_backend->setTargetPosition(static_cast<int32_t>(slavePos));
    }
}

void CiA402Axis::checkMotionComplete() {
    if (!m_motionActive) return;
    
    DriveState ds = m_backend->getState();
    
    // Check target reached bit
    if (ds.statusWord & static_cast<uint16_t>(StatusWordBit::TargetReached)) {
        m_motionActive = false;
        m_newSetpoint = false;
        
        if (m_motionCompleteCallback) {
            m_motionCompleteCallback(true);
        }
    }
}

void CiA402Axis::checkHomingComplete() {
    if (!m_homingActive) return;
    
    DriveState ds = m_backend->getState();
    
    // Check homing attained and target reached
    bool homingAttained = (ds.statusWord & 
        static_cast<uint16_t>(StatusWordBit::HomingAttained)) != 0;
    bool targetReached = (ds.statusWord & 
        static_cast<uint16_t>(StatusWordBit::TargetReached)) != 0;
    
    // Check homing error (bit 13)
    bool homingError = (ds.statusWord & 0x2000) != 0;
    
    if (homingAttained && targetReached) {
        m_homingActive = false;
        m_homed = true;
        
        if (m_homingCompleteCallback) {
            m_homingCompleteCallback(true, HomingError::None);
        }
    } else if (homingError) {
        m_homingActive = false;
        m_homed = false;
        
        if (m_homingCompleteCallback) {
            m_homingCompleteCallback(false, HomingError::DriveError);
        }
    }
}

uint16_t CiA402Axis::buildControlWord() {
    uint16_t controlWord = m_stateMachine.getControlWord();
    
    // Add mode-specific bits
    if (m_operatingMode == OperatingMode::ProfilePosition) {
        // Bit 4: new setpoint
        if (m_newSetpoint) {
            controlWord |= static_cast<uint16_t>(ControlWordBit::NewSetpoint);
            m_newSetpoint = false; // Clear after one cycle
        }
        
        // Bit 5: change set immediately
        if (m_currentMotion.immediate) {
            controlWord |= static_cast<uint16_t>(ControlWordBit::ChangeSetImmed);
        }
        
        // Bit 6: relative (abs=0, rel=1)
        if (m_currentMotion.relative) {
            controlWord |= static_cast<uint16_t>(ControlWordBit::AbsRel);
        }
    } else if (m_operatingMode == OperatingMode::Homing) {
        // Bit 4: homing operation start
        if (m_homingActive) {
            controlWord |= static_cast<uint16_t>(ControlWordBit::HomingStart);
        }
    }
    
    return controlWord;
}

} // namespace CiA402
