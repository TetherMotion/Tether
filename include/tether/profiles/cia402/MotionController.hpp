/**
 * @file MotionController.hpp
 * @brief CiA 402 Motion Controller - High-level API facade
 * 
 * This facade combines all CiA 402 components into a cohesive motion control system:
 * - State machine management
 * - Multi-mode operation (PP, PV, PT, HM, IP, CSP, CSV, CST)
 * - Motion profile generation
 * - Multi-axis path interpolation
 * - Electronic gearing
 * - PID control
 */

#pragma once

#include "tether/common/IMotionController.hpp"
#include "tether/common/IAxis.hpp"

#include "CiA402Config.hpp"
#include "CiA402StateMachine.hpp"
#include "DriveBackend.hpp"
#include "MotionProfile.hpp"
#include "MultiAxisPath.hpp"
#include "HomingHandler.hpp"
#include "ElectronicGearing.hpp"
#include "PIDController.hpp"

#include <memory>
#include <vector>
#include <map>
#include <functional>
#include <string>
#include <atomic>
#include <magic_enum/magic_enum.hpp>
#include <mutex>

namespace CiA402 {

// ============================================================================
// Axis Controller
// ============================================================================

/**
 * @brief Single axis controller
 * 
 * Manages state machine, motion generation, and communication for one drive.
 */
class CiA402Axis : public tether::common::IAxis {
public:
    using AxisId = uint32_t;
    using MotionCompleteCallback = std::function<void(bool success)>;
    using HomingCompleteCallback = std::function<void(bool success, HomingError error)>;
    using FaultCallback = std::function<void(uint16_t errorCode, const std::string& msg)>;
    
    /**
     * @brief Axis status
     */
    struct Status {
        State state{State::NotReadyToSwitchOn};
        OperatingMode mode{OperatingMode::NoMode};
        bool motionComplete{true};
        bool homingComplete{false};
        bool targetReached{false};
        bool followingError{false};
        bool fault{false};
        int32_t actualPosition{0};
        int32_t actualVelocity{0};
        int16_t actualTorque{0};
        uint16_t errorCode{0};
    };
    
    CiA402Axis(AxisId id, DriveBackendUPtr backend);
    ~CiA402Axis();
    
    // Non-copyable
    CiA402Axis(const CiA402Axis&) = delete;
    CiA402Axis& operator=(const CiA402Axis&) = delete;
    
    // Moveable
    CiA402Axis(CiA402Axis&&) = default;
    CiA402Axis& operator=(CiA402Axis&&) = default;
    
    // ========================================================================
    // Identification
    // ========================================================================
    
    AxisId getId() const { return m_id; }
    std::string getName() const;
    void setName(const std::string& name);
    
    // ========================================================================
    // State Machine Control
    // ========================================================================
    
    /**
     * @brief Get current status
     */
    Status getStatus() const;
    
    /**
     * @brief Get CiA 402 state
     */
    State getState() const;
    
    /**
     * @brief Request state transition
     * @param target Target state
     * @param timeoutMs Timeout in milliseconds
     * @return true if transition successful
     */
    bool requestState(State target, uint32_t timeoutMs = 5000);
    
    /**
     * @brief Enable drive (transition to OperationEnabled)
     */
    bool enable(uint32_t timeoutMs = 5000) override;
    
    /**
     * @brief Disable drive (transition to SwitchOnDisabled)
     */
    bool disable(uint32_t timeoutMs = 5000) override;
    
    /**
     * @brief Quick stop
     */
    bool quickStop();
    
    /**
     * @brief Clear fault and reset
     */
    bool clearFault() override;
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    /**
     * @brief Set operating mode
     */
    bool setOperatingMode(OperatingMode mode);
    
    /**
     * @brief Get current operating mode
     */
    OperatingMode getOperatingMode() const;
    
    // ========================================================================
    // Position Mode Commands
    // ========================================================================
    
    /**
     * @brief Execute point-to-point motion
     */
    bool moveAbsolute(int32_t position, uint32_t velocity = 0);
    
    /**
     * @brief Execute relative motion
     */
    bool moveRelative(int32_t distance, uint32_t velocity = 0);
    
    /**
     * @brief Execute motion with full parameters
     */
    bool executeMotion(const MotionCommand& cmd);
    
    /**
     * @brief Halt motion immediately
     */
    bool halt();
    
    /**
     * @brief Wait for motion complete
     * @param timeoutMs Timeout in milliseconds
     */
    bool waitMotionComplete(uint32_t timeoutMs = 60000) override;
    
    // ========================================================================
    // Velocity Mode Commands
    // ========================================================================
    
    /**
     * @brief Set target velocity
     */
    bool setVelocity(int32_t velocity);
    
    /**
     * @brief Execute velocity command
     */
    bool executeVelocity(const VelocityCommand& cmd);
    
    // ========================================================================
    // Torque Mode Commands
    // ========================================================================
    
    /**
     * @brief Set target torque
     */
    bool setTorque(int16_t torque);
    
    /**
     * @brief Execute torque command
     */
    bool executeTorque(const TorqueCommand& cmd);
    
    // ========================================================================
    // Cyclic Synchronous Mode
    // ========================================================================
    
    /**
     * @brief Set cyclic position target
     */
    void setCyclicPosition(int32_t position, int32_t velocityFF = 0, int16_t torqueFF = 0);
    
    /**
     * @brief Set cyclic velocity target
     */
    void setCyclicVelocity(int32_t velocity, int16_t torqueFF = 0);
    
    /**
     * @brief Set cyclic torque target
     */
    void setCyclicTorque(int16_t torque);
    
    // ========================================================================
    // Homing
    // ========================================================================
    
    /**
     * @brief Start homing operation
     */
    bool startHoming(const HomingCommand& cmd);
    
    /**
     * @brief Start homing with default parameters
     */
    bool startHoming(HomingMethod method = HomingMethod::CurrentPosition);
    
    /**
     * @brief Wait for homing complete
     */
    bool waitHomingComplete(uint32_t timeoutMs = 30000) override;
    
    /**
     * @brief Check if homing is complete
     */
    bool isHomingComplete() const;
    
    /**
     * @brief Check if axis is homed
     */
    bool isHomed() const override;
    
    // ========================================================================
    // IAxis interface (remaining methods)
    // ========================================================================
    
    bool stop() override { return halt(); }
    bool hasFault() const override;
    bool isEnabled() const override;
    
    int32_t getActualPosition() const override { return m_backend->getActualPosition(); }
    int32_t getActualVelocity() const override { return m_backend->getActualVelocity(); }
    int16_t getActualTorque() const override { return m_backend->getActualTorque(); }
    
    bool setTargetPosition(int32_t target, const MotionCommand& mode) override;
    bool setTargetVelocity(int32_t target, const VelocityCommand& mode) override;
    bool setTargetTorque(int16_t target, const TorqueCommand& mode) override;
    bool home(const HomingCommand& cmd) override;
    void update(double dtSeconds) override { update(static_cast<float>(dtSeconds)); }
    
    // ========================================================================
    // Motion Profile Configuration
    // ========================================================================
    
    /**
     * @brief Set motion limits
     */
    void setMotionLimits(const MotionLimits& limits) override;
    
    /**
     * @brief Get motion limits
     */
    MotionLimits getMotionLimits() const override;
    
    /**
     * @brief Set profile velocity
     */
    void setProfileVelocity(uint32_t velocity);
    
    /**
     * @brief Set profile acceleration
     */
    void setProfileAcceleration(uint32_t acceleration);
    
    /**
     * @brief Set profile deceleration
     */
    void setProfileDeceleration(uint32_t deceleration);
    
    /**
     * @brief Set default profile type
     */
    void setDefaultProfileType(ProfileType type);
    
    // ========================================================================
    // PID Configuration
    // ========================================================================
    
    /**
     * @brief Configure position loop PID
     */
    void setPositionPID(const PIDGains& gains, const PIDLimits& limits = {});
    
    /**
     * @brief Configure velocity loop PID
     */
    void setVelocityPID(const PIDGains& gains, const PIDLimits& limits = {});
    
    /**
     * @brief Get position PID controller
     */
    PIDController& getPositionPID();
    
    /**
     * @brief Get velocity PID controller
     */
    PIDController& getVelocityPID();
    
    // ========================================================================
    // Electronic Gearing
    // ========================================================================
    
    /**
     * @brief Set gearing master axis
     */
    void setGearingMaster(CiA402Axis* masterAxis);
    
    /**
     * @brief Set gear ratio
     */
    void setGearRatio(int32_t numerator, int32_t denominator);
    
    /**
     * @brief Enable/disable gearing
     */
    void enableGearing(bool enable);
    
    /**
     * @brief Check if axis is gearing slave
     */
    bool isGearingSlave() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setMotionCompleteCallback(MotionCompleteCallback cb);
    void setHomingCompleteCallback(HomingCompleteCallback cb);
    void setFaultCallback(FaultCallback cb);
    
    // ========================================================================
    // Cycle Update
    // ========================================================================
    
    /**
     * @brief Update cycle - call once per control cycle
     * @param dtSeconds Time since last update in seconds
     */
    void update(float dtSeconds);
    
    /**
     * @brief Get drive backend (for advanced access)
     */
    DriveBackend* getBackend() { return m_backend.get(); }
    const DriveBackend* getBackend() const { return m_backend.get(); }
    
private:
    void updateStateMachine();
    void updateMotionGeneration(float dt);
    void updateGearing();
    void checkMotionComplete();
    void checkHomingComplete();
    uint16_t buildControlWord();
    
    AxisId m_id;
    std::string m_name;
    DriveBackendUPtr m_backend;
    StateMachine m_stateMachine;
    
    // Motion profiles
    MotionLimits m_limits;
    ProfileType m_defaultProfileType{ProfileType::Trapezoidal};
    std::unique_ptr<MotionProfile> m_activeProfile;
    MotionState m_profileState;
    double m_profileTime{0.0};  ///< Current time in profile [s]
    
    // Homing
    std::unique_ptr<HomingHandler> m_homingHandler;
    HomingCommand m_homingCommand;
    bool m_homingActive{false};
    bool m_homed{false};
    
    // PID controllers
    PIDController m_positionPID;
    PIDController m_velocityPID;
    
    // Gearing
    CiA402Axis* m_gearingMaster{nullptr};
    int32_t m_gearNumerator{1};
    int32_t m_gearDenominator{1};
    bool m_gearingEnabled{false};
    
    // State
    OperatingMode m_operatingMode{OperatingMode::NoMode};
    MotionCommand m_currentMotion;
    bool m_motionActive{false};
    bool m_newSetpoint{false};
    
    // Callbacks
    MotionCompleteCallback m_motionCompleteCallback;
    HomingCompleteCallback m_homingCompleteCallback;
    FaultCallback m_faultCallback;
    
    mutable std::mutex m_mutex;
};

using CiA402AxisPtr = std::shared_ptr<CiA402Axis>;

} // namespace CiA402
