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
#include <mutex>

namespace CiA402 {

// ============================================================================
// Motion Command Types
// ============================================================================

/**
 * @brief Point-to-point motion command
 */
struct MotionCommand {
    int32_t targetPosition{0};
    uint32_t velocity{0};           // 0 = use profile default
    uint32_t acceleration{0};       // 0 = use profile default
    uint32_t deceleration{0};       // 0 = use profile default
    uint32_t jerk{0};               // 0 = use default (for S-curve)
    bool relative{false};           // Relative move
    bool immediate{false};          // Start immediately (don't wait for previous)
    bool buffered{false};           // Add to motion buffer
    ProfileType profileType{ProfileType::Trapezoidal};
};

/**
 * @brief Velocity command
 */
struct VelocityCommand {
    int32_t targetVelocity{0};
    uint32_t acceleration{0};       // 0 = use profile default
    uint32_t deceleration{0};       // 0 = use profile default
    int32_t maxDuration{-1};        // -1 = indefinite
};

/**
 * @brief Torque command  
 */
struct TorqueCommand {
    int16_t targetTorque{0};
    int16_t torqueSlope{0};         // Torque ramp rate
    int32_t maxDuration{-1};        // -1 = indefinite
};

/**
 * @brief Homing command
 */
struct HomingCommand {
    HomingMethod method{HomingMethod::CurrentPosition};
    uint32_t speedSwitch{1000};
    uint32_t speedZero{100};
    uint32_t acceleration{1000};
    int32_t offset{0};
    int64_t timeoutMs{30000};
};

// ============================================================================
// Axis Controller
// ============================================================================

/**
 * @brief Single axis controller
 * 
 * Manages state machine, motion generation, and communication for one drive.
 */
class CiA402Axis {
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
    bool enable(uint32_t timeoutMs = 5000);
    
    /**
     * @brief Disable drive (transition to SwitchOnDisabled)
     */
    bool disable(uint32_t timeoutMs = 5000);
    
    /**
     * @brief Quick stop
     */
    bool quickStop();
    
    /**
     * @brief Clear fault and reset
     */
    bool clearFault();
    
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
    bool waitMotionComplete(uint32_t timeoutMs = 60000);
    
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
    bool waitHomingComplete(uint32_t timeoutMs = 30000);
    
    /**
     * @brief Check if homing is complete
     */
    bool isHomingComplete() const;
    
    /**
     * @brief Check if axis is homed
     */
    bool isHomed() const;
    
    // ========================================================================
    // Motion Profile Configuration
    // ========================================================================
    
    /**
     * @brief Set motion limits
     */
    void setMotionLimits(const MotionLimits& limits);
    
    /**
     * @brief Get motion limits
     */
    MotionLimits getMotionLimits() const;
    
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

// ============================================================================
// Multi-Axis Motion Controller
// ============================================================================

/**
 * @brief Multi-axis coordinated motion controller
 */
class MotionController {
public:
    using AxisMap = std::map<CiA402Axis::AxisId, CiA402AxisPtr>;
    
    /**
     * @brief Global motion parameters
     */
    struct GlobalParams {
        float speedFactor{1.0f};        // Global speed override (0.0 to 2.0)
        bool allowNegativeSpeed{false}; // Allow negative speed factor (reverse)
        float minSpeedFactor{0.0f};
        float maxSpeedFactor{2.0f};
    };
    
    MotionController();
    ~MotionController();
    
    // ========================================================================
    // Axis Management
    // ========================================================================
    
    /**
     * @brief Add axis to controller
     */
    CiA402AxisPtr addAxis(CiA402Axis::AxisId id, DriveBackendUPtr backend);
    
    /**
     * @brief Get axis by ID
     */
    CiA402AxisPtr getAxis(CiA402Axis::AxisId id);
    
    /**
     * @brief Get all axes
     */
    const AxisMap& getAxes() const { return m_axes; }
    
    /**
     * @brief Remove axis
     */
    bool removeAxis(CiA402Axis::AxisId id);
    
    /**
     * @brief Get number of axes
     */
    size_t getAxisCount() const { return m_axes.size(); }
    
    // ========================================================================
    // Group Operations
    // ========================================================================
    
    /**
     * @brief Enable all axes
     */
    bool enableAll(uint32_t timeoutMs = 5000);
    
    /**
     * @brief Disable all axes
     */
    bool disableAll(uint32_t timeoutMs = 5000);
    
    /**
     * @brief Quick stop all axes
     */
    void quickStopAll();
    
    /**
     * @brief Clear faults on all axes
     */
    void clearAllFaults();
    
    /**
     * @brief Check if all axes are enabled
     */
    bool allEnabled() const;
    
    /**
     * @brief Check if any axis has fault
     */
    bool anyFault() const;
    
    // ========================================================================
    // Multi-Axis Coordinated Motion
    // ========================================================================
    
    /**
     * @brief Set axes for coordinated motion
     * @param axisIds Ordered list of axis IDs for path interpolation
     */
    void setCoordinatedAxes(const std::vector<CiA402Axis::AxisId>& axisIds);
    
    /**
     * @brief Execute multi-axis linear move
     */
    bool moveLinear(const std::vector<double>& targetPositions, double velocity);
    
    /**
     * @brief Execute circular arc (2D)
     */
    bool moveCircular(double centerX, double centerY, double endX, double endY,
                     bool clockwise, double velocity);
    
    /**
     * @brief Execute helical move (circular + linear Z)
     */
    bool moveHelical(double centerX, double centerY, double endX, double endY,
                    double pitch, bool clockwise, double velocity);
    
    /**
     * @brief Execute path (multi-segment)
     */
    bool executePath(MultiSegmentPath& path, double velocity);
    
    /**
     * @brief Add path segment to buffer
     */
    bool addPathSegment(std::unique_ptr<PathSegment> segment);
    
    /**
     * @brief Start buffered path execution
     */
    bool startPath(double velocity);
    
    /**
     * @brief Stop path execution
     */
    void stopPath();
    
    /**
     * @brief Check if path is executing
     */
    bool isPathExecuting() const { return m_pathExecuting; }
    
    // ========================================================================
    // Global Parameters
    // ========================================================================
    
    /**
     * @brief Set global speed factor
     * @param factor Speed multiplier (1.0 = normal)
     */
    void setSpeedFactor(float factor);
    
    /**
     * @brief Get global speed factor
     */
    float getSpeedFactor() const { return m_globalParams.speedFactor; }
    
    /**
     * @brief Set global parameters
     */
    void setGlobalParams(const GlobalParams& params);
    
    /**
     * @brief Get global parameters
     */
    const GlobalParams& getGlobalParams() const { return m_globalParams; }
    
    // ========================================================================
    // Electronic Gearing
    // ========================================================================
    
    /**
     * @brief Configure electronic gearing
     */
    bool configureGearing(CiA402Axis::AxisId slaveId, CiA402Axis::AxisId masterId,
                         int32_t numerator, int32_t denominator);
    
    /**
     * @brief Enable gearing for axis
     */
    bool enableGearing(CiA402Axis::AxisId slaveId, bool enable);
    
    // ========================================================================
    // Homing
    // ========================================================================
    
    /**
     * @brief Home single axis
     */
    bool homeAxis(CiA402Axis::AxisId id, const HomingCommand& cmd);
    
    /**
     * @brief Home multiple axes sequentially
     */
    bool homeAxesSequential(const std::vector<CiA402Axis::AxisId>& ids,
                           const HomingCommand& cmd);
    
    /**
     * @brief Home multiple axes simultaneously
     */
    bool homeAxesSimultaneous(const std::vector<CiA402Axis::AxisId>& ids,
                             const HomingCommand& cmd);
    
    /**
     * @brief Check if all axes are homed
     */
    bool allHomed() const;
    
    // ========================================================================
    // Cycle Update
    // ========================================================================
    
    /**
     * @brief Update all axes - call once per control cycle
     * @param dtSeconds Time since last update
     */
    void update(float dtSeconds);
    
    /**
     * @brief Set cycle time
     */
    void setCycleTimeUs(uint32_t cycleTimeUs) { m_cycleTimeUs = cycleTimeUs; }
    
    /**
     * @brief Get cycle time
     */
    uint32_t getCycleTimeUs() const { return m_cycleTimeUs; }
    
private:
    void updatePathExecution(float dt);
    void applyPathPoint(const PathPoint& point);
    
    AxisMap m_axes;
    GlobalParams m_globalParams;
    
    // Coordinated motion
    std::vector<CiA402Axis::AxisId> m_coordinatedAxes;
    std::unique_ptr<MultiSegmentPath> m_activePath;
    std::unique_ptr<PathSampler> m_pathSampler;
    double m_pathTime{0.0};
    bool m_pathExecuting{false};
    
    uint32_t m_cycleTimeUs{1000}; // 1ms default
    
    mutable std::mutex m_mutex;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Create motion profile based on type
 */
std::unique_ptr<MotionProfile> createProfile(ProfileType type,
                                             const MotionLimits& limits);

/**
 * @brief Convert state to string
 */
const char* stateToString(State state);

/**
 * @brief Convert mode to string
 */
const char* modeToString(OperatingMode mode);

/**
 * @brief Convert error code to string
 */
const char* errorToString(uint16_t errorCode);

/**
 * @brief Convert homing method to string
 */
const char* homingMethodToString(HomingMethod method);

} // namespace CiA402
