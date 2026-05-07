/**
 * @file HomingModes.hpp
 * @brief Comprehensive Homing Mode Implementation for CiA 402
 * 
 * Implements all CiA 402 homing modes (1-37) with configurable behavior
 * and proper error handling including disconnected endstops.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <chrono>

namespace CiA402 {
namespace Homing {

// ============================================================================
// Homing Methods (CiA 402)
// ============================================================================

/**
 * @brief CiA 402 Homing Methods
 */
enum class HomingMethod : int8_t {
    // No homing
    NoHoming = 0,
    
    // Methods 1-14: Move to limit switch, then to index
    MethodNegLimitIndex = 1,     // Negative limit switch + index
    MethodPosLimitIndex = 2,     // Positive limit switch + index
    MethodPosHomeIndex = 3,      // Positive home switch + index (positive speed)
    MethodPosHomeIndex2 = 4,     // Positive home switch + index (negative speed)
    MethodNegHomeIndex = 5,      // Negative home switch + index (negative speed)
    MethodNegHomeIndex2 = 6,     // Negative home switch + index (positive speed)
    MethodHomePosIndex = 7,      // Home switch + index (positive rising edge)
    MethodHomePosIndex2 = 8,     // Home switch + index (positive falling edge)
    MethodHomePosIndex3 = 9,     // Home switch + index (negative rising edge)
    MethodHomePosIndex4 = 10,    // Home switch + index (negative falling edge)
    MethodHomePosIndex5 = 11,    // Home switch + index (positive, single edge)
    MethodHomePosIndex6 = 12,    // Home switch + index (negative, single edge)
    MethodHomePosIndex7 = 13,    // Home switch + index (positive, reverse)
    MethodHomePosIndex8 = 14,    // Home switch + index (negative, reverse)
    
    // Methods 17-30: Move to limit switch or home switch without index
    MethodNegLimit = 17,         // Negative limit switch
    MethodPosLimit = 18,         // Positive limit switch
    MethodPosHome = 19,          // Positive home switch (positive speed)
    MethodPosHome2 = 20,         // Positive home switch (negative speed)
    MethodNegHome = 21,          // Negative home switch (negative speed)
    MethodNegHome2 = 22,         // Negative home switch (positive speed)
    MethodHomePos = 23,          // Home switch (positive rising edge)
    MethodHomePos2 = 24,         // Home switch (positive falling edge)
    MethodHomePos3 = 25,         // Home switch (negative rising edge)
    MethodHomePos4 = 26,         // Home switch (negative falling edge)
    MethodHomePos5 = 27,         // Home switch (positive, single edge)
    MethodHomePos6 = 28,         // Home switch (negative, single edge)
    MethodHomePos7 = 29,         // Home switch (positive, reverse)
    MethodHomePos8 = 30,         // Home switch (negative, reverse)
    
    // Methods 33-34: Index only
    MethodIndexNeg = 33,         // Index pulse (negative direction first)
    MethodIndexPos = 34,         // Index pulse (positive direction first)
    
    // Method 35-37: Current position
    MethodCurrentPositionIndex = 35,  // Current position + index
    MethodCurrentPosition = 37,       // Current position (no movement)
};

/**
 * @brief Homing state machine states
 */
enum class HomingState {
    Idle,               // Not homing
    Starting,           // Preparing to home
    FindSwitch,         // Moving to find switch
    LeaveSwitch,        // Moving off switch
    FindIndex,          // Looking for index pulse
    ZeroVelocity,       // Decelerating to stop
    Attained,           // Homing attained
    Error,              // Error during homing
    Interrupted         // Homing was interrupted
};

/**
 * @brief Homing error codes
 */
enum class HomingError {
    None = 0,
    
    // General errors
    InvalidMethod = 0x8001,
    NotInitialized = 0x8002,
    AlreadyHoming = 0x8003,
    
    // Limit switch errors
    NegativeLimitNotConfigured = 0x8010,
    PositiveLimitNotConfigured = 0x8011,
    NegativeLimitDisconnected = 0x8012,
    PositiveLimitDisconnected = 0x8013,
    NegativeLimitNotFound = 0x8014,
    PositiveLimitNotFound = 0x8015,
    BothLimitsActive = 0x8016,
    
    // Home switch errors
    HomeSwitchNotConfigured = 0x8020,
    HomeSwitchDisconnected = 0x8021,
    HomeSwitchNotFound = 0x8022,
    HomeSwitchStuck = 0x8023,
    
    // Index errors
    IndexNotConfigured = 0x8030,
    IndexDisconnected = 0x8031,
    IndexNotFound = 0x8032,
    IndexTimeout = 0x8033,
    MultipleIndexPulses = 0x8034,
    
    // Motion errors
    MotionFault = 0x8040,
    FollowingError = 0x8041,
    PositionLimit = 0x8042,
    Timeout = 0x8043,
    Interrupted = 0x8044,
    
    // Drive errors
    DriveNotEnabled = 0x8050,
    DriveFault = 0x8051,
};

/**
 * @brief Homing configuration
 */
struct HomingConfig {
    // Speeds (user units/s)
    int32_t searchVelocity = 1000;   // Switch search velocity
    int32_t zeroVelocity = 100;      // Index search / zero velocity
    
    // Accelerations
    int32_t acceleration = 10000;    // Acceleration rate
    int32_t deceleration = 10000;    // Deceleration rate
    
    // Offsets
    int32_t homeOffset = 0;          // Offset applied after homing
    
    // Timeouts
    uint32_t searchTimeoutMs = 30000;   // Switch search timeout
    uint32_t indexTimeoutMs = 5000;     // Index search timeout
    uint32_t settleTimeMs = 100;        // Time to wait at home position
    
    // Error handling
    bool stopOnError = true;         // Stop motion on error
    bool faultOnError = true;        // Raise fault on error
    
    // Endstop validation
    bool validateEndstops = true;    // Check endstop connectivity
    uint32_t endstopDebounceMs = 10; // Debounce time for endstop signals
    
    // Index validation
    bool validateIndex = true;       // Check for single index per revolution
    double expectedIndexInterval = 0; // Expected counts between index pulses
    double indexIntervalTolerance = 0.01; // Tolerance for index interval
};

/**
 * @brief Switch states for homing
 */
struct SwitchStates {
    bool positiveLimit = false;
    bool negativeLimit = false;
    bool homeSwitch = false;
    bool indexPulse = false;
    bool indexDetected = false;
    int32_t lastIndexPosition = 0;
};

/**
 * @brief Homing statistics
 */
struct HomingStatistics {
    uint32_t homingAttempts = 0;
    uint32_t successfulHomings = 0;
    uint32_t failedHomings = 0;
    uint32_t timeoutErrors = 0;
    uint32_t endstopErrors = 0;
    uint32_t indexErrors = 0;
    uint32_t motionErrors = 0;
    
    // Timing
    uint32_t lastHomingDurationMs = 0;
    uint32_t totalHomingTimeMs = 0;
    int32_t lastHomePosition = 0;
    
    void reset() {
        homingAttempts = 0;
        successfulHomings = 0;
        failedHomings = 0;
        timeoutErrors = 0;
        endstopErrors = 0;
        indexErrors = 0;
        motionErrors = 0;
        lastHomingDurationMs = 0;
        totalHomingTimeMs = 0;
        lastHomePosition = 0;
    }
};

/**
 * @brief Homing error injection for testing
 */
struct HomingErrorInjection {
    bool enabled = false;
    
    // Endstop faults
    bool disconnectNegativeLimit = false;
    bool disconnectPositiveLimit = false;
    bool disconnectHomeSwitch = false;
    bool disconnectIndex = false;
    
    // Timing faults
    bool simulateTimeout = false;
    bool simulateSlowResponse = false;
    double responseDelayMs = 100;
    
    // Signal faults
    bool simulateNoise = false;
    bool simulateSticking = false;
    bool simulateMultipleIndex = false;
    
    // Motion faults
    bool simulateFollowingError = false;
    bool simulateJam = false;
    
    void reset() {
        enabled = false;
        disconnectNegativeLimit = false;
        disconnectPositiveLimit = false;
        disconnectHomeSwitch = false;
        disconnectIndex = false;
        simulateTimeout = false;
        simulateSlowResponse = false;
        simulateNoise = false;
        simulateSticking = false;
        simulateMultipleIndex = false;
        simulateFollowingError = false;
        simulateJam = false;
    }
};

/**
 * @brief Callbacks for homing operations
 */
struct HomingCallbacks {
    // Motion commands
    std::function<void(int32_t velocity)> setVelocity;
    std::function<void()> stopMotion;
    std::function<void(int32_t position)> setHomePosition;
    
    // State queries
    std::function<int32_t()> getPosition;
    std::function<int32_t()> getVelocity;
    std::function<SwitchStates()> getSwitchStates;
    std::function<bool()> isMotionComplete;
    std::function<bool()> hasDriveFault;
    
    // Notifications
    std::function<void(HomingState state)> onStateChange;
    std::function<void(HomingError error, const std::string& message)> onError;
    std::function<void(int32_t homePosition)> onHomingComplete;
    std::function<void()> onHomingInterrupted;
};

/**
 * @brief Comprehensive Homing State Machine
 */
class HomingStateMachine {
public:
    HomingStateMachine();
    ~HomingStateMachine() = default;
    
    // Configuration
    void setConfig(const HomingConfig& config);
    const HomingConfig& getConfig() const { return config_; }
    
    void setCallbacks(const HomingCallbacks& callbacks);
    
    void setErrorInjection(const HomingErrorInjection& injection);
    const HomingErrorInjection& getErrorInjection() const { return errorInjection_; }
    
    // Control
    bool start(HomingMethod method);
    void stop();
    void reset();
    
    // Update
    void update();
    
    // Status
    HomingState getState() const { return state_; }
    HomingMethod getMethod() const { return method_; }
    HomingError getLastError() const { return lastError_; }
    std::string getLastErrorMessage() const { return lastErrorMessage_; }
    const HomingStatistics& getStatistics() const { return stats_; }
    
    bool isHoming() const;
    bool isComplete() const;
    bool hasError() const;
    
    // Method information
    static std::string getMethodName(HomingMethod method);
    static bool methodRequiresNegativeLimit(HomingMethod method);
    static bool methodRequiresPositiveLimit(HomingMethod method);
    static bool methodRequiresHomeSwitch(HomingMethod method);
    static bool methodRequiresIndex(HomingMethod method);

private:
    // State handlers
    void handleIdle();
    void handleStarting();
    void handleFindSwitch();
    void handleLeaveSwitch();
    void handleFindIndex();
    void handleZeroVelocity();
    void handleAttained();
    void handleError();
    void handleInterrupted();
    
    // Validation
    bool validateConfiguration();
    bool validateEndstopConnectivity();
    bool checkEndstopDisconnection();
    
    // Homing implementation by method type
    void startNegativeLimitHoming(bool withIndex);
    void startPositiveLimitHoming(bool withIndex);
    void startHomeSwitchHoming(HomingMethod method);
    void startIndexOnlyHoming(bool negativeFirst);
    void startCurrentPositionHoming(bool withIndex);
    
    // Motion helpers
    void movePositive();
    void moveNegative();
    void moveToIndex();
    void stopAndSetHome();
    
    // Error handling
    void setError(HomingError error, const std::string& message = "");
    void checkTimeouts();
    void checkMotionFaults();
    
    // State transition
    void setState(HomingState newState);
    
    // Time tracking
    uint64_t getTimeMs() const;
    
    // Configuration
    HomingConfig config_;
    HomingCallbacks callbacks_;
    HomingErrorInjection errorInjection_;
    
    // State
    HomingState state_ = HomingState::Idle;
    HomingMethod method_ = HomingMethod::NoHoming;
    HomingError lastError_ = HomingError::None;
    std::string lastErrorMessage_;
    HomingStatistics stats_;
    
    // Internal state
    int subState_ = 0;                    // Sub-state within main state
    int32_t searchDirection_ = 0;          // Current search direction
    bool lookingForRisingEdge_ = true;    // Edge detection type
    bool reverseAfterSwitch_ = false;     // Reverse direction after finding switch
    int32_t switchFoundPosition_ = 0;     // Position where switch was found
    int32_t indexFoundPosition_ = 0;      // Position where index was found
    bool indexSeen_ = false;              // Index pulse seen flag
    uint32_t indexCount_ = 0;             // Count of index pulses seen
    
    // Timing
    uint64_t stateStartTime_ = 0;
    uint64_t homingStartTime_ = 0;
    
    // Debouncing
    SwitchStates debouncedStates_;
    SwitchStates rawStates_;
    uint64_t lastStateChangeTime_ = 0;
    bool statesStable_ = false;
};

/**
 * @brief Factory to create pre-configured homing setups
 */
class HomingFactory {
public:
    /**
     * @brief Create homing config for typical servo motor
     */
    static HomingConfig createServoConfig(
        int32_t searchVelocity = 1000,
        int32_t zeroVelocity = 100,
        int32_t homeOffset = 0
    );
    
    /**
     * @brief Create homing config for linear axis
     */
    static HomingConfig createLinearAxisConfig(
        int32_t maxTravel,
        int32_t searchVelocity = 500,
        int32_t zeroVelocity = 50
    );
    
    /**
     * @brief Create homing config for rotary axis
     */
    static HomingConfig createRotaryAxisConfig(
        int32_t searchVelocity = 2000,
        bool useIndex = true
    );
    
    /**
     * @brief Create strict config with validation
     */
    static HomingConfig createStrictConfig(const HomingConfig& base);
    
    /**
     * @brief Create lenient config without validation
     */
    static HomingConfig createLenientConfig(const HomingConfig& base);
};

/**
 * @brief Utility functions for homing
 */
namespace HomingUtils {

/**
 * @brief Describe a homing method
 */
std::string describeMethod(HomingMethod method);

/**
 * @brief Get requirements for a homing method
 */
struct MethodRequirements {
    bool needsNegativeLimit;
    bool needsPositiveLimit;
    bool needsHomeSwitch;
    bool needsIndex;
    bool needsMotion;
    std::string description;
};

MethodRequirements getMethodRequirements(HomingMethod method);

/**
 * @brief Suggest best homing method based on available signals
 */
HomingMethod suggestMethod(
    bool hasNegativeLimit,
    bool hasPositiveLimit,
    bool hasHomeSwitch,
    bool hasIndex
);

/**
 * @brief Validate homing method against available signals
 */
bool validateMethod(
    HomingMethod method,
    bool hasNegativeLimit,
    bool hasPositiveLimit,
    bool hasHomeSwitch,
    bool hasIndex
);

} // namespace HomingUtils

} // namespace Homing
} // namespace CiA402
