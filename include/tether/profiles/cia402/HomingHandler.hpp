/**
 * @file HomingHandler.hpp
 * @brief CiA 402 Homing Mode Implementation
 * 
 * @details
 * Implements all CiA 402 homing methods (0-37) including:
 * - Methods 1-14: Home on limit switch
 * - Methods 15-16: Reserved
 * - Methods 17-30: Home on home switch
 * - Methods 31-32: Reserved  
 * - Methods 33-34: Home on index pulse
 * - Methods 35-37: Home on current position
 * 
 * ## Homing Method Overview
 * 
 * ```
 * Method 1:  Negative limit + index, negative initial direction
 * Method 2:  Positive limit + index, positive initial direction
 * Method 3:  Positive home + index, positive initial direction
 * Method 4:  Positive home + index, negative initial direction
 * Method 5:  Negative home + index, negative initial direction
 * Method 6:  Negative home + index, positive initial direction
 * Method 7:  Home on home switch, positive initial (w/o index)
 * ...
 * Method 33: Negative index pulse
 * Method 34: Positive index pulse
 * Method 35: Current position (no motion)
 * Method 37: Current position (no motion, alternative)
 * ```
 * 
 * ## State Machine
 * 
 * ```
 *                    ┌─────────────┐
 *         ┌─────────►│    IDLE     │◄────────────┐
 *         │         └──────┬──────┘             │
 *         │                │ start()            │ complete/error
 *         │                ▼                    │
 *         │         ┌─────────────┐             │
 *         │         │  SEARCHING  │─────────────┤
 *         │         │  (switch)   │             │
 *         │         └──────┬──────┘             │
 *         │                │ found              │
 *         │                ▼                    │
 *         │         ┌─────────────┐             │
 *         │         │  REVERSING  │─────────────┤
 *         │         │             │             │
 *         │         └──────┬──────┘             │
 *         │                │                    │
 *         │                ▼                    │
 *         │         ┌─────────────┐             │
 *   abort │         │  SEARCHING  │─────────────┤
 *         │         │  (index)    │             │
 *         │         └──────┬──────┘             │
 *         │                │ found              │
 *         │                ▼                    │
 *         │         ┌─────────────┐             │
 *         │         │  ATTAINED   │─────────────┘
 *         │         └─────────────┘
 *         │
 *         └─────────────────────────────────────
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include "DriveBackend.hpp"
#include <functional>
#include <cstdint>

namespace CiA402 {

/**
 * @brief Homing state
 */
enum class HomingState {
    Idle,               ///< Not homing
    SearchingSwitch,    ///< Searching for switch/limit
    Reversing,          ///< Reversing direction after switch
    SearchingIndex,     ///< Searching for index pulse
    Attained,           ///< Homing complete
    Error               ///< Homing error
};

/**
 * @brief Homing error codes
 */
enum class HomingError {
    None,
    Timeout,
    LimitReached,
    DriveError,
    InvalidMethod,
    Aborted
};

/**
 * @brief Switch/limit status
 */
struct HomingSwitchStatus {
    bool positiveLimitActive{false};
    bool negativeLimitActive{false};
    bool homeSwitchActive{false};
    bool indexPulseDetected{false};
};

/**
 * @brief Homing status callback
 */
using HomingStatusCallback = std::function<HomingSwitchStatus()>;

/**
 * @brief Homing complete callback
 */
using HomingCompleteCallback = std::function<void(bool success, HomingError error)>;

/**
 * @brief CiA 402 Homing Handler
 * 
 * Manages the homing sequence according to CiA 402 specification.
 */
class HomingHandler {
public:
    HomingHandler();
    ~HomingHandler() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Configure homing parameters
     */
    void configure(const HomingParams& params);
    
    /**
     * @brief Set homing method
     */
    void setMethod(HomingMethod method) { m_method = method; }
    
    /**
     * @brief Get current homing method
     */
    HomingMethod getMethod() const { return m_method; }
    
    /**
     * @brief Set switch status callback
     */
    void setSwitchCallback(HomingStatusCallback callback);
    
    /**
     * @brief Set completion callback
     */
    void setCompleteCallback(HomingCompleteCallback callback);
    
    /**
     * @brief Set drive backend
     */
    void setBackend(DriveBackendPtr backend) { m_backend = backend; }
    
    /**
     * @brief Set timeout in milliseconds
     */
    void setTimeout(uint32_t timeoutMs) { m_timeoutMs = timeoutMs; }
    
    // ========================================================================
    // Operations
    // ========================================================================
    
    /**
     * @brief Start homing sequence
     * 
     * @return true if homing started
     */
    bool start();
    
    /**
     * @brief Abort homing
     */
    void abort();
    
    /**
     * @brief Update homing state machine
     * 
     * Call this cyclically during homing.
     * 
     * @param currentPosition Current axis position
     * @return true if still homing
     */
    bool update(int32_t currentPosition);
    
    /**
     * @brief Check if homing is complete
     */
    bool isComplete() const { return m_state == HomingState::Attained; }
    
    /**
     * @brief Check if error occurred
     */
    bool hasError() const { return m_state == HomingState::Error; }
    
    /**
     * @brief Get current state
     */
    HomingState getState() const { return m_state; }
    
    /**
     * @brief Get error code
     */
    HomingError getError() const { return m_error; }
    
    /**
     * @brief Get home position (after homing complete)
     */
    int32_t getHomePosition() const { return m_homePosition; }
    
    // ========================================================================
    // Method-Specific Information
    // ========================================================================
    
    /**
     * @brief Check if method requires limit switch
     */
    static bool methodRequiresLimit(HomingMethod method);
    
    /**
     * @brief Check if method requires home switch
     */
    static bool methodRequiresHome(HomingMethod method);
    
    /**
     * @brief Check if method requires index pulse
     */
    static bool methodRequiresIndex(HomingMethod method);
    
    /**
     * @brief Get initial search direction for method
     * 
     * @return +1 for positive, -1 for negative
     */
    static int getInitialDirection(HomingMethod method);
    
    /**
     * @brief Get method description
     */
    static const char* getMethodDescription(HomingMethod method);
    
private:
    /**
     * @brief Process method 1: Negative limit + index, negative initial
     */
    void processMethod1(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 2: Positive limit + index, positive initial
     */
    void processMethod2(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 3-6: Home switch + index
     */
    void processHomeSwitchWithIndex(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 7-14: Home switch without index
     */
    void processHomeSwitchOnly(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 17-30: Block-based homing
     */
    void processBlockHoming(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 33-34: Index only
     */
    void processIndexOnly(int32_t position, const HomingSwitchStatus& status);
    
    /**
     * @brief Process method 35/37: Current position
     */
    void processCurrentPosition(int32_t position);
    
    /**
     * @brief Set velocity command
     */
    void setVelocity(int32_t velocity);
    
    /**
     * @brief Stop motion
     */
    void stop();
    
    /**
     * @brief Complete homing
     */
    void complete(bool success, HomingError error = HomingError::None);
    
    // Configuration
    HomingParams m_params;
    HomingMethod m_method{HomingMethod::CurrentPosition};
    uint32_t m_timeoutMs{CIA402_HOMING_TIMEOUT_MS};
    
    // State
    HomingState m_state{HomingState::Idle};
    HomingError m_error{HomingError::None};
    int32_t m_homePosition{0};
    int m_searchDirection{1};
    bool m_switchFound{false};
    bool m_indexFound{false};
    
    // Timing
    uint64_t m_startTime{0};
    
    // Callbacks
    HomingStatusCallback m_switchCallback;
    HomingCompleteCallback m_completeCallback;
    
    // Backend
    DriveBackendPtr m_backend;
};

} // namespace CiA402
