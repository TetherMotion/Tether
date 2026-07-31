/**
 * @file TorqueHomingController.hpp
 * @brief Torque-based sensorless homing controller for CiA 402 drives
 *
 * @details
 * Implements sensorless homing by driving an axis in Cyclic Synchronous
 * Torque mode (CST, CiA 402 mode 10) against a mechanical end stop. No
 * limit switch, home switch, or index pulse is required. The controller
 * applies a configurable signed target torque with a software-enforced
 * velocity cap and a software-enforced maximum position deviation, then
 * detects the mechanical stop via stall (position-derivative and/or
 * torque-saturation). After the coarse stall, it performs a configurable
 * number of "back-off & re-approach" fine passes at lower torque/velocity
 * to obtain a repeatable home position.
 *
 * ## State Machine
 *
 * ```
 *          start()
 *  IDLE ──────────────► CONFIGURING (save prev mode, switch to CST)
 *                          │
 *                          ▼
 *                    COARSE_APPROACH ──┐
 *                          │ stall      │
 *                          ▼            │
 *                    BACK_OFF (pass k)  │  k = 1..N
 *                          │ Δpos reached
 *                          ▼            │
 *                    FINE_APPROACH (k) ─┤ stall
 *                          │            │
 *                   (after last pass)   │
 *                          ▼            │
 *                    SETTLING            │
 *                          │ settle elapsed
 *                          ▼            │
 *                    ATTAINED ──restore mode──► IDLE
 *
 *                    ERROR ──restore mode──► IDLE
 * ```
 *
 * The controller is standalone: it talks to a DriveBackend (or via
 * callbacks) and is driven by a cyclic update() call, mirroring the
 * ergonomics of HomingHandler. It does not require CiA402Axis.
 *
 * ## Usage Example
 *
 * ```cpp
 * auto backend = ...; // DriveBackendPtr
 *
 * CiA402::TorqueHomingConfig cfg;
 * cfg.coarsePass.targetTorque        = 500;   // 50% rated, +dir
 * cfg.coarsePass.velocityLimit       = 2000;  // internal units/s
 * cfg.coarsePass.stallWindowMs       = 200;
 * cfg.coarsePass.stallPositionThreshold = 5;
 * cfg.coarsePass.passTimeoutMs       = 20000;
 * cfg.maxPositionDeviation           = 100000;
 *
 * CiA402::TorqueHomingPassConfig fine;
 * fine.targetTorque           = 150;
 * fine.velocityLimit          = 300;
 * fine.backOffDistance        = -200;   // opposite sign of approach
 * fine.stallWindowMs          = 300;
 * fine.stallPositionThreshold = 2;
 * fine.passTimeoutMs          = 10000;
 * cfg.finePasses.push_back(fine);
 * cfg.finePasses.push_back(fine);
 *
 * CiA402::TorqueHomingController ctrl;
 * ctrl.setBackend(backend);
 * ctrl.setConfig(cfg);
 * // populate callbacks (getActualPosition, setTargetTorque, ...)
 * ctrl.setCallbacks(cbs);
 * ctrl.start();
 *
 * while (ctrl.update()) { /* cyclic *\/ }
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include "DriveBackend.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace CiA402 {

/**
 * @brief Torque homing state machine states
 */
enum class TorqueHomingState {
    Idle,             ///< Not homing
    Configuring,      ///< Saving previous mode and switching to CST
    CoarseApproach,   ///< First approach at coarse torque
    BackOff,          ///< Backing off the stop before a fine pass
    FineApproach,     ///< Slow re-approach (one of N fine passes)
    Settling,         ///< Settling at the home position
    Attained,         ///< Homing complete
    Error             ///< Homing failed
};

/**
 * @brief Torque homing error codes
 */
enum class TorqueHomingError {
    None = 0,
    NotInitialized,          ///< Backend or callbacks missing
    InvalidConfig,           ///< Configuration invalid (e.g. zero torque)
    BackendDisconnected,     ///< Backend not connected
    ModeChangeFailed,        ///< Could not switch to/restore operating mode
    DriveNotEnabled,         ///< Drive not in OperationEnabled state
    DriveFault,              ///< Drive reported a fault during homing
    Timeout,                 ///< Global totalTimeoutMs exceeded
    PassTimeout,             ///< Per-pass passTimeoutMs exceeded without stall
    PositionDeviationExceeded, ///< Software position-deviation limit exceeded
    Aborted                  ///< abort() called
};

/**
 * @brief Velocity-limiting strategy used to cap speed in CST mode
 */
enum class VelocityLimitStrategy {
    /// Reduce torque proportionally as |v| approaches velocityLimit,
    /// beginning at softStartFraction * velocityLimit. Hard zero at the
    /// limit. Smooth, keeps the motor pushing near the cap.
    ProportionalTorqueReduction,
    /// Apply full torque until |v| reaches velocityLimit, then zero torque.
    /// Bang-bang; simpler but more torque ripple.
    HardZeroAboveLimit
};

/**
 * @brief Per-pass configuration (coarse pass and each fine pass)
 *
 * The sign of `targetTorque` defines the approach direction. The sign of
 * `backOffDistance` should be opposite to the approach direction.
 */
struct TorqueHomingPassConfig {
    /// Signed target torque in 0.1% of rated torque (sign = direction).
    int16_t targetTorque{0};
    /// Software velocity cap in internal units/s. The controller reduces
    /// the commanded torque so that |actualVelocity| stays at or below this.
    int32_t velocityLimit{0};
    /// Signed back-off distance in internal units (typically opposite sign
    /// of targetTorque). Used for fine passes; ignored for the coarse pass.
    int32_t backOffDistance{0};
    /// Stall window: position must remain "still" for this many ms before
    /// a stall is declared.
    uint32_t stallWindowMs{200};
    /// |Δposition| over the stall window must be ≤ this for a position stall.
    int32_t stallPositionThreshold{5};
    /// |actualTorque - targetTorque| must be ≤ this for a torque-saturation
    /// stall (only used when useTorqueSaturationDetection is true).
    int16_t stallTorqueTolerance{50};
    /// Per-pass timeout in ms. If no stall is detected within this time,
    /// PassTimeout is raised.
    uint32_t passTimeoutMs{20000};
    /// Settle time at the home position after this pass completes (ms).
    uint32_t settleMs{0};
};

/**
 * @brief Torque homing configuration
 */
struct TorqueHomingConfig {
    /// Coarse approach pass (always executed).
    TorqueHomingPassConfig coarsePass;
    /// Fine passes executed after the coarse stall (back-off & re-approach).
    /// Empty means no fine passes; homing completes after the coarse stall.
    std::vector<TorqueHomingPassConfig> finePasses;

    /// Enable position-derivative stall detector.
    bool usePositionStallDetection{true};
    /// Enable torque-saturation stall detector.
    bool useTorqueSaturationDetection{false};

    /// Software maximum position deviation from the start-of-pass anchor.
    /// If |actualPosition - anchor| exceeds this, PositionDeviationExceeded
    /// is raised. Bounds runaway if the axis never stalls.
    int32_t maxPositionDeviation{0};
    /// Enable the software position-deviation limit.
    bool usePositionDeviationLimit{true};

    /// Velocity-limiting strategy.
    VelocityLimitStrategy velocityStrategy{VelocityLimitStrategy::ProportionalTorqueReduction};
    /// Fraction of velocityLimit at which proportional torque reduction
    /// begins (0.0–1.0). Only used by ProportionalTorqueReduction.
    double softStartFraction{0.9};

    /// Save the operating mode on start() and restore it on completion,
    /// abort, or error.
    bool restorePreviousMode{true};

    /// Offset added to the actual position at Attained before publishing
    /// the home position.
    int32_t homeOffset{0};
    /// If true, call the setHomePosition callback at Attained.
    bool setHomePosition{true};

    /// Global timeout for the entire homing sequence in ms.
    uint64_t totalTimeoutMs{60000};

    /// Stop motion (zero torque) when entering the Error state.
    bool stopOnError{true};
    /// Raise a drive fault via the backend when entering the Error state.
    /// (Currently advisory; the controller does not force a CiA 402 fault
    ///  transition — it stops torque and notifies via onError.)
    bool faultOnError{false};
};

/**
 * @brief Callbacks used by the torque homing controller
 *
 * If a callback is unset, the controller falls back to the equivalent
 * DriveBackend method when one is available. Motion/state callbacks
 * (getActualPosition, setTargetTorque, setOperatingMode, stopMotion)
 * are required either as callbacks or via the backend.
 */
struct TorqueHomingCallbacks {
    std::function<int32_t()> getActualPosition;
    std::function<int32_t()> getActualVelocity;
    std::function<int16_t()> getActualTorque;
    std::function<bool()> hasDriveFault;

    std::function<void(int16_t)> setTargetTorque;
    std::function<bool(OperatingMode)> setOperatingMode;
    std::function<OperatingMode()> getOperatingMode;
    std::function<void()> stopMotion;
    std::function<void(int32_t)> setHomePosition;

    /// @brief Time source in milliseconds. If unset, the controller uses
    ///        std::chrono::steady_clock. Supply a simulated clock in tests
    ///        so timeouts and the stall window align with simulated motion.
    std::function<uint64_t()> getTimeMs;

    std::function<void(TorqueHomingState)> onStateChange;
    std::function<void(TorqueHomingError, const std::string&)> onError;
    std::function<void(int32_t homePosition)> onComplete;
};

/**
 * @brief Torque-based sensorless homing controller
 *
 * Drives a CiA 402 axis in CST mode against a mechanical stop, detects
 * the stop via stall, and performs configurable back-off & re-approach
 * fine passes. Call update() once per control cycle.
 */
class TorqueHomingController {
public:
    TorqueHomingController();
    ~TorqueHomingController() = default;

    // Non-copyable
    TorqueHomingController(const TorqueHomingController&) = delete;
    TorqueHomingController& operator=(const TorqueHomingController&) = delete;

    // ========================================================================
    // Configuration
    // ========================================================================

    /// @brief Set the drive backend (used for callback fallbacks and mode
    ///        restore). Optional if all required callbacks are populated.
    void setBackend(DriveBackendPtr backend) { m_backend = backend; }

    /// @brief Set homing configuration. Must be called before start().
    void setConfig(const TorqueHomingConfig& config) { m_config = config; }
    const TorqueHomingConfig& getConfig() const { return m_config; }

    /// @brief Set callbacks. Required: getActualPosition, setTargetTorque,
    ///        setOperatingMode, stopMotion (or a backend providing them).
    void setCallbacks(const TorqueHomingCallbacks& callbacks) { m_callbacks = callbacks; }

    // ========================================================================
    // Operations
    // ========================================================================

    /**
     * @brief Start the homing sequence.
     * @param direction +1, -1, or 0. If 0, the sign of
     *        coarsePass.targetTorque is used. The sign of every pass's
     *        targetTorque and backOffDistance is forced to match this
     *        direction so a single `start()` call is consistent.
     * @return true if the sequence was started.
     */
    bool start(int8_t direction = 0);

    /// @brief Abort the homing sequence. Restores the previous operating
    ///        mode and transitions to Error with Aborted.
    void abort();

    /// @brief Reset to Idle, clearing error state. Does not restore the
    ///        operating mode (use abort() first if a sequence is active).
    void reset();

    /**
     * @brief Cyclic update. Call once per control cycle.
     * @return true while homing is active (not Idle/Attained/Error); false
     *         when the sequence has finished.
     */
    bool update();

    // ========================================================================
    // Status
    // ========================================================================

    TorqueHomingState getState() const { return m_state; }
    TorqueHomingError getLastError() const { return m_lastError; }
    const std::string& getLastErrorMessage() const { return m_lastErrorMsg; }
    int32_t getHomePosition() const { return m_homePosition; }
    /// 0 = coarse pass, 1..N = fine pass index (1-based).
    int getCurrentPass() const { return m_currentPass; }
    bool isComplete() const { return m_state == TorqueHomingState::Attained; }
    bool hasError() const { return m_state == TorqueHomingState::Error; }
    bool isIdle() const { return m_state == TorqueHomingState::Idle; }

private:
    // State handlers
    void handleConfiguring();
    void handleApproach(const TorqueHomingPassConfig& pc);
    void handleBackOff();
    void handleSettling();
    void handleAttained();
    void handleError();

    // Helpers
    int16_t applyVelocityLimit(int16_t rawTorque, int32_t actualVel,
                                int32_t velLimit) const;
    bool detectStall(const TorqueHomingPassConfig& pc);
    void resetStallWindow(int32_t initialPosition);
    bool checkPositionDeviation(int32_t actualPos);
    void startBackOff(const TorqueHomingPassConfig& pc);
    void startNextPass();
    void completeHoming();
    void complete(bool success, TorqueHomingError err = TorqueHomingError::None,
                   const std::string& msg = "");
    void setState(TorqueHomingState s);
    void setError(TorqueHomingError err, const std::string& msg);
    uint64_t getTimeMs() const;

    // Callback / backend accessors with fallback
    int32_t callGetActualPosition();
    int32_t callGetActualVelocity();
    int16_t callGetActualTorque();
    bool callHasDriveFault();
    void callSetTargetTorque(int16_t torque);
    bool callSetOperatingMode(OperatingMode mode);
    OperatingMode callGetOperatingMode();
    void callStopMotion();
    void callSetHomePosition(int32_t pos);
    bool validateConfig();

    DriveBackendPtr m_backend;
    TorqueHomingConfig m_config;
    TorqueHomingCallbacks m_callbacks;

    TorqueHomingState m_state{TorqueHomingState::Idle};
    TorqueHomingError m_lastError{TorqueHomingError::None};
    std::string m_lastErrorMsg;
    int32_t m_homePosition{0};
    int m_currentPass{-1};      ///< -1 = coarse, 0..N-1 = fine index
    int8_t m_direction{1};

    // Timing
    uint64_t m_startTimeMs{0};
    uint64_t m_stateStartTimeMs{0};

    // Stall detection ring buffer (positions sampled each cycle)
    std::vector<int32_t> m_positionWindow;
    std::vector<uint64_t> m_timeWindow;
    size_t m_windowIndex{0};
    bool m_windowFilled{false};

    // Position-deviation anchor (reset at entry of each approach/back-off)
    int32_t m_positionAnchor{0};

    // Back-off tracking
    int32_t m_backOffStartPos{0};
    int32_t m_backOffTargetDelta{0};   ///< absolute distance to travel
    int16_t m_backOffTorque{0};

    // Approach tracking
    int16_t m_activeTorque{0};         ///< signed torque currently commanded

    // Mode restore
    OperatingMode m_previousMode{OperatingMode::NoMode};
    bool m_modeSaved{false};
};

} // namespace CiA402
