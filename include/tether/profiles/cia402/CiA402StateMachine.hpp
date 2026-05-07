/**
 * @file CiA402StateMachine.hpp
 * @brief CiA 402 state machine and protocol implementation
 * 
 * @details
 * Implements the complete CiA 402 drive state machine according to IEC 61800-7-201.
 * 
 * ## State Machine Diagram
 * 
 * ```
 *                    ┌─────────────────────────────────────┐
 *                    │                                     │
 *     ┌──────────────▼───────────────┐                    │
 *     │    NOT READY TO SWITCH ON    │◄───────────┐       │
 *     │         (State 0)            │            │       │
 *     └──────────────┬───────────────┘            │       │
 *                    │ (automatic)                │       │
 *     ┌──────────────▼───────────────┐            │       │
 *     │     SWITCH ON DISABLED       │◄───────────┤       │
 *     │         (State 1)            │            │       │
 *     └──────────────┬───────────────┘            │       │
 *                    │ Shutdown                   │       │
 *     ┌──────────────▼───────────────┐            │       │
 *     │    READY TO SWITCH ON        │            │       │
 *     │         (State 2)            │            │       │
 *     └──────────────┬───────────────┘            │       │
 *                    │ Switch On                  │       │
 *     ┌──────────────▼───────────────┐            │       │
 *     │        SWITCHED ON           │            │       │
 *     │         (State 3)            │────────────┤       │
 *     └──────────────┬───────────────┘            │       │
 *                    │ Enable Operation           │       │
 *     ┌──────────────▼───────────────┐            │       │
 *     │    OPERATION ENABLED         │────────────┤       │
 *     │         (State 4)            │            │       │
 *     └──────────┬───────────────────┘            │       │
 *                │ Quick Stop                     │       │
 *     ┌──────────▼───────────────────┐            │       │
 *     │      QUICK STOP ACTIVE       │────────────┘       │
 *     │         (State 5)            │                    │
 *     └──────────────────────────────┘                    │
 *                                                         │
 *     ┌──────────────────────────────┐                    │
 *     │    FAULT REACTION ACTIVE     │                    │
 *     │         (State 6)            │                    │
 *     └──────────────┬───────────────┘                    │
 *                    │ (automatic)                        │
 *     ┌──────────────▼───────────────┐                    │
 *     │           FAULT              │────────────────────┘
 *     │         (State 7)            │
 *     └──────────────────────────────┘
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "CiA402StateMachine.hpp"
 * 
 * using namespace CiA402;
 * 
 * // Create state machine with status/control word callbacks
 * StateMachine sm;
 * sm.setCallbacks(
 *     []() -> uint16_t { return read_status_word(); },
 *     [](uint16_t cw) { write_control_word(cw); }
 * );
 * 
 * // Enable drive
 * if (sm.requestState(State::OperationEnabled)) {
 *     // Drive is now enabled
 * }
 * 
 * // Quick stop
 * sm.quickStop();
 * 
 * // Reset fault
 * sm.resetFault();
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include <functional>
#include <atomic>
#include <cstdint>

namespace CiA402 {

/**
 * @brief Callback types for status/control word access
 */
using StatusWordCallback = std::function<uint16_t()>;
using ControlWordCallback = std::function<void(uint16_t)>;
using ModeCallback = std::function<void(OperatingMode)>;

/**
 * @brief State transition result
 */
enum class TransitionResult {
    Success,            ///< Transition completed
    Pending,            ///< Transition in progress
    InvalidTransition,  ///< Requested transition not allowed
    Timeout,            ///< Transition timed out
    FaultOccurred,      ///< Fault occurred during transition
};

/**
 * @brief CiA 402 State Machine
 * 
 * Manages drive state transitions according to CiA 402 specification.
 * Works with any backend that provides status/control word access.
 */
class StateMachine {
public:
    /**
     * @brief Default constructor
     */
    StateMachine();
    
    /**
     * @brief Destructor
     */
    ~StateMachine() = default;
    
    // ========================================================================
    // Callback Configuration
    // ========================================================================
    
    /**
     * @brief Set status and control word callbacks
     * 
     * @param readStatus Callback to read status word from drive
     * @param writeControl Callback to write control word to drive
     */
    void setCallbacks(StatusWordCallback readStatus, 
                      ControlWordCallback writeControl);
    
    /**
     * @brief Set mode change callback
     * 
     * @param modeCallback Called when operating mode changes
     */
    void setModeCallback(ModeCallback modeCallback);
    
    // ========================================================================
    // State Machine Operations
    // ========================================================================
    
    /**
     * @brief Update state machine from current status word
     * 
     * Call this periodically to synchronize with drive state.
     * 
     * @return Current state after update
     */
    State update();
    
    /**
     * @brief Get current state
     */
    State getCurrentState() const { return m_currentState; }
    
    /**
     * @brief Get previous state
     */
    State getPreviousState() const { return m_previousState; }
    
    /**
     * @brief Request transition to target state
     * 
     * @param targetState Desired state
     * @param timeoutMs Maximum time to wait for transition (0 = no wait)
     * @return Transition result
     * 
     * @note This performs multi-step transitions automatically.
     *       E.g., requesting OperationEnabled from SwitchOnDisabled
     *       will go through ReadyToSwitchOn and SwitchedOn.
     */
    TransitionResult requestState(State targetState, uint32_t timeoutMs = 1000);
    
    /**
     * @brief Execute single state transition
     * 
     * @param transition Transition command
     * @return true if command was sent
     */
    bool executeTransition(uint16_t controlWordMask);
    
    /**
     * @brief Quick stop the drive
     * 
     * @return true if quick stop initiated
     */
    bool quickStop();
    
    /**
     * @brief Reset fault condition
     * 
     * @return true if fault reset initiated
     */
    bool resetFault();
    
    /**
     * @brief Halt motion (controlled stop, stays enabled)
     * 
     * @param halt true to halt, false to resume
     */
    void setHalt(bool halt);
    
    // ========================================================================
    // Operating Mode
    // ========================================================================
    
    /**
     * @brief Set operating mode
     * 
     * @param mode Desired operating mode
     * @return true if mode change initiated
     * 
     * @note Mode can only be changed in certain states.
     */
    bool setOperatingMode(OperatingMode mode);
    
    /**
     * @brief Get current operating mode
     */
    OperatingMode getOperatingMode() const { return m_operatingMode; }
    
    /**
     * @brief Get mode as displayed by drive
     */
    OperatingMode getDisplayedMode() const { return m_displayedMode; }
    
    // ========================================================================
    // Status Information
    // ========================================================================
    
    /**
     * @brief Check if drive is in fault state
     */
    bool isFaulted() const;
    
    /**
     * @brief Check if drive is enabled
     */
    bool isEnabled() const;
    
    /**
     * @brief Check if target is reached
     */
    bool isTargetReached() const;
    
    /**
     * @brief Check if drive is in motion
     */
    bool isInMotion() const;
    
    /**
     * @brief Check if warning is active
     */
    bool hasWarning() const;
    
    /**
     * @brief Check if internal limit is active
     */
    bool isLimitActive() const;
    
    /**
     * @brief Get raw status word
     */
    uint16_t getStatusWord() const { return m_statusWord; }
    
    /**
     * @brief Get raw control word
     */
    uint16_t getControlWord() const { return m_controlWord; }
    
    // ========================================================================
    // Homing Status (when in homing mode)
    // ========================================================================
    
    /**
     * @brief Check if homing is attained
     */
    bool isHomingAttained() const;
    
    /**
     * @brief Check if homing error occurred
     */
    bool hasHomingError() const;
    
    /**
     * @brief Start homing sequence
     */
    bool startHoming();
    
private:
    /**
     * @brief Decode state from status word
     */
    State decodeState(uint16_t statusWord) const;
    
    /**
     * @brief Calculate control word for transition
     */
    uint16_t calculateControlWord(State currentState, State targetState) const;
    
    /**
     * @brief Get intermediate state for multi-step transition
     */
    State getIntermediateState(State current, State target) const;
    
    // Callbacks
    StatusWordCallback m_readStatus;
    ControlWordCallback m_writeControl;
    ModeCallback m_modeCallback;
    
    // State
    State m_currentState;
    State m_previousState;
    State m_targetState;
    
    // Operating mode
    OperatingMode m_operatingMode;
    OperatingMode m_displayedMode;
    
    // Status/Control words
    uint16_t m_statusWord;
    uint16_t m_controlWord;
    
    // Flags
    bool m_haltActive;
    bool m_transitionPending;
};

// ============================================================================
// Control Word Builders
// ============================================================================

/**
 * @brief Build control word for state transition
 */
namespace ControlWord {

/**
 * @brief Shutdown: go to Ready to Switch On
 */
constexpr uint16_t Shutdown() {
    return 0x0006;  // xxxx.xxxx.0xxx.0110
}

/**
 * @brief Switch On: go to Switched On
 */
constexpr uint16_t SwitchOn() {
    return 0x0007;  // xxxx.xxxx.0xxx.0111
}

/**
 * @brief Switch On + Enable: go to Operation Enabled
 */
constexpr uint16_t SwitchOnEnable() {
    return 0x000F;  // xxxx.xxxx.0xxx.1111
}

/**
 * @brief Disable Voltage: go to Switch On Disabled
 */
constexpr uint16_t DisableVoltage() {
    return 0x0000;  // xxxx.xxxx.0xxx.xx0x
}

/**
 * @brief Quick Stop
 */
constexpr uint16_t QuickStop() {
    return 0x0002;  // xxxx.xxxx.0xxx.x01x
}

/**
 * @brief Disable Operation: go to Switched On
 */
constexpr uint16_t DisableOperation() {
    return 0x0007;  // xxxx.xxxx.0xxx.0111
}

/**
 * @brief Enable Operation: go to Operation Enabled
 */
constexpr uint16_t EnableOperation() {
    return 0x000F;  // xxxx.xxxx.0xxx.1111
}

/**
 * @brief Fault Reset
 */
constexpr uint16_t FaultReset() {
    return 0x0080;  // xxxx.xxxx.1xxx.xxxx
}

/**
 * @brief Mask for state transition bits
 */
constexpr uint16_t TransitionMask() {
    return 0x008F;  // Bits 0-3 and 7
}

} // namespace ControlWord

// ============================================================================
// Status Word Decoders
// ============================================================================

namespace StatusWord {

/**
 * @brief Check if Ready to Switch On
 */
inline bool isReadyToSwitchOn(uint16_t sw) {
    return (sw & 0x006F) == 0x0021;
}

/**
 * @brief Check if Switched On
 */
inline bool isSwitchedOn(uint16_t sw) {
    return (sw & 0x006F) == 0x0023;
}

/**
 * @brief Check if Operation Enabled
 */
inline bool isOperationEnabled(uint16_t sw) {
    return (sw & 0x006F) == 0x0027;
}

/**
 * @brief Check if Fault
 */
inline bool isFault(uint16_t sw) {
    return (sw & 0x004F) == 0x0008;
}

/**
 * @brief Check if Fault Reaction Active
 */
inline bool isFaultReactionActive(uint16_t sw) {
    return (sw & 0x004F) == 0x000F;
}

/**
 * @brief Check if Switch On Disabled
 */
inline bool isSwitchOnDisabled(uint16_t sw) {
    return (sw & 0x004F) == 0x0040;
}

/**
 * @brief Check if Quick Stop Active
 */
inline bool isQuickStopActive(uint16_t sw) {
    return (sw & 0x006F) == 0x0007;
}

/**
 * @brief Check if Target Reached
 */
inline bool isTargetReached(uint16_t sw) {
    return (sw & static_cast<uint16_t>(StatusWordBit::TargetReached)) != 0;
}

/**
 * @brief Check if Warning
 */
inline bool hasWarning(uint16_t sw) {
    return (sw & static_cast<uint16_t>(StatusWordBit::Warning)) != 0;
}

/**
 * @brief Check if Internal Limit Active
 */
inline bool isLimitActive(uint16_t sw) {
    return (sw & static_cast<uint16_t>(StatusWordBit::InternalLimitActive)) != 0;
}

/**
 * @brief Check Homing Attained (HM mode)
 */
inline bool isHomingAttained(uint16_t sw) {
    return (sw & static_cast<uint16_t>(StatusWordBit::HomingAttained)) != 0;
}

/**
 * @brief Check Homing Error (HM mode)
 */
inline bool hasHomingError(uint16_t sw) {
    return (sw & static_cast<uint16_t>(StatusWordBit::HomingError)) != 0;
}

} // namespace StatusWord

} // namespace CiA402
