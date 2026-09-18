#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <stop_token>
#include <string>

#include "tether/drives/NexcobotESC211/ConfigManager.hpp"
#include "tether/drives/NexcobotESC211/ControlCommandChannel.hpp"
#include "tether/drives/NexcobotESC211/Registers/UserSystem.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/**
 * @brief Manages the ESC211 safety-application state machine via the 0xF100
 *        Control Command object.
 *
 * Manufacturer requirement (Nexcobot): BEFORE every command written to
 * 0xF100:0x01, the host MUST write CMD=0 (ResetResponseToStandby) to
 * 0xF100:0x01 and wait 100 ms.  If the response register is not in Standby
 * when a command is sent, the device will silently ignore the command.
 * The reset is NOT performed after the command — only before.
 *
 * This class delegates the low-level command protocol to
 * ControlCommandChannel and the config-data lifecycle to ConfigManager.
 * The public API is unchanged from the original implementation.
 *
 * Implements the documented workflow:
 *   1. Write CMD=0 and wait 100 ms — manufacturer requirement (pre-reset).
 *   2. Write the start command:
 *      - FSoEConnectionOnly mode: command 11 (fire-and-forget, no Done
 *        response — the device starts the FSoE connection asynchronously).
 *      - FSoEAndSafetyApp mode: command 10 (produces a Done/Failed response,
 *        waits for Done).
 *   3. Poll the system state (0xF101) until it reaches MONITORING (6)
 *      before returning — the FSoE/safety application is not running until
 *      then, so continuing earlier is a protocol error.
 *   4. The safety state machine commands should be issued only after the
 *      device is in OP mode with PDO exchange active — this class does NOT
 *      perform the OP transition or PDO setup.
 *   5. After success, expose helpers to rapidly read the FSoE0 frame/safe-data
 *      registers via SDO.
 */
class SafetyStateMachineManager {
public:
    enum class Result {
        Success,
        Failed,
        Timeout,
        SDOError,
        InvalidState,
    };

    /// Start mode for startMonitoring().
    ///   FSoEConnectionOnly  — command 11: start FSoE connection only (fire-and-forget).
    ///   FSoEAndSafetyApp    — command 10: start FSoE + safety application (waits for Done).
    enum class StartMode {
        FSoEConnectionOnly,
        FSoEAndSafetyApp,
    };

    struct Config {
        std::chrono::milliseconds command_timeout;
        std::chrono::milliseconds poll_interval;
        std::chrono::milliseconds fsoe0_read_timeout;
        // After issuing command 10/11, poll system state (0xF101) until it
        // matches this value (default 6 = MONITORING).  Set 0xFFFFFFFF to
        // skip the wait entirely.
        uint32_t expected_monitoring_state = 6;
        // Timeout for the post-start MONITORING state wait.
        std::chrono::milliseconds monitoring_state_timeout;
        uint32_t max_command_retries = 0;
        /// Admin password entered on the device (0xF105) before privileged
        /// commands; enables automatic login+retry on access-denied.
        std::string admin_password;
        Config()
            : command_timeout(10000)
            , poll_interval(20)
            , fsoe0_read_timeout(1000)
            , monitoring_state_timeout(15000) {}
    };

    SafetyStateMachineManager(EtherCAT::Slave& slave,
                              Config config = {},
                              const char* tag = "ESC211SafetyStateMachineManager");

    // Start monitoring.  In FSoEConnectionOnly mode, sends command 11
    // (fire-and-forget).  In FSoEAndSafetyApp mode, sends command 10 and
    // waits for Done.  Both modes write CMD=0 + 100 ms wait before the
    // command (manufacturer requirement), and both wait for the system
    // state (0xF101) to reach MONITORING before returning.
    Result startMonitoring(StartMode mode = StartMode::FSoEConnectionOnly);

    // Stop the safety application (command 9).
    Result stopMonitoring();

    // Clear the application error state (command 7).
    // Transitions the device from APPERR to READY.
    Result clearApplicationErrorState();

    // Clear the error state (command 8, ClearLastErrorCode).
    // Used to clear a latched system error (0xF102) before starting.
    Result clearErrorState();

    // Perform a flash memory reset (command 9001).
    // This resets the device's flash memory to factory defaults.
    // The caller is responsible for obtaining user confirmation before calling.
    Result flashMemoryReset();

    // Returns true after startMonitoring() returned Success.
    bool isMonitoring() const;

    // Last observed command response value (0xF100:02).
    uint32_t lastCommandResponse() const;

    // Last observed system state (0xF101).
    uint32_t lastSystemState() const;

    // Human-readable result string.
    static const char* resultToString(Result result);

    /// Set a callback invoked after each command-response poll (0xF100:0x02).
    /// Useful for logging PDO/TxPDU contents alongside the poll to correlate
    /// the safety state machine progress with live FSoE frame traffic.
    void setPollObserver(std::function<void()> observer);

    /// Set a stop_token for cooperative cancellation.  When stop is
    /// requested on the token, all polling loops abort immediately.
    void setStopToken(std::stop_token token);

    /// Set the admin password on both command channels (0xF100 command
    /// channel and the ConfigManager's channel).  Overrides
    /// Config::admin_password.
    void setAdminPassword(std::string password);

private:
    using ControlCommandCode = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode;
    using CommandResponseCode = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::CommandResponseCode;

    EtherCAT::Slave& slave_;
    Config config_;
    const char* tag_;

    // Delegated subsystems.
    ControlCommandChannel channel_;
    ConfigManager config_manager_;

    std::atomic<bool> monitoring_{false};
    std::atomic<uint32_t> last_system_state_{0xFFFFFFFF};
    std::function<void()> poll_observer_;
    std::stop_token stop_token_{};

    bool pollSystemState(uint32_t& out);
    // Poll 0xF101 until the system state reaches
    // config_.expected_monitoring_state (MONITORING by default).
    Result waitForMonitoringState();
    // Convert ControlCommandChannel::Result to our Result.
    static Result toResult(ControlCommandChannel::Result r);
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
