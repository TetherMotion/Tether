#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <stop_token>
#include <string>

#include "tether/drives/NexcobotESC211/Registers/UserSystem.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Low-level transport for the ESC211 0xF100 control command protocol.
///
/// Manufacturer requirement (Nexcobot): BEFORE every command written to
/// 0xF100:0x01, the host MUST write CMD=0 (ResetResponseToStandby) and
/// wait 100 ms.  If the response register is not in Standby when a command
/// is sent, the device will silently ignore the command.
///
/// Thin ESC211-specific layer over EtherCAT::SdoCommandChannel (Tether):
/// the generic channel owns the reset → send → poll machinery; this class
/// contributes the 0xF100 object indices, the CMD=0 manufacturer quirk,
/// the ESC211 command/response enums and the ESC211 matchers.
///
/// Command-specific response logic (e.g. command 4 returns Standby+errCode=0
/// instead of Done) is handled via a ResponseMatcher callable.
class ControlCommandChannel {
public:
    using ControlCommandCode = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode;
    using CommandResponseCode = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::CommandResponseCode;

    /// Result of a command operation.
    enum class Result {
        Success,
        Failed,     // Device reported Failed (0xF100:0x02 == 3)
        Timeout,    // Response did not match within the timeout
        SDOError,   // SDO read/write failed
    };

    struct Config {
        std::chrono::milliseconds command_timeout{10000};
        std::chrono::milliseconds poll_interval{20};
        std::chrono::milliseconds pre_command_reset_delay{100};
        /// If true, send CMD=0 after the command completes (post-command reset).
        /// Some callers (esc211_fni.cpp) do this; SafetyStateMachineManager does not.
        bool post_command_reset{false};
        /// Admin password for privileged commands (config activation,
        /// flash save, etc.).  When non-empty, sendAndWait() performs
        /// adminLogin() automatically and retries once when a command is
        /// rejected with the access-denied error code.
        std::string admin_password;
    };

    /// Error code reported via 0xF100:0x03 when a command is denied
    /// (device debug log: "Command denied. Please login with password.").
    static constexpr uint32_t kAccessDeniedError = 0xFF676918;

    /// A callable that determines whether a polled response value indicates
    /// success for a given command.  Receives (cmdCode, responseValue, slave)
    /// and returns true if the response is a success.
    ///
    /// Default matcher: response == Done (2).
    /// Command 4 matcher: response == Standby (0) && errCode (0xF100:0x03) == 0.
    /// Command 10/11 matcher: response == Ongoing (1) || Done (2).
    using ResponseMatcher = EtherCAT::SdoCommandChannel::ResponseMatcher;

    ControlCommandChannel(EtherCAT::Slave& slave,
                          Config config,
                          const char* tag = "ESC211ControlCommandChannel");

    // --- Low-level primitives ---

    /// Write CMD=0 (ResetResponseToStandby) and wait pre_command_reset_delay.
    /// Called automatically by sendAndWait(), or can be called manually.
    bool resetResponseToStandby();

    /// Write a command code to 0xF100:0x01 (no reset, no polling).
    bool sendCommand(ControlCommandCode cmd);

    /// Poll 0xF100:0x02 once.  Returns true on successful read, false on error.
    /// On success, `out` is set to the response value.
    bool pollResponse(uint32_t& out);

    /// Poll 0xF100:0x02 until the matcher returns true or the timeout expires.
    /// If the response is Failed (3), returns Result::Failed immediately.
    Result waitForResponse(ControlCommandCode cmd,
                           const ResponseMatcher& matcher,
                           std::chrono::milliseconds timeout);

    // --- Convenience ---

    /// Full sequence: resetResponseToStandby() → sendCommand(cmd) →
    /// waitForResponse(cmd, matcher, timeout) → optional post_command_reset.
    Result sendAndWait(ControlCommandCode cmd,
                       const ResponseMatcher& matcher = {},
                       std::chrono::milliseconds timeout = {});

    /// Read the response error code from 0xF100:0x03.
    uint32_t readErrorCode();

    // --- Admin login (password) ---
    //
    // Privileged commands (e.g. config activation cmd 3/4, flash save
    // cmd 5/6) are rejected with "Command denied. Please login with
    // password." until an admin password has been entered.  The password
    // is written to the password-entry object 0xF105 and the resulting
    // privilege level is reported by the Admin Mode flag 0xF610.
    //
    // adminLogin() procedure:
    //   1. Write the password bytes to 0xF105:0x01 (visible-string write).
    //      Fallbacks if that write aborts: subindex 0x00, then a U32
    //      numeric interpretation of the password string.
    //   2. Poll 0xF610:0x00 (Admin Mode) until it reports a nonzero
    //      privilege level (2 s).  If 0xF610 is not readable the write
    //      is still considered accepted — the retried command is the
    //      verdict.

    /// Set/clear the admin password (empty = disable auto-login).
    void setAdminPassword(std::string password) {
        admin_password_ = std::move(password);
        if (admin_password_.empty()) admin_logged_in_ = false;
    }
    const std::string& adminPassword() const { return admin_password_; }

    /// Enter the configured admin password on the device (see procedure
    /// above).  Returns true when the password was accepted.
    bool adminLogin();
    bool isAdminLoggedIn() const { return admin_logged_in_; }

    // --- Generic SDO polling (uses this channel's timing/cancellation) ---

    /// Poll an arbitrary U32 register until `match(value)` returns true.
    /// SDO read errors are retried; the registered poll observer is
    /// invoked after each read; the stop token aborts the wait.
    /// `onValue` (optional) receives every successfully-read value.
    Result pollUntil(uint16_t index, uint8_t subindex,
                     const std::function<bool(uint32_t)>& match,
                     std::chrono::milliseconds timeout,
                     const std::function<void(uint32_t)>& onValue = {});

    // --- Accessors ---

    uint32_t lastResponse() const { return channel_.lastResponse(); }
    void setStopToken(std::stop_token token) { channel_.setStopToken(std::move(token)); }
    void setPollObserver(std::function<void()> observer) {
        channel_.setPollObserver(std::move(observer));
    }

    /// Human-readable result string.
    static const char* resultToString(Result r);

    /// Human-readable command name.
    static const char* commandName(ControlCommandCode cmd);

    /// Human-readable response name.
    static const char* responseName(uint32_t code);

    // --- Pre-built matchers ---

    /// Default: response == Done (2).
    static bool matchDone(uint32_t cmdCode, uint32_t response, EtherCAT::Slave& slave);

    /// Command 4 (DownloadRSPAndSDDByObject): response == Standby (0) with
    /// errCode (0xF100:0x03) == 0, OR response == Done (2).
    static bool matchCmd4(uint32_t cmdCode, uint32_t response, EtherCAT::Slave& slave);

    /// Command 10/11 (start/stop safety app): response == Ongoing (1) || Done (2).
    static bool matchOngoingOrDone(uint32_t cmdCode, uint32_t response, EtherCAT::Slave& slave);

private:
    static Result mapResult(EtherCAT::SdoPollResult r);

    EtherCAT::Slave& slave_;
    EtherCAT::SdoCommandChannel channel_;
    const char* tag_;
    std::string admin_password_;
    bool admin_logged_in_ = false;
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
