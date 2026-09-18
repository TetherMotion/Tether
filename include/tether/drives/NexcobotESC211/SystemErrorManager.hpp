#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/**
 * @brief Monitors the ESC211 current system error state (0xF102 / 0xF103).
 *
 * Runs a background thread that periodically (every 1000 ms) reads:
 *   - 0xF102:0x00 — System Error Code (Integer32, current error)
 *   - 0xF103:0x00 — System Error Message (VisibleString, up to 256 bytes)
 *                   — only read when 0xF102 is non-zero
 *
 * Non-zero error codes are mapped to human-readable names/descriptions via
 * NexcobotESC211Errors and logged.
 *
 * 0xF104 (Last Error Code A/B, subindexes 1 and 2) is NOT read by the
 * active polling path — it holds error codes from the *last boot*, not the
 * current error, and its subindexes 1/2 are rejected by some firmware with
 * SDO abort 0x06020000.
 *
 * The synchronous check() method can be used for a one-shot verification
 * before starting the safety state machine.  start()/stop() control the
 * background polling thread.
 */
class SystemErrorManager {
public:
    enum class Result {
        NoError,
        AppError,
        SDOError,
    };

    SystemErrorManager(EtherCAT::Slave& slave,
                       std::chrono::milliseconds poll_interval = std::chrono::milliseconds(250),
                       const char* tag = "ESC211SystemErrorManager");
    ~SystemErrorManager();

    SystemErrorManager(const SystemErrorManager&) = delete;
    SystemErrorManager& operator=(const SystemErrorManager&) = delete;

    /**
     * @brief One-shot synchronous check of 0xF102 (and 0xF103 if non-zero).
     *
     * @return Result::NoError if the system error code is zero;
     *         Result::AppError if a non-zero error code was read;
     *         Result::SDOError if the SDO read itself failed.
     */
    Result check();

    /** Start the background polling thread. */
    void start();

    /** Stop the background polling thread. */
    void stop();

    /** Whether the background thread is running. */
    bool isRunning() const;

    /**
     * @brief Check and consume a restart request.
     *
     * Returns true if the system state (0xF101) transitioned to the
     * configured trigger state (default 3) since the last call.
     * The flag is atomically cleared on read.
     */
    bool consumeRestartRequest();

    /**
     * @brief Set the system state value that triggers a restart request.
     *
     * When the background poll sees 0xF101 transition to this value
     * (and it was not this value on the previous poll), the restart
     * flag is set.  Use 0xFFFFFFFF to disable (default).
     */
    void setRestartTriggerState(uint32_t state);

    /** True if the last check found a non-zero current error code (0xF102). */
    bool hasError() const;

    /** Last system error code read from 0xF102:0x00 (0 if none). */
    int32_t lastErrorCode() const;

    /** Last system current state read from 0xF101:0x00. */
    uint32_t lastSystemState() const;

    /** Last system error message read from 0xF103:0x00 (empty if none). */
    std::string lastErrorMessage() const;

    /** Human-readable result string. */
    static const char* resultToString(Result result);

private:
    EtherCAT::Slave& slave_;
    std::chrono::milliseconds poll_interval_;
    const char* tag_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    std::atomic<bool> restart_requested_{false};
    std::atomic<uint32_t> restart_trigger_state_{0xFFFFFFFF};
    std::atomic<uint32_t> prev_system_state_{0xFFFFFFFF};
    std::chrono::steady_clock::time_point last_restart_fire_{};

    std::atomic<int32_t> last_error_code_{0};
    std::atomic<uint32_t> last_system_state_{0};
    std::atomic<int32_t> last_logged_error_code_{0};
    mutable std::string last_error_message_;

    void pollLoop();
    void readAndLog();
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
