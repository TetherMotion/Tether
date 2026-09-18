#include "tether/drives/NexcobotESC211/SafetyStateMachineManager.hpp"

#include <cstring>
#include <thread>

#include "tether/drives/NexcobotESC211/SystemStateName.hpp"
#include "logging/Logger.hpp"
#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"
#include "tether/drives/NexcobotESC211/Registers/BulkData.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;
namespace BulkData   = EtherCAT::Drives::Registers::NexcobotESC211::BulkData;

// --- Constructor ---

SafetyStateMachineManager::SafetyStateMachineManager(EtherCAT::Slave& slave,
                                                     Config config,
                                                     const char* tag)
    : slave_(slave), config_(config), tag_(tag),
      channel_(slave,
               ControlCommandChannel::Config{
                   .command_timeout = config.command_timeout,
                   .poll_interval = config.poll_interval,
                   .pre_command_reset_delay = std::chrono::milliseconds(100),
                   .post_command_reset = false,  // SSM does NOT reset after commands
               },
               tag),
      config_manager_(slave,
                      ConfigManager::Config{
                          .command_timeout = config.command_timeout,
                          .crc_timeout = std::chrono::milliseconds(5000),
                          .crc_poll_interval = std::chrono::milliseconds(100),
                          .post_flash_load_settle = std::chrono::milliseconds(500),
                          .post_command_reset = false,
                      },
                      tag) {}

// --- Result conversion ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::toResult(ControlCommandChannel::Result r) {
    switch (r) {
        case ControlCommandChannel::Result::Success:  return Result::Success;
        case ControlCommandChannel::Result::Failed:   return Result::Failed;
        case ControlCommandChannel::Result::Timeout:  return Result::Timeout;
        case ControlCommandChannel::Result::SDOError: return Result::SDOError;
    }
    return Result::Failed;
}

// --- startMonitoring ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::startMonitoring(StartMode mode) {
    monitoring_.store(false);

    // Manufacturer requirement: write CMD=0 and wait 100 ms before any command.
    if (!channel_.resetResponseToStandby()) {
        return Result::SDOError;
    }

    // Before issuing cmd 10/11, ensure FNI/RSP/SDD config data is loaded.
    if (!config_manager_.ensureLoaded()) {
        return Result::Failed;
    }

    if (mode == StartMode::FSoEConnectionOnly) {
        // Command 11 (Start FSoE connection only).
        //
        // Manufacturer note (Nexcobot): command 11 is fire-and-forget —
        // it does NOT produce a Done/Failed response in 0xF100:0x02.
        // The device starts the FSoE connection asynchronously and the
        // command response register remains in Standby.  Therefore we
        // must NOT wait for Done here; doing so causes a 10-second timeout.
        if (!channel_.sendCommand(UserSystem::ControlCommandCode::StartFSoEConnectionOnly)) {
            return Result::SDOError;
        }
    } else {
        // Command 10 (Start FSoE and safety application).
        //
        // Unlike command 11, command 10 DOES produce a Done/Failed response.
        // However, in the black-channel relay setup the FSoE state machine
        // may already be running, so the response may remain "Ongoing"
        // indefinitely.  We accept either "Ongoing" or "Done" as success.
        uint32_t retries = 0;
        while (true) {
            if (!channel_.sendCommand(UserSystem::ControlCommandCode::StartFSoEAndSafetyApp)) {
                return Result::SDOError;
            }
            auto r = channel_.waitForResponse(
                UserSystem::ControlCommandCode::StartFSoEAndSafetyApp,
                ControlCommandChannel::matchOngoingOrDone,
                config_.command_timeout);
            if (r == ControlCommandChannel::Result::Success) {
                break;
            }
            if (r == ControlCommandChannel::Result::Failed) {
                TETHER_LOGE(tag_, "Start FSoE and safety app command failed");
                return Result::Failed;
            }
            if (r == ControlCommandChannel::Result::SDOError) {
                return Result::SDOError;
            }
            // Timeout — retry if configured.
            if (retries >= config_.max_command_retries) {
                TETHER_LOGE(tag_, "Timeout waiting for Ongoing/Done on start command 10");
                return Result::Timeout;
            }
            ++retries;
            TETHER_LOGW(tag_, "Retrying start command 10 ({}/{})",
                        retries, config_.max_command_retries);
        }
    }

    // After command 10/11 is issued (and its response accepted for cmd 10),
    // the FSoE/safety application is not yet running — poll 0xF101 until
    // the system state reaches MONITORING before continuing.
    const Result wait_result = waitForMonitoringState();
    if (wait_result != Result::Success) {
        return wait_result;
    }

    monitoring_.store(true);
    TETHER_LOGI(tag_, "Safety state machine start command sent successfully");
    return Result::Success;
}

// --- stopMonitoring ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::stopMonitoring() {
    // Like startMonitoring's command-10 path, accept "Ongoing" as success.
    // In the relay topology, the response stays at "Ongoing" indefinitely.
    // Waiting for "Done" would monopolize the EtherCAT transport and starve
    // the PDO exchange thread, causing the ESC211's sync-manager watchdog to
    // expire (AL status code 0x001B).
    auto r = channel_.sendAndWait(
        UserSystem::ControlCommandCode::StopSafetyAppAndFSoE,
        ControlCommandChannel::matchOngoingOrDone);
    if (r == ControlCommandChannel::Result::Success) {
        monitoring_.store(false);
        TETHER_LOGI(tag_, "Safety application stopped");
        return Result::Success;
    }
    return toResult(r);
}

// --- clearApplicationErrorState ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::clearApplicationErrorState() {
    TETHER_LOGI(tag_, "Clearing application error state (command 7)...");
    auto r = channel_.sendAndWait(
        UserSystem::ControlCommandCode::ClearApplicationErrorState,
        ControlCommandChannel::matchDone);
    if (r == ControlCommandChannel::Result::Success) {
        TETHER_LOGI(tag_, "Application error state cleared successfully");
        return Result::Success;
    }
    return toResult(r);
}

// --- clearErrorState ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::clearErrorState() {
    TETHER_LOGI(tag_, "Clearing error state (command 8)...");
    auto r = channel_.sendAndWait(
        UserSystem::ControlCommandCode::ClearLastErrorCode,
        ControlCommandChannel::matchDone);
    if (r == ControlCommandChannel::Result::Success) {
        TETHER_LOGI(tag_, "Error state cleared successfully");
        return Result::Success;
    }
    return toResult(r);
}

// --- flashMemoryReset ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::flashMemoryReset() {
    TETHER_LOGI(tag_, "Sending flash memory reset (command 9001)...");
    auto r = channel_.sendAndWait(
        UserSystem::ControlCommandCode::FlashMemoryReset,
        ControlCommandChannel::matchDone);
    if (r == ControlCommandChannel::Result::Success) {
        TETHER_LOGI(tag_, "Flash memory reset completed successfully");
        return Result::Success;
    }
    return toResult(r);
}

// --- Accessors ---

bool SafetyStateMachineManager::isMonitoring() const {
    return monitoring_.load();
}

uint32_t SafetyStateMachineManager::lastCommandResponse() const {
    return channel_.lastResponse();
}

uint32_t SafetyStateMachineManager::lastSystemState() const {
    return last_system_state_.load();
}

// --- setPollObserver / setStopToken ---

void SafetyStateMachineManager::setPollObserver(std::function<void()> observer) {
    poll_observer_ = observer;
    channel_.setPollObserver(std::move(observer));
}

void SafetyStateMachineManager::setStopToken(std::stop_token token) {
    stop_token_ = token;
    channel_.setStopToken(token);
    config_manager_.setStopToken(token);
}

// --- Strings ---

const char* SafetyStateMachineManager::resultToString(Result result) {
    switch (result) {
        case Result::Success:      return "Success";
        case Result::Failed:       return "Failed";
        case Result::Timeout:      return "Timeout";
        case Result::SDOError:     return "SDOError";
        case Result::InvalidState: return "InvalidState";
    }
    return "Unknown";
}

// --- Private helpers ---

SafetyStateMachineManager::Result
SafetyStateMachineManager::waitForMonitoringState() {
    const uint32_t expected = config_.expected_monitoring_state;
    if (expected == 0xFFFFFFFFu) {
        return Result::Success;  // state check disabled
    }

    TETHER_LOGI(tag_, "Waiting for system state {} (0x{:08X})...",
                systemStateName(expected), expected);

    uint32_t last_logged = 0xFFFFFFFF;
    const auto r = channel_.pollUntil(
        UserSystem::SystemCurrentStateIndex, 0x00,
        [expected](uint32_t state) { return state == expected; },
        config_.monitoring_state_timeout,
        [this, &last_logged](uint32_t state) {
            last_system_state_.store(state);
            if (state != last_logged) {
                last_logged = state;
                TETHER_LOGI(tag_, "System state (0xF101) = 0x{:08X} [{}]",
                            state, systemStateName(state));
            }
        });

    if (r == ControlCommandChannel::Result::Success) {
        return Result::Success;
    }
    // The pre-Tether implementation reported a stop-request abort as
    // Failed (pollUntil maps it to Timeout) — keep that behavior.
    if (stop_token_.stop_requested()) {
        TETHER_LOGW(tag_, "Monitoring-state wait aborted (stop requested)");
        return Result::Failed;
    }
    return toResult(r);
}

bool SafetyStateMachineManager::pollSystemState(uint32_t& out) {
    auto err = slave_.sdoReadU32(UserSystem::SystemCurrentStateIndex, 0x00, out);
    if (err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(tag_, "Failed to read system state: {}",
                    EtherCAT::slaveErrorToString(err));
        return false;
    }
    last_system_state_.store(out);
    TETHER_LOGI(tag_, "System state (0xF101) = 0x{:08X} [{}]", out, systemStateName(out));
    return true;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
