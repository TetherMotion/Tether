#include "tether/drives/NexcobotESC211/SystemErrorManager.hpp"

#include <array>

#include "logging/Logger.hpp"
#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"
#include "tether/drives/NexcobotESC211Errors.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;
namespace Errors = EtherCAT::Drives::ErrorCodes::NexcobotESC211;

namespace {
/// The two U32 registers read every poll cycle: system state, error code.
/// The error message string (0xF103) stays a bespoke raw read — it is only
/// fetched when the error code is non-zero.
constexpr std::array<EtherCAT::SdoObjectEntry, 2> kPollObjects = {{
    {UserSystem::SystemCurrentStateIndex, 0x00, 4},
    {UserSystem::SystemErrorCodeIndex,    0x00, 4},
}};
constexpr size_t kStateObj = 0;
constexpr size_t kErrObj   = 1;
} // namespace

SystemErrorManager::SystemErrorManager(EtherCAT::Slave& slave,
                                       std::chrono::milliseconds poll_interval,
                                       const char* tag)
    : slave_(slave), poll_interval_(poll_interval), tag_(tag) {}

SystemErrorManager::~SystemErrorManager() {
    stop();
}

SystemErrorManager::Result SystemErrorManager::check() {
    readAndLog();
    return last_error_code_.load() != 0 ? Result::AppError : Result::NoError;
}

void SystemErrorManager::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&SystemErrorManager::pollLoop, this);
}

void SystemErrorManager::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

bool SystemErrorManager::isRunning() const {
    return running_.load();
}

bool SystemErrorManager::consumeRestartRequest() {
    return restart_requested_.exchange(false);
}

void SystemErrorManager::setRestartTriggerState(uint32_t state) {
    restart_trigger_state_.store(state);
}

bool SystemErrorManager::hasError() const {
    return last_error_code_.load() != 0;
}

int32_t SystemErrorManager::lastErrorCode() const {
    return last_error_code_.load();
}

uint32_t SystemErrorManager::lastSystemState() const {
    return last_system_state_.load();
}

std::string SystemErrorManager::lastErrorMessage() const {
    return last_error_message_;
}

const char* SystemErrorManager::resultToString(Result result) {
    switch (result) {
        case Result::NoError:   return "NoError";
        case Result::AppError:  return "AppError";
        case Result::SDOError:  return "SDOError";
    }
    return "Unknown";
}

// ---- private ----

void SystemErrorManager::pollLoop() {
    TETHER_LOGI(tag_, "Background polling started (interval={} ms)",
                static_cast<long long>(poll_interval_.count()));
    while (running_.load()) {
        readAndLog();
        auto next = std::chrono::steady_clock::now() + poll_interval_;
        while (running_.load() && std::chrono::steady_clock::now() < next) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    TETHER_LOGI(tag_, "Background polling stopped");
}

void SystemErrorManager::readAndLog() {
    // --- 0xF101 (state) + 0xF102 (error code) via the generic table poll ---
    std::array<uint32_t, kPollObjects.size()> values{};
    std::array<bool,     kPollObjects.size()> ok{};
    EtherCAT::pollSdoObjects(
        slave_, kPollObjects, values, ok, {},
        [this](size_t i, const EtherCAT::SdoObjectEntry& e,
               EtherCAT::SlaveError err) {
            TETHER_LOGE(tag_, "Failed to read {} (0x{:04X}): {}",
                        i == kStateObj ? "system current state"
                                       : "system error code",
                        e.index, EtherCAT::slaveErrorToString(err));
        });

    // --- 0xF101:0x00 — System Current State ---
    const uint32_t system_state = values[kStateObj];
    if (ok[kStateObj]) {
        last_system_state_.store(system_state);

        // Check for transition to the restart trigger state (e.g. state 3).
        // Fires immediately on the edge (prev != trigger), and again every
        // 5 seconds if the state remains at the trigger value.
        uint32_t trigger = restart_trigger_state_.load();
        uint32_t prev = prev_system_state_.exchange(system_state);
        if (trigger != 0xFFFFFFFF && system_state == trigger) {
            bool fire = false;
            if (prev != trigger) {
                // Edge transition — fire immediately.
                fire = true;
            } else {
                // Still at trigger state — fire if 5 s elapsed since last fire.
                auto now = std::chrono::steady_clock::now();
                if (now - last_restart_fire_ >= std::chrono::seconds(5)) {
                    fire = true;
                }
            }
            if (fire) {
                TETHER_LOGI(tag_, "System state is {} — setting restart request", system_state);
                restart_requested_.store(true);
                last_restart_fire_ = std::chrono::steady_clock::now();
            }
        }
    }

    // --- 0xF102:0x00 — System Error Code (current error) ---
    if (!ok[kErrObj]) {
        return;
    }
    const uint32_t raw_code = values[kErrObj];
    last_error_code_.store(static_cast<int32_t>(raw_code));

    // --- 0xF103:0x00 — System Error Message (only if there is an error) ---
    // F103 is a VisibleString describing the current error.  It is only
    // meaningful when 0xF102 is non-zero, so we skip the read when there
    // is no error to avoid unnecessary bus traffic.  The message read and
    // the error log are edge-triggered: they happen only when the error
    // code transitions to non-zero or changes — a latched error must not
    // spam the log and the mailbox on every poll cycle.
    const int32_t code = static_cast<int32_t>(raw_code);
    if (code == 0) {
        last_error_message_.clear();
        last_logged_error_code_.store(0);
    } else if (code != last_logged_error_code_.load()) {
        {
            char buf[256] = {};
            size_t actual = sizeof(buf);
            auto msg_err = slave_.sdoRead(UserSystem::SystemErrorMessageIndex, 0x00,
                                          buf, actual);
            if (msg_err == EtherCAT::SlaveError::Ok && actual > 0) {
                last_error_message_.assign(buf, strnlen(buf, actual));
            } else {
                last_error_message_.clear();
            }
        }
        last_logged_error_code_.store(code);

        // --- Log results ---
        auto info = Errors::NexcobotESC211Error::parse(code);
        TETHER_LOGE(tag_, "System error (0xF102): {} (0x{:08X}) -> {} [{}]: {} | {}",
                    code, raw_code,
                    info.name,
                    Errors::ErrorCategoryToString(info.category),
                    info.description,
                    info.error_handling);
        if (!last_error_message_.empty()) {
            TETHER_LOGE(tag_, "System error message (0xF103): {}", last_error_message_.c_str());
        }
    }

}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
