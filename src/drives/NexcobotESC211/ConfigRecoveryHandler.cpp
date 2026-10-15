#include "tether/drives/NexcobotESC211/ConfigRecoveryHandler.hpp"

#include "logging/Logger.hpp"

namespace EtherCAT::Drives::ESC211 {

ConfigRecoveryHandler::ConfigRecoveryHandler(ConfigManager& config_mgr,
                                             Config config,
                                             const char* tag)
    : config_mgr_(config_mgr), config_(config), tag_(tag) {}

void ConfigRecoveryHandler::storeData(std::span<const Entry> entries) {
    entries_.assign(entries.begin(), entries.end());
    TETHER_LOGI(tag_, "Stored {} config data entr{} for recovery",
                entries_.size(), entries_.size() == 1 ? "y" : "ies");
    for (const auto& e : entries_) {
        const auto& info = configDataTypeInfo(e.type);
        TETHER_LOGI(tag_, "  {}: {} sections, expected CRC=0x{:08X}",
                    info.name, e.sections.size(), e.crc.expectedCrc);
    }
}

bool ConfigRecoveryHandler::reWriteAndActivate() {
    if (entries_.empty()) {
        TETHER_LOGE(tag_, "No config data stored for recovery");
        return false;
    }

    const uint32_t attempt = recovery_count_.fetch_add(1) + 1;
    TETHER_LOGI(tag_, "Config recovery #{}: re-writing {} data type(s)...",
                attempt, entries_.size());

    // Lock SDO mutex if set (SDO channel is not internally mutex-protected).
    std::unique_lock<std::mutex> sdo_lock;
    if (sdo_mutex_) sdo_lock = std::unique_lock(*sdo_mutex_);

    // Stop token check helper.
    auto stopped = [&]() { return stop_token_.stop_requested(); };

    // 1. Re-write all sections to temp objects.
    for (const auto& e : entries_) {
        if (stopped()) {
            TETHER_LOGW(tag_, "Recovery aborted: stop requested");
            return false;
        }
        const auto& info = configDataTypeInfo(e.type);
        TETHER_LOGI(tag_, "Re-writing {} sections to 0x{:04X} ({})...",
                    e.sections.size(), info.inputIndex, info.name);
        auto wr = config_mgr_.writeSections(e.type, e.sections);
        if (wr != ConfigManager::Result::Success) {
            TETHER_LOGE(tag_, "Re-write of {} failed: {}",
                        info.name, ConfigManager::resultToString(wr));
            return false;
        }
    }

    // 2. Optionally activate.
    if (config_.re_activate) {
        if (stopped()) {
            TETHER_LOGW(tag_, "Recovery aborted: stop requested before activation");
            return false;
        }

        std::vector<ConfigDataType> types;
        std::vector<CrcExpectation> expectations;
        for (const auto& e : entries_) {
            types.push_back(e.type);
            if (config_.verify_crc) {
                expectations.push_back(e.crc);
            }
        }

        TETHER_LOGI(tag_, "Re-activating {} data type(s)...", types.size());
        auto ar = config_mgr_.activate(types, expectations);
        if (ar != ConfigManager::Result::Success) {
            TETHER_LOGE(tag_, "Re-activation failed: {}",
                        ConfigManager::resultToString(ar));
            return false;
        }
        TETHER_LOGI(tag_, "Re-activation succeeded, CRC verified");
    } else {
        TETHER_LOGI(tag_, "Re-write complete (activation skipped)");
    }

    // 3. Post-recovery callback (e.g. re-start safety state machine).
    if (post_recovery_callback_) {
        TETHER_LOGI(tag_, "Invoking post-recovery callback...");
        post_recovery_callback_();
    }

    TETHER_LOGI(tag_, "Config recovery #{} completed successfully", attempt);
    return true;
}

void ConfigRecoveryHandler::setPostRecoveryCallback(std::function<void()> callback) {
    post_recovery_callback_ = std::move(callback);
}

void ConfigRecoveryHandler::setSdoMutex(std::mutex* mutex) {
    sdo_mutex_ = mutex;
}

void ConfigRecoveryHandler::setStopToken(std::stop_token token) {
    stop_token_ = std::move(token);
    config_mgr_.setStopToken(stop_token_);
}

} // namespace EtherCAT::Drives::ESC211
