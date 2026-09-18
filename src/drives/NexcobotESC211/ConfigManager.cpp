#include "tether/drives/NexcobotESC211/ConfigManager.hpp"

#include <cstring>
#include <thread>

#include "tether/drives/NexcobotESC211/SystemStateName.hpp"
#include "logging/Logger.hpp"
#include "tether/drives/NexcobotESC211Errors.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;
namespace Errors      = EtherCAT::Drives::ErrorCodes::NexcobotESC211;

ConfigManager::ConfigManager(EtherCAT::Slave& slave,
                             Config config,
                             const char* tag)
    : slave_(slave), config_(config), tag_(tag),
      channel_(slave,
               ControlCommandChannel::Config{
                   .command_timeout = config.command_timeout,
                   .poll_interval = std::chrono::milliseconds(200),  // 5 Hz
                   .pre_command_reset_delay = std::chrono::milliseconds(100),
                   .post_command_reset = config.post_command_reset,
               },
               tag),
      crc_verifier_(slave, config.crc_poll_interval, tag) {}

// --- Write ---

ConfigManager::Result
ConfigManager::writeSections(ConfigDataType type, std::span<const std::vector<uint8_t>> sections) {
    const auto& info = configDataTypeInfo(type);

    if (sections.size() > info.maxSections) {
        TETHER_LOGE(tag_, "Too many sections for {}: {} (max {})",
                    info.name, sections.size(), info.maxSections);
        return Result::NoData;
    }

    TETHER_LOGI(tag_, "Writing {} sections to 0x{:04X} ({})...",
                sections.size(), info.inputIndex, info.name);

    for (size_t i = 0; i < sections.size(); ++i) {
        uint8_t sub = static_cast<uint8_t>(i + 1);
        auto err = slave_.sdoWrite(info.inputIndex, sub,
                                   sections[i].data(), sections[i].size());
        if (err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(tag_, "Failed to write {} section {}: {}",
                        info.name, i + 1, EtherCAT::slaveErrorToString(err));
            return Result::SDOError;
        }
    }

    TETHER_LOGI(tag_, "Wrote {} section(s) to 0x{:04X} ({})",
                sections.size(), info.inputIndex, info.name);
    return Result::Success;
}

ConfigManager::Result
ConfigManager::writeRawData(ConfigDataType type, std::span<const uint8_t> rawData) {
    const auto& info = configDataTypeInfo(type);
    const size_t kSectionSize = 256;

    // Split into 256-byte zero-padded sections.
    std::vector<std::vector<uint8_t>> sections;
    for (size_t i = 0; i < rawData.size(); i += kSectionSize) {
        size_t end = std::min(i + kSectionSize, rawData.size());
        std::vector<uint8_t> section(rawData.begin() + i, rawData.begin() + end);
        section.resize(kSectionSize, 0x00);  // zero-pad
        sections.push_back(std::move(section));
    }

    if (sections.empty()) {
        TETHER_LOGE(tag_, "No data to write for {}", info.name);
        return Result::NoData;
    }

    return writeSections(type, sections);
}

// --- Activate ---

ConfigManager::Result
ConfigManager::activate(std::span<const ConfigDataType> types,
                        std::span<const CrcExpectation> expected_crcs,
                        bool do_flash_workaround) {
    // FNI uses cmd 3 (UpdateFNIFromObject) with matchDone.
    // RSP/SDD uses cmd 4 (DownloadRSPAndSDDByObject) with matchCmd4.
    bool hasFNI = false, hasRSPorSDD = false;
    for (auto t : types) {
        if (t == ConfigDataType::FNI) hasFNI = true;
        else hasRSPorSDD = true;
    }

    // Helper: send the activate commands for the requested data types.
    auto sendActivate = [&]() -> Result {
        if (hasFNI) {
            auto r = channel_.sendAndWait(
                UserSystem::ControlCommandCode::UpdateFNIFromObject,
                ControlCommandChannel::matchDone);
            if (r != ControlCommandChannel::Result::Success) {
                TETHER_LOGE(tag_, "FNI activation failed: {}",
                            ControlCommandChannel::resultToString(r));
                logFniErrorSnapshot();
                return Result::Failed;
            }
            TETHER_LOGI(tag_, "FNI activated successfully");
        }
        if (hasRSPorSDD) {
            auto r = channel_.sendAndWait(
                UserSystem::ControlCommandCode::DownloadRSPAndSDDByObject,
                ControlCommandChannel::matchCmd4);
            if (r != ControlCommandChannel::Result::Success) {
                TETHER_LOGE(tag_, "RSP/SDD activation failed: {}",
                            ControlCommandChannel::resultToString(r));
                return Result::Failed;
            }
            TETHER_LOGI(tag_, "RSP/SDD activated successfully");
        }
        return Result::Success;
    };

    if (do_flash_workaround) {
        // The ESC211 CRC register does not always update correctly on a single
        // activation pass.  The workaround is to activate, then deactivate (load
        // from flash to revert active data), then activate again.  The CRC
        // register is correctly computed after this double-activation cycle.
        //
        // Step 1: First activation (temp → active).
        TETHER_LOGI(tag_, "Activation pass 1 (temp → active)...");
        auto r1 = sendActivate();
        if (r1 != Result::Success) return r1;

        // Step 2: Deactivate (flash → active) to revert the active data.
        // If flash is empty or the load fails, log a warning but continue —
        // the second activation will re-apply the temp data regardless.
        TETHER_LOGI(tag_, "Deactivation pass (flash → active)...");
        auto dr = loadFromFlash(types);
        if (dr != Result::Success) {
            TETHER_LOGW(tag_, "Deactivation (load from flash) failed: {} — "
                              "continuing with second activation",
                        resultToString(dr));
        }

        // Step 3: Second activation (temp → active).
        TETHER_LOGI(tag_, "Activation pass 2 (temp → active)...");
        auto r2 = sendActivate();
        if (r2 != Result::Success) return r2;
    } else {
        TETHER_LOGI(tag_, "Activating temp data (single pass, no flash access)...");
        auto r1 = sendActivate();
        if (r1 != Result::Success) return r1;
    }

    // Verify CRC if expectations were provided.
    if (!expected_crcs.empty()) {
        TETHER_LOGI(tag_, "Waiting for device CRC to match (timeout {} ms)...",
                    static_cast<long long>(config_.crc_timeout.count()));
        if (!crc_verifier_.waitUntilMatch(expected_crcs, config_.crc_timeout)) {
            TETHER_LOGE(tag_, "CRC verification failed after activation");
            return Result::Failed;
        }
        TETHER_LOGI(tag_, "CRC verification: all matched");
    }

    return Result::Success;
}

// --- Flash ---

ConfigManager::Result
ConfigManager::saveToFlash(std::span<const ConfigDataType> types) {
    bool hasFNI = false, hasRSPorSDD = false;
    for (auto t : types) {
        if (t == ConfigDataType::FNI) hasFNI = true;
        else hasRSPorSDD = true;
    }

    if (hasFNI) {
        auto r = channel_.sendAndWait(
            UserSystem::ControlCommandCode::SaveFNIToFlash,
            ControlCommandChannel::matchDone);
        if (r != ControlCommandChannel::Result::Success) {
            TETHER_LOGE(tag_, "FNI flash save failed: {}",
                        ControlCommandChannel::resultToString(r));
            logFniErrorSnapshot();
            return Result::Failed;
        }
        TETHER_LOGI(tag_, "FNI saved to flash successfully");
    }

    if (hasRSPorSDD) {
        auto r = channel_.sendAndWait(
            UserSystem::ControlCommandCode::SaveRSPAndSDDToFlash,
            ControlCommandChannel::matchDone);
        if (r != ControlCommandChannel::Result::Success) {
            TETHER_LOGE(tag_, "RSP/SDD flash save failed: {}",
                        ControlCommandChannel::resultToString(r));
            return Result::Failed;
        }
        TETHER_LOGI(tag_, "RSP/SDD saved to flash successfully");
    }

    return Result::Success;
}

ConfigManager::Result
ConfigManager::loadFromFlash(std::span<const ConfigDataType> types) {
    bool hasFNI = false, hasRSPorSDD = false;
    for (auto t : types) {
        if (t == ConfigDataType::FNI) hasFNI = true;
        else hasRSPorSDD = true;
    }

    if (hasFNI) {
        auto r = channel_.sendAndWait(
            UserSystem::ControlCommandCode::LoadFNIFromFlash,
            ControlCommandChannel::matchDone);
        if (r != ControlCommandChannel::Result::Success) {
            TETHER_LOGE(tag_, "FNI flash load failed: {}",
                        ControlCommandChannel::resultToString(r));
            logFniErrorSnapshot();
            return Result::Failed;
        }
        TETHER_LOGI(tag_, "FNI loaded from flash successfully");
    }

    if (hasRSPorSDD) {
        auto r = channel_.sendAndWait(
            UserSystem::ControlCommandCode::LoadRSPAndSDDFromFlash,
            ControlCommandChannel::matchDone);
        if (r != ControlCommandChannel::Result::Success) {
            TETHER_LOGE(tag_, "RSP/SDD flash load failed: {}",
                        ControlCommandChannel::resultToString(r));
            return Result::Failed;
        }
        TETHER_LOGI(tag_, "RSP and SDD loaded from flash successfully");
    }

    return Result::Success;
}

// --- Verify ---

bool ConfigManager::verifyCrc(std::span<const CrcExpectation> expectations) {
    return crc_verifier_.waitUntilMatch(expectations, config_.crc_timeout);
}

uint32_t ConfigManager::readCrc(ConfigDataType type) {
    const auto& info = configDataTypeInfo(type);
    return crc_verifier_.readCrc(info.crcIndex);
}

// --- Ensure loaded ---

bool ConfigManager::ensureLoaded() {
    auto fniCrc = readCrc(ConfigDataType::FNI);
    auto rspCrc = readCrc(ConfigDataType::RSP);
    auto sddCrc = readCrc(ConfigDataType::SDD);

    TETHER_LOGI(tag_, "Config CRCs: FNI=0x{:08X}, RSP=0x{:08X}, SDD=0x{:08X}",
                fniCrc, rspCrc, sddCrc);

    bool anyZero = (fniCrc == 0x0000) || (rspCrc == 0x0000) || (sddCrc == 0x0000);

    if (!anyZero) {
        TETHER_LOGI(tag_, "FNI/RSP/SDD config data already loaded");
        return true;
    }

    TETHER_LOGW(tag_, "Config data not loaded — loading all from flash");

    // Load all from flash: FNI first (cmd 1), then RSP+SDD (cmd 2).
    ConfigDataType all[] = {ConfigDataType::FNI, ConfigDataType::RSP, ConfigDataType::SDD};
    auto r = loadFromFlash(all);
    if (r != Result::Success) {
        TETHER_LOGE(tag_, "Failed to load config from flash");
        return false;
    }

    // Wait for CRC registers to settle.
    TETHER_LOGI(tag_, "Waiting {} ms for CRC registers to settle...",
                static_cast<long long>(config_.post_flash_load_settle.count()));
    std::this_thread::sleep_for(config_.post_flash_load_settle);

    // Re-verify.
    fniCrc = readCrc(ConfigDataType::FNI);
    rspCrc = readCrc(ConfigDataType::RSP);
    sddCrc = readCrc(ConfigDataType::SDD);

    if (fniCrc == 0x0000 || rspCrc == 0x0000 || sddCrc == 0x0000) {
        TETHER_LOGE(tag_, "Config data still not loaded after flash load "
                          "(FNI=0x%08X, RSP=0x%08X, SDD=0x%08X). "
                          "Flash may be empty or corrupted.",
                    fniCrc, rspCrc, sddCrc);
        return false;
    }

    TETHER_LOGI(tag_, "Config data loaded and verified "
                      "(FNI=0x%08X, RSP=0x%08X, SDD=0x%08X)",
                fniCrc, rspCrc, sddCrc);
    return true;
}

// --- Read temp data ---

std::vector<uint8_t> ConfigManager::readTempData(ConfigDataType type) {
    const auto& info = configDataTypeInfo(type);
    std::vector<uint8_t> rawData;

    for (uint16_t i = 1; i <= info.maxSections; ++i) {
        std::vector<uint8_t> buf(256, 0);
        size_t actual = buf.size();
        auto err = slave_.sdoRead(info.inputIndex, static_cast<uint8_t>(i),
                                  buf.data(), actual);
        if (err != EtherCAT::SlaveError::Ok) break;
        buf.resize(actual);
        rawData.insert(rawData.end(), buf.begin(), buf.end());
    }

    return rawData;
}

// --- Diagnostics ---

void ConfigManager::logFniErrorSnapshot() {
    TETHER_LOGI(tag_, "Dumping ESC system error state and debug log after FNI command failure...");

    // 0xF101:0x00 — System Current State
    uint32_t system_state = 0;
    auto e = slave_.sdoReadU32(UserSystem::SystemCurrentStateIndex, 0x00, system_state);
    if (e == EtherCAT::SlaveError::Ok) {
        TETHER_LOGI(tag_, "  0xF101:0x00 System current state = {} (0x{:08X}) [{}]",
                    system_state, system_state, systemStateName(system_state));
    } else {
        TETHER_LOGE(tag_, "  0xF101:0x00 read failed: {}",
                    EtherCAT::slaveErrorToString(e));
    }

    // 0xF102:0x00 — System Error Code
    uint32_t raw_error = 0;
    e = slave_.sdoReadU32(UserSystem::SystemErrorCodeIndex, 0x00, raw_error);
    if (e == EtherCAT::SlaveError::Ok) {
        auto error_code = static_cast<int32_t>(raw_error);
        if (error_code != 0) {
            auto info = Errors::NexcobotESC211Error::parse(error_code);
            TETHER_LOGE(tag_,
                        "  0xF102:0x00 System error: {} (0x{:08X}) -> {} [{}]: {} | {}",
                        error_code, raw_error,
                        info.name,
                        Errors::ErrorCategoryToString(info.category),
                        info.description,
                        info.error_handling);

            char buf[256] = {};
            size_t actual = sizeof(buf);
            auto msg_err = slave_.sdoRead(UserSystem::SystemErrorMessageIndex, 0x00,
                                          buf, actual);
            if (msg_err == EtherCAT::SlaveError::Ok && actual > 0) {
                TETHER_LOGE(tag_, "  0xF103:0x00 System error message: {}",
                            std::string(buf, strnlen(buf, actual)));
            } else if (msg_err != EtherCAT::SlaveError::Ok) {
                TETHER_LOGE(tag_, "  0xF103:0x00 read failed: {}",
                            EtherCAT::slaveErrorToString(msg_err));
            }
        } else {
            TETHER_LOGI(tag_, "  0xF102:0x00 System error code = 0 (no error)");
        }
    } else {
        TETHER_LOGE(tag_, "  0xF102:0x00 read failed: {}",
                    EtherCAT::slaveErrorToString(e));
    }

    // 0xF110 — ESC Debug Msg
    TETHER_LOGI(tag_, "  Reading ESC debug messages (0x{:04X})...",
                UserSystem::ESCDebugMsgIndex);
    uint8_t count = 0;
    auto count_err = slave_.sdoReadU8(UserSystem::ESCDebugMsgIndex, 0x00, count);
    if (count_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(tag_, "    0x{:04X}:0x00 (count) read failed: {}",
                    UserSystem::ESCDebugMsgIndex,
                    EtherCAT::slaveErrorToString(count_err));
    } else {
        TETHER_LOGI(tag_, "    0x{:04X}:0x00 entry count = {}",
                    UserSystem::ESCDebugMsgIndex,
                    static_cast<unsigned>(count));
        uint8_t n = std::min<uint8_t>(count, 16);
        for (uint8_t sub = 1; sub <= n; ++sub) {
            char buf[512] = {};
            size_t actual = sizeof(buf);
            auto read_err = slave_.sdoRead(UserSystem::ESCDebugMsgIndex, sub,
                                           buf, actual);
            if (read_err != EtherCAT::SlaveError::Ok) {
                TETHER_LOGE(tag_, "    0x{:04X}:0x{:02X} read failed: {}",
                            UserSystem::ESCDebugMsgIndex,
                            static_cast<unsigned>(sub),
                            EtherCAT::slaveErrorToString(read_err));
                continue;
            }
            size_t len = strnlen(buf, actual);
            if (len == 0) {
                TETHER_LOGI(tag_, "    0x{:04X}:0x{:02X}: (empty)",
                            UserSystem::ESCDebugMsgIndex,
                            static_cast<unsigned>(sub));
            } else {
                TETHER_LOGI(tag_, "    0x{:04X}:0x{:02X}: {}",
                            UserSystem::ESCDebugMsgIndex,
                            static_cast<unsigned>(sub),
                            std::string(buf, len));
            }
        }
    }
}

// --- Strings ---

const char* ConfigManager::resultToString(Result r) {
    switch (r) {
        case Result::Success:  return "Success";
        case Result::Failed:   return "Failed";
        case Result::Timeout:  return "Timeout";
        case Result::SDOError: return "SDOError";
        case Result::NoData:   return "NoData";
    }
    return "Unknown";
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
