#pragma once

/// @file MultiMcuManager.hpp
/// @brief Multi-MCU coordination manager

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Secondary MCU configuration.
struct SecondaryMcuConfig {
    int id = 0;
    std::string serialPath;
    int baudRate = 250000;
    bool enabled = false;
    uint32_t clockFreq = 48000000;
    bool connected = false;
    std::string firmwareVersion;
    uint32_t bytesRead = 0;
    uint32_t bytesWrite = 0;
    uint32_t retransmits = 0;
};

/// @brief Multi-MCU coordination manager.
class MultiMcuManager {
public:
    /// @brief Set serial path for a secondary MCU (M860).
    void setSerialPath(int id, const std::string& path) {
        mcus_[id].id = id;
        mcus_[id].serialPath = path;
    }

    /// @brief Set baud rate for a secondary MCU (M861).
    void setBaudRate(int id, int baud) {
        mcus_[id].id = id;
        mcus_[id].baudRate = baud;
    }

    /// @brief Enable/disable a secondary MCU (M862).
    void setEnabled(int id, bool enable) {
        mcus_[id].id = id;
        mcus_[id].enabled = enable;
        mcus_[id].connected = enable; // Simplified: enabling connects
    }

    /// @brief Set clock frequency for a secondary MCU (M863).
    void setClockFreq(int id, uint32_t freq) {
        mcus_[id].id = id;
        mcus_[id].clockFreq = freq;
    }

    /// @brief Get status of a secondary MCU (M876).
    std::string getStatus(int id) const {
        auto it = mcus_.find(id);
        if (it == mcus_.end()) return "MCU " + std::to_string(id) + ": not configured";
        const auto& mcu = it->second;
        std::ostringstream ss;
        ss << "MCU " << id << ": " << (mcu.connected ? "connected" : "disconnected")
           << " serial=" << mcu.serialPath
           << " baud=" << mcu.baudRate
           << " freq=" << mcu.clockFreq
           << " read=" << mcu.bytesRead
           << " write=" << mcu.bytesWrite;
        return ss.str();
    }

    /// @brief Update MCU statistics.
    void updateStats(int id, uint32_t bytesRead, uint32_t bytesWrite,
                     uint32_t retransmits) {
        auto& mcu = mcus_[id];
        mcu.bytesRead = bytesRead;
        mcu.bytesWrite = bytesWrite;
        mcu.retransmits = retransmits;
    }

    /// @brief Set firmware version for an MCU.
    void setFirmwareVersion(int id, const std::string& version) {
        mcus_[id].firmwareVersion = version;
    }

    /// @brief Get all configured MCU IDs.
    std::vector<int> mcuIds() const {
        std::vector<int> result;
        for (const auto& [id, _] : mcus_) result.push_back(id);
        return result;
    }

    /// @brief Get configuration for an MCU.
    const SecondaryMcuConfig* getMcu(int id) const {
        auto it = mcus_.find(id);
        return it != mcus_.end() ? &it->second : nullptr;
    }

private:
    std::map<int, SecondaryMcuConfig> mcus_;
};

} // namespace tether::klipper::klippy
