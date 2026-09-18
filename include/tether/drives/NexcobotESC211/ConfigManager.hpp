#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <span>
#include <stop_token>
#include <vector>

#include "tether/drives/NexcobotESC211/ConfigDataTypes.hpp"
#include "tether/drives/NexcobotESC211/ControlCommandChannel.hpp"
#include "tether/drives/NexcobotESC211/CrcVerifier.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Manages the FNI/RSP/SDD configuration data lifecycle on the ESC211:
///   - Write data sections to temp objects (0xF200/0xF210/0xF220)
///   - Activate temp data → active (cmd 3: FNI, cmd 4: RSP+SDD)
///   - Save active data to flash (cmd 5: FNI, cmd 6: RSP+SDD)
///   - Load from flash to active (cmd 1: FNI, cmd 2: RSP+SDD)
///   - Verify CRC registers match expected values
///   - Ensure config data is loaded before starting the safety state machine
///
/// This class replaces the duplicated sendCommand lambdas in esc211_fni.cpp
/// and the ensureConfigDataLoaded() method in SafetyStateMachineManager.
class ConfigManager {
public:
    enum class Result {
        Success,
        Failed,
        Timeout,
        SDOError,
        NoData,
    };

    struct Config {
        std::chrono::milliseconds command_timeout{10000};
        std::chrono::milliseconds crc_timeout{15000};
        std::chrono::milliseconds crc_poll_interval{200};
        std::chrono::milliseconds post_flash_load_settle{500};
        /// If true, send CMD=0 after each command (esc211_fni.cpp style).
        /// If false, only reset before each command (SafetyStateMachineManager style).
        bool post_command_reset{true};
    };

    ConfigManager(EtherCAT::Slave& slave,
                  Config config,
                  const char* tag = "ESC211ConfigManager");

    // --- Write ---

    /// Write 256-byte sections to the temp/input object for the given data type.
    /// The sections are written to subindexes 0x01, 0x02, ... (max maxSections).
    Result writeSections(ConfigDataType type, std::span<const std::vector<uint8_t>> sections);

    /// Convenience: split raw data into 256-byte zero-padded sections and
    /// write them.  Returns NoData if the raw data exceeds maxSections * 256.
    Result writeRawData(ConfigDataType type, std::span<const uint8_t> rawData);

    // --- Activate ---

    /// Activate already-written temp data on the device.
    /// If `types` contains FNI, sends cmd 3 (UpdateFNIFromObject).
    /// If `types` contains RSP or SDD, sends cmd 4 (DownloadRSPAndSDDByObject).
    ///
    /// When `do_flash_workaround` is true the ESC211 firmware workaround is
    /// performed: activate, load from flash (revert active data), then
    /// activate again.  This is required for the CRC registers to reflect the
    /// newly activated data, and should be used when `saveToFlash()` follows.
    ///
    /// When `do_flash_workaround` is false a single temp -> active activation
    /// is performed and no flash-load command is issued.
    ///
    /// After activation, optionally verifies CRC if `expected_crcs` is non-empty.
    Result activate(std::span<const ConfigDataType> types,
                    std::span<const CrcExpectation> expected_crcs = {},
                    bool do_flash_workaround = true);

    // --- Flash ---

    /// Save active data to flash.
    /// If `types` contains FNI, sends cmd 5 (SaveFNIToFlash).
    /// If `types` contains RSP or SDD, sends cmd 6 (SaveRSPAndSDDToFlash).
    Result saveToFlash(std::span<const ConfigDataType> types);

    /// Load data from flash to active memory.
    /// If `types` contains FNI, sends cmd 1 (LoadFNIFromFlash).
    /// If `types` contains RSP or SDD, sends cmd 2 (LoadRSPAndSDDFromFlash).
    Result loadFromFlash(std::span<const ConfigDataType> types);

    // --- Verify ---

    /// Poll CRC registers until they match the expected values or timeout.
    bool verifyCrc(std::span<const CrcExpectation> expectations);

    /// Read a single CRC register.  Returns 0xFFFFFFFF on error.
    uint32_t readCrc(ConfigDataType type);

    // --- Ensure loaded ---

    /// Verify that FNI/RSP/SDD config data is loaded by checking CRC registers.
    /// If any CRC is 0x0000, load ALL from flash (cmd 1 + cmd 2), wait for
    /// settle, then re-verify.  Returns false if any CRC is still 0x0000.
    bool ensureLoaded();

    // --- Read temp data ---

    /// Read all sections from the temp/input object for the given data type.
    /// Returns the concatenated raw data, or an empty vector on error.
    std::vector<uint8_t> readTempData(ConfigDataType type);

    // --- Accessors ---

    ControlCommandChannel& channel() { return channel_; }
    const Config& config() const { return config_; }

    void setStopToken(std::stop_token token) { channel_.setStopToken(std::move(token)); }

    static const char* resultToString(Result r);

private:
    EtherCAT::Slave& slave_;
    Config config_;
    const char* tag_;

    ControlCommandChannel channel_;
    CrcVerifier crc_verifier_;

    /// Read and log 0xF100/0xF101/0xF102/0xF103 and 0xF110 after an
    /// FNI command fails, so the operator has immediate diagnostic context.
    void logFniErrorSnapshot();

    /// Send a command for the given data types.  Groups FNI vs RSP/SDD
    /// and sends the appropriate command for each group.
    Result sendForTypes(std::span<const ConfigDataType> types,
                        ControlCommandChannel::ControlCommandCode fniCmd,
                        ControlCommandChannel::ControlCommandCode rspSddCmd,
                        ControlCommandChannel::ResponseMatcher matcher);
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
