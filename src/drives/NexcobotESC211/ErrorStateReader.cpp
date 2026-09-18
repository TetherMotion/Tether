#include "tether/drives/NexcobotESC211/ErrorStateReader.hpp"

#include <algorithm>
#include <cstring>

#include "tether/drives/NexcobotESC211/SystemStateName.hpp"
#include "logging/Logger.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoETypes.hpp"
#include "tether/drives/NexcobotESC211/Registers/UserSystem.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;
namespace CoE       = EtherCAT::CoE;

ErrorStateReader::ErrorStateReader(EtherCAT::Master& master,
                                   uint16_t slave_index,
                                   const char* tag)
    : master_(master), slave_index_(slave_index), tag_(tag) {}

ErrorStateReader::Snapshot ErrorStateReader::readAndLog() {
    Snapshot snap;
    bool all_read = true;
    auto& coe = master_.sdoManager(slave_index_);

    // ---- 0xF101:0x00 — System Current State (Unsigned32) ----
    TETHER_LOGI(tag_, "Reading system current state (0xF101:0x00)...");
    {
        CoE::CoETransactionOptions opts;
        opts.timeout_ms = 2000;
        auto result = coe.readSync<uint32_t>(UserSystem::SystemCurrentStateIndex,
                                             0x00, opts);
        if (result.has_value()) {
            snap.system_state = result.value();
            TETHER_LOGI(tag_, "ESC current state: {} (0x{:08X}) [{}]",
                        snap.system_state, snap.system_state, systemStateName(snap.system_state));
        } else {
            all_read = false;
            const uint32_t abort = coe.lastSdoAbortCode();
            TETHER_LOGE(tag_,
                        "Failed to read system current state (0xF101): %s "
                        "[abort=0x%08X]",
                        CoE::coeErrorStr(result.error()), abort);
            // Keep going — the remaining fields may still produce useful
            // diagnostics for the operator, mirroring the reference impl.
        }
    }

    // ---- 0xF102:0x00 — System Error Code (Integer32) ----
    TETHER_LOGI(tag_, "Reading system error code (0xF102:0x00)...");
    {
        CoE::CoETransactionOptions opts;
        opts.timeout_ms = 2000;
        auto result = coe.readSync<int32_t>(UserSystem::SystemErrorCodeIndex,
                                            0x00, opts);
        if (result.has_value()) {
            snap.error_code = result.value();
            TETHER_LOGI(tag_, "ESC error code: {} (0x{:08X})",
                        snap.error_code,
                        static_cast<uint32_t>(snap.error_code));
        } else {
            all_read = false;
            const uint32_t abort = coe.lastSdoAbortCode();
            TETHER_LOGE(tag_,
                        "Failed to read system error code (0xF102): %s "
                        "[abort=0x%08X]",
                        CoE::coeErrorStr(result.error()), abort);
        }
    }

    // ---- 0xF103:0x00 — System Error Message (VisibleString, 256 B) ----
    TETHER_LOGI(tag_, "Reading system error message (0xF103:0x00)...");
    {
        CoE::CoETransactionOptions opts;
        opts.timeout_ms = 2000;
        auto result = coe.readSync<std::vector<uint8_t>>(
            UserSystem::SystemErrorMessageIndex, 0x00, opts);
        if (result.has_value()) {
            const auto& vec = result.value();
            size_t nul = 0;
            while (nul < vec.size() && vec[nul] != 0) ++nul;
            snap.error_message.assign(
                reinterpret_cast<const char*>(vec.data()), nul);
            TETHER_LOGI(tag_, "ESC error message: {}",
                        snap.error_message.empty()
                            ? "(empty)" : snap.error_message.c_str());
        } else {
            all_read = false;
            const uint32_t abort = coe.lastSdoAbortCode();
            TETHER_LOGE(tag_,
                        "Failed to read system error message (0xF103): %s "
                        "[abort=0x%08X]",
                        CoE::coeErrorStr(result.error()), abort);
        }
    }

    // ---- 0xF110:0x01 — ESC Debug Msg entry 1 (STRING(64) in ESI v0.9) ----
    // Read without Complete Access: the slave rejects CA on this entry with
    // abort 0x06010000 (Unsupported access). A plain subindex upload returns
    // the full entry.
    TETHER_LOGI(tag_, "Reading ESC diagnostic (0xF110:0x01)...");
    {
        CoE::CoETransactionOptions opts;
        opts.timeout_ms = 2000;
        opts.complete_access = false;
        auto result = coe.readSync<std::vector<uint8_t>>(
            UserSystem::ESCDebugMsgIndex, 0x01, opts);
        if (result.has_value()) {
            const auto& vec = result.value();
            size_t nul = 0;
            while (nul < vec.size() && vec[nul] != 0) ++nul;
            snap.diagnostic_message.assign(
                reinterpret_cast<const char*>(vec.data()), nul);
            TETHER_LOGI(tag_, "ESC diagnostic: {}",
                        snap.diagnostic_message.empty()
                            ? "(empty)" : snap.diagnostic_message.c_str());
        } else {
            all_read = false;
            const uint32_t abort = coe.lastSdoAbortCode();
            TETHER_LOGE(tag_,
                        "Failed to read ESC diagnostic (0xF110:0x01): %s "
                        "[abort=0x%08X]",
                        CoE::coeErrorStr(result.error()), abort);
        }
    }

    snap.valid = all_read;
    return snap;
}

bool ErrorStateReader::isOk(const Snapshot& s) {
    return s.valid && s.error_code == 0 && s.error_message.empty();
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
