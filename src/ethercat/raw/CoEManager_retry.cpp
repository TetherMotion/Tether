/**
 * @file CoEManager_retry.cpp
 * @brief CoEManager — SDO upload/download retry wrappers + AL status logging.
 *
 * TU split out of CoEManager.cpp.
 */

#include "tether/ethercat/CoEManager.hpp"
#include "raw/CoEErrorStrings.hpp"
#include "tether/ethercat/CustomPDOMapping.hpp" // inferByteSize
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/platform/Platform.hpp"

#ifdef TETHER_COMPILE_MASTER
#include "tether/ethercat/Raw.hpp"
#include "raw/internal.hpp"
#endif

#include <cstring>
#include <algorithm>
#include <type_traits>
#include <vector>
#include <format>

namespace EtherCAT {
namespace CoE {

static const char* TAG = "coe_mgr";

// ============================================================================
// Retry Wrappers
// ============================================================================

bool CoEManager::sdoUploadWithRetry(uint16_t index, uint8_t subindex,
                                    uint8_t* out, size_t out_cap, size_t* out_len,
                                    const CoETransactionOptions& options) {
    if (transport_.isCancelRequested()) {
        return false;
    }
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    if (!resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
        TETHER_LOGE(TAG, "{}: sdoUploadWithRetry: mailbox not configured", log_prefix_.c_str());
        return false;
    }

    // Translate options.complete_access into Tether's internal Complete-Access
    // signal (bit 7 set in the subindex). The raw SDO upload layer reads this
    // marker and converts it into the spec-compliant ETG.1000.6 CA bit (0x10)
    // in the SDO command byte, sending the clean subindex on the wire. Without
    // this translation the CA flag is silently dropped and the slave receives
    // a normal (non-CA) upload request, which mirrors the download path below.
    const uint8_t effective_sub = options.complete_access
        ? static_cast<uint8_t>(subindex | 0x80u)
        : subindex;

    if (options.complete_access) {
        TETHER_LOGI(TAG, "{}: SDO upload 0x{:04X}:{} (Complete Access) cap={}",
                    log_prefix_.c_str(), index, subindex, out_cap);
    }

    last_sdo_abort_code_.store(0, std::memory_order_relaxed);
    last_sdo_was_download_.store(false, std::memory_order_relaxed);
    last_sdo_attempted_length_.store(out_cap, std::memory_order_relaxed);

    const uint8_t max_attempts = options.max_retries + 1;
    for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
        bool ok = transport_.sdoUpload(
            slave_index_, mbxCounterPtr(),
            wr_addr, wr_len, rd_addr, rd_len,
            index, effective_sub,
            out, out_cap, out_len,
            diag_enabled_.load(),
            options.poll_interval_ms, options.timeout_ms);

        if (behaviour_options_.request_al_status_after_coe_requests) {
            logALStatusAfterRequest();
        }

        if (ok) return true;

        // Cancelled mid-operation (Ctrl-C / Master::stop): failures are
        // expected — abort the retry loop without error spam.
        if (transport_.isCancelRequested()) {
            TETHER_LOGI(TAG, "{}: SDO upload 0x{:04X}:{} cancelled",
                        log_prefix_.c_str(), index, subindex);
            return false;
        }

        // Definitive slave SDO abort: the slave explicitly rejected the
        // request (e.g. wrong payload size, read-only object, subindex does
        // not exist). Retrying the identical request cannot succeed, so do
        // NOT retry — record the abort code and escalate immediately. The
        // abort detail travels to the caller inside the CoEError payload,
        // so this stays at debug level to keep the log to a single line
        // emitted by the caller.
        const uint32_t abort_code = transport_.lastAbortCode();
        if (abort_code != 0) {
            last_sdo_abort_code_.store(abort_code, std::memory_order_relaxed);
            TETHER_LOGD(TAG, "{}: SDO upload 0x{:04X}:{} aborted by slave — code 0x{:08X} ({}). Attempted read buffer capacity: {} bytes. Not retrying.",
                        log_prefix_.c_str(), index, subindex, abort_code,
                        sdoAbortCodeStr(abort_code), out_cap);
            return false;
        }

        if (attempt + 1 < max_attempts) {
            TETHER_LOGW(TAG, "{}: SDO upload 0x{:04X}:{} failed on attempt {}/{}, retrying",
                        log_prefix_.c_str(), index, subindex, attempt + 1, max_attempts);
        }
    }

    TETHER_LOGE(TAG, "{}: SDO upload 0x{:04X}:{} failed after {} attempts",
                log_prefix_.c_str(), index, subindex, max_attempts);
    return false;
}

bool CoEManager::sdoDownloadWithRetry(uint16_t index, uint8_t subindex,
                                      const uint8_t* data, size_t data_len,
                                      const CoETransactionOptions& options) {
    if (transport_.isCancelRequested()) {
        return false;
    }
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    if (!resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
        TETHER_LOGE(TAG, "{}: sdoDownloadWithRetry: mailbox not configured", log_prefix_.c_str());
        return false;
    }

    const uint8_t effective_sub = options.complete_access
        ? static_cast<uint8_t>(subindex | 0x80u)
        : subindex;

    if (options.complete_access) {
        TETHER_LOGI(TAG, "{}: SDO download 0x{:04X}:{} (Complete Access) {} bytes",
                    log_prefix_.c_str(), index, subindex, data_len);
    }

    last_sdo_abort_code_.store(0, std::memory_order_relaxed);
    last_sdo_was_download_.store(true, std::memory_order_relaxed);
    last_sdo_attempted_length_.store(data_len, std::memory_order_relaxed);

    const uint8_t max_attempts = options.max_retries + 1;
    for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
        bool ok = transport_.sdoDownload(
            slave_index_, mbxCounterPtr(),
            wr_addr, wr_len, rd_addr, rd_len,
            index, effective_sub,
            data, data_len,
            diag_enabled_.load(),
            options.poll_interval_ms, options.timeout_ms);

        if (behaviour_options_.request_al_status_after_coe_requests) {
            logALStatusAfterRequest();
        }

        if (ok) return true;

        // Cancelled mid-operation (Ctrl-C / Master::stop): failures are
        // expected — abort the retry loop without error spam.
        if (transport_.isCancelRequested()) {
            TETHER_LOGI(TAG, "{}: SDO download 0x{:04X}:{} cancelled",
                        log_prefix_.c_str(), index, subindex);
            return false;
        }

        // Definitive slave SDO abort: the slave explicitly rejected the
        // request (e.g. wrong payload size, read-only object, subindex does
        // not exist). Retrying the identical request cannot succeed, so do
        // NOT retry — record the abort code and escalate immediately. The
        // abort detail travels to the caller inside the CoEError payload,
        // so this stays at debug level to keep the log to a single line
        // emitted by the caller.
        const uint32_t abort_code = transport_.lastAbortCode();
        if (abort_code != 0) {
            last_sdo_abort_code_.store(abort_code, std::memory_order_relaxed);
            TETHER_LOGD(TAG, "{}: SDO download 0x{:04X}:{} aborted by slave — code 0x{:08X} ({}). Attempted payload length: {} bytes. Not retrying.",
                        log_prefix_.c_str(), index, subindex, abort_code,
                        sdoAbortCodeStr(abort_code), data_len);
            return false;
        }

        if (attempt + 1 < max_attempts) {
            TETHER_LOGW(TAG, "{}: SDO download 0x{:04X}:{} failed on attempt {}/{}, retrying",
                        log_prefix_.c_str(), index, subindex, attempt + 1, max_attempts);
        }
    }

    TETHER_LOGE(TAG, "{}: SDO download 0x{:04X}:{} failed after {} attempts",
                log_prefix_.c_str(), index, subindex, max_attempts);
    return false;
}

uint8_t* CoEManager::mbxCounterPtr() {
    return &mbx_.mbx_counter;
}

void CoEManager::logALStatusAfterRequest() {
    if (transport_.isCancelRequested()) {
        return;
    }
    uint16_t al_status = 0;
    uint16_t al_code = 0;

    if (!transport_.readSlaveRegister(slave_index_, 0x0130, &al_status, sizeof(al_status), 200)) {
        TETHER_LOGW(TAG, "{}: post-SDO AL_STATUS read FAILED", log_prefix_.c_str());
        return;
    }
    if (!transport_.readSlaveRegister(slave_index_, 0x0134, &al_code, sizeof(al_code), 200)) {
        al_code = 0;
    }

    if (al_status_has_error(al_status) || al_code != 0) {
        TETHER_LOGE(TAG, "{}: post-SDO AL_STATUS=0x{:04X} ({}){} | AL status code: {} (0x{:04X})",
                    log_prefix_.c_str(),
                    al_status,
                    al_status_get_state_name(al_status),
                    al_status_has_error(al_status) ? " ERROR" : "",
                    getALStatusCodeName(al_code),
                    al_code);
    }
}

} // namespace CoE
} // namespace EtherCAT

