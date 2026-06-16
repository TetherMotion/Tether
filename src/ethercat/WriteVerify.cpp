/**
 * @file WriteVerify.cpp
 * @brief Implementation of EtherCAT Write-Verify (instance-based, no globals)
 */

#include "tether/ethercat/WriteVerify.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>
#include <algorithm>

namespace EtherCAT {
namespace Verify {

static const char* TAG = "WriteVerify";

// ============================================================================
// WriteVerifier — construction
// ============================================================================

WriteVerifier::WriteVerifier(IWriteVerifyTransport& transport)
    : transport_(transport)
    , config_(WriteVerifyConfig::defaults())
{
}

WriteVerifier::WriteVerifier(IWriteVerifyTransport& transport,
                             const WriteVerifyConfig& config)
    : transport_(transport)
    , config_(config)
{
}

// ============================================================================
// Configuration
// ============================================================================

void WriteVerifier::setConfig(const WriteVerifyConfig& config) {
    config_ = config;
}

const WriteVerifyConfig& WriteVerifier::config() const {
    return config_;
}

void WriteVerifier::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool WriteVerifier::isEnabled() const {
    return enabled_;
}

// ============================================================================
// Statistics
// ============================================================================

const WriteVerifyStats& WriteVerifier::stats() const {
    return stats_;
}

void WriteVerifier::resetStats() {
    stats_ = {};
}

void WriteVerifier::logStats() const {
    TETHER_LOGI(TAG, "Write-Verify Statistics:\n  Total writes:       %llu\n  First-try success:  %llu\n  Verify failures:    %llu\n  Write failures:     %llu\n  Total retries:      %llu\n  Eventual success:   %llu\n  Permanent failures: %llu",
               (unsigned long long)stats_.total_writes,
               (unsigned long long)stats_.successful_writes,
               (unsigned long long)stats_.verify_failures,
               (unsigned long long)stats_.write_failures,
               (unsigned long long)stats_.retries,
               (unsigned long long)stats_.eventual_success,
               (unsigned long long)stats_.permanent_failures);
}

// ============================================================================
// Helpers
// ============================================================================

bool WriteVerifier::compareBuffers(const uint8_t* expected,
                                   const uint8_t* actual,
                                   size_t len, size_t& offset) {
    for (size_t i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            offset = i;
            return false;
        }
    }
    return true;
}

// ============================================================================
// apwrVerify — main write-and-verify implementation
// ============================================================================

WriteVerifyResult WriteVerifier::apwrVerify(uint16_t adp, uint16_t ado,
                                            const void* data, uint16_t len,
                                            unsigned int timeout_ms) {
    stats_.total_writes++;

    // When disabled, do a simple write via sendDatagram + waitForResponse
    if (!enabled_) {
        uint8_t idx = transport_.allocIdx();
        bool sent = transport_.sendDatagram(EtherCATCmd::APWR, idx, adp, ado,
                                            data, len, true);
        if (!sent) {
            stats_.write_failures++;
            return WriteVerifyResult::WriteFailed(0, 1);
        }
        DatagramResponse resp = {};
        if (!transport_.waitForResponse(idx, timeout_ms, resp) || resp.wkc == 0) {
            stats_.write_failures++;
            return WriteVerifyResult::WriteFailed(0, 1);
        }
        stats_.successful_writes++;
        return WriteVerifyResult::Success(resp.wkc, 1);
    }

    const uint8_t* write_data = static_cast<const uint8_t*>(data);
    uint8_t read_buffer[kMaxDataLen];

    if (len > kMaxDataLen) {
        if (config_.log_failures) {
            TETHER_LOGE(TAG, "Data too large for verify: %u", len);
        }
        return WriteVerifyResult::WriteFailed(0, 0);
    }

    for (uint32_t attempt = 0; attempt <= config_.retry_count; attempt++) {
        if (attempt > 0) {
            stats_.retries++;
            if (config_.retry_delay_ms > 0) {
                transport_.delayMs(config_.retry_delay_ms);
            }
        }

        // Send APWR datagram
        uint8_t idx = transport_.allocIdx();
        bool ok = transport_.sendDatagram(EtherCATCmd::APWR, idx, adp, ado,
                                          data, len, true);
        if (!ok) {
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "APWR send failed at 0x%04X:0x%04X (attempt %u)",
                            adp, ado, attempt + 1);
            }
            continue;
        }

        // Wait for response
        DatagramResponse resp = {};
        if (!transport_.waitForResponse(idx, timeout_ms, resp)) {
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "APWR timeout at 0x%04X:0x%04X (attempt %u)",
                            adp, ado, attempt + 1);
            }
            continue;
        }

        if (resp.wkc == 0) {
            stats_.write_failures++;
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "APWR WKC=0 at 0x%04X:0x%04X (attempt %u)",
                            adp, ado, attempt + 1);
            }
            continue;
        }

        uint16_t write_wkc = resp.wkc;

        // Optional delay before read-back
        if (config_.read_delay_ms > 0) {
            transport_.delayMs(config_.read_delay_ms);
        }

        // Read back for verification
        std::memset(read_buffer, 0, len);
        if (transport_.readRegister(adp, ado, read_buffer, len, timeout_ms)) {
            size_t mismatch_offset = 0;
            if (!compareBuffers(write_data, read_buffer, len, mismatch_offset)) {
                stats_.verify_failures++;
                if (config_.log_failures) {
                    TETHER_LOGW(TAG, "Verify mismatch at 0x%04X:0x%04X[%zu]: "
                                "wrote 0x%02X, read 0x%02X",
                                adp, ado, mismatch_offset,
                                write_data[mismatch_offset],
                                read_buffer[mismatch_offset]);
                }
                if (attempt == config_.retry_count) {
                    stats_.permanent_failures++;
                    return WriteVerifyResult::VerifyFailed(
                        write_wkc, 1, attempt + 1, mismatch_offset,
                        write_data[mismatch_offset],
                        read_buffer[mismatch_offset]);
                }
                continue;
            }

            // Verification succeeded
            if (attempt == 0) {
                stats_.successful_writes++;
            } else {
                stats_.eventual_success++;
            }
            return WriteVerifyResult::Success(write_wkc, attempt + 1);
        } else {
            // Read failed
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "Verify read failed at 0x%04X:0x%04X",
                            adp, ado);
            }
            continue;
        }
    }

    stats_.permanent_failures++;
    return WriteVerifyResult::WriteFailed(0, config_.retry_count + 1);
}

// ============================================================================
// Typed write-verify helpers
// ============================================================================

WriteVerifyResult WriteVerifier::apwrVerifyU16(uint16_t adp, uint16_t ado,
                                               uint16_t value,
                                               unsigned int timeout_ms) {
    // Store as little-endian
    uint8_t le[2];
    le[0] = static_cast<uint8_t>(value & 0xFF);
    le[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    return apwrVerify(adp, ado, le, sizeof(le), timeout_ms);
}

WriteVerifyResult WriteVerifier::apwrVerifyU32(uint16_t adp, uint16_t ado,
                                               uint32_t value,
                                               unsigned int timeout_ms) {
    uint8_t le[4];
    for (int i = 0; i < 4; i++) {
        le[i] = static_cast<uint8_t>(value & 0xFF);
        value >>= 8;
    }
    return apwrVerify(adp, ado, le, sizeof(le), timeout_ms);
}

WriteVerifyResult WriteVerifier::apwrVerifyU64(uint16_t adp, uint16_t ado,
                                               uint64_t value,
                                               unsigned int timeout_ms) {
    uint8_t le[8];
    for (int i = 0; i < 8; i++) {
        le[i] = static_cast<uint8_t>(value & 0xFF);
        value >>= 8;
    }
    return apwrVerify(adp, ado, le, sizeof(le), timeout_ms);
}

// ============================================================================
// bwrVerify — broadcast write + per-slave APRD verification
// ============================================================================

WriteVerifyResult WriteVerifier::bwrVerify(uint16_t slave_to_verify,
                                           uint16_t ado,
                                           const void* data, uint16_t len,
                                           unsigned int timeout_ms) {
    stats_.total_writes++;

    if (len > kMaxDataLen) {
        if (config_.log_failures) {
            TETHER_LOGE(TAG, "Data too large for BWR verify: %u", len);
        }
        return WriteVerifyResult::WriteFailed(0, 0);
    }

    const uint8_t* write_data = static_cast<const uint8_t*>(data);
    uint8_t read_buffer[kMaxDataLen];

    for (uint32_t attempt = 0; attempt <= config_.retry_count; attempt++) {
        if (attempt > 0) {
            stats_.retries++;
            if (config_.retry_delay_ms > 0) {
                transport_.delayMs(config_.retry_delay_ms);
            }
        }

        // Send BWR datagram (broadcast write, adp=0 for BWR)
        uint8_t idx = transport_.allocIdx();
        bool ok = transport_.sendDatagram(EtherCATCmd::BWR, idx, 0, ado,
                                          data, len, true);
        if (!ok) {
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "BWR send failed at ado=0x%04X (attempt %u)",
                            ado, attempt + 1);
            }
            continue;
        }

        // Wait for BWR response
        DatagramResponse resp = {};
        if (!transport_.waitForResponse(idx, timeout_ms, resp)) {
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "BWR timeout at ado=0x%04X (attempt %u)",
                            ado, attempt + 1);
            }
            continue;
        }

        if (resp.wkc == 0) {
            stats_.write_failures++;
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "BWR WKC=0 at ado=0x%04X (attempt %u)",
                            ado, attempt + 1);
            }
            continue;
        }

        uint16_t write_wkc = resp.wkc;

        // Skip verification when disabled
        if (!enabled_) {
            if (attempt == 0) stats_.successful_writes++;
            else stats_.eventual_success++;
            return WriteVerifyResult::Success(write_wkc, attempt + 1);
        }

        // Optional delay before read-back
        if (config_.read_delay_ms > 0) {
            transport_.delayMs(config_.read_delay_ms);
        }

        // Verify via APRD on the specific slave
        std::memset(read_buffer, 0, len);
        if (transport_.readRegister(slave_to_verify, ado, read_buffer,
                                    len, timeout_ms)) {
            size_t mismatch_offset = 0;
            if (!compareBuffers(write_data, read_buffer, len, mismatch_offset)) {
                stats_.verify_failures++;
                if (config_.log_failures) {
                    TETHER_LOGW(TAG, "BWR verify mismatch at slave 0x%04X "
                                "ado=0x%04X[%zu]: wrote 0x%02X, read 0x%02X",
                                slave_to_verify, ado, mismatch_offset,
                                write_data[mismatch_offset],
                                read_buffer[mismatch_offset]);
                }
                if (attempt == config_.retry_count) {
                    stats_.permanent_failures++;
                    return WriteVerifyResult::VerifyFailed(
                        write_wkc, 1, attempt + 1, mismatch_offset,
                        write_data[mismatch_offset],
                        read_buffer[mismatch_offset]);
                }
                continue;
            }

            if (attempt == 0) stats_.successful_writes++;
            else stats_.eventual_success++;
            return WriteVerifyResult::Success(write_wkc, attempt + 1);
        } else {
            if (config_.log_failures) {
                TETHER_LOGW(TAG, "BWR verify read failed at slave 0x%04X "
                            "ado=0x%04X", slave_to_verify, ado);
            }
            continue;
        }
    }

    stats_.permanent_failures++;
    return WriteVerifyResult::WriteFailed(0, config_.retry_count + 1);
}

} // namespace Verify
} // namespace EtherCAT
