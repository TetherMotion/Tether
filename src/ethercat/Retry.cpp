/**
 * @file Retry.cpp
 * @brief Implementation of timeout re-send logic
 */

#include "Retry.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/TetherConfig.hpp"

#define LOG_TAG "EC_RETRY"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

static inline uint64_t get_time_ms() {
    return esp_timer_get_time() / 1000;
}

namespace EtherCAT {
namespace Raw {

// ============================================================================
// Request Builder Functions
// ============================================================================

StoredDatagram buildAPRD(uint8_t idx, uint16_t slave_position, uint16_t ado, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::APRD;
    dgram.idx = idx;
    // ADP: negative offset from slave position (0 for first slave, -1 for second, etc.)
    // After passing through N slaves, ADP becomes 0 for the addressed slave
    dgram.adp = static_cast<uint16_t>(-static_cast<int16_t>(slave_position) - 1);
    dgram.ado = ado;
    dgram.datalen = length;
    std::memset(dgram.data, 0, length);
    return dgram;
}

StoredDatagram buildAPWR(uint8_t idx, uint16_t slave_position, uint16_t ado,
                          const uint8_t* data, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::APWR;
    dgram.idx = idx;
    dgram.adp = static_cast<uint16_t>(-static_cast<int16_t>(slave_position) - 1);
    dgram.ado = ado;
    dgram.datalen = length;
    if (data && length > 0 && length <= sizeof(dgram.data)) {
        std::memcpy(dgram.data, data, length);
    }
    return dgram;
}

StoredDatagram buildFPRD(uint8_t idx, uint16_t configured_addr, uint16_t ado, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::FPRD;
    dgram.idx = idx;
    dgram.adp = configured_addr;  // Configured station address
    dgram.ado = ado;
    dgram.datalen = length;
    std::memset(dgram.data, 0, length);
    return dgram;
}

StoredDatagram buildFPWR(uint8_t idx, uint16_t configured_addr, uint16_t ado,
                          const uint8_t* data, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::FPWR;
    dgram.idx = idx;
    dgram.adp = configured_addr;
    dgram.ado = ado;
    dgram.datalen = length;
    if (data && length > 0 && length <= sizeof(dgram.data)) {
        std::memcpy(dgram.data, data, length);
    }
    return dgram;
}

StoredDatagram buildBRD(uint8_t idx, uint16_t ado, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::BRD;
    dgram.idx = idx;
    dgram.adp = 0;  // Not used for broadcast
    dgram.ado = ado;
    dgram.datalen = length;
    std::memset(dgram.data, 0, length);
    return dgram;
}

StoredDatagram buildBWR(uint8_t idx, uint16_t ado, const uint8_t* data, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::BWR;
    dgram.idx = idx;
    dgram.adp = 0;
    dgram.ado = ado;
    dgram.datalen = length;
    if (data && length > 0 && length <= sizeof(dgram.data)) {
        std::memcpy(dgram.data, data, length);
    }
    return dgram;
}

StoredDatagram buildLRW(uint8_t idx, uint32_t logical_addr, const uint8_t* data, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::LRW;
    dgram.idx = idx;
    // Logical address is stored in ADO (low 16 bits) and ADP (high 16 bits)
    dgram.adp = static_cast<uint16_t>(logical_addr >> 16);
    dgram.ado = static_cast<uint16_t>(logical_addr & 0xFFFF);
    dgram.datalen = length;
    if (data && length > 0 && length <= sizeof(dgram.data)) {
        std::memcpy(dgram.data, data, length);
    }
    return dgram;
}

StoredDatagram buildLRD(uint8_t idx, uint32_t logical_addr, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::LRD;
    dgram.idx = idx;
    dgram.adp = static_cast<uint16_t>(logical_addr >> 16);
    dgram.ado = static_cast<uint16_t>(logical_addr & 0xFFFF);
    dgram.datalen = length;
    std::memset(dgram.data, 0, length);
    return dgram;
}

StoredDatagram buildLWR(uint8_t idx, uint32_t logical_addr, const uint8_t* data, uint16_t length) {
    StoredDatagram dgram;
    dgram.cmd = Raw::EtherCATCommand::LWR;
    dgram.idx = idx;
    dgram.adp = static_cast<uint16_t>(logical_addr >> 16);
    dgram.ado = static_cast<uint16_t>(logical_addr & 0xFFFF);
    dgram.datalen = length;
    if (data && length > 0 && length <= sizeof(dgram.data)) {
        std::memcpy(dgram.data, data, length);
    }
    return dgram;
}

// ============================================================================
// RetryExecutor Implementation
// ============================================================================

RetryExecutor::RetryExecutor(ConditionalPacketRouter& router, SendFunction send_func)
    : router_(router), send_func_(send_func)
{
}

RetryResult RetryExecutor::execute(const StoredDatagram& request,
                                    const PacketFilter& filter,
                                    uint8_t* buffer,
                                    size_t buffer_size,
                                    const RetryPolicy& policy)
{
    RetryResult result;
    result.attempts = 0;

    uint64_t start_time = get_time_ms();
    uint32_t max_attempts = policy.max_retries + 1;  // Initial try + retries

    for (uint32_t attempt = 0; attempt < max_attempts; attempt++) {
        result.attempts = attempt + 1;

        // Send the request
        if (!send_func_(request)) {
            LOGW("Send failed on attempt %lu", (unsigned long)attempt);
            continue;  // Try again
        }

        stats_.total_requests++;

        // Calculate timeout for this attempt
        uint32_t timeout_ms = policy.getTimeoutForAttempt(attempt);

        // Wait for response
        WaitResult wait_result = router_.waitForPacket(filter, buffer, buffer_size, timeout_ms);

        if (wait_result.success) {
            result.success = true;
            result.timeout = false;
            result.wkc = wait_result.wkc;
            result.data_length = wait_result.data_length;
            result.idx = wait_result.idx;
            result.total_time_ms = static_cast<uint32_t>(get_time_ms() - start_time);

            if (attempt == 0) {
                stats_.first_try_success++;
            } else {
                stats_.retry_success++;
                stats_.total_retries += attempt;
                if (attempt > stats_.max_retries_used) {
                    stats_.max_retries_used = attempt;
                }
                LOGD("Success on retry %lu", (unsigned long)attempt);
            }

            return result;
        }

        // Timeout - log and retry
        stats_.total_timeouts++;
        LOGD("Timeout on attempt %lu/%lu (timeout=%lu ms)", 
             (unsigned long)attempt + 1, (unsigned long)max_attempts, (unsigned long)timeout_ms);
    }

    // All attempts failed
    result.success = false;
    result.timeout = true;
    result.total_time_ms = static_cast<uint32_t>(get_time_ms() - start_time);

    stats_.total_failures++;
    stats_.total_retries += policy.max_retries;

    LOGW("All %lu attempts failed for idx=%u", (unsigned long)max_attempts, request.idx);

    return result;
}

RetryResult RetryExecutor::executeWithWkc(const StoredDatagram& request,
                                           const PacketFilter& filter,
                                           uint8_t* buffer,
                                           size_t buffer_size,
                                           uint16_t expected_wkc,
                                           const RetryPolicy& policy)
{
    RetryResult result = execute(request, filter, buffer, buffer_size, policy);

    if (result.success && result.wkc != expected_wkc) {
        LOGW("WKC mismatch: expected %u, got %u", expected_wkc, result.wkc);
        // Still return success=true since we got a response,
        // let caller check isWkcError()
    }

    return result;
}

void RetryExecutor::logStats() const {
    LOGI("Retry Statistics:");
    LOGI("  Total requests:     %llu", (unsigned long long)stats_.total_requests);
    LOGI("  First-try success:  %llu (%.1f%%)", 
         (unsigned long long)stats_.first_try_success, stats_.firstTryRate() * 100.0f);
    LOGI("  Retry success:      %llu", (unsigned long long)stats_.retry_success);
    LOGI("  Total failures:     %llu", (unsigned long long)stats_.total_failures);
    LOGI("  Total retries:      %llu", (unsigned long long)stats_.total_retries);
    LOGI("  Total timeouts:     %llu", (unsigned long long)stats_.total_timeouts);
    LOGI("  Max retries used:   %llu", (unsigned long long)stats_.max_retries_used);
    LOGI("  Success rate:       %.1f%%", stats_.successRate() * 100.0f);
}

// ============================================================================
// High-Level Convenience Functions
// ============================================================================

RetryResult retryAPRD(RetryExecutor& executor, uint8_t idx, uint16_t slave_position,
                      uint16_t ado, uint8_t* buffer, uint16_t length,
                      const RetryPolicy& policy)
{
    StoredDatagram request = buildAPRD(idx, slave_position, ado, length);

    // Build filter: match by index and expect APRD response
    PacketFilter filter = PacketFilter::aprd(slave_position, ado, idx);
    filter.min_wkc = 1;  // Expect at least one slave response

    return executor.execute(request, filter, buffer, length, policy);
}

RetryResult retryAPWRVerify(RetryExecutor& executor, uint8_t idx, uint16_t slave_position,
                            uint16_t ado, const uint8_t* data, uint16_t length,
                            const RetryPolicy& policy)
{
    // First, do the write
    StoredDatagram write_request = buildAPWR(idx, slave_position, ado, data, length);

    PacketFilter write_filter = PacketFilter::apwr(slave_position, ado, idx);
    write_filter.min_wkc = 1;

    uint8_t dummy[64];  // APWR response contains echoed data
    RetryResult write_result = executor.execute(write_request, write_filter, 
                                                 dummy, sizeof(dummy), policy);

    if (!write_result.success) {
        LOGW("APWR failed for slave %u, ado 0x%04X", slave_position, ado);
        return write_result;
    }

    // Now verify by reading back
    uint8_t verify_idx = (idx + 1) & 0xFF;  // Use different index
    uint8_t read_buffer[ECAT_RETRY_VERIFY_BUFFER_SIZE];
    if (length > sizeof(read_buffer)) {
        LOGE("Data too large for Tether internal retry verify buffer: %u bytes (max=%zu). "
             "This is a Tether limit, not a slave limit. "
             "Increase ECAT_RETRY_VERIFY_BUFFER_SIZE in TetherConfig.hpp.",
             length, sizeof(read_buffer));
        write_result.success = false;
        return write_result;
    }

    RetryResult read_result = retryAPRD(executor, verify_idx, slave_position, 
                                         ado, read_buffer, length, policy);

    if (!read_result.success) {
        LOGW("Verify read failed for slave %u, ado 0x%04X", slave_position, ado);
        return read_result;
    }

    // Compare
    if (std::memcmp(data, read_buffer, length) != 0) {
        LOGW("Verify mismatch for slave %u, ado 0x%04X", slave_position, ado);
        LOGW("  Expected: %02X %02X ...", data[0], length > 1 ? data[1] : 0);
        LOGW("  Got:      %02X %02X ...", read_buffer[0], length > 1 ? read_buffer[1] : 0);
        write_result.success = false;
    }

    return write_result;
}

RetryResult retryBRD(RetryExecutor& executor, uint8_t idx, uint16_t ado,
                     uint8_t* buffer, uint16_t length, uint16_t expected_wkc,
                     const RetryPolicy& policy)
{
    StoredDatagram request = buildBRD(idx, ado, length);

    PacketFilter filter = PacketFilter::byIndex(idx);
    filter.command = Raw::EtherCATCommand::BRD;
    if (expected_wkc > 0) {
        filter.min_wkc = expected_wkc;
    }

    return executor.execute(request, filter, buffer, length, policy);
}

RetryResult retryBWR(RetryExecutor& executor, uint8_t idx, uint16_t ado,
                     const uint8_t* data, uint16_t length, uint16_t expected_wkc,
                     const RetryPolicy& policy)
{
    StoredDatagram request = buildBWR(idx, ado, data, length);

    PacketFilter filter = PacketFilter::byIndex(idx);
    filter.command = Raw::EtherCATCommand::BWR;
    if (expected_wkc > 0) {
        filter.min_wkc = expected_wkc;
    }

    uint8_t dummy[64];
    return executor.execute(request, filter, dummy, sizeof(dummy), policy);
}

RetryResult retryLRW(RetryExecutor& executor, uint8_t idx, uint32_t logical_addr,
                     const uint8_t* tx_data, uint8_t* rx_buffer, uint16_t length,
                     uint16_t expected_wkc,
                     const RetryPolicy& policy)
{
    StoredDatagram request = buildLRW(idx, logical_addr, tx_data, length);

    PacketFilter filter = PacketFilter::byIndex(idx);
    filter.command = Raw::EtherCATCommand::LRW;
    filter.min_wkc = expected_wkc;

    return executor.execute(request, filter, rx_buffer, length, policy);
}

// ============================================================================
}  // namespace raw
}  // namespace EtherCAT
