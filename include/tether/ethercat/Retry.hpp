/**
 * @file Retry.hpp
 * @brief Timeout re-send logic for EtherCAT packet operations
 * 
 * Implements TODO #5: "ALL code waiting for packets must react, upon timeout,
 * with a re-send. You will need to split up the 'send request' and 'wait for response'
 * into separate functions."
 * 
 * Architecture:
 * - RequestBuilder: Constructs and stores EtherCAT request datagrams
 * - RetryableRequest: Encapsulates a request that can be re-sent
 * - RetryPolicy: Configuration for retry behavior
 * - RetryExecutor: Executes requests with automatic retry on timeout
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>
#include <chrono>

#include "Types.hpp"
#include "ConditionalPacketRouter.hpp"

namespace EtherCAT {
namespace Raw {

using EtherCATCommand = Command;

// ============================================================================
// Retry Policy Configuration
// ============================================================================

/**
 * @brief Configuration for retry behavior
 */
struct RetryPolicy {
    uint32_t max_retries = 3;              ///< Maximum number of retry attempts
    uint32_t initial_timeout_ms = 10;      ///< Initial timeout in milliseconds
    uint32_t max_timeout_ms = 100;         ///< Maximum timeout (for exponential backoff)
    float backoff_multiplier = 2.0f;       ///< Multiplier for exponential backoff
    bool use_exponential_backoff = true;   ///< Enable exponential backoff

    /// Default fast retry (for real-time operations)
    static RetryPolicy fast() {
        return RetryPolicy{
            .max_retries = 2,
            .initial_timeout_ms = 5,
            .max_timeout_ms = 20,
            .backoff_multiplier = 1.5f,
            .use_exponential_backoff = true
        };
    }

    /// Default standard retry
    static RetryPolicy standard() {
        return RetryPolicy{
            .max_retries = 3,
            .initial_timeout_ms = 10,
            .max_timeout_ms = 100,
            .backoff_multiplier = 2.0f,
            .use_exponential_backoff = true
        };
    }

    /// Slow retry (for configuration operations)
    static RetryPolicy slow() {
        return RetryPolicy{
            .max_retries = 5,
            .initial_timeout_ms = 50,
            .max_timeout_ms = 500,
            .backoff_multiplier = 2.0f,
            .use_exponential_backoff = true
        };
    }

    /// Single attempt, no retry
    static RetryPolicy none() {
        return RetryPolicy{
            .max_retries = 0,
            .initial_timeout_ms = 100,
            .max_timeout_ms = 100,
            .backoff_multiplier = 1.0f,
            .use_exponential_backoff = false
        };
    }

    /// Calculate timeout for a given attempt number
    uint32_t getTimeoutForAttempt(uint32_t attempt) const {
        if (!use_exponential_backoff || attempt == 0) {
            return initial_timeout_ms;
        }
        float timeout = initial_timeout_ms;
        for (uint32_t i = 0; i < attempt; i++) {
            timeout *= backoff_multiplier;
        }
        return (timeout > max_timeout_ms) ? max_timeout_ms : static_cast<uint32_t>(timeout);
    }
};

// ============================================================================
// Request Storage
// ============================================================================

/**
 * @brief Stores a complete EtherCAT datagram for transmission
 */
struct StoredDatagram {
    EtherCATCommand cmd = EtherCATCommand::NOP;
    uint8_t idx = 0;
    uint16_t adp = 0;        ///< Slave address (ADP for auto-increment, position for position-based)
    uint16_t ado = 0;        ///< Memory address / offset
    uint16_t datalen = 0;
    uint8_t data[1486] = {0};  ///< Max EtherCAT datagram data

    /// Create empty datagram
    StoredDatagram() = default;

    /// Create with command info
    StoredDatagram(EtherCATCommand c, uint8_t i, uint16_t ap, uint16_t ao, 
                   const uint8_t* d, uint16_t len)
        : cmd(c), idx(i), adp(ap), ado(ao), datalen(len)
    {
        if (d && len > 0 && len <= sizeof(data)) {
            std::memcpy(data, d, len);
        }
    }

    /// Check if valid
    bool isValid() const {
        return cmd != EtherCATCommand::NOP && datalen > 0;
    }
};

// ============================================================================
// Retry Result
// ============================================================================

/**
 * @brief Result of a retry operation
 */
struct RetryResult {
    bool success = false;           ///< True if operation succeeded
    bool timeout = false;           ///< True if all retries timed out
    uint32_t attempts = 0;          ///< Number of attempts made
    uint32_t total_time_ms = 0;     ///< Total time spent
    uint16_t wkc = 0;               ///< Working counter from response
    uint16_t data_length = 0;       ///< Length of response data
    uint8_t idx = 0;                ///< Index from response

    /// Check if this was a WKC error (got response but WKC was wrong)
    bool isWkcError(uint16_t expected_wkc) const {
        return success && wkc != expected_wkc;
    }
};

// ============================================================================
// Statistics
// ============================================================================

/**
 * @brief Statistics for retry operations
 */
struct RetryStats {
    uint64_t total_requests = 0;       ///< Total requests sent
    uint64_t first_try_success = 0;    ///< Succeeded on first try
    uint64_t retry_success = 0;        ///< Succeeded after retry
    uint64_t total_failures = 0;       ///< Failed after all retries
    uint64_t total_retries = 0;        ///< Total retry attempts
    uint64_t total_timeouts = 0;       ///< Total timeout events
    uint64_t max_retries_used = 0;     ///< Maximum retries needed for success

    /// Calculate success rate
    float successRate() const {
        return total_requests > 0 
            ? static_cast<float>(first_try_success + retry_success) / total_requests 
            : 0.0f;
    }

    /// Calculate first-try success rate
    float firstTryRate() const {
        return total_requests > 0
            ? static_cast<float>(first_try_success) / total_requests
            : 0.0f;
    }
};

// ============================================================================
// Request Builder Functions
// ============================================================================

/// Build APRD (Auto-increment Physical Read) datagram
StoredDatagram buildAPRD(uint8_t idx, uint16_t slave_position, uint16_t ado, uint16_t length);

/// Build APWR (Auto-increment Physical Write) datagram
StoredDatagram buildAPWR(uint8_t idx, uint16_t slave_position, uint16_t ado, 
                          const uint8_t* data, uint16_t length);

/// Build FPRD (Configured Address Physical Read) datagram
StoredDatagram buildFPRD(uint8_t idx, uint16_t configured_addr, uint16_t ado, uint16_t length);

/// Build FPWR (Configured Address Physical Write) datagram
StoredDatagram buildFPWR(uint8_t idx, uint16_t configured_addr, uint16_t ado,
                          const uint8_t* data, uint16_t length);

/// Build BRD (Broadcast Read) datagram
StoredDatagram buildBRD(uint8_t idx, uint16_t ado, uint16_t length);

/// Build BWR (Broadcast Write) datagram
StoredDatagram buildBWR(uint8_t idx, uint16_t ado, const uint8_t* data, uint16_t length);

/// Build LRW (Logical Read/Write) datagram
StoredDatagram buildLRW(uint8_t idx, uint32_t logical_addr, const uint8_t* data, uint16_t length);

/// Build LRD (Logical Read) datagram
StoredDatagram buildLRD(uint8_t idx, uint32_t logical_addr, uint16_t length);

/// Build LWR (Logical Write) datagram
StoredDatagram buildLWR(uint8_t idx, uint32_t logical_addr, const uint8_t* data, uint16_t length);

// ============================================================================
// Send Function Type
// ============================================================================

/**
 * @brief Function signature for sending a datagram
 * 
 * The send function should:
 * 1. Build an EtherCAT frame with the datagram
 * 2. Send it via the network interface
 * 3. Return immediately (non-blocking)
 * 
 * @param dgram The datagram to send
 * @return True if send succeeded, false if send failed
 */
using SendFunction = std::function<bool(const StoredDatagram& dgram)>;

// ============================================================================
// RetryExecutor Class
// ============================================================================

/**
 * @brief Executes EtherCAT requests with automatic retry on timeout
 * 
 * Usage:
 * @code
 * RetryExecutor executor(router, sendFunc);
 * 
 * // Build a request
 * auto request = buildAPRD(idx, slave, 0x0130, 2);
 * 
 * // Execute with retry
 * uint8_t buffer[2];
 * auto result = executor.execute(request, filter, buffer, sizeof(buffer), policy);
 * if (result.success) {
 *     // Process buffer
 * }
 * @endcode
 */
class RetryExecutor {
public:
    /**
     * @brief Construct a retry executor
     * @param router The packet router to wait on
     * @param send_func Function to send datagrams
     */
    RetryExecutor(ConditionalPacketRouter& router, SendFunction send_func);

    /**
     * @brief Execute a request with automatic retry
     * 
     * @param request The datagram to send
     * @param filter Filter for matching response
     * @param buffer Buffer to receive response data
     * @param buffer_size Size of buffer
     * @param policy Retry policy
     * @return RetryResult with success status and response data
     */
    RetryResult execute(const StoredDatagram& request,
                        const PacketFilter& filter,
                        uint8_t* buffer,
                        size_t buffer_size,
                        const RetryPolicy& policy = RetryPolicy::standard());

    /**
     * @brief Execute request expecting specific WKC
     * 
     * Same as execute(), but also validates working counter
     */
    RetryResult executeWithWkc(const StoredDatagram& request,
                               const PacketFilter& filter,
                               uint8_t* buffer,
                               size_t buffer_size,
                               uint16_t expected_wkc,
                               const RetryPolicy& policy = RetryPolicy::standard());

    /// Get statistics
    RetryStats getStats() const { return stats_; }

    /// Reset statistics
    void resetStats() { stats_ = {}; }

    /// Log statistics
    void logStats() const;

private:
    ConditionalPacketRouter& router_;
    SendFunction send_func_;
    RetryStats stats_;
};

// ============================================================================
// High-Level Convenience Functions
// ============================================================================

/**
 * @brief Execute APRD with retry
 * 
 * @param executor Retry executor
 * @param idx Datagram index
 * @param slave_position Slave position (auto-increment address)
 * @param ado Register address
 * @param buffer Output buffer
 * @param length Number of bytes to read
 * @param policy Retry policy
 * @return RetryResult
 */
RetryResult retryAPRD(RetryExecutor& executor, uint8_t idx, uint16_t slave_position,
                      uint16_t ado, uint8_t* buffer, uint16_t length,
                      const RetryPolicy& policy = RetryPolicy::standard());

/**
 * @brief Execute APWR with retry and verify
 * 
 * Sends APWR, then reads back with APRD to verify write
 * 
 * @param executor Retry executor
 * @param idx Datagram index
 * @param slave_position Slave position
 * @param ado Register address
 * @param data Data to write
 * @param length Length of data
 * @param policy Retry policy
 * @return RetryResult with success=true if write and verify succeeded
 */
RetryResult retryAPWRVerify(RetryExecutor& executor, uint8_t idx, uint16_t slave_position,
                            uint16_t ado, const uint8_t* data, uint16_t length,
                            const RetryPolicy& policy = RetryPolicy::standard());

/**
 * @brief Execute BRD with retry
 */
RetryResult retryBRD(RetryExecutor& executor, uint8_t idx, uint16_t ado,
                     uint8_t* buffer, uint16_t length, uint16_t expected_wkc = 0,
                     const RetryPolicy& policy = RetryPolicy::standard());

/**
 * @brief Execute BWR with retry
 */
RetryResult retryBWR(RetryExecutor& executor, uint8_t idx, uint16_t ado,
                     const uint8_t* data, uint16_t length, uint16_t expected_wkc = 0,
                     const RetryPolicy& policy = RetryPolicy::standard());

/**
 * @brief Execute LRW with retry for cyclic PDO exchange
 * 
 * Uses fast retry policy by default for real-time performance
 */
RetryResult retryLRW(RetryExecutor& executor, uint8_t idx, uint32_t logical_addr,
                     const uint8_t* tx_data, uint8_t* rx_buffer, uint16_t length,
                     uint16_t expected_wkc,
                     const RetryPolicy& policy = RetryPolicy::fast());

}  // namespace raw
}  // namespace EtherCAT
