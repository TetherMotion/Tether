/**
 * @file WriteVerify.hpp
 * @brief Write-and-Verify Configuration Writes for EtherCAT
 *
 * @details
 * This module provides write-and-verify functionality for all EtherCAT
 * configuration writes. After writing data to a slave register, it
 * reads back the value and verifies it matches what was written.
 *
 * This helps detect:
 * - Communication errors during write
 * - Slaves that silently reject writes
 * - Register protection issues
 * - Timing issues where writes haven't taken effect
 *
 * ## Architecture
 *
 * The WriteVerifier class owns all state internally.  Network I/O is
 * abstracted via the IWriteVerifyTransport interface, allowing unit
 * testing with a mock transport and supporting multiple independent
 * instances (no global state, no singletons).
 *
 * ## Usage
 *
 * ```cpp
 * WriteVerifier wv(transport);
 * auto result = wv.apwrVerify(adp, reg, &data, len, timeout);
 * if (!result.success) {
 *     // Handle verification failure
 * }
 * ```
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

#include "tether/ethercat/TetherConfig.hpp"

namespace EtherCAT {
namespace Verify {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Write-verify configuration
 */
struct WriteVerifyConfig {
    uint32_t retry_count;          ///< Number of retries on verification failure
    uint32_t retry_delay_ms;       ///< Delay between retries
    uint32_t read_delay_ms;        ///< Delay before read-back verification
    bool log_failures;             ///< Log verification failures

    static WriteVerifyConfig defaults() {
        return {
            .retry_count = 3,
            .retry_delay_ms = 10,
            .read_delay_ms = 1,
            .log_failures = true
        };
    }
};

/**
 * @brief Result of a write-verify operation
 */
struct WriteVerifyResult {
    bool success;                  ///< Write and verify succeeded
    bool write_ok;                 ///< Initial write succeeded (WKC > 0)
    bool verify_ok;                ///< Verification read-back matched
    uint16_t write_wkc;            ///< WKC from write operation
    uint16_t read_wkc;             ///< WKC from verify read operation
    uint32_t attempts;             ///< Number of attempts made
    size_t mismatch_offset;        ///< First byte that mismatched (if verify failed)
    uint8_t expected_byte;         ///< Expected byte at mismatch_offset
    uint8_t actual_byte;           ///< Actual byte at mismatch_offset

    static WriteVerifyResult Success(uint16_t wkc, uint32_t attempts) {
        WriteVerifyResult r = {};
        r.success = true;
        r.write_ok = true;
        r.verify_ok = true;
        r.write_wkc = wkc;
        r.read_wkc = wkc;
        r.attempts = attempts;
        return r;
    }

    static WriteVerifyResult WriteFailed(uint16_t wkc, uint32_t attempts) {
        WriteVerifyResult r = {};
        r.success = false;
        r.write_ok = false;
        r.write_wkc = wkc;
        r.attempts = attempts;
        return r;
    }

    static WriteVerifyResult VerifyFailed(uint16_t write_wkc, uint16_t read_wkc,
                                           uint32_t attempts, size_t offset,
                                           uint8_t expected, uint8_t actual) {
        WriteVerifyResult r = {};
        r.success = false;
        r.write_ok = true;
        r.verify_ok = false;
        r.write_wkc = write_wkc;
        r.read_wkc = read_wkc;
        r.attempts = attempts;
        r.mismatch_offset = offset;
        r.expected_byte = expected;
        r.actual_byte = actual;
        return r;
    }
};

/**
 * @brief Write-verify statistics
 */
struct WriteVerifyStats {
    uint64_t total_writes;         ///< Total write operations attempted
    uint64_t successful_writes;    ///< Writes that succeeded on first try
    uint64_t verify_failures;      ///< Verification failures
    uint64_t write_failures;       ///< Write failures (WKC = 0)
    uint64_t retries;              ///< Total retry attempts
    uint64_t eventual_success;     ///< Writes that succeeded after retry
    uint64_t permanent_failures;   ///< Writes that failed after all retries
};

// ============================================================================
// Datagram Response (for transport interface)
// ============================================================================

/**
 * @brief Response from a datagram operation
 */
struct DatagramResponse {
    uint16_t wkc;                  ///< Working Counter
    uint16_t datalen;              ///< Data length
    uint8_t data[256];             ///< Payload data
};

// ============================================================================
// Transport Abstraction
// ============================================================================

/**
 * @brief EtherCAT command constants used by the transport interface
 */
struct EtherCATCmd {
    static constexpr uint8_t APRD = 0x01;  ///< Auto Position Read
    static constexpr uint8_t APWR = 0x02;  ///< Auto Position Write
    static constexpr uint8_t BWR  = 0x08;  ///< Broadcast Write
};

/**
 * @brief Abstract transport interface for write-verify I/O
 *
 * Implementations provide the actual datagram send/receive operations.
 * A concrete implementation backed by Raw EtherCAT frames would wrap
 * alloc_idx / send_single_datagram / wait_for_response_idx / ec_aprd.
 * A mock implementation is used for unit tests.
 */
class IWriteVerifyTransport {
public:
    virtual ~IWriteVerifyTransport() = default;

    /**
     * @brief Allocate a datagram index for tracking responses
     * @return Index value
     */
    virtual uint8_t allocIdx() = 0;

    /**
     * @brief Send a single EtherCAT datagram
     *
     * @param cmd     EtherCAT command (e.g. APWR, BWR)
     * @param idx     Datagram index (from allocIdx)
     * @param adp     Address position (slave)
     * @param ado     Address offset (register)
     * @param data    Data to send
     * @param len     Data length
     * @param roundtrip Whether to wait for the frame to return
     * @return true on successful send
     */
    virtual bool sendDatagram(uint8_t cmd, uint8_t idx, uint16_t adp,
                              uint16_t ado, const void* data, uint16_t len,
                              bool roundtrip) = 0;

    /**
     * @brief Wait for a datagram response by index
     *
     * @param idx        Expected datagram index
     * @param timeout_ms Timeout in milliseconds
     * @param response   Output response structure
     * @return true if response received within timeout
     */
    virtual bool waitForResponse(uint8_t idx, unsigned int timeout_ms,
                                 DatagramResponse& response) = 0;

    /**
     * @brief Read a register from a slave (APRD)
     *
     * This is used for verification read-back.
     *
     * @param adp        Address position (slave)
     * @param ado        Address offset (register)
     * @param out        Output buffer
     * @param len        Number of bytes to read
     * @param timeout_ms Timeout in milliseconds
     * @return true on success
     */
    virtual bool readRegister(uint16_t adp, uint16_t ado, void* out,
                              uint16_t len, unsigned int timeout_ms) = 0;

    /**
     * @brief Blocking delay
     * @param ms Milliseconds to wait
     */
    virtual void delayMs(unsigned int ms) = 0;
};

// ============================================================================
// WriteVerifier — owns all state (no globals)
// ============================================================================

/**
 * @brief Instance-based write-verifier.
 *
 * Each WriteVerifier owns its own configuration, enabled flag, and
 * statistics.  Multiple independent instances can co-exist.  Network I/O
 * is performed through the injected IWriteVerifyTransport.
 */
class WriteVerifier {
public:
    /// Maximum data length for a single write-verify operation.
    /// Configurable via ECAT_WRITE_VERIFY_MAX_DATA_LEN in TetherConfig.hpp.
    /// This is a Tether-internal stack buffer limit.
    static constexpr uint16_t kMaxDataLen = ECAT_WRITE_VERIFY_MAX_DATA_LEN;

    /**
     * @brief Construct a WriteVerifier with default config.
     *
     * The transport reference must outlive the WriteVerifier.
     */
    explicit WriteVerifier(IWriteVerifyTransport& transport);

    /**
     * @brief Construct a WriteVerifier with custom config.
     *
     * @param transport Transport implementation
     * @param config    Initial configuration
     */
    WriteVerifier(IWriteVerifyTransport& transport, const WriteVerifyConfig& config);

    ~WriteVerifier() = default;

    // Non-copyable
    WriteVerifier(const WriteVerifier&) = delete;
    WriteVerifier& operator=(const WriteVerifier&) = delete;

    // Movable
    WriteVerifier(WriteVerifier&&) = default;
    WriteVerifier& operator=(WriteVerifier&&) = default;

    // ----- Configuration -----

    /** @brief Set the write-verify configuration */
    void setConfig(const WriteVerifyConfig& config);

    /** @brief Get the current configuration */
    const WriteVerifyConfig& config() const;

    /** @brief Enable or disable write-verify */
    void setEnabled(bool enabled);

    /** @brief Check if write-verify is enabled */
    bool isEnabled() const;

    // ----- Statistics -----

    /** @brief Get current statistics */
    const WriteVerifyStats& stats() const;

    /** @brief Reset statistics to zero */
    void resetStats();

    /** @brief Log statistics summary */
    void logStats() const;

    // ----- Write-Verify Operations -----

    /**
     * @brief Write data and verify by read-back (APWR)
     *
     * @param adp        Address position (slave)
     * @param ado        Address offset (register)
     * @param data       Data to write
     * @param len        Data length
     * @param timeout_ms Timeout for each operation
     * @return WriteVerifyResult with success status and details
     */
    WriteVerifyResult apwrVerify(uint16_t adp, uint16_t ado,
                                  const void* data, uint16_t len,
                                  unsigned int timeout_ms);

    /**
     * @brief Write 16-bit value and verify
     */
    WriteVerifyResult apwrVerifyU16(uint16_t adp, uint16_t ado,
                                     uint16_t value, unsigned int timeout_ms);

    /**
     * @brief Write 32-bit value and verify
     */
    WriteVerifyResult apwrVerifyU32(uint16_t adp, uint16_t ado,
                                     uint32_t value, unsigned int timeout_ms);

    /**
     * @brief Write 64-bit value and verify
     */
    WriteVerifyResult apwrVerifyU64(uint16_t adp, uint16_t ado,
                                     uint64_t value, unsigned int timeout_ms);

    /**
     * @brief Broadcast write and verify on specific slave (BWR + APRD verify)
     *
     * @param slave_to_verify  ADP of the specific slave to verify
     * @param ado              Address offset (register)
     * @param data             Data to write
     * @param len              Data length
     * @param timeout_ms       Timeout for each operation
     * @return WriteVerifyResult
     */
    WriteVerifyResult bwrVerify(uint16_t slave_to_verify, uint16_t ado,
                                 const void* data, uint16_t len,
                                 unsigned int timeout_ms);

private:
    IWriteVerifyTransport& transport_;
    WriteVerifyConfig config_;
    bool enabled_ = true;
    WriteVerifyStats stats_ = {};

    static bool compareBuffers(const uint8_t* expected, const uint8_t* actual,
                               size_t len, size_t& offset);
};

// ============================================================================
// Free-function API (delegates to a WriteVerifier instance)
// ============================================================================

/** @brief Write and verify via APWR */
inline WriteVerifyResult apwr_verify(WriteVerifier& wv,
                                      uint16_t adp, uint16_t ado,
                                      const void* data, uint16_t len,
                                      unsigned int timeout_ms) {
    return wv.apwrVerify(adp, ado, data, len, timeout_ms);
}

/** @brief Write and verify 16-bit value */
inline WriteVerifyResult apwr_verify_u16(WriteVerifier& wv,
                                          uint16_t adp, uint16_t ado,
                                          uint16_t value,
                                          unsigned int timeout_ms) {
    return wv.apwrVerifyU16(adp, ado, value, timeout_ms);
}

/** @brief Write and verify 32-bit value */
inline WriteVerifyResult apwr_verify_u32(WriteVerifier& wv,
                                          uint16_t adp, uint16_t ado,
                                          uint32_t value,
                                          unsigned int timeout_ms) {
    return wv.apwrVerifyU32(adp, ado, value, timeout_ms);
}

/** @brief Write and verify 64-bit value */
inline WriteVerifyResult apwr_verify_u64(WriteVerifier& wv,
                                          uint16_t adp, uint16_t ado,
                                          uint64_t value,
                                          unsigned int timeout_ms) {
    return wv.apwrVerifyU64(adp, ado, value, timeout_ms);
}

/** @brief Broadcast write and verify on specific slave */
inline WriteVerifyResult bwr_verify(WriteVerifier& wv,
                                     uint16_t slave_to_verify, uint16_t ado,
                                     const void* data, uint16_t len,
                                     unsigned int timeout_ms) {
    return wv.bwrVerify(slave_to_verify, ado, data, len, timeout_ms);
}

/** @brief Set configuration on a WriteVerifier */
inline void set_config(WriteVerifier& wv, const WriteVerifyConfig& config) {
    wv.setConfig(config);
}

/** @brief Get configuration from a WriteVerifier */
inline const WriteVerifyConfig& get_config(WriteVerifier& wv) {
    return wv.config();
}

/** @brief Enable/disable write-verify on a WriteVerifier */
inline void set_enabled(WriteVerifier& wv, bool enabled) {
    wv.setEnabled(enabled);
}

/** @brief Check if write-verify is enabled on a WriteVerifier */
inline bool is_enabled(WriteVerifier& wv) {
    return wv.isEnabled();
}

/** @brief Get statistics from a WriteVerifier */
inline const WriteVerifyStats& get_stats(WriteVerifier& wv) {
    return wv.stats();
}

/** @brief Reset statistics on a WriteVerifier */
inline void reset_stats(WriteVerifier& wv) {
    wv.resetStats();
}

/** @brief Log statistics from a WriteVerifier */
inline void log_stats(WriteVerifier& wv) {
    wv.logStats();
}

} // namespace Verify
} // namespace EtherCAT
