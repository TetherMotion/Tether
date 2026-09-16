/**
 * @file SDOManager.hpp
 * @brief EtherCAT Service Data Object (SDO) asynchronous access API
 * 
 * @details
 * This module provides asynchronous SDO (Service Data Object) access that can
 * be used while the realtime loop is running. SDOs are used for non-realtime
 * parameter access and configuration of EtherCAT slaves.
 * 
 * ## SDO vs PDO
 * 
 * | Feature | SDO | PDO |
 * |---------|-----|-----|
 * | Timing | Non-realtime, request/response | Realtime, cyclic |
 * | Use | Configuration, diagnostics | Process data |
 * | Protocol | CoE mailbox | Direct memory |
 * | Size | Up to 64KB (segmented) | Typically < 256 bytes |
 * 
 * ## Asynchronous Operation
 * 
 * Since the realtime loop is time-critical, SDO operations cannot block it.
 * This module provides a queue-based system:
 * 
 * 1. Application submits SDO request to queue
 * 2. Background task processes requests between PDO cycles
 * 3. Application polls for completion or uses callback
 * 
 * ## Instance-based API (SDOManager)
 * 
 * The SDOManager class owns all state (queue, worker thread, mailbox configs,
 * response map).  Network I/O is abstracted via the ISDOTransport interface,
 * allowing unit testing with mocks.  Multiple independent SDOManager instances
 * can co-exist.
 * 
 * @code
 * // Create manager with injected transport
 * SDOManager mgr(transport);
 * mgr.init();
 * mgr.configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);
 * 
 * // Synchronous read
 * uint16_t controlword;
 * mgr.readSync(0, 0x6040, 0, &controlword, sizeof(controlword), 1000);
 * 
 * mgr.deinit();
 * @endcode
 * 
 * ## Thread Safety
 * 
 * The SDO queue is thread-safe. Requests can be submitted from any task.
 * Callbacks are invoked from the SDO processing task context.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"
#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

namespace EtherCAT {

class PDOManager; // forward

namespace SDO {

// ============================================================================
// Constants and Configuration
// ============================================================================

/**
 * @brief Maximum SDO data size for a single transfer
 * 
 * For larger transfers, segmented transfer is used automatically.
 */
constexpr size_t kMaxSDODataSize = ECAT_SDO_MANAGER_MAX_DATA_SIZE;

/**
 * @brief Maximum number of pending SDO requests in the queue
 */
constexpr size_t kMaxSDOQueueDepth = ECAT_SDO_MANAGER_QUEUE_DEPTH;

/**
 * @brief Default timeout for SDO operations in milliseconds
 */
// Default SDO timeout increased to 3000 ms to account for slower mailbox/CoE transactions on some hardware
constexpr uint32_t kDefaultSDOTimeoutMs = ECAT_SDO_MANAGER_DEFAULT_TIMEOUT_MS;

// ============================================================================
// SDO Error Codes
// ============================================================================
//
// The canonical SDOAbortCode enum and sdoAbortCodeStr() now live in
// tether/ethercat/SDOAbortCodes.hpp at EtherCAT scope, shared with the CoE
// layer and the slave-emulation object dictionary.

// ============================================================================
// SDO Request/Response Structures
// ============================================================================

/**
 * @brief SDO operation type
 */
enum class SDOOperation : uint8_t {
    Upload = 0,    ///< Read from slave (slave→master)
    Download = 1   ///< Write to slave (master→slave)
};

/**
 * @brief SDO request status
 */
enum class SDOStatus : uint8_t {
    Pending = 0,   ///< Request is queued, not yet processed
    InProgress,    ///< Request is being processed
    Complete,      ///< Request completed successfully
    Failed,        ///< Request failed (check abort_code)
    Timeout,       ///< Request timed out
    Cancelled      ///< Request was cancelled
};

// Forward declaration for callback
struct SDOResponse;

/**
 * @brief Callback function type for SDO completion notification
 * 
 * @param response The completed SDO response
 * 
 * @note Callbacks are invoked from the SDO task context. Keep processing
 * minimal to avoid blocking other SDO operations.
 */
using SDOCallback = std::function<void(const SDOResponse& response)>;

/**
 * @brief SDO request structure
 * 
 * Fill this structure and submit to sdo_queue_request() to initiate
 * an asynchronous SDO operation.
 */
struct SDORequest {
    // Target specification
    uint16_t slave_index;      ///< Target slave (0-based index)
    uint16_t index;            ///< Object dictionary index (e.g., 0x6040)
    uint8_t  subindex;         ///< Object dictionary subindex
    
    // Operation
    SDOOperation operation;    ///< Upload (read) or Download (write)
    
    // Data for download operations
    uint8_t  data[kMaxSDODataSize]; ///< Data buffer
    size_t   data_size;        ///< Size of data to write (download) or max to read (upload)
    
    // Options
    uint32_t timeout_ms;       ///< Timeout in milliseconds (0 = use default)
    
    // Completion notification (optional)
    SDOCallback callback;      ///< Called when operation completes
    void*    user_context;     ///< User-provided context pointer
    
    // Internal use
    uint32_t request_id;       ///< Assigned by the queue system
};

/**
 * @brief SDO response structure
 * 
 * Contains the result of an SDO operation. For uploads, the data
 * field contains the read value. For downloads, it confirms the write.
 */
struct SDOResponse {
    // Request identification
    uint32_t request_id;       ///< Matches the original request
    uint16_t slave_index;      ///< Target slave
    uint16_t index;            ///< Object dictionary index
    uint8_t  subindex;         ///< Object dictionary subindex
    SDOOperation operation;    ///< Upload or Download
    
    // Result
    SDOStatus status;          ///< Completion status
    SDOAbortCode abort_code;   ///< Abort code if failed
    
    // Data for upload operations
    uint8_t  data[kMaxSDODataSize]; ///< Received data (upload) or echoed data (download)
    size_t   data_size;        ///< Actual size of data
    
    // Context
    void*    user_context;     ///< From the original request
    
    // Timing
    uint32_t duration_ms;      ///< Time taken for the operation
    
    // Helper to check success
    bool success() const { return status == SDOStatus::Complete; }
};

// ============================================================================
// Transport Abstraction
// ============================================================================

/**
 * @brief Abstract transport interface for SDO I/O
 *
 * Implementations provide the actual CoE mailbox read/write operations.
 * A concrete implementation backed by Raw EtherCAT frames would wrap
 * coe_sdo_upload / coe_sdo_download.  A mock implementation is used
 * for unit tests.
 */
class ISDOTransport {
public:
    virtual ~ISDOTransport() = default;

    /**
     * @brief Perform a CoE SDO upload (read from slave)
     *
     * @param slave_index    Zero-based slave index
     * @param mbx_counter    Mailbox counter (incremented by implementation)
     * @param mbx_wr_addr    Mailbox write address (Master→Slave)
     * @param mbx_wr_len     Mailbox write length
     * @param mbx_rd_addr    Mailbox read address (Slave→Master)
     * @param mbx_rd_len     Mailbox read length
     * @param index          Object dictionary index
     * @param sub            Object dictionary subindex
     * @param out            Output data buffer
     * @param out_cap        Buffer capacity
     * @param out_len        Actual bytes read
     * @return true on success
     */
    virtual bool sdoUpload(uint16_t slave_index, uint8_t* mbx_counter,
                           uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                           uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                           uint16_t index, uint8_t sub,
                           uint8_t* out, size_t out_cap, size_t* out_len,
                           bool diag_enabled = false,
                           unsigned int poll_interval_ms = 5,
                           unsigned int transaction_timeout_ms = 1000) = 0;

    /**
     * @brief Perform a CoE SDO download (write to slave)
     *
     * @param slave_index    Zero-based slave index
     * @param mbx_counter    Mailbox counter (incremented by implementation)
     * @param mbx_wr_addr    Mailbox write address (Master→Slave)
     * @param mbx_wr_len     Mailbox write length
     * @param mbx_rd_addr    Mailbox read address (Slave→Master)
     * @param mbx_rd_len     Mailbox read length
     * @param index          Object dictionary index
     * @param sub            Object dictionary subindex
     * @param data           Data to write
     * @param data_len       Data length
     * @return true on success
     */
    virtual bool sdoDownload(uint16_t slave_index, uint8_t* mbx_counter,
                             uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                             uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                             uint16_t index, uint8_t sub,
                             const uint8_t* data, size_t data_len,
                             bool diag_enabled = false,
                             unsigned int poll_interval_ms = 5,
                             unsigned int transaction_timeout_ms = 1000) = 0;

    /**
     * @brief Get current monotonic time in microseconds
     */
    virtual uint64_t getMicroseconds() = 0;

    /**
     * @brief Read an ESC register from a slave (e.g. AL_STATUS 0x0130)
     *
     * Default implementation returns false. Concrete transports override
     * to provide actual register access.
     *
     * @param slave_index  Zero-based slave index
     * @param reg_addr     ESC register address (e.g. 0x0130)
     * @param out          Output buffer
     * @param len          Number of bytes to read
     * @param timeout_ms   Read timeout in milliseconds
     * @return true on success
     */
    virtual bool readSlaveRegister(uint16_t slave_index, uint16_t reg_addr,
                                   void* out, uint16_t len,
                                   unsigned int timeout_ms = 200) {
        (void)slave_index; (void)reg_addr; (void)out; (void)len; (void)timeout_ms;
        return false;
    }

    /**
     * @brief Return the CoE SDO abort code reported by the slave on the most
     * recent sdoUpload/sdoDownload call.
     *
     * @return 0 if the last call succeeded or failed for a non-abort reason
     *         (transport/timeout); otherwise the standard 32-bit CoE SDO
     *         abort code (e.g. 0x06070010 for "data type / length mismatch").
     *
     * The default implementation returns 0, so mock transports used in unit
     * tests do not need to override this unless they specifically want to
     * simulate slave aborts. Real transports override to read the value
     * captured by the underlying CoE SDO channel.
     *
     * @note SDO operations on a slave are serialized by CoEManager, so this
     *       is safe to read immediately after a call returns false.
     */
    virtual uint32_t lastAbortCode() const { return 0; }

    /**
     * @brief Return true if master-level cancellation has been requested
     *        (e.g. Master::requestCancel() from a signal handler or during
     *        Master::stop()).
     *
     * CoEManager checks this to fail newly submitted SDO requests
     * immediately and to abort in-flight retry loops quietly — failures
     * during shutdown are expected and must not produce error log spam.
     *
     * The default implementation returns false, so mock transports used in
     * unit tests do not need to override it.
     */
    virtual bool isCancelRequested() const { return false; }
};

// ============================================================================
// Emergency Message Handling
// ============================================================================

struct EmergencyMessage {
    uint16_t slave_index;
    uint16_t error_code;
    uint8_t  error_register;
    uint8_t  data[5];
    uint64_t timestamp_ns;
};

using EmergencyCallback = std::function<void(const EmergencyMessage& emg)>;

} // namespace SDO
} // namespace EtherCAT

