/**
 * @file EtherCATSDO.hpp
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
#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
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
constexpr size_t kMaxSDODataSize = 256;

/**
 * @brief Maximum number of pending SDO requests in the queue
 */
constexpr size_t kMaxSDOQueueDepth = 16;

/**
 * @brief Default timeout for SDO operations in milliseconds
 */
// Default SDO timeout increased to 3000 ms to account for slower mailbox/CoE transactions on some hardware
constexpr uint32_t kDefaultSDOTimeoutMs = 1000;

// ============================================================================
// SDO Error Codes
// ============================================================================

/**
 * @brief SDO abort codes (CoE specification)
 * 
 * These are the standard abort codes returned by slaves when an SDO
 * operation fails. The code indicates what went wrong.
 */
enum class SDOAbortCode : uint32_t {
    Success                  = 0x00000000, ///< No error
    ToggleBitNotChanged      = 0x05030000, ///< Toggle bit not alternated
    Timeout                  = 0x05040000, ///< SDO protocol timeout
    InvalidCommand           = 0x05040001, ///< Command specifier unknown
    InvalidBlockSize         = 0x05040002, ///< Invalid block size
    InvalidSequenceNumber    = 0x05040003, ///< Invalid sequence number
    CrcError                 = 0x05040004, ///< CRC error
    OutOfMemory              = 0x05040005, ///< Out of memory
    UnsupportedAccess        = 0x06010000, ///< Unsupported access
    ReadOnlyObject           = 0x06010001, ///< Write to read-only object
    WriteOnlyObject          = 0x06010002, ///< Read from write-only object
    ObjectNotFound           = 0x06020000, ///< Object does not exist
    PdoMappingError          = 0x06040041, ///< Object cannot be mapped to PDO
    PdoLengthExceeded        = 0x06040042, ///< Number/length would exceed PDO
    ParameterIncompatible    = 0x06040043, ///< Parameter incompatibility
    InternalError            = 0x06040047, ///< General internal incompatibility
    HardwareError            = 0x06060000, ///< Hardware error
    DataTypeMismatch         = 0x06070010, ///< Data type mismatch, length mismatch
    DataTypeTooLong          = 0x06070012, ///< Data type mismatch, length too high
    DataTypeTooShort         = 0x06070013, ///< Data type mismatch, length too low
    SubindexNotFound         = 0x06090011, ///< Subindex does not exist
    InvalidValue             = 0x06090030, ///< Invalid value for parameter
    ValueTooHigh             = 0x06090031, ///< Value too high
    ValueTooLow              = 0x06090032, ///< Value too low
    MaxLessThanMin           = 0x06090036, ///< Maximum less than minimum
    ResourceNotAvailable     = 0x060A0023, ///< Resource not available
    GeneralError             = 0x08000000, ///< General error
    TransferAborted          = 0x08000020, ///< Data transfer aborted
    LocalControlError        = 0x08000021, ///< Local control error
    DeviceStateError         = 0x08000022, ///< Wrong device state
    DictionaryNotPresent     = 0x08000023, ///< Object dictionary not present
    NoDataAvailable          = 0x08000024, ///< No data available
};

/**
 * @brief Convert SDO abort code to human-readable string
 */
const char* sdo_abort_code_str(SDOAbortCode code);

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
                           uint8_t* out, size_t out_cap, size_t* out_len) = 0;

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
                             const uint8_t* data, size_t data_len) = 0;

    /**
     * @brief Get current monotonic time in microseconds
     */
    virtual uint64_t getMicroseconds() = 0;
};

// ============================================================================
// SDOManager — owns all state (no globals)
// ============================================================================

/**
 * @brief Instance-based SDO manager.
 *
 * Each SDOManager owns its own request queue, worker thread, slave
 * mailbox configurations, and response map.  Multiple independent
 * instances can co-exist.  Network I/O is performed through the
 * injected ISDOTransport.
 */
class SDOManager {
public:
    static constexpr size_t kMaxSlaves = 16;
    static constexpr size_t kQueueDepth = 16;
    /// Maximum age (microseconds) before a queued request times out
    static constexpr uint64_t kQueueTimeoutUs = 5000000ULL;

    /**
     * @brief Construct an SDOManager with the given transport.
     *
     * The transport reference must outlive the SDOManager.
     */
    explicit SDOManager(ISDOTransport& transport);

    /**
     * @brief Destructor — calls deinit() if still initialized.
     */
    ~SDOManager();

    // Non-copyable
    SDOManager(const SDOManager&) = delete;
    SDOManager& operator=(const SDOManager&) = delete;

    // ----- Lifecycle -----

    /**
     * @brief Initialize the manager and start the worker thread.
     * @return true on success
     */
    bool init();

    /**
     * @brief Stop the worker thread and clear all state.
     */
    void deinit();

    /**
     * @brief Check if the manager is initialized (worker running).
     */
    bool isInitialized() const;

    // ----- Slave Mailbox Configuration -----

    /**
     * @brief Configure slave mailbox information for SDO
     *
     * @param slave_index  Slave index (0-based)
     * @param mbx_write_addr Mailbox write address (Master→Slave)
     * @param mbx_write_len  Mailbox write length
     * @param mbx_read_addr  Mailbox read address (Slave→Master)
     * @param mbx_read_len   Mailbox read length
     */
    void configureSlaveMailbox(uint16_t slave_index,
                               uint16_t mbx_write_addr, uint16_t mbx_write_len,
                               uint16_t mbx_read_addr, uint16_t mbx_read_len);

    /**
     * @brief Query configured mailbox info for a slave
     * @return true if the slave has been configured
     */
    bool getSlaveMailbox(uint16_t slave_index,
                         uint16_t* mbx_write_addr, uint16_t* mbx_write_len,
                         uint16_t* mbx_read_addr, uint16_t* mbx_read_len) const;

    // ----- Async API -----

    /**
     * @brief Queue an asynchronous SDO request
     * @return Request ID on success, 0 on failure (queue full)
     */
    uint32_t queueRequest(SDORequest& request);

    /**
     * @brief Check if an SDO request has completed
     */
    bool isComplete(uint32_t request_id);

    /**
     * @brief Get the response for a completed request (removes it)
     */
    bool getResponse(uint32_t request_id, SDOResponse& response);

    /**
     * @brief Get the number of pending SDO requests
     */
    size_t pendingCount();

    // ----- Sync API -----

    /**
     * @brief Read an SDO value synchronously (blocking)
     */
    bool readSync(uint16_t slave_index, uint16_t index, uint8_t sub,
                  void* data, size_t max_size, uint32_t timeout_ms,
                  size_t* actual_size = nullptr);

    /**
     * @brief Write an SDO value synchronously (blocking)
     */
    bool writeSync(uint16_t slave_index, uint16_t index, uint8_t sub,
                   const void* data, size_t size, uint32_t timeout_ms);

    // ----- Type-Safe Sync Helpers -----

    bool readU8(uint16_t slave, uint16_t idx, uint8_t sub,
                uint8_t& value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return readSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool readU16(uint16_t slave, uint16_t idx, uint8_t sub,
                 uint16_t& value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return readSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool readU32(uint16_t slave, uint16_t idx, uint8_t sub,
                 uint32_t& value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return readSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool readI32(uint16_t slave, uint16_t idx, uint8_t sub,
                 int32_t& value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return readSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool writeU8(uint16_t slave, uint16_t idx, uint8_t sub,
                 uint8_t value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return writeSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool writeU16(uint16_t slave, uint16_t idx, uint8_t sub,
                  uint16_t value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return writeSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool writeU32(uint16_t slave, uint16_t idx, uint8_t sub,
                  uint32_t value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return writeSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }
    bool writeI32(uint16_t slave, uint16_t idx, uint8_t sub,
                  int32_t value, uint32_t timeout_ms = kDefaultSDOTimeoutMs) {
        return writeSync(slave, idx, sub, &value, sizeof(value), timeout_ms);
    }

    // ----- Diagnostics -----

    // ----- PDO integration -----

    /** Inject PDOManager so SDO can read slave SM configs from it. */
    void setPDOManager(PDOManager* mgr) { pdo_manager_ = mgr; }
    PDOManager* pdoManager() const { return pdo_manager_; }

    void setDiagEnabled(bool enabled);
    bool isDiagEnabled() const;

private:
    struct SlaveMailboxConfig {
        uint16_t write_addr = 0;
        uint16_t write_len  = 0;
        uint16_t read_addr  = 0;
        uint16_t read_len   = 0;
        uint8_t  mbx_counter = 1;
        bool     configured  = false;
    };

    struct SyncContext {
        std::mutex mutex;
        std::condition_variable cv;
        bool complete = false;
        SDOResponse response{};
    };

    struct QueuedRequest {
        SDORequest request{};
        std::shared_ptr<SyncContext> sync_ctx{};
        uint64_t enqueue_time_us = 0;
    };

    void workerFunction();
    bool executeRequest(const SDORequest& req, SDOResponse& resp);

    ISDOTransport& transport_;

    std::deque<QueuedRequest> queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::thread worker_thread_;
    volatile bool shutdown_requested_ = false;
    std::atomic<uint32_t> next_request_id_{1};

    SlaveMailboxConfig slave_mbx_[kMaxSlaves]{};

    mutable std::mutex responses_mutex_;
    std::map<uint32_t, SDOResponse> completed_responses_;

    std::atomic<bool> diag_enabled_{false};
    bool initialized_ = false;

    PDOManager* pdo_manager_ = nullptr;
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

