/**
 * @file EtherCATFoE.hpp
 * @brief File over EtherCAT (FoE) protocol implementation
 * 
 * @details
 * FoE (File over EtherCAT) provides file transfer capabilities over the
 * EtherCAT mailbox protocol. It's primarily used for:
 * 
 * - **Firmware Updates**: Upload new firmware to slaves in bootloader mode
 * - **Configuration Files**: Transfer configuration data to/from slaves
 * - **Log Retrieval**: Download diagnostic logs from slaves
 * - **Recipe Management**: Upload production recipes to machine controllers
 * 
 * ## Architecture
 * 
 * The FoEManager class owns ALL state internally.  Network I/O is
 * abstracted via the IFoETransport interface, allowing unit testing
 * with a mock transport and supporting multiple independent instances
 * (no global state, no singletons).
 * 
 * @note Configure FoE in EtherCATConfig.hpp with ECAT_FEATURE_FOE_ENABLED
 */

#pragma once

#include "EtherCATConfig.hpp"

#if ECAT_FEATURE_FOE_ENABLED

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <atomic>
#include <array>
#include <mutex>
#include <thread>

namespace EtherCAT {
namespace FoE {

// ============================================================================
// Constants
// ============================================================================

/** @brief FoE mailbox type identifier */
constexpr uint8_t kFoEMailboxType = 0x04;

/** @brief Maximum FoE filename length */
constexpr size_t kMaxFilename = ECAT_FOE_MAX_FILENAME;

/** @brief FoE data buffer size */
constexpr size_t kBufferSize = ECAT_FOE_BUFFER_SIZE;

/** @brief Maximum concurrent transfers */
constexpr size_t kMaxTransfers = ECAT_FOE_MAX_TRANSFERS;

// ============================================================================
// FoE Opcodes
// ============================================================================

/**
 * @brief FoE operation codes
 */
enum class FoEOpcode : uint8_t {
    RRQ   = 1,  ///< Read request (download from slave)
    WRQ   = 2,  ///< Write request (upload to slave)
    DATA  = 3,  ///< Data block
    ACK   = 4,  ///< Acknowledge
    ERROR = 5,  ///< Error
    BUSY  = 6,  ///< Slave busy
};

/**
 * @brief Convert FoE opcode to string
 */
const char* foe_opcode_string(FoEOpcode op);

// ============================================================================
// FoE Error Codes
// ============================================================================

/**
 * @brief FoE error codes returned by slaves
 */
enum class FoEError : uint32_t {
    SUCCESS          = 0x0000,  ///< No error
    NOT_FOUND        = 0x8001,  ///< File not found
    ACCESS_DENIED    = 0x8002,  ///< Access denied
    DISK_FULL        = 0x8003,  ///< Disk/storage full
    ILLEGAL_OP       = 0x8004,  ///< Illegal operation
    PACKET_NUM       = 0x8005,  ///< Wrong packet number
    ALREADY_EXISTS   = 0x8006,  ///< File already exists
    NO_USER          = 0x8007,  ///< No user logged in
    BOOTSTRAP_ONLY   = 0x8008,  ///< Only available in bootstrap
    NOT_BOOTSTRAP    = 0x8009,  ///< Not in bootstrap state
    NO_RIGHTS        = 0x800A,  ///< Insufficient rights
    PROGRAM_ERROR    = 0x800B,  ///< Programming error
    CHECKSUM_ERROR   = 0x800C,  ///< Checksum mismatch
    
    // Internal errors (not from slave)
    TIMEOUT          = 0xFF01,  ///< Communication timeout
    MAILBOX_ERROR    = 0xFF02,  ///< Mailbox communication failed
    LOCAL_FILE_ERROR = 0xFF03,  ///< Local filesystem error
    INVALID_STATE    = 0xFF04,  ///< Invalid protocol state
    BUFFER_OVERFLOW  = 0xFF05,  ///< Buffer too small
    CANCELLED        = 0xFF06,  ///< Transfer cancelled
    NOT_INITIALIZED  = 0xFF07,  ///< FoE not initialized
};

/**
 * @brief Get human-readable error string
 */
const char* foe_error_string(FoEError error);

// ============================================================================
// Transfer Progress
// ============================================================================

/**
 * @brief Transfer progress information
 */
struct FoEProgress {
    uint16_t slave_index;       ///< Slave being accessed
    const char* filename;       ///< File being transferred
    uint32_t bytes_transferred; ///< Bytes transferred so far
    uint32_t total_bytes;       ///< Total file size (0 if unknown)
    uint32_t block_number;      ///< Current block number
    bool is_upload;             ///< true = upload to slave, false = download
    float throughput_bps;       ///< Current throughput in bytes/sec
};

/**
 * @brief Progress callback type
 * 
 * Called periodically during transfer. Return false to cancel.
 */
using FoEProgressCallback = std::function<bool(const FoEProgress&)>;

// ============================================================================
// Transfer Configuration
// ============================================================================

/**
 * @brief FoE transfer configuration
 */
struct FoETransferConfig {
    uint16_t slave_index;           ///< Target slave (0-based index)
    const char* filename;           ///< Remote filename on slave
    uint32_t password;              ///< Password (0 if none)
    uint32_t timeout_ms;            ///< Per-block timeout (default: ECAT_FOE_TIMEOUT_MS)
    FoEProgressCallback progress_callback;  ///< Optional progress callback
    
    /** @brief Default constructor with sensible defaults */
    FoETransferConfig() 
        : slave_index(0)
        , filename(nullptr)
        , password(ECAT_FOE_DEFAULT_PASSWORD)
        , timeout_ms(ECAT_FOE_TIMEOUT_MS)
        , progress_callback(nullptr)
    {}
};

// ============================================================================
// Transfer Result
// ============================================================================

/**
 * @brief Result of a FoE transfer operation
 */
struct FoEResult {
    bool success;               ///< true if transfer completed successfully
    FoEError error_code;        ///< Error code if failed
    uint32_t bytes_transferred; ///< Total bytes transferred
    uint32_t blocks_transferred;///< Number of blocks transferred
    uint32_t duration_ms;       ///< Transfer duration in milliseconds
    char error_text[64];        ///< Slave's error text (if provided)
    
    /** @brief Check if transfer succeeded */
    operator bool() const { return success; }

    /** @brief Create a failure result with the given error code */
    static FoEResult Failure(FoEError error) {
        FoEResult r{};
        r.success = false;
        r.error_code = error;
        r.bytes_transferred = 0;
        r.blocks_transferred = 0;
        r.duration_ms = 0;
        r.error_text[0] = '\0';
        return r;
    }
};

// ============================================================================
// Transfer Handle (for async operations)
// ============================================================================

/**
 * @brief Handle for tracking async FoE transfers
 */
struct FoETransferHandle {
    uint32_t id;                ///< Unique transfer ID
    std::atomic<bool> complete; ///< Set when transfer completes
    std::atomic<bool> cancel;   ///< Set to request cancellation
    FoEResult result;           ///< Result (valid when complete)
    FoEProgress progress;       ///< Current progress
    
    FoETransferHandle() : id(0), complete(false), cancel(false) {}
    
    /** @brief Check if transfer is still in progress */
    bool in_progress() const { return !complete.load(); }
    
    /** @brief Request cancellation */
    void request_cancel() { cancel.store(true); }
};

// ============================================================================
// Statistics
// ============================================================================

/**
 * @brief FoE transfer statistics
 */
struct FoEStats {
    uint32_t uploads_completed;     ///< Successful uploads
    uint32_t uploads_failed;        ///< Failed uploads
    uint32_t downloads_completed;   ///< Successful downloads
    uint32_t downloads_failed;      ///< Failed downloads
    uint64_t bytes_uploaded;        ///< Total bytes uploaded
    uint64_t bytes_downloaded;      ///< Total bytes downloaded
    uint32_t timeouts;              ///< Timeout count
    uint32_t retries;               ///< Retry count
};

// ============================================================================
// Wire Format Structures
// ============================================================================

/**
 * @brief FoE header structure
 */
struct __attribute__((packed)) FoEHeader {
    uint8_t opcode;         ///< FoE opcode
    uint8_t reserved;       ///< Reserved (0)
    uint32_t packet_no_le;  ///< Packet/block number (little-endian)
};
static_assert(sizeof(FoEHeader) == 6, "FoEHeader must be 6 bytes");

/**
 * @brief FoE error response structure
 */
struct __attribute__((packed)) FoEErrorResponse {
    uint8_t opcode;         ///< Always 5 (ERROR)
    uint8_t reserved;
    uint32_t error_code_le; ///< Error code
};
static_assert(sizeof(FoEErrorResponse) == 6, "FoEErrorResponse must be 6 bytes");

// ============================================================================
// Transport Abstraction
// ============================================================================

/**
 * @brief Abstract transport interface for FoE I/O
 *
 * Implementations provide the actual mailbox read/write operations,
 * file I/O, and timing.  A concrete implementation backed by Raw
 * EtherCAT frames would wrap mailbox send/receive functions.  A mock
 * implementation is used for unit tests.
 */
class IFoETransport {
public:
    virtual ~IFoETransport() = default;

    /**
     * @brief Send FoE data via mailbox to a slave
     *
     * @param slave_index  Zero-based slave index
     * @param mbx_wr_addr  Mailbox write address (Master->Slave)
     * @param mbx_wr_len   Mailbox write length
     * @param data         FoE payload (opcode + data)
     * @param data_len     Payload length
     * @param mbx_counter  Mailbox counter (incremented by implementation)
     * @return true on success
     */
    virtual bool mailboxWrite(uint16_t slave_index,
                              uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                              const uint8_t* data, size_t data_len,
                              uint8_t* mbx_counter) = 0;

    /**
     * @brief Read FoE response from slave mailbox
     *
     * @param slave_index  Zero-based slave index
     * @param mbx_rd_addr  Mailbox read address (Slave->Master)
     * @param mbx_rd_len   Mailbox read length
     * @param data         Output buffer for FoE payload
     * @param data_cap     Buffer capacity
     * @param data_len     Actual bytes read
     * @param timeout_ms   Timeout in milliseconds
     * @return true on success
     */
    virtual bool mailboxRead(uint16_t slave_index,
                             uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                             uint8_t* data, size_t data_cap, size_t* data_len,
                             uint32_t timeout_ms) = 0;

    /**
     * @brief Open a local file for reading or writing
     *
     * @param path       Local file path
     * @param for_write  true to open for writing, false for reading
     * @param file_size  If reading, set to file size; if writing, ignored
     * @return true on success
     */
    virtual bool fileOpen(const char* path, bool for_write, uint32_t* file_size) = 0;

    /**
     * @brief Read from the currently open file
     * @return Bytes actually read, or -1 on error
     */
    virtual int32_t fileRead(void* buffer, size_t size) = 0;

    /**
     * @brief Write to the currently open file
     * @return Bytes actually written, or -1 on error
     */
    virtual int32_t fileWrite(const void* buffer, size_t size) = 0;

    /**
     * @brief Close the currently open file
     */
    virtual void fileClose() = 0;

    /**
     * @brief Get current monotonic time in milliseconds
     */
    virtual uint32_t getTimeMs() = 0;

    /**
     * @brief Blocking delay
     */
    virtual void delayMs(uint32_t ms) = 0;
};

// ============================================================================
// FoEManager -- owns all state (no globals)
// ============================================================================

/**
 * @brief Instance-based FoE manager.
 *
 * Each FoEManager owns its own request queue, worker thread, transfer
 * state array, and statistics.  Multiple independent instances can
 * co-exist.  Network I/O is performed through the injected IFoETransport.
 */
class FoEManager {
public:
    /**
     * @brief Construct an FoEManager with the given transport.
     *
     * The transport reference must outlive the FoEManager.
     */
    explicit FoEManager(IFoETransport& transport);

    /**
     * @brief Destructor -- calls deinit() if still initialized.
     */
    ~FoEManager();

    // Non-copyable
    FoEManager(const FoEManager&) = delete;
    FoEManager& operator=(const FoEManager&) = delete;

    // ----- Lifecycle -----

    /**
     * @brief Initialize FoE subsystem and start background thread.
     * @return true on success
     */
    bool init();

    /**
     * @brief Shut down FoE subsystem, cancel pending transfers.
     */
    void deinit();

    /**
     * @brief Check if FoE is initialized.
     */
    bool isInitialized() const;

    // ----- Synchronous File Transfer API -----

    FoEResult uploadFile(const char* local_path, const FoETransferConfig& config);
    FoEResult downloadFile(const char* local_path, const FoETransferConfig& config);
    FoEResult uploadMemory(const void* data, size_t size, const FoETransferConfig& config);
    FoEResult downloadMemory(void* buffer, size_t buffer_size,
                              size_t* received_size, const FoETransferConfig& config);

    // ----- Asynchronous Transfer API -----

    bool uploadFileAsync(const char* local_path, const FoETransferConfig& config,
                         FoETransferHandle* handle);
    bool downloadFileAsync(const char* local_path, const FoETransferConfig& config,
                           FoETransferHandle* handle);
    bool waitComplete(FoETransferHandle* handle, uint32_t timeout_ms = 0);
    void cancel(FoETransferHandle* handle);

    // ----- Statistics -----

    FoEStats getStats() const;
    void resetStats();

    // ----- Queue management -----

    size_t pendingCount() const;
    size_t activeTransferCount() const;

private:
    // Internal transfer state
    struct TransferState {
        bool active = false;
        bool is_upload = false;
        uint16_t slave_index = 0;
        uint32_t password = 0;
        char filename[kMaxFilename] = {};
        uint32_t block_number = 0;
        uint32_t bytes_transferred = 0;
        uint32_t total_bytes = 0;
        uint32_t start_time_ms = 0;
        FoEProgressCallback progress_callback;
        FoETransferHandle* handle = nullptr;

        uint16_t mbx_write_addr = 0;
        uint16_t mbx_write_len = 0;
        uint16_t mbx_read_addr = 0;
        uint16_t mbx_read_len = 0;
        uint8_t mbx_cnt = 0;

        uint8_t buffer[kBufferSize] = {};
        size_t buffer_len = 0;

        const uint8_t* src_memory = nullptr;
        uint8_t* dst_memory = nullptr;
        size_t memory_size = 0;
        size_t memory_offset = 0;
    };

    // Internal request entry
    struct RequestEntry {
        enum class Type { UPLOAD_FILE, DOWNLOAD_FILE, UPLOAD_MEMORY, DOWNLOAD_MEMORY };
        Type type = Type::UPLOAD_FILE;
        char local_path[128] = {};
        FoETransferConfig config;
        FoETransferHandle* handle = nullptr;

        const void* src_data = nullptr;
        void* dst_data = nullptr;
        size_t data_size = 0;
        size_t* received_size = nullptr;
    };

    IFoETransport& transport_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread worker_thread_;
    mutable std::mutex mutex_;
    FoEStats stats_{};
    std::array<TransferState, kMaxTransfers> transfers_{};
    std::atomic<uint32_t> next_transfer_id_{1};

    static constexpr size_t kQueueCapacity = 10;
    std::array<RequestEntry, kQueueCapacity> request_queue_{};
    size_t queue_head_ = 0;
    size_t queue_tail_ = 0;
    size_t queue_count_ = 0;

    bool enqueueRequest(const RequestEntry& entry);
    bool dequeueRequest(RequestEntry& entry);
};

// ============================================================================
// Free-function API (backward compat, delegates to a FoEManager)
// ============================================================================

inline bool foe_init(FoEManager& mgr) { return mgr.init(); }
inline void foe_deinit(FoEManager& mgr) { mgr.deinit(); }
inline bool foe_is_initialized(FoEManager& mgr) { return mgr.isInitialized(); }

inline FoEResult foe_upload_file(FoEManager& mgr, const char* path, const FoETransferConfig& cfg) {
    return mgr.uploadFile(path, cfg);
}
inline FoEResult foe_download_file(FoEManager& mgr, const char* path, const FoETransferConfig& cfg) {
    return mgr.downloadFile(path, cfg);
}
inline FoEResult foe_upload_memory(FoEManager& mgr, const void* data, size_t sz, const FoETransferConfig& cfg) {
    return mgr.uploadMemory(data, sz, cfg);
}
inline FoEResult foe_download_memory(FoEManager& mgr, void* buf, size_t sz,
                                      size_t* received, const FoETransferConfig& cfg) {
    return mgr.downloadMemory(buf, sz, received, cfg);
}
inline FoEStats foe_get_stats(FoEManager& mgr) { return mgr.getStats(); }
inline void foe_reset_stats(FoEManager& mgr) { mgr.resetStats(); }

} // namespace FoE

// ============================================================================
// Legacy EtherCAT::FoEManager wrapper (for Master compatibility)
// ============================================================================

class Master; // forward

/**
 * @brief Null transport used by the legacy FoEManager wrapper.
 *
 * All operations return failure / safe defaults.  This allows the
 * legacy wrapper to be constructed without a real transport.
 */
class NullFoETransport : public FoE::IFoETransport {
public:
    bool mailboxWrite(uint16_t, uint16_t, uint16_t, const uint8_t*, size_t, uint8_t*) override { return false; }
    bool mailboxRead(uint16_t, uint16_t, uint16_t, uint8_t*, size_t, size_t*, uint32_t) override { return false; }
    bool fileOpen(const char*, bool, uint32_t*) override { return false; }
    int32_t fileRead(void*, size_t) override { return -1; }
    int32_t fileWrite(const void*, size_t) override { return -1; }
    void fileClose() override {}
    uint32_t getTimeMs() override { return 0; }
    void delayMs(uint32_t) override {}
};

/**
 * @brief Legacy FoEManager wrapper for Master.
 *
 * Constructed with an Master& reference for backward
 * compatibility.  Internally owns a NullFoETransport and a
 * FoE::FoEManager.  When a real transport is needed, the master
 * should switch to using FoE::FoEManager directly.
 */
class FoEManager {
public:
    explicit FoEManager(Master& master)
        : master_(master), null_transport_(), mgr_(null_transport_) {}
    ~FoEManager() = default;

    bool init()   { return mgr_.init(); }
    void deinit() { mgr_.deinit(); }
    bool isInitialized() const { return mgr_.isInitialized(); }

    FoE::FoEResult uploadFile(const char* path, const FoE::FoETransferConfig& cfg) {
        return mgr_.uploadFile(path, cfg);
    }
    FoE::FoEResult downloadFile(const char* path, const FoE::FoETransferConfig& cfg) {
        return mgr_.downloadFile(path, cfg);
    }
    FoE::FoEResult uploadMemory(const void* data, size_t sz, const FoE::FoETransferConfig& cfg) {
        return mgr_.uploadMemory(data, sz, cfg);
    }
    FoE::FoEResult downloadMemory(void* buf, size_t sz, const FoE::FoETransferConfig& cfg,
                                   size_t* bytes_read = nullptr) {
        return mgr_.downloadMemory(buf, sz, bytes_read, cfg);
    }

    bool waitComplete(FoE::FoETransferHandle* h, uint32_t timeout_ms = 0) {
        return mgr_.waitComplete(h, timeout_ms);
    }
    void cancel(FoE::FoETransferHandle* h) { mgr_.cancel(h); }
    FoE::FoEStats getStats() const { return mgr_.getStats(); }
    void resetStats() { mgr_.resetStats(); }

    Master& master() { return master_; }
    FoE::FoEManager& inner() { return mgr_; }

private:
    Master& master_;
    NullFoETransport null_transport_;
    FoE::FoEManager mgr_;
};

} // namespace EtherCAT

#endif // ECAT_FEATURE_FOE_ENABLED
