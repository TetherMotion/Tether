/**
 * @file CoEManager.cpp
 * @brief CoEManager implementation — multi-slave CoE mailbox transaction manager
 */

#include "tether/ethercat/CoEManager.hpp"
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
#include <vector>

namespace EtherCAT {
namespace CoE {

static const char* TAG = "coe_mgr";

// ============================================================================
// Error String Helpers
// ============================================================================

const char* sdoAbortCodeStr(SDO::SDOAbortCode code) {
    switch (code) {
        case SDO::SDOAbortCode::Success:              return "Success";
        case SDO::SDOAbortCode::ToggleBitNotChanged:  return "Toggle bit not alternated";
        case SDO::SDOAbortCode::Timeout:              return "SDO timeout";
        case SDO::SDOAbortCode::InvalidCommand:       return "Invalid command";
        case SDO::SDOAbortCode::OutOfMemory:          return "Out of memory";
        case SDO::SDOAbortCode::UnsupportedAccess:    return "Unsupported access";
        case SDO::SDOAbortCode::ReadOnlyObject:       return "Write to read-only object";
        case SDO::SDOAbortCode::WriteOnlyObject:      return "Read from write-only object";
        case SDO::SDOAbortCode::ObjectNotFound:       return "Object not found";
        case SDO::SDOAbortCode::SubindexNotFound:     return "Subindex not found";
        case SDO::SDOAbortCode::InvalidValue:         return "Invalid value";
        case SDO::SDOAbortCode::GeneralError:         return "General error";
        case SDO::SDOAbortCode::TransferAborted:      return "Transfer aborted";
        case SDO::SDOAbortCode::DeviceStateError:     return "Wrong device state";
        default:                                 return "Unknown error";
    }
}

const char* coeErrorStr(CoEError error) {
    switch (error) {
        case CoEError::Ok:              return "Ok";
        case CoEError::Timeout:         return "Timeout";
        case CoEError::Aborted:         return "Aborted";
        case CoEError::TransportError:  return "Transport error";
        case CoEError::QueueFull:       return "Queue full";
        case CoEError::NotConfigured:   return "Mailbox not configured";
        case CoEError::ShuttingDown:    return "Shutting down";
        case CoEError::SlaveNotFound:   return "Slave not found";
        case CoEError::InternalError:   return "Internal error";
        default:                        return "Unknown error";
    }
}

// ============================================================================
// CoEError → SDOAbortCode mapping
// ============================================================================

static SDO::SDOAbortCode coeErrorToAbortCode(CoEError err) {
    switch (err) {
        case CoEError::Ok:             return SDO::SDOAbortCode::Success;
        case CoEError::Timeout:        return SDO::SDOAbortCode::Timeout;
        case CoEError::NotConfigured:  return SDO::SDOAbortCode::DeviceStateError;
        case CoEError::TransportError: return SDO::SDOAbortCode::GeneralError;
        case CoEError::QueueFull:      return SDO::SDOAbortCode::OutOfMemory;
        case CoEError::Aborted:        return SDO::SDOAbortCode::TransferAborted;
        case CoEError::ShuttingDown:   return SDO::SDOAbortCode::DeviceStateError;
        case CoEError::SlaveNotFound:  return SDO::SDOAbortCode::ObjectNotFound;
        case CoEError::InternalError:  return SDO::SDOAbortCode::InternalError;
    }
    return SDO::SDOAbortCode::GeneralError;
}

// ============================================================================
// Pending future implementations for legacy queueRequest API
// ============================================================================

namespace {

class PendingReadFuture : public IPendingFuture {
public:
    PendingReadFuture(std::future<CoEResult<std::vector<uint8_t>>>&& fut,
                      uint32_t request_id, uint16_t slave_index,
                      uint16_t index, uint8_t subindex)
        : fut_(std::move(fut))
        , request_id_(request_id)
        , slave_index_(slave_index)
        , index_(index)
        , subindex_(subindex) {}

    bool isReady() const override {
        return fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    SDO::SDOResponse consume() override {
        SDO::SDOResponse resp{};
        resp.request_id = request_id_;
        resp.slave_index = slave_index_;
        resp.index = index_;
        resp.subindex = subindex_;
        resp.operation = SDO::SDOOperation::Upload;

        auto result = fut_.get();
        if (result.has_value()) {
            resp.status = SDO::SDOStatus::Complete;
            resp.abort_code = SDO::SDOAbortCode::Success;
            auto& vec = result.value();
            resp.data_size = std::min(vec.size(), sizeof(resp.data));
            std::memcpy(resp.data, vec.data(), resp.data_size);
        } else {
            resp.status = SDO::SDOStatus::Failed;
            resp.abort_code = coeErrorToAbortCode(result.error());
        }
        return resp;
    }

private:
    std::future<CoEResult<std::vector<uint8_t>>> fut_;
    uint32_t request_id_;
    uint16_t slave_index_;
    uint16_t index_;
    uint8_t subindex_;
};

class PendingWriteFuture : public IPendingFuture {
public:
    PendingWriteFuture(std::future<CoEResult<void>>&& fut,
                       uint32_t request_id, uint16_t slave_index,
                       uint16_t index, uint8_t subindex,
                       const uint8_t* echo_data, size_t echo_size)
        : fut_(std::move(fut))
        , request_id_(request_id)
        , slave_index_(slave_index)
        , index_(index)
        , subindex_(subindex) {
        echo_size_ = std::min(echo_size, sizeof(echo_data_));
        std::memcpy(echo_data_, echo_data, echo_size_);
    }

    bool isReady() const override {
        return fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    SDO::SDOResponse consume() override {
        SDO::SDOResponse resp{};
        resp.request_id = request_id_;
        resp.slave_index = slave_index_;
        resp.index = index_;
        resp.subindex = subindex_;
        resp.operation = SDO::SDOOperation::Download;
        resp.data_size = echo_size_;
        std::memcpy(resp.data, echo_data_, echo_size_);

        auto result = fut_.get();
        if (result.has_value()) {
            resp.status = SDO::SDOStatus::Complete;
            resp.abort_code = SDO::SDOAbortCode::Success;
        } else {
            resp.status = SDO::SDOStatus::Failed;
            resp.abort_code = coeErrorToAbortCode(result.error());
        }
        return resp;
    }

private:
    std::future<CoEResult<void>> fut_;
    uint32_t request_id_;
    uint16_t slave_index_;
    uint16_t index_;
    uint8_t subindex_;
    uint8_t echo_data_[SDO::kMaxSDODataSize];
    size_t echo_size_ = 0;
};

} // anonymous namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

CoEManager::CoEManager(uint16_t slave_index, SDO::ISDOTransport& transport)
    : slave_index_(slave_index)
    , transport_(transport)
{
}

CoEManager::~CoEManager() {
    deinit();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool CoEManager::init() {
    std::lock_guard<std::mutex> wlock(state_.worker_mutex);

    if (state_.worker_running.load()) {
        initialized_.store(true);
        return true;
    }

    if (state_.worker_thread && state_.worker_thread->joinable()) {
        state_.worker_thread->join();
    }
    state_.worker_thread.reset();

    state_.shutdown_requested.store(false);

    try {
        state_.worker_thread = std::make_unique<std::thread>(&CoEManager::workerLoop, this);
        state_.worker_running.store(true);
    } catch (...) {
        TETHER_LOGE(TAG, "Slave %u: Failed to create CoE worker thread", slave_index_);
        return false;
    }

    initialized_.store(true);
    TETHER_LOGI(TAG, "CoEManager initialized for slave %u", slave_index_);
    return true;
}

void CoEManager::deinit() {
    initialized_.store(false);

    state_.shutdown_requested.store(true);
    state_.work_cv.notify_all();

    {
        std::lock_guard<std::mutex> wlock(state_.worker_mutex);
        if (state_.worker_thread && state_.worker_thread->joinable()) {
            state_.worker_thread->join();
        }
        state_.worker_thread.reset();
        state_.worker_running.store(false);
    }

    // Worker is joined — safe to clear all state without concurrent access
    {
        std::lock_guard<std::mutex> qlock(state_.queue_mutex);
        for (auto& txn : state_.read_queue) {
            txn->fail(CoEError::ShuttingDown);
        }
        state_.read_queue.clear();
        for (auto& entry : state_.write_queue) {
            entry.txn.promise.set_value(std::unexpected(CoEError::ShuttingDown));
        }
        state_.write_queue.clear();
    }

    {
        std::lock_guard<std::mutex> flock(pending_futures_mutex_);
        pending_futures_.clear();
    }

    {
        std::lock_guard<std::mutex> rlock(responses_mutex_);
        completed_responses_.clear();
    }

    next_request_id_.store(1);
    state_.shutdown_requested.store(false);

    TETHER_LOGI(TAG, "CoEManager deinitialized for slave %u", slave_index_);
}

bool CoEManager::isInitialized() const {
    return initialized_.load();
}

// ============================================================================
// Mailbox Configuration
// ============================================================================

void CoEManager::configureMailbox(uint16_t mbx_write_addr, uint16_t mbx_write_len,
                                   uint16_t mbx_read_addr, uint16_t mbx_read_len) {
    mbx_.write_addr  = mbx_write_addr;
    mbx_.write_len   = mbx_write_len;
    mbx_.read_addr   = mbx_read_addr;
    mbx_.read_len    = mbx_read_len;
    mbx_.mbx_counter = 1;
    mbx_.configured  = true;

    TETHER_LOGI(TAG, "Slave %u mailbox: Receive(SM0/MbxIn)=0x%04x/%u, Send(SM1/MbxOut)=0x%04x/%u",
             slave_index_, mbx_write_addr, mbx_write_len, mbx_read_addr, mbx_read_len);
}

bool CoEManager::getMailbox(uint16_t* mbx_write_addr, uint16_t* mbx_write_len,
                             uint16_t* mbx_read_addr, uint16_t* mbx_read_len) const {
    const PDO::SlaveConfig* slave_configs = pdo_manager_ ? pdo_manager_->slaveConfigs() : nullptr;
    if (slave_configs) {
        const auto& sm0 = slave_configs[slave_index_].sm[0];
        const auto& sm1 = slave_configs[slave_index_].sm[1];
        if (sm0.type == PDO::SyncManagerType::MailboxWrite ||
            sm1.type == PDO::SyncManagerType::MailboxRead) {
            if (mbx_write_addr) *mbx_write_addr = sm0.phys_start_addr;
            if (mbx_write_len)  *mbx_write_len  = sm0.length;
            if (mbx_read_addr)  *mbx_read_addr  = sm1.phys_start_addr;
            if (mbx_read_len)   *mbx_read_len   = sm1.length;
            return true;
        }
    }

    if (!mbx_.configured) return false;
    if (mbx_write_addr) *mbx_write_addr = mbx_.write_addr;
    if (mbx_write_len)  *mbx_write_len  = mbx_.write_len;
    if (mbx_read_addr)  *mbx_read_addr  = mbx_.read_addr;
    if (mbx_read_len)   *mbx_read_len   = mbx_.read_len;
    return true;
}

// ============================================================================
// Mailbox Resolution
// ============================================================================

bool CoEManager::resolveMailbox(uint16_t& wr_addr, uint16_t& wr_len,
                                 uint16_t& rd_addr, uint16_t& rd_len) {
    return getMailbox(&wr_addr, &wr_len, &rd_addr, &rd_len);
}

// ============================================================================
// Retry Wrappers
// ============================================================================

bool CoEManager::sdoUploadWithRetry(uint16_t index, uint8_t subindex,
                                    uint8_t* out, size_t out_cap, size_t* out_len,
                                    const CoETransactionOptions& options) {
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    if (!resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
        TETHER_LOGE(TAG, "Slave %u: sdoUploadWithRetry: mailbox not configured", slave_index_);
        return false;
    }

    last_sdo_abort_code_.store(0, std::memory_order_relaxed);

    const uint8_t max_attempts = options.max_retries + 1;
    for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
        bool ok = transport_.sdoUpload(
            slave_index_, mbxCounterPtr(),
            wr_addr, wr_len, rd_addr, rd_len,
            index, subindex,
            out, out_cap, out_len,
            diag_enabled_.load(),
            options.poll_interval_ms, options.timeout_ms);

        if (behaviour_options_.request_al_status_after_coe_requests) {
            logALStatusAfterRequest();
        }

        if (ok) return true;

        // Definitive slave SDO abort: the slave explicitly rejected the
        // request (e.g. wrong payload size, read-only object, subindex does
        // not exist). Retrying the identical request cannot succeed, so do
        // NOT retry — record the abort code and escalate immediately.
        const uint32_t abort_code = transport_.lastAbortCode();
        if (abort_code != 0) {
            last_sdo_abort_code_.store(abort_code, std::memory_order_relaxed);
            Raw::SDOErrorDecoder decoder;
            TETHER_LOGE(TAG, "Slave %u: SDO upload 0x%04X:%u aborted by slave — code 0x%08X (%s). Not retrying.",
                        slave_index_, index, subindex, abort_code,
                        decoder.sdoAbortCodeStr(abort_code));
            return false;
        }

        if (attempt + 1 < max_attempts) {
            TETHER_LOGW(TAG, "Slave %u: SDO upload 0x%04X:%u failed on attempt %u/%u, retrying",
                        slave_index_, index, subindex, attempt + 1, max_attempts);
        }
    }

    TETHER_LOGE(TAG, "Slave %u: SDO upload 0x%04X:%u failed after %u attempts",
                slave_index_, index, subindex, max_attempts);
    return false;
}

bool CoEManager::sdoDownloadWithRetry(uint16_t index, uint8_t subindex,
                                      const uint8_t* data, size_t data_len,
                                      const CoETransactionOptions& options) {
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    if (!resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
        TETHER_LOGE(TAG, "Slave %u: sdoDownloadWithRetry: mailbox not configured", slave_index_);
        return false;
    }

    const uint8_t effective_sub = options.complete_access
        ? static_cast<uint8_t>(subindex | 0x80u)
        : subindex;

    if (options.complete_access) {
        TETHER_LOGI(TAG, "Slave %u: SDO download 0x%04X:%u (Complete Access) %zu bytes",
                    slave_index_, index, subindex, data_len);
    }

    last_sdo_abort_code_.store(0, std::memory_order_relaxed);

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

        // Definitive slave SDO abort: the slave explicitly rejected the
        // request (e.g. wrong payload size, read-only object, subindex does
        // not exist). Retrying the identical request cannot succeed, so do
        // NOT retry — record the abort code and escalate immediately.
        const uint32_t abort_code = transport_.lastAbortCode();
        if (abort_code != 0) {
            last_sdo_abort_code_.store(abort_code, std::memory_order_relaxed);
            Raw::SDOErrorDecoder decoder;
            TETHER_LOGE(TAG, "Slave %u: SDO download 0x%04X:%u aborted by slave — code 0x%08X (%s). Not retrying.",
                        slave_index_, index, subindex, abort_code,
                        decoder.sdoAbortCodeStr(abort_code));
            return false;
        }

        if (attempt + 1 < max_attempts) {
            TETHER_LOGW(TAG, "Slave %u: SDO download 0x%04X:%u failed on attempt %u/%u, retrying",
                        slave_index_, index, subindex, attempt + 1, max_attempts);
        }
    }

    TETHER_LOGE(TAG, "Slave %u: SDO download 0x%04X:%u failed after %u attempts",
                slave_index_, index, subindex, max_attempts);
    return false;
}

uint8_t* CoEManager::mbxCounterPtr() {
    return &mbx_.mbx_counter;
}

void CoEManager::logALStatusAfterRequest() {
    uint16_t al_status = 0;
    uint16_t al_code = 0;

    if (!transport_.readSlaveRegister(slave_index_, 0x0130, &al_status, sizeof(al_status), 200)) {
        TETHER_LOGW(TAG, "Slave %u: post-SDO AL_STATUS read FAILED", slave_index_);
        return;
    }
    if (!transport_.readSlaveRegister(slave_index_, 0x0134, &al_code, sizeof(al_code), 200)) {
        al_code = 0;
    }

    if (al_status_has_error(al_status) || al_code != 0) {
        TETHER_LOGE(TAG, "Slave %u: post-SDO AL_STATUS=0x%04X (%s)%s | AL status code: %s (0x%04X)",
                    slave_index_,
                    al_status,
                    al_status_get_state_name(al_status),
                    al_status_has_error(al_status) ? " ERROR" : "",
                    getALStatusCodeName(al_code),
                    al_code);
    }
}

// ============================================================================
// Async Write API
// ============================================================================

// ============================================================================
// Legacy Polling Queue API
// ============================================================================

uint32_t CoEManager::queueRequest(SDO::SDORequest& request) {
    uint32_t id = nextRequestId();
    request.request_id = id;

    const uint32_t timeout_ms = (request.timeout_ms > 0) ? request.timeout_ms : kDefaultTimeoutMs;
    CoETransactionOptions opts;
    opts.timeout_ms = timeout_ms;

    if (request.operation == SDO::SDOOperation::Upload) {
        auto future = read<std::vector<uint8_t>>(request.index, request.subindex, opts);

        // Check if future is immediately ready (queue full or not initialized)
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = future.get();
            if (!result.has_value() && result.error() == CoEError::QueueFull) {
                return 0;
            }

            // Completed immediately (e.g., NotConfigured error) — store response
            SDO::SDOResponse resp{};
            resp.request_id = id;
            resp.slave_index = slave_index_;
            resp.index = request.index;
            resp.subindex = request.subindex;
            resp.operation = request.operation;

            if (result.has_value()) {
                resp.status = SDO::SDOStatus::Complete;
                resp.abort_code = SDO::SDOAbortCode::Success;
                auto& vec = result.value();
                resp.data_size = std::min(vec.size(), sizeof(resp.data));
                std::memcpy(resp.data, vec.data(), resp.data_size);
            } else {
                resp.status = SDO::SDOStatus::Failed;
                resp.abort_code = coeErrorToAbortCode(result.error());
            }
            storeResponse(id, resp);
            return id;
        }

        // Not ready — store future for later retrieval via getResponse
        std::lock_guard<std::mutex> lock(pending_futures_mutex_);
        pending_futures_[id] = std::make_unique<PendingReadFuture>(
            std::move(future), id, slave_index_, request.index, request.subindex);
    } else {
        auto future = write(request.index, request.subindex,
            request.data, request.data_size, opts);

        // Check if future is immediately ready (queue full or not initialized)
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = future.get();
            if (!result.has_value() && result.error() == CoEError::QueueFull) {
                return 0;
            }

            // Completed immediately — store response
            SDO::SDOResponse resp{};
            resp.request_id = id;
            resp.slave_index = slave_index_;
            resp.index = request.index;
            resp.subindex = request.subindex;
            resp.operation = request.operation;
            resp.data_size = std::min(request.data_size, sizeof(resp.data));
            std::memcpy(resp.data, request.data, resp.data_size);

            if (result.has_value()) {
                resp.status = SDO::SDOStatus::Complete;
                resp.abort_code = SDO::SDOAbortCode::Success;
            } else {
                resp.status = SDO::SDOStatus::Failed;
                resp.abort_code = coeErrorToAbortCode(result.error());
            }
            storeResponse(id, resp);
            return id;
        }

        // Not ready — store future for later retrieval via getResponse
        std::lock_guard<std::mutex> lock(pending_futures_mutex_);
        pending_futures_[id] = std::make_unique<PendingWriteFuture>(
            std::move(future), id, slave_index_, request.index, request.subindex,
            request.data, request.data_size);
    }

    return id;
}

bool CoEManager::getResponse(uint32_t request_id, SDO::SDOResponse& response) {
    // Check pending futures first
    {
        std::lock_guard<std::mutex> lock(pending_futures_mutex_);
        auto it = pending_futures_.find(request_id);
        if (it != pending_futures_.end()) {
            if (!it->second->isReady()) {
                return false;
            }
            response = it->second->consume();
            pending_futures_.erase(it);
            return true;
        }
    }
    // Fall back to completed_responses_ (for immediately-completed requests)
    return popResponse(request_id, response);
}

bool CoEManager::waitForResponse(uint32_t request_id, SDO::SDOResponse& response,
                                  uint32_t timeout_ms) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (getResponse(request_id, response)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return getResponse(request_id, response);
}

size_t CoEManager::pendingCount() const {
    std::lock_guard<std::mutex> lock(pending_futures_mutex_);
    return pending_futures_.size();
}

// ============================================================================
// Async Write API
// ============================================================================

std::future<CoEResult<void>> CoEManager::write(uint16_t index, uint8_t subindex,
                                                const void* data, size_t size,
                                                CoETransactionOptions options) {
    if (debug_flags_.coeWrites) {
        TETHER_LOGI(TAG, "Slave %u: CoE write START index=0x%04X:%u size=%zu",
                    slave_index_, index, subindex, size);
    }

    CoEWriteTransaction txn;
    txn.index = index;
    txn.subindex = subindex;
    txn.options = options;
    txn.enqueue_time = std::chrono::steady_clock::now();
    txn.data.assign(static_cast<const uint8_t*>(data),
                    static_cast<const uint8_t*>(data) + size);

    auto future = txn.promise.get_future();

    WriteQueueEntry entry{std::move(txn)};

    if (!initialized_.load() || state_.shutdown_requested.load()) {
        CoEWriteTransaction fail_txn;
        fail_txn.promise.set_value(std::unexpected(CoEError::NotConfigured));
        return fail_txn.promise.get_future();
    }

    {
        std::lock_guard<std::mutex> lock(state_.queue_mutex);
        if (state_.shutdown_requested.load()) {
            CoEWriteTransaction fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEError::ShuttingDown));
            return fail_txn.promise.get_future();
        }
        if (state_.write_queue.size() >= kMaxQueueDepth) {
            if (debug_flags_.coeWrites) {
                TETHER_LOGI(TAG, "Slave %u: CoE write QUEUE FULL index=0x%04X:%u",
                            slave_index_, index, subindex);
            }
            CoEWriteTransaction fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEError::QueueFull));
            return fail_txn.promise.get_future();
        }
        state_.write_queue.push_back(std::move(entry));
    }
    state_.work_cv.notify_one();

    if (debug_flags_.coeWrites) {
        TETHER_LOGI(TAG, "Slave %u: CoE write ENQUEUED index=0x%04X:%u",
                    slave_index_, index, subindex);
    }
    return future;
}

// ============================================================================
// Sync Convenience
// ============================================================================

CoEResult<void> CoEManager::writeSync(uint16_t index, uint8_t subindex,
                                       const void* data, size_t size,
                                       CoETransactionOptions options) {
    if (debug_flags_.coeWrites) {
        TETHER_LOGI(TAG, "Slave %u: CoE writeSync index=0x%04X:%u size=%zu",
                    slave_index_, index, subindex, size);
    }
    return write(index, subindex, data, size, options).get();
}

bool CoEManager::readSync(uint16_t index, uint8_t subindex,
                           void* data, size_t max_size, uint32_t timeout_ms,
                           size_t* actual_size) {
    if (debug_flags_.coeReads) {
        TETHER_LOGI(TAG, "Slave %u: CoE readSync index=0x%04X:%u max_size=%zu",
                    slave_index_, index, subindex, max_size);
    }
    CoETransactionOptions opts;
    opts.timeout_ms = timeout_ms;

    auto result = readSync<std::vector<uint8_t>>(index, subindex, opts);
    if (!result.has_value()) return false;

    auto& vec = result.value();
    size_t copy_len = std::min(vec.size(), max_size);
    std::memcpy(data, vec.data(), copy_len);
    if (actual_size) *actual_size = copy_len;
    return true;
}

// ============================================================================
// Typed Sync Helpers
// ============================================================================

CoEResult<uint8_t> CoEManager::readU8(uint16_t idx, uint8_t sub,
                                       CoETransactionOptions opts) {
    return readSync<uint8_t>(idx, sub, opts);
}

CoEResult<uint16_t> CoEManager::readU16(uint16_t idx, uint8_t sub,
                                         CoETransactionOptions opts) {
    return readSync<uint16_t>(idx, sub, opts);
}

CoEResult<uint32_t> CoEManager::readU32(uint16_t idx, uint8_t sub,
                                         CoETransactionOptions opts) {
    return readSync<uint32_t>(idx, sub, opts);
}

CoEResult<int32_t> CoEManager::readI32(uint16_t idx, uint8_t sub,
                                        CoETransactionOptions opts) {
    return readSync<int32_t>(idx, sub, opts);
}

CoEResult<void> CoEManager::writeU8(uint16_t idx, uint8_t sub,
                                     uint8_t val, CoETransactionOptions opts) {
    return writeSync(idx, sub, &val, sizeof(val), opts);
}

CoEResult<void> CoEManager::writeU16(uint16_t idx, uint8_t sub,
                                      uint16_t val, CoETransactionOptions opts) {
    return writeSync(idx, sub, &val, sizeof(val), opts);
}

CoEResult<void> CoEManager::writeU32(uint16_t idx, uint8_t sub,
                                      uint32_t val, CoETransactionOptions opts) {
    return writeSync(idx, sub, &val, sizeof(val), opts);
}

CoEResult<void> CoEManager::writeI32(uint16_t idx, uint8_t sub,
                                      int32_t val, CoETransactionOptions opts) {
    return writeSync(idx, sub, &val, sizeof(val), opts);
}

// ============================================================================
// Queue Status
// ============================================================================

size_t CoEManager::pendingReadCount() const {
    std::lock_guard<std::mutex> lock(state_.queue_mutex);
    return state_.read_queue.size();
}

size_t CoEManager::pendingWriteCount() const {
    std::lock_guard<std::mutex> lock(state_.queue_mutex);
    return state_.write_queue.size();
}

size_t CoEManager::totalPendingCount() const {
    return pendingReadCount() + pendingWriteCount();
}

// ============================================================================
// Worker Thread Management
// ============================================================================

void CoEManager::workerLoop() {
    TETHER_LOGI(TAG, "Slave %u: CoE worker thread started", slave_index_);

    while (!state_.shutdown_requested.load()) {
        {
            std::unique_lock<std::mutex> lock(state_.queue_mutex);
            state_.work_cv.wait(lock, [this] {
                return !state_.read_queue.empty()
                       || !state_.write_queue.empty()
                       || state_.shutdown_requested.load();
            });
        }

        if (state_.shutdown_requested.load()) break;

        {
            std::unique_lock<std::mutex> lock(state_.queue_mutex);
            if (!state_.read_queue.empty()) {
                auto txn = std::move(state_.read_queue.front());
                state_.read_queue.pop_front();
                lock.unlock();

                if (request_in_flight_.exchange(true)) {
                    TETHER_LOGW(TAG, "Slave %u: CoE read started while previous request in flight — stale response possible",
                                slave_index_);
                }
                try {
                    txn->execute(*this);
                } catch (const std::exception& e) {
                    TETHER_LOGE(TAG, "Slave %u: CoE read threw: %s", slave_index_, e.what());
                } catch (...) {
                    TETHER_LOGE(TAG, "Slave %u: CoE read threw unknown exception", slave_index_);
                }
                request_in_flight_.store(false);
                continue;
            }
        }

        {
            std::unique_lock<std::mutex> lock(state_.queue_mutex);
            if (!state_.write_queue.empty()) {
                auto entry = std::move(state_.write_queue.front());
                state_.write_queue.pop_front();
                lock.unlock();

                auto& txn = entry.txn;
                if (request_in_flight_.exchange(true)) {
                    TETHER_LOGW(TAG, "Slave %u: CoE write started while previous request in flight — stale response possible",
                                slave_index_);
                }
                bool ok = false;
                try {
                    ok = sdoDownloadWithRetry(
                        txn.index, txn.subindex,
                        txn.data.data(), txn.data.size(),
                        txn.options);
                } catch (const std::exception& e) {
                    TETHER_LOGE(TAG, "Slave %u: CoE write threw: %s", slave_index_, e.what());
                } catch (...) {
                    TETHER_LOGE(TAG, "Slave %u: CoE write threw unknown exception", slave_index_);
                }
                request_in_flight_.store(false);

                if (ok) {
                    txn.promise.set_value({});
                } else if (last_sdo_abort_code_.load(std::memory_order_relaxed) != 0) {
                    // Slave explicitly aborted the SDO (definitive rejection,
                    // e.g. 0x06070010 length mismatch). Surface as Aborted so
                    // callers can distinguish it from a transport/timeout
                    // failure and read lastSdoAbortCode() for the code.
                    txn.promise.set_value(std::unexpected(CoEError::Aborted));
                } else {
                    txn.promise.set_value(std::unexpected(CoEError::TransportError));
                }
                continue;
            }
        }
    }

    state_.worker_running.store(false);
    TETHER_LOGI(TAG, "Slave %u: CoE worker thread stopped", slave_index_);
}

// ============================================================================
// Response storage helpers
// ============================================================================

void CoEManager::storeResponse(uint32_t request_id, const SDO::SDOResponse& resp) {
    std::lock_guard<std::mutex> lock(responses_mutex_);
    completed_responses_[request_id] = resp;
}

bool CoEManager::popResponse(uint32_t request_id, SDO::SDOResponse& resp) {
    std::lock_guard<std::mutex> lock(responses_mutex_);
    auto it = completed_responses_.find(request_id);
    if (it == completed_responses_.end()) return false;
    resp = it->second;
    completed_responses_.erase(it);
    return true;
}

uint32_t CoEManager::nextRequestId() {
    return next_request_id_.fetch_add(1);
}

// ============================================================================
// CoEReadTransactionImpl<T>::execute
// ============================================================================

template<typename T>
void CoEReadTransactionImpl<T>::execute(CoEManager& mgr) {
    std::vector<uint8_t> buf(1500);
    size_t out_len = 0;

    bool ok = mgr.sdoUploadWithRetry(
        txn_.index, txn_.subindex,
        buf.data(), buf.size(), &out_len,
        txn_.options);

    if (!ok) {
        if (mgr.lastSdoAbortCode() != 0) {
            txn_.promise.set_value(std::unexpected(CoEError::Aborted));
        } else {
            txn_.promise.set_value(std::unexpected(CoEError::TransportError));
        }
        return;
    }

    if (out_len < sizeof(T)) {
        txn_.promise.set_value(std::unexpected(CoEError::InternalError));
        return;
    }

    T value;
    std::memcpy(&value, buf.data(), sizeof(T));
    txn_.promise.set_value(value);
}

// Specialization for std::vector<uint8_t> — used by the raw-buffer readSync overload
template<>
void CoEReadTransactionImpl<std::vector<uint8_t>>::execute(CoEManager& mgr) {
    std::vector<uint8_t> buf(1500);
    size_t out_len = 0;

    bool ok = mgr.sdoUploadWithRetry(
        txn_.index, txn_.subindex,
        buf.data(), buf.size(), &out_len,
        txn_.options);

    if (!ok) {
        if (mgr.lastSdoAbortCode() != 0) {
            txn_.promise.set_value(std::unexpected(CoEError::Aborted));
        } else {
            txn_.promise.set_value(std::unexpected(CoEError::TransportError));
        }
        return;
    }

    std::vector<uint8_t> result(buf.data(), buf.data() + out_len);
    txn_.promise.set_value(std::move(result));
}

// Explicit template instantiations
template class CoEReadTransactionImpl<uint8_t>;
template class CoEReadTransactionImpl<uint16_t>;
template class CoEReadTransactionImpl<uint32_t>;
template class CoEReadTransactionImpl<int32_t>;
template class CoEReadTransactionImpl<std::vector<uint8_t>>;

} // namespace CoE
} // namespace EtherCAT
