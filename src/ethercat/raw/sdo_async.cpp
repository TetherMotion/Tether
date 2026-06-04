/**
 * @file sdo_async.cpp
 * @brief SDOManager implementation (instance-based, no globals) + backward-compat free functions
 */

#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/platform/Platform.hpp"

#ifdef TETHER_COMPILE_MASTER
#include "EtherCATRaw.hpp"
#include "raw/internal.hpp"
#endif

#include <cstring>
#include <algorithm>
#include <vector>

namespace EtherCAT {
namespace SDO {

static const char* TAG = "ec_sdo_async";

// ============================================================================
// Abort Code Strings
// ============================================================================

const char* sdo_abort_code_str(SDOAbortCode code)
{
    switch (code) {
        case SDOAbortCode::Success:              return "Success";
        case SDOAbortCode::ToggleBitNotChanged:  return "Toggle bit not alternated";
        case SDOAbortCode::Timeout:              return "SDO timeout";
        case SDOAbortCode::InvalidCommand:       return "Invalid command";
        case SDOAbortCode::OutOfMemory:          return "Out of memory";
        case SDOAbortCode::UnsupportedAccess:    return "Unsupported access";
        case SDOAbortCode::ReadOnlyObject:       return "Write to read-only object";
        case SDOAbortCode::WriteOnlyObject:      return "Read from write-only object";
        case SDOAbortCode::ObjectNotFound:       return "Object not found";
        case SDOAbortCode::SubindexNotFound:     return "Subindex not found";
        case SDOAbortCode::InvalidValue:         return "Invalid value";
        case SDOAbortCode::GeneralError:         return "General error";
        case SDOAbortCode::TransferAborted:      return "Transfer aborted";
        case SDOAbortCode::DeviceStateError:     return "Wrong device state";
        default:                                 return "Unknown error";
    }
}

// ============================================================================
// SDOManager — Construction / Destruction
// ============================================================================

SDOManager::SDOManager(ISDOTransport& transport)
    : transport_(transport)
{
}

SDOManager::~SDOManager()
{
    deinit();
}

// ============================================================================
// SDOManager — Lifecycle
// ============================================================================

bool SDOManager::init()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Clear any existing state for a clean start
    queue_.clear();
    {
        std::lock_guard<std::mutex> lock2(responses_mutex_);
        completed_responses_.clear();
    }
    next_request_id_.store(1, std::memory_order_relaxed);

    if (worker_thread_.joinable()) {
        TETHER_LOGW(TAG, "SDOManager already initialized — state reset");
        initialized_ = true;
        return true;
    }

    shutdown_requested_ = false;
    initialized_ = true;

    try {
        worker_thread_ = std::thread(&SDOManager::workerFunction, this);
    } catch (...) {
        TETHER_LOGE(TAG, "Failed to create SDO worker thread");
        initialized_ = false;
        return false;
    }

    TETHER_LOGI(TAG, "SDOManager initialized");
    return true;
}

void SDOManager::deinit()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!worker_thread_.joinable()) {
            initialized_ = false;
            return;
        }
        shutdown_requested_ = true;
        queue_cv_.notify_all();
    }

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(responses_mutex_);
        completed_responses_.clear();
    }

    initialized_ = false;
    TETHER_LOGI(TAG, "SDOManager deinitialized");
}

bool SDOManager::isInitialized() const
{
    return initialized_;
}

// ============================================================================
// SDOManager — Slave Mailbox Configuration
// ============================================================================

void SDOManager::configureSlaveMailbox(uint16_t slave_index,
                                       uint16_t mbx_write_addr, uint16_t mbx_write_len,
                                       uint16_t mbx_read_addr, uint16_t mbx_read_len)
{
    if (slave_index >= kMaxSlaves) {
        TETHER_LOGE(TAG, "Slave index %u exceeds max %zu", slave_index, kMaxSlaves);
        return;
    }

    slave_mbx_[slave_index].write_addr  = mbx_write_addr;
    slave_mbx_[slave_index].write_len   = mbx_write_len;
    slave_mbx_[slave_index].read_addr   = mbx_read_addr;
    slave_mbx_[slave_index].read_len    = mbx_read_len;
    slave_mbx_[slave_index].mbx_counter = 1;
    slave_mbx_[slave_index].configured  = true;

    TETHER_LOGI(TAG, "Slave %u mailbox: Send(SM0/MbxOut)=0x%04x/%u, Receive(SM1/MbxIn)=0x%04x/%u",
             slave_index, mbx_read_addr, mbx_read_len, mbx_write_addr, mbx_write_len);
}

bool SDOManager::getSlaveMailbox(uint16_t slave_index,
                                  uint16_t* mbx_write_addr, uint16_t* mbx_write_len,
                                  uint16_t* mbx_read_addr, uint16_t* mbx_read_len) const
{
    if (slave_index >= kMaxSlaves) return false;

    // Prefer the PDO manager's SyncManager configuration when available so the
    // SDO subsystem always reflects the node's configured SyncManagers.
    const PDO::SlaveConfig* slave_configs = pdo_manager_ ? pdo_manager_->slaveConfigs() : nullptr;
    if (slave_configs) {
        // Standard EtherCAT convention: SM0 = Send/MbxOut (S→M), SM1 = Receive/MbxIn (M→S)
        const auto& sm0 = slave_configs[slave_index].sm[0]; // SM0 = MailboxRead (S→M)
        const auto& sm1 = slave_configs[slave_index].sm[1]; // SM1 = MailboxWrite (M→S)
        if (sm0.type == PDO::SyncManagerType::MailboxRead ||
            sm1.type == PDO::SyncManagerType::MailboxWrite) {
            if (mbx_write_addr) *mbx_write_addr = sm1.phys_start_addr;
            if (mbx_write_len)  *mbx_write_len  = sm1.length;
            if (mbx_read_addr)  *mbx_read_addr  = sm0.phys_start_addr;
            if (mbx_read_len)   *mbx_read_len   = sm0.length;
            return true;
        }
    }

    // Fallback to explicit SDO-configured mailbox (legacy behavior)
    if (!slave_mbx_[slave_index].configured) return false;
    if (mbx_write_addr) *mbx_write_addr = slave_mbx_[slave_index].write_addr;
    if (mbx_write_len)  *mbx_write_len  = slave_mbx_[slave_index].write_len;
    if (mbx_read_addr)  *mbx_read_addr  = slave_mbx_[slave_index].read_addr;
    if (mbx_read_len)   *mbx_read_len   = slave_mbx_[slave_index].read_len;
    return true;
}

// ============================================================================
// SDOManager — Async API
// ============================================================================

uint32_t SDOManager::queueRequest(SDORequest& request)
{
    uint32_t req_id = next_request_id_.fetch_add(1, std::memory_order_relaxed);
    request.request_id = req_id;

    QueuedRequest queued{};
    queued.request = request;
    queued.enqueue_time_us = transport_.getMicroseconds();

    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (queue_.size() >= kQueueDepth) {
            TETHER_LOGW(TAG, "SDO queue full");
            return 0;
        }
        queue_.push_back(queued);
        queue_cv_.notify_one();
    }

    return req_id;
}

bool SDOManager::isComplete(uint32_t request_id)
{
    std::lock_guard<std::mutex> lock(responses_mutex_);
    return completed_responses_.find(request_id) != completed_responses_.end();
}

bool SDOManager::getResponse(uint32_t request_id, SDOResponse& response)
{
    std::lock_guard<std::mutex> lock(responses_mutex_);
    auto it = completed_responses_.find(request_id);
    if (it != completed_responses_.end()) {
        response = it->second;
        completed_responses_.erase(it);
        return true;
    }
    return false;
}

size_t SDOManager::pendingCount()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
}

// ============================================================================
// SDOManager — Sync API
// ============================================================================

bool SDOManager::readSync(uint16_t slave_index, uint16_t index, uint8_t sub,
                           void* data, size_t max_size, uint32_t timeout_ms,
                           size_t* out_len)
{
    auto context = std::make_shared<SyncContext>();

    {
        SDORequest req{};
        req.operation   = SDOOperation::Upload;
        req.slave_index = slave_index;
        req.index       = index;
        req.subindex    = sub;
        req.data_size   = 0;
        req.timeout_ms  = timeout_ms;

        QueuedRequest queued{};
        queued.request = req;
        queued.enqueue_time_us = transport_.getMicroseconds();
        queued.sync_ctx = context;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_.push_back(queued);
            queue_cv_.notify_one();
        }
    }

    // Wait for completion
    {
        std::unique_lock<std::mutex> wait_lock(context->mutex);
        if (!context->cv.wait_for(wait_lock, std::chrono::milliseconds(timeout_ms),
                                  [&]{ return context->complete; })) {
            return false;
        }
    }

    SDOResponse resp{};
    {
        std::lock_guard<std::mutex> lock(context->mutex);
        resp = context->response;
    }

    if (resp.status != SDOStatus::Complete || resp.abort_code != SDOAbortCode::Success) {
        return false;
    }
    if (out_len) *out_len = resp.data_size;
    if (data && max_size > 0) {
        size_t cp = std::min(max_size, resp.data_size);
        std::memcpy(data, resp.data, cp);
    }
    return true;
}

bool SDOManager::writeSync(uint16_t slave_index, uint16_t index, uint8_t sub,
                            const void* data, size_t len, uint32_t timeout_ms)
{
    auto context = std::make_shared<SyncContext>();

    {
        SDORequest req{};
        req.operation   = SDOOperation::Download;
        req.slave_index = slave_index;
        req.index       = index;
        req.subindex    = sub;
        if (data && len > 0) {
            size_t cp = std::min(len, sizeof(req.data));
            std::memcpy(req.data, data, cp);
            req.data_size = cp;
        } else {
            req.data_size = 0;
        }
        req.timeout_ms = timeout_ms;

        QueuedRequest queued{};
        queued.request = req;
        queued.enqueue_time_us = transport_.getMicroseconds();
        queued.sync_ctx = context;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_.push_back(queued);
            queue_cv_.notify_one();
        }
    }

    {
        std::unique_lock<std::mutex> wait_lock(context->mutex);
        if (!context->cv.wait_for(wait_lock, std::chrono::milliseconds(timeout_ms),
                                  [&]{ return context->complete; })) {
            return false;
        }
    }

    SDOResponse resp{};
    {
        std::lock_guard<std::mutex> lock(context->mutex);
        resp = context->response;
    }

    return (resp.status == SDOStatus::Complete && resp.abort_code == SDOAbortCode::Success);
}

// ============================================================================
// SDOManager — Diagnostics
// ============================================================================

void SDOManager::setDiagEnabled(bool enabled)
{
    diag_enabled_.store(enabled, std::memory_order_relaxed);
}

bool SDOManager::isDiagEnabled() const
{
    return diag_enabled_.load(std::memory_order_relaxed);
}

// ============================================================================
// SDOManager — Worker Thread
// ============================================================================

void SDOManager::workerFunction()
{
    while (true) {
        QueuedRequest item;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]{ return !queue_.empty() || shutdown_requested_; });

            if (shutdown_requested_ && queue_.empty()) {
                break;
            }

            item = queue_.front();
            queue_.pop_front();
        }

        SDOResponse resp{};
        resp.request_id = item.request.request_id;

        // Timeout check (if sat in queue too long)
        uint64_t now_us = transport_.getMicroseconds();
        if ((now_us - item.enqueue_time_us) > kQueueTimeoutUs) {
            resp.status     = SDOStatus::Timeout;
            resp.abort_code = SDOAbortCode::Timeout;
        } else {
            bool ok = executeRequest(item.request, resp);
            if (!ok && resp.status == SDOStatus::Complete) {
                resp.status     = SDOStatus::Failed;
                resp.abort_code = SDOAbortCode::GeneralError;
            }
        }

        // Invoke callback if present
        if (item.request.callback) {
            item.request.callback(resp);
        }

        // Handle completion
        if (item.sync_ctx) {
            // Synchronous caller waiting
            std::lock_guard<std::mutex> lock(item.sync_ctx->mutex);
            item.sync_ctx->response = resp;
            item.sync_ctx->complete = true;
            item.sync_ctx->cv.notify_one();
        } else {
            // Asynchronous — store response
            std::lock_guard<std::mutex> lock(responses_mutex_);
            completed_responses_[resp.request_id] = resp;
            // Limit history
            if (completed_responses_.size() > 100) {
                completed_responses_.erase(completed_responses_.begin());
            }
        }
    }
}

bool SDOManager::executeRequest(const SDORequest& req, SDOResponse& resp)
{
    uint16_t slave = req.slave_index;
    resp.request_id = req.request_id;
    resp.slave_index = req.slave_index;
    resp.index      = req.index;
    resp.subindex   = req.subindex;
    resp.operation  = req.operation;

    if (slave >= kMaxSlaves) {
        resp.status     = SDOStatus::Failed;
        resp.abort_code = SDOAbortCode::DeviceStateError;
        return false;
    }

    // Determine effective mailbox addresses/lengths. Prefer the PDO manager's
    // SyncManager configuration so SDO always uses the node-configured SMs.
    uint16_t mbx_wr_addr = 0, mbx_wr_len = 0, mbx_rd_addr = 0, mbx_rd_len = 0;
    bool mbx_available = false;

    const PDO::SlaveConfig* slave_configs = pdo_manager_ ? pdo_manager_->slaveConfigs() : nullptr;
    if (slave_configs) {
        const auto& sm0 = slave_configs[slave].sm[0]; // SM0 = Mailbox Read (SLAVE->MASTER)
        const auto& sm1 = slave_configs[slave].sm[1]; // SM1 = Mailbox Write (MASTER->SLAVE)
        if (sm0.type == PDO::SyncManagerType::MailboxRead ||
            sm1.type == PDO::SyncManagerType::MailboxWrite) {
            mbx_wr_addr = sm1.phys_start_addr;
            mbx_wr_len  = sm1.length;
            mbx_rd_addr = sm0.phys_start_addr;
            mbx_rd_len  = sm0.length;
            mbx_available = true;
        }
    }

    // Fallback to explicit SDO-configured mailbox (legacy)
    if (!mbx_available && slave_mbx_[slave].configured) {
        mbx_wr_addr = slave_mbx_[slave].write_addr;
        mbx_wr_len  = slave_mbx_[slave].write_len;
        mbx_rd_addr = slave_mbx_[slave].read_addr;
        mbx_rd_len  = slave_mbx_[slave].read_len;
        mbx_available = true;
    }

    if (!mbx_available) {
        resp.status     = SDOStatus::Failed;
        resp.abort_code = SDOAbortCode::DeviceStateError;
        return false;
    }

    // Use persistent mailbox counter from internal state so CoE counter semantics
    // are maintained even when addresses come from PDO configs.
    uint8_t* mbx_counter_ptr = &slave_mbx_[slave].mbx_counter;

    if (req.operation == SDOOperation::Upload) {
        size_t out_len = 0;
        std::vector<uint8_t> buf(1500);

        bool ok = transport_.sdoUpload(
            slave, mbx_counter_ptr,
            mbx_wr_addr, mbx_wr_len,
            mbx_rd_addr, mbx_rd_len,
            req.index, req.subindex,
            buf.data(), buf.size(), &out_len
        );

        if (ok) {
            resp.status     = SDOStatus::Complete;
            resp.abort_code = SDOAbortCode::Success;
            resp.data_size  = std::min(out_len, kMaxSDODataSize);
            std::memcpy(resp.data, buf.data(), resp.data_size);
            resp.duration_ms = 0;
            return true;
        } else {
            resp.status     = SDOStatus::Failed;
            resp.abort_code = SDOAbortCode::TransferAborted;
            return false;
        }

    } else if (req.operation == SDOOperation::Download) {
        size_t to_send = std::min(req.data_size, kMaxSDODataSize);

        bool ok = transport_.sdoDownload(
            slave, mbx_counter_ptr,
            mbx_wr_addr, mbx_wr_len,
            mbx_rd_addr, mbx_rd_len,
            req.index, req.subindex,
            req.data, to_send
        );

        if (ok) {
            resp.status     = SDOStatus::Complete;
            resp.abort_code = SDOAbortCode::Success;
            resp.data_size  = 0;
            return true;
        } else {
            resp.status     = SDOStatus::Failed;
            resp.abort_code = SDOAbortCode::TransferAborted;
            return false;
        }
    }

    resp.status     = SDOStatus::Failed;
    resp.abort_code = SDOAbortCode::InvalidCommand;
    return false;
}

} // namespace SDO
} // namespace EtherCAT
