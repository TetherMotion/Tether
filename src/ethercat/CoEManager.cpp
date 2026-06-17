/**
 * @file CoEManager.cpp
 * @brief CoEManager implementation — multi-slave CoE mailbox transaction manager
 */

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
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
        case CoEError::SlaveNotFound:   return "Slave not found";
        case CoEError::InternalError:   return "Internal error";
        default:                        return "Unknown error";
    }
}

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
    initialized_.store(true);
    TETHER_LOGI(TAG, "CoEManager initialized for slave %u", slave_index_);
    return true;
}

void CoEManager::deinit() {
    initialized_.store(false);

    state_.shutdown_requested.store(true);
    state_.read_cv.notify_all();
    state_.write_cv.notify_all();
    state_.worker_cv.notify_all();

    if (state_.worker_thread && state_.worker_thread->joinable()) {
        state_.worker_thread->join();
    }

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

uint8_t* CoEManager::mbxCounterPtr() {
    return &mbx_.mbx_counter;
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

    if (request.operation == SDO::SDOOperation::Upload) {
        auto future = readSync<std::vector<uint8_t>>(request.index, request.subindex,
            CoETransactionOptions{.timeout_ms = request.timeout_ms});

        SDO::SDOResponse resp{};
        resp.request_id = id;
        resp.slave_index = slave_index_;
        resp.index = request.index;
        resp.subindex = request.subindex;
        resp.operation = request.operation;

        if (future.has_value()) {
            resp.status = SDO::SDOStatus::Complete;
            auto& vec = future.value();
            resp.data_size = std::min(vec.size(), sizeof(resp.data));
            std::memcpy(resp.data, vec.data(), resp.data_size);
        } else {
            resp.status = SDO::SDOStatus::Failed;
        }
        storeResponse(id, resp);
    } else {
        auto future = writeSync(request.index, request.subindex,
            request.data, request.data_size,
            CoETransactionOptions{.timeout_ms = request.timeout_ms});

        SDO::SDOResponse resp{};
        resp.request_id = id;
        resp.slave_index = slave_index_;
        resp.index = request.index;
        resp.subindex = request.subindex;
        resp.operation = request.operation;
        resp.status = future.has_value() ? SDO::SDOStatus::Complete : SDO::SDOStatus::Failed;
        resp.data_size = std::min(request.data_size, sizeof(resp.data));
        std::memcpy(resp.data, request.data, resp.data_size);
        storeResponse(id, resp);
    }

    return id;
}

bool CoEManager::getResponse(uint32_t request_id, SDO::SDOResponse& response) {
    return popResponse(request_id, response);
}

size_t CoEManager::pendingCount() const {
    return totalPendingCount();
}

// ============================================================================
// Async Write API
// ============================================================================

std::future<CoEResult<void>> CoEManager::write(uint16_t index, uint8_t subindex,
                                                const void* data, size_t size,
                                                CoETransactionOptions options) {
    CoEWriteTransaction txn;
    txn.index = index;
    txn.subindex = subindex;
    txn.options = options;
    txn.enqueue_time = std::chrono::steady_clock::now();
    txn.data.assign(static_cast<const uint8_t*>(data),
                    static_cast<const uint8_t*>(data) + size);

    auto future = txn.promise.get_future();

    WriteQueueEntry entry{std::move(txn)};

    {
        std::lock_guard<std::mutex> lock(state_.write_mutex);
        if (state_.write_queue.size() >= kMaxQueueDepth) {
            CoEWriteTransaction fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEError::QueueFull));
            return fail_txn.promise.get_future();
        }
        state_.write_queue.push_back(std::move(entry));
    }
    state_.write_cv.notify_one();
    ensureWorkerRunning();

    return future;
}

// ============================================================================
// Sync Convenience
// ============================================================================

CoEResult<void> CoEManager::writeSync(uint16_t index, uint8_t subindex,
                                       const void* data, size_t size,
                                       CoETransactionOptions options) {
    return write(index, subindex, data, size, options).get();
}

bool CoEManager::readSync(uint16_t index, uint8_t subindex,
                           void* data, size_t max_size, uint32_t timeout_ms,
                           size_t* actual_size) {
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
    std::lock_guard<std::mutex> lock(state_.read_mutex);
    return state_.read_queue.size();
}

size_t CoEManager::pendingWriteCount() const {
    std::lock_guard<std::mutex> lock(state_.write_mutex);
    return state_.write_queue.size();
}

size_t CoEManager::totalPendingCount() const {
    return pendingReadCount() + pendingWriteCount();
}

// ============================================================================
// Worker Thread Management
// ============================================================================

void CoEManager::ensureWorkerRunning() {
    if (state_.worker_running.load()) return;

    std::lock_guard<std::mutex> lock(state_.worker_mutex);
    if (state_.worker_running.load()) return;

    if (state_.worker_thread && state_.worker_thread->joinable()) {
        state_.worker_thread->join();
    }

    state_.shutdown_requested.store(false);
    state_.worker_running.store(true);

    try {
        state_.worker_thread = std::make_unique<std::thread>(&CoEManager::workerLoop, this);
    } catch (...) {
        TETHER_LOGE(TAG, "Slave %u: Failed to create CoE worker thread", slave_index_);
        state_.worker_running.store(false);
    }
}

void CoEManager::workerLoop() {
    TETHER_LOGI(TAG, "Slave %u: CoE worker thread started", slave_index_);

    while (!state_.shutdown_requested.load()) {
        bool did_work = false;

        // Process one read if available
        {
            std::unique_lock<std::mutex> lock(state_.read_mutex);
            if (!state_.read_queue.empty()) {
                auto txn = std::move(state_.read_queue.front());
                state_.read_queue.pop_front();
                lock.unlock();
                txn->execute(*this);
                did_work = true;
            }
        }

        // Process one write if available
        {
            std::unique_lock<std::mutex> lock(state_.write_mutex);
            if (!state_.write_queue.empty()) {
                auto entry = std::move(state_.write_queue.front());
                state_.write_queue.pop_front();
                lock.unlock();

                auto& txn = entry.txn;
                uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
                if (!resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
                    txn.promise.set_value(std::unexpected(CoEError::NotConfigured));
                } else {
                    bool ok = transport_.sdoDownload(
                        slave_index_, mbxCounterPtr(),
                        wr_addr, wr_len, rd_addr, rd_len,
                        txn.index, txn.subindex,
                        txn.data.data(), txn.data.size(),
                        diag_enabled_.load(),
                        txn.options.poll_interval_ms, txn.options.timeout_ms);

                    if (ok) {
                        txn.promise.set_value({});
                    } else {
                        txn.promise.set_value(std::unexpected(CoEError::TransportError));
                    }
                }
                did_work = true;
            }
        }

        if (!did_work) {
            bool has_work = false;
            {
                std::unique_lock<std::mutex> lock(state_.read_mutex);
                state_.read_cv.wait_for(lock, kWorkerIdleTimeout, [this] {
                    return !state_.read_queue.empty() || state_.shutdown_requested.load();
                });
                has_work = !state_.read_queue.empty();
            }
            if (!has_work) {
                std::unique_lock<std::mutex> lock(state_.write_mutex);
                state_.write_cv.wait_for(lock, kWorkerIdleTimeout, [this] {
                    return !state_.write_queue.empty() || state_.shutdown_requested.load();
                });
                has_work = !state_.write_queue.empty();
            }

            if (!has_work && !state_.shutdown_requested.load()) {
                break;
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
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    if (!mgr.resolveMailbox(wr_addr, wr_len, rd_addr, rd_len)) {
        txn_.promise.set_value(std::unexpected(CoEError::NotConfigured));
        return;
    }

    std::vector<uint8_t> buf(1500);
    size_t out_len = 0;

    bool ok = mgr.transport().sdoUpload(
        mgr.slaveIndex(), mgr.mbxCounterPtr(),
        wr_addr, wr_len, rd_addr, rd_len,
        txn_.index, txn_.subindex,
        buf.data(), buf.size(), &out_len,
        mgr.isDiagEnabled(),
        txn_.options.poll_interval_ms, txn_.options.timeout_ms);

    if (!ok) {
        txn_.promise.set_value(std::unexpected(CoEError::TransportError));
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

// Explicit template instantiations
template class CoEReadTransactionImpl<uint8_t>;
template class CoEReadTransactionImpl<uint16_t>;
template class CoEReadTransactionImpl<uint32_t>;
template class CoEReadTransactionImpl<int32_t>;

} // namespace CoE
} // namespace EtherCAT
