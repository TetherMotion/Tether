/**
 * @file CoEManager.cpp
 * @brief CoEManager implementation — multi-slave CoE mailbox transaction manager
 */

#include "tether/ethercat/CoEManager.hpp"
#include "raw/CoEErrorStrings.hpp"
#include "tether/ethercat/CustomPDOMapping.hpp" // inferByteSize
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
#include <type_traits>
#include <vector>
#include <format>

namespace EtherCAT {
namespace CoE {

static const char* TAG = "coe_mgr";

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
        TETHER_LOGE(TAG, "{}: Failed to create CoE worker thread", log_prefix_.c_str());
        return false;
    }

    initialized_.store(true);
    if (debug_flags_.mailboxConfiguration) {
        TETHER_LOGI(TAG, "{}: CoEManager initialized", log_prefix_.c_str());
    }
    return true;
}

void CoEManager::deinit() {
    // Guard against multiple deinit() calls (from Master::stop(), ~Master(),
    // and ~CoEManager()).  Only the first call performs the full shutdown.
    if (!initialized_.exchange(false)) {
        return;
    }

    // Hold queue_mutex while setting shutdown_requested and notifying the
    // worker.  This closes the lost-wakeup window: without the lock, the
    // worker could be between the predicate check in work_cv.wait() and the
    // actual blocking futex call when notify_all() fires, causing the
    // notification to be lost and join() to hang forever.
    {
        std::lock_guard<std::mutex> qlock(state_.queue_mutex);
        state_.shutdown_requested.store(true);
        state_.work_cv.notify_all();
    }

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
            txn->fail(CoEErrorCode::ShuttingDown);
        }
        state_.read_queue.clear();
        for (auto& entry : state_.write_queue) {
            entry.txn.promise.set_value(std::unexpected(CoEErrorCode::ShuttingDown));
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

    if (debug_flags_.shutdown) {
        TETHER_LOGI(TAG, "{}: CoEManager deinitialized", log_prefix_.c_str());
    }
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

    if (debug_flags_.mailboxConfiguration) {
        TETHER_LOGI(TAG, "{}: mailbox: Receive(SM0/MbxIn)=0x{:04x}/{}, Send(SM1/MbxOut)=0x{:04x}/{}",
                 log_prefix_.c_str(), mbx_write_addr, mbx_write_len, mbx_read_addr, mbx_read_len);
    }
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
// Async Write API
// ============================================================================

// ============================================================================
// Async Write API
// ============================================================================

std::future<CoEResult<void>> CoEManager::write(uint16_t index, uint8_t subindex,
                                                const void* data, size_t size,
                                                CoETransactionOptions options) {
    if (debug_flags_.coeWrites) {
        TETHER_LOGI(TAG, "{}: CoE write START index=0x{:04X}:{} size={}",
                    log_prefix_.c_str(), index, subindex, size);
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
        fail_txn.promise.set_value(std::unexpected(CoEErrorCode::NotConfigured));
        return fail_txn.promise.get_future();
    }

    // Master-level cancellation (Ctrl-C / Master::stop) — fail immediately
    // instead of queueing an SDO that is guaranteed to fail.
    if (transport_.isCancelRequested()) {
        CoEWriteTransaction fail_txn;
        fail_txn.promise.set_value(std::unexpected(CoEErrorCode::ShuttingDown));
        return fail_txn.promise.get_future();
    }

    {
        std::lock_guard<std::mutex> lock(state_.queue_mutex);
        if (state_.shutdown_requested.load()) {
            CoEWriteTransaction fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEErrorCode::ShuttingDown));
            return fail_txn.promise.get_future();
        }
        if (state_.write_queue.size() >= kMaxQueueDepth) {
            if (debug_flags_.coeWrites) {
                TETHER_LOGI(TAG,
                    "{}: CoE write QUEUE FULL (index=0x{:04X}:{}, "
                    "Tether max={} pending). This is a Tether limit, not a slave limit. "
                    "Increase ECAT_COE_QUEUE_DEPTH in EtherCATConfig.hpp.",
                    log_prefix_.c_str(), index, subindex, kMaxQueueDepth);
            }
            CoEWriteTransaction fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEErrorCode::QueueFull));
            return fail_txn.promise.get_future();
        }
        state_.write_queue.push_back(std::move(entry));
    }
    state_.work_cv.notify_one();

    if (debug_flags_.coeWrites) {
        TETHER_LOGI(TAG, "{}: CoE write ENQUEUED index=0x{:04X}:{}",
                    log_prefix_.c_str(), index, subindex);
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
        TETHER_LOGI(TAG, "{}: CoE writeSync index=0x{:04X}:{} size={}",
                    log_prefix_.c_str(), index, subindex, size);
    }
    return write(index, subindex, data, size, options).get();
}

bool CoEManager::readSync(uint16_t index, uint8_t subindex,
                           void* data, size_t max_size, uint32_t timeout_ms,
                           size_t* actual_size) {
    if (debug_flags_.coeReads) {
        TETHER_LOGI(TAG, "{}: CoE readSync index=0x{:04X}:{} max_size={}",
                    log_prefix_.c_str(), index, subindex, max_size);
    }
    CoETransactionOptions opts;
    opts.timeout_ms = timeout_ms;

    auto result = readSync<std::vector<uint8_t>>(index, subindex, opts);
    if (!result.has_value()) return false;

    auto& vec = result.value();
    size_t copy_len = std::min(vec.size(), max_size);
    if (data && copy_len > 0) {
        std::memcpy(data, vec.data(), copy_len);
    }
    if (actual_size) *actual_size = vec.size();
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

} // namespace CoE
} // namespace EtherCAT
