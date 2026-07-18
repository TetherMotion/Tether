/**
 * @file CoEManager.hpp
 * @brief Per-slave CoE mailbox transaction manager
 *
 * Each CoEManager instance is bound to a single slave and manages its own
 * priority queues (read/write) with a demand-driven worker thread.
 * Enqueue returns std::future<std::expected<T, CoEError>> for async/await
 * compatibility. A legacy queueRequest/getResponse API is also provided.
 */

#pragma once

#include "tether/ethercat/CoETypes.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/SDOManager.hpp" // ISDOTransport, SDORequest, SDOResponse
#include "logging/Logger.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <map>
#include <vector>

namespace EtherCAT {

class PDOManager;

namespace CoE {

static constexpr size_t kMaxQueueDepth = 32;
static constexpr uint32_t kDefaultTimeoutMs = 1000;
static constexpr uint32_t kDefaultPollIntervalMs = 5;
// kWorkerIdleTimeout removed — persistent worker, no idle timeout

// ============================================================================
// Type-erased read transaction base for queue storage
// ============================================================================

class ICoEReadTransaction {
public:
    virtual ~ICoEReadTransaction() = default;
    virtual const CoETransactionBase& base() const = 0;
    virtual void execute(class CoEManager& mgr) = 0;
    virtual void fail(CoEError err) = 0;
};

// ============================================================================
// Type-erased pending future for legacy queueRequest API
// ============================================================================

struct IPendingFuture {
    virtual ~IPendingFuture() = default;
    virtual bool isReady() const = 0;
    virtual SDO::SDOResponse consume() = 0;
};

// ============================================================================
// CoEManager
// ============================================================================

class CoEManager {
public:
    explicit CoEManager(uint16_t slave_index, SDO::ISDOTransport& transport);
    ~CoEManager();

    CoEManager(const CoEManager&) = delete;
    CoEManager& operator=(const CoEManager&) = delete;

    // ----- Lifecycle -----

    bool init();
    void deinit();
    bool isInitialized() const;

    uint16_t slaveIndex() const { return slave_index_; }

    // ----- Mailbox Configuration -----

    void configureMailbox(uint16_t mbx_write_addr, uint16_t mbx_write_len,
                          uint16_t mbx_read_addr, uint16_t mbx_read_len);

    bool getMailbox(uint16_t* mbx_write_addr, uint16_t* mbx_write_len,
                    uint16_t* mbx_read_addr, uint16_t* mbx_read_len) const;

    // ----- Async Read API -----

    template<typename T>
    std::future<CoEResult<T>> read(uint16_t index, uint8_t subindex,
                                   CoETransactionOptions options = {});

    // ----- Async Write API -----

    std::future<CoEResult<void>> write(uint16_t index, uint8_t subindex,
                                       const void* data, size_t size,
                                       CoETransactionOptions options = {});

    // ----- Legacy Polling Queue API -----

    uint32_t queueRequest(SDO::SDORequest& request);
    bool getResponse(uint32_t request_id, SDO::SDOResponse& response);
    bool waitForResponse(uint32_t request_id, SDO::SDOResponse& response,
                         uint32_t timeout_ms = kDefaultTimeoutMs);
    size_t pendingCount() const;

    // ----- Sync Convenience -----

    template<typename T>
    CoEResult<T> readSync(uint16_t index, uint8_t subindex,
                          CoETransactionOptions options = {});

    bool readSync(uint16_t index, uint8_t subindex,
                  void* data, size_t max_size, uint32_t timeout_ms,
                  size_t* actual_size = nullptr);

    CoEResult<void> writeSync(uint16_t index, uint8_t subindex,
                              const void* data, size_t size,
                              CoETransactionOptions options = {});

    // ----- Typed Sync Helpers -----

    CoEResult<uint8_t>  readU8(uint16_t idx, uint8_t sub,
                               CoETransactionOptions opts = {});
    CoEResult<uint16_t> readU16(uint16_t idx, uint8_t sub,
                                CoETransactionOptions opts = {});
    CoEResult<uint32_t> readU32(uint16_t idx, uint8_t sub,
                                CoETransactionOptions opts = {});
    CoEResult<int32_t>  readI32(uint16_t idx, uint8_t sub,
                                CoETransactionOptions opts = {});
    CoEResult<void>     writeU8(uint16_t idx, uint8_t sub,
                                uint8_t val, CoETransactionOptions opts = {});
    CoEResult<void>     writeU16(uint16_t idx, uint8_t sub,
                                 uint16_t val, CoETransactionOptions opts = {});
    CoEResult<void>     writeU32(uint16_t idx, uint8_t sub,
                                 uint32_t val, CoETransactionOptions opts = {});
    CoEResult<void>     writeI32(uint16_t idx, uint8_t sub,
                                 int32_t val, CoETransactionOptions opts = {});

    // ----- Queue Status -----

    size_t pendingReadCount() const;
    size_t pendingWriteCount() const;
    size_t totalPendingCount() const;

    // ----- PDO Integration -----

    void setPDOManager(PDOManager* mgr) { pdo_manager_ = mgr; }
    PDOManager* pdoManager() const { return pdo_manager_; }

    void setDiagEnabled(bool enabled) { diag_enabled_.store(enabled); }
    bool isDiagEnabled() const { return diag_enabled_.load(); }

    void setBehaviourOptions(const BehaviourOptions& opts) { behaviour_options_ = opts; }
    const BehaviourOptions& behaviourOptions() const { return behaviour_options_; }

    // ----- Debug flags -----

    /** @brief Update the per-slave debug flags distributed by the master. */
    void updateDebugFlags(const EtherCATSlaveDebugFlags& flags) { debug_flags_ = flags; }

    /** @brief Access the current per-slave debug flags. */
    const EtherCATSlaveDebugFlags& debugFlags() const { return debug_flags_; }

    // ----- Internal: called by ICoEReadTransaction::execute -----

    SDO::ISDOTransport& transport() { return transport_; }
    uint8_t* mbxCounterPtr();

    bool resolveMailbox(uint16_t& wr_addr, uint16_t& wr_len,
                        uint16_t& rd_addr, uint16_t& rd_len);

    bool isRequestInFlight() const { return request_in_flight_.load(); }
    void clearRequestInFlight() { request_in_flight_.store(false); }

    bool sdoUploadWithRetry(uint16_t index, uint8_t subindex,
                            uint8_t* out, size_t out_cap, size_t* out_len,
                            const CoETransactionOptions& options);

    bool sdoDownloadWithRetry(uint16_t index, uint8_t subindex,
                              const uint8_t* data, size_t data_len,
                              const CoETransactionOptions& options);

private:
    struct MailboxConfig {
        uint16_t write_addr = 0;
        uint16_t write_len  = 0;
        uint16_t read_addr  = 0;
        uint16_t read_len   = 0;
        uint8_t  mbx_counter = 1;
        bool configured = false;
    };

    struct WriteQueueEntry {
        CoEWriteTransaction txn;
        bool operator<(const WriteQueueEntry& other) const {
            return static_cast<const CoETransactionBase&>(txn) <
                   static_cast<const CoETransactionBase&>(other.txn);
        }
    };

    struct SlaveState {
        std::deque<std::unique_ptr<ICoEReadTransaction>> read_queue;
        std::deque<WriteQueueEntry> write_queue;
        mutable std::mutex queue_mutex;
        std::condition_variable work_cv;
        std::unique_ptr<std::thread> worker_thread;
        std::atomic<bool> shutdown_requested{false};
        std::atomic<bool> worker_running{false};
        mutable std::mutex worker_mutex;
    };

    void workerLoop();
    void storeResponse(uint32_t request_id, const SDO::SDOResponse& resp);
    bool popResponse(uint32_t request_id, SDO::SDOResponse& resp);
    uint32_t nextRequestId();
    void logALStatusAfterRequest();

    uint16_t slave_index_;
    SDO::ISDOTransport& transport_;
    MailboxConfig mbx_;
    SlaveState state_;

    mutable std::mutex responses_mutex_;
    std::map<uint32_t, SDO::SDOResponse> completed_responses_;

    mutable std::mutex pending_futures_mutex_;
    std::map<uint32_t, std::unique_ptr<IPendingFuture>> pending_futures_;

    std::atomic<uint32_t> next_request_id_{1};

    PDOManager* pdo_manager_ = nullptr;
    std::atomic<bool> diag_enabled_{false};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> request_in_flight_{false};

    BehaviourOptions behaviour_options_;

    EtherCATSlaveDebugFlags debug_flags_;
};

// ============================================================================
// Template Implementations
// ============================================================================

template<typename T>
class CoEReadTransactionImpl : public ICoEReadTransaction {
public:
    explicit CoEReadTransactionImpl(CoEReadTransaction<T> txn)
        : txn_(std::move(txn)) {}

    const CoETransactionBase& base() const override { return txn_; }
    void execute(CoEManager& mgr) override;
    void fail(CoEError err) override {
        txn_.promise.set_value(std::unexpected(err));
    }

    CoEReadTransaction<T> txn_;
};

template<typename T>
std::future<CoEResult<T>> CoEManager::read(uint16_t index, uint8_t subindex,
                                            CoETransactionOptions options) {
    if (debug_flags_.coeReads) {
        TETHER_LOGI("coe_mgr", "Slave %u: CoE read START index=0x%04X:%u", slave_index_, index, subindex);
    }

    CoEReadTransaction<T> txn;
    txn.index = index;
    txn.subindex = subindex;
    txn.options = options;
    txn.enqueue_time = std::chrono::steady_clock::now();

    auto future = txn.promise.get_future();

    auto impl = std::make_unique<CoEReadTransactionImpl<T>>(std::move(txn));

    if (!initialized_.load() || state_.shutdown_requested.load()) {
        CoEReadTransaction<T> fail_txn;
        fail_txn.promise.set_value(std::unexpected(CoEError::NotConfigured));
        return fail_txn.promise.get_future();
    }

    {
        std::lock_guard<std::mutex> lock(state_.queue_mutex);
        if (state_.shutdown_requested.load()) {
            CoEReadTransaction<T> fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEError::ShuttingDown));
            return fail_txn.promise.get_future();
        }
        if (state_.read_queue.size() >= kMaxQueueDepth) {
            if (debug_flags_.coeReads) {
                TETHER_LOGI("coe_mgr", "Slave %u: CoE read QUEUE FULL index=0x%04X:%u", slave_index_, index, subindex);
            }
            CoEReadTransaction<T> fail_txn;
            fail_txn.promise.set_value(std::unexpected(CoEError::QueueFull));
            return fail_txn.promise.get_future();
        }
        state_.read_queue.push_back(std::move(impl));
    }
    state_.work_cv.notify_one();

    if (debug_flags_.coeReads) {
        TETHER_LOGI("coe_mgr", "Slave %u: CoE read ENQUEUED index=0x%04X:%u", slave_index_, index, subindex);
    }
    return future;
}

template<typename T>
CoEResult<T> CoEManager::readSync(uint16_t index, uint8_t subindex,
                                   CoETransactionOptions options) {
    return read<T>(index, subindex, options).get();
}

} // namespace CoE
} // namespace EtherCAT
