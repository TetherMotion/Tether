/**
 * @file CoEManager_worker.cpp
 * @brief CoEManager — worker thread, response storage and entry transactions.
 *
 * TU split out of CoEManager.cpp.
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
// Worker Thread Management
// ============================================================================

void CoEManager::workerLoop() {
    TETHER_LOGI(TAG, "{}: CoE worker thread started", log_prefix_.c_str());

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
                    TETHER_LOGW(TAG, "{}: CoE read started while previous request in flight — stale response possible",
                                log_prefix_.c_str());
                }
                try {
                    txn->execute(*this);
                    // Clear the in-flight flag BEFORE waking the caller so that a
                    // readSync() returning from the promise never observes a
                    // request still in flight.
                    request_in_flight_.store(false);
                    txn->deliver();
                } catch (const std::exception& e) {
                    TETHER_LOGE(TAG, "{}: CoE read threw: {}", log_prefix_.c_str(), e.what());
                } catch (...) {
                    TETHER_LOGE(TAG, "{}: CoE read threw unknown exception", log_prefix_.c_str());
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
                    TETHER_LOGW(TAG, "{}: CoE write started while previous request in flight — stale response possible",
                                log_prefix_.c_str());
                }
                bool ok = false;
                try {
                    if (!getMailbox(nullptr, nullptr, nullptr, nullptr)) {
                        txn.promise.set_value(std::unexpected(CoEErrorCode::NotConfigured));
                        request_in_flight_.store(false);
                        continue;
                    }
                    ok = sdoDownloadWithRetry(
                        txn.index, txn.subindex,
                        txn.data.data(), txn.data.size(),
                        txn.options);
                } catch (const std::exception& e) {
                    TETHER_LOGE(TAG, "{}: CoE write threw: {}", log_prefix_.c_str(), e.what());
                } catch (...) {
                    TETHER_LOGE(TAG, "{}: CoE write threw unknown exception", log_prefix_.c_str());
                }
                request_in_flight_.store(false);

                if (ok) {
                    txn.promise.set_value({});
                } else if (last_sdo_abort_code_.load(std::memory_order_relaxed) != 0) {
                    // Slave explicitly aborted the SDO (definitive rejection,
                    // e.g. 0x06070010 length mismatch). Surface as Aborted with
                    // the real abort code so callers can decode it directly.
                    txn.promise.set_value(std::unexpected(
                        CoEError::aborted(last_sdo_abort_code_.load(std::memory_order_relaxed))));
                } else if (transport_.isCancelRequested()) {
                    txn.promise.set_value(std::unexpected(CoEErrorCode::ShuttingDown));
                } else {
                    txn.promise.set_value(std::unexpected(CoEErrorCode::TransportError));
                }
                continue;
            }
        }
    }

    state_.worker_running.store(false);
    if (debug_flags_.shutdown) {
        TETHER_LOGI(TAG, "{}: CoE worker thread stopped", log_prefix_.c_str());
    }
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
    // Check mailbox configuration before attempting I/O so that the
    // proper CoEErrorCode::NotConfigured is surfaced (maps to DeviceStateError
    // abort code) rather than a generic TransportError.
    if (!mgr.getMailbox(nullptr, nullptr, nullptr, nullptr)) {
        result_ = CoEResult<T>(std::unexpected(CoEErrorCode::NotConfigured));
        return;
    }

    std::vector<uint8_t> buf(1500);
    size_t out_len = 0;

    bool ok = mgr.sdoUploadWithRetry(
        txn_.index, txn_.subindex,
        buf.data(), buf.size(), &out_len,
        txn_.options);

    if (!ok) {
        if (mgr.lastSdoAbortCode() != 0) {
            result_ = CoEResult<T>(std::unexpected(CoEError::aborted(mgr.lastSdoAbortCode())));
        } else if (mgr.transport().isCancelRequested()) {
            result_ = CoEResult<T>(std::unexpected(CoEErrorCode::ShuttingDown));
        } else {
            result_ = CoEResult<T>(std::unexpected(CoEErrorCode::TransportError));
        }
        return;
    }

    if (out_len < sizeof(T)) {
        // The slave returned fewer bytes than the typed read expects. This is
        // not a transport/timeout — the SDO upload itself succeeded, but the
        // response payload does not match the requested C++ type (e.g. reading
        // a 1-byte Unsigned8 object with sdoReadU32). Surface the actual
        // received length and the expected length so the caller can correct
        // the read type instead of chasing a misleading "transport" error.
        TETHER_LOGE(TAG,
                    "{}: SDO upload 0x{:04X}:{} succeeded but response size "
                    "does not match requested type: got {} byte(s), expected {}. "
                    "Use a smaller typed read (e.g. sdoReadU8 for a 1-byte object) "
                    "or read as a raw byte vector.",
                    mgr.logPrefix().c_str(), txn_.index, txn_.subindex,
                    out_len, sizeof(T));
        result_ = CoEResult<T>(std::unexpected(CoEErrorCode::InternalError));
        return;
    }

    if (out_len > sizeof(T)) {
        // The slave returned MORE bytes than the typed read expects. The
        // leading sizeof(T) bytes are still copied below, but warn so a
        // truncated response does not pass silently. Callers that knowingly
        // read a narrow value from a wider SDO entry (e.g. a 1-byte module
        // ID stored in a 4-byte OD entry) may set
        // CoETransactionOptions::allow_trailing_bytes to suppress this
        // warning for this transaction.
        if (!txn_.options.allow_trailing_bytes) {
            TETHER_LOGW(TAG,
                        "{}: SDO upload 0x{:04X}:{} returned {} bytes, only "
                        "the first {} are used for the requested typed read (trailing "
                        "{} byte(s) discarded).",
                        mgr.logPrefix().c_str(), txn_.index, txn_.subindex,
                        out_len, sizeof(T), out_len - sizeof(T));
        }
    }

    T value;
    std::memcpy(&value, buf.data(), sizeof(T));
    result_ = CoEResult<T>(value);
}

// Specialization for std::vector<uint8_t> — used by the raw-buffer readSync overload
template<>
void CoEReadTransactionImpl<std::vector<uint8_t>>::execute(CoEManager& mgr) {
    if (!mgr.getMailbox(nullptr, nullptr, nullptr, nullptr)) {
        result_ = CoEResult<std::vector<uint8_t>>(std::unexpected(CoEErrorCode::NotConfigured));
        return;
    }

    std::vector<uint8_t> buf(1500);
    size_t out_len = 0;

    bool ok = mgr.sdoUploadWithRetry(
        txn_.index, txn_.subindex,
        buf.data(), buf.size(), &out_len,
        txn_.options);

    if (!ok) {
        if (mgr.lastSdoAbortCode() != 0) {
            result_ = CoEResult<std::vector<uint8_t>>(std::unexpected(CoEError::aborted(mgr.lastSdoAbortCode())));
        } else if (mgr.transport().isCancelRequested()) {
            result_ = CoEResult<std::vector<uint8_t>>(std::unexpected(CoEErrorCode::ShuttingDown));
        } else {
            result_ = CoEResult<std::vector<uint8_t>>(std::unexpected(CoEErrorCode::TransportError));
        }
        return;
    }

    std::vector<uint8_t> result(buf.data(), buf.data() + out_len);
    result_ = CoEResult<std::vector<uint8_t>>(std::move(result));
}

// Explicit template instantiations
template class CoEReadTransactionImpl<uint8_t>;
template class CoEReadTransactionImpl<uint16_t>;
template class CoEReadTransactionImpl<uint32_t>;
template class CoEReadTransactionImpl<int32_t>;
template class CoEReadTransactionImpl<std::vector<uint8_t>>;

} // namespace CoE
} // namespace EtherCAT

