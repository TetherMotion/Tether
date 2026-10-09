/**
 * @file CoEEntryTransactions.cpp
 * @brief CoE object-dictionary entry transaction implementations
 *        (worker-side read/write decode + the CoEManager entry API).
 *
 * TU split out of CoEManager.cpp.
 */

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoETypes.hpp"
#include "tether/ethercat/CustomPDOMapping.hpp" // inferByteSize
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"
#include "raw/CoEErrorStrings.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <format>
#include <type_traits>
#include <vector>

namespace EtherCAT {
namespace CoE {

static const char* TAG = "coe_mgr";


namespace {

// Worker-side read transaction that uploads the raw payload and decodes it
// against the entry's declared data_type (width + signedness).
class CoEEntryReadTransactionImpl : public ICoEReadTransaction {
public:
    explicit CoEEntryReadTransactionImpl(CoEEntryReadTransaction txn)
        : txn_(std::move(txn)) {}

    const CoETransactionBase& base() const override { return txn_; }

    void execute(CoEManager& mgr) override {
        if (!mgr.getMailbox(nullptr, nullptr, nullptr, nullptr)) {
            result_ = CoEResult<uint64_t>(
                std::unexpected(CoEErrorCode::NotConfigured));
            return;
        }

        std::vector<uint8_t> buf(1500);
        size_t out_len = 0;
        const bool ok = mgr.sdoUploadWithRetry(
            txn_.index, txn_.subindex, buf.data(), buf.size(), &out_len,
            txn_.options);
        if (!ok) {
            if (mgr.lastSdoAbortCode() != 0) {
                result_ = std::unexpected(
                    CoEError::aborted(mgr.lastSdoAbortCode()));
            } else if (mgr.transport().isCancelRequested()) {
                result_ = std::unexpected(CoEError{CoEErrorCode::ShuttingDown});
            } else {
                result_ = std::unexpected(CoEError{CoEErrorCode::TransportError});
            }
            return;
        }

        const size_t expected = inferByteSize(txn_.data_type);
        if (out_len < expected) {
            TETHER_LOGE(TAG,
                "{}: SDO upload 0x{:04X}:{} succeeded but response size "
                "does not match declared data type: got {} byte(s), "
                "expected {}. Update the register definition's data_type.",
                mgr.logPrefix().c_str(), txn_.index, txn_.subindex,
                out_len, expected);
            result_ = std::unexpected(CoEError{CoEErrorCode::InternalError});
            return;
        }
        if (out_len > expected && !txn_.options.allow_trailing_bytes) {
            TETHER_LOGW(TAG,
                "{}: SDO upload 0x{:04X}:{} returned {} bytes, only the "
                "first {} are used for the declared data type (trailing "
                "{} byte(s) discarded).",
                mgr.logPrefix().c_str(), txn_.index, txn_.subindex,
                out_len, expected, out_len - expected);
        }

        uint64_t value = 0;
        for (size_t i = 0; i < expected; ++i) {
            value |= static_cast<uint64_t>(buf[i]) << (i * 8);
        }
        if (isSignedEntryType(txn_.data_type) && expected < 8 &&
            (value & (uint64_t{1} << (expected * 8 - 1)))) {
            value |= ~uint64_t{0} << (expected * 8);  // sign-extend
        }
        result_ = CoEResult<uint64_t>(value);
    }

    void deliver() override {
        if (!result_) return;
        txn_.promise.set_value(std::move(*result_));
        result_.reset();
    }
    void fail(CoEError err) override {
        txn_.promise.set_value(std::unexpected(err));
    }

private:
    static bool isSignedEntryType(
        ObjectDictionary::ObjectDictionaryDataType dt) noexcept {
        using D = ObjectDictionary::ObjectDictionaryDataType;
        switch (dt) {
            case D::Integer8:  case D::Integer16: case D::Integer24:
            case D::Integer32: case D::Integer40: case D::Integer48:
            case D::Integer56: case D::Integer64:
                return true;
            default:
                return false;
        }
    }

    CoEEntryReadTransaction txn_;
    std::optional<CoEResult<uint64_t>> result_;
};

} // namespace

std::future<CoEResult<uint64_t>> CoEManager::readEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    CoETransactionOptions options) {
    auto failNow = [](CoEErrorCode c) {
        CoEEntryReadTransaction t;
        t.promise.set_value(std::unexpected(c));
        return t.promise.get_future();
    };

    if (inferByteSize(entry.data_type) == 0) {
        TETHER_LOGE(TAG, "{}: readEntry 0x{:04X}:{} ('{}') has unsupported "
                    "data type 0x{:04X} for typed access",
                    log_prefix_.c_str(), entry.index, entry.subindex,
                    entry.name ? entry.name : "?",
                    static_cast<uint16_t>(entry.data_type));
        return failNow(CoEErrorCode::NotConfigured);
    }

    if (debug_flags_.coeReads) {
        TETHER_LOGI(TAG, "{}: CoE entry read START 0x{:04X}:{}",
                    log_prefix_.c_str(), entry.index, entry.subindex);
    }

    CoEEntryReadTransaction txn;
    txn.index = entry.index;
    txn.subindex = entry.subindex;
    txn.data_type = entry.data_type;
    txn.options = options;
    txn.enqueue_time = std::chrono::steady_clock::now();

    auto future = txn.promise.get_future();
    auto impl = std::make_unique<CoEEntryReadTransactionImpl>(std::move(txn));

    if (!initialized_.load() || state_.shutdown_requested.load()) {
        return failNow(CoEErrorCode::NotConfigured);
    }
    if (transport_.isCancelRequested()) {
        return failNow(CoEErrorCode::ShuttingDown);
    }
    {
        std::lock_guard<std::mutex> lock(state_.queue_mutex);
        if (state_.shutdown_requested.load()) {
            return failNow(CoEErrorCode::ShuttingDown);
        }
        if (state_.read_queue.size() >= kMaxQueueDepth) {
            return failNow(CoEErrorCode::QueueFull);
        }
        state_.read_queue.push_back(std::move(impl));
    }
    state_.work_cv.notify_one();
    return future;
}

std::future<CoEResult<void>> CoEManager::writeEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint64_t value, CoETransactionOptions opts) {
    const uint8_t width = inferByteSize(entry.data_type);
    if (width == 0) {
        TETHER_LOGE(TAG, "{}: writeEntry 0x{:04X}:{} ('{}') has unsupported "
                    "data type 0x{:04X} for typed access",
                    log_prefix_.c_str(), entry.index, entry.subindex,
                    entry.name ? entry.name : "?",
                    static_cast<uint16_t>(entry.data_type));
        CoEWriteTransaction t;
        t.promise.set_value(std::unexpected(CoEErrorCode::NotConfigured));
        return t.promise.get_future();
    }
    uint8_t buf[8];
    for (size_t i = 0; i < width; ++i) {
        buf[i] = static_cast<uint8_t>(value >> (i * 8));
    }
    return write(entry.index, entry.subindex, buf, width, opts);
}

CoEResult<uint64_t> CoEManager::readEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    CoETransactionOptions opts) {
    auto future = readEntryAsync(entry, opts);
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : kDefaultTimeoutMs;
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
            std::future_status::ready) {
        return std::unexpected(CoEErrorCode::Timeout);
    }
    return future.get();
}

CoEResult<void> CoEManager::writeEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint64_t value, CoETransactionOptions opts) {
    auto future = writeEntryAsync(entry, value, opts);
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : kDefaultTimeoutMs;
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
            std::future_status::ready) {
        return std::unexpected(CoEErrorCode::Timeout);
    }
    return future.get();
}

std::future<CoEResult<uint64_t>> CoEManager::readEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, CoETransactionOptions opts) {
    ObjectDictionary::ObjectDictionaryEntry e = entry;
    e.subindex = subindex;
    return readEntryAsync(e, opts);
}

std::future<CoEResult<void>> CoEManager::writeEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, uint64_t value, CoETransactionOptions opts) {
    ObjectDictionary::ObjectDictionaryEntry e = entry;
    e.subindex = subindex;
    return writeEntryAsync(e, value, opts);
}

CoEResult<uint64_t> CoEManager::readEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, CoETransactionOptions opts) {
    auto future = readEntryAsync(entry, subindex, opts);
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : kDefaultTimeoutMs;
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
            std::future_status::ready) {
        return std::unexpected(CoEErrorCode::Timeout);
    }
    return future.get();
}

CoEResult<void> CoEManager::writeEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, uint64_t value, CoETransactionOptions opts) {
    auto future = writeEntryAsync(entry, subindex, value, opts);
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : kDefaultTimeoutMs;
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) !=
            std::future_status::ready) {
        return std::unexpected(CoEErrorCode::Timeout);
    }
    return future.get();
}

} // namespace CoE
} // namespace EtherCAT

