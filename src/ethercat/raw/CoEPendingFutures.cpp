/**
 * @file CoEPendingFutures.cpp
 * @brief CoE legacy queueRequest pending-future adapters.
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
            resp.abort_code = SDOAbortCode::Success;
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
            resp.abort_code = SDOAbortCode::Success;
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
            if (!result.has_value() && result.error() == CoEErrorCode::QueueFull) {
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
                resp.abort_code = SDOAbortCode::Success;
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
            if (!result.has_value() && result.error() == CoEErrorCode::QueueFull) {
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
                resp.abort_code = SDOAbortCode::Success;
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

} // namespace CoE
} // namespace EtherCAT

