/**
 * @file FoE.cpp
 * @brief File over EtherCAT (FoE) protocol implementation — instance-based, no globals
 */

#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"

#if ECAT_FEATURE_FOE_ENABLED

#include <cstring>
#include <algorithm>

namespace EtherCAT {
namespace FoE {

// ============================================================================
// String helpers (stateless)
// ============================================================================

const char* foe_opcode_string(FoEOpcode op) {
    switch (op) {
        case FoEOpcode::RRQ:   return "RRQ";
        case FoEOpcode::WRQ:   return "WRQ";
        case FoEOpcode::DATA:  return "DATA";
        case FoEOpcode::ACK:   return "ACK";
        case FoEOpcode::ERROR: return "ERROR";
        case FoEOpcode::BUSY:  return "BUSY";
        default:               return "UNKNOWN";
    }
}

const char* foe_error_string(FoEError error) {
    switch (error) {
        case FoEError::SUCCESS:          return "Success";
        case FoEError::NOT_FOUND:        return "File not found";
        case FoEError::ACCESS_DENIED:    return "Access denied";
        case FoEError::DISK_FULL:        return "Disk full";
        case FoEError::ILLEGAL_OP:       return "Illegal operation";
        case FoEError::PACKET_NUM:       return "Wrong packet number";
        case FoEError::ALREADY_EXISTS:   return "File already exists";
        case FoEError::NO_USER:          return "No user logged in";
        case FoEError::BOOTSTRAP_ONLY:   return "Only in bootstrap state";
        case FoEError::NOT_BOOTSTRAP:    return "Not in bootstrap state";
        case FoEError::NO_RIGHTS:        return "Insufficient rights";
        case FoEError::PROGRAM_ERROR:    return "Programming error";
        case FoEError::CHECKSUM_ERROR:   return "Checksum error";
        case FoEError::TIMEOUT:          return "Timeout";
        case FoEError::MAILBOX_ERROR:    return "Mailbox error";
        case FoEError::LOCAL_FILE_ERROR: return "Local file error";
        case FoEError::INVALID_STATE:    return "Invalid state";
        case FoEError::BUFFER_OVERFLOW:  return "Buffer overflow";
        case FoEError::CANCELLED:        return "Cancelled";
        case FoEError::NOT_INITIALIZED:  return "Not initialized";
        default:                         return "Unknown error";
    }
}

// ============================================================================
// FoEManager implementation
// ============================================================================

FoEManager::FoEManager(IFoETransport& transport)
    : transport_(transport)
{
    std::memset(&stats_, 0, sizeof(stats_));
}

FoEManager::~FoEManager() {
    if (initialized_.load()) {
        deinit();
    }
}

bool FoEManager::init() {
    if (initialized_.load()) return true;

    std::lock_guard<std::mutex> lock(mutex_);

    // Reset queue
    queue_head_ = 0;
    queue_tail_ = 0;
    queue_count_ = 0;

    // Reset transfers
    for (auto& t : transfers_) {
        t.active = false;
    }

    // Reset stats
    std::memset(&stats_, 0, sizeof(stats_));

    running_.store(true);
    // Worker thread not started here since transfers are currently stubbed.
    // When full transfer logic is implemented, start thread:
    // worker_thread_ = std::thread([this]{ workerFunction(); });

    initialized_.store(true);
    return true;
}

void FoEManager::deinit() {
    if (!initialized_.load()) return;

    running_.store(false);

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Cancel active transfers
    for (auto& t : transfers_) {
        if (t.active && t.handle) {
            t.handle->result = FoEResult::Failure(FoEError::CANCELLED);
            t.handle->complete.store(true);
        }
        t.active = false;
    }

    queue_head_ = 0;
    queue_tail_ = 0;
    queue_count_ = 0;

    initialized_.store(false);
}

bool FoEManager::isInitialized() const {
    return initialized_.load();
}

// ----- Queue management -----

bool FoEManager::enqueueRequest(const RequestEntry& entry) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_count_ >= kQueueCapacity) return false;
    request_queue_[queue_tail_] = entry;
    queue_tail_ = (queue_tail_ + 1) % kQueueCapacity;
    queue_count_++;
    return true;
}

bool FoEManager::dequeueRequest(RequestEntry& entry) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_count_ == 0) return false;
    entry = request_queue_[queue_head_];
    queue_head_ = (queue_head_ + 1) % kQueueCapacity;
    queue_count_--;
    return true;
}

size_t FoEManager::pendingCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_count_;
}

size_t FoEManager::activeTransferCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto& t : transfers_) {
        if (t.active) count++;
    }
    return count;
}

// ----- Statistics -----

FoEStats FoEManager::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void FoEManager::resetStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::memset(&stats_, 0, sizeof(stats_));
}

// ----- Synchronous Transfer API (stubbed) -----

FoEResult FoEManager::uploadFile(const char* /*local_path*/, const FoETransferConfig& /*config*/) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    // Full implementation would open file via transport_, send WRQ, then
    // iterate DATA/ACK blocks.  Currently stubbed.
    return FoEResult::Failure(FoEError::INVALID_STATE);
}

FoEResult FoEManager::downloadFile(const char* /*local_path*/, const FoETransferConfig& /*config*/) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    return FoEResult::Failure(FoEError::INVALID_STATE);
}

FoEResult FoEManager::uploadMemory(const void* /*data*/, size_t /*size*/, const FoETransferConfig& /*config*/) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    return FoEResult::Failure(FoEError::INVALID_STATE);
}

FoEResult FoEManager::downloadMemory(void* /*buffer*/, size_t /*buffer_size*/,
                                      size_t* /*received_size*/, const FoETransferConfig& /*config*/) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    return FoEResult::Failure(FoEError::INVALID_STATE);
}

// ----- Asynchronous Transfer API (stubbed) -----

bool FoEManager::uploadFileAsync(const char* /*local_path*/, const FoETransferConfig& /*config*/,
                                  FoETransferHandle* /*handle*/) {
    if (!initialized_.load()) return false;
    return false;  // Stubbed
}

bool FoEManager::downloadFileAsync(const char* /*local_path*/, const FoETransferConfig& /*config*/,
                                    FoETransferHandle* /*handle*/) {
    if (!initialized_.load()) return false;
    return false;  // Stubbed
}

bool FoEManager::waitComplete(FoETransferHandle* handle, uint32_t /*timeout_ms*/) {
    if (!handle) return false;
    return handle->complete.load();
}

void FoEManager::cancel(FoETransferHandle* handle) {
    if (handle) {
        handle->cancel.store(true);
    }
}

} // namespace FoE
} // namespace EtherCAT

#endif // ECAT_FEATURE_FOE_ENABLED
