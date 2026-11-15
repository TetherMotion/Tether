/**
 * @file FoE.cpp
 * @brief File over EtherCAT (FoE) protocol implementation — instance-based, no globals
 */

#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"

#if ECAT_FEATURE_FOE_ENABLED

#include <cstring>
#include <algorithm>
#include <thread>
#include <chrono>

namespace EtherCAT {
namespace FoE {

namespace {

// Standard FoE mailbox locations (master->slave WRQ/DATA, slave->master
// responses). Slaves that place FoE elsewhere must be handled by the
// transport implementation, which receives these addresses per call.
constexpr uint16_t kFoeMbxWrAddr = 0x1000;
constexpr uint16_t kFoeMbxRdAddr = 0x1080;
constexpr int kMaxBlockRetries = 2;

// Payload bytes per DATA block (FoE header is 6 bytes on top).
constexpr size_t kChunkPayload = kBufferSize - 16;

inline void putU32le(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

inline uint32_t getU32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// Build an RRQ/WRQ request: opcode(1) + reserved(1) + password(4) + name\0
size_t buildFileRequest(uint8_t opcode, uint32_t password,
                        const char* filename, uint8_t* buf, size_t cap) {
    const size_t nameLen = std::strlen(filename) + 1;
    if (6 + nameLen > cap) return 0;
    buf[0] = opcode;
    buf[1] = 0;
    putU32le(buf + 2, password);
    std::memcpy(buf + 6, filename, nameLen);
    return 6 + nameLen;
}

// Build a DATA or ACK message: opcode(1) + reserved(1) + block_no(4) + data
size_t buildDataMessage(uint8_t opcode, uint32_t blockNo,
                        const uint8_t* data, size_t dataLen,
                        uint8_t* buf, size_t cap) {
    if (6 + dataLen > cap) return 0;
    buf[0] = opcode;
    buf[1] = 0;
    putU32le(buf + 2, blockNo);
    if (dataLen > 0) std::memcpy(buf + 6, data, dataLen);
    return 6 + dataLen;
}

// Transact one FoE message: write request, read response (retrying BUSY).
// Returns true and fills resp/respLen on a non-BUSY response.
// *mailboxFailed is set when the mailbox write itself fails.
bool foeTransact(IFoETransport& transport, uint16_t slaveIndex,
                 const FoETransferConfig& config,
                 const uint8_t* req, size_t reqLen,
                 uint8_t* resp, size_t respCap, size_t* respLen,
                 bool* mailboxFailed = nullptr) {
    if (mailboxFailed) *mailboxFailed = false;
    uint8_t counter = 0;
    for (int attempt = 0; attempt <= kMaxBlockRetries; ++attempt) {
        if (!transport.mailboxWrite(slaveIndex, kFoeMbxWrAddr, kBufferSize,
                                    req, reqLen, &counter)) {
            if (mailboxFailed) *mailboxFailed = true;
            return false;
        }
        size_t len = 0;
        if (!transport.mailboxRead(slaveIndex, kFoeMbxRdAddr, kBufferSize,
                                   resp, respCap, &len, config.timeout_ms)) {
            continue;  // timeout — retry
        }
        if (len >= 1 && resp[0] == static_cast<uint8_t>(FoEOpcode::BUSY)) {
            transport.delayMs(10);
            continue;
        }
        *respLen = len;
        return true;
    }
    return false;
}

// Parse a response header; returns opcode or 0 on malformed input.
uint8_t parseResponseOpcode(const uint8_t* resp, size_t len) {
    return (len >= 1) ? resp[0] : 0;
}

// If resp is an ERROR message, extract the slave error code.
bool parseErrorResponse(const uint8_t* resp, size_t len, FoEError& err) {
    if (len >= 6 && resp[0] == static_cast<uint8_t>(FoEOpcode::ERROR)) {
        err = static_cast<FoEError>(getU32le(resp + 2));
        return true;
    }
    return false;
}

FoEResult makeResult(bool ok, FoEError err, uint32_t bytes, uint32_t blocks,
                     uint32_t startMs, IFoETransport& transport) {
    FoEResult r{};
    r.success = ok;
    r.error_code = err;
    r.bytes_transferred = bytes;
    r.blocks_transferred = blocks;
    r.duration_ms = transport.getTimeMs() - startMs;
    r.error_text[0] = '\0';
    return r;
}

} // namespace

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
    worker_thread_ = std::thread([this] { workerFunction(); });

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

// ============================================================================
// Transfer engine
// ============================================================================

void FoEManager::workerFunction() {
    RequestEntry entry;
    while (running_.load()) {
        if (!dequeueRequest(entry)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        FoEResult result = executeRequest(entry);
        if (entry.handle) {
            entry.handle->result = result;
            entry.handle->complete.store(true);
        }
        recordResult(entry, result);
    }
}

void FoEManager::recordResult(const RequestEntry& entry, const FoEResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool isUpload = (entry.type == RequestEntry::Type::UPLOAD_FILE ||
                           entry.type == RequestEntry::Type::UPLOAD_MEMORY);
    if (result.success) {
        if (isUpload) {
            stats_.uploads_completed++;
            stats_.bytes_uploaded += result.bytes_transferred;
        } else {
            stats_.downloads_completed++;
            stats_.bytes_downloaded += result.bytes_transferred;
        }
    } else {
        if (isUpload) stats_.uploads_failed++;
        else stats_.downloads_failed++;
        if (result.error_code == FoEError::TIMEOUT) stats_.timeouts++;
    }
}

FoEResult FoEManager::executeRequest(const RequestEntry& entry) {
    const uint32_t startMs = transport_.getTimeMs();
    const bool isUpload = (entry.type == RequestEntry::Type::UPLOAD_FILE ||
                           entry.type == RequestEntry::Type::UPLOAD_MEMORY);
    const char* remoteName = entry.config.filename;

    FoEResult result{};
    result.success = false;
    result.error_code = FoEError::INVALID_STATE;

    if (!remoteName || remoteName[0] == '\0') {
        result.error_code = FoEError::INVALID_STATE;
        result.duration_ms = transport_.getTimeMs() - startMs;
        return result;
    }

    // ---- Source/sink setup ----
    bool fileOpen = false;
    uint32_t fileSize = 0;
    const uint8_t* memSrc = nullptr;
    size_t memSize = 0, memOffset = 0;
    uint8_t* memDst = nullptr;
    size_t memDstCap = 0, memReceived = 0;

    if (entry.type == RequestEntry::Type::UPLOAD_FILE) {
        if (!transport_.fileOpen(entry.local_path, false, &fileSize)) {
            result.error_code = FoEError::LOCAL_FILE_ERROR;
            result.duration_ms = transport_.getTimeMs() - startMs;
            return result;
        }
        fileOpen = true;
    } else if (entry.type == RequestEntry::Type::DOWNLOAD_FILE) {
        if (!transport_.fileOpen(entry.local_path, true, nullptr)) {
            result.error_code = FoEError::LOCAL_FILE_ERROR;
            result.duration_ms = transport_.getTimeMs() - startMs;
            return result;
        }
        fileOpen = true;
    } else if (entry.type == RequestEntry::Type::UPLOAD_MEMORY) {
        memSrc = static_cast<const uint8_t*>(entry.src_data);
        memSize = entry.data_size;
        if (!memSrc && memSize > 0) {
            result.error_code = FoEError::INVALID_STATE;
            result.duration_ms = transport_.getTimeMs() - startMs;
            return result;
        }
    } else {  // DOWNLOAD_MEMORY
        memDst = static_cast<uint8_t*>(entry.dst_data);
        memDstCap = entry.data_size;
        if (entry.received_size) *entry.received_size = 0;
    }

    auto cleanup = [&]() {
        if (fileOpen) transport_.fileClose();
    };

    // Progress reporting helper; returns false when cancelled.
    auto reportProgress = [&](uint32_t blockNo, uint32_t total,
                              bool up) -> bool {
        FoEProgress prog{};
        prog.slave_index = entry.config.slave_index;
        prog.filename = remoteName;
        prog.is_upload = up;
        prog.bytes_transferred = total;
        prog.total_bytes =
            (entry.type == RequestEntry::Type::UPLOAD_FILE) ? fileSize : 0;
        prog.block_number = blockNo;
        if (entry.handle) entry.handle->progress = prog;
        const FoEProgressCallback& cb = entry.config.progress_callback;
        return !cb || cb(prog);
    };

    auto cancelled = [&]() -> bool {
        return entry.handle && entry.handle->cancel.load();
    };

    // ---- WRQ: upload to slave ----
    if (isUpload) {
        uint8_t req[kMaxFilename + 16];
        uint8_t resp[kBufferSize];
        size_t respLen = 0;
        const size_t reqLen = buildFileRequest(
            static_cast<uint8_t>(FoEOpcode::WRQ), entry.config.password,
            remoteName, req, sizeof(req));
        if (reqLen == 0) {
            cleanup();
            result.error_code = FoEError::INVALID_STATE;
            result.duration_ms = transport_.getTimeMs() - startMs;
            return result;
        }
        bool mbxFail = false;
        if (!foeTransact(transport_, entry.config.slave_index, entry.config,
                         req, reqLen, resp, sizeof(resp), &respLen, &mbxFail)) {
            cleanup();
            result = makeResult(false, mbxFail ? FoEError::MAILBOX_ERROR
                                               : FoEError::TIMEOUT,
                                0, 0, startMs, transport_);
            return result;
        }
        FoEError slaveErr;
        if (parseErrorResponse(resp, respLen, slaveErr)) {
            cleanup();
            result = makeResult(false, slaveErr, 0, 0, startMs, transport_);
            return result;
        }
        if (parseResponseOpcode(resp, respLen) !=
            static_cast<uint8_t>(FoEOpcode::ACK)) {
            cleanup();
            result = makeResult(false, FoEError::INVALID_STATE, 0, 0, startMs, transport_);
            return result;
        }

        uint32_t blockNo = 1;
        uint32_t totalSent = 0;
        uint32_t blocks = 0;
        for (;;) {
            if (cancelled()) {
                cleanup();
                result = makeResult(false, FoEError::CANCELLED, totalSent, blocks,
                                    startMs, transport_);
                return result;
            }

            // Read next chunk from file or memory. A short or empty read
            // terminates the transfer (the slave ends on a sub-max DATA
            // block, which also covers exact-multiple file sizes via a
            // final empty DATA block).
            uint8_t chunk[kChunkPayload];
            size_t chunkLen = 0;
            if (entry.type == RequestEntry::Type::UPLOAD_FILE) {
                int32_t got = transport_.fileRead(chunk, sizeof(chunk));
                if (got < 0) {
                    cleanup();
                    result = makeResult(false, FoEError::LOCAL_FILE_ERROR,
                                        totalSent, blocks, startMs, transport_);
                    return result;
                }
                chunkLen = static_cast<size_t>(got);
            } else {
                chunkLen = std::min(sizeof(chunk), memSize - memOffset);
                if (chunkLen > 0) {
                    std::memcpy(chunk, memSrc + memOffset, chunkLen);
                    memOffset += chunkLen;
                }
            }
            const bool lastBlock = (chunkLen < sizeof(chunk));

            uint8_t msg[kBufferSize];
            const size_t msgLen = buildDataMessage(
                static_cast<uint8_t>(FoEOpcode::DATA), blockNo,
                chunkLen ? chunk : nullptr, chunkLen, msg, sizeof(msg));
            size_t respLen2 = 0;
            bool mbxFail = false;
            if (msgLen == 0 ||
                !foeTransact(transport_, entry.config.slave_index, entry.config,
                             msg, msgLen, resp, sizeof(resp), &respLen2, &mbxFail)) {
                cleanup();
                result = makeResult(false, mbxFail ? FoEError::MAILBOX_ERROR
                                                   : FoEError::TIMEOUT,
                                    totalSent, blocks, startMs, transport_);
                return result;
            }
            if (parseErrorResponse(resp, respLen2, slaveErr)) {
                cleanup();
                result = makeResult(false, slaveErr, totalSent, blocks,
                                    startMs, transport_);
                return result;
            }
            if (parseResponseOpcode(resp, respLen2) !=
                static_cast<uint8_t>(FoEOpcode::ACK)) {
                cleanup();
                result = makeResult(false, FoEError::INVALID_STATE, totalSent,
                                    blocks, startMs, transport_);
                return result;
            }

            totalSent += static_cast<uint32_t>(chunkLen);
            ++blocks;
            ++blockNo;
            if (!reportProgress(blockNo, totalSent, true)) {
                cleanup();
                result = makeResult(false, FoEError::CANCELLED, totalSent, blocks,
                                    startMs, transport_);
                return result;
            }
            if (lastBlock) break;
        }

        cleanup();
        result = makeResult(true, FoEError::SUCCESS, totalSent, blocks,
                            startMs, transport_);
        return result;
    }

    // ---- RRQ: download from slave ----
    {
        uint8_t req[kMaxFilename + 16];
        uint8_t resp[kBufferSize];
        size_t respLen = 0;
        const size_t reqLen = buildFileRequest(
            static_cast<uint8_t>(FoEOpcode::RRQ), entry.config.password,
            remoteName, req, sizeof(req));
        if (reqLen == 0) {
            cleanup();
            result.error_code = FoEError::INVALID_STATE;
            result.duration_ms = transport_.getTimeMs() - startMs;
            return result;
        }
        bool mbxFail = false;
        if (!foeTransact(transport_, entry.config.slave_index, entry.config,
                         req, reqLen, resp, sizeof(resp), &respLen, &mbxFail)) {
            cleanup();
            result = makeResult(false, mbxFail ? FoEError::MAILBOX_ERROR
                                               : FoEError::TIMEOUT,
                                0, 0, startMs, transport_);
            return result;
        }
        FoEError slaveErr;
        if (parseErrorResponse(resp, respLen, slaveErr)) {
            cleanup();
            result = makeResult(false, slaveErr, 0, 0, startMs, transport_);
            return result;
        }
        if (parseResponseOpcode(resp, respLen) !=
            static_cast<uint8_t>(FoEOpcode::ACK) || respLen < 10) {
            cleanup();
            result = makeResult(false, FoEError::INVALID_STATE, 0, 0, startMs, transport_);
            return result;
        }
        // ACK for RRQ carries the file size in bytes 6..9.
        const uint32_t remoteSize = getU32le(resp + 6);

        uint32_t totalReceived = 0;
        uint32_t blocks = 0;
        bool done = false;
        while (!done) {
            if (cancelled()) {
                cleanup();
                result = makeResult(false, FoEError::CANCELLED, totalReceived,
                                    blocks, startMs, transport_);
                return result;
            }
            // Read next DATA block from slave.
            size_t dataLen = 0;
            uint8_t counter = 0;
            // The slave pushes DATA unsolicited; poll the read mailbox.
            bool got = false;
            for (int attempt = 0; attempt <= kMaxBlockRetries && !got; ++attempt) {
                got = transport_.mailboxRead(
                    entry.config.slave_index, kFoeMbxRdAddr, kBufferSize,
                    resp, sizeof(resp), &dataLen, entry.config.timeout_ms);
                (void)counter;
                if (!got) transport_.delayMs(5);
            }
            if (!got || dataLen < 6) {
                cleanup();
                result = makeResult(false, FoEError::TIMEOUT, totalReceived,
                                    blocks, startMs, transport_);
                return result;
            }
            const uint8_t op = resp[0];
            if (op == static_cast<uint8_t>(FoEOpcode::ERROR)) {
                FoEError e = FoEError::INVALID_STATE;
                parseErrorResponse(resp, dataLen, e);
                cleanup();
                result = makeResult(false, e, totalReceived, blocks,
                                    startMs, transport_);
                return result;
            }
            if (op != static_cast<uint8_t>(FoEOpcode::DATA)) {
                cleanup();
                result = makeResult(false, FoEError::INVALID_STATE, totalReceived,
                                    blocks, startMs, transport_);
                return result;
            }
            const uint32_t blockNo = getU32le(resp + 2);
            const size_t payloadLen = dataLen - 6;
            done = (payloadLen < kChunkPayload) ||
                   (remoteSize > 0 && totalReceived + payloadLen >= remoteSize);

            // Write payload to file/memory sink (bounded by capacity).
            const uint8_t* payload = resp + 6;
            size_t toWrite = payloadLen;
            if (entry.type == RequestEntry::Type::DOWNLOAD_FILE) {
                int32_t wrote = transport_.fileWrite(payload, toWrite);
                if (wrote < 0) {
                    cleanup();
                    result = makeResult(false, FoEError::LOCAL_FILE_ERROR,
                                        totalReceived, blocks, startMs, transport_);
                    return result;
                }
                toWrite = static_cast<size_t>(wrote);
            } else {
                const size_t room = memDstCap - memReceived;
                toWrite = std::min(toWrite, room);
                if (toWrite > 0) {
                    std::memcpy(memDst + memReceived, payload, toWrite);
                    memReceived += toWrite;
                }
                if (payloadLen > room) {
                    // Sink full — keep draining but report overflow at end.
                }
            }
            totalReceived += static_cast<uint32_t>(toWrite);
            ++blocks;

            // ACK the block.
            uint8_t ack[6];
            const size_t ackLen = buildDataMessage(
                static_cast<uint8_t>(FoEOpcode::ACK), blockNo, nullptr, 0,
                ack, sizeof(ack));
            uint8_t dummyResp[kBufferSize];
            size_t dummyLen = 0;
            // ACK is fire-and-forget per FoE (no response expected); still
            // write it so the slave advances.
            uint8_t ackCounter = 0;
            transport_.mailboxWrite(entry.config.slave_index, kFoeMbxWrAddr,
                                    kBufferSize, ack, ackLen, &ackCounter);
            (void)dummyResp; (void)dummyLen;

            if (!reportProgress(blockNo, totalReceived, false)) {
                cleanup();
                result = makeResult(false, FoEError::CANCELLED, totalReceived,
                                    blocks, startMs, transport_);
                return result;
            }
            if (done) break;
        }

        if (entry.received_size) *entry.received_size = memReceived;
        cleanup();
        result = makeResult(true, FoEError::SUCCESS, totalReceived, blocks,
                            startMs, transport_);
        return result;
    }
}

// ----- Synchronous Transfer API -----

FoEResult FoEManager::uploadFile(const char* local_path, const FoETransferConfig& config) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    RequestEntry e{};
    e.type = RequestEntry::Type::UPLOAD_FILE;
    std::strncpy(e.local_path, local_path ? local_path : "", sizeof(e.local_path) - 1);
    e.config = config;
    FoEResult r = executeRequest(e);
    recordResult(e, r);
    return r;
}

FoEResult FoEManager::downloadFile(const char* local_path, const FoETransferConfig& config) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    RequestEntry e{};
    e.type = RequestEntry::Type::DOWNLOAD_FILE;
    std::strncpy(e.local_path, local_path ? local_path : "", sizeof(e.local_path) - 1);
    e.config = config;
    FoEResult r = executeRequest(e);
    recordResult(e, r);
    return r;
}

FoEResult FoEManager::uploadMemory(const void* data, size_t size, const FoETransferConfig& config) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    RequestEntry e{};
    e.type = RequestEntry::Type::UPLOAD_MEMORY;
    e.config = config;
    e.src_data = data;
    e.data_size = size;
    FoEResult r = executeRequest(e);
    recordResult(e, r);
    return r;
}

FoEResult FoEManager::downloadMemory(void* buffer, size_t buffer_size,
                                      size_t* received_size, const FoETransferConfig& config) {
    if (!initialized_.load()) return FoEResult::Failure(FoEError::NOT_INITIALIZED);
    RequestEntry e{};
    e.type = RequestEntry::Type::DOWNLOAD_MEMORY;
    e.config = config;
    e.dst_data = buffer;
    e.data_size = buffer_size;
    e.received_size = received_size;
    FoEResult r = executeRequest(e);
    recordResult(e, r);
    return r;
}

// ----- Asynchronous Transfer API -----

bool FoEManager::uploadFileAsync(const char* local_path, const FoETransferConfig& config,
                                  FoETransferHandle* handle) {
    if (!initialized_.load() || !handle) return false;
    RequestEntry e{};
    e.type = RequestEntry::Type::UPLOAD_FILE;
    std::strncpy(e.local_path, local_path ? local_path : "", sizeof(e.local_path) - 1);
    e.config = config;
    e.handle = handle;
    handle->id = next_transfer_id_.fetch_add(1);
    handle->complete.store(false);
    handle->cancel.store(false);
    return enqueueRequest(e);
}

bool FoEManager::downloadFileAsync(const char* local_path, const FoETransferConfig& config,
                                    FoETransferHandle* handle) {
    if (!initialized_.load() || !handle) return false;
    RequestEntry e{};
    e.type = RequestEntry::Type::DOWNLOAD_FILE;
    std::strncpy(e.local_path, local_path ? local_path : "", sizeof(e.local_path) - 1);
    e.config = config;
    e.handle = handle;
    handle->id = next_transfer_id_.fetch_add(1);
    handle->complete.store(false);
    handle->cancel.store(false);
    return enqueueRequest(e);
}

bool FoEManager::waitComplete(FoETransferHandle* handle, uint32_t timeout_ms) {
    if (!handle) return false;
    if (handle->complete.load()) return true;
    if (timeout_ms == 0) return false;  // poll once
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (!handle->complete.load()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

void FoEManager::cancel(FoETransferHandle* handle) {
    if (handle) {
        handle->cancel.store(true);
    }
}

} // namespace FoE
} // namespace EtherCAT

#endif // ECAT_FEATURE_FOE_ENABLED
